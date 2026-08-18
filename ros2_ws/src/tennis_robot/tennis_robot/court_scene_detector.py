"""Neural net/fence perception for the simulated OAK-D pipeline.

The detector consumes the same timestamp-matched RGB/depth acquisition as the
ball detector.  A custom YOLO ONNX model supplies two classes:

    0 = net
    1 = fence

There is intentionally no OpenCV/classical fallback.  Missing or invalid
weights make the perception node fail startup, matching the primary neural
perception contract used for tennis balls.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from typing import Protocol, runtime_checkable

import numpy as np

from tennis_robot.onnx_runtime_config import create_cpu_inference_session

from tennis_robot.ball_detector import _nms


DEFAULT_CLASS_LABELS = {0: "net", 1: "fence"}


@dataclass(frozen=True)
class CourtSceneDetection:
    """One pixel-space semantic obstacle detection."""

    x: int
    y: int
    width: int
    height: int
    confidence: float
    label: str
    class_id: int

    @property
    def center_x(self) -> float:
        return self.x + self.width / 2.0

    @property
    def center_y(self) -> float:
        return self.y + self.height / 2.0

    @property
    def area_px(self) -> int:
        return self.width * self.height


@dataclass(frozen=True)
class CourtSceneObservation:
    """A neural detection fused with depth from the same acquisition."""

    detection: CourtSceneDetection
    distance_m: float | None
    bearing_rad: float
    valid_depth_count: int


@runtime_checkable
class CourtSceneDetector(Protocol):
    name: str

    @property
    def available(self) -> bool:
        ...

    def detect(self, frame_bgr: np.ndarray) -> list[CourtSceneDetection]:
        ...


class YoloOnnxCourtSceneDetector:
    """YOLOv8/v11 ONNX detector with class-aware NMS.

    Class-aware suppression is important here: the foreground net and the
    background fence overlap heavily in camera space and must remain distinct
    model hypotheses.
    """

    name = "court_scene_yolo_onnx"

    def __init__(
        self,
        model_path: str,
        *,
        input_size: int = 640,
        conf_threshold: float = 0.45,
        iou_threshold: float = 0.45,
        class_labels: dict[int, str] | None = None,
        providers: list[str] | None = None,
    ) -> None:
        import onnxruntime as ort

        self.model_path = model_path
        self.input_size = int(input_size)
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.class_labels = dict(class_labels or DEFAULT_CLASS_LABELS)
        if not self.class_labels:
            raise ValueError("court-scene class map must not be empty")
        if (
            len(self.class_labels) != 2
            or set(self.class_labels.values()) != {"net", "fence"}
        ):
            raise ValueError("court-scene class map must contain exactly 'net' and 'fence'")

        self._session = create_cpu_inference_session(
            ort,
            model_path,
            providers=providers,
        )
        self._input_name = self._session.get_inputs()[0].name
        shape = self._session.get_inputs()[0].shape
        if len(shape) == 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
            if int(shape[2]) != int(shape[3]):
                raise ValueError("court-scene model input must be square")
            self.input_size = int(shape[2])

    @property
    def available(self) -> bool:
        return True

    def detect(self, frame_bgr: np.ndarray) -> list[CourtSceneDetection]:
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        h0, w0 = frame_bgr.shape[:2]
        blob, scale, pad = self._preprocess(frame_bgr)
        outputs = self._session.run(None, {self._input_name: blob})
        boxes, scores, class_ids = self._postprocess(outputs[0])
        if boxes.shape[0] == 0:
            return []

        # Suppress duplicate boxes only within the same semantic class.  A net
        # is allowed to overlap a fence visible behind it.
        kept: list[int] = []
        for class_id in sorted(set(int(value) for value in class_ids)):
            indices = np.flatnonzero(class_ids == class_id)
            local_keep = _nms(boxes[indices], scores[indices], self.iou_threshold)
            kept.extend(int(indices[index]) for index in local_keep)
        kept.sort(key=lambda index: float(scores[index]), reverse=True)

        detections: list[CourtSceneDetection] = []
        for index in kept:
            x1, y1, x2, y2 = boxes[index]
            x1 = max(0.0, min(float(w0 - 1), (x1 - pad[0]) / scale))
            y1 = max(0.0, min(float(h0 - 1), (y1 - pad[1]) / scale))
            x2 = max(0.0, min(float(w0 - 1), (x2 - pad[0]) / scale))
            y2 = max(0.0, min(float(h0 - 1), (y2 - pad[1]) / scale))
            width = int(round(x2 - x1))
            height = int(round(y2 - y1))
            if width <= 0 or height <= 0:
                continue
            class_id = int(class_ids[index])
            detections.append(
                CourtSceneDetection(
                    x=int(round(x1)),
                    y=int(round(y1)),
                    width=width,
                    height=height,
                    confidence=float(scores[index]),
                    label=self.class_labels[class_id],
                    class_id=class_id,
                )
            )
        detections.sort(
            key=lambda detection: (detection.confidence, detection.area_px),
            reverse=True,
        )
        return detections

    def _preprocess(
        self, frame_bgr: np.ndarray
    ) -> tuple[np.ndarray, float, tuple[float, float]]:
        import cv2

        h0, w0 = frame_bgr.shape[:2]
        size = self.input_size
        scale = min(size / w0, size / h0)
        new_width, new_height = int(round(w0 * scale)), int(round(h0 * scale))
        pad_x = (size - new_width) / 2.0
        pad_y = (size - new_height) / 2.0
        resized = cv2.resize(
            frame_bgr, (new_width, new_height), interpolation=cv2.INTER_LINEAR
        )
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        left, top = int(round(pad_x)), int(round(pad_y))
        canvas[top : top + new_height, left : left + new_width] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        blob = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None]
        return np.ascontiguousarray(blob), scale, (left, top)

    def _postprocess(
        self, raw: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:
            raise ValueError(f"unsupported court-scene YOLO output shape: {pred.shape}")
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T
        if pred.shape[1] < 5:
            return (
                np.empty((0, 4), np.float32),
                np.empty((0,), np.float32),
                np.empty((0,), np.int64),
            )

        boxes_cxcywh = pred[:, :4]
        class_scores = pred[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]
        known = np.isin(class_ids, list(self.class_labels))
        mask = known & (confidences >= self.conf_threshold)
        if not np.any(mask):
            return (
                np.empty((0, 4), np.float32),
                np.empty((0,), np.float32),
                np.empty((0,), np.int64),
            )
        boxes_cxcywh = boxes_cxcywh[mask]
        confidences = confidences[mask].astype(np.float32)
        class_ids = class_ids[mask].astype(np.int64)
        cx, cy, width, height = boxes_cxcywh.T
        boxes = np.stack(
            [
                cx - width / 2.0,
                cy - height / 2.0,
                cx + width / 2.0,
                cy + height / 2.0,
            ],
            axis=1,
        ).astype(np.float32)
        return boxes, confidences, class_ids


def fuse_court_scene_detections(
    detections: list[CourtSceneDetection],
    depth_frame: np.ndarray,
    frame_width: int,
    frame_height: int,
    camera_fov_rad: float,
    *,
    depth_min_m: float = 0.1,
    depth_max_m: float = 10.0,
) -> list[CourtSceneObservation]:
    """Fuse court-scene boxes with the timestamp-matched optical-Z depth image."""

    depth_height, depth_width = depth_frame.shape[:2]
    observations: list[CourtSceneObservation] = []
    for detection in detections:
        # Use the central 60% of each box.  The low percentile selects the
        # foreground mesh when a farther fence is visible through a net.
        x0 = detection.x + int(detection.width * 0.20)
        x1 = detection.x + int(detection.width * 0.80)
        y0 = detection.y + int(detection.height * 0.20)
        y1 = detection.y + int(detection.height * 0.80)
        dx0 = max(0, min(depth_width, int(x0 * depth_width / max(1, frame_width))))
        dx1 = max(0, min(depth_width, int(x1 * depth_width / max(1, frame_width))))
        dy0 = max(0, min(depth_height, int(y0 * depth_height / max(1, frame_height))))
        dy1 = max(0, min(depth_height, int(y1 * depth_height / max(1, frame_height))))
        roi = depth_frame[dy0:dy1, dx0:dx1]
        valid = roi[
            np.isfinite(roi) & (roi >= depth_min_m) & (roi <= depth_max_m)
        ]
        # Net strands occupy a small fraction of a detection box.  The 10th
        # percentile retains that foreground surface without trusting a single
        # noisy minimum pixel.
        distance = float(np.percentile(valid, 10)) if valid.size else None
        normalized_x = (
            detection.center_x - frame_width * 0.5
        ) / max(1.0, frame_width * 0.5)
        # SurveyVision follows image convention here: positive is camera-right.
        bearing = math.atan(normalized_x * math.tan(camera_fov_rad / 2.0))
        observations.append(
            CourtSceneObservation(
                detection=detection,
                distance_m=distance,
                bearing_rad=float(bearing),
                valid_depth_count=int(valid.size),
            )
        )
    observations.sort(
        key=lambda observation: (
            observation.distance_m is None,
            observation.distance_m
            if observation.distance_m is not None
            else math.inf,
            -observation.detection.confidence,
        )
    )
    return observations


def select_primary_observation(
    observations: list[CourtSceneObservation],
    frame_width: int,
    *,
    net_fence_depth_tolerance_m: float = 0.35,
    overlap_min_fraction: float = 0.20,
) -> CourtSceneObservation | None:
    """Select the obstacle blocking the robot's forward camera corridor."""

    if not observations:
        return None
    center_x = frame_width * 0.5

    def contains_center(observation: CourtSceneObservation) -> bool:
        detection = observation.detection
        return detection.x <= center_x <= detection.x + detection.width

    def key(observation: CourtSceneObservation) -> tuple:
        detection = observation.detection
        center_error = abs(detection.center_x - center_x) / max(1.0, frame_width)
        distance = (
            observation.distance_m
            if observation.distance_m is not None
            else math.inf
        )
        return (
            not contains_center(observation),
            distance,
            center_error,
            -detection.confidence,
        )

    primary = min(observations, key=key)
    if primary.detection.label != "fence":
        return primary

    def overlap_fraction(
        first: CourtSceneDetection, second: CourtSceneDetection
    ) -> float:
        x0 = max(first.x, second.x)
        y0 = max(first.y, second.y)
        x1 = min(first.x + first.width, second.x + second.width)
        y1 = min(first.y + first.height, second.y + second.height)
        intersection = max(0, x1 - x0) * max(0, y1 - y0)
        smaller_area = min(first.area_px, second.area_px)
        return intersection / smaller_area if smaller_area > 0 else 0.0

    # A net and the fence visible through it frequently produce nearly equal
    # low-percentile depth because both ROIs contain foreground net strands.
    # In that specific co-depth, overlapping case, the net is the semantic
    # obstacle the survey must lock onto. A genuinely closer fence still wins.
    net_candidates: list[CourtSceneObservation] = []
    for observation in observations:
        if observation.detection.label != "net" or not contains_center(observation):
            continue
        if overlap_fraction(observation.detection, primary.detection) < overlap_min_fraction:
            continue
        if primary.distance_m is None:
            co_depth = observation.distance_m is None
        else:
            co_depth = (
                observation.distance_m is not None
                and observation.distance_m
                <= primary.distance_m + net_fence_depth_tolerance_m
            )
        if co_depth:
            net_candidates.append(observation)
    return min(net_candidates, key=key) if net_candidates else primary


class SemanticConfirmation:
    """Require the same neural class in consecutive synchronized frames."""

    def __init__(self, required_frames: int = 3) -> None:
        if required_frames < 1:
            raise ValueError("required_frames must be >= 1")
        self.required_frames = int(required_frames)
        self._candidate: str | None = None
        self._count = 0

    def update(self, label: str | None) -> str | None:
        if label is None:
            self._candidate = None
            self._count = 0
            return None
        if label == self._candidate:
            self._count += 1
        else:
            self._candidate = label
            self._count = 1
        return label if self._count >= self.required_frames else None


def _parse_class_labels(value: str) -> dict[int, str]:
    labels: dict[int, str] = {}
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            class_id_text, label = item.split(":", 1)
            class_id = int(class_id_text)
        except ValueError as exc:
            raise ValueError(
                "COURT_SCENE_CLASS_MAP must be comma-separated id:label pairs"
            ) from exc
        labels[class_id] = label.strip().lower()
    if len(labels) != 2 or set(labels.values()) != {"net", "fence"}:
        raise ValueError(
            "COURT_SCENE_CLASS_MAP must contain exactly one net and one fence class"
        )
    return labels


def load_court_scene_detector(
    *,
    backend: str | None = None,
    model_path: str | None = None,
    conf_threshold: float | None = None,
    iou_threshold: float | None = None,
    logger=None,
) -> CourtSceneDetector:
    """Load the required neural court-scene detector or fail startup."""

    backend = (
        backend or os.getenv("COURT_SCENE_DETECTOR_BACKEND", "yolo_onnx")
    ).strip().lower()
    if backend != "yolo_onnx":
        raise ValueError(
            f"unsupported COURT_SCENE_DETECTOR_BACKEND={backend!r}; "
            "only 'yolo_onnx' is allowed"
        )
    path = model_path or os.getenv(
        "COURT_SCENE_MODEL_PATH", "models/court_scene_yolov8n.onnx"
    )
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"court-scene YOLO model not found at '{path}'. "
            "Train/export the net/fence model or set COURT_SCENE_MODEL_PATH."
        )
    try:
        import onnxruntime  # noqa: F401
    except Exception as exc:  # pragma: no cover - environment-dependent
        raise RuntimeError(
            "onnxruntime is required for neural court-scene perception"
        ) from exc

    conf = (
        conf_threshold
        if conf_threshold is not None
        else float(os.getenv("COURT_SCENE_CONF_THRESHOLD", "0.45"))
    )
    iou = (
        iou_threshold
        if iou_threshold is not None
        else float(os.getenv("COURT_SCENE_IOU_THRESHOLD", "0.45"))
    )
    labels = _parse_class_labels(
        os.getenv("COURT_SCENE_CLASS_MAP", "0:net,1:fence")
    )
    detector = YoloOnnxCourtSceneDetector(
        path,
        conf_threshold=conf,
        iou_threshold=iou,
        class_labels=labels,
    )
    if logger is not None:
        logger.info(
            f"loaded court-scene YOLO ONNX detector '{path}' "
            f"(input={detector.input_size}, conf>={conf}, classes={labels})"
        )
    return detector
