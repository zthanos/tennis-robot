"""Neural tennis-ball detectors for the simulated OAK-D AI pipeline.

The real OAK-D runs a YOLO model *on-device* (DepthAI) and streams detections.
In simulation we have no OAK-D — Gazebo only gives us a plain RGB image. This
module closes that gap: it runs the *same kind* of neural detector (a YOLOv8/
YOLOv11-nano model) on the simulated RGB frame, so the perception node produces
detections that are interchangeable with the real device's output.

Design contract
---------------
Every detector implements :class:`BallDetector`:

    detect(frame_bgr) -> list[BallDetection]

`BallDetection` (defined in :mod:`tennis_robot.perception`) is the single
pixel-space detection type shared by the whole pipeline. Swapping the simulated
:class:`YoloOnnxBallDetector` for a future on-device OAK-D source therefore
requires **no change** to the depth-fusion / publishing code downstream — they
only ever see `BallDetection` objects and the ROS messages built from them.

Why ONNX Runtime (not PyTorch/ultralytics)
------------------------------------------
ONNX Runtime is a small, CPU-friendly, dependency-light wheel that installs
cleanly into the ROS 2 Humble image and onto the robot's SBC. A stock
``yolov8n.onnx`` (COCO) already detects the *sports ball* class (id 32), which
covers tennis balls, so no custom training is required to stand the pipeline up.
See ``scripts/export_yolo_onnx.py`` to produce the model file.
"""

from __future__ import annotations

import os
from typing import Protocol, runtime_checkable

import numpy as np

from tennis_robot.perception import BallDetection
from tennis_robot.onnx_runtime_config import create_cpu_inference_session

# COCO class id 32 == "sports ball". A tennis ball is detected under this class
# by stock YOLOv8/v11 weights. A custom single-class model would use id 0 — set
# BALL_CLASS_IDS accordingly.
DEFAULT_SPORTS_BALL_CLASS_ID = 32


# ---------------------------------------------------------------------------
# Detector interface
# ---------------------------------------------------------------------------
@runtime_checkable
class BallDetector(Protocol):
    """A source of tennis-ball detections from a single BGR frame."""

    name: str

    @property
    def available(self) -> bool:
        """True when the detector is loaded and able to produce detections."""
        ...

    def detect(self, frame_bgr: np.ndarray) -> list[BallDetection]:
        """Return all tennis-ball detections in *frame_bgr* (largest first)."""
        ...


# ---------------------------------------------------------------------------
# YOLO (ONNX Runtime) detector
# ---------------------------------------------------------------------------
class YoloOnnxBallDetector:
    """Run a YOLOv8/YOLOv11-nano ONNX model on a BGR frame via ONNX Runtime.

    Handles letterbox pre-processing, the two common YOLOv8 output layouts
    ((1, 84, N) and (1, N, 84)), class filtering and Non-Max-Suppression in
    pure NumPy (no torchvision dependency).
    """

    name = "yolo_onnx"

    def __init__(
        self,
        model_path: str,
        *,
        input_size: int = 640,
        conf_threshold: float = 0.35,
        iou_threshold: float = 0.45,
        class_ids: tuple[int, ...] = (DEFAULT_SPORTS_BALL_CLASS_ID,),
        providers: list[str] | None = None,
        center_zoom_factor: float = 1.0,
        zoom_tiles: tuple[tuple[float, float], ...] = ((0.5, 0.5),),
    ) -> None:
        import onnxruntime as ort  # imported lazily so the module loads without it

        self.model_path = model_path
        self.input_size = int(input_size)
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.class_ids = set(int(c) for c in class_ids)
        self.center_zoom_factor = float(center_zoom_factor)
        if not np.isfinite(self.center_zoom_factor) or self.center_zoom_factor < 1.0:
            raise ValueError("center_zoom_factor must be finite and >= 1.0")
        self.zoom_tiles = tuple((float(x), float(y)) for x, y in zoom_tiles)
        if not self.zoom_tiles or any(
            not np.isfinite(x) or not np.isfinite(y) or not 0.0 <= x <= 1.0 or not 0.0 <= y <= 1.0
            for x, y in self.zoom_tiles
        ):
            raise ValueError("zoom_tiles must contain finite normalized (x, y) centres")

        self._session = create_cpu_inference_session(
            ort,
            model_path,
            providers=providers,
        )
        self._input_name = self._session.get_inputs()[0].name
        # Respect a fixed input H/W baked into the model if present.
        shape = self._session.get_inputs()[0].shape
        if len(shape) == 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
            self.input_size = int(shape[2])

    @property
    def available(self) -> bool:
        return True

    # -- public API ---------------------------------------------------------
    def detect(self, frame_bgr: np.ndarray) -> list[BallDetection]:
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        detections = self._detect_single(frame_bgr)
        if self.center_zoom_factor > 1.0:
            h0, w0 = frame_bgr.shape[:2]
            crop_w = max(1, int(round(w0 / self.center_zoom_factor)))
            crop_h = max(1, int(round(h0 / self.center_zoom_factor)))
            for centre_x, centre_y in self.zoom_tiles:
                x0 = int(round(centre_x * w0 - crop_w / 2.0))
                y0 = int(round(centre_y * h0 - crop_h / 2.0))
                x0 = max(0, min(w0 - crop_w, x0))
                y0 = max(0, min(h0 - crop_h, y0))
                crop = frame_bgr[y0:y0 + crop_h, x0:x0 + crop_w]
                for detection in self._detect_single(crop):
                    detections.append(
                        BallDetection(
                            detection.x + x0,
                            detection.y + y0,
                            detection.width,
                            detection.height,
                            confidence=detection.confidence,
                            label=detection.label,
                        )
                    )
        return _nms_ball_detections(detections, self.iou_threshold)

    def _detect_single(self, frame_bgr: np.ndarray) -> list[BallDetection]:
        """Run one model pass and map boxes into this frame's pixel space."""
        h0, w0 = frame_bgr.shape[:2]
        blob, scale, pad = self._preprocess(frame_bgr)
        outputs = self._session.run(None, {self._input_name: blob})
        boxes_xyxy, scores = self._postprocess(outputs[0])
        if boxes_xyxy.shape[0] == 0:
            return []

        keep = _nms(boxes_xyxy, scores, self.iou_threshold)
        detections: list[BallDetection] = []
        for i in keep:
            x1, y1, x2, y2 = boxes_xyxy[i]
            # Undo letterbox: subtract padding, divide by scale, clamp to image.
            x1 = (x1 - pad[0]) / scale
            y1 = (y1 - pad[1]) / scale
            x2 = (x2 - pad[0]) / scale
            y2 = (y2 - pad[1]) / scale
            x1 = max(0.0, min(float(w0 - 1), x1))
            y1 = max(0.0, min(float(h0 - 1), y1))
            x2 = max(0.0, min(float(w0 - 1), x2))
            y2 = max(0.0, min(float(h0 - 1), y2))
            w = int(round(x2 - x1))
            h = int(round(y2 - y1))
            if w <= 0 or h <= 0:
                continue
            detections.append(
                BallDetection(
                    int(round(x1)), int(round(y1)), w, h,
                    confidence=float(scores[i]),
                    label="tennis_ball",
                )
            )
        detections.sort(key=lambda d: d.area_px, reverse=True)
        return detections

    # -- internals ----------------------------------------------------------
    def _preprocess(self, frame_bgr: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        """Letterbox to a square `input_size`, return NCHW float blob + transform."""
        h0, w0 = frame_bgr.shape[:2]
        s = self.input_size
        scale = min(s / w0, s / h0)
        nw, nh = int(round(w0 * scale)), int(round(h0 * scale))
        pad_x = (s - nw) / 2.0
        pad_y = (s - nh) / 2.0

        import cv2

        resized = cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((s, s, 3), 114, dtype=np.uint8)
        top, left = int(round(pad_y)), int(round(pad_x))
        canvas[top:top + nh, left:left + nw] = resized

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        blob = rgb.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None]  # NCHW
        return np.ascontiguousarray(blob), scale, (left, top)

    def _postprocess(self, raw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Decode a YOLOv8 head into (boxes_xyxy, scores) for target classes."""
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]  # drop batch -> (C, N) or (N, C)
        # Normalise to (N, C): C = 4 bbox + num_classes
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T  # (84, 8400) -> (8400, 84)

        if pred.shape[1] < 5:
            return np.empty((0, 4), np.float32), np.empty((0,), np.float32)

        boxes_cxcywh = pred[:, :4]
        class_scores = pred[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]

        mask = confidences >= self.conf_threshold
        if self.class_ids:
            wanted = np.isin(class_ids, list(self.class_ids))
            mask &= wanted
        if not np.any(mask):
            return np.empty((0, 4), np.float32), np.empty((0,), np.float32)

        boxes_cxcywh = boxes_cxcywh[mask]
        confidences = confidences[mask].astype(np.float32)

        cx, cy, w, h = (
            boxes_cxcywh[:, 0], boxes_cxcywh[:, 1],
            boxes_cxcywh[:, 2], boxes_cxcywh[:, 3],
        )
        boxes_xyxy = np.stack(
            [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0], axis=1
        ).astype(np.float32)
        return boxes_xyxy, confidences


def _nms(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    """Pure-NumPy Non-Max-Suppression. Returns kept indices, highest score first."""
    if boxes_xyxy.shape[0] == 0:
        return []
    x1, y1, x2, y2 = boxes_xyxy[:, 0], boxes_xyxy[:, 1], boxes_xyxy[:, 2], boxes_xyxy[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / union, 0.0)
        order = rest[iou <= iou_threshold]
    return keep


def _nms_ball_detections(
    detections: list[BallDetection], iou_threshold: float
) -> list[BallDetection]:
    """Merge full-frame and zoom-pass detections in original pixel space.

    The two passes can predict substantially different box sizes for the same
    small object.  In addition to ordinary IoU, suppress a box when at least
    80% of the smaller box is covered by a higher-confidence box.
    """
    if not detections:
        return []
    boxes = np.asarray(
        [[d.x, d.y, d.x + d.width, d.y + d.height] for d in detections],
        dtype=np.float32,
    )
    scores = np.asarray([d.confidence for d in detections], dtype=np.float32)
    areas = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(
        0.0, boxes[:, 3] - boxes[:, 1]
    )
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[index, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[index, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[index, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[index, 3], boxes[rest, 3])
        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[index] + areas[rest] - intersection
        iou = np.where(union > 0.0, intersection / union, 0.0)
        smaller = np.minimum(areas[index], areas[rest])
        containment = np.where(smaller > 0.0, intersection / smaller, 0.0)
        order = rest[(iou <= iou_threshold) & (containment < 0.8)]
    merged = [detections[index] for index in keep]
    merged.sort(key=lambda detection: detection.area_px, reverse=True)
    return merged


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def load_ball_detector(
    *,
    backend: str | None = None,
    model_path: str | None = None,
    conf_threshold: float | None = None,
    iou_threshold: float | None = None,
    logger=None,
) -> BallDetector:
    """Build the required neural detector or raise a startup error.

    Env overrides (used when the matching argument is None):
      BALL_DETECTOR_BACKEND  yolo_onnx                       (default yolo_onnx)
      BALL_MODEL_PATH        path to the .onnx model        (default models/yolov8n.onnx)
      BALL_CONF_THRESHOLD    float                          (default 0.35)
      BALL_IOU_THRESHOLD     float                          (default 0.45)
      BALL_CLASS_IDS         comma list of class ids        (default 32)
      BALL_CENTER_ZOOM_FACTOR neural center-crop scale       (default 1.0)
      BALL_CENTER_ZOOM_TILES normalized x:y crop centres     (default 0.5:0.5)

    There is intentionally no classical or no-op fallback. Perception without
    its neural model is an unhealthy system and must fail loudly at startup.
    """
    def _log(level: str, msg: str) -> None:
        if logger is not None:
            getattr(logger, level, logger.info)(msg)

    backend = (backend or os.getenv("BALL_DETECTOR_BACKEND", "yolo_onnx")).strip().lower()
    conf = conf_threshold if conf_threshold is not None else float(os.getenv("BALL_CONF_THRESHOLD", "0.35"))
    iou = iou_threshold if iou_threshold is not None else float(os.getenv("BALL_IOU_THRESHOLD", "0.45"))
    class_ids = tuple(
        int(c) for c in os.getenv("BALL_CLASS_IDS", str(DEFAULT_SPORTS_BALL_CLASS_ID)).split(",")
        if c.strip() != ""
    ) or (DEFAULT_SPORTS_BALL_CLASS_ID,)
    center_zoom_factor = float(os.getenv("BALL_CENTER_ZOOM_FACTOR", "1.0"))
    zoom_tiles = tuple(
        tuple(float(value) for value in item.split(":"))
        for item in os.getenv("BALL_CENTER_ZOOM_TILES", "0.5:0.5").split(",")
        if item.strip()
    )
    if any(len(tile) != 2 for tile in zoom_tiles):
        raise ValueError("BALL_CENTER_ZOOM_TILES must be comma-separated x:y pairs")

    if backend != "yolo_onnx":
        raise ValueError(
            f"unsupported BALL_DETECTOR_BACKEND={backend!r}; only 'yolo_onnx' is allowed"
        )

    path = model_path or os.getenv("BALL_MODEL_PATH", "models/yolov8n.onnx")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"YOLO model not found at '{path}'. "
            "Run scripts/export_yolo_onnx.py or set BALL_MODEL_PATH."
        )
    try:
        import onnxruntime  # noqa: F401
    except Exception as exc:  # pragma: no cover - env-dependent
        raise RuntimeError("onnxruntime is required for neural ball perception") from exc

    det = YoloOnnxBallDetector(
        path,
        conf_threshold=conf,
        iou_threshold=iou,
        class_ids=class_ids,
        center_zoom_factor=center_zoom_factor,
        zoom_tiles=zoom_tiles,
    )
    _log("info", f"loaded YOLO ONNX detector '{path}' "
                 f"(input={det.input_size}, conf>={conf}, classes={sorted(det.class_ids)}, "
                 f"center_zoom={det.center_zoom_factor}, zoom_tiles={det.zoom_tiles})")
    return det
