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
    ) -> None:
        import onnxruntime as ort  # imported lazily so the module loads without it

        self.model_path = model_path
        self.input_size = int(input_size)
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.class_ids = set(int(c) for c in class_ids)

        self._session = ort.InferenceSession(
            model_path,
            providers=providers or ["CPUExecutionProvider"],
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
        path, conf_threshold=conf, iou_threshold=iou, class_ids=class_ids,
    )
    _log("info", f"loaded YOLO ONNX detector '{path}' "
                 f"(input={det.input_size}, conf>={conf}, classes={sorted(det.class_ids)})")
    return det
