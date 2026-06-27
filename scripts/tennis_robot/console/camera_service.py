"""CameraService — webcam capture + tennis-ball computer vision.

Owns the shared capture device and turns a frame into the JSON payload the
webcam view consumes (annotated JPEG data URL + monocular distance/bearing).
Vision deps (cv2, perception) are optional; when missing the service reports
``available: False`` instead of raising.
"""

from __future__ import annotations

import base64
import math
import threading

from .config import WEBCAM_FOV_DEG

try:
    import cv2
    from tennis_robot.perception import detect_largest_ball, TENNIS_BALL_DIAMETER_M
    _VISION_AVAILABLE = True
except ImportError:
    _VISION_AVAILABLE = False


class CameraService:
    def __init__(self, fov_deg: float = WEBCAM_FOV_DEG, device_index: int = 0) -> None:
        self._fov_deg = fov_deg
        self._device_index = device_index
        self._cap = None
        self._lock = threading.Lock()

    @property
    def available(self) -> bool:
        return _VISION_AVAILABLE

    def _get_frame(self) -> tuple[bool, object]:
        if not _VISION_AVAILABLE:
            return False, None
        with self._lock:
            if self._cap is None or not self._cap.isOpened():
                self._cap = cv2.VideoCapture(self._device_index)
            if not self._cap.isOpened():
                return False, None
            return self._cap.read()

    def release(self) -> None:
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None

    def frame(self) -> dict[str, object]:
        """Capture, detect, annotate; return the webcam payload dict."""
        if not _VISION_AVAILABLE:
            return {"available": False, "error": "cv2 / perception not installed"}
        ok, frame = self._get_frame()
        if not ok or frame is None:
            return {"available": False, "error": "no webcam or read failed"}

        h, w = frame.shape[:2]
        detection = detect_largest_ball(frame)
        result: dict[str, object] = {"available": True, "detected": detection is not None, "width": w, "height": h}

        if detection:
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                (0, 220, 100), 2,
            )
            focal_px = (w / 2) / math.tan(math.radians(self._fov_deg / 2))
            diam_px = detection.apparent_diameter_px
            distance_m = (TENNIS_BALL_DIAMETER_M * focal_px) / max(1.0, diam_px)
            normalized_x = (detection.center_x - w / 2) / (w / 2)
            bearing_rad = math.atan(normalized_x * math.tan(math.radians(self._fov_deg / 2)))
            bearing_deg = math.degrees(bearing_rad)
            label = f"{distance_m:.2f}m"
            cv2.putText(frame, label, (detection.x, max(20, detection.y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 100), 2)
            result.update({
                "distance_m": round(distance_m, 3),
                "bearing_deg": round(bearing_deg, 1),
                "diameter_px": round(diam_px, 1),
            })

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        result["data_url"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
        return result
