"""ROS 2 sensor snapshot node for the web control panel.

Subscribes to Gazebo-bridged ROS sensor topics and writes the same
robot_sensors.json payload shape that the existing UI already renders.
"""

from __future__ import annotations

import json
import base64
import math
import os
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from tennis_robot.control_bus import RobotSensorStore
from tennis_robot.debug_display import bgra_bmp_data_url
from tennis_robot.lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m

_STATUS_FILE = os.getenv(
    "ROBOT_STATUS_FILE",
    os.path.join(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"), "runtime", "robot_status.json"),
)

WRITE_INTERVAL_S = float(os.getenv("SENSOR_SNAPSHOT_INTERVAL_S", "1.0"))
SNAPSHOT_MODE = os.getenv("SENSOR_SNAPSHOT_MODE", "local").strip().lower()
PREVIEW_TOPIC = os.getenv("SENSOR_SNAPSHOT_PREVIEW_TOPIC", "/telemetry/sensor_snapshot")
PREVIEW_WIDTH = int(os.getenv("SENSOR_SNAPSHOT_PREVIEW_WIDTH", "320"))
PREVIEW_JPEG_QUALITY = int(os.getenv("SENSOR_SNAPSHOT_JPEG_QUALITY", "65"))
LIDAR_FRONT_INDEX_RATIO = float(os.getenv("LIDAR_FRONT_INDEX_RATIO", "0.5"))
LIDAR_FRONT_MIN_OBSTACLE_RANGE_M = float(os.getenv("LIDAR_FRONT_MIN_OBSTACLE_RANGE_M", "0.18"))
IR_THRESHOLD = float(os.getenv("IR_INTAKE_TRIGGER_THRESHOLD", "500.0"))
IR_CONFIRM_SYMMETRY_MAX_DELTA = float(
    os.getenv("BEAM_SYMMETRY_MAX_DELTA", "200.0")
)


class SensorSnapshotNode(Node):
    def __init__(self) -> None:
        if SNAPSHOT_MODE not in {"local", "publisher", "receiver"}:
            raise RuntimeError(
                f"invalid SENSOR_SNAPSHOT_MODE={SNAPSHOT_MODE!r}; "
                "use local, publisher, or receiver"
            )
        node_name = (
            "sensor_snapshot_node"
            if SNAPSHOT_MODE == "local"
            else f"sensor_snapshot_{SNAPSHOT_MODE}"
        )
        super().__init__(node_name)
        self._sensor_store = RobotSensorStore.from_env()
        self._camera: dict | None = None
        self._depth: dict | None = None
        self._lidar: dict | None = None
        self._lidar_candidates: list[dict] = []
        self._entry_ir_left = 0.0
        self._entry_ir_right = 0.0
        self._confirmed_ir_left = 0.0
        self._confirmed_ir_right = 0.0
        self._entry_left_available = False
        self._entry_right_available = False
        self._confirmed_left_available = False
        self._confirmed_right_available = False
        self._entry_broken = False
        self._confirmed_broken = False
        self._entry_crossing_count = 0
        self._confirmed_crossing_count = 0
        self._entry_last_crossing_at_s: float | None = None
        self._confirmed_last_crossing_at_s: float | None = None
        self._last_write_s = 0.0
        self._front_range_m: float = math.inf
        self._survey_vision: dict = {}

        preview_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._preview_pub = None
        if SNAPSHOT_MODE == "receiver":
            self.create_subscription(
                String, PREVIEW_TOPIC, self._on_preview, preview_qos
            )
            self.get_logger().info(
                f"sensor_snapshot_node started (receiver: {PREVIEW_TOPIC})"
            )
            return
        if SNAPSHOT_MODE == "publisher":
            self._preview_pub = self.create_publisher(
                String, PREVIEW_TOPIC, preview_qos
            )

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 1)
        self.create_subscription(Image, "/camera/depth", self._on_depth, 1)
        self.create_subscription(String, "/survey/vision", self._on_survey_vision, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 1)
        self.create_subscription(
            LaserScan, "/gz/ir_left/scan", self._on_entry_ir_left, 10
        )
        self.create_subscription(
            LaserScan, "/gz/ir_right/scan", self._on_entry_ir_right, 10
        )
        self.create_subscription(
            LaserScan,
            "/gz/basket_ir_left/scan",
            self._on_confirmed_ir_left,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/gz/basket_ir_right/scan",
            self._on_confirmed_ir_right,
            10,
        )
        self.create_timer(0.1, self._write_if_due)
        self.get_logger().info(
            f"sensor_snapshot_node started ({SNAPSHOT_MODE})"
        )

    def _on_preview(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict) or payload.get("schema") != "sensor_preview/v1":
            return
        payload.pop("schema", None)
        payload["transport"] = {
            "mode": "compressed_dds_preview",
            "topic": PREVIEW_TOPIC,
            "received_at": time.time(),
        }
        self._sensor_store.write(payload)

    def _on_survey_vision(self, msg: String) -> None:
        try:
            self._survey_vision = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._survey_vision = {}

    def _on_image(self, msg: Image) -> None:
        frame = self._decode_color_image(msg)
        if frame is None:
            return
        height, width = frame.shape[:2]
        preview_width = min(width, max(160, PREVIEW_WIDTH))
        preview_height = max(1, round(height * preview_width / max(1, width)))
        if preview_width != width:
            frame = cv2.resize(frame, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
            height, width = frame.shape[:2]
        self._draw_recognition_overlay(frame)
        self._camera = {
            "width": int(width),
            "height": int(height),
            "native_width": int(msg.width),
            "native_height": int(msg.height),
            "format": "jpeg",
            "data_url": self._jpeg_data_url(frame),
        }

    def _draw_recognition_overlay(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        vision = self._survey_vision
        source_width = max(1, int(vision.get("image_width") or w))
        source_height = max(1, int(vision.get("image_height") or h))
        scale_x = w / source_width
        scale_y = h / source_height
        detections = vision.get("court_scene_detections") or []
        for detection in detections:
            bbox = detection.get("bbox") or {}
            label = str(detection.get("label") or "")
            color = (80, 220, 255) if label == "net" else (0, 210, 120)
            x0 = int(float(bbox.get("x") or 0) * scale_x)
            y0 = int(float(bbox.get("y") or 0) * scale_y)
            x1 = x0 + int(float(bbox.get("width") or 0) * scale_x)
            y1 = y0 + int(float(bbox.get("height") or 0) * scale_y)
            cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
            confidence = float(detection.get("confidence") or 0.0)
            distance = detection.get("distance_m")
            distance_text = f"{float(distance):.2f}m" if distance is not None else "—"
            cv2.putText(
                frame,
                f"{label} {distance_text} {confidence:.0%}",
                (x0 + 4, max(16, y0 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                1,
                cv2.LINE_AA,
            )

        confirmed = vision.get("obstacle_class")
        candidate = vision.get("obstacle_candidate_class")
        semantic_text = (
            f"neural:{confirmed}"
            if confirmed
            else f"neural candidate:{candidate or 'none'}"
        )
        cv2.putText(
            frame,
            semantic_text,
            (8, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (80, 220, 255),
            1,
            cv2.LINE_AA,
        )

        # ── court line / baseline telemetry from the perception node ─────────
        if bool(vision.get("line_detected")):
            line_y = int(h * 0.82)
            roi_x0, roi_x1 = int(w * 0.05), int(w * 0.95)
            cv2.line(frame, (roi_x0, line_y), (roi_x1, line_y), (255, 255, 100), 2)
            line_offset = vision.get("line_offset_m")
            offset_str = f"{float(line_offset):.2f}m" if line_offset is not None else "?"
            conf_str = f"{float(vision.get('line_confidence') or 0.0):.0%}"
            cv2.putText(frame, f"baseline  off={offset_str}  {conf_str}",
                        (roi_x0+ 4, line_y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 100), 1, cv2.LINE_AA)

        # ── survey state (read from status file, cached 1 s) ─────────────────
        survey_text = self._survey_state_text()
        if survey_text:
            cv2.putText(frame, survey_text, (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 255), 1, cv2.LINE_AA)

        # ── LiDAR front range ─────────────────────────────────────────────────
        if math.isfinite(self._front_range_m):
            cv2.putText(frame, f"LiDAR {self._front_range_m:.2f}m", (8, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 80), 1, cv2.LINE_AA)

    _survey_state_cache: str = ""
    _survey_state_ts: float = 0.0

    def _survey_state_text(self) -> str:
        now = time.time()
        if now - self._survey_state_ts < 1.0:
            return self._survey_state_cache
        self._survey_state_ts = now
        try:
            import json
            with open(_STATUS_FILE, "r", encoding="utf-8") as f:
                status = json.load(f)
            nav = (status.get("survey") or {}).get("navigation") or {}
            state = nav.get("state", "")
            event = nav.get("last_event", "")
            obs = nav.get("obstacle_survey") or {}
            obs_state = obs.get("state", "")
            parts = []
            if state:
                parts.append(f"survey:{state}")
            if obs_state and obs_state != state:
                parts.append(f"obs:{obs_state}")
            if event:
                parts.append(event[:40])
            self._survey_state_cache = "  |  ".join(parts)
        except Exception:
            self._survey_state_cache = ""
        return self._survey_state_cache

    def _on_depth(self, msg: Image) -> None:
        depth = self._decode_depth_image(msg)
        if depth is None:
            return
        valid = depth[np.isfinite(depth) & (depth > 0)]
        min_range = float(np.min(valid)) if valid.size else 0.0
        max_range = float(np.percentile(valid, 95)) if valid.size else 10.0
        span = max(0.001, max_range - min_range)
        clipped = np.where(np.isfinite(depth) & (depth > 0), depth, max_range)
        normalized = np.clip((clipped - min_range) / span, 0.0, 1.0)
        intensity = ((1.0 - normalized) * 255).astype(np.uint8)
        preview_width = min(intensity.shape[1], max(160, PREVIEW_WIDTH))
        preview_height = max(
            1, round(intensity.shape[0] * preview_width / max(1, intensity.shape[1]))
        )
        if preview_width != intensity.shape[1]:
            intensity = cv2.resize(
                intensity,
                (preview_width, preview_height),
                interpolation=cv2.INTER_AREA,
            )
        bgr = cv2.cvtColor(intensity, cv2.COLOR_GRAY2BGR)
        height, width = depth.shape[:2]
        self._depth = {
            "width": int(preview_width),
            "height": int(preview_height),
            "native_width": int(width),
            "native_height": int(height),
            "format": "depth-jpeg",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "valid_count": int(valid.size),
            "median_range_m": None if valid.size == 0 else float(np.median(valid)),
            "data_url": self._jpeg_data_url(bgr),
        }

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = [float(r) for r in msg.ranges]
        if not ranges:
            return
        width = len(ranges)
        height = 64
        min_range = float(msg.range_min)
        max_range = float(msg.range_max)
        span = max(0.001, max_range - min_range)
        pixels = bytearray()
        ranges_m: list[float | None] = []
        normalized_ranges: list[float] = []
        for value in ranges:
            if math.isfinite(value) and value > 0:
                normalized_ranges.append(max(0.0, min(1.0, (value - min_range) / span)))
                ranges_m.append(value)
            else:
                normalized_ranges.append(1.0)
                ranges_m.append(None)
        for y in range(height):
            for normalized in normalized_ranges:
                bar = max(1, int((1.0 - normalized) * (height - 1)))
                pixels.extend((80, 220, 120, 255) if y >= height - bar else (18, 24, 28, 255))

        self._lidar = {
            "width": width,
            "height": height,
            "format": "lidar-bmp",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "angle_min_rad": float(msg.angle_min),
            "angle_max_rad": float(msg.angle_max),
            "angle_increment_rad": float(msg.angle_increment),
            "front_range_m": lidar_front_range_m(
                ranges, LIDAR_FRONT_INDEX_RATIO, LIDAR_FRONT_MIN_OBSTACLE_RANGE_M
            ),
            "ranges_m": ranges_m,
            "data_url": bgra_bmp_data_url(bytes(pixels), width, height),
        }
        fr = lidar_front_range_m(ranges, LIDAR_FRONT_INDEX_RATIO, LIDAR_FRONT_MIN_OBSTACLE_RANGE_M)
        self._front_range_m = fr if fr is not None and math.isfinite(fr) else math.inf
        self._lidar_candidates = [
            {"robot_x_m": cx, "robot_y_m": cy, "distance_m": math.hypot(cx, cy)}
            for cx, cy in extract_ball_candidates(ranges)
        ]

    def _on_entry_ir_left(self, msg: LaserScan) -> None:
        self._entry_ir_left = self._range_to_ir_value(
            msg.ranges[0] if msg.ranges else float("inf")
        )
        self._entry_left_available = True
        self._update_ir_crossings()

    def _on_entry_ir_right(self, msg: LaserScan) -> None:
        self._entry_ir_right = self._range_to_ir_value(
            msg.ranges[0] if msg.ranges else float("inf")
        )
        self._entry_right_available = True
        self._update_ir_crossings()

    def _on_confirmed_ir_left(self, msg: LaserScan) -> None:
        self._confirmed_ir_left = self._range_to_ir_value(
            msg.ranges[0] if msg.ranges else float("inf")
        )
        self._confirmed_left_available = True
        self._update_ir_crossings()

    def _on_confirmed_ir_right(self, msg: LaserScan) -> None:
        self._confirmed_ir_right = self._range_to_ir_value(
            msg.ranges[0] if msg.ranges else float("inf")
        )
        self._confirmed_right_available = True
        self._update_ir_crossings()

    def _update_ir_crossings(self) -> None:
        entry_broken = (
            self._entry_ir_left > IR_THRESHOLD
            or self._entry_ir_right > IR_THRESHOLD
        )
        confirmed_broken = (
            self._confirmed_ir_left > IR_THRESHOLD
            and self._confirmed_ir_right > IR_THRESHOLD
            and abs(self._confirmed_ir_left - self._confirmed_ir_right)
            <= IR_CONFIRM_SYMMETRY_MAX_DELTA
        )
        now_s = time.time()
        if entry_broken and not self._entry_broken:
            self._entry_crossing_count += 1
            self._entry_last_crossing_at_s = now_s
        if confirmed_broken and not self._confirmed_broken:
            self._confirmed_crossing_count += 1
            self._confirmed_last_crossing_at_s = now_s
        self._entry_broken = entry_broken
        self._confirmed_broken = confirmed_broken

    def _write_if_due(self) -> None:
        now = time.time()
        if now - self._last_write_s < WRITE_INTERVAL_S:
            return
        self._last_write_s = now
        lidar = self._lidar
        if self._preview_pub is not None and isinstance(lidar, dict):
            # The UI renders the scan from ranges_m. Do not send the legacy BMP
            # over DDS; it is typically ~170 kB while the numeric scan is ~7 kB.
            lidar = {key: value for key, value in lidar.items() if key != "data_url"}
        entry_broken = self._entry_broken
        confirmed_broken = self._confirmed_broken
        entry_available = (
            self._entry_left_available and self._entry_right_available
        )
        confirmed_available = (
            self._confirmed_left_available and self._confirmed_right_available
        )
        payload = {
            "front_camera": self._camera,
            "front_depth": self._depth,
            "front_lidar": lidar,
            "lidar_candidates": self._lidar_candidates,
            "ir_intake": {
                "entry": {
                    "broken": entry_broken,
                    "available": entry_available,
                    "left_raw": self._entry_ir_left,
                    "right_raw": self._entry_ir_right,
                    "crossing_count": self._entry_crossing_count,
                    "last_crossing_at_s": self._entry_last_crossing_at_s,
                },
                "confirmed": {
                    "broken": confirmed_broken,
                    "available": confirmed_available,
                    "left_raw": self._confirmed_ir_left,
                    "right_raw": self._confirmed_ir_right,
                    "symmetry_delta": abs(
                        self._confirmed_ir_left - self._confirmed_ir_right
                    ),
                    "crossing_count": self._confirmed_crossing_count,
                    "last_crossing_at_s": self._confirmed_last_crossing_at_s,
                },
                "threshold": IR_THRESHOLD,
                "symmetry_max_delta": IR_CONFIRM_SYMMETRY_MAX_DELTA,
                # Legacy aliases retained for older panels during rolling
                # deployment. They describe the physical entry beam pair.
                "left": self._entry_ir_left,
                "right": self._entry_ir_right,
                "triggered": entry_broken,
                "left_available": self._entry_left_available,
                "right_available": self._entry_right_available,
            },
        }
        if self._preview_pub is not None:
            payload["schema"] = "sensor_preview/v1"
            payload["published_at"] = now
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._preview_pub.publish(message)
        else:
            payload["transport"] = {"mode": "local"}
            self._sensor_store.write(payload)

    @staticmethod
    def _jpeg_data_url(frame: np.ndarray) -> str:
        quality = max(35, min(90, PREVIEW_JPEG_QUALITY))
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            return ""
        return (
            "data:image/jpeg;base64,"
            + base64.b64encode(encoded.tobytes()).decode("ascii")
        )

    def _decode_color_image(self, msg: Image) -> np.ndarray | None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "bgra8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_BGRA2BGR)
        if msg.encoding == "rgba8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_RGBA2BGR)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return arr.reshape((msg.height, msg.width, 3))
        self.get_logger().warn(f"unsupported color image encoding: {msg.encoding}")
        return None

    def _decode_depth_image(self, msg: Image) -> np.ndarray | None:
        raw = bytes(msg.data)
        if msg.encoding == "32FC1":
            return np.array(struct.unpack(f"{msg.width * msg.height}f", raw), dtype=np.float32).reshape(
                (msg.height, msg.width)
            )
        if msg.encoding == "16UC1":
            return np.frombuffer(raw, dtype=np.uint16).reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
        self.get_logger().warn(f"unsupported depth image encoding: {msg.encoding}")
        return None

    @staticmethod
    def _range_to_ir_value(range_m: float) -> float:
        max_range_m = 0.22
        if not math.isfinite(range_m) or range_m > max_range_m:
            return 0.0
        return 1000.0 * max(0.0, 1.0 - range_m / max_range_m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorSnapshotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
