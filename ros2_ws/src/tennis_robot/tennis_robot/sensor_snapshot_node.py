"""ROS 2 sensor snapshot node for the web control panel.

Subscribes to Gazebo-bridged ROS sensor topics and writes the same
robot_sensors.json payload shape that the existing UI already renders.
"""

from __future__ import annotations

import math
import os
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from tennis_robot.control_bus import RobotSensorStore
from tennis_robot.debug_display import bgra_bmp_data_url
from tennis_robot.lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m
from tennis_robot.perception import detect_obstacle_class, build_survey_vision

_STATUS_FILE = os.getenv("ROBOT_STATUS_FILE", "/workspace/runtime/robot_status.json")

WRITE_INTERVAL_S = float(os.getenv("SENSOR_SNAPSHOT_INTERVAL_S", "1.0"))
LIDAR_FRONT_INDEX_RATIO = float(os.getenv("LIDAR_FRONT_INDEX_RATIO", "0.5"))
LIDAR_FRONT_MIN_OBSTACLE_RANGE_M = float(os.getenv("LIDAR_FRONT_MIN_OBSTACLE_RANGE_M", "0.18"))
IR_THRESHOLD = float(os.getenv("IR_INTAKE_TRIGGER_THRESHOLD", "500.0"))


class SensorSnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_snapshot_node")
        self._sensor_store = RobotSensorStore.from_env()
        self._camera: dict | None = None
        self._depth: dict | None = None
        self._lidar: dict | None = None
        self._lidar_candidates: list[dict] = []
        self._ir_left = 0.0
        self._ir_right = 0.0
        self._last_write_s = 0.0
        self._front_range_m: float = math.inf

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 1)
        self.create_subscription(Image, "/camera/depth", self._on_depth, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 1)
        self.create_subscription(LaserScan, "/gz/ir_left/scan", self._on_ir_left, 10)
        self.create_subscription(LaserScan, "/gz/ir_right/scan", self._on_ir_right, 10)
        self.create_timer(0.1, self._write_if_due)
        self.get_logger().info("sensor_snapshot_node started")

    def _on_image(self, msg: Image) -> None:
        frame = self._decode_color_image(msg)
        if frame is None:
            return
        height, width = frame.shape[:2]
        preview_width = min(width, 960)
        preview_height = max(1, round(height * preview_width / max(1, width)))
        if preview_width != width:
            frame = cv2.resize(frame, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
            height, width = frame.shape[:2]
        self._draw_recognition_overlay(frame)
        bgra = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        self._camera = {
            "width": int(width),
            "height": int(height),
            "native_width": int(msg.width),
            "native_height": int(msg.height),
            "format": "bgra-bmp",
            "data_url": bgra_bmp_data_url(bgra.tobytes(), int(width), int(height)),
        }

    def _draw_recognition_overlay(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        roi_y0, roi_y1 = int(h * 0.06), int(h * 0.74)
        roi_x0, roi_x1 = int(w * 0.05), int(w * 0.95)

        # ── obstacle detection (net / fence) ─────────────────────────────────
        detection = detect_obstacle_class(frame)
        if detection is not None:
            label = detection.label
            color = (80, 220, 255) if label == "net" else (0, 210, 120)
            cv2.rectangle(frame, (roi_x0, roi_y0), (roi_x1, roi_y1), color, 2)
            dist_str = f"{self._front_range_m:.2f}m" if math.isfinite(self._front_range_m) else "—"
            text = f"{label}  {dist_str}  {detection.confidence:.0%}"
            cv2.rectangle(frame, (roi_x0, max(0, roi_y0 - 22)),
                          (roi_x0 + len(text) * 9, roi_y0), color, -1)
            cv2.putText(frame, text, (roi_x0 + 4, max(14, roi_y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 1, cv2.LINE_AA)
        else:
            cv2.rectangle(frame, (roi_x0, roi_y0), (roi_x1, roi_y1), (60, 60, 60), 1)

        # ── court line / baseline detection ──────────────────────────────────
        sv = build_survey_vision(frame)
        if sv.line_detected:
            line_y = int(h * 0.82)
            cv2.line(frame, (roi_x0, line_y), (roi_x1, line_y), (255, 255, 100), 2)
            offset_str = f"{sv.line_offset_m:.2f}m" if sv.line_offset_m is not None else "?"
            conf_str = f"{sv.line_confidence:.0%}"
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
        bgra = np.dstack((intensity, intensity, intensity, np.full_like(intensity, 255)))
        height, width = depth.shape[:2]
        self._depth = {
            "width": int(width),
            "height": int(height),
            "format": "depth-bmp",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "valid_count": int(valid.size),
            "median_range_m": None if valid.size == 0 else float(np.median(valid)),
            "data_url": bgra_bmp_data_url(bgra.tobytes(), int(width), int(height)),
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

    def _on_ir_left(self, msg: LaserScan) -> None:
        self._ir_left = self._range_to_ir_value(msg.ranges[0] if msg.ranges else float("inf"))

    def _on_ir_right(self, msg: LaserScan) -> None:
        self._ir_right = self._range_to_ir_value(msg.ranges[0] if msg.ranges else float("inf"))

    def _write_if_due(self) -> None:
        now = time.time()
        if now - self._last_write_s < WRITE_INTERVAL_S:
            return
        self._last_write_s = now
        self._sensor_store.write({
            "front_camera": self._camera,
            "front_depth": self._depth,
            "front_lidar": self._lidar,
            "lidar_candidates": self._lidar_candidates,
            "ir_intake": {
                "left": self._ir_left,
                "right": self._ir_right,
                "threshold": IR_THRESHOLD,
                "triggered": self._ir_left > IR_THRESHOLD or self._ir_right > IR_THRESHOLD,
                "left_available": True,
                "right_available": True,
            },
        })

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
