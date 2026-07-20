"""ROS 2 perception node: simulated OAK-D AI pipeline.

This node emulates the OAK-D's on-device AI. It treats Gazebo purely as the
image source and runs the *full* pipeline a real OAK-D would run internally:

    RGB frame ──▶ neural detector (YOLOv8/v11n, ONNX Runtime) ──▶ 2D boxes
                                                                    │
    depth frame ───────────────────────────────────────────▶ depth fusion
                                                                    │
                                                  bearing + distance + confidence
                                                                    │
                                                                    │
                                                                    ▼
                                      /perception/ball_detections
                                      (BallDetectionArray, canonical contract)

The neural detector is mandatory. A missing or invalid model fails node startup;
there is no classical detector fallback.

Because the published message interface is identical to what the real OAK-D
DepthAI pipeline will expose, the Collector / Nav2 / Behaviour Tree consume
these topics without knowing whether detections came from Gazebo or hardware.

Subscribes:
  /camera/image_raw  (sensor_msgs/Image)
  /camera/depth      (sensor_msgs/Image, 32FC1)
Publishes:
  /perception/ball_detections(tennis_robot_msgs/BallDetectionArray) — DepthAI-equiv
  /survey/vision             (std_msgs/String, JSON-serialised SurveyVision fields)
"""

from __future__ import annotations

import json
import math
import os

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tennis_robot.ball_detector import load_ball_detector
from tennis_robot.perception import (
    build_survey_vision,
    camera_frame_position,
    estimate_depth_ball_observation,
    depth_roi_quality,
    pixel_elevation_rad,
)
from tennis_robot.perception_covariance_calibration import (
    evaluate_producer_spatial_covariance,
    load_spatial_calibration_runtime,
)

DEPTH_MIN_RANGE = float(os.getenv("DEPTH_MIN_RANGE_M", "0.1"))
DEPTH_MAX_RANGE = float(os.getenv("DEPTH_MAX_RANGE_M", "10.0"))
CAMERA_FOV_RAD = float(os.getenv("CAMERA_FOV_RAD", str(math.radians(60))))
CAMERA_FRAME_ID = os.getenv("CAMERA_FRAME_ID", "camera_link_optical_frame")
MAX_PUBLISHED_BALLS = int(os.getenv("PERCEPTION_MAX_BALLS", "8"))
RGB_DEPTH_SYNC_SLOP_S = float(os.getenv("RGB_DEPTH_SYNC_SLOP_S", "0.05"))
RGB_DEPTH_SYNC_QUEUE_SIZE = int(os.getenv("RGB_DEPTH_SYNC_QUEUE_SIZE", "10"))


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        # Primary perception: neural detector emulating the OAK-D on-device AI.
        self._detector = load_ball_detector(logger=self.get_logger())
        runtime = load_spatial_calibration_runtime(
            os.getenv("PERCEPTION_COVARIANCE_CALIBRATION_ARTIFACT"),
            expected_platform=os.getenv("PERCEPTION_CALIBRATION_PLATFORM", "oak_d"),
            expected_calibration_id=os.getenv("PERCEPTION_COVARIANCE_CALIBRATION_ID") or None,
            expected_model_version=os.getenv("PERCEPTION_COVARIANCE_MODEL_VERSION") or None,
            required_path=os.getenv("PERCEPTION_COVARIANCE_REQUIRED_ARTIFACT") or None,
        )
        self._spatial_targets_healthy = runtime.healthy
        self._spatial_targets_health_reason = runtime.health_reason
        self._covariance_model = runtime.model
        self._spatial_targets_artifact_id = runtime.artifact_id
        self._spatial_targets_artifact_version = runtime.artifact_version

        self._rgb_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw", qos_profile=1
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth", qos_profile=1
        )
        self._rgb_depth_sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=RGB_DEPTH_SYNC_QUEUE_SIZE,
            slop=RGB_DEPTH_SYNC_SLOP_S,
        )
        self._rgb_depth_sync.registerCallback(self._on_rgb_depth)

        msgs = __import__("tennis_robot_msgs.msg", fromlist=["BallDetectionArray"])
        self._pub_survey = self.create_publisher(String, "/survey/vision", 1)
        self._pub_detections = self.create_publisher(
            msgs.BallDetectionArray, "/perception/ball_detections", 10
        )

        self.get_logger().info(
            f"perception_node started (detector={self._detector.name}, "
            f"fov={CAMERA_FOV_RAD:.3f} rad, "
            f"rgb_depth_slop={RGB_DEPTH_SYNC_SLOP_S:.3f}s, "
            f"spatial_targets_healthy={self._spatial_targets_healthy}, "
            f"spatial_targets_health_reason={self._spatial_targets_health_reason}, "
            f"calibration_id={self._spatial_targets_artifact_id}, "
            f"calibration_version={self._spatial_targets_artifact_version})"
        )

    # -- subscriptions ------------------------------------------------------
    def _on_rgb_depth(self, image_msg: Image, depth_msg: Image) -> None:
        """Process one timestamp-matched RGB/depth acquisition pair."""
        frame = self._decode_image(image_msg)
        if frame is None:
            return
        depth = self._decode_depth(depth_msg)
        if depth is None:
            return

        # Single neural inference per synchronized acquisition, shared by all
        # publishers. Identical pixels are still processed so consumers receive
        # a fresh heartbeat and explicit empty detections.
        detections = self._detector.detect(frame)
        fused = self._fuse_detections(
            detections, depth, image_msg.width, image_msg.height
        )

        self._publish_detection_array(fused, image_msg.header.stamp, depth_msg.header.stamp)
        self._publish_survey_vision(frame, depth)

    # -- decoding -----------------------------------------------------------
    def _decode_image(self, msg: Image) -> np.ndarray | None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "bgra8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_BGRA2BGR)
        if msg.encoding == "rgba8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_RGBA2BGR)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return arr.reshape((msg.height, msg.width, 3))
        self.get_logger().warn(f"unsupported image encoding: {msg.encoding}")
        return None

    def _decode_depth(self, msg: Image) -> np.ndarray | None:
        if msg.encoding not in {"32FC1", "32FC"}:
            self.get_logger().warn(f"unsupported depth encoding: {msg.encoding}")
            return None
        expected = msg.width * msg.height
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size != expected:
            self.get_logger().warn(
                f"invalid depth image size: got {arr.size}, expected {expected}"
            )
            return None
        return arr.reshape((msg.height, msg.width))

    # -- fusion -------------------------------------------------------------
    def _fuse_detections(
        self, detections: list, depth: np.ndarray, w: int, h: int
    ) -> list[dict]:
        """Fuse each 2D detection with the depth frame into a spatial record.

        Returns dicts sorted nearest-first. Detections with no valid depth are
        kept (has_spatial=False) so the detector's 2D confidence is still
        surfaced — depth fusion failing is not the same as no detection.
        """
        vertical_fov = CAMERA_FOV_RAD * (h / max(1, w))
        records: list[dict] = []
        for det in detections[:MAX_PUBLISHED_BALLS]:
            rec: dict = {
                "detection": det,
                "confidence": float(getattr(det, "confidence", 1.0)),
                "has_spatial": False,
                "bearing_rad": 0.0,
                "distance_m": float("inf"),
                "pos": (0.0, 0.0, 0.0),
            }
            ball_obs = estimate_depth_ball_observation(
                det, depth, w, h, CAMERA_FOV_RAD
            )
            if ball_obs is not None and self._covariance_model is not None:
                elevation = pixel_elevation_rad(det.center_y, h, vertical_fov)
                pos = camera_frame_position(
                    ball_obs.bearing_rad, ball_obs.distance_m, elevation
                )
                quality = depth_roi_quality(det, depth, w, h)
                covariance = evaluate_producer_spatial_covariance(
                    self._covariance_model, pos, quality
                )
                if covariance.covariance is None:
                    records.append(rec)
                    continue
                rec.update(
                    has_spatial=True,
                    bearing_rad=float(ball_obs.bearing_rad),
                    distance_m=float(ball_obs.distance_m),
                    pos=pos,
                    covariance=covariance.covariance,
                )
            records.append(rec)
        records.sort(key=lambda r: (not r["has_spatial"], r["distance_m"]))
        return records

    # -- publishers ---------------------------------------------------------
    def _publish_detection_array(self, fused: list[dict], stamp, depth_stamp) -> None:
        from tennis_robot_msgs.msg import BallDetection, BallDetectionArray

        arr = BallDetectionArray()
        arr.header.stamp = stamp
        arr.header.frame_id = CAMERA_FRAME_ID
        arr.spatial_targets_healthy = self._spatial_targets_healthy
        arr.spatial_targets_health_reason = self._spatial_targets_health_reason
        arr.calibration_id = self._spatial_targets_artifact_id or ""
        arr.configuration_id = self._spatial_targets_artifact_version or ""
        for r in fused:
            det = r["detection"]
            m = BallDetection()
            m.confidence = float(r["confidence"])
            m.bbox_center_x = float(det.center_x)
            m.bbox_center_y = float(det.center_y)
            m.bbox_width = float(det.width)
            m.bbox_height = float(det.height)
            m.has_spatial = bool(r["has_spatial"] and self._spatial_targets_healthy)
            if m.has_spatial:
                m.matched_depth_stamp = depth_stamp
                m.position_covariance = [float(value) for value in r["covariance"]]
                m.bearing_rad = float(r["bearing_rad"])
                m.distance_m = float(r["distance_m"])
                m.position_x, m.position_y, m.position_z = (float(v) for v in r["pos"])
            arr.detections.append(m)
        self._pub_detections.publish(arr)

    def _publish_survey_vision(self, frame: np.ndarray, depth: np.ndarray) -> None:
        sv = build_survey_vision(frame, depth, DEPTH_MIN_RANGE, DEPTH_MAX_RANGE)
        payload = {
            "line_detected": sv.line_detected,
            "line_offset_m": sv.line_offset_m,
            "line_heading_error_rad": sv.line_heading_error_rad,
            "line_confidence": sv.line_confidence,
            "corner_detected": sv.corner_detected,
            "corner_confidence": sv.corner_confidence,
            "center_m": sv.center_m,
            "left_m": sv.left_m,
            "right_m": sv.right_m,
            "valid_count": sv.valid_count,
            "obstacle_class": sv.obstacle_class,
            "junction_type": sv.junction_type,
            "junction_distance_m": sv.junction_distance_m,
            "junction_bearing_rad": sv.junction_bearing_rad,
            "junction_confidence": sv.junction_confidence,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_survey.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(PerceptionNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
