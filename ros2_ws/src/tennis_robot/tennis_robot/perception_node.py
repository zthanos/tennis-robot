"""ROS 2 perception node: camera images → BallObservation + survey vision JSON.

Subscribes:
  /camera/image_raw  (sensor_msgs/Image)
  /camera/depth      (sensor_msgs/Image, 32FC1)
  /odom              (nav_msgs/Odometry)  — for world coordinate transform

Publishes:
  /ball/observation  (tennis_robot_msgs/BallObservation)
  /survey/vision     (std_msgs/String, JSON-serialised SurveyVision fields)
"""

from __future__ import annotations

import json
import math
import os
import struct

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tennis_robot import yaw_from_quaternion

from tennis_robot.perception import (
    CameraMount,
    RobotPose2D,
    build_survey_vision,
    detect_largest_ball,
    estimate_depth_ball_observation,
    observation_to_world,
)

FRONT_CAMERA_MOUNT = CameraMount(x_m=0.535, y_m=0.0, yaw_rad=0.0)
DEPTH_MIN_RANGE = float(os.getenv("DEPTH_MIN_RANGE_M", "0.1"))
DEPTH_MAX_RANGE = float(os.getenv("DEPTH_MAX_RANGE_M", "10.0"))
CAMERA_FOV_RAD = float(os.getenv("CAMERA_FOV_RAD", str(math.radians(60))))


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self._depth: np.ndarray | None = None
        self._depth_w = 640
        self._depth_h = 480
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 1)
        self.create_subscription(Image, "/camera/depth", self._on_depth, 1)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self._pub_obs = self.create_publisher(
            __import__("tennis_robot_msgs.msg", fromlist=["BallObservation"]).BallObservation,
            "/ball/observation",
            10,
        )
        self._pub_survey = self.create_publisher(String, "/survey/vision", 1)

        self.get_logger().info("perception_node started")

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_depth(self, msg: Image) -> None:
        raw = bytes(msg.data)
        arr = np.array(struct.unpack(f"{msg.width * msg.height}f", raw), dtype=np.float32)
        self._depth = arr.reshape((msg.height, msg.width))
        self._depth_w = msg.width
        self._depth_h = msg.height

    def _on_image(self, msg: Image) -> None:
        frame = self._decode_image(msg)
        if frame is None:
            return
        self._publish_ball_observation(frame, msg.width, msg.height)
        self._publish_survey_vision(frame)

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

    def _publish_ball_observation(self, frame: np.ndarray, w: int, h: int) -> None:
        from tennis_robot_msgs.msg import BallObservation

        obs = BallObservation()
        detection = detect_largest_ball(frame)

        if detection is None or self._depth is None:
            obs.visible = False
            obs.source = "no_detection" if detection is None else "depth_unavailable"
            self._pub_obs.publish(obs)
            return

        ball_obs = estimate_depth_ball_observation(
            detection, self._depth, w, h, CAMERA_FOV_RAD
        )
        if ball_obs is None:
            obs.visible = False
            obs.source = "depth_estimation_failed"
            self._pub_obs.publish(obs)
            return

        world = observation_to_world(
            ball_obs,
            RobotPose2D(self._robot_x, self._robot_y, self._robot_yaw),
            FRONT_CAMERA_MOUNT,
        )
        obs.visible = True
        obs.bearing_rad = float(ball_obs.bearing_rad)
        obs.distance_m = float(ball_obs.distance_m)
        obs.confidence = float(min(1.0, detection.area_px / 6000))
        obs.source = ball_obs.distance_source
        obs.world_x_m = float(world.world_x_m or 0.0)
        obs.world_y_m = float(world.world_y_m or 0.0)
        obs.robot_x_m = float(world.robot_x_m or 0.0)
        obs.robot_y_m = float(world.robot_y_m or 0.0)
        self._pub_obs.publish(obs)

    def _publish_survey_vision(self, frame: np.ndarray) -> None:
        sv = build_survey_vision(frame, self._depth, DEPTH_MIN_RANGE, DEPTH_MAX_RANGE)
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
