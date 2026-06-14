"""ROS 2 node: detects court landmarks (net + corners) from OAK-D and
publishes them for Nav2 Behavior Tree consumption.

Subscriptions
-------------
/camera/image_raw    sensor_msgs/Image  (bgr8)
/camera/depth        sensor_msgs/Image  (32FC1, metres)

Publications
------------
/court_landmarks        std_msgs/String        — JSON snapshot each frame
/court_landmark_poses   geometry_msgs/PoseArray — landmark positions in map frame

The node looks up the TF map→base_link each cycle to transform detections into
map frame.  Nav2 BT nodes can read /court_landmark_poses directly or a BT
action plugin can subscribe to /court_landmarks (JSON) for richer metadata.

Environment variables
---------------------
COURT_LANDMARKS_MIN_CONFIDENCE  float  default 0.25
COURT_LANDMARKS_DEPTH_MIN_M     float  default 0.10
COURT_LANDMARKS_DEPTH_MAX_M     float  default 12.0
COURT_LANDMARKS_CAMERA_FOV_DEG  float  default 69.0  (OAK-D)
"""

from __future__ import annotations

import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

try:
    from tennis_robot.court_landmarks import (
        build_court_landmarks,
        landmarks_to_robot_frame,
        robot_landmarks_to_map,
        CourtLandmarks,
    )
except ModuleNotFoundError:
    from court_landmarks import (
        build_court_landmarks,
        landmarks_to_robot_frame,
        robot_landmarks_to_map,
        CourtLandmarks,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Convert a 2-D yaw angle to a geometry_msgs/Quaternion (z-axis rotation)."""
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


def _landmarks_to_json(lm: CourtLandmarks, stamp: float) -> str:
    def _lm_dict(p):
        return {
            "label": p.label,
            "x_m": round(p.x_m, 3),
            "y_m": round(p.y_m, 3),
            "bearing_rad": round(p.bearing_rad, 4),
            "distance_m": round(p.distance_m, 3),
            "confidence": round(p.confidence, 3),
        }

    payload = {
        "stamp": round(stamp, 3),
        "frame_id": lm.frame_id,
        "net": _lm_dict(lm.net) if lm.net else None,
        "corners": [_lm_dict(c) for c in lm.corners],
    }
    return json.dumps(payload)


def _landmarks_to_pose_array(lm: CourtLandmarks, frame_id: str) -> PoseArray:
    msg = PoseArray()
    msg.header.frame_id = frame_id
    for p in lm.all_landmarks:
        pose = Pose()
        pose.position = Point(x=p.x_m, y=p.y_m, z=0.0)
        # Orientation: face the direction from robot toward landmark
        pose.orientation = _yaw_to_quaternion(p.bearing_rad)
        msg.poses.append(pose)
    return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CourtLandmarksNode(Node):
    def __init__(self) -> None:
        super().__init__("court_landmarks_node")

        self._min_confidence: float = float(
            os.getenv("COURT_LANDMARKS_MIN_CONFIDENCE", "0.25")
        )
        self._depth_min_m: float = float(os.getenv("COURT_LANDMARKS_DEPTH_MIN_M", "0.10"))
        self._depth_max_m: float = float(os.getenv("COURT_LANDMARKS_DEPTH_MAX_M", "12.0"))
        self._camera_fov_rad: float = math.radians(
            float(os.getenv("COURT_LANDMARKS_CAMERA_FOV_DEG", "69.0"))
        )

        self._bridge = CvBridge()
        self._latest_depth: np.ndarray | None = None

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # QoS: best-effort for sensor streams
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, "/camera/depth", self._depth_cb, sensor_qos)
        self.create_subscription(Image, "/camera/image_raw", self._image_cb, sensor_qos)

        self._pub_json = self.create_publisher(String, "/court_landmarks", 10)
        self._pub_poses = self.create_publisher(PoseArray, "/court_landmark_poses", 10)

        self.get_logger().info(
            f"court_landmarks_node started "
            f"(min_confidence={self._min_confidence}, "
            f"fov={math.degrees(self._camera_fov_rad):.0f}°)"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            self.get_logger().warn(f"depth decode error: {exc}", throttle_duration_sec=5.0)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image decode error: {exc}", throttle_duration_sec=5.0)
            return

        depth = self._latest_depth

        # Detect in camera frame
        landmarks_cam = build_court_landmarks(
            frame,
            depth,
            camera_fov_rad=self._camera_fov_rad,
            depth_min_m=self._depth_min_m,
            depth_max_m=self._depth_max_m,
        )

        # Filter by confidence
        filtered_corners = [
            c for c in landmarks_cam.corners if c.confidence >= self._min_confidence
        ]
        filtered_net = (
            landmarks_cam.net
            if landmarks_cam.net and landmarks_cam.net.confidence >= self._min_confidence
            else None
        )
        from tennis_robot.court_landmarks import CourtLandmarks as _CL
        landmarks_cam = _CL(
            net=filtered_net,
            corners=filtered_corners,
            frame_id="camera_link",
        )

        # Project to robot frame
        landmarks_robot = landmarks_to_robot_frame(landmarks_cam)

        # Look up robot pose in map frame
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        robot_x, robot_y, robot_yaw = self._get_robot_pose_in_map()

        if robot_x is not None:
            landmarks_map = robot_landmarks_to_map(
                landmarks_robot, robot_x, robot_y, robot_yaw
            )
            map_frame = "map"
        else:
            # TF not yet available — publish in base_link frame as fallback
            landmarks_map = landmarks_robot
            map_frame = "base_link"

        # Publish JSON
        json_msg = String()
        json_msg.data = _landmarks_to_json(landmarks_map, stamp_s)
        self._pub_json.publish(json_msg)

        # Publish PoseArray for Nav2
        if landmarks_map.all_landmarks:
            pose_array = _landmarks_to_pose_array(landmarks_map, map_frame)
            pose_array.header.stamp = msg.header.stamp
            self._pub_poses.publish(pose_array)

    # ------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------

    def _get_robot_pose_in_map(self) -> tuple[float | None, float | None, float | None]:
        """Return (x_m, y_m, yaw_rad) of base_link in map frame, or (None,None,None)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None, None, None

        t = tf.transform.translation
        q = tf.transform.rotation
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(t.x), float(t.y), float(yaw)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CourtLandmarksNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
