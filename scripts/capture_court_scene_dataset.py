#!/usr/bin/env python3
"""Capture auto-labelled Gazebo RGB frames for the net/fence YOLO model.

This is a simulation-only data tool.  It projects the known tennis-court net
and perimeter fence planes into the simulated OAK-D image using Gazebo's
ground-truth robot pose plus the static ``base_link -> camera optical``
transform, then writes Ultralytics images and labels.  Ground truth is required
here because wheel/SLAM drift would silently corrupt training labels.  It never
participates in runtime perception.

Start a sim without perception, drive the robot through varied viewpoints, and
run this node:

    TENNIS_LAUNCH_SIM=true TENNIS_LAUNCH_BRAIN=false \
      TENNIS_PERCEPTION_ON_PC=false ros2 launch tennis_robot sim.launch.py

    ros2 run tennis_robot ...  # source the workspace, then:
    python3 scripts/capture_court_scene_dataset.py \
      --output datasets/court_scene --max-images 1200

Court coordinates match ``gazebo/models/tennis_court/model.sdf``.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import cv2
import numpy as np


CLASS_IDS = {"net": 0, "fence": 1}


def _quaternion_matrix(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = x * x + y * y + z * z + w * w
    if norm <= 0.0:
        return np.eye(3)
    scale = 2.0 / norm
    return np.asarray(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _court_surfaces() -> list[tuple[str, list[tuple[float, float, float]]]]:
    """Return world-frame corners for the semantic court planes."""

    return [
        ("net", [(0, -5.65, 0), (0, 5.65, 0), (0, 5.65, 0.914), (0, -5.65, 0.914)]),
        ("fence", [(16.5, -8.5, 0), (16.5, 8.5, 0), (16.5, 8.5, 4), (16.5, -8.5, 4)]),
        ("fence", [(-16.5, -8.5, 0), (-16.5, 8.5, 0), (-16.5, 8.5, 4), (-16.5, -8.5, 4)]),
        ("fence", [(-16.54, 8.5, 0), (16.54, 8.5, 0), (16.54, 8.5, 4), (-16.54, 8.5, 4)]),
        # South side is split into two 4 m panels plus two 2.2 m gate leaves.
        ("fence", [(-16.54, -8.5, 0), (-1.0, -8.5, 0), (-1.0, -8.5, 4), (-16.54, -8.5, 4)]),
        ("fence", [(1.0, -8.5, 0), (16.54, -8.5, 0), (16.54, -8.5, 4), (1.0, -8.5, 4)]),
        ("fence", [(-1.0, -8.5, 0), (0.0, -8.5, 0), (0.0, -8.5, 2.2), (-1.0, -8.5, 2.2)]),
        ("fence", [(0.0, -8.5, 0), (1.0, -8.5, 0), (1.0, -8.5, 2.2), (0.0, -8.5, 2.2)]),
    ]


def project_surface_box(
    points_odom: list[np.ndarray],
    rotation_camera_from_odom: np.ndarray,
    translation_camera_from_odom: np.ndarray,
    width: int,
    height: int,
    horizontal_fov_rad: float,
    *,
    near_m: float = 0.08,
) -> tuple[float, float, float, float] | None:
    """Project the visible part of a rectangular plane to a clipped YOLO box."""

    camera_points = [
        rotation_camera_from_odom @ point + translation_camera_from_odom
        for point in points_odom
    ]
    # Clip the polygon against the camera near plane. Rejecting a whole surface
    # when one corner is behind the camera creates false-negative labels for a
    # nearby fence that is still plainly visible in most of the image.
    clipped: list[np.ndarray] = []
    previous = camera_points[-1]
    previous_inside = previous[2] >= near_m
    for current in camera_points:
        current_inside = current[2] >= near_m
        if current_inside != previous_inside:
            fraction = (near_m - previous[2]) / (current[2] - previous[2])
            clipped.append(previous + fraction * (current - previous))
        if current_inside:
            clipped.append(current)
        previous = current
        previous_inside = current_inside
    if len(clipped) < 3:
        return None
    focal_x = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    focal_y = focal_x
    pixels = [
        (
            width * 0.5 + focal_x * point[0] / point[2],
            height * 0.5 + focal_y * point[1] / point[2],
        )
        for point in clipped
    ]
    x0 = max(0.0, min(float(width), min(pixel[0] for pixel in pixels)))
    x1 = max(0.0, min(float(width), max(pixel[0] for pixel in pixels)))
    y0 = max(0.0, min(float(height), min(pixel[1] for pixel in pixels)))
    y1 = max(0.0, min(float(height), max(pixel[1] for pixel in pixels)))
    if x1 - x0 < 8.0 or y1 - y0 < 8.0:
        return None
    center_x = (x0 + x1) / (2.0 * width)
    center_y = (y0 + y1) / (2.0 * height)
    box_width = (x1 - x0) / width
    box_height = (y1 - y0) / height
    return center_x, center_y, box_width, box_height


class DatasetCaptureNode:
    def __init__(self, args) -> None:
        import rclpy
        import rclpy.duration
        import rclpy.time
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener

        class _Node(Node):
            pass

        self.node = _Node("court_scene_dataset_capture")
        self._rclpy = rclpy
        self._duration = rclpy.duration.Duration
        self._time = rclpy.time.Time
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self.node)
        self._args = args
        self._last_capture_s = -math.inf
        self._last_capture_pose: tuple[float, float, float, float] | None = None
        self._count = 0
        self._negative_seen = 0
        self._true_robot_pose: tuple[float, float, float, float] | None = None
        self._surfaces = [
            (label, [np.asarray(point, dtype=np.float64) for point in points])
            for label, points in _court_surfaces()
        ]
        self.node.create_subscription(Image, "/camera/image_raw", self._on_image, 1)
        self.node.create_subscription(
            String, "/sim/robot_true_pose", self._on_true_robot_pose, 1
        )
        self.node.get_logger().info(
            f"capturing court-scene dataset -> {args.output} "
            f"(max={args.max_images}, interval={args.interval_s}s)"
        )

    def _on_true_robot_pose(self, msg) -> None:
        import json

        try:
            value = json.loads(msg.data)
            self._true_robot_pose = (
                float(value["x"]),
                float(value["y"]),
                float(value["z"]),
                float(value["yaw"]),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._true_robot_pose = None

    def _on_image(self, msg) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_s - self._last_capture_s < self._args.interval_s:
            return
        if self._true_robot_pose is None:
            return
        if self._last_capture_pose is not None:
            x, y, _z, yaw = self._true_robot_pose
            last_x, last_y, _last_z, last_yaw = self._last_capture_pose
            translation_delta = math.hypot(x - last_x, y - last_y)
            yaw_delta = abs(
                (yaw - last_yaw + math.pi) % (2.0 * math.pi) - math.pi
            )
            if (
                translation_delta < self._args.min_translation_m
                and yaw_delta < self._args.min_yaw_rad
            ):
                return
        try:
            transform = self._buffer.lookup_transform(
                "camera_link_optical_frame",
                "base_link",
                self._time(),
                self._duration(seconds=0.05),
            )
        except Exception:
            return
        frame = self._decode(msg)
        if frame is None:
            return
        rotation_camera_from_base = _quaternion_matrix(
            transform.transform.rotation
        )
        translation_camera_from_base = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        robot_x, robot_y, robot_z, robot_yaw = self._true_robot_pose
        cosine, sine = math.cos(robot_yaw), math.sin(robot_yaw)
        rotation_world_from_base = np.asarray(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        rotation_camera_from_world = (
            rotation_camera_from_base @ rotation_world_from_base.T
        )
        robot_world = np.asarray(
            [robot_x, robot_y, robot_z], dtype=np.float64
        )
        translation_camera_from_world = (
            translation_camera_from_base
            - rotation_camera_from_world @ robot_world
        )
        labels: list[str] = []
        for label, points in self._surfaces:
            box = project_surface_box(
                points,
                rotation_camera_from_world,
                translation_camera_from_world,
                msg.width,
                msg.height,
                self._args.horizontal_fov,
            )
            if box is None:
                continue
            labels.append(
                f"{CLASS_IDS[label]} " + " ".join(f"{value:.6f}" for value in box)
            )
        # Negative/background frames are useful too, but cap them to avoid a
        # dataset dominated by views where the robot faces the court floor.
        if not labels:
            self._negative_seen += 1
            if self._negative_seen % 5 != 0:
                return
        split = "val" if self._count % 5 == 0 else "train"
        stem = f"gazebo_{self._count:06d}_{int(stamp_s * 1000):012d}"
        image_dir = self._args.output / "images" / split
        label_dir = self._args.output / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(image_dir / f"{stem}.jpg"), frame)
        (label_dir / f"{stem}.txt").write_text(
            "\n".join(labels) + ("\n" if labels else ""), encoding="utf-8"
        )
        self._last_capture_s = stamp_s
        self._last_capture_pose = self._true_robot_pose
        self._count += 1
        if self._count % 25 == 0:
            self.node.get_logger().info(f"captured {self._count} labelled frames")
        if self._count >= self._args.max_images:
            self._rclpy.shutdown()

    @staticmethod
    def _decode(msg) -> np.ndarray | None:
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(
                data.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR
            )
        if msg.encoding == "bgr8":
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == "bgra8":
            return cv2.cvtColor(
                data.reshape((msg.height, msg.width, 4)), cv2.COLOR_BGRA2BGR
            )
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("datasets/court_scene"))
    parser.add_argument("--max-images", type=int, default=1200)
    parser.add_argument("--interval-s", type=float, default=0.25)
    parser.add_argument("--min-translation-m", type=float, default=0.08)
    parser.add_argument("--min-yaw-rad", type=float, default=math.radians(2.0))
    parser.add_argument("--horizontal-fov", type=float, default=1.204)
    args = parser.parse_args()
    if (
        args.max_images < 1
        or args.interval_s <= 0.0
        or args.min_translation_m < 0.0
        or args.min_yaw_rad < 0.0
    ):
        parser.error("capture limits must be positive/non-negative")

    try:
        import rclpy
    except ImportError:
        print("ROS 2 rclpy is required; source the Jazzy workspace", file=sys.stderr)
        return 2
    rclpy.init()
    capture = DatasetCaptureNode(args)
    try:
        rclpy.spin(capture.node)
    except KeyboardInterrupt:
        pass
    finally:
        capture.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
