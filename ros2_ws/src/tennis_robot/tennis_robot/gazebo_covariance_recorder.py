"""C2-only raw measurement recorder; it never publishes perception targets."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from tennis_robot.ball_detector import load_ball_detector
from tennis_robot.perception import (
    camera_frame_position,
    depth_roi_quality,
    estimate_depth_ball_observation,
    pixel_elevation_rad,
)
from tennis_robot.gazebo_covariance_trials import apply_calibration_depth_mask
from tennis_robot.gazebo_covariance_association import associate_target_candidate, measured_range_in_bin


class GazeboCovarianceRecorder(Node):
    """Record matched neural/depth XYZ residuals against `/sim/balls` GT."""

    def __init__(self) -> None:
        super().__init__("gazebo_covariance_recorder")
        output = os.environ.get("GAZEBO_COVARIANCE_EVIDENCE_PATH")
        if not output:
            raise RuntimeError("GAZEBO_COVARIANCE_EVIDENCE_PATH is required")
        self._output = Path(output)
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self._output.open("w", encoding="utf-8")
        self._rejection_fp = self._output.with_suffix(self._output.suffix + ".rejections.jsonl").open("w", encoding="utf-8")
        self._non_target_fp = self._output.with_suffix(self._output.suffix + ".non_target.jsonl").open("w", encoding="utf-8")
        self._balls: list[dict] = []
        requested = os.environ.get("GAZEBO_COVARIANCE_REQUESTED_POSE_JSON")
        self._requested_pose = json.loads(requested) if requested else None
        self._nominal_range_bin = os.environ.get("GAZEBO_COVARIANCE_NOMINAL_RANGE_BIN", "")
        self._association_gate_m = float(os.environ.get("GAZEBO_COVARIANCE_ASSOCIATION_GATE_M", "0.20"))
        self._association_ambiguity_margin_m = float(os.environ.get("GAZEBO_COVARIANCE_ASSOCIATION_AMBIGUITY_MARGIN_M", "0.01"))
        self._target_residual_outlier_threshold_m = float(os.environ.get("GAZEBO_COVARIANCE_TARGET_RESIDUAL_OUTLIER_THRESHOLD_M", "0.10"))
        if self._requested_pose is not None and set(self._requested_pose) != {"x_m", "y_m", "yaw_rad"}:
            raise RuntimeError("invalid requested Gazebo pose")
        self._pose_tolerance_m = 0.05
        self._yaw_tolerance_rad = 0.05
        self._required_stable_frames = 3
        self._ready_frames = 0
        self._ready = self._requested_pose is None
        self._settle_stamp_ns: int | None = None
        self._true_robot_pose: dict | None = None
        self._true_robot_pose_stamp_ns: int | None = None
        self._last_velocity = {"linear_m_s": float("inf"), "angular_rad_s": float("inf")}
        self._latest_simulation_time_ns: int | None = None
        self._target_ball_id = os.environ.get("GAZEBO_COVARIANCE_TARGET_BALL_ID")
        self._trial_id = os.environ.get("GAZEBO_COVARIANCE_TRIAL_ID", "unlabelled")
        self._mask_ratio = float(os.environ.get("GAZEBO_COVARIANCE_MASK_MISSING_RATIO", "0"))
        self._mask_seed = int(os.environ.get("GAZEBO_COVARIANCE_MASK_SEED", "0"))
        self._max_accepted = int(os.environ.get("GAZEBO_COVARIANCE_MAX_ACCEPTED", "0"))
        if not self._target_ball_id or self._max_accepted <= 0:
            raise RuntimeError("target ball ID and positive max accepted sample count are required")
        self._stationary = False
        self._accepted = 0
        self._target_samples = 0
        self._target_outliers = 0
        self._non_target_detections = 0
        self._association_rejections = {"ambiguous": 0, "unmatched": 0}
        self._range_readiness: dict | None = None
        self._complete = False
        self._rejected: dict[str, int] = {}
        self._detector = load_ball_detector(logger=self.get_logger())
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(String, "/sim/balls", self._on_balls, 10)
        self.create_subscription(String, "/sim/robot_true_pose", self._on_true_robot_pose, 10)
        # The sim launch remaps controller-facing odometry to the EKF output;
        # use the live stream that is actually available to the trial process.
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odometry, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        rgb = message_filters.Subscriber(self, Image, "/camera/image_raw", qos_profile=1)
        depth = message_filters.Subscriber(self, Image, "/camera/depth", qos_profile=1)
        sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 10, 0.05)
        sync.registerCallback(self._on_pair)
        self._sync = sync

    def _reject(self, reason: str) -> None:
        self._rejected[reason] = self._rejected.get(reason, 0) + 1

    def _on_balls(self, msg: String) -> None:
        try:
            decoded = json.loads(msg.data)
            self._balls = [ball for ball in decoded if {"def", "x", "y", "z"} <= set(ball)]
        except (json.JSONDecodeError, TypeError):
            self._balls = []

    def _on_odometry(self, msg: Odometry) -> None:
        velocity = msg.twist.twist
        linear = math.sqrt(
            velocity.linear.x ** 2 + velocity.linear.y ** 2 + velocity.linear.z ** 2
        )
        angular = math.sqrt(
            velocity.angular.x ** 2 + velocity.angular.y ** 2 + velocity.angular.z ** 2
        )
        self._last_velocity = {"linear_m_s": linear, "angular_rad_s": angular}
        self._stationary = linear <= 0.01 and angular <= 0.01
        if self._requested_pose is None or self._true_robot_pose is None:
            return
        pose = self._true_robot_pose
        requested = self._requested_pose
        yaw_error = math.atan2(
            math.sin(float(pose["yaw"]) - float(requested["yaw_rad"])),
            math.cos(float(pose["yaw"]) - float(requested["yaw_rad"])),
        )
        within_pose = math.hypot(
            float(pose["x"]) - float(requested["x_m"]),
            float(pose["y"]) - float(requested["y_m"]),
        ) <= self._pose_tolerance_m and abs(yaw_error) <= self._yaw_tolerance_rad
        if within_pose and self._stationary:
            self._ready_frames += 1
            if self._ready_frames >= self._required_stable_frames and not self._ready:
                self._ready = True
                self._settle_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        else:
            self._ready_frames = 0

    def _on_true_robot_pose(self, msg: String) -> None:
        try:
            pose = json.loads(msg.data)
            if not {"x", "y", "z", "yaw"} <= set(pose):
                return
            self._true_robot_pose = {key: float(pose[key]) for key in ("x", "y", "z", "yaw")}
            self._true_robot_pose_stamp_ns = self.get_clock().now().nanoseconds
        except (json.JSONDecodeError, TypeError, ValueError):
            return

    def _on_clock(self, msg: Clock) -> None:
        self._latest_simulation_time_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)

    @staticmethod
    def _decode_rgb(msg: Image) -> np.ndarray | None:
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "rgb8":
            return raw.reshape((msg.height, msg.width, 3))[:, :, ::-1]
        if msg.encoding == "bgr8":
            return raw.reshape((msg.height, msg.width, 3))
        return None

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray | None:
        if msg.encoding not in {"32FC1", "32FC"}:
            return None
        raw = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if raw.size != msg.width * msg.height:
            return None
        return raw.reshape((msg.height, msg.width))

    @staticmethod
    def _transform_point(transform, point: tuple[float, float, float]) -> tuple[float, float, float]:
        q = transform.transform.rotation
        t = transform.transform.translation
        x, y, z = point
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        rx = (1 - 2 * (yy * yy + zz * zz)) * x + 2 * (xx * yy - zz * ww) * y + 2 * (xx * zz + yy * ww) * z
        ry = 2 * (xx * yy + zz * ww) * x + (1 - 2 * (xx * xx + zz * zz)) * y + 2 * (yy * zz - xx * ww) * z
        rz = 2 * (xx * zz - yy * ww) * x + 2 * (yy * zz + xx * ww) * y + (1 - 2 * (xx * xx + yy * yy)) * z
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _transform_dict(transform) -> dict:
        t, q = transform.transform.translation, transform.transform.rotation
        return {"translation_m": [t.x, t.y, t.z], "rotation_xyzw": [q.x, q.y, q.z, q.w]}

    def _ground_truth_optical(self, stamp):
        try:
            transform = self._tf.lookup_transform(
                "camera_link_optical_frame", "odom", Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            self._reject("tf_at_rgb_timestamp_unavailable")
            return None
        ball = next((item for item in self._balls if item["def"] == self._target_ball_id), None)
        if ball is None:
            self._reject("required_ground_truth_reference_unavailable")
            return None
        transformed = [
            (item, self._transform_point(transform, (float(item["x"]), float(item["y"]), float(item["z"]))))
            for item in self._balls
        ]
        truth = next(point for item, point in transformed if item["def"] == self._target_ball_id)
        return ball, truth, transform, transformed

    def _on_pair(self, rgb_msg: Image, depth_msg: Image) -> None:
        rgb_stamp_ns = int(rgb_msg.header.stamp.sec) * 1_000_000_000 + int(rgb_msg.header.stamp.nanosec)
        if not self._ready:
            self._reject("readiness_gate_not_passed")
            return
        if self._settle_stamp_ns is not None and rgb_stamp_ns <= self._settle_stamp_ns:
            self._reject("stale_before_settle_frame")
            return
        image, depth = self._decode_rgb(rgb_msg), self._decode_depth(depth_msg)
        if image is None or depth is None:
            self._reject("invalid_camera_pair")
            return
        ground_truth = self._ground_truth_optical(rgb_msg.header.stamp)
        if ground_truth is None:
            self._reject("ground_truth_unavailable")
            return
        ball, truth, camera_tf, all_ground_truth = ground_truth
        measured_target_range = math.sqrt(sum(value * value for value in truth))
        if self._range_readiness is None:
            in_bin = measured_range_in_bin(measured_target_range, self._nominal_range_bin)
            self._range_readiness = {"requested_bin": self._nominal_range_bin, "measured_target_range_m": measured_target_range, "passed": in_bin}
            if not in_bin:
                self._reject("target_range_not_in_bin")
                self._complete = True
                return
        try:
            base_tf = self._tf.lookup_transform("odom", "base_link", Time.from_msg(rgb_msg.header.stamp), timeout=Duration(seconds=0.1))
            camera_from_base_tf = self._tf.lookup_transform("camera_link_optical_frame", "base_link", Time.from_msg(rgb_msg.header.stamp), timeout=Duration(seconds=0.1))
        except Exception:
            self._reject("base_tf_at_rgb_timestamp_unavailable")
            return
        vertical_fov = 1.204 * (rgb_msg.height / max(1, rgb_msg.width))
        ground_truth_by_id = {item["def"]: point for item, point in all_ground_truth}
        for detection in self._detector.detect(image):
            frame_seed = self._mask_seed ^ int(rgb_msg.header.stamp.sec) ^ int(rgb_msg.header.stamp.nanosec)
            calibrated_depth = apply_calibration_depth_mask(
                depth, detection, rgb_msg.width, rgb_msg.height,
                missing_pixel_ratio=self._mask_ratio, seed=frame_seed,
            )
            observation = estimate_depth_ball_observation(detection, calibrated_depth, rgb_msg.width, rgb_msg.height, 1.204)
            if observation is None:
                self._reject("missing_or_invalid_depth")
                continue
            estimate = camera_frame_position(
                observation.bearing_rad,
                observation.distance_m,
                pixel_elevation_rad(detection.center_y, rgb_msg.height, vertical_fov),
            )
            association = associate_target_candidate(
                estimate, ground_truth_by_id, target_ball_id=self._target_ball_id,
                association_gate_m=self._association_gate_m,
                ambiguity_margin_m=self._association_ambiguity_margin_m,
            )
            if association.kind == "non_target":
                self._non_target_detections += 1
                self._non_target_fp.write(json.dumps({
                    "event": "non_target_detection", "trial_id": self._trial_id,
                    "target_ball_id": self._target_ball_id, "associated_ball_id": association.ball_id,
                    "association_distance_m": association.distance_m, "raw_candidate_camera_xyz_m": estimate,
                    "rgb_stamp_ns": rgb_stamp_ns,
                }, sort_keys=True) + "\n")
                self._non_target_fp.flush()
                continue
            if association.kind in {"ambiguous", "unmatched"}:
                self._association_rejections[association.kind] += 1
                self._rejection_fp.write(json.dumps({
                    "event": "association_rejected",
                    "trial_id": self._trial_id,
                    "target_ball_id": self._target_ball_id,
                    "association_reason": association.kind,
                    "nearest_ground_truth_ball_id": association.ball_id,
                    "nearest_distance_m": association.distance_m,
                    "raw_candidate_camera_xyz_m": estimate,
                    "rgb_stamp_ns": rgb_stamp_ns,
                }, sort_keys=True) + "\n")
                self._rejection_fp.flush()
                continue
            error = tuple(estimate[i] - truth[i] for i in range(3))
            residual_m = math.dist(estimate, truth)
            is_target_outlier = residual_m > self._target_residual_outlier_threshold_m
            self._target_samples += 1
            self._target_outliers += int(is_target_outlier)
            row = {
                "ball_id": self._target_ball_id,
                "trial_id": self._trial_id,
                "ground_truth_reference": f"/sim/balls:{self._target_ball_id}",
                "nominal_range_bin": self._nominal_range_bin,
                "gazebo_simulation_time_ns": rgb_stamp_ns,
                "rgb_stamp_ns": rgb_stamp_ns,
                "depth_stamp_ns": int(depth_msg.header.stamp.sec) * 1_000_000_000 + int(depth_msg.header.stamp.nanosec),
                "range_m": math.sqrt(sum(value * value for value in truth)),
                "actual_camera_to_ball_ground_truth_range_m": math.sqrt(sum(value * value for value in truth)),
                "ball_ground_truth_world_pose_m": [ball.get("world_x"), ball.get("world_y"), ball.get("world_z")],
                "robot_base_world_pose": self._true_robot_pose,
                "robot_base_world_pose_observed_at_ns": self._true_robot_pose_stamp_ns,
                "camera_optical_tf_at_rgb_stamp": self._transform_dict(camera_tf),
                "base_tf_at_rgb_stamp": self._transform_dict(base_tf),
                "camera_optical_from_base_tf_at_rgb_stamp": self._transform_dict(camera_from_base_tf),
                "velocity_at_readiness_check": self._last_velocity,
                "depth_quality": depth_roi_quality(detection, calibrated_depth, rgb_msg.width, rgb_msg.height),
                "sample_age_s": None if self._latest_simulation_time_ns is None else (self._latest_simulation_time_ns - rgb_stamp_ns) / 1e9,
                "rgb_depth_delta_s": abs(rgb_stamp_ns - (int(depth_msg.header.stamp.sec) * 1_000_000_000 + int(depth_msg.header.stamp.nanosec))) / 1e9,
                "injected_missing_pixel_ratio": self._mask_ratio,
                "mask_seed": self._mask_seed,
                "estimate_xyz_m": estimate,
                "raw_candidate_camera_xyz_m": estimate,
                "ground_truth_xyz_m": truth,
                "error_xyz_m": error,
                "target_residual_m": residual_m,
                "target_residual_outlier": is_target_outlier,
            }
            self._fp.write(json.dumps(row, sort_keys=True) + "\n")
            self._fp.flush()
            self._accepted += 1
            if self._target_samples >= self._max_accepted:
                self._complete = True
                return

    def destroy_node(self) -> bool:
        association_rejections = sum(self._association_rejections.values())
        target_association_denominator = self._target_samples + association_rejections
        summary = {"trial_id": self._trial_id, "target_ball_id": self._target_ball_id,
                   "injected_missing_pixel_ratio": self._mask_ratio, "mask_seed": self._mask_seed,
                   "requested_pose": self._requested_pose, "readiness": {"passed": self._ready, "stable_frames": self._ready_frames, "settle_stamp_ns": self._settle_stamp_ns, "buffer_reset_per_trial": True, "target_range": self._range_readiness},
                   "metrics": {"target_samples": self._target_samples, "target_outliers": self._target_outliers, "target_outlier_rate": None if not self._target_samples else self._target_outliers / self._target_samples, "non_target_detections": self._non_target_detections, "ambiguous_detections": self._association_rejections["ambiguous"], "unmatched_detections": self._association_rejections["unmatched"], "target_association_rejection_rate": None if not target_association_denominator else association_rejections / target_association_denominator},
                   "accepted": self._accepted, "rejected": self._rejected}
        self.get_logger().info(f"C2 recorder summary: {json.dumps(summary, sort_keys=True)}")
        self._fp.close()
        self._rejection_fp.close()
        self._non_target_fp.close()
        self._output.with_suffix(self._output.suffix + ".summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return super().destroy_node()

    @property
    def complete(self) -> bool:
        return self._complete


def main() -> None:
    rclpy.init()
    node = GazeboCovarianceRecorder()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
