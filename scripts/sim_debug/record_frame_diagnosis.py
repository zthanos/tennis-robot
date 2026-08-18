#!/usr/bin/env python3
"""Record, per tracking update, the frames the collection controller compared.

Phase 11 diagnostic.  It subscribes to the controller plugin's existing state
topic -- which now publishes the exact pose and path point each reported
`lateral_error_m` was computed from, each with its frame -- and samples the
transforms that could reconcile them at the same instant:

    map -> base_footprint     SLAM/localization pose
    odom -> base_footprint    odometry pose
    map -> odom               the SLAM correction between the two
    /sim/robot_true_pose      Gazebo ground truth (simulation only)

Nothing is published and no production node is touched: the recorder is a
separate process that only listens.  Comparisons are never made between raw
coordinates from different frames -- every row carries its frame names and the
timestamps the values were taken at, and the reconciliation is done offline.

    ros2 run --prefix "python3" ...   # not installed; run it directly:
    python3 scripts/sim_debug/record_frame_diagnosis.py --out runtime/frame_diag.jsonl
"""

from __future__ import annotations

import argparse
import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (documentation of the type)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tennis_robot_msgs.msg import CollectionControllerState
from tf2_ros import Buffer, TransformListener

# Resolved from the plugin name, exactly as the production executor does
# (collection_executor_node_factory: base = f"/{controller_id}").
STATE_TOPIC = "/CollectionFollowPath/state"


def yaw_of(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FrameDiagnosisRecorder(Node):
    def __init__(self, path: str, topic: str) -> None:
        super().__init__("frame_diagnosis_recorder")
        self._handle = open(path, "a", encoding="utf-8")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._truth: tuple[float, float, float] | None = None
        self._truth_at: float | None = None
        self._rows = 0
        self.create_subscription(String, "/sim/robot_true_pose", self._on_truth, 1)
        self.create_subscription(
            CollectionControllerState, topic, self._on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.get_logger().info(f"frame diagnosis recording {topic} -> {path}")

    def _on_truth(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            self._truth = (float(data["x"]), float(data["y"]), float(data["yaw"]))
            self._truth_at = self.get_clock().now().nanoseconds * 1e-9
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._truth = None

    def _lookup(self, target: str, source: str) -> dict | None:
        """Latest available transform, with the time it was actually stamped at.

        Deliberately `Time()` (latest) rather than a synchronised lookup: the
        question under investigation is what the running system does, and the
        age is recorded so a stale transform can be rejected offline instead of
        silently interpolated here.
        """
        try:
            transform = self._buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - a missing transform is data
            return {"error": type(exc).__name__, "detail": str(exc)[:120]}
        translation = transform.transform.translation
        return {
            "target_frame": target,
            "source_frame": source,
            "x_m": translation.x,
            "y_m": translation.y,
            "yaw_rad": yaw_of(transform.transform.rotation),
            "stamp_s": stamp_s(transform.header.stamp),
        }

    def _on_state(self, state: CollectionControllerState) -> None:
        # Failures are recorded too -- the update that trips a gate is the one
        # this instrumentation exists for (Phase 15).
        if not state.tracker_has_reference and not state.failure_reason:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        row = {
            "received_at_s": now_s,
            "plan_id": state.plan_id,
            "active_segment_id": state.active_segment_id,
            "progress_s": state.progress_s,
            "has_active_crossing": bool(state.has_active_crossing),
            "active_ball_id": state.active_ball_id,
            "active_crossing_progress_s": state.active_crossing_progress_s,
            "has_geometry": bool(state.tracker_has_geometry),
            "reanchoring": {
                "pose_step_m": state.tracker_pose_step_m,
                "pose_step_yaw_rad": state.tracker_pose_step_yaw_rad,
                "elapsed_s": state.tracker_elapsed_s,
                "plausible_step_bound_m": state.tracker_plausible_step_bound_m,
                "detected": bool(state.tracker_reanchoring_detected),
                "tube_verdict_deferred": bool(state.tracker_tube_verdict_deferred),
            },
            "previous_progress_s": state.tracker_previous_progress_s,
            "raw_projection_progress_s": state.tracker_raw_projection_progress_s,
            "has_raw_projection": bool(state.tracker_has_raw_projection),
            "projection_reach_m": state.tracker_projection_reach_m,
            "raw_projection_constrained": bool(state.tracker_raw_projection_constrained),
            "reported_lateral_error_m": state.lateral_error_m,
            "reported_heading_error_rad": state.heading_error_rad,
            "measured_speed_mps": state.measured_speed_mps,
            "failure_reason": int(state.failure_reason),
            # what the tracker actually compared, verbatim from the plugin
            "tracker": {
                "pose_frame_id": state.tracker_pose_frame_id,
                "pose_stamp_s": state.tracker_pose_stamp_s,
                "x_m": state.tracker_pose_x_m,
                "y_m": state.tracker_pose_y_m,
                "yaw_rad": state.tracker_pose_yaw_rad,
                "update_stamp_s": state.tracker_update_stamp_s,
                "transform_stamp_s": state.tracker_transform_stamp_s,
                "transform_age_s": state.tracker_transform_age_s,
                "transform_was_latest": bool(state.tracker_transform_was_latest),
            },
            "commanded": {
                "linear_mps": state.commanded_linear_mps,
                "angular_rad_s": state.commanded_angular_rad_s,
                "curvature_per_m": state.tracker_commanded_curvature_per_m,
            },
            "pursuit": {
                "lookahead_x_m": state.tracker_lookahead_x_m,
                "lookahead_y_m": state.tracker_lookahead_y_m,
                "lookahead_distance_m": state.tracker_lookahead_distance_m,
            },
            "reference": {
                "path_frame_id": state.tracker_path_frame_id,
                "path_stamp_s": state.tracker_path_stamp_s,
                "x_m": state.tracker_reference_x_m,
                "y_m": state.tracker_reference_y_m,
                "yaw_rad": state.tracker_reference_yaw_rad,
            },
            "map_base": self._lookup("map", "base_footprint"),
            "odom_base": self._lookup("odom", "base_footprint"),
            "map_odom": self._lookup("map", "odom"),
            "map_base_link": self._lookup("map", "base_link"),
            "odom_base_link": self._lookup("odom", "base_link"),
            "truth": None if self._truth is None else {
                "frame": "gazebo_world",
                "x_m": self._truth[0], "y_m": self._truth[1], "yaw_rad": self._truth[2],
                "stamp_s": self._truth_at,
            },
        }
        self._handle.write(json.dumps(row) + "\n")
        self._rows += 1
        if self._rows % 200 == 0:
            self._handle.flush()
            self.get_logger().info(f"{self._rows} tracking updates recorded")

    def destroy_node(self) -> bool:
        self._handle.flush()
        self._handle.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runtime/frame_diagnosis.jsonl")
    parser.add_argument("--topic", default=STATE_TOPIC)
    arguments = parser.parse_args()
    rclpy.init()
    node = FrameDiagnosisRecorder(arguments.out, arguments.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
