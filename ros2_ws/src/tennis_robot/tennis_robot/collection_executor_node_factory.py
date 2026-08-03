"""ROS-node construction of the dormant collection-route executor (Phase 6D.3).

This is the only ROS-aware translation layer for the Phase 6B execution
context. Runtime executor thresholds come from required ROS parameters; the
immutable planner configuration comes from explicit YAML and calibration paths.
The factory does not tick the executor or alter controller-node mode dispatch.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

from tennis_robot.collection_court_model_builder import build_court_model
from tennis_robot.collection_execution_context_builder import CollectionExecutionContextValues
from tennis_robot.collection_execution_frame import (
    RigidTransform2D,
    transform_collection_plan,
)
from tennis_robot.collection_executor_assembly import (
    CollectionExecutorConfig, CollectionExecutorHandles,
    build_collection_route_executor, read_controller_tuning,
)
from tennis_robot.collection_executor_ports import RosMonotonicClock
from tennis_robot.collection_drive_observation import (
    DriveObservationBuffer,
    DriveObservationError,
    DriveViewpointStepper,
    build_drive_snapshot,
)
from tennis_robot.collection_route_config_builder import build_collection_route_configuration
from tennis_robot.collection_route_types import Point2D, Pose2D
from tennis_robot.collection_scan_snapshot import CourtHalfBoundary
from tennis_robot.collection_snapshot_runtime_adapter import (
    CollectionSnapshotRuntimeAdapter,
    CollectionSnapshotRuntimeSession,
)
from tennis_robot.perception_spatial_observation_adapter import TimestampedCameraToMapTransform


class CollectionExecutorNodeFactoryError(ValueError):
    """A required node-side dependency or configuration value is invalid."""


def _planner_audit_sink_from_env(node):
    """Return an opt-in pre-execution snapshot/plan capture callback."""
    audit_dir_value = os.getenv("COLLECTION_ROUTE_AUDIT_DIR", "").strip()
    if not audit_dir_value:
        return None
    directory = Path(audit_dir_value)

    def save(snapshot, plan) -> None:
        safe_scan_id = "".join(
            char if char.isalnum() or char in "-_." else "_"
            for char in str(snapshot.scan_id)
        )
        target = directory / f"{safe_scan_id}.json"
        temporary = directory / f".{safe_scan_id}.json.tmp"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            temporary.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "route_outcome": None,
                        "snapshot": snapshot.to_dict(),
                        "plan": plan.to_dict(),
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, target)
            node.get_logger().info(
                f"collection route pre-execution audit saved: {target}"
            )
        except (OSError, TypeError, ValueError) as exc:
            node.get_logger().error(
                f"collection route pre-execution audit write failed: {exc}"
            )

    return save


_RUNTIME_PARAM_FIELDS = {
    "context_schema_version": "collection_route.context_schema_version",
    "context_activation_timeout_s": "collection_route.context_activation_timeout_s",
    "scan_step_count": "collection_route.scan_step_count",
    "scan_yaw_tolerance_rad": "collection_route.scan_yaw_tolerance_rad",
    "scan_start_yaw_rad": "collection_route.scan_start_yaw_rad",
    "scan_angular_speed_rad_s": "collection_route.scan_angular_speed_rad_s",
    "scan_timeout_s": "collection_route.scan_timeout_s",
    "safety_forward_half_angle_rad": "collection_route.safety_forward_half_angle_rad",
    "safety_stop_distance_m": "collection_route.safety_stop_distance_m",
    "safety_pause_timeout_s": "collection_route.safety_pause_timeout_s",
    "safety_max_scan_age_s": "collection_route.safety_max_scan_age_s",
    "controller_id": "collection_route.controller_id",
    "goal_checker_id": "collection_route.goal_checker_id",
    "drive_viewpoint_spacing_m": "collection_route.drive_viewpoint_spacing_m",
    "drive_known_merge_radius_m": "collection_route.drive_known_merge_radius_m",
}

_PROFILE_FIELDS = (
    "nominal_speed_mps", "min_speed_mps", "max_speed_mps",
    "nominal_speed_warning_tolerance_mps", "max_acceleration_mps2",
    "max_deceleration_mps2", "required_entry_m", "required_run_in_m",
    "required_run_out_m", "max_curvature_per_m", "max_lateral_error_m",
    "max_heading_error_rad", "allow_reversing", "allow_standalone_rotate",
)


@dataclass
class CollectionExecutorNodeCache:
    """Latest values owned by the node's existing topic callbacks."""

    latest_scan: object | None = None
    latest_ball_detections: object | None = None
    robot_x_m: float | None = None
    robot_y_m: float | None = None
    robot_yaw_rad: float | None = None


@dataclass(frozen=True)
class CollectionExecutorRosTypes:
    """ROS types/functions, injectable so construction is unit-testable."""

    Twist: type
    Pose: type
    PoseStamped: type
    Path: type
    FollowPath: type
    ActionClient: type
    GoalStatus: type
    CollectionExecutionContext: type
    CollectionExecutionSegment: type
    CollectionExecutionProfile: type
    CollectionPlannedCrossing: type
    CollectionControllerState: type
    LoadService: type
    ResetService: type
    HoldService: type
    FinalizeService: type
    time_from_seconds: object
    spin_until_future_complete: object


def load_ros_types() -> CollectionExecutorRosTypes:
    """Import the real ROS bindings only when node construction is requested."""
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Pose, PoseStamped, Twist
    from nav_msgs.msg import Path as NavPath
    from nav2_msgs.action import FollowPath
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from tennis_robot_msgs.msg import (
        CollectionControllerState, CollectionExecutionContext,
        CollectionExecutionProfile, CollectionExecutionSegment,
        CollectionPlannedCrossing,
    )
    from tennis_robot_msgs.srv import (
        FinalizeCollectionExecutionContext, LoadCollectionExecutionContext,
        ResetCollectionExecutionContext, SetCollectionSafetyHold,
    )
    return CollectionExecutorRosTypes(
        Twist=Twist, Pose=Pose, PoseStamped=PoseStamped, Path=NavPath,
        FollowPath=FollowPath, ActionClient=ActionClient, GoalStatus=GoalStatus,
        CollectionExecutionContext=CollectionExecutionContext,
        CollectionExecutionSegment=CollectionExecutionSegment,
        CollectionExecutionProfile=CollectionExecutionProfile,
        CollectionPlannedCrossing=CollectionPlannedCrossing,
        CollectionControllerState=CollectionControllerState,
        LoadService=LoadCollectionExecutionContext,
        ResetService=ResetCollectionExecutionContext,
        HoldService=SetCollectionSafetyHold,
        FinalizeService=FinalizeCollectionExecutionContext,
        time_from_seconds=lambda seconds: Time(nanoseconds=round(seconds * 1e9)),
        spin_until_future_complete=rclpy.spin_until_future_complete,
    )


def load_court_boundary(path: str | Path) -> dict:
    """Read and validate one explicit court artifact, with no path fallback."""
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        build_court_model(value)
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise CollectionExecutorNodeFactoryError(
            f"invalid court_boundary at {str(path)!r}: {exc}"
        ) from exc
    return value


def load_collection_route_source(path: str | Path) -> Mapping[str, Any]:
    """Load the explicit YAML source; PyYAML is a declared dependency."""
    try:
        import yaml
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError) as exc:
        raise CollectionExecutorNodeFactoryError(
            f"invalid collection route config at {str(path)!r}: {exc}"
        ) from exc
    if not isinstance(value, Mapping):
        raise CollectionExecutorNodeFactoryError("collection route config must be a mapping")
    return value


def scan_pose_from_court_model(
    court_boundary: Mapping[str, Any], *, robot_pose: Pose2D
) -> tuple[float, float, float]:
    """Return the service-line centre, facing into the robot-side court half.

    Collection targets on the selected half lie away from the net.  Facing the
    net after the scan puts every target behind the non-holonomic robot and can
    leave the connector graph with no valid start edge even though the target
    passes themselves are feasible.
    """
    build_court_model(court_boundary)  # reuse the Phase 6A schema/geometry gate
    if not isinstance(robot_pose, Pose2D):
        raise CollectionExecutorNodeFactoryError("robot_pose must be a Pose2D")
    try:
        net = court_boundary["net"]
        center, axis_length, axis_width = net["center"], net["axis_length"], net["axis_width"]
        lines = court_boundary["court"]["lines_court_frame"]
        cx, cy = float(center["x_m"]), float(center["y_m"])
        lx, ly = float(axis_length["x_m"]), float(axis_length["y_m"])
        wx, wy = float(axis_width["x_m"]), float(axis_width["y_m"])
        center_line_y = float(lines["center_line_y"])
        service_xs = tuple(float(value) for value in lines["service_x"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CollectionExecutorNodeFactoryError(f"court model lacks service-line geometry: {exc}") from exc
    values = (cx, cy, lx, ly, wx, wy, center_line_y, *service_xs)
    if (len(service_xs) != 2 or not all(math.isfinite(value) for value in values)
            or math.hypot(lx, ly) <= 1e-9 or math.hypot(wx, wy) <= 1e-9
            or service_xs[0] * service_xs[1] >= 0.0):
        raise CollectionExecutorNodeFactoryError("invalid court service-line geometry")
    length_norm, width_norm = math.hypot(lx, ly), math.hypot(wx, wy)
    lx, ly, wx, wy = lx / length_norm, ly / length_norm, wx / width_norm, wy / width_norm
    projection = (robot_pose.x_m - cx) * lx + (robot_pose.y_m - cy) * ly
    if abs(projection) <= 1e-9:
        raise CollectionExecutorNodeFactoryError("robot is on the net plane; court side is ambiguous")
    service_x = max(service_xs) if projection > 0.0 else min(service_xs)
    return (
        cx + service_x * lx + center_line_y * wx,
        cy + service_x * ly + center_line_y * wy,
        math.atan2(service_x * ly, service_x * lx),
    )


def court_half_from_court_model(
    court_boundary: Mapping[str, Any], *, robot_pose: Pose2D
) -> CourtHalfBoundary:
    """Build the snapshot filter's robot-side half plane from surveyed posts."""
    build_court_model(court_boundary)
    try:
        first, second = court_boundary["net"]["posts"]
        a = Point2D(float(first["x_m"]), float(first["y_m"]))
        b = Point2D(float(second["x_m"]), float(second["y_m"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise CollectionExecutorNodeFactoryError(f"invalid net posts: {exc}") from exc
    cross = ((b.x_m - a.x_m) * (robot_pose.y_m - a.y_m)
             - (b.y_m - a.y_m) * (robot_pose.x_m - a.x_m))
    if abs(cross) <= 1e-9:
        raise CollectionExecutorNodeFactoryError("robot is on the net plane; court half is ambiguous")
    return CourtHalfBoundary(a, b, 1 if cross > 0.0 else -1)


def execution_context_values_to_msg(values, ros: CollectionExecutorRosTypes):
    """The single 1:1 translation from the Phase 6B value object to ROS."""
    if not isinstance(values, CollectionExecutionContextValues):
        raise TypeError("values must be CollectionExecutionContextValues")
    context = ros.CollectionExecutionContext()
    for name in ("context_schema_version", "plan_id", "path_sha256",
                 "context_activation_timeout_s", "terminal_progress_s",
                 "configuration_snapshot_json"):
        setattr(context, name, getattr(values, name))
    context.terminal_pose = _canonical_pose_to_msg(values.terminal_pose, ros)
    for field in fields(values.controller_tuning):
        setattr(context.controller_tuning, field.name, getattr(values.controller_tuning, field.name))
    for source_segment in values.segments:
        segment = ros.CollectionExecutionSegment()
        segment.segment_id = source_segment.segment_id
        if source_segment.segment_type not in (0, 1, 2):
            raise CollectionExecutorNodeFactoryError(f"unsupported segment_type {source_segment.segment_type!r}")
        segment.segment_type = source_segment.segment_type
        segment.progress_start_s = source_segment.progress_start_s
        segment.progress_end_s = source_segment.progress_end_s
        profile = ros.CollectionExecutionProfile()
        for name in _PROFILE_FIELDS:
            setattr(profile, name, getattr(source_segment.execution_profile, name))
        segment.execution_profile = profile
        for source_crossing in source_segment.planned_crossings:
            crossing = ros.CollectionPlannedCrossing()
            for field in fields(source_crossing):
                setattr(crossing, field.name, getattr(source_crossing, field.name))
            segment.planned_crossings.append(crossing)
        context.segments.append(segment)
    return context


def _canonical_pose_to_msg(source, ros):
    pose = ros.Pose()
    pose.position.x, pose.position.y, pose.position.z = source.x, source.y, source.z
    pose.orientation.x, pose.orientation.y = source.qx, source.qy
    pose.orientation.z, pose.orientation.w = source.qz, source.qw
    return pose


class _TfProvider:
    def __init__(self, tf_buffer, ros):
        self._buffer, self._ros = tf_buffer, ros

    def at(self, timestamp_s: float, camera_frame: str):
        transform = self._buffer.lookup_transform("map", camera_frame, self._ros.time_from_seconds(timestamp_s))
        translation, rotation = transform.transform.translation, transform.transform.rotation
        return TimestampedCameraToMapTransform(
            timestamp_s, "map", camera_frame,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )


class _ExecutionPlanTransformer:
    """Freeze map→odom once, immediately before FollowPath context creation."""

    def __init__(self, tf_buffer, ros):
        self._buffer, self._ros = tf_buffer, ros
        self.last_diagnostics: dict = {}

    def __call__(self, plan):
        if plan.map_frame == "odom":
            self.last_diagnostics = {
                "schema_version": 1,
                "plan_id": plan.plan_id,
                "source_frame": "odom",
                "target_frame": "odom",
                "identity": True,
                "transform": {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "source_crossings": _execution_crossings(plan),
                "execution_crossings": _execution_crossings(plan),
            }
            return plan
        transform = self._buffer.lookup_transform(
            "odom", plan.map_frame, self._ros.time_from_seconds(0.0)
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        rigid = RigidTransform2D(
            target_frame="odom",
            source_frame=plan.map_frame,
            x_m=float(translation.x),
            y_m=float(translation.y),
            yaw_rad=yaw,
        )
        transformed = transform_collection_plan(
            plan,
            rigid,
        )
        stamp = getattr(getattr(transform, "header", None), "stamp", None)
        stamp_s = None
        if stamp is not None:
            stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.last_diagnostics = {
            "schema_version": 1,
            "plan_id": plan.plan_id,
            "source_frame": plan.map_frame,
            "target_frame": transformed.map_frame,
            "identity": False,
            "transform_timestamp_s": stamp_s,
            "transform": {
                "x_m": rigid.x_m,
                "y_m": rigid.y_m,
                "yaw_rad": rigid.yaw_rad,
            },
            "source_crossings": _execution_crossings(plan),
            "execution_crossings": _execution_crossings(transformed),
        }
        return transformed


def _execution_crossings(plan) -> list[dict]:
    """Return the immutable crossing geometry needed for frame auditing."""
    return [
        {
            "ball_id": crossing.ball_id,
            "segment_id": segment.id,
            "x_m": crossing.position_xy.x_m,
            "y_m": crossing.position_xy.y_m,
            "heading_rad": crossing.heading_rad,
            "progress_s": crossing.progress_s,
        }
        for segment in plan.segments
        for crossing in segment.planned_crossings
    ]


class _LiveDriveObserver:
    """Collect off-route ball sightings while a route is being executed.

    Runs alongside the frozen plan and never feeds into it: the result is only
    consulted once the route has finished, to plan the follow-up pass from balls
    the 360 never confirmed instead of repeating that 360 from the same pose.

    Validation is the adapter's and the snapshot builder's, unchanged — see
    :mod:`tennis_robot.collection_drive_observation`.
    """

    def __init__(
        self, *, node, adapter, configuration_snapshot, court_half_boundary,
        frame_provider, robot_pose_provider, clock, scan_id_prefix: str,
        viewpoint_spacing_m: float, merge_radius_m: float, map_frame: str = "map",
    ) -> None:
        self._node = node
        self._adapter = adapter
        self._configuration_snapshot = configuration_snapshot
        self._court_half_boundary = court_half_boundary
        self._frame_provider = frame_provider
        self._robot_pose_provider = robot_pose_provider
        self._clock = clock
        self._scan_id_prefix = scan_id_prefix
        self._viewpoint_spacing_m = viewpoint_spacing_m
        self._merge_radius_m = merge_radius_m
        self._map_frame = map_frame
        self._run = 0
        self._buffer = None
        self._stepper = None
        self._last_frame = None

    def start(self) -> None:
        self._run += 1
        self._buffer = DriveObservationBuffer(
            scan_id=f"{self._scan_id_prefix}/drive-{self._run}"
        )
        self._stepper = DriveViewpointStepper(
            viewpoint_spacing_m=self._viewpoint_spacing_m
        )
        self._last_frame = None

    def observe(self) -> None:
        if self._buffer is None:
            return
        frame = self._frame_provider()
        # The cache holds the newest message; re-forwarding the same object
        # every tick would stack duplicates onto one viewpoint.
        if frame is None or frame is self._last_frame:
            return
        try:
            pose = self._robot_pose_provider()
        except CollectionExecutorNodeFactoryError:
            return  # pose not available yet; skip this frame, never fail a route
        self._last_frame = frame
        step_id = self._stepper.observe_pose(pose.x_m, pose.y_m)
        try:
            self._adapter.forward(
                scan_id=self._buffer.scan_id,
                frame=frame,
                scan_step_id=step_id,
                builder=self._buffer,
            )
        except (TypeError, ValueError) as exc:
            # Opportunistic discovery must never abort a healthy route.
            self._node.get_logger().warning(f"drive observation dropped: {exc}")

    def result(self, *, known_positions):
        if self._buffer is None:
            return None
        try:
            snapshot = build_drive_snapshot(
                buffer=self._buffer,
                configuration_snapshot=self._configuration_snapshot,
                court_half_boundary=self._court_half_boundary,
                robot_pose=self._robot_pose_provider(),
                now_s=self._clock.now_s(),
                map_frame=self._map_frame,
                known_positions=known_positions,
                merge_radius_m=self._merge_radius_m,
            )
        except (CollectionExecutorNodeFactoryError, DriveObservationError, ValueError) as exc:
            self._node.get_logger().warning(f"off-route discovery unavailable: {exc}")
            return None
        if snapshot is not None:
            self._node.get_logger().info(
                f"off-route discovery: {len(snapshot.balls)} new target(s) from "
                f"{self._buffer.observation_count} observations"
            )
        return snapshot


class _CollectionRosTransport:
    def __init__(self, node, ros, *, controller_id: str, goal_checker_id: str):
        self.node, self.ros = node, ros
        self.goal_checker_id = goal_checker_id
        base = f"/{controller_id}"
        self.load_client = node.create_client(ros.LoadService, base + "/load_collection_execution_context")
        self.reset_client = node.create_client(
            ros.ResetService, base + "/reset_collection_execution_context"
        )
        self.hold_client = node.create_client(ros.HoldService, base + "/set_collection_safety_hold")
        self.finalize_client = node.create_client(ros.FinalizeService, base + "/finalize_collection_execution_context")
        self.action_client = ros.ActionClient(node, ros.FollowPath, "/follow_path")
        self.latest_state = None
        self.crossing_telemetry = []
        self.state_subscription = node.create_subscription(ros.CollectionControllerState, base + "/state", self._on_state, 10)
        self.load_future = self.reset_future = self.finalize_future = None
        self._pending_load_context = None
        self.goal_future = self.goal_handle = self.result_future = None

    def wait_ready(self, timeout_sec: float) -> bool:
        """Wait for every real controller endpoint used by the handles."""
        return (
            self.load_client.wait_for_service(timeout_sec=timeout_sec)
            and self.reset_client.wait_for_service(timeout_sec=timeout_sec)
            and self.hold_client.wait_for_service(timeout_sec=timeout_sec)
            and self.finalize_client.wait_for_service(timeout_sec=timeout_sec)
            and self.action_client.wait_for_server(timeout_sec=timeout_sec)
        )

    def _on_state(self, message):
        self.latest_state = message
        if bool(getattr(message, "has_active_crossing", False)):
            sample = {
                name: getattr(message, name)
                for name in (
                    "plan_id", "progress_s", "active_segment_id", "active_ball_id",
                    "active_crossing_progress_s", "measured_speed_mps",
                    "lateral_error_m", "heading_error_rad",
                )
            }
            sample["observed_sim_time_s"] = round(
                self.node.get_clock().now().nanoseconds * 1e-9, 3
            )
            verdict = message.profile_verdict
            sample["profile_verdict"] = {
                name: getattr(verdict, name)
                for name in (
                    "hard_compliant", "hard_violation_reason", "nominal_tracking",
                    "measured_speed_mps", "nominal_speed_error_mps",
                )
            }
            if not self.crossing_telemetry or sample != self.crossing_telemetry[-1]:
                self.crossing_telemetry.append(sample)
                # Bounded status telemetry: enough for debugging a route without
                # allowing a long controller run to grow robot_status forever.
                del self.crossing_telemetry[:-200]

    def load_sender(self, values):
        self.latest_state = None
        self.load_future = None
        # A finalize ack from the previous route must never be read as this
        # route's answer.
        self.finalize_future = None
        self._pending_load_context = values
        # The Nav2 controller owns the context lifecycle and outlives this
        # transport.  A new collection run creates a new transport instance,
        # so local load counters cannot tell whether the controller still has
        # a consumed context from an earlier run.  Reset is idempotent in both
        # idle and consumed states; make it the explicit boundary before every
        # context load.
        self.reset_future = self.reset_client.call_async(
            self.ros.ResetService.Request()
        )

    def _send_pending_load(self):
        request = self.ros.LoadService.Request()
        request.context = execution_context_values_to_msg(
            self._pending_load_context, self.ros
        )
        self._pending_load_context = None
        self.load_future = self.load_client.call_async(request)

    def load_outcome_provider(self):
        if self.reset_future is not None:
            if not self.reset_future.done():
                return None
            response = self.reset_future.result()
            self.reset_future = None
            if response is None or not response.accepted:
                detail = getattr(response, "detail", "reset_rejected")
                self.node.get_logger().error(
                    f"collection controller reset rejected: {detail}"
                )
                return "rejected"
            self._send_pending_load()
        if self.load_future is None or not self.load_future.done(): return None
        response = self.load_future.result()
        if response is not None and response.accepted:
            return "accepted"
        detail = getattr(response, "detail", "load_rejected")
        self.node.get_logger().error(
            f"collection execution context load rejected: {detail}"
        )
        return "rejected"

    def follow_path_sender(self, *, map_frame, poses, controller_id):
        self.goal_handle = self.result_future = None
        goal = self.ros.FollowPath.Goal()
        goal.controller_id = controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.path = self.ros.Path()
        stamp = self.node.get_clock().now().to_msg()
        goal.path.header.frame_id, goal.path.header.stamp = map_frame, stamp
        for canonical in poses:
            pose = self.ros.PoseStamped()
            pose.header.frame_id, pose.header.stamp = map_frame, stamp
            pose.pose = _canonical_pose_to_msg(canonical, self.ros)
            goal.path.poses.append(pose)
        self.goal_future = self.action_client.send_goal_async(goal)

    def goal_status_provider(self):
        if self.goal_future is None or not self.goal_future.done(): return "pending"
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted: return "rejected"
            self.result_future = self.goal_handle.get_result_async()
        if self.result_future is None or not self.result_future.done(): return "accepted"
        status = self.result_future.result().status
        names = {self.ros.GoalStatus.STATUS_SUCCEEDED: "succeeded",
                 self.ros.GoalStatus.STATUS_CANCELED: "canceled",
                 self.ros.GoalStatus.STATUS_ABORTED: "aborted"}
        return names.get(status, "failed")

    def state_provider(self):
        if self.latest_state is None: return None
        return {name: getattr(self.latest_state, name) for name in (
            "plan_id", "path_sha256", "lifecycle_state", "progress_s",
            "active_segment_id", "has_active_crossing", "active_ball_id",
            "active_crossing_progress_s", "measured_speed_mps", "lateral_error_m",
            "heading_error_rad", "profile_verdict", "failure_reason")}

    def hold_sender(self, *, plan_id, path_sha256, hold):
        request = self.ros.HoldService.Request()
        request.plan_id, request.path_sha256, request.hold = plan_id, path_sha256, hold
        self.hold_client.call_async(request)

    def finalize_sender(self, *, plan_id, path_sha256, action_outcome):
        request = self.ros.FinalizeService.Request()
        request.plan_id, request.path_sha256, request.action_outcome = plan_id, path_sha256, action_outcome
        # Dispatch only. finalize_sender runs inside the controller_node timer
        # callback, i.e. already on the single-threaded executor, so blocking
        # here with spin_until_future_complete raises "Executor is already
        # spinning" on Jazzy. The response is read by finalize_outcome_provider
        # on later ticks, the same pattern as load_sender/load_outcome_provider.
        self.finalize_future = self.finalize_client.call_async(request)
        return True

    def finalize_outcome_provider(self):
        """Return None while pending, else ("accepted"|"rejected", detail)."""
        if self.finalize_future is None or not self.finalize_future.done():
            return None
        response = self.finalize_future.result()
        self.finalize_future = None
        if response is not None and response.accepted:
            return ("accepted", None)
        detail = getattr(response, "detail", "") or "finalize_rejected"
        code = getattr(response, "rejection_code", None)
        detail = f"collection controller rejected terminal finalize: {detail} (code {code})"
        self.node.get_logger().error(detail)
        return ("rejected", detail)


class CollectionExecutorNodeFactory:
    """Construct all sixteen handles and assemble one dormant executor."""

    def __init__(self, *, node, tf_buffer, cache: CollectionExecutorNodeCache,
                 lane_navigator, collector_interface, court_boundary_path: str | Path,
                 collection_route_config_path: str | Path,
                 calibration_artifact_path: str | Path, telemetry_sink,
                 entry_beam_provider=None, confirmed_beam_provider=None,
                 collector_minimum_drain_s: float = 0.0,
                 collector_maximum_drain_s: float = 0.0,
                 ros_types: CollectionExecutorRosTypes | None = None) -> None:
        if not isinstance(cache, CollectionExecutorNodeCache):
            raise CollectionExecutorNodeFactoryError("cache must be CollectionExecutorNodeCache")
        if not callable(telemetry_sink):
            raise CollectionExecutorNodeFactoryError("telemetry_sink must be callable")
        self.node, self.cache, self.ros = node, cache, ros_types or load_ros_types()
        self._lane_navigator = lane_navigator
        self._collector_interface = collector_interface
        court_boundary = load_court_boundary(court_boundary_path)
        configuration = build_collection_route_configuration(
            load_collection_route_source(collection_route_config_path),
            calibration_artifact_path=calibration_artifact_path,
        )
        robot_pose = _robot_pose(cache)
        runtime = {field: _required_parameter(node, param) for field, param in _RUNTIME_PARAM_FIELDS.items()}
        self.config = CollectionExecutorConfig(
            controller_tuning=read_controller_tuning(node), court_boundary=court_boundary,
            scan_pose_xy_yaw=scan_pose_from_court_model(court_boundary, robot_pose=robot_pose),
            **runtime,
        )
        step_ids = tuple(f"scan-step-{index}" for index in range(self.config.scan_step_count))
        now_ns = node.get_clock().now().nanoseconds
        snapshot_session = CollectionSnapshotRuntimeSession(
            scan_id=f"collection-scan-{now_ns}", scan_timestamp_s=now_ns * 1e-9,
            robot_pose_at_scan=Pose2D(*self.config.scan_pose_xy_yaw),
            configuration_snapshot=configuration,
            expected_scan_step_ids=step_ids,
            court_half_boundary=court_half_from_court_model(court_boundary, robot_pose=robot_pose),
            tf_provider=_TfProvider(tf_buffer, self.ros),
            robot_pose_provider=lambda: _robot_pose(cache),
        )
        self.transport = _CollectionRosTransport(
            node,
            self.ros,
            controller_id=self.config.controller_id,
            goal_checker_id=self.config.goal_checker_id,
        )
        # Scan rotation owns twist_mux's collection input (priority 70).  The
        # FollowPath controller publishes on /cmd_vel_nav (priority 50).  Once
        # scan rotation stops publishing, the collection input expires after
        # the mux timeout and Nav2 takes over; the two producers never share a
        # topic and the controller's hands-off zero cannot overwrite scanning.
        publisher = node.create_publisher(self.ros.Twist, "/cmd_vel_collection", 1)

        def publish_scan_twist(angular_z):
            message = self.ros.Twist()
            message.angular.z = float(angular_z)
            publisher.publish(message)

        self.cmd_vel_publisher = publisher
        self.snapshot_session = snapshot_session
        # Same validation stack as the 360, driven by travel instead of yaw.
        self.drive_observer = _LiveDriveObserver(
            node=node,
            adapter=CollectionSnapshotRuntimeAdapter(
                tf_provider=_TfProvider(tf_buffer, self.ros),
                validation_config=configuration.perception_spatial_validation,
                localization_xy_covariance=configuration.gazebo_snapshot.localization_xy_covariance,
            ),
            configuration_snapshot=configuration,
            court_half_boundary=court_half_from_court_model(
                court_boundary, robot_pose=robot_pose
            ),
            frame_provider=lambda: cache.latest_ball_detections,
            robot_pose_provider=lambda: _robot_pose(cache),
            clock=RosMonotonicClock(node),
            scan_id_prefix=f"collection-scan-{now_ns}",
            viewpoint_spacing_m=self.config.drive_viewpoint_spacing_m,
            merge_radius_m=self.config.drive_known_merge_radius_m,
        )
        self.execution_plan_transformer = _ExecutionPlanTransformer(
            tf_buffer, self.ros
        )
        self.handles = CollectionExecutorHandles(
            telemetry_sink=telemetry_sink, lane_navigator=lane_navigator,
            collector_interface=collector_interface,
            scan_provider=lambda: cache.latest_scan,
            yaw_provider=lambda: cache.robot_yaw_rad,
            frame_provider=lambda: cache.latest_ball_detections,
            cmd_vel=publish_scan_twist, scan_snapshot_session=snapshot_session,
            load_sender=self.transport.load_sender,
            load_outcome_provider=self.transport.load_outcome_provider,
            follow_path_sender=self.transport.follow_path_sender,
            goal_status_provider=self.transport.goal_status_provider,
            state_provider=self.transport.state_provider,
            hold_sender=self.transport.hold_sender,
            finalize_sender=self.transport.finalize_sender,
            finalize_outcome_provider=self.transport.finalize_outcome_provider,
            execution_plan_transformer=self.execution_plan_transformer,
            planner_audit_sink=_planner_audit_sink_from_env(node),
            entry_beam_provider=entry_beam_provider,
            confirmed_beam_provider=confirmed_beam_provider,
            drive_observer=self.drive_observer,
            collector_minimum_drain_s=collector_minimum_drain_s,
            collector_maximum_drain_s=collector_maximum_drain_s,
        )

    def build(self):
        return build_collection_route_executor(node=self.node, config=self.config, handles=self.handles)

    def stop(self) -> None:
        """Best-effort release of every actuator owned by this factory run."""
        self.handles.cmd_vel(0.0)
        goal_handle = self.transport.goal_handle
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        if hasattr(self._lane_navigator, "reset"):
            self._lane_navigator.reset()
        self._collector_interface.stop()

    @property
    def snapshot_diagnostics(self) -> dict:
        return self.snapshot_session.diagnostics

    @property
    def crossing_telemetry(self):
        return list(self.transport.crossing_telemetry)

    @property
    def execution_frame_diagnostics(self) -> dict:
        return dict(self.execution_plan_transformer.last_diagnostics)

    @property
    def controller_state(self):
        state = self.transport.state_provider()
        if state is None:
            return None
        return {
            name: state[name]
            for name in (
                "plan_id", "lifecycle_state", "progress_s", "active_segment_id",
                "has_active_crossing", "active_ball_id", "active_crossing_progress_s",
                "measured_speed_mps", "lateral_error_m", "heading_error_rad",
                "failure_reason",
            )
        }


def build_collection_route_executor_from_node(**kwargs):
    """Convenience entry point returning only the assembled dormant executor."""
    return CollectionExecutorNodeFactory(**kwargs).build()


def _required_parameter(node, name: str):
    try:
        value = node.get_parameter(name).value
    except Exception as exc:
        raise CollectionExecutorNodeFactoryError(f"required ROS parameter {name!r} is missing") from exc
    if value is None:
        raise CollectionExecutorNodeFactoryError(f"required ROS parameter {name!r} is missing")
    return value


def _robot_pose(cache: CollectionExecutorNodeCache) -> Pose2D:
    values = (cache.robot_x_m, cache.robot_y_m, cache.robot_yaw_rad)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise CollectionExecutorNodeFactoryError("cached robot pose is unavailable or invalid")
    return Pose2D(float(values[0]), float(values[1]), float(values[2]))
