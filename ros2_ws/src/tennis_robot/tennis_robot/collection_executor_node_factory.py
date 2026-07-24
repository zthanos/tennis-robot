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
from tennis_robot.collection_route_config_builder import build_collection_route_configuration
from tennis_robot.collection_route_types import Point2D, Pose2D
from tennis_robot.collection_scan_snapshot import CourtHalfBoundary
from tennis_robot.collection_snapshot_runtime_adapter import CollectionSnapshotRuntimeSession
from tennis_robot.perception_spatial_observation_adapter import TimestampedCameraToMapTransform


class CollectionExecutorNodeFactoryError(ValueError):
    """A required node-side dependency or configuration value is invalid."""


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
        SetCollectionSafetyHold,
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
    """Return the centre of the service line on the robot's current net side."""
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
        math.atan2(-service_x * ly, -service_x * lx),
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

    def __call__(self, plan):
        if plan.map_frame == "odom":
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
        return transform_collection_plan(
            plan,
            RigidTransform2D(
                target_frame="odom",
                source_frame=plan.map_frame,
                x_m=float(translation.x),
                y_m=float(translation.y),
                yaw_rad=yaw,
            ),
        )


class _CollectionRosTransport:
    def __init__(self, node, ros, *, controller_id: str, goal_checker_id: str):
        self.node, self.ros = node, ros
        self.goal_checker_id = goal_checker_id
        base = f"/{controller_id}"
        self.load_client = node.create_client(ros.LoadService, base + "/load_collection_execution_context")
        self.hold_client = node.create_client(ros.HoldService, base + "/set_collection_safety_hold")
        self.finalize_client = node.create_client(ros.FinalizeService, base + "/finalize_collection_execution_context")
        self.action_client = ros.ActionClient(node, ros.FollowPath, "/follow_path")
        self.latest_state = None
        self.crossing_telemetry = []
        self.state_subscription = node.create_subscription(ros.CollectionControllerState, base + "/state", self._on_state, 10)
        self.load_future = self.goal_future = self.goal_handle = self.result_future = None

    def wait_ready(self, timeout_sec: float) -> bool:
        """Wait for all four real controller endpoints used by the handles."""
        return (
            self.load_client.wait_for_service(timeout_sec=timeout_sec)
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
        request = self.ros.LoadService.Request()
        request.context = execution_context_values_to_msg(values, self.ros)
        self.load_future = self.load_client.call_async(request)

    def load_outcome_provider(self):
        if self.load_future is None or not self.load_future.done(): return None
        response = self.load_future.result()
        return "accepted" if response is not None and response.accepted else "rejected"

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
        # Fire-and-forget, exactly like hold_sender above. finalize_sender runs
        # inside the controller_node timer callback, i.e. already on the
        # single-threaded executor, so blocking here with
        # spin_until_future_complete raises "Executor is already spinning" on
        # Jazzy (Humble's rclpy tolerated the nested spin, which is why the
        # route used to complete). The request is still delivered to the
        # collection controller; the ack is not awaited — by the time finalize
        # is sent the Nav2 goal has already reported success.
        self.finalize_client.call_async(request)
        return True


class CollectionExecutorNodeFactory:
    """Construct all sixteen handles and assemble one dormant executor."""

    def __init__(self, *, node, tf_buffer, cache: CollectionExecutorNodeCache,
                 lane_navigator, collector_interface, court_boundary_path: str | Path,
                 collection_route_config_path: str | Path,
                 calibration_artifact_path: str | Path, telemetry_sink,
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
            execution_plan_transformer=_ExecutionPlanTransformer(tf_buffer, self.ros),
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
