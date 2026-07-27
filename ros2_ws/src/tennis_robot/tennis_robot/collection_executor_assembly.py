"""Assemble a CollectionRouteExecutor from ROS handles (Phase 6C.2 synthesis).

Wires all eight executor ports — the 6C.1 sensor/actuator adapters, the 6C.2
live PathFollower, a PurePlanner that runs ``plan_collection_route`` against a
CourtModel built by the Phase 6A builder, and the ScanSession driver — into a
single ``CollectionRouteExecutor``.

No ``rclpy`` import here either: ``node`` and every handle are duck-typed and
supplied by the caller (the Phase 6D node wiring, or fakes in tests).  This
function is deliberately NOT called from ``controller_node`` yet (that is 6D).

``controller_tuning`` is controller-runtime config, not part of the frozen plan:
it comes from ``nav2_params.yaml`` ROS params (see :func:`read_controller_tuning`)
and is passed straight into the 6B context builder.  All thresholds/timeouts are
required — no defaults.
"""

from __future__ import annotations

from dataclasses import dataclass

from tennis_robot.collection_court_model_builder import build_court_model
from tennis_robot.collection_execution_context_builder import ControllerTuning
from tennis_robot.collection_executor_ports import (
    CallbackTelemetrySink,
    ForwardSectorSafetyLogic,
    GazeboCollectorAdapter,
    LidarSafetyMonitor,
    RosMonotonicClock,
    ScanPoseNavigatorAdapter,
    ScanRotationFsm,
    ScanSessionDriver,
)
from tennis_robot.collection_path_follower_port import (
    DEFAULT_CONTROLLER_ID,
    LiveCollectionPathFollower,
)
from tennis_robot.collection_route_executor import CollectionRouteExecutor

# ROS param names for the controller tuning block (see config/nav2_params.yaml).
_TUNING_PARAMS = (
    "lookahead_distance_m",
    "max_angular_velocity_rad_s",
    "progress_projection_window_m",
    "crossing_speed_window_m",
    "terminal_progress_tolerance_m",
)
_TUNING_PARAM_PREFIX = "collection_controller_tuning."


def read_controller_tuning(node, *, prefix: str = _TUNING_PARAM_PREFIX) -> ControllerTuning:
    """Read the 5-field controller tuning from a duck-typed node's ROS params.

    ``ControllerTuning`` validates every field finite and > 0, matching the C++
    ``valid_tuning``.  Duck-typed: uses ``node.get_parameter(name).value`` only,
    so it never imports rclpy.
    """
    values = {name: node.get_parameter(prefix + name).value for name in _TUNING_PARAMS}
    return ControllerTuning(
        lookahead_distance_m=values["lookahead_distance_m"],
        max_angular_velocity_rad_s=values["max_angular_velocity_rad_s"],
        progress_projection_window_m=values["progress_projection_window_m"],
        crossing_speed_window_m=values["crossing_speed_window_m"],
        terminal_progress_tolerance_m=values["terminal_progress_tolerance_m"],
    )


@dataclass(frozen=True)
class CollectionExecutorConfig:
    """Plain (ROS-free) runtime configuration for the assembled executor."""

    controller_tuning: ControllerTuning
    context_schema_version: str
    context_activation_timeout_s: float
    court_boundary: dict
    scan_pose_xy_yaw: tuple[float, float, float]
    scan_step_count: int
    scan_yaw_tolerance_rad: float
    scan_start_yaw_rad: float
    scan_angular_speed_rad_s: float
    scan_timeout_s: float
    safety_forward_half_angle_rad: float
    safety_stop_distance_m: float
    safety_pause_timeout_s: float
    safety_max_scan_age_s: float
    controller_id: str = DEFAULT_CONTROLLER_ID
    goal_checker_id: str = "collection_goal_checker"


@dataclass(frozen=True)
class CollectionExecutorHandles:
    """Injected duck-typed ROS handles the assembled ports drive."""

    telemetry_sink: object          # callable(dict) -> None
    lane_navigator: object          # Nav2LaneNavigator-like (.request, .state)
    collector_interface: object     # CollectorInterface-like (.start, .stop)
    scan_provider: object           # callable() -> LaserScan | None
    yaw_provider: object            # callable() -> float | None
    frame_provider: object          # callable() -> BallDetectionArray | None
    cmd_vel: object                 # callable(angular_z) -> None
    scan_snapshot_session: object   # .forward_frame(frame, *, scan_step_id), .finalize(now_s)
    # Live PathFollower transport handles:
    load_sender: object             # callable(context_values) -> None
    load_outcome_provider: object   # callable() -> None|"accepted"|"rejected"
    follow_path_sender: object      # callable(*, map_frame, poses, controller_id) -> None
    goal_status_provider: object    # callable() -> str
    state_provider: object          # callable() -> dict | None
    hold_sender: object             # callable(*, plan_id, path_sha256, hold) -> None
    finalize_sender: object         # callable(*, plan_id, path_sha256, action_outcome) -> bool
    execution_plan_transformer: object  # callable(CollectionRoutePlan) -> CollectionRoutePlan
    entry_beam_provider: object | None = None
    confirmed_beam_provider: object | None = None
    collector_minimum_drain_s: float = 0.0
    collector_maximum_drain_s: float = 0.0


class _PlanCollectionRoutePlanner:
    """PurePlanner over ``plan_collection_route`` with a fixed CourtModel.

    Uses ``snapshot.configuration_snapshot`` as the planner configuration so it
    always matches the snapshot the executor produced.
    """

    def __init__(self, court) -> None:
        self._court = court

    def plan(self, snapshot):
        from tennis_robot.collection_route_planner_v2 import plan_collection_route

        return plan_collection_route(
            snapshot=snapshot, court=self._court, configuration=snapshot.configuration_snapshot
        ).plan


def build_collection_route_executor(
    *, node, config: CollectionExecutorConfig, handles: CollectionExecutorHandles
) -> CollectionRouteExecutor:
    """Compose all eight ports into a ready-to-tick CollectionRouteExecutor."""
    if not isinstance(config, CollectionExecutorConfig):
        raise TypeError("config must be a CollectionExecutorConfig")
    if not isinstance(handles, CollectionExecutorHandles):
        raise TypeError("handles must be a CollectionExecutorHandles")

    clock = RosMonotonicClock(node)
    telemetry = CallbackTelemetrySink(handles.telemetry_sink)

    navigator = ScanPoseNavigatorAdapter(
        lane_navigator=handles.lane_navigator, scan_pose=config.scan_pose_xy_yaw
    )
    collector = GazeboCollectorAdapter(
        handles.collector_interface,
        entry_beam_provider=handles.entry_beam_provider,
        confirmed_beam_provider=handles.confirmed_beam_provider,
        minimum_drain_s=handles.collector_minimum_drain_s,
        maximum_drain_s=handles.collector_maximum_drain_s,
        clock_fn=clock.now_s,
    )

    safety_logic = ForwardSectorSafetyLogic(
        forward_half_angle_rad=config.safety_forward_half_angle_rad,
        stop_distance_m=config.safety_stop_distance_m,
        safety_pause_timeout_s=config.safety_pause_timeout_s,
        max_scan_age_s=config.safety_max_scan_age_s,
    )
    safety_monitor = LidarSafetyMonitor(logic=safety_logic, clock=clock, scan_provider=handles.scan_provider)

    scan_session = ScanSessionDriver(
        fsm=ScanRotationFsm(
            step_count=config.scan_step_count,
            yaw_tolerance_rad=config.scan_yaw_tolerance_rad,
            # Navigation ends at this yaw.  A separate fixed start yaw can
            # force up to almost one extra revolution before scan-step-0 and
            # exhaust the scan timeout before all headings are visited.
            start_yaw_rad=config.scan_pose_xy_yaw[2],
        ),
        snapshot_session=handles.scan_snapshot_session,
        yaw_provider=handles.yaw_provider,
        frame_provider=handles.frame_provider,
        cmd_vel=handles.cmd_vel,
        clock=clock,
        angular_speed_rad_s=config.scan_angular_speed_rad_s,
        scan_timeout_s=config.scan_timeout_s,
    )

    planner = _PlanCollectionRoutePlanner(build_court_model(config.court_boundary))

    path_follower = LiveCollectionPathFollower(
        controller_tuning=config.controller_tuning,
        context_schema_version=config.context_schema_version,
        context_activation_timeout_s=config.context_activation_timeout_s,
        load_sender=handles.load_sender,
        load_outcome_provider=handles.load_outcome_provider,
        follow_path_sender=handles.follow_path_sender,
        goal_status_provider=handles.goal_status_provider,
        state_provider=handles.state_provider,
        hold_sender=handles.hold_sender,
        finalize_sender=handles.finalize_sender,
        execution_plan_transformer=handles.execution_plan_transformer,
        clock=clock,
        controller_id=config.controller_id,
    )

    return CollectionRouteExecutor(
        navigator=navigator,
        scan_session=scan_session,
        planner=planner,
        collector=collector,
        path_follower=path_follower,
        safety_monitor=safety_monitor,
        telemetry=telemetry,
        clock=clock,
    )
