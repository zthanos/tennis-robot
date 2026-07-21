"""Phase 6C.2: live PathFollower port + executor assembly — fake-ROS, no rclpy.

Fake service/action/state handles drive the handshake so every mapping is
proven offline.  The container smoke (scripts/run_collection_follower_smoke.sh)
proves the same adapter against the real C++ controller.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration
from tennis_robot.collection_execution_context_builder import ControllerTuning
from tennis_robot.collection_executor_assembly import (
    CollectionExecutorConfig,
    CollectionExecutorHandles,
    build_collection_route_executor,
    read_controller_tuning,
)
from tennis_robot.collection_path_follower_port import (
    FAILURE_SAFETY_RESUME_INVALID,
    LIFECYCLE_EXECUTING,
    LIFECYCLE_FAILED,
    LIFECYCLE_SAFETY_PAUSED,
    LiveCollectionPathFollower,
    PathFollowerPortError,
    failure_reason_for_code,
)
from tennis_robot.collection_court_model_builder import build_court_model
from tennis_robot.collection_route_executor import (
    ExecutorReasonCode,
    ExecutorState,
    PathFollowerStatus,
)
from tennis_robot.collection_route_planner_v2 import plan_collection_route
from tennis_robot.collection_route_types import (
    Point2D,
    Pose2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)


class Clock:
    def __init__(self, value=0.0):
        self.value = value

    def now_s(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


_BOUNDARY = {
    "schema": "court_knowledge_model/v2", "status": "OK", "failure_reason": None, "frame": "map",
    "completed": True,
    "net": {"center": {"x_m": 8.0, "y_m": 0.0}, "posts": [{"x_m": 8.0, "y_m": 6.0}, {"x_m": 8.0, "y_m": -6.0}], "span_m": 12.0},
    "fence": {"corners": [{"x_m": -9.0, "y_m": -8.0}, {"x_m": 9.0, "y_m": -8.0}, {"x_m": 9.0, "y_m": 8.0}, {"x_m": -9.0, "y_m": 8.0}]},
    "obstacles": [],
}


def _curved_plan():
    config = default_configuration()
    court = build_court_model(_BOUNDARY)
    snapshot = ScanSnapshot(
        "scan-6c2", 1000.0, "map", Pose2D(0.0, 0.0, 0.0),
        (SnapshotBall("ball-6c2", Point2D(0.0, 3.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        config,
    )
    plan = plan_collection_route(snapshot=snapshot, court=court, configuration=config).plan
    assert plan.is_executable
    return plan


def _tuning():
    return ControllerTuning(1.0, 3.0, 10.0, 0.25, 0.05)


class Transport:
    """Scriptable fake of the 7 ROS handles + records every call."""

    def __init__(self, *, load_outcome=None, goal_status="pending", state=None, finalize_accepted=True):
        self.load_outcome = load_outcome
        self.goal_status = goal_status
        self.state = state
        self._finalize_accepted = finalize_accepted
        self.loaded = []
        self.follow_paths = []
        self.holds = []
        self.finalizes = []

    def load_sender(self, context):
        self.loaded.append(context)

    def load_outcome_provider(self):
        return self.load_outcome

    def follow_path_sender(self, *, map_frame, poses, controller_id):
        self.follow_paths.append((map_frame, poses, controller_id))

    def goal_status_provider(self):
        return self.goal_status

    def state_provider(self):
        return self.state

    def hold_sender(self, *, plan_id, path_sha256, hold):
        self.holds.append((plan_id, path_sha256, hold))

    def finalize_sender(self, *, plan_id, path_sha256, action_outcome):
        self.finalizes.append((plan_id, path_sha256, action_outcome))
        return self._finalize_accepted


def _follower(transport, clock=None):
    clock = clock or Clock()
    return LiveCollectionPathFollower(
        controller_tuning=_tuning(),
        context_schema_version="collection-execution-context/v1",
        context_activation_timeout_s=5.0,
        load_sender=transport.load_sender,
        load_outcome_provider=transport.load_outcome_provider,
        follow_path_sender=transport.follow_path_sender,
        goal_status_provider=transport.goal_status_provider,
        state_provider=transport.state_provider,
        hold_sender=transport.hold_sender,
        finalize_sender=transport.finalize_sender,
        clock=clock,
    ), clock


# ── mapping unit tests ───────────────────────────────────────────────────────
def test_failure_code_mapping():
    assert failure_reason_for_code(FAILURE_SAFETY_RESUME_INVALID) is ExecutorReasonCode.SAFETY_RESUME_INVALID
    for code in (10, 9, 5, 12, 13, 11, 4):  # tube/curvature/speed/reverse/rotate/non-monotonic/profile
        assert failure_reason_for_code(code) is ExecutorReasonCode.PATH_FAILED


def test_start_sends_load_with_matching_sha_context():
    plan = _curved_plan()
    transport = Transport(load_outcome=None)
    follower, _ = _follower(transport)
    follower.start(plan)
    assert len(transport.loaded) == 1
    context = transport.loaded[0]
    assert context.plan_id == plan.plan_id
    assert context.path_sha256 == context.path_sha256  # carried
    # No FollowPath yet (Load still pending) -> running placeholder.
    result = follower.result()
    assert result.status is PathFollowerStatus.RUNNING
    assert transport.follow_paths == []


def test_load_accept_sends_follow_path_then_running():
    plan = _curved_plan()
    transport = Transport(load_outcome="accepted")
    follower, _ = _follower(transport)
    follower.start(plan)
    result = follower.result()
    assert transport.follow_paths, "FollowPath must be sent on Load accept"
    map_frame, poses, controller_id = transport.follow_paths[0]
    assert controller_id == "CollectionFollowPath"
    assert map_frame == plan.map_frame
    assert len(poses) >= 2
    assert result.status is PathFollowerStatus.RUNNING


def test_load_reject_fails():
    transport = Transport(load_outcome="rejected")
    follower, _ = _follower(transport)
    follower.start(_curved_plan())
    result = follower.result()
    assert result.status is PathFollowerStatus.FAILED
    assert result.reason is ExecutorReasonCode.PATH_FAILED


def test_activation_timeout_fails():
    transport = Transport(load_outcome=None)  # Load never resolves
    clock = Clock()
    follower, _ = _follower(transport, clock)
    follower.start(_curved_plan())
    assert follower.result().status is PathFollowerStatus.RUNNING
    clock.advance(6.0)  # past context_activation_timeout_s=5.0
    result = follower.result()
    assert result.status is PathFollowerStatus.FAILED
    assert result.reason is ExecutorReasonCode.PATH_FAILED


def test_executing_state_maps_to_running_with_progress_and_tube():
    plan = _curved_plan()
    transport = Transport(load_outcome="accepted", goal_status="accepted",
                          state={"lifecycle_state": LIFECYCLE_EXECUTING, "progress_s": 2.0, "lateral_error_m": 0.01, "failure_reason": 0})
    follower, _ = _follower(transport)
    follower.start(plan)
    follower.result()  # send follow path
    result = follower.result()
    assert result.status is PathFollowerStatus.RUNNING
    assert result.progress_s == 2.0
    assert result.trajectory_tube_ok is True
    assert result.requires_reverse is False and result.requires_standalone_rotate is False
    # remaining_run_in_m equals the pure follower's value at progress 2.0.
    from tennis_robot.collection_path_follower import CollectionPathFollower
    assert result.remaining_run_in_m == CollectionPathFollower(plan).remaining_run_in_m(2.0)


def test_tube_violation_when_lateral_exceeds_radius():
    plan = _curved_plan()
    radius = plan.configuration_snapshot.safety.trajectory_tube_radius_m
    transport = Transport(load_outcome="accepted", goal_status="accepted",
                          state={"lifecycle_state": LIFECYCLE_EXECUTING, "progress_s": 1.0, "lateral_error_m": radius + 0.01, "failure_reason": 0})
    follower, _ = _follower(transport)
    follower.start(plan)
    follower.result()
    result = follower.result()
    assert result.status is PathFollowerStatus.RUNNING
    assert result.trajectory_tube_ok is False


def test_safety_paused_is_still_running():
    transport = Transport(load_outcome="accepted", goal_status="accepted",
                          state={"lifecycle_state": LIFECYCLE_SAFETY_PAUSED, "progress_s": 1.0, "lateral_error_m": 0.0, "failure_reason": 0})
    follower, _ = _follower(transport)
    follower.start(_curved_plan())
    follower.result()
    assert follower.result().status is PathFollowerStatus.RUNNING


def test_state_failure_reason_maps_to_failed():
    transport = Transport(load_outcome="accepted", goal_status="active",
                          state={"lifecycle_state": LIFECYCLE_EXECUTING, "progress_s": 1.0, "lateral_error_m": 0.0, "failure_reason": 10})
    follower, _ = _follower(transport)
    follower.start(_curved_plan())
    follower.result()
    result = follower.result()
    assert result.status is PathFollowerStatus.FAILED
    assert result.reason is ExecutorReasonCode.PATH_FAILED


def test_lifecycle_failed_maps_to_failed():
    transport = Transport(load_outcome="accepted", goal_status="active",
                          state={"lifecycle_state": LIFECYCLE_FAILED, "progress_s": 1.0, "lateral_error_m": 0.0, "failure_reason": 14})
    follower, _ = _follower(transport)
    follower.start(_curved_plan())
    follower.result()
    result = follower.result()
    assert result.status is PathFollowerStatus.FAILED
    assert result.reason is ExecutorReasonCode.SAFETY_RESUME_INVALID


def test_succeeded_completes_and_finalizes_exactly_once():
    plan = _curved_plan()
    transport = Transport(load_outcome="accepted", goal_status="succeeded", state=None)
    follower, _ = _follower(transport)
    follower.start(plan)
    follower.result()  # send follow path
    result = follower.result()
    assert result.status is PathFollowerStatus.COMPLETED
    assert transport.finalizes == [(plan.plan_id, follower._path_sha256, 0)]
    assert follower.finalize_accepted is True
    # Terminal is cached and does not re-finalize.
    assert follower.result().status is PathFollowerStatus.COMPLETED
    assert len(transport.finalizes) == 1


def test_finalize_only_on_terminal_never_mid_execution():
    transport = Transport(load_outcome="accepted", goal_status="accepted",
                          state={"lifecycle_state": LIFECYCLE_EXECUTING, "progress_s": 1.0, "lateral_error_m": 0.0, "failure_reason": 0})
    follower, _ = _follower(transport)
    follower.start(_curved_plan())
    follower.result()
    follower.result()
    assert transport.finalizes == []  # no Finalize while executing


def test_pause_resume_send_hold_calls():
    plan = _curved_plan()
    transport = Transport(load_outcome="accepted")
    follower, _ = _follower(transport)
    follower.start(plan)
    follower.pause()
    follower.resume()
    assert transport.holds == [
        (plan.plan_id, follower._path_sha256, True),
        (plan.plan_id, follower._path_sha256, False),
    ]


def test_result_before_start_raises():
    transport = Transport()
    follower, _ = _follower(transport)
    with pytest.raises(PathFollowerPortError):
        follower.result()


def test_non_executable_plan_rejected():
    config = default_configuration()
    court = build_court_model(_BOUNDARY)
    empty = plan_collection_route(
        snapshot=ScanSnapshot("s", 1000.0, "map", Pose2D(0, 0, 0), (), config), court=court, configuration=config
    ).plan
    transport = Transport()
    follower, _ = _follower(transport)
    with pytest.raises(PathFollowerPortError):
        follower.start(empty)


# ── read_controller_tuning + assembly ────────────────────────────────────────
class FakeParam:
    def __init__(self, value):
        self.value = value


class FakeNode:
    def __init__(self, params, nanoseconds=0):
        self._params = params
        self._ns = nanoseconds

    def get_parameter(self, name):
        return FakeParam(self._params[name])

    def get_clock(self):
        node = self

        class _C:
            def now(self_inner):
                from types import SimpleNamespace
                return SimpleNamespace(nanoseconds=node._ns)

        return _C()

    def advance(self, seconds):
        self._ns += int(seconds * 1e9)


def test_read_controller_tuning_from_node_params():
    node = FakeNode({
        "collection_controller_tuning.lookahead_distance_m": 1.0,
        "collection_controller_tuning.max_angular_velocity_rad_s": 3.0,
        "collection_controller_tuning.progress_projection_window_m": 10.0,
        "collection_controller_tuning.crossing_speed_window_m": 0.25,
        "collection_controller_tuning.terminal_progress_tolerance_m": 0.05,
    })
    tuning = read_controller_tuning(node)
    assert tuning.lookahead_distance_m == 1.0
    assert tuning.terminal_progress_tolerance_m == 0.05


def _assembly_smoke_handles(plan):
    # A scripted transport whose Load accepts and whose goal immediately succeeds.
    transport = Transport(load_outcome="accepted", goal_status="succeeded", state=None)
    telemetry = []

    class LaneNav:
        def __init__(self):
            self.state = type("S", (), {"value": "reached"})()

        def request(self, x, y, yaw):
            pass

    class Collector:
        def start(self):
            pass

        def stop(self):
            pass

    class ScanSession:
        def forward_frame(self, frame, *, scan_step_id):
            pass

        def finalize(self, now_s):
            return plan_snapshot  # the snapshot the planner turns into `plan`

    plan_snapshot = _snapshot_for(plan)

    handles = CollectionExecutorHandles(
        telemetry_sink=telemetry.append,
        lane_navigator=LaneNav(),
        collector_interface=Collector(),
        scan_provider=lambda: None,
        yaw_provider=lambda: 0.0,
        frame_provider=lambda: None,
        cmd_vel=lambda angular: None,
        scan_snapshot_session=ScanSession(),
        load_sender=transport.load_sender,
        load_outcome_provider=transport.load_outcome_provider,
        follow_path_sender=transport.follow_path_sender,
        goal_status_provider=transport.goal_status_provider,
        state_provider=transport.state_provider,
        hold_sender=transport.hold_sender,
        finalize_sender=transport.finalize_sender,
    )
    return handles, transport, telemetry


def _snapshot_for(plan):
    return ScanSnapshot(
        "scan-6c2", 1000.0, "map", Pose2D(0.0, 0.0, 0.0),
        (SnapshotBall("ball-6c2", Point2D(0.0, 3.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        default_configuration(),
    )


def test_build_collection_route_executor_runs_full_fake_cycle_to_completed():
    plan = _curved_plan()
    handles, transport, telemetry = _assembly_smoke_handles(plan)
    node = FakeNode({}, nanoseconds=0)
    config = CollectionExecutorConfig(
        controller_tuning=_tuning(),
        context_schema_version="collection-execution-context/v1",
        context_activation_timeout_s=5.0,
        court_boundary=_BOUNDARY,
        scan_pose_xy_yaw=(0.0, 0.0, 0.0),
        scan_step_count=1,          # single-step scan for the smoke
        scan_yaw_tolerance_rad=3.0,  # wide: any fed yaw captures the one step
        scan_start_yaw_rad=0.0,
        scan_angular_speed_rad_s=0.5,
        scan_timeout_s=100.0,
        safety_forward_half_angle_rad=0.3,
        safety_stop_distance_m=1.0,
        safety_pause_timeout_s=2.0,
    )
    executor = build_collection_route_executor(node=node, config=config, handles=handles)
    executor.start()
    for _ in range(60):
        executor.tick()
        if executor.state in (ExecutorState.COMPLETED, ExecutorState.COMPLETED_NO_TARGETS,
                              ExecutorState.ABORTED_SCAN, ExecutorState.ABORTED_PLANNING,
                              ExecutorState.ABORTED_COLLECTOR, ExecutorState.ABORTED_SAFETY,
                              ExecutorState.ABORTED_TRACKING):
            break
    assert executor.state is ExecutorState.COMPLETED
    assert transport.follow_paths, "FollowPath was sent through the assembled follower"
    assert transport.finalizes, "Finalize was sent at terminal"
