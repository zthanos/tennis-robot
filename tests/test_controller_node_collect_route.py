"""Phase 6D.4 node wiring: collect_route is executor-owned and hands-off."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
from types import MethodType, ModuleType, SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tennis_robot"))

# The host-side unit environment does not build the ROS interface package.  The
# controller methods under test only need these messages as small value shells.
try:
    import tennis_robot_msgs.msg  # noqa: F401
except ImportError:
    package = ModuleType("tennis_robot_msgs")
    messages = ModuleType("tennis_robot_msgs.msg")

    class Message:
        def __init__(self, **values):
            self.lift_wheel_speed = 0.0
            self.intake_enabled = False
            for name, value in values.items():
                setattr(self, name, value)

    for name in ("BallDetectionArray", "CollectorCmd", "IrReadings", "RobotCommand"):
        setattr(messages, name, type(name, (Message,), {}))
    package.msg = messages
    sys.modules["tennis_robot_msgs"] = package
    sys.modules["tennis_robot_msgs.msg"] = messages

from tennis_robot.collection_executor_node_factory import CollectionExecutorNodeCache
from tennis_robot.collection_route_executor import ExecutorReasonCode, ExecutorState
from tennis_robot.collector import BaseCommand, CollectorCommand, CollectorState, ConceptACommand
from tennis_robot.controller_node import ControllerNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class EmptyPlan:
    segments = ()

    def to_dict(self):
        return {
            "plan_id": "empty-plan",
            "planning_status": "empty_no_balls",
            "ball_results": [],
            "segments": [],
        }


class FakeExecutor:
    def __init__(self):
        self.state = ExecutorState.COMPLETED_NO_TARGETS
        self.route_outcome = None
        self.terminal_reason = None
        self.terminal_detail = None
        self.plan = EmptyPlan()
        self.started = 0
        self.ticks = 0

    @property
    def is_terminal(self):
        return True

    def start(self):
        self.started += 1

    def tick(self):
        self.ticks += 1
        return self.state


class FakeFactory:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.executor = FakeExecutor()
        self.crossing_telemetry = [{"active_ball_id": "ball-1"}]
        self.snapshot_diagnostics = {
            "minimum_confirmation_count": 2,
            "tracks": [
                {"x_m": 3.0, "y_m": 1.0, "steps": ["scan-step-1"], "confirmed": False},
                {"x_m": 4.0, "y_m": 2.0, "steps": ["scan-step-2", "scan-step-3"], "confirmed": True},
            ],
        }
        self.stopped = 0
        self.__class__.instances.append(self)

    def build(self):
        return self.executor

    def stop(self):
        self.stopped += 1


def _node(monkeypatch):
    import ament_index_python.packages

    monkeypatch.setenv("COLLECTION_ROUTE_CALIBRATION_ARTIFACT", "/explicit/calibration.json")
    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_share_directory",
        lambda package: str(ROOT / "ros2_ws/src/tennis_robot"),
    )
    node = SimpleNamespace(
        collection_executor_factory_type=FakeFactory,
        _nav2_lane=object(),
        _tf_buffer=object(),
        _collection_executor_cache=CollectionExecutorNodeCache(),
        _latest_scan_msg="scan",
        _latest_ball_detections_msg="detections",
        _latest_perception_diagnostics={
            "schema_version": 1,
            "detections_2d": 1,
            "spatial_accepted": 0,
            "spatial_rejected": 1,
            "rejection_counts": {"calibration_out_of_domain": 1},
        },
        _robot_x=1.0,
        _robot_y=2.0,
        _robot_yaw=0.3,
        _pub_collector=Publisher(),
        _pub_motion_cmd=Publisher(),
        _collect_route_executor=None,
        _collect_route_executor_factory=None,
        _collect_route_executor_events=[],
        _collect_route_executor_complete_reported=False,
        _collect_route_confirmations=[],
        _collect_route_run_history=[],
        _last_collect_route_summary={},
        _run_id="run",
        collection_count=7,
        _collect_route_run_start_count=5,
        _credit_reconciler=SimpleNamespace(beam_count=2, truth_count=2),
        _collect_route_collector_active=False,
        ball_map=SimpleNamespace(reset=lambda: None),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _runtime_seconds=lambda: 12.0,
        _declare_collection_route_parameters=lambda: None,
        _sim_true_pose=None,
        _pose_frame_offset=None,
        _pose_frame_yaw_offset=None,
    )
    node._on_collection_executor_telemetry = MethodType(
        ControllerNode._on_collection_executor_telemetry, node
    )
    node._start_collection_route_executor = MethodType(
        ControllerNode._start_collection_route_executor, node
    )
    node._collect_route_elapsed_s = MethodType(
        ControllerNode._collect_route_elapsed_s, node
    )
    node._pose_error_m = MethodType(ControllerNode._pose_error_m, node)
    node._pose_yaw_error_rad = MethodType(ControllerNode._pose_yaw_error_rad, node)
    node._build_collect_route_execution_outcomes = MethodType(
        ControllerNode._build_collect_route_execution_outcomes, node
    )
    node._capture_collect_route_run = MethodType(
        ControllerNode._capture_collect_route_run, node
    )
    node._write_collect_route_audit = MethodType(
        ControllerNode._write_collect_route_audit, node
    )
    return node


def test_mode_entry_builds_starts_and_ticks_executor_to_empty_terminal(monkeypatch):
    FakeFactory.instances.clear()
    node = _node(monkeypatch)
    node._on_mode_changed = lambda mode: True
    published_modes = []
    node._publish_command = lambda mode, source: published_modes.append((mode, source))

    command = ControllerNode._collect_route_command_for_mode(node, "collect_route")

    assert len(FakeFactory.instances) == 1
    factory = FakeFactory.instances[0]
    assert factory.executor.started == 1
    assert factory.executor.ticks == 1
    assert factory.kwargs["cache"] is node._collection_executor_cache
    assert factory.kwargs["court_boundary_path"].name == "court_boundary.json"
    assert factory.kwargs["collection_route_config_path"].name == "collection_route.yaml"
    assert factory.kwargs["calibration_artifact_path"] == "/explicit/calibration.json"
    assert published_modes == [("idle", "controller-collect-route-complete")]
    assert command.state is CollectorState.IDLE
    assert command.base == BaseCommand(0.0, 0.0)
    assert command.collector == CollectorCommand(0.0, False)


def test_executor_status_serializes_empty_plan_and_crossing_telemetry(monkeypatch):
    node = _node(monkeypatch)
    node._on_mode_changed = lambda mode: False
    node._publish_command = lambda mode, source: None
    node._start_collection_route_executor()

    status = ControllerNode._build_collect_route_summary(node)

    assert status["state"] == "completed_no_targets"
    assert status["status"] == "completed_no_targets"
    assert status["route_outcome"] is None
    assert status["failure_reason"] is None
    assert status["failure_detail"] is None
    assert status["plan_id"] == "empty-plan"
    assert status["planning_status"] == "empty_no_balls"
    assert status["ball_results"] == []
    assert status["segments"] == []
    assert status["crossings"] == []
    assert status["executed_crossing_telemetry"] == [{"active_ball_id": "ball-1"}]
    assert status["route_collected"] == 2
    assert status["beam_credits"] == 2
    assert status["truth_retained"] == 2
    assert status["basket_retained"] == 2
    assert status["execution_outcomes"] == []
    assert status["confirmations"] == []
    assert status["unassigned_confirmations"] == 0
    assert status["pose_drift_m"] is None
    assert status["yaw_drift_rad"] is None
    assert status["perception_diagnostics"]["rejection_counts"] == {
        "calibration_out_of_domain": 1
    }


def test_aborted_summary_preserves_primary_failure_reason_and_detail(monkeypatch):
    node = _node(monkeypatch)
    node._collect_route_executor = SimpleNamespace(
        state=ExecutorState.ABORTED_TRACKING,
        route_outcome=ExecutorState.ABORTED_TRACKING,
        terminal_reason=ExecutorReasonCode.PATH_FAILED,
        terminal_detail=(
            "heading_error_exceeded | progress 10.198m "
            "lat_err 0.000m head_err -0.151rad"
        ),
        plan=None,
    )

    status = ControllerNode._build_collect_route_summary(node)

    assert status["status"] == "aborted_tracking"
    assert status["route_outcome"] == "aborted_tracking"
    assert status["failure_reason"] == "path_failed"
    assert status["failure_detail"].startswith("heading_error_exceeded")


def test_execution_outcomes_keep_planner_result_immutable_and_add_physical_status(monkeypatch):
    node = _node(monkeypatch)
    node._collect_route_confirmations = [
        {"ball_id": "ball-a", "association": "active_crossing", "t_s": 4.2},
        {"ball_id": None, "association": "unassigned", "t_s": 7.0},
    ]
    planner_results = [
        {"ball_id": "ball-a", "status": "covered", "reason_code": "none"},
        {"ball_id": "ball-b", "status": "covered", "reason_code": "none"},
        {"ball_id": "ball-c", "status": "deferred", "reason_code": "route_conflict"},
    ]
    crossings = [
        {
            "active_ball_id": "ball-a",
            "progress_s": 2.0,
            "active_crossing_progress_s": 1.9,
        },
        {
            "active_ball_id": "ball-b",
            "progress_s": 3.0,
            "active_crossing_progress_s": 2.9,
        },
    ]

    outcomes = ControllerNode._build_collect_route_execution_outcomes(
        node, planner_results, crossings
    )

    assert [item["execution_status"] for item in outcomes] == [
        "confirmed",
        "crossed_unconfirmed",
        "deferred",
    ]
    assert planner_results[0] == {
        "ball_id": "ball-a",
        "status": "covered",
        "reason_code": "none",
    }


def test_incomplete_summary_counts_unresolved_targets_as_remaining(monkeypatch):
    node = _node(monkeypatch)
    plan_data = {
        "plan_id": "partial-plan",
        "planning_status": "partial",
        "ball_results": [
            {"ball_id": "ball-a", "status": "covered", "reason_code": "selected"},
            {"ball_id": "ball-b", "status": "deferred", "reason_code": "route_conflict"},
            {"ball_id": "ball-c", "status": "unreachable", "reason_code": "turn_radius"},
        ],
        "segments": [],
    }
    node._collect_route_executor = SimpleNamespace(
        state=ExecutorState.INCOMPLETE_TARGETS,
        route_outcome=ExecutorState.ROUTE_COMPLETED,
        plan=SimpleNamespace(to_dict=lambda: plan_data),
    )
    node._collect_route_executor_factory = SimpleNamespace(
        crossing_telemetry=[],
        controller_state=None,
        snapshot_diagnostics={},
    )

    status = ControllerNode._build_collect_route_summary(node)

    assert status["state"] == "incomplete_targets"
    assert status["status"] == "incomplete_targets"
    assert status["planned"] == 1
    assert status["skipped"] == 2
    assert status["unresolved_targets"] == 2
    assert status["remaining"] == 3


def test_pose_drift_is_relative_to_collect_route_baseline(monkeypatch):
    node = _node(monkeypatch)
    node._sim_true_pose = (10.2, -3.0, 1.2)
    node._robot_x, node._robot_y, node._robot_yaw = 2.0, 1.0, 0.2
    node._pose_frame_offset = (8.0, -4.0)
    node._pose_frame_yaw_offset = 1.0

    assert ControllerNode._pose_error_m(node) == 0.2
    assert ControllerNode._pose_yaw_error_rad(node) == 0.0


def test_confirmation_uses_recent_crossing_after_controller_leaves_target(monkeypatch):
    node = _node(monkeypatch)
    node._collect_route_executor_factory = SimpleNamespace(
        controller_state={
            "plan_id": "plan",
            "has_active_crossing": False,
            "active_ball_id": "",
        },
        crossing_telemetry=[
            {
                "plan_id": "plan",
                "active_ball_id": "ball-a",
                "active_segment_id": "pass-1",
                "observed_sim_time_s": 10.5,
                "progress_s": 4.1,
                "active_crossing_progress_s": 4.0,
                "lateral_error_m": 0.02,
            }
        ],
    )

    context = ControllerNode._route_confirmation_context(node, 12.0)

    assert context["association"] == "recent_crossing"
    assert context["plan_id"] == "plan"
    assert context["ball_id"] == "ball-a"
    assert context["segment_id"] == "pass-1"


def test_confirmation_associates_upcoming_crossing_at_physical_intake_lead(monkeypatch):
    node = _node(monkeypatch)
    crossing = SimpleNamespace(ball_id="ball-a", progress_s=5.0)
    # Match the production RouteSegment contract: the identifier field is
    # named ``id`` (not ``segment_id``).
    segment = SimpleNamespace(id="pass-1", planned_crossings=(crossing,))
    node._collect_route_executor = SimpleNamespace(
        plan=SimpleNamespace(segments=(segment,))
    )
    node._collect_route_executor_factory = SimpleNamespace(
        controller_state={
            "plan_id": "plan",
            "progress_s": 4.55,
            "has_active_crossing": False,
            "active_ball_id": "",
        },
        crossing_telemetry=[],
    )

    context = ControllerNode._route_confirmation_context(node, 12.0)

    assert context["association"] == "intake_lead_crossing"
    assert context["plan_id"] == "plan"
    assert context["ball_id"] == "ball-a"
    assert context["segment_id"] == "pass-1"
    assert context["crossing_progress_s"] == 5.0


def test_summary_keeps_completed_run_totals_while_follow_up_has_no_plan(monkeypatch):
    node = _node(monkeypatch)
    node._collect_route_executor = SimpleNamespace(
        state=ExecutorState.NAVIGATING_TO_SCAN_POSE,
        route_outcome=None,
        plan=None,
    )
    node._collect_route_run_history = [
        {
            "plan_id": "first-plan",
            "planned": 8,
            "confirmed": 3,
            "crossed_unconfirmed": 5,
            "skipped": 3,
            "execution_outcomes": [
                {"ball_id": "ball-b", "execution_status": "crossed_unconfirmed"}
            ],
        }
    ]

    status = ControllerNode._build_collect_route_summary(node)

    assert status["plan_id"] is None
    assert status["planned"] == 8
    assert status["confirmed"] == 3
    assert status["crossed_unconfirmed"] == 5
    assert status["missing"] == 5
    assert status["skipped"] == 3
    assert status["failed_ball_ids"] == ["ball-b"]
    assert status["run_history"][0]["plan_id"] == "first-plan"


def test_confirmed_beam_credit_requires_one_new_entry_edge(monkeypatch):
    node = _node(monkeypatch)
    node._entry_beam_previous = False
    node._entry_beam_sequence = 0
    node._last_credited_entry_sequence = 0

    ControllerNode._on_intake_beam(node, SimpleNamespace(data=True))
    ControllerNode._on_intake_beam(node, SimpleNamespace(data=True))

    assert node._entry_beam_sequence == 1
    assert ControllerNode._consume_entry_for_confirmation(node) is True
    assert ControllerNode._consume_entry_for_confirmation(node) is False

    ControllerNode._on_intake_beam(node, SimpleNamespace(data=False))
    ControllerNode._on_intake_beam(node, SimpleNamespace(data=True))

    assert node._entry_beam_sequence == 2
    assert ControllerNode._consume_entry_for_confirmation(node) is True


def test_completed_route_is_snapshotted_before_follow_up_clears_plan(monkeypatch):
    node = _node(monkeypatch)
    plan_data = {
        "plan_id": "plan-1",
        "scan_id": "scan-1",
        "planning_status": "complete",
        "ball_results": [
            {"ball_id": "ball-a", "status": "covered", "reason_code": "none"}
        ],
    }
    node._collect_route_executor = SimpleNamespace(
        plan=SimpleNamespace(to_dict=lambda: plan_data)
    )
    node._collect_route_executor_factory = SimpleNamespace(
        crossing_telemetry=[
            {
                "plan_id": "plan-1",
                "active_ball_id": "ball-a",
                "progress_s": 2.1,
                "active_crossing_progress_s": 2.0,
            }
        ]
    )
    node._collect_route_confirmations = [
        {"plan_id": "plan-1", "ball_id": "ball-a", "association": "active_crossing"}
    ]

    ControllerNode._capture_collect_route_run(node, "route_completed")
    node._collect_route_executor.plan = None

    assert node._collect_route_run_history[0]["plan_id"] == "plan-1"
    assert node._collect_route_run_history[0]["confirmed"] == 1
    assert node._collect_route_run_history[0]["route_outcome"] == "route_completed"


def test_optional_route_audit_persists_exact_snapshot_and_plan(
    monkeypatch, tmp_path
):
    node = _node(monkeypatch)
    monkeypatch.setenv("COLLECTION_ROUTE_AUDIT_DIR", str(tmp_path))
    snapshot_data = {
        "scan_id": "scan/audit:1",
        "balls": [{"ball_id": "ball-a"}],
    }
    plan_data = {
        "plan_id": "plan-1",
        "scan_id": "scan/audit:1",
        "planning_status": "partial",
        "ball_results": [
            {
                "ball_id": "ball-a",
                "status": "deferred",
                "reason_code": "route_conflict",
            }
        ],
    }
    executor = SimpleNamespace(
        snapshot=SimpleNamespace(
            scan_id="scan/audit:1",
            to_dict=lambda: snapshot_data,
        ),
        plan=SimpleNamespace(to_dict=lambda: plan_data),
    )

    ControllerNode._write_collect_route_audit(
        node, executor, "route_completed"
    )

    artifact = json.loads((tmp_path / "scan_audit_1.json").read_text())
    assert artifact == {
        "schema_version": 1,
        "run_id": "run",
        "route_outcome": "route_completed",
        "snapshot": snapshot_data,
        "plan": plan_data,
    }
    assert list(tmp_path.glob("*.tmp")) == []


def test_hands_off_apply_does_not_publish_collector_command(monkeypatch):
    node = _node(monkeypatch)
    node.control_mode = "collect_route"
    command = ConceptACommand(
        state=CollectorState.IDLE,
        base=BaseCommand(0.0, 0.0),
        collector=CollectorCommand(0.0, False),
    )

    ControllerNode._apply_command(node, command)

    assert node._pub_motion_cmd.messages == []
    assert node._pub_collector.messages == []


def test_collection_map_surfaces_live_snapshot_tracks(monkeypatch):
    node = _node(monkeypatch)
    node._start_collection_route_executor()
    node.control_mode = "collect_route"
    node.active_mapped_target_id = None
    node.ball_map = SimpleNamespace(
        config=SimpleNamespace(supervised_fov_rad=1.204, supervised_max_range_m=6.765),
        to_console_balls=lambda *args, **kwargs: [],
    )

    payload = ControllerNode._build_map_payload(node)

    assert [(ball["x_m"], ball["y_m"], ball["confirmed"]) for ball in payload["balls"]] == [
        (3.0, 1.0, False),
        (4.0, 2.0, True),
    ]
    assert payload["metrics"]["balls_mapped"] == 2
    assert payload["metrics"]["balls_confirmed"] == 1
