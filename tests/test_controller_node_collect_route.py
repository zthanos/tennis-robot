"""Phase 6D.4 node wiring: collect_route is executor-owned and hands-off."""

from __future__ import annotations

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
from tennis_robot.collection_route_executor import ExecutorState
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
        _robot_x=1.0,
        _robot_y=2.0,
        _robot_yaw=0.3,
        _pub_collector=Publisher(),
        _pub_motion_cmd=Publisher(),
        _collect_route_executor=None,
        _collect_route_executor_factory=None,
        _collect_route_executor_events=[],
        _collect_route_executor_complete_reported=False,
        _last_collect_route_summary={},
        _run_id="run",
        ball_map=SimpleNamespace(reset=lambda: None),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _runtime_seconds=lambda: 12.0,
        _declare_collection_route_parameters=lambda: None,
    )
    node._on_collection_executor_telemetry = MethodType(
        ControllerNode._on_collection_executor_telemetry, node
    )
    node._start_collection_route_executor = MethodType(
        ControllerNode._start_collection_route_executor, node
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
    assert status["plan_id"] == "empty-plan"
    assert status["planning_status"] == "empty_no_balls"
    assert status["ball_results"] == []
    assert status["segments"] == []
    assert status["crossings"] == []
    assert status["executed_crossing_telemetry"] == [{"active_ball_id": "ball-1"}]


def test_hands_off_apply_does_not_publish_collector_command(monkeypatch):
    node = _node(monkeypatch)
    node.control_mode = "collect_route"
    command = ConceptACommand(
        state=CollectorState.IDLE,
        base=BaseCommand(0.0, 0.0),
        collector=CollectorCommand(0.0, False),
    )

    ControllerNode._apply_command(node, command)

    assert len(node._pub_motion_cmd.messages) == 1
    assert node._pub_motion_cmd.messages[0].linear.x == 0.0
    assert node._pub_motion_cmd.messages[0].angular.z == 0.0
    assert node._pub_collector.messages == []
