"""Phase 6D.3: fake-ROS proof of node-side handle construction."""

from __future__ import annotations

from dataclasses import fields
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration
from tennis_robot.collection_execution_context_builder import ControllerTuning, build_execution_context
from tennis_robot.collection_executor_node_factory import (
    CollectionExecutorNodeCache, CollectionExecutorNodeFactory,
    CollectionExecutorNodeFactoryError, CollectionExecutorRosTypes,
    _planner_audit_sink_from_env,
    scan_pose_from_court_model,
)
from tennis_robot.collection_route_planner_v2 import plan_collection_route
from tennis_robot.collection_route_types import (
    Point2D, Pose2D, PositionCovariance2D, ScanSnapshot, SnapshotBall,
)

ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT / "calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json"
ROUTE_CONFIG = ROOT / "ros2_ws/src/tennis_robot/config/collection_route.yaml"
BOUNDARY = {
    "schema": "court_knowledge_model/v2", "status": "OK", "failure_reason": None,
    "frame": "map", "completed": True,
    "net": {"center": {"x_m": 10.0, "y_m": 2.0},
            "axis_length": {"x_m": 0.0, "y_m": 1.0},
            "axis_width": {"x_m": -1.0, "y_m": 0.0},
            "posts": [{"x_m": 4.0, "y_m": 2.0}, {"x_m": 16.0, "y_m": 2.0}]},
    "court": {"lines_court_frame": {"service_x": [-6.4, 6.4],
                                      "center_line_y": 0.0}},
    "fence": {"corners": [{"x_m": 0.0, "y_m": -8.0}, {"x_m": 20.0, "y_m": -8.0},
                            {"x_m": 20.0, "y_m": 12.0}, {"x_m": 0.0, "y_m": 12.0}]},
    "obstacles": [],
}


class Future:
    def __init__(self, result): self._result = result
    def done(self): return True
    def result(self): return self._result


class Client:
    def __init__(self): self.requests = []
    def call_async(self, request):
        self.requests.append(request)
        return Future(SimpleNamespace(accepted=True))


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class ClockNow:
    nanoseconds = 1_000_000_000
    def to_msg(self): return SimpleNamespace(sec=1, nanosec=0)


class Node:
    def __init__(self):
        self.params = {
            "collection_controller_tuning.lookahead_distance_m": 1.0,
            "collection_controller_tuning.max_angular_velocity_rad_s": 3.0,
            "collection_controller_tuning.progress_projection_window_m": 10.0,
            "collection_controller_tuning.crossing_speed_window_m": 0.25,
            "collection_controller_tuning.terminal_progress_tolerance_m": 0.05,
            "collection_route.context_schema_version": "collection-execution-context/v1",
            "collection_route.context_activation_timeout_s": 10.0,
            "collection_route.scan_step_count": 4,
            "collection_route.scan_yaw_tolerance_rad": 0.1,
            "collection_route.scan_start_yaw_rad": 0.0,
            "collection_route.scan_angular_speed_rad_s": 0.5,
            "collection_route.scan_timeout_s": 20.0,
            "collection_route.safety_forward_half_angle_rad": 0.35,
            "collection_route.safety_stop_distance_m": 0.6,
            "collection_route.safety_pause_timeout_s": 10.0,
            "collection_route.safety_max_scan_age_s": 0.5,
            "collection_route.controller_id": "CollectionFollowPath",
            "collection_route.goal_checker_id": "collection_goal_checker",
            "collection_route.drive_viewpoint_spacing_m": 0.75,
            "collection_route.drive_known_merge_radius_m": 0.50,
        }
        self.clients, self.publishers, self.subscriptions = [], [], []
    def get_parameter(self, name):
        if name not in self.params: raise KeyError(name)
        return SimpleNamespace(value=self.params[name])
    def get_clock(self): return SimpleNamespace(now=lambda: ClockNow())
    def create_client(self, service_type, topic):
        client = Client(); self.clients.append((topic, client)); return client
    def create_publisher(self, message_type, topic, qos):
        publisher = Publisher(); self.publishers.append((topic, publisher)); return publisher
    def create_subscription(self, message_type, topic, callback, qos):
        value = SimpleNamespace(topic=topic, callback=callback); self.subscriptions.append(value); return value


def nested(): return SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
class Pose:
    def __init__(self): self.position, self.orientation = nested(), nested()
class Header:
    def __init__(self): self.frame_id, self.stamp = "", None
class PoseStamped:
    def __init__(self): self.header, self.pose = Header(), Pose()
class PathMsg:
    def __init__(self): self.header, self.poses = Header(), []
class Twist:
    def __init__(self): self.linear, self.angular = nested(), nested()
class Profile:
    pass
class Crossing:
    pass
class Segment:
    def __init__(self): self.execution_profile, self.planned_crossings = Profile(), []
class Tuning:
    pass
class Context:
    def __init__(self): self.controller_tuning, self.segments = Tuning(), []
class State:
    pass
class Service:
    class Request:
        pass
class Load(Service):
    class Request:
        def __init__(self): self.context = None
class FollowPath:
    class Goal:
        def __init__(self):
            self.path, self.controller_id, self.goal_checker_id = None, "", ""
class GoalStatus:
    STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 4, 5, 6
class ActionClient:
    def __init__(self, node, action_type, topic): self.topic, self.goals = topic, []
    def send_goal_async(self, goal):
        self.goals.append(goal)
        return Future(SimpleNamespace(accepted=False))


ROS = CollectionExecutorRosTypes(
    Twist, Pose, PoseStamped, PathMsg, FollowPath, ActionClient, GoalStatus,
    Context, Segment, Profile, Crossing, State, Load, Service, Service, Service,
    lambda seconds: seconds, lambda node, future, timeout_sec: None,
)


class Lane:
    state = "idle"
    def request(self, *args): pass
class Collector:
    def start(self): pass
    def stop(self): pass
class Tf:
    def lookup_transform(self, target, source, stamp):
        return SimpleNamespace(transform=SimpleNamespace(translation=nested(), rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))


@pytest.fixture
def factory(tmp_path):
    boundary_path = tmp_path / "court_boundary.json"
    boundary_path.write_text(json.dumps(BOUNDARY))
    node = Node()
    cache = CollectionExecutorNodeCache(latest_scan="scan", latest_ball_detections="frame",
                                        robot_x_m=10.0, robot_y_m=-5.0, robot_yaw_rad=0.2)
    built = CollectionExecutorNodeFactory(
        node=node, tf_buffer=Tf(), cache=cache, lane_navigator=Lane(),
        collector_interface=Collector(), court_boundary_path=boundary_path,
        collection_route_config_path=ROUTE_CONFIG,
        calibration_artifact_path=CALIBRATION, telemetry_sink=lambda event: None,
        ros_types=ROS,
    )
    return built, node, cache


def curved_plan():
    configuration = default_configuration()
    snapshot = ScanSnapshot(
        "scan", 1.0, "map", Pose2D(10.0, -5.0, 0.0),
        (SnapshotBall("ball", Point2D(13.0, -5.0), 0.95,
                      PositionCovariance2D(1e-6, 0.0, 1e-6)),), configuration,
    )
    from tennis_robot.collection_court_model_builder import build_court_model
    plan = plan_collection_route(snapshot=snapshot, court=build_court_model(BOUNDARY),
                                 configuration=configuration).plan
    assert plan.is_executable
    return plan


def test_factory_constructs_every_assembly_handle_with_live_cache_shapes(factory):
    built, node, cache = factory
    assert built.build() is not None
    optional_names = {
        "entry_beam_provider",
        "confirmed_beam_provider",
        "planner_audit_sink",
    }
    assert all(
        getattr(built.handles, field.name) is not None
        for field in fields(built.handles)
        if field.name not in optional_names
    )
    callable_names = {"telemetry_sink", "scan_provider", "yaw_provider", "frame_provider", "cmd_vel",
                      "load_sender", "load_outcome_provider", "follow_path_sender",
                      "goal_status_provider", "state_provider", "hold_sender", "finalize_sender",
                      "execution_plan_transformer"}
    assert all(callable(getattr(built.handles, name)) for name in callable_names)
    assert built.handles.scan_provider() == "scan"
    assert built.handles.yaw_provider() == 0.2
    assert built.handles.frame_provider() == "frame"
    cache.latest_scan = "new-scan"
    assert built.handles.scan_provider() == "new-scan"
    built.handles.cmd_vel(0.75)
    assert node.publishers[0][0] == "/cmd_vel_collection"
    assert node.publishers[0][1].messages[-1].angular.z == 0.75
    assert {topic for topic, _ in node.clients} == {
        "/CollectionFollowPath/load_collection_execution_context",
        "/CollectionFollowPath/reset_collection_execution_context",
        "/CollectionFollowPath/set_collection_safety_hold",
        "/CollectionFollowPath/finalize_collection_execution_context",
    }
    assert node.subscriptions[0].topic == "/CollectionFollowPath/state"
    session = built.handles.scan_snapshot_session
    assert callable(session.forward_frame) and callable(session.finalize)
    assert session.builder.robot_pose_at_scan == Pose2D(*built.config.scan_pose_xy_yaw)
    source_plan = curved_plan()
    execution_plan = built.handles.execution_plan_transformer(source_plan)
    assert execution_plan.map_frame == "odom"
    assert execution_plan.start_pose == source_plan.start_pose
    diagnostics = built.execution_frame_diagnostics
    assert diagnostics["plan_id"] == source_plan.plan_id
    assert diagnostics["source_frame"] == "map"
    assert diagnostics["target_frame"] == "odom"
    assert diagnostics["source_crossings"]
    assert len(diagnostics["source_crossings"]) == len(
        diagnostics["execution_crossings"]
    )
    cache.robot_x_m, cache.robot_y_m, cache.robot_yaw_rad = 11.0, -4.0, -0.3
    assert session._robot_pose_provider() == Pose2D(11.0, -4.0, -0.3)

    values = build_execution_context(
        curved_plan(), controller_tuning=ControllerTuning(1.0, 3.0, 10.0, 0.25, 0.05),
        context_schema_version="collection-execution-context/v1",
        context_activation_timeout_s=10.0,
    )
    built.handles.follow_path_sender(
        map_frame=values.map_frame, poses=values.follow_path_poses,
        controller_id="CollectionFollowPath",
    )
    goal = built.transport.action_client.goals[-1]
    assert goal.controller_id == "CollectionFollowPath"
    assert goal.goal_checker_id == "collection_goal_checker"
    assert goal.path.header.frame_id == "map"
    assert len(goal.path.poses) == len(values.follow_path_poses)
    assert built.handles.goal_status_provider() == "rejected"
    built.handles.hold_sender(plan_id="plan", path_sha256="sha", hold=True)
    assert vars(node.clients[2][1].requests[-1]) == {
        "plan_id": "plan", "path_sha256": "sha", "hold": True,
    }
    assert built.handles.finalize_sender(
        plan_id="plan", path_sha256="sha", action_outcome=0
    ) is True
    assert vars(node.clients[3][1].requests[-1]) == {
        "plan_id": "plan", "path_sha256": "sha", "action_outcome": 0,
    }
    profile_verdict = SimpleNamespace(
        hard_compliant=True,
        hard_violation_reason=0,
        nominal_tracking=True,
        measured_speed_mps=0.8,
        nominal_speed_error_mps=0.0,
    )
    state_values = {
        "plan_id": "plan", "path_sha256": "sha", "lifecycle_state": 3,
        "progress_s": 1.0, "active_segment_id": "segment",
        "has_active_crossing": True, "active_ball_id": "ball",
        "active_crossing_progress_s": 1.1, "measured_speed_mps": 0.8,
        "lateral_error_m": 0.01, "heading_error_rad": 0.02,
        "profile_verdict": profile_verdict, "failure_reason": 0,
        # Terminal diagnosis published for terminal_not_reached triage.
        "terminal_progress_s": 4.0, "terminal_distance_m": 0.12,
        "terminal_ready": True,
    }
    node.subscriptions[0].callback(SimpleNamespace(**state_values))
    assert built.handles.state_provider() == state_values
    assert built.controller_state == {
        key: state_values[key]
        for key in (
            "plan_id", "lifecycle_state", "progress_s", "active_segment_id",
            "has_active_crossing", "active_ball_id", "active_crossing_progress_s",
            "measured_speed_mps", "lateral_error_m", "heading_error_rad",
            "failure_reason",
        )
    }
    assert built.crossing_telemetry == [{
        key: state_values[key]
        for key in (
            "plan_id", "progress_s", "active_segment_id", "active_ball_id",
            "active_crossing_progress_s", "measured_speed_mps",
            "lateral_error_m", "heading_error_rad",
        )
    } | {
        "observed_sim_time_s": 1.0,
        "profile_verdict": vars(profile_verdict),
    }]


def test_load_sender_fills_real_context_message_field_for_field(factory):
    built, node, _ = factory
    values = build_execution_context(
        curved_plan(), controller_tuning=ControllerTuning(1.0, 3.0, 10.0, 0.25, 0.05),
        context_schema_version="collection-execution-context/v1",
        context_activation_timeout_s=10.0,
    )
    built.handles.load_sender(values)
    assert len(node.clients[1][1].requests) == 1
    assert len(node.clients[0][1].requests) == 0
    assert built.handles.load_outcome_provider() == "accepted"
    message = node.clients[0][1].requests[-1].context
    for name in ("context_schema_version", "plan_id", "path_sha256",
                 "context_activation_timeout_s", "terminal_progress_s",
                 "configuration_snapshot_json"):
        assert getattr(message, name) == getattr(values, name)
    assert vars(message.terminal_pose.position) == {
        "x": values.terminal_pose.x, "y": values.terminal_pose.y,
        "z": values.terminal_pose.z, "w": 0.0,
    }
    assert vars(message.terminal_pose.orientation) == {
        "x": values.terminal_pose.qx, "y": values.terminal_pose.qy,
        "z": values.terminal_pose.qz, "w": values.terminal_pose.qw,
    }
    assert vars(message.controller_tuning) == vars(SimpleNamespace(**{
        field.name: getattr(values.controller_tuning, field.name)
        for field in fields(values.controller_tuning)
    }))
    assert len(message.segments) == len(values.segments)
    for actual, expected in zip(message.segments, values.segments):
        assert actual.segment_id == expected.segment_id
        assert actual.segment_type == expected.segment_type
        assert actual.segment_type in (0, 1, 2)
        assert actual.progress_start_s == expected.progress_start_s
        assert actual.progress_end_s == expected.progress_end_s
        assert vars(actual.execution_profile) == {
            field.name: getattr(expected.execution_profile, field.name)
            for field in fields(expected.execution_profile)
        }
        assert [vars(item) for item in actual.planned_crossings] == [
            {field.name: getattr(crossing, field.name) for field in fields(crossing)}
            for crossing in expected.planned_crossings
        ]
def test_every_load_resets_controller_context_first(factory):
    built, node, _ = factory
    values = build_execution_context(
        curved_plan(),
        controller_tuning=ControllerTuning(1.0, 3.0, 10.0, 0.25, 0.05),
        context_schema_version="collection-execution-context/v1",
        context_activation_timeout_s=10.0,
    )
    built.handles.load_sender(values)
    assert len(node.clients[1][1].requests) == 1
    assert len(node.clients[0][1].requests) == 0
    assert built.handles.load_outcome_provider() == "accepted"
    assert len(node.clients[0][1].requests) == 1

    built.handles.load_sender(values)
    assert len(node.clients[1][1].requests) == 2
    assert len(node.clients[0][1].requests) == 1
    assert built.handles.load_outcome_provider() == "accepted"
    assert len(node.clients[0][1].requests) == 2


def test_scan_pose_is_service_line_center_on_robot_side_with_survey_axes():
    assert scan_pose_from_court_model(BOUNDARY, robot_pose=Pose2D(10.0, -5.0, 0.0)) == pytest.approx(
        (10.0, -4.4, -math.pi / 2.0)
    )
    assert scan_pose_from_court_model(BOUNDARY, robot_pose=Pose2D(10.0, 9.0, 0.0)) == pytest.approx(
        (10.0, 8.4, math.pi / 2.0)
    )


def test_scan_pose_yaw_gives_planner_a_start_edge_into_robot_side_half():
    from tennis_robot.collection_court_model_builder import build_court_model

    configuration = default_configuration()
    scan_pose = Pose2D(*scan_pose_from_court_model(
        BOUNDARY, robot_pose=Pose2D(10.0, -5.0, 0.0)
    ))
    snapshot = ScanSnapshot(
        "scan-facing-robot-half",
        1.0,
        "map",
        scan_pose,
        (
            SnapshotBall(
                "ball",
                Point2D(10.0, -6.3),
                0.95,
                PositionCovariance2D(1e-6, 0.0, 1e-6),
            ),
        ),
        configuration,
    )

    plan = plan_collection_route(
        snapshot=snapshot,
        court=build_court_model(BOUNDARY),
        configuration=configuration,
    ).plan

    assert plan.is_executable
    assert plan.ball_results[0].status.value == "covered"


def test_pre_execution_audit_sink_is_opt_in_and_atomic(monkeypatch, tmp_path):
    messages = []
    node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=messages.append,
            error=messages.append,
        )
    )
    monkeypatch.delenv("COLLECTION_ROUTE_AUDIT_DIR", raising=False)
    assert _planner_audit_sink_from_env(node) is None

    monkeypatch.setenv("COLLECTION_ROUTE_AUDIT_DIR", str(tmp_path))
    sink = _planner_audit_sink_from_env(node)
    snapshot = SimpleNamespace(
        scan_id="scan/audit:1",
        to_dict=lambda: {"scan_id": "scan/audit:1", "balls": []},
    )
    plan = SimpleNamespace(
        to_dict=lambda: {
            "plan_id": "plan-1",
            "planning_status": "partial",
        }
    )

    sink(snapshot, plan)

    artifact = json.loads((tmp_path / "scan_audit_1.json").read_text())
    assert artifact["route_outcome"] is None
    assert artifact["snapshot"]["scan_id"] == "scan/audit:1"
    assert artifact["plan"]["plan_id"] == "plan-1"
    assert list(tmp_path.glob("*.tmp")) == []
    assert messages and "pre-execution audit saved" in messages[-1]


def test_missing_required_runtime_parameter_fails_loud(tmp_path):
    boundary_path = tmp_path / "court_boundary.json"
    boundary_path.write_text(json.dumps(BOUNDARY))
    node = Node(); del node.params["collection_route.safety_max_scan_age_s"]
    with pytest.raises(CollectionExecutorNodeFactoryError, match="safety_max_scan_age_s"):
        CollectionExecutorNodeFactory(
            node=node, tf_buffer=Tf(),
            cache=CollectionExecutorNodeCache(robot_x_m=10., robot_y_m=-5., robot_yaw_rad=0.),
            lane_navigator=Lane(), collector_interface=Collector(),
            court_boundary_path=boundary_path, collection_route_config_path=ROUTE_CONFIG,
            calibration_artifact_path=CALIBRATION, telemetry_sink=lambda event: None,
            ros_types=ROS,
        )
