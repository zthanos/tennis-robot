"""Phase 6B Part 2/3: pure CollectionRoutePlan -> execution-context serializer.

Uses a real executable plan from ``plan_collection_route`` (no hand-built
plan), then asserts the serialized field values mirror the plan and the
C++ wire contract.  Cross-language acceptance of the resulting context is
proven separately by the container parity harness.
"""

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
from tennis_robot.collection_execution_context_builder import (
    CONTEXT_SCHEMA_VERSION,
    ControllerTuning,
    ExecutionContextBuildError,
    build_execution_context,
    build_follow_path_poses,
    canonical_configuration_snapshot_json,
)
from tennis_robot.collection_path_canonicalization import collection_path_sha256_v1
from tennis_robot.collection_route_planner_v2 import CourtModel, plan_collection_route
from tennis_robot.collection_route_types import (
    Point2D,
    Pose2D,
    PositionCovariance2D,
    RouteSegmentType,
    ScanSnapshot,
    SnapshotBall,
)

_PROFILE_FIELDS = (
    "nominal_speed_mps", "min_speed_mps", "max_speed_mps", "nominal_speed_warning_tolerance_mps",
    "max_acceleration_mps2", "max_deceleration_mps2", "required_entry_m", "required_run_in_m",
    "required_run_out_m", "max_curvature_per_m", "max_lateral_error_m", "max_heading_error_rad",
    "allow_reversing", "allow_standalone_rotate",
)


def _court(extent=20.0):
    poly = tuple(Point2D(float(x), float(y)) for x, y in ((-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent)))
    return CourtModel(poly, ())


def _snapshot(config, *entries):
    return ScanSnapshot(
        "scan-6b", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)) for ball_id, x, y in entries),
        config,
    )


def _plan(*entries):
    config = default_configuration()
    if not entries:
        entries = (("a", 3.0, 0.0),)
    return plan_collection_route(snapshot=_snapshot(config, *entries), court=_court(), configuration=config).plan


def _tuning():
    return ControllerTuning(1.0, 2.0, 5.0, 0.25, 0.05)


def _context(plan=None, tuning=None):
    return build_execution_context(
        plan if plan is not None else _plan(),
        controller_tuning=tuning if tuning is not None else _tuning(),
        context_schema_version=CONTEXT_SCHEMA_VERSION,
        context_activation_timeout_s=5.0,
    )


def test_context_identity_and_terminal_fields_mirror_the_plan():
    plan = _plan()
    context = _context(plan)
    assert context.context_schema_version == CONTEXT_SCHEMA_VERSION
    assert context.plan_id == plan.plan_id
    assert context.map_frame == plan.map_frame
    assert context.context_activation_timeout_s == 5.0
    assert context.terminal_progress_s == plan.total_length_m
    # terminal pose: yaw -> quaternion, z=0.
    assert context.terminal_pose.x == plan.terminal_pose.x_m
    assert context.terminal_pose.y == plan.terminal_pose.y_m
    assert context.terminal_pose.z == 0.0
    assert context.terminal_pose.qz == pytest.approx(math.sin(plan.terminal_pose.yaw_rad / 2.0))
    assert context.terminal_pose.qw == pytest.approx(math.cos(plan.terminal_pose.yaw_rad / 2.0))


def test_segments_and_profiles_and_crossings_map_one_to_one():
    plan = _plan()
    context = _context(plan)
    assert len(context.segments) == len(plan.segments)
    type_code = {RouteSegmentType.CONNECTOR: 0, RouteSegmentType.FUNNEL_PASS: 1, RouteSegmentType.TERMINAL_CONNECTOR: 2}
    for wire, segment in zip(context.segments, plan.segments):
        assert wire.segment_id == segment.id
        assert wire.segment_type == type_code[segment.type]
        assert wire.progress_start_s == segment.progress_start_m
        assert wire.progress_end_s == segment.progress_end_m
        for name in _PROFILE_FIELDS:
            assert getattr(wire.execution_profile, name) == getattr(segment.execution_profile, name)
        assert len(wire.planned_crossings) == len(segment.planned_crossings)
        for wire_crossing, crossing in zip(wire.planned_crossings, segment.planned_crossings):
            assert wire_crossing.ball_id == crossing.ball_id
            assert wire_crossing.position_x_m == crossing.position_xy.x_m
            assert wire_crossing.position_y_m == crossing.position_xy.y_m
            assert wire_crossing.progress_s == crossing.progress_s
            assert wire_crossing.heading_rad == crossing.heading_rad
            assert wire_crossing.predicted_lateral_error == crossing.predicted_lateral_error


def test_funnel_pass_crossings_are_present_and_ordered():
    # The two-ball case exercises a real funnel pass with a planned crossing.
    plan = _plan(("a", 3.0, 0.0))
    context = _context(plan)
    passes = [segment for segment in context.segments if segment.segment_type == 1]
    assert passes
    for wire_pass in passes:
        assert wire_pass.planned_crossings
        progresses = [crossing.progress_s for crossing in wire_pass.planned_crossings]
        assert progresses == sorted(progresses)
        assert all(wire_pass.progress_start_s < progress < wire_pass.progress_end_s for progress in progresses)


def test_path_sha256_matches_the_follow_path_poses_it_carries():
    plan = _plan()
    context = _context(plan)
    assert context.path_sha256 == collection_path_sha256_v1(context.map_frame, context.follow_path_poses)


def test_follow_path_poses_are_tracking_plan_valid():
    plan = _plan()
    context = _context(plan)
    poses = context.follow_path_poses
    assert len(poses) >= 2
    # Every consecutive 2D step is strictly positive (no duplicate join poses).
    length = 0.0
    for previous, current in zip(poses, poses[1:]):
        step = math.hypot(current.x - previous.x, current.y - previous.y)
        assert step > 0.0
        length += step
    # Accumulated polyline length equals terminal progress within tuning tolerance.
    assert abs(length - context.terminal_progress_s) <= context.controller_tuning.terminal_progress_tolerance_m
    # Last pose equals the terminal pose within the same tolerance.
    last = poses[-1]
    assert math.hypot(last.x - context.terminal_pose.x, last.y - context.terminal_pose.y) <= context.controller_tuning.terminal_progress_tolerance_m


def test_join_poses_are_deduplicated():
    plan = _plan()
    # Raw concatenation of every segment path point contains the shared join
    # points twice; the flattened list must have strictly fewer poses.
    raw_point_count = sum(len(segment.path.points) for segment in plan.segments)
    poses = build_follow_path_poses(plan)
    assert len(plan.segments) >= 2  # at least one join exists
    assert len(poses) < raw_point_count


def test_configuration_snapshot_json_is_compact_sorted_and_round_trips():
    plan = _plan()
    text = canonical_configuration_snapshot_json(plan)
    # Compact separators: no spaces after ',' or ':'.
    assert ", " not in text and ": " not in text
    # Structurally equal to the source dict.
    assert json.loads(text) == plan.configuration_snapshot.to_dict()
    # Idempotent under a Python parse->dump with the same canonical options
    # (a necessary condition for the C++ nlohmann canonicality check).
    assert json.dumps(json.loads(text), sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False) == text
    # Top-level keys are sorted.
    top_keys = list(json.loads(text).keys())
    assert top_keys == sorted(top_keys)


def test_context_is_deterministic():
    plan = _plan()
    first = _context(plan)
    second = _context(plan)
    assert first == second


def test_controller_tuning_rejects_non_positive():
    with pytest.raises(ExecutionContextBuildError):
        ControllerTuning(0.0, 2.0, 5.0, 0.25, 0.05)
    with pytest.raises(ExecutionContextBuildError):
        ControllerTuning(1.0, 2.0, 5.0, 0.25, -0.05)


def test_invalid_builder_inputs_are_typed_errors():
    plan = _plan()
    with pytest.raises(ExecutionContextBuildError):
        build_execution_context(plan, controller_tuning=_tuning(), context_schema_version="", context_activation_timeout_s=5.0)
    with pytest.raises(ExecutionContextBuildError):
        build_execution_context(plan, controller_tuning=_tuning(), context_schema_version=CONTEXT_SCHEMA_VERSION, context_activation_timeout_s=0.0)
    with pytest.raises(ExecutionContextBuildError):
        build_execution_context(plan, controller_tuning=object(), context_schema_version=CONTEXT_SCHEMA_VERSION, context_activation_timeout_s=5.0)


def test_non_executable_plan_without_segments_is_rejected():
    # An empty snapshot yields an empty (segment-less) plan; the C++ valid_load
    # rejects empty segments, so the builder refuses it up front.
    empty = plan_collection_route(snapshot=_snapshot(default_configuration()), court=_court(), configuration=default_configuration()).plan
    assert empty.segments == ()
    with pytest.raises(ExecutionContextBuildError):
        _context(empty)
