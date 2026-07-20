"""Pure-domain contract tests for continuous collection routes."""

from dataclasses import FrozenInstanceError
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_route_types import (
    BallReasonCode,
    BallResult,
    BallStatus,
    CollectionRouteConfiguration,
    ConnectorConfiguration,
    CollectionRoutePlan,
    DomainValidationError,
    ExecutionProfile,
    FeasibilityConfiguration,
    FollowUpConfiguration,
    GazeboSnapshotConfiguration,
    GlobalRouteSearchConfiguration,
    LocalizationXYCovariance,
    MechanicalConfiguration,
    ObstacleConstraint,
    ObstacleConstraintKind,
    Path2D,
    PathPoint,
    PlanningConfiguration,
    PlanningSearchStatus,
    PlanningStatus,
    PlannedCrossing,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    RouteSegment,
    RouteSegmentType,
    SafetyConfiguration,
    ScanConfiguration,
    ScanSnapshot,
    SharedPassConfiguration,
    SnapshotBall,
    SnapshotAssociationConfiguration,
    UncertaintyConfiguration,
)


def profile() -> ExecutionProfile:
    return ExecutionProfile(1.0, 0.5, 1.5, 0.1, 1.0, 1.0, 0.2, 1.0, 0.3, 1.0, 0.1, 0.1, False, False)


def configuration() -> CollectionRouteConfiguration:
    return CollectionRouteConfiguration(
        "collection-route/v1",
        MechanicalConfiguration(0.17, 0.033, 0.8, 0.6, 0.4, 0.34, 0.05, 0.8, 1.25, 0.2, 1.0, 0.3, 0.2),
        UncertaintyConfiguration(0.02, 0.03, 0.04),
        SafetyConfiguration(0.1, 0.15, 0.2, 0.5, 0.2, 2.0, 2.0, 10.0),
        ScanConfiguration(1.0, 20.0, 2),
        FeasibilityConfiguration(16, 2.0, 0.04, 0.05, 0.50, 0.75, 0.20),
        ConnectorConfiguration(20.0, 1.5, 3.0),
        GlobalRouteSearchConfiguration(1000, 0.5, 1.0, 0.8, 0.2, 1.0, 1.0, 1.0, 1.0, 1.0),
        SharedPassConfiguration(3, 100, 0.5),
        FollowUpConfiguration(False, 1),
        PlanningConfiguration(profile(), 1.0, 20, 1.0, 1.0, 1.0, 1.0, 1.0),
        GazeboSnapshotConfiguration(
            "gazebo-mvp-provisional-planning-safety-v1",
            LocalizationXYCovariance(PositionCovariance2D(0.01, 0.0, 0.01)),
            SnapshotAssociationConfiguration(9.0, 2, 2),
        ),
    )


def snapshot(ball_ids: tuple[str, ...] = ("ball_1", "ball_2")) -> ScanSnapshot:
    balls = tuple(SnapshotBall(ball_id, Point2D(float(index), 1.0), 0.9, PositionCovariance2D(0.01, 0.0, 0.01)) for index, ball_id in enumerate(ball_ids))
    return ScanSnapshot("scan_1", 100.0, "map", Pose2D(0.0, 0.0, 0.0), balls, configuration())


def path(start_x: float, end_x: float) -> Path2D:
    return Path2D((PathPoint(Pose2D(start_x, 0.0, 0.0)), PathPoint(Pose2D(end_x, 0.0, 0.0))))


def segment(segment_id: str, kind: RouteSegmentType, start: float, end: float, balls: tuple[str, ...] = ()) -> RouteSegment:
    crossings = tuple(
        PlannedCrossing(ball_id, Point2D(start + (index + 1) * (end - start) / (len(balls) + 1), 0.0), start + (index + 1) * (end - start) / (len(balls) + 1), 0.0, 0.0)
        for index, ball_id in enumerate(balls)
    )
    return RouteSegment(segment_id, kind, path(start, end), start, end, profile(), balls, ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0), crossings)


def feasible_plan() -> CollectionRoutePlan:
    return CollectionRoutePlan(
        "plan_1", "scan_1", "map", Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), 3.0, 3.0,
        PlanningStatus.FEASIBLE, PlanningSearchStatus.COMPLETE,
        (segment("pass", RouteSegmentType.FUNNEL_PASS, 0.0, 2.0, ("ball_1", "ball_2")), segment("terminal", RouteSegmentType.TERMINAL_CONNECTOR, 2.0, 3.0)),
        ("ball_1", "ball_2"),
        (BallResult("ball_1", BallStatus.COVERED, BallReasonCode.SELECTED, "pass", 0.0), BallResult("ball_2", BallStatus.COVERED, BallReasonCode.SELECTED, "pass", 0.0)),
        configuration(),
    )


def test_round_trip_preserves_complete_plan_and_snapshot_validation():
    plan = feasible_plan()
    restored = CollectionRoutePlan.from_dict(plan.to_dict())
    assert restored == plan
    assert restored.is_executable
    restored.validate_against(snapshot())


def test_invalid_enum_and_unknown_serialized_field_are_rejected():
    with pytest.raises(DomainValidationError):
        BallResult.from_dict({"ball_id": "ball_1", "status": "invalid", "reason_code": "selected", "pass_id": None, "predicted_lateral_error_m": None})
    encoded = feasible_plan().to_dict()
    encoded["unexpected"] = True
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan.from_dict(encoded)


def test_models_are_immutable_after_construction():
    with pytest.raises(FrozenInstanceError):
        feasible_plan().segments[0].progress_end_m = 4.0
    with pytest.raises(FrozenInstanceError):
        snapshot().balls[0].confidence = 0.1


def test_missing_or_duplicate_ball_result_is_rejected():
    plan = feasible_plan()
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            plan.plan_id, plan.scan_id, plan.map_frame, plan.start_pose, plan.terminal_pose, plan.total_length_m, plan.expected_duration_s,
            plan.planning_status, plan.planning_search_status, plan.segments, plan.snapshot_ball_ids, plan.ball_results[:1], plan.configuration_snapshot,
        )
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            plan.plan_id, plan.scan_id, plan.map_frame, plan.start_pose, plan.terminal_pose, plan.total_length_m, plan.expected_duration_s,
            plan.planning_status, plan.planning_search_status, plan.segments, plan.snapshot_ball_ids, (plan.ball_results[0], plan.ball_results[0]), plan.configuration_snapshot,
        )


def test_empty_and_partial_plans_have_required_status_shapes():
    empty_snapshot = snapshot(())
    empty = CollectionRoutePlan("empty", "scan_1", "map", empty_snapshot.robot_pose_at_scan, empty_snapshot.robot_pose_at_scan, 0.0, 0.0, PlanningStatus.EMPTY_NO_BALLS, PlanningSearchStatus.COMPLETE, (), (), (), configuration())
    assert not empty.is_executable

    partial = CollectionRoutePlan(
        "partial", "scan_1", "map", Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), 3.0, 3.0, PlanningStatus.PARTIAL, PlanningSearchStatus.COMPLETE,
        (segment("pass", RouteSegmentType.FUNNEL_PASS, 0.0, 2.0, ("ball_1",)), segment("terminal", RouteSegmentType.TERMINAL_CONNECTOR, 2.0, 3.0)),
        ("ball_1", "ball_2"),
        (BallResult("ball_1", BallStatus.COVERED, BallReasonCode.SELECTED, "pass"), BallResult("ball_2", BallStatus.DEFERRED, BallReasonCode.ROUTE_CONFLICT)),
        configuration(),
    )
    assert partial.is_executable
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan("bad", "scan_1", "map", empty.start_pose, empty.terminal_pose, 0.0, 0.0, PlanningStatus.EMPTY_NO_BALLS, PlanningSearchStatus.COMPLETE, (), ("ball_1",), (), configuration())


def test_geometry_profile_and_segment_contracts_are_strict():
    assert Pose2D(0.0, 0.0, math.pi).yaw_rad == -math.pi
    with pytest.raises(DomainValidationError):
        PositionCovariance2D(1.0, 2.0, 1.0)
    with pytest.raises(DomainValidationError):
        PositionCovariance2D(1e-4, 1e-3, 1e-4)
    with pytest.raises(DomainValidationError):
        ExecutionProfile(1.0, 1.1, 1.5, 0.1, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.1, False, False)
    with pytest.raises(DomainValidationError):
        segment("connector", RouteSegmentType.CONNECTOR, 0.0, 1.0, ("ball_1",))


def test_plan_requires_contiguous_progress_terminal_and_matching_snapshot():
    plan = feasible_plan()
    broken_terminal = RouteSegment("terminal", RouteSegmentType.TERMINAL_CONNECTOR, path(2.1, 3.0), 2.1, 3.0, profile(), (), ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0))
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(plan.plan_id, plan.scan_id, plan.map_frame, plan.start_pose, plan.terminal_pose, plan.total_length_m, plan.expected_duration_s, plan.planning_status, plan.planning_search_status, (plan.segments[0], broken_terminal), plan.snapshot_ball_ids, plan.ball_results, plan.configuration_snapshot)
    with pytest.raises(DomainValidationError):
        plan.validate_against(snapshot(("ball_1",)))


def test_covered_result_requires_its_matching_funnel_pass():
    plan = feasible_plan()
    mismatched = (
        BallResult("ball_1", BallStatus.COVERED, BallReasonCode.SELECTED, "not_the_pass"),
        plan.ball_results[1],
    )
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            plan.plan_id, plan.scan_id, plan.map_frame, plan.start_pose, plan.terminal_pose,
            plan.total_length_m, plan.expected_duration_s, plan.planning_status, plan.planning_search_status, plan.segments,
            plan.snapshot_ball_ids, mismatched, plan.configuration_snapshot,
        )


def test_non_executable_artifacts_cannot_describe_route_geometry():
    start = Pose2D(0.0, 0.0, 0.0)
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            "timeout", "scan_1", "map", start, Pose2D(1.0, 0.0, 0.0), 1.0, 1.0,
            PlanningStatus.PLANNING_TIMEOUT, PlanningSearchStatus.BUDGET_EXHAUSTED, (), ("ball_1",),
            (BallResult("ball_1", BallStatus.DEFERRED, BallReasonCode.PLANNING_BUDGET),),
            configuration(),
        )
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            "empty", "scan_empty", "map", start, start, 42.0, 1.0,
            PlanningStatus.EMPTY_NO_BALLS, PlanningSearchStatus.COMPLETE, (), (), (), configuration(),
        )
    with pytest.raises(DomainValidationError):
        CollectionRoutePlan(
            "no-feasible", "scan_unreachable", "map", start, start, 42.0, 1.0,
            PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS, PlanningSearchStatus.COMPLETE, (), ("ball_1",),
            (BallResult("ball_1", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),),
            configuration(),
        )
