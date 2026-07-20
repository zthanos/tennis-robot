"""Pure Phase 5A flattened follower contract tests."""

from dataclasses import FrozenInstanceError, replace
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration, default_profile
from tennis_robot.collection_path_follower import (
    CollectionPathFollower, FollowerTelemetryCode, NominalTracking,
    ProfileViolationReason,
)
from tennis_robot.collection_route_executor import PathFollowerStatus
from tennis_robot.collection_route_types import (
    BallReasonCode, BallResult, BallStatus, CollectionRoutePlan,
    ObstacleConstraint, ObstacleConstraintKind, Path2D, PathPoint,
    PlannedCrossing, PlanningSearchStatus, PlanningStatus, Point2D, Pose2D,
    RouteSegment, RouteSegmentType, ScanSnapshot, SnapshotBall,
)


def plan(profile=None):
    configuration = default_configuration()
    profile = profile or default_profile()
    crossing = PlannedCrossing("ball", Point2D(1.0, 0.0), 1.0, 0.0, 0.0)
    constraint = ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0)
    passed = RouteSegment(
        "pass", RouteSegmentType.FUNNEL_PASS,
        Path2D((PathPoint(Pose2D(0.0, 0.0, 0.0)), PathPoint(Pose2D(1.0, 0.0, 0.0)), PathPoint(Pose2D(2.0, 0.0, 0.0)))),
        0.0, 2.0, profile, ("ball",), constraint, (crossing,),
    )
    terminal = RouteSegment(
        "terminal", RouteSegmentType.TERMINAL_CONNECTOR,
        Path2D((PathPoint(Pose2D(2.0, 0.0, 0.0)), PathPoint(Pose2D(3.0, 0.0, 0.0)))),
        2.0, 3.0, profile, (), constraint,
    )
    return CollectionRoutePlan(
        "plan-follower", "scan-follower", "map", Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), 3.0, 3.0,
        PlanningStatus.FEASIBLE, PlanningSearchStatus.COMPLETE, (passed, terminal), ("ball",),
        (BallResult("ball", BallStatus.COVERED, BallReasonCode.SELECTED, "pass"),), configuration,
    )


def telemetry(update, code):
    return next(event for event in update.telemetry if event.code is code)


def test_flattened_view_and_monotonic_progress():
    follower = CollectionPathFollower(plan())
    assert follower.view.crossings[0].ball_id == "ball"
    assert follower.view.crossings[0].progress_s == 1.0
    first = follower.observe(Pose2D(1.5, 0.0, 0.0), 1.0)
    second = follower.observe(Pose2D(0.5, 0.0, 0.0), 1.0)
    assert first.result.progress_s == 1.5
    assert second.result.progress_s == 1.5


def test_projection_boundary_tube_and_remaining_run_in():
    follower = CollectionPathFollower(plan())
    boundary = follower.observe(Pose2D(2.0, 0.0, 0.0), 1.0)
    assert boundary.result.progress_s == 2.0
    off_tube = CollectionPathFollower(plan()).observe(Pose2D(0.5, 1.0, 0.0), 1.0)
    assert off_tube.result.trajectory_tube_ok is False
    assert telemetry(off_tube, FollowerTelemetryCode.TRAJECTORY_TUBE_VIOLATION)
    near = CollectionPathFollower(plan()).observe(Pose2D(0.95, 0.0, 0.0), 1.0)
    assert near.result.remaining_run_in_m == pytest.approx(0.05)
    assert near.result.remaining_run_in_m < plan().segments[0].execution_profile.required_run_in_m


def test_crossing_metrics_hard_speed_limits_and_nominal_telemetry():
    below = CollectionPathFollower(plan()).observe(Pose2D(1.0, 0.0, 0.0), 0.4)
    below_measurement = telemetry(below, FollowerTelemetryCode.CROSSING_MEASUREMENT).crossing
    assert below_measurement.verdict.hard_violation_reason is ProfileViolationReason.SPEED_BELOW_MIN
    assert telemetry(below, FollowerTelemetryCode.PROFILE_VIOLATION)

    above = CollectionPathFollower(plan()).observe(Pose2D(1.0, 0.0, 0.0), 1.6)
    assert telemetry(above, FollowerTelemetryCode.CROSSING_MEASUREMENT).crossing.verdict.hard_violation_reason is ProfileViolationReason.SPEED_ABOVE_MAX

    deviated = CollectionPathFollower(plan()).observe(Pose2D(1.0, 0.0, 0.0), 0.8)
    verdict = telemetry(deviated, FollowerTelemetryCode.CROSSING_MEASUREMENT).crossing.verdict
    assert verdict.hard_compliant and verdict.nominal_tracking is NominalTracking.DEVIATED
    assert telemetry(deviated, FollowerTelemetryCode.NOMINAL_SPEED_DEVIATION)
    assert not any(event.code is FollowerTelemetryCode.PROFILE_VIOLATION for event in deviated.telemetry)

    nominal = CollectionPathFollower(plan()).observe(Pose2D(1.0, 0.0, 0.0), 1.05)
    assert telemetry(nominal, FollowerTelemetryCode.CROSSING_MEASUREMENT).crossing.verdict.nominal_tracking is NominalTracking.WITHIN_TOLERANCE


def test_crossing_lateral_heading_metrics_terminal_and_immutable_plan():
    follower = CollectionPathFollower(plan())
    update = follower.observe(Pose2D(1.0, 0.11, 0.2), 1.0)
    measurement = telemetry(update, FollowerTelemetryCode.CROSSING_MEASUREMENT).crossing
    assert measurement.lateral_error_m == pytest.approx(0.11)
    assert measurement.heading_error_rad == pytest.approx(0.2)
    assert measurement.verdict.hard_violation_reason is ProfileViolationReason.LATERAL_ERROR_EXCEEDED
    completed = follower.observe(Pose2D(3.0, 0.0, 0.0), 1.0)
    assert completed.result.status is PathFollowerStatus.COMPLETED
    frozen = plan()
    with pytest.raises(FrozenInstanceError):
        frozen.segments[0].planned_crossings[0].progress_s = 2.0
    before = frozen.to_dict()
    CollectionPathFollower(frozen).observe(Pose2D(1.0, 0.0, 0.0), 1.0)
    assert frozen.to_dict() == before
