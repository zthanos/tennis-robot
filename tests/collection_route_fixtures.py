"""Deterministic, ROS-free fixtures for the continuous collection-route rewrite.

These fixtures specify the expected planner outcomes for later phases.  They
are intentionally not generated from the legacy per-stop planner: that code
is regression evidence only and is not an acceptance oracle for this route.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.perception_covariance_calibration import (
    CalibrationArtifact,
    PerceptionSpatialValidationConfig,
    compute_artifact_sha256,
)
from tennis_robot.collection_route_types import (
    BallReasonCode,
    BallResult,
    BallStatus,
    CollectionRouteConfiguration,
    ConnectorConfiguration,
    ExecutionProfile,
    FeasibilityConfiguration,
    FollowUpConfiguration,
    GazeboSnapshotConfiguration,
    GlobalRouteSearchConfiguration,
    LocalizationXYCovariance,
    MechanicalConfiguration,
    PlanningConfiguration,
    PlanningStatus,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    SafetyConfiguration,
    ScanConfiguration,
    ScanSnapshot,
    SharedPassConfiguration,
    SnapshotAssociationConfiguration,
    SnapshotBall,
)

MAP_FRAME = "map"
FAKE_TIME_S = 1_000.0
DEFAULT_COVARIANCE = PositionCovariance2D(0.01, 0.0, 0.01)
SCAN_POSE = Pose2D(2.0, -3.0, 0.0)


@dataclass(frozen=True)
class StaticObstacleFixture:
    obstacle_id: str
    center: Point2D
    radius_m: float


@dataclass(frozen=True)
class CourtFixture:
    """Pure court-frame helper; geometry evaluation belongs to Phase 3."""

    fence_min: Point2D = Point2D(-10.0, -10.0)
    fence_max: Point2D = Point2D(10.0, 10.0)
    net_start: Point2D = Point2D(0.0, -6.0)
    net_end: Point2D = Point2D(0.0, 6.0)
    static_obstacles: tuple[StaticObstacleFixture, ...] = ()


@dataclass(frozen=True)
class CollectionRouteFixture:
    name: str
    snapshot: ScanSnapshot
    expected_planning_status: PlanningStatus
    expected_ball_results: tuple[BallResult, ...]
    court: CourtFixture = CourtFixture()
    moving_obstacle: bool = False
    moving_obstacle_start: Point2D | None = None


def default_profile() -> ExecutionProfile:
    return ExecutionProfile(1.0, 0.5, 1.5, 0.1, 1.0, 1.0, 0.2, 1.0, 0.3, 1.25, 0.1, 0.1, False, False)


def default_court_half_boundary():
    """Robot-side half plane derived from the fixture court net (robot at x>0)."""
    from tennis_robot.collection_scan_snapshot import CourtHalfBoundary

    court = CourtFixture()
    return CourtHalfBoundary(court.net_start, court.net_end, -1)


def default_spatial_validation() -> PerceptionSpatialValidationConfig:
    return PerceptionSpatialValidationConfig(0.05, 0.5, 1e-9, 1e-6, 1.0)


def default_calibration_artifact() -> CalibrationArtifact:
    # Mirrors the approved gazebo-v2 artifact identity/parameters with a
    # genuinely computed content hash; fixtures carry no implicit defaults.
    data = {
        "schema_version": "perception-covariance-calibration/v1",
        "calibration_id": "gazebo-range-depth-quality-diagonal-v1-20260719-v2",
        "model_id": "range_depth_quality_diagonal_v1",
        "model_version": "gazebo-v2",
        "platform": "gazebo",
        "calibrated_at": "2026-07-19",
        "parameters": {
            "axis_variance_floor_m2": [0.0, 0.000514, 0.002111],
            "axis_range_variance_per_m2": [2e-06, 4.4e-05, 7.1e-05],
            "axis_quality_variance_m2": [0.0, 0.001049, 0.0],
        },
        "range_validity_domain_m": {"min": 1.021574547701437, "max": 2.9799470906893557},
        "depth_quality_validity_domain": {"min": 0.8979591836734694, "max": 1.0},
        "evidence_reference": "docs/gazebo-perception-covariance-c2-amendment-coverage-report-el.md",
        "acceptance_metrics": {"accepted_sample_count": 270},
    }
    data["artifact_sha256"] = compute_artifact_sha256(data)
    return CalibrationArtifact.from_dict(data)


def default_configuration(maximum_candidate_count: int = 20) -> CollectionRouteConfiguration:
    return CollectionRouteConfiguration(
        "collection-route/v1",
        MechanicalConfiguration(0.17, 0.033, 0.8, 0.6, 0.4, 0.34, 0.05, 0.8, 1.25, 0.2, 1.0, 0.3, 0.5),
        SafetyConfiguration(0.1, 0.15, 0.2, 0.5, 0.2, 2.0, 2.0, 10.0),
        ScanConfiguration(1.0, 20.0, 2),
        FeasibilityConfiguration(16, 2.0, 0.04, 0.05, 0.50, 0.75, 0.20),
        ConnectorConfiguration(20.0, 1.5, 3.0),
        GlobalRouteSearchConfiguration(1000, 0.5, 1.0, 0.8, 0.2, 1.0, 1.0, 1.0, 1.0, 1.0),
        SharedPassConfiguration(3, 100, 0.5),
        FollowUpConfiguration(False, 1),
        PlanningConfiguration(default_profile(), 1.0, maximum_candidate_count, 1.0, 1.0, 1.0, 1.0, 1.0),
        GazeboSnapshotConfiguration(
            "gazebo-mvp-provisional-planning-safety-v1",
            LocalizationXYCovariance(PositionCovariance2D(0.01, 0.0, 0.01)),
            SnapshotAssociationConfiguration(9.0, 2, 2),
        ),
        default_spatial_validation(),
        default_calibration_artifact(),
    )


def snapshot_for(
    *balls: tuple[str, float, float],
    scan_id: str,
    maximum_candidate_count: int = 20,
) -> ScanSnapshot:
    return ScanSnapshot(
        scan_id,
        FAKE_TIME_S,
        MAP_FRAME,
        SCAN_POSE,
        tuple(SnapshotBall(ball_id, Point2D(x_m, y_m), 0.95, DEFAULT_COVARIANCE) for ball_id, x_m, y_m in balls),
        default_configuration(maximum_candidate_count),
    )


def result(ball_id: str, status: BallStatus, reason: BallReasonCode, pass_id: str | None = None) -> BallResult:
    return BallResult(ball_id, status, reason, pass_id)


EMPTY_SCAN = CollectionRouteFixture(
    "empty_scan",
    snapshot_for(scan_id="scan_empty"),
    PlanningStatus.EMPTY_NO_BALLS,
    (),
)

ONE_FREE_BALL = CollectionRouteFixture(
    "one_free_ball",
    snapshot_for(("ball_free", 3.0, -3.0), scan_id="scan_free"),
    PlanningStatus.FEASIBLE,
    (result("ball_free", BallStatus.COVERED, BallReasonCode.SELECTED, "pass_free"),),
)

TWO_CLOSE_SHARED_PASS = CollectionRouteFixture(
    "two_close_shared_pass",
    snapshot_for(("ball_close_a", 3.0, -3.0), ("ball_close_b", 3.8, -3.0), scan_id="scan_close"),
    PlanningStatus.FEASIBLE,
    (
        result("ball_close_a", BallStatus.COVERED, BallReasonCode.SELECTED, "pass_shared"),
        result("ball_close_b", BallStatus.COVERED, BallReasonCode.SELECTED, "pass_shared"),
    ),
)

PLANNING_BUDGET_DEFERRED = CollectionRouteFixture(
    "planning_budget_deferred",
    snapshot_for(
        ("ball_budget_first", 3.0, -3.0),
        ("ball_budget_deferred", 6.0, -3.0),
        scan_id="scan_budget",
        maximum_candidate_count=1,
    ),
    PlanningStatus.PARTIAL,
    (
        result("ball_budget_first", BallStatus.COVERED, BallReasonCode.SELECTED, "pass_budget"),
        result("ball_budget_deferred", BallStatus.DEFERRED, BallReasonCode.PLANNING_BUDGET),
    ),
)

NET_CORNER_UNREACHABLE = CollectionRouteFixture(
    "net_corner_unreachable",
    snapshot_for(("ball_net_corner", 0.05, 5.95), scan_id="scan_net_corner"),
    PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS,
    (result("ball_net_corner", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),),
)

FENCE_CORNER_UNREACHABLE = CollectionRouteFixture(
    "fence_corner_unreachable",
    snapshot_for(("ball_fence_corner", 9.95, 9.95), scan_id="scan_fence_corner"),
    PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS,
    (result("ball_fence_corner", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),),
)

STATIC_OBSTACLE_UNREACHABLE = CollectionRouteFixture(
    "static_obstacle_unreachable",
    snapshot_for(("ball_static", 5.0, -3.0), scan_id="scan_static"),
    PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS,
    (result("ball_static", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),),
    court=CourtFixture(static_obstacles=(StaticObstacleFixture("bench_1", Point2D(5.0, -3.0), 0.5),)),
)

MOVING_OBSTACLE = CollectionRouteFixture(
    "moving_obstacle",
    snapshot_for(("ball_moving", 3.0, -3.0), scan_id="scan_moving"),
    PlanningStatus.FEASIBLE,
    (result("ball_moving", BallStatus.COVERED, BallReasonCode.SELECTED, "pass_moving"),),
    moving_obstacle=True,
    moving_obstacle_start=Point2D(2.5, -3.0),
)

ALL_UNREACHABLE = CollectionRouteFixture(
    "all_unreachable",
    snapshot_for(("ball_net", 0.05, 0.0), ("ball_fence", 9.95, 0.0), scan_id="scan_all_unreachable"),
    PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS,
    (
        result("ball_net", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),
        result("ball_fence", BallStatus.UNREACHABLE, BallReasonCode.KEEPOUT),
    ),
)

ALL_FIXTURES = (
    EMPTY_SCAN,
    ONE_FREE_BALL,
    TWO_CLOSE_SHARED_PASS,
    PLANNING_BUDGET_DEFERRED,
    NET_CORNER_UNREACHABLE,
    FENCE_CORNER_UNREACHABLE,
    STATIC_OBSTACLE_UNREACHABLE,
    MOVING_OBSTACLE,
    ALL_UNREACHABLE,
)
