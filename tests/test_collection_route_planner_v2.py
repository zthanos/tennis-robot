"""Pure Phase 3A feasibility tests; no legacy planner/runtime imports."""

from dataclasses import replace
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, SCAN_POSE, default_configuration
from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    PlannerInputError,
    PolygonObstacle,
    analyze_snapshot,
)
from tennis_robot.collection_route_types import (
    BallReasonCode,
    DomainValidationError,
    Point2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)


def polygon(*coordinates):
    return tuple(Point2D(float(x), float(y)) for x, y in coordinates)


COURT_POLYGON = polygon((-5, -5), (5, -5), (5, 5), (-5, 5))


def configuration(*, run_in=1.0, run_out=0.3):
    base = default_configuration()
    mechanical = replace(base.mechanical, minimum_run_in_m=run_in, minimum_run_out_m=run_out)
    return replace(base, mechanical=mechanical)


def snapshot_for(configuration, x=0.0, y=0.0):
    return ScanSnapshot(
        "scan-phase-3a",
        FAKE_TIME_S,
        "map",
        SCAN_POSE,
        (SnapshotBall("ball-1", Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        configuration,
    )


def court(*obstacles):
    return CourtModel(COURT_POLYGON, tuple(obstacles))


def obstacle(obstacle_id, kind, *coordinates):
    return PolygonObstacle(obstacle_id, kind, polygon(*coordinates))


def result(configuration, model, *, x=0.0, y=0.0):
    return analyze_snapshot(snapshot_for(configuration, x, y), model, configuration)[0]


def horizontal_boundary(kind="net"):
    return obstacle(f"{kind}-near", kind, (-4.0, 0.65), (4.0, 0.65), (4.0, 0.75), (-4.0, 0.75))


def test_free_ball_returns_all_feasible_straight_candidates_with_positive_corridor():
    config = configuration()
    feasibility = result(config, court())
    assert feasibility.reachable
    assert feasibility.unreachable_reason is None
    assert len(feasibility.candidates) == config.feasibility.heading_sample_count
    assert all(candidate.effective_capture_half_width_m > 0.0 for candidate in feasibility.candidates)
    assert all(candidate.crossing == Point2D(0.0, 0.0) for candidate in feasibility.candidates)


def test_ball_inside_inflated_keepout_is_deterministically_unreachable():
    model = court(obstacle("bench-1", "bench", (-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)))
    feasibility = result(configuration(), model)
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.KEEPOUT


def test_entry_segment_collision_is_no_entry():
    blockers = (
        obstacle("left-entry", "other", (-1.1, -0.1), (-0.9, -0.1), (-0.9, 0.1), (-1.1, 0.1)),
        obstacle("right-entry", "other", (0.9, -0.1), (1.1, -0.1), (1.1, 0.1), (0.9, 0.1)),
    )
    feasibility = result(configuration(run_in=1.0, run_out=0.3), court(horizontal_boundary(), *blockers))
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.NO_ENTRY


def test_exit_segment_collision_is_no_exit():
    blockers = (
        obstacle("left-exit", "other", (-1.2, -0.1), (-1.0, -0.1), (-1.0, 0.1), (-1.2, 0.1)),
        obstacle("right-exit", "other", (1.0, -0.1), (1.2, -0.1), (1.2, 0.1), (1.0, 0.1)),
    )
    feasibility = result(configuration(run_in=0.3, run_out=1.0), court(horizontal_boundary(), *blockers))
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.NO_EXIT


@pytest.mark.parametrize("kind", ["net", "fence"])
def test_near_net_or_fence_allows_only_tangent_headings(kind):
    feasibility = result(configuration(), court(horizontal_boundary(kind)))
    assert feasibility.reachable
    assert all(abs(math.sin(candidate.heading_rad)) < 1e-9 for candidate in feasibility.candidates)


def test_corner_with_incompatible_tangent_constraints_is_no_entry():
    horizontal = horizontal_boundary("net")
    vertical = obstacle("fence-near", "fence", (0.65, -4.0), (0.75, -4.0), (0.75, 4.0), (0.65, 4.0))
    feasibility = result(configuration(), court(horizontal, vertical))
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.NO_ENTRY


def test_corridor_collapse_is_no_candidate_found_not_no_entry():
    # A large isotropic position covariance drives the effective capture
    # corridor non-positive for every heading; that is a corridor collapse
    # (no_candidate_found), never an entry-geometry failure.
    config = configuration()
    snapshot = ScanSnapshot(
        "scan-corridor-collapse", FAKE_TIME_S, "map", SCAN_POSE,
        (SnapshotBall("ball-1", Point2D(0.0, 0.0), 0.95, PositionCovariance2D(1.0, 0.0, 1.0)),),
        config,
    )
    feasibility = analyze_snapshot(snapshot, court(), config)[0]
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.NO_CANDIDATE_FOUND


def test_phase_3a_never_emits_turn_radius_reason():
    feasibility = result(configuration(), court())
    assert feasibility.unreachable_reason is not BallReasonCode.TURN_RADIUS


def test_invalid_or_missing_planning_configuration_is_rejected():
    config = configuration()
    snapshot = snapshot_for(config)
    with pytest.raises(PlannerInputError):
        analyze_snapshot(snapshot, court(), None)
    with pytest.raises(DomainValidationError):
        analyze_snapshot(snapshot, court(), replace(config, feasibility=replace(config.feasibility, heading_sample_count=0)))
