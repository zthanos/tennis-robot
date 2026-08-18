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
    plan_collection_route,
)
from tennis_robot.collection_route_types import (
    BallReasonCode,
    DomainValidationError,
    Point2D,
    Pose2D,
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


def live_second_run_case(config):
    model = CourtModel(
        polygon(
            (-8.861, -8.729), (25.188, -8.490),
            (25.065, 8.910), (-8.984, 8.670),
        ),
        (
            # Exact 40 mm net wall built from the surveyed post endpoints.
            obstacle(
                "net-live", "net",
                (8.425, 5.707), (8.505, -5.593),
                (8.465, -5.593), (8.385, 5.707),
            ),
        ),
    )
    snapshot = ScanSnapshot(
        "scan-live-second-run", FAKE_TIME_S, "map", Pose2D(1.958, -0.067, 3.1204),
        (
            SnapshotBall("turn", Point2D(6.064, -0.678), 0.95, PositionCovariance2D(0.0000645, 0.0, 0.0000645)),
            SnapshotBall("net-a", Point2D(8.018, -0.690), 0.95, PositionCovariance2D(0.0002695, 0.0, 0.0002695)),
            SnapshotBall("net-b", Point2D(8.086, 1.744), 0.95, PositionCovariance2D(0.0002180, 0.0, 0.0002180)),
        ),
        config,
    )
    return snapshot, model


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


def test_net_keepout_ball_gets_parallel_boundary_contact_passes():
    config = configuration()
    # The ball is 0.35 m from the net: inside the ordinary 0.50 m keepout.
    # Moving the route centreline one 0.205 m funnel-mouth half-width away
    # leaves the complete pass outside that same unchanged 0.50 m keepout.
    net = obstacle(
        "net-contact", "net",
        (-4.0, 0.35), (4.0, 0.35), (4.0, 0.39), (-4.0, 0.39),
    )
    feasibility = result(config, court(net))

    assert feasibility.reachable
    assert feasibility.unreachable_reason is None
    assert feasibility.candidates
    assert all(candidate.boundary_recovery for candidate in feasibility.candidates)
    assert all(abs(math.sin(candidate.heading_rad)) < 1e-9 for candidate in feasibility.candidates)
    assert all(candidate.crossing_positions == (Point2D(0.0, 0.0),) for candidate in feasibility.candidates)
    assert all(candidate.crossing.y_m == pytest.approx(-0.205) for candidate in feasibility.candidates)


def test_boundary_contact_pass_does_not_relax_the_canonical_keepout():
    config = configuration()
    # Even the full contact offset cannot put the centreline outside the
    # canonical 0.50 m swept disk, so the target remains unreachable.
    net = obstacle(
        "net-too-close", "net",
        (-4.0, 0.15), (4.0, 0.15), (4.0, 0.19), (-4.0, 0.19),
    )
    feasibility = result(config, court(net))

    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.KEEPOUT


def test_boundary_recovery_never_bypasses_an_unrelated_static_keepout():
    config = configuration()
    net = obstacle(
        "net-contact", "net",
        (-4.0, 0.35), (4.0, 0.35), (4.0, 0.39), (-4.0, 0.39),
    )
    bench = obstacle(
        "bench-near", "bench",
        (0.30, -0.10), (0.40, -0.10), (0.40, 0.10), (0.30, 0.10),
    )
    feasibility = result(config, court(net, bench))

    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.KEEPOUT


def test_live_second_run_net_targets_receive_boundary_recovery_candidates():
    """Regression for the 2026-08-03 distributed localization run."""
    config = configuration()
    snapshot, model = live_second_run_case(config)

    by_id = {item.ball_id: item for item in analyze_snapshot(snapshot, model, config)}
    assert by_id["turn"].reachable
    assert by_id["net-a"].reachable
    assert by_id["net-b"].reachable
    assert all(candidate.boundary_recovery for candidate in by_id["net-a"].candidates)
    assert all(candidate.boundary_recovery for candidate in by_id["net-b"].candidates)


def test_live_second_run_geometry_has_full_forward_only_route_with_u_turn_cap():
    """The exact start/targets need a safe forward U-turn, not a smaller radius."""
    base = configuration()
    config = replace(
        base,
        mechanical=replace(base.mechanical, minimum_turning_radius_m=1.25),
        connector=replace(
            base.connector,
            max_connector_arc_angle_rad=3.0,
            max_connector_total_turn_rad=6.0,
        ),
        planning=replace(base.planning, maximum_candidate_count=200),
        global_route_search=replace(base.global_route_search, max_search_expansions=3000),
    )
    snapshot, model = live_second_run_case(config)

    plan = plan_collection_route(
        snapshot=snapshot, court=model, configuration=config
    ).plan
    assert plan.planning_status.value == "feasible"
    assert {item.ball_id for item in plan.ball_results if item.status.value == "covered"} == {
        "turn", "net-a", "net-b",
    }
    assert all(item.reason_code is not BallReasonCode.TURN_RADIUS for item in plan.ball_results)


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


def test_shared_localization_covariance_cancels_from_relative_capture_width():
    config = configuration()
    localization = config.gazebo_snapshot.localization_xy_covariance.covariance
    measurement_variance = 1e-4
    snapshot = ScanSnapshot(
        "scan-shared-localization", FAKE_TIME_S, "map", SCAN_POSE,
        (SnapshotBall(
            "ball-1", Point2D(0.0, 0.0), 0.95,
            PositionCovariance2D(
                localization.xx + measurement_variance,
                localization.xy,
                localization.yy + measurement_variance,
            ),
        ),),
        config,
    )
    feasibility = analyze_snapshot(snapshot, court(), config)[0]
    assert feasibility.reachable
    expected = (
        config.mechanical.capture_half_width_m
        - config.mechanical.ball_radius_m
        - config.feasibility.confidence_multiplier * math.sqrt(measurement_variance)
        - config.feasibility.tracking_lateral_error_bound_m
        - config.feasibility.capture_safety_margin_m
    )
    assert all(
        candidate.effective_capture_half_width_m == pytest.approx(expected)
        for candidate in feasibility.candidates
    )


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
