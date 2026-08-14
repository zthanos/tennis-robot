"""Pure Phase 3C multi-ball pass generation tests (line enumeration)."""

from dataclasses import replace
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
from tennis_robot.collection_route_connector_graph import build_directed_candidate_graph
from tennis_robot.collection_route_global_solver import solve_global_route
from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, PerBallFeasibility, PolygonObstacle
from tennis_robot.collection_route_shared_pass import generate_shared_passes
from tennis_robot.collection_route_types import BallStatus, Point2D, Pose2D, PositionCovariance2D, ScanSnapshot, SnapshotBall


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


def court(*obstacles):
    return CourtModel(polygon((-20, -20), (20, -20), (20, 20), (-20, 20)), tuple(obstacles))


def single(ball_id, x, y=0.0, width=0.1):
    point = Point2D(x, y)
    return FunnelPassCandidate(ball_id, (ball_id,), 0.0, Pose2D(x - 1.0, y, 0.0), point, Pose2D(x + 0.3, y, 0.0), width, (point,))


def snapshot_of(configuration, *entries):
    return ScanSnapshot(
        "scan-shared", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(SnapshotBall(ball_id, Point2D(x, y), 0.9, PositionCovariance2D(1e-6, 0.0, 1e-6)) for ball_id, x, y in entries),
        configuration,
    )


def generate(configuration, *entries, model=None, singles=None):
    """Enumerate line candidates over balls at the given positions."""
    candidates = singles if singles is not None else tuple(single(ball_id, x, y) for ball_id, x, y in entries)
    return generate_shared_passes(
        snapshot=snapshot_of(configuration, *entries),
        single_ball_candidates=tuple(candidates),
        court=model or court(),
        configuration=configuration,
    )


def covered_sets(result):
    return {candidate.covered_ball_ids for candidate in result.candidates}


def test_valid_pair_and_ordered_triplet():
    configuration = default_configuration()
    pair = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    assert ("a", "b") in covered_sets(pair)
    triplet = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 5.0, 0.0))
    assert ("a", "b", "c") in covered_sets(triplet)


def test_nearly_collinear_triplet_is_grouped():
    # The exact-heading merge this replaced could never do this: the a->b, a->c
    # and b->c headings all differ slightly, so the three never shared a bucket.
    configuration = default_configuration()
    result = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.02), ("c", 5.0, -0.015))
    assert ("a", "b", "c") in covered_sets(result)
    candidate = next(item for item in result.candidates if item.covered_ball_ids == ("a", "b", "c"))
    # The line is fitted to the group rather than taken from any one pair, so it
    # runs along the x axis to within a couple of degrees.
    assert min(abs(candidate.heading_rad), abs(abs(candidate.heading_rad) - math.pi)) < 0.05


def test_both_travel_directions_are_offered():
    configuration = default_configuration()
    result = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    headings = sorted(candidate.heading_rad for candidate in result.candidates if candidate.covered_ball_ids in (("a", "b"), ("b", "a")))
    assert len(headings) == 2
    assert abs(abs(headings[0] - headings[1]) - math.pi) < 1e-9
    # Travelling the other way reverses which ball is met first.
    assert {candidate.covered_ball_ids for candidate in result.candidates} == {("a", "b"), ("b", "a")}


def test_crossing_positions_are_the_real_ball_positions():
    configuration = default_configuration()
    result = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.02))
    candidate = next(item for item in result.candidates if item.covered_ball_ids == ("a", "b"))
    assert [(point.x_m, point.y_m) for point in candidate.crossing_positions] == [(3.0, 0.0), (4.0, 0.02)]


def test_lateral_corridor_excludes_a_third_ball_off_the_line():
    configuration = default_configuration()
    # Any two balls define a line and sit on it by construction, so the corridor
    # is what decides whether a third one joins them.
    result = generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 5.0, 0.3))
    assert ("a", "b") in covered_sets(result)
    assert not any(len(ids) == 3 for ids in covered_sets(result))


def test_longitudinal_spacing_rejection():
    configuration = default_configuration()
    # 0.2 m apart is below the mechanical spacing the intake needs.
    assert covered_sets(generate(configuration, ("a", 3.0, 0.0), ("b", 3.2, 0.0))) == set()


def test_obstacle_failure_rejects_full_shared_entry_to_exit_segment():
    configuration = default_configuration()
    blocker = PolygonObstacle("bench", "bench", polygon((3.65, -0.1), (3.85, -0.1), (3.85, 0.1), (3.65, 0.1)))
    assert generate(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), model=court(blocker)).candidates == ()


def test_tangent_constrained_ball_only_joins_a_parallel_line():
    configuration = default_configuration()
    # A net segment along x: a ball beside it may only be approached parallel,
    # so a line running across it is rejected.
    net = PolygonObstacle("net", "net", polygon((0.0, 1.2), (8.0, 1.2), (8.0, 1.24), (0.0, 1.24)))
    # "a" sits inside the tangent activation distance of the net, so a line
    # running across it is rejected however well it fits the two balls.
    across = generate(configuration, ("a", 3.0, 0.5), ("b", 3.05, -1.5), model=court(net))
    assert covered_sets(across) == set()
    alongside = generate(configuration, ("a", 3.0, 0.5), ("b", 4.5, 0.5), model=court(net))
    assert ("a", "b") in covered_sets(alongside)


def test_boundary_contact_candidate_is_never_merged():
    configuration = default_configuration()
    point = Point2D(4.0, 0.0)
    recovery = FunnelPassCandidate(
        "b", ("b",), 0.0,
        Pose2D(3.0, 0.0, 0.0), point, Pose2D(4.3, 0.0, 0.0),
        0.205, (point,), True,
    )
    result = generate(
        configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0),
        singles=(single("a", 3.0), recovery),
    )
    assert covered_sets(result) == set()


def test_deterministic_order_and_candidate_cap_budget_telemetry():
    base = default_configuration()
    configuration = replace(base, shared_pass=replace(base.shared_pass, max_shared_pass_candidates=1))
    entries = (("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 5.0, 0.0))
    result = generate(configuration, *entries)
    assert result.candidate_budget_exhausted
    assert len(result.candidates) == 1
    # Largest group first, so a starved budget keeps the most useful pass.
    assert result.candidates[0].covered_ball_ids == ("a", "b", "c")
    assert [c.covered_ball_ids for c in generate(configuration, *entries).candidates] == \
           [c.covered_ball_ids for c in generate(configuration, *entries).candidates]


def test_b2_solver_selects_multi_ball_node_when_it_increases_coverage():
    configuration = default_configuration()
    singles = (single("a", 3.0), single("b", 4.0))
    snapshot = snapshot_of(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    shared = generate_shared_passes(
        snapshot=snapshot, single_ball_candidates=singles, court=court(), configuration=configuration
    ).candidates[0]
    graph = build_directed_candidate_graph(snapshot=snapshot, candidates=singles + (shared,), court=court(), configuration=configuration)
    feasibility = (PerBallFeasibility("a", (singles[0],), None), PerBallFeasibility("b", (singles[1],), None))
    plan = solve_global_route(snapshot=snapshot, feasibility=feasibility, graph=graph, court=court(), configuration=configuration)
    assert {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED} == {"a", "b"}
    funnel = next(segment for segment in plan.segments if segment.type.value == "funnel_pass")
    assert tuple(crossing.ball_id for crossing in funnel.planned_crossings) == funnel.covered_ball_ids
    assert all(first.progress_s < second.progress_s for first, second in zip(funnel.planned_crossings, funnel.planned_crossings[1:]))
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1
