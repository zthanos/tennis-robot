"""Pure Phase 3C shared-pass generation tests."""

from dataclasses import replace
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


def generate(configuration, *items, model=None):
    return generate_shared_passes(single_ball_candidates=tuple(items), court=model or court(), configuration=configuration)


def test_valid_pair_and_ordered_triplet():
    configuration = default_configuration()
    pair = generate(configuration, single("a", 3.0), single("b", 4.0)).candidates
    assert pair[0].covered_ball_ids == ("a", "b")
    triplet = generate(configuration, single("c", 5.0), single("a", 3.0), single("b", 4.0)).candidates
    assert any(candidate.covered_ball_ids == ("a", "b", "c") for candidate in triplet)


def test_lateral_corridor_and_longitudinal_spacing_rejections():
    configuration = default_configuration()
    assert generate(configuration, single("a", 3.0, 0.0, 0.05), single("b", 4.0, 0.3, 0.05)).candidates == ()
    assert generate(configuration, single("a", 3.0), single("b", 3.2)).candidates == ()


def test_obstacle_failure_rejects_full_shared_entry_to_exit_segment():
    configuration = default_configuration()
    blocker = PolygonObstacle("bench", "bench", polygon((3.65, -0.1), (3.85, -0.1), (3.85, 0.1), (3.65, 0.1)))
    assert generate(configuration, single("a", 3.0), single("b", 4.0), model=court(blocker)).candidates == ()


def test_non_common_heading_tangent_constrained_candidates_are_not_shared():
    configuration = default_configuration()
    constrained = FunnelPassCandidate("b", ("b",), 1.5707963267948966, Pose2D(4.0, -1.0, 1.5707963267948966), Point2D(4.0, 0.0), Pose2D(4.0, 0.3, 1.5707963267948966), 0.1, (Point2D(4.0, 0.0),))
    assert generate(configuration, single("a", 3.0), constrained).candidates == ()


def test_boundary_contact_candidate_is_never_merged_into_shared_pass():
    configuration = default_configuration()
    point = Point2D(4.0, 0.2)
    recovery = FunnelPassCandidate(
        "b", ("b",), 0.0,
        Pose2D(3.0, 0.0, 0.0), Point2D(4.0, 0.0), Pose2D(4.3, 0.0, 0.0),
        0.205, (point,), True,
    )

    assert generate(configuration, single("a", 3.0), recovery).candidates == ()


def test_deterministic_order_and_candidate_cap_budget_telemetry():
    base = default_configuration()
    configuration = replace(base, shared_pass=replace(base.shared_pass, max_shared_pass_candidates=2))
    result = generate(configuration, single("c", 5.0), single("a", 3.0), single("b", 4.0))
    assert result.candidate_budget_exhausted
    assert [candidate.covered_ball_ids for candidate in result.candidates] == [("a", "b"), ("a", "c")]


def test_b2_solver_selects_shared_node_when_it_increases_coverage():
    configuration = default_configuration()
    singles = (single("a", 3.0), single("b", 4.0))
    shared = generate(configuration, *singles).candidates[0]
    snapshot = ScanSnapshot(
        "scan-shared", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(SnapshotBall(ball_id, Point2D(index, 0.0), 0.9, PositionCovariance2D(1e-6, 0.0, 1e-6)) for index, ball_id in enumerate(("a", "b"))),
        configuration,
    )
    graph = build_directed_candidate_graph(snapshot=snapshot, candidates=singles + (shared,), court=court(), configuration=configuration)
    feasibility = (PerBallFeasibility("a", (singles[0],), None), PerBallFeasibility("b", (singles[1],), None))
    plan = solve_global_route(snapshot=snapshot, feasibility=feasibility, graph=graph, court=court(), configuration=configuration)
    assert {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED} == {"a", "b"}
    funnel = next(segment for segment in plan.segments if segment.type.value == "funnel_pass")
    assert tuple(crossing.ball_id for crossing in funnel.planned_crossings) == funnel.covered_ball_ids
    assert all(first.progress_s < second.progress_s for first, second in zip(funnel.planned_crossings, funnel.planned_crossings[1:]))
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1
