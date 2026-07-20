"""Pure Phase 3B2 global search tests."""

from dataclasses import replace
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
from tennis_robot.collection_route_connector_graph import ConnectorRejectionCode, DirectedCandidateGraph, build_directed_candidate_graph
from tennis_robot.collection_route_global_solver import solve_global_route
from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, PerBallFeasibility, PolygonObstacle
from tennis_robot.collection_route_types import (
    BallReasonCode, BallStatus, Point2D, Pose2D, PositionCovariance2D,
    PlanningSearchStatus, PlanningStatus, ScanSnapshot, SnapshotBall,
)


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


def court(*obstacles):
    return CourtModel(polygon((-20, -20), (20, -20), (20, 20), (-20, 20)), tuple(obstacles))


def candidate(node_ball_id, covered, x):
    crossings = tuple(Point2D(x + 1.0 + index * 0.2, 0.0) for index, _ in enumerate(covered))
    return FunnelPassCandidate(node_ball_id, tuple(covered), 0.0, Pose2D(x, 0.0, 0.0), crossings[0], Pose2D(x + 2.0, 0.0, 0.0), 0.05, crossings)


def snapshot(configuration, *ball_ids):
    balls = tuple(SnapshotBall(ball_id, Point2D(float(index), 0.0), 0.9, PositionCovariance2D(1e-6, 0.0, 1e-6)) for index, ball_id in enumerate(ball_ids))
    return ScanSnapshot("scan-b2", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0), balls, configuration)


def solve(configuration, balls, candidates, model=None, graph=None):
    snap = snapshot(configuration, *balls)
    candidate_tuple = tuple(candidates)
    graph = graph or build_directed_candidate_graph(snapshot=snap, candidates=candidate_tuple, court=model or court(), configuration=configuration)
    feasibility = tuple(PerBallFeasibility(ball_id, tuple(item for item in candidate_tuple if ball_id in item.covered_ball_ids), None) for ball_id in balls)
    return solve_global_route(snapshot=snap, feasibility=feasibility, graph=graph, court=model or court(), configuration=configuration)


def test_score_prioritizes_unique_coverage_before_cost_and_pass_count():
    configuration = default_configuration()
    single = candidate("single", ("ball-a",), 3.0)
    shared = candidate("shared", ("ball-a", "ball-b"), 8.0)
    plan = solve(configuration, ("ball-a", "ball-b"), (single, shared))
    assert plan.planning_status is PlanningStatus.FEASIBLE
    assert {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED} == {"ball-a", "ball-b"}
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1


def test_score_uses_cost_after_coverage():
    base = default_configuration()
    configuration = replace(base, global_route_search=replace(base.global_route_search, weight_length=1.0, weight_time=0.0, weight_curvature=0.0, weight_energy=0.0, weight_pass_count=0.0))
    near = candidate("near", ("ball-a",), 4.0)
    far = candidate("far", ("ball-a",), 8.0)
    plan = solve(configuration, ("ball-a",), (far, near))
    assert any(segment.id == "pass:pass:near:1" for segment in plan.segments)


def test_score_uses_pass_count_after_equal_coverage_and_cost_weights():
    base = default_configuration()
    configuration = replace(base, global_route_search=replace(base.global_route_search, weight_length=0.0, weight_time=0.0, weight_curvature=0.0, weight_energy=0.0, weight_pass_count=1.0))
    shared = candidate("shared", ("ball-a", "ball-b"), 8.0)
    first = candidate("first", ("ball-a",), 4.0)
    second = candidate("second", ("ball-b",), 7.0)
    plan = solve(configuration, ("ball-a", "ball-b"), (first, second, shared))
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1


def test_directed_edges_only_define_valid_route_search():
    configuration = default_configuration()
    item = candidate("a", ("ball-a",), 4.0)
    snap = snapshot(configuration, "ball-a")
    full = build_directed_candidate_graph(snapshot=snap, candidates=(item,), court=court(), configuration=configuration)
    graph = DirectedCandidateGraph(full.pass_nodes, tuple(edge for edge in full.edges if edge.source_node_id != "start"))
    plan = solve(configuration, ("ball-a",), (item,), graph=graph)
    assert plan.planning_status is PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS
    assert plan.ball_results[0].reason_code is BallReasonCode.ROUTE_CONFLICT


def test_terminal_extension_rejection_invalidates_route():
    configuration = default_configuration()
    item = candidate("a", ("ball-a",), 4.0)
    blocker = PolygonObstacle("terminal", "other", polygon((6.3, -0.1), (6.6, -0.1), (6.6, 0.1), (6.3, 0.1)))
    plan = solve(configuration, ("ball-a",), (item,), court(blocker))
    assert plan.planning_status is PlanningStatus.PLANNING_TIMEOUT


def test_budget_exhaustion_returns_partial_plan_and_planning_budget_deferral():
    base = default_configuration()
    configuration = replace(base, global_route_search=replace(base.global_route_search, max_search_expansions=1))
    first = candidate("a", ("ball-a",), 4.0)
    second = candidate("b", ("ball-b",), 8.0)
    plan = solve(configuration, ("ball-a", "ball-b"), (first, second))
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    assert any(result.reason_code is BallReasonCode.PLANNING_BUDGET for result in plan.ball_results)


def test_deferred_route_conflict_remains_distinct_from_3a_unreachable():
    configuration = default_configuration()
    covered = candidate("a", ("ball-a",), 4.0)
    snap = snapshot(configuration, "ball-a", "ball-b")
    graph = build_directed_candidate_graph(snapshot=snap, candidates=(covered,), court=court(), configuration=configuration)
    feasibility = (
        PerBallFeasibility("ball-a", (covered,), None),
        PerBallFeasibility("ball-b", (), BallReasonCode.KEEPOUT),
    )
    plan = solve_global_route(snapshot=snap, feasibility=feasibility, graph=graph, court=court(), configuration=configuration)
    assert {result.ball_id: result.status for result in plan.ball_results}["ball-a"] is BallStatus.COVERED
    assert {result.ball_id: result.reason_code for result in plan.ball_results}["ball-b"] is BallReasonCode.KEEPOUT


def test_turn_radius_requires_turning_only_relaxation_of_start_and_terminal_route():
    configuration = default_configuration()
    item = candidate("a", ("ball-a",), 4.0)
    snap = snapshot(configuration, "ball-a")
    full = build_directed_candidate_graph(snapshot=snap, candidates=(item,), court=court(), configuration=configuration)
    graph = DirectedCandidateGraph(
        full.pass_nodes,
        tuple(
            replace(edge, path=None, maximum_curvature_per_m=None, collision_free=False, rejection=ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED)
            if edge.source_node_id == "start" else edge
            for edge in full.edges
        ),
    )
    plan = solve(configuration, ("ball-a",), (item,), graph=graph)
    assert plan.ball_results[0].status is BallStatus.UNREACHABLE
    assert plan.ball_results[0].reason_code is BallReasonCode.TURN_RADIUS


def test_stable_route_id_breaks_equal_score_ties_deterministically():
    configuration = default_configuration()
    first = candidate("a", ("ball-a",), 4.0)
    second = candidate("b", ("ball-a",), 4.0)
    first_plan = solve(configuration, ("ball-a",), (second, first))
    second_plan = solve(configuration, ("ball-a",), (first, second))
    assert first_plan.plan_id == second_plan.plan_id
