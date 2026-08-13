"""Pure Phase 3D end-to-end planner composition tests."""

from dataclasses import replace
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
from tennis_robot.collection_route_planner_v2 import CourtModel, PolygonObstacle, plan_collection_route
from tennis_robot.collection_route_types import (
    BallReasonCode, BallStatus, CollectionRoutePlan, Point2D, Pose2D, PositionCovariance2D,
    PlanningSearchStatus, PlanningStatus, ScanSnapshot, SnapshotBall,
)


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


def court(*obstacles, extent=20.0):
    return CourtModel(polygon((-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent)), tuple(obstacles))


def snapshot(configuration, *entries):
    return ScanSnapshot(
        "scan-compose", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)) for ball_id, x, y in entries),
        configuration,
    )


def test_empty_snapshot_composes_valid_empty_plan():
    configuration = default_configuration()
    plan = plan_collection_route(snapshot=snapshot(configuration), court=court(), configuration=configuration).plan
    assert plan.planning_status is PlanningStatus.EMPTY_NO_BALLS
    assert plan.segments == ()


def test_one_free_ball_and_multiple_ball_connector_route():
    configuration = default_configuration()
    one = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0)), court=court(), configuration=configuration).plan
    assert one.planning_status is PlanningStatus.FEASIBLE
    assert one.ball_results[0].status is BallStatus.COVERED
    multi = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.4)), court=court(), configuration=configuration).plan
    assert any(segment.type.value == "connector" for segment in multi.segments)
    # Both balls are collected, but a ball the connector already sweeps no longer
    # needs a dedicated straight pass, so coverage is counted across segments.
    covered = tuple(ball for segment in multi.segments for ball in segment.covered_ball_ids)
    assert sorted(covered) == ["a", "b"]
    assert all(result.status is BallStatus.COVERED for result in multi.ball_results)


def test_connector_segments_relax_heading_gate_while_passes_keep_default():
    configuration = default_configuration()
    plan = plan_collection_route(
        snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.4)),
        court=court(), configuration=configuration,
    ).plan
    default_gate = configuration.planning.default_execution_profile.max_heading_error_rad
    connector_gate = configuration.planning.connector_max_heading_error_rad
    assert connector_gate > default_gate
    connectors = [s for s in plan.segments if s.type.value in ("connector", "terminal_connector")]
    passes = [s for s in plan.segments if s.type.value == "funnel_pass"]
    assert any(s.type.value == "connector" for s in connectors) and passes
    for s in plan.segments:
        if s.type.value == "connector":
            # Transit relaxes the gate; a connector that collects is capture
            # motion for that stretch and holds the capture-grade gate instead.
            expected = default_gate if s.planned_crossings else connector_gate
            assert s.execution_profile.max_heading_error_rad == expected
        elif s.type.value == "funnel_pass":
            assert s.execution_profile.max_heading_error_rad == default_gate


def test_shared_pass_is_selected_when_it_covers_two_aligned_balls():
    configuration = default_configuration()
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0)), court=court(), configuration=configuration).plan
    assert {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED} == {"a", "b"}
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1


def test_keepout_is_unreachable_and_mixed_result_is_partial():
    configuration = default_configuration()
    blocker = PolygonObstacle("bench", "bench", polygon((4.8, -0.2), (5.2, -0.2), (5.2, 0.2), (4.8, 0.2)))
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.0)), court=court(blocker), configuration=configuration).plan
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert {result.ball_id: result.reason_code for result in plan.ball_results}["b"] is BallReasonCode.KEEPOUT


def test_budget_exhaustion_returns_executable_partial_plan():
    base = default_configuration()
    configuration = replace(base, global_route_search=replace(base.global_route_search, max_search_expansions=1))
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.4)), court=court(), configuration=configuration).plan
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    assert plan.is_executable


def test_declared_candidate_cap_bounds_graph_and_reports_unexamined_target():
    base = default_configuration()
    configuration = replace(base, planning=replace(base.planning, maximum_candidate_count=1))
    plan = plan_collection_route(
        snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 8.0, 3.0)),
        court=court(), configuration=configuration,
    ).plan
    assert plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert any(result.reason_code is BallReasonCode.PLANNING_BUDGET for result in plan.ball_results)


def test_no_valid_terminal_route_returns_non_executable_planning_timeout():
    base = default_configuration()
    configuration = replace(
        base,
        global_route_search=replace(base.global_route_search, terminal_run_out_m=10.0),
    )
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0)), court=court(extent=5.0), configuration=configuration).plan
    assert plan.planning_status is PlanningStatus.PLANNING_TIMEOUT
    assert not plan.is_executable


def test_planner_result_wraps_plan_and_shared_pass_budget_telemetry():
    # Cap shared-pass candidate generation so the budget is exhausted, and
    # confirm the flag propagates onto the PlannerResult without touching the
    # immutable plan contract.
    base = default_configuration()
    configuration = replace(base, shared_pass=replace(base.shared_pass, max_shared_pass_candidates=1))
    aligned = snapshot(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 5.0, 0.0))
    result = plan_collection_route(snapshot=aligned, court=court(), configuration=configuration)
    assert isinstance(result.plan, CollectionRoutePlan)
    assert result.shared_pass_candidate_budget_exhausted is True
    assert not hasattr(result.plan, "shared_pass_candidate_budget_exhausted")
    # A single-ball snapshot generates no shared candidates, so the budget is
    # not exhausted.
    free = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0)), court=court(), configuration=configuration)
    assert free.shared_pass_candidate_budget_exhausted is False


def test_repeated_pure_planning_is_deterministic():
    configuration = default_configuration()
    snap = snapshot(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    first = plan_collection_route(snapshot=snap, court=court(), configuration=configuration)
    second = plan_collection_route(snapshot=snap, court=court(), configuration=configuration)
    assert first.plan.to_dict() == second.plan.to_dict()
    assert first.shared_pass_candidate_budget_exhausted == second.shared_pass_candidate_budget_exhausted
