"""Pure Phase 3D end-to-end planner composition tests."""

from dataclasses import replace
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
from tennis_robot.collection_route_planner_v2 import CourtModel, PolygonObstacle, plan_collection_route
from tennis_robot.collection_route_types import (
    BallReasonCode, BallStatus, Point2D, Pose2D, PositionCovariance2D,
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
    plan = plan_collection_route(snapshot=snapshot(configuration), court=court(), configuration=configuration)
    assert plan.planning_status is PlanningStatus.EMPTY_NO_BALLS
    assert plan.segments == ()


def test_one_free_ball_and_multiple_ball_connector_route():
    configuration = default_configuration()
    one = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0)), court=court(), configuration=configuration)
    assert one.planning_status is PlanningStatus.FEASIBLE
    assert one.ball_results[0].status is BallStatus.COVERED
    multi = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.4)), court=court(), configuration=configuration)
    assert len([segment for segment in multi.segments if segment.type.value == "funnel_pass"]) >= 2
    assert any(segment.type.value == "connector" for segment in multi.segments)


def test_shared_pass_is_selected_when_it_covers_two_aligned_balls():
    configuration = default_configuration()
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0)), court=court(), configuration=configuration)
    assert {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED} == {"a", "b"}
    assert len([segment for segment in plan.segments if segment.type.value == "funnel_pass"]) == 1


def test_keepout_is_unreachable_and_mixed_result_is_partial():
    configuration = default_configuration()
    blocker = PolygonObstacle("bench", "bench", polygon((4.8, -0.2), (5.2, -0.2), (5.2, 0.2), (4.8, 0.2)))
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.0)), court=court(blocker), configuration=configuration)
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert {result.ball_id: result.reason_code for result in plan.ball_results}["b"] is BallReasonCode.KEEPOUT


def test_budget_exhaustion_returns_executable_partial_plan():
    base = default_configuration()
    configuration = replace(base, global_route_search=replace(base.global_route_search, max_search_expansions=1))
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0), ("b", 5.0, 0.4)), court=court(), configuration=configuration)
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    assert plan.is_executable


def test_no_valid_terminal_route_returns_non_executable_planning_timeout():
    base = default_configuration()
    configuration = replace(
        base,
        global_route_search=replace(base.global_route_search, terminal_run_out_m=10.0),
    )
    plan = plan_collection_route(snapshot=snapshot(configuration, ("a", 3.0, 0.0)), court=court(extent=5.0), configuration=configuration)
    assert plan.planning_status is PlanningStatus.PLANNING_TIMEOUT
    assert not plan.is_executable


def test_repeated_pure_planning_is_deterministic():
    configuration = default_configuration()
    snap = snapshot(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    first = plan_collection_route(snapshot=snap, court=court(), configuration=configuration)
    second = plan_collection_route(snapshot=snap, court=court(), configuration=configuration)
    assert first.to_dict() == second.to_dict()
