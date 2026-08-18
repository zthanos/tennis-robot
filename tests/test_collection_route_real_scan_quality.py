"""Route quality on a real recorded scan, against the measured baselines.

Synthetic geometries prove the invariants; only a real scan says whether the
route a court actually produces is any good.  The numbers here are measurements
of the two planners this one replaces, taken on the same snapshot with the same
shipped configuration (debug log #57):

    flat global solver   10/10 covered, 55.92 m, 15.21 rad, 6 passes, 22.6 s
    cluster-block router 10/10 covered, 58.73 m, 23.74 rad, 8 passes,  8.0 s

Both length and curvature are gates.  Runtime is not asserted -- it belongs to
the machine, not to the planner -- but the expansion budget these tests use is
the shipped one, so the quality checked here is the quality the robot gets.
"""

import json
import math
import os
import sys
from dataclasses import replace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_route_cost import plan_objective_cost  # noqa: E402
from tennis_robot.collection_route_planner_v2 import plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    BallStatus,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    RouteSegmentType,
    ScanSnapshot,
    SnapshotBall,
)

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_AUDIT = os.path.join(
    _ROOT, "runtime", "route_audit", "clean_current_20260728_1315",
    "collection-scan-39264000000.json",
)
_BOUNDARY = os.path.join(
    _ROOT, "runtime", "route_audit", "clean_current_20260728_1315", "court_boundary.json"
)
_CONFIG = os.path.join(_ROOT, "ros2_ws", "src", "tennis_robot", "config", "collection_route.yaml")
_CALIBRATION = os.path.join(
    _ROOT, "calibration_artifacts", "gazebo", "range_depth_quality_diagonal_v1-gazebo-v3.json"
)

_FLAT_BASELINE_LENGTH_M = 55.92
_FLAT_BASELINE_TURN_RAD = 15.21
_SHIPPED_EXPANSIONS = 600000

pytestmark = pytest.mark.skipif(
    not all(os.path.exists(path) for path in (_AUDIT, _BOUNDARY, _CONFIG, _CALIBRATION)),
    reason="recorded scan or shipped configuration absent",
)


def shipped_configuration():
    import yaml

    from tennis_robot.collection_route_config_builder import (
        build_collection_route_configuration,
    )

    with open(_CONFIG, encoding="utf-8") as handle:
        source = yaml.safe_load(handle)
    return build_collection_route_configuration(source, calibration_artifact_path=_CALIBRATION)


def recorded_scan(configuration):
    """The recorded ball positions, planned with today's configuration.

    The artifact's own configuration is two schema versions old; what is being
    replayed is the court and the perception output, not the settings.
    """
    with open(_AUDIT, encoding="utf-8") as handle:
        snapshot = json.load(handle)["snapshot"]
    balls = tuple(
        SnapshotBall(
            ball["ball_id"],
            Point2D(ball["position"]["x_m"], ball["position"]["y_m"]),
            ball["confidence"],
            PositionCovariance2D(
                ball["position_covariance"]["xx"],
                ball["position_covariance"]["xy"],
                ball["position_covariance"]["yy"],
            ),
        )
        for ball in snapshot["balls"]
    )
    pose = snapshot["robot_pose_at_scan"]
    return ScanSnapshot(
        snapshot["scan_id"], snapshot["scan_timestamp"], snapshot["map_frame"],
        Pose2D(pose["x_m"], pose["y_m"], pose["yaw_rad"]), balls, configuration,
    )


def court_model():
    from tennis_robot.collection_court_model_builder import build_court_model

    with open(_BOUNDARY, encoding="utf-8") as handle:
        return build_court_model(json.load(handle))


def total_turn(plan):
    total = 0.0
    for segment in plan.segments:
        points = segment.path.points
        for first, second in zip(points, points[1:]):
            total += abs(
                math.atan2(
                    math.sin(second.pose.yaw_rad - first.pose.yaw_rad),
                    math.cos(second.pose.yaw_rad - first.pose.yaw_rad),
                )
            )
    return total


_CACHE: dict[int, object] = {}


def plan_with(expansions, seconds=600.0):
    """Plan the recorded scan, reusing results: each full-budget run costs ~18 s."""
    if expansions in _CACHE:
        return _CACHE[expansions]
    configuration = shipped_configuration()
    configuration = replace(
        configuration,
        planning=replace(configuration.planning, maximum_planning_time_s=seconds),
        global_route_search=replace(
            configuration.global_route_search, max_search_expansions=expansions
        ),
    )
    _CACHE[expansions] = plan_collection_route(
        snapshot=recorded_scan(configuration), court=court_model(), configuration=configuration
    )
    return _CACHE[expansions]


def test_real_scan_route_beats_the_planner_it_replaces_on_length_and_turning():
    # The shipped expansion budget, with enough wall clock to spend it.
    # Measured: 45.03 m and 14.07 rad against 55.92 m and 15.21 rad.
    result = plan_with(expansions=_SHIPPED_EXPANSIONS)
    plan = result.plan
    covered = [item for item in plan.ball_results if item.status is BallStatus.COVERED]
    assert len(covered) == len(plan.ball_results) == 10
    assert plan.total_length_m <= _FLAT_BASELINE_LENGTH_M
    assert total_turn(plan) <= _FLAT_BASELINE_TURN_RAD
    assert not result.wall_clock_truncated


def test_real_scan_is_deterministic_at_the_shipped_budget():
    # Determinism survives only while the expansion budget, not the clock, is
    # what ends the run -- so this is also a guard on the budget being reachable.
    first = plan_with(expansions=_SHIPPED_EXPANSIONS)
    _CACHE.clear()
    second = plan_with(expansions=_SHIPPED_EXPANSIONS)
    assert first.plan.plan_id == second.plan.plan_id
    assert not second.wall_clock_truncated


def test_real_scan_quality_improves_monotonically_with_budget():
    # Over the objective, not over length: a shorter-turning route may be a few
    # centimetres longer and still cheaper.
    costs = []
    for expansions in (20000, 100000, 200000, 600000):
        result = plan_with(expansions=expansions)
        covered = sum(
            1 for item in result.plan.ball_results if item.status is BallStatus.COVERED
        )
        assert covered == 10
        costs.append(plan_objective_cost(result.plan, result.plan.configuration_snapshot))
    for earlier, later in zip(costs, costs[1:]):
        assert later <= earlier + 1e-6


def test_real_scan_route_collects_in_multi_ball_sweeps():
    plan = plan_with(expansions=_SHIPPED_EXPANSIONS).plan
    collecting = [segment for segment in plan.segments if segment.covered_ball_ids]
    assert any(len(segment.covered_ball_ids) > 1 for segment in collecting)
    # Every declared ball appears exactly once across the whole route.
    declared = [ball for segment in plan.segments for ball in segment.covered_ball_ids]
    assert len(declared) == len(set(declared)) == 10
    assert total_turn(plan) < _FLAT_BASELINE_TURN_RAD
    assert all(
        segment.type in (RouteSegmentType.FUNNEL_PASS, RouteSegmentType.CONNECTOR)
        for segment in collecting
    )
