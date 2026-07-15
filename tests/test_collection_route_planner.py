"""Tests for collection_route_planner — ordering, insertion, approach poses.

Pure geometry; no ROS. Court fixture: 20x20 m fence square centered on the
origin with the net running along x=0 (posts at y=±6).
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_route_planner import (
    ApproachPose,
    CourtModel,
    RoutePlannerConfig,
    RouteStop,
    _two_opt,
    approach_pose_for_ball,
    cheapest_insertion,
    order_route,
    remaining_route_length_m,
    route_polyline,
)

CFG = RoutePlannerConfig()


def _court() -> CourtModel:
    return CourtModel(
        fence_corners=[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)],
        net_segment=((0.0, -6.0), (0.0, 6.0)),
    )


def _path_length(start, ordered_ids, balls_by_id):
    total = 0.0
    cx, cy = start
    for bid in ordered_ids:
        bx, by = balls_by_id[bid]
        total += math.hypot(bx - cx, by - cy)
        cx, cy = bx, by
    return total


# ── order_route ────────────────────────────────────────────────────────────────


def test_order_route_visits_every_ball_once():
    balls = [(1, 1.0, 0.0), (2, 5.0, 2.0), (3, 3.0, -1.0), (4, 8.0, 0.5)]
    order = order_route((0.0, 0.0), balls, CFG)
    assert sorted(order) == [1, 2, 3, 4]


def test_order_route_empty():
    assert order_route((0.0, 0.0), [], CFG) == []


def test_two_opt_uncrosses_a_bad_tour():
    # S->P1->P2->P3->P4 zigzags with two long diagonals (total ≈ 8.47 m);
    # the optimal open path S->P2->P4->P3->P1 is ≈ 5.24 m.
    ordered = [(1, 2.0, 0.0), (2, 0.0, 1.0), (3, 2.0, 1.0), (4, 0.0, 2.0)]
    by_id = {i: (x, y) for i, x, y in ordered}
    improved = _two_opt((0.0, 0.0), list(ordered), max_passes=4)
    length = _path_length((0.0, 0.0), [b[0] for b in improved], by_id)
    assert length < 5.25


def test_two_opt_never_worse_than_greedy():
    import random

    rng = random.Random(7)
    balls = [(i, rng.uniform(-9, 9), rng.uniform(-9, 9)) for i in range(40)]
    by_id = {i: (x, y) for i, x, y in balls}
    start = (0.0, 0.0)
    greedy = order_route(start, balls, RoutePlannerConfig(two_opt=False))
    polished = order_route(start, balls, RoutePlannerConfig(two_opt=True))
    assert sorted(polished) == sorted(greedy)
    assert _path_length(start, polished, by_id) <= _path_length(start, greedy, by_id) + 1e-6


# ── cheapest_insertion ─────────────────────────────────────────────────────────


def test_insertion_between_closest_pair():
    route = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    index, delta = cheapest_insertion(route, (3.0, 0.1), start_index=1)
    assert index == 2
    assert delta < 0.2


def test_insertion_respects_start_index():
    route = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    # Best slot would be index 1, but the leg in progress is locked.
    index, delta = cheapest_insertion(route, (1.0, 0.1), start_index=2)
    assert index >= 2


def test_insertion_appends_when_cheapest():
    route = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    index, delta = cheapest_insertion(route, (-1.0, 0.0), start_index=2)
    assert index == 3  # append: 5.0 beats the detour of 6.0 via the middle
    assert delta == 5.0


# ── approach poses ─────────────────────────────────────────────────────────────


def test_interior_ball_direct_approach():
    pose = approach_pose_for_ball((3.0, 0.0), (0.0, 0.0), _court(), CFG)
    assert pose.mode == "direct" and pose.risk == "normal"
    assert math.isclose(pose.x_m, 3.0 - CFG.standoff_m, abs_tol=1e-6)
    assert math.isclose(pose.y_m, 0.0, abs_tol=1e-6)
    assert math.isclose(pose.yaw_rad, 0.0, abs_tol=1e-6)


def test_no_court_model_degrades_to_direct():
    pose = approach_pose_for_ball((3.0, 0.0), (0.0, 0.0), None, CFG)
    assert pose.mode == "direct"


def test_fence_ball_gets_lateral_approach():
    court = _court()
    ball = (9.7, 0.0)  # 0.3 m from the east fence (tangent (0, 1))
    pose = approach_pose_for_ball(ball, (0.0, 0.0), court, CFG)
    assert pose.mode == "lateral" and pose.risk == "net_wall"
    heading = (math.cos(pose.yaw_rad), math.sin(pose.yaw_rad))
    assert abs(heading[1]) > math.cos(math.radians(30))  # ≈ parallel to fence
    # Standoff must be drivable: robot radius off the fence.
    assert court.fence_distance(pose.x_m, pose.y_m) >= CFG.robot_radius_m - 1e-6
    assert math.isclose(
        math.hypot(pose.x_m - ball[0], pose.y_m - ball[1]), CFG.standoff_m, abs_tol=1e-6
    )


def test_net_ball_gets_lateral_approach():
    court = _court()
    ball = (0.5, 0.0)  # 0.5 m from the net (tangent (0, 1))
    pose = approach_pose_for_ball(ball, (5.0, 0.0), court, CFG)
    assert pose.mode == "lateral" and pose.risk == "net_wall"
    heading = (math.cos(pose.yaw_rad), math.sin(pose.yaw_rad))
    assert abs(heading[1]) > math.cos(math.radians(30))  # ≈ parallel to net
    assert court.net_distance(pose.x_m, pose.y_m) >= CFG.robot_radius_m - 1e-6
    # Standoff stays on the ball's side of the net.
    assert pose.x_m > 0.0


def test_cornered_ball_still_returns_a_pose():
    pose = approach_pose_for_ball((9.8, 9.8), (0.0, 0.0), _court(), CFG)
    assert isinstance(pose, ApproachPose)
    assert pose.mode == "lateral"


# ── CourtModel ─────────────────────────────────────────────────────────────────


def test_court_model_queries():
    court = _court()
    assert court.pose_is_free(3.0, 3.0, 0.36)
    assert not court.pose_is_free(11.0, 0.0, 0.36)   # outside fence
    assert not court.pose_is_free(0.1, 0.0, 0.36)    # on the net
    assert court.ball_risk(0.0, 8.0, 0.9) == "normal"  # past the net posts
    assert court.ball_risk(3.0, 0.0, 0.9) == "normal"
    assert court.ball_risk(9.5, 0.0, 0.9) == "net_wall"
    dist, tangent = court.nearest_boundary(9.0, 0.0)
    assert math.isclose(dist, 1.0, abs_tol=1e-6)
    assert abs(tangent[1]) == 1.0


def test_court_model_from_v2_data():
    data = {
        "fence": {
            "corners": [
                {"x_m": -8.6, "y_m": -8.7},
                {"x_m": 24.5, "y_m": -8.6},
                {"x_m": 24.5, "y_m": 8.8},
                {"x_m": -8.6, "y_m": 8.7},
            ]
        },
        "net": {
            "center": {"x_m": 8.0, "y_m": 0.0},
            "axis_width": {"x_m": 0.0, "y_m": 1.0},
            "span_m": 11.3,
            "posts": [{"x_m": 8.0, "y_m": -5.65}, {"x_m": 8.0, "y_m": 5.65}],
        },
        "obstacles": [
            {"center": {"x_m": -8.2, "y_m": 8.3}, "size_m": {"w": 0.5, "h": 0.3}},
        ],
    }
    court = CourtModel.from_boundary_data(data)
    assert court.net_distance(8.0, 1.0) < 0.01
    assert court.ball_risk(-8.0, 8.2, 0.5) == "obstacle"
    assert court.pose_is_free(5.0, 0.0, 0.36)


def test_same_side_uses_real_net_line():
    # Net at map x=8 (run-3 world): the legacy across_net(net_x=0) convention
    # calls (0,0) and (12,0) "same side" (both x>0) — the real line says no.
    court = CourtModel(
        fence_corners=[(-9.0, -9.0), (25.0, -9.0), (25.0, 9.0), (-9.0, 9.0)],
        net_segment=((8.0, -6.0), (8.0, 6.0)),
    )
    assert court.same_side(0.0, 0.0, 3.0, 2.0) is True
    assert court.same_side(0.0, 0.0, 12.0, 0.0) is False
    assert court.same_side(12.0, 1.0, 15.0, -2.0) is True
    # Points hugging the net line count as same side (clearance band).
    assert court.same_side(7.9, 0.0, 8.2, 0.0) is True
    assert court.contains(3.0, 0.0) is True
    assert court.contains(30.0, 0.0) is False


def test_missing_boundary_file_returns_none():
    from pathlib import Path

    assert CourtModel.from_boundary_file(Path("/nonexistent/court.json")) is None


# ── console export ─────────────────────────────────────────────────────────────


def _stop(ball_id, bx, by, ax, ay, order, status="pending"):
    return RouteStop(
        ball_id=ball_id,
        ball_x_m=bx,
        ball_y_m=by,
        approach=ApproachPose(ax, ay, 0.0, "direct", "normal"),
        order=order,
        status=status,
    )


def test_route_polyline_skips_terminal_stops():
    stops = [
        _stop(1, 2.0, 0.0, 1.0, 0.0, 1, status="collected"),
        _stop(2, 4.0, 0.0, 3.0, 0.0, 2),
        _stop(3, 6.0, 0.0, 5.0, 0.0, 3, status="skipped"),
        _stop(4, 8.0, 0.0, 7.0, 0.0, 4),
    ]
    line = route_polyline((0.0, 0.0), stops)
    xs = [p["x_m"] for p in line]
    assert xs == [0.0, 3.0, 4.0, 7.0, 8.0]
    assert remaining_route_length_m((0.0, 0.0), stops) == 8.0
