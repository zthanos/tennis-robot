"""Tests for CollectRouteMission — scan/plan/nav/approach transitions and
dynamic cheapest insertion. Pure Python (no rclpy); BallMap is seeded directly
like tests/test_ball_map_console_export.py.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.ball_map import BallMap, BallMapConfig, MappedBall
from tennis_robot.collect_route_mission import CollectRouteMission
from tennis_robot.collection_route_planner import RoutePlannerConfig
from tennis_robot.collector import (
    BallObservationInput,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptAConfig,
)

NOW = 100.0
NO_BALL = BallObservationInput(visible=False, source="test")


def _ball_map(*positions, min_seen=3):
    m = BallMap(BallMapConfig(min_seen_count=min_seen, stale_after_s=45.0))
    for i, (x, y) in enumerate(positions, start=1):
        m.balls[i] = MappedBall(
            i, x, y, 0.9, 0.0, NOW, "oak_depth", seen_count=min_seen + 1, state="detected"
        )
    m._next_id = len(positions) + 1
    return m


def _mission():
    return CollectRouteMission(RoutePlannerConfig(two_opt=True))


def _tick(mission, ball_map, pose=(0.0, 0.0, 0.0), nav_state="idle",
          observation=NO_BALL, confirmed=False, now=NOW, behavior=None, dt=0.032):
    behavior = behavior or ConceptACollectorBehavior(ConceptAConfig())
    return mission.update(
        observation, confirmed, dt, pose, behavior, ball_map, now, nav_state, None
    )


def _run_scan(mission, ball_map, start_now=NOW):
    """Drive the 360° step-rotation to completion with a simulated yaw."""
    yaw = 0.0
    now = start_now
    for _ in range(200):
        if mission.phase != "scan":
            return now
        cmd = _tick(mission, ball_map, pose=(0.0, 0.0, yaw), now=now)
        if abs(cmd.base.angular_speed_rad_s) > 0.0:
            yaw = mission._scan_target_yaw  # jump straight to the step target
        now += 0.3  # always past the settle window
    raise AssertionError("scan did not complete")


# ── scan & plan ────────────────────────────────────────────────────────────────


def test_scan_completes_full_rotation_then_plans():
    mission = _mission()
    ball_map = _ball_map((3.0, 0.0), (5.0, 1.0))
    mission.start((0.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    assert mission._scan_steps_taken == 12
    # First post-scan tick runs PLAN and enters the first nav leg.
    _tick(mission, ball_map, now=now)
    assert mission.phase == "nav"
    assert mission.nav_goal is not None
    assert len(mission.stops) == 2
    assert [s.order for s in mission.stops] == [1, 2]
    events = dict(mission.drain_events())
    assert "route_planned" in events and events["route_planned"]["stops"] == 2


def test_no_balls_after_scan_finishes_done():
    mission = _mission()
    ball_map = _ball_map()
    mission.start((0.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    _tick(mission, ball_map, now=now)
    assert mission.is_done


def test_plan_orders_by_route_not_id():
    mission = _mission()
    # id 1 far, id 2 near: route order must start with the near one.
    ball_map = _ball_map((8.0, 0.0), (2.0, 0.0))
    mission.start((0.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    _tick(mission, ball_map, now=now)
    assert [s.ball_id for s in mission.stops] == [2, 1]


def test_across_net_balls_excluded_from_plan():
    mission = _mission()
    ball_map = _ball_map((-3.0, 0.0), (4.0, 0.0))  # robot at x=-6: +4 is across
    mission.start((-6.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    _tick(mission, ball_map, pose=(-6.0, 0.0, 0.0), now=now)
    assert [s.ball_id for s in mission.stops] == [1]


# ── nav leg ────────────────────────────────────────────────────────────────────


def _mission_in_nav(ball_positions=((3.0, 0.0),)):
    mission = _mission()
    ball_map = _ball_map(*ball_positions)
    mission.start((0.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    _tick(mission, ball_map, now=now)
    assert mission.phase == "nav"
    mission.drain_events()
    return mission, ball_map, now


def test_nav_reached_enters_fine_approach():
    mission, ball_map, now = _mission_in_nav()
    _tick(mission, ball_map, nav_state="pending", now=now)
    _tick(mission, ball_map, nav_state="reached", now=now)
    assert mission.phase == "approach"
    assert mission.nav_goal is None
    assert mission._locked_world == (3.0, 0.0)
    assert mission.current_ball_id == 1


def test_nav_failures_skip_after_retries():
    mission, ball_map, now = _mission_in_nav()
    # Each failure needs a fresh transition (idle → failed).
    for _ in range(4):
        if mission.phase != "nav":
            break
        _tick(mission, ball_map, nav_state="active", now=now)
        _tick(mission, ball_map, nav_state="failed", now=now)
        _tick(mission, ball_map, nav_state="idle", now=now)  # cancel + re-issue
    assert mission.is_done
    assert mission.stops[0].status == "skipped"
    assert ball_map.balls[1].state == "collection_failed"


def test_nav_unavailable_blocks_loudly():
    mission, ball_map, now = _mission_in_nav()
    _tick(mission, ball_map, nav_state="unavailable", now=now)
    assert mission.phase == "nav"
    assert mission.current_blocker == "nav2_action_unavailable"


# ── fine approach / capture ────────────────────────────────────────────────────


def _mission_in_approach(ball_positions=((3.0, 0.0), (6.0, 0.0))):
    mission, ball_map, now = _mission_in_nav(ball_positions)
    behavior = ConceptACollectorBehavior(ConceptAConfig())
    _tick(mission, ball_map, nav_state="pending", now=now, behavior=behavior)
    _tick(mission, ball_map, nav_state="reached", now=now, behavior=behavior)
    assert mission.phase == "approach"
    return mission, ball_map, behavior, now


def test_collection_advances_to_next_stop():
    mission, ball_map, behavior, now = _mission_in_approach()
    cmd = _tick(mission, ball_map, confirmed=True, now=now, behavior=behavior)
    assert cmd.state == CollectorState.COLLECTED
    assert mission.stops[0].status == "collected"
    assert mission.phase == "settle"
    for _ in range(10):  # 10 × 0.25 s > settle hold
        _tick(mission, ball_map, now=now, behavior=behavior, dt=0.25)
    assert mission.phase == "nav"
    assert mission.current_ball_id == 2
    assert mission.stops[1].status == "active"


def test_last_ball_collection_completes_mission():
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    _tick(mission, ball_map, confirmed=True, now=now, behavior=behavior)
    for _ in range(10):
        _tick(mission, ball_map, now=now, behavior=behavior, dt=0.25)
    assert mission.is_done
    events = [e for e, _ in mission.drain_events()]
    assert "route_complete" in events


def test_approach_tracks_locked_ball():
    mission, ball_map, behavior, now = _mission_in_approach()
    # Robot 1.3 m before the ball, facing it: mission feeds the locked obs.
    cmd = _tick(
        mission, ball_map, pose=(1.7, 0.0, 0.0), now=now, behavior=behavior
    )
    assert behavior.state in (CollectorState.ALIGN, CollectorState.APPROACH, CollectorState.CAPTURE)
    assert cmd.base.linear_speed_m_s >= 0.0


def test_missing_ball_marks_missing_after_scan_budget():
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    # The mapped entry vanishes (e.g. pruned) → lock cannot be built.
    mission._locked_world = None
    for _ in range(30):  # 30 × 0.25 s = 7.5 s > MISSING_SCAN_S
        if mission.phase != "approach":
            break
        _tick(mission, ball_map, now=now, behavior=behavior, dt=0.25)
    assert mission.stops[0].status == "missing"
    assert mission.is_done


# ── dynamic insertion ──────────────────────────────────────────────────────────


def test_new_ball_inserted_at_cheapest_slot():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0), (8.0, 0.0))
    )
    order_before = [s.ball_id for s in mission.stops]
    assert order_before == [1, 2, 3]
    # New confirmed ball between stops 2 and 3.
    ball_map.balls[9] = MappedBall(
        9, 6.0, 0.2, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)
    ids = [s.ball_id for s in mission.stops]
    assert ids == [1, 2, 9, 3]
    assert mission.insertion_count == 1
    # The in-progress leg (stop 1) is untouched and still active.
    assert mission.stops[0].status == "active" and mission.current_index == 0
    events = dict(mission.drain_events())
    assert "route_insertion" in events


def test_far_detour_ball_appended_at_end():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0))
    )
    ball_map.balls[9] = MappedBall(
        9, 3.0, 9.0, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)
    assert [s.ball_id for s in mission.stops] == [1, 2, 9]


def test_route_export_orders_and_polyline():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0))
    )
    polyline, planned_order = mission.route_export((0.0, 0.0))
    assert planned_order == {1: 1, 2: 2}
    assert polyline[0] == {"x_m": 0.0, "y_m": 0.0}
    assert len(polyline) == 5  # robot + (approach, ball) × 2
