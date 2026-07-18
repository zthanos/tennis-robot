"""Tests for CollectRouteMission — scan/plan/nav/approach transitions.

Pure Python (no rclpy); BallMap is seeded directly like
tests/test_ball_map_console_export.py.
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


def _mission(*, freeze_initial_plan=True, sweep=False):
    return CollectRouteMission(
        RoutePlannerConfig(two_opt=True),
        freeze_initial_plan=freeze_initial_plan,
        sweep=sweep,
    )


def test_runtime_default_is_fixed_drive_through_route(monkeypatch):
    """The deployed mission must not be switched back by an env variable."""
    monkeypatch.setenv("COLLECT_ROUTE_SWEEP", "false")
    mission = CollectRouteMission(RoutePlannerConfig(two_opt=True))
    assert mission.sweep is True


def _tick(mission, ball_map, pose=(0.0, 0.0, 0.0), nav_state="idle",
          observation=NO_BALL, confirmed=False, confirmed_ball_id=None,
          capture_pending_ball_id=None, now=NOW, behavior=None, dt=0.032):
    behavior = behavior or ConceptACollectorBehavior(ConceptAConfig())
    return mission.update(
        observation, confirmed, dt, pose, behavior, ball_map, now, nav_state, None,
        confirmed_ball_id,
        capture_pending_ball_id,
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


def _mission_in_nav_with_insertion(ball_positions=((3.0, 0.0),)):
    mission = _mission(freeze_initial_plan=False)
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
    _tick(mission, ball_map, nav_state="active", now=now, dt=3.0)
    cmd = _tick(mission, ball_map, nav_state="failed", now=now, dt=0.1)
    assert mission.is_done
    assert cmd.base.linear_speed_m_s == 0.0
    assert mission.stops[0].status == "skipped"
    assert ball_map.balls[1].state == "collection_failed"


def test_instant_nav_failure_skips_without_reverse_and_advances():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0))
    )
    _tick(mission, ball_map, nav_state="pending", now=now)
    cmd = _tick(mission, ball_map, nav_state="failed", now=now)  # instant fail
    assert mission.phase == "nav"
    assert cmd.base.linear_speed_m_s == 0.0
    assert mission.nav_goal is not None
    assert mission.stops[0].status == "skipped"
    assert mission.current_ball_id == 2
    # The previous action's failed result may linger until the new goal is
    # accepted. It must not cascade-skip ball 2.
    _tick(mission, ball_map, nav_state="failed", now=now + 0.1)
    assert mission.current_ball_id == 2
    assert mission.stops[1].status == "active"
    events = [t for t, _ in mission.drain_events()]
    assert "route_nav_recovery" not in events
    assert "route_leg_skip" in events
    assert "route_advance" in events


def test_persistent_nav_failures_walk_the_whole_plan():
    # User decision (log #13): a nav collapse records the failed stop and the
    # SAME plan continues from the next ball — completion only when every
    # planned ball is accounted for (collected or failed).
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.5))
    )
    for _ in range(120):
        if mission.is_done:
            break
        _tick(mission, ball_map, nav_state="active", now=now, dt=3.0)
        _tick(mission, ball_map, nav_state="failed", now=now, dt=0.1)
        _tick(mission, ball_map, nav_state="idle", now=now, dt=0.1)
    assert mission.is_done
    statuses = [s.status for s in mission.stops]
    assert statuses.count("skipped") == 4      # every stop accounted for
    assert "pending" not in statuses
    events = dict(mission.drain_events())
    complete = events.get("route_complete", {})
    assert complete.get("planned_total") == 4
    assert sorted(complete.get("failed_ball_ids", [])) == sorted(
        s.ball_id for s in mission.stops
    )


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


def test_ball_behind_turns_toward_it_instead_of_blind_scan():
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    # Robot at the standoff but facing AWAY from the ball (loose Nav2 yaw).
    cmd = _tick(
        mission, ball_map, pose=(1.7, 0.0, math.pi), now=now, behavior=behavior
    )
    assert cmd.base.linear_speed_m_s == 0.0
    assert cmd.base.angular_speed_rad_s != 0.0  # direct turn, not behavior scan
    assert behavior.state == CollectorState.SCAN  # behavior not engaged yet
    # Once facing the ball, the behavior takes over.
    _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now, behavior=behavior)
    assert behavior.state in (CollectorState.ALIGN, CollectorState.APPROACH, CollectorState.CAPTURE)


def test_other_visible_ball_does_not_steal_tracking():
    mission, ball_map, behavior, now = _mission_in_approach(
        ball_positions=((3.0, 0.0), (6.0, 0.0))
    )
    # A DIFFERENT ball (far from the locked target) is visible to the camera.
    other = BallObservationInput(
        visible=True, bearing_rad=0.5, distance_m=3.2, confidence=0.9,
        source="oak_ai_depth", world_x_m=6.0, world_y_m=0.0,
    )
    _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now,
          behavior=behavior, observation=other)
    assert mission._locked_world == (3.0, 0.0)  # lock not hijacked


def test_timeout_fires_even_while_target_is_behind():
    # Run-3 regression: the turn-toward-target branch returned before the
    # timeout check, so a blocked approach never expired. Live sightings are
    # present (not a phantom) but the robot never gets to face the ball.
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    live = BallObservationInput(
        visible=True, bearing_rad=3.1, distance_m=1.3, confidence=0.9,
        source="oak_ai_depth", world_x_m=3.0, world_y_m=0.05,
    )
    for _ in range(45):
        if mission.phase != "approach":
            break
        _tick(mission, ball_map, pose=(1.7, 0.0, math.pi), now=now,
              behavior=behavior, dt=1.0, observation=live)
    assert mission.phase != "approach"  # retry (nav) or skip — not stuck
    assert mission.stops[0].attempts >= 1
    assert mission.stops[0].status != "missing"


def test_phantom_without_live_sighting_goes_missing_fast():
    # No live camera sighting from the 1.3 m standoff => map entry is a
    # phantom; must be dropped in ~MISSING_SCAN_S, not dead-reckoned into
    # whatever stands there (run 3 pushed the net for 78 s).
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    for _ in range(20):
        if mission.phase != "approach":
            break
        _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now,
              behavior=behavior, dt=0.5)
    assert mission.stops[0].status == "missing"
    events = dict(mission.drain_events())
    assert events.get("route_ball_missing", {}).get("reason") == "no_live_sighting_at_standoff"


def test_onboard_capture_is_not_marked_missing_before_bin_retention():
    mission, ball_map, behavior, now = _mission_in_approach(
        ball_positions=((3.0, 0.0),)
    )
    for _ in range(20):
        _tick(
            mission,
            ball_map,
            pose=(1.7, 0.0, 0.0),
            now=now,
            behavior=behavior,
            dt=0.5,
            capture_pending_ball_id=1,
        )

    assert mission.phase == "approach"
    assert mission.stops[0].status == "active"
    events = dict(mission.drain_events())
    assert events["route_missing_deferred"]["reason"] == "onboard_capture_pending_retention"

    _tick(
        mission,
        ball_map,
        confirmed=True,
        confirmed_ball_id=1,
        capture_pending_ball_id=1,
        now=now,
        behavior=behavior,
    )
    assert mission.phase == "settle"
    assert mission.stops[0].status == "collected"


def test_capture_that_leaves_robot_no_longer_blocks_missing_decision():
    mission, ball_map, behavior, now = _mission_in_approach(
        ball_positions=((3.0, 0.0),)
    )
    for _ in range(14):
        _tick(
            mission,
            ball_map,
            pose=(1.7, 0.0, 0.0),
            now=now,
            behavior=behavior,
            dt=0.5,
            capture_pending_ball_id=1,
        )

    assert mission.stops[0].status == "active"
    _tick(
        mission,
        ball_map,
        pose=(1.7, 0.0, 0.0),
        now=now,
        behavior=behavior,
        dt=0.5,
        capture_pending_ball_id=None,
    )
    assert mission.stops[0].status == "missing"


def test_first_sighting_adopts_mislocated_scan_entry():
    # Scan-created map entries carry up to ~0.5 m error (run-3 lock_error_m);
    # the FIRST sighting within 1.0 m of the plan re-centres the lock, while
    # later refreshes keep the strict 0.6 m anti-steal gate.
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    off_plan = BallObservationInput(
        visible=True, bearing_rad=0.3, distance_m=1.5, confidence=0.9,
        source="oak_ai_depth", world_x_m=3.5, world_y_m=0.7,  # 0.86 m from lock
    )
    _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now,
          behavior=behavior, observation=off_plan)
    assert mission._locked_world == (3.5, 0.7)
    # A different ball 0.9 m away can no longer steal the refreshed lock.
    thief = BallObservationInput(
        visible=True, bearing_rad=-0.4, distance_m=2.0, confidence=0.9,
        source="oak_ai_depth", world_x_m=4.3, world_y_m=1.2,
    )
    _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now,
          behavior=behavior, observation=thief)
    assert mission._locked_world == (3.5, 0.7)


def test_live_sighting_keeps_approach_alive_past_missing_budget():
    mission, ball_map, behavior, now = _mission_in_approach(ball_positions=((3.0, 0.0),))
    live = BallObservationInput(
        visible=True, bearing_rad=0.0, distance_m=1.3, confidence=0.9,
        source="oak_ai_depth", world_x_m=3.0, world_y_m=0.05,
    )
    for _ in range(16):  # 8 s > MISSING_SCAN_S with sightings present
        _tick(mission, ball_map, pose=(1.7, 0.0, 0.0), now=now,
              behavior=behavior, dt=0.5, observation=live)
    assert mission.phase == "approach"
    assert mission.stops[0].status == "active"


def test_plan_uses_real_net_line_for_side_filter():
    from tennis_robot.collection_route_planner import CourtModel

    court = CourtModel(
        fence_corners=[(-9.0, -9.0), (25.0, -9.0), (25.0, 9.0), (-9.0, 9.0)],
        net_segment=((8.0, -6.0), (8.0, 6.0)),
    )
    mission = _mission()
    # Both balls have x>0: the legacy net_x=0 filter calls both same-side;
    # the real net at x=8 excludes the far-court one.
    ball_map = _ball_map((3.0, 0.0), (12.0, 0.0))
    mission.start((0.0, 0.0, 0.0))
    yaw, now = 0.0, NOW
    behavior = ConceptACollectorBehavior(ConceptAConfig())
    for _ in range(200):
        if mission.phase != "scan":
            break
        cmd = mission.update(NO_BALL, False, 0.032, (0.0, 0.0, yaw),
                             behavior, ball_map, now, "idle", court)
        if abs(cmd.base.angular_speed_rad_s) > 0.0:
            yaw = mission._scan_target_yaw
        now += 0.3
    mission.update(NO_BALL, False, 0.032, (0.0, 0.0, yaw),
                   behavior, ball_map, now, "idle", court)
    assert [s.ball_id for s in mission.stops] == [1]


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
    mission, ball_map, now = _mission_in_nav_with_insertion(
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
    mission, ball_map, now = _mission_in_nav_with_insertion(
        ball_positions=((2.0, 0.0), (4.0, 0.0))
    )
    ball_map.balls[9] = MappedBall(
        9, 3.0, 9.0, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)
    assert [s.ball_id for s in mission.stops] == [1, 2, 9]


def test_default_route_ignores_new_balls_after_initial_plan():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0), (8.0, 0.0))
    )
    ball_map.balls[9] = MappedBall(
        9, 6.0, 0.2, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)
    assert [s.ball_id for s in mission.stops] == [1, 2, 3]
    assert mission.insertion_count == 0
    assert "route_insertion" not in [event for event, _ in mission.drain_events()]


def test_frozen_route_keeps_scan_time_goal_when_map_entry_drifts():
    mission, ball_map, now = _mission_in_nav(ball_positions=((3.0, 0.0),))
    old_goal = mission.nav_goal
    old_position = (mission.stops[0].ball_x_m, mission.stops[0].ball_y_m)
    ball_map.balls[1].x_m, ball_map.balls[1].y_m = 3.9, 0.8  # drift 1.2 m
    _tick(mission, ball_map, nav_state="active", now=now)
    stop = mission.stops[0]
    assert (stop.ball_x_m, stop.ball_y_m) == old_position
    assert mission.nav_goal == old_goal
    events = dict(mission.drain_events())
    assert "route_goal_updated" not in events


def test_dynamic_route_goal_follows_drifted_map_entry():
    mission, ball_map, now = _mission_in_nav_with_insertion(
        ball_positions=((3.0, 0.0),)
    )
    old_goal = mission.nav_goal
    ball_map.balls[1].x_m, ball_map.balls[1].y_m = 3.9, 0.8
    _tick(mission, ball_map, nav_state="active", now=now)
    stop = mission.stops[0]
    assert (stop.ball_x_m, stop.ball_y_m) == (3.9, 0.8)
    assert mission.nav_goal != old_goal
    assert "route_goal_updated" in dict(mission.drain_events())


def test_dynamic_route_abandons_wandering_map_entry():
    # Run-4 stop 6: chain-merges dragged the entry 4+ m; the mission must
    # drop the stop instead of following the wandering entry across court.
    mission, ball_map, now = _mission_in_nav_with_insertion(
        ball_positions=((3.0, 0.0),)
    )
    ball_map.balls[1].x_m, ball_map.balls[1].y_m = 3.5, 4.0  # 4 m from plan
    _tick(mission, ball_map, nav_state="active", now=now)
    assert mission.stops[0].status == "missing"
    events = dict(mission.drain_events())
    assert events.get("route_ball_lost", {}).get("reason") == "map_entry_drifted"


def test_current_target_xy_reports_pursued_ball():
    mission, ball_map, now = _mission_in_nav(ball_positions=((3.0, 0.0),))
    assert mission.current_target_xy is None  # nav legs watch the road ahead
    _tick(mission, ball_map, nav_state="pending", now=now)
    _tick(mission, ball_map, nav_state="reached", now=now)
    assert mission.phase == "approach"
    assert mission.current_target_xy == mission._locked_world


def _front_ball_obs(x=1.0, y=0.1):
    return BallObservationInput(
        visible=True, bearing_rad=math.atan2(y, x), distance_m=math.hypot(x, y),
        confidence=0.9, source="oak_ai_depth", world_x_m=x, world_y_m=y,
    )


def test_planned_ball_ahead_on_leg_is_promoted_to_next_stop():
    # Route rule R2 (no chase capture): a PLANNED ball visible ahead becomes
    # the NEXT stop with a normal standoff + straight fine approach — the old
    # chase punted balls away (run 9) and bounced launches out (run 10).
    mission, ball_map, now = _mission_in_nav_with_insertion(
        ball_positions=((6.0, 0.0),)
    )
    # A confirmed new ball joins the plan via insertion, then shows up ahead.
    ball_map.balls[9] = MappedBall(
        9, 1.0, 0.1, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)  # insertion tick
    assert 9 in {s.ball_id for s in mission.stops}
    _tick(mission, ball_map, nav_state="active", now=now,
          observation=_front_ball_obs(1.0, 0.1))
    assert mission.phase == "nav"  # no chase: a proper leg to its standoff
    assert mission.current_ball_id == 9
    assert mission.nav_goal is not None
    postponed = [s for s in mission.stops if s.ball_id == 1]
    assert postponed[0].status == "pending"
    events = dict(mission.drain_events())
    assert events["route_on_path_promoted"]["ball_id"] == 9
    assert events["route_on_path_promoted"]["postponed_stop"] == 1


def test_unplanned_ball_ahead_never_diverts_the_leg():
    # Plan-only rule (user decision, log #13): an unconfirmed stray sighting
    # ahead does not hijack the leg.
    mission, ball_map, now = _mission_in_nav(ball_positions=((6.0, 0.0),))
    _tick(mission, ball_map, nav_state="active", now=now,
          observation=_front_ball_obs(1.0, 0.1))  # no matching plan ball
    assert mission.phase == "nav"
    assert mission.nav_goal is not None


def test_current_stop_ball_ahead_keeps_the_leg():
    # Seeing the CURRENT leg's own ball ahead is the normal end of the leg:
    # Nav2 is already driving to its standoff — no promotion, no goal churn.
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((5.0, 0.0), (1.1, 0.05))
    )
    goal_before = mission.nav_goal
    _tick(mission, ball_map, nav_state="active", now=now,
          observation=_front_ball_obs(1.1, 0.05))
    assert mission.phase == "nav"
    assert mission.nav_goal == goal_before
    assert "route_on_path_promoted" not in [t for t, _ in mission.drain_events()]


def test_delayed_retention_is_credited_to_original_route_stop():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((3.0, 0.0), (6.0, 0.0))
    )
    # The first ball entered the receiver, later rolled out, and its stop was
    # finalized missing. It finally settles while the second stop is active.
    mission.stops[0].status = "missing"
    mission.stops[1].status = "active"
    mission.current_index = 1
    mission.phase = "opportunistic"
    mission._opp_stop_id = 2
    mission._opp_locked = (6.0, 0.0)

    _tick(
        mission,
        ball_map,
        confirmed=True,
        confirmed_ball_id=1,
        now=now,
    )

    assert mission.stops[0].status == "collected"
    assert mission.stops[1].status == "active"
    # The in-progress opportunistic capture of ball 2 must NOT be aborted by
    # the delayed credit of ball 1: the lock and phase stay put.
    assert mission.phase == "opportunistic"
    assert mission._opp_locked == (6.0, 0.0)
    assert ball_map.balls[1].state == "collected"
    event = dict(mission.drain_events())["route_opportunistic_collected"]
    assert event["ball_id"] == 1
    assert event["resumed_stop"] == 2
    assert event["delayed_attribution"] is True


def test_nav_phase_delayed_retention_credits_current_stop_and_skips_leg():
    # Run-8 regression: a ball launched at the tail of an aborted capture
    # settled into the bin AFTER the leg to its ghost standoff had started;
    # the credit must land on the ledger and the ghost leg must be skipped.
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((3.0, 0.0), (6.0, 0.0))
    )
    _tick(
        mission,
        ball_map,
        nav_state="active",
        now=now,
        confirmed=True,
        confirmed_ball_id=1,
    )
    assert mission.stops[0].status == "collected"
    assert ball_map.balls[1].state == "collected"
    assert mission.current_ball_id == 2
    assert mission.phase == "nav"
    events = dict(mission.drain_events())
    assert events["route_delayed_collection_attributed"]["phase"] == "nav"
    assert "route_advance" in events


def test_nav_phase_delayed_retention_for_other_stop_keeps_current_leg():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((3.0, 0.0), (6.0, 0.0))
    )
    _tick(
        mission,
        ball_map,
        nav_state="active",
        now=now,
        confirmed=True,
        confirmed_ball_id=2,
    )
    assert mission.stops[1].status == "collected"
    assert mission.current_ball_id == 1
    assert mission.phase == "nav"
    # An ownerless confirm (no ground-truth attribution) must NOT credit the
    # untouched current stop.
    _tick(mission, ball_map, nav_state="active", now=now, confirmed=True)
    assert mission.stops[0].status == "active"


def test_promotion_does_not_churn_while_ball_stays_visible():
    # After promotion the on-path ball IS the current stop; seeing it ahead
    # on subsequent ticks must not re-promote or reset the leg.
    mission, ball_map, now = _mission_in_nav_with_insertion(
        ball_positions=((6.0, 0.0),)
    )
    ball_map.balls[9] = MappedBall(
        9, 1.0, 0.0, 0.9, 0.0, NOW, "oak_depth", seen_count=4, state="detected"
    )
    _tick(mission, ball_map, nav_state="active", now=now)  # insertion tick
    _tick(mission, ball_map, nav_state="active", now=now,
          observation=_front_ball_obs())
    assert mission.current_ball_id == 9
    goal = mission.nav_goal
    mission.drain_events()
    for _ in range(5):
        _tick(mission, ball_map, nav_state="active", now=now,
              observation=_front_ball_obs())
    assert mission.current_ball_id == 9
    assert mission.nav_goal == goal
    assert "route_on_path_promoted" not in [t for t, _ in mission.drain_events()]


def test_route_export_orders_and_polyline():
    mission, ball_map, now = _mission_in_nav(
        ball_positions=((2.0, 0.0), (4.0, 0.0))
    )
    polyline, planned_order = mission.route_export((0.0, 0.0))
    assert planned_order == {1: 1, 2: 2}
    assert polyline[0] == {"x_m": 0.0, "y_m": 0.0}
    assert len(polyline) == 5  # robot + (approach, ball) × 2


# ── Sweep mode (log #21: collection decoupled from the route) ──────────────────


def _sweep_mission_in_drive(ball_positions=((3.0, 0.0), (6.0, 0.0))):
    mission = _mission(sweep=True)
    ball_map = _ball_map(*ball_positions)
    mission.start((0.0, 0.0, 0.0))
    now = _run_scan(mission, ball_map)
    _tick(mission, ball_map, now=now)  # plan → drive
    assert mission.phase == "nav"
    mission.drain_events()
    return mission, ball_map, now


def test_sweep_goals_are_entry_poses_before_the_ball():
    # Nav2 drives only to the run-in ENTRY; the crossing is mission-driven
    # (Nav2 corrections near the ball slapped it away with the cheeks).
    mission, ball_map, now = _sweep_mission_in_drive()
    goal = mission.nav_goal
    assert goal is not None and abs(goal[0] - 2.0) < 1e-6  # 1.0 m before ball 1
    cmd = _tick(mission, ball_map, nav_state="active", now=now)
    assert cmd.collector.intake_enabled  # intake on for the whole route


def test_sweep_pass_is_straight_and_advances_without_stopping():
    mission, ball_map, now = _sweep_mission_in_drive()
    _tick(mission, ball_map, nav_state="active", now=now)
    # Entry reached: the mission takes the base and drives the crossing.
    cmd = _tick(mission, ball_map, pose=(2.0, 0.0, 0.0), nav_state="reached", now=now)
    assert mission.nav_goal is None  # Nav2 cancelled for the pass
    assert cmd.base.linear_speed_m_s > 0.0
    assert cmd.base.angular_speed_rad_s == 0.0  # heading frozen near the ball
    assert cmd.collector.intake_enabled
    # Past the ball + overrun: link to the next run-in remains mission-owned,
    # so Nav2 cannot stop and rotate before ball 2.
    cmd = _tick(mission, ball_map, pose=(3.4, 0.0, 0.0), now=now)
    assert mission.phase == "nav"
    assert mission.current_ball_id == 2
    assert mission.nav_goal is None
    assert mission._link_active
    assert cmd.base.linear_speed_m_s > 0.0
    events = [t for t, _ in mission.drain_events()]
    assert "route_pass_start" in events
    assert "route_ball_swept" in events
    assert "route_fine_approach" not in events


def test_sweep_link_bends_to_second_ball_without_in_place_rotation():
    mission, ball_map, now = _sweep_mission_in_drive(
        ball_positions=((3.0, 0.0), (6.0, 1.0))
    )
    _tick(mission, ball_map, pose=(2.0, 0.0, 0.0), nav_state="reached", now=now)
    _tick(mission, ball_map, pose=(3.4, 0.0, 0.0), now=now)
    cmd = _tick(mission, ball_map, pose=(3.4, 0.0, 0.0), now=now)

    assert mission._link_active
    assert cmd.base.linear_speed_m_s > 0.0
    assert cmd.base.angular_speed_rad_s > 0.0


def test_sweep_beam_credit_marks_nearest_ball_only_once():
    mission, ball_map, now = _sweep_mission_in_drive()
    # Beam latch stays True for several ticks while crossing ball 1.
    _tick(mission, ball_map, pose=(3.0, 0.0, 0.0), nav_state="active",
          now=now, confirmed=True)
    _tick(mission, ball_map, pose=(3.0, 0.0, 0.0), nav_state="active",
          now=now, confirmed=True)
    collected = [s.ball_id for s in mission.stops if s.status == "collected"]
    assert collected == [1]
    assert ball_map.balls[1].state == "collected"


def test_sweep_confirmation_never_waits_or_replans_the_route():
    """A retained ball is a metric; it cannot alter the planned traversal."""
    mission, ball_map, now = _sweep_mission_in_drive()
    first_goal = mission.nav_goal

    # The first crossing is credited before the robot has passed it.  The
    # route still owns the same active crossing and its same fixed goal.
    _tick(mission, ball_map, pose=(2.0, 0.0, 0.0), nav_state="active",
          now=now, confirmed=True)
    assert mission.phase == "nav"
    assert mission.current_ball_id == 1
    # Reaching the run-in transfers the base to the straight pass.  This is
    # geometric progression, not a capture transition.
    assert first_goal is not None
    assert mission.nav_goal is None
    assert mission._pass_active
    assert mission.stops[0].status == "collected"

    # Once the geometric crossing is complete, it advances normally; no
    # settle/fine-approach path is entered because the credit exists.
    _tick(mission, ball_map, pose=(2.0, 0.0, 0.0), nav_state="reached", now=now)
    _tick(mission, ball_map, pose=(3.4, 0.0, 0.0), now=now)
    assert mission.phase == "nav"
    assert mission.current_ball_id == 2
    assert "route_fine_approach" not in [event for event, _ in mission.drain_events()]


def test_sweep_route_completes_regardless_of_missed_balls():
    mission, ball_map, now = _sweep_mission_in_drive()
    _tick(mission, ball_map, nav_state="reached", now=now)  # ball 1: no credit
    _tick(mission, ball_map, pose=(6.0, 0.0, 0.0), nav_state="active",
          now=now, confirmed=True)  # ball 2 credited at the crossing
    assert mission.is_done
    done = dict(mission.drain_events())["route_complete"]
    assert done["collected"] == 1
    assert done["swept_uncollected"] == 1
    assert done["failed_ball_ids"] == [1]
