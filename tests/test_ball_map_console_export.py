"""Tests for BallMap.to_console_balls — the Collection Map export.

Verifies recognized balls are exported at their detected world coordinates with
the fields the renderer (scripts/control_panel/collection_map.js) consumes.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.ball_map import BallMap, BallMapConfig, MappedBall


def _seed():
    m = BallMap(BallMapConfig(min_seen_count=3, stale_after_s=45.0))
    m.balls = {
        1: MappedBall(1, -3.0, 1.2, 0.9, 0.0, 100.0, "oak_depth", seen_count=6, state="detected"),
        2: MappedBall(2, -5.5, -2.0, 0.5, 0.0, 100.0, "oak_depth", seen_count=2, state="detected"),
        3: MappedBall(3, -1.0, 0.0, 0.8, 0.0, 100.0, "oak_depth", seen_count=9, state="collected"),
        4: MappedBall(4, 4.0, 1.0, 0.8, 0.0, 100.0, "oak_depth", seen_count=9, state="detected"),
    }
    return m


def test_exports_recognized_world_points():
    out = _seed().to_console_balls(-6.0, active_target_id=1, now=100.0)
    b1 = next(b for b in out if b["id"] == 1)
    assert (b1["x_m"], b1["y_m"]) == (-3.0, 1.2)  # exact recognition point


def test_collected_excluded_by_default():
    ids = {b["id"] for b in _seed().to_console_balls(-6.0, now=100.0)}
    assert ids == {1, 2, 4}
    ids_all = {b["id"] for b in _seed().to_console_balls(-6.0, now=100.0, include_collected=True)}
    assert 3 in ids_all


def test_confirmed_threshold_and_active_target():
    out = _seed().to_console_balls(-6.0, active_target_id=1, now=100.0)
    by_id = {b["id"]: b for b in out}
    assert by_id[1]["confirmed"] is True and by_id[1]["planned"] is True
    assert by_id[2]["confirmed"] is False        # seen_count below min
    assert by_id[4]["planned"] is False


def test_across_net_classification():
    out = _seed().to_console_balls(-6.0, now=100.0)   # robot on negative-x side
    by_id = {b["id"]: b for b in out}
    assert by_id[4]["side"] == "across_net"           # ball at +4 m
    assert by_id[1]["side"] == "same_side"


def test_staleness_marks_not_visible():
    m = _seed()
    # now far past last_seen => stale => not a visible candidate even if confirmed
    out = m.to_console_balls(-6.0, now=100.0 + 1000.0)
    b1 = next(b for b in out if b["id"] == 1)
    assert b1["confirmed"] is True
    assert b1["visible_candidate"] is False


def test_planned_order_populates_route_fields():
    out = _seed().to_console_balls(-6.0, now=100.0, planned_order={1: 2, 4: 1})
    by_id = {b["id"]: b for b in out}
    assert by_id[1]["planned"] is True and by_id[1]["order"] == 2
    assert by_id[4]["planned"] is True and by_id[4]["order"] == 1
    assert by_id[2]["planned"] is False and by_id[2]["order"] is None


def test_create_distance_override_allows_far_scan_entries():
    from tennis_robot.collector import BallObservationInput

    def far_obs():
        return BallObservationInput(
            visible=True, distance_m=8.0, confidence=0.8, source="oak_ai_depth",
            world_x_m=-8.0, world_y_m=1.0,
        )

    m = BallMap(BallMapConfig())
    ball_id, is_new = m.update(far_obs(), now=100.0)
    assert ball_id is None  # default 3 m create gate rejects an 8 m sighting

    m.max_create_distance_override_m = 9.0
    ball_id, is_new = m.update(far_obs(), now=100.0)
    assert ball_id is not None and is_new

    m.max_create_distance_override_m = None
    ball_id, is_new = m.update(
        BallObservationInput(
            visible=True, distance_m=8.0, confidence=0.8, source="oak_ai_depth",
            world_x_m=-2.0, world_y_m=-4.0,
        ),
        now=100.0,
    )
    assert ball_id is None  # gate restored after the scan


def test_terminal_failed_ball_is_not_resurrected_by_update():
    from tennis_robot.collector import BallObservationInput

    m = BallMap(BallMapConfig())
    m.balls[1] = MappedBall(
        1, 2.0, 0.0, 0.9, 0.0, 100.0, "oak_depth", seen_count=6, state="collection_failed"
    )
    ball_id, is_new = m.update(
        BallObservationInput(
            visible=True,
            distance_m=1.0,
            confidence=0.9,
            source="oak_ai_depth",
            world_x_m=2.1,
            world_y_m=0.05,
        ),
        now=101.0,
    )
    assert (ball_id, is_new) == (1, False)
    assert m.balls[1].state == "collection_failed"
    assert len(m.balls) == 1
