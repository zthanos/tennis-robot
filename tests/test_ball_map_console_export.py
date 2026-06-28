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
