"""The basket mover must never drive the carriage into a mechanical stop.

Reaching a hard stop UNDER COMMAND latches the DART joint — the same failure as
parking on the limit, which the over-travel margin alone does not prevent. A
live run drove the carriage from 97.4 mm past its 0 mm target into the -10 mm
stop and froze it, because the drive was bounded only by a generous wall-clock
timeout while joint feedback had gone quiet.

These tests drive the real move_to() with a stubbed ROS node so the bounds are
exercised without a simulator.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

rclpy = pytest.importorskip("rclpy", reason="mover is a ROS module")
from tennis_robot import basket_lift_mover as mover  # noqa: E402


class _StubNode:
    def __init__(self):
        self.commands = []

    def create_subscription(self, *a, **k):
        return None

    def create_publisher(self, *a, **k):
        node = self

        class _Pub:
            def publish(self, msg):
                node.commands.append(msg.data[0])
        return _Pub()


@pytest.fixture
def driver(monkeypatch):
    node = _StubNode()
    d = mover.BasketLiftDriver(node)
    d.velocity = 0.0
    # Drive simulated time forward on every spin so the bounds are reachable.
    clock = {"t": 0.0}
    monkeypatch.setattr(mover.time, "monotonic", lambda: clock["t"])
    def fake_spin(_node, timeout_sec=0.0):
        clock["t"] += 0.02
        hook = getattr(d, "_hook", None)
        if hook:
            hook(clock["t"])
    monkeypatch.setattr(mover.rclpy, "spin_once", fake_spin)
    d._node = node
    d._clock = clock
    return d


def test_frozen_feedback_aborts_instead_of_driving_blind(driver):
    """The exact live failure: position never updates, so stop — do not drive on."""
    driver.position = 0.0974            # start raised
    driver._hook = lambda t: None        # feedback never changes
    ok, detail = driver.move_to(0.0, timeout_s=30.0)
    assert not ok
    assert "stale" in detail
    # Bounded by staleness, not by the 30 s caller timeout.
    assert driver._clock["t"] < 2.0
    assert driver._node.commands[-1] == 0.0, "must always stop the carriage"


def test_drive_is_bounded_by_distance_not_by_the_caller_timeout(driver):
    """Feedback alive but the joint refuses to move: stop after the travel time."""
    driver.position = 0.0974

    def creep(t):
        # Position changes (so not stale) but never approaches the target.
        driver.position = 0.0974 + (t % 2) * 1e-6
    driver._hook = creep
    ok, detail = driver.move_to(0.0, timeout_s=120.0)
    assert not ok
    # 97.4 mm at 120 mm/s is ~0.83 s; the bound must be seconds, not the 120 s
    # the caller was willing to wait.
    assert driver._clock["t"] < 5.0, f"drive ran for {driver._clock['t']:.2f}s"
    assert driver._node.commands[-1] == 0.0


def test_a_normal_move_still_reaches_the_endpoint(driver):
    driver.position = 0.0974

    def descend(t):
        driver.position = max(0.0, 0.0974 - mover.SPEED_MPS * t)
        driver.velocity = -mover.SPEED_MPS if driver.position > 0.0 else 0.0
    driver._hook = descend
    ok, detail = driver.move_to(0.0, timeout_s=30.0)
    assert ok, detail
    assert abs(driver.position) <= mover.POSITION_TOLERANCE_M
    assert driver._node.commands[-1] == 0.0
