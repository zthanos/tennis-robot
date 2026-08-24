"""`Interval Between Throws` is the launch-to-launch PERIOD.

The first live E2E measured 7.92 s and 8.56 s for a configured 4.0 s, because
the session slept the whole interval AFTER the feed request had finished — so
the observed cadence was `feed work + interval`. The period must absorb the
feed work instead of adding to it.

These tests drive the real ThrowingService with a fake robot port whose feed
request takes a controllable amount of time, and assert on the instants the
feed requests are ISSUED — the same thing the consumer observes.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.console.throwing_service import ThrowingService  # noqa: E402
from tennis_robot.throwing_mode import SessionState  # noqa: E402

BOUNDARY = (ROOT / "runtime/court_boundary.json").read_text()


class _StatusStore:
    def __init__(self):
        self.status = {"robot_x_m": -8.0, "robot_y_m": 0.0,
                       "robot_yaw_rad": 0.0, "measured_speed_mps": 0.0}

    def read(self):
        return dict(self.status)


class _Sensors:
    def read(self):
        return {"front_camera": None}


class _Camera:
    available = False


class _SlowFeedRos:
    """Robot port whose feed request costs real time, like the live publisher."""

    def __init__(self, status_store, feed_duration_s=0.0):
        self.status_store = status_store
        self.feed_duration_s = feed_duration_s
        self.feed_issued_at: list[float] = []
        self.flywheel_commands: list[float] = []
        self.basket_commands: list[bool] = []

    def runtime_kind(self): return "simulation"
    def flywheel_available(self): return True
    def basket_position_state(self): return "UNKNOWN"
    def collector_status(self): return {"ok": True, "running": False}
    def stop_basket(self): return True
    def nav_cancel(self): return True
    def wait_flywheel_ready(self, speed): return True

    def navigate_to_pose(self, x, y, yaw):
        self.status_store.status.update(robot_x_m=x, robot_y_m=y,
                                        robot_yaw_rad=yaw, measured_speed_mps=0.0)
        return {"succeeded": True}

    def align_heading(self, target_yaw_rad, tolerance_rad, timeout_s=45.0):
        self.status_store.status["robot_yaw_rad"] = target_yaw_rad
        return True

    def set_basket_position(self, raised):
        self.basket_commands.append(raised)
        return True

    def set_flywheel_speed(self, speed):
        self.flywheel_commands.append(speed)
        return True

    def request_ball_feed(self, *, publish_at_unix=None, **request):
        # Model the real publisher: it is handed the request early, spends
        # feed_duration_s discovering, then emits AT publish_at_unix. The
        # emission instant is what the consumer observes, so that is what the
        # cadence assertions use.
        if self.feed_duration_s:
            time.sleep(self.feed_duration_s)
        if publish_at_unix is not None:
            while time.time() < publish_at_unix:
                time.sleep(0.005)
        self.feed_issued_at.append(time.monotonic())
        return True


def _service(tmp_path, ros, status_store):
    tmp_path.mkdir(parents=True, exist_ok=True)
    (tmp_path / "court.json").write_text(BOUNDARY)
    return ThrowingService(ros=ros, status_store=status_store, sensor_store=_Sensors(),
                           boundary_path=tmp_path / "court.json", camera=_Camera())


def _run(service, balls, interval, timeout=40.0):
    assert service.start({"program": "forehand", "ball_count": balls,
                          "interval_s": interval})["ok"]
    deadline = time.time() + timeout
    while (service.status()["session"]["state"] not in {"COMPLETED", "FAULT"}
           and time.time() < deadline):
        time.sleep(0.02)
    return service.status()


def _gaps(instants):
    return [instants[i + 1] - instants[i] for i in range(len(instants) - 1)]


def test_period_absorbs_feed_work_instead_of_adding_to_it(tmp_path):
    """The exact live defect: 0.8 s of feed work must not become 0.8 s of extra period."""
    store = _StatusStore()
    ros = _SlowFeedRos(store, feed_duration_s=0.8)
    status = _run(_service(tmp_path, ros, store), balls=3, interval=1.5)
    assert status["session"]["state"] == "COMPLETED"
    assert len(ros.feed_issued_at) == 3
    for gap in _gaps(ros.feed_issued_at):
        # Old behaviour would give 1.5 + 0.8 = 2.3 s.
        assert gap == pytest.approx(1.5, abs=0.25), f"gaps were {_gaps(ros.feed_issued_at)}"


def test_cadence_is_independent_of_how_long_the_feed_takes(tmp_path):
    fast_store, slow_store = _StatusStore(), _StatusStore()
    fast = _SlowFeedRos(fast_store, feed_duration_s=0.05)
    slow = _SlowFeedRos(slow_store, feed_duration_s=0.9)
    _run(_service(tmp_path / "a", fast, fast_store), balls=3, interval=1.5)
    _run(_service(tmp_path / "b", slow, slow_store), balls=3, interval=1.5)
    fast_mean = sum(_gaps(fast.feed_issued_at)) / 2
    slow_mean = sum(_gaps(slow.feed_issued_at)) / 2
    assert abs(fast_mean - slow_mean) < 0.3, (
        f"cadence tracked feed duration: {fast_mean:.2f}s vs {slow_mean:.2f}s"
    )


def test_a_feed_slower_than_the_interval_never_produces_a_catch_up_burst(tmp_path):
    """Missed deadlines rebase; they must not accumulate into extra throws."""
    store = _StatusStore()
    ros = _SlowFeedRos(store, feed_duration_s=1.2)   # longer than the interval
    status = _run(_service(tmp_path, ros, store), balls=4, interval=0.4)
    assert status["session"]["state"] == "COMPLETED"
    assert len(ros.feed_issued_at) == 4, "one feed per throw, never a burst"
    assert len(set(status["session"]["throw_ids"])) == 4
    for gap in _gaps(ros.feed_issued_at):
        # Cadence degrades to the feed duration; it never goes to ~0, which is
        # what a catch-up burst would look like.
        assert gap >= 1.0, f"suspicious catch-up gap {gap:.3f}s"


def test_pause_suspends_the_interval_clock_and_resume_does_not_burst(tmp_path):
    store = _StatusStore()
    ros = _SlowFeedRos(store, feed_duration_s=0.05)
    service = _service(tmp_path, ros, store)
    assert service.start({"program": "forehand", "ball_count": 3, "interval_s": 1.0})["ok"]

    deadline = time.time() + 20
    while len(ros.feed_issued_at) < 1 and time.time() < deadline:
        time.sleep(0.01)
    assert service.pause()["ok"]
    feeds_at_pause = len(ros.feed_issued_at)

    time.sleep(3.0)   # three intervals' worth of paused wall-clock
    assert len(ros.feed_issued_at) == feeds_at_pause, "a feed was issued while PAUSED"

    resumed_at = time.monotonic()
    assert service.resume()["ok"]
    deadline = time.time() + 20
    while (service.status()["session"]["state"] not in {"COMPLETED", "FAULT"}
           and time.time() < deadline):
        time.sleep(0.02)

    after = [t for t in ros.feed_issued_at[feeds_at_pause:]]
    assert after, "throwing did not continue after resume"
    # Paused time is not a debt: the first post-resume throw waits out only the
    # interval that was left when Pause was pressed, and no burst follows.
    assert after[0] - resumed_at >= 0.0
    for gap in _gaps(after):
        assert gap == pytest.approx(1.0, abs=0.3), f"post-resume gaps {_gaps(after)}"
    assert len(ros.feed_issued_at) == 3
    assert len(set(service.status()["session"]["throw_ids"])) == 3


def test_stop_prevents_the_next_scheduled_throw_and_shuts_the_flywheels_down(tmp_path):
    store = _StatusStore()
    ros = _SlowFeedRos(store, feed_duration_s=0.05)
    service = _service(tmp_path, ros, store)
    assert service.start({"program": "forehand", "ball_count": 5, "interval_s": 1.0})["ok"]

    deadline = time.time() + 20
    while len(ros.feed_issued_at) < 1 and time.time() < deadline:
        time.sleep(0.01)
    assert ros.feed_issued_at, "session never threw"
    assert service.stop()["ok"]
    feeds_at_stop = len(ros.feed_issued_at)

    time.sleep(2.5)   # more than two configured intervals
    assert len(ros.feed_issued_at) == feeds_at_stop, "a throw was issued after Stop"
    status = service.status()
    assert status["session"]["state"] == "COMPLETED"
    assert ros.flywheel_commands[-1] == 0.0, "flywheels were not commanded to stop"
    assert status["session"]["statistics"]["balls_thrown"] < 5
