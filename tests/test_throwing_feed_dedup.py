"""Feed requests must be consumed effectively-once.

A feed request is a discrete event, unlike the idempotent velocity setpoints
that share the same publisher. RosService sends it as a short burst so a DDS
drop cannot stall a session, so the consumer has to de-duplicate — the first
live E2E run produced 15 feed events on the topic for 3 throws, which a real
feeder would have turned into 15 balls.

throw_id is the correlation key that makes this safe, so the test drives the
node's real handler rather than a stand-in.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

rclpy = pytest.importorskip("rclpy", reason="feed handler lives in a ROS node")
from tennis_robot.gazebo_extras_node import GazeboExtrasNode  # noqa: E402


class _Logger:
    def __init__(self):
        self.accepted, self.warnings, self.debug_lines = [], [], []

    def info(self, msg):
        if "accepted placeholder throwing feed event" in msg:
            self.accepted.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def debug(self, msg):
        self.debug_lines.append(msg)


class _Handler:
    """The real handler, bound to a bare object (no ROS graph needed)."""

    def __init__(self):
        self.logger = _Logger()
        self._throwing_session_id = None
        self._throwing_seen_ids = set()

    def get_logger(self):
        return self.logger

    def feed(self, **payload):
        GazeboExtrasNode._on_throwing_feed_request(
            self, SimpleNamespace(data=json.dumps(payload))
        )


def _request(session="s1", throw="t1", zone="forehand_zone"):
    return {"session_id": session, "throw_id": throw, "target_zone": zone, "count": 1}


def test_a_burst_of_identical_requests_feeds_once():
    h = _Handler()
    for _ in range(5):  # exactly what RosService publishes per throw
        h.feed(**_request())
    assert len(h.logger.accepted) == 1


def test_each_distinct_throw_id_feeds_once():
    h = _Handler()
    for throw in ("t1", "t2", "t3"):
        for _ in range(5):
            h.feed(**_request(throw=throw))
    assert len(h.logger.accepted) == 3


def test_a_new_session_may_reuse_ids_without_being_suppressed():
    h = _Handler()
    for _ in range(5):
        h.feed(**_request(session="s1", throw="t1"))
    for _ in range(5):
        h.feed(**_request(session="s2", throw="t1"))
    assert len(h.logger.accepted) == 2


@pytest.mark.parametrize("payload", [
    {"session_id": "", "throw_id": "t1", "count": 1},
    {"session_id": "s1", "throw_id": "", "count": 1},
    {"session_id": "s1", "throw_id": "t1", "count": 2},
    {"session_id": "s1", "throw_id": "t1", "count": 0},
])
def test_incomplete_requests_never_feed(payload):
    h = _Handler()
    h.feed(**payload)
    assert h.logger.accepted == []
    assert h.logger.warnings
