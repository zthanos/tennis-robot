from __future__ import annotations

import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from tennis_robot.lidar_preview import LaserScanMetadataTracker, with_liveness


def scan(stamp_s: int = 10) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="lidar_link",
            stamp=SimpleNamespace(sec=stamp_s, nanosec=250_000_000),
        ),
        ranges=[1.0, float("inf"), 2.5, float("nan"), 0.01, 20.0],
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 360,
        range_min=0.05,
        range_max=16.0,
        scan_time=0.1,
        time_increment=0.1 / 720,
    )


def test_metadata_is_independent_of_scan_source_and_measures_rate() -> None:
    tracker = LaserScanMetadataTracker(window_size=3)
    first = tracker.observe(scan(), monotonic_s=1.0, wall_time_s=100.0)
    second = tracker.observe(scan(11), monotonic_s=1.1, wall_time_s=100.1)

    assert first["scan_rate_hz"] is None
    assert second["scan_rate_hz"] == pytest.approx(10.0)
    assert second["frame_id"] == "lidar_link"
    assert second["source_stamp_s"] == pytest.approx(11.25)
    assert second["sample_count"] == 6
    assert second["valid_sample_count"] == 2
    assert second["invalid_sample_count"] == 4
    assert second["angle_min_rad"] == -math.pi
    assert second["range_max_m"] == 16.0


def test_liveness_ages_after_heartbeat_stops() -> None:
    metadata = {"last_message_at_s": 100.0, "frame_id": "lidar_link"}

    live = with_liveness(metadata, now_s=101.0, stale_after_s=2.5)
    stale = with_liveness(metadata, now_s=103.0, stale_after_s=2.5)

    assert live["state"] == "live"
    assert live["last_message_age_s"] == 1.0
    assert stale["state"] == "stale"
    assert stale["last_message_age_s"] == 3.0
    assert "state" not in metadata


def test_invalid_liveness_timestamp_is_waiting() -> None:
    result = with_liveness(
        {"last_message_at_s": None}, now_s=100.0, stale_after_s=2.5
    )
    assert result == {
        "last_message_at_s": None,
        "last_message_age_s": None,
        "state": "waiting",
    }
