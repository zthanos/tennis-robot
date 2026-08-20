"""Source-agnostic LaserScan metadata for diagnostics and liveness."""

from __future__ import annotations

import math
from collections import deque


class LaserScanMetadataTracker:
    """Measure arrival rate and expose stable metadata from any LaserScan source."""

    def __init__(self, window_size: int = 30) -> None:
        if window_size < 1:
            raise ValueError("window_size must be positive")
        self._intervals_s: deque[float] = deque(maxlen=window_size)
        self._last_monotonic_s: float | None = None

    def observe(
        self,
        message: object,
        *,
        monotonic_s: float,
        wall_time_s: float,
    ) -> dict:
        if self._last_monotonic_s is not None:
            interval_s = monotonic_s - self._last_monotonic_s
            if math.isfinite(interval_s) and 0.0 < interval_s < 10.0:
                self._intervals_s.append(interval_s)
        self._last_monotonic_s = monotonic_s

        ranges = tuple(float(value) for value in message.ranges)
        range_min = float(message.range_min)
        range_max = float(message.range_max)
        valid_count = sum(
            1
            for value in ranges
            if math.isfinite(value) and range_min <= value <= range_max
        )
        mean_interval_s = (
            sum(self._intervals_s) / len(self._intervals_s)
            if self._intervals_s
            else None
        )
        stamp = message.header.stamp
        return {
            "last_message_at_s": float(wall_time_s),
            "source_stamp_s": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            "frame_id": str(message.header.frame_id),
            "scan_rate_hz": (
                None if mean_interval_s is None else 1.0 / mean_interval_s
            ),
            "sample_count": len(ranges),
            "valid_sample_count": valid_count,
            "invalid_sample_count": len(ranges) - valid_count,
            "angle_min_rad": float(message.angle_min),
            "angle_max_rad": float(message.angle_max),
            "angle_increment_rad": float(message.angle_increment),
            "range_min_m": range_min,
            "range_max_m": range_max,
            "scan_time_s": float(message.scan_time),
            "time_increment_s": float(message.time_increment),
        }


def with_liveness(metadata: dict | None, *, now_s: float, stale_after_s: float) -> dict | None:
    """Return a copy with wall-clock age and live/stale state."""

    if metadata is None:
        return None
    if stale_after_s <= 0:
        raise ValueError("stale_after_s must be positive")
    result = dict(metadata)
    received_at_s = result.get("last_message_at_s")
    if not isinstance(received_at_s, (int, float)) or not math.isfinite(received_at_s):
        result.update({"last_message_age_s": None, "state": "waiting"})
        return result
    age_s = max(0.0, float(now_s) - float(received_at_s))
    result.update(
        {
            "last_message_age_s": age_s,
            "state": "live" if age_s <= stale_after_s else "stale",
        }
    )
    return result
