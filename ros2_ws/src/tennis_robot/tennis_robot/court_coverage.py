"""Coverage planning for Court Survey v2 (ROS-free, pure functions).

The goal is NOT a precise perimeter -- it is to give the 360-degree LiDAR enough
views that `court_extraction` can fit every fence side. We pick vantage points in
the court frame; the mission node drives to them deterministically. Measurement
comes from the accumulated map, not from the path.

NO FALLBACKS: the controller keeps covering only while failures are *recoverable*
(more views could help). Any structural failure (non-standard court, ambiguous
width, non-rectangular fence) stops immediately -- fail-loud.
"""

from __future__ import annotations

import math

try:
    from tennis_robot.court_extraction import CourtFrame, CourtSpec
except ModuleNotFoundError:
    from court_extraction import CourtFrame, CourtSpec


# Failures that MORE coverage might fix -> keep visiting vantage points.
_RECOVERABLE_PREFIXES = (
    "coverage_incomplete",
    "fence_side_missing",
    "net_not_observed",
)


def is_recoverable_failure(reason: str) -> bool:
    """True if visiting more vantage points could resolve the failure."""
    return any(reason.startswith(p) for p in _RECOVERABLE_PREFIXES)


def vantage_points(frame: CourtFrame, spec: CourtSpec,
                   lidar_range_m: float = 11.0) -> list[dict]:
    """Map-frame poses that together bring every fence within LiDAR range.

    Drive DEEP into each half (toward the fence, where the fence-approach stop
    halts the robot ~1.5 m short for dense mapping), crossing the net through the
    post->fence GAP. A final RETURN pass re-crosses the net and revisits the near
    half so slam_toolbox loop-closure can align the two halves (kills the drift
    double-line) and complete the map.
    """
    quarter = spec.half_length_m / 2.0
    yaw = math.atan2(frame.uy, frame.ux)  # +x' heading (irrelevant for 360-deg scan)
    gap_y = spec.post_half_span_doubles_m + 1.05  # ~6.7 m: past the post, inside fence
    deep = spec.half_length_m + 4.0               # into the run-off toward the fence
    # stop_short=True ONLY for the deep fence-approach points, where the front
    # fence-stop is meant to halt the robot ~1.4 m short for dense fence mapping.
    # For gap-crossing / intermediate / return points it MUST be False: the net
    # is now LiDAR-visible, and a True here makes the robot mistake the net for
    # the target fence and stop instead of crossing to the far half.
    # (court_x, court_y, stop_short)
    court_path = [
        (-deep, 0.0, True),     # deep near half -> near fence solidly mapped
        (-2.0, gap_y, False),   # out to the gap, near side
        (2.0, gap_y, False),    # cross x'=0 at the gap (clear of the net), far side
        (quarter, 0.0, False),  # far-half centre (intermediate hop)
        (deep, 0.0, True),      # deep far half -> far fence solidly mapped
        # Return pass: re-cross the net through the OTHER gap and come back to the
        # near half -> loop-closure overlap aligns the two halves and finishes the
        # map. Measurement is already locked by now; this pass is purely for a
        # clean, complete map.
        (2.0, -gap_y, False),   # back out to the gap, far side (opposite side)
        (-2.0, -gap_y, False),  # cross x'=0 again -> re-observe the net (loop overlap)
        (-quarter, 0.0, False), # near-half centre -> overlap with the start -> loop closes
    ]
    poses: list[dict] = []
    for court_x, court_y, stop_short in court_path:
        mx, my = frame.to_map(court_x, court_y)
        poses.append({
            "x_m": round(mx, 3),
            "y_m": round(my, 3),
            "yaw_rad": round(yaw, 4),
            "court_x": round(court_x, 3),
            "court_y": round(court_y, 3),
            "stop_short": stop_short,
        })
    return poses
