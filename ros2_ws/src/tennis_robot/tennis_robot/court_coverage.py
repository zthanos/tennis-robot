"""Coverage planning for Court Survey v2 (ROS-free, pure functions).

The goal is NOT a precise perimeter — it is to give the 360° LiDAR enough views
that `court_extraction` can fit every fence side. We pick a few vantage points in
the court frame; the mission node drives to them with Nav2 (which routes around
the net/obstacles automatically via the costmap). Measurement comes from the
accumulated map, not from the path.

NO FALLBACKS: the controller keeps covering only while failures are *recoverable*
(more views could help). Any structural failure (non-standard court, ambiguous
width, non-rectangular fence) stops immediately — fail-loud.
"""

from __future__ import annotations

import math

try:
    from tennis_robot.court_extraction import CourtFrame, CourtSpec
except ModuleNotFoundError:
    from court_extraction import CourtFrame, CourtSpec


# Failures that MORE coverage might fix → keep visiting vantage points.
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

    Centre of each half on the centre line (y'=0). From x'=±L/4 the far/near
    baselines sit ~ (L/4 + run-off) ≈ 9 m away (< lidar_range_m), and the side
    fences are seen in segments from both halves. Nav2 routes between them around
    the net (it is an obstacle in the costmap), so no manual net crossing.
    """
    quarter = spec.half_length_m / 2.0
    yaw = math.atan2(frame.uy, frame.ux)  # +x' heading (irrelevant for 360° scan)
    poses: list[dict] = []
    for court_x in (-quarter, quarter):
        mx, my = frame.to_map(court_x, 0.0)
        poses.append({
            "x_m": round(mx, 3),
            "y_m": round(my, 3),
            "yaw_rad": round(yaw, 4),
            "court_x": round(court_x, 3),
            "court_y": 0.0,
        })
    return poses
