"""LiDAR signal processing helpers — pure functions, no Webots dependency."""

from __future__ import annotations

import math


def extract_ball_candidates(
    ranges: list[float],
    local_x: float = -0.08,
    local_y: float = 0.0,
) -> list[tuple[float, float]]:
    """Return (robot-local x, y) centroids of LiDAR clusters that could be balls.

    The LiDAR is mounted at ball height so isolated close returns between
    CANDIDATE_MIN_M and CANDIDATE_MAX_M are plausible balls.  Each cluster
    centroid is in robot-local (x, y) coordinates; the caller navigates toward
    the closest one while the OAK-D camera confirms or rejects within visual range.
    """
    if not ranges:
        return []

    n = len(ranges)
    BODY_EXCLUSION_M = 0.55
    CANDIDATE_MIN_M = 0.50
    CANDIDATE_MAX_M = 8.0
    MIN_CLUSTER_SPAN_DEG = 1.0
    MAX_CLUSTER_SPAN_DEG = 25.0

    def _robot_xy(i: int, r: float) -> tuple[float, float]:
        angle = (i / n) * 2 * math.pi - math.pi
        return local_x + r * math.cos(angle), local_y + r * math.sin(angle)

    valid_indices = [
        i for i, r in enumerate(ranges)
        if math.isfinite(r) and CANDIDATE_MIN_M <= r <= CANDIDATE_MAX_M
        and math.hypot(*_robot_xy(i, r)) >= BODY_EXCLUSION_M
    ]
    if not valid_indices:
        return []

    GAP_DEG = 5.0
    gap_indices = int(GAP_DEG / 360.0 * n)
    clusters: list[list[tuple[float, float, float]]] = []
    current: list[tuple[float, float, float]] = []
    prev_i: int | None = None

    for idx in valid_indices:
        if prev_i is not None and idx - prev_i > gap_indices:
            if current:
                clusters.append(current)
            current = []
        r = ranges[idx]
        lx, ly = _robot_xy(idx, r)
        current.append((r, lx, ly))
        prev_i = idx
    if current:
        clusters.append(current)

    candidates: list[tuple[float, float]] = []
    for cluster in clusters:
        span_deg = len(cluster) / n * 360.0
        if not (MIN_CLUSTER_SPAN_DEG <= span_deg <= MAX_CLUSTER_SPAN_DEG):
            continue
        cx = sum(p[1] for p in cluster) / len(cluster)
        cy = sum(p[2] for p in cluster) / len(cluster)
        candidates.append((cx, cy))

    return candidates


def front_range_m(
    ranges: list[float],
    center_ratio: float = 0.5,
    min_obstacle_range_m: float = 0.18,
) -> float | None:
    """Return median range of the forward-facing LiDAR window, or None."""
    if not ranges:
        return None
    center = int(round((len(ranges) - 1) * center_ratio))
    half_width = max(2, len(ranges) // 72)
    window = ranges[max(0, center - half_width) : min(len(ranges), center + half_width + 1)]
    valid = [v for v in window if math.isfinite(v) and v >= min_obstacle_range_m]
    if not valid:
        return None
    valid.sort()
    return valid[len(valid) // 2]
