#!/usr/bin/env python3
"""P2-68 geometric wall audit from the existing P2-67 R75/75 traces.

This is deliberately not a simulator.  It reads one restitution duplicate
(e=0.3) for each of the 30 speed/friction/lateral conditions and interpolates
the recorded ball-centre chords to at most 0.1 mm.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
MATRIX = ROOT / "gazebo_p267/matrix/a_75/r_75"
OUT = ROOT / "p268_wall_geometry.json"
BALL_RADIUS_MM = 33.0
CLEARANCE_MM = 10.0
ENVELOPE_MM = BALL_RADIUS_MM + CLEARANCE_MM
PANEL_THICKNESS_MM = 4.0
PANEL_HALF_WIDTH_MM = 172.0
WALL_BOTTOM_MM = 46.1
SELECTED_X_MM = 225.0
SELECTED_TOP_MM = 235.0


def dense_trace(path: Path) -> list[tuple[float, float, float, float]]:
    raw = []
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            t = float(row["time_s"])
            if 2.40 <= t <= 3.45:
                raw.append(
                    (
                        1000.0 * float(row["ball_base_x_m"]),
                        1000.0 * float(row["ball_base_y_m"]),
                        1000.0 * float(row["ball_base_z_m"]),
                        t,
                    )
                )
    dense: list[tuple[float, float, float, float]] = []
    for a, b in zip(raw, raw[1:]):
        subdivisions = max(1, math.ceil(math.dist(a[:3], b[:3]) / 0.1))
        for index in range(subdivisions):
            u = index / subdivisions
            dense.append(tuple(a[k] + u * (b[k] - a[k]) for k in range(4)))
    dense.append(raw[-1])
    return dense


def top_limits(traces, wall_x_mm: float) -> tuple[float, float]:
    circular = math.inf
    box = math.inf
    half_t = PANEL_THICKNESS_MM / 2.0
    for _, points in traces:
        for x, _, z, _ in points:
            dx = max(0.0, abs(x - wall_x_mm) - half_t)
            if dx <= ENVELOPE_MM:
                circular = min(
                    circular,
                    z - math.sqrt(max(0.0, ENVELOPE_MM**2 - dx**2)),
                )
            # Conservative rectangular R33+10 envelope.  This intentionally
            # ignores the rounded corner credit of the exact Minkowski sum.
            if abs(x - wall_x_mm) <= ENVELOPE_MM + half_t:
                box = min(box, z - ENVELOPE_MM)
    return circular, box


def selected_clearance(traces) -> tuple[float, float, float]:
    minimum_euclidean = math.inf
    minimum_box = math.inf
    maximum_abs_y = 0.0
    half_t = PANEL_THICKNESS_MM / 2.0
    for _, points in traces:
        for x, y, z, _ in points:
            dx = max(0.0, abs(x - SELECTED_X_MM) - half_t)
            dy = max(0.0, abs(y) - PANEL_HALF_WIDTH_MM)
            dz = max(WALL_BOTTOM_MM - z, 0.0, z - SELECTED_TOP_MM)
            minimum_euclidean = min(
                minimum_euclidean, math.sqrt(dx * dx + dy * dy + dz * dz)
            )
            minimum_box = min(minimum_box, max(dx, dy, dz))
            if abs(x - SELECTED_X_MM) <= ENVELOPE_MM + half_t:
                maximum_abs_y = max(maximum_abs_y, abs(y))
    return minimum_euclidean, minimum_box, maximum_abs_y


def main() -> None:
    paths = sorted(MATRIX.glob("v_*/tm_*/y_*/e_0p3/telemetry.csv"))
    if len(paths) != 30:
        raise SystemExit(f"expected 30 P2-67 paths, found {len(paths)}")
    traces = [(str(path), dense_trace(path)) for path in paths]
    candidates = []
    for wall_x in range(180, 281, 5):
        circular, box = top_limits(traces, float(wall_x))
        candidates.append(
            {
                "wall_x_mm": wall_x,
                "exact_circular_top_limit_mm": circular,
                "conservative_box_top_limit_mm": box,
            }
        )
    euclidean, box, maximum_abs_y = selected_clearance(traces)
    result = {
        "phase": "P2-68",
        "method": "geometry_only_from_existing_P2-67_R75_75_e0p3_trajectories",
        "new_gazebo_runs": 0,
        "trace_count": len(paths),
        "interpolation_max_chord_mm": 0.1,
        "ball_radius_mm": BALL_RADIUS_MM,
        "required_surface_clearance_mm": CLEARANCE_MM,
        "panel": {
            "x_mm": SELECTED_X_MM,
            "thickness_mm": PANEL_THICKNESS_MM,
            "y_extent_mm": [-PANEL_HALF_WIDTH_MM, PANEL_HALF_WIDTH_MM],
            "bottom_mm": WALL_BOTTOM_MM,
            "top_mm": SELECTED_TOP_MM,
            "top_edge_radius_mm": 2.0,
        },
        "selected_clearance": {
            "exact_minimum_ball_center_to_panel_mm": euclidean,
            "exact_minimum_ball_surface_clearance_mm": euclidean - BALL_RADIUS_MM,
            "conservative_box_center_clearance_mm": box,
            "conservative_box_surface_clearance_mm": box - BALL_RADIUS_MM,
            "maximum_abs_ball_y_near_wall_mm": maximum_abs_y,
            "lateral_clearance_to_panel_edge_mm": PANEL_HALF_WIDTH_MM
            - maximum_abs_y
            - BALL_RADIUS_MM,
        },
        "candidate_top_limits": candidates,
        "feed_door": {
            "clear_width_mm": 80.0,
            "clear_height_mm": 80.0,
            "closed_z_mm": [WALL_BOTTOM_MM, WALL_BOTTOM_MM + 80.0],
            "fail_state": "spring_closed",
            "opens_only_at_launcher_deg": [15.0, 20.0],
            "fixed_channel_mm": [[227.0, WALL_BOTTOM_MM], [278.0, 42.0]],
            "moving_mouth_floor_mm": {
                "plus15": [329.2, 26.1],
                "plus20": [331.5, 21.1],
            },
        },
    }
    OUT.write_text(json.dumps(result, indent=2) + "\n")
    print(OUT)
    print(json.dumps(result["selected_clearance"], indent=2))
    for row in candidates:
        if row["wall_x_mm"] in (200, 220, 225, 230, 240, 270):
            print(row)


if __name__ == "__main__":
    main()
