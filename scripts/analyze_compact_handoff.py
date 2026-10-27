#!/usr/bin/env python3
"""Measure the compact ramp/chute contact geometry for a tennis-ball centre.

All coordinates are compact robot CAD coordinates in millimetres.  This tool is
diagnostic only: it reads generated STL collision surfaces and never modifies
authoritative geometry.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from export_compact_cad_envelopes import stl_vertices


def closest_point_on_triangle(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    """Return the closest point using the Voronoi-region test from RTCD."""
    a, b, c = triangle
    ab, ac = b - a, c - a
    ap = point - a
    d1, d2 = np.dot(ab, ap), np.dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        return a
    bp = point - b
    d3, d4 = np.dot(ab, bp), np.dot(ac, bp)
    if d3 >= 0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return a + (d1 / (d1 - d3)) * ab
    cp = point - c
    d5, d6 = np.dot(ab, cp), np.dot(ac, cp)
    if d6 >= 0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return a + (d2 / (d2 - d6)) * ac
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b)
    denom = 1.0 / (va + vb + vc)
    return a + vb * denom * ab + vc * denom * ac


def nearest_surface(path: Path, point: np.ndarray, offset_x: float = 0.0) -> dict:
    triangles = stl_vertices(path).reshape((-1, 3, 3)).copy()
    triangles[:, :, 0] += offset_x
    closest = min(
        (closest_point_on_triangle(point, tri) for tri in triangles),
        key=lambda q: float(np.dot(point - q, point - q)),
    )
    delta = point - closest
    distance = float(np.linalg.norm(delta))
    normal = delta / distance if distance > 1e-12 else np.zeros(3)
    return {"point": closest, "distance": distance, "normal": normal}


def fmt(vector: np.ndarray) -> str:
    return "(" + ", ".join(f"{value:.3f}" for value in vector) + ")"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ball-centre", nargs=3, type=float, default=(403.5, 0, 33.0))
    parser.add_argument("--ball-radius", type=float, default=33.0)
    parser.add_argument("--relieved-bin", type=Path,
                        default=Path("ros2_ws/src/tennis_robot/meshes/compact_relieved_bin.stl"))
    parser.add_argument("--receiving-chute", type=Path,
                        default=Path("runtime/compact_receiving_chute.stl"))
    parser.add_argument("--relieved-ramp", type=Path,
                        default=Path("ros2_ws/src/tennis_robot/meshes/compact_relieved_handoff_ramp.stl"))
    parser.add_argument("--original-ramp", type=Path,
                        default=Path("runtime/compact_unrelieved_handoff_ramp.stl"))
    args = parser.parse_args()

    centre = np.asarray(args.ball_centre, dtype=float)
    surfaces = (
        ("relieved_bin_collision", args.relieved_bin, -100.0),
        ("receiving_chute_source", args.receiving_chute, 0.0),
        ("relieved_ramp_collision", args.relieved_ramp, 0.0),
        ("original_ramp_source", args.original_ramp, 0.0),
    )
    print(f"ball centre mm: {fmt(centre)}; radius: {args.ball_radius:.3f}")
    for name, path, offset_x in surfaces:
        if not path.exists():
            print(f"{name}: skipped (export not found: {path})")
            continue
        result = nearest_surface(path, centre, offset_x)
        depth = args.ball_radius - result["distance"]
        print(
            f"{name}: closest={fmt(result['point'])} "
            f"normal(surface_to_ball)={fmt(result['normal'])} "
            f"distance={result['distance']:.3f} depth={depth:.3f}"
        )

    # A centre moving from +X toward -X meets a vertical front datum x=d at
    # d+R.  The wheel-first datum is an independently protected CAD value.
    if not args.receiving_chute.exists():
        return 0
    chute_front_x = max(stl_vertices(args.receiving_chute)[:, 0])
    chute_first_x = chute_front_x + args.ball_radius
    wheel_first_x = 481.2 - 100.0
    print(f"chute front max X: {chute_front_x:.3f}")
    print(f"first possible chute contact centre X: {chute_first_x:.3f}")
    print(f"protected wheel-contact centre X: {wheel_first_x:.3f}")
    print(
        "chronological order (+X toward -X): "
        + ("CHUTE BEFORE WHEEL" if chute_first_x > wheel_first_x else "WHEEL BEFORE CHUTE")
        + f"; lead={abs(chute_first_x - wheel_first_x):.3f} mm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
