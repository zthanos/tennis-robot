#!/usr/bin/env python3
"""Generate the authoritative Option A handoff-ramp collision mesh.

The mesh reproduces ``short_handoff_ramp()`` from
``cad/collector-intake-v1/option-a/option-a.scad``: a solid wedge under the
``oa_ramp_z`` smoothstep sheet (520 mm -> 420 mm, 1.5 mm -> 35 mm, 180 mm
wide) plus the two 4 mm side walls standing ``oa_ramp_wall_h = 18 mm`` above
the local sheet height.

Units are millimetres in the CAD ground frame with the packaging-variant
functional shift baked in, matching the existing intake mesh convention (the
URDF places the link at ``xyz = 0 0 -base_link_height`` and scales by 0.001).

This replaces ``compact_relieved_handoff_ramp.stl``, whose 460 -> 420 mm run is
the compact packaging study's own steeper ramp, not the frozen Option A ramp.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402

DEFAULT_OUTPUT = ROOT / "ros2_ws/src/tennis_robot/meshes/option_a_handoff_ramp.stl"


def _quad(triangles: list, a, b, c, d) -> None:
    triangles.append((a, b, c))
    triangles.append((a, c, d))


def _box_strip(triangles: list, stations, y0: float, y1: float, z_low, z_high) -> None:
    """Sweep an axis-aligned rectangular cross-section along the x stations."""

    for index in range(len(stations) - 1):
        x0, x1 = stations[index], stations[index + 1]
        z0l, z0h = z_low(x0), z_high(x0)
        z1l, z1h = z_low(x1), z_high(x1)
        # top
        _quad(triangles, (x0, y0, z0h), (x0, y1, z0h), (x1, y1, z1h), (x1, y0, z1h))
        # bottom
        _quad(triangles, (x0, y0, z0l), (x1, y0, z1l), (x1, y1, z1l), (x0, y1, z0l))
        # +y side
        _quad(triangles, (x0, y1, z0l), (x1, y1, z1l), (x1, y1, z1h), (x0, y1, z0h))
        # -y side
        _quad(triangles, (x0, y0, z0l), (x0, y0, z0h), (x1, y0, z1h), (x1, y0, z1l))
    x_front, x_rear = stations[0], stations[-1]
    _quad(triangles, (x_front, y0, z_low(x_front)), (x_front, y0, z_high(x_front)),
          (x_front, y1, z_high(x_front)), (x_front, y1, z_low(x_front)))
    _quad(triangles, (x_rear, y0, z_low(x_rear)), (x_rear, y1, z_low(x_rear)),
          (x_rear, y1, z_high(x_rear)), (x_rear, y0, z_high(x_rear)))


def build_triangles(variant: str, steps: int = 48) -> list:
    shift_mm = 1000.0 * geom.packaging_shift_x_m(variant)
    front_mm = 1000.0 * geom.RAMP_FRONT_X_M
    rear_mm = 1000.0 * geom.RAMP_REAR_X_M
    half_width_mm = 1000.0 * geom.RAMP_WIDTH_M / 2.0
    wall_mm = 1000.0 * geom.RAMP_WALL_THICKNESS_M
    wall_h_mm = 1000.0 * geom.RAMP_WALL_HEIGHT_M

    stations = [front_mm - (front_mm - rear_mm) * i / steps for i in range(steps + 1)]

    def sheet_z(x_mm: float) -> float:
        return 1000.0 * geom.ramp_z_m(x_mm / 1000.0)

    triangles: list = []
    _box_strip(triangles, stations, -half_width_mm, half_width_mm,
               lambda x: 0.0, sheet_z)
    for sign in (-1.0, 1.0):
        inner = sign * half_width_mm
        outer = sign * (half_width_mm + wall_mm)
        y0, y1 = min(inner, outer), max(inner, outer)
        _box_strip(triangles, stations, y0, y1,
                   lambda x: 0.0, lambda x: sheet_z(x) + wall_h_mm)

    return [
        tuple((vertex[0] + shift_mm, vertex[1], vertex[2]) for vertex in triangle)
        for triangle in triangles
    ]


def to_ascii_stl(name: str, triangles: list) -> str:
    lines = [f"solid {name}"]
    for a, b, c in triangles:
        ux, uy, uz = (b[i] - a[i] for i in range(3))
        vx, vy, vz = (c[i] - a[i] for i in range(3))
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        lines.append(f"  facet normal {nx/length:.6f} {ny/length:.6f} {nz/length:.6f}")
        lines.append("    outer loop")
        for vertex in (a, b, c):
            lines.append(f"      vertex {vertex[0]:.4f} {vertex[1]:.4f} {vertex[2]:.4f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--variant", default=geom.CAD_ALIGNED_VARIANT)
    parser.add_argument("--steps", type=int, default=48)
    args = parser.parse_args()

    triangles = build_triangles(args.variant, args.steps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(to_ascii_stl("option_a_handoff_ramp", triangles),
                           encoding="utf-8")
    xs = [v[0] for t in triangles for v in t]
    ys = [v[1] for t in triangles for v in t]
    zs = [v[2] for t in triangles for v in t]
    print(f"wrote {args.output} ({len(triangles)} triangles)")
    print(f"x {min(xs):.2f}..{max(xs):.2f} mm  y {min(ys):.2f}..{max(ys):.2f} mm  "
          f"z {min(zs):.2f}..{max(zs):.2f} mm  (variant {args.variant})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
