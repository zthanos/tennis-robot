#!/usr/bin/env python3
"""Verify that declared straight holes are open along their full axes.

Each declaration is an axis segment.  Any STL triangle intersecting the open
segment means material blocks the nominal hole, so the check fails.  This is
deliberately independent of the source CAD and catches a lip, skin or bridge
that accidentally closes an otherwise correctly positioned bore.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

_components_path = Path(__file__).with_name("check-stl-components.py")
_components_spec = importlib.util.spec_from_file_location(
    "check_stl_components", _components_path
)
if _components_spec is None or _components_spec.loader is None:
    raise RuntimeError(f"cannot load {_components_path}")
_components = importlib.util.module_from_spec(_components_spec)
_components_spec.loader.exec_module(_components)
triangles = _components.triangles


def _subtract(left, right):
    return tuple(left[index] - right[index] for index in range(3))


def _dot(left, right):
    return sum(left[index] * right[index] for index in range(3))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def axis_intersections(mesh, origin, direction, length, epsilon=1e-7):
    """Return unique distances where a finite ray crosses mesh triangles."""
    hits = set()
    for a, b, c in mesh:
        edge_ab = _subtract(b, a)
        edge_ac = _subtract(c, a)
        perpendicular = _cross(direction, edge_ac)
        determinant = _dot(edge_ab, perpendicular)
        if abs(determinant) <= epsilon:
            continue
        inverse = 1.0 / determinant
        offset = _subtract(origin, a)
        u = inverse * _dot(offset, perpendicular)
        if u < -epsilon or u > 1.0 + epsilon:
            continue
        side = _cross(offset, edge_ab)
        v = inverse * _dot(direction, side)
        if v < -epsilon or u + v > 1.0 + epsilon:
            continue
        distance = inverse * _dot(edge_ac, side)
        if epsilon < distance < length - epsilon:
            hits.add(round(distance, 6))
    return sorted(hits)


def parse_hole(value: str):
    fields = value.split(",")
    if len(fields) != 8:
        raise argparse.ArgumentTypeError(
            "hole must be label,x0,y0,z0,x1,y1,z1,diameter"
        )
    label = fields[0]
    try:
        start = tuple(float(item) for item in fields[1:4])
        end = tuple(float(item) for item in fields[4:7])
        diameter = float(fields[7])
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if start == end or diameter <= 0:
        raise argparse.ArgumentTypeError("hole axis must have length and positive diameter")
    return label, start, end, diameter


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument(
        "--hole",
        action="append",
        required=True,
        type=parse_hole,
        help="label,x0,y0,z0,x1,y1,z1,diameter",
    )
    args = parser.parse_args()

    mesh = list(triangles(args.stl))
    failures = []
    for label, start, end, diameter in args.hole:
        vector = _subtract(end, start)
        length = math.sqrt(_dot(vector, vector))
        direction = tuple(component / length for component in vector)
        hits = axis_intersections(mesh, start, direction, length)
        if hits:
            failures.append((label, diameter, hits))

    status = "PASS" if not failures else "FAIL"
    print(
        f"THROUGH_HOLES {status} {args.stl}: "
        f"{len(args.hole) - len(failures)}/{len(args.hole)} axes clear"
    )
    for label, diameter, hits in failures:
        positions = ", ".join(f"{distance:.3f}" for distance in hits[:12])
        print(f"  blocked {label} (Ø{diameter:g}) at axis distances {positions} mm")
    if failures:
        print(
            f"THROUGH-HOLE FAIL: {args.stl} contains material on a declared hole axis",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
