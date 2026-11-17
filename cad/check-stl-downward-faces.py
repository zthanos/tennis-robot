#!/usr/bin/env python3
"""Reject unsupported horizontal downward STL patches above the print bed.

Triangles are grouped only when they are coplanar and share a complete edge.
The reported area is therefore one connected downward face, not the sum of
unrelated islands that merely happen to occur at the same Z height.
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
UnionFind = _components.UnionFind
triangles = _components.triangles


def _key(vertex: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(round(value, 6) for value in vertex)


def downward_patches(path: Path, normal_limit: float = -0.99):
    downward = []
    minimum_z = math.inf
    for triangle in triangles(path):
        minimum_z = min(minimum_z, *(vertex[2] for vertex in triangle))
        a, b, c = triangle
        ab = tuple(b[index] - a[index] for index in range(3))
        ac = tuple(c[index] - a[index] for index in range(3))
        cross = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        magnitude = math.sqrt(sum(value * value for value in cross))
        if magnitude == 0 or cross[2] / magnitude >= normal_limit:
            continue
        z_values = [vertex[2] for vertex in triangle]
        if max(z_values) - min(z_values) > 1e-5:
            continue
        area = abs(cross[2]) / 2.0
        downward.append((triangle, area, sum(z_values) / 3.0))

    groups = UnionFind()
    edge_owner: dict[tuple, int] = {}
    areas: list[float] = []
    heights: list[float] = []
    for triangle, area, height in downward:
        item = groups.add()
        areas.append(area)
        heights.append(height)
        keys = [_key(vertex) for vertex in triangle]
        for left, right in ((0, 1), (1, 2), (2, 0)):
            edge = tuple(sorted((keys[left], keys[right])))
            previous = edge_owner.setdefault(edge, item)
            if abs(heights[previous] - height) <= 1e-5:
                groups.union(item, previous)

    patches: dict[int, dict[str, float]] = {}
    for item, area in enumerate(areas):
        root = groups.find(item)
        patch = patches.setdefault(root, {"area": 0.0, "z": heights[item]})
        patch["area"] += area
    return minimum_z, sorted(patches.values(), key=lambda patch: patch["area"], reverse=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--max-area", type=float, default=40.0)
    parser.add_argument("--bed-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--allow-z", action="append", default=[], metavar="MIN:MAX:CLASS",
        help="classify a known support-free bridge interval without hiding other faces",
    )
    args = parser.parse_args()

    allowed = []
    for value in args.allow_z:
        try:
            lower_text, upper_text, classification = value.split(":", 2)
            lower, upper = float(lower_text), float(upper_text)
        except ValueError:
            parser.error(f"invalid --allow-z {value!r}; expected MIN:MAX:CLASS")
        allowed.append((lower, upper, classification.strip()))

    minimum_z, patches = downward_patches(args.stl)
    elevated = [patch for patch in patches if patch["z"] > minimum_z + args.bed_tolerance]
    worst = elevated[0] if elevated else {"area": 0.0, "z": minimum_z}
    failures = [patch for patch in elevated if patch["area"] > args.max_area + 1e-6]
    unclassified = []
    classified = []
    for patch in failures:
        match = next((item for item in allowed if item[0] - 1e-6 <= patch["z"] <= item[1] + 1e-6), None)
        if match is None:
            unclassified.append(patch)
        else:
            classified.append((patch, match[2]))
    status = "PASS" if not unclassified else "FAIL"
    print(
        f"DOWNWARD_FACES {status} {args.stl}: max {worst['area']:.2f} mm2 "
        f"at Z{worst['z']:.3f}; limit {args.max_area:.2f} mm2; "
        f"violations {len(unclassified)}"
    )
    if failures:
        for patch in failures[:12]:
            print(f"  unsupported {patch['area']:.2f} mm2 at Z{patch['z']:.3f}")
    for patch, classification in classified:
        print(
            f"  CLASSIFIED SUPPORT-FREE: {classification}: "
            f"{patch['area']:.2f} mm2 at Z{patch['z']:.3f}"
        )
    if unclassified:
        print(
            f"PRINTABILITY FAIL: {args.stl} has horizontal downward faces above the print bed",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
