#!/usr/bin/env python3
"""Reject unsupported horizontal downward STL patches above the print bed.

Triangles are grouped only when they are coplanar and share a complete edge.
The reported area is therefore one connected downward face, not the sum of
unrelated islands that merely happen to occur at the same Z height.

A patch is a bridge, and is excluded from the area limit, only when mesh
topology shows material descending below both ends of one footprint axis and
the unsupported span is at most 50 mm.  A large face supported at only one end
is still unsupported.
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

_slices_path = Path(__file__).with_name("check-stl-slices.py")
_slices_spec = importlib.util.spec_from_file_location("check_stl_slices", _slices_path)
if _slices_spec is None or _slices_spec.loader is None:
    raise RuntimeError(f"cannot load {_slices_path}")
_slices = importlib.util.module_from_spec(_slices_spec)
_slices_spec.loader.exec_module(_slices)


def _key(vertex: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(round(value, 6) for value in vertex)


def downward_patches(path: Path, normal_limit: float = -0.99,
                     max_bridge_span: float = 50.0):
    downward = []
    minimum_z = math.inf
    mesh = list(triangles(path))
    edge_owners: dict[tuple, list[int]] = {}
    for mesh_item, triangle in enumerate(mesh):
        minimum_z = min(minimum_z, *(vertex[2] for vertex in triangle))
        keys = [_key(vertex) for vertex in triangle]
        for left, right in ((0, 1), (1, 2), (2, 0)):
            edge = tuple(sorted((keys[left], keys[right])))
            edge_owners.setdefault(edge, []).append(mesh_item)
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
        downward.append((mesh_item, triangle, area, sum(z_values) / 3.0))

    groups = UnionFind()
    edge_owner: dict[tuple, int] = {}
    areas: list[float] = []
    heights: list[float] = []
    for _mesh_item, triangle, area, height in downward:
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
        patch = patches.setdefault(
            root, {"area": 0.0, "z": heights[item], "items": []}
        )
        patch["area"] += area
        patch["items"].append(item)

    for patch in patches.values():
        patch_mesh_items = {downward[item][0] for item in patch["items"]}
        patch_vertices = [
            vertex
            for item in patch["items"]
            for vertex in downward[item][1]
        ]
        x_values = [vertex[0] for vertex in patch_vertices]
        y_values = [vertex[1] for vertex in patch_vertices]
        patch["bbox"] = (min(x_values), min(y_values),
                         max(x_values), max(y_values))
        dominant_axis = (0 if max(x_values) - min(x_values)
                         >= max(y_values) - min(y_values) else 1)
        support_points = []
        patch_edges: dict[tuple, tuple[int, tuple, tuple]] = {}
        for item in patch["items"]:
            triangle = downward[item][1]
            keys = [_key(vertex) for vertex in triangle]
            for left, right in ((0, 1), (1, 2), (2, 0)):
                edge = tuple(sorted((keys[left], keys[right])))
                count, _left_vertex, _right_vertex = patch_edges.get(
                    edge, (0, triangle[left], triangle[right])
                )
                patch_edges[edge] = (count + 1, triangle[left], triangle[right])
                neighbours = [
                    owner for owner in edge_owners.get(edge, [])
                    if owner not in patch_mesh_items
                ]
                if any(
                    min(vertex[2] for vertex in mesh[owner]) < patch["z"] - 1e-5
                    for owner in neighbours
                ):
                    midpoint = tuple(
                        (triangle[left][axis] + triangle[right][axis]) / 2.0
                        for axis in range(3)
                    )
                    support_points.append((midpoint[0], midpoint[1]))

        # Boolean tessellation can split the two sides of a support junction
        # into unequal edge subdivisions, so exact STL edge adjacency alone
        # is insufficient.  Probe just below every boundary edge as a
        # topology-preserving fallback; only actual material below the face
        # contributes a support coordinate.
        try:
            below_loops = _slices.section_loops(mesh, patch["z"] - 0.05)
        except ValueError:
            below_loops = _slices.section_loops(mesh, patch["z"] - 0.0513)

        def material_below(point):
            return sum(_slices.contains(loop, point) for loop in below_loops) % 2 == 1

        for count, left_vertex, right_vertex in patch_edges.values():
            if count != 1:
                continue
            dx = right_vertex[0] - left_vertex[0]
            dy = right_vertex[1] - left_vertex[1]
            length = math.hypot(dx, dy)
            if length < 1e-8:
                continue
            midpoint = ((left_vertex[0] + right_vertex[0]) / 2.0,
                        (left_vertex[1] + right_vertex[1]) / 2.0)
            normal = (-dy / length * 0.05, dx / length * 0.05)
            probes = [midpoint,
                      (midpoint[0] + normal[0], midpoint[1] + normal[1]),
                      (midpoint[0] - normal[0], midpoint[1] - normal[1])]
            if any(material_below(point) for point in probes):
                support_points.append(midpoint)

        # A bridge may span either footprint axis: a long face resting on two
        # longitudinal rails is supported across its short axis.  Accept the
        # shortest axis whose two ends both have proven material below.
        spans = []
        for axis, coordinates in enumerate((x_values, y_values)):
            low, high = min(coordinates), max(coordinates)
            span = high - low
            endpoint_tolerance = max(0.25, min(5.0, span * 0.25))
            supported_low = any(
                point[axis] <= low + endpoint_tolerance
                for point in support_points
            )
            supported_high = any(
                point[axis] >= high - endpoint_tolerance
                for point in support_points
            )
            if (span <= max_bridge_span + 1e-6
                    and supported_low and supported_high):
                spans.append(span)
        dominant_coordinates = x_values if dominant_axis == 0 else y_values
        patch["span"] = (min(spans) if spans
                         else max(dominant_coordinates)-min(dominant_coordinates))
        patch["bridge"] = bool(spans)
        del patch["items"]
    return minimum_z, sorted(
        patches.values(), key=lambda patch: patch["area"], reverse=True
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--max-area", type=float, default=40.0)
    parser.add_argument("--max-bridge-span", type=float, default=50.0)
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

    minimum_z, patches = downward_patches(
        args.stl, max_bridge_span=args.max_bridge_span
    )
    elevated = [patch for patch in patches if patch["z"] > minimum_z + args.bed_tolerance]
    bridges = [patch for patch in elevated if patch["bridge"]]
    unsupported = [patch for patch in elevated if not patch["bridge"]]
    worst = unsupported[0] if unsupported else {"area": 0.0, "z": minimum_z}
    failures = [
        patch for patch in unsupported
        if patch["area"] > args.max_area + 1e-6
    ]
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
        f"bridges {len(bridges)}; violations {len(unclassified)}"
    )
    if failures:
        for patch in failures[:12]:
            x0, y0, x1, y1 = patch["bbox"]
            print(
                f"  unsupported {patch['area']:.2f} mm2 at Z{patch['z']:.3f}; "
                f"bbox X{x0:.2f}..{x1:.2f} Y{y0:.2f}..{y1:.2f}"
            )
    for patch, classification in classified:
        print(
            f"  CLASSIFIED SUPPORT-FREE: {classification}: "
            f"{patch['area']:.2f} mm2 at Z{patch['z']:.3f}"
        )
    for patch in bridges[:12]:
        if patch["area"] > args.max_area + 1e-6:
            print(
                f"  SUPPORTED BRIDGE: {patch['area']:.2f} mm2 at "
                f"Z{patch['z']:.3f}; span {patch['span']:.2f} mm"
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
