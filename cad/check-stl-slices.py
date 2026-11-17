#!/usr/bin/env python3
"""Reject thin horizontal sections and small printable islands in an STL.

The mesh is intersected at layer mid-planes.  Closed section loops are used
for exact shoelace areas.  The thickness gate is a boundary-to-boundary buffer
test: opposing boundary samples whose connecting segment lies in material
must be at least the requested distance apart.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

_components_path = Path(__file__).with_name("check-stl-components.py")
_spec = importlib.util.spec_from_file_location("check_stl_components", _components_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"cannot load {_components_path}")
_components = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_components)
triangles = _components.triangles

Point = tuple[float, float]


def _key(point: Point) -> tuple[int, int]:
    # STL coordinates and independent triangle interpolation differ by a few
    # 1e-5 mm at shared edges.  A 1 um key joins those numerical twins while
    # remaining far below any printable feature in this gate.
    return (round(point[0] * 1_000), round(point[1] * 1_000))


def _crossing(a, b, z: float) -> Point | None:
    da, db = a[2] - z, b[2] - z
    if (da < 0 and db < 0) or (da > 0 and db > 0) or abs(da - db) < 1e-12:
        return None
    if abs(da) < 1e-9:
        return (a[0], a[1])
    if abs(db) < 1e-9:
        return (b[0], b[1])
    if da * db > 0:
        return None
    fraction = da / (da - db)
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))


def section_loops(mesh, z: float) -> list[list[Point]]:
    points: dict[tuple[int, int], Point] = {}
    edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    for triangle in mesh:
        hits = []
        for a, b in ((triangle[0], triangle[1]), (triangle[1], triangle[2]),
                     (triangle[2], triangle[0])):
            hit = _crossing(a, b, z)
            if hit is not None and _key(hit) not in {_key(p) for p in hits}:
                hits.append(hit)
        if len(hits) == 2:
            left, right = _key(hits[0]), _key(hits[1])
            if left != right:
                points[left], points[right] = hits[0], hits[1]
                edges.add(tuple(sorted((left, right))))

    adjacency: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for left, right in edges:
        adjacency.setdefault(left, []).append(right)
        adjacency.setdefault(right, []).append(left)
    # Boolean meshes can contain tangent section loops sharing one degree-4
    # vertex.  They are still closed printable contours; odd-degree vertices
    # are the actual open/branched error.
    bad = [key for key, neighbours in adjacency.items()
           if len(neighbours) < 2 or len(neighbours) % 2]
    if bad:
        raise ValueError(f"open/branched section at Z{z:.3f}: {len(bad)} vertices")

    unused = set(edges)
    loops = []
    while unused:
        first_edge = next(iter(unused))
        start, current = first_edge
        loop_keys = [start]
        previous = start
        while current != start:
            loop_keys.append(current)
            candidates = [item for item in adjacency[current]
                          if tuple(sorted((current, item))) in unused
                          and item != previous]
            if not candidates:
                raise ValueError(f"ambiguous section at Z{z:.3f}")
            # At a tangent degree-4 vertex, keep the contour on the straightest
            # continuation and leave the other pair for the next loop.
            incoming = (points[current][0] - points[previous][0],
                        points[current][1] - points[previous][1])
            def continuation(candidate):
                outgoing = (points[candidate][0] - points[current][0],
                            points[candidate][1] - points[current][1])
                denominator = math.hypot(*incoming) * math.hypot(*outgoing)
                return ((incoming[0] * outgoing[0] + incoming[1] * outgoing[1])
                        / denominator if denominator else -2)
            following = max(candidates, key=continuation)
            unused.discard(tuple(sorted((previous, current))))
            previous, current = current, following
            if len(loop_keys) > len(edges) + 1:
                raise ValueError(f"non-closing section at Z{z:.3f}")
        unused.discard(tuple(sorted((previous, current))))
        loops.append([points[key] for key in loop_keys])
    # Tangent Boolean edges can trace a zero-area backtrack at one sampled Z.
    # It is not a printable contour and must not become a synthetic island.
    return [loop for loop in loops if polygon_area(loop) > 1e-6]


def polygon_area(loop: list[Point]) -> float:
    return abs(sum(
        left[0] * right[1] - right[0] * left[1]
        for left, right in zip(loop, loop[1:] + loop[:1])
    )) / 2


def contains(loop: list[Point], point: Point) -> bool:
    x, y = point
    inside = False
    for left, right in zip(loop, loop[1:] + loop[:1]):
        if (left[1] > y) != (right[1] > y):
            crossing_x = left[0] + (y - left[1]) * (right[0] - left[0]) / (right[1] - left[1])
            if crossing_x > x:
                inside = not inside
    return inside


def interior_point(loop: list[Point]) -> Point:
    for left, right in zip(loop, loop[1:] + loop[:1]):
        dx, dy = right[0] - left[0], right[1] - left[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            continue
        middle = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
        for sign in (-1, 1):
            candidate = (middle[0] - sign * dy / length * 1e-4,
                         middle[1] + sign * dx / length * 1e-4)
            if contains(loop, candidate):
                return candidate
    raise ValueError("cannot find polygon interior")


def island_areas(loops: list[list[Point]]) -> list[float]:
    samples = [interior_point(loop) for loop in loops]
    parents = []
    for index, sample in enumerate(samples):
        containers = [other for other, loop in enumerate(loops)
                      if other != index and contains(loop, sample)]
        parents.append(min(containers, key=lambda item: polygon_area(loops[item]))
                       if containers else None)
    depths = []
    for index in range(len(loops)):
        depth, parent = 0, parents[index]
        while parent is not None:
            depth += 1
            parent = parents[parent]
        depths.append(depth)
    areas = []
    for index, loop in enumerate(loops):
        if depths[index] % 2 == 0:
            holes = sum(polygon_area(loops[child]) for child, parent in enumerate(parents)
                        if parent == index and depths[child] % 2 == 1)
            areas.append(polygon_area(loop) - holes)
    return areas


def _point_segment_distance(point: Point, left: Point, right: Point):
    dx, dy = right[0] - left[0], right[1] - left[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.inf, left, 0.0
    t = ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length2
    t = max(0.0, min(1.0, t))
    nearest = (left[0] + t * dx, left[1] + t * dy)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1]), nearest, t


def minimum_material_width(loops: list[list[Point]], limit: float) -> float:
    segments = [(loop_index, segment_index, left, right)
                for loop_index, loop in enumerate(loops)
                for segment_index, (left, right) in enumerate(zip(loop, loop[1:] + loop[:1]))]
    arc_positions = []
    perimeters = []
    for loop in loops:
        positions = []
        travelled = 0.0
        for left, right in zip(loop, loop[1:] + loop[:1]):
            positions.append(travelled + math.dist(left, right) / 2)
            travelled += math.dist(left, right)
        arc_positions.append(positions)
        perimeters.append(travelled)
    cell_size = limit
    grid: dict[tuple[int, int], set[int]] = {}
    for item, (_, _, left, right) in enumerate(segments):
        x0 = math.floor(min(left[0], right[0]) / cell_size)
        x1 = math.floor(max(left[0], right[0]) / cell_size)
        y0 = math.floor(min(left[1], right[1]) / cell_size)
        y1 = math.floor(max(left[1], right[1]) / cell_size)
        for cell_x in range(x0, x1 + 1):
            for cell_y in range(y0, y1 + 1):
                grid.setdefault((cell_x, cell_y), set()).add(item)
    best = math.inf
    for loop_index, segment_index, left, right in segments:
        sample = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
        loop_size = len(loops[loop_index])
        sample_cell = (math.floor(sample[0] / cell_size),
                       math.floor(sample[1] / cell_size))
        nearby = set()
        for cell_x in range(sample_cell[0] - 1, sample_cell[0] + 2):
            for cell_y in range(sample_cell[1] - 1, sample_cell[1] + 2):
                nearby.update(grid.get((cell_x, cell_y), ()))
        for item in nearby:
            other_loop, other_index, a, b = segments[item]
            if other_loop == loop_index and ((other_index - segment_index) % loop_size in (0, 1, loop_size - 1)):
                continue
            if other_loop == loop_index:
                arc_delta = abs(arc_positions[loop_index][segment_index]
                                - arc_positions[other_loop][other_index])
                arc_delta = min(arc_delta, perimeters[loop_index] - arc_delta)
                # A polygonal buffer rounds nearby facets on the same local
                # corner; those are not opposing walls.  Ignore boundary
                # samples reachable within two buffer diameters along the
                # same contour.
                if arc_delta < 4 * limit:
                    continue
            distance, nearest, t = _point_segment_distance(sample, a, b)
            if distance >= min(best, limit) or t <= 1e-4 or t >= 1 - 1e-4:
                continue
            midpoint = ((sample[0] + nearest[0]) / 2, (sample[1] + nearest[1]) / 2)
            if sum(contains(loop, midpoint) for loop in loops) % 2 == 1:
                best = distance
    return best


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--layer-height", type=float, default=0.2)
    parser.add_argument("--start-z", type=float, default=0.5)
    parser.add_argument("--min-thickness", type=float, default=1.6)
    parser.add_argument("--min-island-area", type=float, default=25.0)
    args = parser.parse_args()

    mesh = list(triangles(args.stl))
    if not mesh:
        parser.error(f"no triangles in {args.stl}")
    minimum_z = min(vertex[2] for triangle in mesh for vertex in triangle)
    maximum_z = max(vertex[2] for triangle in mesh for vertex in triangle)
    # Z thresholds are model/print coordinates, not offsets from a tessellation
    # ridge that may extend a few hundredths below nominal Z0.
    z = max(minimum_z + 1e-4,
            args.start_z + args.layer_height / 2 + 1e-4)
    failures = []
    checked = 0
    worst_width = math.inf
    smallest_island = math.inf
    while z < maximum_z - 1e-5:
        last_error = None
        for jitter in (0.0, 0.0013, -0.0013):
            try:
                loops = section_loops(mesh, z + jitter)
                break
            except ValueError as error:
                last_error = error
        else:
            raise last_error  # type: ignore[misc]
        areas = island_areas(loops)
        width = minimum_material_width(loops, args.min_thickness)
        worst_width = min(worst_width, width)
        smallest_island = min(smallest_island, *areas)
        thin = width < args.min_thickness - 1e-3
        small = [area for area in areas if area < args.min_island_area - 1e-3]
        if thin or small:
            failures.append((z, width, small, len(areas)))
        checked += 1
        z += args.layer_height

    status = "PASS" if not failures else "FAIL"
    width_text = (f"{worst_width:.3f} mm" if math.isfinite(worst_width)
                  else f">={args.min_thickness:.3f} mm")
    print(f"SLICE_GEOMETRY {status} {args.stl}: {checked} sections; "
          f"buffer width {width_text} (limit {args.min_thickness:.3f}); "
          f"smallest island {smallest_island:.2f} mm2 (limit {args.min_island_area:.2f})")
    for z, width, small, count in failures[:16]:
        details = []
        if width < args.min_thickness - 1e-3:
            details.append(f"width {width:.3f}")
        if small:
            details.append("small islands " + ",".join(f"{area:.2f}" for area in small[:6]))
        print(f"  Z{z:.3f}: {count} islands; {'; '.join(details)}")
    if failures:
        print(f"PRINTABILITY FAIL: {args.stl} failed {len(failures)} section(s)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
