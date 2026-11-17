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


def minimum_material_width_witness(
    loops: list[list[Point]], limit: float
) -> tuple[float, tuple[Point, Point] | None]:
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
    witness = None
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
                witness = (sample, nearest)
    return best, witness


def minimum_material_width(loops: list[list[Point]], limit: float) -> float:
    return minimum_material_width_witness(loops, limit)[0]


def write_section_svg(
    path: Path,
    loops: list[list[Point]],
    z: float,
    witness: tuple[Point, Point] | None,
    width: float,
    areas: list[float],
) -> None:
    points = [point for loop in loops for point in loop]
    x0, x1 = min(point[0] for point in points), max(point[0] for point in points)
    y0, y1 = min(point[1] for point in points), max(point[1] for point in points)
    margin = max(x1 - x0, y1 - y0) * 0.06 + 2
    x0, x1, y0, y1 = x0 - margin, x1 + margin, y0 - margin, y1 + margin
    scale = min(1000 / (x1 - x0), 700 / (y1 - y0))

    def screen(point: Point) -> tuple[float, float]:
        return ((point[0] - x0) * scale + 30, (y1 - point[1]) * scale + 75)

    commands = []
    for loop in loops:
        first = screen(loop[0])
        commands.append(f"M {first[0]:.2f},{first[1]:.2f}")
        commands.extend(
            f"L {sx:.2f},{sy:.2f}" for sx, sy in (screen(point) for point in loop[1:])
        )
        commands.append("Z")
    view_width = (x1 - x0) * scale + 60
    view_height = (y1 - y0) * scale + 105
    width_text = f"{width:.3f} mm" if math.isfinite(width) else ">= limit"
    overlay = ""
    if witness is not None:
        left, right = screen(witness[0]), screen(witness[1])
        overlay = (
            f'<line x1="{left[0]:.2f}" y1="{left[1]:.2f}" '
            f'x2="{right[0]:.2f}" y2="{right[1]:.2f}" '
            'stroke="#d7263d" stroke-width="3"/>'
            f'<circle cx="{left[0]:.2f}" cy="{left[1]:.2f}" r="5" fill="#d7263d"/>'
            f'<circle cx="{right[0]:.2f}" cy="{right[1]:.2f}" r="5" fill="#d7263d"/>'
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f'''<svg xmlns="http://www.w3.org/2000/svg" width="{view_width:.0f}" height="{view_height:.0f}" viewBox="0 0 {view_width:.0f} {view_height:.0f}">
<rect width="100%" height="100%" fill="#f7f7f4"/>
<text x="30" y="30" font-family="sans-serif" font-size="20" font-weight="bold">Z={z:.3f} mm</text>
<text x="30" y="55" font-family="sans-serif" font-size="16">width {width_text}; islands {', '.join(f'{area:.2f}' for area in areas)} mm2</text>
<path d="{' '.join(commands)}" fill="#1f6f8b" fill-rule="evenodd" stroke="#173f4f" stroke-width="1"/>
{overlay}
</svg>\n''',
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--layer-height", type=float, default=0.2)
    parser.add_argument("--start-z", type=float, default=0.5)
    parser.add_argument("--min-thickness", type=float, default=1.6)
    parser.add_argument("--min-island-area", type=float, default=25.0)
    parser.add_argument(
        "--failure-image", type=Path,
        help="write an SVG of the worst failing section, with the width witness in red",
    )
    parser.add_argument(
        "--allow-z", action="append", default=[], metavar="MIN:MAX:CLASS",
        help=("classify a known non-structural Z interval; failures inside it are "
              "reported but do not fail the export"),
    )
    args = parser.parse_args()

    allowed = []
    for value in args.allow_z:
        try:
            lower_text, upper_text, classification = value.split(":", 2)
            lower, upper = float(lower_text), float(upper_text)
        except ValueError:
            parser.error(f"invalid --allow-z {value!r}; expected MIN:MAX:CLASS")
        if lower > upper or not classification.strip():
            parser.error(f"invalid --allow-z {value!r}")
        allowed.append((lower, upper, classification.strip()))

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
    worst_failure = None
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
        width, witness = minimum_material_width_witness(loops, args.min_thickness)
        worst_width = min(worst_width, width)
        smallest_island = min(smallest_island, *areas)
        thin = width < args.min_thickness - 1e-3
        small = [area for area in areas if area < args.min_island_area - 1e-3]
        if thin or small:
            failures.append((z, width, small, len(areas)))
            severity = min(
                width / args.min_thickness if thin else math.inf,
                min(areas) / args.min_island_area if small else math.inf,
            )
            if worst_failure is None or severity < worst_failure[0]:
                worst_failure = (severity, z, loops, witness, width, areas)
        checked += 1
        z += args.layer_height

    unwaived = []
    waived = []
    for failure in failures:
        match = next((item for item in allowed if item[0] - 1e-6 <= failure[0] <= item[1] + 1e-6), None)
        if match is None:
            unwaived.append(failure)
        else:
            waived.append((failure, match[2]))
    status = "PASS" if not unwaived else "FAIL"
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
    if waived:
        by_class: dict[str, list[float]] = {}
        for (z, _width, _small, _count), classification in waived:
            by_class.setdefault(classification, []).append(z)
        for classification, heights in by_class.items():
            print(
                f"  CLASSIFIED NON-STRUCTURAL: {classification}: "
                f"{len(heights)} section(s), Z{min(heights):.3f}..{max(heights):.3f}"
            )
    if unwaived:
        if args.failure_image and worst_failure is not None:
            _, image_z, image_loops, image_witness, image_width, image_areas = worst_failure
            write_section_svg(
                args.failure_image, image_loops, image_z, image_witness,
                image_width, image_areas,
            )
            print(f"  failure image: {args.failure_image}")
        print(
            f"PRINTABILITY FAIL: {args.stl} failed {len(unwaived)} unclassified section(s)",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
