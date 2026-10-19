#!/usr/bin/env python3
"""Fail unless an STL contains the declared number of connected bodies.

Connectivity is topological: triangles belong to the same body when they
share a vertex.  OpenSCAD writes coincident vertices with identical values;
rounding to one nanometre in model units also tolerates ASCII/binary output
formatting without bridging a real print gap.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path


class UnionFind:
    def __init__(self) -> None:
        self.parent: list[int] = []

    def add(self) -> int:
        item = len(self.parent)
        self.parent.append(item)
        return item

    def find(self, item: int) -> int:
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, left: int, right: int) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def triangles(path: Path):
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            for index in range(count):
                offset = 84 + index * 50 + 12
                yield tuple(
                    struct.unpack_from("<fff", data, offset + vertex * 12)
                    for vertex in range(3)
                )
            return

    vertices = []
    for raw_line in data.decode("utf-8").splitlines():
        fields = raw_line.strip().split()
        if len(fields) == 4 and fields[0] == "vertex":
            vertices.append(tuple(float(value) for value in fields[1:]))
            if len(vertices) == 3:
                yield tuple(vertices)
                vertices = []
    if vertices:
        raise ValueError(f"truncated ASCII STL triangle in {path}")


def _signed_volume(triangle_group) -> float:
    volume = 0.0
    for a, b, c in triangle_group:
        volume += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            + a[1] * (b[2] * c[0] - b[0] * c[2])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        ) / 6.0
    return volume


def component_count(path: Path) -> int:
    groups = UnionFind()
    first_triangle_at_vertex: dict[tuple[float, float, float], int] = {}
    all_triangles = []
    for triangle in triangles(path):
        triangle_id = groups.add()
        all_triangles.append(triangle)
        for vertex in triangle:
            key = tuple(round(coordinate, 9) for coordinate in vertex)
            previous = first_triangle_at_vertex.setdefault(key, triangle_id)
            groups.union(triangle_id, previous)
    if not all_triangles:
        raise ValueError(f"no triangles in {path}")
    surface_groups: dict[int, list] = {}
    for item, triangle in enumerate(all_triangles):
        surface_groups.setdefault(groups.find(item), []).append(triangle)

    # A single hollow solid legitimately has disconnected boundary shells:
    # its outside shell and one or more closed cavity shells.  OpenSCAD orients
    # those with opposite signed volume.  Count only shells having the same
    # orientation as the largest shell; those are independent material bodies.
    volumes = [_signed_volume(group) for group in surface_groups.values()]
    reference = max(volumes, key=abs)
    reference_sign = 1 if reference > 0 else -1
    return sum(1 for volume in volumes if volume * reference_sign > 1e-9)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("expected", type=int)
    args = parser.parse_args()
    actual = component_count(args.stl)
    print(f"STL_COMPONENTS {args.stl}: {actual} (expected {args.expected})")
    if actual != args.expected:
        parser.error(f"{args.stl} has {actual} connected bodies; expected {args.expected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
