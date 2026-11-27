#!/usr/bin/env python3
"""Tower-stability gate: reject tall, slender islands that print on their own.

The mesh is sliced at layer mid-planes and every section is rasterised on a
fine grid (4-connected cells, so a corner touch is not a connection); gaps up
to 6 mm inside one island (a bore through a post) do not split it.  On each
layer the main island is the largest island of the largest piece printed so
far.  Every other island belongs to a tower: a chain of overlapping non-main
islands from the bed (or where it left the main island) up to the layer where
it rejoins the main island.  A thin bed tab does not count as a joint, because
the chain is traced per layer.  The gate fails when

    (tower height) / sqrt(smallest island area along the tower) > limit

(default 3).  Such towers detach or sway while they print alone; the
ramp-entry launcher pusher v2 failed twice this way (2026-11-26).
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

_slices_path = Path(__file__).with_name("check-stl-slices.py")
_spec = importlib.util.spec_from_file_location("check_stl_slices", _slices_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"cannot load {_slices_path}")
_slices = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_slices)


def rasterise(loops, x0: float, y0: float, nx: int, ny: int, cell: float):
    """Even-odd scanline fill of all loops (holes included) at cell centres."""
    grid = np.zeros((ny, nx), dtype=bool)
    if not loops:
        return grid
    segments = []
    for loop in loops:
        pts = np.asarray(loop, dtype=float)
        segments.append(np.stack([pts, np.roll(pts, -1, axis=0)], axis=1))
    seg = np.concatenate(segments)
    ax, ay = seg[:, 0, 0], seg[:, 0, 1]
    bx, by = seg[:, 1, 0], seg[:, 1, 1]
    xs = x0 + (np.arange(nx) + 0.5) * cell
    for row in range(ny):
        y = y0 + (row + 0.5) * cell
        hit = (ay > y) != (by > y)
        if not hit.any():
            continue
        cx = np.sort(ax[hit] + (y - ay[hit]) * (bx[hit] - ax[hit]) / (by[hit] - ay[hit]))
        for left, right in zip(cx[0::2], cx[1::2]):
            grid[row, (xs >= left) & (xs < right)] = True
    return grid


def sections(path: Path, layer: float, cell: float, gap: float):
    mesh = list(_slices.triangles(path))
    pts = np.array([v for t in mesh for v in t], dtype=float)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    x0, y0 = lo[0] - cell, lo[1] - cell
    nx = int(math.ceil((hi[0] - lo[0]) / cell)) + 3
    ny = int(math.ceil((hi[1] - lo[1]) / cell)) + 3
    heights, labels, areas = [], [], []
    z = lo[2] + layer / 2
    while z < hi[2]:
        loops = None
        for nudge in (1e-3, 4e-3, -3e-3, 9e-3):
            try:
                loops = _slices.section_loops(mesh, z + nudge)
                break
            except ValueError:
                continue
        if loops is None:
            raise SystemExit(f"TOWER_STABILITY ERROR {path}: cannot section Z{z:.2f}")
        grid = rasterise(loops, x0, y0, nx, ny, cell)
        # Gaps up to `gap` (a bore through a post, a slot) do not split an
        # island: label on a closed copy, measure on the real cells.
        radius = int(round(gap / 2 / cell))
        yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
        closed = ndimage.binary_closing(grid, structure=(xx * xx + yy * yy) <= radius * radius,
                                        border_value=0)
        lab, count = ndimage.label(closed | grid)
        lab = lab * grid
        present = np.unique(lab)
        present = present[present > 0]
        remap = np.zeros(count + 1, dtype=lab.dtype)
        remap[present] = np.arange(1, len(present) + 1)
        lab = remap[lab]
        count = len(present)
        area = ndimage.sum(grid, lab, index=np.arange(1, count + 1)) * cell * cell
        heights.append(z - lo[2])
        labels.append(lab)
        areas.append(np.asarray(area, dtype=float))
        z += layer
    return heights, labels, areas


def main_islands(labels, areas, layer):
    """Main island per layer.

    Pieces are the 3D-connected parts of what has been printed so far (joined
    only through the bed).  The main piece is the one with the largest printed
    volume; its largest island on the layer is the main island.
    """
    parent: dict[int, int] = {}
    volume: dict[int, float] = {}

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    result = []
    previous: dict[int, int] = {}
    next_id = 0
    for k, lab in enumerate(labels):
        current: dict[int, int] = {}
        for island in range(1, len(areas[k]) + 1):
            below = set()
            if k:
                for prev in set(np.unique(labels[k - 1][lab == island])) - {0}:
                    below.add(find(previous[prev]))
            if not below:
                parent[next_id], volume[next_id] = next_id, 0.0
                below = {next_id}
                next_id += 1
            keep = max(below, key=lambda b: volume[b])
            for other in below - {keep}:
                parent[other] = keep
                volume[keep] += volume[other]
            volume[keep] += float(areas[k][island - 1]) * layer
            current[island] = keep
        current = {island: find(piece) for island, piece in current.items()}
        if current:
            main_piece = max(set(current.values()), key=lambda b: volume[b])
            members = [i for i, piece in current.items() if piece == main_piece]
            result.append(max(members, key=lambda i: areas[k][i - 1]))
        else:
            result.append(0)
        previous = current
    return result


def towers(heights, labels, areas, layer):
    """Chains of non-main islands, linked by overlap between layers.

    A chain stands on its own from its lowest layer (the bed, or where it left
    the main island) to the layer where it rejoins the main island (or its own
    top).  Its minimum area is the smallest island in the chain, so a slender
    leg is not hidden by a wide part it only meets higher up.
    """
    main = main_islands(labels, areas, layer)
    parent: dict[tuple, tuple] = {}

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    for k, lab in enumerate(labels):
        for island in range(1, len(areas[k]) + 1):
            if island == main[k]:
                continue
            parent[(k, island)] = (k, island)
            if k == 0:
                continue
            for prev in set(np.unique(labels[k - 1][lab == island])) - {0, main[k - 1]}:
                a, b = find((k, island)), find((k - 1, int(prev)))
                if a != b:
                    parent[a] = b
    chains: dict[tuple, dict] = {}
    for node in parent:
        k, island = node
        root = find(node)
        record = chains.setdefault(root, {"start": math.inf, "end": -math.inf,
                                          "min_area": math.inf})
        record["start"] = min(record["start"], heights[k] - layer / 2)
        record["end"] = max(record["end"], heights[k] + layer / 2)
        record["min_area"] = min(record["min_area"], float(areas[k][island - 1]))
    return list(chains.values())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--limit", type=float, default=3.0)
    parser.add_argument("--layer-height", type=float, default=0.2)
    parser.add_argument("--cell", type=float, default=0.2)
    parser.add_argument("--gap", type=float, default=6.0,
                        help="in-island gaps up to this width do not split it")
    parser.add_argument("--table", action="store_true",
                        help="print island count / smallest / main area per layer")
    args = parser.parse_args()

    heights, labels, areas = sections(args.mesh, args.layer_height, args.cell,
                                      args.gap)
    if args.table:
        for z, area in zip(heights, areas):
            print(f"  Z{z:6.2f}: {len(area)} islands {sorted(round(a) for a in area)}")
    found = towers(heights, labels, areas, args.layer_height)
    failures = []
    for record in sorted(found, key=lambda r: r["start"]):
        height = record["end"] - record["start"]
        ratio = height / math.sqrt(max(record["min_area"], 1e-9))
        record["height"], record["ratio"] = height, ratio
        if ratio > args.limit:
            failures.append(record)
    worst = max((r["ratio"] for r in found), default=0.0)
    if failures:
        print(f"TOWER_STABILITY FAIL {args.mesh}: {len(failures)} tower(s) over "
              f"height/sqrt(area) {args.limit:g}")
        for r in failures:
            print(f"  Z{r['start']:.1f}->{r['end']:.1f}: height {r['height']:.1f} mm, "
                  f"min area {r['min_area']:.0f} mm2, ratio {r['ratio']:.2f}")
        return 1
    print(f"TOWER_STABILITY PASS {args.mesh}: {len(found)} separate tower(s); "
          f"worst ratio {worst:.2f} (limit {args.limit:g})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
