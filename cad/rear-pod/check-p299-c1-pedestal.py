#!/usr/bin/env python3
"""Exact-solid P2-99f C1 screw-head/tool corridor gate."""

from __future__ import annotations

import argparse
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh


def manifold(path: Path) -> m3d.Manifold:
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError(f"empty or invalid mesh: {path}")
    return m3d.Manifold(
        m3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, np.float32),
            tri_verts=np.asarray(mesh.faces, np.uint32),
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pedestal", type=Path)
    parser.add_argument("keepouts", type=Path)
    parser.add_argument("--max-volume-mm3", type=float, default=0.0001)
    args = parser.parse_args()

    pedestal = manifold(args.pedestal)
    keepouts = manifold(args.keepouts)
    intersection = pedestal ^ keepouts
    volume = intersection.volume()
    status = "PASS" if volume <= args.max_volume_mm3 else "FAIL"
    print(
        f"P299_C1_TOOL_CORRIDORS {status}: intersection {volume:.6f} mm3; "
        f"limit {args.max_volume_mm3:.6f} mm3; 4 x diameter 4.5 mm"
    )
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
