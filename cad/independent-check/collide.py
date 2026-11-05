#!/usr/bin/env python3
"""Independent collision check: raw pairwise intersections, no exclusions.

  uv run --with manifold3d --with trimesh --with numpy python collide.py [--only a,b]

Exports every body of parts.py in its installed pose (cached by source mtime),
then reports every pair whose intersection volume exceeds --min-vol mm^3.
Touching faces have zero volume and pass; nothing is subtracted first.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import itertools
import json
import os
import subprocess
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh

from parts import EXCLUSIVE_GROUPS, PARTS, SOURCES

HERE = Path(__file__).resolve().parent
CAD = HERE.parent
OUT = HERE / "out"
OPENSCAD = os.environ.get("OPENSCAD_BIN", "/snap/bin/openscad-nightly")


def newest_source_mtime() -> float:
    return max(p.stat().st_mtime for p in CAD.rglob("*.scad")
               if "independent-check" not in p.parts)


def export(part) -> tuple[str, str | None]:
    name, _group, header, body = part
    scad = OUT / f"{name}.scad"
    stl = OUT / f"{name}.stl"
    text = SOURCES[header].format(cad=CAD) + f"\n{body}\n"
    if stl.exists() and scad.exists() and scad.read_text() == text \
            and stl.stat().st_mtime > newest_source_mtime():
        return name, None
    scad.write_text(text)
    r = subprocess.run([OPENSCAD, "--backend=manifold", "-o", str(stl), str(scad)],
                       capture_output=True, text=True, timeout=1800)
    if r.returncode != 0 or not stl.exists():
        return name, r.stderr[-1500:]
    return name, None


def load(name: str) -> m3d.Manifold:
    mesh = trimesh.load(OUT / f"{name}.stl", force="mesh")
    return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32),
                                 tri_verts=np.asarray(mesh.faces, np.uint32)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", help="comma list of part names")
    ap.add_argument("--min-vol", type=float, default=0.5)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    parts = [p for p in PARTS if not args.only or p[0] in args.only.split(",")]

    failed = {}
    with cf.ThreadPoolExecutor(args.jobs) as ex:
        for name, err in ex.map(export, parts):
            if err:
                failed[name] = err
                print(f"EXPORT FAIL {name}:\n{err}")
    solids = {}
    for name, group, *_ in parts:
        if name in failed:
            continue
        man = load(name)
        if man.status() != m3d.Error.NoError or man.is_empty():
            print(f"BAD MESH {name}: {man.status()} empty={man.is_empty()}")
            continue
        solids[name] = (group, man)
        print(f"  {name:24s} vol={man.volume():10.0f} mm3")

    hits = []
    for (a, (ga, ma)), (b, (gb, mb)) in itertools.combinations(solids.items(), 2):
        if any({a, b} <= g for g in EXCLUSIVE_GROUPS):
            continue
        ba, bb = ma.bounding_box(), mb.bounding_box()
        if any(ba[i + 3] < bb[i] or bb[i + 3] < ba[i] for i in range(3)):
            continue
        inter = ma ^ mb
        vol = inter.volume()
        if vol > args.min_vol:
            box = inter.bounding_box()
            hits.append({"a": a, "b": b, "vol": round(vol, 1),
                         "bbox": [round(v, 1) for v in box]})
            inter_mesh = inter.to_mesh()
            trimesh.Trimesh(inter_mesh.vert_properties[:, :3], inter_mesh.tri_verts) \
                .export(OUT / f"hit__{a}__{b}.stl")
    hits.sort(key=lambda h: -h["vol"])
    (OUT / "report.json").write_text(json.dumps({"failed": list(failed), "hits": hits}, indent=1))
    print(f"\n{len(hits)} collisions > {args.min_vol} mm3 ({len(failed)} export failures)")
    for h in hits:
        print(f"  {h['vol']:9.1f} mm3  {h['a']:24s} x {h['b']:24s}  bbox {h['bbox']}")
    return 1 if hits or failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
