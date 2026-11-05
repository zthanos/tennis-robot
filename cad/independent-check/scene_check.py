#!/usr/bin/env python3
"""Full-CAD collision gate: every body of the current robot scene, raw.

  uv run --with manifold3d --with trimesh --with numpy --with scipy python scene_check.py

The scene is `current_robot(angle,false)` of body-panels/current-design-p293.scad
at -10 (yoke limit), -3, 0, +15 and +20 degrees.  Bodies come from the CAD's own placement code
(scadtree.py), so a part added to the scene is checked without editing a list.
Also reported:
  - printed-part modules (they have an STL) that never appear in the scene,
  - bodies rendered twice by nested context scenes (kept once),
  - the basket ball space (convex hull of the basket) against every other body.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import itertools
import json
import os
import subprocess
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh

import scadtree

HERE = Path(__file__).resolve().parent
CAD = HERE.parent
OUT = HERE / "out" / "scene"
SRC = HERE / "out" / "src"
OPENSCAD = os.environ.get("OPENSCAD_BIN", "/snap/bin/openscad-nightly")
SCENE = "body-panels/current-design-p293.scad"
ANGLES = (-10, -3, 0, 15, 20)   # full yoke range plus the operating angles


SRC_DIGEST = ""


def wrapper(angle, sel, disc):
    sel_txt = "undef" if sel is None else json.dumps(list(sel))
    return (f'// sources {SRC_DIGEST}\nuse <{SRC / SCENE}>\n$p=[]; $disc={str(disc).lower()}; $sel={sel_txt};\n'
            f'current_robot({angle},false);\n')


def run(scad: Path, out: Path):
    r = subprocess.run([OPENSCAD, "--backend=manifold", "-o", str(out), str(scad)],
                       capture_output=True, text=True, timeout=1800)
    return r.returncode, r.stderr


def export(job):
    key, angle, path = job
    stl = OUT / f"{key}.stl"
    scad = OUT / f"{key}.scad"
    text = wrapper(angle, path, False)
    if stl.exists() and scad.exists() and scad.read_text() == text:
        return key, "cached"
    scad.write_text(text)
    rc, err = run(scad, stl)
    if not stl.exists():
        return key, "empty" if "empty" in err.lower() or rc == 0 else err[-800:]
    return key, "ok"


def manifold(mesh: trimesh.Trimesh) -> m3d.Manifold:
    return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32),
                                 tri_verts=np.asarray(mesh.faces, np.uint32)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-vol", type=float, default=0.0001)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    parts, _fids, helpers = scadtree.mirror(CAD, SRC)
    # Exports are cached only while every instrumented source is unchanged.
    global SRC_DIGEST
    digest = hashlib.sha1()
    for f in sorted(SRC.rglob("*.scad")):
        digest.update(f.read_bytes())
    SRC_DIGEST = digest.hexdigest()
    jobs, names, seen_modules = [], {}, set()
    for angle in ANGLES:
        disc = OUT / f"disc_{angle}.scad"
        disc.write_text(wrapper(angle, None, True))
        echo = OUT / f"disc_{angle}.echo"
        run(disc, echo)
        stmts, mods = scadtree.parse_echo(echo.read_text())
        seen_modules |= {x for v in mods.values() for x in v}
        count = {}
        for name, path in scadtree.bodies(stmts, mods, parts, helpers):
            if name.startswith("geometry"):
                fid, off = path[-1].split(":") if ":" in path[-1] else path[-2].split(":")
                txt = (CAD / _fids[int(fid)]).read_text()[int(off):int(off) + 160]
                imp = __import__("re").search(r'import\("([^"]+)"', txt)
                name = (Path(imp.group(1)).stem if imp else txt.split("(")[0].strip() or "geometry") + name[8:]
            count[name] = count.get(name, 0) + 1
            uniq = name if count[name] == 1 else f"{name}#{count[name]}"
            key = hashlib.sha1(f"{angle}|{path}".encode()).hexdigest()[:12]
            names[key] = (uniq, angle)
            jobs.append((key, angle, path))

    with cf.ThreadPoolExecutor(args.jobs) as ex:
        status = dict(ex.map(export, jobs))
    failed = {names[k][0]: v for k, v in status.items() if v not in ("ok", "cached", "empty")}

    # Load, drop empties, merge bodies that are identical across angles, and
    # drop exact duplicates rendered twice by nested context scenes.
    by_angle = {a: {} for a in ANGLES}
    geo_seen, duplicates, files = {}, [], {}
    for key, (name, angle) in names.items():
        stl = OUT / f"{key}.stl"
        if status[key] == "empty" or not stl.exists():
            continue
        mesh = trimesh.load(stl, force="mesh")
        if len(mesh.faces) == 0:
            continue
        sig = (round(abs(mesh.volume), 1), tuple(np.round(mesh.bounds.ravel(), 1)))
        if (sig, angle) in geo_seen:
            duplicates.append((name, geo_seen[(sig, angle)]))
            continue
        geo_seen[(sig, angle)] = name
        files[f"{name}@{angle}"] = f"{key}.stl"
        by_angle[angle][name] = (sig, manifold(mesh), mesh)

    basket = [n for n in by_angle[0] if n.startswith(("p271_basket", "p271_front"))]
    ball_from = [n for n in basket if n.startswith("p271_basket")]
    hits = {}
    for angle, solids in by_angle.items():
        if any(n in solids for n in ball_from):
            hull = trimesh.util.concatenate([solids[n][2] for n in ball_from if n in solids]).convex_hull
            # Balls live above the P2-51 floor plane, (X-85,Z70)..(X278,Z42),
            # and behind the frozen P2-71 front grid at X225.
            n = np.array([28.0, 0.0, 363.0]) / np.hypot(28.0, 363.0)
            space = manifold(hull).trim_by_plane((-1.0, 0.0, 0.0), -225.0) \
                .trim_by_plane(tuple(n), float(n @ np.array([-85.0, 0.0, 70.0])))
            solids = dict(solids)
            solids["basket_ball_space"] = (None, space, hull)
        items = list(solids.items())
        boxes = {n: s[1].bounding_box() for n, s in items}
        for (a, sa), (b, sb) in itertools.combinations(items, 2):
            if "basket_ball_space" in (a, b) and ({a, b} & set(basket)):
                continue
            ba, bb = boxes[a], boxes[b]
            if any(ba[i + 3] <= bb[i] or bb[i + 3] <= ba[i] for i in range(3)):
                continue
            inter = sa[1] ^ sb[1]
            vol = inter.volume()
            if vol <= args.min_vol:
                continue
            pair = tuple(sorted((a, b)))
            smaller = min(sa[1].volume(), sb[1].volume())
            kind = "contained" if vol >= 0.99 * smaller else "collision"
            label = angle
            h = hits.setdefault(pair, {"a": pair[0], "b": pair[1], "angles": [], "vol": 0, "kind": kind})
            if label not in h["angles"]:
                h["angles"].append(label)
            h["vol"] = max(h["vol"], round(vol, 4))
            h["bbox"] = [round(v, 1) for v in inter.bounding_box()]
            im = inter.to_mesh()
            trimesh.Trimesh(im.vert_properties[:, :3], im.tri_verts).export(
                OUT / f"hit__{pair[0]}__{pair[1]}.stl".replace("/", "_"))

    stl_parts_missing = sorted(m for m in parts if m not in seen_modules)
    contained = sorted((h for h in hits.values() if h["kind"] == "contained"), key=lambda h: -h["vol"])
    hit_list = sorted((h for h in hits.values() if h["kind"] == "collision"), key=lambda h: -h["vol"])
    report = {"files": files, "bodies": {a: sorted(s) for a, s in by_angle.items()},
              "duplicates_dropped": duplicates, "export_failures": failed,
              "stl_modules_not_in_scene": stl_parts_missing, "hits": hit_list, "contained": contained}
    (OUT / "report.json").write_text(json.dumps(report, indent=1, default=str))

    print(f"bodies per angle: " + ", ".join(f"{a}°={len(s)}" for a, s in by_angle.items()))
    print(f"duplicates dropped: {len(duplicates)}   export failures: {len(failed)}")
    for n, e in failed.items():
        print(f"  EXPORT FAIL {n}: {e[-300:]}")
    print(f"STL part modules absent from the scene ({len(stl_parts_missing)}): "
          + ", ".join(stl_parts_missing))
    print(f"\n{len(contained)} pairs where one body lies wholly inside another "
          "(drawn twice by a context scene, or a part inside a surrogate envelope):")
    for h in contained:
        print(f"  {h['vol']:11.1f} mm3  {h['a']} inside/with {h['b']}")
    print(f"\n{len(hit_list)} collisions > {args.min_vol} mm3")
    for h in hit_list:
        ang = ",".join(str(x) for x in h["angles"])
        print(f"  {h['vol']:11.1f} mm3  {h['a']:34s} x {h['b']:34s} [{ang}]  bbox {h['bbox']}")
    return 1 if hit_list or failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
