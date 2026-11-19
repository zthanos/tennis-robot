#!/usr/bin/env python3
"""P2-99e: OAK-D S2 field-of-view study on the CAD scene (ray casting).

For a lens position (CAD mm, ground frame), a downward pitch and a camera FOV,
casts one ray per pixel and reports which body group is hit first (or ground /
sky), per tilt angle of the moving module. Also reports the visible ground
range and whether the top of a ball (z=66) in front of the ski is visible.

Usage: p299e_oak_fov_raycast.py <stl_dir> --lens X Y Z --pitch DEG
Meshes: <stl_dir>/<group>_<angle>.stl (moving groups) or <group>_0.stl (fixed).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

FIXED = ["ski", "arch", "F_tub", "F_front", "chassis"]
MOVING = ["module", "basket"]
CAMERAS = {"rgb_ff": (69.0, 55.0), "rgb_af": (66.0, 54.0), "stereo": (80.0, 55.0)}


def rays(lens, pitch_deg, hfov, vfov, nx, ny):
    p = math.radians(pitch_deg)
    fwd = np.array([math.cos(p), 0.0, -math.sin(p)])
    left = np.array([0.0, 1.0, 0.0])
    up = np.cross(fwd, left)
    up = -up if up[2] < 0 else up
    tx = np.tan(np.radians(hfov / 2)) * np.linspace(1, -1, nx)   # +Y on the left
    ty = np.tan(np.radians(vfov / 2)) * np.linspace(1, -1, ny)
    gx, gy = np.meshgrid(tx, ty)
    d = fwd[None, None, :] + gx[..., None] * left + gy[..., None] * up
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    return np.repeat(np.array(lens, float)[None, :], nx * ny, 0), d.reshape(-1, 3)


def first_hits(meshes, o, d):
    best = np.full(len(d), np.inf)
    who = np.full(len(d), "sky", dtype=object)
    for name, m in meshes.items():
        loc, idx_ray, _ = m.ray.intersects_location(o, d, multiple_hits=False)
        if len(idx_ray) == 0:
            continue
        dist = np.linalg.norm(loc - o[idx_ray], axis=1)
        better = dist < best[idx_ray]
        best[idx_ray[better]] = dist[better]
        who[idx_ray[better]] = name
    # ground plane z = 0
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = -o[:, 2] / d[:, 2]
    g = (d[:, 2] < 0) & (tg < best)
    best[g] = tg[g]
    who[g] = "ground"
    return who, best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("stl_dir", type=Path)
    ap.add_argument("--lens", type=float, nargs=3, required=True)
    ap.add_argument("--pitch", type=float, required=True)
    ap.add_argument("--nx", type=int, default=160)
    ap.add_argument("--ny", type=int, default=128)
    a = ap.parse_args()
    out = {"lens_mm": a.lens, "pitch_deg": a.pitch, "per_tilt": {}}
    fixed = {g: trimesh.load(a.stl_dir / f"{g}_0.stl") for g in FIXED}
    for tilt in (-3, 0, 20):
        meshes = dict(fixed)
        meshes.update({g: trimesh.load(a.stl_dir / f"{g}_{tilt}.stl") for g in MOVING})
        res = {}
        for cam, (hf, vf) in CAMERAS.items():
            o, d = rays(a.lens, a.pitch, hf, vf, a.nx, a.ny)
            who, dist = first_hits(meshes, o, d)
            frac = {k: round(100.0 * float(np.mean(who == k)), 2) for k in sorted(set(who))}
            gpts = o[who == "ground"] + d[who == "ground"] * dist[who == "ground"][:, None]
            ground = None
            if len(gpts):
                ground = {"x_min_mm": round(float(gpts[:, 0].min()), 1),
                          "x_max_mm": round(float(gpts[:, 0].max()), 1),
                          "near_ground_ahead_of_lens_mm": round(float(gpts[:, 0].min() - a.lens[0]), 1)}
            res[cam] = {"image_pct": frac, "ground": ground,
                        "horizon_visible": bool(np.any(d[:, 2] >= 0))}
        # ball-top visibility along the centreline in front of the ski (RGB FF)
        vis = []
        hf, vf = CAMERAS["rgb_ff"]
        p = math.radians(a.pitch)
        for x in range(440, 1001, 20):
            tgt = np.array([x, 0.0, 66.0])
            v = tgt - np.array(a.lens)
            dist = np.linalg.norm(v)
            v /= dist
            elev = math.degrees(math.asin(-v[2])) - a.pitch     # below optical axis (+)
            if abs(elev) > vf / 2:
                continue
            who, dd = first_hits(meshes, np.array([a.lens]), v[None, :])
            if who[0] in ("ground", "sky") or dd[0] >= dist - 1.0:
                vis.append(x)
        res["ball_top_visible_x_mm"] = [min(vis), max(vis)] if vis else None
        out["per_tilt"][str(tilt)] = res
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
