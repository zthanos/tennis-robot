#!/usr/bin/env python3
"""P2-99e: camera blind zone in front of the ski / flywheel nose.

A ball (r 33 mm) rolls in along the centreline (y=0, centre z=33). For each
module tilt it finds the first ball-flywheel contact (largest centre X where
the ball touches the module mesh), then the last centre X (approaching) at
which (a) the ball top and (b) the whole ball (all sampled silhouette points
facing the lens) are inside the camera FOV and not occluded. Blind distance =
last visible X - contact X.
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
R = 33.0


def visible(meshes, lens, pitch, hf, vf, pts):
    p = math.radians(pitch)
    fwd = np.array([math.cos(p), 0.0, -math.sin(p)])
    up = np.array([math.sin(p), 0.0, math.cos(p)])
    left = np.array([0.0, 1.0, 0.0])
    o = np.asarray(lens, float)
    for q in pts:
        d = q - o
        dist = np.linalg.norm(d)
        d /= dist
        x, y, z = d @ fwd, d @ left, d @ up
        if x <= 0 or abs(math.degrees(math.atan2(y, x))) > hf / 2 or abs(math.degrees(math.atan2(z, x))) > vf / 2:
            return False
        for m in meshes:
            loc, _, _ = m.ray.intersects_location([o], [d], multiple_hits=False)
            if len(loc) and np.linalg.norm(loc[0] - o) < dist - 0.5:
                return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("stl_dir", type=Path)
    ap.add_argument("--lens", type=float, nargs=3, default=(508, 0, 300))
    ap.add_argument("--pitch", type=float, default=25.0)
    a = ap.parse_args()
    fixed = [trimesh.load(a.stl_dir / f"{g}_0.stl") for g in FIXED]
    out = {"lens_mm": a.lens, "pitch_deg": a.pitch, "per_tilt": {}}
    for tilt in (-3, 0, 20):
        mod = trimesh.load(a.stl_dir / f"module_{tilt}.stl")
        bas = trimesh.load(a.stl_dir / f"basket_{tilt}.stl")
        meshes = fixed + [mod, bas]
        contact = None
        for x in np.arange(700.0, 380.0, -1.0):
            c = np.array([[x, 0.0, R]])
            d = trimesh.proximity.closest_point(mod, c)[1][0]
            if d <= R:
                contact = float(x)
                break
        res = {"first_flywheel_contact_centre_x_mm": contact}
        for cam, (hf, vf) in {"rgb_ff": (69, 55), "stereo": (80, 55)}.items():
            top_last = whole_last = None
            for x in np.arange(contact, 1001.0, 2.0):
                top = np.array([[x, 0.0, 2 * R]])
                if top_last is None and visible(meshes, a.lens, a.pitch, hf, vf, top):
                    top_last = float(x)
                if whole_last is None:
                    c = np.array([x, 0.0, R])
                    v = np.asarray(a.lens) - c
                    v /= np.linalg.norm(v)
                    side = np.cross(v, [0, 1, 0]); side /= np.linalg.norm(side)
                    pts = [c + R * 0.98 * k for k in (v, -side, side, np.array([0, 1, 0]), np.array([0, -1, 0]),
                                                     np.array([0, 0, 1]))]
                    pts.append(c + R * 0.98 * np.array([math.cos(math.radians(200)), 0, math.sin(math.radians(200))]))
                    if visible(meshes, a.lens, a.pitch, hf, vf, pts):
                        whole_last = float(x)
                if top_last is not None and whole_last is not None:
                    break
            res[cam] = {"ball_top_visible_from_x": top_last, "whole_ball_visible_from_x": whole_last,
                        "blind_top_mm": None if top_last is None else round(top_last - contact, 1),
                        "blind_whole_mm": None if whole_last is None else round(whole_last - contact, 1)}
        out["per_tilt"][str(tilt)] = res
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
