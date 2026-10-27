#!/usr/bin/env python3
"""Concept F (body-panels-f.scad, planter tub) QA on the real v3 robot.  No STL release;
meshes go to a temporary _build/qa3/ only.

Reuses the mesh/sampling/exact-distance helpers of qa-clearance.py.
Gates (exact-solid zero intersection, OpenSCAD Manifold):
  moving set -10..+20 deg (P2-40 module, P2-44 cheeks, P2-54 actuator/arm/FIT,
  cam, D5065 cables, P2-68 door + selector nose) -- plain and panels +10 mm;
  drive tyres -- plain and +10 mm; fixed world; P2-70 pod contents; launch
  cone; P2-67 camera FOV; Z>=375; ball path (R75/75 ski + flight + basket);
  feed path; plan outside the R421 keep circle about X117.
Numeric minimum distances per piece, plan radius, bbox, mass.

Usage: python3 qa-clearance-f.py
"""
import importlib.util, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("q1", os.path.join(HERE, "qa-mesh-utils.py"))
q1 = importlib.util.module_from_spec(spec); spec.loader.exec_module(q1)
load, sample, min_dist, volume_cm3 = q1.load, q1.sample, q1.min_dist, q1.volume_cm3

SCAD = os.path.join(HERE, "body-panels-f.scad")
SCAD_SUPPORT = os.path.join(HERE, "body-panels-f-support.scad")
BUILD = os.path.join(HERE, "_build", "qaf")
OPENSCAD = os.environ.get("OPENSCAD_BIN", "/snap/bin/openscad-nightly")
ANGLES = list(range(-10, 21))
ICR = np.array([117.0, 0.0]); R_TURN = 425.0; R_KEEP = 424.5   # tub rear arc wraps the P2-70 pod corners (R421.4)
PIECES = [
    "rear_lo", "rear_hi", "c1_lo", "c1_hi", "c2", "c3",
    "l1", "l2", "l3", "deck_rear", "drawer_face",
    "deck_s1", "deck_s2", "deck_s3",
]
RHO = 1.27
os.makedirs(BUILD, exist_ok=True)


def scad(out, defines, src=SCAD):
    import subprocess
    cmd = [OPENSCAD, "--backend=Manifold", "-o", out]
    for k, v in defines.items():
        cmd += ["-D", f"{k}={json.dumps(v) if isinstance(v, str) else v}"]
    r = subprocess.run(cmd + [src], capture_output=True, text=True)
    return "Current top level object is empty" in (r.stdout + r.stderr)


def targets():
    tg = {}
    for a in ANGLES:
        for name, view in [("moving", "mesh_moving"), ("d5065_airflow", "mesh_airflow")]:
            f = os.path.join(BUILD, f"{name}_{a}.stl")
            if not os.path.exists(f):
                scad(f, {"bp_support_view": view, "bp_angle": a}, src=SCAD_SUPPORT)
            tg.setdefault(name, []).append((a, load(f)))
    for name, view in [("tyres", "mesh_tyres"), ("stops", "mesh_stops"),
                       ("fixed", "mesh_fixed"), ("console_temp", "mesh_console")]:
        f = os.path.join(BUILD, f"{name}.stl")
        if not os.path.exists(f):
            scad(f, {"bp_support_view": view}, src=SCAD_SUPPORT)
        tg[name] = [(None, load(f))]
    for name, mod in [("launch_cone", "ko_launch_cone()"), ("camera_fov", "ko_camera_fov()"),
                      ("ball_path", "ko_ball_path()"), ("feed_path_temp", "ko_feed_path()"),
                      ("pod_contents", "ko_pod_contents()")]:
        f = os.path.join(BUILD, f"ko_{name}.stl")
        if not os.path.exists(f):
            w = os.path.join(BUILD, f"ko_{name}.scad")
            open(w, "w").write(f"use <{SCAD_SUPPORT}>\n{mod};\n")
            scad(f, {}, src=w)
        tg[name] = [(None, load(f))]
    return tg


def gates(style):
    out = {}
    for v in ["qa_moving_sweep", "qa_tyres", "qa_fixed", "qa_pod_contents", "qa_cone", "qa_fov",
              "qa_lidar", "qa_ball", "qa_feed", "qa_turn", "qa_console"]:
        out[v] = "PASS_EMPTY" if scad(os.path.join(BUILD, f"gate_{style}_{v}.stl"),
                                      {"bp_view": v}) else "INTERSECTS"
    for v in ["qa_moving_sweep", "qa_tyres"]:
        out[v + "_inflated10"] = "PASS_EMPTY" if scad(
            os.path.join(BUILD, f"gate_{style}_{v}_p10.stl"),
            {"bp_view": v, "qa_probe": 10}) else "INTERSECTS"
    return out


def audit(style, tg):
    rows = []
    for pc in PIECES:
        f = os.path.join(BUILD, f"piece_{style}_{pc}.stl")
        scad(f, {"bp_view": "mesh_piece", "bp_piece": pc, "bp_side": 1})
        t = load(f); v = t.reshape(-1, 3)
        size = (v.max(0) - v.min(0)).round(1).tolist()
        dims = sorted(size, reverse=True)
        vol = volume_cm3(t)
        tr = t.copy(); tr[:, :, 1] *= -1
        both = np.concatenate([t, tr])
        rplan = float(np.linalg.norm(v[:, :2] - ICR, axis=1).max())
        row = {"piece": pc, "qty": 2, "bbox_mm": size,
               "fits_250x250": bool(dims[1] <= 250 and dims[0] <= 256),
               "volume_cm3_each": round(vol, 1), "mass_g_each_PETG": round(vol * RHO),
               "z_max": round(float(v[:, 2].max()), 1),
               "abs_y_max": round(float(np.abs(v[:, 1]).max()), 1),
               "plan_radius_max_mm": round(rplan, 1), "dist": {}}
        for name, lst in tg.items():
            best = (float("inf"), None, None)
            for a, tris in lst:
                d, p = min_dist(both, tris)
                if d < best[0]:
                    best = (d, a, p)
            row["dist"][name] = {"min_mm": None if best[0] == float("inf") else round(best[0], 1),
                                 "angle_deg": best[1], "at": best[2]}
        rows.append(row)
    return rows


def main():
    styles = ["planter"]
    np.random.seed(1)
    tg = targets()
    res = {"scad": "cad/body-panels/body-panels-f.scad",
           "context": "full-robot-v3.scad defaults (P2-65..P2-71), 2026-10-17",
           "method": "exact-solid zero-intersection (Manifold) + 2.5 mm sample pass and exact "
                     "point-to-triangle refinement, +/-1 mm; plan radius about X117",
           "turn_envelope_mm": R_TURN, "panel_keep_radius_mm": R_KEEP, "styles": {}}
    for s in styles:
        rows = audit(s, tg)
        res["styles"][s] = {"gates": gates(s), "pieces": rows,
                            "total_pieces": sum(r["qty"] for r in rows),
                            "total_mass_g_PETG": round(sum(r["mass_g_each_PETG"] * r["qty"] for r in rows))}
        print(s, json.dumps(res["styles"][s]["gates"]), "mass", res["styles"][s]["total_mass_g_PETG"])
        for r in rows:
            d = r["dist"]
            g = lambda k: d[k]["min_mm"]
            print(f"  {r['piece']:13s} bbox {r['bbox_mm']} fit {r['fits_250x250']} {r['mass_g_each_PETG']:4d} g "
                  f"Rplan {r['plan_radius_max_mm']} zmax {r['z_max']} | mov {g('moving')}@{d['moving']['angle_deg']} "
                  f"air {g('d5065_airflow')} tyre {g('tyres')} stops {g('stops')} fixed {g('fixed')} "
                  f"pod {g('pod_contents')} cone {g('launch_cone')} fov {g('camera_fov')} "
                  f"ball {g('ball_path')} feed {g('feed_path_temp')} console {g('console_temp')}")
    json.dump(res, open(os.path.join(HERE, "qa-results-f.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
