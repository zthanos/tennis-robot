#!/usr/bin/env python3
"""Body-panel concept QA (no STL release; meshes go to a temporary _build/).

1. Exact-solid zero-intersection gates in OpenSCAD (Manifold backend):
   panels vs swept moving module (-10..+20 deg, 1 deg), drive tyres, fixed
   world, launch cone, camera FOV, LiDAR ceiling, ball path, basket lift-out,
   temporary feed path.  Moving-sweep and tyre gates are repeated with the
   panels Minkowski-inflated by 10 mm (the brief's clearance rule).
2. Numeric minimum distance per panel piece by dense surface sampling
   (2.5 mm global pass, 0.4 mm local refinement, KD-tree), reported with +/-1 mm numerical uncertainty.
3. Per-piece bounding box (<=250x250 print footprint) and mass estimate.

Usage: python3 qa-clearance.py [A B C]   (system python3 with numpy+scipy)
"""
import json, os, struct, subprocess, sys
import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "body-panels-concepts.scad")
BUILD = os.path.join(HERE, "_build", "qa")
OPENSCAD = os.environ.get("OPENSCAD_BIN", "/snap/bin/openscad-nightly")
ANGLES = list(range(-10, 21))
SPACING = 0.4          # mm fine sample spacing (refinement pass)
RHO = {"PETG": 1.27, "ASA": 1.07}  # g/cm3
PIECES = {
    "A": ["rp_lid", "rp_console", "rp_door", "rp_side", "sp_shell", "sp_door"],
}
PIECES["B"] = PIECES["A"] + ["fender_rear", "fender_front", "shoulder", "brow", "front_guard"]
PIECES["C"] = ["rp_lid", "rp_console", "rp_door", "skin_rear", "skin_mid", "sp_shell_c",
               "fender_rear", "fender_front", "shoulder", "brow", "front_guard"]
SINGLE = {"rp_console"}     # centre piece, one off; everything else is L+R
os.makedirs(BUILD, exist_ok=True)


def scad(out, defines, src=SCAD):
    cmd = [OPENSCAD, "--backend=Manifold", "-o", out]   # no -q: gate reads the "empty" notice
    for k, v in defines.items():
        cmd += ["-D", f"{k}={json.dumps(v) if isinstance(v, str) else v}"]
    r = subprocess.run(cmd + [src], capture_output=True, text=True)
    empty = "Current top level object is empty" in (r.stdout + r.stderr)
    return empty, r


def load(p):
    b = open(p, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = [list(map(float, l.split()[1:4])) for l in b.decode().splitlines()
             if l.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50],
                      dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def volume_cm3(t):
    return abs(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum()) / 6 / 1000


def sample(t, spacing=SPACING, box=None):
    if box is not None:
        lo, hi = box
        tmin, tmax = t.min(1), t.max(1)
        keep = np.all(tmax >= lo, 1) & np.all(tmin <= hi, 1)
        t = t[keep]
    if len(t) == 0:
        return np.zeros((0, 3))
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.maximum(1, np.ceil(area / (spacing * spacing * 0.5)).astype(int))
    idx = np.repeat(np.arange(len(t)), n)
    r1, r2 = np.random.rand(len(idx)), np.random.rand(len(idx))
    s = np.sqrt(r1)
    p = (1 - s)[:, None] * a[idx] + (s * (1 - r2))[:, None] * b[idx] + (s * r2)[:, None] * c[idx]
    edges = np.concatenate([a, b, c, (a + b) / 2, (b + c) / 2, (c + a) / 2])
    return np.concatenate([p, edges])


def pt_tri_dist(p, tri):
    """Exact distance from points p (N,3) to triangles tri (M,3,3) -> (N,) min."""
    a, b, c = tri[:, 0][None], tri[:, 1][None], tri[:, 2][None]
    P = p[:, None, :]
    ab, ac, ap = b - a, c - a, P - a
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = P - b; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = P - c; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2
    den = va + vb + vc; den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    v = vb / den; w = vc / den
    q = a + ab * v[..., None] + ac * w[..., None]                  # interior
    def edge(x0, e, t):
        return x0 + e * np.clip(t, 0, 1)[..., None]
    with np.errstate(divide="ignore", invalid="ignore"):
        e_ab = edge(a, ab, d1 / np.where((d1 - d3) == 0, 1e-12, d1 - d3))
        e_ac = edge(a, ac, d2 / np.where((d2 - d6) == 0, 1e-12, d2 - d6))
        bc = c - b
        e_bc = edge(b, bc, ((P - b) * bc).sum(-1) / np.maximum((bc * bc).sum(-1), 1e-12))
    inside = (v >= 0) & (w >= 0) & (v + w <= 1)
    cand = [np.linalg.norm(P - e, axis=-1) for e in (e_ab, e_ac, e_bc)]
    di = np.where(inside, np.linalg.norm(P - q, axis=-1), np.inf)
    return np.minimum.reduce([di] + cand).min(1)


def min_dist(pt_tris, target_tris, margin=80.0):
    """Coarse global pass (2.5 mm samples) then exact point-to-triangle
    distance from 0.4 mm panel samples around the best coarse candidates."""
    pc = sample(pt_tris, 2.5)
    lo, hi = pc.min(0) - margin, pc.max(0) + margin
    qc = sample(target_tris, 2.5, box=(lo, hi))
    if len(qc) == 0:
        return float("inf"), None
    d, _ = cKDTree(qc).query(pc, k=1)
    order = np.argsort(d)[:400]
    best = (float("inf"), None)
    seen = []
    tmin, tmax = target_tris.min(1), target_tris.max(1)
    for j in order:
        c = pc[j]
        if any(np.linalg.norm(c - s0) < 15 for s0 in seen):
            continue
        seen.append(c)
        if len(seen) > 6:
            break
        if d[j] > 60:                     # far: coarse value is enough
            if d[j] < best[0]:
                best = (float(d[j]), c.round(1).tolist())
            continue
        r = 6.0
        box = (c - r, c + r)
        pf = sample(pt_tris, 0.4, box=box)
        pf = pf[np.all((pf >= box[0]) & (pf <= box[1]), 1)]
        reach = d[j] + r + 4
        keep = np.all(tmax >= c - reach, 1) & np.all(tmin <= c + reach, 1)
        tt = target_tris[keep]
        if len(pf) == 0 or len(tt) == 0:
            continue
        for k0 in range(0, len(pf), 2000):
            chunk = pf[k0:k0 + 2000]
            dd = np.concatenate([pt_tri_dist(chunk, tt[m:m + 300]) [:, None]
                                 for m in range(0, len(tt), 300)], 1).min(1)
            k = int(np.argmin(dd))
            if dd[k] < best[0]:
                best = (float(dd[k]), chunk[k].round(1).tolist())
    return best


def export_targets():
    tg = {}
    for a in ANGLES:
        f = os.path.join(BUILD, f"moving_{a}.stl")
        if not os.path.exists(f):
            scad(f, {"bp_view": "mesh_moving", "bp_angle": a})
        tg.setdefault("moving", []).append((a, load(f)))
        f = os.path.join(BUILD, f"airflow_{a}.stl")
        if not os.path.exists(f):
            scad(f, {"bp_view": "mesh_airflow", "bp_angle": a})
        tg.setdefault("d5065_airflow", []).append((a, load(f)))
    for name, view in [("tyres", "mesh_tyres"), ("stops", "mesh_stops"), ("fixed", "mesh_fixed")]:
        f = os.path.join(BUILD, f"{name}.stl")
        if not os.path.exists(f):
            scad(f, {"bp_view": view})
        tg[name] = [(None, load(f))]
    # keep-out solids written through a tiny wrapper
    for name, mod in [("launch_cone", "ko_launch_cone()"), ("camera_fov", "ko_camera_fov()"),
                      ("ball_path", "ko_ball_path()"), ("feed_path_temp", "ko_feed_path()"),
                      ("basket_lift", "ko_basket_lift()")]:
        f = os.path.join(BUILD, f"ko_{name}.stl")
        if not os.path.exists(f):
            w = os.path.join(BUILD, f"ko_{name}.scad")
            open(w, "w").write(f"use <{SCAD}>\n{mod};\n")
            scad(f, {}, src=w)
        tg[name] = [(None, load(f))]
    return tg


def gates(concept):
    out = {}
    for v in ["qa_moving_sweep", "qa_tyres", "qa_fixed", "qa_cone", "qa_fov", "qa_lidar",
              "qa_ball", "qa_basket", "qa_feed"]:
        e, _ = scad(os.path.join(BUILD, f"gate_{concept}_{v}.stl"), {"concept": concept, "bp_view": v})
        out[v] = "PASS_EMPTY" if e else "INTERSECTS"
    for v in ["qa_moving_sweep", "qa_tyres"]:
        e, _ = scad(os.path.join(BUILD, f"gate_{concept}_{v}_p10.stl"),
                    {"concept": concept, "bp_view": v, "qa_probe": 10})
        out[v + "_inflated10"] = "PASS_EMPTY" if e else "INTERSECTS"
    return out


def audit(concept, tg):
    rows = []
    for pc in PIECES[concept]:
        f = os.path.join(BUILD, f"piece_{concept}_{pc}.stl")
        scad(f, {"concept": concept, "bp_view": "mesh_piece", "bp_piece": pc, "bp_side": 1})
        t = load(f)
        v = t.reshape(-1, 3)
        size = (v.max(0) - v.min(0)).round(1).tolist()
        dims = sorted(size, reverse=True)
        fits = dims[1] <= 250 and dims[0] <= 256  # any orientation inside 256^3, <=250x250 footprint
        vol = volume_cm3(t)
        qty = 1 if pc in SINGLE else 2
        # mirror image for the right-hand piece
        tr = t.copy(); tr[:, :, 1] *= -1
        pts = np.concatenate([t, tr]) if pc not in SINGLE else t   # triangles, both sides
        row = {"piece": pc, "qty": qty, "bbox_mm": size, "fits_250x250": bool(fits),
               "volume_cm3_each": round(vol, 1),
               "mass_g_each_PETG": round(vol * RHO["PETG"], 0),
               "mass_g_each_ASA": round(vol * RHO["ASA"], 0),
               "z_max": round(float(v[:, 2].max()), 1),
               "abs_y_max": round(float(np.abs(v[:, 1]).max()), 1), "dist": {}}
        for name, lst in tg.items():
            best = (float("inf"), None, None)
            for a, tris in lst:
                d, p = min_dist(pts, tris)
                if d < best[0]:
                    best = (d, a, p)
            row["dist"][name] = {"min_mm": None if best[0] == float("inf") else round(best[0], 1),
                                 "angle_deg": best[1], "at": best[2]}
        rows.append(row)
    return rows


def main():
    concepts = sys.argv[1:] or ["A", "B", "C"]
    np.random.seed(1)
    tg = export_targets()
    res = {"method": "exact-solid zero-intersection (OpenSCAD Manifold) + surface-sampled "
                     "KD-tree distance, 2.5 mm global sample pass + exact point-to-triangle refinement from 0.4 mm panel samples, numerical uncertainty +/-1 mm",
           "angles_deg": [ANGLES[0], ANGLES[-1], 1], "concepts": {}}
    for c in concepts:
        rows = audit(c, tg)
        mass = sum(r["mass_g_each_PETG"] * r["qty"] for r in rows)
        res["concepts"][c] = {"gates": gates(c), "pieces": rows,
                              "total_pieces": sum(r["qty"] for r in rows),
                              "total_mass_g_PETG_solid_model": round(mass),
                              "total_mass_g_ASA_solid_model": round(sum(r["mass_g_each_ASA"] * r["qty"] for r in rows))}
        print(c, json.dumps(res["concepts"][c]["gates"]), "mass PETG", round(mass), "g")
        for r in rows:
            d = r["dist"]
            print(f"  {r['piece']:13s} x{r['qty']} bbox {r['bbox_mm']} fit {r['fits_250x250']} "
                  f"{r['mass_g_each_PETG']:5.0f} g | mov {d['moving']['min_mm']}@{d['moving']['angle_deg']} "
                  f"air {d['d5065_airflow']['min_mm']} tyre {d['tyres']['min_mm']} stops {d['stops']['min_mm']} "
                  f"cone {d['launch_cone']['min_mm']} fov {d['camera_fov']['min_mm']} ball {d['ball_path']['min_mm']} "
                  f"bask {d['basket_lift']['min_mm']} feed {d['feed_path_temp']['min_mm']} zmax {r['z_max']}")
    json.dump(res, open(os.path.join(HERE, "qa-results.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
