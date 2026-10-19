#!/usr/bin/env python3
"""P2-77 torsion-deck repair and rear-pod panel-post stiffness screen.

CalculiX B32R chassis comparison plus a small linear 2-D frame model of the
P2-76 panel posts.  Units for both models are mm/N/MPa.  Contact, bolt slip,
printed-layer strength and panel local bending remain physical/design gates.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess

import numpy as np

from run_global_frame_fea import ROOT, SOLVER, runtime_env
from run_phase2_p65_crossbar_connection import CASES, build_model
from run_shared_flywheel_phase1 import mass_model, pivot_loads, traction_screen
from run_shared_flywheel_phase2 import PETG_E_MPA, parse_dat, write_deck

OUT = ROOT / "results/shared_flywheel_phase2/p277_torsion_repair.json"
WORK = ROOT / "fea/work/p277_torsion_repair"
VARIANTS = ("crossed", "single_diagonal", "underslung_offset", "underdeck_node")


def run_chassis() -> tuple[list[dict], dict]:
    pivot = pivot_loads(mass_model(), traction_screen())
    rows: list[dict] = []
    WORK.mkdir(parents=True, exist_ok=True)
    for variant in VARIANTS:
        for mesh_mm in (10.0, 5.0):
            model = build_model(mesh_mm, variant)
            for case_name in CASES:
                stem = f"p277_{variant}_{case_name}_{mesh_mm:g}mm"
                deck = WORK / f"{stem}.inp"
                write_deck(deck, model, case_name, pivot, PETG_E_MPA)
                proc = subprocess.run(
                    [str(SOLVER), "-i", stem], cwd=WORK, env=runtime_env(),
                    capture_output=True, text=True, timeout=60,
                )
                (WORK / f"{stem}.log").write_text(proc.stdout + proc.stderr)
                if proc.returncode or "*ERROR" in (proc.stdout + proc.stderr).upper():
                    raise RuntimeError(f"CalculiX failed: {stem}")
                rows.append({
                    "variant": variant,
                    "load_case": case_name,
                    "mesh_mm": mesh_mm,
                    "element_count": len(model[1]),
                    **parse_dat(WORK / f"{stem}.dat", model[0], model[3], model[4]),
                })

    comparison: dict[str, dict] = {}
    for case_name in CASES:
        case: dict[str, dict] = {}
        base = next(r for r in rows if r["variant"] == "crossed"
                    and r["load_case"] == case_name and r["mesh_mm"] == 5)
        for variant in VARIANTS:
            fine = next(r for r in rows if r["variant"] == variant
                        and r["load_case"] == case_name and r["mesh_mm"] == 5)
            coarse = next(r for r in rows if r["variant"] == variant
                          and r["load_case"] == case_name and r["mesh_mm"] == 10)
            value = fine["max_bearing_displacement_mm"]
            case[variant] = {
                "max_bearing_displacement_mm": value,
                "change_vs_p265_crossed_percent": 100 * (value / base["max_bearing_displacement_mm"] - 1),
                "max_bearing_relative_to_local_front_mount_mm": fine[
                    "max_bearing_relative_to_local_front_mount_mm"
                ],
                "mesh_change_10_to_5_percent": 100 * abs(
                    value - coarse["max_bearing_displacement_mm"]
                ) / value,
                "beam_integration_point_peak_von_mises_mpa": fine[
                    "beam_integration_point_peak_von_mises_mpa"
                ],
            }
        comparison[case_name] = case
    return rows, comparison


def frame_element(p0, p1, e, area, inertia):
    dy, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dy, dz)
    c, s = dy / length, dz / length
    ea_l = e * area / length
    ei = e * inertia
    local = np.array([
        [ea_l, 0, 0, -ea_l, 0, 0],
        [0, 12*ei/length**3, 6*ei/length**2, 0, -12*ei/length**3, 6*ei/length**2],
        [0, 6*ei/length**2, 4*ei/length, 0, -6*ei/length**2, 2*ei/length],
        [-ea_l, 0, 0, ea_l, 0, 0],
        [0, -12*ei/length**3, -6*ei/length**2, 0, 12*ei/length**3, -6*ei/length**2],
        [0, 6*ei/length**2, 2*ei/length, 0, -6*ei/length**2, 4*ei/length],
    ])
    transform = np.array([
        [c, s, 0, 0, 0, 0], [-s, c, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0],
        [0, 0, 0, c, s, 0], [0, 0, 0, -s, c, 0], [0, 0, 0, 0, 0, 1],
    ])
    return transform.T @ local @ transform


def panel_post_frame(paths: list[list[tuple[float, float]]]) -> dict:
    # Independent post nodes are retained even when two posts share Y/Z.
    nodes: list[tuple[float, float]] = []
    elements: list[tuple[int, int, float, float, float]] = []
    fixed: list[int] = []
    tops: list[int] = []
    e = 1500.0
    area = math.pi * 12**2 / 4
    inertia = math.pi * 12**4 / 64
    for path in paths:
        ids = []
        for point in path:
            ids.append(len(nodes)); nodes.append(point)
        fixed.append(ids[0]); tops.append(ids[-1])
        for a, b in zip(ids, ids[1:]):
            elements.append((a, b, e, area, inertia))

    # Rigid-diaphragm surrogate for the panel, with the reference at E-stop.
    ref = len(nodes); nodes.append((-58.0, 347.0))
    for top in tops:
        elements.append((top, ref, e*1000, area*100, inertia*1000))

    ndof = 3 * len(nodes)
    stiffness = np.zeros((ndof, ndof))
    for a, b, ee, aa, ii in elements:
        ke = frame_element(nodes[a], nodes[b], ee, aa, ii)
        dofs = [3*a, 3*a+1, 3*a+2, 3*b, 3*b+1, 3*b+2]
        stiffness[np.ix_(dofs, dofs)] += ke
    load = np.zeros(ndof); load[3*ref+1] = -30.0
    restrained = {3*n+d for n in fixed for d in range(3)}
    free = [i for i in range(ndof) if i not in restrained]
    displacement = np.zeros(ndof)
    displacement[free] = np.linalg.solve(stiffness[np.ix_(free, free)], load[free])
    return {
        "vertical_displacement_mm": float(displacement[3*ref+1]),
        "lateral_y_displacement_mm": float(displacement[3*ref]),
        "rotation_rad": float(displacement[3*ref+2]),
        "model": "2-D Euler-Bernoulli frame, four d12 PETG posts, rigid panel diaphragm",
    }


def post_screen() -> dict:
    bent = [
        [(-54,149),(-56,347)],
        [(-54,149),(-30,179),(-30,268),(-56,308),(-56,347)],
        [(54,149),(84,222),(84,294),(56,334),(56,347)],
        [(54,149),(84,222),(84,294),(56,334),(56,347)],
    ]
    straight = [
        [(-54,149),(-56,347)], [(-54,149),(-56,347)],
        [(54,149),(56,347)], [(54,149),(56,347)],
    ]
    # Conservative local strip bounds for the unchanged 4 mm panel.
    def strip(width, thickness=4.0):
        span, load, e = 116.0, 30.0, 1500.0
        inertia = width * thickness**3 / 12
        return load * span**3 / (48 * e * inertia)
    return {
        "bent_p276_posts_rigid_panel": panel_post_frame(bent),
        "straight_posts_rigid_panel": panel_post_frame(straight),
        "unchanged_4mm_panel_center_strip_mm": {
            "effective_width_40mm": strip(40),
            "optimistic_full_width_138mm": strip(138),
            "interpretation": "post straightening alone does not prove <=1 mm because local panel bending is comparable or dominant",
        },
        "proposed_local_12mm_ribbed_strip_mm": {
            "effective_width_40mm": strip(40, 12),
            "straight_post_plus_strip_bound_mm": abs(panel_post_frame(straight)["vertical_displacement_mm"]) + strip(40, 12),
            "interpretation": "screen for a monolithic 12 mm local E-stop rib/doubler tied directly to the straight-post pair",
        },
    }


def main() -> None:
    rows, comparison = run_chassis()
    report = {
        "status": "P2_77_UNDERSLUNG_OFFSET_FEA_SCREEN",
        "p265_reference_mm": {
            "vertical_2g": 0.6455188490962454,
            "launch_reaction": 2.2118413360915947,
            "antisymmetric_racking": 2.1953988671826923,
        },
        "topology": {
            "single_diagonal": "one X240/Y-150 to X300/Y+150 brace only",
            "underslung_offset": "both braces below X240/X300; first flat, second centreline +12 mm at X270/Y0; no crossing node tie",
            "underdeck_node": "selected common node at Z17; both braces share the node, with conservative 10x10 PETG endpoint transfer links to crossbar centrelines",
        },
        "comparison": comparison,
        "cases": rows,
        "rear_pod_panel_post_screen": post_screen(),
        "limitations": [
            "linear isotropic PETG screen; no layer, bolt bearing, clamp slip or fatigue certification",
            "offset brace endpoint clevises are ideal beam ties; physical coupon/proof load remains mandatory",
            "panel-post frame assumes a rigid control panel; separate strip bounds expose the local 4 mm panel risk",
        ],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"comparison": comparison, "panel_posts": report["rear_pod_panel_post_screen"]}, indent=2))


if __name__ == "__main__":
    main()
