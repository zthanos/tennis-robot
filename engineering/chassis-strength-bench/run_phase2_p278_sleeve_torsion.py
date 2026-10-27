#!/usr/bin/env python3
"""P2-78 sleeve/socket crossbar and torsion-deck comparison.

CalculiX B32R centreline screen in mm/N/MPa.  It replaces the P2-77 narrow
T-clamp transfer paths with full-section hollow terminal sockets and compares
no diagonal, one continuous 20x15 diagonal, and paired short side braces.
Bolt slip, layer anisotropy, local hole bearing and fatigue remain gates.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess

from run_global_frame_fea import ROOT, SOLVER, runtime_env
from run_phase2_p65_crossbar_connection import CASES, build_model
from run_shared_flywheel_phase1 import mass_model, pivot_loads, traction_screen
from run_shared_flywheel_phase2 import PETG_E_MPA, parse_dat, write_deck

OUT = ROOT / "results/shared_flywheel_phase2/p278_sleeve_torsion.json"
WORK = ROOT / "fea/work/p278_sleeve_torsion"
VARIANTS = (
    "none",
    "single_diagonal",
    "paired_short_inboard",
    "paired_short_outboard",
    "paired_short_near_rail",
)
P277_SINGLE_MM = {
    "vertical_2g": 0.6454265055731803,
    "launch_reaction": 2.3754331838128766,
    "antisymmetric_racking": 2.354186401705328,
}


def main() -> None:
    pivot = pivot_loads(mass_model(), traction_screen())
    rows: list[dict] = []
    WORK.mkdir(parents=True, exist_ok=True)
    for variant in VARIANTS:
        for mesh_mm in (10.0, 5.0):
            model = build_model(
                mesh_mm, variant, "sleeve_socket",
                single_diagonal_height_mm=15.0,
            )
            for case_name in CASES:
                stem = f"p278_sleeve_{variant}_{case_name}_{mesh_mm:g}mm"
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

    comparison = {}
    for case_name in CASES:
        case = {}
        one = next(r for r in rows if r["variant"] == "single_diagonal"
                   and r["load_case"] == case_name and r["mesh_mm"] == 5)
        for variant in VARIANTS:
            fine = next(r for r in rows if r["variant"] == variant
                        and r["load_case"] == case_name and r["mesh_mm"] == 5)
            coarse = next(r for r in rows if r["variant"] == variant
                          and r["load_case"] == case_name and r["mesh_mm"] == 10)
            value = fine["max_bearing_displacement_mm"]
            case[variant] = {
                "max_bearing_displacement_mm": value,
                "change_vs_one_diagonal_percent": 100 * (value / one["max_bearing_displacement_mm"] - 1),
                "change_vs_p277_single_percent": 100 * (value / P277_SINGLE_MM[case_name] - 1),
                "mesh_change_10_to_5_percent": 100 * abs(value - coarse["max_bearing_displacement_mm"]) / value,
                "beam_integration_point_peak_von_mises_mpa": fine["beam_integration_point_peak_von_mises_mpa"],
            }
        comparison[case_name] = case

    report = {
        "status": "P2_78_SLEEVE_SOCKET_TORSION_SCREEN",
        "model": "full-section hollow X240/X300 terminal sockets; no bolt/contact stiffness; 20x15 torsion members",
        "selected_variant": "paired_short_inboard",
        "selection": (
            "X240/Y+/-150 to X300/Y+/-125: best variant that passes exact-solid; "
            "paired_short_outboard was slightly stiffer but intersects the future basket floor"
        ),
        "p277_single_diagonal_reference_mm": P277_SINGLE_MM,
        "comparison": comparison,
        "cases": rows,
        "limitations": [
            "linear isotropic PETG beam screen; not printed-layer, bolt-bearing, clamp-slip or fatigue certification",
            "socket-to-terminal and sleeve preload are ideal at the beam endpoints",
            "local M4 holes and the X300 C-hook relief are not represented by the centreline mesh",
        ],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(comparison, indent=2))


if __name__ == "__main__":
    main()
