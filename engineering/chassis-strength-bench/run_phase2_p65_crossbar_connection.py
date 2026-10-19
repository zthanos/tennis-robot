#!/usr/bin/env python3
"""P2-65 FEA screen for the physical X240/X300 T-connections.

CalculiX B32R centreline model, mm/N/MPa.  This keeps the released Phase-2
combined topology and replaces only the ideal crossbar-to-rail overlap with
195 mm half-crossbars plus the printed T-clamp load paths.  Bolt/contact and
printed-layer strength remain physical gates, not claims of this beam model.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess

from run_global_frame_fea import ROOT, SOLVER, runtime_env
from run_shared_flywheel_phase1 import mass_model, pivot_loads, traction_screen
from run_shared_flywheel_phase2 import (
    PETG_E_MPA,
    PETG_NU,
    parse_dat,
    write_deck,
)

CASES = ("vertical_2g", "launch_reaction", "antisymmetric_racking")
OLD_MM = {
    "vertical_2g": 0.6444354782568961,
    "launch_reaction": 1.9400930111550323,
    "antisymmetric_racking": 1.9811470587030258,
}


def build_model(
    mesh_mm: float,
    torsion_variant: str = "crossed",
    connection_style: str = "t_clamp",
    single_diagonal_height_mm: float = 10.0,
):
    nodes: dict[int, tuple[float, float, float]] = {}
    index: dict[tuple[float, float, float], int] = {}
    elements: list[tuple[int, int, int, int]] = []
    sections = []

    def node(point):
        key = tuple(round(value, 6) for value in point)
        if key not in index:
            index[key] = len(index) + 1
            nodes[index[key]] = key
        return index[key]

    def member(a, b, w=50, h=30, t=3.2, kind="BOX", material="PETG"):
        length = math.dist(a, b)
        count = max(1, math.ceil(length / mesh_mm))
        ids = [
            node(tuple(a[k] + (b[k] - a[k]) * i / (2 * count) for k in range(3)))
            for i in range(2 * count + 1)
        ]
        eids = []
        for i in range(count):
            eid = len(elements) + 1
            elements.append((eid, ids[2 * i], ids[2 * i + 1], ids[2 * i + 2]))
            eids.append(eid)
        tangent = [(b[k] - a[k]) / length for k in range(3)]
        axis = [0, 0, 1] if abs(tangent[2]) < 0.95 else [0, 1, 0]
        dot = sum(axis[k] * tangent[k] for k in range(3))
        normal = [axis[k] - dot * tangent[k] for k in range(3)]
        magnitude = math.sqrt(sum(value * value for value in normal))
        normal = [value / magnitude for value in normal]
        sections.append((eids, w, h, t, kind, normal, material))

    # Identical longitudinal rails, supports and carrier to the released
    # combined model.
    stations = [-165, -65, 35, 200, 208, 240, 272, 300, 320, 330]
    for side in (-1, 1):
        for a, b in zip(stations, stations[1:]):
            member((a, side * 220, 37), (b, side * 220, 37))
    member((-165, -220, 37), (-165, 220, 37))

    # Physical 195 mm X240 crossbar and retained local braces.
    ys = [-195, -150, 0, 150, 195]
    for a, b in zip(ys, ys[1:]):
        member((240, a, 32), (240, b, 32), 50, 20)
    for side in (-1, 1):
        member((208, side * 220, 37), (240, side * 150, 32), 6, 20, 0, "RECT")
        member((272, side * 220, 37), (240, side * 150, 32), 6, 20, 0, "RECT")

    bearings = []
    for side in (-1, 1):
        bearing = (397, side * 237, 64.3)
        member((300, side * 220, 37), (330, side * 220, 52), 45, 12, 0, "RECT")
        member((330, side * 220, 52), bearing, 45, 12, 0, "RECT")
        member((320, side * 220, 37), bearing, 45, 12, 0, "RECT")
        bearings.append(node(bearing))

        # Released arch and side triangulation.
        member((330, side * 220, 37), (330, side * 220, 97), 50, 30, 3.2)
        member((330, side * 220, 97), (330, side * 220, 245))
        for start in ((240, side * 220, 37), (300, side * 220, 37)):
            end = (330, side * 220, 245)
            length = math.dist(start, end)
            fraction = 60 / length
            split = tuple(start[k] + (end[k] - start[k]) * fraction for k in range(3))
            member(start, split, 40, 25, 3.2)
            member(split, end, 40, 25)
    member((330, -220, 245), (330, 220, 245), 50, 30)

    # Physical 195 mm X300 crossbar and torsion deck.  The optional variants
    # are used by the P2-77 repair study; the default preserves P2-65 exactly.
    member((300, -195, 32), (300, 195, 32), 20, 20)
    if torsion_variant in ("crossed", "single_diagonal", "underslung_offset"):
        member(
            (240, -150, 32), (300, 150, 32),
            20, single_diagonal_height_mm, 0, "RECT",
        )
    if torsion_variant == "crossed":
        member((240, 150, 32), (300, -150, 32), 20, 10, 0, "RECT")
    elif torsion_variant == "underslung_offset":
        # P2-77 analytical centreline surrogate: same endpoints, but the
        # second brace rises 12 mm locally at the crossing and is not tied
        # to the first brace there.
        member((240, 150, 32), (270, 0, 44), 20, 10, 0, "RECT")
        member((270, 0, 44), (300, -150, 32), 20, 10, 0, "RECT")
    elif torsion_variant == "underdeck_node":
        # Accepted P2-77 candidate: both diagonals share one node at Z17 and
        # connect to the existing crossbar grid through the 15 mm endpoint
        # bolt/bearing path.  The 10x10 PETG links avoid crediting steel-bolt
        # stiffness and expose the placement eccentricity conservatively.
        member((240, -150, 17), (300, 150, 17), 20, 10, 0, "RECT")
        member((240, 150, 17), (300, -150, 17), 20, 10, 0, "RECT")
        for x, y in ((240, -150), (240, 150), (300, -150), (300, 150)):
            member((x, y, 17), (x, y, 32), 10, 10, 0, "RECT")
    elif torsion_variant == "paired_short_inboard":
        # One short brace per side, clear of the central feed corridor and
        # X300 ski-bracket stations Y75/Y100.
        for side in (-1, 1):
            member(
                (240, side * 150, 32), (300, side * 125, 32),
                20, single_diagonal_height_mm, 0, "RECT",
            )
    elif torsion_variant == "paired_short_outboard":
        for side in (-1, 1):
            member(
                (240, side * 125, 32), (300, side * 150, 32),
                20, single_diagonal_height_mm, 0, "RECT",
            )
    elif torsion_variant == "paired_short_near_rail":
        for side in (-1, 1):
            member(
                (240, side * 150, 32), (300, side * 175, 32),
                20, single_diagonal_height_mm, 0, "RECT",
            )
    elif torsion_variant not in ("single_diagonal", "none"):
        raise ValueError(f"unknown torsion variant: {torsion_variant}")

    # Conservative printed T-clamp paths.  Each is tied at the crossbar end
    # and at the longitudinal-rail centreline.  The rectangular sections use
    # only the 6 mm upright and its actual 24/11 mm widths; steel bolt stiffness
    # is not credited.
    for side in (-1, 1):
        if connection_style == "t_clamp":
            member((240, side * 195, 32), (240, side * 220, 37), 24, 6, 0, "RECT")
            member((300, side * 195, 32), (300, side * 220, 37), 11, 6, 0, "RECT")
        elif connection_style == "sleeve_socket":
            # P2-78: the terminal grows hollow sockets having the complete
            # crossbar section.  The internal 60 mm-per-side sleeve transfers
            # load through two through-M4 stations.  The beam screen credits
            # only the printed socket tube, not bolt or contact stiffness.
            member((240, side * 195, 32), (240, side * 220, 37), 50, 20, 3.2)
            member((300, side * 195, 32), (300, side * 220, 37), 20, 20, 3.2)
        else:
            raise ValueError(f"unknown connection style: {connection_style}")

    supports = {
        "rear_right": node((-65, -220, 37)),
        "rear_left": node((-65, 220, 37)),
        "front_right": node((300, -220, 37)),
        "front_left": node((300, 220, 37)),
    }
    return nodes, elements, sections, supports, bearings, {"sleeve_e_mpa": PETG_E_MPA}


def run(work: Path) -> dict:
    pivot = pivot_loads(mass_model(), traction_screen())
    rows = []
    for mesh_mm in (10.0, 5.0):
        model = build_model(mesh_mm)
        for case_name in CASES:
            stem = f"p265_t_connection_{case_name}_{mesh_mm:g}mm"
            deck = work / f"{stem}.inp"
            write_deck(deck, model, case_name, pivot, PETG_E_MPA)
            process = subprocess.run(
                [str(SOLVER), "-i", stem],
                cwd=work,
                env=runtime_env(),
                capture_output=True,
                text=True,
                timeout=60,
            )
            (work / f"{stem}.log").write_text(process.stdout + process.stderr)
            if process.returncode or "*ERROR" in (process.stdout + process.stderr).upper():
                raise RuntimeError(f"CalculiX failed: {stem}")
            rows.append(
                {
                    "load_case": case_name,
                    "mesh_mm": mesh_mm,
                    "element_count": len(model[1]),
                    **parse_dat(work / f"{stem}.dat", model[0], model[3], model[4]),
                }
            )

    comparison = {}
    for case_name in CASES:
        fine = next(row for row in rows if row["load_case"] == case_name and row["mesh_mm"] == 5)
        coarse = next(row for row in rows if row["load_case"] == case_name and row["mesh_mm"] == 10)
        new_mm = fine["max_bearing_displacement_mm"]
        comparison[case_name] = {
            "old_ideal_overlap_mm": OLD_MM[case_name],
            "p265_t_connection_mm": new_mm,
            "change_percent": 100 * (new_mm / OLD_MM[case_name] - 1),
            "mesh_change_10_to_5_percent": 100 * abs(
                new_mm - coarse["max_bearing_displacement_mm"]
            ) / new_mm,
            "max_bearing_relative_to_local_front_mount_mm": fine[
                "max_bearing_relative_to_local_front_mount_mm"
            ],
            "beam_integration_point_peak_von_mises_mpa": fine[
                "beam_integration_point_peak_von_mises_mpa"
            ],
        }
    return {
        "status": "P2_65_T_CONNECTION_FEA_PASS",
        "model": "released combined topology; 195 mm X240/X300 halves; 24x6 and 11x6 PETG T-upright load paths; bolt stiffness omitted",
        "material": f"PETG isotropic screen, E={PETG_E_MPA / 1000:g} GPa, nu={PETG_NU}",
        "comparison": comparison,
        "cases": rows,
        "limitations": [
            "centreline beam comparison, not printed-layer/fastener/contact certification",
            "T-clamp ties are ideal at their endpoints; M4 preload, hole bearing, slip and fatigue require physical inspection",
            "the X300 C-hook relief is too local for this beam mesh and is not represented",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--work",
        type=Path,
        default=ROOT / "fea/work/p265_crossbar_connection",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/shared_flywheel_phase2/p265_crossbar_connection.json",
    )
    args = parser.parse_args()
    args.work.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = run(args.work)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["comparison"], indent=2))


if __name__ == "__main__":
    main()
