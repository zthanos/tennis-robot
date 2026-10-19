#!/usr/bin/env python3
"""P2-79/P2-80 rear support comparison; linear analytical screen only.

Units are mm, N and MPa.  This is deliberately not an STL generator.  Joint
slip, printed anisotropy, preload loss and fatigue remain proof-load gates.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

PETG_E = 1500.0
AL_E = 69000.0
STEEL_E = 200000.0
PANEL_LOAD = 30.0
SIDE_LOAD = 20.0
POST_L = 198.0
CAGE_L = 285.0
LOWER_BRACED_L = 87.0


def cantilever(load: float, length: float, e: float, inertia_total: float) -> float:
    return load * length**3 / (3 * e * inertia_total)


def strip(load: float, span: float, width: float, thickness: float, e: float) -> float:
    inertia = width * thickness**3 / 12
    return load * span**3 / (48 * e * inertia)


def p279() -> dict:
    d = 23.0
    area = math.pi * d**2 / 4
    inertia = math.pi * d**4 / 64
    section = math.pi * d**3 / 32
    axial = PANEL_LOAD * POST_L / (4 * PETG_E * area)
    # Existing four-hole panel, full-width strip bound used in P2-77.  The
    # 40 mm local-strip sensitivity is also retained and reported separately.
    panel_full = strip(PANEL_LOAD, 116, 138, 4, PETG_E)
    panel_local = strip(PANEL_LOAD, 116, 40, 4, PETG_E)
    side = cantilever(SIDE_LOAD, POST_L, PETG_E, 4 * inertia)
    stress = (SIDE_LOAD / 4) * POST_L / section
    pitch_couple = SIDE_LOAD * (347 - 55) / 30
    vertical_2g = 2 * 9.81 * (1.0 + 1.30)
    brake_force = 0.5 * 9.81 * (1.0 + 1.30)
    return {
        "top_displacement_mm": {
            "30N_vertical_full_panel_bound": panel_full + axial,
            "30N_vertical_40mm_strip_sensitivity": panel_local + axial,
            "20N_lateral": side,
            "20N_fore_aft": side,
            "0p5g_braking_loaded_2p3kg_scaled_fore_aft": side * brake_force / SIDE_LOAD,
            "2g_vertical_loaded_2p3kg_post_shortening": vertical_2g * POST_L / (4 * PETG_E * area),
        },
        "stress_mpa": {"post_bending_at_20N": stress},
        "bolt_loads_N": {
            "20N_fore_aft_total_M4_couple_at_30mm_pitch": pitch_couple,
            "per_two_bolts_on_tension_line": pitch_couple / 2,
            "30N_vertical_per_four_primary_M4": PANEL_LOAD / 4,
            "0p5g_braking_total_M4_couple_at_30mm_pitch": brake_force * (347 - 55) / 30,
            "2g_vertical_loaded_2p3kg_per_four_primary_M4": vertical_2g / 4,
        },
        "structural_mass_kg_estimate": 1.43,
        "mass_basis": "PETG CAD-volume order estimate for 8 mm bases/ribs, d23 posts, side frames/ties plus M4 hardware",
    }


def p280() -> dict:
    # Four 20x3 aluminium flats: x is the strong direction, y the weak one.
    area = 20 * 3
    ix_weak = 20 * 3**3 / 12
    iy_strong = 3 * 20**3 / 12
    zx_weak = ix_weak / (3 / 2)
    zy_strong = iy_strong / (20 / 2)
    vertical_panel = strip(PANEL_LOAD, 48, 40, 4, PETG_E)
    vertical_column = PANEL_LOAD * CAGE_L / (4 * AL_E * area)
    fore_aft = cantilever(SIDE_LOAD, CAGE_L, AL_E, 4 * iy_strong)
    lower_sway = cantilever(SIDE_LOAD, LOWER_BRACED_L, AL_E, 4 * ix_weak)
    diag_length = math.hypot(2 * 165.5, CAGE_L - LOWER_BRACED_L)
    diag_cos = 2 * 165.5 / diag_length
    diag_force = SIDE_LOAD / diag_cos
    m4_tensile_area = 8.78
    diag_extension_y = (diag_force * diag_length / (STEEL_E * m4_tensile_area)) / diag_cos
    vertical_2g = 2 * 9.81 * (1.0 + 1.30)
    brake_force = 0.5 * 9.81 * (1.0 + 1.30)
    return {
        "top_displacement_mm": {
            "30N_vertical": vertical_panel + vertical_column,
            "20N_lateral_braced": lower_sway + diag_extension_y,
            "20N_fore_aft": fore_aft,
            "0p5g_braking_loaded_2p3kg_scaled_fore_aft": fore_aft * brake_force / SIDE_LOAD,
            "2g_vertical_loaded_2p3kg_column_shortening": vertical_2g * CAGE_L / (4 * AL_E * area),
        },
        "stress_mpa": {
            "aluminium_column_fore_aft_at_20N": (SIDE_LOAD / 4) * CAGE_L / zy_strong,
            "aluminium_lower_weak_axis_with_brace_at_20N": (SIDE_LOAD / 4) * LOWER_BRACED_L / zx_weak,
            "M4_tension_rod_at_20N": diag_force / m4_tensile_area,
        },
        "bolt_loads_N": {
            "M4_diagonal_tension_at_20N_lateral": diag_force,
            "fore_aft_total_base_couple_at_30mm_column_pitch": SIDE_LOAD * CAGE_L / 30,
            "per_side_base_couple": SIDE_LOAD * CAGE_L / 60,
            "2g_vertical_loaded_2p3kg_per_four_columns": vertical_2g / 4,
        },
        "structural_mass_kg_estimate": 0.84,
        "mass_basis": "4x 20x3 Al flats, Al top frame/control rails, 2x M4 steel diagonals, PETG sockets/ties and M4 hardware",
    }


def main() -> None:
    old, cage = p279(), p280()
    result = {
        "status": "P2_80_ANALYSIS_ONLY_NO_STL",
        "load_basis": {
            "electronics_high_kg": 1.0,
            "future_Parkside_pair_kg": 1.30,
            "braking_g": 0.5,
            "vertical_g": 2.0,
            "panel_press_N": 30.0,
            "panel_side_or_fore_aft_N": 20.0,
        },
        "P2_79": old,
        "P2_80": cage,
        "change_percent_P280_vs_P279": {
            "20N_lateral": 100 * (cage["top_displacement_mm"]["20N_lateral_braced"] /
                                     old["top_displacement_mm"]["20N_lateral"] - 1),
            "20N_fore_aft": 100 * (cage["top_displacement_mm"]["20N_fore_aft"] /
                                      old["top_displacement_mm"]["20N_fore_aft"] - 1),
            "structural_mass": 100 * (cage["structural_mass_kg_estimate"] /
                                        old["structural_mass_kg_estimate"] - 1),
        },
        "limitations": [
            "linear isotropic member screen; no joint slip, local hole bearing, preload loss, fatigue or printed-layer certification",
            "P2-79 vertical result is sensitive to effective panel width; the 40 mm strip value is reported and does not pass 1 mm",
            "P2-80 30 N vertical result credits the two added top-frame control rails at X=panel centre +/-24 mm",
            "P2-80 lateral result requires both preloaded M4 steel diagonals; four unbraced 20x3 flats are not acceptable",
            "Parkside and purchased connector dimensions remain physical measurement gates",
        ],
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2/p280_rear_cage.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
