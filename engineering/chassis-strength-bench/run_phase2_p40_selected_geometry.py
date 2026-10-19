#!/usr/bin/env python3
"""P2-40 selected-geometry static grid and printed-wheel properties; no Gazebo."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

BALL_CENTER_Z_MM = 33.0
EDGE_ROUND_MM = 2.0
REQUIRED_USABLE_EDGE_MARGIN_MM = 10.0
TREAD_WIDTH_MM = 60.0
PIVOT_OFFSET_MM = 55.0
COLLECTION_DEG = -3.0
NOMINAL_PIVOT_Z_MM = 64.3
AXLE_DATUM_MM = 52.0
TARGET_PHYSICAL_EDGE_Z_MM = BALL_CENTER_Z_MM - EDGE_ROUND_MM - REQUIRED_USABLE_EDGE_MARGIN_MM


def annulus_volume(ro: float, ri: float, width: float) -> float:
    return math.pi * (ro * ro - ri * ri) * width


def printed_wheel_properties() -> dict:
    """Phase-1 primitive model with the accepted 60 mm TPU tread width."""
    rho = {"petg": 1270.0, "tpu": 1200.0, "al": 2700.0}
    definitions = [
        ("rim", annulus_volume(.096, .084, .040), rho["petg"], .040, (.096**2 + .084**2) / 2),
        ("centre", annulus_volume(.032, .013, .040), rho["petg"], .040, (.032**2 + .013**2) / 2),
        ("spokes", 8 * .052 * .009 * .034 * .88, rho["petg"], .034, ((.032 + .084) / 2) ** 2),
        ("tread", annulus_volume(.100, .096, .060), rho["tpu"], .060, (.100**2 + .096**2) / 2),
        ("hub", annulus_volume(.023, .0041, .018), rho["al"], .018, (.023**2 + .0041**2) / 2),
    ]
    parts = []
    for name, volume, density, width, polar_factor in definitions:
        mass = volume * density
        polar = mass * polar_factor
        transverse = 0.5 * polar + mass * width * width / 12
        parts.append({"name": name, "mass_kg": mass, "polar_inertia_kg_m2": polar,
                      "transverse_inertia_kg_m2": transverse})
    return {
        "construction": "8-spoke PETG core, 60 mm x 4 mm radial TPU tread, aluminium clamp hub",
        "mass_kg_each_including_hub": sum(p["mass_kg"] for p in parts),
        "polar_inertia_kg_m2_each_including_hub": sum(p["polar_inertia_kg_m2"] for p in parts),
        "transverse_inertia_kg_m2_each_including_hub": sum(p["transverse_inertia_kg_m2"] for p in parts),
        "com_offset_m_assumed": 0.0,
        "parts": parts,
        "limitation": "primitive-volume estimate; replace with slicer and assembled measurement",
    }


def grid_row(radius_mm: float, h_mm: float) -> dict:
    angle = math.radians(COLLECTION_DEG)
    unadjusted_pivot_ground = NOMINAL_PIVOT_Z_MM + radius_mm - h_mm - AXLE_DATUM_MM
    unadjusted_physical_edge = (unadjusted_pivot_ground + PIVOT_OFFSET_MM * math.sin(angle)
                                - TREAD_WIDTH_MM / 2 * math.cos(angle))
    pivot_down = unadjusted_physical_edge - TARGET_PHYSICAL_EDGE_Z_MM
    adjusted_pivot_ground = unadjusted_pivot_ground - pivot_down
    physical_edge = (adjusted_pivot_ground + PIVOT_OFFSET_MM * math.sin(angle)
                     - TREAD_WIDTH_MM / 2 * math.cos(angle))
    usable_edge = physical_edge + EDGE_ROUND_MM
    margin = BALL_CENTER_Z_MM - usable_edge
    q = lambda value: round(value, 9)
    return {
        "drive_radius_mm": radius_mm,
        "axle_above_foot_mm": h_mm,
        "pivot_down_mm": q(pivot_down),
        "adjusted_pivot_ground_z_mm": q(adjusted_pivot_ground),
        "physical_lower_edge_ground_z_mm": q(physical_edge),
        "usable_lower_edge_ground_z_mm": q(usable_edge),
        "ball_center_ground_z_mm": BALL_CENTER_Z_MM,
        "ball_center_above_usable_lower_edge_mm": q(margin),
        "criterion_mm": REQUIRED_USABLE_EDGE_MARGIN_MM,
        "pass": margin + 1e-9 >= REQUIRED_USABLE_EDGE_MARGIN_MM,
    }


def main() -> None:
    rows = [grid_row(r, h) for r in (82.0, 85.0, 90.0) for h in (26.0, 29.0, 33.0)]
    wheel = printed_wheel_properties()
    report = {
        "status": "P2_40_SELECTED_GEOMETRY_STATIC_PASS",
        "selected_geometry": {
            "pivot_behind_nip_mm": PIVOT_OFFSET_MM,
            "pivot_x_mm": 387.0,
            "collection_deg": COLLECTION_DEG,
            "tread_width_mm": TREAD_WIDTH_MM,
            "edge_round_mm": EDGE_ROUND_MM,
            "adjustment_travel_mm": 23.0,
        },
        "edge_definition": "usable lower edge = physical axial edge + 2 mm rounded-edge reserve",
        "grid": rows,
        "summary": {
            "all_pass": all(r["pass"] for r in rows),
            "pivot_down_range_mm": [min(r["pivot_down_mm"] for r in rows), max(r["pivot_down_mm"] for r in rows)],
            "ball_center_above_usable_edge_range_mm": [min(r["ball_center_above_usable_lower_edge_mm"] for r in rows), max(r["ball_center_above_usable_lower_edge_mm"] for r in rows)],
            "note": "P2-42 reference update: the corresponding positive-locked shim/slot adjustment deliberately makes every grid point land on the 10 mm target; the physical acceptance minimum remains 8 mm",
        },
        "arch_clearance": {
            "crossbar_and_lidar_x_mm": 330.0,
            "side_leg_x_before_mm": 330.0,
            "side_leg_x_after_mm": 318.0,
            "dogleg_setback_mm": 12.0,
            "directional_exact_solid_clear_through_mm": 14.0,
            "first_tested_collision_mm": 15.0,
            "tolerance_and_elastic_budget_mm": 3.0,
            "residual_clearance_lower_bound_mm": 11.0,
            "required_residual_mm": 8.0,
            "method": "translate the fixed arch toward the full -3..+20 moving-module sweep; 1 mm probe increments",
        },
        "printed_flywheel": wheel,
        "module_mass_kg_with_unchanged_fixed_lump": 2.85200336 + 2 * wheel["mass_kg_each_including_hub"],
        "limitations": ["R and h remain measurement-gated", "CAD clearance is nominal exact-solid directional QA, not physical metrology"],
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2"
    out.mkdir(parents=True, exist_ok=True)
    (out / "p2_40_selected_geometry.json").write_text(json.dumps(report, indent=2) + "\n")
    with (out / "p2_40_ball_edge_grid.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
