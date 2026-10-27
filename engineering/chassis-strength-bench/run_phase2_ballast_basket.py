#!/usr/bin/env python3
"""Phase-2 drive-ballast and 45-ball basket bookkeeping.

Quasi-static only: this deliberately does not claim dynamic equivalence,
printed-part strength, ball-flow validation, or skid-steer tyre behaviour.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

G = 9.80665
REAR_AXLE_X_M = -0.065
FRONT_AXLE_X_M = 0.300
MODULE_MASS_KG = 4.706
MODULE_CG_X_M = 0.427
BALLAST_CG_X_M = 0.360
BALL_MASS_KG = 0.057
BALL_DIAMETER_M = 0.066


def axle_loads(parts: list[dict]) -> dict:
    mass = sum(p["mass_kg"] for p in parts)
    cg_x = sum(p["mass_kg"] * p["x_m"] for p in parts) / mass
    front_fraction = (cg_x - REAR_AXLE_X_M) / (FRONT_AXLE_X_M - REAR_AXLE_X_M)
    front_n = mass * G * front_fraction
    return {
        "mass_kg": mass,
        "cg_x_m": cg_x,
        "front_axle_fraction": front_fraction,
        "rear_axle_n": mass * G - front_n,
        "front_axle_n": front_n,
        "rear_wheel_each_n": (mass * G - front_n) / 2,
        "front_wheel_each_n": front_n / 2,
        "parts": parts,
    }


def polygon_area_and_centroid(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    twice_area = 0.0
    cx_term = 0.0
    cz_term = 0.0
    for p0, p1 in zip(points, points[1:] + points[:1]):
        cross = p0[0] * p1[1] - p1[0] * p0[1]
        twice_area += cross
        cx_term += (p0[0] + p1[0]) * cross
        cz_term += (p0[1] + p1[1]) * cross
    area = abs(twice_area) / 2
    sign = 1 if twice_area >= 0 else -1
    return area, sign * cx_term / (6 * area), sign * cz_term / (6 * area)


def clip_below(points: list[tuple[float, float]], z_max: float) -> list[tuple[float, float]]:
    clipped: list[tuple[float, float]] = []
    for p0, p1 in zip(points, points[1:] + points[:1]):
        p0_in, p1_in = p0[1] <= z_max, p1[1] <= z_max
        if p0_in:
            clipped.append(p0)
        if p0_in != p1_in:
            ratio = (z_max - p0[1]) / (p1[1] - p0[1])
            clipped.append((p0[0] + ratio * (p1[0] - p0[0]), z_max))
    return clipped


def main() -> None:
    # Match the module moment about the rear axle. Plate, fasteners and loose
    # weights are all included in this target mass.
    ballast_mass = MODULE_MASS_KG * (
        (MODULE_CG_X_M - REAR_AXLE_X_M) /
        (BALLAST_CG_X_M - REAR_AXLE_X_M)
    )
    module_point_inertia = MODULE_MASS_KG * (MODULE_CG_X_M - REAR_AXLE_X_M) ** 2
    ballast_point_inertia = ballast_mass * (BALLAST_CG_X_M - REAR_AXLE_X_M) ** 2

    baseline_parts = [
        {"name": "bare_mobile_base", "mass_kg": 10.0, "x_m": 0.100},
        {"name": "shared_flywheel_module_design", "mass_kg": MODULE_MASS_KG, "x_m": MODULE_CG_X_M},
        {"name": "basket_and_supports_45_ball_design", "mass_kg": 1.70, "x_m": 0.120},
        {"name": "two_Parkside_X20V_4Ah", "mass_kg": 1.30, "x_m": -0.165},
        {"name": "UGREEN_12000mAh_100W", "mass_kg": 0.309, "x_m": -0.100},
        {"name": "Actuonix_P16", "mass_kg": 0.095, "x_m": 0.320},
    ]
    ballast_parts = [p for p in baseline_parts if "flywheel_module" not in p["name"]]
    ballast_parts.append({"name": "front_plate_plus_secured_ballast", "mass_kg": ballast_mass, "x_m": BALLAST_CG_X_M})

    full_parts = baseline_parts + [
        {"name": "45_tennis_balls", "mass_kg": 45 * BALL_MASS_KG, "x_m": 0.105}
    ]

    # Inner cavity approximation from the parametric CAD after 3 mm walls.
    # Coordinates are metres in the longitudinal/vertical cross-section.
    inner_polygon = [(-0.012, 0.046), (0.275, 0.058), (0.202, 0.219), (-0.012, 0.219)]
    area_m2, cavity_cg_x_m, cavity_cg_z_m = polygon_area_and_centroid(inner_polygon)
    inner_width_m = 0.344
    gross_cavity_m3 = area_m2 * inner_width_m
    guide_and_feeder_keepout_m3 = 0.00130
    usable_m3 = gross_cavity_m3 - guide_and_feeder_keepout_m3
    ball_volume_m3 = 4 / 3 * math.pi * (BALL_DIAMETER_M / 2) ** 3
    uniform_fill_thresholds = {}
    for label, z_m in (("guide_lower_lip", 0.094), ("guide_center", 0.132), ("guide_upper_lip", 0.168)):
        clipped_area, _, _ = polygon_area_and_centroid(clip_below(inner_polygon, z_m))
        volume_m3 = clipped_area * inner_width_m
        uniform_fill_thresholds[label] = {
            "z_m": z_m,
            "volume_l": volume_m3 * 1000,
            "balls_at_0.58_packing_after_0.2_l_floor_keepout":
                max(0.0, volume_m3 - 0.0002) * 0.58 / ball_volume_m3,
        }

    report = {
        "status": "PHASE_2_PRE_PHYSICAL_TEST_PARAMETRIC",
        "ballast": {
            "target_total_plate_fasteners_and_ballast_kg": ballast_mass,
            "target_x_m": BALLAST_CG_X_M,
            "reference_module_mass_kg": MODULE_MASS_KG,
            "reference_module_x_m": MODULE_CG_X_M,
            "front_axle_load_match_basis": "equal moment about rear axle X=-0.065 m",
            "point_mass_yaw_pitch_inertia_about_rear_axle_kg_m2": {
                "module": module_point_inertia,
                "ballast": ballast_point_inertia,
                "difference_percent": 100 * (ballast_point_inertia / module_point_inertia - 1),
            },
            "with_real_module": axle_loads(baseline_parts),
            "with_drive_test_ballast": axle_loads(ballast_parts),
            "dummy_power_masses_if_packs_absent": [
                {"item": "Parkside left", "mass_kg": 0.650, "x_m": -0.165, "y_m": -0.110},
                {"item": "Parkside right", "mass_kg": 0.650, "x_m": -0.165, "y_m": 0.110},
                {"item": "UGREEN", "mass_kg": 0.309, "x_m": -0.100, "y_m": 0.0},
            ],
        },
        "basket": {
            "outer_parameters_mm": {
                "rear_x": -15, "front_x": 278, "width": 350,
                "rear_floor_z": 42, "front_floor_z": 54,
                "top_z": 215, "upper_front_x": 205,
            },
            "inner_cross_section_area_m2": area_m2,
            "inner_cavity_centroid_m": {"x": cavity_cg_x_m, "z": cavity_cg_z_m},
            "gross_inner_volume_l": gross_cavity_m3 * 1000,
            "guide_and_feeder_keepout_l_assumed": guide_and_feeder_keepout_m3 * 1000,
            "usable_volume_l": usable_m3 * 1000,
            "ball_solid_volume_l": ball_volume_m3 * 1000,
            "capacity_by_random_packing": {
                str(phi): usable_m3 * phi / ball_volume_m3 for phi in (0.55, 0.58, 0.60, 0.64)
            },
            "uniform_fill_inlet_threshold_screen": uniform_fill_thresholds,
            "feed_load_screen": {
                "design_local_ball_column_count": 10,
                "ball_column_weight_n": 10 * BALL_MASS_KG * G,
                "singulator_effective_radius_m": 0.023,
                "gravity_torque_nm": 10 * BALL_MASS_KG * G * 0.023,
                "design_factor_for_friction_and_wedge": 3.0,
                "minimum_output_torque_nm": 3 * 10 * BALL_MASS_KG * G * 0.023,
                "minimum_tangential_force_n": 15.0,
            },
            "full_45_ball_case": axle_loads(full_parts),
            "ball_mass_45_kg": 45 * BALL_MASS_KG,
            "limitations": [
                "capacity is geometric and depends strongly on guide/feeder keepout and packing",
                "ball CG X=0.105 m and basket/support mass 1.70 kg are pre-physical-test estimates",
                "flow, bridging and inlet closure require physical collection testing",
            ],
        },
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2/ballast_basket.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "ballast_target_kg": ballast_mass,
        "ballast_inertia_difference_percent": report["ballast"]["point_mass_yaw_pitch_inertia_about_rear_axle_kg_m2"]["difference_percent"],
        "drive_test": report["ballast"]["with_drive_test_ballast"],
        "basket_usable_l": report["basket"]["usable_volume_l"],
        "basket_capacity": report["basket"]["capacity_by_random_packing"],
        "full_45": report["basket"]["full_45_ball_case"],
    }, indent=2))


if __name__ == "__main__":
    main()
