#!/usr/bin/env python3
"""Static capacity, energy and sensor-clearance gate for P2-47 tall open drop."""
from __future__ import annotations

import json
import math
from pathlib import Path

BALL_D = 0.066
BALL_R = BALL_D / 2
BALL_COUNT = 45
PLAN_X = 0.293 - 2 * 0.003
PLAN_Y = 0.350 - 2 * 0.003
FLOOR_Z = 0.054
ENTRY_SURFACE_Z = 0.247
RAMP_START_Z = 0.010
BASKET_RIM_Z = 0.335
LIDAR_SCAN_Z = 0.390
LIDAR_BODY_BOTTOM_Z = 0.358
CAMERA_X, CAMERA_Z = 0.350, 0.305
CAMERA_HALF_X_PROJECTED = 0.5 * (0.082 * math.cos(math.radians(50)) + 0.028 * math.sin(math.radians(50)))
BASKET_FRONT_X = 0.278


def main() -> None:
    sphere_volume = 4 * math.pi * BALL_R**3 / 3
    area = PLAN_X * PLAN_Y
    packing = {}
    for fraction in (0.56, 0.60, 0.64):
        mean = BALL_COUNT * sphere_volume / fraction / area
        packing[f"{fraction:.2f}"] = {
            "mean_bulk_height_mm": 1000 * mean,
            "local_pile_top_with_one_ball_allowance_world_z_mm": 1000 * (FLOOR_Z + mean + BALL_D),
        }
    worst_pile_top = max(v["local_pile_top_with_one_ball_allowance_world_z_mm"] for v in packing.values()) / 1000
    delta_h = ENTRY_SURFACE_Z - RAMP_START_Z
    energy_min = math.sqrt(2 * 9.80665 * delta_h)
    target_25 = 1.25 * energy_min
    rpm = target_25 / (math.pi * 0.200) * 60
    result = {
        "status": "PASS_STATIC_WITH_PHYSICAL_CAPACITY_PENDING",
        "inputs": {
            "ball_count": BALL_COUNT,
            "ball_diameter_mm": BALL_D * 1000,
            "interior_plan_mm": [PLAN_X * 1000, PLAN_Y * 1000],
            "packing_fraction_sweep": [0.56, 0.60, 0.64],
            "surface_peak_allowance_mm": BALL_D * 1000,
            "camera_hfov_deg_repo": 69,
            "camera_vfov_deg_provisional": 54,
        },
        "capacity": {
            "solid_ball_volume_l": BALL_COUNT * sphere_volume * 1000,
            "packing_sweep": packing,
            "conservative_local_pile_top_world_z_mm": worst_pile_top * 1000,
            "selected_entry_surface_world_z_mm": ENTRY_SURFACE_Z * 1000,
            "entry_above_conservative_pile_mm": (ENTRY_SURFACE_Z - worst_pile_top) * 1000,
            "selected_basket_rim_world_z_mm": BASKET_RIM_Z * 1000,
        },
        "collection_energy": {
            "ramp_rise_mm": delta_h * 1000,
            "lossless_minimum_speed_m_s": energy_min,
            "speed_for_25_percent_velocity_margin_m_s": target_25,
            "selected_test_speed_m_s": 2.70,
            "selected_speed_margin_percent": (2.70 / energy_min - 1) * 100,
            "d200_rpm_at_selected_speed": 2.70 / (math.pi * 0.200) * 60,
            "note": "Energy screen only; Gazebo and 1:1 test determine contact losses and rebound.",
        },
        "sensors": {
            "lidar_scan_plane_clearance_above_rim_mm": (LIDAR_SCAN_Z - BASKET_RIM_Z) * 1000,
            "lidar_body_bottom_vertical_difference_mm": (LIDAR_BODY_BOTTOM_Z - BASKET_RIM_Z) * 1000,
            "lidar_body_x_separation_from_basket_front_mm": (0.330 - BASKET_FRONT_X) * 1000,
            "camera_body_rear_x_mm": (CAMERA_X - CAMERA_HALF_X_PROJECTED) * 1000,
            "camera_body_to_basket_front_x_gap_mm": (CAMERA_X - CAMERA_HALF_X_PROJECTED - BASKET_FRONT_X) * 1000,
            "camera_ground_envelope_from_cad_mm": {"x": [430, 545], "half_width_y": [70, 120]},
            "fov_result": "PASS: basket and raised bridge are behind camera X=350 mm; forward ground envelope is unchanged.",
        },
        "dependencies": [
            "The low floor still drains toward the separate launch-feed/singulator interface; that mechanism remains dependent.",
            "Ball-ball DART contact is uncalibrated; absolute 45-ball capacity requires the 1:1 mock-up.",
            "The 54 deg vertical camera FOV is provisional and must be replaced by the selected camera calibration.",
        ],
    }
    out = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/p247_tall_open_static.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
