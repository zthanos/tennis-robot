#!/usr/bin/env python3
"""P2-48 static sizing for the 40-required / 45-best-effort printed basket."""
from __future__ import annotations

import json
import math
from pathlib import Path

BALL_D = 0.066
BALL_R = BALL_D / 2
PLAN = (0.287, 0.344)
PACKING = (0.56, 0.60, 0.64)
FLOOR_HIGH_Z = 0.054
RAMP_START_Z = 0.010
RIM_Z = 0.335
LIDAR_Z = 0.390
HEIGHTS = (0.247, 0.260, 0.275)
EFFECTIVE_DENSITY_G_CM3 = 0.72


def pile(count: int) -> dict:
    sphere = 4 * math.pi * BALL_R**3 / 3
    rows = []
    for packing in PACKING:
        mean = count * sphere / (packing * PLAN[0] * PLAN[1])
        rows.append({
            "packing_fraction": packing,
            "mean_height_mm": mean * 1000,
            "local_top_world_z_mm": (FLOOR_HIGH_Z + mean + BALL_D) * 1000,
        })
    return {
        "count": count,
        "solid_volume_l": count * sphere * 1000,
        "packing_sweep": rows,
        "conservative_local_top_world_z_mm": max(row["local_top_world_z_mm"] for row in rows),
    }


def bridge(z: float, pile40: float, pile45: float) -> dict:
    rise = z - RAMP_START_Z
    lossless = math.sqrt(2 * 9.80665 * rise)
    speed = 1.25 * lossless
    return {
        "surface_z_mm": z * 1000,
        "margin_over_40_mm": z * 1000 - pile40,
        "required_40_margin_pass": z * 1000 - pile40 >= 20,
        "margin_over_45_mm": z * 1000 - pile45,
        "rise_from_ramp_start_mm": rise * 1000,
        "lossless_minimum_speed_m_s": lossless,
        "speed_for_25_percent_margin_m_s": speed,
        "d200_rpm_for_25_percent_margin": speed / (math.pi * 0.2) * 60,
        "inclined_board_release_height_with_12_percent_loss_allowance_mm":
            1.12 * (5 / 6) * speed ** 2 / 9.80665 * 1000,
        "surface_to_rim_mm": (RIM_Z - z) * 1000,
        "ball_top_to_rim_mm": (RIM_Z - z - BALL_D) * 1000,
        "surface_to_lidar_scan_mm": (LIDAR_Z - z) * 1000,
        "ball_top_to_lidar_scan_mm": (LIDAR_Z - z - BALL_D) * 1000,
    }


def mass_estimate() -> dict:
    # Outer panel areas.  The front/rear floor datums are 42/54 mm and the
    # 335 mm rim is fixed; side height uses the mean of both ends.
    side_area = 2 * 0.293 * ((0.335 - 0.042 + 0.335 - 0.054) / 2)
    end_area = 0.350 * (0.335 - 0.042) + 0.350 * (0.335 - 0.054)
    floor_area = 0.293 * 0.350
    thickness = 0.003
    density = EFFECTIVE_DENSITY_G_CM3 * 1e6  # g/m3
    solid_shell_g = (side_area + end_area + floor_area) * thickness * density
    wall_grid_fraction = 0.37
    floor_grid_fraction = 0.44
    grid_shell_g = ((side_area + end_area) * wall_grid_fraction + floor_area * floor_grid_fraction) * thickness * density
    # Printed bridge, seam straps, corner angles, mount ears and port frame.
    common_hardware_print_g = 205
    bridge_and_rails_g = 95
    solid_total = solid_shell_g + common_hardware_print_g + bridge_and_rails_g
    grid_total = grid_shell_g + common_hardware_print_g + bridge_and_rails_g
    return {
        "effective_density_g_cm3": EFFECTIVE_DENSITY_G_CM3,
        "solid_shell_plus_interfaces_g": solid_total,
        "grid_shell_plus_interfaces_g": grid_total,
        "filament_saved_g": solid_total - grid_total,
        "saving_percent": 100 * (solid_total - grid_total) / solid_total,
        "uncertainty": "plus/minus 15 percent until sliced with the selected walls, infill and hardware",
    }


def feed_interface() -> dict:
    port = (0.278, 0.0, 0.075)
    mouths = {
        "launch_15_deg": (0.329183, 0.0, 0.059063),
        "launch_20_deg": (0.331475, 0.0, 0.054115),
    }
    paths = {}
    for name, mouth in mouths.items():
        dx = mouth[0] - port[0]
        dz = mouth[2] - port[2]
        paths[name] = {
            "mouth_ball_center_mm": [v * 1000 for v in mouth],
            "delta_x_mm": dx * 1000,
            "drop_mm": -dz * 1000,
            "centreline_length_mm": math.hypot(dx, dz) * 1000,
            "downward_angle_deg": math.degrees(math.atan2(-dz, dx)),
        }
    floor_angle = math.atan2(0.054 - 0.042, 0.278 - (-0.015))
    ideal_accel = (5 / 7) * 9.80665 * math.sin(floor_angle)
    return {
        "port_clear_mm": [80, 80],
        "port_frame_mm": [120, 120],
        "port_ball_center_mm": [v * 1000 for v in port],
        "paths": paths,
        "floor_slope_deg": math.degrees(floor_angle),
        "ideal_solid_sphere_acceleration_m_s2": ideal_accel,
        "ideal_speed_over_293_mm_m_s": math.sqrt(2 * ideal_accel * 0.293),
        "claim_limit": "Interface/keep-out only; feeder and singulator remain Phase 5d.",
    }


def main() -> None:
    p40, p45 = pile(40), pile(45)
    result = {
        "status": "P2_48_STATIC_PASS_HEIGHT_260_DEPENDENT_PHYSICAL_TEST_REQUIRED",
        "acceptance_rule": {"required_ball_count": 40, "best_effort_ball_range": [41, 45]},
        "geometry": {"interior_plan_mm": [287, 344], "floor_world_z_rear_front_mm": [54, 42], "rim_z_mm": 335, "lidar_scan_z_mm": 390},
        "piles": {"40": p40, "45": p45},
        "bridge_options": [bridge(z, p40["conservative_local_top_world_z_mm"], p45["conservative_local_top_world_z_mm"]) for z in HEIGHTS],
        "selected_reference_surface_z_mm": 260,
        "mass": mass_estimate(),
        "panelization": {
            "print_bed_mm": [256, 256],
            "largest_main_panel_mm": [175, 146.5],
            "wall_clear_opening_mm": 38,
            "floor_clear_opening_mm": 18,
            "solid_feed_lane_mm": 86,
            "chassis_mount_grid": "M4 at X=240/300 mm, Y=+/-100/150 mm",
        },
        "feed_interface": feed_interface(),
        "inclined_board": {
            "thin_shell_lossless_height_for_2p7_m_s_mm": (5 / 6) * 2.7**2 / 9.80665 * 1000,
            "recommended_initial_vertical_drop_mm": 700,
            "example_board_length_mm": 1200,
            "example_angle_deg": math.degrees(math.asin(0.700 / 1.200)),
            "method": "Measure speed over the last 300 mm by video; adjust release height rather than treating the energy estimate as calibration."
        },
    }
    out = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/p248_basket_static.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
