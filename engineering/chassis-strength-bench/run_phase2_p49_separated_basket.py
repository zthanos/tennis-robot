#!/usr/bin/env python3
"""P2-49 static sizing for separated collection ramp and launch-feed channel."""
from __future__ import annotations

import json
import math
from pathlib import Path

BALL_D_MM = 66.0
BALL_R_MM = BALL_D_MM / 2
BALL_VOLUME_L = 4 * math.pi * (BALL_R_MM / 1000) ** 3 / 3 * 1000
PACKING = (0.56, 0.60, 0.64)
RIM_Z_MM = 360.0
LIDAR_Z_MM = 390.0
HIGH_SILL_Z_MM = 285.0
FLOOR_LONG_DEG = 4.5
FLOOR_CROSS_DEG = 2.0

# Concave storage outline.  The central front notch is outside the basket and
# contains the collection ramp plus the launch feed channel underneath it.
STORAGE_POLYGON_MM = [
    (-15, -172), (278, -172), (278, -50), (145, -50),
    (100, -40), (100, 40), (145, 50), (278, 50),
    (278, 172), (-15, 172),
]


def inside_polygon(x: float, y: float) -> bool:
    inside = False
    j = len(STORAGE_POLYGON_MM) - 1
    for i, (xi, yi) in enumerate(STORAGE_POLYGON_MM):
        xj, yj = STORAGE_POLYGON_MM[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def floor_z_mm(x: float, y: float) -> float:
    """Piecewise funnel: rear field and both forward wings drain to X100/Y0."""
    tx = math.tan(math.radians(FLOOR_LONG_DEG))
    ty = math.tan(math.radians(FLOOR_CROSS_DEG))
    if x <= 100:
        return 42 + tx * (100 - x) + ty * abs(y)
    return 42 + tx * (x - 100) + ty * max(0.0, abs(y) - 40)


def integrate_storage(step_mm: float = 1.0) -> dict:
    area_mm2 = 0.0
    gross_mm3 = 0.0
    full_rectangle_mm3 = 0.0
    floor_min = 1e9
    floor_max = -1e9
    x = -14.5
    while x < 278:
        y = -171.5
        while y < 172:
            full_rectangle_mm3 += (RIM_Z_MM - floor_z_mm(x, y)) * step_mm**2
            if inside_polygon(x, y):
                z = floor_z_mm(x, y)
                area_mm2 += step_mm**2
                gross_mm3 += (RIM_Z_MM - z) * step_mm**2
                floor_min = min(floor_min, z)
                floor_max = max(floor_max, z)
            y += step_mm
        x += step_mm

    excluded = {
        "panel_ribs_seams_and_fastener_backs_l": 0.55,
        "two_smooth_notch_corner_fairings_l": 0.25,
        "low_feed_throat_gate_keepout_l": 0.35,
    }
    excluded_total_l = sum(excluded.values())
    gross_l = gross_mm3 / 1e6
    full_rectangle_l = full_rectangle_mm3 / 1e6
    net_l = gross_l - excluded_total_l
    return {
        "storage_plan_area_mm2": area_mm2,
        "floor_z_range_mm": [floor_min, floor_max],
        "gross_storage_below_rim_l": gross_l,
        "full_rectangle_reference_l": full_rectangle_l,
        "external_ramp_channel_bay_removed_from_storage_l": full_rectangle_l - gross_l,
        "explicit_internal_displacements_l": excluded,
        "net_clean_storage_l": net_l,
        "capacity_by_packing": {
            str(p): net_l * p / BALL_VOLUME_L for p in PACKING
        },
    }


def pile(count: int, area_mm2: float, floor_max_mm: float) -> dict:
    # Remove an additional projected 3500 mm2 for the two fairings and gate
    # neighborhood so the local pile-height screen is more conservative than
    # using the net volume alone.
    effective_area_m2 = (area_mm2 - 3500.0) / 1e6
    rows = []
    for packing in PACKING:
        mean_mm = count * BALL_VOLUME_L / 1000 / (packing * effective_area_m2) * 1000
        top = floor_max_mm + mean_mm + BALL_D_MM
        rows.append({
            "packing_fraction": packing,
            "mean_height_mm": mean_mm,
            "conservative_local_top_z_mm": top,
            "high_sill_margin_mm": HIGH_SILL_Z_MM - top,
        })
    return {
        "count": count,
        "solid_ball_volume_l": count * BALL_VOLUME_L,
        "effective_plan_area_mm2": effective_area_m2 * 1e6,
        "packing_sweep": rows,
        "worst_local_top_z_mm": max(row["conservative_local_top_z_mm"] for row in rows),
        "minimum_high_sill_margin_mm": min(row["high_sill_margin_mm"] for row in rows),
    }


def route_screen() -> dict:
    pivot = (387.0, 49.837363636)
    local_mouth = (-62.0, -8.0)

    def mouth(angle_deg: float) -> dict:
        a = math.radians(angle_deg)
        x = pivot[0] + local_mouth[0] * math.cos(a) - local_mouth[1] * math.sin(a)
        floor_z = pivot[1] + local_mouth[0] * math.sin(a) + local_mouth[1] * math.cos(a)
        return {"angle_deg": angle_deg, "floor_xyz_mm": [x, 0, floor_z], "ball_center_z_mm": floor_z + BALL_R_MM}

    collection_energy_rise_m = (HIGH_SILL_Z_MM - mouth(-3)["floor_xyz_mm"][2]) / 1000
    lossless = math.sqrt(2 * 9.80665 * collection_energy_rise_m)
    return {
        "collection_ramp_transition_radius_mm": 125,
        "collection_ramp_continuous_side_rails": True,
        "collection_ramp_side_rail_height_mm": 20,
        "collection_ramp_clear_width_mm": 80,
        "collection_ramp_surface_points_xz_mm": [
            [402, 10], [384.603, 11.216], [367.545, 14.842],
            [351.158, 20.807], [335.760, 28.994], [321.652, 39.244],
            [309.107, 51.359], [270, 95.10], [220, 151.05],
            [170, 207.00], [120, 262.95], [100, HIGH_SILL_Z_MM],
        ],
        "launch_feed_surface_points_xz_mm": [
            [100, 42], [180, 42], [225, 40], [250, 38],
        ],
        "selector_envelope_xz_mm": [225, 335, 32, 130],
        "mouths": {
            "collection_minus3": mouth(-3),
            "launch_plus15": mouth(15),
            "launch_plus20": mouth(20),
        },
        "collection_lossless_speed_m_s": lossless,
        "collection_speed_with_25_percent_margin_m_s": 1.25 * lossless,
        "d200_rpm_with_25_percent_margin": 1.25 * lossless / (math.pi * 0.2) * 60,
        "switch_states": {
            "collect": "upper collection branch open; low feed shutter closed; selector connects common mouth throat to upper ramp",
            "transport": "upper and low branch shutters closed; launcher stopped",
            "launch": "upper collection shutter closed; low feed branch open; selector connects floor channel to common mouth throat",
        },
    }


def packaging() -> dict:
    return {
        "basket_rear_x_mm": -15,
        "reason_no_rear_extension": "Clean capacity already exceeds 45; retain measured 32 mm rear-drive clearance and external battery access.",
        "rear_fit0403_front_x_mm": -47,
        "rear_drive_clearance_mm": 32,
        "ugreen_envelope_x_mm": [-157.5, -42.5],
        "ugreen_to_basket_x_gap_mm": 27.5,
        "parkside_forward_edge_x_mm": -97.5,
        "parkside_to_basket_x_gap_mm": 82.5,
        "rim_to_lidar_scan_clearance_mm": LIDAR_Z_MM - RIM_Z_MM,
        "camera": "Camera at X350 is forward of basket X278; central notch preserves the collection ground view. Optical FOV remains a physical validation item.",
    }


def main() -> None:
    storage = integrate_storage()
    p40 = pile(40, storage["storage_plan_area_mm2"], storage["floor_z_range_mm"][1])
    p45 = pile(45, storage["storage_plan_area_mm2"], storage["floor_z_range_mm"][1])
    result = {
        "status": "P2_49_STATIC_CAD_SEPARATED_PATHS_PASS_PENDING_PHYSICAL_TEST_AND_5D_GATE_MECHANISM",
        "supersedes": "P2-48 ramp-inside-basket functional concept",
        "geometry_mm": {
            "storage_polygon_xy": STORAGE_POLYGON_MM,
            "central_external_bay_clear_width": 80,
            "central_external_bay_outer_width": 100,
            "divider_x": 100,
            "side_wing_front_x": 278,
            "rear_x": -15,
            "rim_z": RIM_Z_MM,
            "high_collection_sill_z": HIGH_SILL_Z_MM,
            "lidar_scan_z": LIDAR_Z_MM,
            "floor_longitudinal_slope_deg": FLOOR_LONG_DEG,
            "floor_cross_slope_deg": FLOOR_CROSS_DEG,
        },
        "storage": storage,
        "pile_40": p40,
        "pile_45": p45,
        "mandatory_40_high_entry_pass": p40["minimum_high_sill_margin_mm"] >= 20,
        "target_45_is_best_effort": True,
        "routes_and_selector": route_screen(),
        "packaging": packaging(),
        "limitations": [
            "Static geometry and energy screen only; no Gazebo.",
            "Gate/diverter actuation, timing, sensing and ball singulation remain Phase 5d.",
            "Packing and pile shape require the printed-basket physical test.",
            "No STL release; all P2-49 panels remain dependent.",
        ],
    }
    out = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/p249_separated_basket_static.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
