#!/usr/bin/env python3
"""P2-70 rear-pod packaging, turn-envelope, CG and cable-length screen.

Analytical proposal only.  It deliberately does not generate manufacturing
geometry, firmware or simulation inputs.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

REAR_AXLE_X_MM = -65.0
FRONT_AXLE_X_MM = 300.0
ICR_X_MM = 117.0
FRONT_DATUM_X_MM = 542.0
TURN_RADIUS_MM = FRONT_DATUM_X_MM - ICR_X_MM
POD_VERTICES_MM = [
    (-125.0, -167.0), (-267.0, -167.0), (-272.0, -162.0),
    (-272.0, 162.0), (-267.0, 167.0), (-125.0, 167.0),
]


def axle_solution(parts: list[dict]) -> dict:
    mass = sum(p["mass_kg"] for p in parts)
    cg_x = sum(p["mass_kg"] * p["x_m"] for p in parts) / mass
    front = (cg_x - REAR_AXLE_X_MM / 1000) / (
        (FRONT_AXLE_X_MM - REAR_AXLE_X_MM) / 1000
    )
    return {
        "mass_kg": mass,
        "cg_x_mm": 1000 * cg_x,
        "front_axle_percent": 100 * front,
        "rear_axle_percent": 100 * (1 - front),
    }


def path_length_mm(points: list[tuple[float, float, float]], service_loop_mm: float) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:])) + service_loop_mm


def cable(name: str, points: list[tuple[float, float, float]], loop: float) -> dict:
    routed = path_length_mm(points, loop)
    cut = math.ceil(routed / 100.0) * 100.0
    return {
        "name": name,
        "routed_estimate_mm": round(routed, 1),
        "recommended_cut_length_mm": cut,
        "includes_service_loop_mm": loop,
        "status": "MEASUREMENT USER before harness cut",
    }


def main() -> None:
    radii = [
        {"x_mm": x, "y_mm": y, "radius_mm": math.hypot(x - ICR_X_MM, y)}
        for x, y in POD_VERTICES_MM
    ]
    worst = max(radii, key=lambda item: item["radius_mm"])
    y_at_minus_300 = math.sqrt(max(0.0, TURN_RADIUS_MM**2 - (-300 - ICR_X_MM) ** 2))

    baseline_parts = [
        {"name": "bare_mobile_base", "mass_kg": 10.0, "x_m": 0.100},
        {"name": "shared_flywheel_module_design", "mass_kg": 4.706, "x_m": 0.427},
        {"name": "basket_and_supports_45_ball_design", "mass_kg": 1.700, "x_m": 0.120},
        {"name": "two_Parkside_X20V_4Ah", "mass_kg": 1.300, "x_m": -0.165},
        {"name": "UGREEN_12000mAh_100W", "mass_kg": 0.309, "x_m": -0.100},
        {"name": "Actuonix_P16", "mass_kg": 0.095, "x_m": 0.320},
    ]
    relocated_parts = [p for p in baseline_parts if p["name"] not in {
        "two_Parkside_X20V_4Ah", "UGREEN_12000mAh_100W"
    }]
    relocated_parts += [
        {"name": "two_Parkside_X20V_4Ah_relocated", "mass_kg": 1.300, "x_m": -0.1985},
        {"name": "UGREEN_relocated", "mass_kg": 0.309, "x_m": -0.195},
        {
            "name": "rear_pod_printed_group_proxy_FROM_PRIOR_CAD_NOT_THIS_GEOMETRY",
            "mass_kg": 0.770,
            "x_m": -0.1985,
        },
    ]
    nominal_unmeasured = [
        {"name": "Pi5_case_heatsink", "mass_kg": 0.200, "x_m": -0.192},
        {"name": "Mega_holder_board", "mass_kg": 0.060, "x_m": -0.1985},
        {"name": "perfboard_harness", "mass_kg": 0.150, "x_m": -0.1985},
        {"name": "four_BTS7960", "mass_kg": 0.200, "x_m": -0.1985},
        {"name": "fuse_relay_dc_dc_ideal_diodes", "mass_kg": 0.300, "x_m": -0.215},
        {"name": "top_controls", "mass_kg": 0.100, "x_m": -0.153},
    ]

    cables = [
        cable("front FIT0403 left", [(-145, 0, 180), (-155, 180, 90), (300, 180, 85), (300, 166, 85)], 150),
        cable("front FIT0403 right", [(-145, 0, 180), (-155, -180, 90), (300, -180, 85), (300, -166, 85)], 150),
        cable("rear FIT0403 left", [(-145, 0, 180), (-155, 180, 90), (-65, 180, 85), (-65, 166, 85)], 100),
        cable("rear FIT0403 right", [(-145, 0, 180), (-155, -180, 90), (-65, -180, 85), (-65, -166, 85)], 100),
        cable("LiDAR", [(-145, 0, 258), (-125, 180, 260), (390, 180, 354), (390, 0, 374)], 150),
        cable("camera", [(-145, 0, 258), (-125, 180, 260), (390, 180, 305), (410, 0, 305)], 150),
        cable("FIT0186 actuator", [(-145, -50, 180), (-125, -180, 180), (310, -180, 180), (310, -164.5, 180)], 250),
        cable("D5065 controller left", [(-145, 0, 180), (-125, 145, 200), (80, 145, 245)], 150),
        cable("D5065 controller right", [(-145, 0, 180), (-125, -145, 200), (80, -145, 245)], 150),
    ]

    output = {
        "status": "P2_70_LAYOUT_PROPOSAL_ONLY",
        "turn_envelope": {
            "icr_x_mm": ICR_X_MM,
            "existing_front_datum_x_mm": FRONT_DATUM_X_MM,
            "existing_radius_mm": TURN_RADIUS_MM,
            "pod_vertices_mm": POD_VERTICES_MM,
            "worst_pod_vertex": worst,
            "radial_margin_mm": TURN_RADIUS_MM - worst["radius_mm"],
            "allowed_half_width_at_x_minus_300_mm": y_at_minus_300,
            "conclusion": "X=-300 is available only inside about Y +/-82 mm; not for the full-width pod",
        },
        "clearance_screen_mm": {
            "pod_envelope": {"x": [-272, -125], "y": [-167, 167], "z": [55, 350], "highest_control_z": 374},
            "tilted_backboard_to_front_face_min": 6.3,
            "basket_side_grid_to_pod_side": 5.0,
            "rear_tyre_inner_face_to_pod_side": 88.5,
            "rear_FIT0403_rearmost_to_pod_front": 40.0,
            "chassis_crossbar_top_to_pod_body_bottom": 3.0,
            "lidar_scan_plane_to_highest_control": 16.0,
            "note": "mount feet intentionally mate with the rear-crossbar M4 grid and are excluded from body collision probes",
        },
        "mass_balance": {
            "current_repo_baseline": axle_solution(baseline_parts),
            "relocated_power_plus_770g_prior_CAD_shell_proxy": axle_solution(relocated_parts),
            "nominal_if_all_unweighed_electronics_are_additional": axle_solution(relocated_parts + nominal_unmeasured),
            "unweighed_component_assumptions_REQUIRE_USER_MEASUREMENT": nominal_unmeasured,
            "sensitivity_front_axle_percent_per_added_0p5kg_at_x_minus185": [
                {
                    "additional_unweighed_mass_kg": added,
                    **axle_solution(relocated_parts + ([{"name": "unweighed", "mass_kg": added, "x_m": -0.185}] if added else [])),
                }
                for added in (0.0, 0.5, 1.0, 1.5)
            ],
        },
        "cables": cables,
        "assumptions": [
            "No new Gazebo run; geometry and routing use existing P2-67/P2-68/P2-69 CAD data",
            "Parkside clear reservation is 141x91x81 mm; final rail/latch is blocked on purchase and user measurement",
            "BTS7960, fuse, relay, DC-DC, ideal-diode, connector and LED bodies require user measurement",
            "Cable cut lengths are rounded up to the next 100 mm and must be remeasured on the physical route",
            "MPU6050 remains on the chassis near the centre and is not included in the rear pod",
        ],
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2/p270_rear_pod.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
