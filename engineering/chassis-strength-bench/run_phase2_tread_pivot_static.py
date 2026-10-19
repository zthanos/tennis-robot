#!/usr/bin/env python3
"""P2-38 static tread/pivot/angle screen; no ROS or Gazebo."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path


BALL_Z_MM = 33.0
BALL_MASS_KG = 0.057
BALL_EDGE_MARGIN_MM = 8.0
FLYWHEEL_RADIUS_MM = 100.0
NIP_U_MM = 45.0
PIVOT_Z_CHASSIS_MM = 64.3
AXLE_DATUM_MM = 52.0
DYNAMIC_ALLOWANCE_MM = 5.0
REQUIRED_AFTER_ALLOWANCE_MM = 12.0
REQUIRED_RAW_MM = REQUIRED_AFTER_ALLOWANCE_MM + DYNAMIC_ALLOWANCE_MM
ACCEPTED_STOP_RANGE_DEG = (-10.0, -4.5)
LAUNCH_ANGLE_DEG = 20.0
TPU_DENSITY_KG_M3 = 1200.0
BASE_PRINTED_MASS_KG = 0.8375985883443636
BASE_PRINTED_POLAR_INERTIA = 0.004692250767087274
BASE_TREAD_WIDTH_MM = 46.0
TREAD_RO_MM = 100.0
TREAD_RI_MM = 96.0


def tread_delta(width_mm: float) -> tuple[float, float]:
    delta_width_m = (width_mm - BASE_TREAD_WIDTH_MM) / 1000.0
    delta_mass = math.pi * ((TREAD_RO_MM / 1000) ** 2 - (TREAD_RI_MM / 1000) ** 2) * delta_width_m * TPU_DENSITY_KG_M3
    delta_inertia = delta_mass * ((TREAD_RO_MM / 1000) ** 2 + (TREAD_RI_MM / 1000) ** 2) / 2
    return delta_mass, delta_inertia


def lower_edge(pivot_ground: float, width: float, angle_deg: float) -> float:
    angle = math.radians(angle_deg)
    return pivot_ground + NIP_U_MM * math.sin(angle) - width / 2 * math.cos(angle)


def wheel_clearance(pivot_ground: float, width: float, angle_deg: float) -> float:
    angle = math.radians(angle_deg)
    return lower_edge(pivot_ground, width, angle_deg) - FLYWHEEL_RADIUS_MM * abs(math.sin(angle))


def profile_row(width: float, edge_round: float, label: str, manufactured: str) -> dict:
    # When the contact criterion is exactly met, the physical lower edge is
    # BALL_Z-edge_round-margin.  The ground-clearance feasibility bound is
    # therefore independent of tread width.
    target_physical_edge = BALL_Z_MM - edge_round - BALL_EDGE_MARGIN_MM
    sine_budget = (target_physical_edge - REQUIRED_RAW_MM) / FLYWHEEL_RADIUS_MM
    feasible_angle = -math.degrees(math.asin(max(-1.0, min(1.0, sine_budget)))) if sine_budget >= 0 else None
    within_stop = feasible_angle is not None and ACCEPTED_STOP_RANGE_DEG[0] <= feasible_angle <= ACCEPTED_STOP_RANGE_DEG[1]
    chosen_angle = feasible_angle if feasible_angle is not None else ACCEPTED_STOP_RANGE_DEG[1]
    downs = []
    launch_clearances = []
    nip_heights = []
    for radius in (82.0, 85.0, 90.0):
        for h in (26.0, 29.0, 33.0):
            pivot_ground = PIVOT_Z_CHASSIS_MM + radius - h - AXLE_DATUM_MM
            down = max(0.0, lower_edge(pivot_ground, width, chosen_angle) - target_physical_edge)
            adjusted_pivot = pivot_ground - down
            downs.append(down)
            nip_heights.append(adjusted_pivot + NIP_U_MM * math.sin(math.radians(chosen_angle)))
            launch_clearances.append(wheel_clearance(adjusted_pivot, width, LAUNCH_ANGLE_DEG))
    dm, di = tread_delta(width)
    angle_at_stop = ACCEPTED_STOP_RANGE_DEG[1]
    clearance_if_contact_at_stop = target_physical_edge - FLYWHEEL_RADIUS_MM * abs(math.sin(math.radians(angle_at_stop)))
    return {
        "profile": label,
        "manufacturing_status": manufactured,
        "width_mm": width,
        "edge_rounding_or_unusable_inset_mm": edge_round,
        "wheel_mass_kg_each": BASE_PRINTED_MASS_KG + dm,
        "mass_delta_from_46mm_model_kg_each": dm,
        "polar_inertia_kg_m2_each": BASE_PRINTED_POLAR_INERTIA + di,
        "polar_inertia_delta_percent": 100 * di / BASE_PRINTED_POLAR_INERTIA,
        "nip_gap_change_mm": 0.0,
        "widens_each_axial_side_vs_50mm_mm": (width - 50.0) / 2,
        "angle_needed_for_both_intake_criteria_deg": feasible_angle,
        "inside_accepted_minus10_to_minus4p5_stop": within_stop,
        "pivot_down_range_at_needed_angle_mm": [min(downs), max(downs)],
        "nip_height_range_at_needed_angle_mm": [min(nip_heights), max(nip_heights)],
        "intake_raw_clearance_at_needed_angle_mm": REQUIRED_RAW_MM if feasible_angle is not None else None,
        "intake_clearance_after_allowance_mm": REQUIRED_AFTER_ALLOWANCE_MM if feasible_angle is not None else None,
        "raw_clearance_if_contact_met_at_minus4p5_mm": clearance_if_contact_at_stop,
        "margin_to_required_raw_at_minus4p5_mm": clearance_if_contact_at_stop - REQUIRED_RAW_MM,
        "launch_plus20_raw_clearance_range_mm": [min(launch_clearances), max(launch_clearances)],
        "launch_plus20_after_allowance_range_mm": [min(launch_clearances) - DYNAMIC_ALLOWANCE_MM, max(launch_clearances) - DYNAMIC_ALLOWANCE_MM],
        "passes_intake_all_grid": feasible_angle is not None,
        "passes_intake_with_accepted_stop": within_stop,
        "passes_launch_plus20_all_grid": min(launch_clearances) - DYNAMIC_ALLOWANCE_MM >= REQUIRED_AFTER_ALLOWANCE_MM,
    }


def delivery(selected: dict) -> dict:
    angle_deg = selected["angle_needed_for_both_intake_criteria_deg"]
    down_reference = selected["pivot_down_range_at_needed_angle_mm"][0] + 7.0  # R85/h29 is 7 mm above lowest-shift corner.
    pivot_ground_reference = PIVOT_Z_CHASSIS_MM + 85 - 29 - AXLE_DATUM_MM - down_reference
    handoff_ground = pivot_ground_reference + (-60.0) * math.sin(math.radians(angle_deg))
    guide_peak_ground = [136.0 + (r - h - AXLE_DATUM_MM) for r in (82.0, 85.0, 90.0) for h in (26.0, 29.0, 33.0)]
    lifts = [(z - handoff_ground) / 1000 for z in guide_peak_ground]
    ballistic = []
    rearward_up_angle = abs(math.radians(angle_deg))
    for speed in (0.6, 0.9, 1.2):
        t = 0.080 / (speed * math.cos(rearward_up_angle))
        dz = speed * math.sin(rearward_up_angle) * t - 0.5 * 9.81 * t * t
        ballistic.append({"speed_mps": speed, "vertical_change_over_80mm_mm": 1000 * dz})
    min_lift, max_lift = min(lifts), max(lifts)
    return {
        "selected_intake_angle_deg": angle_deg,
        "reference_handoff_ball_center_ground_mm": handoff_ground,
        "existing_fixed_guide_peak_ground_range_mm": [min(guide_peak_ground), max(guide_peak_ground)],
        "required_peak_lift_range_mm": [1000 * min_lift, 1000 * max_lift],
        "ball_potential_energy_range_j": [BALL_MASS_KG * 9.81 * min_lift, BALL_MASS_KG * 9.81 * max_lift],
        "minimum_speed_ideal_range_mps": [math.sqrt(2 * 9.81 * min_lift), math.sqrt(2 * 9.81 * max_lift)],
        "minimum_speed_at_50_percent_transfer_range_mps": [math.sqrt(2 * 9.81 * min_lift / 0.5), math.sqrt(2 * 9.81 * max_lift / 0.5)],
        "ballistic_free_flight_over_80mm": ballistic,
        "decision": "1.9 m/s only barely clears the worst 50%-transfer energy screen; reshape/lower the fixed guide entrance and retain containment before dynamic release",
    }


def main() -> None:
    profiles = []
    for width in (50.0, 55.0, 60.0):
        for edge in (2.0, 3.0):
            profiles.append(profile_row(width, edge, f"printed_flat_w{int(width)}_r{int(edge)}", "printed TPU candidate"))
    pneumatic = profile_row(50.0, 8.0, "pneumatic_rounded_screen_r8", "purchased tyre; edge profile and loaded width unmeasured")
    pneumatic.update({"wheel_mass_kg_each": None, "mass_delta_from_46mm_model_kg_each": None,
                      "polar_inertia_kg_m2_each": None, "polar_inertia_delta_percent": None})
    profiles.append(pneumatic)
    selected = next(x for x in profiles if x["profile"] == "printed_flat_w60_r2")
    report = {
        "status": "NO_FULL_ARC_SOLUTION_WITH_FIXED_PIVOT_AND_PLUS20_LAUNCH",
        "criteria": {
            "R_mm": [82, 90], "h_mm": [26, 33], "ball_center_z_mm": BALL_Z_MM,
            "ball_center_above_usable_edge_mm": BALL_EDGE_MARGIN_MM,
            "flywheel_clearance_after_5mm_allowance_mm": REQUIRED_AFTER_ALLOWANCE_MM,
            "accepted_lower_stop_deg": list(ACCEPTED_STOP_RANGE_DEG),
        },
        "profiles": profiles,
        "best_intake_candidate": selected,
        "finding": {
            "intake": "W60/R2 minimizes pivot descent, but needs -3.440 deg, outside the accepted -10..-4.5 deg stop range",
            "launch": "the W60/R2 intake-compliant lowered pivot leaves only 8.64 mm raw at +20 deg (3.64 mm after allowance), so no screened profile passes the full arc",
            "pneumatic": "rounded purchased tyre is worse and remains measurement-gated",
            "cad_policy": "reserve >=23 mm positive-lock Z adjustment; show adjustable stop as dependent CAD only; do not release STL or lock an endpoint",
        },
        "cad_collision_qa": {
            "engine": "openscad-nightly, exact solid intersections",
            "candidate": "W60/r2, pivot down 12.654 mm at R85/h29, sweep -3.4398..+20 deg with explicit +20 endpoint",
            "empty_intersections": ["moving-vs-frame", "moving-vs-drive-envelopes", "moving-vs-basket-battery", "moving-vs-high-guide", "moving-vs-arch", "actuator-vs-basket-battery-drive"],
            "moving_throat_guide_vs_low_deck": "empty at 13 angles from -3.4398 through +20 deg",
            "limitation": "nominal solid QA only; no tolerance/dynamic credit and does not override the flywheel-ground clearance failure",
            "reproduction": "/snap/bin/openscad-nightly -o /tmp/p2_38_<view>.stl -D 'view=\"qa_sweep_<view>\"' cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad",
        },
        "delivery_to_existing_guide": delivery(selected),
    }
    out_dir = Path(__file__).resolve().parent / "results/shared_flywheel_phase2"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "tread_pivot_static.json"
    csv_path = out_dir / "tread_pivot_profiles.csv"
    json_path.write_text(json.dumps(report, indent=2) + "\n")
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(profiles[0]))
        writer.writeheader(); writer.writerows(profiles)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
