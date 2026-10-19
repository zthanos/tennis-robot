#!/usr/bin/env python3
"""P2-39 static pivot-X/Z and operating-position sweep; no ROS/Gazebo."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

WHEEL_R = 100.0
TREAD_W = 60.0
EDGE_R = 2.0
BALL_Z = 33.0
USABLE_MARGIN = 8.0
TARGET_EDGE = BALL_Z - EDGE_R - USABLE_MARGIN
ALLOWANCE = 5.0
MOVING_REQUIRED_AFTER = 12.0
LAUNCH_STATIC_REQUIRED = 8.0
NOMINAL_PIVOT_Z = 64.3
AXLE_DATUM = 52.0
COLLECTION_ANGLE = -3.0


def physical_lower_edge(pivot_ground: float, offset: float, angle_deg: float) -> float:
    a = math.radians(angle_deg)
    return pivot_ground + offset * math.sin(a) - TREAD_W / 2 * math.cos(a)


def wheel_clearance(pivot_ground: float, offset: float, angle_deg: float) -> float:
    a = math.radians(angle_deg)
    return physical_lower_edge(pivot_ground, offset, angle_deg) - WHEEL_R * abs(math.sin(a))


def pivot_ground_for_collection(offset: float, collection_angle: float = COLLECTION_ANGLE) -> float:
    a = math.radians(collection_angle)
    return TARGET_EDGE - offset * math.sin(a) + TREAD_W / 2 * math.cos(a)


def offset_row(offset: float) -> dict:
    pivot_ground = pivot_ground_for_collection(offset)
    downs = [NOMINAL_PIVOT_Z + r - h - AXLE_DATUM - pivot_ground
             for r in (82.0, 85.0, 90.0) for h in (26.0, 29.0, 33.0)]
    collection_raw = wheel_clearance(pivot_ground, offset, COLLECTION_ANGLE)
    transports = {str(a): wheel_clearance(pivot_ground, offset, a) for a in range(0, 6)}
    launches = {str(a): wheel_clearance(pivot_ground, offset, a) for a in range(15, 21)}
    cad_clear = offset <= 73.0  # exact OpenSCAD scan: first arch intersection at 74 mm
    return {
        "pivot_behind_nip_mm": offset,
        "pivot_x_at_transport_mm": 442.0 - offset,
        "pivot_ground_z_when_adjusted_mm": pivot_ground,
        "pivot_down_range_over_R_h_mm": [min(downs), max(downs)],
        "pivot_down_reference_R85_h29_mm": NOMINAL_PIVOT_Z + 4.0 - pivot_ground,
        "collection_angle_deg": COLLECTION_ANGLE,
        "collection_raw_clearance_mm": collection_raw,
        "collection_after_allowance_mm": collection_raw - ALLOWANCE,
        "transport_clearance_raw_mm_by_angle": transports,
        "best_transport_angle_deg": max(transports, key=transports.get),
        "best_transport_after_allowance_mm": max(transports.values()) - ALLOWANCE,
        "transport_plus5_after_allowance_mm": transports["5"] - ALLOWANCE,
        "launch_static_clearance_mm_by_angle": launches,
        "launch_plus20_static_clearance_mm": launches["20"],
        "launch_plus15_static_clearance_mm": launches["15"],
        "launch_15_vs_20_clearance_gain_mm": launches["15"] - launches["20"],
        "passes_collection": collection_raw - ALLOWANCE >= MOVING_REQUIRED_AFTER,
        "passes_transport_0_to_5": min(transports.values()) - ALLOWANCE >= MOVING_REQUIRED_AFTER,
        "passes_launch_15_to_20": min(launches.values()) >= LAUNCH_STATIC_REQUIRED,
        "cad_bearing_nominal_collision_free": cad_clear,
        "cad_collision_note": "OpenSCAD pivot-running-envelope intersection is empty through 73 mm; first arch intersection at 74 mm" if cad_clear else "nominal bearing/guard intersection with X330 arch",
        "passes_all_static_and_cad": collection_raw - ALLOWANCE >= MOVING_REQUIRED_AFTER and min(transports.values()) - ALLOWANCE >= MOVING_REQUIRED_AFTER and min(launches.values()) >= LAUNCH_STATIC_REQUIRED and cad_clear and max(downs) <= 23.0,
    }


def delivery(selected: dict) -> dict:
    offset = selected["pivot_behind_nip_mm"]
    pivot_ground = selected["pivot_ground_z_when_adjusted_mm"]
    # The existing rear throat is kept 105 mm behind the nip.
    handoff = pivot_ground + (offset - 105.0) * math.sin(math.radians(COLLECTION_ANGLE))
    guide_entry = [78.0 + r - h - AXLE_DATUM for r in (82.0, 85.0, 90.0) for h in (26.0, 29.0, 33.0)]
    guide_peak = [136.0 + r - h - AXLE_DATUM for r in (82.0, 85.0, 90.0) for h in (26.0, 29.0, 33.0)]
    lifts = [(z - handoff) / 1000 for z in guide_peak]
    required_50 = [math.sqrt(2 * 9.81 * lift / 0.5) for lift in lifts]
    worst = max(required_50)
    speeds = []
    for v in (2.0, 2.25, 2.35, 2.5):
        speeds.append({
            "surface_speed_mps": v,
            "rpm_d200": v / (2 * math.pi * 0.1) * 60,
            "speed_margin_over_worst_min_percent": 100 * (v / worst - 1),
            "ball_kinetic_energy_j": 0.5 * 0.057 * v * v,
            "energy_available_at_50_percent_j": 0.5 * 0.5 * 0.057 * v * v,
            "meets_25_percent_speed_margin": v >= 1.25 * worst,
        })
    return {
        "handoff_ball_center_ground_mm": handoff,
        "existing_guide_entry_ground_range_mm": [min(guide_entry), max(guide_entry)],
        "entry_step_up_range_mm": [min(guide_entry) - handoff, max(guide_entry) - handoff],
        "recommended_entry_lowering_mm": "reshape the first two guide stations downward by about 20..25 mm and make the final value shim-adjustable after R/h measurement",
        "guide_peak_ground_range_mm": [min(guide_peak), max(guide_peak)],
        "lift_to_peak_range_mm": [1000 * min(lifts), 1000 * max(lifts)],
        "minimum_speed_at_50_percent_transfer_range_mps": [min(required_50), worst],
        "speed_for_25_percent_margin_worst_case_mps": 1.25 * worst,
        "speed_for_25_percent_margin_worst_case_rpm": 1.25 * worst / (2 * math.pi * 0.1) * 60,
        "speed_cases": speeds,
        "rebound_note": "2.35..2.50 m/s closes the energy margin but increases residual rebound energy; retain contained roof, compliant rear deflector and anti-return flap, then validate in 5c",
    }


def actuator_spacing(selected: dict) -> dict:
    pivot = (selected["pivot_x_at_transport_mm"], 0.0,
             selected["pivot_ground_z_when_adjusted_mm"] - 4.0)
    fixed = (320.0, -190.0, 120.0)
    local = (50.0, -190.0, 90.0)
    values = {}
    for angle in (-3.0, 0.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0):
        a = math.radians(angle)
        moving = (pivot[0] + math.cos(a) * local[0] - math.sin(a) * local[2],
                  local[1],
                  pivot[2] + math.sin(a) * local[0] + math.cos(a) * local[2])
        values[str(int(angle))] = math.dist(fixed, moving)
    return {"pin_spacing_mm_by_angle": values,
            "required_spacing_change_minus3_to_plus20_mm": values["-3"] - values["20"],
            "note": "33.4 mm travel fits inside the prior 40 mm stroke screen, but the 89.7..123.1 mm absolute installed length requires P16/bracket drawing verification"}


def main() -> None:
    rows = [offset_row(float(x)) for x in range(45, 111, 5)]
    selected = next(x for x in rows if x["pivot_behind_nip_mm"] == 55.0)
    report = {
        "status": "STATIC_CAD_SOLUTION_FOUND_WITH_STATIONARY_LAUNCH_RULE",
        "operating_rule": {
            "collection": "robot moving; -3 deg; apply 5 mm allowance",
            "transport": "robot moving slowly; 0 deg preferred, +5 deg still passes; apply 5 mm allowance",
            "launch": "robot stationary; +15..+20 deg; require raw static clearance >=8 mm; no 5 mm motion allowance",
        },
        "geometry": {"tread": "printed TPU 60 mm/r2", "stop_adjustment_deg": [-10, -3], "pivot_offset_sweep_mm": [45, 110], "pivot_z_adjustment_travel_mm": 23},
        "rows": rows,
        "selected_static_candidate": selected,
        "cad_collision_sweep": {
            "offsets_empty_mm": [45, 50, 55, 60, 65, 70, 71, 72, 73],
            "first_collision_mm": 74,
            "collision_component": "bearing/guard running envelope vs X330 front arch",
            "other_checked_components": ["drive tyres/mounts/motors", "frame beams", "basket", "fixed guide"],
            "selected_offset_parametric_margin_to_first_collision_mm": 19,
            "limitation": "nominal exact-solid screen, not tolerance/runout proof",
        },
        "moving_module_arch_sweep": {
            "selected_offset_mm": 55,
            "selected_angle_range_deg": [-3, 20],
            "selected_full_sweep_result": "empty",
            "offset_sweep_at_plus20": {"empty_through_mm": 57, "first_collision_mm": 58},
            "selected_offset_parametric_margin_to_first_collision_mm": 3,
            "other_selected_full_sweep_intersections": "frame, drive, basket/battery, fixed guide, actuator and low deck are empty",
        },
        "launch_angle_trade": {
            "plus15_clearance_mm": selected["launch_plus15_static_clearance_mm"],
            "plus16_clearance_mm": selected["launch_static_clearance_mm_by_angle"]["16"],
            "plus20_clearance_mm": selected["launch_plus20_static_clearance_mm"],
            "clearance_gain_plus15_vs_plus20_mm": selected["launch_15_vs_20_clearance_gain_mm"],
            "cad_result": "selected 55 mm offset is collision-free through +20 deg; at +20 the first arch intersection appears when offset reaches 58 mm",
            "ballistic_note": "+15..+20 deg trades static ground clearance against trajectory elevation; range, apex and net clearance depend on exit speed and will be evaluated in 5d",
        },
        "four_bar_gate": "NOT_TRIGGERED: the 55 mm pivot offset passes collection, transport and stationary launch through +20 deg; retain a four-bar only as fallback if measured envelopes consume the margin",
        "delivery": delivery(selected),
        "actuator_geometry": actuator_spacing(selected),
        "recommendation": "Use 55 mm pivot-behind-nip, -3 deg collection, 0 deg transport and adjustable +15..+20 deg stationary launch. Keep 23 mm shims/slots and do not release STL until loaded R, tread profile and physical clearances are measured.",
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2"
    out.mkdir(parents=True, exist_ok=True)
    (out / "pivot_operating_positions.json").write_text(json.dumps(report, indent=2) + "\n")
    with (out / "pivot_operating_positions.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
