#!/usr/bin/env python3
"""P2-52 static mechanical-transition and packaging screen (no ROS/Gazebo)."""
from __future__ import annotations

import json
import math
from pathlib import Path

PIVOT = (387.0, 0.0, 49.837363636 - 4.0)
FIXED_PIN = (310.0, -205.0, 120.0)
MOVING_LOCAL = (50.0, -205.0, 90.0)
ANGLES = (-10.0, -3.0, 0.0, 15.0, 20.0)
CONTINUOUS_HINGE_NM = 5.3
STOP_DESIGN_NM = 8.0


def moving_pin(angle_deg: float) -> tuple[float, float, float]:
    a = math.radians(angle_deg)
    x, y, z = MOVING_LOCAL
    return (PIVOT[0] + x * math.cos(a) - z * math.sin(a), y,
            PIVOT[2] + x * math.sin(a) + z * math.cos(a))


def linkage_row(angle_deg: float) -> dict:
    m = moving_pin(angle_deg)
    line = tuple(FIXED_PIN[i] - m[i] for i in range(3))
    length = math.sqrt(sum(v * v for v in line))
    unit = tuple(v / length for v in line)
    radius = tuple(m[i] - PIVOT[i] for i in range(3))
    cross = (radius[1] * unit[2] - radius[2] * unit[1],
             radius[2] * unit[0] - radius[0] * unit[2],
             radius[0] * unit[1] - radius[1] * unit[0])
    # The hinge axis is Y.  Only the Y component of r x F creates hinge
    # torque; using the full cross-product magnitude would incorrectly count
    # the 190 mm lateral actuator offset as lever arm.
    arm_m = abs(cross[1]) / 1000.0
    return {
        "angle_deg": angle_deg,
        "moving_pin_xyz_mm": [round(v, 3) for v in m],
        "pin_spacing_mm": round(length, 3),
        "effective_arm_mm": round(arm_m * 1000, 3),
        "axial_force_for_5p3_Nm_N": round(CONTINUOUS_HINGE_NM / arm_m, 2),
        "axial_force_for_8p0_Nm_N": round(STOP_DESIGN_NM / arm_m, 2),
    }


def screw_screen(max_required_force: float, travel_mm: float) -> dict:
    lead_m = 0.002
    rpm = 251.0
    output_speed_mm_s = rpm * 2.0 / 60.0
    rows = []
    for efficiency in (0.15, 0.25, 0.35):
        torque = max_required_force * lead_m / (2 * math.pi * efficiency)
        rows.append({"assumed_screw_efficiency": efficiency,
                     "motor_output_torque_required_Nm": round(torque, 3),
                     "fraction_of_1p77_Nm_stall": round(torque / 1.77, 3)})
    lead_angle = math.degrees(math.atan(2.0 / (math.pi * 7.0)))
    return {
        "motor": "DFRobot FIT0186 / GB37Y3530-12V-251R already owned",
        "official_or_user_inputs": {"voltage_V": 12, "no_load_rpm": rpm,
                                     "stall_torque_Nm": 1.77, "stall_current_A": 7.0,
                                     "mass_g": 205, "encoder_counts_per_output_rev": 700},
        "continuous_torque_status": "NOT SPECIFIED: do not size from stall",
        "screening_continuous_band_Nm": [0.35, 0.53],
        "screening_band_basis": "20..30% of stall is a conservative design hypothesis, not a vendor rating; verify thermally at 12 V",
        "tr8x2": {"single_start_lead_mm": 2.0, "assumed_pitch_diameter_mm": 7.0,
                   "lead_angle_deg": round(lead_angle, 2),
                   "no_load_linear_speed_mm_s": round(output_speed_mm_s, 2),
                   "travel_mm_minus3_to_plus20": round(travel_mm, 3),
                   "no_load_time_s": round(travel_mm / output_speed_mm_s, 2),
                   "loaded_time_design_range_s": [4.5, 6.0],
                   "output_revolutions": round(travel_mm / 2.0, 2),
                   "encoder_counts": round(travel_mm / 2.0 * 700),
                   "required_torque_sweep": rows,
                   "self_locking": "LIKELY only if effective thread friction coefficient exceeds tan(lead angle)=%.3f; verify chosen nut, lubrication, backlash and vibration" % math.tan(math.radians(lead_angle))},
        "mechanical_requirements": ["separate thrust bearing", "flexible coupler", "anti-rotation nut carriage",
                                    "bellows/wiper against clay", "positive upper/lower stops; screw is not the launch load path"],
    }


def packaging() -> dict:
    old_x, new_x, mass = -0.100, -0.135, 0.309
    cases = [("empty", 17.815, 0.1690), ("full45", 20.675, 0.1581)]
    cg = []
    for name, total, old_cg in cases:
        new_cg = (total * old_cg + mass * (new_x - old_x)) / total
        front_fraction = (new_cg + 0.065) / 0.365
        cg.append({"case": name, "mass_kg": total, "old_cg_x_mm": old_cg * 1000,
                   "new_cg_x_mm": round(new_cg * 1000, 3),
                   "shift_mm": round((new_cg - old_cg) * 1000, 3),
                   "front_axle_fraction_percent": round(front_fraction * 100, 3)})
    return {
        "basket_center_extension_x_mm": [-85, 278],
        "ugreen_old": {"center_xyz_mm": [-100, 0, 145], "size_xyz_mm": [115, 46, 45.5]},
        "ugreen_selected": {"center_xyz_mm": [-135, 0, 145], "size_xyz_mm": [46, 115, 45.5],
                            "orientation": "115-mm dimension transverse between Parkside packs"},
        "nominal_clearances_mm": {"basket_rear_to_ugreen_front": 27.0,
                                   "ugreen_to_each_Parkside_inner_face": 10.0,
                                   "ugreen_to_rear_electronics_front_face": 12.0},
        "parkside_access": "unchanged outboard extraction at Y sides; Ugreen removes vertically/rearward from centre bridge",
        "measurement_gate": "Parkside pack/adapter envelope remains unmeasured; require >=5 mm after physical measurement and connector bend radius",
        "cg_cases": cg,
    }


def main() -> None:
    rows = [linkage_row(a) for a in ANGLES]
    op = [r for r in rows if -3 <= r["angle_deg"] <= 20]
    travel = next(r["pin_spacing_mm"] for r in rows if r["angle_deg"] == -3) - next(r["pin_spacing_mm"] for r in rows if r["angle_deg"] == 20)
    max_f = max(r["axial_force_for_5p3_Nm_N"] for r in rows)
    result = {
        "status": "P2_52_STATIC_CAD_TRANSITION_PACKAGE_PASS_WITH_MEASUREMENT_GATES",
        "frozen_geometry": "P2-40 pivot X387, collection -3, transport 0, launch +15..+20; no change",
        "linkage": {"pivot_xyz_mm": PIVOT, "fixed_pin_xyz_mm": FIXED_PIN,
                    "moving_pin_local_xyz_mm": MOVING_LOCAL, "rows": rows,
                    "operating_travel_minus3_to_plus20_mm": round(travel, 3),
                    "max_operating_force_5p3_Nm_N": max(r["axial_force_for_5p3_Nm_N"] for r in op),
                    "max_stop_equivalent_force_8Nm_N": max(r["axial_force_for_8p0_Nm_N"] for r in rows)},
        "actuators": {
            "p16_50_256_12_S": {"decision": "compact reference / buy only if custom actuator test fails",
                "suffix": "-S = internal end-of-stroke switches, two-wire control; not position feedback",
                "three_positions": "-P is not required because AS5600 measures hinge angle; -S limits only physical stroke ends and external control stops at -3/0/+15..20",
                "rated_peak_efficiency_force_N": 150, "max_lifted_force_N": 300,
                "backdrive_force_N": 500, "mass_g": 95, "no_load_speed_mm_s": 4.8,
                "transition_no_load_s": round(travel / 4.8, 2), "nominal_40mm_time_s": round(40 / 4.8, 2),
                "installed_length_range_minus10_to_plus20_mm": [round(min(r["pin_spacing_mm"] for r in rows), 3), round(max(r["pin_spacing_mm"] for r in rows), 3)],
                "p16_50_closed_to_open_length_mm": [97, 147],
                "torque_range_at_150N_Nm": [round(min(r["effective_arm_mm"] for r in rows) * .15, 2), round(max(r["effective_arm_mm"] for r in rows) * .15, 2)],
                "note": "hold/backdrive is not the launch reaction path; launch must seat on upper stop"},
            "fit0186_tr8x2": screw_screen(max_f, travel),
            "fit0186_printed_worm": {"decision": "not baseline", "minimum_ratio_ideal_at_0p35Nm": round(5.3 / .35, 1),
                "practical_ratio_screen": "50..60:1 with assumed printed-worm efficiency 0.25..0.40",
                "estimated_hinge_torque_at_0p35Nm_60to1_eff0p30_Nm": 6.3,
                "estimated_no_load_23deg_time_s_60to1": round(23 / (251/60*360/60), 2),
                "risks": ["8 Nm tooth-root load", "PETG creep/wear and backlash", "self-lock depends on lead/friction", "large sector conflicts with arch/basket"]}},
        "selected": "FIT0186 + Tr8x2 after bench verification of continuous torque/temperature; preserve P16 clevis hole pattern as fallback",
        "sensor": {"type": "AS5600 absolute magnetic angle", "i2c_address": "0x36", "mpu6050_address": "0x68 or 0x69",
                   "mega_pins": {"SDA": 20, "SCL": 21}, "nominal_board_center_xyz_mm": [387, 216, 50],
                   "placement": "inboard +Y trunnion face, opposite actuator; board plane normal to Y",
                   "magnet": "D6x2.5 diametric magnet in non-magnetic shaft cap; 1.5 mm nominal airgap, adjustable 0.5..2.5 mm, concentricity target <=0.25 mm",
                   "levels": "one verified bus pull-up voltage only; raw AS5600 may run at 5 V. Audit MPU6050/AS5600 breakout pull-ups and level-shift if either module is 3.3-V-only"},
        "driver": {"hardware": "one existing BTS7960 channel", "preferred_pins": {"RPWM": 44, "LPWM": 45, "ENABLE": 40},
                   "alternate_pins": {"RPWM": 46, "LPWM": 11, "ENABLE": 42},
                   "supply": "12 V regulated actuator rail from Parkside 15..21 V preferred; PWM limiting alone is not a regulated 12 V supply",
                   "raw_bus_temporary_duty_caps": {"15V": 0.8, "18V": 0.667, "21V": 0.571},
                   "note": "no Arduino change in P2-52"},
        "stops": {"lower_deg": [-10, -3], "upper_deg": [15, 20], "transport_deg": 0,
                  "design_load_Nm": 8.0, "construction": "replaceable indexed stop blocks at 1-deg holes, two M6 fasteners plus bearing shoulder; slots only for calibration, not shear retention",
                  "load_path": "launch reaction -> upper stop -> arch/yoke; lower stop supports selected collection endpoint; actuator unloaded at both endpoints"},
        "selector": {"mechanism": "shaft cam + two opposed followers, lost-motion deadband and spring-centred fail-safe",
                     "states": {"-3": "collection branch OPEN; feed CLOSED", "0": "both CLOSED (transport)", "+15": "collection CLOSED; feed OPEN", "+20": "collection CLOSED; feed OPEN"},
                     "transition": "between dwells both branches closed; adjustable cam phase follows the selected lower-stop index",
                     "failure_state": "broken link/spring or power loss -> both branches closed",
                     "note": "single rear mouth retained; cam does not singulate"},
        "singulator_reserve": {"envelope_xyz_mm": [145, 0, 76], "size_xyz_mm": [80, 100, 88],
                               "concept": "two-stage escapement with two fingers 70..75 mm apart and removable blind plate",
                               "actuation": "reserve a low-force servo/solenoid interface for 5d; tilt cam selects route but cannot reliably count one ball",
                               "status": "interface only"},
        "sequence": ["command both D5065 to zero and verify rpm below threshold", "close both selector branches", "drive tilt using AS5600 closed-loop angle and timeout/current limit", "seat endpoint on indexed stop or hold 0 deg by self-locking screw", "verify angle and selector state", "only then enable collection or launch; launch additionally requires robot stationary"],
        "packaging": packaging(),
        "nominal_actuator_clearance": {
            "lateral_shift_from_p16_baseline_mm": -15.0,
            "fit0186_outer_surface_to_basket_side_mm": 14.5,
            "fit0186_outer_surface_to_drive_tyre_inner_face_mm": 32.0,
            "fit0186_outer_surface_to_arch_leg_mm": 24.0,
            "fit0186_outer_surface_to_flywheel_tread_mm": 26.5,
            "lidar_and_crossbar": "clear in nominal -10..+20 sweep",
            "note": "Y=-190 would overlap the new Y=-172 basket wall by 0.5 mm; dependent actuator brackets move to Y=-205 without changing X/Z leverage"
        },
        "dependent_parts": {"locks_with_p52": ["actuator fixed clevis on X318 arch", "moving lever/clevis", "Tr8 nut carriage and thrust-bearing housing", "AS5600 pod and magnet cap", "indexed upper/lower stop blocks", "shaft cam/followers and fail-safe springs", "front basket/common-throat panels and feed-channel interface"],
                            "remains_open": ["physical FIT0186 continuous torque/thermal/current test", "Tr8 nut friction/backdrive and backlash", "measured Parkside adapter envelopes", "selector timing and contamination test", "singulator design in 5d", "printed basket physical 40-ball gate", "cable bend radii and strain-relief test"]},
        "limitations": ["static analytic and nominal-envelope CAD only; no Gazebo", "stall torque is never treated as continuous torque", "no STL, firmware, robot-stack or electronics changes"],
    }
    out = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/p252_transition_static.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
