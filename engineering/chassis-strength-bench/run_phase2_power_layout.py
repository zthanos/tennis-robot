#!/usr/bin/env python3
"""Phase-2 battery-layout mass, axle-load and traction comparison.

This is a quasi-static packaging screen.  It does not model a skid-steer tyre,
electrical current sharing, battery BMS behaviour or dynamic load transfer.
"""
from __future__ import annotations

import json
from pathlib import Path

G = 9.80665
REAR_X_M = -0.065
FRONT_X_M = 0.300
TRACK_M = 0.594
WHEEL_RADIUS_M = 0.085
FIT0403_STALL_TORQUE_NM_12V = 3.73
MUS = (0.3, 0.6, 0.9)
BALL_MASS_KG = 0.057


def solve(parts: list[dict]) -> dict:
    mass = sum(p["mass_kg"] for p in parts)
    cg_x = sum(p["mass_kg"] * p["x_m"] for p in parts) / mass
    front_fraction = (cg_x - REAR_X_M) / (FRONT_X_M - REAR_X_M)
    weight = mass * G
    front = weight * front_fraction
    rear = weight - front
    per_wheel_motor_force = FIT0403_STALL_TORQUE_NM_12V / WHEEL_RADIUS_M
    traction = {}
    for mu in MUS:
        traction[str(mu)] = {
            "rear_each_friction_n": mu * rear / 2,
            "front_each_friction_n": mu * front / 2,
            "rear_each_effective_cap_n": min(mu * rear / 2, per_wheel_motor_force),
            "front_each_effective_cap_n": min(mu * front / 2, per_wheel_motor_force),
            "all_wheels_n": mu * weight,
            "rear_each_limited_by": "tyre" if mu * rear / 2 < per_wheel_motor_force else "motor",
            "front_each_limited_by": "tyre" if mu * front / 2 < per_wheel_motor_force else "motor",
        }

    # Coulomb skid-steer comparison only: longitudinal traction produces yaw
    # at half-track; lateral scrub acts at each axle's CG lever arm.
    yaw_per_mu = weight * TRACK_M / 2
    scrub_per_mu = front * (FRONT_X_M - cg_x) + rear * (cg_x - REAR_X_M)
    return {
        "mass_kg": mass,
        "cg_x_m": cg_x,
        "front_axle_fraction": front_fraction,
        "axle_load_n": {"rear_total": rear, "front_total": front,
            "rear_each": rear / 2, "front_each": front / 2},
        "traction_caps": traction,
        "pivot_turn_screen": {
            "available_longitudinal_yaw_moment_per_mu_nm": yaw_per_mu,
            "lateral_scrub_moment_per_mu_nm": scrub_per_mu,
            "available_to_scrub_ratio_if_mu_long_equals_mu_lat": yaw_per_mu / scrub_per_mu,
            "warning": "comparative Coulomb screen only; tyre deformation, surface and dynamic load transfer dominate real skid steer",
        },
        "parts": parts,
    }


def parts(power: str, full: bool) -> list[dict]:
    common = [
        {"name": "bare_mobile_base", "mass_kg": 10.0, "x_m": 0.100},
        {"name": "shared_flywheel_module_design", "mass_kg": 4.706, "x_m": 0.427},
        {"name": "basket_and_supports", "mass_kg": 1.50, "x_m": 0.165},
    ]
    if power == "parkside_pair":
        common += [
            {"name": "two_Parkside_X20V_4Ah", "mass_kg": 1.30, "x_m": -0.165},
            {"name": "UGREEN_12000mAh_100W", "mass_kg": 0.309, "x_m": -0.100},
        ]
    elif power == "redodo":
        common += [{"name": "Redodo_12V_50Ah", "mass_kg": 5.29, "x_m": -0.055}]
    else:
        raise ValueError(power)
    if full:
        common += [{"name": "28_tennis_balls", "mass_kg": 28 * BALL_MASS_KG, "x_m": 0.165}]
    return common


def main() -> None:
    scenarios = {}
    for power in ("parkside_pair", "redodo"):
        scenarios[power] = {
            "empty_basket": solve(parts(power, False)),
            "full_28_balls": solve(parts(power, True)),
        }
    report = {
        "status": "PHASE_2_POWER_LAYOUT_PARAMETRIC_PRE_PURCHASE",
        "geometry": {"rear_axle_x_m": REAR_X_M, "front_axle_x_m": FRONT_X_M,
            "track_m": TRACK_M, "wheel_radius_m": WHEEL_RADIUS_M},
        "power_architecture": {
            "baseline": "two Parkside X20V Team 4 Ah packs, individually ideal-diode ORed into one 15..21 V propulsion bus",
            "parkside_pack_current_limit_a_each": None,
            "parkside_energy_wh": {"nominal_18v_pair": 144, "20v_max_label_pair": 160},
            "FIT0403_pwm_duty_ceiling_for_12v_average": {"15v_bus": 0.8,
                "18v_bus": 2/3, "21v_bus": 12/21},
            "logic_power": "separate UGREEN 12000 mAh 100 W USB-PD power bank; 309 g",
            "redodo_comparison": "12.8 V 50 Ah, 195x166x172 mm, 5.29 kg official listing, 50 A continuous BMS",
        },
        "assumptions": [
            "Parkside pair centres reserved at X=-165 mm on external-access rear quick-change docks; exact pack/adapter dimensions remain a measurement gate",
            "UGREEN centre X=-100 mm; manufacturer dimensions 45.5x46x115 mm and mass 309 g",
            "module uses conservative 4.706 kg design mass; basket/supports 1.50 kg",
            "no mass allowance yet for ideal diodes, adapters, fuses, XT60, buck converter, docks or guards",
        ],
        "cad_packaging_qa": {
            "parkside_pair": {"battery_vs_frame": "empty intersection",
                "battery_vs_D5065_controllers": "empty intersection",
                "battery_vs_rear_electronics": "empty intersection"},
            "redodo": {"battery_vs_frame": "empty intersection",
                "battery_vs_D5065_controllers": "empty intersection",
                "battery_vs_rear_electronics": "empty intersection"},
            "method": "OpenSCAD positive-solid intersection views; Parkside result remains provisional because pack/adapter dimensions are unmeasured",
        },
        "scenarios": scenarios,
        "electrical_gates": [
            "one fuse and one ideal-diode path per Parkside pack before the common bus; packs must not be hard-paralleled",
            "ideal diodes prevent reverse current but do not guarantee equal sharing; log current and voltage per pack branch",
            "pack current/BMS limit and connector temperature are unknown parameters until bench measurement",
            "FIT0403 duty clamp alone is not stall protection; require current limit, ramp and measured 12 V-equivalent behaviour",
            "P16 requires a regulated 12 V buck sized from measured peak current and protected against propulsion-bus transients",
            "propulsion bus needs low-voltage cutoff with hysteresis validated under sag and a master disconnect",
            "Pi/Mega/sensors stay galvanically on the USB power branch; Mega is never connected directly to 15..21 V",
        ],
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2/power_layout.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: {case: {"mass_kg": v[case]["mass_kg"],
        "cg_x_m": v[case]["cg_x_m"], "front_fraction": v[case]["front_axle_fraction"]}
        for case in v} for k, v in scenarios.items()}, indent=2))


if __name__ == "__main__":
    main()
