#!/usr/bin/env python3
"""Build the provisional wheel/hub Gate A baseline and motor sensitivity screen."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JSON = ROOT / "config/flywheel_launcher_provisional_gate_a.json"
DEFAULT_CSV = ROOT / "docs/mechanism/flywheel-wheel-candidate-capability-screen.csv"
BALL_RESULTS = ROOT / "config/tennis_ball_compliance_calibration_results.json"


def annular_mass(density: float, outer_d: float, inner_d: float, length: float) -> float:
    return density * math.pi * (outer_d**2 - inner_d**2) * length / 4.0


def annular_polar_inertia(mass: float, outer_d: float, inner_d: float) -> float:
    return 0.5 * mass * ((outer_d / 2.0) ** 2 + (inner_d / 2.0) ** 2)


def annular_transverse_inertia(mass: float, outer_d: float, inner_d: float, length: float) -> float:
    return mass * (3.0 * ((outer_d / 2.0) ** 2 + (inner_d / 2.0) ** 2) + length**2) / 12.0


def hub_properties() -> dict[str, object]:
    density = 2810.0  # provisional 7075-T6 aluminium
    parts = [
        # name, OD, ID, axial length, launcher-local Z centre
        ("split_d_clamp_collar", 0.022, 0.008, 0.0135, 0.03175),
        ("pilot_sleeve_over_shaft_tip", 0.010, 0.008, 0.008, 0.021),
        ("solid_10mm_wheel_pilot", 0.010, 0.0, 0.042, -0.004),
        ("m8_retention_stem", 0.008, 0.0, 0.008, -0.029),
        ("retention_washer", 0.022, 0.0084, 0.002, -0.026),
        ("retention_nut_envelope", 0.013, 0.008, 0.006, -0.030),
    ]
    evaluated = []
    for name, od, inner_d, length, z in parts:
        mass = annular_mass(density, od, inner_d, length)
        evaluated.append(
            {
                "name": name,
                "outer_diameter_m": od,
                "inner_diameter_m": inner_d,
                "length_m": length,
                "centre_z_m": z,
                "mass_kg": mass,
                "polar_inertia_kg_m2": annular_polar_inertia(mass, od, inner_d),
                "centroidal_transverse_inertia_kg_m2": annular_transverse_inertia(
                    mass, od, inner_d, length
                ),
            }
        )
    total_mass = sum(part["mass_kg"] for part in evaluated)
    centre_z = sum(part["mass_kg"] * part["centre_z_m"] for part in evaluated) / total_mass
    polar = sum(part["polar_inertia_kg_m2"] for part in evaluated)
    transverse = sum(
        part["centroidal_transverse_inertia_kg_m2"]
        + part["mass_kg"] * (part["centre_z_m"] - centre_z) ** 2
        for part in evaluated
    )
    return {
        "status": "PROVISIONAL_HUB_FOR_SIMULATION",
        "material": "7075-T6 aluminium, analysis assumption; metal required but alloy not frozen",
        "density_kg_m3": density,
        "parts": evaluated,
        "mass_kg": total_mass,
        "centre_z_m": centre_z,
        "polar_inertia_kg_m2": polar,
        "transverse_inertia_about_hub_com_kg_m2": transverse,
    }


def build_result() -> dict[str, object]:
    hub = hub_properties()
    ball = json.loads(BALL_RESULTS.read_text(encoding="utf-8"))
    ball_sha = hashlib.sha256(BALL_RESULTS.read_bytes()).hexdigest()
    rebound = min(ball["rebound_results"], key=lambda item: item["timestep_s"])
    calibrated_event_energy = rebound["incident_energy_j"]

    wheel_radius = 0.100
    wheel_width = 0.050
    nominal_wheel_mass = 0.90
    wheel_inertia_factor_nominal = 0.75
    nominal_wheel_inertia = wheel_inertia_factor_nominal * nominal_wheel_mass * wheel_radius**2
    nominal_total_mass = nominal_wheel_mass + hub["mass_kg"]
    nominal_com_z = hub["mass_kg"] * hub["centre_z_m"] / nominal_total_mass
    wheel_transverse = 0.5 * nominal_wheel_inertia + nominal_wheel_mass * wheel_width**2 / 12.0
    nominal_transverse = (
        wheel_transverse
        + nominal_wheel_mass * nominal_com_z**2
        + hub["transverse_inertia_about_hub_com_kg_m2"]
        + hub["mass_kg"] * (hub["centre_z_m"] - nominal_com_z) ** 2
    )
    nominal_polar = nominal_wheel_inertia + hub["polar_inertia_kg_m2"]

    # Fixed assembly after the post-nip corridor audit: two shaped-relief
    # aluminium panels and two complete 0.49 kg motor envelopes. Values are
    # the integrated 2-D plate sections extruded through 8 mm, including the
    # two existing 12 mm upper shaft holes. Rotor inertia stays separate.
    fixed_mass = 4.3347825646063045
    fixed_com_x = -0.002312972947848475
    fixed_com_z = 0.018506146220124177
    fixed_ixx = 0.05617132656275642
    fixed_iyy = 0.029416334185606255
    fixed_izz = 0.06303724768379373
    fixed_ixz = -0.00035044300627756883

    kt = 0.031
    kv = 270.0
    resistance = 0.039
    bus_voltage = 12.8
    current_limit = 20.0
    torque_limit = kt * current_limit
    acceleration_objective_s = 2.0
    rotor_inertia_sensitivity = [0.0, 0.0001, 0.0002]

    cases = []
    for mass in (0.70, 0.80, 0.90):
        for distribution, factor in (("LOW_SOLID_DISK_BOUND", 0.5), ("HIGH_THIN_RING_BOUND", 1.0)):
            wheel_inertia = factor * mass * wheel_radius**2
            driven_inertia = wheel_inertia + hub["polar_inertia_kg_m2"]
            for rpm in (1000, 1250, 1500, 1750, 2000):
                omega = rpm * 2.0 * math.pi / 60.0
                voltage_at_limit = rpm / kv + current_limit * resistance
                available_current = min(current_limit, max(0.0, (bus_voltage - rpm / kv) / resistance))
                available_torque = kt * available_current
                required_torque = driven_inertia * omega / acceleration_objective_s
                required_current = required_torque / kt
                spinup_time = driven_inertia * omega / available_torque
                wheel_energy = 0.5 * driven_inertia * omega**2
                pair_reservoir = 2.0 * wheel_energy
                if pair_reservoir > calibrated_event_energy:
                    post_event_omega = math.sqrt(omega**2 - calibrated_event_energy / driven_inertia)
                    droop_rpm = (omega - post_event_omega) * 60.0 / (2.0 * math.pi)
                    recovery_s = driven_inertia * (omega - post_event_omega) / available_torque
                else:
                    droop_rpm = None
                    recovery_s = None
                cases.append(
                    {
                        "wheel_mass_kg": mass,
                        "wheel_inertia_distribution": distribution,
                        "wheel_inertia_kg_m2": wheel_inertia,
                        "hub_inertia_kg_m2": hub["polar_inertia_kg_m2"],
                        "motor_rotor_inertia_included_kg_m2": 0.0,
                        "driven_inertia_without_motor_rotor_kg_m2": driven_inertia,
                        "target_rpm": rpm,
                        "spinup_time_at_20a_s": spinup_time,
                        "two_second_objective_torque_nm": required_torque,
                        "two_second_objective_current_a": required_current,
                        "available_current_at_target_a": available_current,
                        "available_torque_at_target_nm": available_torque,
                        "voltage_required_at_20a_v": voltage_at_limit,
                        "kinetic_energy_per_wheel_j": wheel_energy,
                        "pair_contact_energy_reservoir_j": pair_reservoir,
                        "peak_mechanical_power_per_motor_w": available_torque * omega,
                        "calibrated_rebound_event_energy_surrogate_j": calibrated_event_energy,
                        "predicted_droop_for_surrogate_rpm": droop_rpm,
                        "predicted_recovery_for_surrogate_s": recovery_s,
                        "spinup_time_with_0p2e_minus3_rotor_inertia_s": (driven_inertia + 0.0002)
                        * omega
                        / available_torque,
                    }
                )

    gravity = 9.80665
    screen_radial_reaction = 1.25 * max(item["peak_force_n"] for item in ball["rebound_results"])
    motor_static_moment = 0.49 * gravity * 0.065 / 2.0
    rotating_static_moment = nominal_total_mass * gravity * 0.043
    radial_moment = screen_radial_reaction * 0.043
    total_overturning_moment = radial_moment + motor_static_moment + rotating_static_moment
    bolt_tension = total_overturning_moment / (2.0 * 0.015)
    cutout_d = 0.012
    cutout_ligament = 0.015 - 0.004 / 2.0 - cutout_d / 2.0
    net_ligament_area = 2.0 * cutout_ligament * 0.008

    classes = {
        "D5065_DIRECT_PANEL_MOUNT_GEOMETRICALLY_VALID": True,
        "D5065_DIRECT_PANEL_MOUNT_STRUCTURALLY_SCREENED": True,
        "FLYWHEEL_PANEL_CUTOUT_DEFINED": True,
        "FLYWHEEL_DIRECT_DRIVE_HUB_DEFINED_FOR_SIMULATION": True,
        "FLYWHEEL_SHAFT_ENGAGEMENT_VALIDATED_FOR_SIMULATION": True,
        "FLYWHEEL_AXIAL_RETENTION_DEFINED_FOR_SIMULATION": True,
        "FLYWHEEL_ROTATING_MASS_BOUNDED": True,
        "FLYWHEEL_ROTATING_INERTIA_BOUNDED": True,
        "FLYWHEEL_MECHANICAL_GATE_A_SIMULATION_READY": True,
        "FLYWHEEL_MECHANICAL_GATE_A_PHYSICAL_VALIDATED": False,
    }
    return {
        "schema_version": 1,
        "generated_on": "2026-08-26",
        "scope": "standalone flywheel provisional mechanical Gate A",
        "decision": {
            "status": "SIMULATION_READY_PROVISIONAL_BASELINE",
            "capability_phase_authorized": True,
            "physical_hardware_released": False,
            "procurement_frozen": False,
            "launcher_trials_run_by_this_gate": False,
        },
        "wheel_candidate": {
            "description": "AliExpress electric-skateboard/off-road pneumatic aluminium-hub rubber wheel",
            "evidence": "USER_SUPPLIED_SELLER_DATA",
            "diameter_m": 0.200,
            "width_m": 0.050,
            "bore_diameter_m": 0.010,
            "hub_material": "aluminium alloy",
            "tyre_material": "rubber / pneumatic",
            "mass_range_kg": [0.70, 0.90],
            "nominal_simulation_mass_kg": nominal_wheel_mass,
            "physical_measurement_pending": True,
            "revisit_allowed": True,
            "critical_interface_assumption": "The 10 mm datum is a concentric rigid through-bore with clampable aluminium hub faces, not the inner race of a free-running bearing stack.",
        },
        "hub": {
            **hub,
            "shaft_bore": "8 mm blind D-bore, split clamp plus dog-point screw registered to shaft flat",
            "shaft_engagement_m": 0.0215,
            "shaft_flat_available_m": 0.024,
            "collar_outer_diameter_m": 0.022,
            "wheel_pilot_diameter_m": 0.010,
            "wheel_pilot_length_m": 0.050,
            "wheel_torque_transfer": "wheel aluminium hub clamped between outer flange and inner washer/M8 all-metal locknut; friction torque screen only",
            "axial_retention": "integral outer flange at z=+25 mm and removable inner washer + M8 all-metal locknut at z=-25 mm",
            "assumed_wheel_clamp_preload_n": 5000.0,
            "assumed_dry_aluminium_interface_friction": 0.15,
            "assumed_effective_friction_radius_m": 0.008,
            "screened_wheel_interface_torque_nm": 6.0,
            "required_peak_motor_torque_nm": 2.635,
            "manufacturing_cad_released": False,
        },
        "axial_stack_launcher_local_z_m": {
            "motor_mounting_face_and_panel_outside": 0.047,
            "panel_inside": 0.039,
            "hub_outer_clearance_face": 0.0385,
            "hub_clamp_flange_start": 0.0385,
            "hub_flange_wheel_shoulder": 0.025,
            "flywheel_outer_face": 0.025,
            "shaft_tip": 0.017,
            "flywheel_centre": 0.0,
            "flywheel_inner_face": -0.025,
            "retention_washer_outer_face": -0.027,
            "retention_nut_end": -0.033,
        },
        "panel_cutout": {
            "shape": "circular",
            "diameter_m": cutout_d,
            "shaft_radial_clearance_m": (cutout_d - 0.008) / 2.0,
            "mount_hole_edge_ligament_m": cutout_ligament,
            "hub_passes_through_panel": False,
            "installation_sequence": "Mount motor to panel, insert hub from cradle interior onto protruding shaft, tighten radial clamp tools from open cradle side, then install wheel and inner retention hardware.",
            "reason_hub_not_passed_through": "A 22 mm collar through-opening would leave only 2 mm to the nominal 4 mm mount-hole edge; inside-first assembly preserves a practical 12 mm cutout and 7 mm ligament.",
        },
        "structural_rescreen": {
            "status": "SCREEN_PASS_WITH_PROVISIONAL_INPUTS",
            "screen_radial_reaction_n": screen_radial_reaction,
            "radial_moment_nm": radial_moment,
            "static_moment_nominal_wheel_plus_hub_and_motor_nm": motor_static_moment + rotating_static_moment,
            "total_overturning_moment_nm": total_overturning_moment,
            "worst_cardinal_bolt_tension_n": bolt_tension,
            "panel_bearing_stress_pa": bolt_tension / (0.008 * 0.004),
            "strip_bending_stress_30mm_pa": 6.0 * total_overturning_moment / (0.030 * 0.008**2),
            "strip_bending_stress_50mm_pa": 6.0 * total_overturning_moment / (0.050 * 0.008**2),
            "cutout_net_ligament_tension_stress_pa": bolt_tension / net_ligament_area,
            "panel_alloy": "6061-T6 comparison assumption only",
            "physical_validation": False,
        },
        "rotating_mass_and_inertia": {
            "wheel_mass_sensitivity_kg": [0.70, 0.80, 0.90],
            "wheel_inertia_lower_law": "0.5*m*R^2 solid-disk bound",
            "wheel_inertia_upper_law": "m*R^2 thin-ring bound",
            "wheel_inertia_overall_range_kg_m2": [0.0035, 0.0090],
            "nominal_wheel_inertia_law": "0.75*m*R^2 midpoint provisional approximation",
            "nominal_wheel_inertia_kg_m2": nominal_wheel_inertia,
            "hub_mass_kg": hub["mass_kg"],
            "hub_polar_inertia_kg_m2": hub["polar_inertia_kg_m2"],
            "nominal_rotating_link_mass_kg": nominal_total_mass,
            "nominal_rotating_link_com_z_m": nominal_com_z,
            "nominal_rotating_link_polar_inertia_kg_m2": nominal_polar,
            "nominal_rotating_link_transverse_inertia_kg_m2": nominal_transverse,
            "motor_rotor_inertia_manufacturer_value": None,
            "motor_rotor_inertia_sensitivity_kg_m2": rotor_inertia_sensitivity,
            "motor_rotor_inertia_included_in_xacro": False,
        },
        "motor_model": {
            "speed_constant_rpm_per_v": kv,
            "torque_constant_nm_per_a": kt,
            "phase_neutral_resistance_ohm": resistance,
            "bus_voltage_v": bus_voltage,
            "current_limit_a": current_limit,
            "torque_limit_nm": torque_limit,
            "continuous_current_free_air_a": 45.0,
            "peak_current_3s_a": 85.0,
            "constant_20a_torque_base_speed_rpm": (bus_voltage - current_limit * resistance) * kv,
            "acceleration_objective_s": acceleration_objective_s,
            "not_ideal_velocity_source": True,
            "capability_cases": cases,
        },
        "calibrated_ball_event_surrogate": {
            "source_path": str(BALL_RESULTS.relative_to(ROOT)),
            "source_sha256": ball_sha,
            "event": "independent 2.54 m rebound incident energy at converged 0.25 ms step",
            "energy_j": calibrated_event_energy,
            "is_launcher_event": False,
            "use_limit": "Only an energy-withdrawal sensitivity for provisional RPM droop/recovery. It is not launch-contact validation and does not calibrate tyre friction.",
        },
        "target_ball_translation_energy_j": {
            str(speed): 0.5 * ball["physical_parameters"]["mass_kg"] * speed**2
            for speed in (12, 14, 16, 18)
        },
        "standalone_model": {
            "xacro": "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro",
            "bench_control_interface": "effort",
            "bench_effort_limit_nm": torque_limit,
            "ideal_velocity_command_removed": True,
            "nominal_wheel_mass_kg": nominal_wheel_mass,
            "nominal_hub_mass_kg": hub["mass_kg"],
            "nominal_spin_inertia_without_motor_rotor_kg_m2": nominal_polar,
            "fixed_assembly_mass_kg": fixed_mass,
            "fixed_assembly_com_x_m": fixed_com_x,
            "fixed_assembly_com_z_m": fixed_com_z,
            "fixed_assembly_inertia_kg_m2": {
                "ixx": fixed_ixx,
                "iyy": fixed_iyy,
                "izz": fixed_izz,
                "ixz": fixed_ixz,
            },
            "complete_robot_propagated": False,
        },
        "status": {
            "FLYWHEEL_WHEEL_CANDIDATE_SELECTED": True,
            "FLYWHEEL_WHEEL_FINAL_PROCUREMENT_FROZEN": False,
            "FLYWHEEL_WHEEL_PHYSICAL_MEASUREMENT_PENDING": True,
            "FLYWHEEL_WHEEL_REVISIT_ALLOWED": True,
        },
        "revisit_stop_criteria": [
            "The delivered 10 mm interface is a bearing inner race or is not a rigid concentric through-bore with clampable hub faces.",
            "Measured wheel mass is outside 0.70..0.90 kg or measured polar inertia is outside the 0.5*m*R^2..m*R^2 bracket.",
            "The delivered wheel face cannot sustain the assumed 5 kN axial clamp or does not provide at least 8 mm effective friction radius.",
            "D5065 flat location or usable shaft engagement prevents 21.5 mm D-bore clamping.",
            "Final motor fastener engagement, panel alloy/cutout, runout, balance, vibration, hub torque or axial retention checks fail.",
        ],
        "classifications": classes,
    }


def write_csv(path: Path, cases: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(cases[0]))
        writer.writeheader()
        writer.writerows(cases)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()
    result = build_result()
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    write_csv(args.csv, result["motor_model"]["capability_cases"])
    print(json.dumps(result["classifications"], indent=2, sort_keys=True))
    return 0 if result["classifications"]["FLYWHEEL_MECHANICAL_GATE_A_SIMULATION_READY"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
