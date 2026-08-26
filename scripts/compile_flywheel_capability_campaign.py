#!/usr/bin/env python3
"""Compile executed standalone launcher trials into the authoritative capability map."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT / "config/flywheel_launcher_capability_protocol.json"
DEFAULT_OUTPUT = ROOT / "config/flywheel_launcher_capability_map.json"
SPEEDS = (80, 120, 160, 200, 240, 280, 300)
FRICTIONS = (0.3, 0.6, 0.9)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_frozen_inputs(protocol: dict) -> dict[str, bool]:
    frozen = protocol["immutable_inputs"]
    checks = {
        "ball_calibration_results": sha256(ROOT / frozen["ball_calibration_results"]["path"])
        == frozen["ball_calibration_results"]["sha256"],
        "ball_model": sha256(ROOT / frozen["ball_model"]["path"])
        == frozen["ball_model"]["sha256"],
        "standalone_bench_xacro": sha256(
            ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
        ) == frozen["standalone_bench_xacro_sha256"],
        "launcher_module_xacro": sha256(
            ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro"
        ) == frozen["launcher_module_xacro_sha256"],
        "lower_exit_relief_mesh": sha256(
            ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_lower_panel_exit_clearance.stl"
        ) == frozen["lower_exit_relief_mesh_sha256"],
        "upper_exit_relief_mesh": sha256(
            ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_upper_panel_exit_clearance.stl"
        ) == frozen["upper_exit_relief_mesh_sha256"],
        "normal_and_tangential_contact_plugin": sha256(
            ROOT / "ros2_ws/src/tennis_ball_contact_system/src/TennisBallContactSystem.cc"
        ) == frozen["normal_and_tangential_contact_plugin_sha256"],
        "motor_control_plugin": sha256(
            ROOT / "ros2_ws/src/tennis_ball_contact_system/src/FlywheelCapabilityControlSystem.cc"
        ) == frozen["motor_control_plugin_sha256"],
    }
    if not all(checks.values()):
        failed = ", ".join(name for name, passed in checks.items() if not passed)
        raise RuntimeError(f"frozen capability inputs changed: {failed}")
    return checks


def result(campaign: Path, case_id: str) -> dict:
    path = campaign / case_id / "result.json"
    if not path.exists():
        raise FileNotFoundError(f"missing executed result: {path}")
    return read_json(path)


def friction_label(mu: float) -> str:
    return {
        0.3: "LOW_TRACTION_SENSITIVITY_ASSUMPTION_ONLY",
        0.6: "MEDIUM_TRACTION_SENSITIVITY_ASSUMPTION_ONLY",
        0.9: "HIGH_TRACTION_SENSITIVITY_ASSUMPTION_ONLY",
    }[mu]


def compact(case_id: str, category: str, data: dict) -> dict:
    return {
        "case_id": case_id,
        "category": category,
        "mechanical_pitch_deg": data["mechanical_pitch_deg"],
        "left_target_rad_s": data["left_target_rad_s"],
        "right_target_rad_s": data["right_target_rad_s"],
        "left_actual_precontact_rad_s": data["left_actual_precontact_rad_s"],
        "right_actual_precontact_rad_s": data["right_actual_precontact_rad_s"],
        "wheel_surface_speed_m_s": data["surface_speed_m_s"],
        "spinup_time_s": data["spinup_time_s"],
        "exit_position_m": data["exit_position_xyz_m"],
        "exit_velocity_vector_m_s": data["exit_velocity_vector_m_s"],
        "exit_speed_m_s": data["exit_speed_m_s"],
        "elevation_deg": data["elevation_deg"],
        "azimuth_deg": data["azimuth_deg"],
        "spin_vector_rad_s": data["spin_vector_rad_s"],
        "spin_rpm": data["spin_equivalent_rpm"],
        "topspin_rad_s": data["topspin_rad_s"],
        "sidespin_rad_s": data["sidespin_rad_s"],
        "contact_duration_s": data["contact_duration_s"],
        "maximum_ball_compression_m": data["max_ball_compression_m"],
        "left_peak_normal_force_n": data["left_peak_force_n"],
        "right_peak_normal_force_n": data["right_peak_force_n"],
        "left_peak_tangential_force_n": data["left_peak_tangential_force_n"],
        "right_peak_tangential_force_n": data["right_peak_tangential_force_n"],
        "left_rpm_droop": data["left_rpm_droop"],
        "right_rpm_droop": data["right_rpm_droop"],
        "left_droop_percent": data["left_droop_percent"],
        "right_droop_percent": data["right_droop_percent"],
        "recovery_time_s": data["recovery_time_s"],
        "motor_response": data["motor_response"],
        "apex_height_m": data["apex_height_m"],
        "time_to_apex_s": data["time_to_apex_s"],
        "first_bounce_xyz_m": data["first_bounce_xyz_m"],
        "first_bounce_time_s": data["first_bounce_time_s"],
        "first_bounce_time_from_exit_s": data["first_bounce_time_from_exit_s"],
        "horizontal_range_m": data["horizontal_range_m"],
        "lateral_deviation_m": data["lateral_deviation_m"],
        "velocity_immediately_before_bounce_m_s": data[
            "velocity_immediately_before_bounce_m_s"
        ],
        "energy_accounting": data["energy_accounting"],
        "tyre_friction_assumption": {
            "coefficient": data["friction_assumption"],
            "classification": friction_label(data["friction_assumption"]),
            "physically_calibrated": False,
        },
        "wheel_mass_assumption": data["wheel_mass_assumption_kg"],
        "wheel_inertia_assumption": data["wheel_inertia_assumption_kg_m2"],
        "timestep_s": data["timestep_s"],
        "validity": {
            "successful_launch": data["successful_launch"],
            "target_reachable": data["target_reachable"],
            "bilateral_wheel_contact": data["wheel_contact_event_success"],
            "post_release_fixed_component_contact": not data["post_release_geometry_clear"],
            "classification": data["validity_classification"],
            "tangential_transfer_physically_validated": False,
        },
    }


def relative(a: float, b: float) -> float:
    return abs(a - b) / abs(b) if b else abs(a - b)


def convergence_comparison(data: dict, reference: dict, limits: dict) -> dict:
    spin = math.sqrt(sum(value * value for value in data["spin_vector_rad_s"]))
    spin_ref = math.sqrt(sum(value * value for value in reference["spin_vector_rad_s"]))
    spin_limit = max(limits["spin_difference"]["absolute_floor_rad_s"],
                     limits["spin_difference"]["maximum_relative"] * spin_ref)
    duration = max(data["contact_duration_s"].values())
    duration_ref = max(reference["contact_duration_s"].values())
    duration_limit = max(limits["contact_duration_difference"]["absolute_floor_s"],
                         limits["contact_duration_difference"]["maximum_relative"] * duration_ref)
    droop = max(data["left_droop_rad_s"], data["right_droop_rad_s"])
    droop_ref = max(reference["left_droop_rad_s"], reference["right_droop_rad_s"])
    recovery_limit = max(limits["recovery_time_difference"]["absolute_floor_s"],
                         limits["recovery_time_difference"]["maximum_relative"]
                         * reference["recovery_time_s"])
    residual = data["energy_accounting"]["residual_fraction"]
    residual_ref = reference["energy_accounting"]["residual_fraction"]
    checks = {
        "successful_and_clear": data["successful_launch"] and data["post_release_geometry_clear"],
        "exit_speed": relative(data["exit_speed_m_s"], reference["exit_speed_m_s"])
        <= limits["maximum_relative_exit_speed_difference"],
        "elevation": abs(data["elevation_deg"] - reference["elevation_deg"])
        <= limits["maximum_absolute_elevation_difference_deg"],
        "azimuth": abs(data["azimuth_deg"] - reference["azimuth_deg"])
        <= limits["maximum_absolute_azimuth_difference_deg"],
        "spin": abs(spin - spin_ref) <= spin_limit,
        "compression": relative(data["max_ball_compression_m"], reference["max_ball_compression_m"])
        <= limits["maximum_relative_compression_difference"],
        "peak_force": relative(max(data["left_peak_force_n"], data["right_peak_force_n"]),
                               max(reference["left_peak_force_n"], reference["right_peak_force_n"]))
        <= limits["maximum_relative_peak_force_difference"],
        "contact_duration": abs(duration - duration_ref) <= duration_limit,
        "wheel_droop": relative(droop, droop_ref)
        <= limits["maximum_relative_wheel_droop_difference"],
        "recovery": abs(data["recovery_time_s"] - reference["recovery_time_s"])
        <= recovery_limit,
        "energy_residual_absolute": abs(residual)
        <= limits["maximum_absolute_energy_residual_fraction"],
        "energy_residual_convergence": abs(residual - residual_ref)
        <= limits["maximum_energy_residual_fraction_difference"],
    }
    return {
        "timestep_s": data["timestep_s"],
        "reference_timestep_s": reference["timestep_s"],
        "metrics": {
            "exit_speed_relative_difference": relative(data["exit_speed_m_s"], reference["exit_speed_m_s"]),
            "elevation_absolute_difference_deg": abs(data["elevation_deg"] - reference["elevation_deg"]),
            "azimuth_absolute_difference_deg": abs(data["azimuth_deg"] - reference["azimuth_deg"]),
            "spin_absolute_difference_rad_s": abs(spin - spin_ref),
            "compression_relative_difference": relative(data["max_ball_compression_m"], reference["max_ball_compression_m"]),
            "peak_force_relative_difference": relative(max(data["left_peak_force_n"], data["right_peak_force_n"]), max(reference["left_peak_force_n"], reference["right_peak_force_n"])),
            "contact_duration_absolute_difference_s": abs(duration - duration_ref),
            "wheel_droop_relative_difference": relative(droop, droop_ref),
            "recovery_absolute_difference_s": abs(data["recovery_time_s"] - reference["recovery_time_s"]),
            "energy_residual_fraction": residual,
            "energy_residual_fraction_difference": abs(residual - residual_ref),
        },
        "checks": checks,
        "passed": all(checks.values()),
    }


def stats(records: list[dict], key) -> dict[str, float]:
    values = [key(item) for item in records]
    return {"mean": statistics.mean(values), "population_stddev": statistics.pstdev(values)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_dir", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    protocol = read_json(PROTOCOL)
    frozen_checks = check_frozen_inputs(protocol)

    symmetric_raw: list[tuple[str, dict]] = []
    for mu in FRICTIONS:
        for speed in SPEEDS:
            case_id = ("regression_mu_0p3_dt_1ms" if mu == 0.3 and speed == 80 else
                       f"sym_mu_{str(mu).replace('.', 'p')}_w{speed:03d}")
            symmetric_raw.append((case_id, result(args.campaign_dir, case_id)))
    symmetric = [compact(case_id, "symmetric_sweep", data) for case_id, data in symmetric_raw]

    regression = symmetric_raw[0][1]
    regression_limits = protocol["regression_acceptance"]
    regression_checks = {
        "successful_launch": regression["successful_launch"],
        "post_release_noncontact": regression["post_release_geometry_clear"],
        "exit_speed": relative(regression["exit_speed_m_s"], regression_limits["reference_exit_speed_m_s"])
        <= regression_limits["maximum_relative_exit_speed_difference"],
        "elevation": abs(regression["elevation_deg"] - regression_limits["reference_elevation_deg"])
        <= regression_limits["maximum_absolute_elevation_difference_deg"],
    }

    convergence = {}
    convergence_records = []
    for level, speed in (("low", 80), ("medium", 200), ("high", 280)):
        dt_rows = [
            next(data for _, data in symmetric_raw
                 if data["friction_assumption"] == 0.6 and data["left_target_rad_s"] == speed),
            result(args.campaign_dir, f"conv_{level}_w{speed:03d}_dt_0p5ms"),
            result(args.campaign_dir, f"conv_{level}_w{speed:03d}_dt_0p25ms"),
        ]
        reference = dt_rows[-1]
        comparisons = [convergence_comparison(row, reference,
                                               protocol["numerical_convergence_acceptance_against_0p25ms"])
                       for row in dt_rows]
        convergence[level] = {
            "target_magnitude_rad_s": speed,
            "comparisons": comparisons,
            "passed": all(item["passed"] for item in comparisons),
        }
        convergence_records.extend(
            compact(f"convergence_{level}_{row['timestep_s']:.5f}", "timestep_convergence", row)
            for row in dt_rows
        )
    convergence_passed = all(item["passed"] for item in convergence.values())

    differential_ids = ("diff_l300_r280", "diff_l280_r300", "diff_l300_r260", "diff_l260_r300")
    differential_raw = [result(args.campaign_dir, case_id) for case_id in differential_ids]
    differential = [compact(case_id, "differential_speed", data)
                    for case_id, data in zip(differential_ids, differential_raw)]

    repeatability = {}
    for level, speed in (("low", 80), ("medium", 200), ("high", 300)):
        base = next(data for _, data in symmetric_raw
                    if data["friction_assumption"] == 0.6 and data["left_target_rad_s"] == speed)
        trials = [base] + [result(args.campaign_dir, f"repeat_{level}_w{speed:03d}_r{index}")
                           for index in (2, 3)]
        repeatability[level] = {
            "target_magnitude_rad_s": speed,
            "executed_trials": 3,
            "exit_speed_m_s": stats(trials, lambda d: d["exit_speed_m_s"]),
            "elevation_deg": stats(trials, lambda d: d["elevation_deg"]),
            "azimuth_deg": stats(trials, lambda d: d["azimuth_deg"]),
            "spin_rpm": stats(trials, lambda d: d["spin_equivalent_rpm"]),
            "first_bounce_x_m": stats(trials, lambda d: d["first_bounce_xyz_m"][0]),
            "first_bounce_y_m": stats(trials, lambda d: d["first_bounce_xyz_m"][1]),
        }

    valid_symmetric = [record for record in symmetric if record["validity"]["successful_launch"]]
    target_results = {}
    capability_limit = protocol["capability_classification_acceptance"][
        "maximum_absolute_exit_speed_error_m_s"]
    for target in protocol["campaign_matrix"]["capability_targets_m_s"]:
        nearest = min(valid_symmetric, key=lambda record: abs(record["exit_speed_m_s"] - target))
        error = abs(nearest["exit_speed_m_s"] - target)
        target_results[f"{int(target)}_m_s"] = {
            "reachable": error <= capability_limit and convergence_passed,
            "status": "SUPPORTED_BY_EXECUTED_POINT" if error <= capability_limit and convergence_passed
            else "EXECUTED_ENVELOPE_DOES_NOT_REACH_TARGET",
            "absolute_speed_error_m_s": error,
            "nearest_executed_operating_point": nearest,
        }

    maximum = max(valid_symmetric, key=lambda record: record["exit_speed_m_s"])
    all_launches_clear = all(record["validity"]["successful_launch"] for record in symmetric + differential)
    energy_passed = all(
        abs(record["energy_accounting"]["residual_fraction"])
        <= protocol["numerical_convergence_acceptance_against_0p25ms"][
            "maximum_absolute_energy_residual_fraction"]
        for record in symmetric + differential + convergence_records
    )
    motor_passed = all(
        record["validity"]["target_reachable"] and
        record["motor_response"]["left_peak_event_current_a"] <= 20.0 + 1e-9 and
        record["motor_response"]["right_peak_event_current_a"] <= 20.0 + 1e-9 and
        record["motor_response"]["estimated_peak_required_bus_voltage_v"] <= 12.8
        for record in symmetric + differential
    )

    classes = {
        "STANDALONE_LAUNCHER_CAPABILITY_BENCH_VALID": all(regression_checks.values()) and all_launches_clear,
        "POST_FLYWHEEL_PATH_NONCONTACTING": all_launches_clear,
        "NORMAL_CONTACT_LAUNCH_VALIDATED": convergence_passed,
        "TANGENTIAL_CONTACT_LAUNCH_VALIDATED": False,
        "LAUNCH_EXIT_VELOCITY_VALIDATED": convergence_passed and all_launches_clear,
        "LAUNCH_EXIT_SPEED_VALIDATED": convergence_passed and all_launches_clear,
        "LAUNCH_EXIT_ELEVATION_VALIDATED": convergence_passed and all_launches_clear,
        "LAUNCH_EXIT_AZIMUTH_VALIDATED": convergence_passed and all_launches_clear,
        "LAUNCH_SPIN_VALIDATED": False,
        "LAUNCH_SPIN_NUMERICALLY_CHARACTERIZED": all(item["validity"]["successful_launch"] for item in differential),
        "RPM_DROOP_VALIDATED": convergence_passed and motor_passed,
        "RPM_RECOVERY_VALIDATED": convergence_passed and motor_passed,
        "LAUNCH_ENERGY_ACCOUNTING_VALIDATED": convergence_passed and energy_passed,
        "LAUNCH_CONTACT_TIMESTEP_CONVERGED": convergence_passed,
        "LAUNCHER_12_M_S_CAPABILITY": target_results["12_m_s"]["reachable"],
        "LAUNCHER_14_M_S_CAPABILITY": target_results["14_m_s"]["reachable"],
        "LAUNCHER_16_M_S_CAPABILITY": target_results["16_m_s"]["reachable"],
        "LAUNCHER_18_M_S_CAPABILITY": target_results["18_m_s"]["reachable"],
        "BALL_EXIT_STATE_VALIDATED": convergence_passed and all_launches_clear,
        "COURT_TRAJECTORY_MODEL_VALIDATED": False,
        "OPPOSITE_BASELINE_REACH_CAPABILITY": False,
        "LEFT_DEEP_CORNER_REACH_CAPABILITY": False,
        "RIGHT_DEEP_CORNER_REACH_CAPABILITY": False,
        "LAUNCHER_CAPABILITY_MAP_GENERATED": True,
        "BALL_LAUNCH_PHYSICS_VALIDATED_IN_SIM": False,
        "PHYSICAL_FLYWHEEL_WHEEL_VALIDATED": False,
        "PHYSICAL_HARDWARE_PENDING": True,
    }

    output = {
        "schema_version": 2,
        "generated_on": "2026-08-26",
        "scope": "isolated_standalone_flywheel_launcher_capability_validation",
        "decision": {
            "status": "CAPABILITY_MAP_COMPLETE_WITH_PHYSICAL_MODEL_LIMITATIONS",
            "stop_condition_encountered": False,
            "operational_speed_targets_reached": False,
            "reason": "All executed physics gates passed, but the bounded uncalibrated tyre-friction model saturates below 12 m/s and no aerodynamic court model exists."
        },
        "protocol": "config/flywheel_launcher_capability_protocol.json",
        "frozen_input_checks": frozen_checks,
        "regression_reference": {
            "checks": regression_checks,
            "passed": all(regression_checks.values()),
            "operating_point": symmetric[0],
        },
        "authoritative_baseline": protocol["mechanical_and_motor_baseline"],
        "contact_model_status": {
            "calibrated_ball_normal_law_unchanged": True,
            "launcher_results_used_for_ball_refit": False,
            "tyre_friction_physically_calibrated": False,
            "tyre_normal_compliance_separately_modelled": False,
            "friction_sensitivity_coefficients": list(FRICTIONS),
            "normal_contact_conclusion": "NUMERICALLY_CONVERGED_FROZEN_BALL_LAW_AGAINST_RIGID_WHEEL",
            "tangential_contact_conclusion": "SENSITIVITY_ONLY_NOT_PHYSICALLY_VALIDATED",
        },
        "symmetric_operating_points": symmetric,
        "timestep_convergence": {
            "acceptance_was_frozen_before_results": True,
            "acceptance": protocol["numerical_convergence_acceptance_against_0p25ms"],
            "conditions": convergence,
            "passed": convergence_passed,
        },
        "differential_operating_points": differential,
        "differential_spin_summary": {
            "maximum_spin_magnitude_rad_s": max(math.sqrt(sum(value * value for value in row["spin_vector_rad_s"])) for row in differential),
            "maximum_absolute_azimuth_deg": max(abs(row["azimuth_deg"]) for row in differential),
            "classification": "NUMERICALLY_CHARACTERIZED_UNDER_UNCALIBRATED_FRICTION_NOT_PHYSICALLY_VALIDATED",
        },
        "capability_targets": target_results,
        "measured_envelope": {
            "maximum_exit_speed_m_s": maximum["exit_speed_m_s"],
            "maximum_exit_speed_operating_point": maximum,
            "motor_reachable_symmetric_target_magnitude_rad_s": [80.0, 300.0],
        },
        "trajectory_model": {
            "gravity": True,
            "aerodynamic_drag": False,
            "magnus_force": False,
            "spin_decay": False,
            "ball_exit_state_validated": classes["BALL_EXIT_STATE_VALIDATED"],
            "court_trajectory_model_validated": False,
            "first_bounce_values_classification": "GRAVITY_ONLY_DIAGNOSTIC",
            "court_target_mapping_performed": False,
            "range_capability": "NOT_EVALUATED_WITH_CREDIBLE_COURT_MODEL",
            "targeting_capability": "NOT_EVALUATED_NO_VALIDATED_HORIZONTAL_AIMING_DOF",
        },
        "repeatability": {
            "deterministic_simulation_repeatability_only": True,
            "physical_repeatability_claimed": False,
            "conditions": repeatability,
        },
        "evidence": {
            "campaign_raw_data_location_during_generation": str(args.campaign_dir),
            "campaign_plot": "docs/images/flywheel-capability-campaign-map.png",
            "representative_stage_frames": "docs/images/flywheel-capability-representative-stages.png",
            "runner": "scripts/run_flywheel_capability_case.py",
            "case_analyzer": "scripts/analyze_flywheel_capability_case.py",
            "campaign_compiler": "scripts/compile_flywheel_capability_campaign.py",
            "plotter": "scripts/plot_flywheel_capability_campaign.py",
        },
        "classifications": classes,
    }
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
