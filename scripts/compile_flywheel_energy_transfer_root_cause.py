#!/usr/bin/env python3
"""Compile the frozen flywheel energy-transfer diagnostic campaign."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.compile_flywheel_capability_campaign import convergence_comparison


PROTOCOL_PATH = ROOT / "config/flywheel_energy_transfer_diagnostic_protocol.json"
CAPABILITY_PROTOCOL_PATH = ROOT / "config/flywheel_launcher_capability_protocol.json"
DEFAULT_OUTPUT = ROOT / "config/flywheel_energy_transfer_root_cause.json"
DEFAULT_SUMMARY = ROOT / "docs/mechanism/flywheel-energy-transfer-case-summary.csv"
DEFAULT_TELEMETRY = ROOT / "docs/mechanism/flywheel-energy-transfer-contact-telemetry.csv"
SPEEDS = (80, 120, 160, 200, 240, 280, 300)
MUS = (0.3, 0.6, 0.9, 1.2, 1.5, 2.0, 2.5, 3.0)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def label(value: float) -> str:
    return str(value).replace(".", "p")


def baseline_case(campaign: Path, mu: float, speed: int) -> Path:
    if mu == 0.3 and speed == 80:
        return campaign / "regression_mu_0p3_dt_1ms"
    return campaign / f"sym_mu_{label(mu)}_w{speed:03d}"


def extended_case(campaign: Path, mu: float, speed: int) -> Path:
    return campaign / f"mu_{label(mu)}_w{speed:03d}"


def load_case(case_dir: Path, case_id: str, category: str) -> tuple[dict, list[dict]]:
    result_path = case_dir / "diagnostic_result.json"
    telemetry_path = case_dir / "diagnostic_timeseries.csv"
    if not result_path.exists() or not telemetry_path.exists():
        raise FileNotFoundError(f"missing analyzed diagnostic case: {case_dir}")
    result = read_json(result_path)
    capability = result["capability"]
    diagnostic = result["diagnostic"]
    left = diagnostic["contacts"]["left"]
    energy = diagnostic["energy_transfer"]
    record = {
        "case_id": case_id,
        "category": category,
        "friction_coefficient": diagnostic["friction_assumption"],
        "friction_classification": diagnostic["friction_classification"],
        "target_wheel_speed_rad_s": abs(capability["left_target_rad_s"]),
        "actual_wheel_speed_rad_s": abs(capability["left_actual_precontact_rad_s"]),
        "wheel_surface_speed_m_s": capability["surface_speed_m_s"],
        "actual_wheel_speed_rpm": abs(capability["left_actual_precontact_rad_s"]) * 60.0 / (2.0 * 3.141592653589793),
        "timestep_s": capability["timestep_s"],
        "exit_position_m": capability["exit_position_xyz_m"],
        "exit_velocity_vector_m_s": capability["exit_velocity_vector_m_s"],
        "exit_speed_m_s": capability["exit_speed_m_s"],
        "elevation_deg": capability["elevation_deg"],
        "spin_vector_rad_s": capability["spin_vector_rad_s"],
        "spin_rpm": capability["spin_equivalent_rpm"],
        "contact_duration_s": max(capability["contact_duration_s"].values()),
        "sampled_bilateral_duration_s": diagnostic["bilateral_contact"]["sampled_duration_s"],
        "contact_travel_m": diagnostic["bilateral_contact"]["ball_travel_distance_m"],
        "contact_arc_deg_per_wheel": abs(left["contact_arc_deg"]),
        "first_contact_position_m": diagnostic["bilateral_contact"]["ball_start_position_world_m"],
        "release_position_m": diagnostic["bilateral_contact"]["ball_end_position_world_m"],
        "maximum_compression_position_world_m": left["maximum_compression_position_world_m"],
        "maximum_compression_position_launcher_local_m": left["maximum_compression_position_launcher_local_m"],
        "normal_impulse_n_s_per_wheel": left["normal_impulse_n_s"],
        "tangential_impulse_n_s_per_wheel": left["tangential_impulse_n_s"],
        "mean_normal_force_n_per_wheel": left["mean_normal_force_n"],
        "mean_tangential_force_n_per_wheel": left["mean_tangential_force_n"],
        "peak_normal_force_n_per_wheel": left["peak_normal_force_n"],
        "peak_tangential_force_n_per_wheel": left["peak_tangential_force_n"],
        "maximum_diametral_compression_m": diagnostic["bilateral_contact"]["maximum_diametral_compression_m"],
        "loading_duration_s": left["loading_duration_s"],
        "unloading_duration_s": left["unloading_duration_s"],
        "near_coulomb_limit_fraction": left["near_coulomb_limit_fraction"],
        "mean_friction_utilization": left["mean_friction_utilization"],
        "mean_slip_velocity_m_s": left["mean_slip_velocity_m_s"],
        "mean_slip_ratio": left["mean_slip_ratio"],
        "wheel_energy_pre_j": energy["wheel_rotational_energy_pre_j"],
        "wheel_energy_loss_j": energy["wheel_rotational_energy_loss_j"],
        "postrelease_wheel_energy_fraction": energy["postrelease_wheel_energy_fraction"],
        "motor_work_during_contact_j": energy["motor_work_during_contact_j"],
        "ball_mechanical_energy_gain_j": energy["ball_mechanical_energy_gain_j"],
        "contact_dissipation_j": energy["contact_dissipation_j"],
        "energy_residual_fraction": energy["energy_residual_fraction"],
        "wheel_droop_percent": max(capability["left_droop_percent"], capability["right_droop_percent"]),
        "recovery_time_s": capability["recovery_time_s"],
        "peak_event_current_a": max(capability["motor_response"]["left_peak_event_current_a"], capability["motor_response"]["right_peak_event_current_a"]),
        "peak_event_torque_nm": max(capability["motor_response"]["left_peak_event_torque_nm"], capability["motor_response"]["right_peak_event_torque_nm"]),
        "estimated_peak_bus_voltage_v": capability["motor_response"]["estimated_peak_required_bus_voltage_v"],
        "successful_and_clear": capability["successful_launch"] and capability["post_release_geometry_clear"],
        "secondary_checks_passed": diagnostic["secondary_checks"]["passed"],
    }
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        telemetry = list(csv.DictReader(stream))
    return record, telemetry


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_campaign", type=Path)
    parser.add_argument("extended_campaign", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--summary-csv", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--telemetry-csv", type=Path, default=DEFAULT_TELEMETRY)
    args = parser.parse_args()

    protocol = read_json(PROTOCOL_PATH)
    baseline = protocol["authoritative_capability_baseline"]
    frozen = protocol["frozen_physics_hashes"]
    frozen_checks = {
        "capability_map": sha256(ROOT / baseline["map_path"]) == baseline["map_sha256"],
        "capability_protocol": sha256(ROOT / baseline["protocol_path"]) == baseline["protocol_sha256"],
        "ball_calibration_results": sha256(ROOT / "config/tennis_ball_compliance_calibration_results.json") == frozen["ball_calibration_results_sha256"],
        "ball_model": sha256(ROOT / "gazebo/models/tennis_ball_compliant/model.sdf") == frozen["ball_model_sha256"],
        "standalone_bench_xacro": sha256(ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro") == frozen["standalone_bench_xacro_sha256"],
        "launcher_module_xacro": sha256(ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro") == frozen["launcher_module_xacro_sha256"],
        "lower_exit_relief_mesh": sha256(ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_lower_panel_exit_clearance.stl") == frozen["lower_exit_relief_mesh_sha256"],
        "upper_exit_relief_mesh": sha256(ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_upper_panel_exit_clearance.stl") == frozen["upper_exit_relief_mesh_sha256"],
        "contact_plugin": sha256(ROOT / "ros2_ws/src/tennis_ball_contact_system/src/TennisBallContactSystem.cc") == frozen["contact_plugin_sha256"],
        "motor_plugin": sha256(ROOT / "ros2_ws/src/tennis_ball_contact_system/src/FlywheelCapabilityControlSystem.cc") == frozen["motor_plugin_sha256"],
    }
    if not all(frozen_checks.values()):
        raise RuntimeError("frozen inputs changed: " + ", ".join(key for key, value in frozen_checks.items() if not value))

    records: list[dict] = []
    telemetry_rows: list[dict] = []

    def add_case(path: Path, case_id: str, category: str) -> None:
        record, telemetry = load_case(path, case_id, category)
        records.append(record)
        for row in telemetry:
            telemetry_rows.append({
                "case_id": case_id,
                "category": category,
                "friction_coefficient": record["friction_coefficient"],
                "target_wheel_speed_rad_s": record["target_wheel_speed_rad_s"],
                "timestep_s": record["timestep_s"],
                **row,
            })

    for mu in MUS:
        for speed in SPEEDS:
            path = (baseline_case(args.baseline_campaign, mu, speed) if mu <= 0.9
                    else extended_case(args.extended_campaign, mu, speed))
            add_case(path, path.name, "friction_speed_matrix")
    for speed in (130, 140, 150):
        path = extended_case(args.extended_campaign, 3.0, speed)
        add_case(path, path.name, "adaptive_target_search")
    for mu in (5.0, 10.0):
        path = extended_case(args.extended_campaign, mu, 300)
        add_case(path, path.name, "ideal_traction_upper_bound")
    for mu, speed in ((2.5, 300), (3.0, 140), (3.0, 160), (10.0, 300)):
        for dt_name in ("0p5ms", "0p25ms"):
            path = args.extended_campaign / f"conv_mu_{label(mu)}_w{speed:03d}_dt_{dt_name}"
            add_case(path, path.name, "high_traction_timestep_convergence")

    by_id = {item["case_id"]: item for item in records}
    cap_limits = read_json(CAPABILITY_PROTOCOL_PATH)["numerical_convergence_acceptance_against_0p25ms"]
    convergence = {}
    for mu, speed in ((2.5, 300), (3.0, 140), (3.0, 160), (10.0, 300)):
        base_id = f"mu_{label(mu)}_w{speed:03d}"
        fine_id = f"conv_mu_{label(mu)}_w{speed:03d}_dt_0p25ms"
        raw_fine = read_json(args.extended_campaign / fine_id / "diagnostic_result.json")["capability"]
        comparisons = []
        for case_id in (base_id, f"conv_mu_{label(mu)}_w{speed:03d}_dt_0p5ms", fine_id):
            raw = read_json(args.extended_campaign / case_id / "diagnostic_result.json")["capability"]
            comparisons.append({"case_id": case_id, **convergence_comparison(raw, raw_fine, cap_limits)})
        convergence[f"mu_{label(mu)}_w{speed:03d}"] = {
            "comparisons_against_0p25ms": comparisons,
            "all_timesteps_passed": all(item["passed"] for item in comparisons),
            "fine_pair_passed": all(item["passed"] for item in comparisons[1:]),
        }

    matrix = [row for row in records if row["category"] == "friction_speed_matrix"]
    plateaus = {}
    for mu in MUS:
        rows_mu = [row for row in matrix if row["friction_coefficient"] == mu]
        r160 = next(row for row in rows_mu if row["target_wheel_speed_rad_s"] == 160)
        r300 = next(row for row in rows_mu if row["target_wheel_speed_rad_s"] == 300)
        delta = abs(r300["exit_speed_m_s"] - r160["exit_speed_m_s"])
        plateaus[label(mu)] = {
            "exit_speed_at_160_rad_s": r160["exit_speed_m_s"],
            "exit_speed_at_300_rad_s": r300["exit_speed_m_s"],
            "absolute_change_m_s": delta,
            "saturated_by_frozen_rule": delta <= protocol["root_cause_evidence_rules"]["saturation_reproduced_maximum_exit_speed_change_160_to_300_m_s"],
        }

    converged_14 = by_id["conv_mu_3p0_w160_dt_0p25ms"]
    upper = by_id["conv_mu_10p0_w300_dt_0p25ms"]
    fine_mu25 = by_id["conv_mu_2p5_w300_dt_0p25ms"]
    interpolated_mu14 = 2.5 + 0.5 * (14.0 - fine_mu25["exit_speed_m_s"]) / (converged_14["exit_speed_m_s"] - fine_mu25["exit_speed_m_s"])
    crossing_specs = {
        "10_m_s": (1.5, "mu_1p5_w120"),
        "12_m_s": (2.5, "mu_2p5_w120"),
        "14_m_s": (3.0, converged_14["case_id"]),
        "16_m_s": (5.0, "mu_5p0_w300"),
        "18_m_s": (5.0, "mu_5p0_w300"),
    }
    target_crossings = {}
    for target, (mu, case_id) in crossing_specs.items():
        target_crossings[target] = {
            "reached": True,
            "minimum_tested_mu": mu,
            "classification": ("DIAGNOSTIC_ONLY_NOT_PHYSICAL_CALIBRATION" if mu <= 3.0
                               else "IDEAL_TRACTION_UPPER_BOUND_NOT_A_PHYSICAL_LAUNCH_RESULT"),
            "operating_point": by_id[case_id],
        }
    target_crossings["14_m_s"]["estimated_diagnostic_mu_threshold"] = interpolated_mu14

    output = {
        "schema_version": 1,
        "generated_on": "2026-08-26",
        "scope": "frozen_standalone_flywheel_energy_transfer_root_cause",
        "decision": {
            "case": "CASE_B_GEOMETRY_HAS_KINEMATIC_POTENTIAL_BUT_REQUIRED_TRACTION_IS_NOT_PHYSICALLY_VALIDATED",
            "root_cause": "PRIMARY_TRACTION_LIMIT_WITH_COUPLED_FINITE_CONTACT_DURATION",
            "summary": "The frozen geometry exceeds 14 m/s only at diagnostic mu=3.0. Motor energy and torque remain available, but every target point remains at the Coulomb limit with substantial slip; interface-specific evidence does not validate mu=3.0 for tennis felt against a practical tread.",
        },
        "protocol": str(PROTOCOL_PATH.relative_to(ROOT)),
        "frozen_input_checks": frozen_checks,
        "campaign_counts": {
            "friction_speed_matrix": len(matrix),
            "adaptive_target_search": sum(row["category"] == "adaptive_target_search" for row in records),
            "ideal_upper_bound": sum(row["category"] == "ideal_traction_upper_bound" for row in records),
            "high_traction_timestep_cases": sum(row["category"] == "high_traction_timestep_convergence" for row in records),
            "total_compiled_cases": len(records),
        },
        "saturation_by_friction": plateaus,
        "target_crossings": target_crossings,
        "converged_14_m_s_point": converged_14,
        "ideal_high_traction_upper_bound": {**upper, "physical_launch_result": False},
        "high_traction_timestep_convergence": convergence,
        "physical_interface_evidence": {
            "validated_dynamic_tennis_felt_tread_coefficient": False,
            "required_diagnostic_coefficient_for_14_m_s": 3.0,
            "launcher_polypropylene_static_test_coefficient": 2.05,
            "court_surface_measured_range": "approximately 0.42-0.80; coarse P150 abrasive surface reported above 1.0 with felt damage",
            "conclusion": "MU_3P0_IS_OUTSIDE_AVAILABLE_INTERFACE_SPECIFIC_EVIDENCE_AND_MUST_NOT_BE_USED_AS_A_HARDWARE_PREDICTION",
            "sources": [
                {"title": "Wojcicki et al. 2011, Mathematical Analysis for a New Tennis Ball Launcher", "url": "https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-article-BPB2-0062-0016/c/httpwww_actawm_pb_edu_plvol5no4wojcickikuleszapucilowskien2010085.pdf", "use": "static cut-ball against polypropylene launcher-interface experiment; selected coefficient 2.05"},
                {"title": "Cross 2003, Measurements of horizontal and vertical speeds of tennis courts", "url": "https://www.researchgate.net/publication/225514950_Measurements_of_the_horizontal_and_vertical_speeds_of_tennis_courts", "use": "measured tennis-ball court coefficients and felt damage on coarse abrasive surface"},
                {"title": "ITF 2020 classified court surfaces", "url": "https://www.itftennis.com/media/2714/2020-itf-approved-tennis-balls-classified-court-surfaces-and-recognised-courts.pdf", "use": "court COF categories; high category starts at 0.71"},
                {"title": "US 6470873 three-wheel ball throwing machine", "url": "https://patents.justia.com/patent/6470873", "use": "candidate urethane/nitrile/butyl tread hardness guidance, not coefficient calibration"},
            ],
        },
        "root_cause_classification": {
            "TRACTION_LIMITED": True,
            "CONTACT_DURATION_LIMITED": True,
            "CONTACT_GEOMETRY_LIMITED": False,
            "COMPLIANCE_LIMITED": False,
            "MOTOR_TORQUE_LIMITED": False,
            "WHEEL_ENERGY_LIMITED": False,
            "NUMERICALLY_LIMITED": False,
            "COMBINED_LIMIT": True,
        },
        "design_gate_booleans": {
            "CURRENT_LAUNCHER_GEOMETRY_HAS_SUFFICIENT_KINEMATIC_POTENTIAL": True,
            "REALISTIC_TRACTION_INSUFFICIENT": True,
            "CURRENT_LAUNCHER_GEOMETRY_14M_S_CAPABILITY": True,
            "CURRENT_LAUNCHER_GEOMETRY_REMAINS_VIABLE": False,
            "PHYSICAL_TYRE_FRICTION_VALIDATED": False,
            "HIGHER_TRACTION_WHEEL_IS_PREFERRED_NEXT_STEP": True,
            "NIP_REDESIGN_REQUIRED": False,
            "MOTOR_CHANGE_REQUIRED": False,
            "LAUNCH_CONTACT_TIMESTEP_CONVERGED": convergence["mu_3p0_w160"]["all_timesteps_passed"],
            "LAUNCH_ENERGY_ACCOUNTING_VALIDATED": converged_14["secondary_checks_passed"],
            "POST_NIP_PATH_VALIDATED": converged_14["successful_and_clear"],
        },
        "final_classifications": {
            "EXIT_SPEED_SATURATION_REPRODUCED": all(item["saturated_by_frozen_rule"] for item in plateaus.values()),
            "PRIMARY_LIMIT_IDENTIFIED": True,
            "IDEAL_TRACTION_UPPER_BOUND_M_S": upper["exit_speed_m_s"],
            "DIAGNOSTIC_10_M_S_REACHED": True,
            "DIAGNOSTIC_12_M_S_REACHED": True,
            "DIAGNOSTIC_14_M_S_REACHED": True,
            "DIAGNOSTIC_16_M_S_REACHED": True,
            "DIAGNOSTIC_18_M_S_REACHED": True,
            "MINIMUM_TESTED_MU_FOR_14_M_S": 3.0,
            "PHYSICAL_TYRE_FRICTION_VALIDATED": False,
            "CURRENT_LAUNCHER_GEOMETRY_REMAINS_VIABLE": False,
            "CURRENT_LAUNCHER_GEOMETRY_14M_S_CAPABILITY": True,
            "HIGHER_TRACTION_WHEEL_IS_PREFERRED_NEXT_STEP": True,
            "NIP_REDESIGN_REQUIRED": False,
            "MOTOR_CHANGE_REQUIRED": False,
            "POST_FLYWHEEL_PATH_NONCONTACTING": converged_14["successful_and_clear"],
            "LAUNCH_CONTACT_TIMESTEP_CONVERGED": convergence["mu_3p0_w160"]["all_timesteps_passed"],
            "LAUNCH_ENERGY_ACCOUNTING_VALIDATED": converged_14["secondary_checks_passed"],
            "PHYSICAL_FLYWHEEL_WHEEL_VALIDATED": False,
            "PHYSICAL_HARDWARE_PENDING": True,
        },
        "recommended_next_gate": "Measure dynamic tennis-felt/candidate-tread traction and durability on a coupon or instrumented roller rig before selecting a tread or reopening nip geometry.",
        "operating_points": records,
        "artifacts": {
            "case_summary_csv": str(args.summary_csv.relative_to(ROOT)),
            "contact_telemetry_csv": str(args.telemetry_csv.relative_to(ROOT)),
        },
    }

    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_csv(args.summary_csv, records)
    write_csv(args.telemetry_csv, telemetry_rows)
    print(args.output)
    print(args.summary_csv)
    print(args.telemetry_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
