import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULT_PATH = ROOT / "config/flywheel_energy_transfer_root_cause.json"
SUMMARY_PATH = ROOT / "docs/mechanism/flywheel-energy-transfer-case-summary.csv"
TELEMETRY_PATH = ROOT / "docs/mechanism/flywheel-energy-transfer-contact-telemetry.csv"
REPORT_PATH = ROOT / "docs/mechanism/flywheel-energy-transfer-root-cause-report.md"


def result():
    return json.loads(RESULT_PATH.read_text(encoding="utf-8"))


def test_frozen_inputs_and_campaign_are_complete():
    data = result()
    assert all(data["frozen_input_checks"].values())
    assert data["campaign_counts"] == {
        "friction_speed_matrix": 56,
        "adaptive_target_search": 3,
        "ideal_upper_bound": 2,
        "high_traction_timestep_cases": 8,
        "total_compiled_cases": 69,
    }
    assert all(item["saturated_by_frozen_rule"]
               for item in data["saturation_by_friction"].values())


def test_14_m_s_point_is_converged_clear_and_inside_limits():
    data = result()
    point = data["converged_14_m_s_point"]
    assert point["case_id"] == "conv_mu_3p0_w160_dt_0p25ms"
    assert point["friction_coefficient"] == 3.0
    assert point["timestep_s"] == 0.00025
    assert point["exit_speed_m_s"] >= 14.0
    assert point["near_coulomb_limit_fraction"] >= 0.95
    assert point["mean_slip_velocity_m_s"] > 0.0
    assert point["peak_event_current_a"] <= 20.0
    assert point["estimated_peak_bus_voltage_v"] <= 12.8
    assert abs(point["energy_residual_fraction"]) <= 0.02
    assert point["successful_and_clear"]
    assert point["secondary_checks_passed"]
    convergence = data["high_traction_timestep_convergence"]["mu_3p0_w160"]
    assert convergence["all_timesteps_passed"]


def test_root_cause_and_physical_gate_are_conservative():
    data = result()
    cause = data["root_cause_classification"]
    assert cause["TRACTION_LIMITED"]
    assert cause["CONTACT_DURATION_LIMITED"]
    assert not cause["MOTOR_TORQUE_LIMITED"]
    assert not cause["WHEEL_ENERGY_LIMITED"]
    assert not cause["NUMERICALLY_LIMITED"]
    gate = data["design_gate_booleans"]
    assert gate["CURRENT_LAUNCHER_GEOMETRY_HAS_SUFFICIENT_KINEMATIC_POTENTIAL"]
    assert gate["REALISTIC_TRACTION_INSUFFICIENT"]
    assert gate["CURRENT_LAUNCHER_GEOMETRY_14M_S_CAPABILITY"]
    assert not gate["CURRENT_LAUNCHER_GEOMETRY_REMAINS_VIABLE"]
    assert not gate["PHYSICAL_TYRE_FRICTION_VALIDATED"]
    assert not gate["MOTOR_CHANGE_REQUIRED"]


def test_target_crossings_and_upper_bound_are_labelled_diagnostic():
    data = result()
    crossings = data["target_crossings"]
    assert all(item["reached"] for item in crossings.values())
    assert crossings["14_m_s"]["minimum_tested_mu"] == 3.0
    assert crossings["14_m_s"]["estimated_diagnostic_mu_threshold"] < 3.0
    assert crossings["16_m_s"]["classification"].startswith("IDEAL_TRACTION")
    upper = data["ideal_high_traction_upper_bound"]
    assert upper["exit_speed_m_s"] > 18.0
    assert not upper["physical_launch_result"]
    assert not data["physical_interface_evidence"][
        "validated_dynamic_tennis_felt_tread_coefficient"]


def test_csv_and_plot_artifacts_are_complete():
    with SUMMARY_PATH.open(newline="", encoding="utf-8") as stream:
        summary = list(csv.DictReader(stream))
    with TELEMETRY_PATH.open(newline="", encoding="utf-8") as stream:
        telemetry = list(csv.DictReader(stream))
    assert len(summary) == 69
    assert len(telemetry) == 1730
    assert {row["side"] for row in telemetry} == {"left", "right"}
    assert {"applied_motor_torque_nm", "wheel_speed_droop_percent",
            "wheel_rotational_energy_j"}.issubset(telemetry[0])
    assert any(row["case_id"] == "conv_mu_3p0_w160_dt_0p25ms" for row in telemetry)
    plots = sorted((ROOT / "docs/images").glob("flywheel-energy-transfer-*.png"))
    assert len(plots) == 14
    assert all(path.stat().st_size > 20_000 for path in plots)
    report = REPORT_PATH.read_text(encoding="utf-8")
    for index in range(1, 15):
        assert f"flywheel-energy-transfer-{index:02d}-" in report
