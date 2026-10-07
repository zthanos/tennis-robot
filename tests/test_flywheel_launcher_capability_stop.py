import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAPABILITY_MAP = ROOT / "config/flywheel_launcher_capability_map.json"
PROTOCOL = ROOT / "config/flywheel_launcher_capability_protocol.json"
REPORT = ROOT / "docs/mechanism/flywheel-launcher-capability-validation-report.md"
BALL_RESULTS = ROOT / "config/tennis_ball_compliance_calibration_results.json"
BALL_MODEL = ROOT / "gazebo/models/tennis_ball_compliant/model.sdf"
RUNNER = ROOT / "scripts/run_flywheel_capability_case.py"


def load_map():
    return json.loads(CAPABILITY_MAP.read_text(encoding="utf-8"))


def test_capability_campaign_resumes_after_corridor_pass_and_completes():
    data = load_map()
    assert data["decision"]["status"] == "CAPABILITY_MAP_COMPLETE_WITH_PHYSICAL_MODEL_LIMITATIONS"
    assert data["decision"]["stop_condition_encountered"] is False
    assert data["regression_reference"]["passed"] is True
    assert len(data["symmetric_operating_points"]) == 21
    assert len(data["differential_operating_points"]) == 4
    assert all(row["validity"]["successful_launch"] for row in data["symmetric_operating_points"])
    assert all(row["validity"]["successful_launch"] for row in data["differential_operating_points"])
    assert data["classifications"]["POST_FLYWHEEL_PATH_NONCONTACTING"] is True


def test_ball_calibration_is_frozen_and_friction_is_bounded_sensitivity_only():
    data = load_map()
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    digest = hashlib.sha256(BALL_RESULTS.read_bytes()).hexdigest()
    assert digest == protocol["immutable_inputs"]["ball_calibration_results"]["sha256"]
    assert all(data["frozen_input_checks"].values())
    assert data["contact_model_status"]["launcher_results_used_for_ball_refit"] is False
    assert data["contact_model_status"]["tyre_friction_physically_calibrated"] is False
    assert data["contact_model_status"]["friction_sensitivity_coefficients"] == [0.3, 0.6, 0.9]
    assert "<friction_coefficient>" not in BALL_MODEL.read_text(encoding="utf-8")
    assert {row["tyre_friction_assumption"]["coefficient"]
            for row in data["symmetric_operating_points"]} == {0.3, 0.6, 0.9}


def test_runner_anchors_only_bench_root_and_prevents_native_tyre_double_count():
    source = RUNNER.read_text(encoding="utf-8")
    assert '"bench_world_anchor"' in source
    assert 'ET.SubElement(anchor, "parent").text = "world"' in source
    assert 'ET.SubElement(anchor, "child").text = "launcher_bench_datum_link"' in source
    assert 'tyre_token = f"flywheel_{side}_col_collision"' in source
    assert 'if tyre_token in collision.attrib.get("name", ""):' in source
    assert source.count("link.remove(collision)") == 1


def test_timestep_energy_and_repeatability_gates_pass():
    data = load_map()
    convergence = data["timestep_convergence"]
    assert convergence["acceptance_was_frozen_before_results"] is True
    assert convergence["passed"] is True
    assert all(condition["passed"] for condition in convergence["conditions"].values())
    assert all(
        comparison["passed"]
        for condition in convergence["conditions"].values()
        for comparison in condition["comparisons"]
    )
    assert data["classifications"]["LAUNCH_ENERGY_ACCOUNTING_VALIDATED"] is True
    repeatability = data["repeatability"]
    assert repeatability["deterministic_simulation_repeatability_only"] is True
    for condition in repeatability["conditions"].values():
        assert condition["executed_trials"] == 3
        for metric in ("exit_speed_m_s", "elevation_deg", "azimuth_deg", "spin_rpm",
                       "first_bounce_x_m", "first_bounce_y_m"):
            assert condition[metric]["population_stddev"] == 0.0


def test_operating_points_include_required_capability_schema():
    required = {
        "mechanical_pitch_deg", "left_target_rad_s", "right_target_rad_s",
        "left_actual_precontact_rad_s", "right_actual_precontact_rad_s",
        "wheel_surface_speed_m_s", "exit_position_m", "exit_velocity_vector_m_s",
        "exit_speed_m_s", "elevation_deg", "azimuth_deg", "spin_vector_rad_s",
        "spin_rpm", "maximum_ball_compression_m", "left_peak_normal_force_n",
        "right_peak_normal_force_n", "left_rpm_droop", "right_rpm_droop",
        "recovery_time_s", "apex_height_m", "first_bounce_xyz_m",
        "first_bounce_time_s", "energy_accounting", "tyre_friction_assumption",
        "wheel_mass_assumption", "wheel_inertia_assumption", "timestep_s", "validity",
    }
    for point in load_map()["symmetric_operating_points"]:
        assert required <= set(point)


def test_speed_targets_are_classified_from_executed_points_not_extrapolation():
    data = load_map()
    targets = data["capability_targets"]
    assert set(targets) == {"12_m_s", "14_m_s", "16_m_s", "18_m_s"}
    assert data["measured_envelope"]["maximum_exit_speed_m_s"] < 12.0
    for item in targets.values():
        assert item["reachable"] is False
        assert item["status"] == "EXECUTED_ENVELOPE_DOES_NOT_REACH_TARGET"
        assert item["nearest_executed_operating_point"] is not None


def test_final_classifications_are_conservative_and_reported():
    data = load_map()
    classes = data["classifications"]
    assert classes["STANDALONE_LAUNCHER_CAPABILITY_BENCH_VALID"] is True
    assert classes["BALL_EXIT_STATE_VALIDATED"] is True
    assert classes["LAUNCH_SPIN_NUMERICALLY_CHARACTERIZED"] is True
    assert classes["LAUNCH_SPIN_VALIDATED"] is False
    assert classes["TANGENTIAL_CONTACT_LAUNCH_VALIDATED"] is False
    assert classes["COURT_TRAJECTORY_MODEL_VALIDATED"] is False
    assert classes["BALL_LAUNCH_PHYSICS_VALIDATED_IN_SIM"] is False
    assert classes["PHYSICAL_FLYWHEEL_WHEEL_VALIDATED"] is False
    assert classes["PHYSICAL_HARDWARE_PENDING"] is True
    report = REPORT.read_text(encoding="utf-8")
    for name, value in classes.items():
        assert f"{name} = {str(value).lower()}" in report
