"""Acceptance tests for the standalone intake handoff study (schema 2).

Schema 2 re-derives the study on the AUTHORITATIVE Option A handoff ramp. The
flat-ground revision (schema 1) and every number downstream of its exit vector
are withdrawn, so these tests pin the ramp-based invariants instead.
"""

import csv
import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402

RESULT = ROOT / "config" / "standalone_intake_handoff_capability.json"
TRIALS = ROOT / "docs" / "mechanism" / "standalone-intake-handoff-trials.csv"
TELEMETRY = ROOT / "docs" / "mechanism" / "standalone-intake-handoff-telemetry.csv"
REPORT = ROOT / "docs" / "mechanism" / "standalone-intake-handoff-capability-report.md"
SCRIPT = ROOT / "scripts" / "run_standalone_intake_handoff_study.py"


def _result():
    return json.loads(RESULT.read_text(encoding="utf-8"))


def test_study_freezes_corrected_fixed_geometry_without_carriage():
    result = _result()
    geometry = result["geometry"]

    assert result["schema_version"] == 2
    assert "flat" in result["supersedes"]
    assert geometry["geometry_frozen"] is True
    assert geometry["legacy_translating_carriage_used"] is False
    assert geometry["wheel_centres_m"] == {
        "left": [0.0, 0.09, 0.07],
        "right": [0.0, -0.09, 0.07],
    }
    assert geometry["wheel_axis"] == pytest.approx(
        [math.sin(math.radians(35)), 0.0, math.cos(math.radians(35))]
    )
    assert geometry["wheel_diameter_m"] == pytest.approx(0.124)
    assert geometry["wheel_width_m"] == pytest.approx(0.073)
    assert geometry["bridge_aabb_m"]["z"] == [0.19, 0.208]
    # The two frozen sources disagree on the bridge height; that is reported.
    assert geometry["bridge_under_z_source_conflict_m"]["used"] == 0.190

    source = SCRIPT.read_text(encoding="utf-8")
    assert "sphere_finite_cylinder_contact" in source
    assert "carriage_joint" not in source


def test_measurement_model_runs_on_the_authoritative_ramp():
    ground = _result()["ground_profile"]

    assert ground["classification"] == "GROUND_PROFILE_IS_AUTHORITATIVE_RAMP"
    assert ground["law"] == "smoothstep t*t*(3-2*t)"
    assert ground["front_x_cad_m"] == pytest.approx(geom.RAMP_FRONT_X_M)
    assert ground["rear_x_cad_m"] == pytest.approx(geom.RAMP_REAR_X_M)
    assert ground["rear_z_m"] - ground["front_z_m"] == pytest.approx(0.0335)
    assert ground["maximum_slope_deg"] == pytest.approx(26.68, abs=0.02)
    assert ground["z_at_nip_mid_m"] == pytest.approx(0.01825, abs=1e-5)
    assert ground["ball_centre_lateral_bound_m"] == pytest.approx(0.057)
    # The physical justification: the ball cannot coast up the rise.
    assert ground["available_coast_climb_m"] < ground["required_climb_m"] / 3.0
    assert ground["court_moves_in_robot_frame"] is True


def test_bounded_model_does_not_claim_physical_calibration():
    result = _result()
    c = result["classifications"]

    assert result["evidence_classification"] == "SIMULATION_BOUNDED"
    assert result["sensitivity_bounds"]["bounds_physically_calibrated"] is False
    assert result["sensitivity_bounds"]["ball_ramp_friction_coefficients"] == list(
        geom.BALL_RAMP_FRICTION_BOUNDS
    )
    assert c["INTAKE_TYRE_COMPLIANCE_PHYSICALLY_VALIDATED"] is False
    assert c["INTAKE_TREAD_FRICTION_PHYSICALLY_VALIDATED"] is False
    assert c["BALL_RAMP_FRICTION_PHYSICALLY_VALIDATED"] is False
    assert c["PHYSICAL_HARDWARE_PENDING"] is True
    assert c["FULL_HANDOFF_TRAJECTORY_PHYSICS_VALIDATED"] is False
    assert result["trajectory_physics"]["aerodynamic_drag"] is False
    assert result["trajectory_physics"]["magnus_effect"] is False
    # D4: Gazebo droop/torque/current must never be presented as measurable.
    assert result["motor_model"]["droop_torque_current_measurable_in_gazebo"] is False


def test_corrected_model_reports_the_plough_stop_condition():
    result = _result()
    c = result["classifications"]
    approach = result["approach_speed"]
    sweep = approach["sweep"]
    threshold = approach["capture_threshold"]

    # A centred ball is ploughed ahead at every speed the collection route
    # commands. This is task section 14 stop condition 1 and must be reported,
    # never tuned away.
    assert sweep["ROUTE_NOMINAL"]["centred_mechanism_captured"] == 0
    assert sweep["ROUTE_MAX"]["centred_mechanism_captured"] == 0
    assert c["CENTRED_BALL_PLOUGHED_AHEAD_AT_ROUTE_SPEEDS"] is True
    assert c["INTAKE_CAPTURES_CENTRED_BALL_AT_ROUTE_NOMINAL_APPROACH"] is False
    assert threshold["route_nominal_speed_m_s"] == pytest.approx(0.35)
    assert threshold["MINIMUM_APPROACH_SPEED_WITH_ANY_CAPTURE_M_S"] > threshold["route_max_speed_m_s"]
    assert (
        threshold["MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S"]
        <= approach["envelope_speed_m_s"]
    )
    assert result["ramp_dynamics"]["ploughed_ahead_cases"] > 0
    assert any("ploughed" in reason.lower() for reason in result["stop_reasons"])


def test_exit_envelope_is_labelled_conditional_and_ramp_effect_is_measured():
    result = _result()
    approach = result["approach_speed"]
    effect = result["ramp_effect"]["at_envelope_approach_speed"]

    assert result["classifications"][
        "EXIT_ENVELOPE_CONDITIONAL_ON_APPROACH_SPEED_M_S"
    ] == approach["envelope_speed_m_s"]
    assert result["campaign"]["matrix_approach_speed_m_s"] == approach["envelope_speed_m_s"]
    # Both opposing consequences of the ramp are present: the exit rises and the
    # elevation falls. The report states the measured net effect.
    assert effect["delta"]["exit_z_m"] > 0.0
    assert effect["delta"]["exit_elevation_deg"] < 0.0
    assert effect["flat_ground_datum"]["outcome"] == "CAPTURED"
    assert effect["authoritative_ramp"]["outcome"] == "CAPTURED"


def test_model_validity_gates_are_separated_from_mechanism_failures():
    campaign = _result()["campaign"]

    assert set(campaign["mechanism_outcomes"]) <= {
        "CAPTURED", "PLOUGHED_AHEAD", "STALLED_IN_NIP", "NO_BILATERAL_CONTACT",
        "REJECTED_FORWARD", "NOT_TRANSPORTED",
    }
    gates = campaign["model_validity_gate_rejections"]
    assert gates["bridge_collision"] == 0
    separated = campaign["gate_rejected_but_mechanically_captured"]
    assert separated["count"] == gates["tyre_deflection_above_5mm_model_validity_bound"]
    assert "NOT mechanism failures" in separated["note"]


def test_basket_on_base_windows_are_population_qualified():
    basket = _result()["basket_on_base"]
    windows = basket["rim_windows"]

    assert basket["chassis_cut_out"] is False
    assert basket["floor_plane_z_m"] == pytest.approx(0.052)
    assert windows["headline_population"] in windows
    assert windows["MAX_BASKET_RIM_HEIGHT_ON_BASE_M"] is not None
    for population in ("centred_LOW", "centred_NOMINAL", "centred_HIGH"):
        assert population in windows
        assert windows[population]["population_size"] > 0
    # The flat-ground reference is present ONLY as a withdrawn comparison.
    assert "INVALID_FLAT_GROUND" in basket["flat_ground_reference_withdrawn"]["status"]
    assert "NOT resolved" in basket["low_rim_versus_rebound_conflict"]["statement"]


def test_gazebo_owned_questions_are_never_answered_by_the_solver():
    c = _result()["classifications"]
    gazebo = _result()["gazebo_campaign"]

    for key in ("INTAKE_CAPTURE_HALF_WIDTH_M", "INTAKE_MAX_LATERAL_ENTRY_VELOCITY_M_S",
                "CHEEK_THROAT_ADEQUATE", "BALL_CLIMBS_RAMP_IN_SIM"):
        value = c[key]
        assert value == "PENDING_GAZEBO_CAMPAIGN" or gazebo.get("status") == "COMPLETE"


def test_telemetry_plots_and_numerical_convergence_are_present():
    result = _result()
    with TRIALS.open(newline="", encoding="utf-8") as handle:
        trials = list(csv.DictReader(handle))
    with TELEMETRY.open(newline="", encoding="utf-8") as handle:
        telemetry = list(csv.DictReader(handle))

    assert len(trials) == result["campaign"]["trial_count"]
    assert len(telemetry) > 500
    assert {"flat", "ramp"} <= {row["ground_profile"] for row in telemetry}
    assert REPORT.is_file()
    assert len(result["plots"]) == 17
    assert all((ROOT / path).is_file() for path in result["plots"])
    convergence = result["campaign"]["timestep_convergence"]
    assert convergence["exit_speed_relative_spread"] < 0.002
    assert all(case["capture_valid"] for case in convergence["cases"])
