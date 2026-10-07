import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.run_tennis_ball_calibration import run, simulate_platen, simulate_rebound
from scripts.tennis_ball_contact_model import (
    CompliantNormalModel,
    ContactState,
    NormalParameters,
    contact_wrench,
    sphere_finite_cylinder_contact,
)


def test_zero_or_negative_penetration_has_zero_force():
    model = CompliantNormalModel()
    assert model.evaluate(0.0, 1.0, 0.0).force_n == 0.0
    assert model.evaluate(-0.001, 1.0, 0.0).force_n == 0.0


def test_loading_force_increases_and_guard_rejects_extrapolation():
    model = CompliantNormalModel()
    forces = [model.evaluate(delta, 0.0, delta).force_n for delta in (0.001, 0.004, 0.008, 0.016)]
    assert forces == sorted(forces)
    assert all(force > 0.0 for force in forces)
    with pytest.raises(ValueError, match="exceeds calibrated guard"):
        model.evaluate(0.036, 0.0, 0.036)


def test_unloading_hysteresis_and_no_tensile_force():
    p = NormalParameters()
    model = CompliantNormalModel(p)
    maximum = 0.02
    delta = 0.01
    loading = model.evaluate(delta, 0.0, maximum)
    unloading = model.evaluate(delta, -0.1, maximum)
    assert unloading.elastic_force_n < loading.elastic_force_n
    assert unloading.force_n < unloading.elastic_force_n
    assert model.evaluate(0.0001, -20.0, maximum).force_n == 0.0


def test_state_separates_without_tensile_reengagement():
    model = CompliantNormalModel()
    state = ContactState()
    model.step_state(state, 0.01, 1.0)
    sample = model.step_state(state, 0.0001, -20.0)
    assert sample.force_n == 0.0
    assert state.separated is True
    assert model.step_state(state, 0.00005, -1.0).force_n == 0.0


def test_finite_cylinder_side_cap_and_edge_regions():
    common = dict(
        sphere_radius_m=0.033,
        cylinder_center=(0.0, 0.0, 0.0),
        cylinder_axis=(0.0, 0.0, 1.0),
        cylinder_radius_m=0.1,
        cylinder_half_width_m=0.025,
    )
    side = sphere_finite_cylinder_contact(sphere_center=(0.129, 0.0, 0.0), **common)
    assert side.region == "side" and side.active
    assert side.normal_world == pytest.approx((1.0, 0.0, 0.0))
    assert side.compression_m == pytest.approx(0.004)

    cap = sphere_finite_cylinder_contact(sphere_center=(0.0, 0.0, 0.050), **common)
    assert cap.region == "cap" and cap.active
    assert cap.normal_world == pytest.approx((0.0, 0.0, 1.0))
    assert cap.compression_m == pytest.approx(0.008)

    edge = sphere_finite_cylinder_contact(sphere_center=(0.12, 0.0, 0.045), **common)
    assert edge.region == "edge" and edge.active
    assert edge.normal_world[0] > 0.0 and edge.normal_world[2] > 0.0
    assert math.isclose(math.sqrt(sum(x*x for x in edge.normal_world)), 1.0)


def test_bilateral_central_contact_is_symmetric():
    ball = (0.0, 0.0, 0.0)
    contacts = [
        sphere_finite_cylinder_contact(
            sphere_center=ball,
            sphere_radius_m=0.033,
            cylinder_center=(0.0, y, 0.0),
            cylinder_axis=(0.0, 0.0, 1.0),
            cylinder_radius_m=0.1,
            cylinder_half_width_m=0.025,
        )
        for y in (0.129, -0.129)
    ]
    assert contacts[0].compression_m == pytest.approx(0.004)
    assert contacts[1].compression_m == pytest.approx(0.004)
    assert contacts[0].normal_world == pytest.approx(tuple(-x for x in contacts[1].normal_world))
    model = CompliantNormalModel()
    forces = [model.evaluate(contact.compression_m, 0.0, contact.compression_m).force_n for contact in contacts]
    assert forces[0] == pytest.approx(forces[1], rel=0, abs=1e-12)


def test_contact_wrench_exposes_equal_opposite_reaction_and_torque():
    geometry = sphere_finite_cylinder_contact(
        sphere_center=(0.129, 0.0, 0.0), sphere_radius_m=0.033,
        cylinder_center=(0.0, 0.0, 0.0), cylinder_axis=(0.0, 0.0, 1.0),
        cylinder_radius_m=0.1, cylinder_half_width_m=0.025,
    )
    sample = CompliantNormalModel().evaluate(geometry.compression_m, 0.0, geometry.compression_m)
    wrench = contact_wrench(
        geometry=geometry, force_sample=sample,
        ball_center=(0.129, 0.0, 0.0), ball_linear_velocity=(0.0, 1.0, 0.0), ball_angular_velocity=(0.0, 0.0, 0.0),
        wheel_center=(0.0, 0.0, 0.0), wheel_linear_velocity=(0.0, 0.0, 0.0), wheel_angular_velocity=(0.0, 0.0, 0.0),
        friction_coefficient=0.5,
    )
    assert wrench.ball_force_world_n == pytest.approx(tuple(-x for x in wrench.wheel_force_world_n))
    assert abs(wrench.tangential_force_world_n[1]) > 0.0
    assert abs(wrench.ball_torque_world_nm[2]) > 0.0
    assert abs(wrench.wheel_torque_world_nm[2]) > 0.0
    assert wrench.friction_limit_n == pytest.approx(0.5 * wrench.normal_force_n)


def test_unknown_tyre_friction_produces_no_silent_tangential_force():
    geometry = sphere_finite_cylinder_contact(
        sphere_center=(0.129, 0.0, 0.0), sphere_radius_m=0.033,
        cylinder_center=(0.0, 0.0, 0.0), cylinder_axis=(0.0, 0.0, 1.0),
        cylinder_radius_m=0.1, cylinder_half_width_m=0.025,
    )
    sample = CompliantNormalModel().evaluate(geometry.compression_m, 0.0, geometry.compression_m)
    wrench = contact_wrench(
        geometry=geometry, force_sample=sample,
        ball_center=(0.129, 0.0, 0.0), ball_linear_velocity=(0.0, 1.0, 0.0), ball_angular_velocity=(0.0, 0.0, 0.0),
        wheel_center=(0.0, 0.0, 0.0), wheel_linear_velocity=(0.0, 0.0, 0.0), wheel_angular_velocity=(0.0, 0.0, 0.0),
        friction_coefficient=None,
    )
    assert wrench.tangential_force_world_n == (0.0, 0.0, 0.0)
    assert wrench.friction_limit_n is None


def test_calibration_cases_are_deterministic():
    p = NormalParameters()
    first, _ = simulate_rebound(0.00025, p)
    second, _ = simulate_rebound(0.00025, p)
    assert asdict(first) == asdict(second)
    platen_a, _ = simulate_platen(0.0005, p)
    platen_b, _ = simulate_platen(0.0005, p)
    assert asdict(platen_a) == asdict(platen_b)


def test_shell_inertia_choice_and_sensitivity_are_independent_of_launcher(tmp_path):
    result_path = tmp_path / "result.json"
    result = run(tmp_path / "plots", result_path)
    physical = result["physical_parameters"]
    shell = (2.0 / 3.0) * 0.058 * 0.033**2
    solid = (2.0 / 5.0) * 0.058 * 0.033**2
    assert physical["shell_inertia_kg_m2"] == pytest.approx(shell)
    assert physical["solid_sphere_sensitivity_inertia_kg_m2"] == pytest.approx(solid)
    assert shell / solid == pytest.approx(5.0 / 3.0)
    assert result["calibration_provenance"]["launcher_results_used_for_fit"] is False


def test_generated_calibration_gate_passes_and_keeps_friction_pending(tmp_path):
    result = run(tmp_path / "plots", tmp_path / "result.json")
    classifications = result["classifications"]
    assert classifications["BALL_COMPLIANCE_MODEL_IMPLEMENTED"] is True
    assert classifications["BALL_DEFORMATION_CALIBRATED_TO_ITF"] is True
    assert classifications["BALL_REBOUND_CALIBRATED_TO_ITF"] is True
    assert classifications["BALL_LOADING_UNLOADING_HYSTERESIS_VALIDATED"] is True
    assert classifications["TIME_STEP_CONVERGENCE_VALIDATED"] is True
    assert classifications["ENERGY_ACCOUNTING_VALIDATED"] is True
    assert classifications["FINITE_CYLINDER_CONTACT_VALIDATED"] is True
    assert classifications["BILATERAL_CONTACT_SYMMETRY_VALIDATED"] is True
    assert classifications["LAUNCHER_TYRE_FRICTION_CALIBRATION_PENDING"] is True
    assert classifications["LAUNCHER_PHYSICS_TRIALS_AUTHORIZED"] is True


def test_gazebo_plugin_coefficients_match_machine_readable_calibration():
    result = json.loads(
        (ROOT / "config/tennis_ball_compliance_calibration_results.json").read_text()
    )
    model_text = (ROOT / "gazebo/models/tennis_ball_compliant/model.sdf").read_text()
    calibrated = result["calibrated_parameters"]
    assert f"<loading_stiffness>{calibrated['loading_stiffness_n_m_pow']}</loading_stiffness>" in model_text
    assert f"<dynamic_damping>{calibrated['dynamic_damping_n_s_m_pow']}</dynamic_damping>" in model_text
    assert "<friction_coefficient>" not in model_text
    assert "<category_bitmask>4</category_bitmask><collide_bitmask>8</collide_bitmask>" in model_text
