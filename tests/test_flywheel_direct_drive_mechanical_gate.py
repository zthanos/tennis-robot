import json
import math
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "config/flywheel_launcher_direct_drive_mechanical_gate.json"


def load_result():
    return json.loads(RESULT.read_text(encoding="utf-8"))


def test_gate_classifications_are_explicit_and_conservative():
    classes = load_result()["classifications"]
    assert classes["D5065_DIRECT_PANEL_MOUNT_GEOMETRICALLY_VALID"] is True
    assert classes["D5065_DIRECT_PANEL_MOUNT_STRUCTURALLY_SCREENED"] is True
    assert all(
        classes[name] is False
        for name in (
            "FLYWHEEL_PANEL_CUTOUT_DEFINED",
            "FLYWHEEL_DIRECT_DRIVE_HUB_DEFINED",
            "FLYWHEEL_SHAFT_ENGAGEMENT_VALIDATED",
            "FLYWHEEL_AXIAL_RETENTION_DEFINED",
            "FLYWHEEL_ROTATING_MASS_DEFINED",
            "FLYWHEEL_ROTATING_INERTIA_DEFINED",
            "FLYWHEEL_MECHANICAL_GATE_A_PASSED",
        )
    )


def test_direct_mount_and_axial_stack_match_accepted_geometry():
    data = load_result()
    mount = data["direct_panel_mount"]
    assert mount["motor_centres_launcher_local_m"] == [[0.0, 0.129, 0.047], [0.0, -0.129, 0.047]]
    assert mount["shaft_axes_launcher_local"] == [[0.0, 0.0, -1.0], [0.0, 0.0, -1.0]]
    assert math.isclose(mount["motor_body_edge_clearance_m"], 0.003, abs_tol=1e-12)
    assert math.isclose(mount["mount_hole_edge_ligament_m"], 0.011, abs_tol=1e-12)

    stack = data["axial_stack_launcher_local_z_m"]
    assert math.isclose(stack["shaft_projection_beyond_panel_inside_m"], 0.022, abs_tol=1e-12)
    assert math.isclose(stack["shaft_projection_inside_wheel_envelope_m"], 0.008, abs_tol=1e-12)
    assert math.isclose(stack["panel_inside_to_flywheel_outer_gap_m"], 0.014, abs_tol=1e-12)
    assert stack["hub_start"] is None
    assert stack["hub_end"] is None
    assert stack["axial_retention_hardware"] is None


def test_cutout_cases_are_sensitivity_only():
    cutout = load_result()["panel_cutout_screen"]
    assert cutout["final_cutout_defined"] is False
    expected_mm = {
        "diameter_10_mm": 8.0,
        "diameter_16_mm": 5.0,
        "diameter_18_mm": 4.0,
        "diameter_20_mm": 3.0,
    }
    for key, ligament_mm in expected_mm.items():
        assert math.isclose(cutout["central_opening_to_mount_hole_edge_ligament_m"][key] * 1000, ligament_mm, abs_tol=1e-9)


def test_structural_screen_is_reproducible_but_not_validation():
    screen = load_result()["structural_screen"]
    assert screen["status"] == "SCREEN_PASS_WITH_ASSUMPTIONS_NOT_PHYSICAL_VALIDATION"
    assert math.isclose(screen["derived"]["motor_torque_at_manufacturer_85a_3s_nm"], 2.635, abs_tol=1e-12)
    assert math.isclose(screen["derived"]["worst_case_bolt_tension_n"], 388.80299855870146, rel_tol=1e-12)
    assert screen["provisional_inputs"]["hub_mass_kg"] is None
    assert screen["vibration_resonance"].startswith("UNRESOLVED")


def test_hub_is_audited_not_selected_and_inertia_is_not_promoted():
    data = load_result()
    assert data["purchased_hub_audit"]["selected"] is None
    assert len(data["purchased_hub_audit"]["candidates"]) == 2
    rotating = data["rotating_assembly"]
    assert rotating["flywheel_mass_kg"] is None
    assert rotating["flywheel_inertia_kg_m2"] is None
    assert rotating["hub_inertia_kg_m2"] is None
    assert rotating["motor_rotor_inertia_kg_m2"] is None
    assert rotating["provisional_values_accepted_for_capability_simulation"] is False


def test_machine_evaluator_passes_for_expected_stop_state():
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/evaluate_flywheel_direct_drive_mechanical_gate.py")],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
