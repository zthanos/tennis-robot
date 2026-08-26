import csv
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GATE = ROOT / "config/flywheel_launcher_provisional_gate_a.json"
CSV_PATH = ROOT / "docs/mechanism/flywheel-wheel-candidate-capability-screen.csv"
BENCH_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/flywheel_launcher_bench_controllers.yaml"


def load_gate():
    return json.loads(GATE.read_text(encoding="utf-8"))


def render_bench():
    result = subprocess.run(
        ["xacro", str(BENCH_XACRO), f"controllers_config:={CONTROLLERS}"],
        cwd=ROOT,
        env=os.environ.copy(),
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def floats(text):
    return tuple(float(value) for value in text.split())


def test_candidate_status_and_gate_are_provisional_not_physical():
    data = load_gate()
    assert data["decision"]["status"] == "SIMULATION_READY_PROVISIONAL_BASELINE"
    assert data["decision"]["capability_phase_authorized"] is True
    assert data["decision"]["physical_hardware_released"] is False
    assert data["status"] == {
        "FLYWHEEL_WHEEL_CANDIDATE_SELECTED": True,
        "FLYWHEEL_WHEEL_FINAL_PROCUREMENT_FROZEN": False,
        "FLYWHEEL_WHEEL_PHYSICAL_MEASUREMENT_PENDING": True,
        "FLYWHEEL_WHEEL_REVISIT_ALLOWED": True,
    }
    assert data["classifications"]["FLYWHEEL_MECHANICAL_GATE_A_SIMULATION_READY"] is True
    assert data["classifications"]["FLYWHEEL_MECHANICAL_GATE_A_PHYSICAL_VALIDATED"] is False


def test_hub_axial_stack_cutout_and_torque_screen_are_credible_for_simulation():
    data = load_gate()
    hub = data["hub"]
    assert hub["status"] == "PROVISIONAL_HUB_FOR_SIMULATION"
    assert math.isclose(hub["shaft_engagement_m"], 0.0215, abs_tol=1e-12)
    assert hub["shaft_engagement_m"] <= hub["shaft_flat_available_m"]
    assert hub["screened_wheel_interface_torque_nm"] > hub["required_peak_motor_torque_nm"]
    stack = data["axial_stack_launcher_local_z_m"]
    assert stack["hub_outer_clearance_face"] < stack["panel_inside"]
    assert stack["hub_flange_wheel_shoulder"] == stack["flywheel_outer_face"]
    assert stack["shaft_tip"] == 0.017
    cutout = data["panel_cutout"]
    assert cutout["diameter_m"] == 0.012
    assert math.isclose(cutout["mount_hole_edge_ligament_m"], 0.007, abs_tol=1e-12)
    assert cutout["hub_passes_through_panel"] is False


def test_mass_inertia_and_motor_sensitivity_cases_cover_required_grid():
    data = load_gate()
    rotating = data["rotating_mass_and_inertia"]
    assert rotating["wheel_mass_sensitivity_kg"] == [0.7, 0.8, 0.9]
    assert rotating["wheel_inertia_overall_range_kg_m2"] == [0.0035, 0.009]
    assert rotating["motor_rotor_inertia_manufacturer_value"] is None
    assert rotating["motor_rotor_inertia_sensitivity_kg_m2"] == [0.0, 0.0001, 0.0002]
    cases = data["motor_model"]["capability_cases"]
    assert len(cases) == 30
    assert {case["target_rpm"] for case in cases} == {1000, 1250, 1500, 1750, 2000}
    assert {case["wheel_mass_kg"] for case in cases} == {0.7, 0.8, 0.9}
    assert all(case["available_current_at_target_a"] <= 20.0 for case in cases)
    assert all(case["voltage_required_at_20a_v"] < 12.8 for case in cases)
    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        assert len(list(csv.DictReader(handle))) == 30


def test_rendered_standalone_contains_motor_shaft_hub_retention_and_real_inertia():
    root = render_bench()
    frame = root.find("./link[@name='flywheel_launcher_frame_link']")
    assert frame is not None
    frame_inertial = frame.find("inertial")
    assert math.isclose(float(frame_inertial.find("mass").attrib["value"]), 4.33478256461, rel_tol=1e-12)
    assert math.isclose(floats(frame_inertial.find("origin").attrib["xyz"])[0], -0.00231297294785, rel_tol=1e-12)
    assert math.isclose(floats(frame_inertial.find("origin").attrib["xyz"])[2], 0.0185061462201, rel_tol=1e-12)
    frame_collisions = {item.attrib["name"] for item in frame.findall("collision")}
    assert {"d5065_motor_left_col", "d5065_motor_right_col"} <= frame_collisions
    upper = frame.find("./collision[@name='flywheel_cradle_upper_plate_col']/geometry/mesh")
    lower = frame.find("./collision[@name='flywheel_cradle_lower_plate_col']/geometry/mesh")
    assert lower is not None and lower.attrib["filename"].endswith("flywheel_lower_panel_exit_clearance.stl")
    assert upper is not None and upper.attrib["filename"].endswith("flywheel_upper_panel_exit_clearance.stl")
    for side, expected_y in (("left", 0.129), ("right", -0.129)):
        link = root.find(f"./link[@name='flywheel_{side}_link']")
        joint = root.find(f"./joint[@name='flywheel_{side}_joint']")
        assert link is not None and joint is not None
        names = {item.attrib["name"] for item in link.findall("collision")}
        assert {
            f"flywheel_{side}_col",
            f"flywheel_{side}_shaft_col",
            f"flywheel_{side}_hub_collar_col",
            f"flywheel_{side}_hub_pilot_col",
            f"flywheel_{side}_retainer_col",
        } <= names
        assert floats(joint.find("origin").attrib["xyz"]) == (0.0, expected_y, 0.0)
        assert math.isclose(float(joint.find("limit").attrib["effort"]), 0.62, abs_tol=1e-12)
        inertial = link.find("inertial")
        assert math.isclose(float(inertial.find("mass").attrib["value"]), 0.9267636462761, rel_tol=1e-12)
        assert math.isclose(float(inertial.find("inertia").attrib["izz"]), 0.00675116210829, rel_tol=1e-12)


def test_machine_evaluator_passes():
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/evaluate_flywheel_provisional_gate_a.py")],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
