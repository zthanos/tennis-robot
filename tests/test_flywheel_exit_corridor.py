import hashlib
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
AUDIT = ROOT / "config/flywheel_launcher_exit_corridor_audit.json"
BALL_RESULTS = ROOT / "config/tennis_ball_compliance_calibration_results.json"
BENCH = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/flywheel_launcher_bench_controllers.yaml"
CAD = ROOT / "cad/flywheel-launcher-v0/launcher-envelope.scad"
PARAMS = ROOT / "cad/flywheel-launcher-v0/params.scad"
RELIEF = ROOT / "cad/flywheel-launcher-v0/provisional-cradle-exit-clearance.scad"
REPORT = ROOT / "docs/mechanism/flywheel-launcher-post-nip-exit-corridor-audit.md"


def data():
    return json.loads(AUDIT.read_text(encoding="utf-8"))


def rendered_bench():
    result = subprocess.run(
        ["xacro", str(BENCH), f"controllers_config:={CONTROLLERS}"],
        cwd=ROOT,
        env=os.environ.copy(),
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_cad_cylinder_is_a_nonmanufacturing_nominal_corridor():
    audit = data()["cad_cylinder"]
    assert audit["classification"] == "NOMINAL_LAUNCH_CORRIDOR"
    assert audit["physical_hardware"] is False
    assert audit["exit_keep_out_or_reference"] is True
    assert audit["diameter_m"] == 0.09
    assert audit["length_m"] == 0.22
    assert audit["launcher_local_x_extent_m"] == [0.1, 0.32]
    assert audit["collision_geometry"] is False
    assert "module exit_guide_envelope()" in CAD.read_text(encoding="utf-8")
    params = PARAMS.read_text(encoding="utf-8")
    assert "exit_guide_len = 220;" in params
    assert "exit_clear_d = 90;" in params


def test_measured_preimplementation_interference_is_fully_recorded():
    audit = data()
    path = audit["measured_path"]
    assert path["final_wheel_contact_time_s"] == 2.035
    assert path["wheel_release_time_s"] == 2.037
    assert math.isclose(path["centre_distance_release_to_contact_m"], 0.0368289276461, rel_tol=1e-9)
    assert path["collision_name"].endswith("flywheel_cradle_lower_plate_col_collision")
    assert path["geometric_penetration_m"] > 0
    root = audit["root_cause"]
    assert root["A_cradle_incorrectly_reconstructed"] is False
    assert root["B_cad_lacks_required_exit_opening"] is True
    assert root["C_actual_exit_departs_nominal_axis_enough_to_exhaust_6mm_clearance"] is True
    assert root["D_shaped_exit_cutout_required"] is True
    assert root["E_other_fixed_component_redirects_ball_before_impact"] is False


def test_practical_relief_is_local_and_remote_from_motor_mounts():
    proposed = data()["proposed_practical_cutout"]
    lower = proposed["lower_plate"]
    assert lower["bounding_box_xy_m"]["x"] == [0.022800000000000015, 0.128]
    assert lower["remaining_side_ligament_each_m"] > 0.133
    assert lower["removed_6061_mass_kg"] < 0.086
    assert proposed["minimum_distance_to_motor_mount_hole_edge_m"] > 0.100
    assert proposed["minimum_distance_to_shaft_axis_m"] > 0.118
    assert proposed["structural_classification"] == "GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED"
    source = RELIEF.read_text(encoding="utf-8")
    assert "ball_swept_r = 38;" in source
    assert "cad_corridor_r = 45;" in source
    assert "measured_lower_swept_cutout_2d" in source


def test_active_standalone_uses_relief_meshes_and_updated_inertia():
    root = rendered_bench()
    frame = root.find("./link[@name='flywheel_launcher_frame_link']")
    lower = frame.find("./collision[@name='flywheel_cradle_lower_plate_col']/geometry/mesh")
    upper = frame.find("./collision[@name='flywheel_cradle_upper_plate_col']/geometry/mesh")
    assert lower.attrib["filename"].endswith("flywheel_lower_panel_exit_clearance.stl")
    assert upper.attrib["filename"].endswith("flywheel_upper_panel_exit_clearance.stl")
    assert (ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_lower_panel_exit_clearance.stl").exists()
    assert (ROOT / "ros2_ws/src/tennis_robot/meshes/flywheel_upper_panel_exit_clearance.stl").exists()
    inertial = frame.find("inertial")
    assert math.isclose(float(inertial.find("mass").attrib["value"]), 4.33478256461, rel_tol=1e-12)
    origin = tuple(float(value) for value in inertial.find("origin").attrib["xyz"].split())
    assert origin[0] < 0
    assert math.isclose(origin[2], 0.0185061462201, rel_tol=1e-12)
    assert float(inertial.find("inertia").attrib["ixz"]) < 0


def test_identical_low_energy_retest_clears_geometry_without_steering():
    audit = data()
    retest = audit["postimplementation_retest"]
    assert retest["passed"] is True
    assert retest["post_release_geometry_clear"] is True
    assert math.isclose(retest["exit_speed_m_s"], 5.27320189859, rel_tol=1e-9)
    assert math.isclose(retest["exit_elevation_deg"], 17.8603434239, rel_tol=1e-9)
    assert retest["maximum_non_gravity_delta_v_before_ground_m_s"] < 1e-5
    decisions = audit["decisions_final"]
    assert decisions == {
        "CAD_CYLINDER_IS_PHYSICAL_HARDWARE": False,
        "CAD_CYLINDER_IS_EXIT_KEEP_OUT_OR_REFERENCE": True,
        "CURRENT_CRADLE_VIOLATES_BALL_EXIT_ENVELOPE": False,
        "LOWER_PLATE_EXIT_CUTOUT_REQUIRED": True,
        "MINIMUM_EXIT_CUTOUT_DEFINED": True,
        "PRACTICAL_EXIT_CLEARANCE_DEFINED": True,
        "POST_FLYWHEEL_PATH_NONCONTACTING": True,
        "POST_FLYWHEEL_BARREL_CONTACT_REQUIRED": False,
        "CRADLE_EXIT_GEOMETRY_READY_FOR_CAPABILITY_RETEST": True,
        "STRUCTURAL_REVIEW_REQUIRED": True,
    }


def test_ball_calibration_remains_untouched_and_capability_sweep_resumes_only_after_pass():
    audit = data()
    digest = hashlib.sha256(BALL_RESULTS.read_bytes()).hexdigest()
    assert digest == "a7aa85327219d624c562b4c528f946bb62e1326d0b58dc7064f7439a07731a8a"
    capability = json.loads((ROOT / "config/flywheel_launcher_capability_map.json").read_text(encoding="utf-8"))
    assert capability["regression_reference"]["passed"] is True
    assert len(capability["symmetric_operating_points"]) == 21
    assert capability["classifications"]["POST_FLYWHEEL_PATH_NONCONTACTING"] is True
    assert capability["decision"]["stop_condition_encountered"] is False
    report = REPORT.read_text(encoding="utf-8")
    for name, value in audit["decisions_final"].items():
        assert f"{name} = {str(value).lower()}" in report
