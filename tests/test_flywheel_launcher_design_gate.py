import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.evaluate_flywheel_launcher_design_gate import BALL_DESIGN, BENCH_WORLD, ROOT, evaluate


BENCH_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/flywheel_launcher_bench_controllers.yaml"


def _render_bench() -> ET.Element:
    env = os.environ.copy()
    result = subprocess.run(
        [
            "xacro",
            str(BENCH_XACRO),
            f"controllers_config:={CONTROLLERS}",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return ET.fromstring(result.stdout)


def _floats(text: str) -> tuple[float, ...]:
    return tuple(float(value) for value in text.split())


def test_static_design_and_independent_ball_calibration_gate_passes():
    result = evaluate()
    assert result["authoritative_geometry_reconstructed"] is True
    assert result["isolated_architecture"] is True
    assert result["explicit_dart_engine"] is True
    assert result["ball_absent_until_calibrated"] is True
    assert result["compliant_ball_model_design_ready"] is True
    assert result["compliant_ball_model_implemented"] is True
    assert result["compliant_ball_model_calibrated"] is True
    assert result["launcher_tyre_friction_calibration_pending"] is True
    assert result["launcher_physics_trials_authorized"] is True


def test_rendered_launcher_matches_authoritative_nip_and_cradle():
    root = _render_bench()
    mount = root.find("./joint[@name='flywheel_launcher_mount_joint']/origin")
    assert mount is not None
    assert _floats(mount.attrib["xyz"]) == (0.0, 0.0, 0.35)
    assert math.isclose(_floats(mount.attrib["rpy"])[1], -math.radians(20), abs_tol=1e-12)

    frame = root.find("./link[@name='flywheel_launcher_frame_link']")
    assert frame is not None
    collisions = {item.attrib["name"]: item for item in frame.findall("collision")}
    assert set(collisions) == {
        "flywheel_cradle_lower_plate_col",
        "flywheel_cradle_upper_plate_col",
    }
    for collision in collisions.values():
        assert _floats(collision.find("./geometry/box").attrib["size"]) == (
            0.256,
            0.314,
            0.008,
        )
    assert _floats(collisions["flywheel_cradle_lower_plate_col"].find("origin").attrib["xyz"])[2] == -0.043
    assert _floats(collisions["flywheel_cradle_upper_plate_col"].find("origin").attrib["xyz"])[2] == 0.043


def test_rendered_wheels_have_real_collision_geometry_and_spacing():
    root = _render_bench()
    for side, expected_y in (("left", 0.129), ("right", -0.129)):
        joint = root.find(f"./joint[@name='flywheel_{side}_joint']")
        link = root.find(f"./link[@name='flywheel_{side}_link']")
        assert joint is not None and link is not None
        assert _floats(joint.find("origin").attrib["xyz"]) == (0.0, expected_y, 0.0)
        cylinder = link.find("./collision/geometry/cylinder")
        assert cylinder is not None
        assert float(cylinder.attrib["radius"]) == 0.1
        assert float(cylinder.attrib["length"]) == 0.05
    assert math.isclose(0.129 - (-0.129) - 2 * 0.1, 0.058, abs_tol=1e-12)


def test_compliance_contract_uses_itf_targets_and_calibrated_coefficients():
    contract = json.loads(BALL_DESIGN.read_text(encoding="utf-8"))
    reference = contract["itf_2026_reference"]
    assert reference["deformation_test_total_load_n"] == 95.64
    assert reference["forward_deformation_m"] == [0.0056, 0.0074]
    assert reference["return_deformation_m"] == [0.008, 0.0108]
    assert reference["rebound_drop_height_m"] == 2.54
    assert reference["rebound_height_m"] == [1.35, 1.47]
    selected = contract["selected_representation"]
    assert selected["implementation_status"] == "IMPLEMENTED_AND_CALIBRATED"
    assert "calibration_results.json" in selected["normal_force_law"]
    assert contract["required_calibration_evidence"]["launcher_tyre_friction_measurement"] is False
    assert contract["acceptance"]["launcher_physics_trials_authorized"] is True


def test_geometry_world_contains_no_ball_or_robot_subsystem_dependencies():
    text = BENCH_WORLD.read_text(encoding="utf-8").lower()
    assert "gz-physics-dartsim-plugin" in text
    assert '<model name="ball' not in text
    for forbidden in ("intake", "basket", "feeder", "oak-d", "navigation", "throwing mode"):
        assert forbidden not in text


def test_native_flywheel_collision_is_filtered_at_runtime_from_analytical_ball_pair():
    source = (
        ROOT / "ros2_ws/src/tennis_ball_contact_system/src/TennisBallContactSystem.cc"
    ).read_text(encoding="utf-8")
    assert "ChildrenByComponents" in source
    assert "RequestRemoveEntity(collision)" in source
    ball_model = (
        ROOT / "gazebo/models/tennis_ball_compliant/model.sdf"
    ).read_text(encoding="utf-8")
    assert "<category_bitmask>4</category_bitmask><collide_bitmask>8</collide_bitmask>" in ball_model
