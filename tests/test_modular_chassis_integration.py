import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "engineering" / "chassis-strength-bench"
CAD = ROOT / "cad" / "modular-test-chassis" / "modular-chassis-v2.scad"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_cad_uses_one_50x30_interface_and_outboard_alignment():
    source = CAD.read_text(encoding="utf-8")
    assert "rail_w = 50;" in source
    assert "rail_h = 30;" in source
    assert "gamma_body_w = 70;" in source
    assert "drive_center_y = 220" in source
    assert "gamma_body_center_y = 210" in source
    assert "50 mm interface must stay flush with the outboard face" in source


def test_generated_sdf_contains_all_modules_and_connection_sensors():
    generator = _load("modular_sdf", BENCH / "generate_modular_integration_sdf.py")
    root = generator.generate(0.0).getroot()
    model = root.find('./world/model[@name="modular_test_chassis_v2"]')
    assert model is not None
    assert len(model.findall("link")) == 16
    assert len(model.findall("joint/sensor")) == 15
    assert len(model.findall('joint[@type="revolute"]')) == 4
    assert model.find("./link[@name='left_gamma_body']") is not None
    assert model.find("./link[@name='right_gamma_body']") is not None
    assert model.find("./plugin/topic").text == "/modular_chassis_integration/cmd_vel"


def test_front_wheels_are_near_gamma_and_axes_are_in_model_frame():
    generator = _load("modular_sdf_front", BENCH / "generate_modular_integration_sdf.py")
    model = generator.generate(0).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
    for side in ("left", "right"):
        assert float(model.find(f"link[@name='front_{side}_wheel']/pose").text.split()[0]) == 0.300
        axis = model.find(f"joint[@name='front_{side}_wheel_joint']/axis/xyz")
        assert axis.attrib["expressed_in"] == "__model__"
        assert model.find(f"link[@name='{side}_rail_extension']") is not None
    source = CAD.read_text()
    assert "front_motor_x = 300;" in source
    assert "module front_motor_gamma(side=1)" in source


def test_gamma_references_restore_assembly_orientation_and_are_preview_only():
    source = CAD.read_text()
    assert "translate([330,69,52])" in source
    assert "translate([515,-69,52]) rotate([0,0,180])" in source
    assert "%gamma_mount_reference(side);" in source
    assert "front_motor_gamma_preview(1);" in source
    assert "front_motor_gamma_preview(-1);" in source


def test_payload_is_optional_and_parameterized():
    generator = _load("modular_sdf_payload", BENCH / "generate_modular_integration_sdf.py")
    empty = generator.generate(0.0).getroot()
    loaded = generator.generate(10.0).getroot()
    assert empty.find(".//link[@name='payload']") is None
    payload = loaded.find(".//link[@name='payload']/inertial/mass")
    assert payload is not None
    assert float(payload.text) == 10.0


def test_integration_summary_uses_filtered_connector_loads(tmp_path):
    summarizer = _load("modular_summary", BENCH / "summarize_modular_integration.py")
    joints = {}
    for names in summarizer.GROUPS.values():
        for name in names:
            joints[name] = {
                "force_norm_n": {"maximum": 999.0, "p99_5": 20.0},
                "torque_norm_nm": {"maximum": 99.0, "p99_5": 4.0},
            }
    path = tmp_path / "payload_0kg" / "wrench_summary.json"
    path.parent.mkdir()
    path.write_text(json.dumps({"joints": joints}), encoding="utf-8")
    report = summarizer.summarize([path], design_factor=2.5)
    assert report["interfaces"]["rear_corner_50x50"]["design_force_n"] == 50.0
    assert report["interfaces"]["rear_corner_50x50"]["design_torque_nm"] == 10.0
    assert report["status"] == "CAD_LOADS_AVAILABLE_STRUCTURAL_CAPACITY_PENDING"


def test_checked_in_sdf_is_well_formed():
    path = BENCH / "gazebo" / "modular_chassis_integration_bench.sdf"
    root = ET.parse(path).getroot()
    assert root.tag == "sdf"


def test_motion_summary_unwraps_a_turn(tmp_path):
    motion = _load("motion_summary", BENCH / "summarize_integration_motion.py")
    path = tmp_path / "odometry.jsonl"
    messages = [
        {"pose": {"position": {"x": 0, "y": 0}, "orientation": {"z": 0, "w": 1}}},
        {"pose": {"position": {"x": 0.01, "y": 0}, "orientation": {"z": 0.7071068, "w": 0.7071068}}},
    ]
    path.write_text("\n".join(json.dumps(item) for item in messages), encoding="utf-8")
    report = motion.summarize(path)
    assert report["turn_executed"] is True
    assert report["approximately_in_place"] is True
