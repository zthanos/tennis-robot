import importlib.util
import hashlib
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
    root = generator.generate(0.0, rigid_chassis=False).getroot()
    model = root.find('./world/model[@name="modular_test_chassis_v2"]')
    assert model is not None
    assert len(model.findall("link")) == 17
    assert len(model.findall("joint/sensor")) == 17
    assert len(model.findall('joint[@type="revolute"]')) == 4
    assert model.find("./link[@name='left_gamma_body']") is not None
    assert model.find("./link[@name='right_gamma_body']") is not None
    assert model.find("./plugin/topic").text == "/modular_chassis_integration/cmd_vel"


def test_front_wheels_are_near_gamma_and_axes_are_in_model_frame():
    generator = _load("modular_sdf_front", BENCH / "generate_modular_integration_sdf.py")
    model = generator.generate(0, rigid_chassis=False).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
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
    empty = generator.generate(0.0, rigid_chassis=False).getroot()
    loaded = generator.generate(10.0, rigid_chassis=False).getroot()
    assert empty.find(".//link[@name='payload']") is None
    payload = loaded.find(".//link[@name='payload']/inertial/mass")
    assert payload is not None
    assert float(payload.text) == 10.0


def test_electronics_are_vertical_at_rear_and_cargo_is_independent():
    generator = _load("modular_vertical_electronics", BENCH / "generate_modular_integration_sdf.py")
    model = generator.generate(10, rigid_chassis=False).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
    tray = model.find("link[@name='electronics_tray']")
    assert tray.find("pose").text.split()[:3] == ["-0.165", "0", "0.147"]
    assert tray.find("collision/geometry/box/size").text == "0.004 0.29 0.19"
    assert model.find("joint[@name='payload_joint']/parent").text == "rear_crossbar"


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


def test_ramp_back_crossbar_connects_both_front_motor_modules():
    generator = _load("modular_middle_t", BENCH / "generate_modular_integration_sdf.py")
    model = generator.generate(0, rigid_chassis=False).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
    bar = model.find("link[@name='middle_crossbar']")
    assert bar.find("pose").text.split()[:3] == ["0.2944", "0", "0.032"]
    assert bar.find("collision/geometry/box/size").text == "0.05 0.32 0.02"
    for side in ("left", "right"):
        joint = model.find(f"joint[@name='{side}_middle_t_joint']")
        assert joint.find("parent").text == f"front_{side}_motor_module"
        assert joint.find("child").text == "middle_crossbar"
    assert 160 - 2*30 == 100  # free span between the smaller insertion sleeves
    assert "module rail_extension_positive()" in CAD.read_text()


def test_front_50x20_sleeve_fit_and_unchanged_ramp_face():
    source=CAD.read_text()
    for setting in ("front_brace_w = 50;", "front_brace_h = 20;",
                    "front_brace_x = 294.4;", "front_brace_sleeve_w = 43;",
                    "front_brace_sleeve_h = 13;"):
        assert setting in source
    assert abs(294.4+50/2-319.4)<1e-9
    assert abs((50-2*3.2-43)/2-.3)<1e-9
    assert abs((20-2*3.2-13)/2-.3)<1e-9


def test_front_print_fit_report_matches_current_cad_and_stls():
    report=json.loads((BENCH/'results/front_print_fit/report.json').read_text())
    assert report['cad_sha256']==hashlib.sha256(CAD.read_bytes()).hexdigest()
    assert len(report['intersections'])==5
    assert all(check['empty'] for check in report['intersections'])
    for row in report['stls']:
        path=CAD.parent/'archive/v2-chassis/stl'/f"{row['part']}.stl"
        if not path.exists():
            path=CAD.parent/'stl-v2'/f"{row['part']}.stl"
        assert row['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
        assert row['nonmanifold_edge_count']==0
        assert row['positive_volume_shells']==1


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


def test_rigid_chassis_removes_loop_preserving_mass_and_geometry():
    generator = _load("modular_rigid", BENCH / "generate_modular_integration_sdf.py")
    for payload in (0,10):
        old = generator.generate(payload,rigid_chassis=False).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
        model = generator.generate(payload).getroot().find('./world/model[@name="modular_test_chassis_v2"]')
        assert len(model.findall("link")) == 5
        assert len(model.findall("joint/sensor")) == 4
        assert all(j.attrib["type"]=="revolute" for j in model.findall("joint"))
        assert model.attrib["canonical_link"] == "chassis_body"
        old_mass=sum(float(link.find("inertial/mass").text) for link in old.findall("link"))
        mass=sum(float(link.find("inertial/mass").text) for link in model.findall("link"))
        assert abs(old_mass-mass) < 1e-9
        body = model.find("link[@name='chassis_body']")
        pose = list(map(float,body.find("pose").text.split()[:3]))
        collision = body.find("collision[@name='left_gamma_body_landing_collision']/pose")
        local = list(map(float,collision.text.split()[:3]))
        assert all(abs(a+b-c)<1e-8 for a,b,c in zip(pose,local,[.3925,.210,.037]))
