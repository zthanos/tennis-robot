"""Generated-model acceptance tests for the authoritative compact CAD model."""

from __future__ import annotations

import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts" / "generate_robot_urdf.py"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/controllers.yaml"
VALIDATOR_PATH = ROOT / "scripts/validate_compact_mechanics.py"
CONTRACT = ROOT / "config/compact_mechanical_contract.json"
MEASUREMENTS = ROOT / "config/compact_cad_measurements.json"

pytestmark = pytest.mark.skipif(
    shutil.which("xacro") is None or not os.environ.get("AMENT_PREFIX_PATH"),
    reason="ROS 2 environment not sourced (need xacro + AMENT_PREFIX_PATH)",
)

spec = importlib.util.spec_from_file_location("compact_validator", VALIDATOR_PATH)
validator = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = validator
spec.loader.exec_module(validator)


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("compact-model")
    collect = tmp / "compact.urdf"
    launch = tmp / "compact-launch.urdf"
    sdf = tmp / "compact.sdf"

    def generate(output: Path, **env_overrides: str) -> None:
        env = {**os.environ, **env_overrides}
        command = [
            sys.executable, str(GENERATOR), "--packaging-variant", "compact",
            "--output", str(output), "--controllers-config", str(CONTROLLERS),
        ]
        if output == collect:
            command += ["--sdf-output", str(sdf)]
        subprocess.run(command, check=True, cwd=ROOT, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    generate(collect)
    generate(launch, BASKET_LAUNCH_TILT_DEG="12")
    return ET.parse(collect).getroot(), ET.parse(launch).getroot(), sdf, collect, launch


def _link(root: ET.Element, name: str) -> ET.Element:
    return next(link for link in root.findall("link") if link.get("name") == name)


def _joint(root: ET.Element, name: str) -> ET.Element:
    return next(joint for joint in root.findall("joint") if joint.get("name") == name)


def test_compact_contains_the_cad_physical_hierarchy(models):
    root = models[0]
    links = {link.get("name") for link in root.findall("link")}
    assert {
        "compact_bridge_link", "compact_intake_cheeks_link",
        "compact_handoff_ramp_link", "intake_wheel_left_carriage_link",
        "intake_wheel_right_carriage_link", "intake_wheel_left_link",
        "intake_wheel_right_link", "basket_rails_link",
        "basket_guide_path_link", "basket_link",
        "compact_fixed_entry_hood_link",
        "flywheel_launcher_frame_link", "flywheel_left_link",
        "flywheel_right_link",
    } <= links
    assert "funnel_link" not in links, "compact must not inherit the legacy funnel solid"
    assert "basket_lift_carriage_link" not in links

    assert _joint(root, "intake_wheel_left_carriage_joint").find("parent").get("link") == "compact_bridge_link"
    assert _joint(root, "compact_intake_cheeks_joint").find("parent").get("link") == "compact_bridge_link"
    assert _joint(root, "compact_handoff_ramp_joint").find("parent").get("link") == "compact_bridge_link"
    assert _joint(root, "flywheel_launcher_mount_joint").find("parent").get("link") == "compact_bridge_link"
    assert _joint(root, "basket_joint").find("parent").get("link") == "base_link"


def test_cad_wheel_dimensions_and_cradle_structure(models):
    root = models[0]
    intake = _link(root, "intake_wheel_left_link").find("collision/geometry/cylinder")
    assert float(intake.get("radius")) == pytest.approx(0.062)
    assert float(intake.get("length")) == pytest.approx(0.073)
    flywheel = _link(root, "flywheel_left_link").find("collision/geometry/cylinder")
    assert float(flywheel.get("radius")) == pytest.approx(0.100)
    assert float(flywheel.get("length")) == pytest.approx(0.050)

    cradle = _link(root, "flywheel_launcher_frame_link")
    names = {collision.get("name") for collision in cradle.findall("collision")}
    assert names == {"flywheel_cradle_lower_plate_col", "flywheel_cradle_upper_plate_col"}
    for collision in cradle.findall("collision"):
        assert [float(v) for v in collision.find("geometry/box").get("size").split()] == pytest.approx([0.256, 0.258, 0.008])


def test_widened_bridge_matches_the_shared_cad_datum(models):
    bridge = _link(models[0], "compact_bridge_link")
    collisions = {collision.get("name"): collision
                  for collision in bridge.findall("collision")}
    forward = collisions["bridge_top_forward_col"]
    assert [float(v) for v in forward.find("geometry/box").get("size").split()] == pytest.approx(
        [0.116, 0.490, 0.018]
    )
    rear_left = collisions["bridge_top_rear_left_col"]
    assert [float(v) for v in rear_left.find("origin").get("xyz").split()][1] == pytest.approx(0.205)
    assert [float(v) for v in rear_left.find("geometry/box").get("size").split()][1] == pytest.approx(0.080)

    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    support = contract["bridge_support_geometry_mm"]
    assert support["bridge_outer_y"] == [-245.0, 245.0]
    assert support["bridge_to_chassis_outer_edge_margin_each_side"] == 45.0
    assert support["upright_contact_width_y"] == 18.0
    assert support["upright_contact_length_x"] == 175.0
    assert support["contact_area_each_mm2"] == 3150.0


def test_compact_basket_has_two_guides_holders_and_no_fictitious_cage(models):
    collect, launch = models[0], models[1]
    guide = _link(collect, "basket_rails_link")
    guide_names = {collision.get("name") for collision in guide.findall("collision")}
    assert guide_names == {
        "basket_left_rail_col", "basket_right_rail_col",
        "basket_left_rail_foot_col", "basket_right_rail_foot_col",
    }
    all_collision_names = {
        collision.get("name") for link in collect.findall("link")
        for collision in link.findall("collision")
    }
    assert not any("carriage_" in name or "crossbrace" in name
                   for name in all_collision_names)
    holders = _link(launch, "basket_raised_holders_link")
    assert len(holders.findall("collision")) == 4


def test_lift_compliance_tilt_and_control_contracts(models):
    collect, launch = models[0], models[1]
    basket = _joint(collect, "basket_joint")
    assert basket.get("type") == "prismatic"
    assert float(basket.find("limit").get("lower")) == pytest.approx(-0.010)
    assert float(basket.find("limit").get("upper")) == pytest.approx(0.100)
    for side in ("left", "right"):
        carriage = _joint(collect, f"intake_wheel_{side}_carriage_joint")
        assert carriage.get("type") == "prismatic"
        assert float(carriage.find("limit").get("upper")) == pytest.approx(0.008)

    tilt_collect = _joint(collect, "basket_launch_pose_joint")
    tilt_launch = _joint(launch, "basket_launch_pose_joint")
    assert tilt_collect.get("type") == tilt_launch.get("type") == "fixed"
    assert float(tilt_collect.find("origin").get("rpy").split()[1]) == pytest.approx(0.0)
    assert float(tilt_launch.find("origin").get("rpy").split()[1]) == pytest.approx(math.radians(12), abs=1e-8)

    physical_joints = {joint.get("name") for joint in collect.findall("joint")}
    control_joints = {
        joint.get("name") for control in collect.findall("ros2_control")
        for joint in control.findall("joint")
    }
    assert control_joints <= physical_joints
    assert {"basket_joint", "flywheel_left_joint", "flywheel_right_joint",
            "intake_wheel_left_joint", "intake_wheel_right_joint"} <= control_joints
    assert "basket_launch_pose_joint" not in control_joints, (
        "the CAD tilt pose is configured mechanically; no actuator was selected"
    )


def test_major_dynamic_links_have_credible_positive_inertia(models):
    root = models[0]
    names = {
        "intake_wheel_left_carriage_link", "intake_wheel_right_carriage_link",
        "intake_wheel_left_link", "intake_wheel_right_link",
        "basket_link",
        "flywheel_left_link", "flywheel_right_link",
    }
    for name in names:
        inertial = _link(root, name).find("inertial")
        assert inertial is not None, name
        mass = float(inertial.find("mass").get("value"))
        assert mass >= 0.05, f"{name} uses placeholder mass {mass}"
        inertia = inertial.find("inertia")
        matrix = np.asarray([
            [float(inertia.get("ixx")), float(inertia.get("ixy")), float(inertia.get("ixz"))],
            [float(inertia.get("ixy")), float(inertia.get("iyy")), float(inertia.get("iyz"))],
            [float(inertia.get("ixz")), float(inertia.get("iyz")), float(inertia.get("izz"))],
        ])
        assert np.linalg.eigvalsh(matrix).min() > 0.0, name


def test_all_major_cad_envelopes_are_within_contract(models):
    result = validator.validate(models[3], CONTRACT, models[4])
    failures = {name: value for name, value in result["geometry"].items() if not value["pass"]}
    assert not failures
    assert max(value.get("max_abs_delta_m", 0.0) for value in result["geometry"].values()) < 0.001


def test_static_sat_has_only_the_interferences_present_in_source_cad(models):
    result = validator.validate(models[3], CONTRACT, models[4])
    for state, checks in result["collisions"].items():
        failed = {name for name, value in checks.items() if not value["pass"]}
        assert failed == {"launcher_vs_bridge"}, state
    blockers = result["known_cad_interferences"]
    assert blockers["launcher_vs_bridge"]["physical_intersection_volume_mm3"] > 140_000
    assert blockers["launcher_vs_basket_hood"]["physical_intersection_volume_mm3"] == 0.0
    assert blockers["launcher_vs_basket_launch"]["physical_intersection_volume_mm3"] > 700
    assert blockers["launcher_vs_basket_raised"]["physical_intersection_volume_mm3"] > 40
    assert blockers["basket_raised_vs_bridge"]["physical_intersection_volume_mm3"] == pytest.approx(2240.0)
    assert blockers["basket_launch_vs_bridge"]["physical_intersection_volume_mm3"] > 1050.0
    measurements = json.loads(MEASUREMENTS.read_text(encoding="utf-8"))["parts"]
    for name in (
        "launcher_basket_hood_intersection", "launcher_basket_collect_intersection",
        "basket_collect_intake_intersection", "basket_collect_left_wheel_intersection",
        "basket_collect_right_wheel_intersection",
        "basket_hood_intake_intersection", "basket_bin_intake_intersection",
        "basket_walls_chassis_intersection", "basket_floor_chassis_intersection",
        "hood_supports_wheels_intersection", "hood_supports_launcher_intersection",
        "hood_supports_bridge_intersection", "hood_supports_basket_intersection",
        "handoff_ramp_left_wheel_intersection",
        "handoff_ramp_right_wheel_intersection",
    ):
        assert measurements[name]["intersection_volume_mm3"] == pytest.approx(0.0, abs=1e-6)
    assert measurements["basket_flange_chassis_intersection"]["intersection_volume_mm3"] == pytest.approx(360.0)
    assert result["pass"] is False, "physics capability must stay gated while CAD solids overlap"


def test_mass_com_stability_and_ground_contact(models):
    result = validator.validate(models[3], CONTRACT, models[4])
    for state, props in result["mass_properties"].items():
        expected_mass = 32.687 if state == "basket_launch_pose" else 32.23
        assert props["total_mass_kg"] == pytest.approx(expected_mass, abs=0.01)
        assert props["statically_stable"], state
        assert props["support_polygon_margin_m"] > 0.36
        assert props["com_m"][2] < 0.14
    wheel_bbox = result["geometry"]["drive_wheels"]["urdf_bbox_m"]
    assert wheel_bbox[0][2] == pytest.approx(0.0, abs=1e-6)
    chassis_bbox = result["geometry"]["chassis"]["urdf_bbox_m"]
    assert chassis_bbox[0][2] == pytest.approx(0.038, abs=1e-6)
    basket_inertial = _link(models[0], "basket_link").find("inertial")
    assert float(basket_inertial.find("mass").get("value")) == pytest.approx(3.577)
    assert [float(value) for value in basket_inertial.find("origin").get("xyz").split()] == pytest.approx(
        [0.209460, 0.0, 0.046481]
    )


def test_gazebo_sdf_preserves_contact_owner_links(models):
    sdf = ET.parse(models[2]).getroot()
    names = {link.get("name") for link in sdf.findall(".//model/link")}
    assert {"compact_bridge_link", "compact_intake_cheeks_link",
            "compact_handoff_ramp_link", "flywheel_launcher_frame_link",
            "basket_rails_link", "basket_guide_path_link"} <= names
    assert "basket_link" not in names, "basket fixed child should reduce onto the guide-path link"
