"""Mechanical configuration invariants, checked against the GENERATED model.

These tests run the real generator and inspect the resulting URDF, so they
cover the whole arg chain (generator defaults -> xacro args -> macro params)
rather than the presence of a string in a source file.

The invariants:
  * The intake assembly and the flywheel launcher claim the same front volume
    (intake x 446...884 / z 0...281, flywheel wheels x 459...661 / z 162...268),
    so no collection configuration may carry the launcher and the launch
    configuration must not carry the intake.
  * The launcher keeps real collision geometry — a visual-only launcher would
    make Throwing Mode physics meaningless.
  * The basket lift travel is the CAD baseline 100 mm, not the earlier 450 mm
    that pushed the raised rim through the LiDAR scan plane.
  * The LiDAR OPTICAL CENTRE (not the link origin) is 498 mm above ground.

Requires xacro, so the whole module skips when ROS is not sourced.
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts" / "generate_robot_urdf.py"
CONTROLLERS = ROOT / "ros2_ws" / "src" / "tennis_robot" / "config" / "controllers.yaml"

pytestmark = pytest.mark.skipif(
    shutil.which("xacro") is None, reason="xacro not on PATH (ROS not sourced)"
)

COLLECTION_VARIANTS = ("baseline", "option-a-collect")
LAUNCH_VARIANT = "option-a-launch"


def _generate(tmp_path: Path, variant: str, **env_overrides: str) -> ET.Element:
    out = tmp_path / f"{variant}.urdf"
    env = {**os.environ, "ROBOT_PACKAGING_VARIANT": variant, **env_overrides}
    subprocess.run(
        [sys.executable, str(GENERATOR), "--output", str(out),
         "--controllers-config", str(CONTROLLERS), "--packaging-variant", variant],
        check=True, env=env, cwd=str(ROOT),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    return ET.parse(out).getroot()


def _link_names(root: ET.Element) -> set[str]:
    return {link.get("name", "") for link in root.findall("link")}


def _joint(root: ET.Element, name: str) -> ET.Element | None:
    for joint in root.findall("joint"):
        if joint.get("name") == name:
            return joint
    return None


def _ground_height(root: ET.Element, link_name: str) -> float:
    """Sum the z of the fixed-joint chain from the root link down to link_name."""
    parent_joint = {j.find("child").get("link"): j for j in root.findall("joint")}
    height, current = 0.0, link_name
    while current in parent_joint:
        joint = parent_joint[current]
        origin = joint.find("origin")
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split())]
        assert all(abs(v) < 1e-9 for v in rpy), f"{joint.get('name')} is rotated; chain sum is invalid"
        xyz = (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split())
        height += float(xyz[2])
        current = joint.find("parent").get("link")
    return height


# ---------------------------------------------------------------- variants

@pytest.mark.parametrize("variant", COLLECTION_VARIANTS)
def test_collection_configuration_has_intake_and_no_launcher(tmp_path, variant):
    links = _link_names(_generate(tmp_path, variant))
    assert {"intake_wheel_left_link", "intake_wheel_right_link", "funnel_link"} <= links
    assert not {name for name in links if "flywheel" in name}


def test_launch_configuration_has_launcher_and_no_conflicting_intake(tmp_path):
    root = _generate(tmp_path, LAUNCH_VARIANT)
    links = _link_names(root)
    assert {"flywheel_left_link", "flywheel_right_link",
            "flywheel_launcher_frame_link"} <= links
    # The conflicting assembly: driven wheels, funnel cheeks, ramp, deflector.
    assert "funnel_link" not in links
    assert not {name for name in links
                if name.startswith("intake_wheel_")}
    # ...and its ros2_control joints go with it, or the controller cannot load.
    control_joints = {
        joint.get("name")
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
    }
    assert not {name for name in control_joints if name and name.startswith("intake_wheel_")}
    assert {"flywheel_left_joint", "flywheel_right_joint"} <= control_joints


def test_launcher_keeps_real_collision_geometry(tmp_path):
    root = _generate(tmp_path, LAUNCH_VARIANT)
    for link_name in ("flywheel_left_link", "flywheel_right_link"):
        link = next(l for l in root.findall("link") if l.get("name") == link_name)
        collisions = link.findall("collision")
        assert collisions, f"{link_name} must keep collision geometry, not visual-only"
        for collision in collisions:
            assert collision.find("geometry") is not None


@pytest.mark.parametrize("variant", COLLECTION_VARIANTS)
def test_flywheel_is_off_by_default_in_every_collection_variant(tmp_path, variant):
    """A plain run_native.sh must not put the launcher inside the intake."""
    assert not {n for n in _link_names(_generate(tmp_path, variant)) if "flywheel" in n}


def test_launch_variant_is_required_to_get_a_launcher(tmp_path):
    """The launcher follows the mechanical variant, not an implicit default."""
    default = _generate(tmp_path, "baseline")
    assert not {n for n in _link_names(default) if "flywheel" in n}
    explicit = _generate(tmp_path, LAUNCH_VARIANT)
    assert {n for n in _link_names(explicit) if "flywheel" in n}


# ------------------------------------------------------------------ basket

@pytest.mark.parametrize("variant", (*COLLECTION_VARIANTS, LAUNCH_VARIANT))
def test_basket_lift_upper_stop_is_one_hundred_millimetres(tmp_path, variant):
    joint = _joint(_generate(tmp_path, variant), "basket_joint")
    assert joint is not None and joint.get("type") == "prismatic"
    assert float(joint.find("limit").get("upper")) == pytest.approx(0.100, abs=1e-9)


@pytest.mark.parametrize("variant", (*COLLECTION_VARIANTS, LAUNCH_VARIANT))
def test_basket_lower_stop_sits_below_the_parked_position(tmp_path, variant):
    """The parked carriage must not rest ON its lower hard stop.

    A prismatic joint parked exactly on its limit sits on a permanently active
    DART limit constraint, and gz-sim then cannot drive it at all — the joint
    stays frozen even with hold_joints=false and even under gravity, while
    every other joint in the same model actuates normally. The commanded travel
    is still 0..100 mm; only the modelled hard stop moves below it.
    """
    limit = _joint(_generate(tmp_path, variant), "basket_joint").find("limit")
    lower = float(limit.get("lower"))
    assert lower < 0.0, (
        "basket_joint lower limit coincides with the parked position (0.0); "
        "the carriage will rest on an active limit constraint and freeze"
    )
    assert lower == pytest.approx(-0.010, abs=1e-9)


def test_basket_commanded_travel_is_unchanged_by_the_over_travel_margin():
    """The margin is a physics accommodation, not a mechanical dimension."""
    service = (ROOT / "scripts/tennis_robot/console/ros_service.py").read_text()
    assert 'BASKET_LIFT_TRAVEL_M", "0.100"' in service
    assert "BASKET_LIFT_OVERTRAVEL" not in service, (
        "the supervisor must command 0..travel and know nothing about the "
        "modelled hard stops"
    )


def test_no_stale_450_mm_basket_travel_default_remains():
    """The 0.45 m travel must be gone from every operational default."""
    operational = [
        ROOT / "scripts/generate_robot_urdf.py",
        ROOT / "scripts/tennis_robot/console/ros_service.py",
        ROOT / "ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro",
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/ros2_control.urdf.xacro",
    ]
    for path in operational:
        text = path.read_text(encoding="utf-8")
        for marker in ("BASKET_LIFT_TRAVEL_M', '0.45", 'basket_lift_travel" default="0.45',
                       "basket_lift_travel:=0.45", 'BASKET_LIFT_TRAVEL_M", "0.45'):
            assert marker not in text, f"{path.name} still defaults basket travel to 0.45 m"
    basket = (ROOT / "ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro").read_text()
    assert "lift_travel:=0.45\"" not in basket and "lift_travel:=0.450" not in basket


# ------------------------------------------------------------------- LiDAR

@pytest.mark.parametrize("variant", (*COLLECTION_VARIANTS, LAUNCH_VARIANT))
def test_lidar_optical_centre_is_498_mm_above_ground(tmp_path, variant):
    """The datum is the SCAN PLANE, which is the sensor pose, not lidar_link.

    Chain: base_footprint -> base_link (base_link_height) -> lidar_link
    (lidar_xyz.z) -> gpu_lidar <pose> (scan offset). Summing only the first two
    is exactly how the scan plane silently ended up at 578 mm.
    """
    root = _generate(tmp_path, variant)
    link_height = _ground_height(root, "lidar_link")

    sensor_z = None
    for gazebo in root.findall("gazebo"):
        if gazebo.get("reference") != "lidar_link":
            continue
        for sensor in gazebo.findall("sensor"):
            if sensor.get("name") == "front_lidar":
                sensor_z = float((sensor.findtext("pose") or "0 0 0 0 0 0").split()[2])
    assert sensor_z is not None, "front_lidar sensor pose not found on lidar_link"

    optical_centre = link_height + sensor_z
    assert optical_centre == pytest.approx(0.498, abs=1e-6), (
        f"LiDAR optical centre is {optical_centre * 1000:.1f} mm above ground; "
        f"the CAD datum is 498 mm (link {link_height * 1000:.1f} mm "
        f"+ scan offset {sensor_z * 1000:.1f} mm)"
    )


def test_lidar_mast_never_protrudes_below_the_chassis(tmp_path):
    """The mast is derived from the mount height, so the datum can move safely."""
    root = _generate(tmp_path, "baseline")
    link_height = _ground_height(root, "lidar_link")
    link = next(l for l in root.findall("link") if l.get("name") == "lidar_link")
    mast = next(v for v in link.findall("visual") if v.get("name") == "mast_vis")
    centre_z = float(mast.find("origin").get("xyz").split()[2])
    length = float(mast.find("geometry/box").get("size").split()[2])
    bottom_ground = link_height + centre_z - length / 2.0
    assert bottom_ground > 0.0, "LiDAR mast reaches below the ground plane"
    assert bottom_ground == pytest.approx(0.045 + 0.007, abs=1e-6), (
        "mast should still land on the chassis top"
    )
