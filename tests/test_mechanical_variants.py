"""Mechanical configuration invariants, checked against the GENERATED model.

These tests run the real generator and inspect the resulting URDF, so they
cover the whole arg chain (generator defaults -> xacro args -> macro params)
rather than the presence of a string in a source file.

The invariants:
  * P2-99c (c+d1a): the default robot carries no legacy intake / fixed
    launcher, its wheels sit on the v3 axles and its mass matches the v3
    bench model.
  * The basket lift travel is the CAD baseline 100 mm, not the earlier 450 mm
    that pushed the raised rim through the LiDAR scan plane.
  * The LiDAR OPTICAL CENTRE (not the link origin) is the P2-99b C1 scan plane,
    419.8 mm above ground, and nothing on the robot enters its +-2 deg band.

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

# `xacro` on PATH is not enough: the generator subprocess also needs the ROS
# Python environment (AMENT_PREFIX_PATH / PYTHONPATH). Without it these would
# FAIL rather than skip, which reads as a broken model instead of an
# unconfigured shell.
pytestmark = pytest.mark.skipif(
    shutil.which("xacro") is None or not os.environ.get("AMENT_PREFIX_PATH"),
    reason="ROS 2 environment not sourced (need xacro + AMENT_PREFIX_PATH)",
)

# ONE robot. The baseline / option-a-collect / option-a-launch packaging
# variants were archived on 2026-08-27: `compact` is the only machine, and it
# carries the frozen CAD intake AND the flywheel launcher together. What used
# to be a variant choice is now an env toggle on the same robot.
VARIANT = "compact"
#: launcher off, intake fitted — the collection duty of the one robot.
COLLECT_ONLY = {"ROBOT_ENABLE_FLYWHEEL": "false"}
#: intake off, launcher fitted — the throwing duty of the one robot.
LAUNCH_ONLY = {"ROBOT_ENABLE_INTAKE": "false"}


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

LEGACY_MECHANISM_LINKS = {
    "intake_wheel_left_link", "intake_wheel_right_link",
    "intake_wheel_left_mount_link", "intake_wheel_right_mount_link",
    "compact_intake_cheeks_link", "compact_handoff_ramp_link",
    "compact_bridge_link", "basket_rails_link", "compact_fixed_entry_hood_link",
    "flywheel_launcher_frame_link", "flywheel_left_link", "flywheel_right_link",
    "motion_electronics_tray_link", "pi_case_link", "pi_buck_converter_link",
    "battery_link",
}


def test_default_robot_has_no_legacy_intake_or_fixed_launcher(tmp_path):
    """P2-99c (c+d1a): the generated robot is the active design without the
    legacy dual-wheel intake, its bridge/rails/hood, the fixed +20 deg launcher
    and the retired electronics/battery placeholders."""
    links = _link_names(_generate(tmp_path, VARIANT))
    assert not (links & LEGACY_MECHANISM_LINKS), sorted(links & LEGACY_MECHANISM_LINKS)
    assert {"parkside_left_link", "parkside_right_link", "ugreen_link",
            "shared_flywheel_mass_placeholder_link"} <= links


def test_drive_wheels_are_on_the_v3_axles(tmp_path):
    """v3 axles CAD X-65/+300, Y+-297 -> base_link +-0.1825 (D1)."""
    root = _generate(tmp_path, VARIANT)
    got = {}
    for side in ("rear_left", "rear_right", "front_left", "front_right"):
        x, y, _ = (float(v) for v in _joint(root, f"{side}_wheel_joint").find("origin").get("xyz").split())
        got[side] = (round(x, 6), round(y, 6))
    assert got == {"rear_left": (-0.1825, 0.297), "rear_right": (-0.1825, -0.297),
                   "front_left": (0.1825, 0.297), "front_right": (0.1825, -0.297)}


def test_whole_robot_mass_matches_the_v3_bench_model(tmp_path):
    """Total mass within 5 % of the v3 bench full-45 model (20.585 kg)."""
    root = _generate(tmp_path, VARIANT)
    total = sum(float(l.find("inertial/mass").get("value"))
                for l in root.findall("link") if l.find("inertial/mass") is not None)
    assert total == pytest.approx(20.585, rel=0.05)


# ------------------------------------------------------------------ basket

def test_basket_lift_upper_stop_is_one_hundred_millimetres(tmp_path):
    joint = _joint(_generate(tmp_path, VARIANT), "basket_joint")
    assert joint is not None and joint.get("type") == "prismatic"
    assert float(joint.find("limit").get("upper")) == pytest.approx(0.100, abs=1e-9)


def test_basket_lower_stop_sits_below_the_parked_position(tmp_path):
    """The parked carriage must not rest ON its lower hard stop.

    A prismatic joint parked exactly on its limit sits on a permanently active
    DART limit constraint, and gz-sim then cannot drive it at all — the joint
    stays frozen even with hold_joints=false and even under gravity, while
    every other joint in the same model actuates normally. The commanded travel
    is still 0..100 mm; only the modelled hard stop moves below it.
    """
    limit = _joint(_generate(tmp_path, VARIANT), "basket_joint").find("limit")
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

def _origin_matrix(origin: ET.Element | None):
    import numpy as np

    m = np.eye(4)
    if origin is None:
        return m
    r, p, y = (float(v) for v in origin.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    return m


def _world_transforms(root: ET.Element) -> dict:
    """Zero-pose transform of every link in the ground (base_footprint) frame."""
    import numpy as np

    parent = {j.find("child").get("link"): j for j in root.findall("joint")}
    out: dict = {}

    def resolve(link: str):
        if link not in out:
            joint = parent.get(link)
            out[link] = (np.eye(4) if joint is None else
                         resolve(joint.find("parent").get("link")) @ _origin_matrix(joint.find("origin")))
        return out[link]

    for link in root.findall("link"):
        resolve(link.get("name"))
    return out


def _visual_samples(root: ET.Element):
    """(link, visual name, Nx3 ground-frame points) on each visual's box/cylinder hull."""
    import numpy as np

    world = _world_transforms(root)
    ticks = np.linspace(-1.0, 1.0, 9)
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")[0]
            if geometry.tag == "box":
                half = [float(v) / 2.0 for v in geometry.get("size").split()]
            elif geometry.tag == "cylinder":
                r = float(geometry.get("radius"))
                half = [r, r, float(geometry.get("length")) / 2.0]
            else:
                continue
            m = world[link.get("name")] @ _origin_matrix(visual.find("origin"))
            local = [(a * half[0], b * half[1], c * half[2])
                     for a in ticks for b in ticks for c in ticks
                     if max(abs(a), abs(b), abs(c)) == 1.0]
            pts = (m @ np.c_[np.array(local), np.ones(len(local))].T).T[:, :3]
            yield link.get("name"), visual.get("name") or geometry.tag, pts


def _lidar_scan_origin(root: ET.Element) -> tuple[float, float, float]:
    world = _world_transforms(root)
    sensor_z = None
    for gazebo in root.findall("gazebo"):
        if gazebo.get("reference") != "lidar_link":
            continue
        for sensor in gazebo.findall("sensor"):
            if sensor.get("name") == "front_lidar":
                sensor_z = float((sensor.findtext("pose") or "0 0 0 0 0 0").split()[2])
    assert sensor_z is not None, "front_lidar sensor pose not found on lidar_link"
    x, y, z = world["lidar_link"][:3, 3]
    return float(x), float(y), float(z + sensor_z)


def test_lidar_scan_plane_is_the_p299b_c1_datum(tmp_path):
    """P2-99c (a): C1 scan plane = pedestal top Z390 + 29.8 mm = 419.8 mm.

    The datum is the SCAN PLANE (sensor pose), not lidar_link: summing only
    the joint chain is exactly how the scan plane once ended up at 578 mm.
    """
    root = _generate(tmp_path, VARIANT)
    _, _, scan_z = _lidar_scan_origin(root)
    assert scan_z == pytest.approx(0.4198, abs=1e-6)
    assert _ground_height(root, "lidar_link") == pytest.approx(0.390, abs=1e-6)


def test_lidar_position_goes_through_the_d1_cad_offset(tmp_path):
    """CAD C1 centre [-198.5, +110] mm -> base_link via D1 (X-117.5 mm)."""
    root = _generate(tmp_path, VARIANT)
    origin = _joint(root, "lidar_joint").find("origin")
    x, y, z = (float(v) for v in origin.get("xyz").split())
    assert (x, y, z) == pytest.approx((-0.316, 0.110, 0.345), abs=1e-9)
    assert origin.get("rpy") is None or all(
        abs(float(v)) < 1e-12 for v in origin.get("rpy").split())


def test_c1_pedestal_carries_the_sensor_from_the_panel(tmp_path):
    root = _generate(tmp_path, VARIANT)
    pedestal = next(pts for link, name, pts in _visual_samples(root)
                    if name == "p299_c1_pedestal_vis")
    assert pedestal[:, 2].min() == pytest.approx(0.323, abs=1e-6)
    assert pedestal[:, 2].max() == pytest.approx(0.390, abs=1e-6)


def test_nothing_on_the_robot_enters_the_c1_scan_band(tmp_path):
    """Geometric +-2 deg band check (same criterion as P2-99a/b), 360 deg.

    Every visual outside lidar_link whose height range reaches
    Zscan +- r*tan(2 deg) at its own horizontal distance r is a blind sector
    for gpu_lidar (it renders visuals)."""
    root = _generate(tmp_path, VARIANT)
    lx, ly, lz = _lidar_scan_origin(root)
    slope = math.tan(math.radians(2.0))
    offenders = []
    for link, name, pts in _visual_samples(root):
        if link == "lidar_link":
            continue
        r = ((pts[:, 0] - lx) ** 2 + (pts[:, 1] - ly) ** 2) ** 0.5
        band = r.max() * slope
        if pts[:, 2].max() >= lz - band and pts[:, 2].min() <= lz + band:
            offenders.append(f"{link}/{name} z={pts[:, 2].min():.3f}..{pts[:, 2].max():.3f}")
    assert not offenders, offenders


def test_body_f_ko_lidar_ceiling(tmp_path):
    """D3: above ground 0.375 m only the C1, its pedestal and (until P2-99c
    step b moves it) the OAK-D camera may exist."""
    root = _generate(tmp_path, VARIANT)
    allowed = {"lidar_link", "camera_link"}
    above = [f"{link}/{name} zmax={pts[:, 2].max():.3f}"
             for link, name, pts in _visual_samples(root)
             if pts[:, 2].max() > 0.375 + 1e-9 and link not in allowed
             and name != "p299_c1_pedestal_vis"]
    assert not above, above
