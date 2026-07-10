"""Generate the tennis robot URDF from its xacro source."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "ros2_ws" / "src" / "tennis_robot" / "urdf" / "tennis_robot.urdf.xacro"
DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "tennis_robot.urdf"
DEFAULT_SDF_OUTPUT = PROJECT_ROOT / "runtime" / "tennis_robot.sdf"
DEFAULT_CONTROLLERS = (
    PROJECT_ROOT / "ros2_ws" / "src" / "tennis_robot" / "config" / "controllers.yaml"
)


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--sim-mode",
        default="true",
        choices=["true", "false"],
        help="ros2_control hardware backend: true=GazeboSimSystem, false=real robot.",
    )
    parser.add_argument(
        "--controllers-config",
        type=Path,
        default=DEFAULT_CONTROLLERS,
        help="Absolute path to controllers.yaml baked into the gz_ros2_control plugin.",
    )
    parser.add_argument(
        "--xacro-command",
        default="xacro",
        help="xacro executable to use; defaults to PATH lookup.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if the output URDF is missing or does not match the rendered xacro.",
    )
    parser.add_argument(
        "--sdf-output",
        type=Path,
        default=None,
        help="Optional Gazebo SDF output generated from the URDF with patched contact surfaces.",
    )
    return parser.parse_args()


def _set_text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = value
    return child


def _replace_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp")
    tmp_path.write_text(text, encoding="utf-8")
    tmp_path.replace(path)


def _patch_collision_surface(
    collision: ET.Element,
    mu: str,
    mu2: str,
    slip1: str,
    slip2: str,
    contact_ode: dict[str, str] | None = None,
) -> None:
    surface = collision.find("surface")
    if surface is None:
        surface = ET.SubElement(collision, "surface")
    friction = surface.find("friction")
    if friction is None:
        friction = ET.SubElement(surface, "friction")
    ode = friction.find("ode")
    if ode is None:
        ode = ET.SubElement(friction, "ode")
    _set_text(ode, "mu", mu)
    _set_text(ode, "mu2", mu2)
    _set_text(ode, "slip1", slip1)
    _set_text(ode, "slip2", slip2)
    if contact_ode:
        contact = surface.find("contact")
        if contact is None:
            contact = ET.SubElement(surface, "contact")
        contact_ode_node = contact.find("ode")
        if contact_ode_node is None:
            contact_ode_node = ET.SubElement(contact, "ode")
        for tag, value in contact_ode.items():
            _set_text(contact_ode_node, tag, value)


def _patch_sdf_contacts(sdf_text: str) -> str:
    """Patch contact tuning and Gazebo-native intake collision geometry."""
    root = ET.fromstring(sdf_text)
    lip_height_m = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.002")))
    # Keep the hand-carved Gazebo collision in sync with the generated visual
    # mesh. The front lip follows the roller X tuning; the rear ramp endpoint
    # stays fixed at the basket floor.
    roller_x_offset_m = float(os.getenv("INTAKE_ROLLER_X_OFFSET_M", "0.0"))
    lip_x_offset_m = float(os.getenv("INTAKE_LIP_X_OFFSET_M", "0.0"))
    ramp_clear_run_m = max(0.004, float(os.getenv("INTAKE_RAMP_CLEAR_RUN_M", "0.030")))
    ramp_clear_z_m = max(lip_height_m, float(os.getenv("INTAKE_RAMP_CLEAR_Z_M", "0.004")))
    surfaces = {
        "rear_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "rear_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        # Continuous foam/rubber intake roller. High friction gives bite; the
        # soft contact terms keep the small geometry overlap from behaving
        # like a rigid metal cylinder.
        "lift_wheel_link": ("2.0", "2.0", "0.0", "0.0"),
    }
    soft_contacts = {
        "lift_wheel_link": {
            "kp": os.getenv("INTAKE_ROLLER_CONTACT_KP", "8000"),
            "kd": os.getenv("INTAKE_ROLLER_CONTACT_KD", "35"),
            "max_vel": os.getenv("INTAKE_ROLLER_CONTACT_MAX_VEL", "0.6"),
            "min_depth": os.getenv("INTAKE_ROLLER_CONTACT_MIN_DEPTH", "0.001"),
        },
    }
    for link in root.findall(".//link"):
        name = link.attrib.get("name")
        if name not in surfaces:
            continue
        for invalid in list(link):
            if invalid.tag in {"mu1", "mu2", "slip1", "slip2"}:
                link.remove(invalid)
        for collision in link.findall("collision"):
            _patch_collision_surface(collision, *surfaces[name], soft_contacts.get(name))

    # sdformat may rename collisions while converting the URDF, so locate the
    # roller's actual collision name and point the contact sensor at it.
    roller_col_name = ""
    for link in root.findall(".//link"):
        lname = link.attrib.get("name", "")
        if lname != "lift_wheel_link":
            continue
        collision = link.find("collision")
        if collision is not None:
            roller_col_name = collision.attrib.get("name", "")
            break
    if not roller_col_name:
        raise RuntimeError("expected lift_wheel_link collision for roller contact sensor")
    for sensor in root.findall(".//sensor"):
        sname = sensor.attrib.get("name", "")
        if sname != "roller_contact_0":
            continue
        collision = sensor.find("./contact/collision")
        if collision is not None:
            collision.text = roller_col_name

    # URDF has no ramp-prism primitive, and DART does not handle STL
    # collisions reliably here. Replace only the generated SDF collision with
    # a native extruded polyline of the launcher scoop. Keep this shape in
    # sync with scripts/generate_curved_scoop_mesh.py: short 2 mm lip for
    # initial roller bite, quick clearance opening, then a free ramp into the
    # basket instead of a continuous roller/scoop pinch.
    intake_channel_link: ET.Element | None = None
    intake_channel_collision_name = ""
    for link in root.findall(".//link"):
        for collision in link.findall("collision"):
            if "intake_channel_col" not in collision.attrib.get("name", ""):
                continue
            intake_channel_link = link
            intake_channel_collision_name = collision.attrib.get("name", "")
            pose = collision.find("pose")
            if pose is None:
                pose = ET.SubElement(collision, "pose")
            # rot X +90deg maps the +Z extrusion onto -Y: spans y=+0.09..-0.09.
            # Point y-coords below are funnel-frame z (ground at -0.038). Gazebo
            # lumps the fixed funnel_link into base_footprint, so preserve the
            # funnel frame's +38 mm height when replacing the original collision.
            # Match the -15 mm channel origin in funnel.urdf.xacro.
            pose.text = "-0.015 0.09 0.038 1.57079632679 0 0"
            geometry = collision.find("geometry")
            if geometry is None:
                geometry = ET.SubElement(collision, "geometry")
            geometry.clear()
            polyline = ET.SubElement(geometry, "polyline")
            ET.SubElement(polyline, "height").text = "0.18"
            ground = -0.038
            lip_x = 0.600 + roller_x_offset_m + lip_x_offset_m
            ramp_clear_x = lip_x - ramp_clear_run_m
            ramp_knee_x = 0.520
            ramp_end_x = 0.400
            ramp_clear_z = ramp_clear_z_m
            ramp_knee_z = 0.024
            ramp_end_z = 0.128
            ramp_steps = 28
            sheet_thickness = 0.002
            collision_clearance = 0.001

            def smoothstep(t: float) -> float:
                t = max(0.0, min(1.0, t))
                return t * t * (3.0 - 2.0 * t)

            def ramp_z(x: float) -> float:
                if x >= ramp_clear_x:
                    t = (lip_x - x) / max(lip_x - ramp_clear_x, 1e-6)
                    return lip_height_m + (ramp_clear_z - lip_height_m) * smoothstep(t)
                if x >= ramp_knee_x:
                    t = (ramp_clear_x - x) / max(ramp_clear_x - ramp_knee_x, 1e-6)
                    return ramp_clear_z + (ramp_knee_z - ramp_clear_z) * smoothstep(t)
                t = (ramp_knee_x - x) / max(ramp_knee_x - ramp_end_x, 1e-6)
                return ramp_knee_z + (ramp_end_z - ramp_knee_z) * smoothstep(t)

            top_points = []
            for i in range(ramp_steps + 1):
                t = i / ramp_steps
                x = lip_x + (ramp_end_x - lip_x) * t
                top_points.append((round(x, 5), round(ground + ramp_z(x), 5)))
            underside = [
                (x, max(ground + collision_clearance, z - sheet_thickness))
                for x, z in reversed(top_points)
            ]
            points = top_points + underside
            for px, pz in points:
                ET.SubElement(polyline, "point").text = f"{px} {pz}"
            # Low/medium ramp friction: enough to guide the launched ball, not so
            # much that the scoop becomes a brake or a conveyor wall.
            _patch_collision_surface(collision, "0.35", "0.35", "0.0", "0.0")
    if intake_channel_link is not None and intake_channel_collision_name:
        for old_sensor in list(intake_channel_link.findall("sensor")):
            if old_sensor.attrib.get("name") == "lip_contact_0":
                intake_channel_link.remove(old_sensor)
        sensor = ET.SubElement(
            intake_channel_link,
            "sensor",
            {"name": "lip_contact_0", "type": "contact"},
        )
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "100"
        ET.SubElement(sensor, "topic").text = "/gz/lip_contact_0"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = intake_channel_collision_name

    # Funnel cheeks: smooth guides — low friction so an off-centre ball slides
    # toward the scoop instead of being grabbed and pushed.
    for collision in root.findall(".//collision"):
        if "cheek_col" in collision.attrib.get("name", ""):
            _patch_collision_surface(collision, "0.1", "0.1", "0.0", "0.0")

    # High safety deflector only; keep it comparatively slick so it does not
    # become another transport surface if a launched ball clips it.
    for collision in root.findall(".//collision"):
        if "intake_deflector_col" in collision.attrib.get("name", ""):
            _patch_collision_surface(collision, "0.2", "0.2", "0.0", "0.0")

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()

    if not source.exists():
        print(f"Source xacro not found: {source}", file=sys.stderr)
        return 2

    xacro_exe = shutil.which(args.xacro_command)
    if xacro_exe is None:
        print(
            "xacro executable not found. Install ros-humble-xacro or source a ROS "
            "environment that provides xacro.",
            file=sys.stderr,
        )
        return 2

    env = os.environ.copy()
    env.setdefault("PYTHONUTF8", "1")
    controllers_config = args.controllers_config.resolve()
    result = subprocess.run(
        [
            xacro_exe,
            str(source),
            f"sim_mode:={args.sim_mode}",
            f"controllers_config:={controllers_config}",
            f"intake_roller_x_offset:={os.getenv('INTAKE_ROLLER_X_OFFSET_M', '0.0')}",
            f"intake_roller_z_offset:={os.getenv('INTAKE_ROLLER_Z_OFFSET_M', '0.0')}",
        ],
        check=False,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr, end="")
        return result.returncode

    ET.fromstring(result.stdout)
    if args.check:
        if not output.exists():
            print(f"Generated URDF is missing: {output}", file=sys.stderr)
            return 1
        current = output.read_text(encoding="utf-8")
        if current != result.stdout:
            print(
                f"Generated URDF is stale: {output.relative_to(PROJECT_ROOT)}",
                file=sys.stderr,
            )
            return 1
        print(f"Generated URDF is current: {_display_path(output)}")
        return 0

    _replace_text(output, result.stdout)
    print(f"Generated {_display_path(output)} from {_display_path(source)}")

    if args.sdf_output is not None:
        sdf_output = args.sdf_output.resolve()
        sdf_result = subprocess.run(
            ["gz", "sdf", "-p", str(output)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if sdf_result.returncode != 0:
            print(sdf_result.stderr, file=sys.stderr, end="")
            return sdf_result.returncode
        _replace_text(sdf_output, _patch_sdf_contacts(sdf_result.stdout))
        print(f"Generated {_display_path(sdf_output)} with patched contact surfaces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
