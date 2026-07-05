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


def _patch_collision_surface(collision: ET.Element, mu: str, mu2: str, slip1: str, slip2: str) -> None:
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


def _patch_sdf_contacts(sdf_text: str) -> str:
    """Patch contact tuning and Gazebo-native intake collision geometry."""
    root = ET.fromstring(sdf_text)
    surfaces = {
        "rear_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "rear_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "lift_wheel_link": ("1.5", "1.5", "0.0", "0.0"),
    }
    for link in root.findall(".//link"):
        name = link.attrib.get("name")
        if name not in surfaces:
            continue
        for invalid in list(link):
            if invalid.tag in {"mu1", "mu2", "slip1", "slip2"}:
                link.remove(invalid)
        for collision in link.findall("collision"):
            _patch_collision_surface(collision, *surfaces[name])

    # URDF has no curved-prism primitive, and DART does not handle STL
    # collisions reliably here. Replace only the generated SDF collision with
    # a native extruded polyline of the roller-first intake channel — keep in
    # sync with scripts/generate_curved_scoop_mesh.py: mesh-local channel tip
    # at x=0.555, placed at effective x=0.540 by the -15 mm SDF pose
    # (fully BEHIND the roller's leading edge at 0.595, so the paddles are
    # the first contact), arc of radius 0.105 around the roller centre
    # (0.55, 0.105 above ground) back to x=0.445, near-vertical rear wall up
    # to 0.155. The 2-D x/z profile is extruded 180 mm across the intake
    # after rotating the extrusion axis onto Y.
    for collision in root.findall(".//collision"):
        if "intake_channel_col" not in collision.attrib.get("name", ""):
            continue
        pose = collision.find("pose")
        if pose is None:
            pose = ET.SubElement(collision, "pose")
        # rot X +90deg maps the +Z extrusion onto -Y: spans y=+0.09..-0.09.
        # Point y-coords below are funnel-frame z (ground at -0.038), so no
        # extra z offset is needed.
        # Match the -15 mm channel origin in funnel.urdf.xacro.
        pose.text = "-0.015 0.09 0 1.57079632679 0 0"
        geometry = collision.find("geometry")
        if geometry is None:
            geometry = ET.SubElement(collision, "geometry")
        geometry.clear()
        polyline = ET.SubElement(geometry, "polyline")
        ET.SubElement(polyline, "height").text = "0.18"
        ground = -0.038
        roller_x, roller_z, channel_r = 0.55, 0.105, 0.105
        # Thin 2 mm curved sheet. The previous polygon closed from the front
        # tip to a rear point on the court, creating a broad flat underside
        # that dragged under the robot. Build top and underside profiles so
        # only the front lip reaches the ground.
        top_points = [(0.4435, ground + 0.155), (0.445, ground + roller_z)]
        arc_steps = 24
        for i in range(1, arc_steps + 1):
            x = 0.445 + (roller_x - 0.445) * i / arc_steps
            dx = roller_x - x
            z = roller_z - (channel_r * channel_r - dx * dx) ** 0.5
            # A 3 mm floor leaves the 2 mm sheet underside 1 mm clear.
            top_points.append((round(x, 5), round(ground + max(z, 0.003), 5)))
        # Short tip behind the roller front; at ZERO height exactly on the
        # court so the blade gets underneath the rigid sphere (same trick as
        # the old wedge collision). The 2 mm tip stays in the visual mesh.
        top_points.append((0.555, ground + 0.002))
        sheet_thickness = 0.002
        collision_clearance = 0.001
        underside = [
            (x, max(ground + collision_clearance, z - sheet_thickness))
            for x, z in reversed(top_points)
        ]
        points = top_points + underside
        for px, pz in points:
            ET.SubElement(polyline, "point").text = f"{px} {pz}"
        # Smooth plastic channel. The paddled roller supplies traction; the
        # channel must not pin the ball against the court through friction.
        _patch_collision_surface(collision, "0.15", "0.15", "0.0", "0.0")

    # Funnel cheeks: smooth guides — low friction so an off-centre ball slides
    # toward the scoop instead of being grabbed and pushed.
    for collision in root.findall(".//collision"):
        if "cheek_col" in collision.attrib.get("name", ""):
            _patch_collision_surface(collision, "0.1", "0.1", "0.0", "0.0")

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
        print(f"Generated URDF is current: {output.relative_to(PROJECT_ROOT)}")
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(result.stdout, encoding="utf-8")
    print(f"Generated {output.relative_to(PROJECT_ROOT)} from {source.relative_to(PROJECT_ROOT)}")

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
        sdf_output.parent.mkdir(parents=True, exist_ok=True)
        sdf_output.write_text(_patch_sdf_contacts(sdf_result.stdout), encoding="utf-8")
        print(f"Generated {sdf_output.relative_to(PROJECT_ROOT)} with patched contact surfaces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
