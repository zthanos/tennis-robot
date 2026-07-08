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
    lip_raise_m = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.0")))
    lip_raise_taper_m = 0.020
    # Must track tennis_robot.urdf.xacro's intake_x/intake_z (same envs) so
    # this hand-carved channel collision stays concentric with the ACTUAL
    # roller instead of the old fixed baseline. Keep in sync with
    # scripts/generate_curved_scoop_mesh.py, which applies the same offsets
    # to the visual mesh.
    roller_x_offset_m = float(os.getenv("INTAKE_ROLLER_X_OFFSET_M", "0.0"))
    roller_z_offset_m = float(os.getenv("INTAKE_ROLLER_Z_OFFSET_M", "0.0"))
    surfaces = {
        "rear_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "rear_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        # Hub: rarely ball-facing; the grip comes from the rubber paddles.
        "lift_wheel_link": ("0.8", "0.8", "0.0", "0.0"),
    }
    for index in range(8):
        surfaces[f"paddle_{index}_link"] = ("2.5", "2.5", "0.0", "0.0")
    for link in root.findall(".//link"):
        name = link.attrib.get("name")
        if name not in surfaces:
            continue
        for invalid in list(link):
            if invalid.tag in {"mu1", "mu2", "slip1", "slip2"}:
                link.remove(invalid)
        for collision in link.findall("collision"):
            _patch_collision_surface(collision, *surfaces[name])

    # sdformat may rename collisions while converting the URDF (fixed-joint
    # lumping etc.), so locate each paddle's ACTUAL collision name and point
    # its contact sensor at it. A hardcoded name here would silently produce
    # dead sensors (0 contacts forever) whenever the link structure changes.
    paddle_col_names: dict[str, str] = {}
    for link in root.findall(".//link"):
        lname = link.attrib.get("name", "")
        if not (lname.startswith("paddle_") and lname.endswith("_link")):
            continue
        collision = link.find("collision")
        if collision is not None:
            paddle_col_names[lname[len("paddle_"):-len("_link")]] = (
                collision.attrib.get("name", "")
            )
    if len(paddle_col_names) != 8:
        raise RuntimeError(
            f"expected 8 paddle collisions, found {len(paddle_col_names)} — "
            "contact sensors would be dead; refusing to generate a broken SDF."
        )
    for sensor in root.findall(".//sensor"):
        sname = sensor.attrib.get("name", "")
        if not sname.startswith("roller_contact_"):
            continue
        index = sname[len("roller_contact_"):]
        collision = sensor.find("./contact/collision")
        if collision is not None and index in paddle_col_names:
            collision.text = paddle_col_names[index]

    # Paddle springs (foam-compliance emulation): URDF cannot express joint
    # springs, so inject the SDF spring params here. k=0.25 N*m/rad at the
    # 25 mm paddle lever gives a few newtons of grip at the ~40 deg bend the
    # 59 mm guide passage forces on a 66 mm ball — soft enough to wrap around
    # the ball, stiff enough to drive it (intake-debug-log #14).
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name", "")
        if not (name.startswith("paddle_") and name.endswith("_joint")):
            continue
        axis = joint.find("axis")
        if axis is None:
            continue
        dynamics = axis.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(axis, "dynamics")
        _set_text(dynamics, "spring_reference", "0.0")
        _set_text(dynamics, "spring_stiffness", "0.25")
        _set_text(dynamics, "damping", "0.02")

    # URDF has no curved-prism primitive, and DART does not handle STL
    # collisions reliably here. Replace only the generated SDF collision with
    # a native extruded polyline of the roller-first intake channel — keep in
    # sync with scripts/generate_curved_scoop_mesh.py: mesh-local channel tip
    # at x=0.600, placed at effective x=0.585 by the -15 mm SDF pose. This is
    # behind the roller axis, ensuring that the roller is the first hard
    # contact. The 108 mm arc
    # is concentric with the effective roller centre (0.600, 0.112), giving
    # 63 mm clearance / 3 mm nominal interference for a 66 mm rigid ball.
    # The 2-D x/z profile is extruded 180 mm across the intake
    # after rotating the extrusion axis onto Y.
    for collision in root.findall(".//collision"):
        if "intake_channel_col" not in collision.attrib.get("name", ""):
            continue
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
        roller_x = 0.615 + roller_x_offset_m
        roller_z = 0.112 + roller_z_offset_m
        # 64 mm roller-to-arc passage: 2 mm nominal squeeze on a 66 mm ball,
        # absorbed by the sprung roller carriage (grip force, not a rigid
        # jam). 113 mm opened a dead pocket behind the axis where the ball
        # lost roller contact (intake-debug-log #9). Keep in sync with
        # generate_curved_scoop_mesh.py.
        channel_r = 0.109
        lip_x = 0.600 + roller_x_offset_m
        # Thin 2 mm curved sheet. The previous polygon closed from the front
        # tip to a rear point on the court, creating a broad flat underside
        # that dragged under the robot. Build top and underside profiles so
        # only the front lip reaches the ground.
        arc_back_x = roller_x - channel_r
        # Concentric guide up the BACK of the roller (top edge first, then
        # down to the arc base): the roller drives the ball tangentially the
        # whole way to the release edge — the old near-vertical wall left a
        # ~58 mm unpowered gap above the roller's reach that stalled every
        # capture (intake-debug-log #12). Keep in sync with
        # scripts/generate_curved_scoop_mesh.py.
        import math as _math
        guide_top_z = 0.155
        sin_max = max(0.0, min(1.0, (guide_top_z - roller_z) / channel_r))
        phi_max = _math.asin(sin_max)
        guide_steps = 10
        top_points = []
        for i in range(guide_steps, 0, -1):
            phi = phi_max * i / guide_steps
            top_points.append((
                round(roller_x - channel_r * _math.cos(phi), 5),
                round(ground + roller_z + channel_r * _math.sin(phi), 5),
            ))
        top_points.append((arc_back_x, ground + roller_z))
        arc_steps = 24
        for i in range(1, arc_steps + 1):
            x = arc_back_x + (lip_x - arc_back_x) * i / arc_steps
            dx = roller_x - x
            z = roller_z - (channel_r * channel_r - dx * dx) ** 0.5
            if lip_raise_m > 0.0:
                lip_t = max(0.0, min(1.0, 1.0 - abs(lip_x - x) / lip_raise_taper_m))
                z += lip_raise_m * lip_t
            # A 3 mm floor leaves the 2 mm sheet underside 1 mm clear.
            top_points.append((round(x, 5), round(ground + max(z, 0.003), 5)))
        # No forward lead-in: the arc ends 15 mm behind the roller axis, so
        # the channel cannot push the ball before roller contact.
        sheet_thickness = 0.002
        collision_clearance = 0.001
        # Underside: vertical offset for the floor/arc part, but RADIAL
        # (away from the roller) for the guide part — a vertical offset there
        # would poke 2 mm into the ball passage.
        underside = [
            (x, max(ground + collision_clearance, z - sheet_thickness))
            for x, z in reversed(top_points[guide_steps:])
        ]
        for i in range(1, guide_steps + 1):
            phi = phi_max * i / guide_steps
            underside.append((
                round(roller_x - (channel_r + sheet_thickness) * _math.cos(phi), 5),
                round(ground + roller_z + (channel_r + sheet_thickness) * _math.sin(phi), 5),
            ))
        points = top_points + underside
        for px, pz in points:
            ET.SubElement(polyline, "point").text = f"{px} {pz}"
        # Grippy channel: the ball climbs the rear wall by converting its
        # roller-driven spin into translation through WALL friction — with a
        # slick wall it spins in place like a bearing, pinned at the top of
        # the arc. Measured force balance at the collect_test4/5 stall point
        # (intake-debug-log #10-#11): needs mu_wall*N > 0.65 N with N≈0.8-1.3 N
        # from the roller spring — mu 0.8 gave 0.62 N (borderline stuck),
        # 1.4 gives ~2.7x margin. Real collectors line this wall with rubber.
        _patch_collision_surface(collision, "1.4", "1.4", "0.0", "0.0")

    # Funnel cheeks: smooth guides — low friction so an off-centre ball slides
    # toward the scoop instead of being grabbed and pushed.
    for collision in root.findall(".//collision"):
        if "cheek_col" in collision.attrib.get("name", ""):
            _patch_collision_surface(collision, "0.1", "0.1", "0.0", "0.0")

    # Spin-walk ceiling (intake_deflector): needs grip so the backspinning
    # ball pressed against its underside is friction-walked backward into
    # the basket (intake-debug-log #15).
    for collision in root.findall(".//collision"):
        if "intake_deflector_col" in collision.attrib.get("name", ""):
            _patch_collision_surface(collision, "1.2", "1.2", "0.0", "0.0")

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
