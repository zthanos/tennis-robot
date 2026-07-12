"""Generate the tennis robot URDF from its xacro source."""

from __future__ import annotations

import argparse
import math
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
    lip_height_m = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.0")))
    # Keep the hand-carved Gazebo collision in sync with the generated visual
    # mesh. The ramp entry sits just ahead of the wheel nip so the wheels feed
    # the ball onto a rising surface; the rear endpoint stays at the basket
    # floor.
    # Defaults are the Phase 1-4 bench-proven geometry (debug-log #41-#42):
    # the wheels kick the ball up-back, the short bar becomes a ski-jump that
    # converts the kick's horizontal KE into climb over a 3 mm retention lip
    # into the chassis-flush hopper.
    nip_x_m = float(os.getenv("INTAKE_NIP_X_M", "0.540"))
    ramp_entry_x_m = float(os.getenv("INTAKE_RAMP_ENTRY_X_M", str(nip_x_m - 0.040)))
    ramp_clear_run_m = max(0.004, float(os.getenv("INTAKE_RAMP_CLEAR_RUN_M", "0.030")))
    ramp_clear_z_m = max(lip_height_m, float(os.getenv("INTAKE_RAMP_CLEAR_Z_M", "0.004")))
    ramp_knee_x_m = float(os.getenv("INTAKE_RAMP_KNEE_X_M", "0.465"))
    ramp_end_x_m = float(os.getenv("INTAKE_RAMP_END_X_M", "0.425"))
    surfaces = {
        "rear_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "rear_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_left_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        "front_right_wheel_link": ("1.2", "1.2", "0.0", "0.0"),
        # Dual intake wheels: rubber/foam sleeves. High friction gives grip;
        # the soft contact terms keep the nominal 3 mm/side interference from
        # behaving like rigid metal (the carriage springs below provide the
        # actual compliance).
        "intake_wheel_left_link": ("2.5", "2.5", "0.0", "0.0"),
        "intake_wheel_right_link": ("2.5", "2.5", "0.0", "0.0"),
    }
    wheel_soft_contact = {
        "kp": os.getenv("INTAKE_WHEEL_CONTACT_KP", "8000"),
        "kd": os.getenv("INTAKE_WHEEL_CONTACT_KD", "35"),
        "max_vel": os.getenv("INTAKE_WHEEL_CONTACT_MAX_VEL", "0.6"),
        "min_depth": os.getenv("INTAKE_WHEEL_CONTACT_MIN_DEPTH", "0.001"),
    }
    soft_contacts = {
        "intake_wheel_left_link": wheel_soft_contact,
        "intake_wheel_right_link": wheel_soft_contact,
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

    # sdformat may rename collisions while converting the URDF, so locate each
    # intake wheel's actual collision name and point its contact sensor at it.
    # Fail loud on both wheels: a silent mismatch means the bench counts zero
    # contacts forever.
    wheel_sensors = {
        "roller_contact_0": "intake_wheel_left_link",
        "roller_contact_1": "intake_wheel_right_link",
    }
    wheel_col_names: dict[str, str] = {}
    for link in root.findall(".//link"):
        lname = link.attrib.get("name", "")
        if lname not in wheel_sensors.values():
            continue
        collision = link.find("collision")
        if collision is not None:
            wheel_col_names[lname] = collision.attrib.get("name", "")
    missing = [l for l in wheel_sensors.values() if not wheel_col_names.get(l)]
    if missing:
        raise RuntimeError(f"expected intake wheel collisions for contact sensors: {missing}")
    patched_sensors: set[str] = set()
    for sensor in root.findall(".//sensor"):
        sname = sensor.attrib.get("name", "")
        if sname not in wheel_sensors:
            continue
        collision = sensor.find("./contact/collision")
        if collision is not None:
            collision.text = wheel_col_names[wheel_sensors[sname]]
            patched_sensors.add(sname)
    if patched_sensors != set(wheel_sensors):
        raise RuntimeError(
            f"expected both intake wheel contact sensors, patched only {sorted(patched_sensors)}"
        )

    # Lateral compliance for the rigid nip (docs/dual-wheel-intake-design-el.md):
    # each wheel's passive prismatic y-carriage gets a spring so the nominal
    # 3 mm/side interference becomes grip force instead of a rigid jam
    # (lesson of collect_test1/2; same SDF spring technique as debug-log #9).
    # URDF cannot express joint springs, hence the SDF patch. Fail loud if the
    # carriage joints are missing.
    spring_k = os.getenv("INTAKE_WHEEL_SPRING_K", "1000")
    carriage_joints = {
        "intake_wheel_left_carriage_joint",
        "intake_wheel_right_carriage_joint",
    }
    patched_joints: set[str] = set()
    for joint in root.findall(".//joint"):
        jname = joint.attrib.get("name", "")
        if jname not in carriage_joints:
            continue
        axis = joint.find("axis")
        if axis is None:
            raise RuntimeError(f"carriage joint {jname} has no <axis> to patch")
        dynamics = axis.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(axis, "dynamics")
        _set_text(dynamics, "spring_reference", "0")
        _set_text(dynamics, "spring_stiffness", spring_k)
        patched_joints.add(jname)
    if patched_joints != carriage_joints:
        raise RuntimeError(
            f"expected carriage joints {sorted(carriage_joints)}, patched {sorted(patched_joints)}"
        )

    # URDF has no ramp-prism primitive, and DART does not handle STL
    # collisions reliably here. Replace only the generated SDF collision with
    # short box segments that approximate the elevation ramp. Keep this shape
    # in sync with scripts/generate_curved_scoop_mesh.py. When the ramp is
    # disabled (enable_ramp:=false, validation Phases 1-2) there is no
    # intake_channel_col in the SDF and this whole block is a no-op.
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
            geometry = collision.find("geometry")
            if geometry is None:
                geometry = ET.SubElement(collision, "geometry")
            geometry.clear()
            lip_x = ramp_entry_x_m
            ramp_clear_x = lip_x - ramp_clear_run_m
            ramp_knee_x = ramp_knee_x_m
            ramp_end_x = ramp_end_x_m
            ramp_clear_z = ramp_clear_z_m
            ramp_knee_z = float(os.getenv("INTAKE_RAMP_KNEE_Z_M", "0.020"))
            ramp_end_z = float(os.getenv("INTAKE_RAMP_END_Z_M", "0.055"))
            ramp_steps = 28
            sheet_thickness = 0.002
            collision_clearance = 0.001
            channel_origin_x = -0.015
            channel_width = 0.18

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
                top_points.append((x, ramp_z(x)))

            ramp_collisions = [collision]
            base_name = intake_channel_collision_name or "intake_channel_col"
            for i in range(1, ramp_steps):
                ramp_collisions.append(
                    ET.SubElement(
                        intake_channel_link,
                        "collision",
                        {"name": f"{base_name}_seg_{i:02d}"},
                    )
                )

            for i, segment_collision in enumerate(ramp_collisions):
                (x0, z0), (x1, z1) = top_points[i], top_points[i + 1]
                dx = x0 - x1
                dz = z1 - z0
                length = max(0.001, (dx * dx + dz * dz) ** 0.5)
                pitch = math.atan2(dz, dx)
                thickness = max(0.003, sheet_thickness + collision_clearance)
                normal_x = math.sin(pitch)
                normal_z = math.cos(pitch)
                center_x = (x0 + x1) / 2.0 - normal_x * thickness / 2.0
                center_z = (z0 + z1) / 2.0 - normal_z * thickness / 2.0

                segment_pose = segment_collision.find("pose")
                if segment_pose is None:
                    segment_pose = ET.SubElement(segment_collision, "pose")
                segment_pose.text = (
                    f"{channel_origin_x + center_x:.6f} "
                    f"0 "
                    f"{max(collision_clearance + thickness / 2.0, center_z):.6f} "
                    f"0 {pitch:.6f} 0"
                )

                segment_geometry = segment_collision.find("geometry")
                if segment_geometry is None:
                    segment_geometry = ET.SubElement(segment_collision, "geometry")
                segment_geometry.clear()
                box = ET.SubElement(segment_geometry, "box")
                ET.SubElement(box, "size").text = (
                    f"{length:.6f} {channel_width:.6f} {thickness:.6f}"
                )
                # Low/medium ramp friction: enough to guide the ball, not so
                # much that the scoop becomes a brake or a conveyor wall.
                _patch_collision_surface(segment_collision, "0.35", "0.35", "0.0", "0.0")
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
            # Dual-wheel intake tuning + Concept Validation Plan gates
            # (docs/dual-wheel-intake-design-el.md).
            f"intake_wheel_radius:={os.getenv('INTAKE_WHEEL_RADIUS_M', '0.060')}",
            f"intake_wheel_gap:={os.getenv('INTAKE_WHEEL_GAP_M', '0.056')}",
            f"intake_nip_x:={os.getenv('INTAKE_NIP_X_M', '0.540')}",
            f"intake_wheel_tilt_deg:={os.getenv('INTAKE_WHEEL_TILT_DEG', '35.0')}",
            f"intake_wheel_max_vel:={os.getenv('INTAKE_WHEEL_MAX_VEL_RAD_S', '26.3')}",
            f"intake_wheel_effort:={os.getenv('INTAKE_WHEEL_EFFORT_NM', '1.77')}",
            f"enable_funnel:={os.getenv('INTAKE_ENABLE_FUNNEL', 'true')}",
            f"enable_ramp:={os.getenv('INTAKE_ENABLE_RAMP', 'true')}",
            f"enable_assist:={os.getenv('INTAKE_ENABLE_ASSIST', 'false')}",
            f"enable_conveyor:={os.getenv('INTAKE_ENABLE_CONVEYOR', 'false')}",
            f"intake_assist_x:={os.getenv('INTAKE_ASSIST_X_M', '0.545')}",
            f"intake_assist_z:={os.getenv('INTAKE_ASSIST_Z_M', '0.050')}",
            f"intake_assist_radius:={os.getenv('INTAKE_ASSIST_RADIUS_M', '0.030')}",
            f"intake_assist_length:={os.getenv('INTAKE_ASSIST_LENGTH_M', '0.200')}",
            f"intake_conveyor_x_bias:={os.getenv('INTAKE_CONVEYOR_X_BIAS_M', '0.000')}",
            f"intake_conveyor_z_bias:={os.getenv('INTAKE_CONVEYOR_Z_BIAS_M', '0.000')}",
            f"basket_floor_front_x:={os.getenv('INTAKE_BASKET_FLOOR_FRONT_X_M', '0.42')}",
            f"basket_floor_top_z:={os.getenv('INTAKE_BASKET_FLOOR_TOP_Z_M', '0.030')}",
            f"basket_rear_x:={os.getenv('BASKET_REAR_X_M', '0.02')}",
            f"basket_half_width:={os.getenv('BASKET_HALF_WIDTH_M', '0.14')}",
            f"basket_wall_top_z:={os.getenv('BASKET_WALL_TOP_Z_M', '0.25')}",
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
