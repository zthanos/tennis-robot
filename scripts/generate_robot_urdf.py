"""Generate the tennis robot URDF from its xacro source."""

from __future__ import annotations

import argparse
import copy
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
        "--packaging-variant",
        default=os.getenv("ROBOT_PACKAGING_VARIANT", "baseline"),
        choices=["baseline", "compact"],
        help=(
            "Robot packaging model. baseline preserves the frozen collection "
            "simulation; compact enables the -100 mm functional shift, rear "
            "electronics and flywheel mass model."
        ),
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


def _patch_collision_bounce(
    collision: ET.Element,
    restitution: str,
    threshold: str,
) -> None:
    surface = collision.find("surface")
    if surface is None:
        surface = ET.SubElement(collision, "surface")
    bounce = surface.find("bounce")
    if bounce is None:
        bounce = ET.SubElement(surface, "bounce")
    _set_text(bounce, "restitution_coefficient", restitution)
    _set_text(bounce, "threshold", threshold)


def _patch_sdf_contacts(sdf_text: str, packaging_variant: str = "baseline") -> str:
    """Patch contact tuning and Gazebo-native intake collision geometry."""
    root = ET.fromstring(sdf_text)
    lip_height_m = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.0")))
    ramp_profile = os.getenv("INTAKE_RAMP_PROFILE", "launch").strip().lower()
    if ramp_profile not in {"rolling", "launch"}:
        raise ValueError(
            f"unsupported INTAKE_RAMP_PROFILE={ramp_profile!r}; use rolling or launch"
        )
    # Keep the hand-carved Gazebo collision in sync with the generated visual
    # mesh. The ramp entry sits just ahead of the wheel nip so the wheels feed
    # the ball onto a rising surface; the rear endpoint stays at the basket
    # floor.
    # Defaults are the Phase 1-4 bench-proven geometry (debug-log #41-#42):
    # the wheels kick the ball up-back, the short bar becomes a ski-jump that
    # converts the kick's horizontal KE into climb over a 3 mm retention lip
    # into the chassis-flush hopper.
    compact_packaging = packaging_variant == "compact"
    nip_x_m = float(os.getenv("INTAKE_NIP_X_M", "0.540"))
    if compact_packaging:
        # Funnel link itself is shifted -0.100 m. These are local link-frame
        # values: channel_origin_x=-0.015 makes the world ramp run 0.360..0.320,
        # matching the corrected OpenSCAD handoff behind the X=0.370 wheel nip.
        ramp_entry_default_m = 0.475
    else:
        ramp_entry_default_m = nip_x_m if ramp_profile == "launch" else nip_x_m - 0.040
    ramp_entry_x_m = float(
        os.getenv("INTAKE_RAMP_ENTRY_X_M", str(ramp_entry_default_m))
    )
    ramp_clear_run_m = max(0.004, float(os.getenv("INTAKE_RAMP_CLEAR_RUN_M", "0.030")))
    ramp_clear_z_m = max(lip_height_m, float(os.getenv("INTAKE_RAMP_CLEAR_Z_M", "0.004")))
    ramp_knee_x_m = float(os.getenv("INTAKE_RAMP_KNEE_X_M", "0.465"))
    ramp_end_x_m = float(os.getenv("INTAKE_RAMP_END_X_M", "0.425"))
    launch_exit_x_m = float(
        os.getenv("INTAKE_LAUNCH_EXIT_X_M", "0.435" if compact_packaging else "0.465")
    )
    launch_exit_z_m = float(os.getenv("INTAKE_LAUNCH_EXIT_Z_M", "0.032"))
    launch_exit_angle_deg = float(os.getenv("INTAKE_LAUNCH_EXIT_ANGLE_DEG", "35.0"))
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
        "flywheel_left_link": ("2.0", "2.0", "0.0", "0.0"),
        "flywheel_right_link": ("2.0", "2.0", "0.0", "0.0"),
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
        "flywheel_left_link": {
            "kp": os.getenv("FLYWHEEL_CONTACT_KP", "18000"),
            "kd": os.getenv("FLYWHEEL_CONTACT_KD", "45"),
            "max_vel": os.getenv("FLYWHEEL_CONTACT_MAX_VEL", "1.0"),
            "min_depth": os.getenv("FLYWHEEL_CONTACT_MIN_DEPTH", "0.0005"),
        },
        "flywheel_right_link": {
            "kp": os.getenv("FLYWHEEL_CONTACT_KP", "18000"),
            "kd": os.getenv("FLYWHEEL_CONTACT_KD", "45"),
            "max_vel": os.getenv("FLYWHEEL_CONTACT_MAX_VEL", "1.0"),
            "min_depth": os.getenv("FLYWHEEL_CONTACT_MIN_DEPTH", "0.0005"),
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

    # Lateral compliance for the rigid nip (docs/mechanism/dual-wheel-intake-design-el.md):
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
            ramp_end_x = (
                launch_exit_x_m if ramp_profile == "launch" else ramp_end_x_m
            )
            ramp_clear_z = ramp_clear_z_m
            ramp_knee_z = float(os.getenv("INTAKE_RAMP_KNEE_Z_M", "0.020"))
            ramp_end_z = float(os.getenv("INTAKE_RAMP_END_Z_M", "0.045"))
            ramp_steps = 28
            sheet_thickness = 0.002
            collision_clearance = 0.001
            # gz sdf fixed-joint reduction lumps funnel_link into base_link.
            # Because this patch replaces the converted pose after lumping, it
            # must reapply the compact funnel joint's -100 mm world shift.
            channel_origin_x = -0.015 + (-0.100 if compact_packaging else 0.0)
            channel_width = 0.18

            def smoothstep(t: float) -> float:
                t = max(0.0, min(1.0, t))
                return t * t * (3.0 - 2.0 * t)

            def ramp_z(x: float) -> float:
                if ramp_profile == "launch":
                    run = lip_x - ramp_end_x
                    if run <= 0.0:
                        raise ValueError(
                            "launch exit x must be behind the ramp entry x "
                            f"({ramp_end_x} >= {lip_x})"
                        )
                    t = max(0.0, min(1.0, (lip_x - x) / run))
                    z0 = lip_height_m
                    z1 = launch_exit_z_m
                    m0 = 0.0
                    m1 = math.tan(math.radians(launch_exit_angle_deg)) * run
                    h00 = 2.0 * t**3 - 3.0 * t**2 + 1.0
                    h10 = t**3 - 2.0 * t**2 + t
                    h01 = -2.0 * t**3 + 3.0 * t**2
                    h11 = t**3 - t**2
                    return h00 * z0 + h10 * m0 + h01 * z1 + h11 * m1
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

            # Replace the STL visual with the same segmented surface.  The
            # compact package moves the ramp relative to the legacy mesh; if
            # only collision is patched, Gazebo shows the lip in front of the
            # rollers even though physics correctly places it behind them.
            channel_visual = next(
                (
                    visual
                    for visual in intake_channel_link.findall("visual")
                    if "intake_channel_vis" in visual.attrib.get("name", "")
                ),
                None,
            )
            if channel_visual is not None:
                visual_material = channel_visual.find("material")
                material_template = (
                    copy.deepcopy(visual_material)
                    if visual_material is not None
                    else None
                )
                visual_name = channel_visual.attrib.get("name", "intake_channel_vis")
                ramp_visuals = [channel_visual]
                for i in range(1, ramp_steps):
                    ramp_visuals.append(
                        ET.SubElement(
                            intake_channel_link,
                            "visual",
                            {"name": f"{visual_name}_seg_{i:02d}"},
                        )
                    )

                for i, segment_visual in enumerate(ramp_visuals):
                    (x0, z0), (x1, z1) = top_points[i], top_points[i + 1]
                    dx = x0 - x1
                    dz = z1 - z0
                    length = max(0.001, (dx * dx + dz * dz) ** 0.5)
                    pitch = math.atan2(dz, dx)
                    normal_x = math.sin(pitch)
                    normal_z = math.cos(pitch)
                    center_x = (x0 + x1) / 2.0 - normal_x * sheet_thickness / 2.0
                    center_z = (z0 + z1) / 2.0 - normal_z * sheet_thickness / 2.0

                    segment_pose = segment_visual.find("pose")
                    if segment_pose is None:
                        segment_pose = ET.SubElement(segment_visual, "pose")
                    segment_pose.text = (
                        f"{channel_origin_x + center_x:.6f} 0 "
                        f"{max(sheet_thickness / 2.0, center_z):.6f} "
                        f"0 {pitch:.6f} 0"
                    )

                    segment_geometry = segment_visual.find("geometry")
                    if segment_geometry is None:
                        segment_geometry = ET.SubElement(segment_visual, "geometry")
                    segment_geometry.clear()
                    box = ET.SubElement(segment_geometry, "box")
                    ET.SubElement(box, "size").text = (
                        f"{length:.6f} {channel_width:.6f} {sheet_thickness:.6f}"
                    )
                    if i > 0 and material_template is not None:
                        segment_visual.append(copy.deepcopy(material_template))
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

    # The physical hopper is expected to use a grippy floor and compliant
    # mesh / rubber lining. Model that explicitly so a captured ball loses
    # energy instead of rebounding from rigid walls and rolling back out.
    basket_floor_contact = {
        "kp": os.getenv("BASKET_FLOOR_CONTACT_KP", "12000"),
        "kd": os.getenv("BASKET_FLOOR_CONTACT_KD", "80"),
        "max_vel": os.getenv("BASKET_CONTACT_MAX_VEL", "0.25"),
        "min_depth": os.getenv("BASKET_CONTACT_MIN_DEPTH", "0.001"),
    }
    basket_lining_contact = {
        "kp": os.getenv("BASKET_LINING_CONTACT_KP", "4000"),
        "kd": os.getenv("BASKET_LINING_CONTACT_KD", "120"),
        "max_vel": os.getenv("BASKET_CONTACT_MAX_VEL", "0.25"),
        "min_depth": os.getenv("BASKET_CONTACT_MIN_DEPTH", "0.001"),
    }
    for collision in root.findall(".//collision"):
        name = collision.attrib.get("name", "")
        if any(
            floor_name in name
            for floor_name in (
                "basket_floor_col",
                "basket_management_tray_col",
                "basket_receiving_chute_col",
                "basket_entry_hood_roof_col",
            )
        ):
            floor_mu = os.getenv("BASKET_FLOOR_MU", "1.0")
            _patch_collision_surface(
                collision, floor_mu, floor_mu, "0.0", "0.0", basket_floor_contact
            )
            _patch_collision_bounce(collision, "0.0", "0.0")
        elif any(
            wall_name in name
            for wall_name in (
                "basket_left_wall_col",
                "basket_right_wall_col",
                "basket_rear_wall_col",
                "basket_front_left_guard_col",
                "basket_front_right_guard_col",
                "basket_center_retention_lip_col",
                "basket_entry_hood_left_cheek_col",
                "basket_entry_hood_right_cheek_col",
            )
        ):
            lining_mu = os.getenv("BASKET_LINING_MU", "0.8")
            _patch_collision_surface(
                collision,
                lining_mu,
                lining_mu,
                "0.0",
                "0.0",
                basket_lining_contact,
            )
            _patch_collision_bounce(
                collision,
                os.getenv("BASKET_LINING_RESTITUTION", "0.05"),
                os.getenv("BASKET_LINING_BOUNCE_THRESHOLD", "0.05"),
            )

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
            "xacro executable not found. Install ros-jazzy-xacro or source a ROS "
            "environment that provides xacro.",
            file=sys.stderr,
        )
        return 2

    env = os.environ.copy()
    env.setdefault("PYTHONUTF8", "1")
    controllers_config = args.controllers_config.resolve()
    compact = args.packaging_variant == "compact"

    def variant_value(env_name: str, baseline: str, compact_value: str) -> str:
        return os.getenv(env_name, compact_value if compact else baseline)

    functional_shift_x = variant_value(
        "ROBOT_FUNCTIONAL_SHIFT_X_M", "0.0", "-0.100"
    )
    intake_cad_alignment_x = variant_value(
        "ROBOT_INTAKE_CAD_ALIGNMENT_X_M", "0.0", "-0.070"
    )
    battery_center_x = variant_value(
        "ROBOT_BATTERY_CENTER_X_M", "-0.143", "-0.255"
    )
    enable_compact_electronics = variant_value(
        "ROBOT_ENABLE_COMPACT_ELECTRONICS", "false", "true"
    )
    enable_flywheel = variant_value("ROBOT_ENABLE_FLYWHEEL", "false", "true")
    # Compact mass hypotheses: 2.0 kg removable basket/carriage contribution
    # plus 45 ITF-range balls at 57 g each. Set payload to 0 for simulations
    # that spawn the balls as individual Gazebo models, avoiding double count.
    basket_empty_mass = variant_value(
        "ROBOT_BASKET_EMPTY_MASS_KG", "0.01", "2.0"
    )
    basket_payload_mass = variant_value(
        "ROBOT_BASKET_PAYLOAD_MASS_KG", "0.0", "2.565"
    )
    result = subprocess.run(
        [
            xacro_exe,
            str(source),
            f"sim_mode:={args.sim_mode}",
            f"controllers_config:={controllers_config}",
            f"functional_shift_x:={functional_shift_x}",
            f"intake_cad_alignment_x:={intake_cad_alignment_x}",
            f"battery_center_x:={battery_center_x}",
            f"enable_compact_electronics:={enable_compact_electronics}",
            f"enable_flywheel:={enable_flywheel}",
            f"flywheel_max_vel:={os.getenv('FLYWHEEL_MAX_VEL_RAD_S', '320.0')}",
            f"basket_empty_mass:={basket_empty_mass}",
            f"basket_payload_mass:={basket_payload_mass}",
            f"expose_intake_carriage_state:={os.getenv('INTAKE_EXPOSE_CARRIAGE_STATE', 'false')}",
            # Dual-wheel intake tuning + Concept Validation Plan gates
            # (docs/mechanism/dual-wheel-intake-design-el.md).
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
            f"basket_floor_top_z:={os.getenv('INTAKE_BASKET_FLOOR_TOP_Z_M', '0.025')}",
            f"basket_rear_x:={os.getenv('BASKET_REAR_X_M', '0.02')}",
            f"basket_half_width:={os.getenv('BASKET_HALF_WIDTH_M', '0.14')}",
            f"basket_wall_top_z:={os.getenv('BASKET_WALL_TOP_Z_M', '0.25')}",
            f"basket_center_lip_height:={os.getenv('BASKET_CENTER_LIP_HEIGHT_M', '0.010')}",
            f"basket_management_run:={os.getenv('BASKET_MANAGEMENT_RUN_M', '0.14')}",
            f"basket_management_rise:={os.getenv('BASKET_MANAGEMENT_RISE_M', '0.010')}",
            f"basket_receiver_run:={os.getenv('BASKET_RECEIVER_RUN_M', '0.050')}",
            f"basket_receiver_rise:={os.getenv('BASKET_RECEIVER_RISE_M', '0.005')}",
            f"basket_hood_rear_overhang:={os.getenv('BASKET_HOOD_REAR_OVERHANG_M', '0.040')}",
            f"basket_hood_rear_clearance_z:={os.getenv('BASKET_HOOD_REAR_CLEARANCE_Z_M', '0.120')}",
            f"basket_hood_front_clearance_z:={os.getenv('BASKET_HOOD_FRONT_CLEARANCE_Z_M', '0.135')}",
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
        _replace_text(
            sdf_output,
            _patch_sdf_contacts(
                sdf_result.stdout, packaging_variant=args.packaging_variant
            ),
        )
        print(f"Generated {_display_path(sdf_output)} with patched contact surfaces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
