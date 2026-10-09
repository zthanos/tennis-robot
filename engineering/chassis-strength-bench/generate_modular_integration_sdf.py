#!/usr/bin/env python3
"""Generate the rigid-body Gazebo bench for modular chassis v2.

The geometry mirrors cad/modular-test-chassis/modular-chassis-v2.scad. Gazebo
owns wheel/contact loads; printed-material stress remains in analyze.py/FEA.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def sub(parent: ET.Element, tag: str, text: object | None = None, **attrs: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def pose_text(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def add_inertial(link: ET.Element, mass: float, size: tuple[float, float, float]) -> None:
    inertial = sub(link, "inertial")
    sub(inertial, "mass", f"{mass:.6g}")
    inertia = sub(inertial, "inertia")
    ixx, iyy, izz = box_inertia(mass, size)
    sub(inertia, "ixx", f"{ixx:.9g}")
    sub(inertia, "iyy", f"{iyy:.9g}")
    sub(inertia, "izz", f"{izz:.9g}")
    sub(inertia, "ixy", "0")
    sub(inertia, "ixz", "0")
    sub(inertia, "iyz", "0")


def add_box(
    link: ET.Element,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0),
    color: str = "0.16 0.48 0.78 1",
) -> None:
    for kind in ("collision", "visual"):
        item = sub(link, kind, name=f"{name}_{kind}")
        sub(item, "pose", pose_text(pose))
        geometry = sub(item, "geometry")
        box = sub(geometry, "box")
        sub(box, "size", " ".join(str(value) for value in size))
        if kind == "visual":
            material = sub(item, "material")
            sub(material, "diffuse", color)


def add_box_link(
    model: ET.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
    size: tuple[float, float, float],
    mass: float,
    color: str = "0.16 0.48 0.78 1",
) -> ET.Element:
    link = sub(model, "link", name=name)
    sub(link, "pose", pose_text(pose))
    add_inertial(link, mass, size)
    add_box(link, name, size, color=color)
    return link


def add_joint(
    model: ET.Element,
    name: str,
    parent: str,
    child: str,
    joint_type: str = "fixed",
    *,
    sensed: bool = True,
) -> ET.Element:
    joint = sub(model, "joint", name=name, type=joint_type)
    sub(joint, "parent", parent)
    sub(joint, "child", child)
    if joint_type == "revolute":
        axis = sub(joint, "axis")
        sub(axis, "xyz", "0 1 0")
        limit = sub(axis, "limit")
        sub(limit, "effort", "3.73")
        sub(limit, "velocity", "13")
    if sensed:
        sensor = sub(joint, "sensor", name="connection_wrench", type="force_torque")
        sub(sensor, "always_on", "true")
        sub(sensor, "update_rate", "500")
        force_torque = sub(sensor, "force_torque")
        sub(force_torque, "frame", "child")
        sub(force_torque, "measure_direction", "child_to_parent")
    return joint


def add_wheel(
    model: ET.Element,
    name: str,
    parent: str,
    x: float,
    y: float,
) -> str:
    link_name = f"{name}_wheel"
    link = sub(model, "link", name=link_name)
    sub(link, "pose", pose_text((x, y, 0.085, math.pi / 2, 0, 0)))
    add_inertial(link, 0.70, (0.17, 0.17, 0.08))
    for kind in ("collision", "visual"):
        item = sub(link, kind, name=f"{name}_{kind}")
        geometry = sub(item, "geometry")
        cylinder = sub(geometry, "cylinder")
        sub(cylinder, "radius", "0.085")
        sub(cylinder, "length", "0.08")
        if kind == "collision":
            surface = sub(item, "surface")
            friction = sub(surface, "friction")
            ode = sub(friction, "ode")
            sub(ode, "mu", "0.9")
            sub(ode, "mu2", "0.9")
        else:
            material = sub(item, "material")
            sub(material, "diffuse", "0.05 0.06 0.07 1")
    joint_name = f"{name}_wheel_joint"
    add_joint(model, joint_name, parent, link_name, "revolute")
    return joint_name


def generate(payload_mass_kg: float) -> ET.ElementTree:
    sdf = ET.Element("sdf", version="1.10")
    world = sub(sdf, "world", name="modular_chassis_integration")
    physics = sub(world, "physics", name="deterministic_1ms", type="ignored")
    sub(physics, "max_step_size", "0.001")
    sub(physics, "real_time_factor", "1.0")
    sub(world, "gravity", "0 0 -9.80665")
    sub(world, "plugin", filename="gz-sim-physics-system", name="gz::sim::systems::Physics")
    sub(world, "plugin", filename="gz-sim-user-commands-system", name="gz::sim::systems::UserCommands")
    sub(world, "plugin", filename="gz-sim-scene-broadcaster-system", name="gz::sim::systems::SceneBroadcaster")
    sub(world, "plugin", filename="gz-sim-forcetorque-system", name="gz::sim::systems::ForceTorque")

    light = sub(world, "light", type="directional", name="sun")
    sub(light, "cast_shadows", "true")
    sub(light, "pose", "0 0 10 0 0 0")
    sub(light, "direction", "-0.5 0.1 -0.9")

    ground = sub(world, "model", name="ground_plane")
    sub(ground, "static", "true")
    ground_link = sub(ground, "link", name="link")
    for kind in ("collision", "visual"):
        item = sub(ground_link, kind, name=kind)
        geometry = sub(item, "geometry")
        plane = sub(geometry, "plane")
        sub(plane, "normal", "0 0 1")
        sub(plane, "size", "20 20")
        if kind == "collision":
            surface = sub(item, "surface")
            friction = sub(surface, "friction")
            ode = sub(friction, "ode")
            sub(ode, "mu", "0.9")
            sub(ode, "mu2", "0.9")

    model = sub(world, "model", name="modular_test_chassis_v2")
    sub(model, "self_collide", "false")

    # Canonical link: two printable crossbar halves represented as one rigid
    # 50 x 30 mm beam. The connection itself is still visible in CAD.
    add_box_link(model, "rear_crossbar", (-0.165, 0, 0.037, 0, 0, 0),
                 (0.05, 0.44, 0.03), 0.45)

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        y = side * 0.220
        add_box_link(model, f"rear_{side_name}_motor_module",
                     (-0.065, y, 0.037, 0, 0, 0), (0.20, 0.05, 0.03), 0.32)
        add_box_link(model, f"front_{side_name}_motor_module",
                     (0.135, y, 0.037, 0, 0, 0), (0.20, 0.05, 0.03), 0.32)

        gamma = sub(model, "link", name=f"{side_name}_gamma_body")
        sub(gamma, "pose", pose_text((0.3925, side*0.210, 0.037, 0, 0, 0)))
        add_inertial(gamma, 0.38, (0.22, 0.07, 0.03))
        add_box(gamma, "landing", (0.125, 0.07, 0.03), color="0.14 0.64 0.46 1")
        add_box(gamma, "socket", (0.065, 0.05, 0.03),
                (-0.125, side*0.010, 0, 0, 0, 0), "0.14 0.64 0.46 1")
        add_box(gamma, "flare", (0.030, 0.060, 0.03),
                (-0.0775, side*0.005, 0, 0, 0, 0), "0.14 0.64 0.46 1")

        # Half of the Gamma/intake/ramp cassette is assigned to each side so
        # both Gamma splices receive realistic static and turning load.
        intake = sub(model, "link", name=f"{side_name}_intake_half")
        sub(intake, "pose", pose_text((0.3925, side*0.105, 0.090, 0, 0, 0)))
        add_inertial(intake, 0.70, (0.24, 0.20, 0.15))
        add_box(intake, "gamma_upright", (0.125, 0.018, 0.120),
                (0, side*0.100, 0.030, 0, 0, 0), "0.78 0.26 0.08 1")
        add_box(intake, "ramp_half", (0.041, 0.094, 0.025),
                (-0.0525, -side*0.058, -0.053, 0, 0, 0), "0.42 0.72 0.32 1")
        add_box(intake, "ramp_cradle", (0.010, 0.101, 0.010),
                (-0.0715, side*0.0395, -0.043, 0, 0, 0), "0.58 0.20 0.76 1")

        add_joint(model, f"rear_{side_name}_corner_joint", "rear_crossbar",
                  f"rear_{side_name}_motor_module")
        add_joint(model, f"{side_name}_motor_splice_joint",
                  f"rear_{side_name}_motor_module", f"front_{side_name}_motor_module")
        add_joint(model, f"{side_name}_gamma_splice_joint",
                  f"front_{side_name}_motor_module", f"{side_name}_gamma_body")
        add_joint(model, f"{side_name}_intake_mount_joint",
                  f"{side_name}_gamma_body", f"{side_name}_intake_half")

    wheel_joints = {
        "rear_left": add_wheel(model, "rear_left", "rear_left_motor_module", -0.065, 0.350),
        "front_left": add_wheel(model, "front_left", "front_left_motor_module", 0.135, 0.350),
        "rear_right": add_wheel(model, "rear_right", "rear_right_motor_module", -0.065, -0.350),
        "front_right": add_wheel(model, "front_right", "front_right_motor_module", 0.135, -0.350),
    }

    tray = add_box_link(model, "electronics_tray", (-0.025, 0, 0.060, 0, 0, 0),
                        (0.19, 0.29, 0.004), 0.35, "0.12 0.55 0.35 1")
    del tray
    add_joint(model, "electronics_tray_joint", "rear_crossbar", "electronics_tray")

    if payload_mass_kg > 0:
        add_box_link(model, "payload", (-0.025, 0, 0.090, 0, 0, 0),
                     (0.18, 0.28, 0.05), payload_mass_kg, "0.35 0.36 0.38 1")
        add_joint(model, "payload_joint", "electronics_tray", "payload", sensed=False)

    drive = sub(model, "plugin", filename="gz-sim-diff-drive-system",
                name="gz::sim::systems::DiffDrive")
    sub(drive, "left_joint", wheel_joints["front_left"])
    sub(drive, "left_joint", wheel_joints["rear_left"])
    sub(drive, "right_joint", wheel_joints["front_right"])
    sub(drive, "right_joint", wheel_joints["rear_right"])
    sub(drive, "wheel_separation", "0.70")
    sub(drive, "wheel_radius", "0.085")
    sub(drive, "topic", "/modular_chassis_integration/cmd_vel")
    sub(drive, "max_linear_acceleration", "2.0")
    sub(drive, "max_angular_acceleration", "4.0")
    sub(drive, "odom_publish_frequency", "100")

    ET.indent(sdf, space="  ")
    return ET.ElementTree(sdf)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--payload-mass-kg", type=float, default=0.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "gazebo" / "modular_chassis_integration_bench.sdf",
    )
    args = parser.parse_args()
    if args.payload_mass_kg < 0:
        parser.error("--payload-mass-kg must be non-negative")
    tree = generate(args.payload_mass_kg)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="unicode", xml_declaration=True)
    print(f"Generated {args.output} with payload={args.payload_mass_kg:g} kg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
