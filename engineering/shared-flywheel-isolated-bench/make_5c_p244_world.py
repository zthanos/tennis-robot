#!/usr/bin/env python3
"""Build one frozen P2-44 open-ramp / cheek-variant world."""
from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def child(parent: ET.Element, name: str) -> ET.Element:
    found = parent.find(name)
    return found if found is not None else ET.SubElement(parent, name)


def add_mask(collision: ET.Element, category: int, collide: int) -> None:
    contact = child(child(collision, "surface"), "contact")
    child(contact, "category_bitmask").text = str(category)
    child(contact, "collide_bitmask").text = str(collide)


def add_friction(collision: ET.Element, mu: float) -> None:
    ode = child(child(child(collision, "surface"), "friction"), "ode")
    child(ode, "mu").text = str(mu)
    child(ode, "mu2").text = str(mu)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-sdf", type=Path, required=True)
    ap.add_argument("--ball-model", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--telemetry", required=True)
    ap.add_argument("--contacts", required=True)
    ap.add_argument("--ball-contacts", required=True)
    ap.add_argument("--approach", type=float, required=True)
    ap.add_argument("--lateral", type=float, required=True)
    ap.add_argument("--surface-speed", type=float, required=True)
    ap.add_argument("--tread-mu", type=float, required=True)
    ap.add_argument("--cheek-mu", type=float, required=True)
    ap.add_argument("--ramp-mu", type=float, default=0.35)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--max-step-size", type=float, default=0.001)
    a = ap.parse_args()

    source = ET.parse(a.model_sdf).getroot()
    robot = source.find("model")
    if robot is None:
        raise SystemExit("model SDF missing model")
    robot.set("name", "phase2_v3_5c_p244")
    pose = robot.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        robot.insert(0, pose)
    pose.text = "0 0 0.004 0 0 0"

    sensor_count = 0
    for link in robot.findall("link"):
        for collision in list(link.findall("collision")):
            name = collision.attrib["name"]
            if any(f"flywheel_{side}_col_collision" in name for side in ("left", "right")):
                link.remove(collision)
                continue
            ball_fixture = any(token in name for token in (
                "intake_cheek_", "intake_fixed_ramp_", "basket_floor_",
                "basket_left_wall_", "basket_right_wall_", "basket_rear_wall_", "basket_front_",
                "basket_p251_",
            ))
            if ball_fixture:
                add_mask(collision, 8, 4)
            if "intake_cheek_" in name:
                add_friction(collision, a.cheek_mu)
            if "intake_fixed_ramp_" in name:
                add_friction(collision, a.ramp_mu)
            safe = re.sub(r"[^A-Za-z0-9_]", "_", f"{link.attrib['name']}_{name}")
            sensor = ET.SubElement(link, "sensor", {"name": f"p2_44_contact_{safe}", "type": "contact"})
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "1000"
            ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = name
            sensor_count += 1

    plugin = ET.SubElement(robot, "plugin", {"filename": "v3_intake_5c_system", "name": "phase2_v3::Intake5cSystem"})
    for key, value in {
        "output_csv": a.telemetry,
        "contacts_csv": a.contacts,
        "approach_speed_m_s": a.approach,
        "surface_speed_m_s": a.surface_speed,
        "duration_s": a.duration,
    }.items():
        ET.SubElement(plugin, key).text = str(value)

    ball = ET.parse(a.ball_model).getroot().find("model")
    if ball is None:
        raise SystemExit("ball SDF missing model")
    ball.set("name", "capture_ball")
    ball.find("pose").text = f"0.75 {a.lateral} 0.033 0 0 0"
    contact_plugin = ball.find("plugin")
    ET.SubElement(contact_plugin, "telemetry_csv").text = a.ball_contacts
    ET.SubElement(contact_plugin, "friction_coefficient").text = str(a.tread_mu)
    ET.SubElement(contact_plugin, "wheel_radius").text = "0.100"
    ET.SubElement(contact_plugin, "wheel_half_width").text = "0.030"

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": "phase2_v3_5c_p244"})
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = str(a.max_step_size)
    ET.SubElement(physics, "real_time_factor").text = "0"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    gl = ET.SubElement(ground, "link", {"name": "link"})
    gc = ET.SubElement(gl, "collision", {"name": "ground_plane_collision"})
    plane = ET.SubElement(ET.SubElement(gc, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    add_friction(gc, 0.7)
    world.extend([robot, ball])
    ET.indent(sdf, space="  ")
    a.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")
    print(f"instrumented_robot_collisions={sensor_count}")


if __name__ == "__main__":
    main()
