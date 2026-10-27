#!/usr/bin/env python3
"""Build a P2-51-based 5c-3 world with one or two incoming balls."""
from __future__ import annotations

import argparse
import copy
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


def make_ball(template: ET.Element, index: int, longitudinal: float,
              lateral: float, telemetry: str,
              tread_mu: float) -> ET.Element:
    ball = copy.deepcopy(template)
    token = f"capture_ball_{index:02d}"
    ball.set("name", token)
    ball.find("pose").text = f"{0.75 + longitudinal} {lateral} 0.033 0 0 0"
    link = ball.find("link")
    link.set("name", f"{token}_link")
    collision = link.find("collision")
    collision.set("name", f"{token}_collision")
    add_mask(collision, 4, 12)
    add_friction(collision, 0.35)
    plugin = ball.find("plugin")
    plugin.find("ball_link").text = f"{token}_link"
    ET.SubElement(plugin, "telemetry_csv").text = telemetry
    ET.SubElement(plugin, "friction_coefficient").text = str(tread_mu)
    ET.SubElement(plugin, "wheel_radius").text = "0.100"
    ET.SubElement(plugin, "wheel_half_width").text = "0.030"
    sensor = ET.SubElement(link, "sensor", {"name": f"p2_56_contact_{token}", "type": "contact"})
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "1000"
    ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = f"{token}_collision"
    return ball


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-sdf", type=Path, required=True)
    ap.add_argument("--ball-model", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--telemetry", required=True)
    ap.add_argument("--contacts", required=True)
    ap.add_argument("--observer", required=True)
    ap.add_argument("--ball-contact", action="append", required=True)
    ap.add_argument("--lateral", action="append", type=float, required=True)
    ap.add_argument("--longitudinal", action="append", type=float, required=True)
    ap.add_argument("--surface-speed", type=float, default=6.0)
    ap.add_argument("--tread-mu", type=float, default=0.6)
    ap.add_argument("--ramp-mu", type=float, default=0.15)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--max-step-size", type=float, default=0.001)
    ap.add_argument("--drive-profile", choices=(
        "continuous", "stop_1s", "reverse_75mm", "p256_reverse_both",
        "p256_asym_left", "p256_reverse150_yaw12"), default="continuous")
    args = ap.parse_args()
    if not (len(args.lateral) == len(args.longitudinal) == len(args.ball_contact)):
        raise SystemExit("one --ball-contact, --lateral and --longitudinal are required per ball")

    robot = ET.parse(args.model_sdf).getroot().find("model")
    if robot is None:
        raise SystemExit("model SDF missing model")
    robot.set("name", "phase2_v3_5c3_p255")
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
                "intake_cheek_", "p257_cheek_", "p257_stagger_", "p257_shield_",
                "intake_fixed_ramp_", "basket_floor_",
                "basket_left_wall_", "basket_right_wall_", "basket_rear_wall_",
                "basket_front_", "basket_p251_",
            ))
            if ball_fixture:
                add_mask(collision, 8, 4)
            if "intake_cheek_" in name:
                add_friction(collision, 0.15)
            if any(token in name for token in ("p257_cheek_", "p257_stagger_", "p257_shield_")):
                add_friction(collision, 0.05)
            if "intake_fixed_ramp_" in name:
                add_friction(collision, args.ramp_mu)
            safe = re.sub(r"[^A-Za-z0-9_]", "_", f"{link.attrib['name']}_{name}")
            sensor = ET.SubElement(link, "sensor", {"name": f"p2_55_contact_{safe}", "type": "contact"})
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "1000"
            ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = name
            sensor_count += 1

    intake = ET.SubElement(robot, "plugin", {"filename": "v3_intake_5c_system", "name": "phase2_v3::Intake5cSystem"})
    for key, value in {
        "output_csv": args.telemetry,
        "contacts_csv": args.contacts,
        "approach_speed_m_s": 0.6,
        "surface_speed_m_s": args.surface_speed,
        "tracked_ball_link": "capture_ball_00_link",
        "duration_s": args.duration,
        "drive_profile": args.drive_profile,
        "recovery_start_s": 2.75,
        "stop_duration_s": 1.0,
        "reverse_speed_m_s": -0.3,
        "reverse_duration_s": 0.35,
    }.items():
        ET.SubElement(intake, key).text = str(value)
    observer = ET.SubElement(robot, "plugin", {"filename": "v3_basket_5c2_observer", "name": "phase2_v3::Basket5c2Observer"})
    for key, value in {
        "output_csv": args.observer,
        "duration_s": args.duration,
        "sample_period_s": 0.005,
        "expected_ball_count": len(args.lateral),
    }.items():
        ET.SubElement(observer, key).text = str(value)

    template = ET.parse(args.ball_model).getroot().find("model")
    if template is None:
        raise SystemExit("ball SDF missing model")
    balls = [make_ball(template, i, args.longitudinal[i], lateral,
                       args.ball_contact[i], args.tread_mu)
             for i, lateral in enumerate(args.lateral)]

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": "phase2_v3_5c3_p255"})
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = str(args.max_step_size)
    ET.SubElement(physics, "real_time_factor").text = "0"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    gui = ET.SubElement(world, "gui", {"fullscreen": "0"})
    scene = ET.SubElement(gui, "plugin", {"filename": "MinimalScene", "name": "3D View"})
    ET.SubElement(scene, "engine").text = "ogre2"
    ET.SubElement(scene, "scene").text = "scene"
    ET.SubElement(scene, "ambient_light").text = "0.7 0.7 0.7"
    ET.SubElement(scene, "background_color").text = "0.12 0.12 0.12"
    ET.SubElement(scene, "camera_pose").text = "0.55 -1.45 0.38 0 0.10 1.5708"
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    gl = ET.SubElement(ground, "link", {"name": "link"})
    gc = ET.SubElement(gl, "collision", {"name": "ground_plane_collision"})
    plane = ET.SubElement(ET.SubElement(gc, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    add_friction(gc, 0.7)
    # Match the proven 5c-2 insertion order: ball entities must exist before
    # Basket5c2Observer::Configure resolves its globally named links.
    world.extend(balls)
    world.append(robot)
    ET.indent(sdf, space="  ")
    args.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")
    print(f"balls={len(balls)} instrumented_robot_collisions={sensor_count}")


if __name__ == "__main__":
    main()
