#!/usr/bin/env python3
"""Build one frozen-core P2-59 reverse-launch world."""
from __future__ import annotations

import argparse
import copy
import math
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
    ap.add_argument("--angle-deg", type=float, choices=(15, 20), required=True)
    ap.add_argument("--surface-speed", type=float, choices=(12, 16, 20, 24), required=True)
    ap.add_argument("--tread-mu", type=float, choices=(0.6, 0.9, 1.2), required=True)
    ap.add_argument("--nip-compression-mm", type=float, choices=(8, 11, 14), default=8)
    ap.add_argument("--feed-speed", type=float, choices=(0, 0.6, 2, 4), default=0.6)
    ap.add_argument("--contact-entry-release", action="store_true")
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--max-step-size", type=float, default=0.001)
    args = ap.parse_args()

    robot = ET.parse(args.model_sdf).getroot().find("model")
    if robot is None:
        raise SystemExit("model SDF missing model")
    robot.set("name", "phase2_v3_5d1_p259")
    pose = robot.find("pose")
    if pose is None:
        pose = ET.Element("pose"); robot.insert(0, pose)
    pose.text = "0 0 0.004 0 0 0"
    # P2-60 diagnostic-only override.  The source xacro remains at the frozen
    # 58 mm gap / 8 mm diametral compression; only this generated world moves
    # the two flywheel joints symmetrically for the sensitivity matrix.
    diagnostic_gap = (66.0 - args.nip_compression_mm) / 1000.0
    diagnostic_wheel_y = 0.100 + diagnostic_gap / 2.0
    for side, sign in (("left", 1.0), ("right", -1.0)):
        joint = robot.find(f"joint[@name='flywheel_{side}_joint']")
        if joint is None or joint.find("pose") is None:
            raise SystemExit(f"missing flywheel_{side}_joint pose")
        values = joint.find("pose").text.split()
        values[1] = f"{sign * diagnostic_wheel_y:.12g}"
        joint.find("pose").text = " ".join(values)
    for link in robot.findall("link"):
        for collision in list(link.findall("collision")):
            name = collision.attrib["name"]
            if any(f"flywheel_{side}_col_collision" in name for side in ("left", "right")):
                link.remove(collision)
                continue
            # The P2-51 storage-floor slab is above the separately reserved
            # low feed channel.  It must not block the launch-feed ball merely
            # because the collection bench represents both on base_link.
            storage_floor = any(token in name for token in (
                "basket_p251_center_floor", "basket_p251_left_floor_wing",
                "basket_p251_right_floor_wing"))
            ball_fixture = not storage_floor and any(token in name for token in (
                "intake_cheek_", "intake_fixed_ramp_", "basket_floor_",
                "basket_left_wall_", "basket_right_wall_", "basket_rear_wall_",
                "basket_front_", "basket_p251_"))
            if ball_fixture:
                add_mask(collision, 8, 4)
            if "intake_cheek_" in name:
                add_friction(collision, 0.15)
            if "intake_fixed_ramp_" in name:
                add_friction(collision, 0.15)
            safe = re.sub(r"[^A-Za-z0-9_]", "_", f"{link.attrib['name']}_{name}")
            sensor = ET.SubElement(link, "sensor", {"name": f"p259_contact_{safe}", "type": "contact"})
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "1000"
            ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = name

    template = ET.parse(args.ball_model).getroot().find("model")
    if template is None:
        raise SystemExit("ball SDF missing model")
    ball = copy.deepcopy(template)
    ball.set("name", "launch_ball")
    ball.find("pose").text = "0.278 0 0.075 0 0 0"
    link = ball.find("link")
    link.set("name", "launch_ball_link")
    collision = link.find("collision")
    collision.set("name", "launch_ball_collision")
    add_mask(collision, 4, 8)
    plugin = ball.find("plugin")
    plugin.find("ball_link").text = "launch_ball_link"
    ET.SubElement(plugin, "telemetry_csv").text = args.ball_contacts
    ET.SubElement(plugin, "friction_coefficient").text = str(args.tread_mu)
    ET.SubElement(plugin, "wheel_radius").text = "0.100"
    ET.SubElement(plugin, "wheel_half_width").text = "0.030"
    sensor = ET.SubElement(link, "sensor", {"name": "p259_launch_ball_contact", "type": "contact"})
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "1000"
    ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = "launch_ball_collision"

    theta = math.radians(args.angle_deg)
    # The P2-51 collection xacro does not contain the separate low launch-feed
    # channel reserved in CAD.  Start at its downstream handoff, 50 mm behind
    # the nip in launcher coordinates, after the proposed singulator stroke.
    nip_x = 0.387 + 0.055*math.cos(theta)
    nip_z = 0.004 + 0.049837363636 + 0.055*math.sin(theta)
    release_position = None
    if args.contact_entry_release:
        centre_offset = diagnostic_wheel_y
        contact_half_length = math.sqrt((0.100 + 0.033)**2 - centre_offset**2)
        # Start 0.5 mm inside the nominal first-contact plane so the v=0 case
        # has a finite normal force instead of an exact zero-force equilibrium.
        release_upstream = contact_half_length - 0.0005
        release_position = (
            nip_x - release_upstream*math.cos(theta), 0,
            nip_z - release_upstream*math.sin(theta))
    hold_x = nip_x - 0.050*math.cos(theta)
    hold_z = nip_z - 0.050*math.sin(theta)
    launch = ET.SubElement(ball, "plugin", {
        "filename": "v3_launch_5d1_system",
        "name": "phase2_v3::Launch5d1System",
    })
    values = {
        "ball_link": "launch_ball_link",
        "surface_speed_m_s": args.surface_speed,
        "tilt_target_rad": theta,
        "minimum_release_time_s": 3.5,
        "duration_s": args.duration,
        "hold_position_world": f"{hold_x:.12g} 0 {hold_z:.12g}",
        "feed_velocity_world": f"{args.feed_speed*math.cos(theta):.12g} 0 {args.feed_speed*math.sin(theta):.12g}",
        "output_csv": args.telemetry,
        "contacts_csv": args.contacts,
    }
    for name, value in values.items():
        ET.SubElement(launch, name).text = str(value)
    if release_position is not None:
        ET.SubElement(launch, "release_position_world").text = " ".join(
            f"{value:.12g}" for value in release_position)

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": "phase2_v3_5d1_p259"})
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
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    ground_collision = ET.SubElement(ground_link, "collision", {"name": "ground_plane_collision"})
    plane = ET.SubElement(ET.SubElement(ground_collision, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    add_friction(ground_collision, 0.7)
    world.extend((ball, robot))
    ET.indent(sdf, space="  ")
    args.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")


if __name__ == "__main__":
    main()
