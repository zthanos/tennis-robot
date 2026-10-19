#!/usr/bin/env python3
"""Build a P2-46 basket-fill, delivery or motion-retention world."""
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
    child(ode, "mu").text = str(mu); child(ode, "mu2").text = str(mu)


def ball_model(template: ET.Element, index: int | None, pose: tuple[float, float, float],
               telemetry: str | None) -> ET.Element:
    model = copy.deepcopy(template)
    incoming = index is None
    token = "capture_ball" if incoming else f"basket_ball_{index:03d}"
    model.set("name", token)
    model.find("pose").text = f"{pose[0]} {pose[1]} {pose[2]} 0 0 0"
    link = model.find("link")
    link.set("name", f"{token}_link")
    collision = link.find("collision")
    collision.set("name", f"{token}_collision")
    add_mask(collision, 4, 12)  # basket fixtures (8) plus other balls (4)
    add_friction(collision, 0.35)
    plugin = model.find("plugin")
    if incoming:
        plugin.find("ball_link").text = f"{token}_link"
        ET.SubElement(plugin, "telemetry_csv").text = telemetry
        ET.SubElement(plugin, "friction_coefficient").text = "0.6"
        ET.SubElement(plugin, "wheel_radius").text = "0.100"
        ET.SubElement(plugin, "wheel_half_width").text = "0.030"
    else:
        model.remove(plugin)
    return model


def drop_pose(index: int, seed: int, variant: str = "p246_open") -> tuple[float, float, float]:
    # Deterministic low-discrepancy release stream above the open basket.
    # These are release locations, not final packed/grid coordinates.
    # P2-47 uses the common unobstructed top behind every candidate hood;
    # this avoids dropping preload balls onto one candidate's roof.
    x_span = 0.165 if variant != "p246_open" else 0.215
    x = 0.025 + x_span * ((index * 0.61803398875 + seed * 0.137) % 1.0)
    if variant == "tall_open_drop":
        # The centre is occupied by the overhead delivery bridge.  Alternate
        # dynamic releases over the two unobstructed side lanes; this is still
        # a fall-and-settle preload, never a final packed grid.
        side = -1.0 if index % 2 == 0 else 1.0
        y = side * (0.072 + 0.045 * ((index * 0.41421356237 + seed * 0.271) % 1.0))
    else:
        y = -0.115 + 0.230 * ((index * 0.41421356237 + seed * 0.271) % 1.0)
    z = 0.30 + 0.070 * index
    return x, y, z


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("delivery", "motion"), required=True)
    ap.add_argument("--model-sdf", type=Path, required=True)
    ap.add_argument("--ball-model", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--observer", required=True)
    ap.add_argument("--telemetry")
    ap.add_argument("--contacts")
    ap.add_argument("--ball-contacts")
    ap.add_argument("--preload", type=int, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--duration", type=float, required=True)
    ap.add_argument("--variant", default="p246_open")
    ap.add_argument("--surface-speed", type=float, default=2.5)
    args = ap.parse_args()
    if args.mode == "delivery" and not all((args.telemetry, args.contacts, args.ball_contacts)):
        raise SystemExit("delivery mode requires telemetry/contact paths")

    robot = ET.parse(args.model_sdf).getroot().find("model")
    if robot is None: raise SystemExit("model SDF missing model")
    robot.set("name", f"phase2_v3_5c2_{args.variant}")
    pose = robot.find("pose")
    if pose is None: pose = ET.Element("pose"); robot.insert(0, pose)
    pose.text = "0 0 0.004 0 0 0"

    sensor_count = 0
    for link in robot.findall("link"):
        for collision in list(link.findall("collision")):
            name = collision.attrib["name"]
            if args.mode == "delivery" and any(f"flywheel_{side}_col_collision" in name for side in ("left", "right")):
                link.remove(collision); continue
            ball_fixture = name.startswith("basket_") or any(token in name for token in (
                "intake_cheek_", "intake_fixed_ramp_", "basket_floor_", "basket_left_wall_",
                "basket_right_wall_", "basket_rear_wall_", "basket_front_"))
            if ball_fixture: add_mask(collision, 8, 4)
            if args.mode == "delivery":
                safe = re.sub(r"[^A-Za-z0-9_]", "_", f"{link.attrib['name']}_{name}")
                sensor = ET.SubElement(link, "sensor", {"name": f"p2_46_contact_{safe}", "type": "contact"})
                ET.SubElement(sensor, "always_on").text = "true"; ET.SubElement(sensor, "update_rate").text = "100"
                ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = name
                sensor_count += 1

    if args.mode == "delivery":
        plugin = ET.SubElement(robot, "plugin", {"filename": "v3_intake_5c_system", "name": "phase2_v3::Intake5cSystem"})
        values = {"output_csv": args.telemetry, "contacts_csv": args.contacts,
                  "approach_speed_m_s": 0.6, "surface_speed_m_s": args.surface_speed,
                  "drive_start_s": 6.0, "tracked_ball_link": "capture_ball_link",
                  "duration_s": args.duration}
        for key, value in values.items(): ET.SubElement(plugin, key).text = str(value)
    observer = ET.SubElement(robot, "plugin", {"filename": "v3_basket_5c2_observer", "name": "phase2_v3::Basket5c2Observer"})
    observer_period = 0.02 if args.mode == "delivery" else 0.05
    for key, value in {"output_csv": args.observer, "duration_s": args.duration,
                       "sample_period_s": observer_period, "expected_ball_count": args.preload + (args.mode == "delivery")}.items():
        ET.SubElement(observer, key).text = str(value)

    template = ET.parse(args.ball_model).getroot().find("model")
    if template is None: raise SystemExit("ball SDF missing model")
    balls = [ball_model(template, i+1, drop_pose(i, args.seed, args.variant), None) for i in range(args.preload)]
    if args.mode == "delivery": balls.append(ball_model(template, None, (0.75, 0.0, 0.033), args.ball_contacts))

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": f"phase2_v3_5c2_{args.variant}"})
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = "0.001"; ET.SubElement(physics, "real_time_factor").text = "0"
    for filename, name in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                           ("gz-sim-contact-system", "gz::sim::systems::Contact"),
                           ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
                           ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    ground = ET.SubElement(world, "model", {"name": "ground_plane"}); ET.SubElement(ground, "static").text = "true"
    gl = ET.SubElement(ground, "link", {"name": "link"}); gc = ET.SubElement(gl, "collision", {"name": "ground_plane_collision"})
    plane = ET.SubElement(ET.SubElement(gc, "geometry"), "plane"); ET.SubElement(plane, "normal").text = "0 0 1"; ET.SubElement(plane, "size").text = "100 100"
    add_friction(gc, 0.7)
    world.extend(balls); world.append(robot)
    ET.indent(sdf, space="  ")
    args.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")
    print(f"mode={args.mode} variant={args.variant} preload={args.preload} balls={len(balls)} instrumented_robot_collisions={sensor_count}")


if __name__ == "__main__": main()
