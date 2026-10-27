#!/usr/bin/env python3
"""Wrap the converted P2-42 model in a headless 5b world with contact sensors."""
from __future__ import annotations
import argparse, re
import xml.etree.ElementTree as ET
from pathlib import Path

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-sdf", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    source = ET.parse(args.model_sdf).getroot()
    model = source.find("model")
    if model is None: raise SystemExit("converted SDF does not contain a model")
    model.set("name", "phase2_v3_5b")
    pose = model.find("pose")
    if pose is None: pose = ET.Element("pose"); model.insert(0, pose)
    pose.text = "0 0 0.001 0 0 0"
    sensor_count = 0
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            collision_name = collision.attrib["name"]
            safe = re.sub(r"[^A-Za-z0-9_]", "_", f"{link.attrib['name']}_{collision_name}")
            sensor = ET.SubElement(link, "sensor", {"name": f"p2_42_contact_{safe}", "type": "contact"})
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "1000"
            contact = ET.SubElement(sensor, "contact")
            ET.SubElement(contact, "collision").text = collision_name
            sensor_count += 1
    if sensor_count == 0: raise SystemExit("no collisions available for contact instrumentation")
    sdf = ET.Element("sdf", {"version":"1.10"})
    world = ET.SubElement(sdf, "world", {"name":"phase2_v3_5b"})
    physics = ET.SubElement(world, "physics", {"name":"1ms", "type":"ignored"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "0"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")):
        ET.SubElement(world, "plugin", {"filename":filename, "name":name})
    ground = ET.SubElement(world, "model", {"name":"ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name":"link"})
    collision = ET.SubElement(link, "collision", {"name":"ground_plane_collision"})
    geometry = ET.SubElement(collision, "geometry"); plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"; ET.SubElement(plane, "size").text = "100 100"
    ground_sensor = ET.SubElement(link, "sensor", {"name":"p2_42_contact_ground", "type":"contact"})
    ET.SubElement(ground_sensor, "always_on").text = "true"; ET.SubElement(ground_sensor, "update_rate").text = "1000"
    ground_contact = ET.SubElement(ground_sensor, "contact"); ET.SubElement(ground_contact, "collision").text = "ground_plane_collision"
    world.append(model)
    ET.indent(sdf, space="  ")
    args.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")
    print(f"instrumented_contact_sensors={sensor_count + 1}")
if __name__ == "__main__": main()
