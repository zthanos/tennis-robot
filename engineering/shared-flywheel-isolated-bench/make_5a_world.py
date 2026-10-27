#!/usr/bin/env python3
"""Wrap a gz-converted v3 model in the isolated 5a ground world."""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-sdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mu", type=float, required=True)
    args = parser.parse_args()

    source = ET.parse(args.model_sdf).getroot()
    model = source.find("model")
    if model is None:
        raise SystemExit("converted SDF does not contain a model")
    model.set("name", "phase2_v3_5a")
    pose = model.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        model.insert(0, pose)
    pose.text = "0 0 0.001 0 0 0"

    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": "phase2_v3_5a"})
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "0"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})

    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    ET.SubElement(ode, "mu").text = str(args.mu)
    ET.SubElement(ode, "mu2").text = str(args.mu)
    bullet = ET.SubElement(friction, "bullet")
    ET.SubElement(bullet, "friction").text = str(args.mu)
    ET.SubElement(bullet, "friction2").text = str(args.mu)
    world.append(model)

    ET.indent(sdf, space="  ")
    args.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n")


if __name__ == "__main__":
    main()
