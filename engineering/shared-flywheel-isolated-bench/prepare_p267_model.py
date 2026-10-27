#!/usr/bin/env python3
"""Build the P2-67/69 inclined arch, overhead deflector and closed wall.

Only the P2-67 geometry is replaced.  The P2-40 launcher, corrected P2-61
ball contact model, v3 chassis and P2-51 basket floor/sides/rear stay frozen.
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def box(parent: ET.Element, kind: str, name: str,
        xyz: tuple[float, float, float], size: tuple[float, float, float],
        pitch: float = 0.0, colour: str = "0.95 0.45 0.06 1") -> ET.Element:
    node = ET.SubElement(parent, kind, {"name": name})
    ET.SubElement(node, "pose").text = (
        f"{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} 0 {pitch:.9f} 0"
    )
    geometry = ET.SubElement(node, "geometry")
    ET.SubElement(ET.SubElement(geometry, "box"), "size").text = (
        f"{size[0]:.9f} {size[1]:.9f} {size[2]:.9f}"
    )
    if kind == "visual":
        material = ET.SubElement(node, "material")
        ET.SubElement(material, "ambient").text = colour
        ET.SubElement(material, "diffuse").text = colour
    return node


def cylinder(parent: ET.Element, kind: str, name: str,
             xyz: tuple[float, float, float], radius: float, length: float,
             colour: str = "0.08 0.09 0.11 1") -> ET.Element:
    node = ET.SubElement(parent, kind, {"name": name})
    ET.SubElement(node, "pose").text = f"{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} 0 0 0"
    geometry = ET.SubElement(node, "geometry")
    shape = ET.SubElement(geometry, "cylinder")
    ET.SubElement(shape, "radius").text = str(radius)
    ET.SubElement(shape, "length").text = str(length)
    if kind == "visual":
        material = ET.SubElement(node, "material")
        ET.SubElement(material, "ambient").text = colour
        ET.SubElement(material, "diffuse").text = colour
    return node


def line_x(parent: ET.Element, name: str, rear: tuple[float, float],
           front: tuple[float, float], width: float, thickness: float,
           colour: str) -> None:
    """Box whose local +X axis runs from rear to front in global XZ."""
    dx, dz = front[0] - rear[0], front[1] - rear[1]
    length = math.hypot(dx, dz)
    # In SDF Ry(p), local +X maps to global (cos(p), 0, -sin(p)).
    pitch = math.atan2(-dz, dx)
    centre = ((rear[0] + front[0]) / 2, 0.0, (rear[1] + front[1]) / 2)
    for kind in ("collision", "visual"):
        box(parent, kind, name, centre, (length, width, thickness), pitch, colour)


def ski_segment(parent: ET.Element, phase: str, index: int,
                front: tuple[float, float], rear: tuple[float, float],
                width: float) -> None:
    line_x(parent, f"{phase}_ski_floor_{index:02d}", rear, front, width, 0.002,
           "0.08 0.62 0.82 1")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--phase", choices=("p267", "p269"), default="p267")
    ap.add_argument("--angle-deg", type=int, choices=(75, 80), required=True)
    ap.add_argument("--arch-rear-edge-mm", type=float, default=365.0)
    ap.add_argument("--wall-tilt-deg", type=float, default=27.5)
    ap.add_argument("--wall-top-mm", type=float, default=90.0)
    ap.add_argument("--wall-x-mm", type=float, default=225.0)
    ap.add_argument("--wall-bottom-mm", type=float, default=46.1)
    ap.add_argument("--deflector-rear-x-mm", type=float, default=250.0)
    ap.add_argument("--deflector-rear-z-mm", type=float, default=363.0)
    ap.add_argument("--deflector-front-x-mm", type=float, default=360.0)
    ap.add_argument("--deflector-front-z-mm", type=float, default=343.0)
    a = ap.parse_args()

    tree = ET.parse(a.input)
    model = tree.getroot().find("model")
    if model is None:
        raise SystemExit("model SDF missing model")
    base = next((x for x in model.findall("link") if x.get("name") == "base_link"), None)
    basket = next((x for x in model.findall("link") if x.get("name") == "basket_support_link"), None)
    if base is None:
        raise SystemExit("P2-67 requires base_link")
    basket = basket if basket is not None else base

    remove_tokens = ("intake_fixed_ramp_", "fixture_arch_", "fixture_lidar_",
                     "fixture_camera_", "p266_", "p267_", "p269_")
    removed = []
    for link in model.findall("link"):
        for node in list(link):
            if node.tag in ("collision", "visual") and any(
                    token in node.get("name", "") for token in remove_tokens):
                removed.append(node.get("name", ""))
                link.remove(node)

    radius = 0.075
    points = [(0.435, 0.010), (0.383, 0.010)]
    for degrees in range(5, a.angle_deg + 1, 5):
        theta = math.radians(degrees)
        points.append((0.383 - radius * math.sin(theta),
                       0.010 + radius * (1 - math.cos(theta))))
    for i, (front, rear) in enumerate(zip(points, points[1:])):
        ski_segment(base, a.phase, i, front, rear, min(0.094, 0.061 + i * 0.006))

    # Minimal inclination satisfying rear edge >= selected datum.  The
    # crossmember remains 50 mm deep, so its centre is rear_edge + 25 mm.
    base_x, base_z, top_z = 0.318, 0.052, 0.245
    cross_x = (a.arch_rear_edge_mm + 25.0) / 1000.0
    leg_dx, leg_dz = cross_x - base_x, top_z - base_z
    leg_len = math.hypot(leg_dx, leg_dz)
    leg_pitch = math.atan2(leg_dx, leg_dz)
    for side in (-1, 1):
        side_name = "right" if side < 0 else "left"
        centre = ((base_x + cross_x) / 2, side * 0.220, (base_z + top_z) / 2)
        for kind in ("collision", "visual"):
            box(base, kind, f"{a.phase}_arch_leg_{side_name}", centre,
                (0.030, 0.050, leg_len), leg_pitch, "0.15 0.48 0.78 1")
    for kind in ("collision", "visual"):
        box(base, kind, f"{a.phase}_arch_crossmember", (cross_x, 0.0, top_z),
            (0.050, 0.440, 0.030), colour="0.15 0.48 0.78 1")

    # Sensor pack translates +60 mm with the arch; orientations and Z datums
    # stay unchanged.  Thus the LiDAR scan plane remains horizontal at Z390.
    lidar_x, camera_x = cross_x, cross_x + 0.020
    for kind in ("collision", "visual"):
        cylinder(base, kind, f"{a.phase}_lidar_body", (lidar_x, 0.0, 0.374), 0.038, 0.032)
        box(base, kind, f"{a.phase}_camera_body", (camera_x, 0.0, 0.305),
            (0.082, 0.032, 0.028), math.radians(50), "0.08 0.09 0.11 1")

    wall_top_x = math.nan
    if a.phase == "p267":
        # Historical P2-67 tilted wall, later rejected for pile retention.
        wall_bottom = (0.278, 0.042)
        wall_top_z = a.wall_top_mm / 1000.0
        wall_top_x = wall_bottom[0] - (wall_top_z-wall_bottom[1]) * math.tan(
            math.radians(a.wall_tilt_deg))
        wall_len = math.hypot(wall_top_x-wall_bottom[0], wall_top_z-wall_bottom[1])
        wall_pitch = -math.radians(a.wall_tilt_deg)
        wall_centre = ((wall_bottom[0]+wall_top_x)/2, 0.0,
                       (wall_bottom[1]+wall_top_z)/2)
        for kind in ("collision", "visual"):
            box(basket, kind, "p267_front_wall_closed", wall_centre,
                (0.004, 0.344, wall_len), wall_pitch, "0.32 0.58 0.36 1")
    else:
        # Accepted P2-68 geometry, exercised as the closed P2-69 confirmation.
        wall_x = a.wall_x_mm / 1000.0
        wall_bottom_z = a.wall_bottom_mm / 1000.0
        wall_top_z = a.wall_top_mm / 1000.0
        door_h = 0.080
        door_hinge_z = wall_bottom_z + door_h
        upper_h = wall_top_z - door_hinge_z
        side_w = 0.132
        side_y = 0.106
        for kind in ("collision", "visual"):
            box(basket, kind, "p269_front_wall_upper",
                (wall_x, 0.0, door_hinge_z + upper_h/2),
                (0.004, 0.344, upper_h), colour="0.32 0.58 0.36 1")
            for side in (-1, 1):
                label = "right" if side < 0 else "left"
                box(basket, kind, f"p269_front_wall_side_{label}",
                    (wall_x, side*side_y, wall_bottom_z + door_h/2),
                    (0.004, side_w, door_h), colour="0.32 0.58 0.36 1")
            # Closed 84x84 leaf: 2 mm overlap around the clear 80x80 aperture.
            box(basket, kind, "p269_feed_door_closed",
                (wall_x, 0.0, door_hinge_z-door_h/2),
                (0.005, 0.084, 0.084), colour="0.95 0.52 0.08 1")

    rear = (a.deflector_rear_x_mm/1000.0, a.deflector_rear_z_mm/1000.0)
    front = (a.deflector_front_x_mm/1000.0, a.deflector_front_z_mm/1000.0)
    line_x(basket, f"{a.phase}_soft_deflector", rear, front, 0.340, 0.010,
           "0.82 0.25 0.62 1")

    ET.indent(tree, space="  ")
    a.output.write_text("<?xml version='1.0'?>\n" +
                        ET.tostring(tree.getroot(), encoding="unicode") + "\n")
    print(f"phase={a.phase} removed_nodes={len(removed)} arch_cross_x_m={cross_x:.6f} "
          f"arch_rear_edge_m={cross_x-.025:.6f} arch_tilt_deg={math.degrees(leg_pitch):.3f} "
          f"lidar_x_m={lidar_x:.6f} camera_x_m={camera_x:.6f} "
          f"wall_top_x_m={wall_top_x:.6f}")


if __name__ == "__main__":
    main()
