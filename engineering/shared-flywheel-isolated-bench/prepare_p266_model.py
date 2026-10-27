#!/usr/bin/env python3
"""Replace only the P2-51 ski/basket-front geometry with P2-66 fixtures.

Input is the SDF converted from the frozen P2-61/P2-51 full-v3 bench.  This
script deliberately leaves the launcher, chassis, arch, sensors and basket
floor/sides/rear untouched.
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def box(parent: ET.Element, kind: str, name: str, xyz: tuple[float, float, float],
        size: tuple[float, float, float], pitch: float = 0.0) -> ET.Element:
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
        ET.SubElement(material, "ambient").text = "0.95 0.45 0.06 1"
        ET.SubElement(material, "diffuse").text = "0.95 0.45 0.06 1"
    return node


def segment(parent: ET.Element, index: str, front: tuple[float, float],
            rear: tuple[float, float], width: float) -> None:
    dx, dz = front[0] - rear[0], rear[1] - front[1]
    length, pitch = math.hypot(dx, dz), math.atan2(dz, dx)
    centre = ((front[0] + rear[0]) / 2, 0.0,
              (front[1] + rear[1]) / 2 - 0.001)
    size = (length + 0.004, width, 0.002)
    box(parent, "collision", f"p266_ski_floor_{index}", centre, size, pitch)
    box(parent, "visual", f"p266_ski_floor_{index}", centre, size, pitch)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--radius-mm", type=int, choices=(75, 110), required=True)
    ap.add_argument("--angle-deg", type=int, choices=(70, 75, 80), required=True)
    ap.add_argument("--deflector-front-x-mm", type=float, default=274.0)
    ap.add_argument("--deflector-front-z-mm", type=float, default=334.0)
    ap.add_argument("--deflector-rear-x-mm", type=float, default=82.0)
    ap.add_argument("--deflector-rear-z-mm", type=float, default=370.0)
    ap.add_argument("--front-wall-top-mm", type=float, default=240.0)
    a = ap.parse_args()

    tree = ET.parse(a.input)
    model = tree.getroot().find("model")
    if model is None:
        raise SystemExit("model SDF missing model")
    base = next((x for x in model.findall("link") if x.get("name") == "base_link"), None)
    basket = next((x for x in model.findall("link") if x.get("name") == "basket_support_link"), None)
    if base is None:
        raise SystemExit("P2-66 requires base_link")
    # gz sdf collapses the fixed basket_support_joint into base_link.  Accept
    # either representation so the post-process is stable before or after
    # URDF-to-SDF conversion.
    basket = basket if basket is not None else base

    removed = 0
    for link in model.findall("link"):
        for node in list(link):
            if node.tag in ("collision", "visual") and "intake_fixed_ramp_" in node.get("name", ""):
                link.remove(node)
                removed += 1

    radius = a.radius_mm / 1000.0
    # Exact P2-51 form: 52 mm tangent followed by the selected circular arc.
    # For R75/70 the surface tip is X312.5/Z59.3; adding the 33 mm ball radius
    # gives the requested X~340 ball-centre take-off datum.
    points = [(0.435, 0.010), (0.383, 0.010)]
    for degrees in range(5, a.angle_deg + 1, 5):
        theta = math.radians(degrees)
        points.append((0.383 - radius * math.sin(theta),
                       0.010 + radius * (1 - math.cos(theta))))
    for i, (front, rear) in enumerate(zip(points, points[1:])):
        # Preserve the P2-51 widening: 61 mm at the tongue, 94 mm by 30 deg.
        width = min(0.094, 0.061 + i * 0.006)
        segment(base, f"{i:02d}", front, rear, width)

    # Closed front wall.  The CAD gate occupies the central 96 mm but is a
    # continuous collision plane in collection/transport (fail closed).
    wall_bottom = 0.042
    wall_top = a.front_wall_top_mm / 1000.0
    wall_centre = (0.278, 0.0, (wall_bottom + wall_top) / 2)
    wall_size = (0.004, 0.344, wall_top - wall_bottom)
    box(basket, "collision", "p266_front_wall_closed", wall_centre, wall_size)
    box(basket, "visual", "p266_front_wall_closed", wall_centre, wall_size)

    fx, fz = a.deflector_front_x_mm / 1000.0, a.deflector_front_z_mm / 1000.0
    rx, rz = a.deflector_rear_x_mm / 1000.0, a.deflector_rear_z_mm / 1000.0
    length, pitch = math.hypot(fx-rx, fz-rz), math.atan2(fz-rz, fx-rx)
    centre = ((fx+rx)/2, 0.0, (fz+rz)/2)
    size = (length, 0.340, 0.010)
    box(basket, "collision", "p266_soft_deflector", centre, size, pitch)
    box(basket, "visual", "p266_soft_deflector", centre, size, pitch)

    ET.indent(tree, space="  ")
    a.output.write_text("<?xml version='1.0'?>\n" + ET.tostring(tree.getroot(), encoding="unicode") + "\n")
    tip = points[-1]
    print(f"removed_nodes={removed} tip_x_m={tip[0]:.6f} tip_z_m={tip[1]:.6f} "
          f"deflector_pitch_deg={math.degrees(pitch):.3f} deflector_top_m={max(fz,rz)+.005:.3f}")


if __name__ == "__main__":
    main()
