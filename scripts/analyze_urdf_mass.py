#!/usr/bin/env python3
"""Report total mass and zero-joint centre of mass for a URDF model."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def _vec(text: str | None) -> tuple[float, float, float]:
    values = [float(value) for value in (text or "0 0 0").split()]
    if len(values) != 3:
        raise ValueError(f"expected xyz/rpy triplet, got {text!r}")
    return values[0], values[1], values[2]


def _rotation(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_vec(matrix, vector):
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def _mat_mul(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _add(left, right):
    return tuple(left[index] + right[index] for index in range(3))


def analyze(path: Path) -> dict[str, object]:
    root = ET.parse(path).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    child_joints: dict[str, list[ET.Element]] = {}
    child_links: set[str] = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        child_joints.setdefault(parent, []).append(joint)
        child_links.add(child)
    roots = sorted(set(links) - child_links)
    if len(roots) != 1:
        raise ValueError(f"expected one URDF root link, found {roots}")

    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    transforms: dict[str, tuple[tuple[float, float, float], tuple[tuple[float, ...], ...]]] = {}

    def visit(link_name, position, rotation):
        if link_name in transforms:
            raise ValueError(f"cycle or duplicate parent for {link_name}")
        transforms[link_name] = position, rotation
        for joint in child_joints.get(link_name, []):
            origin = joint.find("origin")
            xyz = _vec(origin.attrib.get("xyz") if origin is not None else None)
            rpy = _vec(origin.attrib.get("rpy") if origin is not None else None)
            child_position = _add(position, _mat_vec(rotation, xyz))
            child_rotation = _mat_mul(rotation, _rotation(rpy))
            visit(joint.find("child").attrib["link"], child_position, child_rotation)

    visit(roots[0], (0.0, 0.0, 0.0), identity)

    entries = []
    weighted = [0.0, 0.0, 0.0]
    total_mass = 0.0
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None:
            continue
        mass = float(inertial.find("mass").attrib["value"])
        origin = inertial.find("origin")
        local_com = _vec(origin.attrib.get("xyz") if origin is not None else None)
        link_position, link_rotation = transforms[name]
        com = _add(link_position, _mat_vec(link_rotation, local_com))
        total_mass += mass
        for index in range(3):
            weighted[index] += mass * com[index]
        entries.append({"link": name, "mass_kg": mass, "com_m": list(com)})

    if total_mass <= 0:
        raise ValueError("URDF contains no positive inertial mass")
    centre = [value / total_mass for value in weighted]
    return {
        "urdf": str(path),
        "root_link": roots[0],
        "total_mass_kg": total_mass,
        "center_of_mass_m": centre,
        "links": sorted(entries, key=lambda item: (-item["mass_kg"], item["link"])),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--top", type=int, default=12, help="Mass contributors to print")
    args = parser.parse_args()
    report = analyze(args.urdf)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0

    com = report["center_of_mass_m"]
    print(f"URDF: {report['urdf']}")
    print(f"total_mass_kg={report['total_mass_kg']:.3f}")
    print(f"center_of_mass_m=x:{com[0]:+.4f} y:{com[1]:+.4f} z:{com[2]:+.4f}")
    print("largest_mass_contributors:")
    for entry in report["links"][: args.top]:
        position = entry["com_m"]
        print(
            f"  {entry['link']}: {entry['mass_kg']:.3f} kg "
            f"@ ({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
