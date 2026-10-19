#!/usr/bin/env python3
"""Static semantic checks for one expanded Phase-2 v3 isolated URDF."""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def xyz(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.zeros(3)
    return np.array([float(v) for v in element.attrib.get("xyz", "0 0 0").split()])


def rpy_matrix(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.eye(3)
    r, p, y = [float(v) for v in element.attrib.get("rpy", "0 0 0").split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--case", required=True, choices=("empty", "full45", "ballast"))
    ap.add_argument("--contract", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    contract = json.loads(args.contract.read_text())
    root = ET.parse(args.urdf).getroot()
    links = {e.attrib["name"]: e for e in root.findall("link")}
    joints = {e.attrib["name"]: e for e in root.findall("joint")}
    errors: list[str] = []

    parents: dict[str, tuple[str, np.ndarray, np.ndarray]] = {}
    for name, joint in joints.items():
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        parents[child] = (parent, xyz(origin), rpy_matrix(origin))
        if joint.attrib["type"] == "revolute":
            limit = joint.find("limit")
            if limit is None:
                errors.append(f"{name}: missing revolute limit")
            else:
                low, high = float(limit.attrib["lower"]), float(limit.attrib["upper"])
                if not low < high or float(limit.attrib["effort"]) <= 0 or float(limit.attrib["velocity"]) <= 0:
                    errors.append(f"{name}: invalid limits")
        if joint.attrib["type"] == "continuous":
            limit = joint.find("limit")
            if limit is None or float(limit.attrib["effort"]) <= 0 or float(limit.attrib["velocity"]) <= 0:
                errors.append(f"{name}: invalid continuous effort/velocity")

    transforms: dict[str, tuple[np.ndarray, np.ndarray]] = {"base_link": (np.zeros(3), np.eye(3))}
    unresolved = set(links) - {"base_link"}
    while unresolved:
        progressed = False
        for child in list(unresolved):
            if child not in parents:
                errors.append(f"{child}: no parent joint")
                unresolved.remove(child)
                continue
            parent, translation, rotation = parents[child]
            if parent not in transforms:
                continue
            parent_t, parent_r = transforms[parent]
            transforms[child] = (parent_t + parent_r @ translation, parent_r @ rotation)
            unresolved.remove(child)
            progressed = True
        if not progressed and unresolved:
            errors.append(f"unresolved kinematic links: {sorted(unresolved)}")
            break

    total_mass = 0.0
    weighted = np.zeros(3)
    mass_properties: list[tuple[float, np.ndarray, np.ndarray]] = []
    inertias = {}
    collision_links = []
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            errors.append(f"{name}: no inertial")
            continue
        mass = float(inertial.find("mass").attrib["value"])
        ie = inertial.find("inertia").attrib
        matrix = np.array([
            [float(ie["ixx"]), float(ie["ixy"]), float(ie["ixz"])],
            [float(ie["ixy"]), float(ie["iyy"]), float(ie["iyz"])],
            [float(ie["ixz"]), float(ie["iyz"]), float(ie["izz"])],
        ])
        eigen = np.linalg.eigvalsh(matrix)
        inertias[name] = eigen.tolist()
        if mass <= 0 or np.min(eigen) <= 0:
            errors.append(f"{name}: non-positive mass/inertia {mass}, {eigen.tolist()}")
        if name in transforms:
            link_t, link_r = transforms[name]
            inertial_origin = inertial.find("origin")
            local = xyz(inertial_origin)
            position = link_t + link_r @ local
            inertia_rotation = link_r @ rpy_matrix(inertial_origin)
            inertia_base = inertia_rotation @ matrix @ inertia_rotation.T
            total_mass += mass
            weighted += mass * position
            mass_properties.append((mass, position, inertia_base))
        if link.findall("collision"):
            collision_links.append(name)
    cg = weighted / total_mass
    total_inertia = np.zeros((3, 3))
    for mass, position, inertia_base in mass_properties:
        offset = position - cg
        total_inertia += inertia_base + mass * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )

    expected = {"empty": 18.019957627006, "full45": 20.584957627006, "ballast": 18.85788235294}[args.case]
    if abs(total_mass - expected) > 1e-9:
        errors.append(f"mass {total_mass:.9f} kg differs from contract-derived {expected:.9f} kg")
    required_collisions = {"base_link", "rear_left_wheel_link", "rear_right_wheel_link", "front_left_wheel_link", "front_right_wheel_link", "basket_support_link", "parkside_left_link", "parkside_right_link", "ugreen_logic_power_link", "p16_body_link", "launcher_endpoint_stops_link"}
    if args.case == "full45":
        required_collisions.add("basket_ball_payload_link")
    if args.case == "ballast":
        required_collisions.add("drive_test_ballast_link")
    else:
        required_collisions |= {"flywheel_launcher_frame_link", "flywheel_left_link", "flywheel_right_link"}
    missing_collisions = sorted(required_collisions - set(collision_links))
    if missing_collisions:
        errors.append(f"missing collision links: {missing_collisions}")

    ros2 = root.find("ros2_control")
    if ros2 is None:
        errors.append("missing ros2_control")
        control_joints = []
    else:
        control_joints = [e.attrib["name"] for e in ros2.findall("joint")]
        for name in control_joints:
            if name not in joints:
                errors.append(f"ros2_control references missing joint {name}")
        required_control = {"rear_left_wheel_joint", "rear_right_wheel_joint", "front_left_wheel_joint", "front_right_wheel_joint", "launcher_tilt_joint"}
        if args.case != "ballast":
            required_control |= {"flywheel_left_joint", "flywheel_right_joint"}
        if set(control_joints) != required_control:
            errors.append(f"ros2_control joints differ: {sorted(control_joints)} vs {sorted(required_control)}")

    tilt = joints.get("launcher_tilt_joint")
    if tilt is None:
        errors.append("missing launcher_tilt_joint")
    else:
        pivot_expected = np.array(contract["geometry_m"]["launcher_pivot"])
        pivot_actual = xyz(tilt.find("origin"))
        if not np.allclose(pivot_actual, pivot_expected, rtol=0, atol=1e-12):
            errors.append(f"launcher pivot {pivot_actual.tolist()} differs from contract {pivot_expected.tolist()}")
        lim = tilt.find("limit").attrib
        expected_limits = contract["geometry_m"]["launcher_limits_rad"]
        if abs(float(lim["lower"]) - expected_limits[0]) > 1e-9 or abs(float(lim["upper"]) - expected_limits[1]) > 1e-9:
            errors.append("launcher limits differ from contract")

    if args.case != "ballast":
        wheel_contract = contract["geometry_m"]["intake_contact_static_gate"]["p2_40_selected_geometry"]
        for side in ("left", "right"):
            link = links.get(f"flywheel_{side}_link")
            if link is None:
                errors.append(f"missing flywheel_{side}_link")
                continue
            inertial = link.find("inertial")
            mass = float(inertial.find("mass").attrib["value"])
            inertia = inertial.find("inertia").attrib
            collision = link.find(f"collision[@name='flywheel_{side}_col']/geometry/cylinder")
            if abs(mass - wheel_contract["printed_wheel_mass_each_including_hub_kg"]) > 1e-12:
                errors.append(f"flywheel {side} mass differs from P2-40 contract")
            if abs(float(inertia["izz"]) - wheel_contract["printed_wheel_polar_inertia_each_kg_m2"]) > 1e-12:
                errors.append(f"flywheel {side} polar inertia differs from P2-40 contract")
            if abs(float(inertia["ixx"]) - wheel_contract["printed_wheel_transverse_inertia_each_kg_m2"]) > 1e-12:
                errors.append(f"flywheel {side} transverse inertia differs from P2-40 contract")
            if collision is None or abs(float(collision.attrib["length"]) - wheel_contract["printed_wheel_width_m"]) > 1e-12:
                errors.append(f"flywheel {side} collision width differs from P2-40 contract")

        stop_link = links.get("launcher_endpoint_stops_link")
        stop_names = {e.attrib.get("name") for e in stop_link.findall("collision")} if stop_link is not None else set()
        if not {"collection_stop_col", "launch_stop_col"}.issubset(stop_names):
            errors.append("selected collection/launch terminal collisions are missing")

    result = {
        "case": args.case,
        "urdf": str(args.urdf),
        "pass": not errors,
        "errors": errors,
        "link_count": len(links),
        "joint_count": len(joints),
        "total_mass_kg": total_mass,
        "cg_at_zero_joint_pose_m": cg.tolist(),
        "total_inertia_at_cg_kg_m2": total_inertia.tolist(),
        "minimum_inertia_eigenvalue_kg_m2": min(min(v) for v in inertias.values()),
        "collision_links": sorted(collision_links),
        "ros2_control_joints": sorted(control_joints),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
