#!/usr/bin/env python3
"""Validate generated compact URDF geometry, inertia, mass/COM and collisions.

The CAD envelopes in config/compact_mechanical_contract.json were measured by
per-part OpenSCAD STL export from compact-packaging-study.scad.  This script
never reads Xacro source strings: it walks the generated joint tree, transforms
actual collision primitives, and compares their world-frame envelopes.

Collision checks use convex SAT over box and tessellated-cylinder collision
bodies.  The tessellation is deterministic (32 sides) and contact interfaces
can be named explicitly instead of weakening or globally disabling checks.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.spatial import ConvexHull


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "compact_mechanical_contract.json"


def _numbers(text: str | None, count: int) -> np.ndarray:
    values = [float(v) for v in (text or "").split()]
    if not values:
        values = [0.0] * count
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {text!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p),
        math.cos(y), math.sin(y),
    )
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _pose(origin: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if origin is None:
        return result
    result[:3, :3] = _rpy_matrix(_numbers(origin.get("rpy"), 3))
    result[:3, 3] = _numbers(origin.get("xyz"), 3)
    return result


def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


@dataclass(frozen=True)
class Solid:
    link: str
    name: str
    vertices: np.ndarray
    sat_enabled: bool = True

    @property
    def bbox(self) -> np.ndarray:
        return np.asarray([self.vertices.min(axis=0), self.vertices.max(axis=0)])


def _box_vertices(size: np.ndarray) -> np.ndarray:
    half = size / 2.0
    return np.asarray([
        [sx * half[0], sy * half[1], sz * half[2]]
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    ])


def _cylinder_vertices(radius: float, length: float, sides: int = 32) -> np.ndarray:
    return np.asarray([
        [radius * math.cos(2 * math.pi * i / sides),
         radius * math.sin(2 * math.pi * i / sides), z]
        for z in (-length / 2.0, length / 2.0) for i in range(sides)
    ])


def _mesh_vertices(filename: str, scale: np.ndarray) -> np.ndarray:
    prefix = "package://tennis_robot/"
    if not filename.startswith(prefix):
        raise ValueError(f"unsupported compact mesh URI: {filename}")
    path = ROOT / "ros2_ws/src/tennis_robot" / filename[len(prefix):]
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if 84 + 50 * count == len(data):
        vertices = np.empty((count * 3, 3), dtype=float)
        for index in range(count):
            values = struct.unpack_from("<12f", data, 84 + 50 * index)[3:]
            vertices[index * 3:(index + 1) * 3] = np.asarray(values).reshape(3, 3)
    else:
        vertices = np.asarray([
            [float(value) for value in line.split()[1:]]
            for line in data.decode("utf-8").splitlines()
            if line.strip().startswith("vertex ")
        ])
    return vertices * scale


def link_transforms(root: ET.Element, joint_positions: dict[str, float]) -> dict[str, np.ndarray]:
    child_joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    cache: dict[str, np.ndarray] = {}

    def resolve(link: str) -> np.ndarray:
        if link in cache:
            return cache[link]
        joint = child_joints.get(link)
        if joint is None:
            cache[link] = np.eye(4)
            return cache[link]
        parent = joint.find("parent").get("link")
        transform = resolve(parent) @ _pose(joint.find("origin"))
        value = joint_positions.get(joint.get("name", ""), 0.0)
        if joint.get("type") == "prismatic" and value:
            motion = np.eye(4)
            motion[:3, 3] = _numbers(joint.find("axis").get("xyz"), 3) * value
            transform = transform @ motion
        elif joint.get("type") in {"revolute", "continuous"} and value:
            axis = _numbers(joint.find("axis").get("xyz"), 3)
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            c, s, C = math.cos(value), math.sin(value), 1.0 - math.cos(value)
            motion = np.eye(4)
            motion[:3, :3] = np.asarray([
                [x*x*C+c, x*y*C-z*s, x*z*C+y*s],
                [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
                [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
            ])
            transform = transform @ motion
        cache[link] = transform
        return transform

    for link in root.findall("link"):
        resolve(link.get("name", ""))
    return cache


def collision_solids(root: ET.Element, positions: dict[str, float] | None = None) -> list[Solid]:
    transforms = link_transforms(root, positions or {})
    solids: list[Solid] = []
    for link in root.findall("link"):
        lname = link.get("name", "")
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            box = geometry.find("box")
            cylinder = geometry.find("cylinder")
            mesh = geometry.find("mesh")
            sat_enabled = True
            if box is not None:
                local = _box_vertices(_numbers(box.get("size"), 3))
            elif cylinder is not None:
                local = _cylinder_vertices(
                    float(cylinder.get("radius")), float(cylinder.get("length"))
                )
            elif mesh is not None:
                local = _mesh_vertices(
                    mesh.get("filename", ""), _numbers(mesh.get("scale"), 3)
                )
                # Exact non-convex mesh interference is accepted only from the
                # authoritative OpenSCAD booleans.  Vertices still participate
                # in CAD/URDF envelope validation.
                sat_enabled = False
            else:
                continue
            world = _transform_points(transforms[lname] @ _pose(collision.find("origin")), local)
            solids.append(Solid(lname, collision.get("name", ""), world, sat_enabled))
    return solids


def _bbox_for(solids: Iterable[Solid]) -> np.ndarray:
    vertices = np.concatenate([solid.vertices for solid in solids], axis=0)
    return np.asarray([vertices.min(axis=0), vertices.max(axis=0)])


def _unique_axes(axes: Iterable[np.ndarray]) -> list[np.ndarray]:
    result: list[np.ndarray] = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-8:
            continue
        axis = axis / norm
        if axis[np.argmax(np.abs(axis))] < 0:
            axis = -axis
        if not any(abs(float(np.dot(axis, old))) > 1.0 - 1e-6 for old in result):
            result.append(axis)
    return result


def _hull_axes(vertices: np.ndarray) -> tuple[list[np.ndarray], list[np.ndarray]]:
    hull = ConvexHull(vertices)
    normals = _unique_axes(eq[:3] for eq in hull.equations)
    edges = []
    for face in hull.simplices:
        for i in range(len(face)):
            edges.append(vertices[face[(i + 1) % len(face)]] - vertices[face[i]])
    return normals, _unique_axes(edges)


def sat_penetration(a: Solid, b: Solid, tolerance: float = 1e-6) -> float:
    """Return 0 when separated/touching, otherwise minimum SAT overlap."""
    if np.any(a.bbox[1] < b.bbox[0] - tolerance) or np.any(b.bbox[1] < a.bbox[0] - tolerance):
        return 0.0
    normals_a, edges_a = _hull_axes(a.vertices)
    normals_b, edges_b = _hull_axes(b.vertices)
    axes = normals_a + normals_b
    axes += _unique_axes(np.cross(ea, eb) for ea in edges_a for eb in edges_b)
    minimum = math.inf
    for axis in axes:
        pa, pb = a.vertices @ axis, b.vertices @ axis
        overlap = min(pa.max(), pb.max()) - max(pa.min(), pb.min())
        if overlap <= tolerance:
            return 0.0
        minimum = min(minimum, float(overlap))
    return minimum if math.isfinite(minimum) else 0.0


def _group(solids: list[Solid], links: Iterable[str]) -> list[Solid]:
    wanted = set(links)
    return [solid for solid in solids if solid.link in wanted]


def _interferences(solids: list[Solid], left: Iterable[str], right: Iterable[str]) -> list[dict]:
    hits = []
    for a in _group(solids, left):
        for b in _group(solids, right):
            if not a.sat_enabled or not b.sat_enabled:
                continue
            depth = sat_penetration(a, b)
            if depth > 0.0:
                hits.append({"a": f"{a.link}/{a.name}", "b": f"{b.link}/{b.name}", "depth_m": depth})
    return hits


def mass_properties(root: ET.Element, positions: dict[str, float]) -> dict:
    transforms = link_transforms(root, positions)
    entries = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        local = _pose(inertial.find("origin"))[:3, 3]
        world = _transform_points(transforms[link.get("name")], local[None, :])[0]
        entries.append((link.get("name"), mass, world))
    total = sum(mass for _, mass, _ in entries)
    com = sum(mass * point for _, mass, point in entries) / total
    return {
        "total_mass_kg": total,
        "com_m": com.tolist(),
        "links": [{"link": name, "mass_kg": mass, "com_m": point.tolist()} for name, mass, point in entries],
    }


def validate(urdf: Path, contract_path: Path, launch_urdf: Path | None = None) -> dict:
    root = ET.parse(urdf).getroot()
    launch_root = ET.parse(launch_urdf).getroot() if launch_urdf else None
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    states = {
        "basket_lowered": {"basket_joint": 0.0},
        "basket_raised": {"basket_joint": 0.1},
    }
    solids = collision_solids(root, states["basket_lowered"])
    geometry = {}
    overall_pass = True
    for name, spec in contract["components"].items():
        if "cad_bbox" not in spec:
            continue
        if spec.get("launch_only") and launch_root is None:
            continue
        component_root = launch_root if spec.get("launch_only") else root
        component_positions = {"basket_joint": 0.1} if spec.get("launch_only") else states["basket_lowered"]
        component_solids = collision_solids(component_root, component_positions)
        selected = _group(component_solids, spec["links"])
        if spec.get("collision_prefixes"):
            prefixes = tuple(spec["collision_prefixes"])
            selected = [s for s in selected if s.name.startswith(prefixes)]
        if not selected:
            geometry[name] = {"pass": False, "error": "no collision primitives"}
            overall_pass = False
            continue
        actual = _bbox_for(selected)
        expected = np.asarray(spec["cad_bbox"], dtype=float)
        delta = actual - expected
        tolerance = float(spec.get("tolerance_m", contract["bbox_tolerance_m"]))
        passed = bool(np.max(np.abs(delta)) <= tolerance)
        overall_pass &= passed
        geometry[name] = {
            "cad_bbox_m": expected.tolist(), "urdf_bbox_m": actual.tolist(),
            "delta_m": delta.tolist(), "max_abs_delta_m": float(np.max(np.abs(delta))),
            "tolerance_m": tolerance, "pass": passed,
        }

    # The sensor pose is nested in Gazebo XML rather than collision geometry.
    transforms = link_transforms(root, {})
    sensor_z = None
    for gazebo in root.findall("gazebo"):
        if gazebo.get("reference") != "lidar_link":
            continue
        for sensor in gazebo.findall("sensor"):
            if sensor.get("name") == "front_lidar":
                sensor_z = float((sensor.findtext("pose") or "0 0 0 0 0 0").split()[2])
    lidar_z = float(transforms["lidar_link"][2, 3] + (sensor_z or 0.0))
    lidar_expected = contract["components"]["lidar_scan_plane"]["cad_scalar_z"]
    lidar_pass = abs(lidar_z - lidar_expected) <= contract["components"]["lidar_scan_plane"]["tolerance_m"]
    geometry["lidar_scan_plane"] = {"cad_z_m": lidar_expected, "urdf_z_m": lidar_z, "delta_m": lidar_z - lidar_expected, "pass": lidar_pass}
    overall_pass &= lidar_pass

    groups = {
        "launcher": ["flywheel_launcher_frame_link", "flywheel_left_link", "flywheel_right_link"],
        "flywheels": ["flywheel_left_link", "flywheel_right_link"],
        "intake_cheeks": ["compact_intake_cheeks_link"],
        "ramp": ["compact_handoff_ramp_link"],
        "basket": ["basket_link"],
        "lidar": ["lidar_link"],
        "chassis": ["base_link"],
        "drive_wheels": ["rear_left_wheel_link", "rear_right_wheel_link", "front_left_wheel_link", "front_right_wheel_link"],
        "bridge": ["compact_bridge_link"],
        "guides": ["basket_rails_link"],
        "holders": ["basket_raised_holders_link"],
    }
    checks = [
        ("flywheel_vs_cheeks", "flywheels", "intake_cheeks"),
        ("flywheel_vs_ramp", "flywheels", "ramp"),
        ("launcher_vs_basket", "launcher", "basket"),
        ("launcher_vs_bridge", "launcher", "bridge"),
        ("basket_vs_bridge", "basket", "bridge"),
        ("basket_vs_lidar", "basket", "lidar"),
        ("basket_vs_chassis", "basket", "chassis"),
        ("cheeks_vs_bridge", "intake_cheeks", "bridge"),
        ("bridge_vs_chassis", "bridge", "chassis"),
        ("drive_wheels_vs_chassis", "drive_wheels", "chassis"),
        ("guides_vs_bridge", "guides", "bridge"),
        ("guides_vs_launcher", "guides", "launcher"),
        ("guides_vs_intake", "guides", "intake_cheeks"),
        ("holders_vs_bridge", "holders", "bridge"),
        ("holders_vs_launcher", "holders", "launcher"),
        ("holders_vs_basket", "holders", "basket"),
    ]
    collision_states = {}
    state_models = {name: (root, positions) for name, positions in states.items()}
    if launch_root is not None:
        state_models["basket_launch_pose"] = (launch_root, {"basket_joint": 0.1})
    for state, (state_root, positions) in state_models.items():
        state_solids = collision_solids(state_root, positions)
        state_result = {}
        for label, left, right in checks:
            hits = _interferences(state_solids, groups[left], groups[right])
            state_result[label] = {"pass": not hits, "penetrations": hits}
            overall_pass &= not hits
        collision_states[state] = state_result

    masses = {state: mass_properties(state_root, positions)
              for state, (state_root, positions) in state_models.items()}
    # Four rectangular tyre patches form a conservative support rectangle.
    # COM projection must stay inside x +/-0.415, y +/-0.390; margins reported.
    for props in masses.values():
        x, y, _ = props["com_m"]
        props["support_polygon_margin_m"] = min(0.415 - abs(x), 0.390 - abs(y))
        props["statically_stable"] = props["support_polygon_margin_m"] > 0.0
        overall_pass &= props["statically_stable"]

    return {"urdf": str(urdf), "launch_urdf": str(launch_urdf) if launch_urdf else None,
            "contract": str(contract_path), "known_cad_interferences": contract.get("known_cad_interferences", {}), "geometry": geometry,
            "collisions": collision_states, "mass_properties": masses, "pass": bool(overall_pass)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--launch-urdf", type=Path)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    result = validate(args.urdf, args.contract, args.launch_urdf)
    text = json.dumps(result, indent=2)
    if args.json_output:
        args.json_output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
