#!/usr/bin/env python3
"""Export authoritative compact CAD parts and report measured STL envelopes."""

from __future__ import annotations

import argparse
import json
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "cad/flywheel-launcher-v0/compact-validation-export.scad"
PARTS = (
    "chassis", "bridge", "cheeks", "handoff_ramp", "intake_wheels",
    "launcher", "launcher_cradle", "basket_collect", "basket_bin",
    "basket_hood", "basket_launch", "basket_launch_moving",
    "basket_guides", "basket_holders", "hood_supports",
)
INTERSECTIONS = (
    "launcher_bridge_intersection",
    "basket_hood_bridge_intersection",
    "basket_collect_bridge_intersection",
    "basket_raised_bridge_intersection",
    "basket_launch_bridge_intersection",
    "bridge_cheeks_intersection",
    "bridge_chassis_intersection",
    "basket_collect_chassis_intersection",
    "basket_flange_chassis_intersection",
    "basket_walls_chassis_intersection",
    "basket_floor_chassis_intersection",
    "launcher_basket_hood_intersection",
    "launcher_basket_collect_intersection",
    "launcher_basket_raised_intersection",
    "launcher_hood_raised_intersection",
    "launcher_basket_launch_intersection",
    "launcher_hood_launch_intersection",
    "basket_collect_intake_intersection",
    "basket_collect_left_wheel_intersection",
    "basket_collect_right_wheel_intersection",
    "basket_hood_intake_intersection",
    "basket_bin_intake_intersection",
    "basket_collect_battery_intersection",
    "basket_raised_battery_intersection",
    "basket_launch_battery_intersection",
    "basket_launch_lidar_intersection",
    "guides_chassis_intersection",
    "guides_intake_intersection",
    "guides_bridge_intersection",
    "guides_launcher_intersection",
    "basket_launch_holders_intersection",
    "holders_chassis_intersection",
    "holders_bridge_intersection",
    "holders_launcher_intersection",
    "hood_supports_wheels_intersection",
    "hood_supports_launcher_intersection",
    "hood_supports_bridge_intersection",
    "hood_supports_basket_intersection",
    "hood_supports_chassis_intersection",
    "handoff_ramp_left_wheel_intersection",
    "handoff_ramp_right_wheel_intersection",
)
URDF_MESH_PARTS = {
    "basket_bin_local": ROOT / "ros2_ws/src/tennis_robot/meshes/compact_relieved_bin.stl",
    "basket_hood_local": ROOT / "ros2_ws/src/tennis_robot/meshes/compact_fixed_hood.stl",
    "handoff_ramp": ROOT / "ros2_ws/src/tennis_robot/meshes/compact_relieved_handoff_ramp.stl",
}


def stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * count == len(data):
            vertices = np.empty((count * 3, 3), dtype=float)
            for index in range(count):
                values = struct.unpack_from("<12f", data, 84 + 50 * index)[3:]
                vertices[index * 3:(index + 1) * 3] = np.asarray(values).reshape(3, 3)
            return vertices
    return np.asarray([
        [float(value) for value in line.split()[1:]]
        for line in data.decode("utf-8").splitlines()
        if line.strip().startswith("vertex ")
    ])


def mesh_volume_mm3(vertices: np.ndarray) -> float:
    """Return absolute signed volume for an oriented triangle-soup STL."""
    triangles = vertices.reshape((-1, 3, 3))
    signed = np.einsum(
        "ij,ij->i", triangles[:, 0],
        np.cross(triangles[:, 1], triangles[:, 2]),
    ).sum() / 6.0
    return abs(float(signed))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--update-contract", type=Path,
                        help="Synchronize bridge envelope and known blocker volumes.")
    parser.add_argument("--bridge-width-mm", type=float,
                        help="Audit override used to compare the previous bridge width.")
    parser.add_argument("--update-urdf-meshes", action="store_true",
                        help="Regenerate the three compact CAD-derived URDF collision meshes.")
    args = parser.parse_args()
    result = {"source": str(SOURCE.relative_to(ROOT)), "units": "mm", "parts": {}}
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="compact-cad-export-", dir=runtime) as directory:
        directory_path = Path(directory)
        export_parts = [*PARTS, *INTERSECTIONS]
        if args.update_urdf_meshes:
            export_parts.extend(part for part in URDF_MESH_PARTS if part not in export_parts)
        for part in export_parts:
            stl = directory_path / f"{part}.stl"
            command = [
                args.openscad, "-D", f'part="{part}"',
                "-D", "show_guard=false", "-D", "show_feed_keepout=false",
                "-D", "show_reference_balls=false", "-o", str(stl), str(SOURCE),
            ]
            if args.bridge_width_mm is not None:
                command[1:1] = ["-D", f"oa_bridge_width={args.bridge_width_mm:g}"]
            completed = subprocess.run(command, check=False, cwd=ROOT, stdout=subprocess.PIPE,
               stderr=subprocess.STDOUT, text=True)
            if not stl.exists():
                if part not in INTERSECTIONS:
                    raise RuntimeError(
                        f"OpenSCAD failed to export {part}:\n{completed.stdout}"
                    )
                result["parts"][part] = {
                    "bbox_mm": None,
                    "intersection_volume_mm3": 0.0,
                    "vertex_count_with_duplicates": 0,
                }
                continue
            vertices = stl_vertices(stl)
            result["parts"][part] = {
                "bbox_mm": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
                "vertex_count_with_duplicates": int(len(vertices)),
            }
            if part in INTERSECTIONS:
                result["parts"][part]["intersection_volume_mm3"] = mesh_volume_mm3(vertices)
            if args.update_urdf_meshes and part in URDF_MESH_PARTS:
                destination = URDF_MESH_PARTS[part]
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(stl, destination)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    if args.update_contract:
        contract = json.loads(args.update_contract.read_text(encoding="utf-8"))
        bridge_bbox = result["parts"]["bridge"]["bbox_mm"]
        contract["components"]["plywood_bridge"]["cad_bbox"] = [
            [coordinate / 1000.0 for coordinate in point]
            for point in bridge_bbox
        ]
        for component, part in {
            "basket_collect": "basket_bin",
            "basket_fixed_hood": "basket_hood",
            "basket_launch": "basket_launch_moving",
            "handoff_ramp": "handoff_ramp",
        }.items():
            contract["components"][component]["cad_bbox"] = [
                [coordinate / 1000.0 for coordinate in point]
                for point in result["parts"][part]["bbox_mm"]
            ]
        blocker_map = {
            "launcher_vs_bridge": "launcher_bridge_intersection",
            "launcher_vs_basket_hood": "launcher_hood_launch_intersection",
            "launcher_vs_basket_launch": "launcher_basket_launch_intersection",
            "launcher_vs_basket_raised": "launcher_basket_raised_intersection",
            "basket_raised_vs_bridge": "basket_raised_bridge_intersection",
            "basket_launch_vs_bridge": "basket_launch_bridge_intersection",
        }
        for blocker, part in blocker_map.items():
            measurement = result["parts"][part]
            entry = contract["known_cad_interferences"].setdefault(
                blocker, {"status": "CAD_SOURCE_BLOCKER"}
            )
            entry["physical_intersection_volume_mm3"] = round(
                measurement["intersection_volume_mm3"], 2
            )
            entry["bbox_mm"] = measurement["bbox_mm"]
        # The approved local relief removes the genuine wall/chassis blocker.
        # Retain the flange bearing as an explicitly intentional PARKED contact,
        # never as a CAD-source interference.
        contract["known_cad_interferences"].pop(
            "basket_collect_vs_chassis", None
        )
        contact = contract.setdefault("known_intentional_contacts", {}).setdefault(
            "basket_flange_vs_chassis", {}
        )
        flange = result["parts"]["basket_flange_chassis_intersection"]
        contact.update({
            "physical_intersection_volume_mm3": round(
                flange["intersection_volume_mm3"], 2
            ),
            "bbox_mm": flange["bbox_mm"],
            "status": "INTENTIONAL_SUPPORT_CONTACT",
        })
        args.update_contract.write_text(
            json.dumps(contract, indent=2) + "\n", encoding="utf-8"
        )
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
