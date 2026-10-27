#!/usr/bin/env python3
"""Repeatable STL envelope/solid-volume measurements; not a slicer or FEA."""
from __future__ import annotations

import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BENCH = Path(__file__).resolve().parent
PARTS = {
    "motor_left": 1, "motor_right": 1,
    "front_motor_gamma_left": 1, "front_motor_gamma_right": 1,
    "rail_extension_left": 1, "rail_extension_right": 1,
    "corner_left": 1, "corner_right": 1,
    "crossbar_half": 2, "middle_crossbar_half": 2, "straight_sleeve": 8,
    "ramp_cradle_left": 1, "ramp_cradle_right": 1,
    "electronics_tray_left": 1, "electronics_tray_right": 1,
}


def mesh_stats(data: bytes) -> dict:
    triangles = []
    binary_count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * binary_count:
        for i in range(binary_count):
            values = struct.unpack_from("<12fH", data, 84 + 50*i)
            triangles.append(tuple(tuple(values[j:j+3]) for j in (3, 6, 9)))
    else:
        vertices = [tuple(map(float, line.split()[1:]))
                    for line in data.decode().splitlines()
                    if line.strip().startswith("vertex ")]
        if not vertices or len(vertices) % 3:
            raise ValueError("Invalid or empty STL")
        triangles = [tuple(vertices[i:i+3]) for i in range(0, len(vertices), 3)]
    edges = Counter()
    signed_volume = 0.0
    mins, maxs = [float("inf")]*3, [float("-inf")]*3
    for triangle in triangles:
        a, b, c = triangle
        signed_volume += (a[0]*(b[1]*c[2]-b[2]*c[1])
                          + a[1]*(b[2]*c[0]-b[0]*c[2])
                          + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6
        keys = [tuple(round(x, 6) for x in v) for v in triangle]
        for i in range(3):
            edges[tuple(sorted((keys[i], keys[(i+1) % 3])))] += 1
        for v in triangle:
            for i in range(3):
                mins[i], maxs[i] = min(mins[i], v[i]), max(maxs[i], v[i])
    return {
        "triangle_count": len(triangles),
        "bounds_min_mm": mins, "bounds_max_mm": maxs,
        "envelope_mm": [maxs[i]-mins[i] for i in range(3)],
        "solid_volume_cm3": abs(signed_volume)/1000,
        "nonmanifold_edge_count": sum(n != 2 for n in edges.values()),
    }


def measure() -> dict:
    materials = json.loads((BENCH / "config/materials.json").read_text())["materials"]
    rows = []
    for name, quantity in PARTS.items():
        # Frozen historical geometry: current exports may now use ramp_back.
        mesh = mesh_stats(subprocess.check_output([
            "git", "show", f"3d34a3d:cad/modular-test-chassis/stl-v2/{name}.stl"
        ], cwd=ROOT))
        if mesh["nonmanifold_edge_count"]:
            raise ValueError(f"Nonmanifold STL: {name}")
        rows.append({"part": name, "quantity": quantity, **mesh})
    total_volume = sum(r["solid_volume_cm3"]*r["quantity"] for r in rows)
    old_extensions = sum(mesh_stats(subprocess.check_output([
        "git", "show", f"2a5fedc:cad/modular-test-chassis/stl-v2/rail_extension_{side}.stl"
    ], cwd=ROOT))["solid_volume_cm3"] for side in ("left", "right"))
    indexed = {r["part"]: r for r in rows}
    added_volume = (indexed["rail_extension_left"]["solid_volume_cm3"]
                    + indexed["rail_extension_right"]["solid_volume_cm3"]
                    - old_extensions
                    + 2*indexed["middle_crossbar_half"]["solid_volume_cm3"]
                    + 3*indexed["straight_sleeve"]["solid_volume_cm3"])
    spec = importlib.util.spec_from_file_location("chassis_screen", BENCH / "analyze.py")
    screen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(screen)
    config = json.loads((BENCH / "config/baseline.json").read_text())
    config["geometry"].update(wheelbase_mm=365, internal_web_mm=0,
        wheel_contact_to_rail_centroid_z_mm=37,
        wheel_contact_to_rail_centroid_y_mm=130,
        mount_torque_row_spacing_mm=25, splice_bolt_diameter_mm=3,
        splice_bolt_count_per_side=2)
    screening = []
    for payload in (0, 5, 10):
        scenario = dict(base_mass_kg=10, payload_mass_kg=payload, tyre_ground_mu=0.9,
                        dynamic_factor=2, vertical_dynamic_factor=2)
        loads = screen._scenario_loads(config, scenario)
        for name, material in materials.items():
            result = screen._screen_material(config, scenario, loads, material)
            screening.append({"material": name, "base_mass_kg": 10,
                "payload_kg": payload, "rail_deflection_mm": result["rail_deflection_mm"],
                "stress_mpa": result["stress_mpa"],
                "safety_factors": result["safety_factors"]})
    return {
        "schema_version": 1, "status": "CAD_MEASURED_FRAME_STIFFNESS_NOT_VALIDATED",
        "cad_source_commit": "3d34a3d",
        "geometry": {"wheelbase_mm": 365, "rail_section_mm": [50,30],
            "middle_crossbar_x_mm": 117.5, "middle_crossbar_between_sockets_mm": 270,
            "middle_half_length_mm": 135, "sleeves_required_total": 8,
            "sleeves_added": 3, "sleeve_gap_inside_half_mm": 15,
            "ground_clearance_structure_mm": 22, "metal_foot_to_gamma_gap_mm": 10,
            "wheel_inner_face_to_rail_outer_face_mm": 65,
            "frame_length_mm": 645, "wheel_track_mm": 700},
        "parts": rows, "total_solid_volume_cm3": total_volume,
        "additional_solid_volume_cm3_vs_2a5fedc": added_volume,
        "solid_mass_estimates_g": {
            name: {"complete_design": total_volume*mat["density_kg_m3"]/1000,
                   "middle_reinforcement_added": added_volume*mat["density_kg_m3"]/1000}
            for name, mat in materials.items()},
        "rail_only_screen": screening,
        "limitations": [
            "Mass uses fully dense CAD solids; slicer infill, supports and brim are excluded.",
            "Material densities/moduli/allowables are provisional project profiles, not measured coupons.",
            "Rail screen assumes 10 kg base, equally shared wheel loads, centred payload and factor 2.",
            "The 1-D screen excludes the T ribs, frame racking, joints, screw preload and layer anisotropy.",
            "DART rejected the second middle-crossbar weld; its captured structural loads are invalid.",
        ],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = measure()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k: report[k] for k in ("total_solid_volume_cm3",
        "additional_solid_volume_cm3_vs_2a5fedc", "solid_mass_estimates_g")}, indent=2))
