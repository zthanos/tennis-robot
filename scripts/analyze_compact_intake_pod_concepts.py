#!/usr/bin/env python3
"""Measure a HISTORICAL_SUPERSEDED moving-pod package.

NOT_PHYSICAL_INTAKE_ARCHITECTURE. Retained only as a dependency of legacy
corrected launcher-packaging analysis; current motors and wheel centres are fixed.

Collision decisions use exact OpenSCAD/CGAL Boolean intersections. Positive
clearances are explicitly labelled sampled estimates and never override a
non-zero CGAL intersection.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "cad/flywheel-launcher-v0/compact-intake-pod-concept-study.scad"
DISPLACEMENTS = (0, 4, 8)
TARGETS = (
    "basket", "hood", "ramp", "cheeks", "bridge", "chassis", "battery",
    "launcher_cradle", "left_flywheel", "right_flywheel",
)
HARDWARE_COMPONENTS = (
    "motor", "motor_output_shaft", "adapter", "wheel_hex", "wheel",
    "direct_drive_hardware",
)
MOVING_COMPONENTS = HARDWARE_COMPONENTS + (
    "carriage_moving", "complete_moving_pod",
)


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
    triangles = vertices.reshape((-1, 3, 3))
    signed = np.einsum(
        "ij,ij->i", triangles[:, 0],
        np.cross(triangles[:, 1], triangles[:, 2]),
    ).sum() / 6.0
    return abs(float(signed))


def bbox(vertices: np.ndarray) -> list[list[float]]:
    return [vertices.min(axis=0).round(3).tolist(),
            vertices.max(axis=0).round(3).tolist()]


def surface_samples(vertices: np.ndarray, spacing_mm: float = 4.0) -> np.ndarray:
    """Deterministic surface samples; used only for positive-clearance estimates."""
    triangles = vertices.reshape((-1, 3, 3))
    samples = [vertices, triangles.mean(axis=1)]
    for index, tri in enumerate(triangles):
        area = np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) / 2.0
        count = min(10_000, max(0, math.ceil(area / spacing_mm**2) - 1))
        if count == 0:
            continue
        sequence = np.arange(count, dtype=float) + 0.5
        r1 = sequence / count
        r2 = np.mod(sequence * 0.6180339887498949 + index * 0.41421356237, 1.0)
        root = np.sqrt(r1)
        weights = np.column_stack((1.0 - root, root * (1.0 - r2), root * r2))
        samples.append(weights @ tri)
    return np.vstack(samples)


def sampled_clearance_mm(a: np.ndarray, b: np.ndarray) -> float:
    tree_a = cKDTree(a)
    tree_b = cKDTree(b)
    da = tree_b.query(a, k=1, workers=-1)[0].min()
    db = tree_a.query(b, k=1, workers=-1)[0].min()
    return float(min(da, db))


def render(openscad: str, directory: Path, name: str, *, part: str,
           displacement: int = 0, target: str = "bridge",
           component: str = "direct_drive_hardware", selected_side: int = 0,
           adapter_seating_mm: int = 8) -> Path | None:
    path = directory / f"{name}.stl"
    command = [
        openscad,
        "-D", f'displacement={displacement}',
        "-D", f'part="{part}"',
        "-D", f'target="{target}"',
        "-D", f'component="{component}"',
        "-D", f'selected_side={selected_side}',
        "-D", f'adapter_seating_mm={adapter_seating_mm}',
        "-o", str(path), str(SOURCE),
    ]
    completed = subprocess.run(
        command, cwd=ROOT, check=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    if path.exists() and path.stat().st_size > 84:
        return path
    empty_markers = ("Current top level object is empty", "No top level geometry")
    if any(marker in completed.stdout for marker in empty_markers):
        return None
    if completed.returncode != 0:
        raise RuntimeError(f"OpenSCAD failed for {name}:\n{completed.stdout}")
    if not path.exists() or path.stat().st_size <= 84:
        return None
    raise RuntimeError(f"Unexpected OpenSCAD export for {name}:\n{completed.stdout}")


def intersection_record(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {"intersection_volume_mm3": 0.0, "intersection_bbox_mm": None}
    vertices = stl_vertices(path)
    return {
        "intersection_volume_mm3": round(mesh_volume_mm3(vertices), 3),
        "intersection_bbox_mm": bbox(vertices),
    }


def run_intersections(openscad: str, directory: Path,
                      specs: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    def one(spec: dict[str, Any]) -> tuple[str, dict[str, Any]]:
        path = render(openscad, directory, spec["name"], **spec["render"])
        return spec["key"], intersection_record(path)

    with ThreadPoolExecutor(max_workers=4) as pool:
        return dict(pool.map(one, specs))


def add_clearance(record: dict[str, Any], component_samples: np.ndarray,
                  target_samples: np.ndarray) -> None:
    if record["intersection_volume_mm3"] == 0.0:
        record["sampled_min_positive_clearance_mm"] = round(
            sampled_clearance_mm(component_samples, target_samples), 2)
    else:
        record["sampled_min_positive_clearance_mm"] = None


def all_zero(matrix: dict[str, Any], component: str) -> bool:
    return all(
        check["intersection_volume_mm3"] == 0.0
        for position in matrix.values()
        for check in position["components"][component].values()
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=4,
                        help="reserved for compatibility; CGAL jobs are capped at four")
    args = parser.parse_args()

    result: dict[str, Any] = {
        "source": str(SOURCE.relative_to(ROOT)),
        "units": "mm",
        "generated_by": str(Path(__file__).resolve().relative_to(ROOT)),
        "collision_method": "exact OpenSCAD/CGAL Boolean intersection volume",
        "clearance_method": (
            "bidirectional deterministic STL surface sampling at nominal 4 mm; "
            "positive-clearance estimates do not determine collision status"
        ),
        "evidence": {
            "FIT0186_body_and_shaft": "MEASURED_FROM_HARDWARE: body diameter 30, body length 70, 6 mm D-shaft, approximately 20 projection",
            "14-00012630": "PURCHASED_PART_DATUM: 6 mm shaft side, 12 mm hex wheel side, total length 30, quantity 6",
            "14-00012630_external_profile": "MEASUREMENT_PENDING; analysis-only conservative diameter 20",
            "PRO117010_envelope": "MANUFACTURER_SPEC: diameter 124, width 73, Raid 6x30 removable interface and supplied 12 mm hex",
            "PRO117010_installed_hex": "PHYSICAL_MEASUREMENT_PENDING: exact offset, depth and seating",
            "adapter_seating": "ANALYSIS_ONLY_ASSUMPTION: 8 mm nominal; sensitivity at 0 and 12 mm",
            "machine_obstacles": "EXISTING_AUTHORITATIVE_CAD",
            "wheel_centres_axis_and_travel": "DERIVED_FROM_LOCKED_DATUM",
            "candidate_A_structure": "ANALYSIS_ONLY_ASSUMPTION; allocation probe, not manufacturing CAD",
        },
        "removed_false_assumptions": [
            "198 mm transmission shaft", "external bearings and supports",
            "bearing cartridge", "flexible coupler", "printed wheel hub",
            "invented long-stack motor clamp/mount", "belt/pulley/gearing",
        ],
        "historical_results": {
            "status": "INVALID_DUE_TO_INCORRECT_INTAKE_AXIS",
            "motor_bridge_mm3": 33986,
            "motor_launcher_mm3": 25174,
            "motor_flywheel_each_mm3": 722,
            "wheel_basket_mm3": 499,
        },
        "scene_completeness": {
            name: True for name in (
                "left_complete_intake_pod", "right_complete_intake_pod",
                "both_intake_wheels", "both_FIT0186_motors",
                "both_14-00012630_adapters", "intake_cheeks", "handoff_ramp",
                "receiving_hood_channel", "basket_bin", "bridge", "chassis",
                "battery", "launcher_cradle", "left_flywheel", "right_flywheel",
            )
        },
    }
    sin_tilt = math.sin(math.radians(35.0))
    cos_tilt = math.cos(math.radians(35.0))
    motor_axis_distance = 93.5
    result["orientation_gate"] = {
        "status": "PASS",
        "frame": "compact ground frame, mm",
        "tilt_deg": 35.0,
        "tilt_plane": "ROBOT_LONGITUDINAL_X_Z",
        "front_view_axes_parallel": True,
        "independent_motor_outboard_offset_mm": 0.0,
        "sides": {},
    }
    for side, label in ((1, "left"), (-1, "right")):
        axis = [sin_tilt, 0.0, cos_tilt]
        wheel = [370.0, side * 90.0, 70.0]
        vector = [coordinate * motor_axis_distance for coordinate in axis]
        motor = [wheel[index] + vector[index] for index in range(3)]
        result["orientation_gate"]["sides"][label] = {
            "wheel_center_mm": [round(value, 6) for value in wheel],
            "wheel_to_motor_axis_unit_vector": [round(value, 9) for value in axis],
            "motor_center_mm": [round(value, 6) for value in motor],
            "wheel_to_motor_vector_mm": [round(value, 6) for value in vector],
            "lateral_offset_mm": round(vector[1], 6),
            "forward_offset_mm": round(vector[0], 6),
        }

    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="direct-pod-study-", dir=runtime) as temp:
        directory = Path(temp)

        # Stable environment meshes and samples.
        target_samples: dict[str, np.ndarray] = {}
        target_bounds: dict[str, list[list[float]]] = {}
        for target_name in TARGETS:
            path = render(args.openscad, directory, f"env_{target_name}",
                          part="environment", target=target_name)
            if path is None:
                raise RuntimeError(f"missing environment solid: {target_name}")
            vertices = stl_vertices(path)
            target_samples[target_name] = surface_samples(vertices)
            target_bounds[target_name] = bbox(vertices)
        result["environment_bbox_mm"] = target_bounds

        endpoint: dict[str, Any] = {}
        for travel in DISPLACEMENTS:
            component_samples: dict[str, np.ndarray] = {}
            component_bounds: dict[str, list[list[float]]] = {}
            for component_name in MOVING_COMPONENTS:
                path = render(
                    args.openscad, directory, f"component_{travel}_{component_name}",
                    part="component", displacement=travel, component=component_name,
                )
                if path is None:
                    raise RuntimeError(f"missing moving component: {component_name}")
                vertices = stl_vertices(path)
                component_samples[component_name] = surface_samples(vertices)
                component_bounds[component_name] = bbox(vertices)

            specs = []
            for component_name in MOVING_COMPONENTS:
                for target_name in TARGETS:
                    key = f"{component_name}|{target_name}"
                    specs.append({
                        "key": key,
                        "name": f"x_{travel}_{component_name}_{target_name}",
                        "render": {
                            "part": "component_environment_intersection",
                            "displacement": travel, "component": component_name,
                            "target": target_name,
                        },
                    })
            records = run_intersections(args.openscad, directory, specs)
            matrix: dict[str, Any] = {}
            for component_name in MOVING_COMPONENTS:
                matrix[component_name] = {}
                for target_name in TARGETS:
                    record = records[f"{component_name}|{target_name}"]
                    add_clearance(record, component_samples[component_name],
                                  target_samples[target_name])
                    matrix[component_name][target_name] = record

            opposite_path = render(
                args.openscad, directory, f"opposite_{travel}",
                part="opposite_pod_intersection", displacement=travel,
                component="complete_moving_pod",
            )
            side_details: dict[str, Any] = {}
            for component_name in HARDWARE_COMPONENTS[:-1] + ("carriage_moving",):
                for target_name, aggregate in matrix[component_name].items():
                    if aggregate["intersection_volume_mm3"] == 0.0:
                        continue
                    relation = f"{component_name}|{target_name}"
                    side_details[relation] = {}
                    for side, label in ((1, "left"), (-1, "right")):
                        side_path = render(
                            args.openscad, directory,
                            f"side_{travel}_{label}_{component_name}_{target_name}",
                            part="component_environment_intersection",
                            displacement=travel, component=component_name,
                            target=target_name, selected_side=side,
                        )
                        side_details[relation][label] = intersection_record(side_path)
            endpoint[str(travel)] = {
                "component_bbox_mm": component_bounds,
                "components": matrix,
                "collision_details_by_side": side_details,
                "opposite_intake_pod": intersection_record(opposite_path),
            }
        result["endpoint_0_4_8"] = endpoint

        # Fixed Candidate A guide members.
        fixed_path = render(args.openscad, directory, "fixed_guides",
                            part="fixed_guides")
        assert fixed_path is not None
        fixed_vertices = stl_vertices(fixed_path)
        fixed_samples = surface_samples(fixed_vertices)
        fixed_specs = [{
            "key": target_name,
            "name": f"fixed_x_{target_name}",
            "render": {"part": "fixed_guide_environment_intersection",
                       "target": target_name},
        } for target_name in TARGETS]
        fixed_records = run_intersections(args.openscad, directory, fixed_specs)
        for target_name, record in fixed_records.items():
            add_clearance(record, fixed_samples, target_samples[target_name])
        result["candidate_A_fixed_guide"] = {
            "bbox_mm": bbox(fixed_vertices), "environment": fixed_records,
        }

        # Continuous 0..8 mm swept volumes.
        swept_samples: dict[str, np.ndarray] = {}
        swept_bounds: dict[str, list[list[float]]] = {}
        for component_name in MOVING_COMPONENTS:
            path = render(
                args.openscad, directory, f"swept_{component_name}",
                part="swept_component", component=component_name,
            )
            if path is None:
                raise RuntimeError(f"missing swept component: {component_name}")
            vertices = stl_vertices(path)
            swept_samples[component_name] = surface_samples(vertices)
            swept_bounds[component_name] = bbox(vertices)
        swept_specs = []
        for component_name in MOVING_COMPONENTS:
            for target_name in TARGETS:
                swept_specs.append({
                    "key": f"{component_name}|{target_name}",
                    "name": f"swept_x_{component_name}_{target_name}",
                    "render": {
                        "part": "swept_component_environment_intersection",
                        "component": component_name, "target": target_name,
                    },
                })
        swept_raw = run_intersections(args.openscad, directory, swept_specs)
        swept_matrix: dict[str, Any] = {}
        for component_name in MOVING_COMPONENTS:
            swept_matrix[component_name] = {}
            for target_name in TARGETS:
                record = swept_raw[f"{component_name}|{target_name}"]
                add_clearance(record, swept_samples[component_name],
                              target_samples[target_name])
                swept_matrix[component_name][target_name] = record
        opposite_swept_path = render(
            args.openscad, directory, "swept_opposite",
            part="swept_opposite_pod_intersection",
            component="complete_moving_pod",
        )
        swept_side_details: dict[str, Any] = {}
        for component_name in HARDWARE_COMPONENTS[:-1] + ("carriage_moving",):
            for target_name, aggregate in swept_matrix[component_name].items():
                if aggregate["intersection_volume_mm3"] == 0.0:
                    continue
                relation = f"{component_name}|{target_name}"
                swept_side_details[relation] = {}
                for side, label in ((1, "left"), (-1, "right")):
                    side_path = render(
                        args.openscad, directory,
                        f"swept_side_{label}_{component_name}_{target_name}",
                        part="swept_component_environment_intersection",
                        component=component_name, target=target_name,
                        selected_side=side,
                    )
                    swept_side_details[relation][label] = intersection_record(side_path)
        result["continuous_0_to_8_swept"] = {
            "method": "exact CGAL intersection against continuous endpoint-hull swept solids; carriage follower bores conservatively filled",
            "component_bbox_mm": swept_bounds,
            "components": swept_matrix,
            "collision_details_by_side": swept_side_details,
            "opposite_intake_pod": intersection_record(opposite_swept_path),
        }

        # Seating sensitivity only for direct hardware against all obstacles.
        sensitivity: dict[str, Any] = {}
        for seating in (0, 12):
            specs = [{
                "key": target_name,
                "name": f"sensitivity_{seating}_{target_name}",
                "render": {
                    "part": "swept_component_environment_intersection",
                    "component": "direct_drive_hardware", "target": target_name,
                    "adapter_seating_mm": seating,
                },
            } for target_name in TARGETS]
            sensitivity[str(seating)] = run_intersections(
                args.openscad, directory, specs)
        result["adapter_seating_sensitivity_swept"] = sensitivity

    direct_endpoint_clear = all_zero(endpoint, "direct_drive_hardware")
    direct_swept_clear = all(
        item["intersection_volume_mm3"] == 0.0
        for item in result["continuous_0_to_8_swept"]["components"]
                          ["direct_drive_hardware"].values()
    )
    carriage_moving_clear = all_zero(endpoint, "carriage_moving") and all(
        item["intersection_volume_mm3"] == 0.0
        for item in result["continuous_0_to_8_swept"]["components"]
                          ["carriage_moving"].values()
    )
    fixed_guide_clear = all(
        item["intersection_volume_mm3"] == 0.0
        for item in result["candidate_A_fixed_guide"]["environment"].values()
    )
    opposite_clear = all(
        item["opposite_intake_pod"]["intersection_volume_mm3"] == 0.0
        for item in endpoint.values()
    ) and result["continuous_0_to_8_swept"]["opposite_intake_pod"]\
              ["intersection_volume_mm3"] == 0.0

    hardware_clear = direct_endpoint_clear and direct_swept_clear and opposite_clear
    guide_clear = carriage_moving_clear and fixed_guide_clear and opposite_clear
    result["available_carriage_design_envelope"] = {
        "status": "NOT_RELEASED_WHILE_DIRECT_HARDWARE_COLLISION_EXISTS" if not hardware_clear else "MEASURED_ALLOCATION_BOUNDS_ONLY",
        "hardware_swept_bbox_mm": result["continuous_0_to_8_swept"]["component_bbox_mm"]["direct_drive_hardware"],
        "candidate_A_moving_allocation_bbox_mm": result["continuous_0_to_8_swept"]["component_bbox_mm"]["carriage_moving"],
        "candidate_A_fixed_allocation_bbox_mm": result["candidate_A_fixed_guide"]["bbox_mm"],
        "note": "No final support, spring, cable, stop, or fastener geometry is inferred from these bounds.",
    }
    result["bridge_relief_screen"] = {
        "existing_service_opening_xy_mm": [22, 22],
        "motor_horizontal_plane_cross_axis_x_mm": round(30 / math.cos(math.radians(35)), 2),
        "motor_horizontal_plane_cross_axis_y_mm": 30,
        "minimum_opening_with_2mm_each_side_xy_mm": [
            34, round(30 / math.cos(math.radians(35)) + 8 + 4, 2),
        ],
        "classification": "ANALYTIC_SCREEN_ONLY; any bridge relief is a new mechanical decision",
    }
    result["classifications"] = {
        "INTAKE_35_DEGREE_ORIENTATION_CORRECTED": True,
        "LEFT_MOTOR_IS_OUTBOARD": True,
        "RIGHT_MOTOR_IS_OUTBOARD": True,
        "INTAKE_AXES_INTENDED_MIRROR_VALIDATED": True,
        "DIRECT_DRIVE_INTAKE_POD_PACKAGING_GEOMETRICALLY_VALID": hardware_clear and guide_clear,
        "DIRECT_DRIVE_HARDWARE_0_TO_8MM_SWEPT_CLEAR": hardware_clear,
        "CANDIDATE_A_GUIDE_PACKAGING_GEOMETRICALLY_VALID": guide_clear,
        "INTAKE_MOVING_POD_ARCHITECTURAL_REDESIGN_REQUIRED": not hardware_clear,
        "INTAKE_CARRIAGE_MANUFACTURING_CAD_PENDING": True,
        "COMPACT_PHYSICAL_INTAKE_MODEL_COMPLETE": False,
        "COMPACT_INTAKE_STATIC_PACKAGING_VALIDATED": hardware_clear and guide_clear,
        "COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM": False,
        "COMPACT_INTAKE_HANDOFF_VALIDATED_IN_SIM": False,
        "COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM": False,
        "BALL_LAUNCH_PHYSICS_NOT_VALIDATED": True,
        "PHYSICAL_HARDWARE_PENDING": True,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["classifications"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
