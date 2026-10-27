#!/usr/bin/env python3
"""Analysis-only corrected compact packaging relief/translation study."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial import ConvexHull

from analyze_compact_intake_pod_concepts import (
    ROOT, bbox, intersection_record, stl_vertices,
)


SOURCE = ROOT / "cad/flywheel-launcher-v0/compact-corrected-packaging-resolution-study.scad"


def render(openscad: str, directory: Path, name: str, **defines: Any) -> Path | None:
    path = directory / f"{name}.stl"
    command = [openscad]
    for key, value in defines.items():
        if isinstance(value, str):
            expression = f'{key}="{value}"'
        else:
            expression = f"{key}={value}"
        command.extend(("-D", expression))
    command.extend(("-o", str(path), str(SOURCE)))
    completed = subprocess.run(
        command, cwd=ROOT, check=False, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True,
    )
    if path.exists() and path.stat().st_size > 84:
        return path
    if "Current top level object is empty" in completed.stdout or \
       "No top level geometry" in completed.stdout:
        return None
    if completed.returncode != 0:
        raise RuntimeError(f"OpenSCAD failed for {name}:\n{completed.stdout}")
    return None


def exact(openscad: str, directory: Path, name: str, **defines: Any) -> dict[str, Any]:
    return intersection_record(render(openscad, directory, name, **defines))


def exact_many(openscad: str, directory: Path,
               specs: list[tuple[str, dict[str, Any]]]) -> dict[str, dict[str, Any]]:
    def one(item: tuple[str, dict[str, Any]]) -> tuple[str, dict[str, Any]]:
        name, defines = item
        return name, exact(openscad, directory, name, **defines)
    with ThreadPoolExecutor(max_workers=4) as pool:
        return dict(pool.map(one, specs))


def convex_axes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hull = ConvexHull(points)
    normals = hull.equations[:, :3]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    edges: set[tuple[int, int]] = set()
    for face in hull.simplices:
        for index in range(3):
            a, b = sorted((int(face[index]), int(face[(index + 1) % 3])))
            edges.add((a, b))
    directions = np.asarray([points[b] - points[a] for a, b in edges])
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    return normals, directions


def unique_axes(axes: np.ndarray, decimals: int = 7) -> np.ndarray:
    axes = axes[np.linalg.norm(axes, axis=1) > 1e-9]
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    # Axis signs are equivalent for SAT.
    flip = np.argmax(np.abs(axes), axis=1)
    signs = np.sign(axes[np.arange(len(axes)), flip])
    axes *= signs[:, None]
    return np.unique(np.round(axes, decimals), axis=0)


def sat_data(a: np.ndarray, b: np.ndarray) -> dict[str, Any]:
    normals_a, edges_a = convex_axes(a)
    normals_b, edges_b = convex_axes(b)
    crosses = np.cross(edges_a[:, None, :], edges_b[None, :, :]).reshape(-1, 3)
    axes = unique_axes(np.vstack((normals_a, normals_b, crosses)))
    amin = (a @ axes.T).min(axis=0)
    amax = (a @ axes.T).max(axis=0)
    bmin = (b @ axes.T).min(axis=0)
    bmax = (b @ axes.T).max(axis=0)
    return {"axes": axes, "amin": amin, "amax": amax, "bmin": bmin, "bmax": bmax}


def minimum_translation(data: dict[str, Any], extra_gap: float = 0.0) -> tuple[float, np.ndarray]:
    axes = data["axes"]
    # Move B to either side of A; choose the smaller valid translation.
    positive = data["amax"] - data["bmin"] + extra_gap
    negative = data["bmax"] - data["amin"] + extra_gap
    choose_positive = positive <= negative
    distances = np.where(choose_positive, positive, negative)
    index = int(np.argmin(distances))
    direction = axes[index] if choose_positive[index] else -axes[index]
    return float(distances[index]), direction


def directional_translation(data: dict[str, Any], direction: np.ndarray,
                            extra_gap: float = 0.0) -> float | None:
    direction = direction / np.linalg.norm(direction)
    dots = data["axes"] @ direction
    candidates: list[float] = []
    positive = dots > 1e-8
    candidates.extend(((data["amax"][positive] - data["bmin"][positive] + extra_gap)
                       / dots[positive]).tolist())
    negative = dots < -1e-8
    candidates.extend(((data["bmax"][negative] - data["amin"][negative] + extra_gap)
                       / -dots[negative]).tolist())
    positive_candidates = [value for value in candidates if value >= 0]
    return min(positive_candidates) if positive_candidates else None


def launcher_local(vertices: np.ndarray) -> np.ndarray:
    # Inverse of shifted_local([-100]) + [560,0,215] + Ry(-20) + Rx(90)
    p = vertices - np.asarray([460.0, 0.0, 215.0])
    angle_y = math.radians(20.0)
    ry_inv = np.asarray([
        [math.cos(angle_y), 0, math.sin(angle_y)],
        [0, 1, 0],
        [-math.sin(angle_y), 0, math.cos(angle_y)],
    ])
    p = p @ ry_inv.T
    rx_inv = np.asarray([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    p = p @ rx_inv.T
    p += np.asarray([0.0, 0.0, 380.0])
    return p


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--coarse-only", action="store_true")
    args = parser.parse_args()

    result: dict[str, Any] = {
        "source": str(SOURCE.relative_to(ROOT)),
        "units": "mm",
        "status": "ANALYSIS_ONLY",
        "authoritative_geometry_modified": False,
    }
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="corrected-resolution-", dir=runtime) as temp:
        directory = Path(temp)

        # Exact shaped local reliefs.
        specs: list[tuple[str, dict[str, Any]]] = []
        for gap in (0, 2, 3):
            specs.append((f"bridge_motor_{gap}", dict(
                part="relief_intersection", component="motor", target="bridge",
                selected_side=0, clearance=gap)))
        for gap in (0, 2):
            for target in ("basket", "hood"):
                specs.append((f"pocket_{target}_{gap}", dict(
                    part="relief_intersection", component="wheel", target=target,
                    selected_side=0, clearance=gap)))
            specs.append((f"pocket_extension_{gap}", dict(
                part="pocket_extension", component="wheel", selected_side=0,
                clearance=gap)))
            for component in ("motor", "direct_drive_hardware"):
                specs.append((f"cradle_{component}_{gap}", dict(
                    part="cradle_relief_intersection", component=component,
                    selected_side=0, clearance=gap)))
                for plate in (-1, 1):
                    specs.append((f"plate_{plate}_{component}_{gap}", dict(
                        part="plate_relief_intersection", component=component,
                        selected_side=0, selected_plate=plate, clearance=gap)))
        reliefs = exact_many(args.openscad, directory, specs)
        result["reliefs"] = reliefs

        # Local plate-coordinate bounds identify which physical plate is cut.
        plate_local: dict[str, Any] = {}
        for name in ("plate_-1_motor_0", "plate_1_motor_0",
                     "plate_-1_motor_2", "plate_1_motor_2"):
            path = render(args.openscad, directory, f"local_{name}",
                          part="plate_relief_intersection", component="motor",
                          selected_side=0, selected_plate=-1 if "-1" in name else 1,
                          clearance=2 if name.endswith("_2") else 0)
            if path is None:
                plate_local[name] = None
            else:
                plate_local[name] = bbox(launcher_local(stl_vertices(path)))
        result["cradle_relief_launcher_local_bbox_mm"] = plate_local

        # Convex-polyhedron SAT supplies the shortest separating translation
        # for each same-side swept motor / purchased flywheel pair.
        pair_results: dict[str, Any] = {}
        for side, label, flywheel in ((1, "left", "left_flywheel"),
                                      (-1, "right", "right_flywheel")):
            motor_path = render(args.openscad, directory, f"sat_motor_{label}",
                                part="swept_component", component="motor",
                                selected_side=side)
            fly_path = render(args.openscad, directory, f"sat_flywheel_{label}",
                              part="launcher_component", launcher_component=flywheel)
            assert motor_path and fly_path
            motor = stl_vertices(motor_path)
            flywheel_vertices = stl_vertices(fly_path)
            data = sat_data(motor, flywheel_vertices)
            axes: dict[str, list[float]] = {}
            directions = {
                "+X": np.asarray([1., 0., 0.]), "-X": np.asarray([-1., 0., 0.]),
                "+Y": np.asarray([0., 1., 0.]), "-Y": np.asarray([0., -1., 0.]),
                "+Z": np.asarray([0., 0., 1.]), "-Z": np.asarray([0., 0., -1.]),
            }
            for name, direction in directions.items():
                axes[name] = [round(directional_translation(data, direction, gap) or 0, 3)
                              for gap in (0, 2)]
            minimum: dict[str, Any] = {}
            for gap in (0, 2):
                distance, direction = minimum_translation(data, gap)
                vector = direction * distance
                minimum[str(gap)] = {
                    "distance_mm": round(distance, 3),
                    "launcher_translation_vector_mm": np.round(vector, 3).tolist(),
                    "unit_direction": np.round(direction, 6).tolist(),
                }
            pair_results[label] = {
                "motor_swept_bbox_mm": bbox(motor),
                "flywheel_bbox_mm": bbox(flywheel_vertices),
                "shortest": minimum,
                "axis_translation_to_separate_mm_zero_then_2mm": axes,
            }
        result["motor_same_side_flywheel_separation"] = pair_results

        # Parameterized exact-CGAL global rigid-translation search. The hard
        # probe includes both flywheels and all non-relievable protected fixed
        # geometry after the permitted bridge/pocket relief envelopes.
        coarse_specs: list[tuple[str, dict[str, Any]]] = []
        for dx in range(-40, 41, 10):
            for dz in range(-40, 41, 10):
                name = f"coarse_x{dx:+d}_z{dz:+d}"
                coarse_specs.append((name, dict(
                    part="hard_conflict_probe", component="direct_drive_hardware",
                    selected_side=0, launcher_dx=dx, launcher_dy=0,
                    launcher_dz=dz, clearance=2, pocket_clearance=2)))
        coarse = exact_many(args.openscad, directory, coarse_specs)
        viable_coarse = [name for name, record in coarse.items()
                         if record["intersection_volume_mm3"] == 0]
        result["translation_search"] = {
            "coarse_grid_mm": {"x": [-40, 40, 10], "y": [0], "z": [-40, 40, 10]},
            "coarse_records": coarse,
            "coarse_viable": viable_coarse,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        if args.coarse_only:
            print(json.dumps({"coarse_viable": viable_coarse}, indent=2))
            return 0

        # Refine every 1 mm around viable coarse cells, or extend +X if the
        # initial range has no solution.
        refine_points: set[tuple[int, int]] = set()
        if viable_coarse:
            def coarse_point(name: str) -> tuple[int, int]:
                parts = name.replace("coarse_x", "").split("_z")
                return int(parts[0]), int(parts[1])
            cx, cz = min((coarse_point(name) for name in viable_coarse),
                         key=lambda point: (math.hypot(*point), abs(point[1])))
            for dx in range(max(-40, cx - 10), min(40, cx + 10) + 1):
                for dz in range(max(-40, cz - 10), min(40, cz + 10) + 1):
                    refine_points.add((dx, dz))
        else:
            # Bounded coarse extension, justified only if the requested range
            # contains no combined-candidate solution.
            for dx in range(45, 81, 5):
                for dz in range(-40, 41, 5):
                    refine_points.add((dx, dz))
        refine_specs = [(f"refine_x{dx:+d}_z{dz:+d}", dict(
            part="hard_conflict_probe", component="direct_drive_hardware",
            selected_side=0, launcher_dx=dx, launcher_dy=0, launcher_dz=dz,
            clearance=2, pocket_clearance=2)) for dx, dz in sorted(refine_points)]
        refined = exact_many(args.openscad, directory, refine_specs)
        viable_refined = []
        for (dx, dz), (name, _) in zip(sorted(refine_points), refine_specs):
            if refined[name]["intersection_volume_mm3"] == 0:
                viable_refined.append((dx, 0, dz))
        result["translation_search"]["refined_count"] = len(refined)
        result["translation_search"]["refined_viable"] = viable_refined
        if viable_refined:
            result["translation_search"]["minimum_norm_viable"] = min(
                viable_refined, key=lambda point: (math.dist((0, 0, 0), point),
                                                    abs(point[2]), abs(point[0])))

        # Exact decomposed matrices for the minimum-norm and useful practical
        # candidates (the latter adds 2 mm in +X where available).
        selected_candidates: list[tuple[int, int, int]] = []
        if viable_refined:
            minimum_point = result["translation_search"]["minimum_norm_viable"]
            selected_candidates.append(tuple(minimum_point))
            practical = (minimum_point[0] + 2, 0, minimum_point[2])
            if practical in viable_refined:
                selected_candidates.append(practical)
            robust = (minimum_point[0] + 5, 0, minimum_point[2])
            if robust in viable_refined:
                selected_candidates.append(robust)
        matrices: dict[str, Any] = {}
        for dx, dy, dz in dict.fromkeys(selected_candidates):
            candidate_name = f"({dx},{dy},{dz})"
            candidate_specs: list[tuple[str, dict[str, Any]]] = []
            for moving in ("motor", "wheel", "direct_drive_hardware"):
                for launcher_part in ("cradle", "left_flywheel", "right_flywheel"):
                    candidate_specs.append((f"{moving}|{launcher_part}", dict(
                        part="swept_launcher_intersection", component=moving,
                        launcher_component=launcher_part, selected_side=0,
                        launcher_dx=dx, launcher_dy=dy, launcher_dz=dz)))
            for launcher_part in ("cradle", "left_flywheel", "right_flywheel", "assembly"):
                for obstacle in ("basket", "hood", "bridge", "chassis", "battery", "lidar"):
                    candidate_specs.append((f"{launcher_part}|{obstacle}", dict(
                        part="launcher_environment_intersection",
                        launcher_component=launcher_part, target=obstacle,
                        launcher_dx=dx, launcher_dy=dy, launcher_dz=dz)))
            matrices[candidate_name] = exact_many(args.openscad, directory, candidate_specs)
        result["candidate_exact_intersections"] = matrices

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "coarse_viable": result["translation_search"]["coarse_viable"],
        "refined_viable_count": len(result["translation_search"].get("refined_viable", [])),
        "minimum_norm_viable": result["translation_search"].get("minimum_norm_viable"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
