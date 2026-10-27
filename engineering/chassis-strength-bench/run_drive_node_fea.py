#!/usr/bin/env python3
"""Mesh and solve the local drive-node submodel with Gmsh + CalculiX."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
SNAP_ROOT = Path("/snap/freecad/current")
GMSH = SNAP_ROOT / "usr/bin/gmsh"
CCX = SNAP_ROOT / "usr/bin/ccx"


def _runtime_env() -> dict[str, str]:
    env = os.environ.copy()
    libraries = [
        SNAP_ROOT / "usr/lib",
        SNAP_ROOT / "usr/lib/x86_64-linux-gnu",
        SNAP_ROOT / "usr/lib/x86_64-linux-gnu/blas",
        SNAP_ROOT / "usr/lib/x86_64-linux-gnu/lapack",
    ]
    env["LD_LIBRARY_PATH"] = ":".join(map(str, libraries))
    return env


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _mesh(work: Path) -> Path:
    mesh = work / "drive_node_mesh.inp"
    subprocess.run(
        [
            str(GMSH),
            str(ROOT / "fea" / "drive_node.geo"),
            "-3",
            "-format",
            "inp",
            "-o",
            str(mesh),
            "-v",
            "2",
        ],
        check=True,
        env=_runtime_env(),
    )
    return mesh


def _read_nodes(mesh: Path) -> dict[int, tuple[float, float, float]]:
    nodes: dict[int, tuple[float, float, float]] = {}
    active = False
    for line in mesh.read_text(encoding="utf-8").splitlines():
        if line.upper().startswith("*NODE"):
            active = True
            continue
        if line.startswith("*"):
            if active:
                break
            continue
        if active:
            fields = [field.strip() for field in line.split(",")]
            nodes[int(fields[0])] = tuple(float(value) for value in fields[1:4])
    if not nodes:
        raise RuntimeError("Gmsh mesh contains no nodes")
    return nodes


def _node_set(name: str, node_ids: list[int]) -> str:
    if not node_ids:
        raise RuntimeError(f"Node set {name} is empty")
    lines = [f"*NSET,NSET={name}"]
    for start in range(0, len(node_ids), 16):
        lines.append(",".join(str(node) for node in node_ids[start : start + 16]))
    return "\n".join(lines)


def _select_nodes(nodes: dict[int, tuple[float, float, float]]) -> dict[str, list[int]]:
    fixed = [node for node, (x, _y, _z) in nodes.items() if abs(abs(x) - 0.080) < 1e-7]
    rows: dict[str, list[int]] = {"ROW_NEG_X": [], "ROW_POS_X": []}
    all_bolts: list[int] = []
    for node, (x, y, z) in nodes.items():
        if not 0.0219 <= z <= 0.0301:
            continue
        for centre_x, row_name in ((-0.015, "ROW_NEG_X"), (0.015, "ROW_POS_X")):
            for centre_y in (-0.014, 0.011):
                radius = math.hypot(x - centre_x, y - centre_y)
                # Select the cylindrical hole wall plus its top rim. Avoid a
                # single top-surface node carrying a point singularity.
                if abs(radius - 0.0017) <= 0.00018:
                    rows[row_name].append(node)
                    all_bolts.append(node)
                    break
    return {
        "FIXED": sorted(set(fixed)),
        "ROW_NEG_X": sorted(set(rows["ROW_NEG_X"])),
        "ROW_POS_X": sorted(set(rows["ROW_POS_X"])),
        "LOAD_NODES": sorted(set(all_bolts)),
    }


def _write_deck(
    work: Path,
    mesh: Path,
    name: str,
    material: dict[str, Any],
    node_sets: dict[str, list[int]],
    peak_force_n: float,
    peak_torque_nm: float,
) -> Path:
    deck = work / f"drive_node_{name}.inp"
    neg_nodes = node_sets["ROW_NEG_X"]
    pos_nodes = node_sets["ROW_POS_X"]
    load_nodes = node_sets["LOAD_NODES"]
    # Equal/opposite longitudinal rows create torque about Y over 30 mm.
    row_force = peak_torque_nm / 0.030
    neg_fz = row_force / len(neg_nodes)
    pos_fz = -row_force / len(pos_nodes)
    fx = peak_force_n / len(load_nodes)
    loads = []
    for node in load_nodes:
        loads.append(f"{node},1,{fx:.12g}")
    for node in neg_nodes:
        loads.append(f"{node},3,{neg_fz:.12g}")
    for node in pos_nodes:
        loads.append(f"{node},3,{pos_fz:.12g}")

    content = [
        f"*INCLUDE,INPUT={mesh.name}",
        *(_node_set(set_name, ids) for set_name, ids in node_sets.items()),
        f"*MATERIAL,NAME={name}",
        "*ELASTIC",
        f"{material['reference_elastic_modulus_gpa'] * 1e9:.12g},{material['poisson_ratio']:.6g}",
        f"*SOLID SECTION,ELSET=SOLID,MATERIAL={name}",
        "*BOUNDARY",
        "FIXED,1,3,0",
        "*STEP",
        "*STATIC",
        "0.1,1.0",
        "*CLOAD",
        *loads,
        "*EL PRINT,ELSET=SOLID",
        "S",
        "*NODE PRINT,NSET=LOAD_NODES",
        "U",
        "*END STEP",
    ]
    deck.write_text("\n".join(content) + "\n", encoding="utf-8")
    return deck


STRESS_LINE = re.compile(
    r"^\s*(\d+)\s+(\d+)\s+"
    r"([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s+"
    r"([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s*$"
)


def _parse_results(work: Path, stem: str) -> dict[str, float]:
    dat = work / f"{stem}.dat"
    maximum_vm = 0.0
    maximum_component = 0.0
    stress_rows = 0
    for line in dat.read_text(encoding="utf-8", errors="replace").splitlines():
        match = STRESS_LINE.match(line)
        if not match:
            continue
        values = [float(value) for value in match.groups()[2:]]
        sxx, syy, szz, sxy, sxz, syz = values
        vm = math.sqrt(
            0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
            + 3.0 * (sxy**2 + sxz**2 + syz**2)
        )
        maximum_vm = max(maximum_vm, vm)
        maximum_component = max(maximum_component, *(abs(value) for value in values))
        stress_rows += 1
    if stress_rows == 0:
        raise RuntimeError(f"No element stress rows found in {dat}")
    return {
        "stress_rows": stress_rows,
        "maximum_von_mises_pa": maximum_vm,
        "maximum_stress_component_pa": maximum_component,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--materials", type=Path, default=ROOT / "config" / "materials.json"
    )
    parser.add_argument(
        "--config", type=Path, default=ROOT / "config" / "baseline.json"
    )
    parser.add_argument(
        "--wrench-summary",
        type=Path,
        default=ROOT / "results" / "jazzy_harmonic" / "wrench_summary.json",
    )
    parser.add_argument("--work", type=Path, default=ROOT / "fea" / "work")
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results" / "drive_node_fea.json"
    )
    args = parser.parse_args()
    args.work.mkdir(parents=True, exist_ok=True)

    materials = _load(args.materials)["materials"]
    config = _load(args.config)
    wrench = _load(args.wrench_summary)["global"]
    lateral_offset_m = (
        config["geometry"]["wheel_contact_to_rail_centroid_y_mm"] / 1000.0
    )
    motor_gravity_moment_nm = (
        config["drivetrain"]["motor_mass_kg"]
        * 9.80665
        * config["drivetrain"]["motor_cg_offset_from_rail_mm"]
        / 1000.0
    )
    transferred_mount_moment_nm = (
        wrench["maximum_torque_norm_nm"]
        + wrench["maximum_force_norm_n"] * lateral_offset_m
        + motor_gravity_moment_nm
    )
    mesh = _mesh(args.work)
    nodes = _read_nodes(mesh)
    node_sets = _select_nodes(nodes)
    report: dict[str, Any] = {
        "schema_version": 1,
        "model": "local isotropic solid drive-node submodel",
        "solver": "CalculiX 2.21",
        "mesher": "Gmsh 4.13.1",
        "loads": {
            **wrench,
            "wheel_to_rail_lateral_offset_m": lateral_offset_m,
            "motor_gravity_moment_nm": motor_gravity_moment_nm,
            "transferred_mount_design_moment_nm": transferred_mount_moment_nm,
            "transfer_rule": "M_mount = |M_joint| + offset*|F_joint| + motor_weight*motor_CG_offset",
        },
        "node_set_counts": {name: len(ids) for name, ids in node_sets.items()},
        "materials": {},
        "limitations": [
            "first-order tetrahedra are a screening mesh; second-order mesh convergence is required",
            "isotropic material model; printed-layer orthotropy is not represented",
            "fixed cut faces make this a local submodel",
            "bolt loads are distributed on hole walls; no washer/contact/preload model",
            "peak vector norms combine components conservatively",
        ],
    }
    for name, material in materials.items():
        solver_name = name.replace("-", "_")
        deck = _write_deck(
            args.work,
            mesh,
            solver_name,
            material,
            node_sets,
            wrench["maximum_force_norm_n"],
            transferred_mount_moment_nm,
        )
        stem = deck.stem
        completed = subprocess.run(
            [str(CCX), "-i", stem],
            cwd=args.work,
            env=_runtime_env(),
            text=True,
            capture_output=True,
        )
        (args.work / f"{stem}.solver.log").write_text(
            completed.stdout + completed.stderr, encoding="utf-8"
        )
        if completed.returncode != 0:
            raise RuntimeError(f"CalculiX failed for {name}: see {stem}.solver.log")
        result = _parse_results(args.work, stem)
        result["maximum_von_mises_mpa"] = result["maximum_von_mises_pa"] / 1e6
        result["screen_safety_factor"] = (
            material["screen_strength_mpa"] / result["maximum_von_mises_mpa"]
        )
        report["materials"][name] = result

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
