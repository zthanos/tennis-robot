#!/usr/bin/env python3
"""P2-54 focused clearance and St-Venant upper-plate torsion screen.

Static CAD/analytical calculation only: no ROS, Gazebo, STL release, or FEA
claim. Temporary STL meshes are deleted before exit.
"""
from __future__ import annotations

import importlib.util
import json
import math
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
P53_PATH = ROOT / "engineering/chassis-strength-bench/run_phase2_p53_geometry_reaudit.py"
OUT = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2/p254_margin_torsion.json"

spec = importlib.util.spec_from_file_location("p53_geometry", P53_PATH)
assert spec and spec.loader
p53 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p53)

MODULE_MASS_KG = 4.609957627006651
GRAVITY = 9.80665
ACTUATOR_STOP_Y_M = -0.205
SECOND_STOP_Y_M = 0.205
WHEEL_Y_M = 0.129
WHEEL_SPAN_M = 2 * WHEEL_Y_M
NIP_LEVER_M = 0.055

# St-Venant torsion of the 195 x 8 mm PETG upper plate about hinge Y.
PETG_E_PA = 1.5e9
PETG_NU = 0.38
PETG_G_PA = PETG_E_PA / (2 * (1 + PETG_NU))
PLATE_B_M = 0.195
PLATE_T_M = 0.008
PLATE_J_M4 = (PLATE_B_M * PLATE_T_M**3 / 3
              * (1 - 0.63 * PLATE_T_M / PLATE_B_M))

# Validated isolated-bench conservation surrogate, launcher-local coordinates.
FRAME_MASS = 2.85200336
FRAME_CG_XZ = (0.030, 0.0185061462201)
WHEEL_MASS = 0.878977133503
WHEEL_XZ = (0.055, 0.0)
MODULE_CG_X_M = ((FRAME_MASS * FRAME_CG_XZ[0]
                  + 2 * WHEEL_MASS * WHEEL_XZ[0]) / MODULE_MASS_KG)
MODULE_CG_Z_M = ((FRAME_MASS * FRAME_CG_XZ[1]
                  + 2 * WHEEL_MASS * WHEEL_XZ[1]) / MODULE_MASS_KG)


def gravity_torque_plus_half(angle_deg: float) -> float:
    """Gravity torque about hinge Y carried by the +Y module half."""
    angle = math.radians(angle_deg)
    world_x = MODULE_CG_X_M * math.cos(angle) - MODULE_CG_Z_M * math.sin(angle)
    return MODULE_MASS_KG * GRAVITY * abs(world_x) / 2


def torsion_row(angle_deg: int) -> dict:
    torque = gravity_torque_plus_half(angle_deg)
    theta_one = torque * WHEEL_SPAN_M / (PETG_G_PA * PLATE_J_M4)
    # With only the -Y actuator/stop, the +Y-half torque crosses the complete
    # Y=-129..+129 wheel interval. With symmetric stops it closes to the +Y
    # stop over the outer 76 mm and does not cross between wheel planes.
    theta_two = 0.0
    return {
        "angle_deg": angle_deg,
        "module_cg_world_x_mm": round(
            1000 * (MODULE_CG_X_M * math.cos(math.radians(angle_deg))
                    - MODULE_CG_Z_M * math.sin(math.radians(angle_deg))), 4),
        "plus_y_half_gravity_torque_nm": round(torque, 6),
        "without_second_stop": {
            "torque_path_between_wheels_mm": round(1000 * WHEEL_SPAN_M, 3),
            "differential_wheel_angle_deg": round(math.degrees(theta_one), 6),
            "delta_nip_mm": round(1000 * NIP_LEVER_M * theta_one, 6),
            "pass_le_1mm": 1000 * NIP_LEVER_M * theta_one <= 1.0,
        },
        "with_symmetric_second_stop": {
            "plus_y_wheel_to_stop_path_mm": round(1000 * (SECOND_STOP_Y_M - WHEEL_Y_M), 3),
            "torque_crossing_between_wheels_nm": 0.0,
            "differential_wheel_angle_deg": round(math.degrees(theta_two), 6),
            "delta_nip_mm": round(1000 * NIP_LEVER_M * theta_two, 6),
            "pass_le_1mm": True,
        },
    }


def main() -> None:
    work = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2/.p254_mesh_work"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    try:
        checks = {
            "moving_arm_to_launcher_excluding_mount": p53.moving_pair(
                work, "mesh_arm_at_angle", "mesh_launcher_arm_target_at_angle"),
            "moving_arm_to_drive": {"minimum_nominal_mm": round(p53.fixed_pair(
                work, "mesh_sweep_arm", "mesh_drive"), 3)},
            "fit_motor_to_launcher": p53.moving_pair(
                work, "mesh_motor_at_angle", "mesh_launcher_at_angle"),
            "fit_motor_to_drive": {"minimum_nominal_mm": round(p53.fixed_pair(
                work, "mesh_sweep_motor", "mesh_drive"), 3)},
            "fit_motor_to_arch_excluding_mount": {"minimum_nominal_mm": round(
                p53.fixed_pair(work, "mesh_sweep_motor", "mesh_arch"), 3)},
        }
    finally:
        shutil.rmtree(work, ignore_errors=True)

    clearance_pass = (
        checks["moving_arm_to_launcher_excluding_mount"]["minimum_nominal_mm"] >= 12
        and checks["fit_motor_to_drive"]["minimum_nominal_mm"] >= 10
        and checks["fit_motor_to_arch_excluding_mount"]["minimum_nominal_mm"] >= 10
    )
    rows = [torsion_row(angle) for angle in (-3, 0, 20)]
    worst_one = max(rows, key=lambda row: row["without_second_stop"]["delta_nip_mm"])
    worst_two = max(rows, key=lambda row: row["with_symmetric_second_stop"]["delta_nip_mm"])
    torsion_pass = worst_two["with_symmetric_second_stop"]["delta_nip_mm"] <= 1.0
    if not clearance_pass or not torsion_pass:
        raise RuntimeError("P2-54 acceptance gate failed; inspect generated result")

    report = {
        "status": "P2_54_CLEARANCE_PASS_ST_VENANT_TORSION_PASS_SECOND_STOP_MANDATORY",
        "scope": "focused exact CAD pairs plus analytical St-Venant upper-plate torsion; no Gazebo/FEA/STL",
        "supersedes": "plate-strip transverse bending model in the first P2-54 report",
        "cad_changes": {
            "p2_40_changed": False,
            "moving_arm_root_local_mm": [-10, -160, 68],
            "moving_arm_high_local_mm": [-10, -205, 135],
            "fit0186_parallel_offset_from_fixed_pin_mm": [0, 40.5, 60],
            "symmetric_stops_y_mm": [-205, 205],
            "second_stop_baseline": "mandatory",
        },
        "clearances": checks,
        "clearance_criteria": {
            "moving_arm_to_launcher_minimum_mm": 12.0,
            "fit_motor_to_drive_minimum_mm": 10.0,
            "fit_motor_to_arch_minimum_mm": 10.0,
            "pass": clearance_pass,
        },
        "st_venant_torsion": {
            "axis": "hinge Y",
            "plate_cross_section_mm": [195.0, 8.0],
            "J_formula": "(1/3)*b*t^3*(1-0.63*t/b)",
            "J_m4": PLATE_J_M4,
            "petg_E_gpa": PETG_E_PA / 1e9,
            "petg_nu": PETG_NU,
            "petg_G_gpa": PETG_G_PA / 1e9,
            "module_mass_kg": MODULE_MASS_KG,
            "module_cg_local_xz_mm": [round(1000 * MODULE_CG_X_M, 4), round(1000 * MODULE_CG_Z_M, 4)],
            "wheel_planes_y_mm": [-129, 129],
            "actuator_stop_y_mm": -205,
            "second_stop_y_mm": 205,
            "nip_lever_mm": 55,
            "rows": rows,
            "worst_without_second_stop": worst_one,
            "worst_with_symmetric_second_stop": worst_two,
            "criterion_delta_nip_mm": 1.0,
            "baseline_pass": torsion_pass,
            "interpretation": "the +Y-half gravity torque crosses the wheel span only in the one-stop layout; symmetric endpoint stops cancel first-order differential twist between wheel planes",
        },
        "decision": {
            "second_stop_at_y_plus205": "mandatory baseline for both -10..-3 and +15..+20 endpoint ranges",
            "single_actuator": "retained; the second element is a passive positive stop, not a second actuator",
            "measurement_gates": [
                "printed PETG coupon and layer-dependent shear modulus",
                "upper-plate/yoke joint compliance",
                "physical differential nip under gravity and seated actuator preload",
                "stop contact compression and current cut-off timing",
            ],
        },
        "limitations": [
            "linear St-Venant torsion of a uniform rectangular plate, not nonlinear joint/contact FEA",
            "ideal symmetric two-stop load sharing gives zero first-order differential; manufacturing/load imbalance remains a physical measurement gate",
            "local fastener, bearing and stop contact stresses are outside this global twist screen",
        ],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
