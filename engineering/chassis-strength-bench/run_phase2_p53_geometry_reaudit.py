#!/usr/bin/env python3
"""P2-53 exact-solid transition geometry audit; no ROS/Gazebo/STL release."""
from __future__ import annotations

import json
import math
import shutil
import subprocess
from pathlib import Path
import vtk

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "cad/modular-test-chassis/shared-flywheel-transition-p252.scad"
OPENSCAD = "/snap/bin/openscad-nightly"
OUT = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2/p253_geometry_reaudit.json"

PIVOT = (387.0, 0.0, 51.837363636)
FIXED = (310.0, -205.0, 120.0)
LOCAL = (50.0, -205.0, 90.0)
ANGLES = range(-10, 21)
ZERO_CHECKS = {
    "fit_motor_to_launcher": "qa_motor_launcher",
    "fit_motor_to_drive": "qa_motor_drive",
    "fit_motor_to_arch_excluding_mount": "qa_motor_arch",
    "tr8_to_launcher": "qa_tr8_launcher",
    "tr8_to_drive": "qa_tr8_drive",
    "moving_arm_to_launcher_excluding_mount": "qa_arm_launcher_forbidden",
    "moving_arm_to_drive": "qa_arm_drive",
    "whole_actuator_to_frame": "qa_actuator_frame",
    "whole_actuator_to_sensor_stack": "qa_actuator_sensors",
    "whole_actuator_to_basket_ski_feed": "qa_actuator_basket_ski",
    "whole_actuator_to_power_electronics": "qa_actuator_power",
    "as5600_to_launcher_excluding_trunnion_interface": "qa_as5600_launcher",
    "as5600_to_drive": "qa_as5600_drive",
    "cam_to_launcher": "qa_cam_launcher",
    "cam_to_frame_ski_feed": "qa_cam_fixed",
    "stops_to_launcher": "qa_stops_launcher",
    "cables_to_launcher": "qa_cables_launcher",
    "cables_to_fixed_world": "qa_cables_fixed",
}


def cad_empty(view: str, probe: float, target: Path) -> tuple[bool, str]:
    cmd = [OPENSCAD, "-o", str(target), "-D", f'p253_view="{view}"',
           "-D", f"clearance_probe_p253={probe}", str(CAD)]
    p = subprocess.run(cmd, text=True, capture_output=True, timeout=180)
    log = p.stdout + p.stderr
    # OpenSCAD returns rc=1 for an intentionally empty top-level CSG.  The
    # diagnostic string, not the return code, is the expected PASS signal.
    ok = "ERROR:" not in log
    return ok and "Current top level object is empty." in log, log


def export_mesh(view: str, target: Path, angle: int | None = None) -> None:
    cmd = [OPENSCAD, "-o", str(target), "-D", f'p253_view="{view}"']
    if angle is not None:
        cmd += ["-D", f"p253_angle={angle}"]
    cmd += [str(CAD)]
    p = subprocess.run(cmd, text=True, capture_output=True, timeout=180)
    log = p.stdout + p.stderr
    if p.returncode != 0 or "ERROR:" in log or not target.exists():
        raise RuntimeError(f"Mesh export failed: {view} angle={angle}: {log}")


def read_poly(path: Path):
    reader = vtk.vtkSTLReader(); reader.SetFileName(str(path)); reader.Update()
    tri = vtk.vtkTriangleFilter(); tri.SetInputData(reader.GetOutput()); tri.Update()
    return tri.GetOutput()


def vertex_surface_distance(a_path: Path, b_path: Path) -> float:
    a, b = read_poly(a_path), read_poly(b_path)
    def one(src, dst):
        implicit = vtk.vtkImplicitPolyDataDistance(); implicit.SetInput(dst)
        return min(abs(implicit.EvaluateFunction(src.GetPoint(i))) for i in range(src.GetNumberOfPoints()))
    return min(one(a, b), one(b, a))


def fixed_pair(work: Path, a_view: str, b_view: str) -> float:
    a, b = work/f"{a_view}.stl", work/f"{b_view}.stl"
    if not a.exists(): export_mesh(a_view, a)
    if not b.exists(): export_mesh(b_view, b)
    return vertex_surface_distance(a, b)


def moving_pair(work: Path, a_view: str, b_view: str) -> dict:
    best = (float("inf"), None)
    for angle in ANGLES:
        a = work/f"{a_view}_{angle}.stl"; b = work/f"{b_view}_{angle}.stl"
        export_mesh(a_view, a, angle); export_mesh(b_view, b, angle)
        d = vertex_surface_distance(a, b)
        if d < best[0]: best = (d, angle)
    return {"minimum_nominal_mm": round(best[0], 3), "governing_angle_deg": best[1]}


def moving(angle: float) -> tuple[float, float, float]:
    a = math.radians(angle)
    return (PIVOT[0] + LOCAL[0]*math.cos(a) - LOCAL[2]*math.sin(a), LOCAL[1],
            PIVOT[2] + LOCAL[0]*math.sin(a) + LOCAL[2]*math.cos(a))


def linkage(angle: float) -> dict:
    m = moving(angle)
    d = tuple(FIXED[i]-m[i] for i in range(3))
    length = math.sqrt(sum(x*x for x in d)); u = tuple(x/length for x in d)
    r = tuple(m[i]-PIVOT[i] for i in range(3))
    cross_y = r[2]*u[0]-r[0]*u[2]
    arm = abs(cross_y)/1000
    return {"angle_deg": angle, "pin_spacing_mm": round(length, 3),
            "effective_arm_mm": round(arm*1000, 3),
            "force_5p3Nm_N": round(5.3/arm, 2), "force_8Nm_N": round(8/arm, 2)}


def main() -> None:
    work = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2/.p253_mesh_work"
    if work.exists(): shutil.rmtree(work)
    work.mkdir(parents=True)
    zero = {}
    for name, view in ZERO_CHECKS.items():
        empty, log = cad_empty(view, 0, Path("/tmp")/f"p253_zero_{view}.stl")
        zero[name] = empty
        if not empty:
            raise RuntimeError(f"Nominal collision in {name}: {log}")
    clearances = {
        "fit_motor_to_launcher": moving_pair(work,"mesh_motor_at_angle","mesh_launcher_at_angle"),
        "tr8_to_launcher": moving_pair(work,"mesh_tr8_at_angle","mesh_launcher_at_angle"),
        "moving_arm_to_launcher_excluding_mount": moving_pair(work,"mesh_arm_at_angle","mesh_launcher_arm_target_at_angle"),
        "cam_to_launcher": moving_pair(work,"mesh_cam_at_angle","mesh_launcher_at_angle"),
        "cables_to_launcher": moving_pair(work,"mesh_cables_at_angle","mesh_launcher_at_angle"),
        "fit_motor_to_drive": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_motor","mesh_drive"),3)},
        "fit_motor_to_arch_excluding_mount": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_motor","mesh_arch"),3)},
        "tr8_to_drive": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_tr8","mesh_drive"),3)},
        "moving_arm_to_drive": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_arm","mesh_drive"),3)},
        "whole_actuator_to_frame": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_actuator","mesh_frame"),3)},
        "whole_actuator_to_sensor_stack": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_actuator","mesh_sensors"),3)},
        "whole_actuator_to_basket_ski_feed": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_actuator","mesh_basket_ski"),3)},
        "whole_actuator_to_power_electronics": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_actuator","mesh_power"),3)},
        "as5600_to_launcher_excluding_trunnion_interface": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_as5600","mesh_sweep_launcher_without_trunnion"),3)},
        "as5600_to_drive": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_as5600","mesh_drive"),3)},
        "cam_to_frame_ski_feed": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_cam","mesh_cam_fixed_target"),3)},
        "stops_to_launcher": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_stops","mesh_sweep_launcher"),3)},
        "cables_to_fixed_world": {"minimum_nominal_mm": round(fixed_pair(work,"mesh_sweep_cables","mesh_cables_fixed_target"),3)},
    }
    shutil.rmtree(work)
    rows = [linkage(a) for a in (-10, -3, 0, 15, 20)]
    length_minus3 = next(x["pin_spacing_mm"] for x in rows if x["angle_deg"] == -3)
    length_plus20 = next(x["pin_spacing_mm"] for x in rows if x["angle_deg"] == 20)
    report = {
        "status": "P2_53_EXACT_GEOMETRY_REAUDIT_PASS_WITH_ARM_FIT_GATE",
        "basis": {"angle_sweep_deg": [-10, 20, 1], "clearance_method": "exact-solid zero-intersection plus symmetric VTK vertex-to-triangulated-surface distance",
                  "reported_distance_uncertainty_mm": 0.5,
                  "frame_rails": "Y +/-220, width 50", "flywheels": "centres Y +/-129, D200x60, axes local Z",
                  "pivot_xyz_mm": PIVOT, "basket": "P2-51 exact panels/ski values", "no_stl_release": True},
        "superseded_p252_findings": [
            "drum-axis flywheels and Y +/-235 / 30 mm rails",
            "coaxial FIT0186 body falsely reported clear",
            "AS5600 at Y216 and straight moving arm",
            "all P2-52 clearances derived from the surrogate assembly",
        ],
        "corrections": {
            "fit0186": "same Tr8/linkage, but motor is offset +37 mm Y/+50 mm Z on the rocking carrier and drives Tr8 through a short 1:1 toothed belt",
            "moving_arm": "rises from upper-plate edge and stays above tread; old Z45 straight root removed",
            "stops": "mandatory symmetric tongues at local [20,+/-205,115], with fixed lower -10..-3 and upper +15..+20 stop ranges",
            "as5600": "board integrated into +Y guard at Y244; magnet Y240.5",
            "side_projection": "rotate -90 deg about X; +Z up, ground line, labels and FRONT +X arrow",
        },
        "linkage": {"fixed_xyz_mm": FIXED, "moving_local_xyz_mm": LOCAL, "rows": rows,
                    "travel_minus3_to_plus20_mm": round(length_minus3-length_plus20, 3),
                    "p16_full_range_mm": [min(x["pin_spacing_mm"] for x in rows), max(x["pin_spacing_mm"] for x in rows)]},
        "mandatory_symmetric_stops": {
            "y_mm": [-205, 205],
            "lower_range_deg": [-10, -3],
            "upper_range_deg": [15, 20],
            "second_actuator": False,
        },
        "reaudit_after_p254_arm_fit_and_second_stop": True,
        "zero_intersection_pass": zero,
        "clearances": clearances,
        "design_effects": {
            "fit_drive": "1:1 belt adds two pulleys, tensioner/guard and an assumed 0.90 efficiency; continuous motor torque screen must be divided by measured belt efficiency",
            "time": "1:1 ratio leaves ideal Tr8 speed unchanged; loaded time remains measurement-gated",
            "p16": "retains direct slim-body fallback on the Tr8/P16 axis; it does not use the FIT offset carrier",
        },
        "limitations": ["nominal exact solids; no print tolerances, belt deflection or cable bend stiffness",
                        "Parkside pack/adapter remains an assumed envelope", "no Gazebo, ROS, STL, Arduino or electronics changes"],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
