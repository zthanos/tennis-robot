#!/usr/bin/env python3
"""Regression evaluator for the direct-drive flywheel mechanical Gate A."""

from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "config/flywheel_launcher_direct_drive_mechanical_gate.json"
XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro"
CAD = ROOT / "cad/flywheel-launcher-v0/direct-drive-mechanical-definition-study.scad"


def close(actual: float, expected: float, tol: float = 1e-9) -> bool:
    return math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def evaluate() -> list[str]:
    data = json.loads(RESULT.read_text(encoding="utf-8"))
    failures: list[str] = []
    classes = data["classifications"]
    expected = {
        "D5065_DIRECT_PANEL_MOUNT_GEOMETRICALLY_VALID": True,
        "D5065_DIRECT_PANEL_MOUNT_STRUCTURALLY_SCREENED": True,
        "FLYWHEEL_PANEL_CUTOUT_DEFINED": False,
        "FLYWHEEL_DIRECT_DRIVE_HUB_DEFINED": False,
        "FLYWHEEL_SHAFT_ENGAGEMENT_VALIDATED": False,
        "FLYWHEEL_AXIAL_RETENTION_DEFINED": False,
        "FLYWHEEL_ROTATING_MASS_DEFINED": False,
        "FLYWHEEL_ROTATING_INERTIA_DEFINED": False,
        "FLYWHEEL_MECHANICAL_GATE_A_PASSED": False,
    }
    if classes != expected:
        failures.append("classification map changed")

    decision = data["decision"]
    if decision["separate_motor_bracket_required"]:
        failures.append("direct-panel architecture regressed to a separate motor bracket")
    if decision["independent_motor_pitch_hardware_required"]:
        failures.append("module pitch regressed to independent motor pitch hardware")
    if decision["launcher_trials_run"]:
        failures.append("launcher trials must remain blocked while Gate A is false")

    stack = data["axial_stack_launcher_local_z_m"]
    expected_stack = {
        "motor_mounting_face": 0.047,
        "panel_inside_face": 0.039,
        "shaft_tip": 0.017,
        "flywheel_outer_face": 0.025,
        "shaft_projection_beyond_panel_inside_m": 0.022,
        "shaft_projection_inside_wheel_envelope_m": 0.008,
        "panel_inside_to_flywheel_outer_gap_m": 0.014,
    }
    for key, value in expected_stack.items():
        if not close(stack[key], value):
            failures.append(f"axial stack {key} is {stack[key]!r}, expected {value}")
    for key in ("hub_start", "hub_end", "axial_retention_hardware"):
        if stack[key] is not None:
            failures.append(f"unresolved axial datum {key} was invented")

    datums = data["authoritative_launcher_datums"]
    if datums != {
        "flywheel_diameter_m": 0.2,
        "flywheel_width_m": 0.05,
        "wheel_centre_spacing_m": 0.258,
        "nip_m": 0.058,
        "cradle_plate_size_m": [0.256, 0.314, 0.008],
        "launcher_pitch_deg": 20.0,
    }:
        failures.append("accepted launcher datums changed")

    xacro = XACRO.read_text(encoding="utf-8")
    if "wheel_mass:=0.40" not in xacro:
        failures.append("standalone Xacro provisional wheel mass changed before Gate A")
    wheel_macro = (ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher.urdf.xacro").read_text(encoding="utf-8")
    if '<xacro:cylinder_inertial mass="${mass}" radius="${radius}" length="${width}" axis="z"/>' not in wheel_macro:
        failures.append("standalone provisional solid-cylinder inertia law was not found")

    cad = CAD.read_text(encoding="utf-8")
    for token in ("motor_face_z = 47", "analysis_cutout_d = 18", "unresolved_hub_zone"):
        if token not in cad:
            failures.append(f"analysis CAD missing marker: {token}")
    return failures


def main() -> int:
    failures = evaluate()
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS: direct-panel geometry is screened and Gate A remains correctly stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
