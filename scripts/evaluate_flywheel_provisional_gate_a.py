#!/usr/bin/env python3
"""Validate the provisional standalone flywheel mechanical Gate A baseline."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GATE = ROOT / "config/flywheel_launcher_provisional_gate_a.json"
CSV_PATH = ROOT / "docs/mechanism/flywheel-wheel-candidate-capability-screen.csv"
BALL = ROOT / "config/tennis_ball_compliance_calibration_results.json"
MODULE = ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro"
BENCH = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/flywheel_launcher_bench_controllers.yaml"


def evaluate() -> list[str]:
    data = json.loads(GATE.read_text(encoding="utf-8"))
    failures: list[str] = []
    expected_classes = {
        "D5065_DIRECT_PANEL_MOUNT_GEOMETRICALLY_VALID": True,
        "D5065_DIRECT_PANEL_MOUNT_STRUCTURALLY_SCREENED": True,
        "FLYWHEEL_PANEL_CUTOUT_DEFINED": True,
        "FLYWHEEL_DIRECT_DRIVE_HUB_DEFINED_FOR_SIMULATION": True,
        "FLYWHEEL_SHAFT_ENGAGEMENT_VALIDATED_FOR_SIMULATION": True,
        "FLYWHEEL_AXIAL_RETENTION_DEFINED_FOR_SIMULATION": True,
        "FLYWHEEL_ROTATING_MASS_BOUNDED": True,
        "FLYWHEEL_ROTATING_INERTIA_BOUNDED": True,
        "FLYWHEEL_MECHANICAL_GATE_A_SIMULATION_READY": True,
        "FLYWHEEL_MECHANICAL_GATE_A_PHYSICAL_VALIDATED": False,
    }
    if data["classifications"] != expected_classes:
        failures.append("Gate A classification map changed")

    wheel = data["wheel_candidate"]
    if wheel["mass_range_kg"] != [0.70, 0.90] or wheel["nominal_simulation_mass_kg"] != 0.90:
        failures.append("wheel mass baseline changed")
    if (wheel["diameter_m"], wheel["width_m"], wheel["bore_diameter_m"]) != (0.2, 0.05, 0.01):
        failures.append("wheel product-spec geometry changed")

    hub = data["hub"]
    if not math.isclose(hub["shaft_engagement_m"], 0.0215, abs_tol=1e-12):
        failures.append("shaft engagement changed")
    if hub["screened_wheel_interface_torque_nm"] < hub["required_peak_motor_torque_nm"]:
        failures.append("hub torque screen no longer covers motor peak")
    cutout = data["panel_cutout"]
    if cutout["diameter_m"] != 0.012 or not math.isclose(cutout["mount_hole_edge_ligament_m"], 0.007, abs_tol=1e-12):
        failures.append("panel cutout or ligament changed")

    rotating = data["rotating_mass_and_inertia"]
    if rotating["wheel_inertia_overall_range_kg_m2"] != [0.0035, 0.0090]:
        failures.append("wheel inertia bracket changed")
    if rotating["motor_rotor_inertia_manufacturer_value"] is not None:
        failures.append("unproven motor rotor inertia was promoted")

    if data["calibrated_ball_event_surrogate"]["source_sha256"] != hashlib.sha256(BALL.read_bytes()).hexdigest():
        failures.append("calibrated ball reference changed")
    if data["calibrated_ball_event_surrogate"]["is_launcher_event"]:
        failures.append("rebound surrogate was mislabeled as a launcher event")

    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 30:
        failures.append(f"expected 30 capability sensitivity rows, found {len(rows)}")
    if {int(row["target_rpm"]) for row in rows} != {1000, 1250, 1500, 1750, 2000}:
        failures.append("required RPM points are incomplete")

    module = MODULE.read_text(encoding="utf-8")
    for token in (
        "wheel_mass:=0.90",
        "hub_mass:=0.0267636462761",
        "spin_inertia:=0.00675116210829",
        "flywheel_lower_panel_exit_clearance.stl",
        "flywheel_upper_panel_exit_clearance.stl",
        "d5065_motor_left_col",
        "hub_collar_col",
        "retainer_col",
        'effort:=0.62',
    ):
        if token not in module:
            failures.append(f"standalone Xacro missing {token}")
    bench = BENCH.read_text(encoding="utf-8")
    controllers = CONTROLLERS.read_text(encoding="utf-8")
    if '<command_interface name="effort">' not in bench or '<command_interface name="velocity">' in bench:
        failures.append("standalone bench still exposes ideal velocity command")
    if "flywheel_effort_controller" not in controllers or "interface_name: effort" not in controllers:
        failures.append("standalone effort controller is not configured")
    return failures


def main() -> int:
    failures = evaluate()
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS: provisional standalone mechanical Gate A is simulation-ready; physical validation remains false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
