#!/usr/bin/env python3
"""Enforce the physical-completeness stop before launcher performance trials."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "docs/archive/mechanism/flywheel-launcher/config/flywheel_launcher_capability_gate_results.json"
BALL_RESULTS = ROOT / "config/tennis_ball_compliance_calibration_results.json"
MODULE_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro"
CAD_MODULE = ROOT / "cad/flywheel-launcher-v0/launcher-envelope.scad"
PROVISIONAL_GATE = ROOT / "config/flywheel_launcher_provisional_gate_a.json"
BENCH_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate() -> dict[str, object]:
    results = json.loads(RESULTS.read_text(encoding="utf-8"))
    xacro = MODULE_XACRO.read_text(encoding="utf-8").lower()
    cad = CAD_MODULE.read_text(encoding="utf-8").lower()
    ledger = {item["datum"]: item for item in results["mechanical_datum_ledger"]}

    required_missing = (
        "motor_mount_plate_or_bracket_geometry",
        "wheel_hub_geometry_and_material",
        "shaft_engagement_and_attachment_fasteners",
        "axial_retention_solution",
        "motor_rotor_inertia_kg_m2",
        "pitch_pivot_bearings_lock_and_datum_geometry",
    )
    missing_is_explicit = all(
        ledger[name]["classification"] == "MISSING" and ledger[name]["value"] is None
        for name in required_missing
    )
    provisional = json.loads(PROVISIONAL_GATE.read_text(encoding="utf-8"))
    classes = provisional["classifications"]
    motor_geometry = "d5065_motor_left" in xacro and "d5065_motor_right" in xacro
    hub_geometry = "hub_collar" in xacro and "hub_pilot" in xacro
    retention_geometry = "retainer" in xacro
    effort_limited = (
        '<command_interface name="effort"><param name="min">-0.62</param><param name="max">0.62</param>'
        in BENCH_XACRO.read_text(encoding="utf-8")
    )
    ball_unchanged = _sha256(BALL_RESULTS) == results["authoritative_ball_reference"]["sha256"]
    stop_correct = (
        results["decision"]["stopped_at_gate"]
        == "A_STANDALONE_LAUNCHER_MECHANICAL_COMPLETENESS"
        and results["decision"]["subsequent_gates_run"] is False
        and results["decision"]["launch_trials_run"] is False
    )
    no_performance_claims = (
        results["requested_outputs"]["operating_points"] == []
        and not any(
            value
            for name, value in results["classifications"].items()
            if name.startswith("LAUNCHER_") and name != "LAUNCHER_TYRE_FRICTION_CALIBRATED"
        )
    )
    return {
        "ball_reference_unchanged": ball_unchanged,
        "historical_stop_snapshot_preserved": missing_is_explicit and stop_correct and no_performance_claims,
        "provisional_gate_supersedes_historical_stop": classes["FLYWHEEL_MECHANICAL_GATE_A_SIMULATION_READY"],
        "motor_bodies_present_in_standalone_geometry": motor_geometry,
        "provisional_hubs_present_in_standalone_geometry": hub_geometry,
        "axial_retention_present_in_standalone_geometry": retention_geometry,
        "d5065_effort_limit_enforced": effort_limited,
        "capability_phase_authorized": provisional["decision"]["capability_phase_authorized"],
        "physical_validation_remains_false": not classes["FLYWHEEL_MECHANICAL_GATE_A_PHYSICAL_VALIDATED"],
    }


def main() -> int:
    result = evaluate()
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if all(result.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
