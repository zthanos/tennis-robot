#!/usr/bin/env python3
"""Evaluate the standalone launcher reconstruction and calibrated ball gate."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAD_PARAMS = ROOT / "cad/flywheel-launcher-v0/params.scad"
MODULE_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro"
BENCH_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
BENCH_WORLD = ROOT / "gazebo/worlds/flywheel_launcher_geometry_bench.sdf"
BALL_DESIGN = ROOT / "config/tennis_ball_compliance_design.json"


def scad_scalar(name: str) -> float:
    text = CAD_PARAMS.read_text(encoding="utf-8")
    match = re.search(rf"^\s*{re.escape(name)}\s*=\s*([-+0-9.]+)\s*;", text, re.MULTILINE)
    if match is None:
        raise ValueError(f"missing literal CAD scalar: {name}")
    return float(match.group(1))


def xacro_property(name: str) -> float:
    text = MODULE_XACRO.read_text(encoding="utf-8")
    match = re.search(
        rf'<xacro:property\s+name="{re.escape(name)}"\s+value="([-+0-9.]+)"\s*/>',
        text,
    )
    if match is None:
        raise ValueError(f"missing literal Xacro property: {name}")
    return float(match.group(1))


def evaluate() -> dict[str, object]:
    ball_d = scad_scalar("ball_d") / 1000.0
    wheel_d = scad_scalar("wheel_d") / 1000.0
    wheel_width = scad_scalar("wheel_width") / 1000.0
    nip_gap = scad_scalar("nip_gap") / 1000.0
    cradle_margin = scad_scalar("cradle_margin") / 1000.0
    plate_y = 2.0 * (wheel_d / 2.0 + cradle_margin) + nip_gap

    expected = {
        "launcher_ball_d": ball_d,
        "launcher_wheel_radius": wheel_d / 2.0,
        "launcher_wheel_width": wheel_width,
        "launcher_nip_gap": nip_gap,
        "launcher_wheel_center_distance": wheel_d + nip_gap,
        "launcher_wheel_y": (wheel_d + nip_gap) / 2.0,
        "launcher_plate_x": wheel_d + 2.0 * cradle_margin,
        "launcher_plate_y": plate_y,
        "launcher_plate_z": scad_scalar("side_plate_t") / 1000.0,
        "launcher_plate_offset_z": wheel_width / 2.0 + 0.018,
    }

    geometry_checks = {
        key: math.isclose(xacro_property(key), value, rel_tol=0.0, abs_tol=1e-12)
        for key, value in expected.items()
    }

    bench_text = BENCH_XACRO.read_text(encoding="utf-8").lower()
    world_text = BENCH_WORLD.read_text(encoding="utf-8").lower()
    contract = json.loads(BALL_DESIGN.read_text(encoding="utf-8"))
    acceptance = contract["acceptance"]
    isolated = all(
        token not in bench_text
        for token in ("intake_", "basket_", "feeder_", "oak_d", "navigation", "throwing")
    )
    explicit_dart = (
        "gz-physics-dartsim-plugin" in world_text
        and '<physics name="dart_1khz" type="dart">' in world_text
    )
    world_has_no_ball = '<model name="ball' not in world_text

    design_ready = (
        all(geometry_checks.values())
        and isolated
        and explicit_dart
        and world_has_no_ball
        and acceptance["design_architecture_selected"] is True
        and acceptance["coefficients_may_be_fitted_only_to_calibration_trials"] is True
        and acceptance["launch_results_may_not_be_used_for_parameter_fit"] is True
    )
    required_evidence = contract["required_calibration_evidence"]
    # Tyre friction is deliberately outside the independently calibrated ball
    # normal-contact gate. Its pending state prohibits friction-dependent
    # conclusions, but the task contract explicitly permits normal trials.
    applicable_ball_evidence = {
        key: value
        for key, value in required_evidence.items()
        if key != "launcher_tyre_friction_measurement"
    }
    launch_authorized = (
        acceptance["compliant_model_implemented"] is True
        and acceptance["compliant_model_calibrated"] is True
        and all(applicable_ball_evidence.values())
        and acceptance["launcher_physics_trials_authorized"] is True
    )
    return {
        "authoritative_geometry_reconstructed": all(geometry_checks.values()),
        "geometry_checks": geometry_checks,
        "isolated_architecture": isolated,
        "explicit_dart_engine": explicit_dart,
        "ball_absent_until_calibrated": world_has_no_ball,
        "compliant_ball_model_design_ready": design_ready,
        "compliant_ball_model_implemented": acceptance["compliant_model_implemented"],
        "compliant_ball_model_calibrated": acceptance["compliant_model_calibrated"],
        "launcher_tyre_friction_calibration_pending": (
            required_evidence["launcher_tyre_friction_measurement"] is False
        ),
        "launcher_physics_trials_authorized": launch_authorized,
    }


def main() -> int:
    print(json.dumps(evaluate(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
