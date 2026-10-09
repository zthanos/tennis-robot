#!/usr/bin/env python3
"""Combine modular chassis Gazebo cases into connector design loads."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


GROUPS = {
    "rear_corner_50x50": ("rear_left_corner", "rear_right_corner"),
    "straight_motor_splice_50x50": ("left_motor_splice", "right_motor_splice"),
    "gamma_socket_50x50": ("left_gamma_splice", "right_gamma_splice"),
    "gamma_m5_mount": ("left_intake_mount", "right_intake_mount"),
    "electronics_mount": ("electronics_tray",),
}


def summarize(paths: list[Path], design_factor: float) -> dict[str, Any]:
    cases = {path.parent.name: json.loads(path.read_text(encoding="utf-8")) for path in paths}
    interfaces: dict[str, Any] = {}
    for group, names in GROUPS.items():
        samples = []
        for case_name, report in cases.items():
            for name in names:
                joint = report["joints"][name]
                samples.append(
                    {
                        "case": case_name,
                        "joint": name,
                        "force_p99_5_n": joint["force_norm_n"]["p99_5"],
                        "torque_p99_5_nm": joint["torque_norm_nm"]["p99_5"],
                        "raw_force_max_n": joint["force_norm_n"]["maximum"],
                        "raw_torque_max_nm": joint["torque_norm_nm"]["maximum"],
                    }
                )
        force_governing = max(samples, key=lambda item: item["force_p99_5_n"])
        torque_governing = max(samples, key=lambda item: item["torque_p99_5_nm"])
        interfaces[group] = {
            "governing_force": force_governing,
            "governing_torque": torque_governing,
            "design_force_n": force_governing["force_p99_5_n"] * design_factor,
            "design_torque_nm": torque_governing["torque_p99_5_nm"] * design_factor,
        }
    return {
        "schema_version": 1,
        "source": "Gazebo rigid-body modular chassis integration bench",
        "load_statistic": "p99.5 after settle; raw maximum retained only for diagnostics",
        "design_factor": design_factor,
        "cases": sorted(cases),
        "interfaces": interfaces,
        "status": "CAD_LOADS_AVAILABLE_STRUCTURAL_CAPACITY_PENDING",
        "limitations": [
            "Gazebo does not calculate printed-polymer stress or layer separation.",
            "The 10 kg payload is represented at the electronics tray datum.",
            "Connector capacity still requires FEA and printed fit/load coupons.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("summaries", nargs="+", type=Path)
    parser.add_argument("--design-factor", type=float, default=2.5)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.design_factor < 1:
        parser.error("--design-factor must be at least 1")
    report = summarize(args.summaries, args.design_factor)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({name: {
        "design_force_n": round(value["design_force_n"], 2),
        "design_torque_nm": round(value["design_torque_nm"], 2),
    } for name, value in report["interfaces"].items()}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
