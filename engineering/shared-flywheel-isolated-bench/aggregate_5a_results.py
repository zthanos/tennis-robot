#!/usr/bin/env python3
"""Aggregate the immutable per-run 5a summaries into campaign artifacts."""
from __future__ import annotations

import csv
import json
from pathlib import Path


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a")
CASES = ("empty", "full45", "ballast")
MUS = ("0p5", "0p7", "0p9")
SCENARIOS = ("straight_0p3", "straight_0p6", "spin_0p5", "spin_1p0", "moving_turn", "brake_from_0p6")


def update_max(record: dict, key: str, value: float, location: dict) -> None:
    if value > record[key]["value"]:
        record[key] = {"value": value, **location}


def main() -> None:
    rows = []
    extrema = {key: {"value": 0.0} for key in (
        "drive_command_nm", "wheel_joint_force_n", "wheel_joint_torque_nm",
        "tilt_joint_force_n", "tilt_joint_torque_nm", "wheel_centre_rise_m",
        "p99_inertial_force_n", "p99_inertial_yaw_moment_nm")}
    combinations_passed = 0
    scenarios_passed = 0
    for case in CASES:
        for mu_token in MUS:
            path = ROOT / case / f"mu_{mu_token}" / "summary.json"
            data = json.loads(path.read_text())
            combinations_passed += int(data["overall_pass"])
            for scenario in SCENARIOS:
                metrics = data["scenarios"][scenario]
                scenarios_passed += int(metrics["pass"])
                row = {"case": case, "mu": data["mu"], "scenario": scenario, "pass": metrics["pass"]}
                row.update({k: v for k, v in metrics.items() if k != "pass"})
                location = {"case": case, "mu": data["mu"], "scenario": scenario}
                if "max_drive_torque_nm" in metrics:
                    update_max(extrema, "drive_command_nm", metrics["max_drive_torque_nm"], location)
                    update_max(extrema, "wheel_centre_rise_m", metrics["max_wheel_centre_rise_m"], location)
                if scenario in data["loads"]:
                    loads = data["loads"][scenario]
                    row["max_wheel_joint_force_n"] = max(
                        loads[f"{wheel}_peak_force_n"] for wheel in
                        ("rear_left", "front_left", "rear_right", "front_right"))
                    row["max_wheel_joint_torque_nm"] = max(
                        loads[f"{wheel}_peak_torque_nm"] for wheel in
                        ("rear_left", "front_left", "rear_right", "front_right"))
                    row["tilt_joint_peak_force_n"] = loads["tilt_peak_force_n"]
                    row["tilt_joint_peak_torque_nm"] = loads["tilt_peak_torque_nm"]
                    for wheel in ("rear_left", "front_left", "rear_right", "front_right"):
                        update_max(extrema, "wheel_joint_force_n", loads[f"{wheel}_peak_force_n"], {**location, "joint": wheel})
                        update_max(extrema, "wheel_joint_torque_nm", loads[f"{wheel}_peak_torque_nm"], {**location, "joint": wheel})
                    update_max(extrema, "tilt_joint_force_n", loads["tilt_peak_force_n"], location)
                    update_max(extrema, "tilt_joint_torque_nm", loads["tilt_peak_torque_nm"], location)
                rows.append(row)
            inertial = data["inertial_loads"]
            update_max(extrema, "p99_inertial_force_n", inertial["p99_force_from_acceleration_n"], {"case": case, "mu": data["mu"]})
            update_max(extrema, "p99_inertial_yaw_moment_nm", inertial["p99_yaw_moment_from_alpha_nm"], {"case": case, "mu": data["mu"]})

    columns = ["case", "mu", "scenario", "pass"] + sorted({key for row in rows for key in row} - {"case", "mu", "scenario", "pass"})
    with (ROOT / "scenario_results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    campaign = {
        "combination_count": 9,
        "combinations_passed_all_scenarios": combinations_passed,
        "scenario_count": len(rows),
        "scenarios_passed": scenarios_passed,
        "scenarios_failed": len(rows) - scenarios_passed,
        "extrema": extrema,
        "gyro_interpretation": "The frozen campaign is wheel-speed closed loop and yaw open loop. Existing gyro can support a future outer yaw-rate loop; it cannot remove skid-steer scrub or structural loads.",
        "wheel_lift_measurement": "Direct maximum wheel-centre Z rise relative to the settled 1.5-2.0 s window; joint Fz is retained only as a solver-sensitive diagnostic.",
    }
    (ROOT / "campaign_summary.json").write_text(json.dumps(campaign, indent=2) + "\n")
    print(json.dumps(campaign, indent=2))


if __name__ == "__main__":
    main()
