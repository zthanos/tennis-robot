#!/usr/bin/env python3
"""Evaluate one 5a CSV against the criteria frozen before the campaign."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


WHEELS = ("rear_left", "front_left", "rear_right", "front_right")
STEADY_START = {
    "straight_0p3": 4.0, "straight_0p6": 11.0,
    "spin_0p5": 19.0, "spin_1p0": 26.0, "moving_turn": 33.0,
}


def angle_delta(a: np.ndarray, origin: float) -> np.ndarray:
    return np.unwrap(a) - origin


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--criteria", type=Path, required=True)
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    criteria = json.loads(args.criteria.read_text())["pass_criteria"]
    validation = json.loads(args.validation.read_text())
    with args.csv.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 100:
        raise SystemExit(f"insufficient samples: {len(rows)}")
    names = rows[0].keys()
    data = {name: np.array([float(r[name]) for r in rows]) for name in names
            if name not in {"scenario", "case_name"}}
    scenarios = np.array([r["scenario"] for r in rows])
    case_name = rows[0]["case_name"]
    mu = float(rows[0]["mu"])
    settled = (data["time_s"] >= 1.5) & (data["time_s"] < 2.0)
    settled_z = float(np.median(data["z_m"][settled]))
    result = {"case": case_name, "mu": mu, "sample_count": len(rows), "scenarios": {}, "loads": {}}

    for scenario, start in STEADY_START.items():
        all_mask = scenarios == scenario
        steady = all_mask & (data["time_s"] >= start)
        idx0 = int(np.flatnonzero(all_mask)[0])
        yaw0 = data["yaw_rad"][idx0]
        x0, y0 = data["x_m"][idx0], data["y_m"][idx0]
        command_v = float(np.median(data["target_v_m_s"][steady]))
        command_w = float(np.median(data["target_w_rad_s"][steady]))
        metrics: dict[str, float | bool | list[str]] = {}
        checks: list[bool] = []
        if scenario.startswith("straight"):
            c = criteria["straight"]
            speed_error = abs(float(np.mean(data["vx_body_m_s"][steady])) - command_v) / abs(command_v)
            dx, dy = data["x_m"][all_mask] - x0, data["y_m"][all_mask] - y0
            cross = np.max(np.abs(-math.sin(yaw0)*dx + math.cos(yaw0)*dy))
            heading = np.max(np.abs(np.degrees(angle_delta(data["yaw_rad"][all_mask], yaw0))))
            slips = np.concatenate([data[f"{w}_slip_ratio"][steady] for w in WHEELS])
            metrics.update(speed_relative_error=float(speed_error), cross_track_m=float(cross),
                           heading_error_deg=float(heading), mean_slip=float(np.mean(slips)),
                           peak_slip=float(np.max(slips)))
            checks += [speed_error <= c["steady_speed_relative_error_max"], cross <= c["cross_track_deviation_m_max"],
                       heading <= c["heading_error_deg_max"], np.mean(slips) <= c["steady_mean_longitudinal_slip_ratio_max"],
                       np.max(slips) <= c["steady_peak_longitudinal_slip_ratio_max"]]
        elif scenario.startswith("spin"):
            c = criteria["spin"]
            yaw_error = abs(float(np.mean(data["wz_body_rad_s"][steady])) - command_w) / abs(command_w)
            drift = np.max(np.hypot(data["x_m"][all_mask]-x0, data["y_m"][all_mask]-y0))
            target = abs(command_w * 0.297 / 0.085)
            low_yaw = np.abs(data["wz_body_rad_s"][steady]) < 0.85 * abs(command_w)
            rear_over = np.maximum(np.abs(data["rear_left_speed_rad_s"][steady]),
                                   np.abs(data["rear_right_speed_rad_s"][steady])) > 1.15 * target
            free_spin_fraction = float(np.mean(low_yaw & rear_over))
            metrics.update(yaw_rate_relative_error=float(yaw_error), centre_translation_m=float(drift),
                           rear_free_spin_fraction=free_spin_fraction)
            checks += [yaw_error <= c["steady_yaw_rate_relative_error_max"], drift <= c["centre_translation_m_max"],
                       free_spin_fraction <= c["rear_wheel_overspeed_fraction_max"]]
        else:
            c = criteria["moving_turn"]
            v_error = abs(float(np.mean(data["vx_body_m_s"][steady])) - command_v) / abs(command_v)
            w_error = abs(float(np.mean(data["wz_body_rad_s"][steady])) - command_w) / abs(command_w)
            metrics.update(linear_speed_relative_error=float(v_error), yaw_rate_relative_error=float(w_error))
            checks += [v_error <= c["steady_linear_speed_relative_error_max"], w_error <= c["steady_yaw_rate_relative_error_max"]]

        turn_slips = np.concatenate([data[f"{w}_slip_ratio"][steady] for w in WHEELS])
        metrics["mean_wheel_slip"] = float(np.mean(turn_slips))
        metrics["peak_wheel_slip"] = float(np.max(turn_slips))
        min_normal_ratio = 1.0
        max_low_joint_fz_fraction = 0.0
        max_wheel_centre_rise = 0.0
        for wheel in WHEELS:
            settled_normal = float(np.median(np.abs(data[f"{wheel}_wrench_fz_n"][settled])))
            normal = np.abs(data[f"{wheel}_wrench_fz_n"][all_mask])
            if settled_normal > 1e-6:
                min_normal_ratio = min(min_normal_ratio, float(np.min(normal) / settled_normal))
                max_low_joint_fz_fraction = max(max_low_joint_fz_fraction,
                    float(np.mean(normal < 0.05 * settled_normal)))
            settled_wheel_z = float(np.median(data[f"{wheel}_centre_z_m"][settled]))
            max_wheel_centre_rise = max(max_wheel_centre_rise,
                float(np.max(data[f"{wheel}_centre_z_m"][all_mask] - settled_wheel_z)))
        metrics["minimum_wheel_normal_load_ratio_to_settled"] = min_normal_ratio
        metrics["low_joint_fz_fraction_below_5pct_settled_diagnostic"] = max_low_joint_fz_fraction
        metrics["max_wheel_centre_rise_m"] = max_wheel_centre_rise

        stability = criteria["stability"]
        roll = float(np.max(np.abs(np.degrees(data["roll_rad"][all_mask]))))
        pitch = float(np.max(np.abs(np.degrees(data["pitch_rad"][all_mask]))))
        rise = float(np.max(data["z_m"][all_mask] - settled_z))
        torque = np.concatenate([np.abs(data[f"{w}_command_nm"][steady]) for w in WHEELS])
        saturation = float(np.mean(torque >= criteria["drive_torque"]["saturation_threshold_fraction_of_limit"] * 3.73))
        metrics.update(max_abs_roll_deg=roll, max_abs_pitch_deg=pitch, max_base_rise_m=rise,
                       max_drive_torque_nm=float(np.max(torque)), torque_saturation_fraction=saturation)
        checks += [roll <= stability["absolute_roll_deg_max"], pitch <= stability["absolute_pitch_deg_max"],
                   rise <= stability["base_rise_from_settled_m_max"], np.max(torque) <= 3.730001,
                   saturation <= criteria["drive_torque"]["steady_saturation_fraction_max"]]
        metrics["pass"] = bool(all(checks))
        result["scenarios"][scenario] = metrics

    brake = scenarios == "brake_from_0p6"
    brake_indices = np.flatnonzero(brake)
    b0 = brake_indices[0]
    speed = np.abs(data["vx_body_m_s"][brake_indices])
    stopped = np.flatnonzero(speed < criteria["braking"]["speed_threshold_m_s"])
    stop_i = brake_indices[stopped[0]] if len(stopped) else brake_indices[-1]
    stop_time = data["time_s"][stop_i] - data["time_s"][b0]
    stop_distance = math.hypot(data["x_m"][stop_i]-data["x_m"][b0], data["y_m"][stop_i]-data["y_m"][b0])
    heading = abs(math.degrees(float(angle_delta(data["yaw_rad"][brake_indices], data["yaw_rad"][b0])[-1])))
    b = criteria["braking"]
    brake_result = {"stop_time_s": float(stop_time), "stop_distance_m": float(stop_distance),
                    "heading_change_deg": float(heading)}
    stability = criteria["stability"]
    brake_roll = float(np.max(np.abs(np.degrees(data["roll_rad"][brake]))))
    brake_pitch = float(np.max(np.abs(np.degrees(data["pitch_rad"][brake]))))
    brake_rise = float(np.max(data["z_m"][brake] - settled_z))
    brake_wheel_rise = 0.0
    for wheel in WHEELS:
        settled_wheel_z = float(np.median(data[f"{wheel}_centre_z_m"][settled]))
        brake_wheel_rise = max(brake_wheel_rise,
            float(np.max(data[f"{wheel}_centre_z_m"][brake] - settled_wheel_z)))
    brake_torque = np.concatenate([np.abs(data[f"{w}_command_nm"][brake]) for w in WHEELS])
    brake_saturation = float(np.mean(brake_torque >= criteria["drive_torque"]["saturation_threshold_fraction_of_limit"] * 3.73))
    brake_result.update(max_abs_roll_deg=brake_roll, max_abs_pitch_deg=brake_pitch,
                        max_base_rise_m=brake_rise, max_wheel_centre_rise_m=brake_wheel_rise,
                        max_drive_torque_nm=float(np.max(brake_torque)),
                        torque_saturation_fraction=brake_saturation)
    brake_result["pass"] = bool(len(stopped) and stop_time <= b["stop_time_s_max"] and
                                stop_distance <= b["stop_distance_m_max"] and heading <= b["heading_change_deg_max"] and
                                brake_roll <= stability["absolute_roll_deg_max"] and
                                brake_pitch <= stability["absolute_pitch_deg_max"] and
                                brake_rise <= stability["base_rise_from_settled_m_max"] and
                                np.max(brake_torque) <= 3.730001 and
                                brake_saturation <= criteria["drive_torque"]["steady_saturation_fraction_max"])
    result["scenarios"]["brake_from_0p6"] = brake_result

    for scenario in (*STEADY_START, "brake_from_0p6"):
        mask = scenarios == scenario
        load = {}
        for wheel in WHEELS:
            load[f"{wheel}_peak_force_n"] = float(np.max(data[f"{wheel}_wrench_force_n"][mask]))
            load[f"{wheel}_peak_torque_nm"] = float(np.max(data[f"{wheel}_wrench_torque_nm"][mask]))
            for component in ("fx_n", "fy_n", "fz_n", "tx_nm", "ty_nm", "tz_nm"):
                values = data[f"{wheel}_wrench_{component}"][mask]
                load[f"{wheel}_{component}_min"] = float(np.min(values))
                load[f"{wheel}_{component}_max"] = float(np.max(values))
        load["tilt_peak_force_n"] = float(np.max(data["tilt_wrench_force_n"][mask]))
        load["tilt_peak_torque_nm"] = float(np.max(data["tilt_wrench_torque_nm"][mask]))
        result["loads"][scenario] = load
    mass = float(validation["total_mass_kg"])
    izz = float(validation["total_inertia_at_cg_kg_m2"][2][2])
    moving = np.isin(scenarios, list(STEADY_START))
    result["inertial_loads"] = {
        "mass_kg": mass, "yaw_inertia_kg_m2": izz,
        "peak_force_from_raw_acceleration_n": float(mass*np.max(np.hypot(data["ax_body_m_s2"][moving], data["ay_body_m_s2"][moving]))),
        "p99_force_from_acceleration_n": float(mass*np.quantile(np.hypot(data["ax_body_m_s2"][moving], data["ay_body_m_s2"][moving]), .99)),
        "peak_yaw_moment_from_raw_alpha_nm": float(izz*np.max(np.abs(data["alpha_z_rad_s2"][moving]))),
        "p99_yaw_moment_from_alpha_nm": float(izz*np.quantile(np.abs(data["alpha_z_rad_s2"][moving]), .99)),
    }
    result["overall_pass"] = all(v["pass"] for v in result["scenarios"].values())
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"case": case_name, "mu": mu, "overall_pass": result["overall_pass"]}))


if __name__ == "__main__":
    main()
