#!/usr/bin/env python3
"""Freeze P2-62 physical-drive predictions from the existing 5a evidence."""
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2"
OUTDIR = RES / "physical_drive_p262"
RADII_MM = (82, 84, 85)
MUS = (0.5, 0.7, 0.9)
KT_OUT = 3.73 / 7.0  # effective gearbox-output N m/A from repository stall label
WHEELS = ("rear_left", "front_left", "rear_right", "front_right")


def tag(x):
    return str(x).replace(".", "p")


def load(kind, mu, variant=None):
    if kind == "base":
        p = RES / "gazebo_5a" / "ballast" / f"mu_{tag(mu)}"
    else:
        p = RES / "gazebo_5a_turn_diagnostic" / variant / "ballast" / f"mu_{tag(mu)}"
    return json.loads((p / "summary.json").read_text()), p / "telemetry.csv"


def current_stats(csv_path, scenario):
    totals = []
    with csv_path.open(newline="") as f:
        for r in csv.DictReader(f):
            if r["scenario"] == scenario:
                totals.append(sum(abs(float(r[f"{w}_command_nm"])) for w in WHEELS) / KT_OUT)
    return {"mean_total_a": round(sum(totals) / len(totals), 3),
            "peak_total_a": round(max(totals), 3),
            "basis": "sum(abs(5a wheel torque command))/(3.73 Nm/7 A); prediction, not measured BTS7960 current"}


def accel_to_95(csv_path, radius_scale):
    rows = []
    with csv_path.open(newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["scenario"] == "straight_0p6"]
    first = rows[0]
    hit = next((r for r in rows if float(r["vx_body_m_s"]) >= 0.57), rows[-1])
    return {"time_s": round(float(hit["time_s"])-float(first["time_s"]), 3),
            "distance_m": round((float(hit["x_m"])-float(first["x_m"]))*radius_scale, 4),
            "threshold": "first vx_body >= 95% of 0.6 m/s"}


def pose_about_icr(icr, theta):
    cx, cy = icr
    return (cx * (1-math.cos(theta)) + cy * math.sin(theta),
            cy * (1-math.cos(theta)) - cx * math.sin(theta))


def main():
    predictions = []
    for mu in MUS:
        base, base_csv = load("base", mu)
        turns = {}
        for variant in ("open_yaw", "gyro_yaw"):
            turns[variant] = load("turn", mu, variant)
        for radius_mm in RADII_MM:
            rs = radius_mm / 85.0
            for speed in (0.3, 0.6):
                sname = f"straight_{tag(speed)}"
                s = base["scenarios"][sname]
                actual_speed = speed * (1-s["speed_relative_error"]) * rs
                duration = 3.0 / speed
                predictions.append({
                    "scenario": f"straight_3m_{tag(speed)}", "mu": mu, "loaded_radius_mm": radius_mm,
                    "nominal_duration_s": round(duration, 3),
                    "final_pose": {"x_m": round(actual_speed*duration, 4), "y_m": round(s["cross_track_m"], 6),
                                   "yaw_deg": round(s["heading_error_deg"], 5)},
                    "lateral_deviation_m": round(s["cross_track_m"], 6),
                    "actual_speed_m_s": round(actual_speed, 4),
                    "encoder_vs_actual_slip_fraction": round(s["mean_wheel_slip"], 5),
                    "current": current_stats(base_csv, sname)})
            for variant in ("open_yaw", "gyro_yaw"):
                td, tcsv = turns[variant]
                s = td["scenarios"]["spin_0p5"]
                for degrees in (90, 180, 360):
                    duration = math.radians(degrees)/0.5
                    actual_w = s["mean_w_rad_s"] * rs
                    theta = actual_w * duration
                    x, y = pose_about_icr(s["instantaneous_icr_body_median_m"], theta)
                    predictions.append({
                        "scenario": f"spin_{degrees}_{variant}", "mu": mu, "loaded_radius_mm": radius_mm,
                        "nominal_duration_s": round(duration, 3),
                        "final_pose": {"x_m": round(x, 4), "y_m": round(y, 4), "yaw_deg": round(math.degrees(theta), 2)},
                        "centre_translation_m": round(math.hypot(x,y), 4), "yaw_rate_rad_s": round(actual_w, 4),
                        "encoder_vs_actual_slip_fraction": round(s["mean_slip"], 4),
                        "current": current_stats(tcsv, "spin_0p5")})
            td, tcsv = turns["gyro_yaw"]
            s = td["scenarios"]["moving_turn"]
            for arc_r in (0.5, 1.0):
                target_v, target_w = 0.3, 0.3/arc_r
                av = target_v * (s["mean_v_m_s"]/0.4) * rs
                aw = target_w * (s["mean_w_rad_s"]/0.6) * rs
                duration = math.pi*arc_r/target_v
                theta = aw*duration
                rr = av/aw
                x, y = rr*math.sin(theta), rr*(1-math.cos(theta))
                predictions.append({
                    "scenario": f"arc_R{tag(arc_r)}_180deg_gyro", "mu": mu, "loaded_radius_mm": radius_mm,
                    "nominal_duration_s": round(duration,3),
                    "final_pose": {"x_m": round(x,4), "y_m": round(y,4), "yaw_deg": round(math.degrees(theta),2)},
                    "ideal_final_pose": {"x_m": 0, "y_m": 2*arc_r, "yaw_deg": 180},
                    "endpoint_error_m": round(math.hypot(x, y-2*arc_r),4),
                    "actual_radius_m": round(rr,4), "yaw_rate_rad_s": round(aw,4),
                    "encoder_vs_actual_slip_fraction": round(s["mean_slip"],4),
                    "current": current_stats(tcsv, "moving_turn")})
            predictions.append({
                "scenario": "accel_brake_0_to_0p6_to_0", "mu": mu, "loaded_radius_mm": radius_mm,
                "steady_speed_m_s": round(0.6*(1-base["scenarios"]["straight_0p6"]["speed_relative_error"])*rs,4),
                "acceleration_to_95pct": accel_to_95(base_csv, rs),
                "brake_stop_time_s": round(base["scenarios"]["brake_from_0p6"]["stop_time_s"],3),
                "brake_stop_distance_m": round(base["scenarios"]["brake_from_0p6"]["stop_distance_m"]*rs,4),
                "brake_heading_change_deg": round(base["scenarios"]["brake_from_0p6"]["heading_change_deg"],4),
                "encoder_vs_actual_slip_fraction": round(base["scenarios"]["straight_0p6"]["mean_wheel_slip"],5),
                "acceleration_current": current_stats(base_csv, "straight_0p6"),
                "braking_current": current_stats(base_csv, "brake_from_0p6")})

    payload = {
        "schema": "p2-62-frozen-drive-predictions-v1", "created_utc": "2026-10-14T00:00:00Z",
        "source_git_head": "5196885912057f5df8cdf0e444006d1d9ac3c03c",
        "source_tree_note": "working tree intentionally dirty; SHA identifies the repository base, while the sibling SHA-256 freezes this exact generated file",
        "physical_measurements_used": False,
        "configuration": {"motor_supply_v": 12, "bench_supply_max_current_a": None,
            "mega_supply": "USB separate from motor supply; common GND", "motors": "4x DFRobot FIT0403",
            "wheels": "Baja 170x80", "loaded_radius_sweep_mm": list(RADII_MM), "track_m": 0.585,
            "ballast_plate": "5.448 kg at X=360 mm plus README dummy battery masses",
            "ground_mu_sweep": list(MUS)},
        "interpretation_limits": [
            "5a is an ideal torque-command model, not a BTS7960 electrical model.",
            "Current uses the FIT0403 stall-label secant 3.73 Nm/7 A and excludes no-load current, driver loss and supply transients.",
            "The current Mega sketch exposes duty, encoder counts and raw gyro only; it does not measure current and does not implement the 5a wheel-speed/yaw controller.",
            "Radius scaling assumes commands were calculated for 85 mm; validate encoder counts/rev and loaded radius before scoring."],
        "test_definition": {"arcs": "180 deg left arcs at v=0.3 m/s with gyro", "spins": "target 0.5 rad/s",
                            "straight": "open straight command for nominal 3 m duration"},
        "predictions": predictions,
        "acceptance_limits": {"straight_endpoint_error_m": 0.15, "straight_lateral_m": 0.05,
            "straight_heading_deg": 3, "gyro_spin_angle_percent": 15, "gyro_spin_centre_translation_m": 0.55,
            "open_spin_angle_is_diagnostic": True, "arc_endpoint_error_m": 0.20, "arc_yaw_error_deg": 15,
            "brake_stop_time_s": 1.5, "brake_stop_distance_m": 0.45,
            "encoder_actual_slip_mean": 0.20},
        "holdout": "arc_R1p0_180deg_gyro at mu selected by calibration and measured loaded radius; do not use it to fit parameters"
    }
    OUTDIR.mkdir(parents=True, exist_ok=True)
    out = OUTDIR / "drive_predictions.json"
    raw = (json.dumps(payload, indent=2, ensure_ascii=False) + "\n").encode()
    out.write_bytes(raw)
    digest = hashlib.sha256(raw).hexdigest()
    (OUTDIR / "drive_predictions.json.sha256").write_text(f"{digest}  drive_predictions.json\n")
    print(out)
    print(digest)


if __name__ == "__main__":
    main()
