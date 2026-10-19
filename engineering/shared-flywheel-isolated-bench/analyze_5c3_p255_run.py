#!/usr/bin/env python3
"""Analyze both balls and flywheel recovery in one corrected P2-55 case."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return [row for row in csv.DictReader(stream) if None not in row.values()]


def mean_speed(rows: list[dict[str, str]]) -> float:
    if len(rows) < 2:
        return math.inf
    speeds = []
    for one, two in zip(rows, rows[1:]):
        dt = float(two["time_s"]) - float(one["time_s"])
        if dt > 0:
            speeds.append(math.sqrt(sum(((float(two[k]) - float(one[k])) / dt) ** 2
                                        for k in ("base_x_m", "base_y_m", "base_z_m"))))
    return sum(speeds) / len(speeds) if speeds else math.inf


def wheel_contact_window(path: Path) -> tuple[float, float] | None:
    rows = read(path)
    times = [float(row["time_s"]) for row in rows
             if int(float(row["contact_id"])) in (0, 1)
             and float(row["normal_force_n"]) > 0.01]
    return (min(times), max(times)) if times else None


def wheel_recovery(telemetry: list[dict[str, str]], window: tuple[float, float] | None,
                   target_rad_s: float) -> dict:
    if window is None:
        return {"wheel_contact": False, "rpm_drop": None, "recovery_to_95pct_s": None}
    start, end = window
    during = [row for row in telemetry if start - 0.005 <= float(row["time_s"]) <= end + 0.005]
    target_rpm = target_rad_s * 60 / (2 * math.pi)
    minimum = {
        side: min(abs(float(row[f"{side}_speed_rad_s"])) * 60 / (2 * math.pi) for row in during)
        for side in ("left", "right")
    }


def force_diagnostics(native: list[dict[str, str]], ball_contact_paths: list[Path],
                      jam_time: float | None) -> dict:
    ball_ball = []
    for row in native:
        pair = (row.get("collision_1", ""), row.get("collision_2", ""))
        if "capture_ball_00_collision" in pair[0] + pair[1] and "capture_ball_01_collision" in pair[0] + pair[1]:
            fy = float(row.get("force_y_n", 0) or 0)
            force_on_00 = fy if "capture_ball_00_collision" in pair[0] else -fy
            ball_ball.append((float(row["time_s"]), force_on_00, row))
    wheel_series = []
    for index, path in enumerate(ball_contact_paths):
        rows = [row for row in read(path) if int(float(row["contact_id"])) in (0, 1)]
        wheel_series.append([(float(row["time_s"]),
                              float(row["normal_force_n"]) * float(row["normal_y"])
                              + float(row["tangential_force_y_n"]), row) for row in rows])
    force_time = jam_time
    if len(wheel_series) == 2 and all(wheel_series):
        candidates = []
        for time, force0, _ in wheel_series[0]:
            nearest = min(wheel_series[1], key=lambda item: abs(item[0] - time))
            if abs(nearest[0] - time) <= 0.002:
                candidates.append((min(force0, -nearest[1]), time))
        if candidates:
            force_time = max(candidates)[1]
    wheel = []
    for index, rows in enumerate(wheel_series):
        if force_time is not None and rows:
            time, total_y, row = min(rows, key=lambda item: abs(item[0] - force_time))
            wheel.append({"ball": index, "time_s": time,
                          "wheel_contact_id": int(float(row["contact_id"])),
                          "force_y_on_ball_n": total_y})
    if ball_ball and force_time is not None:
        _, force_on_00, source = min(ball_ball, key=lambda item: abs(item[0] - force_time))
    else:
        force_on_00, source = (None, None)
    inward = (len(wheel) == 2 and wheel[0]["force_y_on_ball_n"] > 0
              and wheel[1]["force_y_on_ball_n"] < 0)
    return {
        "evaluation_time_s": force_time,
        "wheel_ball_y_forces": wheel,
        "ball_ball_force_y_on_ball_00_n": force_on_00,
        "ball_ball_force_y_on_ball_01_n": None if force_on_00 is None else -force_on_00,
        "ball_ball_source": source,
        "front_faces_push_inward_confirmed": inward,
    }
    post = [row for row in telemetry if float(row["time_s"]) >= end]
    recovery = None
    required = 10  # 50 ms at the 5 ms telemetry cadence
    for index in range(max(0, len(post) - required + 1)):
        block = post[index:index + required]
        if all(abs(float(row["left_speed_rad_s"])) >= 0.95 * target_rad_s
               and abs(float(row["right_speed_rad_s"])) >= 0.95 * target_rad_s
               for row in block):
            recovery = max(0.0, float(block[0]["time_s"]) - end)
            break
    return {
        "wheel_contact": True,
        "contact_start_s": start,
        "contact_end_s": end,
        "target_rpm": target_rpm,
        "minimum_rpm": minimum,
        "rpm_drop": {side: target_rpm - minimum[side] for side in minimum},
        "maximum_rpm_drop": max(target_rpm - value for value in minimum.values()),
        "recovery_to_95pct_s": recovery,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True)
    ap.add_argument("--tread-mu", type=float, choices=(0.6, 0.9), required=True)
    ap.add_argument("--drive-profile", choices=(
        "continuous", "stop_1s", "reverse_75mm", "p256_reverse_both",
        "p256_asym_left", "p256_reverse150_yaw12"), default="continuous")
    ap.add_argument("--observer", type=Path, required=True)
    ap.add_argument("--telemetry", type=Path, required=True)
    ap.add_argument("--native-contacts", type=Path, required=True)
    ap.add_argument("--ball-contact", type=Path, action="append", required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    criteria = json.loads(args.criteria.read_text())
    observer, telemetry, native = read(args.observer), read(args.telemetry), read(args.native_contacts)
    if not observer or not telemetry:
        raise SystemExit("empty telemetry")
    by_name: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in observer:
        by_name[row["ball_name"]].append(row)
    if len(by_name) != 2 or len(args.ball_contact) != 2:
        raise SystemExit(f"corrected 5c-3 requires exactly two balls; observed {sorted(by_name)}")

    last_t = max(float(row["time_s"]) for row in observer)
    per_ball = {}
    for index, (name, rows) in enumerate(sorted(by_name.items())):
        active = [row for row in rows if float(row["time_s"]) >= 2.0]
        nip = next((row for row in active if float(row["base_x_m"]) <= 0.409), None)
        takeoff = next((row for row in active if float(row["base_x_m"]) <= 0.3288
                        and float(row["base_z_m"]) >= 0.0543), None)
        entry = next((row for row in active if float(row["base_x_m"]) <= 0.274
                      and abs(float(row["base_y_m"])) <= 0.139
                      and float(row["base_z_m"]) >= 0.075), None)
        tail = [row for row in rows if float(row["time_s"]) >= last_t - 0.5]
        means = {key: sum(float(row[key]) for row in tail) / len(tail)
                 for key in ("base_x_m", "base_y_m", "base_z_m")}
        tail_speed = mean_speed(tail)
        retained = (-0.052 <= means["base_x_m"] <= 0.245
                    and abs(means["base_y_m"]) <= 0.139
                    and 0.030 <= means["base_z_m"] <= 0.375
                    and tail_speed <= 0.25)
        pre_basket = active if entry is None else [row for row in active if float(row["time_s"]) <= float(entry["time_s"])]
        highest_top = max(float(row["base_z_m"]) for row in pre_basket) + 0.033
        takeoff_speed = None if takeoff is None else math.sqrt(sum(float(takeoff[key]) ** 2 for key in ("vx_m_s", "vy_m_s", "vz_m_s")))
        per_ball[name] = {
            "nip_capture": nip is not None,
            "ski_takeoff": takeoff is not None,
            "basket_entry": entry is not None,
            "retained": retained,
            "nip_time_s": None if nip is None else float(nip["time_s"]),
            "takeoff_time_s": None if takeoff is None else float(takeoff["time_s"]),
            "takeoff_speed_m_s": takeoff_speed,
            "basket_entry_time_s": None if entry is None else float(entry["time_s"]),
            "highest_ball_top_m": highest_top,
            "lidar_margin_m": 0.390 - highest_top,
            "final_mean_xyz_m": [means[k] for k in ("base_x_m", "base_y_m", "base_z_m")],
            "final_mean_speed_m_s": tail_speed,
            "flywheel_response": wheel_recovery(telemetry, wheel_contact_window(args.ball_contact[index]), 60.0),
        }

    launcher = ("cad_upper_plate", "cad_yoke_", "cad_trunnion_", "d5065_motor_", "flywheel_left", "flywheel_right")
    fixed = ("fixture_arch_", "fixture_lidar_", "fixture_camera_", "chassis_", "ground_plane")
    forbidden = []
    for row in native:
        one, two = row.get("collision_1") or "", row.get("collision_2") or ""
        pair = one + " " + two
        if "capture_ball_" in pair and any(token in pair for token in ("fixture_arch_", "fixture_lidar_", "fixture_camera_", "chassis_")):
            forbidden.append(row)
        elif ((any(token in one for token in launcher) and any(token in two for token in fixed)) or
              (any(token in two for token in launcher) and any(token in one for token in fixed))):
            forbidden.append(row)
    nonfinite = sum(not math.isfinite(float(value)) for row in observer for key, value in row.items()
                    if key != "ball_name" and value)
    nonfinite += sum(not math.isfinite(float(value)) for row in telemetry for value in row.values() if value)
    max_current = max(abs(float(row[key])) for row in telemetry for key in ("left_current_a", "right_current_a"))
    checks = {
        "all_balls_nip_capture": all(row["nip_capture"] for row in per_ball.values()),
        "all_balls_ski_takeoff": all(row["ski_takeoff"] for row in per_ball.values()),
        "all_balls_basket_entry": all(row["basket_entry"] for row in per_ball.values()),
        "all_balls_retained": all(row["retained"] for row in per_ball.values()),
        "lidar_margin": all(row["highest_ball_top_m"] <= 0.380 for row in per_ball.values()),
        "d5065_contact_current": max_current <= 20.0 + 1e-9,
        "d5065_contact_torque": 0.031 * max_current <= 0.62 + 1e-9,
        "forbidden_contact": not forbidden,
        "finite_state": nonfinite == 0,
    }
    hard = {row["name"] for row in criteria["criteria"] if row["hard_fail"]}
    takeoffs = sorted((row["takeoff_time_s"], row["takeoff_speed_m_s"], name)
                      for name, row in per_ball.items() if row["takeoff_time_s"] is not None)
    second_exit = takeoffs[1][1] if len(takeoffs) == 2 else None

    by_time: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in observer:
        by_time[float(row["time_s"])].append(row)
    pair_contact = None
    for time, rows in sorted(by_time.items()):
        if len(rows) != 2:
            continue
        distance = math.sqrt(sum((float(rows[0][key]) - float(rows[1][key])) ** 2
                                 for key in ("base_x_m", "base_y_m", "base_z_m")))
        if distance <= 0.067:
            pair_contact = {"time_s": time, "centre_distance_mm": 1000 * distance,
                            "mean_base_x_m": sum(float(row["base_x_m"]) for row in rows) / 2}
            break
    drive_rows = [row for row in telemetry if 2.75 <= float(row["time_s"]) <= 3.10]
    actual_reverse = 0.0
    if args.drive_profile == "reverse_75mm" and drive_rows:
        peak = max(float(row["base_x_m"]) for row in drive_rows)
        actual_reverse = 1000 * max(0.0, peak - min(float(row["base_x_m"]) for row in drive_rows))
    jam_rows = [row for row in telemetry if int(float(row.get("jam_detected", 0) or 0))]
    jam_time = float(jam_rows[0]["jam_trigger_s"]) if jam_rows else None
    peak_current_after_detection = (max((abs(float(row[key])) for row in jam_rows
                                         for key in ("left_current_a", "right_current_a")), default=None)
                                    if jam_rows else None)
    event_signature = None
    if jam_rows:
        event_signature = {
            "detected": True,
            "trigger_time_s": jam_time,
            "event_duration_ms": 1000 * float(jam_rows[0]["detector_event_duration_s"]),
            "peak_current_sum_a": float(jam_rows[0]["detector_peak_current_sum_a"]),
            "deficit_impulse_rpm_s": float(jam_rows[0]["detector_deficit_impulse_rpm_s"]),
        }
    else:
        event_signature = {"detected": False}
    outside = []
    for name, rows in by_name.items():
        post = rows if jam_time is None else [row for row in rows if float(row["time_s"]) >= jam_time]
        final = per_ball[name]["final_mean_xyz_m"]
        final_distance = math.hypot(final[0] - 0.387, final[1])
        peak_speed = max((math.sqrt(sum(float(row[key]) ** 2 for key in ("vx_m_s", "vy_m_s", "vz_m_s")))
                          for row in post), default=0.0)
        if not per_ball[name]["retained"]:
            outside.append({"ball": name, "final_distance_from_nip_m": final_distance,
                            "peak_post_detection_speed_m_s": peak_speed,
                            "ejected": final_distance > 0.20 and peak_speed > 0.50})
    completion_time = max((row["basket_entry_time_s"] for row in per_ball.values()
                           if row["basket_entry_time_s"] is not None), default=last_t)
    force_report = force_diagnostics(native, args.ball_contact, jam_time)
    vehicle_motion = None
    if jam_rows:
        anchor = jam_rows[0]
        ax, ay, ayaw = (float(anchor[key]) for key in ("base_x_m", "base_y_m", "base_yaw_rad"))
        unjam_rows = [row for row in jam_rows if int(float(row.get("recovery_phase", 0) or 0)) in (1, 2, 3)]
        reverse_rows = [row for row in jam_rows if int(float(row.get("recovery_phase", 0) or 0)) == 1]
        yaw_rows = [row for row in jam_rows if int(float(row.get("recovery_phase", 0) or 0)) == 3]
        if args.drive_profile in ("p256_reverse_both", "p256_asym_left") and unjam_rows:
            ax, ay, ayaw = (float(unjam_rows[0][key]) for key in ("base_x_m", "base_y_m", "base_yaw_rad"))
            reverse_rows = []
        displacement = lambda row: math.hypot(float(row["base_x_m"]) - ax, float(row["base_y_m"]) - ay)
        yaw_delta = lambda row: abs(math.atan2(math.sin(float(row["base_yaw_rad"]) - ayaw),
                                               math.cos(float(row["base_yaw_rad"]) - ayaw)))
        vehicle_motion = {
            "maximum_motion_during_recovery_mm": 1000 * max((displacement(row) for row in unjam_rows), default=0.0),
            "reverse_displacement_mm": 1000 * max((displacement(row) for row in reverse_rows), default=0.0),
            "yaw_change_deg": math.degrees(max((yaw_delta(row) for row in yaw_rows), default=0.0)),
        }
    result = {
        "pass": all(checks.get(name, False) for name in hard),
        "case": args.case,
        "tread_mu": args.tread_mu,
        "drive_profile": args.drive_profile,
        "diagnostic_only": args.drive_profile != "continuous",
        "checks": checks,
        "metrics": {
            "ball_count": len(per_ball),
            "per_ball": per_ball,
            "second_ball_exit_speed_m_s": second_exit,
            "minimum_takeoff_speed_m_s": min((row["takeoff_speed_m_s"] for row in per_ball.values() if row["takeoff_speed_m_s"] is not None), default=None),
            "first_pair_contact": pair_contact,
            "governing_nip_free_width_mm": 58.0,
            "two_ball_reference_width_mm": 134.0,
            "two_ball_width_deficit_mm": 76.0,
            "actual_reverse_displacement_mm": actual_reverse,
            "jam_detector": event_signature,
            "contact_force_diagnostics": force_report,
            "balls_in_basket": sum(row["retained"] for row in per_ball.values()),
            "balls_outside_basket": len(outside),
            "balls_ejected": sum(row["ejected"] for row in outside),
            "outside_ball_metrics": outside,
            "recovery_total_time_s": None if jam_time is None else completion_time - jam_time,
            "vehicle_motion": vehicle_motion,
            "max_current_a": max_current,
            "peak_current_after_detection_a": peak_current_after_detection,
            "max_torque_nm": 0.031 * max_current,
            "forbidden_contact_count": len(forbidden),
            "nonfinite_count": nonfinite,
        },
        "criteria_status": criteria["status"],
        "forbidden_contact_examples": forbidden[:10],
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
