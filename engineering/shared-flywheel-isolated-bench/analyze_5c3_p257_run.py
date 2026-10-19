#!/usr/bin/env python3
"""Analyze one P2-57 one- or two-ball geometric-prevention run."""
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


def speed(row: dict[str, str]) -> float:
    return math.sqrt(sum(float(row[key]) ** 2 for key in ("vx_m_s", "vy_m_s", "vz_m_s")))


def mean_tail_speed(rows: list[dict[str, str]]) -> float:
    return sum(speed(row) for row in rows) / len(rows) if rows else math.inf


def mean_relative_speed(rows: list[dict[str, str]]) -> float:
    values = []
    for one, two in zip(rows, rows[1:]):
        dt = float(two["time_s"]) - float(one["time_s"])
        if dt > 0:
            values.append(math.sqrt(sum(((float(two[key]) - float(one[key])) / dt) ** 2
                                        for key in ("base_x_m", "base_y_m", "base_z_m"))))
    return sum(values) / len(values) if values else math.inf


def wheel_window(path: Path) -> tuple[float, float] | None:
    times = [float(row["time_s"]) for row in read(path)
             if int(float(row["contact_id"])) in (0, 1)
             and float(row["normal_force_n"]) > 0.01]
    return (min(times), max(times)) if times else None


def wheel_response(telemetry: list[dict[str, str]], window: tuple[float, float] | None) -> dict:
    if window is None:
        return {"wheel_contact": False, "rpm_drop": None, "recovery_to_95pct_s": None}
    start, end = window
    target = 60.0
    target_rpm = target * 60.0 / (2.0 * math.pi)
    during = [row for row in telemetry if start - .005 <= float(row["time_s"]) <= end + .005]
    minimum = {side: min(abs(float(row[f"{side}_speed_rad_s"])) * 60.0 / (2.0 * math.pi)
                         for row in during) for side in ("left", "right")}
    post = [row for row in telemetry if float(row["time_s"]) >= end]
    recovery = None
    for index in range(max(0, len(post) - 9)):
        block = post[index:index + 10]
        if all(abs(float(row[f"{side}_speed_rad_s"])) >= .95 * target
               for row in block for side in ("left", "right")):
            recovery = max(0.0, float(block[0]["time_s"]) - end)
            break
    return {
        "wheel_contact": True,
        "contact_start_s": start,
        "contact_end_s": end,
        "rpm_drop": {side: target_rpm - minimum[side] for side in minimum},
        "maximum_rpm_drop": max(target_rpm - value for value in minimum.values()),
        "recovery_to_95pct_s": recovery,
    }


def pair_lock(observer: list[dict[str, str]]) -> dict:
    by_time: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in observer:
        by_time[float(row["time_s"])].append(row)
    active: list[float] = []
    longest: list[float] = []
    minimum_distance = math.inf
    for time, rows in sorted(by_time.items()):
        if time < 2.0 or len(rows) != 2:
            continue
        distance = math.sqrt(sum((float(rows[0][key]) - float(rows[1][key])) ** 2
                                 for key in ("world_x_m", "world_y_m", "world_z_m")))
        minimum_distance = min(minimum_distance, distance)
        in_throat = all(.38 <= float(row["base_x_m"]) <= .62 for row in rows)
        locked = distance <= .068 and in_throat and all(speed(row) <= .15 for row in rows)
        if locked and (not active or time - active[-1] <= .011):
            active.append(time)
        elif locked:
            active = [time]
        else:
            if len(active) > len(longest):
                longest = active
            active = []
    if len(active) > len(longest):
        longest = active
    duration = 0.0 if not longest else longest[-1] - longest[0] + .005
    return {
        "minimum_centre_distance_mm": None if not math.isfinite(minimum_distance) else 1000 * minimum_distance,
        "stationary_pair_lock_duration_s": duration,
        "stationary_pair_lock": duration >= .20,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", required=True)
    ap.add_argument("--case", required=True)
    ap.add_argument("--tread-mu", type=float, choices=(.6, .9), required=True)
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
        raise SystemExit("empty observer or telemetry")
    by_name: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in observer:
        by_name[row["ball_name"]].append(row)
    if len(by_name) != len(args.ball_contact):
        raise SystemExit(f"expected {len(args.ball_contact)} balls; observed {sorted(by_name)}")

    last_t = max(float(row["time_s"]) for row in observer)
    per_ball = {}
    for index, (name, rows) in enumerate(sorted(by_name.items())):
        active = [row for row in rows if float(row["time_s"]) >= 2.0]
        nip = next((row for row in active if float(row["base_x_m"]) <= .409), None)
        takeoff = next((row for row in active if float(row["base_x_m"]) <= .3288
                        and float(row["base_z_m"]) >= .0543), None)
        entry = next((row for row in active if float(row["base_x_m"]) <= .274
                      and abs(float(row["base_y_m"])) <= .139
                      and float(row["base_z_m"]) >= .075), None)
        tail = [row for row in rows if float(row["time_s"]) >= last_t - .5]
        means = {key: sum(float(row[key]) for row in tail) / len(tail)
                 for key in ("world_x_m", "world_y_m", "world_z_m", "base_x_m", "base_y_m", "base_z_m")}
        relative_tail_speed = mean_relative_speed(tail)
        retained = (-.052 <= means["base_x_m"] <= .245 and abs(means["base_y_m"]) <= .139
                    and .030 <= means["base_z_m"] <= .375 and relative_tail_speed <= .25)
        initial = rows[0]
        max_forward = max(float(row["world_x_m"]) - float(initial["world_x_m"]) for row in rows)
        final_speed = mean_tail_speed(tail)
        free_on_court = (not retained and means["world_z_m"] <= .070 and final_speed <= .25)
        per_ball[name] = {
            "nip_capture": nip is not None,
            "ski_takeoff": takeoff is not None,
            "basket_entry": entry is not None,
            "retained": retained,
            "nip_time_s": None if nip is None else float(nip["time_s"]),
            "takeoff_time_s": None if takeoff is None else float(takeoff["time_s"]),
            "basket_entry_time_s": None if entry is None else float(entry["time_s"]),
            "takeoff_speed_m_s": None if takeoff is None else speed(takeoff),
            "initial_world_xyz_m": [float(initial[key]) for key in ("world_x_m", "world_y_m", "world_z_m")],
            "final_mean_world_xyz_m": [means[key] for key in ("world_x_m", "world_y_m", "world_z_m")],
            "final_mean_base_xyz_m": [means[key] for key in ("base_x_m", "base_y_m", "base_z_m")],
            "final_mean_speed_m_s": final_speed,
            "final_mean_relative_speed_m_s": relative_tail_speed,
            "maximum_forward_push_m": max_forward,
            "free_on_court": free_on_court,
            "flywheel_response": wheel_response(telemetry, wheel_window(args.ball_contact[index])),
        }

    launcher = ("cad_upper_plate", "cad_yoke_", "cad_trunnion_", "d5065_motor_", "flywheel_left", "flywheel_right")
    fixed = ("fixture_arch_", "fixture_lidar_", "fixture_camera_", "chassis_", "ground_plane")
    forbidden = []
    for row in native:
        one, two = row.get("collision_1", ""), row.get("collision_2", "")
        pair = one + " " + two
        if "capture_ball_" in pair and any(token in pair for token in fixed[:-1]):
            forbidden.append(row)
        elif ((any(token in one for token in launcher) and any(token in two for token in fixed)) or
              (any(token in two for token in launcher) and any(token in one for token in fixed))):
            forbidden.append(row)
    nonfinite = sum(not math.isfinite(float(value)) for row in observer for key, value in row.items()
                    if key != "ball_name" and value)
    nonfinite += sum(not math.isfinite(float(value)) for row in telemetry for value in row.values() if value)
    max_current = max(abs(float(row[key])) for row in telemetry for key in ("left_current_a", "right_current_a"))
    jam_rows = [row for row in telemetry if int(float(row.get("jam_detected", 0) or 0))]
    lock = pair_lock(observer) if len(by_name) == 2 else {
        "minimum_centre_distance_mm": None, "stationary_pair_lock_duration_s": 0.0,
        "stationary_pair_lock": False,
    }
    side_case = args.case.startswith("side_y") or args.case.startswith("diag_")
    jammed = side_case and (bool(jam_rows) or lock["stationary_pair_lock"])
    rejected = [row for row in per_ball.values() if not row["retained"]]
    rejection_ok = all(row["maximum_forward_push_m"] <= .300 + 1e-9 and row["free_on_court"]
                       for row in rejected)
    electrical_ok = max_current <= 20.0 + 1e-9
    base_checks = {
        "no_jam": not jammed,
        "rejected_ball_free_and_not_pushed": rejection_ok,
        "d5065_contact_current": electrical_ok,
        "d5065_contact_torque": .031 * max_current <= .62 + 1e-9,
        "forbidden_contact": not forbidden,
        "finite_state": nonfinite == 0,
    }
    if args.case.startswith("series_gap_"):
        base_checks["series_both_retained"] = len(per_ball) == 2 and all(row["retained"] for row in per_ball.values())
    elif args.case.startswith("single_y"):
        base_checks["single_retained"] = len(per_ball) == 1 and next(iter(per_ball.values()))["retained"]
    takeoffs = sorted((row["takeoff_time_s"], row["takeoff_speed_m_s"])
                      for row in per_ball.values() if row["takeoff_time_s"] is not None)
    result = {
        "pass": all(base_checks.values()),
        "variant": args.variant,
        "case": args.case,
        "tread_mu": args.tread_mu,
        "checks": base_checks,
        "metrics": {
            "ball_count": len(per_ball),
            "balls_in_basket": sum(row["retained"] for row in per_ball.values()),
            "balls_rejected": len(rejected),
            "balls_free_on_court": sum(row["free_on_court"] for row in rejected),
            "maximum_rejected_forward_push_m": max((row["maximum_forward_push_m"] for row in rejected), default=0.0),
            "jammed": jammed,
            "d5065_detector_triggered": bool(jam_rows),
            "first_detector_trigger_s": None if not jam_rows else float(jam_rows[0]["jam_trigger_s"]),
            "pair_lock": lock,
            "per_ball": per_ball,
            "second_ball_exit_speed_m_s": takeoffs[1][1] if len(takeoffs) == 2 else None,
            "max_current_a": max_current,
            "max_torque_nm": .031 * max_current,
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
