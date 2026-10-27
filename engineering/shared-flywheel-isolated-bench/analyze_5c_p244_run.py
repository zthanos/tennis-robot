#!/usr/bin/env python3
"""Analyze one P2-44 open-ramp / basket-entry repetition."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return [row for row in csv.DictReader(f) if None not in row.values()]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry", type=Path)
    ap.add_argument("ball_contacts", type=Path)
    ap.add_argument("native_contacts", type=Path)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    criteria = json.loads(a.criteria.read_text())
    rows, ball_contacts, native = read(a.telemetry), read(a.ball_contacts), read(a.native_contacts)
    if not rows:
        raise SystemExit("empty telemetry")
    nonfinite = sum(
        not math.isfinite(float(value))
        for row in rows for value in row.values() if value not in (None, "")
    )
    active = [row for row in rows if float(row["time_s"]) >= 2.0]
    nip = next((row for row in active if float(row["ball_launcher_x_m"]) <= -0.033), None)
    basket = next((row for row in active
                   if float(row["ball_base_x_m"]) <= 0.278
                   and abs(float(row["ball_base_y_m"])) <= 0.015
                   and 0.075 <= float(row["ball_base_z_m"]) <= 0.125), None)
    nip_speed = -float(nip["ball_launcher_vx_m_s"]) if nip else None
    initial_x = float(rows[0]["ball_x_m"])
    initial_y = float(rows[0]["ball_y_m"])
    pre_nip = active if nip is None else [row for row in active if float(row["time_s"]) <= float(nip["time_s"])]
    plough = max(float(row["ball_x_m"]) - initial_x for row in pre_nip)
    lateral_push = max(abs(float(row["ball_y_m"]) - initial_y) for row in pre_nip)

    wheel_times = [float(row["time_s"]) for row in ball_contacts
                   if int(float(row["contact_id"])) in (0, 1) and float(row["normal_force_n"]) > 0]
    first_contact = min(wheel_times) if wheel_times else None
    longest = 0.0
    run_start = None
    previous = None
    for row in active:
        t = float(row["time_s"])
        p = tuple(float(row[key]) for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m"))
        speed = math.inf
        if previous is not None:
            pt, pp = previous
            dt = t - pt
            if dt > 0:
                speed = math.dist(p, pp) / dt
        previous = (t, p)
        stalled = (first_contact is not None and t >= first_contact
                   and p[0] > 0.278 and p[0] < 0.50 and speed < 0.05)
        if stalled and run_start is None:
            run_start = t
        if not stalled and run_start is not None:
            longest = max(longest, t - run_start)
            run_start = None
    if run_start is not None:
        longest = max(longest, float(active[-1]["time_s"]) - run_start)

    max_current = max(abs(float(row[key])) for row in rows for key in ("left_current_a", "right_current_a"))
    max_torque = max(abs(float(row[key])) for row in rows for key in ("left_torque_nm", "right_torque_nm"))
    launcher = ("cad_upper_plate", "cad_yoke_", "cad_trunnion_", "d5065_motor_",
                "flywheel_left", "flywheel_right", "intake_cheek_")
    fixed = ("intake_fixed_ramp_", "fixture_arch_", "fixture_lidar_", "chassis_", "ground_plane")
    forbidden = []
    ramp_contact_samples = 0
    ramp_contact_force = 0.0
    for row in native:
        one, two = row.get("collision_1") or "", row.get("collision_2") or ""
        if ((any(token in one for token in launcher) and any(token in two for token in fixed))
                or (any(token in two for token in launcher) and any(token in one for token in fixed))):
            forbidden.append(row)
        if "intake_fixed_ramp_" in one or "intake_fixed_ramp_" in two:
            ramp_contact_samples += 1
            try:
                ramp_contact_force = max(ramp_contact_force, float(row["max_force_n"]))
            except (TypeError, ValueError):
                pass
    checks = {
        "basket_entry_reached": basket is not None,
        "fully_through_nip": nip is not None,
        "plough": plough <= 0.1,
        "jam": longest <= 0.75,
        "d5065_current": max_current <= 20.0 + 1e-9,
        "d5065_torque": max_torque <= 0.62 + 1e-9,
        "forbidden_launcher_contact_count": not forbidden,
        "finite_state": nonfinite == 0,
    }
    hard_names = {item["name"] for item in criteria["criteria"] if item["hard_fail"]}
    unknown_hard = hard_names - set(checks) - {"condition_success_count"}
    if unknown_hard:
        raise SystemExit(f"criteria contain unmapped hard checks: {sorted(unknown_hard)}")
    out = {
        "pass": all(value for name, value in checks.items() if name in hard_names),
        "checks": checks,
        "metrics": {
            "nip_crossing_time_s": float(nip["time_s"]) if nip else None,
            "basket_entry_time_s": float(basket["time_s"]) if basket else None,
            "nip_exit_rearward_speed_m_s": nip_speed,
            "pre_nip_forward_displacement_m": plough,
            "pre_nip_lateral_displacement_m": lateral_push,
            "longest_jam_s": longest,
            "first_wheel_contact_time_s": first_contact,
            "max_current_a": max_current,
            "max_torque_nm": max_torque,
            "forbidden_contact_count": len(forbidden),
            "ramp_contact_samples": ramp_contact_samples,
            "ramp_contact_force_max_n_diagnostic": ramp_contact_force,
            "final_ball_base_xyz_m": [float(rows[-1][key]) for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m")],
            "nonfinite_count": nonfinite,
        },
        "criteria_status": criteria["status"],
        "forbidden_contact_examples": forbidden[:10],
    }
    a.output.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
