#!/usr/bin/env python3
"""Analyze one P2-50 single-ball common-throat / high-ramp run."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

RAMP_PIECEWISE = [(0.402, 0.010), (0.377, 0.01325), (0.352, 0.02345),
        (0.3247, 0.0451), (0.285, 0.065), (0.245, 0.105),
        (0.205, 0.150), (0.155, 0.215), (0.100, 0.285)]
RAMP_R125 = [(0.402, 0.010), (0.384603, 0.011216), (0.367545, 0.014842),
             (0.351158, 0.020807), (0.335760, 0.028994), (0.321652, 0.039244),
             (0.309107, 0.051359), (0.270, 0.09510), (0.220, 0.15105),
             (0.170, 0.20700), (0.120, 0.26295), (0.100, 0.285)]


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return [row for row in csv.DictReader(f) if None not in row.values()]


def ramp_z(x: float, ramp: list[tuple[float, float]]) -> float | None:
    for (xa, za), (xb, zb) in zip(ramp, ramp[1:]):
        if xb <= x <= xa:
            t = (xa - x) / (xa - xb)
            return za + t * (zb - za)
    return None


def relative_speed(rows: list[dict[str, str]], index: int) -> float:
    lo, hi = max(0, index - 1), min(len(rows) - 1, index + 1)
    if hi == lo:
        return 0.0
    dt = float(rows[hi]["time_s"]) - float(rows[lo]["time_s"])
    if dt <= 0:
        return 0.0
    return math.sqrt(sum(
        ((float(rows[hi][key]) - float(rows[lo][key])) / dt) ** 2
        for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m")
    ))


def current_window(rows: list[dict[str, str]], start: float | None, end: float | None) -> dict:
    if start is None or end is None:
        return {"samples": 0, "left_max_a": None, "right_max_a": None,
                "left_mean_a": None, "right_mean_a": None, "saturation_time_s": None}
    selected = [row for row in rows if start - 0.01 <= float(row["time_s"]) <= end + 0.01]
    left = [abs(float(row["left_current_a"])) for row in selected]
    right = [abs(float(row["right_current_a"])) for row in selected]
    saturated = [row for row in selected
                 if max(abs(float(row["left_current_a"])), abs(float(row["right_current_a"]))) >= 19.999]
    return {
        "samples": len(selected),
        "left_max_a": max(left) if left else None,
        "right_max_a": max(right) if right else None,
        "left_mean_a": sum(left) / len(left) if left else None,
        "right_mean_a": sum(right) / len(right) if right else None,
        "saturation_time_s": len(saturated) * 0.005,
    }


def contact_episodes(times: list[float], max_gap_s: float = 0.020) -> list[tuple[float, float]]:
    """Collapse repeated contact samples into physical contact episodes."""
    unique = sorted(set(times))
    if not unique:
        return []
    episodes: list[tuple[float, float]] = []
    start = previous = unique[0]
    for now in unique[1:]:
        if now - previous > max_gap_s:
            episodes.append((start, previous))
            start = now
        previous = now
    episodes.append((start, previous))
    return episodes


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry", type=Path)
    ap.add_argument("ball_contacts", type=Path)
    ap.add_argument("native_contacts", type=Path)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--surface-speed", type=float, required=True)
    ap.add_argument("--lateral", type=float, required=True)
    ap.add_argument("--ramp-profile", choices=("piecewise", "r125"), default="piecewise")
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    criteria = json.loads(a.criteria.read_text())
    ramp = RAMP_R125 if a.ramp_profile == "r125" else RAMP_PIECEWISE
    rows, ball_contacts, native = read(a.telemetry), read(a.ball_contacts), read(a.native_contacts)
    if not rows:
        raise SystemExit("empty telemetry")

    active = [row for row in rows if float(row["time_s"]) >= 2.0]
    nonfinite = sum(not math.isfinite(float(value)) for row in rows
                    for value in row.values() if value not in (None, ""))
    nip_i = next((i for i, row in enumerate(active) if float(row["ball_launcher_x_m"]) <= -0.033), None)
    # Ramp base is immediately after the common throat, not the end of the
    # lower transition.  Keep both measurements so transition losses remain
    # visible rather than being folded into the reported entry speed.
    base_i = next((i for i, row in enumerate(active)
                   if float(row["ball_base_x_m"]) <= 0.390
                   and float(row["ball_base_z_m"]) >= 0.045), None)
    transition_exit_i = next((i for i, row in enumerate(active)
                              if float(row["ball_base_x_m"]) <= 0.325
                              and float(row["ball_base_z_m"]) >= 0.065), None)
    sill_i = next((i for i, row in enumerate(active)
                   if float(row["ball_base_x_m"]) <= 0.110
                   and float(row["ball_base_z_m"]) >= 0.305
                   and abs(float(row["ball_base_y_m"])) <= 0.020), None)

    transition = []
    for row in active:
        x = float(row["ball_base_x_m"])
        if 0.245 <= x <= 0.352:
            z = ramp_z(x, ramp)
            if z is not None:
                transition.append((row, float(row["ball_base_z_m"]) - (z + 0.033)))
    max_gap = max((gap for _, gap in transition), default=math.inf)
    max_abs_y = max((abs(float(row["ball_base_y_m"])) for row in active
                     if 0.100 <= float(row["ball_base_x_m"]) <= 0.325), default=math.inf)

    wheel_times = [float(row["time_s"]) for row in ball_contacts
                   if int(float(row["contact_id"])) in (0, 1)
                   and float(row["normal_force_n"]) > 0.01]
    episodes = contact_episodes(wheel_times)
    contact_start, contact_end = episodes[0] if episodes else (None, None)
    current = current_window(rows, contact_start, contact_end)
    episode_currents = [current_window(rows, start, end) for start, end in episodes]
    finite_episode_maxima = [
        max(value for value in (window["left_max_a"], window["right_max_a"]) if value is not None)
        for window in episode_currents if window["samples"]
    ]
    contact_max_current = max(finite_episode_maxima, default=math.inf)
    contact_max_torque = 0.031 * contact_max_current

    launcher_tokens = ("cad_upper_plate", "cad_yoke_", "cad_trunnion_", "d5065_motor_",
                       "flywheel_left", "flywheel_right", "intake_cheek_")
    fixed_tokens = ("intake_fixed_ramp_", "fixture_arch_", "fixture_lidar_", "chassis_", "ground_plane")
    forbidden = []
    transition_force = 0.0
    transition_contact_samples = 0
    for row in native:
        one, two = row.get("collision_1") or "", row.get("collision_2") or ""
        if ((any(t in one for t in launcher_tokens) and any(t in two for t in fixed_tokens))
                or (any(t in two for t in launcher_tokens) and any(t in one for t in fixed_tokens))):
            forbidden.append(row)
        transition_tokens = (("2p249", "3p249", "4p249") if a.ramp_profile == "piecewise"
                             else tuple(f"{i}r125" for i in range(11)))
        if any(f"intake_fixed_ramp_floor_{token}" in one or f"intake_fixed_ramp_floor_{token}" in two for token in transition_tokens):
            transition_contact_samples += 1
            try:
                transition_force = max(transition_force, float(row["max_force_n"]))
            except ValueError:
                pass

    base_speed = relative_speed(active, base_i) if base_i is not None else None
    transition_exit_speed = relative_speed(active, transition_exit_i) if transition_exit_i is not None else None
    sill_speed = relative_speed(active, sill_i) if sill_i is not None else None
    post_base = active[base_i:] if base_i is not None else []
    apex_i = min(range(len(post_base)), key=lambda i: float(post_base[i]["ball_base_x_m"])) if post_base else None
    apex = post_base[apex_i] if apex_i is not None else None
    checks = {
        "sill_reached": sill_i is not None,
        "sill_speed": sill_speed is not None and sill_speed >= 0.30,
        "transition_bounce_gap": max_gap <= 0.020,
        "throat_lateral_containment": max_abs_y <= 0.040,
        "d5065_contact_current": contact_max_current <= 20.0 + 1e-9,
        "d5065_contact_torque": contact_max_torque <= 0.62 + 1e-9,
        "forbidden_launcher_contact_count": not forbidden,
        "finite_state": nonfinite == 0,
    }
    hard = {item["name"] for item in criteria["criteria"] if item["hard_fail"]}
    if hard - checks.keys():
        raise SystemExit(f"unmapped criteria: {sorted(hard-checks.keys())}")
    out = {
        "pass": all(checks[name] for name in hard),
        "condition": {"surface_speed_m_s": a.surface_speed, "lateral_m": a.lateral},
        "checks": checks,
        "metrics": {
            "nip_crossing_time_s": float(active[nip_i]["time_s"]) if nip_i is not None else None,
            "ramp_base_time_s": float(active[base_i]["time_s"]) if base_i is not None else None,
            "sill_time_s": float(active[sill_i]["time_s"]) if sill_i is not None else None,
            "ramp_base_speed_relative_m_s": base_speed,
            "transition_exit_speed_relative_m_s": transition_exit_speed,
            "sill_speed_relative_m_s": sill_speed,
            "furthest_ramp_progress": None if apex is None else {
                "time_s": float(apex["time_s"]),
                "ball_base_xyz_m": [float(apex[key]) for key in
                                    ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m")],
            },
            "transition_max_airborne_gap_m": max_gap if math.isfinite(max_gap) else None,
            "transition_contact_samples": transition_contact_samples,
            "transition_contact_force_max_n_diagnostic": transition_force,
            "throat_max_abs_y_m": max_abs_y if math.isfinite(max_abs_y) else None,
            "ball_wheel_contact_start_s": contact_start,
            "ball_wheel_contact_end_s": contact_end,
            "ball_wheel_contact_duration_s": contact_end-contact_start if contact_start is not None else None,
            "contact_current": current,
            "contact_episode_count": len(episodes),
            "contact_episodes": [
                {"start_s": start, "end_s": end, "duration_s": end-start,
                 "current": window}
                for (start, end), window in zip(episodes, episode_currents)
            ],
            "contact_max_current_a": contact_max_current if math.isfinite(contact_max_current) else None,
            "contact_max_torque_nm": contact_max_torque if math.isfinite(contact_max_torque) else None,
            "forbidden_contact_count": len(forbidden),
            "nonfinite_count": nonfinite,
            "final_ball_base_xyz_m": [float(rows[-1][key]) for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m")],
        },
        "criteria_status": criteria["status"],
        "forbidden_contact_examples": forbidden[:10],
    }
    a.output.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
