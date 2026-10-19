#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite(row: dict[str, str]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in row.values())
    except (TypeError, ValueError):
        return False


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return value


def local_state(row: dict[str, str]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    q, yaw = float(row["tilt_rad"]), float(row["base_yaw_rad"])
    cy, sy, cq, sq = math.cos(yaw), math.sin(yaw), math.cos(q), math.sin(q)
    ex = (cy*cq, sy*cq, sq)
    ey = (-sy, cy, 0.0)
    ez = (-cy*sq, -sy*sq, cq)
    bx, by, bz = (float(row[key]) for key in ("base_x_m", "base_y_m", "base_z_m"))
    origin = (bx + cy*(0.387 + 0.055*cq), by + sy*(0.387 + 0.055*cq), bz + 0.049837363636 + 0.055*sq)
    delta = tuple(float(row[key]) - origin[i] for i, key in enumerate(("ball_x_m", "ball_y_m", "ball_z_m")))
    velocity = tuple(float(row[key]) for key in ("ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s"))
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    return ((dot(delta, ex), dot(delta, ey), dot(delta, ez)),
            (dot(velocity, ex), dot(velocity, ey), dot(velocity, ez)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=int, required=True)
    ap.add_argument("--mu", type=float, required=True)
    ap.add_argument("--surface", type=float, required=True)
    ap.add_argument("--repeat", type=int, required=True)
    ap.add_argument("--nip-compression-mm", type=float, default=8.0)
    ap.add_argument("--feed-speed", type=float, default=0.6)
    ap.add_argument("--telemetry", type=Path, required=True)
    ap.add_argument("--contacts", type=Path, required=True)
    ap.add_argument("--ball-contacts", type=Path, required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    criteria = json.loads(args.criteria.read_text())
    telemetry = read(args.telemetry)
    ball_contacts = read(args.ball_contacts)
    native = read(args.contacts)
    release_rows = [row for row in telemetry if int(row["released"])]
    release_time = float(release_rows[0]["release_time_s"]) if release_rows else math.nan

    wheel_contacts = []
    for row in ball_contacts:
        try:
            if int(float(row["contact_id"])) in (0, 1) and float(row["normal_force_n"]) > 1.0:
                wheel_contacts.append(row)
        except (TypeError, ValueError):
            continue
    ids = {int(float(row["contact_id"])) for row in wheel_contacts}
    contact_start = min((float(row["time_s"]) for row in wheel_contacts), default=math.nan)
    contact_end = max((float(row["time_s"]) for row in wheel_contacts), default=math.nan)
    exit_candidates = []
    for row in release_rows:
        position, velocity = local_state(row)
        if position[0] >= criteria["per_run_checks"]["forward_exit_plane_launcher_x_m"] and velocity[0] > 0.1:
            exit_candidates.append((row, position, velocity))
    exit_row, exit_position, exit_velocity = exit_candidates[0] if exit_candidates else (None, None, None)
    if exit_row:
        vx, vy, vz = (float(exit_row[key]) for key in ("ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s"))
        wx, wy, wz = (float(exit_row[key]) for key in ("ball_wx_rad_s", "ball_wy_rad_s", "ball_wz_rad_s"))
        exit_speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        elevation = math.degrees(math.atan2(vz, math.hypot(vx, vy)))
        azimuth = math.degrees(math.atan2(vy, vx))
        spin = math.sqrt(wx*wx + wy*wy + wz*wz)
        exit_time = float(exit_row["time_s"])
    else:
        exit_speed = elevation = azimuth = spin = exit_time = math.nan

    forbidden_tokens = tuple(criteria["per_run_checks"]["forbidden_ball_contacts"])
    forbidden = []
    for row in native:
        if not row.get("collision_1") or not row.get("collision_2"):
            continue
        pair = row["collision_1"] + " " + row["collision_2"]
        if "launch_ball" in pair and any(token in pair for token in forbidden_tokens):
            forbidden.append(row)

    target_rpm = args.surface / 0.1 * 60.0 / (2.0*math.pi)
    contact_rows = [row for row in telemetry if wheel_contacts and contact_start <= float(row["time_s"]) <= contact_end]
    drops = {}
    for side in ("left", "right"):
        minimum = min((abs(float(row[f"{side}_speed_rad_s"]))*60/(2*math.pi) for row in contact_rows), default=math.nan)
        drops[side] = target_rpm - minimum if math.isfinite(minimum) else math.nan
    recovery = math.nan
    if wheel_contacts:
        for row in telemetry:
            if float(row["time_s"]) < contact_end: continue
            if all(abs(float(row[f"{side}_speed_rad_s"]))*60/(2*math.pi) >= 0.95*target_rpm for side in ("left", "right")):
                recovery = float(row["time_s"]) - contact_end; break

    current_before = [row for row in telemetry if not wheel_contacts or float(row["time_s"]) < contact_start]
    current_contact = contact_rows
    peak = lambda data, suffix: max((abs(float(row[suffix])) for row in data), default=math.nan)
    first = telemetry[0] if telemetry else None
    base_motion = max((math.hypot(float(row["base_x_m"])-float(first["base_x_m"]),
                                  float(row["base_y_m"])-float(first["base_y_m"])) for row in telemetry), default=math.nan) if first else math.nan
    yaw_motion = max((abs(float(row["base_yaw_rad"])-float(first["base_yaw_rad"])) for row in telemetry), default=math.nan) if first else math.nan
    checks = {
        "released": bool(release_rows),
        "settled_before_release": bool(release_rows) and int(release_rows[0]["settled"]) == 1,
        "bilateral_wheel_contact": ids == {0, 1},
        "forward_exit": exit_row is not None,
        "current_limit": all(peak(telemetry, f"{side}_current_a") <= 20.0001 for side in ("left", "right")),
        "torque_limit": all(peak(telemetry, f"{side}_torque_nm") <= 0.6201 for side in ("left", "right")),
        "stationary_translation": base_motion <= criteria["per_run_checks"]["maximum_vehicle_translation_m"],
        "stationary_yaw": math.degrees(yaw_motion) <= criteria["per_run_checks"]["maximum_vehicle_yaw_deg"],
        "forbidden_contact": not forbidden,
        "finite_state": bool(telemetry) and all(finite(row) for row in telemetry),
    }
    result = {
        "pass": all(checks.values()), "angle_deg": args.angle, "tread_mu": args.mu,
        "surface_speed_m_s": args.surface, "repeat": args.repeat,
        "nip_compression_mm": args.nip_compression_mm,
        "feed_speed_m_s": args.feed_speed, "checks": checks,
        "metrics": {
            "release_time_s": release_time, "wheel_contact_start_s": contact_start,
            "wheel_contact_end_s": contact_end, "exit_time_s": exit_time,
            "exit_speed_m_s": exit_speed, "exit_elevation_deg": elevation,
            "exit_azimuth_deg": azimuth, "launcher_exit_velocity_m_s": exit_velocity,
            "spin_magnitude_rad_s": spin,
            "spin_vector_rad_s": [float(exit_row[key]) for key in ("ball_wx_rad_s", "ball_wy_rad_s", "ball_wz_rad_s")] if exit_row else None,
            "rpm_target": target_rpm, "rpm_drop": drops, "recovery_to_95pct_s": recovery,
            "spinup_peak_current_a": {side: peak(current_before, f"{side}_current_a") for side in ("left", "right")},
            "contact_peak_current_a": {side: peak(current_contact, f"{side}_current_a") for side in ("left", "right")},
            "contact_peak_torque_nm": {side: peak(current_contact, f"{side}_torque_nm") for side in ("left", "right")},
            "vehicle_translation_m": base_motion, "vehicle_yaw_deg": math.degrees(yaw_motion),
            "forbidden_contact_count": len(forbidden),
        },
        "forbidden_contact_examples": forbidden[:10],
    }
    result = clean(result)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"pass": result["pass"], "exit_speed_m_s": exit_speed,
                      "elevation_deg": elevation, "spin_rad_s": spin,
                      "forbidden": len(forbidden)}))


if __name__ == "__main__":
    main()
