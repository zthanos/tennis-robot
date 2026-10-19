#!/usr/bin/env python3
"""Score one P2-66 empty-basket run against the frozen hard criteria."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return [row for row in csv.DictReader(stream) if None not in row.values()]


def speed(rows: list[dict[str, str]], index: int) -> tuple[float, float, float]:
    lo, hi = max(0, index - 2), min(len(rows) - 1, index + 2)
    dt = float(rows[hi]["time_s"]) - float(rows[lo]["time_s"])
    if dt <= 0:
        return (0.0, 0.0, 0.0)
    return tuple((float(rows[hi][key]) - float(rows[lo][key])) / dt
                 for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m"))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry", type=Path)
    ap.add_argument("native_contacts", type=Path)
    ap.add_argument("--radius-mm", type=int, required=True)
    ap.add_argument("--angle-deg", type=int, required=True)
    ap.add_argument("--surface-speed", type=float, required=True)
    ap.add_argument("--tread-mu", type=float, required=True)
    ap.add_argument("--lateral-mm", type=int, required=True)
    ap.add_argument("--restitution", type=float, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--phase", choices=("p266", "p267", "p269"), default="p266")
    a = ap.parse_args()

    rows, contacts = read(a.telemetry), read(a.native_contacts)
    active = [row for row in rows if float(row["time_s"]) >= 2.0]
    radius = a.radius_mm / 1000.0
    theta = math.radians(a.angle_deg)
    tip = (0.383 - radius * math.sin(theta), 0.010 + radius * (1-math.cos(theta)))
    ski_times = []
    for row in contacts:
        pair = (row.get("collision_1") or "") + " " + (row.get("collision_2") or "")
        if "capture_ball" in pair and f"{a.phase}_ski_" in pair:
            ski_times.append(float(row["time_s"]))
    # The high-angle ball can leave nearly vertically without ever crossing
    # rearward of the surface-tip X.  Measure take-off immediately after the
    # first continuous ski-contact episode instead of reusing that P2-51 gate.
    episode_end = None
    if ski_times:
        # Use the first continuous climb episode.  After a successful P2-67
        # deflection the ball may later touch the ski from above; that return
        # contact is not the launch take-off.
        ordered = sorted(set(ski_times))
        episode = [ordered[0]]
        for timestamp in ordered[1:]:
            if timestamp - episode[-1] > 0.030:
                break
            episode.append(timestamp)
        episode_end = episode[-1]
    takeoff_i = (min(range(len(active)), key=lambda i: abs(float(active[i]["time_s"]) - (episode_end + .005)))
                 if episode_end is not None else None)
    takeoff_v = speed(active, takeoff_i) if takeoff_i is not None else None
    takeoff_speed = math.sqrt(sum(v*v for v in takeoff_v)) if takeoff_v else None
    takeoff_angle = (math.degrees(math.atan2(takeoff_v[2], -takeoff_v[0]))
                     if takeoff_v and takeoff_v[0] < 0 and takeoff_v[2] > 0 else None)

    ball_contacts = []
    for row in contacts:
        pair = (row.get("collision_1") or "") + " " + (row.get("collision_2") or "")
        if "capture_ball" in pair:
            ball_contacts.append((row, pair))
    deflector = [(row, pair) for row, pair in ball_contacts if f"{a.phase}_soft_deflector" in pair]
    wall = [(row, pair) for row, pair in ball_contacts if f"{a.phase}_front_wall" in pair]
    door = [(row, pair) for row, pair in ball_contacts if f"{a.phase}_feed_door" in pair]
    forbidden_tokens = ("fixture_arch_", "fixture_lidar_", "fixture_camera_", "chassis_",
                        "p267_arch_", "p267_lidar_", "p267_camera_",
                        "p269_arch_", "p269_lidar_", "p269_camera_",
                        "cad_actuator", "cad_fit0186", "cad_hard_stop", "cad_yoke_")
    forbidden = [(row, pair) for row, pair in ball_contacts if any(t in pair for t in forbidden_tokens)]
    first_deflector_t = min((float(row["time_s"]) for row, _ in deflector), default=None)
    pre = [row for row in active if first_deflector_t is None or float(row["time_s"]) < first_deflector_t]
    max_top = max((float(row["ball_base_z_m"]) + .033 for row in pre), default=math.inf)
    impact = min(deflector, key=lambda item: float(item[0]["time_s"]))[0] if deflector else None

    tail = active[-100:] if len(active) >= 100 else active
    final_xyz = [sum(float(row[key]) for row in tail)/len(tail)
                 for key in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m")]
    tail_speeds = [math.sqrt(sum(v*v for v in speed(tail, i))) for i in range(2, len(tail)-2)]
    final_speed = sum(tail_speeds)/len(tail_speeds) if tail_speeds else math.inf
    # P2-67 keeps the exact P2-51 rear at X=-85 mm, so an R33 ball is inside
    # down to centre X=-52 mm.  P2-66 retains its original conservative gate.
    retained_x_min = -.052 if a.phase in ("p267", "p269") else -.018
    retained = (retained_x_min <= final_xyz[0] <= .245 and abs(final_xyz[1]) <= .139
                and .030 <= final_xyz[2] <= .340 and final_speed <= .25)
    basket_entry = any(float(row["ball_base_x_m"]) < .245 and
                       abs(float(row["ball_base_y_m"])) <= .139 and
                       float(row["ball_base_z_m"]) > .075 for row in active)
    finite = all(math.isfinite(float(value)) for row in rows for value in row.values() if value != "")
    checks = {
        "ski_takeoff": takeoff_angle is not None and takeoff_speed is not None and takeoff_speed > .5,
        "deflector_contact": bool(deflector),
        "basket_entry": basket_entry,
        "retained_empty_basket": retained,
        "front_wall_clearance": not wall,
        "feed_door_clearance": not door,
        "forbidden_contact": not forbidden,
        "pre_deflector_ball_top": max_top <= .380,
        "final_speed": final_speed <= .25,
        "finite_state": finite,
    }
    out = {
        "pass": all(checks.values()),
        "condition": {"angle_deg": a.angle_deg, "radius_mm": a.radius_mm,
                      "surface_speed_m_s": a.surface_speed, "tread_mu": a.tread_mu,
                      "lateral_mm": a.lateral_mm, "deflector_restitution": a.restitution},
        "checks": checks,
        "metrics": {
            "takeoff_speed_m_s": takeoff_speed, "takeoff_angle_deg": takeoff_angle,
            "takeoff_time_s": None if takeoff_i is None else float(active[takeoff_i]["time_s"]),
            "deflector_impact_xyz_m": None if impact is None else
                [float(impact[key]) for key in ("point_x_m", "point_y_m", "point_z_m")],
            "deflector_contact_count": len(deflector), "front_wall_contact_count": len(wall),
            "feed_door_contact_count": len(door),
            "forbidden_contact_count": len(forbidden), "pre_deflector_ball_top_m": max_top,
            "final_mean_xyz_m": final_xyz, "final_mean_speed_m_s": final_speed,
        },
    }
    a.output.write_text(json.dumps(out, indent=2) + "\n")
    print(("PASS" if out["pass"] else "FAIL"), json.dumps(out["metrics"]))


if __name__ == "__main__":
    main()
