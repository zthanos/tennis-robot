#!/usr/bin/env python3
"""Analyze one P2-51 passive ski-jump / empty-basket run."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return [r for r in csv.DictReader(f) if None not in r.values()]


def speed_vector(rows: list[dict[str, str]], i: int) -> tuple[float, float, float]:
    lo, hi = max(0, i - 2), min(len(rows) - 1, i + 2)
    dt = float(rows[hi]["time_s"]) - float(rows[lo]["time_s"])
    if dt <= 0:
        return 0.0, 0.0, 0.0
    return tuple((float(rows[hi][k]) - float(rows[lo][k])) / dt
                 for k in ("ball_base_x_m", "ball_base_y_m", "ball_base_z_m"))


def episodes(times: list[float], gap: float = 0.020) -> list[tuple[float, float]]:
    values = sorted(set(times))
    if not values:
        return []
    out, start, previous = [], values[0], values[0]
    for now in values[1:]:
        if now - previous > gap:
            out.append((start, previous)); start = now
        previous = now
    out.append((start, previous))
    return out


def current_window(rows: list[dict[str, str]], start: float, end: float) -> dict:
    selected = [r for r in rows if start - .01 <= float(r["time_s"]) <= end + .01]
    left = [abs(float(r["left_current_a"])) for r in selected]
    right = [abs(float(r["right_current_a"])) for r in selected]
    return {"start_s": start, "end_s": end, "samples": len(selected),
            "left_max_a": max(left, default=None), "right_max_a": max(right, default=None),
            "left_mean_a": sum(left)/len(left) if left else None,
            "right_mean_a": sum(right)/len(right) if right else None,
            "saturation_time_s": .005 * sum(max(l, r) >= 19.999 for l, r in zip(left, right))}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry", type=Path)
    ap.add_argument("ball_contacts", type=Path)
    ap.add_argument("native_contacts", type=Path)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--angle", type=int, choices=(35,42,45,50,55,60), required=True)
    ap.add_argument("--surface-speed", type=float, required=True)
    ap.add_argument("--tread-mu", type=float, required=True)
    ap.add_argument("--ramp-mu", type=float, required=True)
    ap.add_argument("--lateral", type=float, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    criteria = json.loads(a.criteria.read_text())
    rows, bc, native = read(a.telemetry), read(a.ball_contacts), read(a.native_contacts)
    active = [r for r in rows if float(r["time_s"]) >= 2.0]
    tip = {35:(.339982,.023564),42:(.332815,.029264),45:(.329967,.031967),
           50:(.325547,.036791),55:(.321564,.041982),60:(.318048,.047500)}[a.angle]
    nip_i = next((i for i,r in enumerate(active) if float(r["ball_launcher_x_m"]) <= -.033), None)
    ramp_base_i = next((i for i,r in enumerate(active)
                        if nip_i is not None and i >= nip_i
                        and float(r["ball_base_x_m"]) <= .430), None)
    takeoff_i = next((i for i,r in enumerate(active)
                      if float(r["ball_base_x_m"]) <= tip[0]-.004
                      and float(r["ball_base_z_m"]) >= tip[1]+.025), None)
    entry_i = next((i for i,r in enumerate(active)
                    if float(r["ball_base_x_m"]) <= .274
                    and abs(float(r["ball_base_y_m"])) <= .139
                    and float(r["ball_base_z_m"]) >= .075), None)

    basket_tokens = ("basket_p251_", "basket_left_wall_", "basket_right_wall_")
    fixture_tokens = ("fixture_arch_", "fixture_lidar_", "fixture_camera_", "chassis_")
    basket_contacts, forbidden = [], []
    for r in native:
        one, two = r.get("collision_1") or "", r.get("collision_2") or ""
        pair = one + " " + two
        if "capture_ball" in pair and any(t in pair for t in basket_tokens): basket_contacts.append(r)
        if "capture_ball" in pair and any(t in pair for t in fixture_tokens): forbidden.append(r)
        if ("flywheel_" in pair or "cad_yoke_" in pair or "cad_upper_plate" in pair) and any(t in pair for t in fixture_tokens): forbidden.append(r)
    first_basket_t = min((float(r["time_s"]) for r in basket_contacts), default=None)
    before_contact = [r for r in active if first_basket_t is None or float(r["time_s"]) <= first_basket_t]
    max_center_z = max((float(r["ball_base_z_m"]) for r in before_contact), default=math.inf)
    max_ball_top = max_center_z + .033

    ramp_base_v = speed_vector(active, ramp_base_i) if ramp_base_i is not None else None
    ramp_base_speed = math.sqrt(sum(x*x for x in ramp_base_v)) if ramp_base_v else None
    takeoff_v = speed_vector(active, takeoff_i) if takeoff_i is not None else None
    takeoff_speed = math.sqrt(sum(x*x for x in takeoff_v)) if takeoff_v else None
    takeoff_angle = math.degrees(math.atan2(takeoff_v[2], -takeoff_v[0])) if takeoff_v and takeoff_v[0] < 0 else None
    first_contact_row = min(active, key=lambda r: abs(float(r["time_s"])-first_basket_t)) if first_basket_t is not None else None

    tail = rows[-100:] if len(rows) >= 100 else rows
    final_mean = {k: sum(float(r[k]) for r in tail)/len(tail) for k in
                  ("ball_base_x_m","ball_base_y_m","ball_base_z_m")}
    tail_speed = []
    for i in range(2, len(tail)-2):
        tail_speed.append(math.sqrt(sum(x*x for x in speed_vector(tail,i))))
    retained = (-.085+.033 <= final_mean["ball_base_x_m"] <= .278-.033
                and abs(final_mean["ball_base_y_m"]) <= .172-.033
                and .030 <= final_mean["ball_base_z_m"] <= .375
                and (sum(tail_speed)/len(tail_speed) if tail_speed else math.inf) <= .25)

    wheel_times = [float(r["time_s"]) for r in bc
                   if int(float(r["contact_id"])) in (0,1) and float(r["normal_force_n"])>.01]
    eps = episodes(wheel_times)
    windows = [current_window(rows,s,e) for s,e in eps]
    max_current = max((max(w["left_max_a"],w["right_max_a"]) for w in windows if w["samples"]), default=math.inf)
    nonfinite = sum(not math.isfinite(float(v)) for r in rows for v in r.values() if v not in (None,""))
    plough = nip_i is None and max(float(r["ball_base_x_m"]) for r in active) > .80
    checks = {
        "nip_capture": nip_i is not None and not plough,
        "ski_takeoff": takeoff_i is not None,
        "d5065_contact_current": max_current <= 20.0+1e-9,
        "d5065_contact_torque": .031*max_current <= .62+1e-9,
        "forbidden_contact": not forbidden,
        "finite_state": nonfinite == 0,
        "basket_entry": entry_i is not None,
        "lidar_margin": max_ball_top <= .380,
        "retained_empty_basket": retained,
    }
    hard = {r["name"] for r in criteria["criteria"] if r["hard_fail"]}
    out = {
        "pass": all(checks.get(k,False) for k in hard),
        "delivery_success": all(checks[k] for k in ("nip_capture","ski_takeoff","basket_entry","lidar_margin","retained_empty_basket","forbidden_contact","finite_state")),
        "condition": {"angle_deg":a.angle,"surface_speed_m_s":a.surface_speed,"tread_mu":a.tread_mu,"ramp_mu":a.ramp_mu,"lateral_m":a.lateral},
        "checks": checks,
        "metrics": {
            "nip_time_s": None if nip_i is None else float(active[nip_i]["time_s"]),
            "ramp_base_time_s": None if ramp_base_i is None else float(active[ramp_base_i]["time_s"]),
            "ramp_base_speed_m_s": ramp_base_speed,
            "takeoff_time_s": None if takeoff_i is None else float(active[takeoff_i]["time_s"]),
            "takeoff_speed_m_s": takeoff_speed, "takeoff_angle_deg": takeoff_angle,
            "ski_speed_change_m_s": None if ramp_base_speed is None or takeoff_speed is None else takeoff_speed-ramp_base_speed,
            "ski_speed_retention_ratio": None if ramp_base_speed in (None,0) or takeoff_speed is None else takeoff_speed/ramp_base_speed,
            "basket_entry_time_s": None if entry_i is None else float(active[entry_i]["time_s"]),
            "first_basket_contact_time_s": first_basket_t,
            "first_basket_contact_xyz_m": None if first_contact_row is None else [float(first_contact_row[k]) for k in ("ball_base_x_m","ball_base_y_m","ball_base_z_m")],
            "highest_ball_center_z_m": max_center_z if math.isfinite(max_center_z) else None,
            "highest_ball_top_z_m": max_ball_top if math.isfinite(max_ball_top) else None,
            "lidar_margin_m": .390-max_ball_top if math.isfinite(max_ball_top) else None,
            "contact_current_episodes": windows,
            "contact_max_current_a": max_current if math.isfinite(max_current) else None,
            "contact_max_torque_nm": .031*max_current if math.isfinite(max_current) else None,
            "final_mean_xyz_m": [final_mean[k] for k in ("ball_base_x_m","ball_base_y_m","ball_base_z_m")],
            "final_mean_speed_m_s": sum(tail_speed)/len(tail_speed) if tail_speed else None,
            "basket_contact_count": len(basket_contacts), "forbidden_contact_count":len(forbidden),
            "nonfinite_count":nonfinite,
        },
        "criteria_status":criteria["status"],
    }
    a.output.write_text(json.dumps(out,indent=2)+"\n")
    print(json.dumps(out,indent=2))


if __name__ == "__main__": main()
