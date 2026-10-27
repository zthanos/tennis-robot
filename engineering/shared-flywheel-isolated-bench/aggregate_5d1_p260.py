#!/usr/bin/env python3
"""Aggregate P2-60 saturation matrix and analytical traction bound."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p260")
CRITERIA = Path("engineering/shared-flywheel-isolated-bench/criteria_5d1_p260.json")
MASS = 0.058
K = 107309.294
R_WHEEL = 0.100
R_BALL = 0.033
DT = 0.001


def case_dir(compression: int, mu: float, feed: int) -> Path:
    return ROOT / "cases" / f"compression_{compression}" / f"tm_{str(mu).replace('.', 'p')}" / f"feed_{feed}"


def analytical(compression_mm: int, mu: float, vin: float) -> dict:
    gap = (66.0-compression_mm)/1000.0
    centre_offset = R_WHEEL + gap/2.0
    half = math.sqrt((R_WHEEL+R_BALL)**2-centre_offset**2)
    # Deterministic Simpson integration of the per-wheel Hertz-like force.
    n = 20000
    h = 2.0*half/n
    integral = 0.0
    peak = K*(compression_mm/2000.0)**1.5
    for i in range(n+1):
        x = -half+i*h
        delta = max(0.0, R_WHEEL+R_BALL-math.sqrt(x*x+centre_offset*centre_offset))
        value = K*delta**1.5
        integral += value*(1 if i in (0, n) else 4 if i % 2 else 2)
    integral *= h/3.0
    v_bound = math.sqrt(vin*vin + 4.0*mu*integral/MASS)
    return {"contact_length_m": 2.0*half, "peak_normal_force_per_wheel_n": peak,
            "integral_normal_force_dx_per_wheel_n_m": integral,
            "traction_work_bound_j": 2.0*mu*integral,
            "exit_speed_work_bound_m_s": v_bound,
            "momentum_bound_kg_m_s": MASS*v_bound}


def contact_integrals(path: Path, release_s: float, mu: float) -> dict:
    by_time: dict[float, list[dict[str, str]]] = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                time = float(row["time_s"]); cid = int(float(row["contact_id"]))
            except (ValueError, TypeError):
                continue
            if time+1e-9 < release_s or cid not in (0, 1):
                continue
            by_time.setdefault(time, []).append(row)
    normal_capacity = actual_tangent = normal_forward = 0.0
    peak_sum_n = 0.0
    cx, sx = math.cos(math.radians(20)), math.sin(math.radians(20))
    active_times = []
    for time, rows in sorted(by_time.items()):
        sum_n = sum(float(row["normal_force_n"]) for row in rows)
        if sum_n <= 1e-3:
            continue
        active_times.append(time); peak_sum_n = max(peak_sum_n, sum_n)
        normal_capacity += mu*sum_n*DT
        actual_tangent += sum(
            float(row["tangential_force_x_n"])*cx + float(row["tangential_force_z_n"])*sx
            for row in rows)*DT
        normal_forward += sum(
            float(row["normal_force_n"])*(float(row["normal_x"])*cx + float(row["normal_z"])*sx)
            for row in rows)*DT
    return {"contact_duration_s": (active_times[-1]-active_times[0]+DT) if active_times else None,
            "peak_sum_normal_force_n": peak_sum_n,
            "integral_mu_sum_normal_dt_n_s": normal_capacity,
            "integral_forward_tangential_force_dt_n_s": actual_tangent,
            "integral_forward_normal_force_dt_n_s": normal_forward}


def main() -> None:
    ap = argparse.ArgumentParser(); ap.add_argument("--write-record-manifest", action="store_true"); args = ap.parse_args()
    rows = []
    for compression in (8, 11, 14):
        for mu in (.6, .9, 1.2):
            for feed in (0, 2, 4):
                folder = case_dir(compression, mu, feed)
                summary = json.loads((folder/"summary.json").read_text())
                metrics = summary["metrics"]
                bound = analytical(compression, mu, feed)
                integrals = contact_integrals(folder/"ball_contacts.csv", metrics["release_time_s"], mu)
                exit_speed = metrics["exit_speed_m_s"]
                local_vx = metrics["launcher_exit_velocity_m_s"][0] if metrics["launcher_exit_velocity_m_s"] else None
                rows.append({"compression_mm": compression, "tread_mu": mu, "feed_speed_m_s": feed,
                             "pass": summary["pass"], "exit_speed_m_s": exit_speed,
                             "launcher_exit_vx_m_s": local_vx,
                             "exit_elevation_deg": metrics["exit_elevation_deg"],
                             "rpm_drop": metrics["rpm_drop"],
                             "momentum_change_kg_m_s": MASS*(local_vx-feed) if local_vx is not None else None,
                             "analytical": bound, "gazebo_contact_integrals": integrals,
                             "record_present": (folder/"replay"/"state.tlog").exists()})

    selected = set()
    for row in rows:
        if not row["pass"] or row["exit_speed_m_s"] is None:
            selected.add((row["compression_mm"], row["tread_mu"], row["feed_speed_m_s"]))
    for mu in (.6, .9, 1.2):
        group = [row for row in rows if row["tread_mu"] == mu and row["exit_speed_m_s"] is not None]
        below = [row for row in group if row["exit_speed_m_s"] < 12]
        above = [row for row in group if row["exit_speed_m_s"] >= 12]
        if below:
            row = max(below, key=lambda item: item["exit_speed_m_s"])
            selected.add((row["compression_mm"], mu, row["feed_speed_m_s"]))
        if above:
            row = min(above, key=lambda item: item["exit_speed_m_s"])
            selected.add((row["compression_mm"], mu, row["feed_speed_m_s"]))
    manifest = ROOT/"record_manifest.txt"
    manifest.write_text("".join(f"{c} {mu} {f}\n" for c, mu, f in sorted(selected)))
    if args.write_record_manifest:
        print(json.dumps({"selected": len(selected), "manifest": str(manifest)}, indent=2)); return

    factor_effects = {}
    for name, values in (("compression_mm", (8, 11, 14)), ("tread_mu", (.6, .9, 1.2)), ("feed_speed_m_s", (0, 2, 4))):
        means = {str(value): sum(row["exit_speed_m_s"] for row in rows if row[name] == value)/9.0 for value in values}
        factor_effects[name] = {"mean_exit_speed_by_level_m_s": means,
                                "main_effect_range_m_s": max(means.values())-min(means.values())}
    achieved = [row for row in rows if row["exit_speed_m_s"] is not None and row["exit_speed_m_s"] >= 12]
    report = {"status": "COMPLETE", "diagnostic_only": True,
              "criteria_sha256": hashlib.sha256(CRITERIA.read_bytes()).hexdigest(),
              "run_count": len(rows), "exit_count": sum(row["exit_speed_m_s"] is not None for row in rows),
              "target_12m_s_count": len(achieved), "factor_effects": factor_effects,
              "minimum_target_combination": min(achieved, key=lambda row: (row["compression_mm"], row["tread_mu"], row["feed_speed_m_s"])) if achieved else None,
              "recording": {"selected": len(selected),
                            "present": sum((case_dir(c, mu, f)/"replay"/"state.tlog").exists() for c, mu, f in selected),
                            "manifest": str(manifest)},
              "rows": rows}
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({key: report[key] for key in ("status", "run_count", "exit_count", "target_12m_s_count", "factor_effects", "minimum_target_combination", "recording")}, indent=2))


if __name__ == "__main__": main()
