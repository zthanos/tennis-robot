#!/usr/bin/env python3
"""Aggregate the frozen nine-condition P2-50 one-ball matrix."""
from __future__ import annotations

import csv
import argparse
import json
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--root", type=Path, default=Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p250"))
a = ap.parse_args()
ROOT = a.root
rows = []
missing = []
for speed in (2.8, 3.0, 3.2):
    for lateral in (-0.05, 0.0, 0.05):
        tag = f"v_{str(speed).replace('.', 'p')}/y_{str(lateral).replace('-', 'm').replace('.', 'p')}"
        path = ROOT / tag / "summary.json"
        if not path.exists():
            missing.append(str(path))
            continue
        data = json.loads(path.read_text())
        m = data["metrics"]
        rows.append({
            "surface_speed_m_s": speed, "lateral_m": lateral,
            "pass": data["pass"], "sill_reached": data["checks"]["sill_reached"],
            "ramp_base_speed_m_s": m["ramp_base_speed_relative_m_s"],
            "transition_exit_speed_m_s": m.get("transition_exit_speed_relative_m_s"),
            "sill_speed_m_s": m["sill_speed_relative_m_s"],
            "furthest_x_mm": None if m.get("furthest_ramp_progress") is None else 1000*m["furthest_ramp_progress"]["ball_base_xyz_m"][0],
            "furthest_z_mm": None if m.get("furthest_ramp_progress") is None else 1000*m["furthest_ramp_progress"]["ball_base_xyz_m"][2],
            "transition_gap_mm": None if m["transition_max_airborne_gap_m"] is None else 1000*m["transition_max_airborne_gap_m"],
            "transition_force_n_diag": m["transition_contact_force_max_n_diagnostic"],
            "throat_abs_y_mm": None if m["throat_max_abs_y_m"] is None else 1000*m["throat_max_abs_y_m"],
            "contact_max_current_a": m["contact_max_current_a"],
            "contact_left_mean_a": m["contact_current"]["left_mean_a"],
            "contact_right_mean_a": m["contact_current"]["right_mean_a"],
            "contact_saturation_s": m["contact_current"]["saturation_time_s"],
        })

summary = {
    "status": "INCOMPLETE" if missing else "COMPLETE",
    "stage": "P2-50 one-ball nip-to-Z285 diagnostic",
    "conditions": len(rows), "passes": sum(bool(row["pass"]) for row in rows),
    "failures": sum(not bool(row["pass"]) for row in rows),
    "smooth_transition_required": any(row["transition_gap_mm"] is None or row["transition_gap_mm"] > 20 for row in rows),
    "rows": rows, "missing": missing,
}
ROOT.mkdir(parents=True, exist_ok=True)
(ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
with (ROOT / "condition_matrix.csv").open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["surface_speed_m_s"])
    writer.writeheader(); writer.writerows(rows)
print(json.dumps(summary, indent=2))
