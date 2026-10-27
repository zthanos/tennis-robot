#!/usr/bin/env python3
"""Aggregate the single approved 30-condition P2-69 confirmation."""
from __future__ import annotations

import json
import os
from collections import Counter
from pathlib import Path


ROOT = Path(os.environ.get(
    "P2_69_RESULT_ROOT",
    "engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p269",
))


def bounds(values: list[float]) -> dict[str, float]:
    return {"min": min(values), "mean": sum(values) / len(values), "max": max(values)}


rows = []
for path in sorted((ROOT / "matrix").glob(
        "a_75/r_75/v_*/tm_*/y_*/e_0p3/summary.json")):
    data = json.loads(path.read_text())
    metrics = data["metrics"]
    rows.append({
        **data["condition"],
        "pass": data["pass"],
        "failed_checks": [key for key, value in data["checks"].items() if not value],
        **metrics,
        "summary_path": str(path),
    })

impacts = [row["deflector_impact_xyz_m"] for row in rows
           if row["deflector_impact_xyz_m"] is not None]
report = {
    "phase": "P2-69",
    "status": "COMPLETE" if len(rows) == 30 else "INCOMPLETE",
    "geometry": {
        "ski": "R75/75deg",
        "arch_crossmember_center_mm": [390, 0, 245],
        "deflector_xz_mm": [[250, 363], [360, 343]],
        "front_wall": {"x_mm": 225, "top_z_mm": 235, "thickness_mm": 4},
        "feed_door": {"clear_mm": [80, 80], "state": "closed_fail_closed"},
    },
    "conditions": len(rows),
    "passes": sum(row["pass"] for row in rows),
    "failed_check_counts": dict(Counter(
        check for row in rows for check in row["failed_checks"])),
    "actual_angle_deg": bounds([row["takeoff_angle_deg"] for row in rows
                                if row["takeoff_angle_deg"] is not None]),
    "takeoff_speed_m_s": bounds([row["takeoff_speed_m_s"] for row in rows
                                 if row["takeoff_speed_m_s"] is not None]),
    "pre_deflector_ball_top_m": bounds([row["pre_deflector_ball_top_m"] for row in rows]),
    "final_mean_speed_m_s": bounds([row["final_mean_speed_m_s"] for row in rows]),
    "deflector_impact_world_m": {
        "x": bounds([p[0] for p in impacts]),
        "y": bounds([p[1] for p in impacts]),
        "z": bounds([p[2] for p in impacts]),
    },
    "front_wall_contacts": sum(row["front_wall_contact_count"] for row in rows),
    "feed_door_contacts": sum(row["feed_door_contact_count"] for row in rows),
    "forbidden_contacts": sum(row["forbidden_contact_count"] for row in rows),
    "rows": rows,
    "notes": [
        "One ball and empty basket only; no pile simulation.",
        "Single e=0.3 confirmation because P2-67 established identical DART results across e.",
        "Firmware requirement: full-basket deceleration must be <= approximately 0.2 g.",
    ],
}
ROOT.mkdir(parents=True, exist_ok=True)
(ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({key: report[key] for key in (
    "status", "conditions", "passes", "failed_check_counts",
    "actual_angle_deg", "takeoff_speed_m_s", "front_wall_contacts",
    "feed_door_contacts", "forbidden_contacts")}, indent=2))
