#!/usr/bin/env python3
"""Aggregate the complete 180-condition P2-67 matrix."""
from __future__ import annotations

import csv
import json
import os
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(os.environ.get("P2_67_RESULT_ROOT",
    "engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p267"))


def bounds(values: list[float]) -> dict:
    return ({"min": min(values), "mean": sum(values)/len(values), "max": max(values)}
            if values else {})


rows = []
for path in sorted((ROOT / "matrix").glob("a_*/r_75/v_*/tm_*/y_*/e_*/summary.json")):
    data = json.loads(path.read_text())
    metrics = data["metrics"]
    rows.append({**data["condition"], "pass": data["pass"],
                 "failed_checks": [k for k, v in data["checks"].items() if not v],
                 **metrics, "summary_path": str(path)})

groups: dict[int, list[dict]] = defaultdict(list)
for row in rows:
    groups[row["angle_deg"]].append(row)

geometry = []
for angle, items in sorted(groups.items()):
    impacts = [row["deflector_impact_xyz_m"] for row in items
               if row["deflector_impact_xyz_m"] is not None]
    geometry.append({
        "angle_deg": angle, "radius_mm": 75, "conditions": len(items),
        "passes": sum(row["pass"] for row in items),
        "actual_angle_deg": bounds([row["takeoff_angle_deg"] for row in items
                                    if row["takeoff_angle_deg"] is not None]),
        "takeoff_speed_m_s": bounds([row["takeoff_speed_m_s"] for row in items
                                     if row["takeoff_speed_m_s"] is not None]),
        "deflector_impacts": len(impacts),
        "impact_x_m": bounds([p[0] for p in impacts]),
        "impact_y_m": bounds([p[1] for p in impacts]),
        "impact_z_m": bounds([p[2] for p in impacts]),
        "failed_check_counts": dict(Counter(x for row in items for x in row["failed_checks"])),
    })

report = {
    "phase": "P2-67", "status": "COMPLETE" if len(rows) == 180 else "INCOMPLETE",
    "conditions": len(rows), "passes": sum(row["pass"] for row in rows),
    "failed_check_counts": dict(Counter(x for row in rows for x in row["failed_checks"])),
    "geometry_table": geometry, "rows": rows,
    "notes": [
        "R75 only; R110 rejected by user decision after P2-66.",
        "One ball and empty basket only; no pile simulation.",
        "EVA/net restitution remains a simulation sweep and needs physical measurement.",
    ],
}
ROOT.mkdir(parents=True, exist_ok=True)
(ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: report[k] for k in ("status", "conditions", "passes",
                                         "failed_check_counts", "geometry_table")}, indent=2))
