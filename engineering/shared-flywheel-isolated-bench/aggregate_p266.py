#!/usr/bin/env python3
"""Aggregate P2-66 without hiding material, lateral or restitution failures."""
from __future__ import annotations

import json
import os
import csv
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(os.environ.get("P2_66_RESULT_ROOT",
    "engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p266"))


def bounds(values: list[float]) -> dict:
    return ({"min": min(values), "mean": sum(values)/len(values), "max": max(values)}
            if values else {})


rows = []
for path in sorted((ROOT / "matrix").glob("a_*/r_*/v_*/tm_*/y_*/e_*/summary.json")):
    data = json.loads(path.read_text())
    metrics = data["metrics"]
    native = list(csv.DictReader((path.parent / "native_contacts.csv").open()))
    telemetry = list(csv.DictReader((path.parent / "telemetry.csv").open()))
    wall_times = [float(item["time_s"]) for item in native
                  if "capture_ball" in (item.get("collision_1", "") + item.get("collision_2", ""))
                  and "p266_front_wall" in (item.get("collision_1", "") + item.get("collision_2", ""))]
    safe_wall_top = None
    if wall_times and telemetry:
        first = min(wall_times)
        sample = min(telemetry, key=lambda item: abs(float(item["time_s"]) - first))
        # Ball bottom minus the required 15 mm clearance, in base coordinates.
        safe_wall_top = 1000*float(sample["ball_base_z_m"]) - 33 - 15
    rows.append({**data["condition"], "pass": data["pass"],
                 "failed_checks": [k for k, v in data["checks"].items() if not v],
                 **metrics, "max_wall_top_for_15mm_clearance_mm": safe_wall_top,
                 "summary_path": str(path)})

groups: dict[tuple[int, int], list[dict]] = defaultdict(list)
for row in rows:
    groups[(row["angle_deg"], row["radius_mm"])].append(row)

geometry = []
for (angle, radius), items in sorted(groups.items()):
    geometry.append({
        "angle_deg": angle, "radius_mm": radius, "conditions": len(items),
        "passes": sum(row["pass"] for row in items),
        "actual_angle_deg": bounds([row["takeoff_angle_deg"] for row in items if row["takeoff_angle_deg"] is not None]),
        "takeoff_speed_m_s": bounds([row["takeoff_speed_m_s"] for row in items if row["takeoff_speed_m_s"] is not None]),
        "max_wall_top_for_15mm_clearance_mm": bounds([
            row["max_wall_top_for_15mm_clearance_mm"] for row in items
            if row["max_wall_top_for_15mm_clearance_mm"] is not None]),
        "impact_xyz_m": [row["deflector_impact_xyz_m"] for row in items if row["deflector_impact_xyz_m"] is not None],
        "failed_check_counts": dict(Counter(x for row in items for x in row["failed_checks"])),
    })

report = {
    "phase": "P2-66", "status": "COMPLETE" if len(rows) == 540 else "INCOMPLETE",
    "conditions": len(rows), "passes": sum(row["pass"] for row in rows),
    "failed_check_counts": dict(Counter(x for row in rows for x in row["failed_checks"])),
    "geometry_table": geometry, "rows": rows,
    "notes": [
        "One ball and an empty basket only; no pile simulation was run.",
        "Deflector restitution 0.1/0.3/0.5 is a simulation sweep, not a material measurement.",
        "Physical EVA/net restitution must be measured before release.",
        "The rejected closed overhead corridor/ramp was not modelled or tested.",
    ],
}
(ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: report[k] for k in ("status", "conditions", "passes", "failed_check_counts", "geometry_table")}, indent=2))
