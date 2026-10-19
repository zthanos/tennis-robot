#!/usr/bin/env python3
"""Aggregate the frozen P2-51 R75/42-degree delivery matrix."""
from __future__ import annotations

import json
import os
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(os.environ.get("P2_51_AGG_ROOT", "engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_delivery_matrix"))


def bounds(values: list[float]) -> dict:
    return {"min": min(values), "mean": sum(values) / len(values), "max": max(values)} if values else {}


rows = []
for path in sorted(ROOT.glob("a_42/v_*/tm_*/rm_*/y_*/summary.json")):
    data = json.loads(path.read_text())
    metrics = data["metrics"]
    failed = [name for name, passed in data["checks"].items() if not passed]
    rows.append({
        **data["condition"],
        "pass": data["pass"],
        "delivery_success": data["delivery_success"],
        "failed_checks": failed,
        "takeoff_speed_m_s": metrics["takeoff_speed_m_s"],
        "takeoff_angle_deg": metrics["takeoff_angle_deg"],
        "ramp_base_speed_m_s": metrics.get("ramp_base_speed_m_s"),
        "ski_speed_change_m_s": metrics.get("ski_speed_change_m_s"),
        "ski_speed_retention_ratio": metrics.get("ski_speed_retention_ratio"),
        "lidar_margin_mm": None if metrics["lidar_margin_m"] is None else 1000 * metrics["lidar_margin_m"],
        "contact_max_current_a": metrics["contact_max_current_a"],
        "contact_max_torque_nm": metrics["contact_max_torque_nm"],
        "first_basket_contact_xyz_m": metrics["first_basket_contact_xyz_m"],
    })

groups = defaultdict(list)
for row in rows:
    groups[row["surface_speed_m_s"]].append(row)

by_surface = {}
for surface, items in sorted(groups.items()):
    by_surface[str(surface)] = {
        "rpm_d200": surface / (3.141592653589793 * .2) * 60,
        "conditions": len(items),
        "passes": sum(r["pass"] for r in items),
        "deliveries": sum(r["delivery_success"] for r in items),
        "takeoff_speed_m_s": bounds([r["takeoff_speed_m_s"] for r in items if r["takeoff_speed_m_s"] is not None]),
        "takeoff_angle_deg": bounds([r["takeoff_angle_deg"] for r in items if r["takeoff_angle_deg"] is not None]),
        "ramp_base_speed_m_s": bounds([r["ramp_base_speed_m_s"] for r in items if r["ramp_base_speed_m_s"] is not None]),
        "ski_speed_change_m_s": bounds([r["ski_speed_change_m_s"] for r in items if r["ski_speed_change_m_s"] is not None]),
        "ski_speed_retention_ratio": bounds([r["ski_speed_retention_ratio"] for r in items if r["ski_speed_retention_ratio"] is not None]),
        "lidar_margin_mm": bounds([r["lidar_margin_mm"] for r in items if r["lidar_margin_mm"] is not None]),
        "contact_current_a": bounds([r["contact_max_current_a"] for r in items if r["contact_max_current_a"] is not None]),
        "failed_check_counts": dict(Counter(x for r in items for x in r["failed_checks"])),
    }

cross = defaultdict(list)
for row in rows:
    key = f"tread_{row['tread_mu']}_ramp_{row['ramp_mu']}_offset_{row['lateral_m']}"
    cross[key].append(row)

report = {
    "status": "COMPLETE" if len(rows) == int(os.environ.get("P2_51_EXPECTED", "90")) else "INCOMPLETE",
    "conditions": len(rows),
    "passes": sum(r["pass"] for r in rows),
    "deliveries": sum(r["delivery_success"] for r in rows),
    "failed_check_counts": dict(Counter(x for r in rows for x in r["failed_checks"])),
    "by_surface_speed": by_surface,
    "per_material_offset": {
        key: {
            "rows": len(items),
            "passes": sum(r["pass"] for r in items),
            "takeoff_speed_m_s": bounds([r["takeoff_speed_m_s"] for r in items]),
            "surface_to_exit_ratio": bounds([r["takeoff_speed_m_s"] / r["surface_speed_m_s"] for r in items]),
        }
        for key, items in sorted(cross.items())
    },
    "rows": rows,
    "interpretation": [
        "Surface speed is a command, not a monotonic proxy for ball speed in the DART contact model.",
        "The lowest surface-speed row is the robustness candidate only if it passes every tread/ramp/offset condition.",
        "Current saturation is reported, not hidden; a 20 A maximum can be a brief contact transient.",
        "Empty-basket retention is comparative because DART ball-wall contact is not physically calibrated.",
    ],
}
ROOT.mkdir(parents=True, exist_ok=True)
(ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: report[k] for k in ("status", "conditions", "passes", "deliveries", "failed_check_counts", "by_surface_speed")}, indent=2))
