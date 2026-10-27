#!/usr/bin/env python3
"""Aggregate P2-47 r6 and compare it with retained r3 candidates."""
from __future__ import annotations

import csv
import json
from pathlib import Path

BASE = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
ROOT = BASE / "gazebo_5c2_p247_r6"


def load(path: Path):
    return json.loads(path.read_text()) if path.exists() else None


rows, missing = [], []
for mode, preloads in (("delivery", (0, 25, 40, 44)), ("motion", (45,))):
    for preload in preloads:
        for repeat in (1, 2, 3):
            path = ROOT / "tall_open_drop" / mode / f"preload_{preload}" / f"r{repeat}" / "summary.json"
            result = load(path)
            if result is None:
                missing.append(str(path)); continue
            m = result["metrics"]
            rows.append({
                "variant": "tall_open_drop", "mode": mode, "preload": preload, "repeat": repeat,
                "pass": result["pass"], "collection_success": result["checks"].get("collection_success"),
                "entrance_blocked": m.get("entrance_blocked"), "bounce_out": m.get("bounce_out"),
                "preload_before": m.get("preload_retained_before_delivery", m.get("baseline_retained_count")),
                "preload_final": m.get("preload_retained_final", m.get("final_retained_count")),
                "motion_losses": m.get("motion_ball_loss_count"),
                "contact_peak_current_a": (m.get("ball_contact_current") or {}).get("max_abs_a"),
                "contact_mean_current_a": (m.get("ball_contact_current") or {}).get("mean_abs_a"),
                "forbidden_contacts": m.get("forbidden_contact_count"),
            })

with (ROOT / "condition_matrix.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["variant"])
    writer.writeheader(); writer.writerows(rows)

comparators = {}
for variant in ("high_hood", "lip_cover", "one_way_flap"):
    source = BASE / "gazebo_5c2_p247_r3" / variant
    summaries = [load(p) for p in source.rglob("summary.json")]
    summaries = [s for s in summaries if s is not None]
    comparators[variant] = {
        "available_runs": len(summaries),
        "passing_runs": sum(bool(s["pass"]) for s in summaries),
        "delivery_runs": sum(s.get("mode") == "delivery" for s in summaries),
        "motion_runs": sum(s.get("mode") == "motion" for s in summaries),
        "status_note": ("incomplete/rejected after unloaded and preload-25 diagnostics" if variant == "one_way_flap"
                        else "retained r3 results; see P2-47 limitations"),
    }

delivery = [r for r in rows if r["mode"] == "delivery"]
motion = [r for r in rows if r["mode"] == "motion"]
summary = {
    "status": "COMPLETE" if not missing else "INCOMPLETE",
    "stage": "P2-47 r6 tall-open-drop comparative Gazebo gate",
    "absolute_capacity_claim": False,
    "run_count": len(rows), "missing": missing,
    "tall_open_drop": {
        "passing_runs": sum(r["pass"] for r in rows),
        "delivery_passes": sum(r["pass"] for r in delivery),
        "delivery_runs": len(delivery),
        "motion_passes": sum(r["pass"] for r in motion),
        "motion_runs": len(motion),
        "conditions_all_repeats_pass": {
            str(p): all(r["pass"] for r in delivery if r["preload"] == p) and sum(r["preload"] == p for r in delivery) == 3
            for p in (0, 25, 40, 44)
        },
        "contact_peak_current_a": max((r["contact_peak_current_a"] for r in delivery if r["contact_peak_current_a"] is not None), default=None),
        "contact_mean_current_a_max": max((r["contact_mean_current_a"] for r in delivery if r["contact_mean_current_a"] is not None), default=None),
        "total_motion_losses": sum((r["motion_losses"] or 0) for r in motion),
    },
    "retained_comparators": comparators,
    "limitations": ["Native DART basket and ball-ball contact is uncalibrated.", "Absolute 45-ball capacity requires the 1:1 physical mock-up."],
}
(ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
