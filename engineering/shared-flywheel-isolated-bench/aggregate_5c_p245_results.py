#!/usr/bin/env python3
"""Aggregate the exact 5/5 P2-45 ramp-height conditions."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def tag(tip: str, cheek: str, lateral: str, repeat: str) -> str:
    return f"tip_{tip}mm/{cheek}/y_{lateral.replace('-', 'm').replace('.', 'p')}/r{repeat}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--matrix", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--csv", type=Path, required=True)
    args = ap.parse_args()
    groups = defaultdict(list); missing = []
    for line in args.matrix.read_text().splitlines():
        tip, cheek, lateral, repeat = line.split("\t")
        path = args.root / tag(tip, cheek, lateral, repeat) / "summary.json"
        if path.exists(): groups[(tip, cheek,lateral)].append(json.loads(path.read_text()))
        else: missing.append(str(path))
    rows = []
    for (tip, cheek, lateral), runs in sorted(groups.items(), key=lambda item: (int(item[0][0]), item[0][1], float(item[0][2]))):
        success = sum(run["pass"] for run in runs)
        speeds = [run["metrics"]["nip_exit_rearward_speed_m_s"] for run in runs if run["metrics"]["nip_exit_rearward_speed_m_s"] is not None]
        rows.append({
            "tip_clearance_mm": int(tip), "cheek_variant": cheek, "lateral_m": lateral,
            "successes": success, "repeats": len(runs), "condition_pass": success == 5 and len(runs) == 5,
            "basket_entry_failures": sum(not run["checks"]["basket_entry_reached"] for run in runs),
            "nip_failures": sum(not run["checks"]["fully_through_nip"] for run in runs),
            "plough_diagnostic_count": sum(not run["checks"]["plough"] for run in runs),
            "jam_failures": sum(not run["checks"]["jam"] for run in runs),
            "forward_push_max_m": max(run["metrics"]["pre_nip_forward_displacement_m"] for run in runs),
            "lateral_push_max_m": max(run["metrics"]["pre_nip_lateral_displacement_m"] for run in runs),
            "nip_speed_min_m_s": min(speeds) if speeds else None, "nip_speed_max_m_s": max(speeds) if speeds else None,
            "max_current_a": max(run["metrics"]["max_current_a"] for run in runs),
            "forbidden_contacts": sum(run["metrics"]["forbidden_contact_count"] for run in runs),
        })
    fields = list(rows[0]) if rows else []
    with args.csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    height_summary = {}
    for tip in (3, 5, 8):
        subset = [row for row in rows if row["tip_clearance_mm"] == tip]
        height_summary[str(tip)] = {"passing_conditions": sum(row["condition_pass"] for row in subset), "conditions": len(subset), "all_conditions_pass": bool(subset) and all(row["condition_pass"] for row in subset)}
    result = {"status": "COMPLETE" if not missing else "INCOMPLETE", "stage": "P2-45 ramp-tip clearance sweep",
              "conditions": len(rows), "runs": sum(len(group) for group in groups.values()),
              "passing_conditions": sum(row["condition_pass"] for row in rows), "failing_conditions": sum(not row["condition_pass"] for row in rows),
              "forbidden_contact_total": sum(row["forbidden_contacts"] for row in rows), "height_summary": height_summary,
              "largest_clearance_passing_all_mm": max((tip for tip in (3,5,8) if height_summary[str(tip)]["all_conditions_pass"]), default=None),
              "missing": missing, "criteria_status": "FROZEN_BEFORE_FIRST_P2_45_GAZEBO_RUN", "rows": rows}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key:value for key,value in result.items() if key != "rows"}, indent=2))
    if missing: raise SystemExit(1)


if __name__ == "__main__": main()
