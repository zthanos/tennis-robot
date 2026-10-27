#!/usr/bin/env python3
"""Aggregate five repeats for each frozen P2-44 cheek/offset condition."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def tag(cheek: str, lateral: str, repeat: str) -> str:
    return f"{cheek}/y_{lateral.replace('-', 'm').replace('.', 'p')}/r{repeat}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--matrix", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--csv", type=Path, required=True)
    a = ap.parse_args()
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    missing = []
    for line in a.matrix.read_text().splitlines():
        cheek, lateral, repeat = line.split("\t")
        path = a.root / tag(cheek, lateral, repeat) / "summary.json"
        if not path.exists():
            missing.append(str(path))
        else:
            groups[(cheek, lateral)].append(json.loads(path.read_text()))
    rows = []
    for (cheek, lateral), runs in sorted(groups.items()):
        speeds = [run["metrics"]["nip_exit_rearward_speed_m_s"] for run in runs
                  if run["metrics"]["nip_exit_rearward_speed_m_s"] is not None]
        success = sum(run["pass"] for run in runs)
        rows.append({
            "cheek_variant": cheek,
            "lateral_m": lateral,
            "successes": success,
            "repeats": len(runs),
            "condition_pass": success >= 4,
            "basket_entry_failures": sum(not run["checks"]["basket_entry_reached"] for run in runs),
            "nip_failures": sum(not run["checks"]["fully_through_nip"] for run in runs),
            "plough_failures": sum(not run["checks"]["plough"] for run in runs),
            "jam_failures": sum(not run["checks"]["jam"] for run in runs),
            "nip_speed_min_m_s": min(speeds) if speeds else None,
            "nip_speed_max_m_s": max(speeds) if speeds else None,
            "max_current_a": max(run["metrics"]["max_current_a"] for run in runs),
            "max_torque_nm": max(run["metrics"]["max_torque_nm"] for run in runs),
            "forbidden_contacts": sum(run["metrics"]["forbidden_contact_count"] for run in runs),
        })
    fields = list(rows[0]) if rows else []
    with a.csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    out = {
        "status": "COMPLETE" if not missing else "INCOMPLETE",
        "stage": "P2-44 5c diagnostic subset",
        "conditions": len(rows),
        "runs": sum(len(group) for group in groups.values()),
        "passing_conditions": sum(row["condition_pass"] for row in rows),
        "failing_conditions": sum(not row["condition_pass"] for row in rows),
        "forbidden_contact_total": sum(row["forbidden_contacts"] for row in rows),
        "missing": missing,
        "criteria_status": "FROZEN_BEFORE_FIRST_P2_44_GAZEBO_RUN",
        "rows": rows,
    }
    a.output.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps({key: value for key, value in out.items() if key != "rows"}, indent=2))
    if missing:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
