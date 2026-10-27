#!/usr/bin/env python3
from __future__ import annotations
import csv
import json
import math
from pathlib import Path

bench = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
p255 = bench / "gazebo_5c3_p255" / "matrix"
p256 = bench / "gazebo_5c3_p256"
mapping = {
    "p256_reverse_both_y50": ("p256_reverse_both", "side_by_side_y50"),
    "p256_reverse_both_y100": ("p256_reverse_both", "side_by_side_y100"),
    "p256_asym_left_y50": ("p256_asym_left", "side_by_side_y50"),
    "p256_asym_left_y100": ("p256_asym_left", "side_by_side_y100"),
    "p256_reverse150_yaw12_y50": ("p256_reverse150_yaw12", "side_by_side_y50"),
    "p256_reverse150_yaw12_y100": ("p256_reverse150_yaw12", "side_by_side_y100"),
}


def detector_events(path: Path) -> list[dict]:
    rows = list(csv.DictReader(path.open()))
    points = []
    for row in rows:
        t = float(row["time_s"])
        if t < 2.1:
            continue
        dl = (60.0 - abs(float(row["left_speed_rad_s"]))) * 60 / (2 * math.pi)
        dr = (60.0 - abs(float(row["right_speed_rad_s"]))) * 60 / (2 * math.pi)
        deficit = 0.5 * (dl + dr)
        points.append((t, deficit, abs(float(row["left_current_a"])) + abs(float(row["right_current_a"]))))
    events, active = [], []
    for point in points:
        if point[1] > 5.0:
            active.append(point)
        elif active:
            duration = active[-1][0] - active[0][0] + 0.005
            impulse = sum(item[1] * 0.005 for item in active)
            peak_current = max(item[2] for item in active)
            if duration >= 0.050:
                events.append({"start_s": active[0][0], "duration_ms": 1000 * duration,
                               "peak_current_sum_a": peak_current,
                               "deficit_impulse_rpm_s": impulse,
                               "jam": peak_current < 30.0 or impulse > 1.70})
            active = []
    return events


series_validation = []
for mu in ("0p6", "0p9"):
    for case in ("series_gap_0", "series_gap_50", "series_gap_150", "series_gap_300"):
        events = detector_events(p255 / f"tm_{mu}" / case / "telemetry.csv")
        series_validation.append({"case": case, "tread_mu": float(mu.replace("p", ".")),
                                  "events": events, "false_positive": any(event["jam"] for event in events)})

rows = []
for mu in ("0p6", "0p9"):
    for case_name, (profile, geometry) in mapping.items():
        folder = p256 / "diagnostics" / profile / f"tm_{mu}" / geometry
        summary = json.loads((folder / "summary.json").read_text())
        summary["p256_case"] = case_name
        summary["replay_present"] = (folder / "replay" / "state.tlog").exists()
        rows.append(summary)

commands = [f"engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh {row['p256_case']} {row['tread_mu']}"
            for row in rows]
strategy_digest = []
for profile in ("p256_reverse_both", "p256_asym_left", "p256_reverse150_yaw12"):
    selected = [row for row in rows if row["drive_profile"] == profile]
    outside_distances = [ball["final_distance_from_nip_m"] for row in selected
                         for ball in row["metrics"]["outside_ball_metrics"]]
    strategy_digest.append({
        "profile": profile,
        "runs": len(selected),
        "balls_in_basket": sum(row["metrics"]["balls_in_basket"] for row in selected),
        "balls_outside_basket": sum(row["metrics"]["balls_outside_basket"] for row in selected),
        "balls_ejected": sum(row["metrics"]["balls_ejected"] for row in selected),
        "outside_distance_from_nip_range_m": [min(outside_distances), max(outside_distances)],
        "recovery_total_time_range_s": [min(row["metrics"]["recovery_total_time_s"] for row in selected),
                                         max(row["metrics"]["recovery_total_time_s"] for row in selected)],
        "peak_current_after_detection_range_a": [min(row["metrics"]["peak_current_after_detection_a"] for row in selected),
                                                   max(row["metrics"]["peak_current_after_detection_a"] for row in selected)],
    })
report = {
    "status": "COMPLETE" if len(rows) == 12 else "INCOMPLETE",
    "geometry_changed": False,
    "setup": "unchanged P2-51 / corrected 5c-3 contact model and calibration",
    "detector": {
        "common_rpm_deficit_threshold_rpm": 5.0,
        "minimum_event_duration_ms": 50.0,
        "recovery_confirmation_ms": 15.0,
        "classification": "peak |I_left|+|I_right| <30 A OR integrated common rpm deficit >1.70 rpm*s",
        "series_validation": series_validation,
        "series_cases": 8,
        "series_passage_events": sum(len(row["events"]) for row in series_validation),
        "false_positives": sum(row["false_positive"] for row in series_validation),
    },
    "runs": rows,
    "strategy_digest": strategy_digest,
    "successful_runs": sum(row["metrics"]["balls_in_basket"] == 2 for row in rows),
    "all_recorded": all(row["replay_present"] for row in rows),
    "exact_replay_commands": commands,
}
(p256 / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({"status": report["status"], "successful_runs": report["successful_runs"],
                  "false_positives": report["detector"]["false_positives"],
                  "all_recorded": report["all_recorded"]}, indent=2))
