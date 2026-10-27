#!/usr/bin/env python3
"""Reanalyse existing 5c telemetry, separating spin-up from ball contact."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

SATURATION_A = 19.999
CONTACT_FORCE_N = 0.01


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return [row for row in csv.DictReader(stream) if None not in row.values()]


def clipped_duration(times: list[float], index: int, start: float, end: float) -> float:
    if len(times) < 2:
        return 0.0
    left = max(times[index], start)
    nominal_right = times[index + 1] if index + 1 < len(times) else times[index] + (times[-1] - times[-2])
    return max(0.0, min(nominal_right, end) - left)


def window(rows: list[dict[str, str]], start: float, end: float) -> dict:
    selected = [row for row in rows if start <= float(row["time_s"]) <= end]
    times = [float(row["time_s"]) for row in selected]
    result = {"start_s": start, "end_s": end, "duration_s": max(0.0, end-start), "samples": len(selected)}
    for side in ("left", "right"):
        values = [float(row[f"{side}_current_a"]) for row in selected]
        abs_values = [abs(value) for value in values]
        result[side] = {
            "max_abs_current_a": max(abs_values) if values else None,
            "mean_abs_current_a": sum(abs_values)/len(abs_values) if values else None,
            "mean_signed_current_a": sum(values)/len(values) if values else None,
            "saturation_time_s": sum(
                clipped_duration(times, i, start, end)
                for i, value in enumerate(abs_values) if value >= SATURATION_A
            ),
        }
    return result


def analyse_run(run: Path, campaign: str) -> list[dict]:
    rows = read_csv(run / "telemetry.csv")
    contacts = read_csv(run / "ball_contacts.csv")
    wheel_times = [float(row["time_s"]) for row in contacts
                   if int(float(row["contact_id"])) in (0, 1)
                   and float(row["normal_force_n"]) > CONTACT_FORCE_N]
    windows = [("spin_up", 0.0, 2.0)]
    if wheel_times:
        windows.append(("ball_wheel_contact", min(wheel_times), max(wheel_times)))
    else:
        windows.append(("ball_wheel_contact", 0.0, 0.0))
    output = []
    for name, start, end in windows:
        metrics = window(rows, start, end)
        output.append({"campaign": campaign, "run": str(run), "window": name,
                       "wheel_contact_detected": bool(wheel_times), **metrics})
    return output


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--p244-root", type=Path, required=True)
    ap.add_argument("--cited-run", type=Path, required=True)
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--json", type=Path, required=True)
    args = ap.parse_args()
    records: list[dict] = []
    for telemetry in sorted(args.p244_root.glob("**/telemetry.csv")):
        records.extend(analyse_run(telemetry.parent, "P2-44-r2"))
    records.extend(analyse_run(args.cited_run, "P2-43-cited"))
    flat = []
    for record in records:
        row = {key: value for key, value in record.items() if key not in ("left", "right")}
        for side in ("left", "right"):
            row.update({f"{side}_{key}": value for key, value in record[side].items()})
        flat.append(row)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader(); writer.writerows(flat)
    aggregate = {}
    for campaign in sorted({row["campaign"] for row in flat}):
        aggregate[campaign] = {}
        for name in ("spin_up", "ball_wheel_contact"):
            subset = [row for row in flat if row["campaign"] == campaign and row["window"] == name]
            summary = {"runs": len(subset), "runs_with_wheel_contact": sum(row["wheel_contact_detected"] for row in subset)}
            durations = [row["duration_s"] for row in subset]
            summary.update({"window_duration_s_min": min(durations), "window_duration_s_max": max(durations),
                            "window_duration_s_mean": sum(durations)/len(durations)})
            for side in ("left", "right"):
                for metric in ("max_abs_current_a", "mean_abs_current_a", "saturation_time_s"):
                    values = [row[f"{side}_{metric}"] for row in subset if row[f"{side}_{metric}"] is not None]
                    summary[f"{side}_{metric}_min"] = min(values) if values else None
                    summary[f"{side}_{metric}_max"] = max(values) if values else None
                    summary[f"{side}_{metric}_mean"] = sum(values)/len(values) if values else None
            aggregate[campaign][name] = summary
    result = {
        "status": "COMPLETE_EXISTING_TELEMETRY_ONLY",
        "definitions": {
            "spin_up": "t=0.0..2.0 s, before chassis approach starts",
            "ball_wheel_contact": f"first..last contact_id 0/1 sample with normal_force>{CONTACT_FORCE_N} N",
            "saturation": f"|current| >= {SATURATION_A} A; time integrated from telemetry sample intervals",
        },
        "aggregate": aggregate,
        "per_run_csv": str(args.csv),
    }
    args.json.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
