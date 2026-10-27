#!/usr/bin/env python3
"""Offline P2-59 break-beam and D5065-event calibration.

The beam is represented as a horizontal optical line across Y in the base
frame.  A ball interrupts it while its X/Z centre is within one ball radius
of the line.  This uses only existing P2-55/56/58 telemetry; it does not
modify or rerun any simulation.
"""
from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
OUT = ROOT / "gazebo_5d1_p259"
BALL_RADIUS_M = 0.033
SAMPLE_S = 0.005


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def classification(folder: Path) -> str:
    summary = json.loads((folder / "summary.json").read_text())
    metrics = summary["metrics"]
    if summary.get("pass"):
        return "PASS"
    if metrics.get("balls_in_basket") == 2:
        return "PASS"
    detected = metrics.get("jammed") or metrics.get("jam_detector", {}).get("detected")
    if detected or "side_by_side" in folder.name:
        return "JAM"
    return "REJECT"


def folders() -> list[tuple[str, Path, str]]:
    result: list[tuple[str, Path, str]] = []
    p261 = os.environ.get("P2_61_RESCORE") == "true"
    p255 = ROOT / ("gazebo_5c3_p261_p255" if p261 else "gazebo_5c3_p255") / "matrix"
    for mu in ("0p6", "0p9"):
        for case in ("series_gap_0", "series_gap_50", "series_gap_150", "series_gap_300",
                     "side_by_side_y50", "side_by_side_y100"):
            folder = p255 / f"tm_{mu}" / case
            if (folder / "summary.json").exists():
                result.append(("P2-61/55", folder, classification(folder)))
    p256 = ROOT / "gazebo_5c3_p256" / "diagnostics"
    if not p261:
        for path in sorted(p256.glob("*/tm_*/*")):
            if (path / "summary.json").exists():
                result.append(("P2-56", path, classification(path)))
    p258 = ROOT / ("gazebo_5c3_p261_p258" if p261 else "gazebo_5c3_p258") / "matrix"
    for path in sorted(p258.glob("tm_*/*/dx_*/dy_*")):
        if (path / "summary.json").exists():
            result.append(("P2-58", path, classification(path)))
    return result


def load_observer(path: Path) -> dict[str, list[tuple[float, float, float]]]:
    states: dict[str, list[tuple[float, float, float]]] = {}
    for row in rows(path):
        states.setdefault(row["ball_name"], []).append(
            (float(row["time_s"]), float(row["base_x_m"]), float(row["base_z_m"])))
    return states


def beam_intervals(states: dict[str, list[tuple[float, float, float]]],
                   x_m: float, z_m: float) -> dict[str, list[tuple[float, float]]]:
    output: dict[str, list[tuple[float, float]]] = {}
    for name, points in states.items():
        intervals: list[tuple[float, float]] = []
        start = None
        last = None
        for time, x, z in points:
            broken = math.hypot(x - x_m, z - z_m) <= BALL_RADIUS_M
            if broken and start is None:
                start = time
            if not broken and start is not None:
                intervals.append((start, last if last is not None else time))
                start = None
            last = time
        if start is not None and last is not None:
            intervals.append((start, last))
        output[name] = intervals
    return output


def d5065_events(path: Path) -> list[dict[str, float]]:
    data = rows(path)
    points = []
    for row in data:
        time = float(row["time_s"])
        if time < 2.1:
            continue
        target = float(os.environ.get("P2_61_WHEEL_RAD_S", "60.0"))
        deficit = 0.5 * ((target - abs(float(row["left_speed_rad_s"]))) +
                         (target - abs(float(row["right_speed_rad_s"])))) * 60.0 / (2.0 * math.pi)
        points.append((time, deficit))
    events, active = [], []
    for point in points:
        if point[1] > 5.0:
            active.append(point)
        elif active:
            duration = active[-1][0] - active[0][0] + SAMPLE_S
            if duration >= 0.050:
                events.append({"start_s": active[0][0], "end_s": active[-1][0] + SAMPLE_S,
                               "duration_s": duration})
            active = []
    if active:
        duration = active[-1][0] - active[0][0] + SAMPLE_S
        if duration >= 0.050:
            events.append({"start_s": active[0][0], "end_s": active[-1][0] + SAMPLE_S,
                           "duration_s": duration})
    return events


def first_intervals(interval_map: dict[str, list[tuple[float, float]]]) -> list[dict]:
    output = []
    for name, values in interval_map.items():
        if values:
            start, end = values[0]
            output.append({"ball": name, "start_s": start, "end_s": end})
    return sorted(output, key=lambda item: item["start_s"])


def evaluate_candidate(all_cases: list[dict], x_m: float, z_m: float) -> dict:
    cases = []
    separations, latencies = [], []
    pass_two_pulse = True
    for source in all_cases:
        phase, folder, state = source["phase"], source["folder"], source["classification"]
        intervals = first_intervals(beam_intervals(source["observer"], x_m, z_m))
        events = source["events"]
        if state == "PASS":
            pass_two_pulse &= len(intervals) == 2
            if len(intervals) == 2:
                separations.append(intervals[1]["start_s"] - intervals[0]["end_s"])
            for event, pulse in zip(events, intervals):
                latencies.append(pulse["start_s"] - event["start_s"])
        cases.append({"phase": phase, "path": str(folder), "classification": state,
                      "events": events, "beam_pulses": intervals})
    return {"x_m": x_m, "z_m": z_m, "all_pass_cases_two_pulses": pass_two_pulse,
            "minimum_clear_interval_s": min(separations) if separations else None,
            "maximum_event_to_beam_s": max(latencies) if latencies else None,
            "minimum_event_to_beam_s": min(latencies) if latencies else None,
            "cases": cases}


def main() -> None:
    all_folders = folders()
    all_cases = [{"phase": phase, "folder": folder, "classification": state,
                  "observer": load_observer(folder / "observer.csv"),
                  "events": d5065_events(folder / "telemetry.csv")}
                 for phase, folder, state in all_folders]
    # Search the P2-51 ski/basket-entry neighbourhood.  Prefer the most rearward
    # valid location because it maximises time for a failed nip event to be
    # distinguished while remaining before storage.
    candidates = []
    for xi in range(260, 356, 5):
        for zi in range(45, 151, 5):
            result = evaluate_candidate(all_cases, xi / 1000.0, zi / 1000.0)
            if result["all_pass_cases_two_pulses"]:
                candidates.append(result)
    if not candidates:
        raise SystemExit("no beam line gives two pulses in every passing P2-55/P2-58 case")
    # Largest minimum clear interval, then smallest max event latency.
    selected = max(candidates, key=lambda item: (item["minimum_clear_interval_s"],
                                                  -item["maximum_event_to_beam_s"]))
    max_latency = selected["maximum_event_to_beam_s"]
    timeout_s = math.ceil((max_latency + 0.025) / 0.025) * 0.025

    false_positives = 0
    jam_hits = 0
    jam_count = 0
    phase_counts: dict[str, dict[str, int]] = {}
    for case in selected["cases"]:
        phase_counts.setdefault(case["phase"], {"PASS": 0, "JAM": 0, "REJECT": 0})
        phase_counts[case["phase"]][case["classification"]] += 1
        events, pulses = case["events"], case["beam_pulses"]
        unmatched = []
        for event in events:
            matched = any(event["start_s"] <= pulse["start_s"] <= event["start_s"] + timeout_s
                          for pulse in pulses)
            if not matched:
                unmatched.append(event)
        case["combined_detector_unmatched_events"] = unmatched
        triggered = bool(unmatched)
        if case["classification"] == "PASS" and triggered:
            false_positives += 1
        if case["classification"] == "JAM":
            jam_count += 1
            jam_hits += int(triggered)

    report = {
        "status": "PASS" if false_positives == 0 and jam_hits == jam_count else "FAIL",
        "source": "offline replay of every available P2-55/P2-56/P2-58 result folder",
        "beam": {"frame": "base_link", "axis": "+Y/-Y", "x_m": selected["x_m"],
                 "y_span_m": [-0.14, 0.14], "z_m": selected["z_m"],
                 "ball_radius_m": BALL_RADIUS_M,
                 "minimum_clear_interval_s": selected["minimum_clear_interval_s"]},
        "timing": {"rpm_event_definition": "common mean deficit >5 rpm for >=50 ms",
                   "event_to_beam_range_s": [selected["minimum_event_to_beam_s"], max_latency],
                   "margin_s": 0.025, "recommended_timeout_s": timeout_s},
        "validation": {"phase_case_counts": phase_counts, "case_count": len(selected["cases"]),
                       "passing_false_positives": false_positives,
                       "jam_count": jam_count, "jams_detected": jam_hits},
        "cases": selected["cases"],
    }
    output = ROOT / "gazebo_5d1_p261" if os.environ.get("P2_61_RESCORE") == "true" else OUT
    output.mkdir(parents=True, exist_ok=True)
    (output / "break_beam_offline_rescore.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ("status", "beam", "timing", "validation")}, indent=2))


if __name__ == "__main__":
    main()
