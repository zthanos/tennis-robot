#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259")
CRITERIA = Path("engineering/shared-flywheel-isolated-bench/criteria_5d1_p259.json")


def folder(angle: int, mu: float, surface: int, repeat: int) -> Path:
    return ROOT / "cases" / f"angle_{angle}" / f"tm_{str(mu).replace('.', 'p')}" / f"surface_{surface}" / f"repeat_{repeat}"


def load() -> list[dict]:
    result = []
    for angle in (15, 20):
        for mu in (.6, .9):
            for surface in (12, 16, 20, 24):
                for repeat in (1, 2, 3):
                    path = folder(angle, mu, surface, repeat)
                    row = json.loads((path / "summary.json").read_text())
                    row["record_present"] = (path / "replay" / "state.tlog").exists()
                    result.append(row)
    return result


def mean(rows: list[dict], metric: str) -> float | None:
    values = [row["metrics"][metric] for row in rows if row["metrics"][metric] is not None]
    return statistics.fmean(values) if values else None


def stdev(rows: list[dict], metric: str) -> float | None:
    values = [row["metrics"][metric] for row in rows if row["metrics"][metric] is not None]
    return statistics.pstdev(values) if values else None


def estimate(points: list[dict], target: float) -> dict:
    ordered = sorted(points, key=lambda item: item["mean_exit_speed_m_s"])
    for low, high in zip(ordered, ordered[1:]):
        if low["mean_exit_speed_m_s"] <= target <= high["mean_exit_speed_m_s"]:
            ratio = (target-low["mean_exit_speed_m_s"])/(high["mean_exit_speed_m_s"]-low["mean_exit_speed_m_s"])
            return {"target_exit_speed_m_s": target,
                    "required_surface_speed_m_s": low["surface_speed_m_s"] + ratio*(high["surface_speed_m_s"]-low["surface_speed_m_s"]),
                    "method": "linear interpolation between executed points",
                    "bracket_surface_speed_m_s": [low["surface_speed_m_s"], high["surface_speed_m_s"]]}
    pair = ordered[-2:] if target > ordered[-1]["mean_exit_speed_m_s"] else ordered[:2]
    low, high = pair
    gain = abs(high["mean_exit_speed_m_s"]-low["mean_exit_speed_m_s"])
    if gain < 0.10:
        return {"target_exit_speed_m_s": target, "required_surface_speed_m_s": None,
                "method": "not identifiable: executed 20-to-24 m/s points are traction-limited and saturated",
                "bracket_surface_speed_m_s": [low["surface_speed_m_s"], high["surface_speed_m_s"]],
                "measured_exit_gain_m_s": gain,
                "maximum_executed_exit_speed_m_s": ordered[-1]["mean_exit_speed_m_s"]}
    ratio = (target-low["mean_exit_speed_m_s"])/(high["mean_exit_speed_m_s"]-low["mean_exit_speed_m_s"])
    return {"target_exit_speed_m_s": target,
            "required_surface_speed_m_s": low["surface_speed_m_s"] + ratio*(high["surface_speed_m_s"]-low["surface_speed_m_s"]),
            "method": "linear extrapolation estimate only; not capability validation",
            "bracket_surface_speed_m_s": [low["surface_speed_m_s"], high["surface_speed_m_s"]]}


def cell_rows(rows: list[dict], angle: int, mu: float, surface: int) -> list[dict]:
    return [row for row in rows if row["angle_deg"] == angle and row["tread_mu"] == mu and row["surface_speed_m_s"] == surface]


def main() -> None:
    ap = argparse.ArgumentParser(); ap.add_argument("--write-record-manifest", action="store_true"); args = ap.parse_args()
    rows = load()
    cells = []
    for angle in (15, 20):
        for mu in (.6, .9):
            for surface in (12, 16, 20, 24):
                selected = cell_rows(rows, angle, mu, surface)
                cells.append({
                    "angle_deg": angle, "tread_mu": mu, "surface_speed_m_s": surface,
                    "passes": sum(row["pass"] for row in selected), "runs": len(selected),
                    "mean_exit_speed_m_s": mean(selected, "exit_speed_m_s"),
                    "exit_speed_spread_m_s": stdev(selected, "exit_speed_m_s"),
                    "mean_exit_elevation_deg": mean(selected, "exit_elevation_deg"),
                    "elevation_spread_deg": stdev(selected, "exit_elevation_deg"),
                    "mean_exit_azimuth_deg": mean(selected, "exit_azimuth_deg"),
                    "azimuth_spread_deg": stdev(selected, "exit_azimuth_deg"),
                    "mean_spin_rad_s": mean(selected, "spin_magnitude_rad_s"),
                    "mean_rpm_drop": {side: statistics.fmean(row["metrics"]["rpm_drop"][side] for row in selected) for side in ("left", "right")},
                    "mean_recovery_to_95pct_s": mean(selected, "recovery_to_95pct_s"),
                    "peak_spinup_current_a": max(row["metrics"]["spinup_peak_current_a"][side] for row in selected for side in ("left", "right")),
                    "peak_contact_current_a": max(row["metrics"]["contact_peak_current_a"][side] for row in selected for side in ("left", "right")),
                    "peak_contact_torque_nm": max(row["metrics"]["contact_peak_torque_nm"][side] for row in selected for side in ("left", "right")),
                    "forbidden_contacts": sum(row["metrics"]["forbidden_contact_count"] for row in selected),
                })
    selected_cells = set()
    failed = [row for row in rows if not row["pass"]]
    for row in failed:
        selected_cells.add((row["angle_deg"], row["tread_mu"], int(row["surface_speed_m_s"]), row["repeat"]))
    for angle in (15, 20):
        for mu in (.6, .9):
            group = [cell for cell in cells if cell["angle_deg"] == angle and cell["tread_mu"] == mu and cell["mean_exit_speed_m_s"] is not None]
            for target in (12.0, 14.0):
                below = [cell for cell in group if cell["mean_exit_speed_m_s"] <= target]
                above = [cell for cell in group if cell["mean_exit_speed_m_s"] >= target]
                if below: selected_cells.add((angle, mu, max(below, key=lambda c: c["mean_exit_speed_m_s"])["surface_speed_m_s"], 1))
                if above: selected_cells.add((angle, mu, min(above, key=lambda c: c["mean_exit_speed_m_s"])["surface_speed_m_s"], 1))
    manifest = ROOT / "record_manifest.txt"
    manifest.write_text("".join(f"{a} {mu} {s} {r}\n" for a, mu, s, r in sorted(selected_cells)))
    if args.write_record_manifest:
        print(json.dumps({"runs": len(rows), "record_runs": len(selected_cells), "manifest": str(manifest)}, indent=2)); return
    requirements = []
    for angle in (15, 20):
        for mu in (.6, .9):
            group = [cell for cell in cells if cell["angle_deg"] == angle and cell["tread_mu"] == mu]
            requirements.append({"angle_deg": angle, "tread_mu": mu,
                                 "targets": [estimate(group, target) for target in (12.0, 14.0)]})
    report = {
        "status": "PASS" if all(row["pass"] for row in rows) else "FAIL",
        "run_count": len(rows), "pass_count": sum(row["pass"] for row in rows),
        "criteria_sha256": hashlib.sha256(CRITERIA.read_bytes()).hexdigest(),
        "cells": cells, "required_surface_speed": requirements,
        "recording": {"selected_runs": len(selected_cells),
                      "present_runs": sum((folder(a, mu, s, r)/"replay"/"state.tlog").exists() for a, mu, s, r in selected_cells),
                      "manifest": str(manifest)},
        "runs": rows,
    }
    (ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "runs": len(rows), "passes": report["pass_count"],
                      "records": report["recording"]}, indent=2))


if __name__ == "__main__": main()
