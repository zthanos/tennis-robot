#!/usr/bin/env python3
"""Summarize a sim_physics_probe JSONL log for intake tuning sweeps."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None, "mean": None, "median": None}
    return {
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "median": statistics.median(values),
    }


def summarize(path: Path, status_path: Path | None = None) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    summary: dict[str, Any] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            rec = json.loads(line)
            if rec.get("type") == "summary":
                summary = rec
            elif rec.get("type") == "roller_contact_sample":
                rows.append(rec)

    times = [float(r["t_s"]) for r in rows if r.get("t_s") is not None]
    depths = [float(r["max_depth_m"]) * 1000.0 for r in rows if r.get("max_depth_m") is not None]
    forces = [float(r["max_force_n"]) for r in rows if r.get("max_force_n") is not None]
    joint_vels = [
        float(r["joint_velocity_rad_s"])
        for r in rows
        if r.get("joint_velocity_rad_s") is not None
    ]
    ball_speeds = [
        float(r["ball_speed_m_s"])
        for r in rows
        if r.get("ball_speed_m_s") is not None
    ]
    summary_joint_velocity = summary.get("joint_velocity_rad_s")
    if summary_joint_velocity is not None:
        try:
            joint_vels.append(float(summary_joint_velocity))
        except (TypeError, ValueError):
            pass
    left_samples = sum(1 for r in rows if r.get("wheel") == "left")
    right_samples = sum(1 for r in rows if r.get("wheel") == "right")
    geometries = [r.get("geometry", {}) for r in rows if isinstance(r.get("geometry"), dict)]
    geometry = geometries[-1] if geometries else {
        "nip_x_m": summary.get("nip_x_m"),
        "wheel_gap_m": summary.get("wheel_gap_m"),
        "wheel_radius_m": summary.get("wheel_radius_m"),
        "nominal_bite_dx_m": summary.get("nominal_bite_dx_m"),
    }

    status: dict[str, Any] = {}
    if status_path and status_path.exists():
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            status = {}

    contact_duration_s = (max(times) - min(times)) if times else 0.0
    depth_stats = _stats(depths)
    force_stats = _stats(forces)
    speed_stats = _stats(ball_speeds)

    return {
        "log": str(path),
        "samples": len(rows),
        "contact_duration_s": round(contact_duration_s, 4),
        "first_contact_t_s": round(min(times), 4) if times else None,
        "last_contact_t_s": round(max(times), 4) if times else None,
        "wheel_left_samples": left_samples,
        "wheel_right_samples": right_samples,
        "nip_x_m": geometry.get("nip_x_m"),
        "wheel_gap_m": geometry.get("wheel_gap_m"),
        "wheel_radius_m": geometry.get("wheel_radius_m"),
        "nominal_bite_dx_m": geometry.get("nominal_bite_dx_m"),
        "depth_min_mm": depth_stats["min"],
        "depth_max_mm": depth_stats["max"],
        "depth_mean_mm": depth_stats["mean"],
        "depth_median_mm": depth_stats["median"],
        "force_min_n": force_stats["min"],
        "force_max_n": force_stats["max"],
        "force_mean_n": force_stats["mean"],
        "force_median_n": force_stats["median"],
        "joint_vel_abs_max_rad_s": max((abs(v) for v in joint_vels), default=None),
        "joint_vel_abs_median_rad_s": statistics.median([abs(v) for v in joint_vels])
        if joint_vels else None,
        "ball_speed_max_m_s": speed_stats["max"],
        "ball_speed_median_m_s": speed_stats["median"],
        "points_base_samples": sum(1 for r in rows if r.get("points_base")),
        "nearest_ball": summary.get("nearest_ball"),
        "closest_ball": summary.get("closest_ball"),
        "balls_collected": status.get("balls_collected"),
        "collector_state": status.get("collector_state"),
        "mode": status.get("mode"),
    }


def _write_csv_row(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--status", type=Path, default=None)
    parser.add_argument("--csv-append", type=Path, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args()

    row = summarize(args.jsonl, args.status)
    print(json.dumps(row, indent=2, sort_keys=True))
    if args.csv_append:
        _write_csv_row(args.csv_append, row)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(row, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
