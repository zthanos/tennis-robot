#!/usr/bin/env python3
"""Build a machine-readable camera/execution alignment report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics


def camera_summary(records: list[dict]) -> dict:
    associated = [
        row
        for row in records
        if row.get("event") == "spatial_detection"
        and (row.get("association") or {}).get("status") == "associated"
    ]
    distances = [
        float(row["association"]["distance_m"]) for row in associated
    ]
    residuals = [
        tuple(float(value) for value in row["association"]["residual_camera_xyz_m"])
        for row in associated
    ]
    return {
        "associated_samples": len(associated),
        "median_residual_norm_m": (
            statistics.median(distances) if distances else None
        ),
        "max_residual_norm_m": max(distances) if distances else None,
        "median_residual_camera_xyz_m": (
            [
                statistics.median(residual[index] for residual in residuals)
                for index in range(3)
            ]
            if residuals
            else None
        ),
        "rgb_depth_delta_max_s": (
            max(float(row["rgb_depth_delta_s"]) for row in associated)
            if associated
            else None
        ),
    }


def execution_summary(
    diagnostics: dict,
    truth_snapshot: dict,
) -> dict:
    crossings = diagnostics.get("execution_crossings") or []
    truth = truth_snapshot.get("sim_balls_odom") or []
    results = []
    for crossing in crossings:
        ranked = sorted(
            (
                math.hypot(
                    float(ball["x"]) - float(crossing["x_m"]),
                    float(ball["y"]) - float(crossing["y_m"]),
                ),
                str(ball["def"]),
                ball,
            )
            for ball in truth
            if isinstance(ball, dict) and {"def", "x", "y"} <= set(ball)
        )
        if not ranked:
            continue
        distance_m, truth_ball_id, ball = ranked[0]
        dx = float(ball["x"]) - float(crossing["x_m"])
        dy = float(ball["y"]) - float(crossing["y_m"])
        heading = float(crossing["heading_rad"])
        results.append(
            {
                "planned_ball_id": crossing["ball_id"],
                "nearest_truth_ball_id": truth_ball_id,
                "distance_m": distance_m,
                "longitudinal_error_m": dx * math.cos(heading)
                + dy * math.sin(heading),
                "lateral_error_m": -dx * math.sin(heading)
                + dy * math.cos(heading),
                "crossing_odom_xy_m": [
                    float(crossing["x_m"]),
                    float(crossing["y_m"]),
                ],
                "truth_odom_xy_m": [float(ball["x"]), float(ball["y"])],
            }
        )
    distances = [item["distance_m"] for item in results]
    return {
        "crossing_count": len(results),
        "median_nearest_truth_distance_m": (
            statistics.median(distances) if distances else None
        ),
        "max_nearest_truth_distance_m": max(distances) if distances else None,
        "crossings": results,
        "frozen_transform": diagnostics.get("transform"),
        "transform_timestamp_s": diagnostics.get("transform_timestamp_s"),
        "pose_drift_at_freeze_m": truth_snapshot.get("pose_drift_m"),
        "yaw_drift_at_freeze_rad": truth_snapshot.get("yaw_drift_rad"),
    }


def _load_jsonl(path: Path) -> list[dict]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--status", type=Path)
    source.add_argument("--audit", type=Path)
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    source_path = args.status or args.audit
    source = json.loads(source_path.read_text(encoding="utf-8"))
    if args.status:
        route = source.get("collect_route") or {}
        diagnostics = route.get("execution_frame_diagnostics") or {}
        truth = route.get("execution_truth_snapshot") or {}
    else:
        diagnostics = source.get("execution_frame_diagnostics") or {}
        truth = source.get("execution_truth_snapshot") or {}
    report = {
        "schema_version": 1,
        "camera_alignment": camera_summary(_load_jsonl(args.probe)),
        "execution_alignment": execution_summary(diagnostics, truth),
    }
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
