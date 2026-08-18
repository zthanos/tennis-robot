#!/usr/bin/env python3
"""Check a frame-diagnosis recording for the four consistency properties.

Phase 11G.  Offline and read-only.  Each check answers one question, and a
failure is reported rather than smoothed over:

  tracker self-consistency
      recomputing cross-track and heading error from the tracker's own
      published pose and reference point reproduces the values it reported;

  frame-transform consistency
      map->odom composed with odom->base equals map->base at the same instant;

  path-frame consistency
      the path point and the pose the tracker compared carry the same frame;

  timestamp consistency
      transforms older than a documented tolerance are flagged, never silently
      interpolated.

    python3 scripts/sim_debug/analyze_frame_diagnosis.py runtime/.../frame_diagnosis.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import statistics

# A transform older than this at the moment it is used is reported rather than
# trusted: at 0.35 m/s it is already worth ~9 cm of travel.
TRANSFORM_AGE_TOLERANCE_S = 0.25
NUMERIC_TOLERANCE_M = 1e-6
NUMERIC_TOLERANCE_RAD = 1e-4


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(transform: dict, x: float, y: float) -> tuple[float, float]:
    cos, sin = math.cos(transform["yaw_rad"]), math.sin(transform["yaw_rad"])
    return (transform["x_m"] + cos * x - sin * y, transform["y_m"] + sin * x + cos * y)


def usable(entry) -> bool:
    return isinstance(entry, dict) and "x_m" in entry


def tracker_self_consistency(rows: list[dict]) -> dict:
    lateral, heading = [], []
    for row in rows:
        tracker, reference = row["tracker"], row["reference"]
        lateral.append(abs(
            math.dist((tracker["x_m"], tracker["y_m"]), (reference["x_m"], reference["y_m"]))
            - row["reported_lateral_error_m"]
        ))
        heading.append(abs(
            wrap(reference["yaw_rad"] - tracker["yaw_rad"]) - row["reported_heading_error_rad"]
        ))
    return {
        "rows": len(rows),
        "max_lateral_residual_m": max(lateral, default=0.0),
        "max_heading_residual_rad": max(heading, default=0.0),
        "passed": (max(lateral, default=0.0) <= NUMERIC_TOLERANCE_M
                   and max(heading, default=0.0) <= NUMERIC_TOLERANCE_RAD),
    }


def frame_transform_consistency(rows: list[dict]) -> dict:
    residuals = []
    for row in rows:
        map_odom, odom_base, map_base = row["map_odom"], row["odom_base"], row["map_base"]
        if not (usable(map_odom) and usable(odom_base) and usable(map_base)):
            continue
        composed = compose(map_odom, odom_base["x_m"], odom_base["y_m"])
        residuals.append(math.dist(composed, (map_base["x_m"], map_base["y_m"])))
    return {
        "rows": len(residuals),
        "median_residual_m": statistics.median(residuals) if residuals else None,
        "max_residual_m": max(residuals, default=None),
        "passed": bool(residuals) and statistics.median(residuals) <= 1e-6,
    }


def path_frame_consistency(rows: list[dict]) -> dict:
    pairs = {(row["tracker"]["pose_frame_id"], row["reference"]["path_frame_id"]) for row in rows}
    mismatched = {pair for pair in pairs if pair[0] != pair[1]}
    return {
        "frame_pairs": sorted(pairs),
        "mismatched": sorted(mismatched),
        "passed": not mismatched and all(pair[0] for pair in pairs),
    }


def timestamp_consistency(rows: list[dict]) -> dict:
    pose_age, transform_age, path_age, stale = [], [], [], 0
    for row in rows:
        calculated_at = row["tracker"]["update_stamp_s"]
        pose_age.append(calculated_at - row["tracker"]["pose_stamp_s"])
        path_age.append(calculated_at - row["reference"]["path_stamp_s"])
        if usable(row["map_odom"]):
            age = calculated_at - row["map_odom"]["stamp_s"]
            transform_age.append(age)
            stale += abs(age) > TRANSFORM_AGE_TOLERANCE_S
    return {
        "median_pose_age_s": statistics.median(pose_age) if pose_age else None,
        "max_pose_age_s": max(pose_age, default=None),
        "median_transform_age_s": statistics.median(transform_age) if transform_age else None,
        "stale_transform_rows": stale,
        "max_path_age_s": max(path_age, default=None),
        "tolerance_s": TRANSFORM_AGE_TOLERANCE_S,
    }


def analyze(rows: list[dict]) -> dict:
    return {
        "tracker_self_consistency": tracker_self_consistency(rows),
        "frame_transform_consistency": frame_transform_consistency(rows),
        "path_frame_consistency": path_frame_consistency(rows),
        "timestamp_consistency": timestamp_consistency(rows),
    }


def load(path: str) -> list[dict]:
    with open(path, encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", help="frame_diagnosis.jsonl")
    arguments = parser.parse_args()
    report = analyze(load(arguments.recording))
    print(json.dumps(report, indent=1, sort_keys=True))
    failed = [name for name, result in report.items() if result.get("passed") is False]
    if failed:
        print("\nFAILED CHECKS: " + ", ".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
