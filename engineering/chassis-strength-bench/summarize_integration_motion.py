#!/usr/bin/env python3
"""Verify that the integration chassis actually executes the commanded turn."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _yaw(orientation: dict[str, float]) -> float:
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def summarize(path: Path) -> dict[str, float | int | bool]:
    poses: list[tuple[float, float, float]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            message = json.loads(line)
            pose = message["pose"]
            position = pose["position"]
            poses.append((float(position.get("x", 0)), float(position.get("y", 0)), _yaw(pose["orientation"])))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"Invalid odometry at {path}:{line_number}: {exc}") from exc
    if len(poses) < 2:
        raise ValueError("Need at least two odometry samples")
    unwrapped = [poses[0][2]]
    for _, _, yaw in poses[1:]:
        delta = (yaw - unwrapped[-1] + math.pi) % (2*math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + delta)
    dx = poses[-1][0] - poses[0][0]
    dy = poses[-1][1] - poses[0][1]
    yaw_change = unwrapped[-1] - unwrapped[0]
    translation = math.hypot(dx, dy)
    return {
        "sample_count": len(poses),
        "yaw_change_rad": yaw_change,
        "yaw_change_deg": math.degrees(yaw_change),
        "translation_m": translation,
        "turn_executed": abs(yaw_change) >= 1.0,
        "approximately_in_place": translation <= 0.20,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("odometry", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = summarize(args.odometry)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["turn_executed"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
