#!/usr/bin/env python3
"""Summarize Gazebo ForceTorque JSONL streams without ROS dependencies."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("Cannot summarize an empty wrench stream")
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def summarize_file(path: Path) -> dict[str, Any]:
    force_norms: list[float] = []
    torque_norms: list[float] = []
    force_components = {axis: [] for axis in "xyz"}
    torque_components = {axis: [] for axis in "xyz"}
    timestamps: list[float] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            message = json.loads(line)
            force = [float(message["force"].get(axis, 0.0)) for axis in "xyz"]
            torque = [float(message["torque"].get(axis, 0.0)) for axis in "xyz"]
            stamp = message.get("header", {}).get("stamp", {})
            timestamps.append(float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"Invalid wrench at {path}:{line_number}: {exc}") from exc
        force_norms.append(math.sqrt(sum(value * value for value in force)))
        torque_norms.append(math.sqrt(sum(value * value for value in torque)))
        for axis, value in zip("xyz", force):
            force_components[axis].append(value)
        for axis, value in zip("xyz", torque):
            torque_components[axis].append(value)

    return {
        "sample_count": len(force_norms),
        "time_range_s": [min(timestamps), max(timestamps)],
        "force_norm_n": {
            "maximum": max(force_norms),
            "p99_5": _percentile(force_norms, 0.995),
        },
        "torque_norm_nm": {
            "maximum": max(torque_norms),
            "p99_5": _percentile(torque_norms, 0.995),
        },
        "maximum_absolute_force_components_n": {
            axis: max(abs(value) for value in values)
            for axis, values in force_components.items()
        },
        "maximum_absolute_torque_components_nm": {
            axis: max(abs(value) for value in values)
            for axis, values in torque_components.items()
        },
    }


def summarize(paths: list[Path]) -> dict[str, Any]:
    joints = {path.stem.removesuffix("_joint"): summarize_file(path) for path in paths}
    if not joints:
        raise ValueError("No wrench streams supplied")
    return {
        "schema_version": 1,
        "source": "Gazebo ForceTorque joint sensors; vector norms are frame-independent",
        "joints": joints,
        "global": {
            "maximum_force_norm_n": max(
                joint["force_norm_n"]["maximum"] for joint in joints.values()
            ),
            "p99_5_force_norm_n": max(
                joint["force_norm_n"]["p99_5"] for joint in joints.values()
            ),
            "maximum_torque_norm_nm": max(
                joint["torque_norm_nm"]["maximum"] for joint in joints.values()
            ),
            "p99_5_torque_norm_nm": max(
                joint["torque_norm_nm"]["p99_5"] for joint in joints.values()
            ),
            "maximum_abs_force_z_n": max(
                joint["maximum_absolute_force_components_n"]["z"]
                for joint in joints.values()
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("streams", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = summarize(args.streams)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(report["global"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
