#!/usr/bin/env python3
"""Prove basket entry, settling, retention, load retention, and stability.

Input is the ground-truth JSONL produced by log_gz_poses.py. A run passes only
when the target remains inside the robot-relative bin through the final sample;
an entry checkpoint followed by entity removal therefore fails honestly.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Callable


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            rows.append(json.loads(line))
        except (json.JSONDecodeError, TypeError):
            continue
    return rows


def _quat(entry: dict[str, Any]) -> tuple[float, float, float, float]:
    values = entry.get("q") or [0.0, 0.0, 0.0, 1.0]
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return x / norm, y / norm, z / norm, w / norm


def _rpy(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _to_robot(robot: dict[str, Any], entity: dict[str, Any]) -> list[float]:
    """Rotate a world-space delta by the inverse robot quaternion."""
    qx, qy, qz, qw = _quat(robot)
    vx = float(entity["x"]) - float(robot["x"])
    vy = float(entity["y"]) - float(robot["y"])
    vz = float(entity["z"]) - float(robot["z"])
    # q^-1 * v * q, expanded to avoid a dependency.
    tx = 2.0 * (-qy * vz + qz * vy)
    ty = 2.0 * (-qz * vx + qx * vz)
    tz = 2.0 * (-qx * vy + qy * vx)
    return [
        vx + qw * tx + (-qy * tz + qz * ty),
        vy + qw * ty + (-qz * tx + qx * tz),
        vz + qw * tz + (-qx * ty + qy * tx),
    ]


def _longest_duration(
    samples: list[dict[str, Any]], predicate: Callable[[dict[str, Any]], bool]
) -> float:
    longest = 0.0
    start: float | None = None
    previous_t: float | None = None
    for sample in samples:
        now = float(sample["t"])
        if predicate(sample):
            if start is None or (previous_t is not None and now - previous_t > 0.25):
                start = now
            longest = max(longest, now - start)
        else:
            start = None
        previous_t = now
    return longest


def analyze(
    pose_path: Path,
    *,
    target_name: str,
    expected_stored_count: int,
    stored_prefix: str,
    x_min: float,
    x_max: float,
    half_width: float,
    z_min: float,
    z_max: float,
    dwell_s: float,
    settle_s: float,
    settle_speed_m_s: float,
    max_pitch_deg: float,
    max_roll_deg: float,
) -> dict[str, Any]:
    rows = _load_jsonl(pose_path)
    target_samples: list[dict[str, Any]] = []
    stored_samples: dict[str, list[dict[str, Any]]] = {}
    robot_attitude: list[dict[str, float]] = []
    final_names: set[str] = set()

    def inside(local_xyz: list[float]) -> bool:
        x, y, z = local_xyz
        return x_min <= x <= x_max and abs(y) <= half_width and z_min <= z <= z_max

    for row_index, row in enumerate(rows):
        poses = {str(pose.get("n", "")): pose for pose in row.get("poses", [])}
        robot = poses.get("tennis_robot")
        if robot is None:
            continue
        t = float(row.get("t_sim") or row.get("t_wall") or row_index)
        roll, pitch, _yaw = _rpy(_quat(robot))
        robot_attitude.append(
            {"t": t, "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch)}
        )
        if row_index == len(rows) - 1:
            final_names = set(poses)

        for name, pose in poses.items():
            if name != target_name and not name.startswith(stored_prefix):
                continue
            local_xyz = _to_robot(robot, pose)
            sample = {"t": t, "local_xyz_m": local_xyz, "inside": inside(local_xyz)}
            if name == target_name:
                target_samples.append(sample)
            else:
                stored_samples.setdefault(name, []).append(sample)

    for previous, current in zip(target_samples, target_samples[1:]):
        dt = float(current["t"]) - float(previous["t"])
        if 1e-6 < dt <= 0.25:
            current["relative_speed_m_s"] = math.dist(
                previous["local_xyz_m"], current["local_xyz_m"]
            ) / dt

    target_dwell_s = _longest_duration(target_samples, lambda sample: sample["inside"])
    target_settled_s = _longest_duration(
        target_samples,
        lambda sample: sample["inside"]
        and sample.get("relative_speed_m_s", math.inf) <= settle_speed_m_s,
    )
    target_final = target_samples[-1] if target_samples else None
    target_present_at_end = target_name in final_names
    target_retained = bool(target_present_at_end and target_final and target_final["inside"])

    stored_observed = sorted(stored_samples)
    stored_escaped: list[str] = []
    stored_retained: list[str] = []
    for name, samples in stored_samples.items():
        was_inside = any(sample["inside"] for sample in samples)
        final_inside = bool(name in final_names and samples[-1]["inside"])
        if was_inside and final_inside:
            stored_retained.append(name)
        elif was_inside:
            stored_escaped.append(name)

    max_abs_pitch = max((abs(row["pitch_deg"]) for row in robot_attitude), default=None)
    max_abs_roll = max((abs(row["roll_deg"]) for row in robot_attitude), default=None)
    required = {
        "target_entered_bin": any(sample["inside"] for sample in target_samples),
        "target_dwell_met": target_dwell_s >= dwell_s,
        "target_settled": target_settled_s >= settle_s,
        "target_retained_at_end": target_retained,
        "stored_load_count_observed": len(stored_observed) == expected_stored_count,
        "all_stored_balls_retained": (
            len(stored_retained) == expected_stored_count and not stored_escaped
        ),
        "pitch_within_limit": max_abs_pitch is not None and max_abs_pitch <= max_pitch_deg,
        "roll_within_limit": max_abs_roll is not None and max_abs_roll <= max_roll_deg,
    }
    passed = sum(value is True for value in required.values())
    return {
        "pose_log": str(pose_path),
        "target_name": target_name,
        "expected_stored_count": expected_stored_count,
        "parameters": {
            "bin_robot_frame_m": {
                "x": [x_min, x_max], "abs_y_max": half_width, "z": [z_min, z_max]
            },
            "dwell_s": dwell_s,
            "settle_s": settle_s,
            "settle_speed_m_s": settle_speed_m_s,
            "max_pitch_deg": max_pitch_deg,
            "max_roll_deg": max_roll_deg,
        },
        "measurements": {
            "pose_rows": len(rows),
            "target_samples": len(target_samples),
            "target_present_at_end": target_present_at_end,
            "target_final_local_xyz_m": target_final["local_xyz_m"] if target_final else None,
            "target_longest_dwell_s": round(target_dwell_s, 3),
            "target_longest_settled_s": round(target_settled_s, 3),
            "stored_observed_count": len(stored_observed),
            "stored_retained_count": len(stored_retained),
            "stored_escaped": sorted(stored_escaped),
            "max_abs_pitch_deg": round(max_abs_pitch, 3) if max_abs_pitch is not None else None,
            "max_abs_roll_deg": round(max_abs_roll, 3) if max_abs_roll is not None else None,
        },
        "required": required,
        "required_pass": f"{passed}/{len(required)}",
        "pass": passed == len(required),
        "notes": {
            "proof": "PASS requires physical target retention through the final pose sample",
            "stored_names": f"preloaded balls must use {stored_prefix}* names",
            "checkpoint_guard": "a removed target cannot pass target_retained_at_end",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose_jsonl", type=Path)
    parser.add_argument("--target-name", default="ball_02")
    parser.add_argument("--expected-stored-count", type=int, default=0)
    parser.add_argument("--stored-prefix", default="stored_ball_")
    parser.add_argument("--x-min", type=float, default=0.02)
    parser.add_argument("--x-max", type=float, default=0.42)
    parser.add_argument("--half-width", type=float, default=0.14)
    parser.add_argument("--z-min", type=float, default=0.045)
    parser.add_argument("--z-max", type=float, default=0.25)
    parser.add_argument("--dwell-s", type=float, default=0.75)
    parser.add_argument("--settle-s", type=float, default=0.50)
    parser.add_argument("--settle-speed-m-s", type=float, default=0.08)
    parser.add_argument("--max-pitch-deg", type=float, default=8.0)
    parser.add_argument("--max-roll-deg", type=float, default=8.0)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    result = analyze(
        args.pose_jsonl,
        target_name=args.target_name,
        expected_stored_count=args.expected_stored_count,
        stored_prefix=args.stored_prefix,
        x_min=args.x_min,
        x_max=args.x_max,
        half_width=args.half_width,
        z_min=args.z_min,
        z_max=args.z_max,
        dwell_s=args.dwell_s,
        settle_s=args.settle_s,
        settle_speed_m_s=args.settle_speed_m_s,
        max_pitch_deg=args.max_pitch_deg,
        max_roll_deg=args.max_roll_deg,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

