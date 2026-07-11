#!/usr/bin/env python3
"""Analyze Gazebo ground-truth poses for the deterministic intake bench."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


BASE_LINK_HEIGHT_M = 0.045
BALL_RADIUS_M = 0.033


def _yaw_from_quat(q: list[float] | None) -> float:
    if not q or len(q) != 4:
        return 0.0
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return records


def _pose_entry(entry: dict[str, Any]) -> dict[str, Any]:
    return {
        "x": float(entry.get("x", 0.0)),
        "y": float(entry.get("y", 0.0)),
        "z": float(entry.get("z", 0.0)),
        "q": entry.get("q"),
    }


def _robot_priority(name: str) -> int:
    if name.endswith("::base_footprint"):
        return 0
    if name.endswith("::base_link"):
        return 1
    if name.endswith("::intake_wheel_left_link"):
        return 2
    if name == "tennis_robot":
        return 10
    if name.startswith("tennis_robot::"):
        return 5
    return 99


def summarize(
    pose_path: Path,
    *,
    ball_name: str,
    nip_x_m: float,
    wheel_radius_m: float,
    wheel_gap_m: float,
    base_link_height_m: float,
) -> dict[str, Any]:
    records = _load_jsonl(pose_path)
    latest_robots: dict[str, dict[str, Any]] = {}
    latest_ball: dict[str, Any] | None = None

    # Dual-wheel throat: wheel axes at (nip_x, +/-wheel_y), vertical.
    wheel_y_m = wheel_gap_m / 2.0 + wheel_radius_m
    expected_contact_radius_m = wheel_radius_m + BALL_RADIUS_M

    evaluated = 0
    closest_x: dict[str, Any] | None = None
    closest_radial: dict[str, Any] | None = None
    first_ball_pose: dict[str, Any] | None = None
    last_ball_pose: dict[str, Any] | None = None
    prev_ball_pose: dict[str, Any] | None = None
    max_ball_speed: dict[str, Any] | None = None

    for record in records:
        for pose in record.get("poses", []):
            name = pose.get("n")
            if isinstance(name, str) and (name == "tennis_robot" or name.startswith("tennis_robot::")):
                latest_robots[name] = _pose_entry(pose)
            elif name == ball_name:
                latest_ball = _pose_entry(pose)

        if not latest_robots or latest_ball is None:
            continue

        if first_ball_pose is None:
            first_ball_pose = {
                "t_sim": record.get("t_sim"),
                "ball_xyz_m": [latest_ball["x"], latest_ball["y"], latest_ball["z"]],
            }
        current_ball_pose = {
            "t_sim": record.get("t_sim"),
            "ball_xyz_m": [latest_ball["x"], latest_ball["y"], latest_ball["z"]],
        }
        if prev_ball_pose is not None:
            prev_t = prev_ball_pose.get("t_sim")
            curr_t = current_ball_pose.get("t_sim")
            if isinstance(prev_t, (int, float)) and isinstance(curr_t, (int, float)):
                dt = curr_t - prev_t
                if dt > 1e-6:
                    prev_xyz = prev_ball_pose["ball_xyz_m"]
                    curr_xyz = current_ball_pose["ball_xyz_m"]
                    vx = (curr_xyz[0] - prev_xyz[0]) / dt
                    vy = (curr_xyz[1] - prev_xyz[1]) / dt
                    vz = (curr_xyz[2] - prev_xyz[2]) / dt
                    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                    if max_ball_speed is None or speed > max_ball_speed["speed_m_s"]:
                        max_ball_speed = {
                            "t_sim": curr_t,
                            "velocity_xyz_m_s": [vx, vy, vz],
                            "speed_m_s": speed,
                        }
        prev_ball_pose = current_ball_pose
        last_ball_pose = current_ball_pose

        robot_name, latest_robot = min(latest_robots.items(), key=lambda item: _robot_priority(item[0]))
        yaw = _yaw_from_quat(latest_robot.get("q"))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_dx = latest_ball["x"] - latest_robot["x"]
        world_dy = latest_ball["y"] - latest_robot["y"]
        ball_base_x = cos_yaw * world_dx + sin_yaw * world_dy
        ball_base_y = -sin_yaw * world_dx + cos_yaw * world_dy
        ball_base_z = latest_ball["z"] - latest_robot["z"]

        dx_to_nip = ball_base_x - nip_x_m
        dy_to_centerline = ball_base_y
        # Horizontal distance from ball centre to the nearer wheel axis
        # (both wheels are vertical cylinders, so only x/y matter).
        gap_left = math.hypot(dx_to_nip, ball_base_y - wheel_y_m)
        gap_right = math.hypot(dx_to_nip, ball_base_y + wheel_y_m)
        radial_distance = min(gap_left, gap_right)
        radial_gap = radial_distance - expected_contact_radius_m
        sample = {
            "t_sim": record.get("t_sim"),
            "robot_name": robot_name,
            "robot_xyz_m": [latest_robot["x"], latest_robot["y"], latest_robot["z"]],
            "ball_xyz_m": [latest_ball["x"], latest_ball["y"], latest_ball["z"]],
            "ball_base_xyz_m": [ball_base_x, ball_base_y, ball_base_z],
            "robot_yaw_rad": yaw,
            "nip_x_m": nip_x_m,
            "wheel_y_m": wheel_y_m,
            "dx_to_nip_m": dx_to_nip,
            "dy_to_centerline_m": dy_to_centerline,
            "wheel_distance_left_m": gap_left,
            "wheel_distance_right_m": gap_right,
            "radial_distance_xy_m": radial_distance,
            "expected_contact_radius_m": expected_contact_radius_m,
            "radial_gap_m": radial_gap,
        }
        evaluated += 1
        if closest_x is None or abs(dx_to_nip) < abs(closest_x["dx_to_nip_m"]):
            closest_x = sample
        if closest_radial is None or radial_gap < closest_radial["radial_gap_m"]:
            closest_radial = sample

    contact_expected = (
        closest_radial is not None and closest_radial["radial_gap_m"] <= 0.0
    )
    ball_delta = None
    if first_ball_pose is not None and last_ball_pose is not None:
        first_xyz = first_ball_pose["ball_xyz_m"]
        last_xyz = last_ball_pose["ball_xyz_m"]
        ball_delta = {
            "xyz_m": [last_xyz[i] - first_xyz[i] for i in range(3)],
            "distance_xy_m": math.hypot(last_xyz[0] - first_xyz[0], last_xyz[1] - first_xyz[1]),
        }

    return {
        "pose_log": str(pose_path),
        "ball_name": ball_name,
        "records": len(records),
        "evaluated_samples": evaluated,
        "nip_x_m": nip_x_m,
        "wheel_y_m": wheel_y_m,
        "expected_contact_radius_m": expected_contact_radius_m,
        "robot_pose_names": sorted(latest_robots),
        "first_ball_pose": first_ball_pose,
        "last_ball_pose": last_ball_pose,
        "ball_delta": ball_delta,
        "max_ball_speed": max_ball_speed,
        "closest_x": closest_x,
        "closest_radial": closest_radial,
        "contact_expected_from_poses": contact_expected,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose_jsonl", type=Path)
    parser.add_argument("--ball-name", default="ball_02")
    parser.add_argument("--nip-x-m", type=float, default=0.590)
    parser.add_argument("--wheel-radius-m", type=float, default=0.060)
    parser.add_argument("--wheel-gap-m", type=float, default=0.060)
    parser.add_argument("--base-link-height-m", type=float, default=BASE_LINK_HEIGHT_M)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    summary = summarize(
        args.pose_jsonl,
        ball_name=args.ball_name,
        nip_x_m=args.nip_x_m,
        wheel_radius_m=args.wheel_radius_m,
        wheel_gap_m=args.wheel_gap_m,
        base_link_height_m=args.base_link_height_m,
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
