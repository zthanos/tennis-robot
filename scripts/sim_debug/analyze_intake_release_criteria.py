#!/usr/bin/env python3
"""Evaluate release-oriented intake bench criteria from contact + pose logs."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any


BALL_RADIUS_M = 0.033
DEFAULT_RELEASE_WINDOW_S = 0.20
DEFAULT_MIN_SPEED_M_S = 0.40
DEFAULT_MAX_CONTACT_DURATION_S = 0.50
DEFAULT_MIN_DIRECTIONAL_VELOCITY_M_S = 0.01
DEFAULT_FRONT_LIP_ZONE_M = 0.008


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _yaw_from_quat(q: list[float] | None) -> float:
    if not q or len(q) != 4:
        return 0.0
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _base_pose(robot: dict[str, Any], ball: dict[str, Any]) -> tuple[float, float, float]:
    yaw = _yaw_from_quat(robot.get("q"))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx = float(ball["x"]) - float(robot["x"])
    dy = float(ball["y"]) - float(robot["y"])
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        float(ball["z"]) - float(robot["z"]),
    )


def _pose_samples(pose_rows: list[dict[str, Any]], ball_name: str) -> list[dict[str, Any]]:
    latest_robot: dict[str, Any] | None = None
    latest_ball: dict[str, Any] | None = None
    samples: list[dict[str, Any]] = []
    for row in pose_rows:
        for pose in row.get("poses", []):
            name = pose.get("n")
            if name == "tennis_robot":
                latest_robot = pose
            elif name == ball_name:
                latest_ball = pose
        if latest_robot is None or latest_ball is None:
            continue
        bx, by, bz = _base_pose(latest_robot, latest_ball)
        samples.append(
            {
                "t_wall": float(row.get("t_wall", 0.0)),
                "t_sim": row.get("t_sim"),
                "base_xyz_m": [bx, by, bz],
                "world_xyz_m": [
                    float(latest_ball["x"]),
                    float(latest_ball["y"]),
                    float(latest_ball["z"]),
                ],
            }
        )
    velocities: list[dict[str, Any]] = []
    for prev, curr in zip(samples, samples[1:]):
        dt = curr["t_wall"] - prev["t_wall"]
        if dt <= 1e-6:
            continue
        prev_xyz = prev["base_xyz_m"]
        curr_xyz = curr["base_xyz_m"]
        vel = [(curr_xyz[i] - prev_xyz[i]) / dt for i in range(3)]
        velocities.append({**curr, "base_velocity_m_s": vel})
    return velocities


def _robot_pose_samples(pose_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for row in pose_rows:
        for pose in row.get("poses", []):
            if pose.get("n") != "tennis_robot":
                continue
            samples.append(
                {
                    "t_wall": float(row.get("t_wall", 0.0)),
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "z": float(pose["z"]),
                    "yaw": _yaw_from_quat(pose.get("q")),
                }
            )
            break
    return samples


def _world_point_to_base(
    point: list[float],
    robot: dict[str, Any],
) -> tuple[float, float, float]:
    cos_yaw = math.cos(robot["yaw"])
    sin_yaw = math.sin(robot["yaw"])
    dx = float(point[0]) - robot["x"]
    dy = float(point[1]) - robot["y"]
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        float(point[2]) - robot["z"],
    )


def _first_crossing(
    samples: list[dict[str, Any]],
    *,
    x_plane_m: float | None = None,
    z_plane_m: float | None = None,
    after_wall_s: float | None = None,
) -> dict[str, Any] | None:
    for sample in samples:
        if after_wall_s is not None and sample["t_wall"] < after_wall_s:
            continue
        x, _y, z = sample["base_xyz_m"]
        if x_plane_m is not None and x > x_plane_m:
            continue
        if z_plane_m is not None and z < z_plane_m:
            continue
        return sample
    return None


def _nearest_sample(samples: list[dict[str, Any]], t_wall: float) -> dict[str, Any] | None:
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample["t_wall"] - t_wall))


def _classify_scoop_contacts(
    lip_contact_rows: list[dict[str, Any]],
    robot_samples: list[dict[str, Any]],
    *,
    front_lip_min_x_m: float,
) -> tuple[list[float], list[float]]:
    front_wall_times: list[float] = []
    ramp_wall_times: list[float] = []
    for row in lip_contact_rows:
        t_wall = row.get("t_wall")
        points_world = row.get("points_world")
        if t_wall is None or not points_world:
            continue
        robot = _nearest_sample(robot_samples, float(t_wall))
        if robot is None:
            continue
        has_front = False
        has_ramp = False
        for point in points_world:
            if len(point) != 3:
                continue
            base_x, _base_y, _base_z = _world_point_to_base(point, robot)
            if base_x >= front_lip_min_x_m:
                has_front = True
            else:
                has_ramp = True
        if has_front:
            front_wall_times.append(float(t_wall))
        if has_ramp:
            ramp_wall_times.append(float(t_wall))
    return front_wall_times, ramp_wall_times


def _percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def analyze(
    contact_jsonl: Path,
    pose_jsonl: Path,
    *,
    ball_name: str,
    ramp_entry_x_m: float,
    ramp_crest_z_m: float,
    release_window_s: float,
    preferred_contact_duration_s: float,
    preferred_speed_m_s: float,
    min_directional_velocity_m_s: float,
    force_p95_threshold_n: float | None,
    front_lip_zone_m: float,
) -> dict[str, Any]:
    all_contact_rows = _load_jsonl(contact_jsonl)
    contact_rows = [
        row
        for row in all_contact_rows
        if row.get("type") == "roller_contact_sample" and row.get("ball") == ball_name
    ]
    lip_contact_rows = [
        row
        for row in all_contact_rows
        if row.get("type") == "lip_contact_sample" and row.get("ball") == ball_name
    ]
    front_lip_contact_rows = [
        row
        for row in all_contact_rows
        if row.get("type") == "front_lip_contact_sample" and row.get("ball") == ball_name
    ]
    ramp_guide_contact_rows = [
        row
        for row in all_contact_rows
        if row.get("type") == "ramp_guide_contact_sample" and row.get("ball") == ball_name
    ]
    pose_rows = _load_jsonl(pose_jsonl)
    pose_samples = _pose_samples(pose_rows, ball_name)
    robot_samples = _robot_pose_samples(pose_rows)

    contact_wall_times = [
        float(row["t_wall"])
        for row in contact_rows
        if row.get("t_wall") is not None
    ]
    contact_elapsed_times = [
        float(row["t_s"])
        for row in contact_rows
        if row.get("t_s") is not None
    ]
    forces = [
        float(row["max_force_n"])
        for row in contact_rows
        if row.get("max_force_n") is not None
    ]
    lip_contact_wall_times = [
        float(row["t_wall"])
        for row in lip_contact_rows
        if row.get("t_wall") is not None
    ]
    front_lip_contact_wall_times = [
        float(row["t_wall"])
        for row in front_lip_contact_rows
        if row.get("t_wall") is not None
    ]
    ramp_guide_contact_wall_times = [
        float(row["t_wall"])
        for row in ramp_guide_contact_rows
        if row.get("t_wall") is not None
    ]
    front_lip_min_x_m = ramp_entry_x_m - front_lip_zone_m
    classified_front_wall_times, classified_ramp_wall_times = _classify_scoop_contacts(
        lip_contact_rows,
        robot_samples,
        front_lip_min_x_m=front_lip_min_x_m,
    )
    if classified_front_wall_times or classified_ramp_wall_times:
        front_lip_contact_wall_times = classified_front_wall_times
        ramp_guide_contact_wall_times = classified_ramp_wall_times
    force_p95 = _percentile(forces, 0.95)
    contact_confirmed = bool(contact_rows)
    contact_duration_s = (
        max(contact_elapsed_times) - min(contact_elapsed_times)
        if contact_elapsed_times
        else 0.0
    )

    release_wall_s = max(contact_wall_times) if contact_wall_times else None
    release_sample = _nearest_sample(pose_samples, release_wall_s) if release_wall_s else None
    release_velocity = release_sample.get("base_velocity_m_s") if release_sample else None
    inward_velocity_m_s = -release_velocity[0] if release_velocity else None
    vertical_velocity_m_s = release_velocity[2] if release_velocity else None
    release_speed_m_s = (
        math.sqrt(sum(v * v for v in release_velocity)) if release_velocity else None
    )

    ramp_entry = _first_crossing(pose_samples, x_plane_m=ramp_entry_x_m)
    ramp_crest = _first_crossing(
        pose_samples,
        z_plane_m=ramp_crest_z_m,
        after_wall_s=release_wall_s,
    )

    pose_log_end_wall_s = pose_samples[-1]["t_wall"] if pose_samples else None
    contact_ends_before_timeout = (
        release_wall_s is not None
        and pose_log_end_wall_s is not None
        and pose_log_end_wall_s - release_wall_s >= release_window_s
    )
    roller_clear_200ms = contact_ends_before_timeout
    has_split_lip_classification = bool(
        front_lip_contact_wall_times or ramp_guide_contact_wall_times
    )
    front_lip_clear_200ms = (
        None
        if release_wall_s is None or (lip_contact_rows and not has_split_lip_classification)
        else not any(
            release_wall_s < t_wall <= release_wall_s + release_window_s
            for t_wall in front_lip_contact_wall_times
        )
    )
    legacy_lip_clear_200ms = (
        None
        if release_wall_s is None
        else not any(
            release_wall_s < t_wall <= release_wall_s + release_window_s
            for t_wall in lip_contact_wall_times
        )
    )

    required = {
        "confirmed_roller_ball_contact": contact_confirmed,
        "ball_crosses_ramp_entry_plane": ramp_entry is not None,
        "positive_inward_velocity_at_release": (
            inward_velocity_m_s is not None
            and inward_velocity_m_s >= min_directional_velocity_m_s
        ),
        "positive_vertical_velocity_at_release": (
            vertical_velocity_m_s is not None
            and vertical_velocity_m_s >= min_directional_velocity_m_s
        ),
        "roller_contact_ends_before_timeout": contact_ends_before_timeout,
        "no_roller_contact_for_release_window": roller_clear_200ms,
        "no_front_lip_contact_for_release_window": front_lip_clear_200ms,
    }
    preferred = {
        "contact_duration_lt_limit": contact_duration_s < preferred_contact_duration_s,
        "post_release_speed_gte_limit": (
            release_speed_m_s is not None and release_speed_m_s >= preferred_speed_m_s
        ),
        "force_p95_below_threshold": (
            None
            if force_p95_threshold_n is None or force_p95 is None
            else force_p95 < force_p95_threshold_n
        ),
        "ramp_crest_crossing": ramp_crest is not None,
    }

    return {
        "contact_log": str(contact_jsonl),
        "pose_log": str(pose_jsonl),
        "ball_name": ball_name,
        "parameters": {
            "ramp_entry_x_m": ramp_entry_x_m,
            "ramp_crest_z_m": ramp_crest_z_m,
            "release_window_s": release_window_s,
            "preferred_contact_duration_s": preferred_contact_duration_s,
            "preferred_speed_m_s": preferred_speed_m_s,
            "min_directional_velocity_m_s": min_directional_velocity_m_s,
            "force_p95_threshold_n": force_p95_threshold_n,
            "front_lip_zone_m": front_lip_zone_m,
            "front_lip_min_x_m": front_lip_min_x_m,
        },
        "measurements": {
            "contact_samples": len(contact_rows),
            "lip_contact_samples": len(lip_contact_rows),
            "front_lip_contact_samples": len(front_lip_contact_wall_times),
            "ramp_guide_contact_samples": len(ramp_guide_contact_wall_times),
            "legacy_lip_clear_for_release_window": legacy_lip_clear_200ms,
            "contact_duration_s": round(contact_duration_s, 4),
            "first_contact_t_s": round(min(contact_elapsed_times), 4)
            if contact_elapsed_times
            else None,
            "last_contact_t_s": round(max(contact_elapsed_times), 4)
            if contact_elapsed_times
            else None,
            "force_p95_n": round(force_p95, 4) if force_p95 is not None else None,
            "force_max_n": round(max(forces), 4) if forces else None,
            "release_wall_s": round(release_wall_s, 6) if release_wall_s else None,
            "release_base_xyz_m": release_sample.get("base_xyz_m") if release_sample else None,
            "release_base_velocity_m_s": release_velocity,
            "release_inward_velocity_m_s": inward_velocity_m_s,
            "release_vertical_velocity_m_s": vertical_velocity_m_s,
            "release_speed_m_s": release_speed_m_s,
            "pose_after_release_s": (
                pose_log_end_wall_s - release_wall_s
                if pose_log_end_wall_s is not None and release_wall_s is not None
                else None
            ),
            "ramp_entry_crossing": ramp_entry,
            "ramp_crest_crossing": ramp_crest,
        },
        "required": required,
        "preferred": preferred,
        "notes": {
            "inward_sign": "positive inward speed is computed as -base_vx because inward is toward smaller base_x",
            "front_lip_contact": "front_lip_contact_sample is classified from /gz/lip_contact_0 points near the front lip",
            "ramp_guide_contact": "ramp guide contact is classified from /gz/lip_contact_0 points behind the front lip and may be acceptable during a ramp-guided launch",
            "legacy_lip_contact": "lip_contact_sample remains the aggregate /gz/lip_contact_0 contact for compatibility",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contact_jsonl", type=Path)
    parser.add_argument("pose_jsonl", type=Path)
    parser.add_argument("--ball-name", default="ball_02")
    parser.add_argument("--ramp-entry-x-m", type=float, required=True)
    parser.add_argument("--ramp-crest-z-m", type=float, default=0.138)
    parser.add_argument("--release-window-s", type=float, default=DEFAULT_RELEASE_WINDOW_S)
    parser.add_argument(
        "--preferred-contact-duration-s",
        type=float,
        default=DEFAULT_MAX_CONTACT_DURATION_S,
    )
    parser.add_argument("--preferred-speed-m-s", type=float, default=DEFAULT_MIN_SPEED_M_S)
    parser.add_argument(
        "--min-directional-velocity-m-s",
        type=float,
        default=DEFAULT_MIN_DIRECTIONAL_VELOCITY_M_S,
    )
    parser.add_argument("--force-p95-threshold-n", type=float, default=None)
    parser.add_argument("--front-lip-zone-m", type=float, default=DEFAULT_FRONT_LIP_ZONE_M)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    result = analyze(
        args.contact_jsonl,
        args.pose_jsonl,
        ball_name=args.ball_name,
        ramp_entry_x_m=args.ramp_entry_x_m,
        ramp_crest_z_m=args.ramp_crest_z_m,
        release_window_s=args.release_window_s,
        preferred_contact_duration_s=args.preferred_contact_duration_s,
        preferred_speed_m_s=args.preferred_speed_m_s,
        min_directional_velocity_m_s=args.min_directional_velocity_m_s,
        force_p95_threshold_n=args.force_p95_threshold_n,
        front_lip_zone_m=args.front_lip_zone_m,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
