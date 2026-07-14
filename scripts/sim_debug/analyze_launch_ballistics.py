#!/usr/bin/env python3
"""Fit the intake launch vector and compare it with the basket trajectory target."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


GRAVITY_M_S2 = 9.81
COMPARISON_TOLERANCE_M = 1e-9


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


def _yaw(entry: dict[str, Any]) -> float:
    x, y, z, w = entry.get("q") or [0.0, 0.0, 0.0, 1.0]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _to_robot(robot: dict[str, Any], ball: dict[str, Any]) -> list[float]:
    yaw = _yaw(robot)
    dx = float(ball["x"]) - float(robot["x"])
    dy = float(ball["y"]) - float(robot["y"])
    return [
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
        float(ball["z"]) - float(robot["z"]),
    ]


def _samples(rows: list[dict[str, Any]], ball_name: str) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        poses = {str(pose.get("n", "")): pose for pose in row.get("poses", [])}
        robot = poses.get("tennis_robot")
        ball = poses.get(ball_name)
        if robot is None or ball is None:
            continue
        t_sim = row.get("t_sim")
        t = float(t_sim) if isinstance(t_sim, (int, float)) else float(row.get("t_wall", index))
        samples.append({"t": t, "xyz_m": _to_robot(robot, ball)})
    return samples


def _linear_fit(times: list[float], values: list[float]) -> tuple[float, float]:
    mean_t = sum(times) / len(times)
    mean_v = sum(values) / len(values)
    variance = sum((t - mean_t) ** 2 for t in times)
    if variance <= 1e-12:
        raise ValueError("launch samples do not span simulation time")
    slope = sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values)) / variance
    return mean_v - slope * mean_t, slope


def _flight_time(z0: float, vz: float, landing_z: float) -> float | None:
    discriminant = vz * vz + 2.0 * GRAVITY_M_S2 * (z0 - landing_z)
    if discriminant < 0.0:
        return None
    value = (vz + math.sqrt(discriminant)) / GRAVITY_M_S2
    return value if value > 0.0 else None


def _select_fit_samples(
    samples: list[dict[str, Any]],
    *,
    fit_x_min: float,
    fit_x_max: float,
    fit_z_min: float,
    landing_z: float,
    target_landing_x: float,
) -> tuple[list[dict[str, Any]], str]:
    def select(x_min: float, stop_at_landing: bool) -> list[dict[str, Any]]:
        selected: list[dict[str, Any]] = []
        started = False
        previous_x: float | None = None
        previous_sample: dict[str, Any] | None = None
        landing_threshold = landing_z + 0.001
        for sample in samples:
            x, _y, z = sample["xyz_m"]
            inward = previous_x is not None and x < previous_x
            eligible = x_min <= x <= fit_x_max and z >= fit_z_min
            if not started:
                if eligible and inward:
                    started = True
                    if previous_sample is not None:
                        previous_xyz = previous_sample["xyz_m"]
                        if (
                            x_min <= previous_xyz[0] <= fit_x_max
                            and previous_xyz[2] >= fit_z_min
                        ):
                            selected.append(previous_sample)
                    selected.append(sample)
                previous_x = x
                previous_sample = sample
                continue
            if stop_at_landing and z <= landing_threshold:
                break
            if eligible:
                selected.append(sample)
            elif x < x_min:
                break
            previous_x = x
            previous_sample = sample
        return selected

    fit = select(fit_x_min, stop_at_landing=False)
    if len(fit) >= 3:
        return fit, "configured_window"

    adaptive_x_min = min(fit_x_min, target_landing_x)
    return select(adaptive_x_min, stop_at_landing=True), "adaptive_to_landing"


def analyze(
    pose_path: Path,
    *,
    ball_name: str,
    fit_x_min: float,
    fit_x_max: float,
    fit_z_min: float,
    landing_z: float,
    front_row_x: float,
    target_landing_x: float,
    target_apex_z: float,
    target_front_clearance_z: float,
) -> dict[str, Any]:
    samples = _samples(_load_jsonl(pose_path), ball_name)
    fit, fit_window = _select_fit_samples(
        samples,
        fit_x_min=fit_x_min,
        fit_x_max=fit_x_max,
        fit_z_min=fit_z_min,
        landing_z=landing_z,
        target_landing_x=target_landing_x,
    )

    measurements: dict[str, Any] = {
        "fit_samples": len(fit),
        "fit_window": fit_window,
    }
    required = {
        "ballistic_fit_available": len(fit) >= 3,
        "apex_height_met": False,
        "front_row_clearance_met": False,
        "landing_depth_met": False,
    }
    if len(fit) >= 3:
        t0 = fit[0]["t"]
        times = [sample["t"] - t0 for sample in fit]
        xs = [sample["xyz_m"][0] for sample in fit]
        zs = [sample["xyz_m"][2] for sample in fit]
        x0, vx = _linear_fit(times, xs)
        ballistic_zs = [z + 0.5 * GRAVITY_M_S2 * t * t for z, t in zip(zs, times)]
        z0, vz = _linear_fit(times, ballistic_zs)
        inward_vx = -vx
        speed = math.hypot(inward_vx, vz)
        angle_deg = math.degrees(math.atan2(vz, inward_vx)) if inward_vx > 0.0 else None
        apex_t = max(0.0, vz / GRAVITY_M_S2)
        apex_z = z0 + vz * apex_t - 0.5 * GRAVITY_M_S2 * apex_t * apex_t
        flight_t = _flight_time(z0, vz, landing_z)
        landing_x = x0 - inward_vx * flight_t if flight_t is not None else None
        front_t = (x0 - front_row_x) / inward_vx if inward_vx > 0.0 else None
        front_z = (
            z0 + vz * front_t - 0.5 * GRAVITY_M_S2 * front_t * front_t
            if front_t is not None and front_t >= 0.0
            else None
        )
        predicted_range = x0 - landing_x if landing_x is not None else None
        target_vz = math.sqrt(
            max(0.0, 2.0 * GRAVITY_M_S2 * (target_apex_z - z0))
        )
        target_flight_t = _flight_time(z0, target_vz, landing_z)
        target_inward_vx = (
            (x0 - target_landing_x) / target_flight_t
            if target_flight_t is not None
            else None
        )
        target_speed = (
            math.hypot(target_inward_vx, target_vz)
            if target_inward_vx is not None
            else None
        )
        target_angle_deg = (
            math.degrees(math.atan2(target_vz, target_inward_vx))
            if target_inward_vx is not None and target_inward_vx > 0.0
            else None
        )
        measurements.update(
            {
                "release_xyz_m": [x0, fit[0]["xyz_m"][1], z0],
                "release_velocity_robot_m_s": [vx, 0.0, vz],
                "inward_velocity_m_s": inward_vx,
                "vertical_velocity_m_s": vz,
                "release_speed_m_s": speed,
                "release_angle_deg": angle_deg,
                "predicted_apex_z_m": apex_z,
                "predicted_front_row_z_m": front_z,
                "predicted_landing_x_m": landing_x,
                "predicted_range_m": predicted_range,
                "predicted_flight_time_s": flight_t,
                "target_release_velocity_robot_m_s": (
                    [-target_inward_vx, 0.0, target_vz]
                    if target_inward_vx is not None
                    else None
                ),
                "target_release_speed_m_s": target_speed,
                "target_release_angle_deg": target_angle_deg,
                "release_velocity_error_m_s": (
                    [inward_vx - target_inward_vx, vz - target_vz]
                    if target_inward_vx is not None
                    else None
                ),
            }
        )
        required.update(
            {
                "apex_height_met": (
                    apex_z + COMPARISON_TOLERANCE_M >= target_apex_z
                ),
                "front_row_clearance_met": (
                    front_z is not None
                    and front_z + COMPARISON_TOLERANCE_M >= target_front_clearance_z
                ),
                "landing_depth_met": (
                    landing_x is not None
                    and landing_x <= target_landing_x + COMPARISON_TOLERANCE_M
                ),
            }
        )

    passed = sum(value is True for value in required.values())
    return {
        "pose_log": str(pose_path),
        "ball_name": ball_name,
        "parameters": {
            "fit_x_range_m": [fit_x_min, fit_x_max],
            "fit_z_min_m": fit_z_min,
            "landing_z_m": landing_z,
            "front_row_x_m": front_row_x,
            "target_landing_x_m": target_landing_x,
            "target_apex_z_m": target_apex_z,
            "target_front_clearance_z_m": target_front_clearance_z,
        },
        "measurements": measurements,
        "required": required,
        "required_pass": f"{passed}/{len(required)}",
        "pass": passed == len(required),
        "notes": {
            "fit": "x is linear and z follows constant gravity; sparse runs expand the window only until landing",
            "direction": "negative robot-frame x is inward toward the basket",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose_jsonl", type=Path)
    parser.add_argument("--ball-name", default="ball_02")
    parser.add_argument("--fit-x-min", type=float, default=0.45)
    parser.add_argument("--fit-x-max", type=float, default=0.52)
    parser.add_argument("--fit-z-min", type=float, default=0.050)
    parser.add_argument("--landing-z", type=float, default=0.058)
    parser.add_argument("--front-row-x", type=float, default=0.35)
    parser.add_argument("--target-landing-x", type=float, default=0.28)
    parser.add_argument("--target-apex-z", type=float, default=0.135)
    parser.add_argument("--target-front-clearance-z", type=float, default=0.124)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    result = analyze(
        args.pose_jsonl,
        ball_name=args.ball_name,
        fit_x_min=args.fit_x_min,
        fit_x_max=args.fit_x_max,
        fit_z_min=args.fit_z_min,
        landing_z=args.landing_z,
        front_row_x=args.front_row_x,
        target_landing_x=args.target_landing_x,
        target_apex_z=args.target_apex_z,
        target_front_clearance_z=args.target_front_clearance_z,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
