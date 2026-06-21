#!/usr/bin/env python3
"""Record live Map Court ticks for deterministic ROS 2 survey replay."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STATUS = ROOT / "runtime" / "robot_status.json"
DEFAULT_SENSORS = ROOT / "runtime" / "robot_sensors.json"
DEFAULT_OUT = ROOT / "runtime" / "survey_replay_latest.jsonl"


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _finite_ranges(values: list[Any]) -> list[float]:
    ranges: list[float] = []
    for value in values:
        try:
            f = float(value)
        except (TypeError, ValueError):
            f = math.inf
        ranges.append(f if math.isfinite(f) else math.inf)
    return ranges


def _tick(status: dict[str, Any], sensors: dict[str, Any]) -> dict[str, Any]:
    lidar = sensors.get("front_lidar") or {}
    survey = status.get("survey") or {}
    nav = survey.get("navigation") or {}
    return {
        "recorded_at": time.time(),
        "status_updated_at": status.get("updated_at"),
        "sensor_updated_at": sensors.get("updated_at"),
        "mode": status.get("mode"),
        "requested_mode": status.get("requested_mode"),
        "survey_state": survey.get("state"),
        "survey_event": nav.get("last_event"),
        "x_m": status.get("robot_x_m"),
        "y_m": status.get("robot_y_m"),
        "yaw_rad": status.get("robot_yaw_rad"),
        "cmd_linear_m_s": status.get("cmd_linear_m_s"),
        "cmd_angular_rad_s": status.get("cmd_angular_rad_s"),
        "dt_s": 0.032,
        "lidar_ranges": _finite_ranges(lidar.get("ranges_m") or []),
        "lidar_angle_min": float(lidar.get("angle_min", -math.pi)),
        "lidar_angle_increment": float(
            lidar.get("angle_increment", 2.0 * math.pi / max(1, len(lidar.get("ranges_m") or [])))
        ),
        "vision": survey.get("vision"),
        "navigation": nav,
        "bounds": survey.get("bounds"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS)
    parser.add_argument("--sensor-file", type=Path, default=DEFAULT_SENSORS)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--interval-s", type=float, default=0.10)
    parser.add_argument("--duration-s", type=float, default=900.0)
    parser.add_argument("--map-court-only", action="store_true", default=True)
    args = parser.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    last_key: tuple[Any, Any] | None = None
    count = 0
    with args.out.open("w", encoding="utf-8") as handle:
        while time.time() - started <= args.duration_s:
            status = _read_json(args.status_file)
            sensors = _read_json(args.sensor_file)
            if status and sensors:
                if not args.map_court_only or status.get("requested_mode") == "map_court" or status.get("mode") == "map_court":
                    tick = _tick(status, sensors)
                    key = (tick["status_updated_at"], tick["sensor_updated_at"])
                    if key != last_key and tick["lidar_ranges"]:
                        handle.write(json.dumps(tick, sort_keys=True) + "\n")
                        handle.flush()
                        last_key = key
                        count += 1
                elif count > 0:
                    break
            time.sleep(max(0.02, args.interval_s))
    print(f"recorded {count} survey replay ticks -> {args.out}")


if __name__ == "__main__":
    main()
