#!/usr/bin/env python3
"""Replay recorded survey cmd_vel commands.

By default this is a dry run. Use --publish inside the ROS/Gazebo container to
publish the recorded commands to /cmd_vel with recorded timing.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FIXTURE = ROOT / "runtime" / "survey_replay_latest.jsonl"


def _load(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _command(row: dict[str, Any]) -> tuple[float, float] | None:
    lin = row.get("cmd_linear_m_s")
    ang = row.get("cmd_angular_rad_s")
    if lin is None or ang is None:
        return None
    try:
        return float(lin), float(ang)
    except (TypeError, ValueError):
        return None


def _delay(prev: dict[str, Any] | None, row: dict[str, Any], speedup: float) -> float:
    if prev is None:
        return 0.0
    now = row.get("recorded_at")
    before = prev.get("recorded_at")
    try:
        dt = float(now) - float(before)
    except (TypeError, ValueError):
        dt = float(row.get("dt_s") or 0.032)
    if not math.isfinite(dt) or dt < 0.0:
        dt = float(row.get("dt_s") or 0.032)
    return max(0.0, dt / max(0.001, speedup))


def _dry_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    command_count = 0
    missing = 0
    moving = 0
    rotating = 0
    stopped = 0
    linear_max = 0.0
    angular_max = 0.0
    first_move: dict[str, Any] | None = None
    for idx, row in enumerate(rows):
        cmd = _command(row)
        if cmd is None:
            missing += 1
            continue
        command_count += 1
        lin, ang = cmd
        linear_max = max(linear_max, abs(lin))
        angular_max = max(angular_max, abs(ang))
        if abs(lin) > 1e-6:
            moving += 1
            if first_move is None:
                first_move = {
                    "tick": idx,
                    "linear_m_s": round(lin, 3),
                    "angular_rad_s": round(ang, 3),
                    "survey_state": row.get("survey_state"),
                    "survey_event": row.get("survey_event"),
                }
        elif abs(ang) > 1e-6:
            rotating += 1
        else:
            stopped += 1
    return {
        "ticks": len(rows),
        "commands": command_count,
        "missing_commands": missing,
        "moving_ticks": moving,
        "rotating_ticks": rotating,
        "stopped_ticks": stopped,
        "max_abs_linear_m_s": round(linear_max, 3),
        "max_abs_angular_rad_s": round(angular_max, 3),
        "first_move": first_move,
    }


def _publish(rows: list[dict[str, Any]], speedup: float, topic: str) -> None:
    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("survey_cmd_vel_replay")
    pub = node.create_publisher(Twist, topic, 1)
    prev: dict[str, Any] | None = None
    try:
        for row in rows:
            cmd = _command(row)
            if cmd is None:
                prev = row
                continue
            time.sleep(_delay(prev, row, speedup))
            msg = Twist()
            msg.linear.x = cmd[0]
            msg.angular.z = cmd[1]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            prev = row
        stop = Twist()
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture", nargs="?", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--speedup", type=float, default=1.0)
    args = parser.parse_args()

    rows = _load(args.fixture)
    summary = _dry_summary(rows)
    print(json.dumps(summary, indent=2))
    if summary["missing_commands"]:
        raise SystemExit("fixture does not contain a complete command trace")
    if args.publish:
        _publish(rows, args.speedup, args.topic)


if __name__ == "__main__":
    main()
