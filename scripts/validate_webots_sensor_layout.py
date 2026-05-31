#!/usr/bin/env python3
"""Validate the Webots robot sensor layout against the baseline architecture."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "worlds" / "tennis_court.wbt"
ROBOT_DEF = "TENNIS_ROBOT"


def translation_z(block: str, def_name: str) -> float:
    match = re.search(
        rf"DEF\s+{re.escape(def_name)}\s+\w+\s*\{{.*?translation\s+[-0-9.]+\s+[-0-9.]+\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{def_name} translation not found")
    return float(match.group(1))


def main() -> None:
    text = WORLD.read_text(encoding="utf-8")
    robot_start = text.index(f"DEF {ROBOT_DEF} Robot")
    robot_block = text[robot_start:]
    robot_z = translation_z(text[robot_start : robot_start + 200], ROBOT_DEF)

    lidar_z = robot_z + translation_z(robot_block, "FRONT_LIDAR")
    camera_z = robot_z + translation_z(robot_block, "FRONT_CAMERA")
    depth_z = robot_z + translation_z(robot_block, "FRONT_DEPTH")
    ir_left_z = robot_z + translation_z(robot_block, "IR_INTAKE_LEFT")
    ir_right_z = robot_z + translation_z(robot_block, "IR_INTAKE_RIGHT")

    assert 0.25 <= lidar_z <= 0.35, f"LiDAR height {lidar_z:.3f}m outside 25-35cm"
    assert 0.40 <= camera_z <= 0.60, f"Camera height {camera_z:.3f}m outside 40-60cm"
    assert abs(camera_z - depth_z) <= 0.01, "RGB and depth camera heights should match"
    assert ir_left_z <= 0.30 and ir_right_z <= 0.30, "front collection sensors should stay low"

    print(
        "webots sensor layout ok: "
        f"lidar={lidar_z:.2f}m camera={camera_z:.2f}m depth={depth_z:.2f}m "
        f"intake_sensors={ir_left_z:.2f}/{ir_right_z:.2f}m"
    )


if __name__ == "__main__":
    main()
