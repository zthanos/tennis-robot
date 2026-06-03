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


def translation_xyz(block: str, def_name: str) -> tuple[float, float, float]:
    match = re.search(
        rf"DEF\s+{re.escape(def_name)}\s+\w+\s*\{{.*?translation\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{def_name} translation not found")
    return tuple(float(value) for value in match.groups())


def rotation_angle(block: str, def_name: str) -> float:
    match = re.search(
        rf"DEF\s+{re.escape(def_name)}\s+\w+\s*\{{.*?rotation\s+[-0-9.]+\s+[-0-9.]+\s+[-0-9.]+\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{def_name} rotation not found")
    return float(match.group(1))


def box_size(block: str, def_name: str) -> tuple[float, float, float]:
    match = re.search(
        rf"DEF\s+{re.escape(def_name)}\s+\w+\s*\{{.*?geometry\s+Box\s*\{{\s*size\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{def_name} box size not found")
    return tuple(float(value) for value in match.groups())


def hopper_top_rim_size(block: str) -> tuple[float, float, float]:
    match = re.search(
        r"DEF\s+COLLECTOR_HOPPER\s+Solid\s*\{.*?Pose\s*\{\s*translation\s+-0\.18\s+0\s+0\.12.*?geometry\s+Box\s*\{\s*size\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("hopper top rim size not found")
    return tuple(float(value) for value in match.groups())


def named_cylinder_bounds(block: str, name: str) -> tuple[float, float]:
    match = re.search(
        rf"name\s+\"{re.escape(name)}\".*?boundingObject\s+Cylinder\s*\{{\s*height\s+([-0-9.]+)\s+radius\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{name} cylinder bounds not found")
    height_m, radius_m = (float(value) for value in match.groups())
    return height_m, radius_m


def robot_bounding_box_bottom(robot_z: float, block: str) -> float:
    match = re.search(
        r"name\s+\"tennis_robot\"\s+boundingObject\s+Pose\s*\{\s*translation\s+[-0-9.]+\s+[-0-9.]+\s+([-0-9.]+)\s+children\s+\[\s*Box\s*\{\s*size\s+[-0-9.]+\s+[-0-9.]+\s+([-0-9.]+)",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("robot boundingObject Pose/Box not found")
    local_z, size_z = (float(value) for value in match.groups())
    return robot_z + local_z - size_z / 2


def named_solid_translation(block: str, name: str) -> tuple[float, float, float]:
    match = re.search(
        rf"endPoint\s+Solid\s*\{{\s*translation\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)(?:(?!endPoint\s+Solid).)*?name\s+\"{re.escape(name)}\"",
        block,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"{name} translation not found")
    return tuple(float(value) for value in match.groups())


def world_box_bottom(robot_z: float, block: str, def_name: str) -> float:
    _x, _y, local_z = translation_xyz(block, def_name)
    _sx, _sy, size_z = box_size(block, def_name)
    return robot_z + local_z - size_z / 2


def main() -> None:
    text = WORLD.read_text(encoding="utf-8")
    robot_start = text.index(f"DEF {ROBOT_DEF} Robot")
    robot_block = text[robot_start:]
    robot_z = translation_z(text[robot_start : robot_start + 200], ROBOT_DEF)

    lidar_x, _lidar_y, lidar_local_z = translation_xyz(robot_block, "FRONT_LIDAR")
    lidar_z = robot_z + lidar_local_z
    camera_x, _camera_y, camera_local_z = translation_xyz(robot_block, "FRONT_CAMERA")
    depth_x, _depth_y, depth_local_z = translation_xyz(robot_block, "FRONT_DEPTH")
    collector_x, _collector_y, collector_local_z = translation_xyz(robot_block, "COLLECTOR_BASE_PLATE")
    roller_x, _roller_y, roller_local_z = translation_xyz(robot_block, "COLLECTOR_LEFT_FUNNEL")
    hopper_x, _hopper_y, hopper_local_z = translation_xyz(robot_block, "COLLECTOR_HOPPER")
    launcher_x, _launcher_y, launcher_local_z = translation_xyz(robot_block, "LAUNCHER_MODULE")
    top_frame_z = robot_z + translation_z(robot_block, "CHASSIS_FRAME_TOP_FRONT_BAR")
    camera_z = robot_z + camera_local_z
    depth_z = robot_z + depth_local_z
    ir_left_x, ir_left_y, ir_left_local_z = translation_xyz(robot_block, "IR_INTAKE_LEFT")
    ir_right_x, ir_right_y, ir_right_local_z = translation_xyz(robot_block, "IR_INTAKE_RIGHT")
    ir_left_z = robot_z + ir_left_local_z
    ir_right_z = robot_z + ir_right_local_z
    camera_tilt_rad = rotation_angle(robot_block, "FRONT_CAMERA")
    depth_tilt_rad = rotation_angle(robot_block, "FRONT_DEPTH")
    roller_width_m, roller_radius_m = named_cylinder_bounds(robot_block, "collector_intake_roller")
    _roller_cx, _roller_cy, roller_center_local_z = named_solid_translation(robot_block, "collector_intake_roller")
    roller_bottom_z = robot_z + roller_center_local_z - roller_radius_m
    _left_wheel_width_m, left_wheel_radius_m = named_cylinder_bounds(robot_block, "left_wheel")
    _left_wheel_x, _left_wheel_y, left_wheel_local_z = named_solid_translation(robot_block, "left_wheel")
    _right_wheel_width_m, right_wheel_radius_m = named_cylinder_bounds(robot_block, "right_wheel")
    _right_wheel_x, _right_wheel_y, right_wheel_local_z = named_solid_translation(robot_block, "right_wheel")
    _caster_width_m, caster_radius_m = named_cylinder_bounds(robot_block, "front_left_passive_caster_wheel")
    _caster_x, _caster_y, caster_local_z = named_solid_translation(robot_block, "front_left_passive_caster_wheel")
    wheel_bottoms = [
        robot_z + left_wheel_local_z - left_wheel_radius_m,
        robot_z + right_wheel_local_z - right_wheel_radius_m,
        robot_z + caster_local_z - caster_radius_m,
    ]
    hopper_top_size = hopper_top_rim_size(robot_block)
    funnel_bottom_z = min(
        world_box_bottom(robot_z, robot_block, "COLLECTOR_LEFT_FUNNEL"),
        world_box_bottom(robot_z, robot_block, "COLLECTOR_RIGHT_FUNNEL"),
    )
    collector_plate_bottom_z = world_box_bottom(robot_z, robot_block, "COLLECTOR_BASE_PLATE")
    robot_collision_bottom_z = robot_bounding_box_bottom(robot_z, robot_block)

    assert 0.42 <= camera_z <= 0.52, f"Camera height {camera_z:.3f}m outside 42-52cm"
    assert -0.005 <= robot_collision_bottom_z <= 0.005, (
        f"robot collision bottom {robot_collision_bottom_z:.3f}m should sit on the court, "
        "not below it where Webots will lift the robot"
    )
    for wheel_bottom_z in wheel_bottoms:
        assert -0.005 <= wheel_bottom_z <= 0.005, f"wheel bottom {wheel_bottom_z:.3f}m should touch the court"
    assert 0.17 <= camera_tilt_rad <= 0.27, f"Camera tilt {camera_tilt_rad:.3f}rad outside 10-15deg"
    assert abs(camera_tilt_rad - depth_tilt_rad) <= 0.01, "RGB and depth camera tilt should match"
    assert abs(camera_z - depth_z) <= 0.01, "RGB and depth camera heights should match"
    assert abs(camera_x - depth_x) <= 0.01, "RGB and depth camera x positions should match"
    assert collector_x < camera_x < 0.62, "Camera should sit between collector throat and front roller/flywheel opening"
    assert 0.115 <= roller_radius_m * 2 <= 0.125, f"Roller diameter {(roller_radius_m * 2):.3f}m should be 12cm"
    assert roller_width_m >= 0.50, f"Roller width {roller_width_m:.3f}m is not full-width"
    assert 0.0 <= roller_bottom_z <= 0.02, f"roller bottom {roller_bottom_z:.3f}m should sit close to court level"
    assert 0.0 <= funnel_bottom_z <= 0.03, f"floor funnel bottom {funnel_bottom_z:.3f}m should sit on the court"
    assert 0.0 <= collector_plate_bottom_z <= 0.03, f"collector plate bottom {collector_plate_bottom_z:.3f}m should sit on the court"
    assert robot_z + roller_local_z <= 0.05, "floor funnels should stay at floor level"
    assert robot_z + collector_local_z <= 0.08, "collector floor should stay low"
    assert robot_z + launcher_local_z > robot_z + hopper_local_z + 0.10, "flywheel must be mounted higher than hopper"
    assert launcher_x > hopper_x, "flywheel module should be front-facing and ahead of hopper"
    assert hopper_top_size[0] <= 0.06, "hopper should remain open-top; only a narrow rim is allowed at the top"
    assert lidar_z >= top_frame_z + 0.03, "LiDAR should sit above the protective frame"
    assert abs(lidar_x) <= 0.12, "LiDAR should stay near the robot centerline for 360-degree visibility"
    assert ir_left_z <= 0.12 and ir_right_z <= 0.12, "front collection sensors should stay at roller throat height"
    assert abs(ir_left_x - 0.54) <= 0.03 and abs(ir_right_x - 0.54) <= 0.03, "IR break beams should sit just after the roller"
    assert abs(ir_left_y + ir_right_y) <= 0.01 and 0.18 <= abs(ir_left_y - ir_right_y) <= 0.24, "IR beams should straddle the intake throat"

    print(
        "webots sensor layout ok: "
        f"lidar={lidar_z:.2f}m camera={camera_z:.2f}m depth={depth_z:.2f}m "
        f"roller_d={(roller_radius_m * 2):.2f}m roller_bottom={roller_bottom_z:.3f}m "
        f"intake_sensors={ir_left_z:.2f}/{ir_right_z:.2f}m"
    )


if __name__ == "__main__":
    main()
