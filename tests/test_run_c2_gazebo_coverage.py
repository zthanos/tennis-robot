from __future__ import annotations

import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_c2_gazebo_coverage", ROOT / "scripts" / "run_c2_gazebo_coverage.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_set_pose_request_uses_manifest_yaw():
    request = MODULE._set_pose_request({"x_m": -10.0, "y_m": -2.0, "yaw_rad": 0.6})
    assert "x: -10.0" in request
    assert "y: -2.0" in request
    assert f"z: {math.sin(0.3)}" in request
    assert f"w: {math.cos(0.3)}" in request


def test_rgb_and_depth_gazebo_fov_are_aligned_for_pixel_roi_fusion():
    source = (
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/oak_d.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert source.count("<horizontal_fov>1.204</horizontal_fov>") == 2
