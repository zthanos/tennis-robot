from __future__ import annotations

import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


probe = _load(
    "collection_alignment_probe",
    "scripts/sim_debug/collection_alignment_probe.py",
)
report = _load(
    "collection_alignment_report",
    "scripts/sim_debug/collection_alignment_report.py",
)


def test_probe_transforms_and_associates_camera_truth_deterministically():
    point = probe.transform_point(
        (1.0, 0.0, 0.0),
        (2.0, -1.0, 0.5),
        (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )
    assert math.isclose(point[0], 2.0, abs_tol=1e-12)
    assert math.isclose(point[1], 0.0, abs_tol=1e-12)
    assert math.isclose(point[2], 0.5, abs_tol=1e-12)

    association = probe.associate_nearest(
        (2.02, 0.01, 0.5),
        {"ball_b": (4.0, 0.0, 0.5), "ball_a": point},
        gate_m=0.5,
        ambiguity_margin_m=0.01,
    )
    assert association["status"] == "associated"
    assert association["ball_id"] == "ball_a"
    assert math.isclose(association["distance_m"], math.hypot(0.02, 0.01))


def test_report_separates_camera_residual_from_execution_lateral_error():
    camera = report.camera_summary(
        [
            {
                "event": "spatial_detection",
                "rgb_depth_delta_s": 0.01,
                "association": {
                    "status": "associated",
                    "distance_m": 0.05,
                    "residual_camera_xyz_m": [0.01, -0.02, 0.04],
                },
            },
            {
                "event": "spatial_detection",
                "rgb_depth_delta_s": 0.02,
                "association": {
                    "status": "associated",
                    "distance_m": 0.07,
                    "residual_camera_xyz_m": [0.03, -0.04, 0.05],
                },
            },
        ]
    )
    assert camera["associated_samples"] == 2
    assert math.isclose(camera["median_residual_norm_m"], 0.06)
    assert camera["median_residual_camera_xyz_m"] == [0.02, -0.03, 0.045]

    execution = report.execution_summary(
        {
            "transform": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.1},
            "execution_crossings": [
                {
                    "ball_id": "target-1",
                    "x_m": 3.0,
                    "y_m": 4.0,
                    "heading_rad": 0.0,
                }
            ],
        },
        {
            "pose_drift_m": 0.03,
            "sim_balls_odom": [
                {"def": "ball_00", "x": 3.1, "y": 4.2, "z": 0.033}
            ],
        },
    )
    crossing = execution["crossings"][0]
    assert crossing["nearest_truth_ball_id"] == "ball_00"
    assert math.isclose(crossing["longitudinal_error_m"], 0.1)
    assert math.isclose(crossing["lateral_error_m"], 0.2)

