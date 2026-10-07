"""Regression tests for the compact wheel-before-chute acceptance gate."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/sim_debug/analyze_intake_release_criteria.py"
spec = importlib.util.spec_from_file_location("release_analyzer", SCRIPT)
analyzer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = analyzer
spec.loader.exec_module(analyzer)


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def _analyze(tmp_path: Path, contacts: list[dict]) -> dict:
    contact_path = tmp_path / "contacts.jsonl"
    pose_path = tmp_path / "poses.jsonl"
    _write_jsonl(contact_path, contacts)
    poses = []
    for t, ball_x in ((1.0, 0.60), (2.0, 0.54), (3.0, 0.49)):
        poses.append({
            "t_wall": t,
            "poses": [
                {"n": "tennis_robot", "x": 0.0, "y": 0.0, "z": 0.0,
                 "q": [0.0, 0.0, 0.0, 1.0]},
                {"n": "ball_02", "x": ball_x, "y": 0.0, "z": 0.033,
                 "q": [0.0, 0.0, 0.0, 1.0]},
            ],
        })
    _write_jsonl(pose_path, poses)
    return analyzer.analyze(
        contact_path, pose_path, ball_name="ball_02", phase="throat",
        nip_x_m=0.540, wheel_radius_m=0.060, wheel_gap_m=0.056,
        ramp_climb_z_m=0.050, ramp_crest_z_m=0.077,
        release_window_s=0.2, preferred_contact_duration_s=0.5,
        transport_target_m_s=0.4, min_directional_velocity_m_s=0.01,
        stall_speed_m_s=0.02, stall_limit_s=2.0,
        force_p95_threshold_n=None,
    )


def _wheel(wheel: str, wall: float) -> dict:
    return {"type": "roller_contact_sample", "wheel": wheel,
            "ball": "ball_02", "t_wall": wall, "t_s": wall,
            "max_force_n": 1.0}


def test_bilateral_wheels_before_chute_pass(tmp_path):
    result = _analyze(tmp_path, [
        _wheel("left", 1.5), _wheel("right", 1.5),
        {"type": "chute_contact_sample", "ball": "ball_02",
         "t_wall": 2.5, "t_s": 2.5},
    ])
    assert result["required"]["wheel_capture_before_blocking_chute_contact"] is True


def test_chute_before_wheels_fails(tmp_path):
    result = _analyze(tmp_path, [
        {"type": "chute_contact_sample", "ball": "ball_02",
         "t_wall": 1.25, "t_s": 1.25},
        _wheel("left", 1.5), _wheel("right", 1.5),
    ])
    assert result["required"]["wheel_capture_before_blocking_chute_contact"] is False


def test_missing_wheel_contact_fails(tmp_path):
    result = _analyze(tmp_path, [_wheel("left", 1.5)])
    assert result["required"]["wheel_capture_before_blocking_chute_contact"] is False
