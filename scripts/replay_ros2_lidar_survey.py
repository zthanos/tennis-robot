#!/usr/bin/env python3
"""Replay recorded Map Court ticks through Ros2LidarCourtSurvey."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.lidar_survey import (  # noqa: E402
    LidarSurveyConfig,
    LidarSurveyState,
    Ros2LidarCourtSurvey,
)

try:
    from tennis_robot.survey import SurveyVision  # noqa: E402
except ModuleNotFoundError:
    from dataclasses import dataclass

    @dataclass(frozen=True)
    class SurveyVision:
        center_m: float | None = None
        left_m: float | None = None
        right_m: float | None = None
        valid_count: int = 0
        obstacle_class: str | None = None
        line_detected: bool = False
        line_offset_m: float | None = None
        line_heading_error_rad: float | None = None
        line_confidence: float = 0.0
        corner_detected: bool = False
        corner_confidence: float = 0.0


DEFAULT_FIXTURE = ROOT / "runtime" / "survey_replay_latest.jsonl"


def _vision(data: dict[str, Any] | None) -> SurveyVision:
    if not data:
        return SurveyVision()
    return SurveyVision(
        center_m=data.get("center_m"),
        left_m=data.get("left_m"),
        right_m=data.get("right_m"),
        valid_count=int(data.get("valid_count") or 0),
        obstacle_class=data.get("obstacle_class"),
        line_detected=bool(data.get("line_detected", False)),
        line_offset_m=data.get("line_offset_m"),
        line_heading_error_rad=data.get("line_heading_error_rad"),
        line_confidence=float(data.get("line_confidence") or 0.0),
        corner_detected=bool(data.get("corner_detected", False)),
        corner_confidence=float(data.get("corner_confidence") or 0.0),
    )


def _load(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture", nargs="?", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--expect-success", action="store_true")
    parser.add_argument("--output-file", type=Path, default=ROOT / "runtime" / "survey_replay_boundary.json")
    args = parser.parse_args()

    ticks = _load(args.fixture)
    if not ticks:
        raise SystemExit(f"no ticks in {args.fixture}")

    cfg = LidarSurveyConfig(output_file=args.output_file)
    behavior = Ros2LidarCourtSurvey(cfg)
    transitions: list[tuple[int, str, str]] = []
    last_state: str | None = None
    for idx, tick in enumerate(ticks):
        ranges = tick.get("lidar_ranges") or None
        yaw = float(tick.get("yaw_rad") or 0.0)
        command = behavior.update(
            float(tick.get("x_m") or 0.0),
            float(tick.get("y_m") or 0.0),
            yaw,
            ranges,
            float(tick.get("dt_s") or 0.032),
            _vision(tick.get("vision")),
            lidar_angle_min=float(tick.get("lidar_angle_min", -math.pi)),
            lidar_angle_increment=tick.get("lidar_angle_increment"),
        )
        state = command.state.value
        if state != last_state:
            transitions.append((idx, state, behavior.telemetry().get("last_event", "")))
            last_state = state
        if command.state == LidarSurveyState.DONE:
            break

    bounds = behavior.court_bounds or {}
    telemetry = behavior.telemetry()
    summary = {
        "ticks_replayed": len(ticks),
        "final_state": behavior.state.value,
        "last_event": telemetry.get("last_event"),
        "status": bounds.get("status"),
        "failure_reason": bounds.get("failure_reason") or telemetry.get("failure_reason"),
        "survey_complete": bounds.get("survey_complete", False),
        "transitions": transitions,
        "boundary_distances": bounds.get("boundary_distances"),
        "navigation_points": bounds.get("navigation_points") or telemetry.get("survey_navigation_points"),
        "navigation_pattern": bounds.get("navigation_pattern") or telemetry.get("survey_pattern"),
        "canonical_fence_model": bounds.get("canonical_fence_model"),
    }
    print(json.dumps(summary, indent=2))
    if args.expect_success and not summary["survey_complete"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
