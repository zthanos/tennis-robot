#!/usr/bin/env python3
"""Exercise the runtime-agnostic camera-led court survey behavior."""

from __future__ import annotations

import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from survey import (  # noqa: E402
    CourtSurveyBehavior,
    LidarObstacleClassifier,
    SurveyConfig,
    SurveyState,
    SurveyVision,
)


def line(offset: float = 0.50, heading_deg: float = 0.0, corner: bool = False) -> SurveyVision:
    return SurveyVision(
        line_detected=True,
        line_offset_m=offset,
        line_heading_error_rad=math.radians(heading_deg),
        line_confidence=0.88,
        corner_detected=corner,
        corner_confidence=0.9 if corner else 0.0,
    )


def ranges_with_front_object(count: int = 180, distance_m: float = 0.50) -> list[float]:
    ranges = [math.inf] * count
    for i in range(78, 103):
        ranges[i] = distance_m
    return ranges


def ranges_with_side_fence(count: int = 180, distance_m: float = 2.2) -> list[float]:
    ranges = [math.inf] * count
    for i in range(42, 49):
        ranges[i] = distance_m
    return ranges


def advance_corner(
    behavior: CourtSurveyBehavior,
    x_m: float,
    y_m: float,
    start_yaw_deg: int,
    output_ranges: list[float],
) -> None:
    command = behavior.update(x_m, y_m, math.radians(start_yaw_deg), output_ranges, 0.032, line(corner=True))
    assert command.state == SurveyState.DETECT_CORNER
    command = behavior.update(x_m, y_m, math.radians(start_yaw_deg), output_ranges, 0.032, line(corner=True))
    assert command.state == SurveyState.TURN_AT_CORNER
    command = None
    for yaw_deg in range(start_yaw_deg + 15, start_yaw_deg + 105, 15):
        command = behavior.update(x_m, y_m, math.radians(yaw_deg), output_ranges, 0.032, line())
    assert command is not None
    assert command.state == SurveyState.MAP_EXTERNAL_BOUNDARY
    command = behavior.update(x_m, y_m, math.radians(start_yaw_deg + 90), output_ranges, 0.032, line())
    assert command.state == SurveyState.FOLLOW_LINE_WITH_OFFSET


def main() -> None:
    output_path = Path(tempfile.gettempdir()) / "tennis_robot_survey_smoke.json"
    output_path.unlink(missing_ok=True)
    cfg = SurveyConfig(drive_speed_m_s=0.8, turn_speed_rad_s=1.0)
    behavior = CourtSurveyBehavior(cfg, output_path)
    open_ranges = [math.inf] * 180

    command = behavior.update(0.0, 0.0, 0.0, open_ranges, 0.032, None)
    assert command.state == SurveyState.FIND_COURT_LINE
    assert command.base.angular_speed_rad_s > 0.0

    command = behavior.update(0.0, 0.0, 0.0, open_ranges, 0.032, line(offset=0.80, heading_deg=14.0))
    assert command.state == SurveyState.ALIGN_OUTSIDE_LINE
    assert behavior.telemetry()["path_driver"] == "camera_court_line"

    command = behavior.update(0.0, 0.0, 0.0, open_ranges, 0.032, line())
    assert command.state == SurveyState.FOLLOW_LINE_WITH_OFFSET
    assert command.base.linear_speed_m_s == 0.0

    command = behavior.update(0.4, 0.0, 0.0, ranges_with_front_object(180, 0.55), 0.032, line())
    assert command.state == SurveyState.FOLLOW_LINE_WITH_OFFSET
    assert command.base.linear_speed_m_s > 0.0

    command = behavior.update(
        0.8,
        0.0,
        0.0,
        ranges_with_front_object(180, 0.55),
        0.032,
        SurveyVision(
            obstacle_class="net",
            line_detected=True,
            line_offset_m=0.50,
            line_heading_error_rad=0.0,
            line_confidence=0.9,
        ),
    )
    assert command.state == SurveyState.FOLLOW_LINE_WITH_OFFSET
    assert behavior.telemetry()["last_event"] != "near_net"

    classifier = LidarObstacleClassifier(cfg)
    net = classifier.classify(0.0, 0.0, 0.2, "net")
    fence = classifier.classify(0.0, 3.0, 2.5, "fence")
    assert net.classification == "internal_obstacle"
    assert net.label == "net"
    assert fence.classification == "external_boundary_candidate"

    behavior.update(1.2, 0.0, 0.0, ranges_with_side_fence(), 0.032, SurveyVision(
        obstacle_class="fence",
        line_detected=True,
        line_offset_m=0.50,
        line_heading_error_rad=0.0,
        line_confidence=0.9,
    ))
    telemetry = behavior.telemetry()
    assert telemetry["external_boundary_candidate_count"] > 0
    assert telemetry["internal_obstacle_count"] > 0

    advance_corner(behavior, 23.77, 0.0, 0, open_ranges)
    advance_corner(behavior, 23.77, 10.97, 90, open_ranges)
    advance_corner(behavior, 0.0, 10.97, 180, open_ranges)
    advance_corner(behavior, 0.0, 0.0, 270, open_ranges)

    behavior._distance_traveled_m = 70.0
    command = behavior.update(0.2, 0.1, math.radians(360), open_ranges, 0.032, line())
    assert command.state == SurveyState.COMPLETE_LOOP
    command = behavior.update(0.2, 0.1, math.radians(360), open_ranges, 0.032, line())
    assert command.state == SurveyState.VALIDATE_SURVEY
    command = behavior.update(0.2, 0.1, math.radians(360), open_ranges, 0.032, line())
    assert command.state == SurveyState.DONE

    assert output_path.exists()
    data = json.loads(output_path.read_text(encoding="utf-8"))
    assert data["navigation"]["source"] == "camera_outer_court_line_fsm"
    assert data["diagnostics"]["path_driver"] == "camera_court_line"
    assert data["diagnostics"]["net_policy"] == "net is internal and ignored for perimeter completion"
    assert data["court_geometry"]["length_m"] >= 23.0
    assert data["court_geometry"]["width_m"] >= 10.0
    assert len(data["detected_court_corners"]) == 4
    assert data["external_boundary_map"]["candidates"]
    assert any(obj["label"] == "net" for obj in data["internal_objects"])
    assert "free_space_between_court_lines_and_fences" in data
    output_path.unlink(missing_ok=True)
    print("map court behavior smoke ok: camera line survey -> boundary/internal mapping -> done")


if __name__ == "__main__":
    main()
