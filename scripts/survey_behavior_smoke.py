#!/usr/bin/env python3
"""Exercise the runtime-agnostic court survey behavior."""

from __future__ import annotations

import sys
import tempfile
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from survey import CourtSurveyBehavior, SurveyConfig, SurveyState, SurveyVision  # noqa: E402


def main() -> None:
    output_path = Path(tempfile.gettempdir()) / "tennis_robot_survey_smoke.json"
    output_path.unlink(missing_ok=True)
    behavior = CourtSurveyBehavior(
        SurveyConfig(
            drive_speed_m_s=0.8,
            turn_speed_rad_s=1.0,
        ),
        output_path,
    )

    open_ranges = [math.inf] * 180
    command = behavior.update(-8.0, 0.0, 0.0, open_ranges, 0.032, None)
    assert command.state == SurveyState.FIND_FIRST_OBSTACLE
    assert command.base.linear_speed_m_s > 0.0

    ranges = [math.inf] * 180
    command = behavior.update(-8.0, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=1.2))
    assert command.state == SurveyState.FIND_FIRST_OBSTACLE
    assert command.base.linear_speed_m_s > 0.0

    behavior.update(-7.4, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=1.2))
    command = behavior.update(-6.8, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=1.2))
    assert command.state == SurveyState.APPROACH_NET
    assert behavior.current_target() is None
    assert behavior.telemetry()["sensor_only_navigation"] is True
    assert behavior.telemetry()["last_event"] == "first_obstacle_net"
    assert behavior.telemetry()["first_obstacle_kind"] == "net"

    fence_behavior = CourtSurveyBehavior(
        SurveyConfig(
            drive_speed_m_s=0.8,
            turn_speed_rad_s=1.0,
        ),
        output_path,
    )
    fence_ranges = [math.inf] * 180
    for i in range(78, 102):
        fence_ranges[i] = 0.9
    command = fence_behavior.update(-8.0, 0.0, 0.0, fence_ranges, 0.032, None)
    assert command.state == SurveyState.TURN_LEFT_AT_FENCE_1
    assert fence_behavior.telemetry()["last_event"] == "first_obstacle_fence"
    assert fence_behavior.telemetry()["first_obstacle_kind"] == "fence"

    turn_behavior = CourtSurveyBehavior(
        SurveyConfig(
            drive_speed_m_s=0.8,
            turn_speed_rad_s=1.0,
        ),
        output_path,
    )
    turn_behavior.update(-8.0, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=1.2, obstacle_class="net"))
    turn_behavior.update(-7.4, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=1.2, obstacle_class="net"))
    turn_behavior.update(-6.8, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=0.4, obstacle_class="net"))
    assert turn_behavior.state == SurveyState.TURN_LEFT_AT_NET
    command = None
    for yaw_deg in range(10, 100, 10):
        command = turn_behavior.update(
            -6.8,
            0.0,
            math.radians(yaw_deg),
            ranges,
            0.032,
            SurveyVision(center_m=0.4, obstacle_class="net"),
        )
    assert command is not None
    assert command.state == SurveyState.FOLLOW_NET_TO_FENCE
    assert turn_behavior.telemetry()["last_event"] == "left_turn_complete"

    behavior._failure_reason = "smoke forced finish"
    behavior._finish()
    command = behavior.update(-8.0, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=2.0))

    assert command.state == SurveyState.DONE
    assert output_path.exists()
    import json
    data = json.loads(output_path.read_text(encoding="utf-8"))
    assert "status" in data                  # Court Knowledge Model status field
    assert "court_geometry" in data          # structured output present
    assert "fence_geometry" in data
    assert "survey_complete" in data
    assert data["navigation"]["source"] == "map_court_sensor_fsm"
    output_path.unlink(missing_ok=True)
    print("map court behavior smoke ok: sensor-only survey -> court knowledge model -> done")


if __name__ == "__main__":
    main()
