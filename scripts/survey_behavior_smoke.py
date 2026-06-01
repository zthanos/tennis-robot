#!/usr/bin/env python3
"""Exercise the court survey behavior without Webots."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from survey import CourtSurveyBehavior, SurveyConfig, SurveyState, SurveyVision  # noqa: E402


def main() -> None:
    output_path = Path(tempfile.gettempdir()) / "tennis_robot_survey_smoke.json"
    output_path.unlink(missing_ok=True)
    behavior = CourtSurveyBehavior(
        SurveyConfig(
            waypoint_tolerance_m=0.1,
            crossing_tolerance_m=0.05,
            drive_speed_m_s=0.8,
            turn_speed_rad_s=1.0,
            oak_min_clearance_m=0.85,
        ),
        output_path,
    )

    ranges = [2.5] * 180
    command = behavior.update(-8.0, 0.0, 0.0, ranges, 0.032, SurveyVision(center_m=2.0))
    assert command.state == SurveyState.GOTO
    assert behavior.current_target() == (-10.5, -4.0)

    behavior.waypoint_index = len(behavior.waypoints) - 1
    target = behavior.current_target()
    assert target is not None
    command = behavior.update(target[0], target[1], 0.0, ranges, 0.032, SurveyVision(center_m=2.0))

    assert command.state == SurveyState.DONE
    assert output_path.exists()
    assert '"survey_complete": true' in output_path.read_text(encoding="utf-8")
    output_path.unlink(missing_ok=True)
    print("survey behavior smoke ok: full-court route -> lidar bounds -> done")


if __name__ == "__main__":
    main()
