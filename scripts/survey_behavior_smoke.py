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
    import json
    data = json.loads(output_path.read_text(encoding="utf-8"))
    assert "status" in data                  # Court Knowledge Model status field
    assert "court_geometry" in data          # structured output present
    assert "fence_geometry" in data
    assert "west_fence_x" in data            # legacy keys still present for backward compat
    assert "survey_complete" in data
    output_path.unlink(missing_ok=True)
    print("map court behavior smoke ok: full-court route -> court knowledge model -> done")


if __name__ == "__main__":
    main()
