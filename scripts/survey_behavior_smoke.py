#!/usr/bin/env python3
"""Exercise the court survey behavior without Webots."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from survey import CourtSurveyBehavior, SurveyConfig, SurveyState  # noqa: E402


def main() -> None:
    output_path = Path(tempfile.gettempdir()) / "tennis_robot_survey_smoke.csv"
    output_path.unlink(missing_ok=True)
    behavior = CourtSurveyBehavior(
        SurveyConfig(
            x_min_m=0.0,
            x_max_m=0.0,
            y_min_m=0.0,
            y_max_m=0.0,
            row_step_m=1.0,
            waypoint_tolerance_m=0.1,
            heading_tolerance_rad=0.01,
            sample_hold_s=0.0,
        ),
        output_path,
    )

    command = behavior.update(0.0, 0.0, 0.0, 2.5, 0.032)
    assert command.state == SurveyState.SAMPLE

    command = behavior.update(0.0, 0.0, 0.0, 2.5, 0.032)

    assert command.state == SurveyState.DONE
    assert output_path.exists()
    rows = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(rows) == 2
    output_path.unlink(missing_ok=True)
    print("survey behavior smoke ok: waypoint -> sample -> done")


if __name__ == "__main__":
    main()
