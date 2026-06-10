"""Unit tests for guard-rejection recovery in Ros2LidarCourtSurvey.

Validates that when the parallelism guard rejects a waypoint proposal the robot:
  - does NOT stop (issues a non-zero command),
  - re-aligns its heading over subsequent ticks, and
  - eventually commits the corner when heading is restored.
"""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.lidar_survey import LidarSurveyConfig, LidarSurveyState, Ros2LidarCourtSurvey  # noqa: E402
from tennis_robot.survey import SurveyVision  # noqa: E402


def _make_survey(**kwargs) -> Ros2LidarCourtSurvey:
    output_file = Path(tempfile.gettempdir()) / "guard_recovery_test.json"
    cfg = LidarSurveyConfig(output_file=output_file, **kwargs)
    s = Ros2LidarCourtSurvey(cfg)
    s._started_at = 1.0
    return s


def _parallel_vision() -> SurveyVision:
    return SurveyVision(line_detected=True, line_heading_error_rad=0.0, line_confidence=1.0)


class GuardRejectionTests(unittest.TestCase):
    def test_rejected_proposal_does_not_stop_robot(self) -> None:
        """Guard rejection must produce a non-zero (keep-moving) command, not a stop."""
        survey = _make_survey()
        survey.state = LidarSurveyState.DRIVE_SIDELINE
        sideline_heading = -math.pi / 2
        survey._sideline_heading = sideline_heading
        survey._distance_traveled_m = survey.config.min_drive_before_corner_m + 0.5
        survey._last_front_range_m = survey.config.short_side_stop_range_m - 0.05

        # Robot heading is off by 25° (> waypoint_boundary_tolerance_rad = 20°)
        off_heading_yaw = sideline_heading + math.radians(25.0)

        cmd = survey._step(0.0, -4.5, off_heading_yaw, None)

        # Guard should have rejected the proposal — robot must NOT stop completely
        self.assertIsNone(survey._active_wp, "No waypoint should have been committed")
        self.assertFalse(
            cmd.linear_speed_m_s == 0.0 and cmd.angular_speed_rad_s == 0.0,
            "Guard recovery must not issue a full stop command",
        )
        self.assertEqual(survey.state, LidarSurveyState.DRIVE_SIDELINE)

    def test_guard_recovery_issues_rotation_when_close_to_fence(self) -> None:
        """When inside safety_slow_range AND heading is off, recovery rotates in place."""
        survey = _make_survey()
        survey.state = LidarSurveyState.DRIVE_SIDELINE
        sideline_heading = -math.pi / 2
        survey._sideline_heading = sideline_heading
        survey._distance_traveled_m = survey.config.min_drive_before_corner_m + 0.5
        # Very close to fence (inside safety_slow_range)
        close_range = survey.config.safety_slow_range_m * 0.5
        survey._last_front_range_m = close_range

        off_heading_yaw = sideline_heading + math.radians(25.0)
        cmd = survey._step(0.0, -4.5, off_heading_yaw, None)

        self.assertEqual(cmd.linear_speed_m_s, 0.0, "Should rotate in place when very close to fence")
        self.assertNotEqual(cmd.angular_speed_rad_s, 0.0, "Angular correction must be non-zero")

    def test_guard_reject_then_realign_then_commit(self) -> None:
        """After a guard rejection the robot re-aligns and eventually commits the corner."""
        survey = _make_survey()
        survey.state = LidarSurveyState.DRIVE_SIDELINE
        sideline_heading = -math.pi / 2
        survey._sideline_heading = sideline_heading
        survey._distance_traveled_m = survey.config.min_drive_before_corner_m + 0.5
        survey._last_front_range_m = survey.config.short_side_stop_range_m - 0.05

        # First tick: heading off → guard rejects
        off_yaw = sideline_heading + math.radians(25.0)
        cmd = survey._step(0.0, -4.5, off_yaw, None)
        self.assertIsNone(survey._active_wp)
        self.assertGreater(survey._guard_reject_count, 0)

        # Second tick: heading corrected → proposal succeeds
        cmd = survey._step(0.0, -4.5, sideline_heading, _parallel_vision())
        # Proposal accepted → active_wp set, command = stop
        self.assertIsNotNone(survey._active_wp, "Waypoint should be proposed on corrected heading")
        self.assertEqual(cmd.linear_speed_m_s, 0.0)
        self.assertEqual(survey._guard_reject_count, 0, "Reject count resets on successful proposal")

        # Third tick at same position: arrive at waypoint → state advances
        survey._step(0.0, -4.5, sideline_heading, _parallel_vision())
        self.assertEqual(survey.state, LidarSurveyState.TURN_TO_LONG_SIDE)
        self.assertIn(
            "near_left_fence_corner",
            [p["label"] for p in survey.telemetry()["survey_navigation_points"]],
        )

    def test_guard_reject_limit_finalizes_survey(self) -> None:
        """Exceeding guard_reject_limit must finalize the survey with an unrecoverable reason."""
        survey = _make_survey(guard_reject_limit=3)
        survey.state = LidarSurveyState.DRIVE_SIDELINE
        sideline_heading = -math.pi / 2
        survey._sideline_heading = sideline_heading
        survey._distance_traveled_m = survey.config.min_drive_before_corner_m + 0.5
        survey._last_front_range_m = survey.config.short_side_stop_range_m - 0.05

        off_yaw = sideline_heading + math.radians(25.0)

        for _ in range(5):  # exceed limit of 3
            survey._step(0.0, -4.5, off_yaw, None)
            if survey.state == LidarSurveyState.DONE:
                break

        self.assertEqual(survey.state, LidarSurveyState.DONE)
        bounds = survey.court_bounds
        self.assertIsNotNone(bounds)
        assert bounds is not None
        self.assertIn("unrecoverable_heading", bounds.get("failure_reason", ""))


if __name__ == "__main__":
    unittest.main()
