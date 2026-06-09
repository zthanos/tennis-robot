from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.lidar_survey import LidarSurveyConfig, LidarSurveyState, Ros2LidarCourtSurvey, _ActiveWaypoint  # noqa: E402
from tennis_robot.mapping import LidarSurveyBoundaryProvider  # noqa: E402
from tennis_robot.survey import SurveyVision  # noqa: E402


def _scan_with_sector(center_rad: float, half_rad: float, distance_m: float) -> list[float]:
    ranges = [math.inf] * 360
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / len(ranges)
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        if abs(angle - center_rad) <= half_rad:
            ranges[i] = distance_m
    return ranges


def _parallel_vision() -> SurveyVision:
    return SurveyVision(line_detected=True, line_heading_error_rad=0.0, line_confidence=1.0)


class LongSideTransitionTests(unittest.TestCase):
    def test_baseline_to_sideline_target_is_left_turn_toward_270(self) -> None:
        survey = Ros2LidarCourtSurvey()
        survey._baseline_heading = math.pi
        survey._active_wp = _ActiveWaypoint(
            "near_baseline_fence_standoff",
            -6.21,
            0.0,
            "test",
            LidarSurveyState.TURN_TO_SIDELINE,
            "baseline_fence_reached",
        )

        survey._commit_active_waypoint(math.pi)

        self.assertAlmostEqual(math.degrees(survey._sideline_heading or 0.0) % 360, 270.0)

    def test_turn_completion_waits_for_parallel_boundary_line(self) -> None:
        survey = Ros2LidarCourtSurvey()
        survey.state = LidarSurveyState.TURN_TO_SIDELINE
        survey._sideline_heading = -math.pi / 2
        survey._pending_turn_heading_reached = True

        command = survey._step(0.0, 0.0, -math.pi / 2, None)
        self.assertEqual(survey.state, LidarSurveyState.TURN_TO_SIDELINE)
        self.assertNotEqual(command.angular_speed_rad_s, 0.0)
        self.assertEqual(survey.telemetry()["last_corner_reject_reason"], "turn_waiting_for_parallel_boundary")

        command = survey._step(0.0, 0.0, -math.pi / 2, _parallel_vision())
        self.assertEqual(survey.state, LidarSurveyState.DRIVE_SIDELINE)
        self.assertEqual(command.linear_speed_m_s, 0.0)

    def test_long_side_starts_when_front_range_triggers_corner(self) -> None:
        output_file = Path(tempfile.gettempdir()) / "tennis_robot_long_side_test.json"
        survey = Ros2LidarCourtSurvey(LidarSurveyConfig(output_file=output_file))
        survey._started_at = 1.0
        survey.state = LidarSurveyState.DRIVE_SIDELINE
        survey._sideline_heading = -math.pi / 2
        survey._last_front_range_m = survey.config.short_side_stop_range_m - 0.05
        survey._distance_traveled_m = survey.config.min_drive_before_corner_m + 0.5

        # Tick 1: detect corner → propose waypoint (stop, active_wp set)
        command = survey._step(0.0, -4.5, -math.pi / 2, _parallel_vision())
        self.assertEqual(command.linear_speed_m_s, 0.0)
        self.assertIsNotNone(survey._active_wp)

        # Tick 2: robot is already at the waypoint position → commit → advance state
        survey._step(0.0, -4.5, -math.pi / 2, _parallel_vision())
        self.assertEqual(survey.state, LidarSurveyState.TURN_TO_LONG_SIDE)
        self.assertIn(
            "near_left_fence_corner",
            [p["label"] for p in survey.telemetry()["survey_navigation_points"]],
        )


class PatternValidationTests(unittest.TestCase):
    def test_pattern_complete_when_all_six_corners_recorded_in_order(self) -> None:
        survey = Ros2LidarCourtSurvey()

        for index, label in enumerate([
            "near_net_standoff",
            "near_baseline_fence_standoff",
            "near_left_fence_corner",
            "far_left_fence_corner",
            "far_right_fence_corner",
            "near_right_fence_corner",
        ]):
            survey._add_survey_navigation_point(label, float(index), float(-index), "test")

        status = survey._survey_pattern_status()
        self.assertTrue(status["complete"])
        self.assertEqual(status["missing"], [])
        self.assertEqual(len(survey.telemetry()["survey_route"]), 6)

    def test_pattern_reports_missing_future_points(self) -> None:
        survey = Ros2LidarCourtSurvey()
        survey._add_survey_navigation_point("near_net_standoff", 0.0, 0.0, "test")
        survey._add_survey_navigation_point("near_baseline_fence_standoff", 1.0, 0.0, "test")

        status = survey._survey_pattern_status()

        self.assertFalse(status["complete"])
        self.assertEqual(status["matched"], ["near_net_standoff", "near_baseline_fence_standoff"])
        self.assertEqual(status["missing"][0], "near_left_fence_corner")
        self.assertEqual(status["expected_next"], "near_left_fence_corner")

    def test_long_side_timeout_reports_waiting_for_far_left_corner(self) -> None:
        output_file = Path(tempfile.gettempdir()) / "tennis_robot_far_left_timeout.json"
        survey = Ros2LidarCourtSurvey(LidarSurveyConfig(output_file=output_file))
        survey._started_at = 1.0
        survey._add_survey_navigation_point("near_net_standoff", 0.0, 0.0, "test")
        survey._add_survey_navigation_point("near_baseline_fence_standoff", -6.21, 0.0, "test")
        survey._add_survey_navigation_point("near_left_fence_corner", -6.37, -4.46, "test")

        survey._finalize_full_survey("long_side_drive_timeout")

        bounds = survey.court_bounds
        self.assertIsNotNone(bounds)
        assert bounds is not None
        self.assertFalse(bounds["navigation_pattern"]["complete"])
        self.assertEqual(bounds["navigation_pattern"]["expected_next"], "far_left_fence_corner")
        self.assertEqual(
            bounds["failure_reason"],
            "long_side_drive_timeout_waiting_for_far_left_fence_corner",
        )
        self.assertIn(
            "missing_route_points:far_left_fence_corner",
            bounds["canonical_fence_model"]["validation_errors"][0],
        )

    def test_geometry_valid_when_corners_match_court_dimensions(self) -> None:
        survey = Ros2LidarCourtSurvey()
        cl = survey.config.expected_court_length_m  # 23.77 m — long axis (X)
        cw = survey.config.expected_court_width_m   # 10.97 m — short axis (Y)

        # Gazebo frame: X = long axis, Y = short axis.
        # Robot approaches net along +X, 180° turn, baseline approach along -X.
        # Left turn after baseline drives along -Y toward left sideline corner.
        coords = [
            ("near_net_standoff", 0.0, 0.0),
            ("near_baseline_fence_standoff", -cl / 2, 0.0),
            ("near_left_fence_corner", -cl / 2, -cw / 2),
            ("far_left_fence_corner",  cl / 2, -cw / 2),
            ("far_right_fence_corner", cl / 2,  cw / 2),
            ("near_right_fence_corner", -cl / 2, cw / 2),
        ]
        for label, x, y in coords:
            survey._add_survey_navigation_point(label, x, y, "test")

        status = survey._survey_pattern_status()
        self.assertTrue(status["complete"])
        self.assertTrue(status["geometry_valid"])

    def test_geometry_invalid_when_near_left_corner_is_too_early(self) -> None:
        survey = Ros2LidarCourtSurvey()
        cl = survey.config.expected_court_length_m
        cw = survey.config.expected_court_width_m

        # Near-left corner only 0.25 m from the baseline standoff (should be ~cw/2 ≈ 5.5 m).
        coords = [
            ("near_net_standoff", 0.0, 0.0),
            ("near_baseline_fence_standoff", -cl / 2, 0.0),
            ("near_left_fence_corner", -cl / 2, -0.25),
            ("far_left_fence_corner",  cl / 2, -cw / 2),
            ("far_right_fence_corner", cl / 2,  cw / 2),
            ("near_right_fence_corner", -cl / 2, cw / 2),
        ]
        for label, x, y in coords:
            survey._add_survey_navigation_point(label, x, y, "test")

        status = survey._survey_pattern_status()

        self.assertTrue(status["complete"])
        self.assertFalse(status["geometry_valid"])
        self.assertFalse(status["geometry_checks"]["near_baseline_to_near_left"]["ok"])

    def test_finalize_outputs_canonical_model_for_mapping(self) -> None:
        output_file = Path(tempfile.gettempdir()) / "tennis_robot_canonical_map_court.json"
        survey = Ros2LidarCourtSurvey(LidarSurveyConfig(output_file=output_file))
        survey._started_at = 1.0
        cl = survey.config.expected_court_length_m
        cw = survey.config.expected_court_width_m

        coords = [
            ("near_net_standoff", 0.0, 0.0),
            ("near_baseline_fence_standoff", -cl / 2, 0.0),
            ("near_left_fence_corner", -cl / 2, -cw / 2),
            ("far_left_fence_corner",  cl / 2, -cw / 2),
            ("far_right_fence_corner", cl / 2,  cw / 2),
            ("near_right_fence_corner", -cl / 2, cw / 2),
        ]
        for label, x, y in coords:
            survey._add_survey_navigation_point(label, x, y, "test")

        survey._finalize_full_survey(None)

        bounds = survey.court_bounds
        self.assertIsNotNone(bounds)
        assert bounds is not None
        self.assertTrue(bounds["survey_complete"])
        self.assertNotIn("fence_geometry", bounds)
        self.assertEqual(bounds["canonical_fence_model"]["status"], "VALID")
        self.assertIn("near_left", bounds["canonical_fence_model"]["corners"])

        provider = LidarSurveyBoundaryProvider(output_file)
        left = provider.get_bounds("left")
        # west_x = min corner X = -cl/2 (near and far baseline fence); south_y = -cw/2
        self.assertAlmostEqual(left.x_min, -cl / 2)
        self.assertAlmostEqual(left.y_min, -cw / 2)


if __name__ == "__main__":
    unittest.main()
