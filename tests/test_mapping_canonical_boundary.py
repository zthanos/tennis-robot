from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.mapping import LidarSurveyBoundaryProvider  # noqa: E402


class CanonicalBoundaryProviderTests(unittest.TestCase):
    def test_bounds_are_derived_from_canonical_corners(self) -> None:
        path = Path(tempfile.gettempdir()) / "canonical_survey_boundary.json"
        path.write_text(
            json.dumps(
                {
                    "survey_complete": True,
                    "canonical_fence_model": {
                        "corners": {
                            "near_left": {"x_m": -7.4, "y_m": -13.8},
                            "far_left": {"x_m": -7.4, "y_m": 13.8},
                            "far_right": {"x_m": 7.4, "y_m": 13.8},
                            "near_right": {"x_m": 7.4, "y_m": -13.8},
                        }
                    },
                }
            ),
            encoding="utf-8",
        )

        provider = LidarSurveyBoundaryProvider(path)

        left = provider.get_bounds("left")
        right = provider.get_bounds("right")

        self.assertEqual((left.x_min, left.x_max), (-7.4, 0.0))
        self.assertEqual((right.x_min, right.x_max), (0.0, 7.4))
        self.assertEqual((left.y_min, left.y_max), (-13.8, 13.8))
        self.assertEqual((right.y_min, right.y_max), (-13.8, 13.8))

    def test_legacy_fence_geometry_without_canonical_is_malformed(self) -> None:
        path = Path(tempfile.gettempdir()) / "legacy_survey_boundary.json"
        path.write_text(
            json.dumps(
                {
                    "survey_complete": True,
                    "fence_geometry": {
                        "west_x": -7.4,
                        "east_x": 7.4,
                        "south_y": -13.8,
                        "north_y": 13.8,
                    },
                }
            ),
            encoding="utf-8",
        )

        provider = LidarSurveyBoundaryProvider(path)

        with self.assertRaisesRegex(RuntimeError, "canonical_fence_model"):
            provider.get_bounds("left")


if __name__ == "__main__":
    unittest.main()
