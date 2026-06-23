from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.mapping import (  # noqa: E402
    CourtFrame,
    HalfCourtBounds,
    ServiceLineDistributionScanMission,
)


class CollectionScanTargetTests(unittest.TestCase):
    def _mission(self, bounds: HalfCourtBounds) -> ServiceLineDistributionScanMission:
        mission = ServiceLineDistributionScanMission(lambda: [])
        mission._frame = CourtFrame(
            center_x_m=0.0,
            center_y_m=0.0,
            axis_length_x=1.0,
            axis_length_y=0.0,
            axis_width_x=0.0,
            axis_width_y=1.0,
        )
        mission._bounds = bounds
        return mission

    def test_selects_first_max_grid_cell_as_collection_target(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))
        mission.grid = [
            [0, 2, 2],
            [4, 1, 0],
            [4, 0, 0],
        ]

        self.assertTrue(mission._select_best_grid_target())

        self.assertEqual(mission.target_grid_cell, (1, 0))
        self.assertEqual(mission.target_pose_map, (-3.0, 2.0))

    def test_positive_side_cell_center_counts_rows_from_fence_to_net(self) -> None:
        mission = self._mission(HalfCourtBounds("side_pos_x", 0.0, 6.0, -3.0, 3.0))
        mission.grid = [
            [1, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]

        self.assertTrue(mission._select_best_grid_target())

        self.assertEqual(mission.target_grid_cell, (0, 0))
        self.assertEqual(mission.target_pose_map, (5.0, 2.0))

    def test_no_target_when_estimate_is_empty(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))

        self.assertFalse(mission._select_best_grid_target())
        self.assertIsNone(mission.target_grid_cell)
        self.assertIsNone(mission.target_pose_map)


if __name__ == "__main__":
    unittest.main()
