from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.mapping import (  # noqa: E402
    _Cand,
    CourtFrame,
    HalfCourtBounds,
    ServiceLineDistributionScanMission,
)


class CollectionLawnmowerTests(unittest.TestCase):
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

    def test_negative_side_lawnmower_starts_from_nearest_fence_corner(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))

        lane_count, lane_spacing, waypoints = mission._build_lawnmower_waypoints(
            -1.0, 2.8, mission._frame, mission._bounds
        )

        self.assertEqual(lane_count, 3)
        self.assertAlmostEqual(lane_spacing, 2.0)
        self.assertEqual(len(waypoints), 6)
        self.assertAlmostEqual(waypoints[0][0], -5.15)
        self.assertAlmostEqual(waypoints[0][1], 2.0)
        self.assertAlmostEqual(waypoints[1][0], -0.65)
        self.assertAlmostEqual(waypoints[1][1], 2.0)
        self.assertAlmostEqual(waypoints[2][0], -0.65)
        self.assertAlmostEqual(waypoints[2][1], 0.0)

    def test_positive_side_lawnmower_respects_nearest_sideline(self) -> None:
        mission = self._mission(HalfCourtBounds("side_pos_x", 0.0, 6.0, -3.0, 3.0))

        lane_count, _lane_spacing, waypoints = mission._build_lawnmower_waypoints(
            1.0, -2.8, mission._frame, mission._bounds
        )

        self.assertEqual(lane_count, 3)
        self.assertAlmostEqual(waypoints[0][0], 5.15)
        self.assertAlmostEqual(waypoints[0][1], -2.0)
        self.assertAlmostEqual(waypoints[1][0], 0.65)
        self.assertAlmostEqual(waypoints[1][1], -2.0)

    def test_grid_is_built_from_local_confirmed_candidates_only(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))
        mission.candidates = [_Cand(-5.0, 2.0)]
        mission.local_candidates = [_Cand(-5.0, 2.0), _Cand(-1.0, -2.0)]

        mission._rebuild_grid()

        self.assertEqual(sum(sum(row) for row in mission.grid), 2)
        self.assertEqual(mission.unassigned_candidates, 0)

    def test_active_lane_filter_rejects_off_lane_observations(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))
        mission._waypoints = [(-5.15, -2.0), (-0.65, -2.0)]
        mission._waypoint_index = 1

        self.assertTrue(mission.observation_in_active_lane(-1.0, -2.1))
        self.assertFalse(mission.observation_in_active_lane(-1.0, 0.5))
        self.assertFalse(mission.observation_in_active_lane(1.0, -2.1))

    def test_nav2_target_advances_waypoints_and_finishes(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))
        mission._state = "sweeping"
        mission._waypoints = [(-5.0, 2.0), (-0.5, 2.0)]
        mission._waypoint_index = 0

        # Far from the first waypoint -> keep targeting it.
        self.assertEqual(mission.nav2_target(-2.0, 2.0), (-5.0, 2.0))
        self.assertEqual(mission._waypoint_index, 0)

        # Arrived at the first waypoint -> advance to the second.
        self.assertEqual(mission.nav2_target(-5.0, 2.0), (-0.5, 2.0))
        self.assertEqual(mission._waypoint_index, 1)

        # Arrived at the last waypoint -> sweep completes.
        self.assertIsNone(mission.nav2_target(-0.5, 2.0))
        self.assertTrue(mission.complete)
        self.assertEqual(mission._state, "complete")

    def test_half_court_gate_rejects_out_of_bounds(self) -> None:
        mission = self._mission(HalfCourtBounds("side_neg_x", -6.0, 0.0, -3.0, 3.0))
        # inside the collect half-court
        self.assertTrue(mission.observation_in_half_court(-3.0, 0.0))
        # inside the net clearance band (too close to the net) -> excluded
        self.assertFalse(mission.observation_in_half_court(-0.3, 0.0))
        # across the net, in the other half -> excluded
        self.assertFalse(mission.observation_in_half_court(2.0, 0.0))
        # past the fence -> excluded
        self.assertFalse(mission.observation_in_half_court(-5.8, 0.0))
        # outside the sidelines -> excluded
        self.assertFalse(mission.observation_in_half_court(-3.0, 4.0))

    def test_half_court_gate_permissive_without_bounds(self) -> None:
        mission = ServiceLineDistributionScanMission(lambda: [])
        self.assertTrue(mission.observation_in_half_court(10.0, 10.0))


if __name__ == "__main__":
    unittest.main()
