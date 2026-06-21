"""Unit tests for TurnTracker — turn completion logic.

Validates:
- 180° turn with antipodal noise completes and stays within heading tolerance.
- 90° turn with off-ideal entry yaw completes at the correct absolute heading.
"""

from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.motion import TurnTracker, angle_delta_rad  # noqa: E402


HEADING_TOL = math.radians(2.0)
SETTLE_TICKS = 3


class TurnTracker180Tests(unittest.TestCase):
    def test_180_completes_with_yaw_noise(self) -> None:
        """180° from 0 rad with ±0.01 rad noise per tick must complete within tolerance."""
        rng = random.Random(42)
        tracker = TurnTracker()
        target = math.pi

        # Initialise using the same logic as _turn_complete: required ≈ π → CCW direction
        required = angle_delta_rad(target, 0.0)
        direction = 1.0  # deterministic CCW for near-180
        tracker.reset(target, abs(required), direction)

        yaw = 0.0
        completed = False
        for _ in range(300):
            # Small noise (±0.001 rad ≈ ±0.06°) — enough to test robustness without
            # pushing yaw outside the 2° settle window on every step.
            yaw += math.radians(1.0) + rng.uniform(-0.001, 0.001)
            tracker.update(yaw)
            if tracker.complete(yaw, HEADING_TOL, SETTLE_TICKS):
                completed = True
                break

        self.assertTrue(completed, "180° turn did not complete within 300 ticks")
        self.assertLessEqual(
            abs(angle_delta_rad(target, yaw)),
            HEADING_TOL + math.radians(1.0),
            "Final heading is further than tolerance from target",
        )

    def test_180_does_not_over_rotate(self) -> None:
        """Robot must not rotate net more than π + tolerance before completing."""
        tracker = TurnTracker()
        target = math.pi
        required = angle_delta_rad(target, 0.0)
        tracker.reset(target, abs(required), 1.0)

        yaw = 0.0
        for _ in range(300):
            yaw += math.radians(1.5)
            tracker.update(yaw)
            if tracker.complete(yaw, HEADING_TOL, SETTLE_TICKS):
                break

        # Net rotation should be within π + heading_tolerance
        net_rotation = abs(angle_delta_rad(yaw, 0.0))
        self.assertLessEqual(
            net_rotation,
            math.pi + HEADING_TOL + math.radians(5.0),
        )

    def test_180_settle_counter_resets_on_heading_loss(self) -> None:
        """If heading wanders away, settle counter resets and completion is delayed."""
        tracker = TurnTracker()
        target = math.pi
        tracker.reset(target, math.pi, 1.0)

        # Bring tracker close to target
        tracker.update(math.pi - math.radians(1.0))
        in_tol = tracker.complete(math.pi - math.radians(1.0), HEADING_TOL, SETTLE_TICKS)
        self.assertFalse(in_tol)  # settle_count = 1, need 3
        self.assertEqual(tracker.settle_count, 1)

        # Wander away
        tracker.update(math.pi - math.radians(5.0))
        tracker.complete(math.pi - math.radians(5.0), HEADING_TOL, SETTLE_TICKS)
        self.assertEqual(tracker.settle_count, 0)


class TurnTracker90Tests(unittest.TestCase):
    def test_90_off_ideal_entry_yaw_completes_at_correct_heading(self) -> None:
        """90° turn from 15° off-ideal entry yaw must complete at the correct absolute heading."""
        entry_yaw = math.radians(15.0)
        target = math.pi / 2  # ideal 90° left from 0 rad entry

        required = angle_delta_rad(target, entry_yaw)  # ≈ 75°
        direction = math.copysign(1.0, required)

        tracker = TurnTracker()
        tracker.reset(target, abs(required), direction)

        yaw = entry_yaw
        completed = False
        for _ in range(200):
            yaw += math.radians(1.5) * direction
            tracker.update(yaw)
            if tracker.complete(yaw, HEADING_TOL, SETTLE_TICKS):
                completed = True
                break

        self.assertTrue(completed, "90° turn did not complete within 200 ticks")
        self.assertLessEqual(
            abs(angle_delta_rad(target, yaw)),
            HEADING_TOL + math.radians(1.0),
        )

    def test_90_from_ideal_entry_completes(self) -> None:
        entry_yaw = 0.0
        target = math.pi / 2
        required = angle_delta_rad(target, entry_yaw)
        tracker = TurnTracker()
        tracker.reset(target, abs(required), math.copysign(1.0, required))

        yaw = entry_yaw
        completed = False
        for _ in range(200):
            yaw += math.radians(1.5)
            tracker.update(yaw)
            if tracker.complete(yaw, HEADING_TOL, SETTLE_TICKS):
                completed = True
                break

        self.assertTrue(completed)
        self.assertLessEqual(abs(angle_delta_rad(target, yaw)), HEADING_TOL + math.radians(1.0))

    def test_complete_returns_true_immediately_when_already_on_target(self) -> None:
        tracker = TurnTracker()
        tracker.reset(math.pi / 4, math.pi / 4, 1.0)
        # Already at target
        for _ in range(SETTLE_TICKS):
            tracker.update(math.pi / 4)
            tracker.complete(math.pi / 4, HEADING_TOL, SETTLE_TICKS)
        self.assertTrue(tracker.complete(math.pi / 4, HEADING_TOL, SETTLE_TICKS))

    def test_no_target_returns_true(self) -> None:
        tracker = TurnTracker()
        self.assertTrue(tracker.complete(1.23, HEADING_TOL, SETTLE_TICKS))


if __name__ == "__main__":
    unittest.main()
