"""Zone gates for sim collection credit (collection_scoring.onboard_ball_zone)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_scoring import onboard_ball_zone


def test_bin_interior_counts_as_bin():
    assert onboard_ball_zone(0.20, 0.0, 0.058) == "bin"
    assert onboard_ball_zone(0.05, -0.13, 0.056) == "bin"


def test_deck_parked_ball_counts_as_deck():
    # Observed run-2 landings (collection-route-debug-log-el.md #3):
    assert onboard_ball_zone(0.061, 0.244, 0.058) == "deck"   # ball_06 beside bin
    assert onboard_ball_zone(-0.224, 0.317, 0.058) == "deck"  # ball_00 rear deck
    assert onboard_ball_zone(-0.019, 0.247, 0.058) == "deck"  # ball_05 beside bin


def test_ground_ball_is_not_onboard():
    assert onboard_ball_zone(0.60, 0.05, 0.033) is None   # plowed in the funnel
    assert onboard_ball_zone(1.30, 0.0, 0.033) is None    # approach target
    assert onboard_ball_zone(0.20, 0.0, 0.033) is None    # under the chassis


def test_off_robot_ball_is_not_onboard():
    assert onboard_ball_zone(0.20, 0.60, 0.058) is None   # beside the robot
    assert onboard_ball_zone(-0.60, 0.0, 0.058) is None   # behind the robot
