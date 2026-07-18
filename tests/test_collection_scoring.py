"""Zone gates and stable basket-retention confirmation."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_scoring import (
    CreditReconciler,
    SimRetentionTracker,
    onboard_ball_zone,
    retained_ball_still_in_bin,
)


def test_bin_interior_counts_as_bin():
    assert onboard_ball_zone(0.20, 0.0, 0.058) == "bin"
    assert onboard_ball_zone(0.05, -0.13, 0.056) == "bin"


def test_deck_parked_ball_counts_as_deck():
    # Observed run-2 landings (archive/collection-route-debug-log-el.md #3):
    assert onboard_ball_zone(0.061, 0.244, 0.058) == "deck"   # ball_06 beside bin
    assert onboard_ball_zone(-0.224, 0.317, 0.058) == "deck"  # ball_00 rear deck
    assert onboard_ball_zone(-0.019, 0.247, 0.058) == "deck"  # ball_05 beside bin


def test_receiver_ball_is_not_classified_as_bin():
    # Latest collect-route run stopped here, outside the x=0.42 retention lip.
    assert onboard_ball_zone(0.439, 0.018, 0.066) == "receiver"
    assert onboard_ball_zone(0.446, -0.055, 0.080) == "receiver"
    assert onboard_ball_zone(0.410, 0.0, 0.070) == "receiver"


def test_retained_monitor_has_small_exit_hysteresis():
    assert retained_ball_still_in_bin(0.410, 0.0, 0.070)
    assert not retained_ball_still_in_bin(0.439, 0.0, 0.070)


def test_ground_ball_is_not_onboard():
    assert onboard_ball_zone(0.60, 0.05, 0.033) is None   # plowed in the funnel
    assert onboard_ball_zone(1.30, 0.0, 0.033) is None    # approach target
    assert onboard_ball_zone(0.20, 0.0, 0.033) is None    # under the chassis


def test_off_robot_ball_is_not_onboard():
    assert onboard_ball_zone(0.20, 0.60, 0.058) is None   # beside the robot
    assert onboard_ball_zone(-0.60, 0.0, 0.058) is None   # behind the robot


def test_retention_requires_continuous_bin_dwell():
    tracker = SimRetentionTracker(dwell_s=0.75)

    entry = tracker.update("ball_03", "receiver", 10.0)
    assert entry.event == "basket_entry_candidate"
    assert not entry.retained

    crossed = tracker.update("ball_03", "bin", 10.2)
    assert crossed.event == "basket_bin_candidate"
    assert not crossed.retained
    assert not tracker.update("ball_03", "bin", 10.94).retained

    retained = tracker.update("ball_03", "bin", 10.96)
    assert retained.event == "basket_retained"
    assert retained.retained


def test_retention_dwell_resets_when_ball_rolls_back():
    tracker = SimRetentionTracker(dwell_s=0.75)
    tracker.update("ball_04", "receiver", 20.0)
    tracker.update("ball_04", "bin", 20.1)

    rolled_back = tracker.update("ball_04", "receiver", 20.5)
    assert rolled_back.event == "basket_bin_candidate_lost"
    assert not rolled_back.retained

    tracker.update("ball_04", "bin", 20.6)
    assert not tracker.update("ball_04", "bin", 21.2).retained
    assert tracker.update("ball_04", "bin", 21.36).retained


def test_receiver_candidate_loss_is_logged():
    tracker = SimRetentionTracker()
    tracker.update("ball_05", "receiver", 30.0)
    lost = tracker.update("ball_05", None, 30.2)
    assert lost.event == "basket_entry_lost"
    assert not lost.retained


def test_reconciler_tolerates_beam_leading_truth():
    # Normal capture: beam break ~1 s before the retention dwell completes.
    r = CreditReconciler(tolerance_s=5.0)
    r.on_beam_credit(10.0)
    assert r.poll(11.0) is None  # transient, within tolerance
    r.on_truth_retained(11.2)
    assert r.poll(30.0) is None  # matched: never reported


def test_reconciler_reports_missed_credit_once_per_delta():
    # Ball retained but beam never fired (e.g. blocked by a full basket).
    r = CreditReconciler(tolerance_s=5.0)
    r.on_truth_retained(10.0)
    assert r.poll(12.0) is None
    report = r.poll(15.1)
    assert report == {
        "event": "beam_missed_credit",
        "beam_count": 0,
        "truth_count": 1,
        "delta": -1,
    }
    assert r.poll(16.0) is None  # same delta: reported once
    r.on_truth_retained(20.0)  # a second miss re-reports
    assert r.poll(26.0)["delta"] == -2


def test_reconciler_reports_false_credit():
    r = CreditReconciler(tolerance_s=5.0)
    r.on_beam_credit(10.0)
    report = r.poll(15.1)
    assert report is not None
    assert report["event"] == "beam_false_credit"
    assert report["delta"] == 1
