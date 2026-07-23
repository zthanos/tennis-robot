from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.perception_diagnostics import (
    format_no_targets_diagnostic,
    summarize_spatial_fusion,
)


def test_summary_preserves_out_of_domain_reason_and_measured_range():
    summary = summarize_spatial_fusion(
        [
            {
                "has_spatial": False,
                "estimated_distance_m": 3.9837,
                "depth_quality": 1.0,
                "spatial_rejection_reason": "calibration_out_of_domain",
            }
        ],
        calibration_range_min_m=1.0215,
        calibration_range_max_m=2.9799,
    )

    assert summary["detections_2d"] == 1
    assert summary["spatial_accepted"] == 0
    assert summary["spatial_rejected"] == 1
    assert summary["rejection_counts"] == {"calibration_out_of_domain": 1}
    assert summary["observed_range_min_m"] == 3.9837
    assert summary["depth_quality_min"] == 1.0


def test_empty_heartbeat_is_distinct_from_rejected_2d_detection():
    summary = summarize_spatial_fusion(
        [], calibration_range_min_m=1.0, calibration_range_max_m=3.0
    )
    assert summary["detections_2d"] == 0
    assert summary["spatial_rejected"] == 0
    assert format_no_targets_diagnostic(summary) == " (perception: detections_2d=0)"


def test_terminal_detail_explains_calibration_domain_rejection():
    summary = summarize_spatial_fusion(
        [
            {
                "has_spatial": False,
                "estimated_distance_m": 3.9837,
                "depth_quality": 1.0,
                "spatial_rejection_reason": "calibration_out_of_domain",
            }
        ],
        calibration_range_min_m=1.0215,
        calibration_range_max_m=2.9799,
    )
    detail = format_no_targets_diagnostic(summary)
    assert "primary_rejection=calibration_out_of_domain" in detail
    assert "observed_range_m=3.984..3.984" in detail
    assert "calibrated_range_m=1.022..2.980" in detail


def test_terminal_detail_does_not_invent_rejection_when_all_are_spatial():
    detail = format_no_targets_diagnostic({
        "detections_2d": 2,
        "spatial_accepted": 2,
        "spatial_rejected": 0,
        "rejection_counts": {},
    })
    assert "spatial=2, rejected=0" in detail
    assert "primary_rejection" not in detail
