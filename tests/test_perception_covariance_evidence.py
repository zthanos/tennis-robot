import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.perception_covariance_calibration import CalibrationError
from tennis_robot.perception_covariance_evidence import (
    CalibrationSample,
    assert_conservative,
    assert_conservative_by_bin,
    assert_c2_coverage,
    fit_conservative_artifact,
)
from tennis_robot.perception import BallDetection, depth_roi_quality
from tennis_robot.gazebo_covariance_trials import apply_calibration_depth_mask
from tennis_robot.gazebo_covariance_association import associate_target_candidate, measured_range_in_bin


def _samples():
    return [
        CalibrationSample(0.8, 1.0, (0.01, -0.02, 0.03)),
        CalibrationSample(1.2, 0.95, (0.02, -0.03, 0.04)),
        CalibrationSample(2.0, 0.85, (0.05, -0.06, 0.08)),
        CalibrationSample(2.8, 0.90, (0.07, -0.05, 0.10)),
    ]


def test_measured_evidence_fit_is_conservative_for_every_sample():
    artifact = fit_conservative_artifact(
        _samples(),
        calibration_id="gazebo-test",
        model_version="v1",
        platform="gazebo",
        calibrated_at="2026-07-18",
        evidence_reference="test-only",
    )
    assert artifact.platform == "gazebo"
    assert artifact.acceptance_metrics["accepted_sample_count"] == 4
    assert_conservative(artifact, _samples())


def test_conservation_check_rejects_underestimated_artifact():
    artifact = fit_conservative_artifact(
        _samples(),
        calibration_id="gazebo-test",
        model_version="v1",
        platform="gazebo",
        calibrated_at="2026-07-18",
        evidence_reference="test-only",
    )
    object.__setattr__(artifact, "floor", (0.0, 0.0, 0.0))
    object.__setattr__(artifact, "range_coeff", (0.0, 0.0, 0.0))
    object.__setattr__(artifact, "quality_coeff", (0.0, 0.0, 0.0))
    with pytest.raises(CalibrationError, match="non-conservative"):
        assert_conservative(artifact, _samples())


def test_conservation_check_is_reported_for_each_range_quality_bin():
    artifact = fit_conservative_artifact(
        _samples(),
        calibration_id="gazebo-test",
        model_version="v1",
        platform="gazebo",
        calibrated_at="2026-07-19",
        evidence_reference="test-only",
    )
    metrics = assert_conservative_by_bin(
        artifact, {"r0_q90": _samples()[:2], "r1_q98": _samples()[2:]}
    )
    assert metrics["r0_q90"]["accepted_target_sample_count"] == 2
    assert metrics["r1_q98"]["conservative_per_axis"] is True


def test_depth_quality_is_the_fraction_of_valid_pixels_in_the_fusion_roi():
    import numpy as np

    detection = BallDetection(2, 2, 4, 4)
    depth = np.ones((10, 10), dtype=np.float32)
    depth[3:5, 3:5] = np.nan
    quality = depth_roi_quality(detection, depth, 10, 10)
    assert 0.0 < quality < 1.0


def test_calibration_only_mask_is_deterministic_and_uses_shared_quality_metric():
    import numpy as np

    detection = BallDetection(2, 2, 4, 4)
    depth = np.ones((10, 10), dtype=np.float32)
    one = apply_calibration_depth_mask(depth, detection, 10, 10, missing_pixel_ratio=0.10, seed=7)
    two = apply_calibration_depth_mask(depth, detection, 10, 10, missing_pixel_ratio=0.10, seed=7)
    assert np.array_equal(np.isnan(one), np.isnan(two))
    assert depth_roi_quality(detection, one, 10, 10) == pytest.approx(8 / 9)
    assert np.isfinite(depth).all()


def test_c2_coverage_gate_requires_every_trial_and_ground_truth_reference():
    manifest = {
        "schema_version": "gazebo-covariance-trials/v1",
        "ground_truth_reference": "/sim/balls:ball_02",
        "minimum_accepted_samples_per_bin": 2,
        "maximum_outlier_rate": 0.35,
        "trials": [
            {"id": "low", "range_bin": "0.75-1.25", "quality_bin": "0.85-0.95"},
            {"id": "high", "range_bin": "1.25-2.00", "quality_bin": "0.95-1.00"},
        ],
    }
    rows = [
        {"trial_id": "low", "range_m": 1.0, "depth_quality": 0.9, "ground_truth_reference": "/sim/balls:ball_02"},
        {"trial_id": "low", "range_m": 1.0, "depth_quality": 0.9, "ground_truth_reference": "/sim/balls:ball_02"},
    ]
    with pytest.raises(CalibrationError, match="high: accepted 0"):
        assert_c2_coverage(rows, manifest=manifest, summaries={"low": {"rejected": {}}})


def test_non_target_detection_does_not_count_as_target_outlier():
    ground_truth = {"ball_02": (0.0, 0.0, 1.0), "ball_09": (1.0, 0.0, 1.0)}
    result = associate_target_candidate((1.02, 0.0, 1.0), ground_truth, target_ball_id="ball_02", association_gate_m=0.2, ambiguity_margin_m=0.01)
    assert result.kind == "non_target" and result.ball_id == "ball_09"


def test_ambiguous_and_unmatched_candidates_are_rejected_from_target_accounting():
    ground_truth = {"ball_02": (0.0, 0.0, 1.0), "ball_09": (0.01, 0.0, 1.0)}
    ambiguous = associate_target_candidate((0.005, 0.0, 1.0), ground_truth, target_ball_id="ball_02", association_gate_m=0.2, ambiguity_margin_m=0.01)
    unmatched = associate_target_candidate((3.0, 0.0, 1.0), ground_truth, target_ball_id="ball_02", association_gate_m=0.2, ambiguity_margin_m=0.01)
    assert ambiguous.kind == "ambiguous"
    assert unmatched.kind == "unmatched"


def test_measured_range_not_nominal_intent_controls_bin_membership():
    assert measured_range_in_bin(1.6, "1.25-2.00")
    assert not measured_range_in_bin(2.095, "1.25-2.00")


def test_coverage_uses_only_associated_target_outlier_metric():
    manifest = {
        "schema_version": "gazebo-covariance-trials/v1",
        "ground_truth_reference": "/sim/balls:ball_02",
        "minimum_accepted_samples_per_bin": 1,
        "maximum_outlier_rate": 0.35,
        "trials": [{"id": "target", "range_bin": "1.25-2.00", "quality_bin": "0.95-1.00"}],
    }
    rows = [{"trial_id": "target", "range_m": 1.6, "depth_quality": 0.98, "ground_truth_reference": "/sim/balls:ball_02"}]
    assert_c2_coverage(rows, manifest=manifest, summaries={"target": {"metrics": {"target_outlier_rate": 0.0}, "rejected": {"ground_truth_match_gate_failed": 99}}})
