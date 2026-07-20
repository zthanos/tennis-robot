import os
import sys
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))
from tennis_robot.perception_covariance_calibration import CalibrationArtifact, CalibrationError, RangeDepthQualityDiagonalModel

def artifact():
    return {"schema_version":"perception-covariance-calibration/v1","calibration_id":"synthetic","model_id":"range_depth_quality_diagonal_v1","model_version":"v1","platform":"test","calibrated_at":"2026-01-01","parameters":{"axis_variance_floor_m2":[.01,.02,.03],"axis_range_variance_per_m2":[.01,.01,.01],"axis_quality_variance_m2":[.1,.1,.1]},"range_validity_domain_m":{"min":.1,"max":10.},"depth_quality_validity_domain":{"min":.2,"max":1.},"evidence_reference":"synthetic-test-only","acceptance_metrics":{"rmse_m":.1}}

def test_calibrated_model_is_finite_symmetric_psd_and_degrades_monotonically():
    model = RangeDepthQualityDiagonalModel(CalibrationArtifact.from_dict(artifact()))
    good = model.evaluate(1., 1.).covariance
    far = model.evaluate(2., 1.).covariance
    poor = model.evaluate(1., .5).covariance
    assert good and far and poor
    assert all(good[i] >= 0 for i in (0,4,8))
    assert far[0] > good[0] and poor[0] > good[0]
    assert all(good[i] == 0 for i in (1,2,3,5,6,7))

def test_invalid_artifact_and_out_of_domain_are_rejected():
    broken = artifact(); broken["parameters"]["axis_variance_floor_m2"] = [-1,0,0]
    with pytest.raises(CalibrationError): CalibrationArtifact.from_dict(broken)
    model = RangeDepthQualityDiagonalModel(CalibrationArtifact.from_dict(artifact()))
    assert model.evaluate(11., 1.).reason == "calibration_out_of_domain"
