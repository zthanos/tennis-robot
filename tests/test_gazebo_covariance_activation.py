"""C2 activation contract for the approved Gazebo artifact only."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.perception_covariance_calibration import (  # noqa: E402
    RangeDepthQualityDiagonalModel,
    evaluate_producer_spatial_covariance,
    load_artifact,
    load_spatial_calibration_runtime,
)


V1_ARTIFACT = ROOT / "calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v1.json"
V2_ARTIFACT = ROOT / "calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json"
MANIFEST = ROOT / "config/gazebo_covariance_c2_trials.json"
EVIDENCE_DIR = ROOT / "runtime/c2_controlled_coverage"
CALIBRATION_ID = "gazebo-range-depth-quality-diagonal-v1-20260719-v2"


def _gazebo_runtime():
    return load_spatial_calibration_runtime(
        V2_ARTIFACT,
        expected_platform="gazebo",
        expected_calibration_id=CALIBRATION_ID,
        expected_model_version="gazebo-v2",
        required_path=V2_ARTIFACT,
    )


def test_approved_gazebo_artifact_loads_healthy_with_runtime_identity():
    runtime = _gazebo_runtime()
    assert runtime.healthy is True
    assert runtime.health_reason == "healthy"
    assert runtime.artifact_id == CALIBRATION_ID
    assert runtime.artifact_version == "gazebo-v2"
    assert runtime.model is not None


def test_v1_remains_rejected_for_native_q100_evidence():
    model = RangeDepthQualityDiagonalModel(load_artifact(V1_ARTIFACT))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for trial in manifest["trials"]:
        row = json.loads((EVIDENCE_DIR / f"{trial['id']}.jsonl").read_text(encoding="utf-8").splitlines()[0])
        result = model.evaluate(row["range_m"], row["depth_quality"])
        if trial["id"].endswith("_q100"):
            assert result.reason == "calibration_out_of_domain"
            continue
        assert result.reason is None, trial["id"]


def test_v2_candidate_covers_each_declared_range_quality_bin_without_runtime_load():
    model = RangeDepthQualityDiagonalModel(load_artifact(V2_ARTIFACT))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for trial in manifest["trials"]:
        row = json.loads((EVIDENCE_DIR / f"{trial['id']}.jsonl").read_text(encoding="utf-8").splitlines()[0])
        result = model.evaluate(row["range_m"], row["depth_quality"])
        assert result.reason is None, trial["id"]
        assert result.covariance is not None
        assert result.covariance[0] >= 0.0
        assert result.covariance[4] >= 0.0
        assert result.covariance[8] >= 0.0


@pytest.mark.parametrize("quality", [0.90, 0.98])
def test_producer_model_adapter_returns_v2_covariance_for_supported_non_native_quality(quality):
    model = RangeDepthQualityDiagonalModel(load_artifact(V2_ARTIFACT))
    expected = model.evaluate(1.5, quality)
    actual = evaluate_producer_spatial_covariance(model, (0.0, 0.0, 1.5), quality)
    assert actual.reason is None
    assert actual.covariance == expected.covariance


def test_producer_model_adapter_rejects_out_of_domain_range_or_quality():
    model = RangeDepthQualityDiagonalModel(load_artifact(V2_ARTIFACT))
    assert evaluate_producer_spatial_covariance(model, (0.0, 0.0, 3.1), 1.0).reason == "calibration_out_of_domain"
    assert evaluate_producer_spatial_covariance(model, (0.0, 0.0, 1.5), 0.80).reason == "calibration_out_of_domain"


def test_out_of_domain_measurement_has_no_covariance():
    runtime = _gazebo_runtime()
    result = runtime.model.evaluate(3.1, 0.95)
    assert result.covariance is None
    assert result.reason == "calibration_out_of_domain"


def test_invalid_artifact_does_not_make_spatial_pipeline_healthy(tmp_path):
    invalid = tmp_path / "invalid.json"
    invalid.write_text("{not-json", encoding="utf-8")
    runtime = load_spatial_calibration_runtime(invalid, expected_platform="gazebo")
    assert runtime.healthy is False
    assert runtime.health_reason == "calibration_invalid"
    assert runtime.model is None


def test_physical_oak_d_platform_rejects_the_gazebo_artifact():
    runtime = load_spatial_calibration_runtime(V2_ARTIFACT, expected_platform="oak_d")
    assert runtime.healthy is False
    assert runtime.health_reason == "calibration_platform_mismatch"
    assert runtime.model is None
