"""Pure C2 evidence fitting for the Gazebo covariance artifact.

This module accepts recorded residuals only.  It deliberately has no ROS,
Gazebo or detector dependency so the resulting artifact can be independently
rechecked from the recorded evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from tennis_robot.perception_covariance_calibration import (
    CalibrationArtifact,
    CalibrationError,
    RangeDepthQualityDiagonalModel,
)


@dataclass(frozen=True)
class CalibrationSample:
    range_m: float
    depth_quality: float
    error_xyz_m: tuple[float, float, float]

    def validate(self) -> None:
        values = (self.range_m, self.depth_quality, *self.error_xyz_m)
        if not all(math.isfinite(value) for value in values):
            raise CalibrationError("non-finite calibration sample")
        if self.range_m <= 0.0 or not 0.0 <= self.depth_quality <= 1.0:
            raise CalibrationError("invalid calibration sample domain")


def _upward_micro_m2(value: float) -> float:
    """Round up, never down, so serialization cannot lose conservatism."""
    return math.ceil(max(0.0, value) * 1_000_000.0) / 1_000_000.0


def _nonnegative_slope(feature: list[float], values: list[float]) -> float:
    mean_feature = sum(feature) / len(feature)
    mean_value = sum(values) / len(values)
    variance = sum((x - mean_feature) ** 2 for x in feature)
    if variance <= 0.0:
        return 0.0
    covariance = sum((x - mean_feature) * (y - mean_value) for x, y in zip(feature, values))
    return max(0.0, covariance / variance)


def fit_conservative_artifact(
    samples: Iterable[CalibrationSample],
    *,
    calibration_id: str,
    model_version: str,
    platform: str,
    calibrated_at: str,
    evidence_reference: str,
) -> CalibrationArtifact:
    """Fit non-negative coefficients, then lift floors to cover every sample.

    Coefficients are estimated from measured squared residuals.  The floor is
    selected from the largest remaining residual, which makes the returned
    model conservative for every input sample rather than merely a regression
    estimate.
    """
    accepted = tuple(samples)
    if not accepted:
        raise CalibrationError("calibration evidence has no accepted samples")
    for sample in accepted:
        sample.validate()

    r2 = [sample.range_m ** 2 for sample in accepted]
    qloss2 = [(1.0 - sample.depth_quality) ** 2 for sample in accepted]
    squared = [tuple(error * error for error in sample.error_xyz_m) for sample in accepted]
    range_coeff: list[float] = []
    quality_coeff: list[float] = []
    floors: list[float] = []
    for axis in range(3):
        values = [row[axis] for row in squared]
        range_slope = _nonnegative_slope(r2, values)
        residuals = [value - range_slope * r for value, r in zip(values, r2)]
        quality_slope = _nonnegative_slope(qloss2, residuals)
        floor = max(
            value - range_slope * r - quality_slope * q
            for value, r, q in zip(values, r2, qloss2)
        )
        range_coeff.append(_upward_micro_m2(range_slope))
        quality_coeff.append(_upward_micro_m2(quality_slope))
        floors.append(_upward_micro_m2(floor))

    max_axis_error_m = [max(abs(sample.error_xyz_m[axis]) for sample in accepted) for axis in range(3)]
    rmse_axis_m = [
        math.sqrt(sum(row[axis] for row in squared) / len(squared)) for axis in range(3)
    ]
    artifact = CalibrationArtifact.from_dict(
        {
            "schema_version": "perception-covariance-calibration/v1",
            "calibration_id": calibration_id,
            "model_id": "range_depth_quality_diagonal_v1",
            "model_version": model_version,
            "platform": platform,
            "calibrated_at": calibrated_at,
            "parameters": {
                "axis_variance_floor_m2": floors,
                "axis_range_variance_per_m2": range_coeff,
                "axis_quality_variance_m2": quality_coeff,
            },
            "range_validity_domain_m": {
                "min": min(sample.range_m for sample in accepted),
                "max": max(sample.range_m for sample in accepted),
            },
            "depth_quality_validity_domain": {
                "min": min(sample.depth_quality for sample in accepted),
                "max": max(sample.depth_quality for sample in accepted),
            },
            "evidence_reference": evidence_reference,
            "acceptance_metrics": {
                "accepted_sample_count": len(accepted),
                "max_axis_error_m": max_axis_error_m,
                "rmse_axis_m": rmse_axis_m,
            },
        }
    )
    assert_conservative(artifact, accepted)
    return artifact


def assert_conservative(
    artifact: CalibrationArtifact, samples: Iterable[CalibrationSample]
) -> None:
    """Raise when a declared artifact underestimates any measured axis error."""
    model = RangeDepthQualityDiagonalModel(artifact)
    for index, sample in enumerate(samples):
        sample.validate()
        result = model.evaluate(sample.range_m, sample.depth_quality)
        if result.covariance is None:
            raise CalibrationError(f"sample {index} outside artifact domain")
        for axis, covariance_index in enumerate((0, 4, 8)):
            if result.covariance[covariance_index] + 1e-15 < sample.error_xyz_m[axis] ** 2:
                raise CalibrationError(f"artifact is non-conservative for sample {index} axis {axis}")


def assert_conservative_by_bin(
    artifact: CalibrationArtifact, samples_by_bin: dict[str, Iterable[CalibrationSample]]
) -> dict[str, dict[str, object]]:
    """Check conservation independently for every declared C2 range×quality bin."""
    metrics: dict[str, dict[str, object]] = {}
    for bin_id, bin_samples_iter in sorted(samples_by_bin.items()):
        bin_samples = tuple(bin_samples_iter)
        if not bin_samples:
            raise CalibrationError(f"{bin_id}: no accepted calibration samples")
        assert_conservative(artifact, bin_samples)
        metrics[bin_id] = {
            "accepted_target_sample_count": len(bin_samples),
            "max_axis_error_m": [
                max(abs(sample.error_xyz_m[axis]) for sample in bin_samples)
                for axis in range(3)
            ],
            "conservative_per_axis": True,
        }
    return metrics


def assert_c2_coverage(
    rows: Iterable[dict], *, manifest: dict, summaries: dict[str, dict]
) -> None:
    """Enforce the C2 range×quality, outlier and GT-reference gate."""
    if manifest.get("schema_version") != "gazebo-covariance-trials/v1":
        raise CalibrationError("invalid C2 trial manifest")
    minimum = manifest.get("minimum_accepted_samples_per_bin")
    maximum_outlier_rate = manifest.get("maximum_outlier_rate")
    reference = manifest.get("ground_truth_reference")
    if not isinstance(minimum, int) or minimum <= 0 or not isinstance(maximum_outlier_rate, (int, float)):
        raise CalibrationError("invalid C2 coverage requirements")
    grouped: dict[str, list[dict]] = {}
    for row in rows:
        grouped.setdefault(row.get("trial_id", ""), []).append(row)
    failures: list[str] = []
    for trial in manifest.get("trials", []):
        trial_id = trial["id"]
        samples = grouped.get(trial_id, [])
        if len(samples) < minimum:
            failures.append(f"{trial_id}: accepted {len(samples)} < {minimum}")
            continue
        if any(sample.get("ground_truth_reference") != reference for sample in samples):
            failures.append(f"{trial_id}: required ground-truth reference missing")
        range_low, range_high = (float(value) for value in trial["range_bin"].split("-"))
        quality_low, quality_high = (float(value) for value in trial["quality_bin"].split("-"))
        if any(not range_low <= float(sample["range_m"]) <= range_high for sample in samples):
            failures.append(f"{trial_id}: range sample outside declared bin")
        if any(not quality_low <= float(sample["depth_quality"]) <= quality_high for sample in samples):
            failures.append(f"{trial_id}: quality sample outside declared bin")
        summary = summaries.get(trial_id)
        if not summary:
            failures.append(f"{trial_id}: summary missing")
            continue
        metrics = summary.get("metrics", {})
        rate = metrics.get("target_outlier_rate")
        if rate is None:
            failures.append(f"{trial_id}: target outlier metric missing")
            continue
        if rate > maximum_outlier_rate:
            failures.append(f"{trial_id}: outlier rate {rate:.3f} > {maximum_outlier_rate:.3f}")
    if failures:
        raise CalibrationError("C2 coverage gate failed: " + "; ".join(failures))
