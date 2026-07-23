#!/usr/bin/env python3
"""Build a C2 Gazebo artifact and evidence report from recorder JSONL output."""

from __future__ import annotations

import argparse
import json
from datetime import date
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.perception_covariance_calibration import (  # noqa: E402
    compute_artifact_sha256,
)
from tennis_robot.perception_covariance_evidence import (  # noqa: E402
    CalibrationSample,
    assert_conservative,
    assert_conservative_by_bin,
    assert_c2_coverage,
    fit_conservative_artifact,
)


def _artifact_json(artifact, acceptance_metrics: dict) -> dict:
    payload = {
        "schema_version": "perception-covariance-calibration/v1",
        "calibration_id": artifact.calibration_id,
        "model_id": artifact.model_id,
        "model_version": artifact.model_version,
        "platform": artifact.platform,
        "calibrated_at": artifact.calibrated_at,
        "parameters": {
            "axis_variance_floor_m2": list(artifact.floor),
            "axis_range_variance_per_m2": list(artifact.range_coeff),
            "axis_quality_variance_m2": list(artifact.quality_coeff),
        },
        "range_validity_domain_m": {"min": artifact.range_min_m, "max": artifact.range_max_m},
        "depth_quality_validity_domain": {"min": artifact.quality_min, "max": artifact.quality_max},
        "evidence_reference": artifact.evidence_reference,
        "acceptance_metrics": acceptance_metrics,
    }
    payload["artifact_sha256"] = compute_artifact_sha256(payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--model-version", default="gazebo-v1")
    parser.add_argument("--evidence-reference", required=True)
    parser.add_argument("--trial-manifest", required=True, type=Path)
    parser.add_argument("--summary-dir", required=True, type=Path)
    args = parser.parse_args()

    rows = []
    samples_by_bin = {}
    for evidence_path in sorted(args.evidence_dir.glob("*.jsonl")):
        if evidence_path.name.endswith((".non_target.jsonl", ".rejections.jsonl")):
            continue
        for line in evidence_path.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            rows.append(row)
            samples_by_bin.setdefault(row["trial_id"], []).append(
                CalibrationSample(
                    row["range_m"], row["depth_quality"], tuple(row["error_xyz_m"])
                )
            )
    samples = [sample for bin_samples in samples_by_bin.values() for sample in bin_samples]
    manifest = json.loads(args.trial_manifest.read_text(encoding="utf-8"))
    summaries = {}
    for path in args.summary_dir.glob("*.summary.json"):
        summary = json.loads(path.read_text(encoding="utf-8"))
        summaries[summary.get("trial_id", "")] = summary
    # This is deliberately before fitting/writing: incomplete evidence must
    # never result in a syntactically valid artifact.
    assert_c2_coverage(rows, manifest=manifest, summaries=summaries)
    artifact = fit_conservative_artifact(
        samples,
        calibration_id=args.calibration_id,
        model_version=args.model_version,
        platform="gazebo",
        calibrated_at=str(date.today()),
        evidence_reference=args.evidence_reference,
    )
    assert_conservative(artifact, samples)
    per_bin_metrics = assert_conservative_by_bin(artifact, samples_by_bin)
    acceptance_metrics = {
        **artifact.acceptance_metrics,
        "coverage_thresholds": {
            "minimum_accepted_samples_per_bin": manifest["minimum_accepted_samples_per_bin"],
            "maximum_target_outlier_rate": manifest["maximum_outlier_rate"],
            "target_association_gate_m": manifest["association_gate_m"],
            "target_association_ambiguity_margin_m": manifest["association_ambiguity_margin_m"],
            "target_residual_outlier_threshold_m": manifest["target_residual_outlier_threshold_m"],
        },
        "trial_manifest_reference": str(args.trial_manifest),
        "coverage_evidence_report_reference": args.evidence_reference,
        "associated_target_evidence_directory": str(args.evidence_dir),
        "range_quality_bin_metrics": per_bin_metrics,
        "trial_metrics": {
            trial_id: summaries[trial_id]["metrics"]
            for trial_id in sorted(samples_by_bin)
        },
        "conservative_check": "passed_per_axis_for_every_accepted_target_sample_in_every_range_quality_bin",
    }
    payload = _artifact_json(artifact, acceptance_metrics)
    args.artifact.parent.mkdir(parents=True, exist_ok=True)
    args.artifact.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    metrics = artifact.acceptance_metrics
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        "# C2 Gazebo covariance evidence report\n\n"
        f"- Evidence directory: `{args.evidence_dir}` (only associated target JSONL rows)\n"
        f"- Accepted samples: {metrics['accepted_sample_count']}\n"
        f"- Range domain (m): [{artifact.range_min_m}, {artifact.range_max_m}]\n"
        f"- Depth-quality domain: [{artifact.quality_min}, {artifact.quality_max}]\n"
        f"- Max axis error (m): {metrics['max_axis_error_m']}\n"
        f"- RMSE axis (m): {metrics['rmse_axis_m']}\n"
        "- Conservation: PASS for every axis in every declared range×quality bin.\n"
        f"- Per-bin metrics: `{json.dumps(per_bin_metrics, sort_keys=True)}`\n"
        "- Producer activation: not performed by this command.\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
