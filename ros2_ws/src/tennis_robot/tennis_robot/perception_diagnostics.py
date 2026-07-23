"""Pure helpers for reporting spatial-fusion outcomes without changing detections."""

from __future__ import annotations

from collections import Counter
import math


def summarize_spatial_fusion(
    records: list[dict],
    *,
    calibration_range_min_m: float | None,
    calibration_range_max_m: float | None,
) -> dict:
    """Summarize one RGB/depth frame for operators and route diagnostics.

    This is deliberately separate from ``BallDetectionArray``: the latter stays
    the sole downstream perception contract, while this payload only explains
    why a 2D detection did or did not become a spatial target.
    """
    rejection_counts = Counter(
        str(record["spatial_rejection_reason"])
        for record in records
        if record.get("spatial_rejection_reason")
    )
    ranges = [
        float(record["estimated_distance_m"])
        for record in records
        if record.get("estimated_distance_m") is not None
        and math.isfinite(float(record["estimated_distance_m"]))
    ]
    qualities = [
        float(record["depth_quality"])
        for record in records
        if record.get("depth_quality") is not None
        and math.isfinite(float(record["depth_quality"]))
    ]
    return {
        "schema_version": 1,
        "detections_2d": len(records),
        "spatial_accepted": sum(bool(record.get("has_spatial")) for record in records),
        "spatial_rejected": sum(not bool(record.get("has_spatial")) for record in records),
        "rejection_counts": dict(sorted(rejection_counts.items())),
        "observed_range_min_m": min(ranges) if ranges else None,
        "observed_range_max_m": max(ranges) if ranges else None,
        "depth_quality_min": min(qualities) if qualities else None,
        "depth_quality_max": max(qualities) if qualities else None,
        "calibration_range_min_m": calibration_range_min_m,
        "calibration_range_max_m": calibration_range_max_m,
    }


def format_no_targets_diagnostic(diagnostic: dict | None) -> str:
    """Return a compact terminal-log suffix for an empty collection plan."""
    if not isinstance(diagnostic, dict):
        return ""
    detections = int(diagnostic.get("detections_2d", 0) or 0)
    accepted = int(diagnostic.get("spatial_accepted", 0) or 0)
    rejected = int(diagnostic.get("spatial_rejected", 0) or 0)
    counts = diagnostic.get("rejection_counts") or {}
    if detections <= 0:
        return " (perception: detections_2d=0)"
    detail = (
        f" (perception: detections_2d={detections}, spatial={accepted}, "
        f"rejected={rejected}"
    )
    if counts:
        detail += f", primary_rejection={max(counts, key=counts.get)}"
    observed_min = diagnostic.get("observed_range_min_m")
    observed_max = diagnostic.get("observed_range_max_m")
    calibrated_min = diagnostic.get("calibration_range_min_m")
    calibrated_max = diagnostic.get("calibration_range_max_m")
    if all(
        isinstance(value, (int, float)) and math.isfinite(float(value))
        for value in (observed_min, observed_max)
    ):
        detail += f", observed_range_m={float(observed_min):.3f}..{float(observed_max):.3f}"
    if all(
        isinstance(value, (int, float)) and math.isfinite(float(value))
        for value in (calibrated_min, calibrated_max)
    ):
        detail += f", calibrated_range_m={float(calibrated_min):.3f}..{float(calibrated_max):.3f}"
    return detail + ")"
