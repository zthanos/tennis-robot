"""Deterministic, calibration-only depth degradation for C2 Gazebo trials."""

from __future__ import annotations

import math

import numpy as np

from tennis_robot.perception import BallDetection, depth_fusion_roi_bounds


def apply_calibration_depth_mask(
    depth_m: np.ndarray,
    detection: BallDetection,
    frame_width_px: int,
    frame_height_px: int,
    *,
    missing_pixel_ratio: float,
    seed: int,
) -> np.ndarray:
    """Mask a deterministic fraction of valid pixels in the fusion ROI.

    This is used only by the C2 recorder, never by a runtime perception
    producer. The returned depth image is independently allocated so raw
    Gazebo camera data remains unchanged.
    """
    if not math.isfinite(missing_pixel_ratio) or not 0.0 <= missing_pixel_ratio < 1.0:
        raise ValueError("missing_pixel_ratio must be in [0, 1)")
    masked = depth_m.copy()
    x0, x1, y0, y1 = depth_fusion_roi_bounds(
        detection, masked, frame_width_px, frame_height_px
    )
    roi = masked[y0:y1, x0:x1]
    valid_flat = np.flatnonzero(np.isfinite(roi) & (roi > 0.0))
    count = int(round(valid_flat.size * missing_pixel_ratio))
    if count:
        rng = np.random.default_rng(seed)
        roi.flat[rng.choice(valid_flat, size=count, replace=False)] = np.nan
    return masked
