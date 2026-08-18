"""Passive camera evidence for singles/doubles court presentation.

The survey/navigation geometry deliberately remains doubles-width.  This
module only estimates which outer baseline corners the camera observed, so the
operator UI can present a singles or doubles overlay with explicit confidence.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraCornerEvidence:
    map_x_m: float
    map_y_m: float
    confidence: float


def camera_corner_to_map(
    *,
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
    bearing_rad: float,
    distance_m: float,
    camera_x_m: float = 0.535,
) -> CameraCornerEvidence | None:
    values = (
        robot_x_m, robot_y_m, robot_yaw_rad, bearing_rad, distance_m, camera_x_m,
    )
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
        return None
    if distance_m <= 0.0:
        return None
    forward_m = camera_x_m + distance_m * math.cos(bearing_rad)
    right_m = distance_m * math.sin(bearing_rad)
    cy = math.cos(robot_yaw_rad)
    sy = math.sin(robot_yaw_rad)
    return CameraCornerEvidence(
        map_x_m=robot_x_m + forward_m * cy + right_m * sy,
        map_y_m=robot_y_m + forward_m * sy - right_m * cy,
        confidence=1.0,
    )


def add_distinct_corner(
    evidence: list[CameraCornerEvidence],
    candidate: CameraCornerEvidence,
    *,
    min_separation_m: float = 0.35,
    max_items: int = 32,
) -> None:
    for index, current in enumerate(evidence):
        if math.hypot(
            current.map_x_m - candidate.map_x_m,
            current.map_y_m - candidate.map_y_m,
        ) < min_separation_m:
            if candidate.confidence > current.confidence:
                evidence[index] = candidate
            return
    evidence.append(candidate)
    if len(evidence) > max_items:
        del evidence[:-max_items]


def estimate_court_format(
    evidence: list[CameraCornerEvidence],
    *,
    net_center_x_m: float,
    net_center_y_m: float,
    length_axis_x: float,
    length_axis_y: float,
    width_axis_x: float,
    width_axis_y: float,
    court_half_length_m: float = 11.885,
    singles_half_width_m: float = 4.115,
    doubles_half_width_m: float = 5.485,
    min_evidence: int = 2,
) -> dict:
    """Classify camera-observed baseline L-corners, or return ``unknown``.

    Only observations near a regulation baseline and near one of the expected
    sideline widths vote. Ambiguous or sparse evidence remains explicit.
    """
    singles_score = 0.0
    doubles_score = 0.0
    accepted = 0
    baseline_tolerance_m = 1.5
    width_tolerance_m = 0.9
    for item in evidence:
        dx = item.map_x_m - net_center_x_m
        dy = item.map_y_m - net_center_y_m
        court_x = dx * length_axis_x + dy * length_axis_y
        court_y = dx * width_axis_x + dy * width_axis_y
        baseline_error = abs(abs(court_x) - court_half_length_m)
        if baseline_error > baseline_tolerance_m:
            continue
        width = abs(court_y)
        singles_error = abs(width - singles_half_width_m)
        doubles_error = abs(width - doubles_half_width_m)
        best_error = min(singles_error, doubles_error)
        if best_error > width_tolerance_m:
            continue
        positional_weight = (
            max(0.0, 1.0 - baseline_error / baseline_tolerance_m)
            * max(0.0, 1.0 - best_error / width_tolerance_m)
        )
        weight = max(0.0, min(1.0, item.confidence)) * positional_weight
        if singles_error < doubles_error:
            singles_score += weight
        else:
            doubles_score += weight
        accepted += 1

    total = singles_score + doubles_score
    label = "unknown"
    confidence = 0.0
    if accepted >= min_evidence and total > 0.0:
        winner = max(singles_score, doubles_score)
        margin = abs(singles_score - doubles_score)
        support = min(1.0, accepted / 4.0)
        confidence = (winner / total) * support
        if margin >= 0.2 and confidence >= 0.55:
            label = "singles" if singles_score > doubles_score else "doubles"

    return {
        "label": label,
        "confidence": round(confidence, 3),
        "source": "camera_line_junctions",
        "evidence_count": accepted,
        "scores": {
            "singles": round(singles_score, 3),
            "doubles": round(doubles_score, 3),
        },
        "affects_navigation": False,
    }
