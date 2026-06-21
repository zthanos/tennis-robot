"""Shared survey sensor types for the ROS 2 survey pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SurveyVision:
    """Camera and line-detection data passed to the survey FSM each tick."""

    line_detected: bool = False
    line_offset_m: float | None = None
    line_heading_error_rad: float | None = None
    line_confidence: float = 0.0
    obstacle_class: str | None = None
    center_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None
    valid_count: int = 0
    corner_detected: bool = False
    corner_confidence: float = 0.0
    # Classified court-line junction from the camera (L=baseline corner,
    # T=service-line junction, +=centre). Enables along-court localisation
    # when the net is beyond camera depth range.
    junction_type: str | None = None
    junction_distance_m: float | None = None
    junction_bearing_rad: float | None = None
    junction_confidence: float = 0.0
