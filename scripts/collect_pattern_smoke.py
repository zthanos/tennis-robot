#!/usr/bin/env python3
"""Exercise the documented search-to-collect pattern without Webots."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from collector import BallObservationInput, CollectorState, ConceptACollectorBehavior  # noqa: E402
from search import HalfCourtSearchBehavior, SearchConfig, SearchState  # noqa: E402


def main() -> None:
    search = HalfCourtSearchBehavior(
        SearchConfig(
            side="left",
            court_half_length_m=4.0,
            court_half_width_m=2.0,
            wall_clearance_m=0.5,
            net_clearance_m=0.4,
            lane_width_m=1.0,
            waypoint_tolerance_m=0.15,
            detection_confidence_threshold=0.5,
            target_hold_s=0.05,
        )
    )
    collector = ConceptACollectorBehavior()

    detected = BallObservationInput(
        True,
        bearing_rad=0.18,
        distance_m=1.3,
        confidence=0.9,
        source="oak_depth",
        world_x_m=-2.8,
        world_y_m=-1.1,
    )
    search_command = search.update(-3.5, -1.5, 0.0, detected, front_range_m=2.0, dt_s=0.032, target_id=3)
    assert search_command.state == SearchState.BALL_DETECTED
    assert search_command.resume_marker.startswith("survey_viewpoint:")

    collector.start_tracking(detected)
    command = collector.update(detected, 0.032)
    assert command.state == CollectorState.ALIGN

    command = collector.update(
        BallObservationInput(True, bearing_rad=0.01, distance_m=1.0, confidence=0.9, source="oak_depth"),
        0.032,
    )
    assert command.state == CollectorState.APPROACH
    assert command.collector.intake_enabled

    command = collector.update(
        BallObservationInput(True, bearing_rad=0.0, distance_m=0.24, confidence=0.9, source="oak_depth"),
        0.032,
    )
    assert command.state == CollectorState.CAPTURE

    command = collector.update(
        BallObservationInput(True, bearing_rad=0.0, distance_m=0.18, confidence=0.9, source="oak_depth"),
        0.032,
        collection_confirmed=True,
    )
    assert command.state == CollectorState.COLLECTED

    search_command = search.update(-3.5, -1.5, 0.0, BallObservationInput(False), front_range_m=2.0, dt_s=0.10)
    assert search_command.state == SearchState.SURVEY_VIEWPOINT
    assert search_command.resume_marker.startswith("survey_viewpoint:")
    print("collect pattern smoke ok: search interrupt -> collector capture -> resume marker preserved")


if __name__ == "__main__":
    main()
