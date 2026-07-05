#!/usr/bin/env python3
"""Exercise the V2 coarse-to-fine search behavior without Webots."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from tennis_robot.collector import BallObservationInput  # noqa: E402
from tennis_robot.search import HalfCourtSearchBehavior, SearchConfig, SearchState  # noqa: E402


def main() -> None:
    cfg = SearchConfig(
        side="left",
        court_half_length_m=4.0,
        court_half_width_m=2.0,
        wall_clearance_m=0.5,
        net_clearance_m=0.4,
        zone_cols=2,
        zone_rows=2,
        survey_viewpoint_dwell_s=0.1,
        lane_width_m=1.0,
        waypoint_tolerance_m=0.15,
        detection_confidence_threshold=0.5,
        target_hold_s=0.05,
    )
    behavior = HalfCourtSearchBehavior(cfg)

    # ---- Starts in survey ----
    cmd = behavior.update(-3.5, 0.0, 0.0, BallObservationInput(False), front_range_m=2.0, dt_s=0.032)
    assert cmd.state == SearchState.SURVEY_VIEWPOINT, f"expected SURVEY_VIEWPOINT, got {cmd.state}"
    assert cmd.coverage_pct == 0.0

    # ---- Ball interrupt during survey ----
    detected = BallObservationInput(
        True, bearing_rad=0.1, distance_m=1.5, confidence=0.9,
        source="oak_depth", world_x_m=-2.5, world_y_m=-0.5,
    )
    cmd = behavior.update(-3.5, 0.0, 0.0, detected, front_range_m=2.0, dt_s=0.032, target_id=7)
    assert cmd.state == SearchState.BALL_DETECTED, f"expected BALL_DETECTED, got {cmd.state}"
    assert cmd.target_status == "detected"
    assert cmd.path_status == "pending_validation"
    assert cmd.resume_marker.startswith("survey_viewpoint:")

    # ---- Hold then resume to survey ----
    cmd = behavior.update(-3.5, 0.0, 0.0, BallObservationInput(False), front_range_m=2.0, dt_s=0.10)
    assert cmd.state == SearchState.SURVEY_VIEWPOINT, f"expected resume to SURVEY_VIEWPOINT, got {cmd.state}"
    assert cmd.target_status == "queued"

    # ---- Zone heatmap accumulates observations ----
    behavior.reset()
    obs_with_world = BallObservationInput(
        True, bearing_rad=0.0, distance_m=1.0, confidence=0.8,
        source="oak_depth", world_x_m=-2.0, world_y_m=0.5,
    )
    behavior.update(-3.5, 0.0, 0.0, obs_with_world, front_range_m=2.0, dt_s=0.032)
    snap = behavior.snapshot(-3.5, 0.0)
    total_estimated = sum(z["estimated_count"] for z in snap["zone_heatmap"])
    assert total_estimated > 0.0, "zone heatmap should have accumulated the observation"

    # ---- Coverage increases as survey advances ----
    behavior.reset()
    # Advance dwell time at first viewpoint
    vp = behavior._survey_viewpoints[0]
    for _ in range(5):
        behavior.update(vp[0], vp[1], 0.0, BallObservationInput(False), front_range_m=2.0, dt_s=0.05)
    cmd = behavior.update(vp[0], vp[1], 0.0, BallObservationInput(False), front_range_m=2.0, dt_s=0.05)
    assert cmd.coverage_pct > 0.0, "coverage should increase after first viewpoint"

    print("search behavior smoke ok: survey -> interrupt -> resume -> heatmap -> coverage")


if __name__ == "__main__":
    main()
