#!/usr/bin/env python3
"""Exercise the Concept A collector state machine without Webots."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from collector import BallObservationInput, CollectorState, ConceptACollectorBehavior  # noqa: E402


def main() -> None:
    behavior = ConceptACollectorBehavior()

    command = behavior.update(BallObservationInput(visible=False), dt_s=0.032)
    assert command.state == CollectorState.SCAN
    assert command.base.angular_speed_rad_s > 0

    command = behavior.update(BallObservationInput(True, bearing_rad=0.20, distance_m=1.4, confidence=0.9), 0.032)
    assert command.state == CollectorState.ALIGN
    assert command.base.angular_speed_rad_s > 0

    command = behavior.update(BallObservationInput(visible=False), 0.032)
    assert command.state == CollectorState.ALIGN
    assert command.base.angular_speed_rad_s > 0

    command = behavior.update(BallObservationInput(True, bearing_rad=0.01, distance_m=1.2, confidence=0.9), 0.032)
    assert command.state == CollectorState.APPROACH
    assert command.collector.intake_enabled

    command = behavior.update(BallObservationInput(True, bearing_rad=0.0, distance_m=0.25, confidence=0.9), 0.032)
    assert command.state == CollectorState.CAPTURE
    assert command.base.linear_speed_m_s > 0

    command = behavior.update(
        BallObservationInput(True, bearing_rad=0.0, distance_m=0.20, confidence=0.9),
        0.032,
        collection_confirmed=True,
    )
    assert command.state == CollectorState.COLLECTED
    assert not command.collector.intake_enabled

    print("collector behavior smoke ok: scan -> align -> approach -> capture -> collected")


if __name__ == "__main__":
    main()
