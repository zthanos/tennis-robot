"""Sim ground-truth collection scoring: which onboard zone holds a ball.

Pure geometry (no rclpy) so the zone gates are unit-testable. Coordinates are
robot-local (base_footprint): +x forward, +y left, z above court.

Two zones (collection-route-debug-log-el.md #3):
  * "bin"  — the basket interior per docs/basket-bin-redesign-spec-el.md §
    (x 0.02…0.42, |y| ≤ 0.14, ball centre ≥ 0.055).
  * "deck" — anywhere else on the robot body: off-centre launches park balls
    on the chassis deck beside/behind the bin (observed at ly 0.2…0.35 and
    lx down to −0.22, resting z 0.058). The ball is off the court either way,
    so the mission must credit the capture instead of chasing a phantom.
"""

from __future__ import annotations

# Basket interior (matches the bin v2 spec and cad/basket-bin-v2/params.scad).
BIN_ZONE_X_M = (0.02, 0.42)
BIN_HALF_WIDTH_M = 0.14
BIN_MIN_BALL_Z_M = 0.055

# Robot body envelope for deck-parked balls. A ground ball rests at z=0.033,
# anything ≥ 0.050 sits ON the robot (deck top ~0.025 → ball centre 0.058).
DECK_ZONE_X_M = (-0.30, 0.45)
DECK_HALF_WIDTH_M = 0.35
DECK_MIN_BALL_Z_M = 0.050


def onboard_ball_zone(local_x_m: float, local_y_m: float, ball_z_m: float) -> str | None:
    """Classify a robot-local ball position: "bin", "deck", or None (not onboard)."""
    if (
        BIN_ZONE_X_M[0] <= local_x_m <= BIN_ZONE_X_M[1]
        and abs(local_y_m) <= BIN_HALF_WIDTH_M
        and ball_z_m >= BIN_MIN_BALL_Z_M
    ):
        return "bin"
    if (
        DECK_ZONE_X_M[0] <= local_x_m <= DECK_ZONE_X_M[1]
        and abs(local_y_m) <= DECK_HALF_WIDTH_M
        and ball_z_m >= DECK_MIN_BALL_Z_M
    ):
        return "deck"
    return None
