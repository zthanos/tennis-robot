"""Sim ground-truth collection scoring and basket-retention confirmation.

Pure geometry (no rclpy) so the zone gates are unit-testable. Coordinates are
robot-local (base_footprint): +x forward, +y left, z above court.

Three zones (archive/collection-route-debug-log-el.md #3):
  * "bin"  — the secure basket interior, conservatively 20 mm behind the
    x=0.42 retention lip (x 0.02…0.40, |y| ≤ 0.14, centre z ≥ 0.055).
  * "receiver" — the short entry chute in front of the retention lip. A ball
    here has entered the mechanism but has not yet been retained.
  * "deck" — anywhere else on the robot body: off-centre launches park balls
    on the chassis deck beside/behind the bin (observed at ly 0.2…0.35 and
    lx down to −0.22, resting z 0.058). Neither receiver nor deck is collection
    success: only stable residence behind the lip, inside the bin, is retained.
"""

from __future__ import annotations

from dataclasses import dataclass

# Basket interior (matches the bin v2 spec and cad/basket-bin-v2/params.scad).
BIN_ZONE_X_M = (0.02, 0.40)
BIN_HALF_WIDTH_M = 0.14
BIN_MIN_BALL_Z_M = 0.055
BIN_RETENTION_EXIT_X_M = 0.42

# Entry transition and inclined receiver around/in front of the retention lip.
RECEIVER_ZONE_X_M = (BIN_ZONE_X_M[1], 0.48)
RECEIVER_HALF_WIDTH_M = 0.10
RECEIVER_MIN_BALL_Z_M = 0.050

# Robot body envelope for deck-parked balls. A ground ball rests at z=0.033,
# anything ≥ 0.050 sits ON the robot (deck top ~0.025 → ball centre 0.058).
DECK_ZONE_X_M = (-0.30, 0.45)
DECK_HALF_WIDTH_M = 0.35
DECK_MIN_BALL_Z_M = 0.050

DEFAULT_RETENTION_DWELL_S = 0.75


def onboard_ball_zone(local_x_m: float, local_y_m: float, ball_z_m: float) -> str | None:
    """Classify a robot-local ball position: bin, receiver, deck, or offboard."""
    if (
        BIN_ZONE_X_M[0] <= local_x_m <= BIN_ZONE_X_M[1]
        and abs(local_y_m) <= BIN_HALF_WIDTH_M
        and ball_z_m >= BIN_MIN_BALL_Z_M
    ):
        return "bin"
    if (
        RECEIVER_ZONE_X_M[0] < local_x_m <= RECEIVER_ZONE_X_M[1]
        and abs(local_y_m) <= RECEIVER_HALF_WIDTH_M
        and ball_z_m >= RECEIVER_MIN_BALL_Z_M
    ):
        return "receiver"
    if (
        DECK_ZONE_X_M[0] <= local_x_m <= DECK_ZONE_X_M[1]
        and abs(local_y_m) <= DECK_HALF_WIDTH_M
        and ball_z_m >= DECK_MIN_BALL_Z_M
    ):
        return "deck"
    return None


def retained_ball_still_in_bin(
    local_x_m: float, local_y_m: float, ball_z_m: float
) -> bool:
    """Post-credit bin gate with 20 mm x hysteresis to ignore small settling bounce."""
    return (
        BIN_ZONE_X_M[0] <= local_x_m <= BIN_RETENTION_EXIT_X_M
        and abs(local_y_m) <= BIN_HALF_WIDTH_M
        and ball_z_m >= BIN_MIN_BALL_Z_M
    )


@dataclass(frozen=True)
class RetentionUpdate:
    """One state transition emitted by :class:`SimRetentionTracker`."""

    event: str | None = None
    retained: bool = False
    dwell_s: float = 0.0
    previous_zone: str | None = None


class SimRetentionTracker:
    """Require continuous bin residence before declaring collection success."""

    def __init__(self, dwell_s: float = DEFAULT_RETENTION_DWELL_S) -> None:
        self.dwell_s = max(0.0, dwell_s)
        self._states: dict[str, tuple[str, float]] = {}

    def update(self, ball_def: str, zone: str | None, now_s: float) -> RetentionUpdate:
        previous = self._states.get(ball_def)
        previous_zone = previous[0] if previous is not None else None

        if zone == "bin":
            if previous_zone != "bin":
                self._states[ball_def] = ("bin", now_s)
                return RetentionUpdate(
                    event="basket_bin_candidate",
                    previous_zone=previous_zone,
                )
            dwell = max(0.0, now_s - previous[1])
            if dwell >= self.dwell_s:
                self._states.pop(ball_def, None)
                return RetentionUpdate(
                    event="basket_retained",
                    retained=True,
                    dwell_s=dwell,
                    previous_zone="bin",
                )
            return RetentionUpdate(dwell_s=dwell, previous_zone="bin")

        if zone == "receiver":
            if previous_zone == "receiver":
                return RetentionUpdate(previous_zone="receiver")
            self._states[ball_def] = ("receiver", now_s)
            return RetentionUpdate(
                event=(
                    "basket_bin_candidate_lost"
                    if previous_zone == "bin"
                    else "basket_entry_candidate"
                ),
                previous_zone=previous_zone,
            )

        self._states.pop(ball_def, None)
        if previous_zone == "bin":
            event = "basket_bin_candidate_lost"
        elif previous_zone == "receiver":
            event = "basket_entry_lost"
        else:
            event = None
        return RetentionUpdate(event=event, previous_zone=previous_zone)

    def retain_only(self, ball_defs: set[str]) -> None:
        """Drop state for Gazebo entities that no longer exist."""
        self._states = {
            ball_def: state
            for ball_def, state in self._states.items()
            if ball_def in ball_defs
        }


DEFAULT_RECONCILE_TOLERANCE_S = 5.0


class CreditReconciler:
    """Referee the beam-latch credits against ground-truth bin retention.

    Beam-primary sim runs confirm collection with the SAME basket IR latch
    hardware uses; ground truth no longer credits, it only counts retentions
    here. A beam break normally precedes the 0.75 s retention dwell by ~1 s,
    so transient count gaps are expected — only a mismatch that persists past
    ``tolerance_s`` is reported, once per delta value:

      * beam_count > truth_count → "beam_false_credit" (beam fired, no ball
        actually stayed in the bin).
      * beam_count < truth_count → "beam_missed_credit" (ball retained, beam
        never saw it — e.g. permanently blocked by a full basket).
    """

    def __init__(self, tolerance_s: float = DEFAULT_RECONCILE_TOLERANCE_S) -> None:
        self.tolerance_s = max(0.0, tolerance_s)
        self.beam_count = 0
        self.truth_count = 0
        self._mismatch_since_s: float | None = None
        self._reported_delta = 0

    def on_beam_credit(self, now_s: float) -> None:
        self.beam_count += 1
        self._track_mismatch(now_s)

    def on_truth_retained(self, now_s: float) -> None:
        self.truth_count += 1
        self._track_mismatch(now_s)

    def _track_mismatch(self, now_s: float) -> None:
        if self.beam_count == self.truth_count:
            self._mismatch_since_s = None
            self._reported_delta = 0
        elif self._mismatch_since_s is None:
            self._mismatch_since_s = now_s

    def poll(self, now_s: float) -> dict | None:
        """Return one report per NEW persistent delta value, else None."""
        if self._mismatch_since_s is None:
            return None
        if now_s - self._mismatch_since_s < self.tolerance_s:
            return None
        delta = self.beam_count - self.truth_count
        if delta == self._reported_delta:
            return None
        self._reported_delta = delta
        return {
            "event": "beam_false_credit" if delta > 0 else "beam_missed_credit",
            "beam_count": self.beam_count,
            "truth_count": self.truth_count,
            "delta": delta,
        }
