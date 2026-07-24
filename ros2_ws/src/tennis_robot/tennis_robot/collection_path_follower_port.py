"""Live PathFollower port: drive the real C++ collection controller (Phase 6C.2).

Implements the executor's ``PathFollower`` Protocol against the
``CollectionFollowPath`` Nav2 controller.  It builds the frozen context + path +
sha256 with the Phase 6B serializers, runs the Load -> FollowPath -> (execute) ->
Finalize handshake, and maps the controller's ``CollectionControllerState`` +
FollowPath goal status onto ``PathFollowerResult``.

Like the 6C.1 adapters this module imports no ``rclpy`` and no ROS message
types: every ROS touch-point is an injected, duck-typed handle (senders,
outcome/state providers, a clock).  The Phase 6C.2 node wiring supplies rclpy
implementations; unit tests supply fakes.  The sequencing/mapping logic is pure.

Handshake (non-blocking polling, like Nav2LaneNavigator):

* ``start(plan)``   builds the 6B context/path, sends Load (async).
* ``result()``      on Load ACCEPTED sends the FollowPath goal (controller_id
  ``CollectionFollowPath``) within ``context_activation_timeout_s``, then maps
  the latest state + goal status to a ``PathFollowerResult``.
* ``pause()`` / ``resume()``  send SetCollectionSafetyHold(hold=True/False).

Finalize decision: the follower sends ``FinalizeCollectionExecutionContext``
(action_outcome=SUCCEEDED) exactly once, automatically, on the FollowPath
action's terminal success — i.e. inside the ``result()`` transition to
``completed`` — because that is the only moment the C++ context is
``terminal_ready`` (otherwise the controller answers TERMINAL_NOT_REACHED).  The
executor therefore never has to know about Finalize.
"""

from __future__ import annotations

import math

from tennis_robot.collection_execution_context_builder import (
    ControllerTuning,
    build_execution_context,
)
from tennis_robot.collection_path_follower import CollectionPathFollower
from tennis_robot.collection_route_executor import (
    ExecutorReasonCode,
    PathFollowerResult,
    PathFollowerStatus,
)
from tennis_robot.collection_route_types import CollectionRoutePlan

# Mirror of tennis_robot_msgs/CollectionControllerState uint8 constants so the
# pure mapping never imports the ROS message.
LIFECYCLE_EXECUTING = 3
LIFECYCLE_SAFETY_PAUSED = 4
LIFECYCLE_SUCCEEDED = 5
LIFECYCLE_FAILED = 6

FAILURE_NONE = 0
FAILURE_SPEED_BELOW_MIN = 5
FAILURE_SPEED_ABOVE_MAX = 6
FAILURE_SAFETY_RESUME_INVALID = 14

# Human-readable labels for the CollectionControllerState.FAILURE_* uint8 codes,
# surfaced verbatim in the collection log so an abort explains itself instead of
# collapsing to a generic "path failed".  Mirrors the msg numbering exactly (the
# published code is the msg constant, not the C++ TrackingFailureCode index).
FAILURE_LABELS = {
    0: "none",
    1: "missing_context",
    2: "path_hash_mismatch",
    3: "context_activation_timeout",
    4: "profile_unenforceable",
    5: "speed_below_min",
    6: "speed_above_max",
    7: "run_in_insufficient",
    8: "run_out_insufficient",
    9: "curvature_exceeded",
    10: "trajectory_tube_exceeded",
    11: "non_monotonic_progress",
    12: "reverse_required",
    13: "standalone_rotate_required",
    14: "safety_resume_invalid",
    15: "heading_error_exceeded",
    16: "terminal_not_reached",
}

# Mirror of FinalizeCollectionExecutionContext.srv action_outcome.
FINALIZE_SUCCEEDED = 0

DEFAULT_CONTROLLER_ID = "CollectionFollowPath"

# FollowPath goal-status strings the transport maps from action_msgs/GoalStatus.
_GOAL_TERMINAL_FAILURES = frozenset({"rejected", "failed", "canceled", "aborted"})


class PathFollowerPortError(ValueError):
    """The live follower was configured with a missing or invalid value."""


def failure_reason_for_code(failure_code: int) -> ExecutorReasonCode:
    """Map a C++ CollectionControllerState.FAILURE_* code to an ExecutorReasonCode.

    Only ``safety_resume_invalid`` has a dedicated executor reason; every other
    hard controller failure (tube, curvature, speed, reverse, rotate,
    non-monotonic progress, heading, run-in/out, profile, terminal) collapses to
    the generic ``path_failed`` — the executor treats them all as a tracking
    abort.
    """
    if failure_code == FAILURE_SAFETY_RESUME_INVALID:
        return ExecutorReasonCode.SAFETY_RESUME_INVALID
    return ExecutorReasonCode.PATH_FAILED


def _failure_detail(failure_code, state) -> str:
    """Human-readable abort diagnostic: specific failure label + live geometry.

    Answers "why did the route stop" in the collection log instead of the
    generic executor reason, which collapses every hard controller failure to
    ``path_failed``.
    """
    label = FAILURE_LABELS.get(failure_code, f"failure_{failure_code}")
    if not state:
        return label
    metrics = []
    segment = state.get("active_segment_id")
    if segment:
        metrics.append(f"seg {segment}")
    fields = [
        ("progress_s", "progress", "m"),
        ("lateral_error_m", "lat_err", "m"),
        ("heading_error_rad", "head_err", "rad"),
    ]
    # measured_speed is only populated at crossings (0.0 on connectors), so it is
    # meaningful only for the speed-limit failures — including it elsewhere would
    # print a misleading "speed 0.000m/s".
    if failure_code in (FAILURE_SPEED_BELOW_MIN, FAILURE_SPEED_ABOVE_MAX):
        fields.append(("measured_speed_mps", "speed", "m/s"))
    for key, name, unit in fields:
        value = state.get(key)
        if isinstance(value, (int, float)):
            metrics.append(f"{name} {float(value):.3f}{unit}")
    return f"{label} | {' '.join(metrics)}" if metrics else label


class LiveCollectionPathFollower:
    """PathFollower port driving the CollectionFollowPath controller."""

    def __init__(
        self,
        *,
        controller_tuning: ControllerTuning,
        context_schema_version: str,
        context_activation_timeout_s: float,
        load_sender,
        load_outcome_provider,
        follow_path_sender,
        goal_status_provider,
        state_provider,
        hold_sender,
        finalize_sender,
        clock,
        controller_id: str = DEFAULT_CONTROLLER_ID,
        execution_plan_transformer=lambda plan: plan,
    ) -> None:
        if not isinstance(controller_tuning, ControllerTuning):
            raise PathFollowerPortError("controller_tuning must be a ControllerTuning")
        if not isinstance(context_schema_version, str) or not context_schema_version:
            raise PathFollowerPortError("context_schema_version must be a non-empty string")
        if (
            isinstance(context_activation_timeout_s, bool)
            or not isinstance(context_activation_timeout_s, (int, float))
            or not math.isfinite(context_activation_timeout_s)
            or context_activation_timeout_s <= 0.0
        ):
            raise PathFollowerPortError("context_activation_timeout_s must be finite and > 0")
        if not controller_id:
            raise PathFollowerPortError("controller_id must be non-empty")
        if not callable(execution_plan_transformer):
            raise PathFollowerPortError("execution_plan_transformer must be callable")
        for name, handle in (
            ("load_sender", load_sender), ("load_outcome_provider", load_outcome_provider),
            ("follow_path_sender", follow_path_sender), ("goal_status_provider", goal_status_provider),
            ("state_provider", state_provider), ("hold_sender", hold_sender),
            ("finalize_sender", finalize_sender),
        ):
            if not callable(handle):
                raise PathFollowerPortError(f"{name} must be callable")

        self._controller_tuning = controller_tuning
        self._context_schema_version = context_schema_version
        self._context_activation_timeout_s = float(context_activation_timeout_s)
        self._load_sender = load_sender
        self._load_outcome_provider = load_outcome_provider
        self._follow_path_sender = follow_path_sender
        self._goal_status_provider = goal_status_provider
        self._state_provider = state_provider
        self._hold_sender = hold_sender
        self._finalize_sender = finalize_sender
        self._clock = clock
        self._controller_id = controller_id
        self._execution_plan_transformer = execution_plan_transformer

        self._reset()

    def _reset(self) -> None:
        self._plan: CollectionRoutePlan | None = None
        self._pure: CollectionPathFollower | None = None
        self._context = None
        self._plan_id: str | None = None
        self._path_sha256: str | None = None
        self._tube_radius_m: float | None = None
        self._phase = "idle"
        self._load_started_at_s: float | None = None
        self._finalize_sent = False
        self.finalize_accepted: bool | None = None
        self._terminal: PathFollowerResult | None = None

    # ── PathFollower Protocol ────────────────────────────────────────────────
    def start(self, plan: CollectionRoutePlan) -> None:
        if not isinstance(plan, CollectionRoutePlan) or not plan.is_executable:
            raise PathFollowerPortError("path follower requires an executable CollectionRoutePlan")
        self._reset()
        plan = self._execution_plan_transformer(plan)
        if not isinstance(plan, CollectionRoutePlan) or not plan.is_executable:
            raise PathFollowerPortError(
                "execution_plan_transformer must return an executable CollectionRoutePlan"
            )
        context = build_execution_context(
            plan,
            controller_tuning=self._controller_tuning,
            context_schema_version=self._context_schema_version,
            context_activation_timeout_s=self._context_activation_timeout_s,
        )
        self._plan = plan
        self._pure = CollectionPathFollower(plan)
        self._context = context
        self._plan_id = context.plan_id
        self._path_sha256 = context.path_sha256
        self._tube_radius_m = plan.configuration_snapshot.safety.trajectory_tube_radius_m
        self._load_started_at_s = self._clock.now_s()
        self._phase = "loading"
        self._load_sender(context)

    def result(self) -> PathFollowerResult:
        if self._terminal is not None:
            return self._terminal
        if self._phase == "loading":
            return self._tick_loading()
        if self._phase == "executing":
            return self._tick_executing()
        raise PathFollowerPortError("result() called before start()")

    def pause(self) -> None:
        self._hold_sender(plan_id=self._plan_id, path_sha256=self._path_sha256, hold=True)

    def resume(self) -> None:
        self._hold_sender(plan_id=self._plan_id, path_sha256=self._path_sha256, hold=False)

    # ── phases ───────────────────────────────────────────────────────────────
    def _tick_loading(self) -> PathFollowerResult:
        outcome = self._load_outcome_provider()
        if outcome == "rejected":
            return self._fail(ExecutorReasonCode.PATH_FAILED)
        if outcome == "accepted":
            self._follow_path_sender(
                map_frame=self._context.map_frame,
                poses=self._context.follow_path_poses,
                controller_id=self._controller_id,
            )
            self._phase = "executing"
            return self._pre_execution_running()
        # still pending: the matching FollowPath must be sent within the
        # activation window, so a stalled Load is a failure.
        if self._clock.now_s() - self._load_started_at_s > self._context_activation_timeout_s:
            return self._fail(ExecutorReasonCode.PATH_FAILED)
        return self._pre_execution_running()

    def _tick_executing(self) -> PathFollowerResult:
        goal_status = self._goal_status_provider()
        state = self._state_provider()
        failure_code = state.get("failure_reason", FAILURE_NONE) if state else FAILURE_NONE
        lifecycle = state.get("lifecycle_state") if state else None

        if goal_status == "succeeded" or lifecycle == LIFECYCLE_SUCCEEDED:
            if not self._finalize():
                return self._fail(
                    ExecutorReasonCode.PATH_FAILED,
                    "collection controller rejected terminal finalize",
                )
            self._terminal = PathFollowerResult(PathFollowerStatus.COMPLETED)
            return self._terminal
        if lifecycle == LIFECYCLE_FAILED or (failure_code is not None and failure_code != FAILURE_NONE):
            return self._fail(failure_reason_for_code(failure_code), _failure_detail(failure_code, state))
        if goal_status in _GOAL_TERMINAL_FAILURES:
            return self._fail(ExecutorReasonCode.PATH_FAILED, f"nav2 goal {goal_status}")
        if lifecycle in (LIFECYCLE_EXECUTING, LIFECYCLE_SAFETY_PAUSED):
            progress_s = float(state.get("progress_s", 0.0))
            lateral_error_m = float(state.get("lateral_error_m", 0.0))
            tube_ok = lateral_error_m <= self._tube_radius_m
            remaining_run_in_m = self._pure.remaining_run_in_m(progress_s)
            return PathFollowerResult(
                PathFollowerStatus.RUNNING, progress_s, tube_ok, remaining_run_in_m, False, False
            )
        # Goal accepted/pending but not yet executing (or no state yet).
        return self._pre_execution_running()

    def _finalize(self) -> bool:
        if self._finalize_sent:
            return self.finalize_accepted
        self._finalize_sent = True
        self.finalize_accepted = bool(
            self._finalize_sender(
                plan_id=self._plan_id, path_sha256=self._path_sha256, action_outcome=FINALIZE_SUCCEEDED
            )
        )
        return self.finalize_accepted

    def _pre_execution_running(self) -> PathFollowerResult:
        return PathFollowerResult(
            PathFollowerStatus.RUNNING, 0.0, True, self._pure.remaining_run_in_m(0.0), False, False
        )

    def _fail(self, reason: ExecutorReasonCode, detail: str | None = None) -> PathFollowerResult:
        self._terminal = PathFollowerResult(PathFollowerStatus.FAILED, reason=reason, detail=detail)
        return self._terminal
