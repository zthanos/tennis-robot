"""Node-side capture of an execution trace, opt-in and cheap.

Wired into callbacks that already fire -- the controller state subscription the
executor keeps anyway, and the robot pose the node already caches -- so nothing
here adds a subscription, a timer or a topic.  That constraint is deliberate:
the distributed runs were previously hurt by high-rate traffic (debug log #48),
and an instrument that changes what it measures is worthless.

Enabled by ``COLLECTION_EXECUTION_TRACE_DIR``; absent, the whole thing is a
no-op and the executor behaves exactly as before.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from tennis_robot.collection_execution_trace import (
    BallObservation,
    ConfirmationEvent,
    CrossingSample,
    ExecutionTraceRecorder,
    TrajectorySample,
)

# Decimation: a sample every 10 cm or every half second, whichever comes first.
# At 0.35 m/s that is roughly three rows a second on a straight, which is dense
# enough to reconstruct a 0.205 m funnel mouth passing a ball and small enough
# that a hundred-metre route stays in the low thousands of rows.
DEFAULT_SPACING_M = 0.10
DEFAULT_INTERVAL_S = 0.5
DEFAULT_MAXIMUM_SAMPLES = 20000


class ExecutionTraceCapture:
    """Collects one trace per executed plan and writes it out at the end."""

    def __init__(self, *, directory, run_id, clock_fn, logger=None,
                 spacing_m=DEFAULT_SPACING_M, interval_s=DEFAULT_INTERVAL_S,
                 maximum_samples=DEFAULT_MAXIMUM_SAMPLES):
        self._directory = Path(directory)
        self._run_id = str(run_id)
        self._clock_fn = clock_fn
        self._logger = logger
        self._spacing_m = float(spacing_m)
        self._interval_s = float(interval_s)
        self._maximum_samples = int(maximum_samples)
        self._recorder: ExecutionTraceRecorder | None = None
        self._plan_id: str | None = None

    @property
    def active(self) -> bool:
        return self._recorder is not None

    def start(self, plan) -> None:
        self._plan_id = plan.plan_id
        self._recorder = ExecutionTraceRecorder(
            run_id=self._run_id, plan_id=plan.plan_id, scan_id=plan.scan_id,
            minimum_spacing_m=self._spacing_m, minimum_interval_s=self._interval_s,
            maximum_samples=self._maximum_samples,
        )

    def record_state(self, *, pose, state) -> None:
        """One controller-state callback: a pose row and, if active, a crossing.

        ``state`` is the controller message already being consumed for
        telemetry; nothing extra is requested from it.
        """
        if self._recorder is None or pose is None:
            return
        moment = self._clock_fn()
        segment_id = _text(getattr(state, "active_segment_id", None))
        progress = _number(getattr(state, "progress_s", None))
        self._recorder.record_pose(
            TrajectorySample(
                moment, pose.x_m, pose.y_m, pose.yaw_rad,
                _number(getattr(state, "measured_speed_mps", None)) or 0.0, 0.0,
                segment_id, progress,
            )
        )
        if not bool(getattr(state, "has_active_crossing", False)):
            return
        ball_id = _text(getattr(state, "active_ball_id", None))
        if not ball_id or not segment_id:
            return
        self._recorder.record_crossing(
            CrossingSample(
                moment, ball_id, segment_id, progress or 0.0,
                _number(getattr(state, "active_crossing_progress_s", None)) or 0.0,
                _number(getattr(state, "lateral_error_m", None)) or 0.0,
                _number(getattr(state, "heading_error_rad", None)) or 0.0,
                _number(getattr(state, "measured_speed_mps", None)) or 0.0,
            )
        )

    def record_beams(self, *, entry: bool, confirmed: bool, state=None) -> None:
        """Edges only, tagged with whatever was active when they fired."""
        if self._recorder is None:
            return
        moment = self._clock_fn()
        segment_id = _text(getattr(state, "active_segment_id", None)) if state else None
        ball_id = _text(getattr(state, "active_ball_id", None)) if state else None
        self._recorder.record_beam(
            t_s=moment, beam="entry", level=bool(entry),
            segment_id=segment_id, active_ball_id=ball_id,
        )
        self._recorder.record_beam(
            t_s=moment, beam="confirmed", level=bool(confirmed),
            segment_id=segment_id, active_ball_id=ball_id,
        )

    def record_confirmation(self, confirmation) -> None:
        """Persist a confirmation the runtime already attributed to a ball.

        Takes the dict the controller builds for ``collect_route.confirmations``
        and copies it verbatim.  An unattributed confirmation (no ball) is not
        recorded as evidence about any ball -- it is dropped here rather than
        guessed at later.
        """
        if self._recorder is None or not isinstance(confirmation, dict):
            return
        ball_id = confirmation.get("ball_id")
        association = confirmation.get("association")
        identifier = confirmation.get("confirmation_id")
        if not ball_id or not association or not isinstance(identifier, int):
            return
        self._recorder.record_confirmation(
            ConfirmationEvent(
                self._clock_fn(), identifier, str(ball_id), str(association),
                _text(confirmation.get("segment_id")),
                _number(confirmation.get("progress_s")),
                _number(confirmation.get("crossing_progress_s")),
                _number(confirmation.get("lateral_error_m")),
                _number(confirmation.get("heading_error_rad")),
                _number(confirmation.get("measured_speed_mps")),
            )
        )

    def record_observation(self, *, ball_id, x_m, y_m, confidence=1.0) -> None:
        """A ball seen while driving.  Diagnostic only; never fed to planning."""
        if self._recorder is None:
            return
        self._recorder.record_observation(
            BallObservation(self._clock_fn(), str(ball_id), float(x_m), float(y_m), float(confidence))
        )

    def finish(self) -> Path | None:
        """Write the trace beside the planner audit and stop recording."""
        if self._recorder is None:
            return None
        trace = self._recorder.build()
        self._recorder = None
        safe = "".join(
            char if char.isalnum() or char in "-_." else "_"
            for char in f"{self._run_id}-{trace.plan_id}"
        )
        target = self._directory / f"{safe}.trace.json"
        temporary = self._directory / f".{safe}.trace.json.tmp"
        try:
            self._directory.mkdir(parents=True, exist_ok=True)
            temporary.write_text(
                json.dumps(trace.to_dict(), indent=1, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, target)
            if self._logger:
                self._logger.info(
                    f"collection execution trace saved: {target} "
                    f"({len(trace.samples)} poses, {len(trace.crossings)} crossings, "
                    f"{len(trace.beams)} beam edges)"
                )
            return target
        except (OSError, TypeError, ValueError) as exc:
            if self._logger:
                self._logger.error(f"collection execution trace write failed: {exc}")
            return None


def capture_from_env(*, run_id, clock_fn, logger=None) -> ExecutionTraceCapture | None:
    """Build a capture when the environment asks for one, else nothing."""
    directory = os.getenv("COLLECTION_EXECUTION_TRACE_DIR", "").strip()
    if not directory:
        return None
    return ExecutionTraceCapture(
        directory=directory, run_id=run_id, clock_fn=clock_fn, logger=logger
    )


def _text(value):
    return value if isinstance(value, str) and value else None


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)
