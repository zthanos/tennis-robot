"""What the robot actually did while executing a frozen collection route.

The plan says where the robot intended to drive and which crossing was meant to
collect which ball.  Nothing so far has recorded what happened next, so a ball
that did not end up in the basket could equally be a planning error, a tracking
error, a ball that had already been knocked aside, or a collector that ran over
it without picking it up.  These four have different fixes and the route
outcome alone cannot tell them apart.

This module is the recording half: an immutable, self-describing trace of the
execution, kept deliberately small.

*Compact by construction.*  Trajectory samples are decimated by distance and
time rather than taken every control tick, crossing samples are only kept while
a crossing is active, and beam events are edges rather than levels.  A route of
a hundred metres costs a few hundred rows.  The previous distributed runs were
hurt by high-rate topics (debug log #48), so nothing here subscribes to a raw
stream or publishes over the network: the recorder is fed from data the executor
already has, and it writes to a local file.

*Diagnostic only.*  A trace never re-enters planning.  Ball observations
recorded here are for deciding afterwards whether a ball moved, and are kept
separate from the frozen planning snapshot.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping


class ExecutionTraceError(ValueError):
    """A trace record was constructed or parsed with invalid contents."""


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ExecutionTraceError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise ExecutionTraceError(f"{name} must be >= {minimum}")
    return float(value)


def _text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ExecutionTraceError(f"{name} must be a non-empty string")
    return value


def _optional_text(value: Any, name: str) -> str | None:
    if value is None:
        return None
    return _text(value, name)


def _fields(data: Mapping[str, Any], expected: set[str], name: str) -> None:
    if not isinstance(data, Mapping):
        raise ExecutionTraceError(f"{name} must be a mapping")
    if set(data) != expected:
        raise ExecutionTraceError(f"{name} fields must be exactly {sorted(expected)}")


@dataclass(frozen=True)
class TrajectorySample:
    """Where the robot was, from the localization the controller already uses."""

    t_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    linear_mps: float
    angular_rps: float
    segment_id: str | None = None
    progress_s: float | None = None

    def __post_init__(self) -> None:
        for name in ("t_s", "x_m", "y_m", "yaw_rad", "linear_mps", "angular_rps"):
            _finite(getattr(self, name), name)
        if self.segment_id is not None:
            _text(self.segment_id, "segment_id")
        if self.progress_s is not None:
            _finite(self.progress_s, "progress_s", minimum=0.0)

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrajectorySample":
        _fields(data, set(cls.__dataclass_fields__), "TrajectorySample")
        return cls(**data)


@dataclass(frozen=True)
class CrossingSample:
    """Controller state while a planned crossing was the active target.

    These are the numbers the tracking core already computes; recording them
    means a miss can be examined at the crossing rather than averaged over a
    whole segment, which is where a route that tracks well overall can still
    drive past a ball.
    """

    t_s: float
    ball_id: str
    segment_id: str
    progress_s: float
    crossing_progress_s: float
    lateral_error_m: float
    heading_error_rad: float
    measured_speed_mps: float

    def __post_init__(self) -> None:
        _text(self.ball_id, "ball_id")
        _text(self.segment_id, "segment_id")
        for name in ("t_s", "progress_s", "crossing_progress_s", "lateral_error_m",
                     "heading_error_rad", "measured_speed_mps"):
            _finite(getattr(self, name), name)

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CrossingSample":
        _fields(data, set(cls.__dataclass_fields__), "CrossingSample")
        return cls(**data)


@dataclass(frozen=True)
class BeamEvent:
    """A rising or falling edge of an intake beam, with what was active then.

    The beams are the only physical evidence that something entered the funnel
    and reached the basket.  They are recorded as edges with the active segment
    and crossing attached; attributing an edge to a ball is a judgement made
    offline by the evaluator, never here.
    """

    t_s: float
    beam: str            # "entry" | "confirmed"
    rising: bool
    segment_id: str | None = None
    active_ball_id: str | None = None

    def __post_init__(self) -> None:
        _finite(self.t_s, "t_s")
        if self.beam not in ("entry", "confirmed"):
            raise ExecutionTraceError("beam must be 'entry' or 'confirmed'")
        if not isinstance(self.rising, bool):
            raise ExecutionTraceError("rising must be bool")
        _optional_text(self.segment_id, "segment_id")
        _optional_text(self.active_ball_id, "active_ball_id")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "BeamEvent":
        _fields(data, set(cls.__dataclass_fields__), "BeamEvent")
        return cls(**data)


@dataclass(frozen=True)
class ConfirmationEvent:
    """A collection the robot itself attributed to a ball.

    This is *authoritative* evidence and is kept distinct from a raw
    :class:`BeamEvent`: the beam is a physical transition, whereas this is the
    controller's own association of that transition with a planned crossing
    (``collect_route.confirmations``).  Nothing is invented here -- every field
    is copied from the confirmation the runtime already builds, and absent
    fields stay absent rather than being guessed.

    ``lateral_error_m`` is the controller's cross-track error of
    ``base_footprint`` against the planned path at the moment the beam fired.
    It is NOT the distance from the ball to the collector mouth; the two are
    different quantities and are compared, never merged (debug log #68).
    """

    t_s: float
    confirmation_id: int
    ball_id: str
    association: str
    segment_id: str | None = None
    progress_s: float | None = None
    crossing_progress_s: float | None = None
    lateral_error_m: float | None = None
    heading_error_rad: float | None = None
    measured_speed_mps: float | None = None

    def __post_init__(self) -> None:
        _finite(self.t_s, "t_s")
        if isinstance(self.confirmation_id, bool) or not isinstance(self.confirmation_id, int):
            raise ExecutionTraceError("confirmation_id must be an int")
        _text(self.ball_id, "ball_id")
        _text(self.association, "association")
        _optional_text(self.segment_id, "segment_id")
        for name in ("progress_s", "crossing_progress_s", "lateral_error_m",
                     "heading_error_rad", "measured_speed_mps"):
            value = getattr(self, name)
            if value is not None:
                _finite(value, name)

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ConfirmationEvent":
        _fields(data, set(cls.__dataclass_fields__), "ConfirmationEvent")
        return cls(**data)


@dataclass(frozen=True)
class BallObservation:
    """A ball seen during execution, kept apart from the planning snapshot.

    The planner's belief is frozen; this is what perception saw afterwards.
    Both are needed to tell "the robot missed the ball" from "the ball was not
    there any more".
    """

    t_s: float
    ball_id: str
    x_m: float
    y_m: float
    confidence: float = 1.0

    def __post_init__(self) -> None:
        _text(self.ball_id, "ball_id")
        for name in ("t_s", "x_m", "y_m"):
            _finite(getattr(self, name), name)
        _finite(self.confidence, "confidence", minimum=0.0)

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "BallObservation":
        _fields(data, set(cls.__dataclass_fields__), "BallObservation")
        return cls(**data)


@dataclass(frozen=True)
class ExecutionTrace:
    """Everything recorded about one execution of one frozen plan."""

    schema_version: str
    run_id: str
    plan_id: str
    scan_id: str
    samples: tuple[TrajectorySample, ...] = ()
    crossings: tuple[CrossingSample, ...] = ()
    beams: tuple[BeamEvent, ...] = ()
    observations: tuple[BallObservation, ...] = ()
    confirmations: tuple[ConfirmationEvent, ...] = ()

    def __post_init__(self) -> None:
        for name in ("schema_version", "run_id", "plan_id", "scan_id"):
            _text(getattr(self, name), name)
        for name, kind in (
            ("samples", TrajectorySample), ("crossings", CrossingSample),
            ("beams", BeamEvent), ("observations", BallObservation),
            ("confirmations", ConfirmationEvent),
        ):
            items = getattr(self, name)
            if not isinstance(items, tuple) or any(not isinstance(item, kind) for item in items):
                raise ExecutionTraceError(f"{name} must be a tuple of {kind.__name__}")
            times = [item.t_s for item in items]
            if any(later < earlier for earlier, later in zip(times, times[1:])):
                raise ExecutionTraceError(f"{name} must be ordered by time")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "plan_id": self.plan_id,
            "scan_id": self.scan_id,
            "samples": [item.to_dict() for item in self.samples],
            "crossings": [item.to_dict() for item in self.crossings],
            "beams": [item.to_dict() for item in self.beams],
            "observations": [item.to_dict() for item in self.observations],
            "confirmations": [item.to_dict() for item in self.confirmations],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExecutionTrace":
        """Read v1 and v2 payloads alike.

        v1 predates attributed confirmations; such a trace loads with none, and
        the evaluator then treats it conservatively rather than inferring a
        collection that was never recorded.
        """
        payload = dict(data)
        payload.setdefault("confirmations", [])
        _fields(payload, set(cls.__dataclass_fields__), "ExecutionTrace")
        return cls(
            payload["schema_version"], payload["run_id"], payload["plan_id"], payload["scan_id"],
            tuple(TrajectorySample.from_dict(item) for item in payload["samples"]),
            tuple(CrossingSample.from_dict(item) for item in payload["crossings"]),
            tuple(BeamEvent.from_dict(item) for item in payload["beams"]),
            tuple(BallObservation.from_dict(item) for item in payload["observations"]),
            tuple(ConfirmationEvent.from_dict(item) for item in payload["confirmations"]),
        )


# v2 adds attributed confirmations.  A v1 trace still reads, with none.
TRACE_SCHEMA_VERSION = "collection-execution-trace/v2"


@dataclass
class ExecutionTraceRecorder:
    """Accumulates a trace, decimating trajectory samples as they arrive.

    Decimation is by distance *or* elapsed time, whichever comes first, so a
    slow turn is still sampled densely enough in time and a fast straight does
    not produce a row per tick.  Both bounds and the total row cap are explicit
    rather than defaulted, because a recorder that silently drops the end of a
    run is worse than one that is not running at all.
    """

    run_id: str
    plan_id: str
    scan_id: str
    minimum_spacing_m: float
    minimum_interval_s: float
    maximum_samples: int
    _samples: list = field(default_factory=list)
    _crossings: list = field(default_factory=list)
    _beams: list = field(default_factory=list)
    _observations: list = field(default_factory=list)
    _confirmations: list = field(default_factory=list)
    _last_kept: TrajectorySample | None = None
    _dropped: int = 0
    _beam_state: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("run_id", "plan_id", "scan_id"):
            _text(getattr(self, name), name)
        for name in ("minimum_spacing_m", "minimum_interval_s"):
            _finite(getattr(self, name), name, minimum=0.0)
        if isinstance(self.maximum_samples, bool) or not isinstance(self.maximum_samples, int) or self.maximum_samples <= 0:
            raise ExecutionTraceError("maximum_samples must be a positive int")

    @property
    def dropped_samples(self) -> int:
        return self._dropped

    def record_pose(self, sample: TrajectorySample) -> bool:
        """Keep this pose if it is far enough or late enough since the last."""
        if not isinstance(sample, TrajectorySample):
            raise ExecutionTraceError("record_pose expects a TrajectorySample")
        if self._last_kept is not None:
            moved = math.hypot(
                sample.x_m - self._last_kept.x_m, sample.y_m - self._last_kept.y_m
            )
            waited = sample.t_s - self._last_kept.t_s
            if moved < self.minimum_spacing_m and waited < self.minimum_interval_s:
                self._dropped += 1
                return False
        if len(self._samples) >= self.maximum_samples:
            self._dropped += 1
            return False
        self._samples.append(sample)
        self._last_kept = sample
        return True

    def record_crossing(self, sample: CrossingSample) -> bool:
        """Keep one row per distinct controller state while a crossing is active."""
        if not isinstance(sample, CrossingSample):
            raise ExecutionTraceError("record_crossing expects a CrossingSample")
        if len(self._crossings) >= self.maximum_samples:
            self._dropped += 1
            return False
        if self._crossings:
            previous = self._crossings[-1]
            same = (
                previous.ball_id == sample.ball_id
                and previous.segment_id == sample.segment_id
                and abs(previous.progress_s - sample.progress_s) < 1e-9
            )
            if same:
                return False
        self._crossings.append(sample)
        return True

    def record_beam(self, *, t_s: float, beam: str, level: bool, segment_id=None, active_ball_id=None) -> bool:
        """Record only edges: the level itself is not interesting, changes are."""
        previous = self._beam_state.get(beam)
        if previous is not None and previous == bool(level):
            return False
        self._beam_state[beam] = bool(level)
        if previous is None and not level:
            # Starting low is not an edge worth a row.
            return False
        self._beams.append(
            BeamEvent(t_s, beam, bool(level), segment_id, active_ball_id)
        )
        return True

    def record_observation(self, observation: BallObservation) -> bool:
        if not isinstance(observation, BallObservation):
            raise ExecutionTraceError("record_observation expects a BallObservation")
        if len(self._observations) >= self.maximum_samples:
            self._dropped += 1
            return False
        self._observations.append(observation)
        return True

    def record_confirmation(self, confirmation: ConfirmationEvent) -> bool:
        """Keep an attributed confirmation exactly as the runtime built it."""
        if not isinstance(confirmation, ConfirmationEvent):
            raise ExecutionTraceError("record_confirmation expects a ConfirmationEvent")
        self._confirmations.append(confirmation)
        return True

    def build(self) -> ExecutionTrace:
        return ExecutionTrace(
            TRACE_SCHEMA_VERSION, self.run_id, self.plan_id, self.scan_id,
            tuple(self._samples), tuple(self._crossings), tuple(self._beams),
            tuple(self._observations), tuple(self._confirmations),
        )
