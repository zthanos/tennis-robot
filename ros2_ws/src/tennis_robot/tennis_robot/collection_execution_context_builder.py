"""Pure serializer: ``CollectionRoutePlan`` -> execution-context field values.

Phase 6B.  Produces the transport-agnostic value object that the Phase 6C ROS
adapter will copy field-for-field into the ``tennis_robot_msgs/
CollectionExecutionContext`` message, plus the flattened ``nav_msgs/Path``
pose list the follower will send on ``FollowPath``.  No ROS import, no msg
population here — only immutable dataclasses carrying every wire field value.

The two outputs are hash-bound: ``path_sha256`` is
:func:`collection_path_sha256_v1` over exactly the ``follow_path_poses`` list
carried alongside it, so the C++ controller (which recomputes the hash on the
received path) accepts the matching ``setPlan``.

Mirrors the C++ acceptance gates so a well-formed plan yields an ACCEPTED
context:

* ``controller_tuning`` positivity == ``valid_tuning``.
* ``configuration_snapshot_json`` is canonical JSON (sorted keys, compact
  separators, no NaN/Inf) so ``nlohmann::json::parse(s).dump() == s`` holds in
  ``valid_load_context``.
* segment/crossing progress ordering follows the immutable
  ``CollectionRoutePlan`` contract, which already satisfies the C++
  ``valid_load_context`` segment/crossing checks.
* ``follow_path_poses`` drops exact-duplicate segment-join poses so every
  consecutive 2D step is > 0 (``make_tracking_plan`` requirement) while the
  accumulated polyline length still equals ``terminal_progress_s`` and the last
  pose equals ``terminal_pose``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math

from tennis_robot.collection_path_canonicalization import (
    CanonicalPose,
    collection_path_sha256_v1,
)
from tennis_robot.collection_route_types import (
    CollectionRoutePlan,
    ExecutionProfile,
    RouteSegment,
    RouteSegmentType,
)

# Mirror of tennis_robot_msgs/CollectionExecutionSegment uint8 constants.
_SEGMENT_TYPE_CODE = {
    RouteSegmentType.CONNECTOR: 0,
    RouteSegmentType.FUNNEL_PASS: 1,
    RouteSegmentType.TERMINAL_CONNECTOR: 2,
}

# The C++ plugin only accepts this schema string.
CONTEXT_SCHEMA_VERSION = "collection-execution-context/v1"

# Segment-join poses are the same geometric point reached from two sides (a
# connector's materialized endpoint vs the next segment's start pose), so they
# can differ by floating-point noise (~1e-15 m) rather than being bit-identical.
# The C++ tracking core requires strictly increasing per-pose progress, so any
# such sub-nanometre gap must be collapsed.  This threshold is far below any
# real path geometry (dense arc chords are centimetres) and far above fp noise.
_JOIN_DEDUP_EPSILON_M = 1e-9

# 1-1 with CollectionExecutionProfile.msg, in message field order.
_PROFILE_FIELDS = (
    "nominal_speed_mps",
    "min_speed_mps",
    "max_speed_mps",
    "nominal_speed_warning_tolerance_mps",
    "max_acceleration_mps2",
    "max_deceleration_mps2",
    "required_entry_m",
    "required_run_in_m",
    "required_run_out_m",
    "max_curvature_per_m",
    "max_lateral_error_m",
    "max_heading_error_rad",
    "allow_reversing",
    "allow_standalone_rotate",
)


class ExecutionContextBuildError(ValueError):
    """The plan / inputs cannot yield a valid execution context."""


@dataclass(frozen=True)
class ControllerTuning:
    """Runtime controller tuning (5 fields), mirror of CollectionControllerTuning.

    Not part of ``CollectionRouteConfiguration``: it is controller config that
    6C supplies (nav2 params).  Validated exactly like the C++ ``valid_tuning``
    (all finite and strictly positive) so a built context loads.
    """

    lookahead_distance_m: float
    max_angular_velocity_rad_s: float
    progress_projection_window_m: float
    crossing_speed_window_m: float
    terminal_progress_tolerance_m: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
                raise ExecutionContextBuildError(f"controller tuning {name} must be finite and > 0")


@dataclass(frozen=True)
class ExecutionContextProfile:
    nominal_speed_mps: float
    min_speed_mps: float
    max_speed_mps: float
    nominal_speed_warning_tolerance_mps: float
    max_acceleration_mps2: float
    max_deceleration_mps2: float
    required_entry_m: float
    required_run_in_m: float
    required_run_out_m: float
    max_curvature_per_m: float
    max_lateral_error_m: float
    max_heading_error_rad: float
    allow_reversing: bool
    allow_standalone_rotate: bool


@dataclass(frozen=True)
class ExecutionContextCrossing:
    ball_id: str
    position_x_m: float
    position_y_m: float
    progress_s: float
    heading_rad: float
    predicted_lateral_error: float


@dataclass(frozen=True)
class ExecutionContextSegment:
    segment_id: str
    segment_type: int
    progress_start_s: float
    progress_end_s: float
    execution_profile: ExecutionContextProfile
    planned_crossings: tuple[ExecutionContextCrossing, ...]


@dataclass(frozen=True)
class CollectionExecutionContextValues:
    """Every ``CollectionExecutionContext`` field value + the path to send."""

    context_schema_version: str
    plan_id: str
    path_sha256: str
    context_activation_timeout_s: float
    segments: tuple[ExecutionContextSegment, ...]
    controller_tuning: ControllerTuning
    terminal_progress_s: float
    terminal_pose: CanonicalPose
    configuration_snapshot_json: str
    map_frame: str
    follow_path_poses: tuple[CanonicalPose, ...]


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _pose_from_yaw(x_m: float, y_m: float, yaw_rad: float) -> CanonicalPose:
    qx, qy, qz, qw = _quaternion_from_yaw(yaw_rad)
    return CanonicalPose(x_m, y_m, 0.0, qx, qy, qz, qw)


def build_follow_path_poses(plan: CollectionRoutePlan) -> tuple[CanonicalPose, ...]:
    """Flatten the plan's ordered segment paths into one pose list.

    Coincident join poses (a connector exit that meets the next segment's entry
    at the same point, up to floating-point noise) are dropped so every
    consecutive 2D step is strictly positive and progress strictly increases, as
    the C++ ``make_tracking_plan`` / tracking core require.
    """
    poses: list[CanonicalPose] = []
    last_xy: tuple[float, float] | None = None
    for segment in plan.segments:
        for point in segment.path.points:
            pose = point.pose
            xy = (pose.x_m, pose.y_m)
            if last_xy is not None and math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1]) <= _JOIN_DEDUP_EPSILON_M:
                continue  # coincident join pose (same point from two segments)
            poses.append(_pose_from_yaw(pose.x_m, pose.y_m, pose.yaw_rad))
            last_xy = xy
    return tuple(poses)


def _polyline_length_2d(poses: tuple[CanonicalPose, ...]) -> float:
    return sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(poses, poses[1:])
    )


def canonical_configuration_snapshot_json(plan: CollectionRoutePlan) -> str:
    """Canonical JSON of the plan's configuration snapshot.

    Sorted keys + compact separators + rejected NaN/Inf, so the C++
    ``nlohmann::json::parse(s).dump() == s`` canonicality check passes.
    """
    try:
        return json.dumps(
            plan.configuration_snapshot.to_dict(),
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    except ValueError as exc:  # non-finite float in configuration
        raise ExecutionContextBuildError(f"configuration snapshot not serializable: {exc}") from exc


def _build_profile(profile: ExecutionProfile) -> ExecutionContextProfile:
    return ExecutionContextProfile(*(getattr(profile, name) for name in _PROFILE_FIELDS))


def _build_segment(segment: RouteSegment) -> ExecutionContextSegment:
    crossings = tuple(
        ExecutionContextCrossing(
            crossing.ball_id,
            crossing.position_xy.x_m,
            crossing.position_xy.y_m,
            crossing.progress_s,
            crossing.heading_rad,
            crossing.predicted_lateral_error,
        )
        for crossing in segment.planned_crossings
    )
    return ExecutionContextSegment(
        segment.id,
        _SEGMENT_TYPE_CODE[segment.type],
        segment.progress_start_m,
        segment.progress_end_m,
        _build_profile(segment.execution_profile),
        crossings,
    )


def build_execution_context(
    plan: CollectionRoutePlan,
    *,
    controller_tuning: ControllerTuning,
    context_schema_version: str,
    context_activation_timeout_s: float,
) -> CollectionExecutionContextValues:
    """Serialize an executable plan into execution-context field values.

    ``plan`` must be an executable (feasible/partial) ``CollectionRoutePlan``:
    the C++ ``valid_load_context`` rejects an empty ``segments`` list, so
    non-executable plans are a typed build error here rather than a context the
    controller would reject.
    """
    if not isinstance(plan, CollectionRoutePlan):
        raise ExecutionContextBuildError("plan must be a CollectionRoutePlan")
    if not isinstance(controller_tuning, ControllerTuning):
        raise ExecutionContextBuildError("controller_tuning must be a ControllerTuning")
    if not isinstance(context_schema_version, str) or not context_schema_version:
        raise ExecutionContextBuildError("context_schema_version must be a non-empty string")
    if (
        isinstance(context_activation_timeout_s, bool)
        or not isinstance(context_activation_timeout_s, (int, float))
        or not math.isfinite(context_activation_timeout_s)
        or context_activation_timeout_s <= 0.0
    ):
        raise ExecutionContextBuildError("context_activation_timeout_s must be finite and > 0")
    if not plan.segments:
        raise ExecutionContextBuildError("cannot build execution context for a plan without segments")

    poses = build_follow_path_poses(plan)
    path_sha256 = collection_path_sha256_v1(plan.map_frame, poses)
    segments = tuple(_build_segment(segment) for segment in plan.segments)
    terminal = plan.terminal_pose

    # terminal_progress_s is the progress the controller *reaches* at the
    # terminal pose.  The controller measures progress as the cumulative 2D chord
    # length of the received path, so it is the flattened polyline length — NOT
    # the plan's arc-based total_length_m.  For any curved connector the arc
    # length strictly exceeds the chord sum, and the C++ tracking core hard-
    # rejects terminal_progress_s > path.back().progress_s (no tolerance); arc
    # densification (Phase 6B.1) shrinks that gap to well under
    # terminal_progress_tolerance_m but never closes it, so the terminal must be
    # chord-based.  For a straight route chord == arc and this equals
    # total_length_m.  Segment progress spans stay arc-based (unchanged).
    terminal_progress_s = _polyline_length_2d(poses)

    return CollectionExecutionContextValues(
        context_schema_version=context_schema_version,
        plan_id=plan.plan_id,
        path_sha256=path_sha256,
        context_activation_timeout_s=float(context_activation_timeout_s),
        segments=segments,
        controller_tuning=controller_tuning,
        terminal_progress_s=terminal_progress_s,
        terminal_pose=_pose_from_yaw(terminal.x_m, terminal.y_m, terminal.yaw_rad),
        configuration_snapshot_json=canonical_configuration_snapshot_json(plan),
        map_frame=plan.map_frame,
        follow_path_poses=poses,
    )
