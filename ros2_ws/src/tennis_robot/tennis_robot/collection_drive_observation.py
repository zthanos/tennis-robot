"""Off-route ball discovery while the robot drives a collection route.

The 360 scan at the service-line T is the only input the *current* route is
planned from, and that stays true: nothing here touches the frozen snapshot the
controller is executing.  What this module adds is a second, independent list —
balls seen while driving that the 360 never confirmed — so the follow-up pass
can be planned from real observations instead of repeating the same 360 from the
same pose and re-discovering the same court.

Validation is deliberately *not* reimplemented.  Observations arrive already
adapted and validated by ``CollectionSnapshotRuntimeAdapter`` (RGB/depth
timestamp skew, detection-to-TF age, covariance), and the snapshot is built by
the same :class:`ScanSnapshotBuilder` the 360 uses, so the court-half filter,
data association and confirmation gates are identical.

The one thing that differs is what a "step" means.  In the 360 a step is a
heading; here it is a *viewpoint*: a new step every ``viewpoint_spacing_m`` of
travel.  The builder's ``min_distinct_scan_steps`` rule therefore still requires
a ball to be seen from two separate places before it is trusted — which while
driving is a stronger check than two adjacent headings taken on the spot,
because the baseline between viewpoints is metres rather than degrees.
"""

from __future__ import annotations

import math

from tennis_robot.collection_scan_snapshot import (
    ScanSnapshotBuilder,
    ScanSnapshotFailure,
    SpatialObservationRejection,
)
from tennis_robot.collection_route_types import Pose2D, ScanSnapshot, SnapshotBall


class DriveObservationError(RuntimeError):
    """Raised on invalid drive-observation configuration."""


def _require_finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise DriveObservationError(f"{name} must be a finite number")
    return float(value)


class DriveViewpointStepper:
    """Turn a stream of robot poses into discrete viewpoint ids.

    A viewpoint changes only after the robot has travelled
    ``viewpoint_spacing_m`` since the previous one, so two observations sharing
    an id were genuinely taken from the same place.
    """

    def __init__(self, *, viewpoint_spacing_m: float) -> None:
        spacing = _require_finite(viewpoint_spacing_m, "viewpoint_spacing_m")
        if spacing <= 0.0:
            raise DriveObservationError("viewpoint_spacing_m must be > 0")
        self._spacing_m = spacing
        self._anchor: tuple[float, float] | None = None
        self._index = 0
        self._visited: list[str] = []

    @property
    def visited_ids(self) -> tuple[str, ...]:
        return tuple(self._visited)

    def observe_pose(self, x_m: float, y_m: float) -> str:
        x_m = _require_finite(x_m, "x_m")
        y_m = _require_finite(y_m, "y_m")
        if self._anchor is None:
            self._anchor = (x_m, y_m)
        elif math.hypot(x_m - self._anchor[0], y_m - self._anchor[1]) >= self._spacing_m:
            self._index += 1
            self._anchor = (x_m, y_m)
        step_id = f"drive-vp-{self._index}"
        if not self._visited or self._visited[-1] != step_id:
            self._visited.append(step_id)
        return step_id

    def reset(self) -> None:
        self._anchor = None
        self._index = 0
        self._visited = []


class DriveObservationBuffer:
    """Duck-typed ``ScanSnapshotBuilder`` sink that defers the actual build.

    The adapter validates each detection against the TF available *at that
    moment*, so observations cannot be replayed later against fresh TF.  They
    are therefore adapted live into this buffer, and only the snapshot assembly
    is deferred to the end of the route — at which point the set of viewpoints
    actually visited is known and becomes the builder's expected steps, making
    the coverage gate an identity instead of something to bypass.
    """

    def __init__(self, *, scan_id: str) -> None:
        if not isinstance(scan_id, str) or not scan_id:
            raise DriveObservationError("scan_id must be a non-empty string")
        self.scan_id = scan_id
        self.accepted: list = []
        self.rejections: list = []
        self.visited_steps: set[str] = set()

    # ── the interface CollectionSnapshotRuntimeAdapter.forward drives ────────
    def add(self, item) -> None:
        if isinstance(item, SpatialObservationRejection):
            self.rejections.append(item)
            return
        self.accepted.append(item)

    def record_visited_step(self, scan_step_id: str) -> None:
        """Accept the adapter's coverage heartbeat without acting on it.

        For a 360 this marks a sector as observed even when every detection in
        the frame was rejected.  A drive has no sectors to cover, and the built
        snapshot's expected steps come from the accepted observations, so this
        is recorded for diagnostics only and never widens the coverage
        denominator.
        """
        if isinstance(scan_step_id, str) and scan_step_id:
            self.visited_steps.add(scan_step_id)

    @property
    def observation_count(self) -> int:
        return len(self.accepted)


def build_drive_snapshot(
    *,
    buffer: DriveObservationBuffer,
    configuration_snapshot,
    court_half_boundary,
    robot_pose: Pose2D,
    now_s: float,
    map_frame: str = "map",
    known_positions: tuple[tuple[float, float], ...] = (),
    merge_radius_m: float = 0.5,
):
    """Assemble the off-route snapshot, or return ``None`` when it has no targets.

    ``known_positions`` are the targets the finished route already knew about
    (its own snapshot balls, which includes everything it collected).  Anything
    within ``merge_radius_m`` of one of them is dropped: re-planning a ball the
    route already handled would loop the mission over the same court.
    """
    merge_radius_m = _require_finite(merge_radius_m, "merge_radius_m")
    if merge_radius_m < 0.0:
        raise DriveObservationError("merge_radius_m must be >= 0")
    now_s = _require_finite(now_s, "now_s")
    if not isinstance(robot_pose, Pose2D):
        raise DriveObservationError("robot_pose must be a Pose2D")
    if not buffer.accepted:
        return None
    # Coverage counts the steps that produced an accepted observation, so the
    # expected set is exactly those: a viewpoint the robot merely drove through
    # without seeing anything is not a gap in a sweep, and must not read as one.
    contributing_step_ids = tuple(
        dict.fromkeys(observation.scan_step_id for observation in buffer.accepted)
    )

    builder = ScanSnapshotBuilder(
        scan_id=buffer.scan_id,
        # The scan timeout guards the 360 rotation, not this accumulation, so
        # the epoch is the build itself rather than the start of the route.
        scan_timestamp_s=now_s,
        robot_pose_at_scan=robot_pose,
        configuration_snapshot=configuration_snapshot,
        expected_scan_step_ids=contributing_step_ids,
        court_half_boundary=court_half_boundary,
        map_frame=map_frame,
    )
    builder.start(now_s)
    for observation in buffer.accepted:
        builder.add(observation)
    try:
        snapshot = builder.finalize(now_s, robot_pose_at_scan=robot_pose)
    except ScanSnapshotFailure:
        # Too few confirmations or no covered viewpoint: nothing trustworthy was
        # seen while driving.  That is a normal outcome, not a route failure.
        return None

    fresh = tuple(
        ball
        for ball in snapshot.balls
        if not any(
            math.hypot(ball.position.x_m - x, ball.position.y_m - y) <= merge_radius_m
            for x, y in known_positions
        )
    )
    if not fresh:
        return None
    # Renumber so the follow-up plan's ball ids stay contiguous and unique.
    renumbered = tuple(
        SnapshotBall(
            f"{snapshot.scan_id}/target-{index + 1}",
            ball.position,
            ball.confidence,
            ball.position_covariance,
        )
        for index, ball in enumerate(fresh)
    )
    return ScanSnapshot(
        snapshot.scan_id,
        snapshot.scan_timestamp,
        snapshot.map_frame,
        snapshot.robot_pose_at_scan,
        renumbered,
        snapshot.configuration_snapshot,
    )
