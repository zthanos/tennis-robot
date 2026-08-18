"""Rigidly express a planned collection route in another planar frame.

Once used to freeze the whole route into ``odom`` before execution.  That is no
longer done: the route stays in the surveyed ``map`` frame it was planned in,
because the balls are in ``map`` too and an odom-frozen corridor slides away
from them by the accumulated localization correction -- 0.13 to 0.44 m measured
live against a 0.205 m funnel half-width (debug log #72).  The collection
controller now brings its pose into the plan frame instead.

The transform itself is exact and is kept: it is what the regression tests use
to model the old behaviour and prove the new one no longer drifts.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from tennis_robot.collection_route_types import (
    CollectionRoutePlan,
    Path2D,
    PathPoint,
    PlannedCrossing,
    Point2D,
    Pose2D,
)


@dataclass(frozen=True)
class RigidTransform2D:
    """Transform from one planar frame into another."""

    target_frame: str
    source_frame: str
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not self.target_frame or not self.source_frame:
            raise ValueError("transform frames must be non-empty")
        if not all(math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("transform values must be finite")

    def point(self, point: Point2D) -> Point2D:
        cosine, sine = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return Point2D(
            self.x_m + cosine * point.x_m - sine * point.y_m,
            self.y_m + sine * point.x_m + cosine * point.y_m,
        )

    def pose(self, pose: Pose2D) -> Pose2D:
        point = self.point(Point2D(pose.x_m, pose.y_m))
        return Pose2D(point.x_m, point.y_m, pose.yaw_rad + self.yaw_rad)


def transform_collection_plan(
    plan: CollectionRoutePlan, transform: RigidTransform2D
) -> CollectionRoutePlan:
    """Return an equivalent immutable plan expressed in ``target_frame``."""
    if not isinstance(plan, CollectionRoutePlan):
        raise TypeError("plan must be a CollectionRoutePlan")
    if plan.map_frame != transform.source_frame:
        raise ValueError(
            f"plan frame {plan.map_frame!r} does not match transform source "
            f"{transform.source_frame!r}"
        )

    def path(source: Path2D) -> Path2D:
        return Path2D(tuple(PathPoint(transform.pose(item.pose)) for item in source.points))

    segments = tuple(
        replace(
            segment,
            path=path(segment.path),
            planned_crossings=tuple(
                replace(
                    crossing,
                    position_xy=transform.point(crossing.position_xy),
                    heading_rad=crossing.heading_rad + transform.yaw_rad,
                )
                for crossing in segment.planned_crossings
            ),
        )
        for segment in plan.segments
    )
    return replace(
        plan,
        map_frame=transform.target_frame,
        start_pose=transform.pose(plan.start_pose),
        terminal_pose=transform.pose(plan.terminal_pose),
        segments=segments,
    )
