"""Rigidly freeze a planned collection route into its execution frame.

The planner works in the surveyed ``map`` frame.  Nav2's controller server,
however, supplies robot poses in the local costmap frame (``odom`` in this
project).  The collection controller intentionally has no TF dependency, so
the complete immutable plan must be expressed in that same frame before its
execution context and path hash are built.
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
