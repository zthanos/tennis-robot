"""Pure builder: court_boundary.json (v2) -> planner ``CourtModel``.

Phase 6A of the continuous collection-route work.  This module is the sole
translation from the persisted survey artifact (schema
``court_knowledge_model/v2``, ``map`` frame, metres) into the immutable
``collection_route_planner_v2.CourtModel`` that the deterministic Phase 3A
geometry consumes.

Design is deliberately constrained so the planner's existing geometry
contract (never modified here) does the right thing:

* **navigable_polygon = the fence corners.**  The planner treats the
  *inflated exterior* of the navigable polygon as keepout
  (``_point_in_eroded_polygon`` / ``_segment_is_collision_free``), so the
  fence-as-court-boundary is already enforced by the polygon itself.  The
  fence must therefore NOT be added as one filled obstacle polygon: an
  interior ball would then be point-in-polygon and wrongly reported KEEPOUT.

* **The fence is ALSO represented as four thin ``"fence"``-kind wall
  obstacles, one per edge.**  The planner only derives fence/net tangent
  headings from obstacles whose kind is ``"net"`` or ``"fence"``
  (``_active_tangent_headings``).  Without these walls a ball hugging the
  fence would get a full 360° heading set instead of the required
  parallel-only (tangent) constraint.  Each wall is a thin quad whose inner
  long edge lies exactly on the fence edge and whose body is offset
  ``FENCE_WALL_THICKNESS_M`` *outward* (away from the play area), so no
  interior ball is ever point-in-wall and the wall's keepout is redundant
  with — never stricter than — the navigable exterior.  A ball near the
  middle of an edge activates only that edge's parallel long edges; the two
  short perpendicular end caps sit at the polygon corners and only matter in
  the genuine corner-constraint case.

* **The net is one thin ``"net"``-kind wall** spanning post-to-post.  Unlike
  the fence walls it is an interior wall (you cannot drive through the net):
  a ball on the net line is correctly KEEPOUT, and a ball near it gets the
  net-parallel tangent.  Cross-half filtering is NOT done here — that lives in
  the snapshot builder's ``CourtHalfBoundary``; the planner only needs the net
  as a wall for tangent + collision.

* **The two net posts are the endpoints of the net wall** rather than two
  separate ``"post"`` obstacles.  Folding them into the wall avoids spurious
  perpendicular tangents from tiny post polygons while still keeping the posts
  as hard keepout (they are the wall's end caps).

* **Interior obstacles** (``obstacles[]`` in the artifact) become axis-aligned
  rectangles from ``center`` + ``size_m``.  ``class`` maps to obstacle kind via
  ``_CLASS_TO_KIND`` (``perimeter_fixture`` -> ``"bench"``, i.e. a static
  keepout with no tangent sliding); any unknown class falls back to
  ``"other"``.  Neither ``"bench"`` nor ``"other"`` contributes tangent
  headings, which is correct: a small fixture is driven *around*, not slid
  *along*.

The core is a pure ``dict -> CourtModel`` function.  It performs NO file I/O
and imports NO ROS: the Phase 6C caller reads the file and passes the parsed
dict.  Required schema fields have no defaults; anything missing, malformed,
or from an unsuccessful survey is a typed rejection
(:class:`CourtModelBuildError`), never a silent default.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    PlannerInputError,
    PolygonObstacle,
)
from tennis_robot.collection_route_types import DomainValidationError, Point2D

# Only this exact successful survey artifact is accepted.
REQUIRED_SCHEMA = "court_knowledge_model/v2"
MAP_FRAME = "map"
SUCCESS_STATUS = "OK"

# Thin-wall thicknesses.  Small explicit constants, documented above: the
# fence walls exist for their tangent headings, not their (redundant) keepout,
# so the thickness only needs to be strictly positive and safely below the
# footprint clearance so it never adds keepout beyond the navigable exterior.
FENCE_WALL_THICKNESS_M = 0.05  # offset outward from each fence edge
NET_WALL_THICKNESS_M = 0.04  # total, centred on the post-to-post net segment

# Artifact obstacle ``class`` -> planner obstacle ``kind``.  Only net/fence
# kinds produce tangent headings, so every interior fixture maps to a
# no-tangent static-keepout kind.
_CLASS_TO_KIND = {
    "perimeter_fixture": "bench",
}
_DEFAULT_OBSTACLE_KIND = "other"


class CourtModelBuildError(ValueError):
    """The court_boundary dict cannot yield a valid planner ``CourtModel``."""


def build_court_model(boundary: Mapping[str, Any]) -> CourtModel:
    """Translate a parsed ``court_boundary.json`` v2 dict into a ``CourtModel``.

    Pure: no file I/O, no ROS.  Raises :class:`CourtModelBuildError` on any
    missing/invalid required field or unsuccessful survey; never returns a
    partially-defaulted model.
    """
    if not isinstance(boundary, Mapping):
        raise CourtModelBuildError("court_boundary must be a mapping")

    _require_successful_survey(boundary)

    fence_corners = _fence_corners(boundary)
    navigable_polygon = tuple(fence_corners)

    obstacles: list[PolygonObstacle] = []
    seen_ids: set[str] = set()

    for obstacle in _fence_wall_obstacles(fence_corners):
        _add_obstacle(obstacles, seen_ids, obstacle)
    _add_obstacle(obstacles, seen_ids, _net_wall_obstacle(boundary))
    for obstacle in _interior_obstacles(boundary):
        _add_obstacle(obstacles, seen_ids, obstacle)

    try:
        return CourtModel(navigable_polygon, tuple(obstacles))
    except (PlannerInputError, DomainValidationError) as exc:
        raise CourtModelBuildError(f"invalid court geometry: {exc}") from exc


# ── survey validity ─────────────────────────────────────────────────────────
def _require_successful_survey(boundary: Mapping[str, Any]) -> None:
    schema = boundary.get("schema")
    if schema != REQUIRED_SCHEMA:
        raise CourtModelBuildError(
            f"schema must be {REQUIRED_SCHEMA!r}, got {schema!r}"
        )
    frame = boundary.get("frame")
    if frame != MAP_FRAME:
        raise CourtModelBuildError(f"frame must be {MAP_FRAME!r}, got {frame!r}")
    if boundary.get("status") != SUCCESS_STATUS:
        raise CourtModelBuildError(
            f"survey status must be {SUCCESS_STATUS!r}, got {boundary.get('status')!r}"
        )
    if boundary.get("completed") is not True:
        raise CourtModelBuildError("survey must be completed")


# ── fence ────────────────────────────────────────────────────────────────────
def _fence_corners(boundary: Mapping[str, Any]) -> list[Point2D]:
    fence = boundary.get("fence")
    if not isinstance(fence, Mapping):
        raise CourtModelBuildError("fence is required")
    corners = fence.get("corners")
    if not isinstance(corners, list) or len(corners) < 3:
        raise CourtModelBuildError("fence.corners must list at least three corners")
    return [_point(corner, "fence.corners[]") for corner in corners]


def _fence_wall_obstacles(corners: list[Point2D]) -> list[PolygonObstacle]:
    centroid_x = sum(corner.x_m for corner in corners) / len(corners)
    centroid_y = sum(corner.y_m for corner in corners) / len(corners)
    walls: list[PolygonObstacle] = []
    count = len(corners)
    for index in range(count):
        start = corners[index]
        end = corners[(index + 1) % count]
        outward = _outward_unit_normal(start, end, centroid_x, centroid_y)
        # Inner long edge = the fence edge itself; body offset outward so the
        # wall never overlaps the navigable interior.
        polygon = (
            start,
            end,
            Point2D(
                end.x_m + FENCE_WALL_THICKNESS_M * outward[0],
                end.y_m + FENCE_WALL_THICKNESS_M * outward[1],
            ),
            Point2D(
                start.x_m + FENCE_WALL_THICKNESS_M * outward[0],
                start.y_m + FENCE_WALL_THICKNESS_M * outward[1],
            ),
        )
        walls.append(PolygonObstacle(f"fence_wall_{index}", "fence", polygon))
    return walls


def _outward_unit_normal(
    start: Point2D, end: Point2D, centroid_x: float, centroid_y: float
) -> tuple[float, float]:
    dx, dy = end.x_m - start.x_m, end.y_m - start.y_m
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise CourtModelBuildError("fence edge has zero length")
    normal_x, normal_y = -dy / length, dx / length
    mid_x, mid_y = (start.x_m + end.x_m) / 2.0, (start.y_m + end.y_m) / 2.0
    if (mid_x - centroid_x) * normal_x + (mid_y - centroid_y) * normal_y < 0.0:
        normal_x, normal_y = -normal_x, -normal_y
    return normal_x, normal_y


# ── net ──────────────────────────────────────────────────────────────────────
def _net_wall_obstacle(boundary: Mapping[str, Any]) -> PolygonObstacle:
    net = boundary.get("net")
    if not isinstance(net, Mapping):
        raise CourtModelBuildError("net is required")
    if "center" not in net:
        raise CourtModelBuildError("net.center is required")
    _point(net["center"], "net.center")  # validated presence/shape; posts define the span
    posts = net.get("posts")
    if not isinstance(posts, list) or len(posts) != 2:
        raise CourtModelBuildError("net.posts must list exactly two posts")
    first, second = (_point(post, "net.posts[]") for post in posts)

    dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise CourtModelBuildError("net posts must be distinct")
    half = NET_WALL_THICKNESS_M / 2.0
    normal_x, normal_y = -dy / length * half, dx / length * half
    polygon = (
        Point2D(first.x_m + normal_x, first.y_m + normal_y),
        Point2D(second.x_m + normal_x, second.y_m + normal_y),
        Point2D(second.x_m - normal_x, second.y_m - normal_y),
        Point2D(first.x_m - normal_x, first.y_m - normal_y),
    )
    return PolygonObstacle("net", "net", polygon)


# ── interior obstacles ───────────────────────────────────────────────────────
def _interior_obstacles(boundary: Mapping[str, Any]) -> list[PolygonObstacle]:
    raw = boundary.get("obstacles", [])
    if not isinstance(raw, list):
        raise CourtModelBuildError("obstacles must be a JSON array")
    obstacles: list[PolygonObstacle] = []
    for entry in _sorted_obstacles(raw):
        obstacles.append(_interior_obstacle(entry))
    return obstacles


def _sorted_obstacles(raw: list[Any]) -> list[Mapping[str, Any]]:
    for entry in raw:
        if not isinstance(entry, Mapping) or "id" not in entry:
            raise CourtModelBuildError("each obstacle must be a mapping with an id")
    # Stable, deterministic order independent of the artifact's array order.
    return sorted(raw, key=lambda entry: str(entry["id"]))


def _interior_obstacle(entry: Mapping[str, Any]) -> PolygonObstacle:
    center = _point(entry.get("center"), "obstacle.center")
    size = entry.get("size_m")
    if not isinstance(size, Mapping):
        raise CourtModelBuildError("obstacle.size_m is required")
    width = _number(size.get("w"), "obstacle.size_m.w")
    height = _number(size.get("h"), "obstacle.size_m.h")
    if width <= 0.0 or height <= 0.0:
        raise CourtModelBuildError("obstacle.size_m.w/h must be positive")
    half_w, half_h = width / 2.0, height / 2.0
    polygon = (
        Point2D(center.x_m - half_w, center.y_m - half_h),
        Point2D(center.x_m + half_w, center.y_m - half_h),
        Point2D(center.x_m + half_w, center.y_m + half_h),
        Point2D(center.x_m - half_w, center.y_m + half_h),
    )
    kind = _CLASS_TO_KIND.get(entry.get("class"), _DEFAULT_OBSTACLE_KIND)
    return PolygonObstacle(f"obstacle_{entry['id']}", kind, polygon)


# ── helpers ──────────────────────────────────────────────────────────────────
def _add_obstacle(
    obstacles: list[PolygonObstacle], seen_ids: set[str], obstacle: PolygonObstacle
) -> None:
    if obstacle.obstacle_id in seen_ids:
        raise CourtModelBuildError(f"duplicate obstacle id {obstacle.obstacle_id!r}")
    seen_ids.add(obstacle.obstacle_id)
    obstacles.append(obstacle)


def _point(data: Any, name: str) -> Point2D:
    if not isinstance(data, Mapping):
        raise CourtModelBuildError(f"{name} must be a mapping with x_m/y_m")
    return Point2D(_number(data.get("x_m"), f"{name}.x_m"), _number(data.get("y_m"), f"{name}.y_m"))


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CourtModelBuildError(f"{name} must be a finite number")
    return float(value)
