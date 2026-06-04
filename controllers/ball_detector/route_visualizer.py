"""Webots scene overlay that draws the planned ball-collection route."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from controller import Supervisor

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS_DIR = _PROJECT_ROOT / "scripts"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

ROUTE_VISUALIZATION_ENABLED = os.getenv("ROUTE_VISUALIZATION", "").strip().lower() in {"1", "true", "yes", "on"}
ROUTE_VISUALIZATION_PRESET = os.getenv("ROUTE_VISUALIZATION_PRESET", "thorough").strip().lower()

try:
    from route_benchmark import (
        NET_CLEARANCE_X_M as ROUTE_NET_CLEARANCE_X_M,
        Ball as RouteBall,
        Obstacle as RouteObstacle,
        Point as RoutePoint,
        Scenario as RouteScenario,
        ball_risk as route_ball_risk,
        half_bounds as route_half_bounds,
        plan_route as route_plan_route,
    )
    ROUTE_PLANNER_AVAILABLE = True
    ROUTE_PLANNER_IMPORT_ERROR: Exception | None = None
except ImportError as exc:
    ROUTE_PLANNER_AVAILABLE = False
    ROUTE_PLANNER_IMPORT_ERROR = exc
    RouteBall = RouteObstacle = RoutePoint = RouteScenario = None  # type: ignore[assignment,misc]
    route_ball_risk = route_half_bounds = route_plan_route = None  # type: ignore[assignment]
    ROUTE_NET_CLEARANCE_X_M = 0.0

NET_X_M = 0.0


class WebotsRouteVisualizer:
    """Draw a lightweight scan-first route overlay in the Webots scene."""

    def __init__(self, supervisor: Supervisor, robot_node, preset: str) -> None:
        self.supervisor = supervisor
        self.robot_node = robot_node
        self.preset = preset if preset in {"fast", "thorough"} else "thorough"
        self.enabled = ROUTE_VISUALIZATION_ENABLED and ROUTE_PLANNER_AVAILABLE
        self._defs: list[str] = []
        if ROUTE_VISUALIZATION_ENABLED and not ROUTE_PLANNER_AVAILABLE:
            print(f"route visualization disabled: {ROUTE_PLANNER_IMPORT_ERROR}")

    def refresh(self) -> None:
        if not self.enabled:
            return
        self.clear()
        scenario = self._scenario_from_world()
        if scenario is None:
            return
        legs, _metrics = route_plan_route(
            scenario,
            area_mode="half",
            travel_speed_m_s=0.85,
            pickup_time_s=1.2,
            scan_time_s=7.0,
            rescan_every=5,
            safety_buffer_m=0.55,
            collection_margin_m=0.55,
            candidate_window=12,
            lidar_costmap=True,
        )
        if not legs:
            return

        route_points = [scenario.robot_start]
        for leg in legs:
            route_points.extend(leg.path[1:])
        self._draw_route_line(route_points)
        planned_ids = {leg.ball_id for leg in legs}
        for order, leg in enumerate(legs, start=1):
            ball = next((b for b in scenario.balls if b.id == leg.ball_id), None)
            if ball is not None:
                self._draw_marker(ball.x, ball.y, order, skipped=False)
        for ball in scenario.balls:
            if ball.id not in planned_ids:
                self._draw_marker(ball.x, ball.y, ball.id, skipped=True)

    def clear(self) -> None:
        if not self.enabled:
            return
        for def_name in self._defs:
            node = self.supervisor.getFromDef(def_name)
            if node is not None:
                node.remove()
        self._defs = []

    def _scenario_from_world(self):
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        side = "left" if robot_x < NET_X_M else "right"
        bounds = route_half_bounds(side)
        balls: list = []
        for index in range(100):
            node = self.supervisor.getFromDef(f"TENNIS_BALL_{index:02d}")
            if node is None:
                continue
            x, y, _z = node.getPosition()
            ball = RouteBall(x=x, y=y, id=index)
            if not (bounds.min_x <= ball.x <= bounds.max_x and bounds.min_y <= ball.y <= bounds.max_y):
                continue
            if self.preset == "fast":
                risk = route_ball_risk(ball, self._route_obstacles(), bounds, collection_margin_m=0.55)
                if risk != "normal":
                    continue
            balls.append(ball)
        if not balls:
            return None
        return RouteScenario(
            seed=0,
            bounds=bounds,
            robot_start=RoutePoint(robot_x, robot_y),
            obstacles=self._route_obstacles(),
            balls=balls,
        )

    def _route_obstacles(self) -> list:
        return [RouteObstacle("rect", "net", NET_X_M, 0.0, width=ROUTE_NET_CLEARANCE_X_M * 2, height=12.0)]

    def _draw_route_line(self, points: list) -> None:
        if len(points) < 2:
            return
        def_name = "ROUTE_VISUAL_LINE"
        color = "0.1 0.85 0.25" if self.preset == "fast" else "0.1 0.45 1.0"
        point_text = ", ".join(f"{p.x:.3f} {p.y:.3f} 0.055" for p in points)
        coord_index = ", ".join([*(str(i) for i in range(len(points))), "-1"])
        node_text = f"""
DEF {def_name} Shape {{
  appearance PBRAppearance {{
    baseColor {color}
    emissiveColor {color}
    roughness 0.3
  }}
  geometry IndexedLineSet {{
    coord Coordinate {{
      point [ {point_text} ]
    }}
    coordIndex [ {coord_index} ]
  }}
}}
"""
        self._import_node(def_name, node_text)

    def _draw_marker(self, x_m: float, y_m: float, index: int, skipped: bool) -> None:
        def_name = f"ROUTE_VISUAL_MARKER_{index:02d}_{'SKIP' if skipped else 'PLAN'}"
        color = "0.45 0.45 0.45" if skipped else ("0.1 0.85 0.25" if self.preset == "fast" else "0.1 0.45 1.0")
        radius = 0.075 if skipped else 0.095
        node_text = f"""
DEF {def_name} Transform {{
  translation {x_m:.3f} {y_m:.3f} 0.095
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {color}
        emissiveColor {color}
        transparency 0.15
      }}
      geometry Sphere {{
        radius {radius:.3f}
      }}
    }}
  ]
}}
"""
        self._import_node(def_name, node_text)

    def _import_node(self, def_name: str, node_text: str) -> None:
        root = self.supervisor.getRoot()
        children = root.getField("children")
        children.importMFNodeFromString(-1, node_text)
        self._defs.append(def_name)
