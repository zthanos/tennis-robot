"""Builds and writes controller status/sensor snapshots to runtime JSON files.

Stable device and behavior references are injected at construction.
Per-tick state (changes every frame) is passed explicitly to write_status()
and write_sensor_snapshots().
"""

from __future__ import annotations

import math
import time as _time
from typing import TYPE_CHECKING, Any

from debug_display import (
    bgra_bmp_data_url,
    draw_recognition_overlay as _draw_recognition_overlay_frame,
    survey_recognized_object as _survey_recognized_object_fn,
)
from lidar_processor import front_range_m as lidar_front_range_m

if TYPE_CHECKING:
    from ball_map import BallMap
    from collect_one_mission import CollectOneMission
    from collector import BallObservationInput, ConceptACollectorBehavior, ConceptACommand
    from control_bus import RobotSensorStore, RobotStatusStore
    from mapping import MapLeftSideMission
    from route_visualizer import WebotsRouteVisualizer
    from search import HalfCourtSearchBehavior
    from survey import CourtSurveyBehavior, SurveyVision
    from telemetry import SimulationTelemetry, TelemetryJournal

try:
    import cv2
    import numpy as np
    _CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    _CV2_AVAILABLE = False

try:
    from route_visualizer import (
        ROUTE_NET_CLEARANCE_X_M,
        ROUTE_PLANNER_AVAILABLE,
        RouteBall,
        RouteObstacle,
        RoutePoint,
        RouteScenario,
        route_half_bounds,
        route_plan_route,
    )
except ImportError:
    ROUTE_PLANNER_AVAILABLE = False
    ROUTE_NET_CLEARANCE_X_M = 0.0
    RouteBall = RouteObstacle = RoutePoint = RouteScenario = None  # type: ignore
    route_half_bounds = route_plan_route = None  # type: ignore

_VISION_ENABLED = False
try:
    import cv2 as _cv2_check  # noqa: F401
    _VISION_ENABLED = True
except ImportError:
    pass

_NET_X_M = 0.0
_SUPERVISED_FOV_RAD = 1.05
_MAPPED_BALL_MAX_CREATE_DISTANCE_M = 3.0


class StatusReporter:
    """Serialises controller state to the runtime status and sensor files."""

    def __init__(
        self,
        *,
        status_store: RobotStatusStore,
        sensor_store: RobotSensorStore,
        robot_node: Any,
        robot: Any,
        camera: Any,
        depth_camera: Any,
        lidar: Any,
        ir_intake_left: Any,
        ir_intake_right: Any,
        started_at: float,
        survey_behavior: CourtSurveyBehavior,
        search_behavior: HalfCourtSearchBehavior,
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        collect_one_mission: CollectOneMission,
        map_mission: MapLeftSideMission,
        route_visualizer: WebotsRouteVisualizer,
        telemetry: SimulationTelemetry,
        telemetry_journal: TelemetryJournal,
        lidar_front_index_ratio: float = 0.5,
        lidar_front_min_obstacle_range_m: float = 0.18,
        ir_intake_trigger_threshold: float = 500.0,
    ) -> None:
        self._status_store = status_store
        self._sensor_store = sensor_store
        self._robot_node = robot_node
        self._robot = robot
        self._camera = camera
        self._depth_camera = depth_camera
        self._lidar = lidar
        self._ir_left = ir_intake_left
        self._ir_right = ir_intake_right
        self._started_at = started_at
        self._survey = survey_behavior
        self._search = search_behavior
        self._behavior = behavior
        self._ball_map = ball_map
        self._collect_one = collect_one_mission
        self._map_mission = map_mission
        self._route_vis = route_visualizer
        self._telemetry = telemetry
        self._journal = telemetry_journal
        self._lidar_front_index_ratio = lidar_front_index_ratio
        self._lidar_front_min_obstacle_range_m = lidar_front_min_obstacle_range_m
        self._ir_threshold = ir_intake_trigger_threshold
        self._last_status_write_s: float = 0.0
        self._last_sensor_write_s: float = 0.0

    # ── Public write methods ───────────────────────────────────────────────────

    def write_status(
        self,
        *,
        requested_mode: str,
        control_mode: str,
        command: ConceptACommand,
        observation: BallObservationInput,
        detection: Any,
        inventory: dict,
        collection_count: int,
        loop_count: int,
        collection_confirmed: bool,
        collection_complete_reported: bool,
        collect_pattern_phase: str,
        collect_pattern_failures: int,
        collect_pattern_collect_elapsed_s: float,
        scan_side_started_at: float | None,
        active_mapped_target_id: int | None,
        collection_animation: dict | None,
        last_survey_vision: SurveyVision | None,
        last_debug_detection: Any,
        last_debug_observation: Any,
        collect_pattern_collection_timeout_s: float,
        scan_side_duration_s: float,
        vision_enabled: bool,
    ) -> None:
        now = _time.time()
        if now - self._last_status_write_s < 0.2 and not collection_confirmed:
            return
        self._last_status_write_s = now

        x_m, y_m, z_m = self._robot_node.getPosition()
        robot_yaw = self._robot_yaw()
        front_range = self._front_range_m()
        search_snapshot = self._search.snapshot(x_m, y_m)

        if control_mode == "search":
            current_action = search_snapshot["search_state"]
        elif control_mode == "collect_pattern":
            current_action = f"collect_pattern:{collect_pattern_phase}"
        else:
            current_action = command.state.value

        recognized_objects = self._recognized_objects(observation, detection, control_mode, last_survey_vision)

        self._status_store.write({
            "requested_mode": requested_mode,
            "actual_mode": control_mode,
            "collector_state": command.state.value,
            "mission_name": "Collect All Balls",
            "mission_elapsed_s": now - self._started_at,
            "current_action": current_action,
            "current_zone": search_snapshot["zone_id"],
            "current_target_id": active_mapped_target_id,
            "coverage_pct": search_snapshot["coverage_pct"],
            "balls_collected": collection_count,
            "loop_count": loop_count,
            "uptime_s": now - self._started_at,
            "telemetry_enabled": self._telemetry.enabled,
            "vision_enabled": vision_enabled,
            "route_visualization_enabled": self._route_vis.enabled,
            "diagnostics": self._diagnostics(inventory, search_snapshot, collect_pattern_failures, vision_enabled),
            "timeline_events": self._journal.snapshot(),
            "completion": {
                "current_side_complete": inventory["same_side_remaining"] == 0,
                "reported": collection_complete_reported,
            },
            "balls": inventory,
            "map": self._route_snapshot(x_m, y_m, robot_yaw, active_mapped_target_id, now),
            "robot": {"x_m": x_m, "y_m": y_m, "z_m": z_m, "yaw_rad": robot_yaw},
            "sensor_mounts": self._sensor_mounts(),
            "observation": {
                "visible": observation.visible,
                "distance_m": None if math.isinf(observation.distance_m) else observation.distance_m,
                "bearing_rad": observation.bearing_rad,
                "bearing_deg": math.degrees(observation.bearing_rad),
                "confidence": observation.confidence,
                "source": observation.source,
                "robot_x_m": observation.robot_x_m,
                "robot_y_m": observation.robot_y_m,
                "world_x_m": observation.world_x_m,
                "world_y_m": observation.world_y_m,
            },
            "oak_depth": self._oak_depth_status(observation),
            "detection": None if detection is None else {
                "x": detection.x, "y": detection.y,
                "width": detection.width, "height": detection.height,
                "area_px": detection.area_px,
                "center_x": detection.center_x, "center_y": detection.center_y,
            },
            "recognized_objects": recognized_objects,
            "command": {
                "linear_speed_m_s": command.base.linear_speed_m_s,
                "angular_speed_rad_s": command.base.angular_speed_rad_s,
                "lift_wheel_speed": command.collector.lift_wheel_speed,
                "intake_enabled": command.collector.intake_enabled,
            },
            "scan": self._scan_status(),
            "target_lock": {
                "mapped_ball_id": active_mapped_target_id,
                "locked": active_mapped_target_id is not None,
            },
            "collect_one": {
                "phase": self._collect_one.phase,
                "start_pose": None if self._collect_one._start_pose is None else {
                    "x_m": self._collect_one._start_pose[0],
                    "y_m": self._collect_one._start_pose[1],
                    "yaw_rad": self._collect_one._start_pose[2],
                },
                "scan_target_yaw_rad": self._collect_one._scan_target_yaw,
                "complete_reported": self._collect_one._complete_reported,
            },
            "collect_pattern": {
                "phase": collect_pattern_phase,
                "collect_elapsed_s": collect_pattern_collect_elapsed_s,
                "failures": collect_pattern_failures,
                "timeout_s": collect_pattern_collection_timeout_s,
                "active_target_id": active_mapped_target_id,
            },
            "scan_side": {
                "active": scan_side_started_at is not None,
                "elapsed_s": None if scan_side_started_at is None else now - scan_side_started_at,
                "duration_s": scan_side_duration_s,
                "progress": 0.0 if scan_side_started_at is None
                    else min(1.0, (now - scan_side_started_at) / scan_side_duration_s),
            },
            "search": search_snapshot,
            "survey": self._survey_status(front_range, last_survey_vision),
            "map_mission": self._map_mission.telemetry(),
            "collection_animation_active": collection_animation is not None,
        })

    def write_sensor_snapshots(
        self,
        *,
        lidar_ranges: list[float] | None,
        ir_left: float | None,
        ir_right: float | None,
        last_debug_detection: Any,
        last_debug_observation: Any,
        control_mode: str,
        last_survey_vision: SurveyVision | None,
    ) -> None:
        now = _time.time()
        if now - self._last_sensor_write_s < 1.0:
            return
        self._last_sensor_write_s = now
        from lidar_processor import extract_ball_candidates
        candidates = extract_ball_candidates(lidar_ranges or [])
        self._sensor_store.write({
            "front_camera": self._camera_snapshot(last_debug_detection, last_debug_observation, control_mode, last_survey_vision),
            "front_depth": self._depth_snapshot(),
            "front_lidar": self._lidar_snapshot(lidar_ranges),
            "lidar_candidates": [
                {"robot_x_m": cx, "robot_y_m": cy, "distance_m": math.hypot(cx, cy)}
                for cx, cy in candidates
            ],
            "ir_intake": {
                "left": ir_left,
                "right": ir_right,
                "threshold": self._ir_threshold,
                "triggered": self._ir_triggered(ir_left, ir_right),
                "left_available": self._ir_left is not None,
                "right_available": self._ir_right is not None,
            },
        })

    def print_status(
        self,
        *,
        control_mode: str,
        command: ConceptACommand,
        observation: BallObservationInput,
        collection_count: int,
        collect_pattern_phase: str,
    ) -> None:
        status = (
            f"mode={control_mode} "
            f"collector={command.state.value} "
            f"visible={observation.visible} "
            f"distance={observation.distance_m:.2f}m "
            f"bearing={math.degrees(observation.bearing_rad):.1f}deg "
            f"range_source={observation.source} "
            f"balls={collection_count}"
        )
        if observation.world_x_m is not None and observation.world_y_m is not None:
            status += f" ball_world=({observation.world_x_m:.2f},{observation.world_y_m:.2f})"

        if control_mode == "map_court":
            nav = self._survey.telemetry()
            def _fmt(v): return "none" if v is None else f"{v:.2f}m"
            line_offset = nav["line_offset_m"]
            status += (
                f" survey_state={self._survey.state.value} "
                f"event={nav['last_event']} "
                f"samples={self._survey.sample_count} "
                f"sensor_only={nav['sensor_only_navigation']} "
                f"path_driver={nav['path_driver']} "
                f"line_detected={nav['line_detected']} "
                f"line_offset={'none' if line_offset is None else f'{line_offset:.2f}m'} "
                f"line_conf={nav['line_confidence']:.2f} "
                f"corners={nav['corner_count']} "
                f"external_boundaries={nav['external_boundary_candidate_count']} "
                f"internal_objects={nav['internal_obstacle_count']} "
                f"front_lidar={_fmt(nav['front_lidar_range_m'])} "
                f"left_lidar={_fmt(nav['left_lidar_range_m'])} "
                f"right_lidar={_fmt(nav['right_lidar_range_m'])} "
                f"distance_traveled={nav['distance_traveled_m']:.2f}m "
                f"phase_distance={nav['phase_distance_m']:.2f}m "
                f"turn={nav['turn_progress_deg']:.1f}/{nav['turn_target_deg'] or 0:.0f}deg "
                f"loop_closed={nav['loop_closed']} "
                f"bounds={'saved' if self._survey.court_bounds else 'pending'} "
                f"cmd=({command.base.linear_speed_m_s:.3f},{command.base.angular_speed_rad_s:.3f})"
            )

        if control_mode in {"search", "collect_pattern"}:
            x_m, y_m, _ = self._robot_node.getPosition()
            snap = self._search.snapshot(x_m, y_m)
            tx, ty = snap["target_x_m"], snap["target_y_m"]
            target_text = "none" if tx is None or ty is None else f"({tx:.2f},{ty:.2f})"
            status += (
                f" search_state={snap['search_state']} "
                f"zone={snap['zone_id']} "
                f"coverage={snap['coverage_pct']:.1f}% "
                f"waypoint={snap['waypoint_index'] + 1}/{snap['waypoint_count']} "
                f"target={target_text} "
                f"path={snap['path_status']}"
            )
            if control_mode == "collect_pattern":
                status += f" collect_pattern_phase={collect_pattern_phase}"
        print(status)

    # ── Internal builders ──────────────────────────────────────────────────────

    def _robot_yaw(self) -> float:
        o = self._robot_node.getOrientation()
        return math.atan2(o[3], o[0])

    def _front_range_m(self) -> float | None:
        if self._lidar is None:
            return None
        ranges = [float(v) for v in self._lidar.getRangeImage()]
        return lidar_front_range_m(ranges, self._lidar_front_index_ratio, self._lidar_front_min_obstacle_range_m)

    def _ir_triggered(self, ir_left: float | None, ir_right: float | None) -> bool:
        left = ir_left is not None and ir_left > self._ir_threshold
        right = ir_right is not None and ir_right > self._ir_threshold
        return left or right

    def _scan_status(self) -> dict:
        b = self._behavior
        return {
            "elapsed_s": b.state_elapsed_s,
            "full_turn_s": b.config.scan_full_turn_s,
            "progress": min(1.0, b.state_elapsed_s / max(0.001, b.config.scan_full_turn_s)),
            "best_visible": b.scan_best_observation is not None,
            "best_distance_m": None
                if b.scan_best_observation is None or math.isinf(b.scan_best_observation.distance_m)
                else b.scan_best_observation.distance_m,
            "best_bearing_rad": None if b.scan_best_observation is None else b.scan_best_observation.bearing_rad,
        }

    def _survey_status(self, front_range_m: float | None, last_survey_vision: SurveyVision | None) -> dict:
        sv = last_survey_vision
        return {
            "state": self._survey.state.value,
            "sample_count": self._survey.sample_count,
            "bounds_saved": self._survey.court_bounds is not None,
            "bounds": self._survey.court_bounds,
            "front_range_m": front_range_m,
            "oak_depth": None if sv is None else {
                "center_m": sv.center_m, "left_m": sv.left_m, "right_m": sv.right_m,
                "valid_count": sv.valid_count, "obstacle_class": sv.obstacle_class,
                "line_detected": sv.line_detected, "line_offset_m": sv.line_offset_m,
                "line_heading_error_rad": sv.line_heading_error_rad,
                "line_confidence": sv.line_confidence,
                "corner_detected": sv.corner_detected, "corner_confidence": sv.corner_confidence,
            },
            "navigation": self._survey.telemetry(),
        }

    def _diagnostics(
        self, inventory: dict, search_snapshot: dict, collect_pattern_failures: int, vision_enabled: bool
    ) -> dict:
        health = "ok"
        reasons: list[str] = []
        events = self._journal.snapshot()
        if not vision_enabled:
            health = "degraded"
            reasons.append("vision_disabled")
        if search_snapshot.get("path_status") == "blocked":
            health = "blocked"
            reasons.append("search_path_blocked")
        if collect_pattern_failures > 0:
            health = "degraded" if health == "ok" else health
            reasons.append("collection_failures")
        if inventory.get("same_side_remaining") == 0:
            reasons.append("same_side_complete")
        return {"health": health, "reasons": reasons, "event_count": len(events), "last_event": None if not events else events[-1]}

    def _recognized_objects(
        self, observation: Any, detection: Any, control_mode: str, last_survey_vision: Any
    ) -> list[dict]:
        objects: list[dict] = []
        if detection is not None:
            objects.append({
                "label": "ball",
                "distance_m": None if math.isinf(observation.distance_m) else observation.distance_m,
                "bearing_deg": math.degrees(observation.bearing_rad),
                "source": observation.source,
                "confidence": observation.confidence,
                "bbox": {"x": detection.x, "y": detection.y, "width": detection.width, "height": detection.height},
            })
        if control_mode == "map_court":
            objects.append(_survey_recognized_object_fn(self._survey, last_survey_vision))
        return objects

    def _sensor_mounts(self) -> dict:
        return {
            "front_lidar": self._node_pose("FRONT_LIDAR"),
            "front_camera": self._node_pose("FRONT_CAMERA"),
            "front_depth": self._node_pose("FRONT_DEPTH"),
            "collector_center": {
                "world_z_m": self._robot_node.getPosition()[2] + 0.04,
                "local_x_m": 0.61, "local_y_m": 0.0, "local_z_m": 0.04,
                "role": "low front collection confirmation",
            },
        }

    def _node_pose(self, def_name: str) -> dict | None:
        node = self._robot.getFromDef(def_name)
        if node is None:
            return None
        x_m, y_m, z_m = node.getPosition()
        roles = {
            "FRONT_LIDAR": "navigation obstacle map and path safety",
            "FRONT_CAMERA": "OAK-D RGB ball detection",
            "FRONT_DEPTH": "OAK-D depth range estimate",
        }
        return {"world_x_m": x_m, "world_y_m": y_m, "world_z_m": z_m, "role": roles.get(def_name, "unknown")}

    def _oak_depth_status(self, observation: Any) -> dict:
        dc = self._depth_camera
        depth_range_m = None if math.isinf(observation.distance_m) else observation.distance_m
        return {
            "available": _VISION_ENABLED and dc is not None,
            "used_for_current_observation": observation.source == "oak_depth",
            "range_m": depth_range_m if observation.source == "oak_depth" else None,
            "source": observation.source,
            "min_range_m": None if dc is None else float(dc.getMinRange()),
            "max_range_m": None if dc is None else float(dc.getMaxRange()),
            "role": "OAK-D stereo depth estimates ball distance; RGB detection provides image coordinates and bearing",
        }

    def _route_snapshot(self, robot_x: float, robot_y: float, robot_yaw: float, active_target_id: int | None, now: float) -> dict:
        ball_rows, same_side_mapped = self._ball_map.rows_for_route(robot_x, robot_y, robot_yaw, now)
        same_side_balls = (
            [RouteBall(x=b.x_m, y=b.y_m, id=b.id) for b in same_side_mapped]
            if ROUTE_PLANNER_AVAILABLE else []
        )
        route_points: list[dict] = []
        legs_payload: list[dict] = []
        bounds_payload = None
        metrics = None
        if ROUTE_PLANNER_AVAILABLE and same_side_balls:
            side = "left" if robot_x < _NET_X_M else "right"
            bounds = route_half_bounds(side)
            bounds_payload = {"min_x": bounds.min_x, "max_x": bounds.max_x, "min_y": bounds.min_y, "max_y": bounds.max_y}
            scenario = RouteScenario(
                seed=0, bounds=bounds, robot_start=RoutePoint(robot_x, robot_y),
                obstacles=[RouteObstacle("rect", "net", _NET_X_M, 0.0, width=ROUTE_NET_CLEARANCE_X_M * 2, height=12.0)],
                balls=same_side_balls,
            )
            legs, metrics = route_plan_route(
                scenario, area_mode="half", travel_speed_m_s=0.85, pickup_time_s=1.2,
                scan_time_s=7.0, rescan_every=5, safety_buffer_m=0.55,
                collection_margin_m=0.55, candidate_window=12, lidar_costmap=True,
            )
            route = [scenario.robot_start]
            planned_orders = {leg.ball_id: order for order, leg in enumerate(legs, start=1)}
            risks = {leg.ball_id: leg.risk for leg in legs}
            for leg in legs:
                route.extend(leg.path[1:])
                legs_payload.append({"ball_id": leg.ball_id, "distance_m": leg.distance_m,
                                     "travel_s": leg.travel_s, "mode": leg.mode, "risk": leg.risk})
            route_points = [{"x_m": p.x, "y_m": p.y} for p in route]
            for row in ball_rows:
                order = planned_orders.get(int(row["id"]))
                if order is not None:
                    row["planned"] = True
                    row["order"] = order
                    row["risk"] = risks.get(int(row["id"]))
        return {
            "planner_available": ROUTE_PLANNER_AVAILABLE,
            "source": "sensor_mapped",
            "updated_at": now,
            "court": {"min_x": -11.885, "max_x": 11.885, "min_y": -5.485, "max_y": 5.485, "net_x": _NET_X_M},
            "active_bounds": bounds_payload,
            "active_target_id": active_target_id,
            "camera_fov_rad": _SUPERVISED_FOV_RAD,
            "camera_max_range_m": _MAPPED_BALL_MAX_CREATE_DISTANCE_M,
            "lidar_fusion_enabled": self._lidar is not None,
            "lidar_front_index_ratio": self._lidar_front_index_ratio,
            "robot": {"x_m": robot_x, "y_m": robot_y, "yaw_rad": robot_yaw},
            "balls": ball_rows,
            "route": route_points,
            "legs": legs_payload,
            "metrics": None if metrics is None else {
                "balls_detected": metrics.balls_detected,
                "balls_collectable": metrics.balls_collectable,
                "balls_blocked": metrics.balls_blocked,
                "total_distance_m": metrics.total_distance_m,
                "total_time_s": metrics.total_time_s,
                "planned_replans": metrics.planned_replans,
            },
        }

    def _camera_snapshot(
        self, last_debug_detection: Any, last_debug_observation: Any, control_mode: str, last_survey_vision: Any
    ) -> dict | None:
        cam = self._camera
        if cam is None:
            return None
        width, height = cam.getWidth(), cam.getHeight()
        raw = cam.getImage()
        if raw is None:
            return None
        preview_width = min(width, 960)
        preview_height = max(1, round(height * preview_width / max(1, width)))
        data = bytes(raw)
        if _CV2_AVAILABLE and cv2 is not None and np is not None:
            frame = np.frombuffer(data, np.uint8).reshape((height, width, 4))
            frame = cv2.resize(frame, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
            bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            _draw_recognition_overlay_frame(bgr, last_debug_detection, last_debug_observation,
                                            control_mode, self._survey, last_survey_vision)
            data = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA).tobytes()
            width, height = preview_width, preview_height
        return {
            "width": width, "height": height,
            "native_width": cam.getWidth(), "native_height": cam.getHeight(),
            "format": "bgra-bmp",
            "data_url": bgra_bmp_data_url(data, width, height),
        }

    def _depth_snapshot(self) -> dict | None:
        dc = self._depth_camera
        if dc is None or not _CV2_AVAILABLE or np is None:
            return None
        width, height = dc.getWidth(), dc.getHeight()
        raw = dc.getRangeImage()
        if raw is None:
            return None
        depth = np.array(raw, dtype=np.float32).reshape((height, width))
        valid = depth[np.isfinite(depth) & (depth > 0)]
        min_range, max_range = float(dc.getMinRange()), float(dc.getMaxRange())
        span = max(0.001, max_range - min_range)
        clipped = np.where(np.isfinite(depth) & (depth > 0), depth, max_range)
        normalized = np.clip((clipped - min_range) / span, 0.0, 1.0)
        intensity = ((1.0 - normalized) * 255).astype(np.uint8)
        bgra = np.dstack((intensity, intensity, intensity, np.full_like(intensity, 255))).tobytes()
        return {
            "width": int(depth.shape[1]), "height": int(depth.shape[0]),
            "format": "depth-bmp",
            "min_range_m": min_range, "max_range_m": max_range,
            "valid_count": int(valid.size),
            "median_range_m": None if valid.size == 0 else float(np.median(valid)),
            "data_url": bgra_bmp_data_url(bgra, int(depth.shape[1]), int(depth.shape[0])),
        }

    def _lidar_snapshot(self, ranges: list[float] | None) -> dict | None:
        if self._lidar is None or not ranges:
            return None
        width = len(ranges)
        height = 64
        max_range = float(self._lidar.getMaxRange())
        min_range = float(self._lidar.getMinRange())
        span = max(0.001, max_range - min_range)
        pixels = bytearray()
        normalized_ranges = []
        ranges_m: list[float | None] = []
        for v in ranges:
            if math.isfinite(v) and v > 0:
                normalized_ranges.append(max(0.0, min(1.0, (v - min_range) / span)))
                ranges_m.append(float(v))
            else:
                normalized_ranges.append(1.0)
                ranges_m.append(None)
        for y in range(height):
            for n in normalized_ranges:
                bar = max(1, int((1.0 - n) * (height - 1)))
                pixels.extend((80, 220, 120, 255) if y >= height - bar else (18, 24, 28, 255))
        return {
            "width": width, "height": height, "format": "lidar-bmp",
            "min_range_m": min_range, "max_range_m": max_range,
            "front_range_m": lidar_front_range_m(ranges, self._lidar_front_index_ratio, self._lidar_front_min_obstacle_range_m),
            "ranges_m": ranges_m,
            "data_url": bgra_bmp_data_url(bytes(pixels), width, height),
        }
