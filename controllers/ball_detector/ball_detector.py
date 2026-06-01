"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import base64
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from config_utils import _env_float
from collector import (
    BallObservationInput,
    BaseCommand,
    CollectorCommand,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptACommand,
    ConceptAConfig,
)
from control_bus import RobotCommandStore, RobotSensorStore, RobotStatusStore
from controller import Camera, Display, DistanceSensor, Lidar, Motor, RangeFinder, Supervisor
from search import HalfCourtSearchBehavior, SearchState
from survey import CourtSurveyBehavior, SurveyState, SurveyVision
from mapping import LidarSurveyBoundaryProvider, MapLeftSideMission
from telemetry import setup_telemetry

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

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
except ImportError as exc:
    ROUTE_PLANNER_AVAILABLE = False
    ROUTE_PLANNER_IMPORT_ERROR = exc

RGB_VISION_REQUESTED = os.getenv("USE_RGB_VISION", "").strip().lower() in {"1", "true", "yes", "on"}
RESET_COMMAND_ON_START = os.getenv("ROBOT_RESET_COMMAND_ON_START", "true").strip().lower() in {"1", "true", "yes", "on"}

try:
    if not RGB_VISION_REQUESTED:
        raise ImportError("RGB vision disabled; set USE_RGB_VISION=true to enable it")

    import cv2
    import numpy as np
    from perception import (
        BallDetection,
        BallObservation,
        CameraMount,
        RobotPose2D,
        detect_largest_ball,
        estimate_depth_ball_observation,
        observation_to_world,
    )

    VISION_ENABLED = True
except ImportError as exc:
    cv2 = None
    np = None
    VISION_ENABLED = False
    VISION_IMPORT_ERROR = exc

    if TYPE_CHECKING:
        from perception import BallDetection, BallObservation
    else:
        BallDetection = object
        BallObservation = object


TIME_STEP_MS = 32
MAX_SPEED_RAD_S = 6.28
WHEEL_RADIUS_M = 0.11
TRACK_WIDTH_M = 0.48
INTAKE_ZONE_X_M = (0.50, 0.70)
# Wide intake roller accepts a wider lateral pickup envelope than the earlier centered wheel.
INTAKE_HALF_WIDTH_M = 0.16
INTAKE_MAX_HEIGHT_M = 0.12
SUPERVISED_FOV_RAD = 1.05
SUPERVISED_MAX_RANGE_M = 8.0
NET_X_M = 0.0
NET_SIDE_CLEARANCE_M = 0.25
COURT_MAX_X_M = 11.885
COURT_MAX_Y_M = 5.485
COURT_BALL_MARGIN_M = _env_float("COURT_BALL_MARGIN_M", 3.2)
COLLECTION_ANIMATION_S = 0.75
COLLECTION_PATH_LOCAL = (
    (0.64, 0.0, 0.045),
    (0.58, 0.0, 0.11),
    (0.28, 0.0, 0.28),
    (0.12, 0.0, 0.40),
)
FRONT_CAMERA_MOUNT = CameraMount(x_m=0.42, y_m=0.0, yaw_rad=0.0) if VISION_ENABLED else None
LIDAR_FRONT_INDEX_RATIO = max(0.0, min(1.0, _env_float("LIDAR_FRONT_INDEX_RATIO", 0.5)))
LIDAR_FRONT_MIN_OBSTACLE_RANGE_M = _env_float("LIDAR_FRONT_MIN_OBSTACLE_RANGE_M", 0.18)
MAPPED_BALL_MERGE_DISTANCE_M = 0.65
MAPPED_BALL_MAX_MERGE_DISTANCE_M = 1.6
MAPPED_BALL_MIN_SEEN_COUNT = 5
# Beyond this range single-frame depth/noise can exceed the merge radius; don't create new entries.
MAPPED_BALL_MAX_CREATE_DISTANCE_M = 3.0
# collect_pattern enters collect phase only within this range (or when a confirmed mapped target exists).
# At 0.22 m/s approach speed and 18s timeout, max reachable distance is ~3-4m accounting for alignment.
COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M = _env_float(
    "COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M", MAPPED_BALL_MAX_CREATE_DISTANCE_M
)
MAPPED_BALL_STALE_AFTER_S = 45.0
MAPPED_BALL_SIM_SNAP_ENABLED = os.getenv("MAPPED_BALL_SIM_SNAP_ENABLED", "false").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
MAPPED_BALL_SIM_SNAP_BEARING_TOLERANCE_RAD = math.radians(4.0)
IR_INTAKE_TRIGGER_THRESHOLD = 500.0
COLLECT_ONE_RETURN_POSITION_TOLERANCE_M = 0.12
COLLECT_ONE_RETURN_YAW_TOLERANCE_RAD = math.radians(6.0)
COLLECT_ONE_RETURN_LINEAR_GAIN = 0.55
COLLECT_ONE_RETURN_ANGULAR_GAIN = 1.8
COLLECT_ONE_RETURN_MAX_SPEED_M_S = 0.28
COLLECT_ONE_RETURN_MAX_TURN_RAD_S = 1.0
COLLECT_ONE_SCAN_STEP_RAD = math.radians(30.0)
COLLECT_ONE_SCAN_STEP_TOLERANCE_RAD = math.radians(2.0)
COLLECT_ONE_SCAN_TURN_SPEED_RAD_S = 0.65
COLLECT_ONE_SCAN_SETTLE_S = 0.20
SCAN_SIDE_DURATION_S = 12.0
COLLECT_PATTERN_COLLECTION_TIMEOUT_S = _env_float("COLLECT_PATTERN_COLLECTION_TIMEOUT_S", 35.0)
LIDAR_CANDIDATE_CONFIDENCE = 0.15


def _angle_delta_rad(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


@dataclass
class MappedBall:
    id: int
    x_m: float
    y_m: float
    confidence: float
    first_seen_s: float
    last_seen_s: float
    source: str = "unknown"
    seen_count: int = 1
    state: str = "detected"


def _bgra_bmp_data_url(bgra: bytes, width: int, height: int) -> str:
    pixel_bytes = width * height * 4
    if len(bgra) != pixel_bytes:
        bgra = bgra[:pixel_bytes].ljust(pixel_bytes, b"\x00")
    file_size = 54 + pixel_bytes
    file_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54)
    dib_header = struct.pack(
        "<IiiHHIIiiII",
        40,
        width,
        -height,
        1,
        32,
        0,
        pixel_bytes,
        2835,
        2835,
        0,
        0,
    )
    return "data:image/bmp;base64," + base64.b64encode(file_header + dib_header + bgra).decode("ascii")


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
            ball = next((candidate for candidate in scenario.balls if candidate.id == leg.ball_id), None)
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
        balls: list[RouteBall] = []
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

    def _route_obstacles(self) -> list[RouteObstacle]:
        return [
            RouteObstacle(
                "rect",
                "net",
                NET_X_M,
                0.0,
                width=ROUTE_NET_CLEARANCE_X_M * 2,
                height=12.0,
            )
        ]

    def _draw_route_line(self, points: list[RoutePoint]) -> None:
        if len(points) < 2:
            return
        def_name = "ROUTE_VISUAL_LINE"
        color = "0.1 0.85 0.25" if self.preset == "fast" else "0.1 0.45 1.0"
        point_text = ", ".join(f"{point.x:.3f} {point.y:.3f} 0.055" for point in points)
        coord_index = ", ".join([*(str(index) for index in range(len(points))), "-1"])
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


class BallDetectorController:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.camera = self._device("front_camera", Camera)
        self.depth_camera = self._optional_device("front_depth", RangeFinder)
        self.lidar = self._optional_device("front_lidar", Lidar)
        self.display = self._device("camera_display", Display)
        self.left_motor = self._device("left_wheel_motor", Motor)
        self.right_motor = self._device("right_wheel_motor", Motor)
        self.lift_motor = self._optional_device("lift_wheel_motor", Motor)
        self.ir_intake_left = self._optional_device("ir_intake_left", DistanceSensor)
        self.ir_intake_right = self._optional_device("ir_intake_right", DistanceSensor)
        self.telemetry = setup_telemetry("ball-detector-controller")
        self.max_speed_rad_s = _env_float("ROBOT_MAX_WHEEL_SPEED_RAD_S", MAX_SPEED_RAD_S)
        self.behavior = ConceptACollectorBehavior(ConceptAConfig.from_env())
        self.search_behavior = HalfCourtSearchBehavior.from_env()
        self.survey_behavior = CourtSurveyBehavior.from_env()
        self._map_mission = MapLeftSideMission(LidarSurveyBoundaryProvider(), self._map_supervisor_balls)
        self.command_store = RobotCommandStore.from_env()
        if RESET_COMMAND_ON_START:
            self.command_store.write("idle", source="webots-startup")
        self.status_store = RobotStatusStore.from_env()
        self.sensor_store = RobotSensorStore.from_env()
        self.control_mode = "idle"
        self.robot_node = self.robot.getSelf()
        self.route_visualizer = WebotsRouteVisualizer(self.robot, self.robot_node, ROUTE_VISUALIZATION_PRESET)
        self.collection_visual_ball = self.robot.getFromDef("COLLECTOR_ANIMATION_BALL")
        self.collection_confirmed = False
        self.collection_count = 0
        self.collection_animation = None
        self._collect_start_time: float | None = None
        self.last_command: ConceptACommand | None = None
        self.loop_count = 0
        self.started_at = time.time()
        self.last_status_write_s = 0.0
        self.last_sensor_write_s = 0.0
        self.collection_complete_reported = False
        self.mapped_balls: dict[int, MappedBall] = {}
        self.next_mapped_ball_id = 1
        self.active_mapped_target_id: int | None = None
        self.collect_one_start_pose: tuple[float, float, float] | None = None
        self.collect_one_phase = "idle"
        self.collect_one_complete_reported = False
        self.collect_one_scan_target_yaw: float | None = None
        self.collect_one_scan_settle_until_s = 0.0
        self.collect_one_scan_steps_taken = 0
        self.collect_one_locked_world: tuple[float, float] | None = None
        self.scan_side_started_at: float | None = None
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s: float = 0.0
        self.collect_pattern_failures = 0
        self._map_completion_reported = False
        self._map_seeded_signature: tuple[tuple[float, float], ...] = ()
        self._survey_complete_reported = False
        self._last_survey_vision: SurveyVision | None = None

        self.camera.enable(TIME_STEP_MS)
        if self.depth_camera is not None:
            self.depth_camera.enable(TIME_STEP_MS)
        if self.lidar is not None:
            self.lidar.enable(TIME_STEP_MS)
        if self.ir_intake_left is not None:
            self.ir_intake_left.enable(TIME_STEP_MS)
        if self.ir_intake_right is not None:
            self.ir_intake_right.enable(TIME_STEP_MS)
        self.left_motor.setPosition(math.inf)
        self.right_motor.setPosition(math.inf)
        if self.lift_motor is not None:
            self.lift_motor.setPosition(math.inf)
        self.set_speed(0.0, 0.0)
        if VISION_ENABLED:
            print("ball_detector controller started with RGB vision and LiDAR")
        else:
            print(f"ball_detector controller started without RGB vision; sensor observations disabled: {VISION_IMPORT_ERROR}")
        if self.route_visualizer.enabled:
            print(f"route visualization enabled: preset={self.route_visualizer.preset}")

    def _device(self, name: str, expected_type: type):
        device = self.robot.getDevice(name)
        if not isinstance(device, expected_type):
            raise TypeError(f"Device {name!r} is not a {expected_type.__name__}")
        return device

    def _optional_device(self, name: str, expected_type: type):
        device = self.robot.getDevice(name)
        if device is None:
            return None
        if not isinstance(device, expected_type):
            raise TypeError(f"Device {name!r} is not a {expected_type.__name__}")
        return device

    def set_speed(self, left: float, right: float) -> None:
        self.left_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, left)))
        self.right_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, right)))

    def set_base_command(self, linear_speed_m_s: float, angular_speed_rad_s: float) -> None:
        left_side = (linear_speed_m_s - angular_speed_rad_s * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        right_side = (linear_speed_m_s + angular_speed_rad_s * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        # The Webots device names are historical: left_wheel_motor is mounted at y=-0.24,
        # i.e. on the robot's right side in the local frame.
        self.set_speed(right_side, left_side)

    def set_collector_command(self, lift_wheel_speed: float) -> None:
        if self.lift_motor is not None:
            self.lift_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, lift_wheel_speed * 4.0)))

    def run(self) -> None:
        while self.robot.step(TIME_STEP_MS) != -1:
            loop_start = time.perf_counter()
            with self.telemetry.start_span("simulation.step"):
                self._update_collection_animation(TIME_STEP_MS / 1000)
                image = self._camera_frame()
                self.telemetry.add_frame()
                detection = self._detect_largest_ball(image)
                if VISION_ENABLED:
                    observation = self._observation_from_detection(detection)
                else:
                    observation = self._sensor_unavailable_observation()
                mapped_observation = self._mapping_observation(observation)
                mapped_ball_id = self._update_mapped_balls(mapped_observation)
                if self.loop_count % 90 == 0:
                    self._prune_phantom_mapped_balls()
                control_command = self.command_store.read()
                inventory = self._ball_inventory()
                effective_mode = self._effective_control_mode(control_command.mode)
                control_observation = self._control_observation_for_mode(
                    effective_mode,
                    mapped_observation,
                    mapped_ball_id,
                )
                if effective_mode == "survey":
                    command = self._survey_command_for_mode(effective_mode)
                elif effective_mode == "search":
                    command = self._search_command_for_mode(
                        effective_mode,
                        self._same_side_search_observation(control_observation),
                    )
                elif effective_mode == "collect_pattern":
                    command = self._collect_pattern_command_for_mode(
                        effective_mode,
                        self._same_side_search_observation(mapped_observation),
                        mapped_ball_id,
                    )
                elif effective_mode == "map_left_side":
                    command = self._map_mission_command_for_mode(effective_mode)
                else:
                    command = self._collector_command_for_mode(effective_mode, control_observation)
                self.collection_confirmed = False
                self._draw_debug(image, detection, command)
                self._apply_command(command)
                self.telemetry.add_collector_state(command.state.value)
                self.collection_confirmed = self._check_collection(command)
                if self.collection_confirmed:
                    self._mark_nearest_mapped_ball_collected()
                    self.route_visualizer.refresh()
                self._write_status(control_command.mode, command, control_observation, detection, inventory)
                self._write_sensor_snapshots()
                self.last_command = command
                self.loop_count += 1
                if self.loop_count % 60 == 0:
                    self._print_status(command, control_observation)
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _all_mapped_balls_collected(self) -> bool:
        """True when every confirmed mapped ball has been collected and the map is non-empty."""
        if not self.mapped_balls:
            return False
        ever_confirmed = any(
            b.seen_count >= MAPPED_BALL_MIN_SEEN_COUNT
            for b in self.mapped_balls.values()
        )
        if not ever_confirmed:
            return False
        now = time.time()
        active = [
            b for b in self.mapped_balls.values()
            if b.state not in {"collected", "collection_failed"}
            and b.seen_count >= MAPPED_BALL_MIN_SEEN_COUNT
            and now - b.last_seen_s <= MAPPED_BALL_STALE_AFTER_S
        ]
        return len(active) == 0

    def _effective_control_mode(self, requested_mode: str) -> str:
        if requested_mode == "collect":
            elapsed = 0.0 if self._collect_start_time is None else time.time() - self._collect_start_time
            min_scan_time = self.behavior.config.scan_full_turn_s
            if self._all_mapped_balls_collected() and elapsed > min_scan_time and self.collection_animation is None:
                if not self.collection_complete_reported:
                    print(f"collection complete; total={self.collection_count}")
                    self.command_store.write("idle", source="webots-complete")
                    self.collection_complete_reported = True
                return "idle"
            if not self._all_mapped_balls_collected():
                self.collection_complete_reported = False
        if requested_mode != "collect_one":
            self.collect_one_complete_reported = False
        return requested_mode

    def _collector_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.search_behavior.reset()
            self.survey_behavior.reset()
            if self.control_mode == "map_left_side" and not self._map_mission.complete:
                self._map_mission.reset()
            self.control_mode = mode
            print(f"control mode changed to {self.control_mode}")
            if mode == "collect":
                self._reset_mapped_balls()
                self._collect_start_time = time.time()
                self.route_visualizer.refresh()
            elif mode == "collect_one":
                self._start_collect_one()
            elif mode == "scan_side":
                self.scan_side_started_at = None
                self._reset_collect_pattern()
            else:
                self._collect_start_time = None
                self.collect_one_start_pose = None
                self.collect_one_phase = "idle"
                self.collect_one_scan_target_yaw = None
                self.collect_one_scan_settle_until_s = 0.0
                self.scan_side_started_at = None
                self._reset_collect_pattern()
                self.route_visualizer.clear()

        if mode == "collect":
            if (
                self.behavior.state == CollectorState.SCAN
                and observation.visible
                and observation.source == "lidar_candidate"
            ):
                self.behavior.start_tracking(observation)
            return self.behavior.update(
                observation,
                TIME_STEP_MS / 1000,
                collection_confirmed=self.collection_confirmed,
            )
        if mode == "collect_one":
            return self._collect_one_command(observation)
        if mode == "scan_side":
            return self._scan_side_command()

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _search_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.search_behavior.reset()
            self.survey_behavior.reset()
            if self.control_mode == "map_left_side" and not self._map_mission.complete:
                self._map_mission.reset()
            self.control_mode = mode
            self._collect_start_time = None
            self.collect_one_start_pose = None
            self.collect_one_phase = "idle"
            self.scan_side_started_at = None
            self._reset_collect_pattern()
            self.route_visualizer.clear()
            print(f"control mode changed to {self.control_mode}")

        x_m, y_m, _z_m = self.robot_node.getPosition()
        search_command = self.search_behavior.update(
            x_m,
            y_m,
            self._robot_yaw_rad(),
            observation,
            self._front_range_m(),
            TIME_STEP_MS / 1000,
            target_id=self.active_mapped_target_id,
        )
        if search_command.state == SearchState.COMPLETE:
            self.command_store.write("idle", source="webots-search-complete")
        return ConceptACommand(
            state=CollectorState.SURVEY
            if search_command.state in {SearchState.SURVEY_VIEWPOINT, SearchState.TRANSIT_TO_ZONE, SearchState.LOCAL_SCAN}
            else CollectorState.SCAN,
            base=search_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _collect_pattern_command_for_mode(
        self,
        mode: str,
        observation: BallObservationInput,
        mapped_ball_id: int | None,
    ) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.search_behavior.reset()
            self.search_behavior.max_interrupt_distance_m = COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            self.survey_behavior.reset()
            if self.control_mode == "map_left_side" and not self._map_mission.complete:
                self._map_mission.reset()
            self.control_mode = mode
            self._reset_mapped_balls()
            self._seed_mapped_balls_from_map_mission()
            self._collect_start_time = time.time()
            self.collection_complete_reported = False
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            self.collect_pattern_failures = 0
            self.route_visualizer.refresh()
            print(f"control mode changed to {self.control_mode}")

        if self.collect_pattern_phase == "collect":
            return self._collect_pattern_collect_command(observation, mapped_ball_id)

        mapped_search_id = mapped_ball_id or self._nearest_mapped_target_id()
        search_observation = observation
        if not search_observation.visible and mapped_search_id is not None:
            search_observation = self._observation_from_mapped_target(mapped_search_id) or search_observation

        x_m, y_m, _z_m = self.robot_node.getPosition()
        search_command = self.search_behavior.update(
            x_m,
            y_m,
            self._robot_yaw_rad(),
            search_observation,
            self._front_range_m(),
            TIME_STEP_MS / 1000,
            target_id=mapped_search_id,
        )
        if search_command.state == SearchState.COMPLETE:
            if not self.collection_complete_reported:
                print(f"collect pattern complete; total={self.collection_count}")
                self.collection_complete_reported = True
            self.command_store.write("idle", source="webots-collect-pattern-complete")

        if search_command.state == SearchState.BALL_DETECTED:
            trigger_obs = search_observation if search_observation.visible else self._observation_from_mapped_target(mapped_search_id)
            has_confirmed_target = mapped_search_id is not None
            close_enough = trigger_obs is not None and trigger_obs.distance_m <= COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            if trigger_obs is not None and trigger_obs.visible and (close_enough or has_confirmed_target):
                self.collect_pattern_phase = "collect"
                self.collect_pattern_collect_elapsed_s = 0.0
                self.active_mapped_target_id = mapped_search_id
                self.behavior.reset()
                self.behavior.start_tracking(trigger_obs)
                return self.behavior.update(
                    trigger_obs,
                    TIME_STEP_MS / 1000,
                    collection_confirmed=False,
                )

        return ConceptACommand(
            state=CollectorState.SURVEY
            if search_command.state in {SearchState.SURVEY_VIEWPOINT, SearchState.TRANSIT_TO_ZONE, SearchState.LOCAL_SCAN}
            else CollectorState.SCAN,
            base=search_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _collect_pattern_collect_command(
        self,
        observation: BallObservationInput,
        mapped_ball_id: int | None,
    ) -> ConceptACommand:
        if self.collection_confirmed:
            self.behavior.reset()
            self.active_mapped_target_id = None
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        self.collect_pattern_collect_elapsed_s += TIME_STEP_MS / 1000
        if self.collect_pattern_collect_elapsed_s > COLLECT_PATTERN_COLLECTION_TIMEOUT_S:
            self.collect_pattern_failures += 1
            print(f"collect pattern target timed out; failures={self.collect_pattern_failures}")
            self.behavior.reset()
            self.active_mapped_target_id = None
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        target_observation = self._collect_pattern_target_observation(observation, mapped_ball_id)
        if self.behavior.state == CollectorState.SCAN and target_observation.visible:
            self.behavior.start_tracking(target_observation)
        cmd = self.behavior.update(
            target_observation,
            TIME_STEP_MS / 1000,
            collection_confirmed=False,
        )
        if self.behavior.gave_up:
            if self.active_mapped_target_id is not None and self.active_mapped_target_id in self.mapped_balls:
                self.mapped_balls[self.active_mapped_target_id].state = "collection_failed"
                print(
                    f"collect pattern gave up on target {self.active_mapped_target_id} "
                    f"after {self.behavior.capture_attempts} attempt(s)"
                )
            self.collect_pattern_failures += 1
            self.behavior.reset()
            self.active_mapped_target_id = None
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        return cmd

    def _collect_pattern_target_observation(
        self,
        observation: BallObservationInput,
        mapped_ball_id: int | None,
    ) -> BallObservationInput:
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self._nearest_mapped_target_id()
        if observation.visible and mapped_ball_id is not None and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self._observation_from_mapped_target(self.active_mapped_target_id)
        if locked is not None:
            return locked
        if observation.visible:
            self.active_mapped_target_id = mapped_ball_id or self.active_mapped_target_id
            return observation
        return BallObservationInput(visible=False, source="collect_pattern_no_target")

    def _same_side_search_observation(self, observation: BallObservationInput) -> BallObservationInput:
        if not observation.visible or observation.world_x_m is None:
            return observation
        robot_x_m = self.robot_node.getPosition()[0]
        if self._across_net(robot_x_m, observation.world_x_m):
            return BallObservationInput(visible=False, source="across_net_filtered")
        if (
            abs(observation.world_x_m) > COURT_MAX_X_M + COURT_BALL_MARGIN_M
            or observation.world_y_m is None
            or abs(observation.world_y_m) > COURT_MAX_Y_M + COURT_BALL_MARGIN_M
        ):
            return BallObservationInput(visible=False, source="out_of_court_filtered")
        return observation

    def _start_collect_one(self) -> None:
        self._reset_mapped_balls()
        self.collect_one_start_pose = self._robot_pose_2d()
        self.collect_one_phase = "collect"
        self.collect_one_complete_reported = False
        self.collect_one_scan_target_yaw = None
        self.collect_one_scan_settle_until_s = 0.0
        self.collect_one_scan_steps_taken = 0
        self.collect_one_locked_world = None
        self._collect_start_time = time.time()
        self.route_visualizer.clear()

    def _scan_side_command(self) -> ConceptACommand:
        now = time.time()
        if self.scan_side_started_at is None:
            self.scan_side_started_at = now
            print("scan_side: LiDAR scan started — robot stationary")
        elapsed = now - self.scan_side_started_at
        if elapsed >= SCAN_SIDE_DURATION_S:
            print(f"scan_side: complete after {elapsed:.1f}s")
            self.command_store.write("idle", source="webots-scan-complete")
            self.scan_side_started_at = None
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _collect_one_command(self, observation: BallObservationInput) -> ConceptACommand:
        if self.collect_one_start_pose is None:
            self._start_collect_one()

        if self.collect_one_phase == "collect":
            if self.collection_confirmed:
                self.behavior.reset()
                self.active_mapped_target_id = None
                self.collect_one_phase = "return"
            else:
                if self.behavior.state == CollectorState.SCAN and observation.visible:
                    self.collect_one_scan_target_yaw = None
                    self.collect_one_scan_settle_until_s = 0.0
                    self.behavior.start_tracking(observation)
                    if observation.world_x_m is not None and observation.world_y_m is not None:
                        self.collect_one_locked_world = (observation.world_x_m, observation.world_y_m)
                    return self.behavior.update(
                        observation,
                        TIME_STEP_MS / 1000,
                        collection_confirmed=False,
                    )
                if self.behavior.state == CollectorState.SCAN and not observation.visible:
                    self.collect_one_locked_world = None
                    return self._collect_one_scan_step_command()
                locked_obs = (
                    self._observation_from_world_pos(*self.collect_one_locked_world)
                    if self.collect_one_locked_world is not None
                    else None
                )
                tracking_obs = locked_obs if locked_obs is not None else observation
                return self.behavior.update(
                    tracking_obs,
                    TIME_STEP_MS / 1000,
                    collection_confirmed=False,
                )

        if self.collect_one_phase == "return":
            command = self._return_to_collect_one_start_command()
            if command is not None:
                return command
            self.collect_one_phase = "done"

        if self.collect_one_phase == "done" and not self.collect_one_complete_reported:
            print(f"collect one complete; total={self.collection_count}")
            self.command_store.write("idle", source="webots-collect-one-complete")
            self.collect_one_complete_reported = True

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _collect_one_scan_step_command(self) -> ConceptACommand:
        now = time.time()
        robot_yaw = self._robot_yaw_rad()
        if self.collect_one_scan_settle_until_s > now:
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        max_scan_steps = int(math.ceil(2 * math.pi / COLLECT_ONE_SCAN_STEP_RAD))
        if self.collect_one_scan_target_yaw is None:
            if self.collect_one_scan_steps_taken >= max_scan_steps:
                self.collect_one_phase = "done"
                return ConceptACommand(
                    state=CollectorState.SCAN,
                    base=BaseCommand(0.0, 0.0),
                    collector=CollectorCommand(0.0, False),
                )
            self.collect_one_scan_target_yaw = robot_yaw + COLLECT_ONE_SCAN_STEP_RAD

        yaw_error = _angle_delta_rad(self.collect_one_scan_target_yaw, robot_yaw)
        if abs(yaw_error) <= COLLECT_ONE_SCAN_STEP_TOLERANCE_RAD:
            self.collect_one_scan_target_yaw = None
            self.collect_one_scan_steps_taken += 1
            self.collect_one_scan_settle_until_s = now + COLLECT_ONE_SCAN_SETTLE_S
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        angular_speed = max(
            -COLLECT_ONE_SCAN_TURN_SPEED_RAD_S,
            min(COLLECT_ONE_SCAN_TURN_SPEED_RAD_S, yaw_error * COLLECT_ONE_RETURN_ANGULAR_GAIN),
        )
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, angular_speed),
            collector=CollectorCommand(0.0, False),
        )

    def _return_to_collect_one_start_command(self) -> ConceptACommand | None:
        if self.collect_one_start_pose is None:
            return None

        start_x, start_y, start_yaw = self.collect_one_start_pose
        robot_x, robot_y, robot_yaw = self._robot_pose_2d()
        dx = start_x - robot_x
        dy = start_y - robot_y
        distance_m = math.hypot(dx, dy)
        if distance_m > COLLECT_ONE_RETURN_POSITION_TOLERANCE_M:
            target_heading = math.atan2(dy, dx)
            heading_error = _angle_delta_rad(target_heading, robot_yaw)
            linear_speed = min(COLLECT_ONE_RETURN_MAX_SPEED_M_S, distance_m * COLLECT_ONE_RETURN_LINEAR_GAIN)
            if abs(heading_error) > math.radians(35.0):
                linear_speed = 0.0
            angular_speed = max(
                -COLLECT_ONE_RETURN_MAX_TURN_RAD_S,
                min(COLLECT_ONE_RETURN_MAX_TURN_RAD_S, heading_error * COLLECT_ONE_RETURN_ANGULAR_GAIN),
            )
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(linear_speed, angular_speed),
                collector=CollectorCommand(0.0, False),
            )

        yaw_error = _angle_delta_rad(start_yaw, robot_yaw)
        if abs(yaw_error) > COLLECT_ONE_RETURN_YAW_TOLERANCE_RAD:
            angular_speed = max(
                -COLLECT_ONE_RETURN_MAX_TURN_RAD_S,
                min(COLLECT_ONE_RETURN_MAX_TURN_RAD_S, yaw_error * COLLECT_ONE_RETURN_ANGULAR_GAIN),
            )
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(0.0, angular_speed),
                collector=CollectorCommand(0.0, False),
            )

        return None

    def _reset_mapped_balls(self) -> None:
        self.mapped_balls.clear()
        self.next_mapped_ball_id = 1
        self.active_mapped_target_id = None

    def _reset_collect_pattern(self) -> None:
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s = 0.0
        self.collect_pattern_failures = 0

    def _map_supervisor_balls(self) -> list[tuple[float, float]]:
        """Return (world_x, world_y) for every ball on the mission's active side."""
        side = self._map_mission.bounds.side if self._map_mission.bounds else "left"
        result: list[tuple[float, float]] = []
        for i in range(100):
            node = self.robot.getFromDef(f"TENNIS_BALL_{i:02d}")
            if node is None or self._is_collection_animation_ball(i):
                continue
            x, y, _z = node.getPosition()
            if side == "left" and x > -0.25:
                continue
            if side == "right" and x < 0.25:
                continue
            result.append((x, y))
        return result

    def _map_mission_command_for_mode(self, mode: str) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.search_behavior.reset()
            self.survey_behavior.reset()
            self._reset_collect_pattern()
            self.scan_side_started_at = None
            self.route_visualizer.clear()
            self.control_mode = mode
            self._map_completion_reported = False
            self._map_seeded_signature = ()
            robot_x, robot_y, _ = self.robot_node.getPosition()
            self._map_mission.start(robot_x, robot_y, self._robot_yaw_rad())
            print(
                f"map_left_side started; side={self._map_mission.bounds.side}"
                f" center=({self._map_mission.bounds.center_x:.1f}, 0)"
            )

        robot_x, robot_y, _ = self.robot_node.getPosition()
        command = self._map_mission.update(robot_x, robot_y, self._robot_yaw_rad(), TIME_STEP_MS / 1000)

        if self._map_mission.complete and not self._map_completion_reported:
            grid = self._map_mission.grid
            seeded = self._seed_mapped_balls_from_map_mission()
            print(
                f"map_left_side complete; candidates={len(self._map_mission.candidates)}"
                f" seeded={seeded} grid={grid}"
            )
            self.route_visualizer.refresh()
            self.command_store.write("idle", source="webots-map-complete")
            self._map_completion_reported = True

        return command

    def _seed_mapped_balls_from_map_mission(self) -> int:
        if not self._map_mission.complete or not self._map_mission.candidates:
            return 0
        signature = tuple(
            sorted((round(candidate.x_m, 2), round(candidate.y_m, 2)) for candidate in self._map_mission.candidates)
        )
        if signature == self._map_seeded_signature and self.mapped_balls:
            return 0

        self._reset_mapped_balls()
        now = time.time()
        for candidate in self._map_mission.candidates:
            ball_id = self.next_mapped_ball_id
            self.next_mapped_ball_id += 1
            self.mapped_balls[ball_id] = MappedBall(
                id=ball_id,
                x_m=candidate.x_m,
                y_m=candidate.y_m,
                confidence=0.95,
                first_seen_s=now,
                last_seen_s=now,
                source="map_mission",
                seen_count=MAPPED_BALL_MIN_SEEN_COUNT,
            )
        self._map_seeded_signature = signature
        self.active_mapped_target_id = self._nearest_mapped_target_id()
        return len(self.mapped_balls)

    def _survey_command_for_mode(self, mode: str) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.search_behavior.reset()
            self.survey_behavior.reset()
            self._survey_complete_reported = False
            if self.control_mode == "map_left_side" and not self._map_mission.complete:
                self._map_mission.reset()
            self.control_mode = mode
            self._reset_collect_pattern()
            print(f"control mode changed to {self.control_mode}")

        x_m, y_m, _z_m = self.robot_node.getPosition()
        self._last_survey_vision = self._survey_vision_summary()
        survey_command = self.survey_behavior.update(
            x_m,
            y_m,
            self._robot_yaw_rad(),
            self._lidar_ranges(),
            TIME_STEP_MS / 1000,
            self._last_survey_vision,
        )
        if survey_command.state == SurveyState.DONE:
            if not self._survey_complete_reported:
                bounds = self.survey_behavior.court_bounds or {}
                pt_count = bounds.get("point_count", 0)
                if pt_count >= 500:
                    print(
                        f"survey complete — "
                        f"W={bounds.get('west_fence_x', '?'):.2f}  "
                        f"E={bounds.get('east_fence_x', '?'):.2f}  "
                        f"S={bounds.get('south_fence_y', '?'):.2f}  "
                        f"N={bounds.get('north_fence_y', '?'):.2f}  "
                        f"pts={pt_count}"
                    )
                else:
                    print(
                        f"survey complete (fallback dims — only {pt_count} pts accumulated)"
                    )
                self.command_store.write("idle", source="webots-survey-complete")
                self._survey_complete_reported = True
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        return ConceptACommand(
            state=CollectorState.SURVEY,
            base=survey_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _camera_frame(self) -> np.ndarray:
        if not VISION_ENABLED:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        raw = self.camera.getImage()
        frame = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _detect_largest_ball(self, frame: np.ndarray) -> BallDetection | None:
        if not VISION_ENABLED or frame is None:
            return None
        return detect_largest_ball(frame)

    def _observation_from_detection(
        self,
        detection: BallDetection | None,
    ) -> BallObservationInput:
        if detection is None:
            return BallObservationInput(visible=False)

        observation = self._estimate_observation(detection)
        if observation is None:
            return BallObservationInput(visible=False, source="oak_depth_unavailable")
        world_observation = observation_to_world(
            observation,
            RobotPose2D(*self._robot_pose_2d()),
            FRONT_CAMERA_MOUNT,
        )
        self.telemetry.add_detection(
            detection.area_px,
            observation.distance_m,
            observation.bearing_rad,
            observation.distance_source,
        )
        return BallObservationInput(
            visible=True,
            bearing_rad=observation.bearing_rad,
            distance_m=observation.distance_m,
            confidence=min(1.0, detection.area_px / 6000),
            source=observation.distance_source,
            robot_x_m=world_observation.robot_x_m,
            robot_y_m=world_observation.robot_y_m,
            world_x_m=world_observation.world_x_m,
            world_y_m=world_observation.world_y_m,
        )

    def _estimate_observation(
        self,
        detection: BallDetection,
    ) -> BallObservation | None:
        depth_frame = self._depth_frame_m()
        if depth_frame is None:
            return None
        return estimate_depth_ball_observation(
            detection,
            depth_frame,
            self.camera.getWidth(),
            self.camera.getHeight(),
            self.camera.getFov(),
        )

    def _sensor_unavailable_observation(self) -> BallObservationInput:
        return BallObservationInput(visible=False, source="sensor_unavailable")

    def _depth_frame_m(self) -> np.ndarray | None:
        if not VISION_ENABLED or self.depth_camera is None:
            return None
        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        raw = self.depth_camera.getRangeImage()
        if raw is None:
            return None
        return np.array(raw, dtype=np.float32).reshape((height, width))

    def _survey_vision_summary(self) -> SurveyVision | None:
        """OAK-D forward clearance used only by survey corridor navigation."""
        depth = self._depth_frame_m()
        if depth is None or self.depth_camera is None:
            return None
        h, w = depth.shape[:2]
        if h < 4 or w < 6:
            return None
        y0 = int(h * 0.28)
        y1 = int(h * 0.78)
        min_range = float(self.depth_camera.getMinRange())
        max_range = float(self.depth_camera.getMaxRange())

        def sector(x0: int, x1: int) -> tuple[float | None, int]:
            roi = depth[y0:y1, max(0, x0):min(w, x1)]
            valid = roi[np.isfinite(roi) & (roi >= min_range) & (roi <= max_range)]
            if valid.size == 0:
                return None, 0
            return float(np.percentile(valid, 20)), int(valid.size)

        left, left_n = sector(0, w // 3)
        center, center_n = sector(w // 3, (2 * w) // 3)
        right, right_n = sector((2 * w) // 3, w)
        return SurveyVision(
            center_m=center,
            left_m=left,
            right_m=right,
            valid_count=left_n + center_n + right_n,
        )

    def _mapping_observation(self, observation: BallObservationInput) -> BallObservationInput:
        if not MAPPED_BALL_SIM_SNAP_ENABLED or not observation.visible:
            return observation
        snapped = self._snap_observation_to_sim_ball(observation)
        if snapped is not None:
            return snapped
        if VISION_ENABLED:
            return BallObservationInput(visible=False)
        return observation

    def _snap_observation_to_sim_ball(self, observation: BallObservationInput) -> BallObservationInput | None:
        robot_world_x = self.robot_node.getPosition()[0]
        best: tuple[float, float, float, float, float, float, float] | None = None
        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None or self._is_collection_animation_ball(index):
                continue
            ball_position = ball.getPosition()
            if self._across_net(robot_world_x, ball_position[0]):
                continue
            x, y, _z = self._world_to_robot_local(ball_position)
            if x <= 0:
                continue
            distance_m = math.hypot(x, y)
            bearing_rad = math.atan2(y, x)
            bearing_error = abs(_angle_delta_rad(bearing_rad, observation.bearing_rad))
            if bearing_error > MAPPED_BALL_SIM_SNAP_BEARING_TOLERANCE_RAD:
                continue
            if best is None or bearing_error < best[0] or (
                math.isclose(bearing_error, best[0]) and distance_m < best[1]
            ):
                best = (bearing_error, distance_m, bearing_rad, x, y, ball_position[0], ball_position[1])

        if best is None:
            return None
        _bearing_error, distance_m, bearing_rad, robot_x_m, robot_y_m, world_x_m, world_y_m = best
        return BallObservationInput(
            visible=True,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
            confidence=max(observation.confidence, 0.5),
            source="sim_snap",
            robot_x_m=robot_x_m,
            robot_y_m=robot_y_m,
            world_x_m=world_x_m,
            world_y_m=world_y_m,
        )

    def _across_net(self, robot_x_m: float, ball_x_m: float) -> bool:
        if abs(robot_x_m - NET_X_M) < NET_SIDE_CLEARANCE_M:
            return False
        if abs(ball_x_m - NET_X_M) < NET_SIDE_CLEARANCE_M:
            return False
        return (robot_x_m - NET_X_M) * (ball_x_m - NET_X_M) < 0

    def _ball_inventory(self) -> dict[str, float | int | None]:
        total_remaining = 0
        same_side_remaining = 0
        across_net_remaining = 0
        visible_candidates = 0
        nearest_same_side_distance_m: float | None = None
        robot_world_x = self.robot_node.getPosition()[0]

        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None or self._is_collection_animation_ball(index):
                continue

            total_remaining += 1
            ball_position = ball.getPosition()
            if self._across_net(robot_world_x, ball_position[0]):
                across_net_remaining += 1
                continue

            same_side_remaining += 1
            x, y, _z = self._world_to_robot_local(ball_position)
            distance_m = math.hypot(x, y)
            if nearest_same_side_distance_m is None or distance_m < nearest_same_side_distance_m:
                nearest_same_side_distance_m = distance_m
            if x > 0 and abs(math.atan2(y, x)) <= SUPERVISED_FOV_RAD / 2 and distance_m <= SUPERVISED_MAX_RANGE_M:
                visible_candidates += 1

        return {
            "total_remaining": total_remaining,
            "same_side_remaining": same_side_remaining,
            "across_net_remaining": across_net_remaining,
            "visible_candidates": visible_candidates,
            "nearest_same_side_distance_m": nearest_same_side_distance_m,
        }

    def _update_mapped_balls(self, observation: BallObservationInput) -> int | None:
        if not observation.visible or observation.world_x_m is None or observation.world_y_m is None:
            return None
        if (
            abs(observation.world_x_m) > COURT_MAX_X_M + COURT_BALL_MARGIN_M
            or abs(observation.world_y_m) > COURT_MAX_Y_M + COURT_BALL_MARGIN_M
        ):
            return None
        now = time.time()
        best_id: int | None = None
        merge_distance_m = self._mapped_ball_merge_distance(observation)
        best_distance = merge_distance_m
        for ball_id, ball in self.mapped_balls.items():
            if ball.state == "collected":
                continue
            distance = math.hypot(ball.x_m - observation.world_x_m, ball.y_m - observation.world_y_m)
            if distance < best_distance:
                best_id = ball_id
                best_distance = distance

        if best_id is None:
            if observation.distance_m > MAPPED_BALL_MAX_CREATE_DISTANCE_M:
                return None
            ball_id = self.next_mapped_ball_id
            self.next_mapped_ball_id += 1
            self.mapped_balls[ball_id] = MappedBall(
                id=ball_id,
                x_m=observation.world_x_m,
                y_m=observation.world_y_m,
                confidence=observation.confidence,
                first_seen_s=now,
                last_seen_s=now,
                source=observation.source,
            )
            return ball_id

        ball = self.mapped_balls[best_id]
        # Weight grows as the robot approaches: far observations barely move the stored
        # position, close observations snap it toward the more accurate estimate.
        close_factor = max(0.0, 1.0 - observation.distance_m / MAPPED_BALL_MAX_CREATE_DISTANCE_M)
        weight = min(0.75, max(0.12, observation.confidence * 0.35 + close_factor * 0.5))
        ball.x_m = ball.x_m * (1.0 - weight) + observation.world_x_m * weight
        ball.y_m = ball.y_m * (1.0 - weight) + observation.world_y_m * weight
        ball.confidence = max(ball.confidence, observation.confidence)
        ball.last_seen_s = now
        ball.seen_count += 1
        ball.source = observation.source
        ball.state = "detected"
        return best_id

    def _nearest_lidar_candidate_observation(self) -> BallObservationInput | None:
        """Return the nearest LiDAR ball candidate as a synthetic observation, or None."""
        candidates = self._lidar_ball_candidates()
        if not candidates:
            return None
        cx, cy = min(candidates, key=lambda c: math.hypot(c[0], c[1]))
        distance_m = math.hypot(cx, cy)
        bearing_rad = math.atan2(cy, cx)
        robot_x, robot_y, robot_yaw = self._robot_pose_2d()
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        world_x = robot_x + cos_yaw * cx - sin_yaw * cy
        world_y = robot_y + sin_yaw * cx + cos_yaw * cy
        return BallObservationInput(
            visible=True,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
            confidence=LIDAR_CANDIDATE_CONFIDENCE,
            source="lidar_candidate",
            robot_x_m=cx,
            robot_y_m=cy,
            world_x_m=world_x,
            world_y_m=world_y,
        )

    def _control_observation_for_mode(
        self,
        mode: str,
        observation: BallObservationInput,
        mapped_ball_id: int | None,
    ) -> BallObservationInput:
        if mode not in {"collect", "collect_pattern"}:
            self.active_mapped_target_id = None
            return observation
        if mode == "collect_pattern" and self.collect_pattern_phase != "collect":
            return observation
        if self.behavior.state == CollectorState.SCAN:
            if not observation.visible and self.lidar is not None:
                lidar_obs = self._nearest_lidar_candidate_observation()
                if lidar_obs is not None:
                    return lidar_obs
            return observation
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self._nearest_mapped_target_id()
        # Prefer live camera/depth over stored position when the camera currently sees
        # the active target; the live bearing is fresher and corrects map drift.
        if observation.visible and mapped_ball_id is not None and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self._observation_from_mapped_target(self.active_mapped_target_id)
        if locked is not None:
            return locked
        self.active_mapped_target_id = mapped_ball_id or self._nearest_mapped_target_id()
        return self._observation_from_mapped_target(self.active_mapped_target_id) or observation

    def _nearest_mapped_target_id(self) -> int | None:
        now = time.time()
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        candidates = [
            ball
            for ball in self.mapped_balls.values()
            if ball.state not in {"collected", "collection_failed"}
            and ball.seen_count >= MAPPED_BALL_MIN_SEEN_COUNT
            and now - ball.last_seen_s <= MAPPED_BALL_STALE_AFTER_S
            and not self._across_net(robot_x, ball.x_m)
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda ball: math.hypot(ball.x_m - robot_x, ball.y_m - robot_y)).id

    def _observation_from_mapped_target(self, ball_id: int | None) -> BallObservationInput | None:
        if ball_id is None:
            return None
        ball = self.mapped_balls.get(ball_id)
        if ball is None or ball.state in {"collected", "collection_failed"}:
            return None
        if time.time() - ball.last_seen_s > MAPPED_BALL_STALE_AFTER_S:
            return None
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        robot_yaw = self._robot_yaw_rad()
        dx = ball.x_m - robot_x
        dy = ball.y_m - robot_y
        local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
        if local_x <= -0.1:
            return None
        return BallObservationInput(
            visible=True,
            bearing_rad=math.atan2(local_y, local_x),
            distance_m=math.hypot(local_x, local_y),
            confidence=ball.confidence,
            source=ball.source,
            robot_x_m=local_x,
            robot_y_m=local_y,
            world_x_m=ball.x_m,
            world_y_m=ball.y_m,
        )

    def _observation_from_world_pos(self, world_x_m: float, world_y_m: float) -> BallObservationInput | None:
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        robot_yaw = self._robot_yaw_rad()
        dx = world_x_m - robot_x
        dy = world_y_m - robot_y
        local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
        if local_x <= -0.1:
            return None
        return BallObservationInput(
            visible=True,
            bearing_rad=math.atan2(local_y, local_x),
            distance_m=math.hypot(local_x, local_y),
            confidence=0.8,
            source="collect_one_locked",
            robot_x_m=local_x,
            robot_y_m=local_y,
            world_x_m=world_x_m,
            world_y_m=world_y_m,
        )

    def _mapped_ball_merge_distance(self, observation: BallObservationInput) -> float:
        if not math.isfinite(observation.distance_m):
            return MAPPED_BALL_MERGE_DISTANCE_M
        distance_margin = observation.distance_m * 0.28
        confidence_margin = (1.0 - max(0.0, min(1.0, observation.confidence))) * 0.35
        return max(
            MAPPED_BALL_MERGE_DISTANCE_M,
            min(MAPPED_BALL_MAX_MERGE_DISTANCE_M, distance_margin + confidence_margin),
        )

    def _prune_phantom_mapped_balls(self) -> None:
        """Remove or merge pending entries that duplicate confirmed or other pending entries."""
        now = time.time()
        confirmed: list[MappedBall] = []
        pending: list[MappedBall] = []
        for ball in self.mapped_balls.values():
            if ball.state == "collected":
                continue
            if now - ball.last_seen_s > MAPPED_BALL_STALE_AFTER_S:
                continue
            if ball.seen_count >= MAPPED_BALL_MIN_SEEN_COUNT:
                confirmed.append(ball)
            else:
                pending.append(ball)

        to_remove: set[int] = set()

        # Drop pending entries that are near a confirmed entry.
        for p in pending:
            for c in confirmed:
                if math.hypot(p.x_m - c.x_m, p.y_m - c.y_m) < MAPPED_BALL_MERGE_DISTANCE_M:
                    to_remove.add(p.id)
                    break

        # Merge pending entries that are near each other: absorb into the higher-count entry.
        survivors = [p for p in pending if p.id not in to_remove]
        survivors.sort(key=lambda b: b.seen_count, reverse=True)
        absorbed: set[int] = set()
        for i, dominant in enumerate(survivors):
            if dominant.id in absorbed:
                continue
            for weak in survivors[i + 1 :]:
                if weak.id in absorbed:
                    continue
                if math.hypot(dominant.x_m - weak.x_m, dominant.y_m - weak.y_m) < MAPPED_BALL_MERGE_DISTANCE_M:
                    dominant.seen_count += weak.seen_count
                    dominant.confidence = max(dominant.confidence, weak.confidence)
                    if dominant.source != "oak_depth" and weak.source == "oak_depth":
                        dominant.source = weak.source
                    dominant.last_seen_s = max(dominant.last_seen_s, weak.last_seen_s)
                    absorbed.add(weak.id)

        to_remove.update(absorbed)

        for ball_id in to_remove:
            del self.mapped_balls[ball_id]

    def _mark_nearest_mapped_ball_collected(self) -> None:
        if not self.mapped_balls:
            return
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        active = [ball for ball in self.mapped_balls.values() if ball.state != "collected"]
        if not active:
            return
        nearest = min(active, key=lambda ball: math.hypot(ball.x_m - robot_x, ball.y_m - robot_y))
        nearest.state = "collected"
        nearest.last_seen_s = time.time()
        if self.active_mapped_target_id == nearest.id:
            self.active_mapped_target_id = None

    def _mapped_ball_rows(self) -> tuple[list[dict[str, object]], list[RouteBall]]:
        now = time.time()
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        robot_yaw = self._robot_yaw_rad()
        rows: list[dict[str, object]] = []
        same_side_balls: list[RouteBall] = []
        for ball in sorted(self.mapped_balls.values(), key=lambda item: item.id):
            if ball.state == "collected":
                continue
            age_s = now - ball.last_seen_s
            if age_s > MAPPED_BALL_STALE_AFTER_S:
                continue
            confirmed = ball.seen_count >= MAPPED_BALL_MIN_SEEN_COUNT
            across_net = self._across_net(robot_x, ball.x_m)
            local_x = math.cos(-robot_yaw) * (ball.x_m - robot_x) - math.sin(-robot_yaw) * (ball.y_m - robot_y)
            local_y = math.sin(-robot_yaw) * (ball.x_m - robot_x) + math.cos(-robot_yaw) * (ball.y_m - robot_y)
            distance_m = math.hypot(local_x, local_y)
            bearing_rad = math.atan2(local_y, local_x)
            visible_candidate = (
                not across_net
                and local_x > 0
                and abs(bearing_rad) <= SUPERVISED_FOV_RAD / 2
                and distance_m <= SUPERVISED_MAX_RANGE_M
            )
            rows.append(
                {
                    "id": ball.id,
                    "x_m": ball.x_m,
                    "y_m": ball.y_m,
                    "side": "across_net" if across_net else "same_side",
                    "visible_candidate": visible_candidate,
                    "confirmed": confirmed,
                    "planned": False,
                    "order": None,
                    "risk": None,
                    "source": ball.source,
                    "confidence": ball.confidence,
                    "seen_count": ball.seen_count,
                    "age_s": age_s,
                }
            )
            if ROUTE_PLANNER_AVAILABLE and not across_net and confirmed:
                same_side_balls.append(RouteBall(x=ball.x_m, y=ball.y_m, id=ball.id))
        return rows, same_side_balls

    def _route_snapshot(self) -> dict[str, object]:
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        robot_yaw = self._robot_yaw_rad()
        ball_rows, same_side_balls = self._mapped_ball_rows()

        route_points: list[dict[str, float]] = []
        legs_payload: list[dict[str, object]] = []
        bounds_payload: dict[str, float] | None = None
        if ROUTE_PLANNER_AVAILABLE and same_side_balls:
            side = "left" if robot_x < NET_X_M else "right"
            bounds = route_half_bounds(side)
            bounds_payload = {
                "min_x": bounds.min_x,
                "max_x": bounds.max_x,
                "min_y": bounds.min_y,
                "max_y": bounds.max_y,
            }
            scenario = RouteScenario(
                seed=0,
                bounds=bounds,
                robot_start=RoutePoint(robot_x, robot_y),
                obstacles=[
                    RouteObstacle(
                        "rect",
                        "net",
                        NET_X_M,
                        0.0,
                        width=ROUTE_NET_CLEARANCE_X_M * 2,
                        height=12.0,
                    )
                ],
                balls=same_side_balls,
            )
            legs, metrics = route_plan_route(
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
            route = [scenario.robot_start]
            planned_orders = {leg.ball_id: order for order, leg in enumerate(legs, start=1)}
            risks = {leg.ball_id: leg.risk for leg in legs}
            for leg in legs:
                route.extend(leg.path[1:])
                legs_payload.append(
                    {
                        "ball_id": leg.ball_id,
                        "distance_m": leg.distance_m,
                        "travel_s": leg.travel_s,
                        "mode": leg.mode,
                        "risk": leg.risk,
                    }
                )
            route_points = [{"x_m": point.x, "y_m": point.y} for point in route]
            for row in ball_rows:
                order = planned_orders.get(int(row["id"]))
                if order is None:
                    continue
                row["planned"] = True
                row["order"] = order
                row["risk"] = risks.get(int(row["id"]))
        else:
            metrics = None

        return {
            "planner_available": ROUTE_PLANNER_AVAILABLE,
            "source": "sensor_mapped",
            "updated_at": time.time(),
            "court": {
                "min_x": -11.885,
                "max_x": 11.885,
                "min_y": -5.485,
                "max_y": 5.485,
                "net_x": NET_X_M,
            },
            "active_bounds": bounds_payload,
            "active_target_id": self.active_mapped_target_id,
            "camera_fov_rad": SUPERVISED_FOV_RAD,
            "camera_max_range_m": MAPPED_BALL_MAX_CREATE_DISTANCE_M,
            "lidar_fusion_enabled": self.lidar is not None,
            "lidar_front_index_ratio": LIDAR_FRONT_INDEX_RATIO,
            "robot": {"x_m": robot_x, "y_m": robot_y, "yaw_rad": robot_yaw},
            "balls": ball_rows,
            "route": route_points,
            "legs": legs_payload,
            "metrics": None
            if metrics is None
            else {
                "balls_detected": metrics.balls_detected,
                "balls_collectable": metrics.balls_collectable,
                "balls_blocked": metrics.balls_blocked,
                "total_distance_m": metrics.total_distance_m,
                "total_time_s": metrics.total_time_s,
                "planned_replans": metrics.planned_replans,
            },
        }

    def _draw_debug(
        self,
        frame: np.ndarray | None,
        detection: BallDetection | None,
        command: ConceptACommand,
    ) -> None:
        if not VISION_ENABLED or frame is None:
            return
        if detection is not None:
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                (0, 0, 255),
                2,
            )
            cv2.circle(frame, (int(detection.center_x), int(detection.center_y)), 4, (255, 0, 0), -1)

        cv2.putText(
            frame,
            f"collector={command.state.value} balls={self.collection_count}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        display_frame = frame
        display_width = self.display.getWidth()
        display_height = self.display.getHeight()
        if frame.shape[1] != display_width or frame.shape[0] != display_height:
            display_frame = cv2.resize(frame, (display_width, display_height), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        image_ref = self.display.imageNew(rgb.tobytes(), Display.RGB, display_width, display_height)
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def _apply_command(self, command: ConceptACommand) -> None:
        self.set_base_command(command.base.linear_speed_m_s, command.base.angular_speed_rad_s)
        self.set_collector_command(command.collector.lift_wheel_speed)

    def _front_range_m(self) -> float | None:
        return self._lidar_front_range_m()

    def _robot_yaw_rad(self) -> float:
        orientation = self.robot_node.getOrientation()
        return math.atan2(orientation[3], orientation[0])

    def _robot_pose_2d(self) -> tuple[float, float, float]:
        x_m, y_m, _z_m = self.robot_node.getPosition()
        return (x_m, y_m, self._robot_yaw_rad())

    def _write_sensor_snapshots(self) -> None:
        now = time.time()
        if now - self.last_sensor_write_s < 1.0:
            return
        self.last_sensor_write_s = now
        candidates = self._lidar_ball_candidates()
        ir_left = self.ir_intake_left.getValue() if self.ir_intake_left is not None else None
        ir_right = self.ir_intake_right.getValue() if self.ir_intake_right is not None else None
        self.sensor_store.write(
            {
                "front_camera": self._camera_snapshot(self.camera),
                "front_depth": self._depth_snapshot(),
                "front_lidar": self._lidar_snapshot(),
                "lidar_candidates": [
                    {"robot_x_m": cx, "robot_y_m": cy, "distance_m": math.hypot(cx, cy)}
                    for cx, cy in candidates
                ],
                "ir_intake": {
                    "left": ir_left,
                    "right": ir_right,
                    "threshold": IR_INTAKE_TRIGGER_THRESHOLD,
                    "triggered": self._ir_intake_triggered(),
                    "left_available": self.ir_intake_left is not None,
                    "right_available": self.ir_intake_right is not None,
                },
            }
        )

    def _sensor_mount_snapshot(self) -> dict[str, object]:
        return {
            "front_lidar": self._node_pose_snapshot("FRONT_LIDAR"),
            "front_camera": self._node_pose_snapshot("FRONT_CAMERA"),
            "front_depth": self._node_pose_snapshot("FRONT_DEPTH"),
            "collector_center": {
                "world_z_m": self.robot_node.getPosition()[2] + 0.04,
                "local_x_m": sum(INTAKE_ZONE_X_M) / 2,
                "local_y_m": 0.0,
                "local_z_m": 0.04,
                "role": "low front collection confirmation",
            },
        }

    def _node_pose_snapshot(self, def_name: str) -> dict[str, object] | None:
        node = self.robot.getFromDef(def_name)
        if node is None:
            return None
        x_m, y_m, z_m = node.getPosition()
        roles = {
            "FRONT_LIDAR": "navigation obstacle map and path safety",
            "FRONT_CAMERA": "OAK-D RGB ball detection",
            "FRONT_DEPTH": "OAK-D depth range estimate",
        }
        return {
            "world_x_m": x_m,
            "world_y_m": y_m,
            "world_z_m": z_m,
            "role": roles.get(def_name, "unknown"),
        }

    def _oak_depth_status(self, observation: BallObservationInput) -> dict[str, object]:
        depth_available = VISION_ENABLED and self.depth_camera is not None
        depth_range_m = None if math.isinf(observation.distance_m) else observation.distance_m
        return {
            "available": depth_available,
            "used_for_current_observation": observation.source == "oak_depth",
            "range_m": depth_range_m if observation.source == "oak_depth" else None,
            "source": observation.source,
            "min_range_m": None if self.depth_camera is None else float(self.depth_camera.getMinRange()),
            "max_range_m": None if self.depth_camera is None else float(self.depth_camera.getMaxRange()),
            "role": "OAK-D stereo depth estimates ball distance; RGB detection provides image coordinates and bearing",
        }

    def _camera_snapshot(self, camera: Camera | None) -> dict[str, object] | None:
        if camera is None:
            return None
        width = camera.getWidth()
        height = camera.getHeight()
        raw = camera.getImage()
        if raw is None:
            return None
        preview_width = min(width, 960)
        preview_height = max(1, round(height * preview_width / max(1, width)))
        data = bytes(raw)
        if VISION_ENABLED and cv2 is not None and np is not None and preview_width != width:
            frame = np.frombuffer(data, np.uint8).reshape((height, width, 4))
            frame = cv2.resize(frame, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
            data = frame.tobytes()
            width = preview_width
            height = preview_height
        return {
            "width": width,
            "height": height,
            "native_width": camera.getWidth(),
            "native_height": camera.getHeight(),
            "format": "bgra-bmp",
            "data_url": _bgra_bmp_data_url(data, width, height),
        }

    def _lidar_ranges(self) -> list[float] | None:
        if self.lidar is None:
            return None
        return [float(value) for value in self.lidar.getRangeImage()]

    def _lidar_ball_candidates(self) -> list[tuple[float, float]]:
        """Return (robot-local x, y) of LiDAR cluster candidates that could be balls.

        The LiDAR is mounted at ball height so isolated close returns between
        LIDAR_CANDIDATE_MIN_M and LIDAR_CANDIDATE_MAX_M are plausible balls.
        Each cluster centroid is returned in robot-local (x, y) coordinates.
        The caller navigates toward the closest candidate; the OAK-D camera
        confirms or rejects it once within visual range.
        """
        ranges = self._lidar_ranges()
        if not ranges:
            return []
        n = len(ranges)
        # Known robot-body self-detection: motor pods appear at ~28-39cm in the
        # ±43°-73° side sectors. Exclude these with a body-exclusion radius.
        BODY_EXCLUSION_M = 0.55
        CANDIDATE_MIN_M = 0.50   # ignore closer returns (robot body / bracket)
        CANDIDATE_MAX_M = 8.0    # RPLIDAR C1 effective range
        # Minimum angular span for a ball at its range:
        # ball diameter 6.7cm at 1m subtends ~3.8 deg, at 3m ~1.3 deg
        MIN_CLUSTER_SPAN_DEG = 1.0
        MAX_CLUSTER_SPAN_DEG = 25.0  # wider blobs are walls/posts, not balls

        # Convert polar to cartesian, filter to candidate range
        LIDAR_LOCAL_X = -0.20  # sensor mount x offset in robot frame
        LIDAR_LOCAL_Y = 0.0
        points: list[tuple[float, float, float]] = []  # (range, robot_x, robot_y)
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < CANDIDATE_MIN_M or r > CANDIDATE_MAX_M:
                continue
            angle = (i / n) * 2 * math.pi - math.pi  # Webots: index 0 = backward, n/2 = forward
            # Transform from LiDAR local to robot local
            lx = LIDAR_LOCAL_X + r * math.cos(angle)
            ly = LIDAR_LOCAL_Y + r * math.sin(angle)
            # Exclude self-body (within body exclusion radius of robot center)
            if math.hypot(lx, ly) < BODY_EXCLUSION_M:
                continue
            points.append((r, lx, ly))

        if not points:
            return []

        # Simple angular clustering: group consecutive scan indices into blobs
        clusters: list[list[tuple[float, float, float]]] = []
        GAP_DEG = 5.0
        gap_indices = int(GAP_DEG / 360.0 * n)
        prev_i: int | None = None
        current: list[tuple[float, float, float]] = []
        valid_indices = [
            i for i, r in enumerate(ranges)
            if math.isfinite(r) and CANDIDATE_MIN_M <= r <= CANDIDATE_MAX_M
            and math.hypot(
                LIDAR_LOCAL_X + r * math.cos((i / n) * 2 * math.pi - math.pi),
                LIDAR_LOCAL_Y + r * math.sin((i / n) * 2 * math.pi - math.pi),
            ) >= BODY_EXCLUSION_M
        ]
        for idx in valid_indices:
            if prev_i is not None and idx - prev_i > gap_indices:
                if current:
                    clusters.append(current)
                current = []
            r = ranges[idx]
            angle = (idx / n) * 2 * math.pi - math.pi
            lx = LIDAR_LOCAL_X + r * math.cos(angle)
            ly = LIDAR_LOCAL_Y + r * math.sin(angle)
            current.append((r, lx, ly))
            prev_i = idx
        if current:
            clusters.append(current)

        candidates: list[tuple[float, float]] = []
        for cluster in clusters:
            span_deg = len(cluster) / n * 360.0
            if not (MIN_CLUSTER_SPAN_DEG <= span_deg <= MAX_CLUSTER_SPAN_DEG):
                continue
            cx = sum(p[1] for p in cluster) / len(cluster)
            cy = sum(p[2] for p in cluster) / len(cluster)
            candidates.append((cx, cy))

        return candidates

    def _depth_snapshot(self) -> dict[str, object] | None:
        if self.depth_camera is None:
            return None
        depth = self._depth_frame_m()
        if depth is None:
            return None
        valid = depth[np.isfinite(depth) & (depth > 0)]
        min_range = float(self.depth_camera.getMinRange())
        max_range = float(self.depth_camera.getMaxRange())
        span = max(0.001, max_range - min_range)
        clipped = np.where(np.isfinite(depth) & (depth > 0), depth, max_range)
        normalized = np.clip((clipped - min_range) / span, 0.0, 1.0)
        intensity = ((1.0 - normalized) * 255).astype(np.uint8)
        bgra = np.dstack((intensity, intensity, intensity, np.full_like(intensity, 255))).tobytes()
        return {
            "width": int(depth.shape[1]),
            "height": int(depth.shape[0]),
            "format": "depth-bmp",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "valid_count": int(valid.size),
            "median_range_m": None if valid.size == 0 else float(np.median(valid)),
            "data_url": _bgra_bmp_data_url(bgra, int(depth.shape[1]), int(depth.shape[0])),
        }

    def _lidar_front_range_m(self) -> float | None:
        ranges = self._lidar_ranges()
        if not ranges:
            return None
        center = int(round((len(ranges) - 1) * LIDAR_FRONT_INDEX_RATIO))
        half_width = max(2, len(ranges) // 72)
        window = ranges[max(0, center - half_width) : min(len(ranges), center + half_width + 1)]
        valid = [
            value for value in window
            if math.isfinite(value) and value >= LIDAR_FRONT_MIN_OBSTACLE_RANGE_M
        ]
        if not valid:
            return None
        valid.sort()
        return valid[len(valid) // 2]

    def _lidar_snapshot(self) -> dict[str, object] | None:
        if self.lidar is None:
            return None
        ranges = self._lidar_ranges()
        if not ranges:
            return None
        width = len(ranges)
        height = 64
        max_range = float(self.lidar.getMaxRange())
        min_range = float(self.lidar.getMinRange())
        span = max(0.001, max_range - min_range)
        pixels = bytearray()
        normalized_ranges = []
        ranges_m: list[float | None] = []
        for value in ranges:
            if math.isfinite(value) and value > 0:
                normalized_ranges.append(max(0.0, min(1.0, (value - min_range) / span)))
                ranges_m.append(float(value))
            else:
                normalized_ranges.append(1.0)
                ranges_m.append(None)
        for y in range(height):
            for normalized in normalized_ranges:
                bar_height = max(1, int((1.0 - normalized) * (height - 1)))
                if y >= height - bar_height:
                    pixels.extend((80, 220, 120, 255))
                else:
                    pixels.extend((18, 24, 28, 255))
        return {
            "width": width,
            "height": height,
            "format": "lidar-bmp",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "front_range_m": self._lidar_front_range_m(),
            "ranges_m": ranges_m,
            "data_url": _bgra_bmp_data_url(bytes(pixels), width, height),
        }

    def _write_status(
        self,
        requested_mode: str,
        command: ConceptACommand,
        observation: BallObservationInput,
        detection: BallDetection | None,
        inventory: dict[str, float | int | None],
    ) -> None:
        now = time.time()
        if now - self.last_status_write_s < 0.2 and not self.collection_confirmed:
            return
        self.last_status_write_s = now

        x_m, y_m, z_m = self.robot_node.getPosition()
        target = self.survey_behavior.current_target()
        front_range_m = self._front_range_m()
        search_snapshot = self.search_behavior.snapshot(x_m, y_m)
        if self.control_mode == "search":
            current_action = search_snapshot["search_state"]
        elif self.control_mode == "collect_pattern":
            current_action = f"collect_pattern:{self.collect_pattern_phase}"
        else:
            current_action = command.state.value
        self.status_store.write(
            {
                "requested_mode": requested_mode,
                "actual_mode": self.control_mode,
                "collector_state": command.state.value,
                "mission_name": "Collect All Balls",
                "mission_elapsed_s": now - self.started_at,
                "current_action": current_action,
                "current_zone": search_snapshot["zone_id"],
                "current_target_id": self.active_mapped_target_id,
                "coverage_pct": search_snapshot["coverage_pct"],
                "balls_collected": self.collection_count,
                "loop_count": self.loop_count,
                "uptime_s": now - self.started_at,
                "telemetry_enabled": self.telemetry.enabled,
                "vision_enabled": VISION_ENABLED,
                "route_visualization_enabled": self.route_visualizer.enabled,
                "completion": {
                    "current_side_complete": inventory["same_side_remaining"] == 0,
                    "reported": self.collection_complete_reported,
                },
                "balls": inventory,
                "map": self._route_snapshot(),
                "robot": {
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": z_m,
                    "yaw_rad": self._robot_yaw_rad(),
                },
                "sensor_mounts": self._sensor_mount_snapshot(),
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
                "detection": None
                if detection is None
                else {
                    "x": detection.x,
                    "y": detection.y,
                    "width": detection.width,
                    "height": detection.height,
                    "area_px": detection.area_px,
                    "center_x": detection.center_x,
                    "center_y": detection.center_y,
                },
                "command": {
                    "linear_speed_m_s": command.base.linear_speed_m_s,
                    "angular_speed_rad_s": command.base.angular_speed_rad_s,
                    "lift_wheel_speed": command.collector.lift_wheel_speed,
                    "intake_enabled": command.collector.intake_enabled,
                },
                "scan": {
                    "elapsed_s": self.behavior.state_elapsed_s,
                    "full_turn_s": self.behavior.config.scan_full_turn_s,
                    "progress": min(1.0, self.behavior.state_elapsed_s / max(0.001, self.behavior.config.scan_full_turn_s)),
                    "best_visible": self.behavior.scan_best_observation is not None,
                    "best_distance_m": None
                    if self.behavior.scan_best_observation is None or math.isinf(self.behavior.scan_best_observation.distance_m)
                    else self.behavior.scan_best_observation.distance_m,
                    "best_bearing_rad": None
                    if self.behavior.scan_best_observation is None
                    else self.behavior.scan_best_observation.bearing_rad,
                },
                "target_lock": {
                    "mapped_ball_id": self.active_mapped_target_id,
                    "locked": self.active_mapped_target_id is not None,
                },
                "collect_one": {
                    "phase": self.collect_one_phase,
                    "start_pose": None
                    if self.collect_one_start_pose is None
                    else {
                        "x_m": self.collect_one_start_pose[0],
                        "y_m": self.collect_one_start_pose[1],
                        "yaw_rad": self.collect_one_start_pose[2],
                    },
                    "scan_target_yaw_rad": self.collect_one_scan_target_yaw,
                    "complete_reported": self.collect_one_complete_reported,
                },
                "collect_pattern": {
                    "phase": self.collect_pattern_phase,
                    "collect_elapsed_s": self.collect_pattern_collect_elapsed_s,
                    "failures": self.collect_pattern_failures,
                    "timeout_s": COLLECT_PATTERN_COLLECTION_TIMEOUT_S,
                    "active_target_id": self.active_mapped_target_id,
                },
                "scan_side": {
                    "active": self.scan_side_started_at is not None,
                    "elapsed_s": None if self.scan_side_started_at is None else now - self.scan_side_started_at,
                    "duration_s": SCAN_SIDE_DURATION_S,
                    "progress": 0.0
                    if self.scan_side_started_at is None
                    else min(1.0, (now - self.scan_side_started_at) / SCAN_SIDE_DURATION_S),
                },
                "search": search_snapshot,
                "survey": {
                    "state": self.survey_behavior.state.value,
                    "waypoint_index": self.survey_behavior.waypoint_index,
                    "waypoint_count": len(self.survey_behavior.waypoints),
                    "target_x_m": None if target is None else target[0],
                    "target_y_m": None if target is None else target[1],
                    "sample_count": self.survey_behavior.sample_count,
                    "bounds_saved": self.survey_behavior.court_bounds is not None,
                    "bounds": self.survey_behavior.court_bounds,
                    "front_range_m": front_range_m,
                    "oak_depth": None
                    if self._last_survey_vision is None
                    else {
                        "center_m": self._last_survey_vision.center_m,
                        "left_m": self._last_survey_vision.left_m,
                        "right_m": self._last_survey_vision.right_m,
                        "valid_count": self._last_survey_vision.valid_count,
                    },
                },
                "map_mission": self._map_mission.telemetry(),
                "collection_animation_active": self.collection_animation is not None,
            }
        )

    def _print_status(
        self,
        command: ConceptACommand,
        observation: BallObservationInput,
    ) -> None:
        status = (
            f"mode={self.control_mode} "
            f"collector={command.state.value} "
            f"visible={observation.visible} "
            f"distance={observation.distance_m:.2f}m "
            f"bearing={math.degrees(observation.bearing_rad):.1f}deg "
            f"range_source={observation.source} "
            f"balls={self.collection_count}"
        )
        if observation.world_x_m is not None and observation.world_y_m is not None:
            status += f" ball_world=({observation.world_x_m:.2f},{observation.world_y_m:.2f})"
        if self.control_mode == "survey":
            x_m, y_m, _z_m = self.robot_node.getPosition()
            target = self.survey_behavior.current_target()
            target_text = "none" if target is None else f"({target[0]:.2f},{target[1]:.2f})"
            status += (
                f" survey_state={self.survey_behavior.state.value} "
                f"waypoint={self.survey_behavior.waypoint_index + 1}/{len(self.survey_behavior.waypoints)} "
                f"samples={self.survey_behavior.sample_count} "
                f"pos=({x_m:.2f},{y_m:.2f}) "
                f"target={target_text} "
                f"bounds={'saved' if self.survey_behavior.court_bounds else 'pending'}"
            )
        if self.control_mode in {"search", "collect_pattern"}:
            x_m, y_m, _z_m = self.robot_node.getPosition()
            search = self.search_behavior.snapshot(x_m, y_m)
            target_x = search["target_x_m"]
            target_y = search["target_y_m"]
            target_text = "none" if target_x is None or target_y is None else f"({target_x:.2f},{target_y:.2f})"
            status += (
                f" search_state={search['search_state']} "
                f"zone={search['zone_id']} "
                f"coverage={search['coverage_pct']:.1f}% "
                f"waypoint={search['waypoint_index'] + 1}/{search['waypoint_count']} "
                f"target={target_text} "
                f"path={search['path_status']}"
            )
            if self.control_mode == "collect_pattern":
                status += f" collect_pattern_phase={self.collect_pattern_phase}"
        print(status)

    def _ir_intake_triggered(self) -> bool:
        left = self.ir_intake_left is not None and self.ir_intake_left.getValue() > IR_INTAKE_TRIGGER_THRESHOLD
        right = self.ir_intake_right is not None and self.ir_intake_right.getValue() > IR_INTAKE_TRIGGER_THRESHOLD
        return left or right

    def _check_collection(self, command: ConceptACommand) -> bool:
        """Trigger collection when the collector is running and a ball enters the intake zone.

        The supervisor's ground-truth position is the primary trigger. IR sensors gate
        the same check in hardware; in simulation their single-ray geometry may miss a
        ball that is slightly off the sensor axis, so we do not require them here.
        The IR values are still logged in sensor snapshots for diagnostics.
        """
        if not command.collector.intake_enabled:
            return False
        if self.collection_animation is not None:
            return False
        nearest_ahead: tuple[float, float, float] | None = None
        nearest_ahead_label = ""
        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None:
                continue
            local_position = self._world_to_robot_local(ball.getPosition())
            if local_position[0] > 0 and (nearest_ahead is None or local_position[0] < nearest_ahead[0]):
                nearest_ahead = local_position
                nearest_ahead_label = f"TENNIS_BALL_{index:02d}"
            if self._in_intake_zone(local_position):
                self._start_collection_animation(index, ball)
                return True
        if nearest_ahead is not None and self.loop_count % 20 == 0:
            x, y, z = nearest_ahead
            print(
                f"  [intake_check] nearest_ahead={nearest_ahead_label} local=({x:.3f},{y:.3f},{z:.3f})"
                f"  zone_x={INTAKE_ZONE_X_M} half_y={INTAKE_HALF_WIDTH_M} max_z={INTAKE_MAX_HEIGHT_M}"
            )
        return False

    def _start_collection_animation(self, index: int, ball) -> None:
        self.collection_count += 1
        self.telemetry.add_collection()
        self.collection_animation = {
            "index": index,
            "elapsed_s": 0.0,
        }
        ball.remove()
        self._set_collection_visual_position(COLLECTION_PATH_LOCAL[0])
        print(f"collecting tennis_ball_{index:02d}; total={self.collection_count}")

    def _update_collection_animation(self, dt_s: float) -> None:
        if self.collection_animation is None:
            return

        elapsed_s = float(self.collection_animation["elapsed_s"]) + max(0.0, dt_s)
        self.collection_animation["elapsed_s"] = elapsed_s
        progress = min(1.0, elapsed_s / COLLECTION_ANIMATION_S)

        local_position = self._collection_path_position(progress)
        self._set_collection_visual_position(local_position)

        if progress >= 1.0:
            self._hide_collection_visual()
            self.collection_animation = None

    def _set_collection_visual_position(self, local_position: tuple[float, float, float]) -> None:
        if self.collection_visual_ball is None:
            return
        self.collection_visual_ball.getField("translation").setSFVec3f(list(local_position))

    def _hide_collection_visual(self) -> None:
        self._set_collection_visual_position((0.0, 0.0, -1.0))

    def _collection_path_position(self, progress: float) -> tuple[float, float, float]:
        segment_count = len(COLLECTION_PATH_LOCAL) - 1
        scaled = min(segment_count - 1e-9, max(0.0, progress) * segment_count)
        segment_index = int(scaled)
        segment_t = scaled - segment_index
        start = COLLECTION_PATH_LOCAL[segment_index]
        end = COLLECTION_PATH_LOCAL[segment_index + 1]
        return (
            start[0] + (end[0] - start[0]) * segment_t,
            start[1] + (end[1] - start[1]) * segment_t,
            start[2] + (end[2] - start[2]) * segment_t,
        )

    def _is_collection_animation_ball(self, index: int) -> bool:
        return self.collection_animation is not None and self.collection_animation["index"] == index

    def _world_to_robot_local(self, world_position: list[float]) -> tuple[float, float, float]:
        robot_position = self.robot_node.getPosition()
        orientation = self.robot_node.getOrientation()
        dx = world_position[0] - robot_position[0]
        dy = world_position[1] - robot_position[1]
        dz = world_position[2] - robot_position[2]
        return (
            orientation[0] * dx + orientation[3] * dy + orientation[6] * dz,
            orientation[1] * dx + orientation[4] * dy + orientation[7] * dz,
            orientation[2] * dx + orientation[5] * dy + orientation[8] * dz,
        )

    def _robot_local_to_world(self, local_position: tuple[float, float, float]) -> tuple[float, float, float]:
        robot_position = self.robot_node.getPosition()
        orientation = self.robot_node.getOrientation()
        x, y, z = local_position
        return (
            robot_position[0] + orientation[0] * x + orientation[1] * y + orientation[2] * z,
            robot_position[1] + orientation[3] * x + orientation[4] * y + orientation[5] * z,
            robot_position[2] + orientation[6] * x + orientation[7] * y + orientation[8] * z,
        )

    def _in_intake_zone(self, local_position: tuple[float, float, float]) -> bool:
        x, y, z = local_position
        return (
            INTAKE_ZONE_X_M[0] <= x <= INTAKE_ZONE_X_M[1]
            and abs(y) <= INTAKE_HALF_WIDTH_M
            and z <= INTAKE_MAX_HEIGHT_M
        )


if __name__ == "__main__":
    BallDetectorController().run()
