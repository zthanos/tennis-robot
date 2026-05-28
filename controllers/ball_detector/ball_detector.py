"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import os
import base64
import struct
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

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
from controller import Camera, Display, Motor, RangeFinder, Supervisor
from survey import CourtSurveyBehavior, SurveyState
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
        raise ImportError("RGB/depth vision disabled; set USE_RGB_VISION=true to enable it")

    import cv2
    import numpy as np
    from perception import (
        BallDetection,
        BallObservation,
        CameraMount,
        RobotPose2D,
        detect_largest_ball,
        estimate_ball_observation,
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
COLLECTION_ANIMATION_S = 0.75
COLLECTION_PATH_LOCAL = (
    (0.64, 0.0, 0.045),
    (0.58, 0.0, 0.11),
    (0.28, 0.0, 0.28),
    (0.12, 0.0, 0.40),
)
FRONT_CAMERA_MOUNT = CameraMount(x_m=0.31, y_m=0.0, yaw_rad=0.0) if VISION_ENABLED else None


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


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"invalid {name}={value!r}; using {default}")
        return default


class BallDetectorController:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.camera = self._device("front_camera", Camera)
        self.depth_camera = self._optional_device("front_depth", RangeFinder)
        self.collector_camera = self._optional_device("collector_camera", Camera)
        self.display = self._device("camera_display", Display)
        self.left_motor = self._device("left_wheel_motor", Motor)
        self.right_motor = self._device("right_wheel_motor", Motor)
        self.lift_motor = self._optional_device("lift_wheel_motor", Motor)
        self.telemetry = setup_telemetry("ball-detector-controller")
        self.max_speed_rad_s = _env_float("ROBOT_MAX_WHEEL_SPEED_RAD_S", MAX_SPEED_RAD_S)
        self.behavior = ConceptACollectorBehavior(ConceptAConfig.from_env())
        self.survey_behavior = CourtSurveyBehavior.from_env()
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
        self.last_command: ConceptACommand | None = None
        self.loop_count = 0
        self.started_at = time.time()
        self.last_status_write_s = 0.0
        self.last_sensor_write_s = 0.0
        self.collection_complete_reported = False

        self.camera.enable(TIME_STEP_MS)
        if self.depth_camera is not None:
            self.depth_camera.enable(TIME_STEP_MS)
        if self.collector_camera is not None:
            self.collector_camera.enable(TIME_STEP_MS)
        self.left_motor.setPosition(math.inf)
        self.right_motor.setPosition(math.inf)
        if self.lift_motor is not None:
            self.lift_motor.setPosition(math.inf)
        self.set_speed(0.0, 0.0)
        if VISION_ENABLED:
            print("ball_detector controller started with RGB/depth vision")
        else:
            print(f"ball_detector controller started in supervised emulator mode: {VISION_IMPORT_ERROR}")
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
                depth_frame = self._depth_frame()
                self.telemetry.add_frame()
                detection = self._detect_largest_ball(image)
                if VISION_ENABLED:
                    observation = self._observation_from_detection(detection, depth_frame)
                else:
                    observation = self._supervised_ball_observation()
                control_command = self.command_store.read()
                inventory = self._ball_inventory()
                effective_mode = self._effective_control_mode(control_command.mode, inventory)
                if effective_mode == "survey":
                    command = self._survey_command_for_mode(effective_mode, depth_frame)
                else:
                    command = self._collector_command_for_mode(effective_mode, observation)
                self.collection_confirmed = False
                self._draw_debug(image, detection, command)
                self._apply_command(command)
                self.telemetry.add_collector_state(command.state.value)
                self.collection_confirmed = self._simulate_collection(command)
                if self.collection_confirmed:
                    self.route_visualizer.refresh()
                self._write_status(control_command.mode, command, observation, depth_frame, inventory)
                self._write_sensor_snapshots(depth_frame)
                self.last_command = command
                self.loop_count += 1
                if self.loop_count % 60 == 0:
                    self._print_status(command, observation, depth_frame)
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _effective_control_mode(self, requested_mode: str, inventory: dict[str, float | int | None]) -> str:
        if requested_mode == "collect" and inventory["same_side_remaining"] == 0 and self.collection_animation is None:
            if not self.collection_complete_reported:
                print(f"collection complete for current side; total={self.collection_count}")
                self.command_store.write("idle", source="webots-complete")
                self.collection_complete_reported = True
            return "idle"
        if requested_mode == "collect" and inventory["same_side_remaining"] > 0:
            self.collection_complete_reported = False
        return requested_mode

    def _collector_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.survey_behavior.reset()
            self.control_mode = mode
            print(f"control mode changed to {self.control_mode}")
            if mode == "collect":
                self.route_visualizer.refresh()
            else:
                self.route_visualizer.clear()

        if mode == "collect":
            return self.behavior.update(
                observation,
                TIME_STEP_MS / 1000,
                collection_confirmed=self.collection_confirmed,
            )

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _survey_command_for_mode(self, mode: str, depth_frame: np.ndarray | None) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.survey_behavior.reset()
            self.control_mode = mode
            print(f"control mode changed to {self.control_mode}")

        x_m, y_m, _z_m = self.robot_node.getPosition()
        survey_command = self.survey_behavior.update(
            x_m,
            y_m,
            self._robot_yaw_rad(),
            self._front_range_m(depth_frame),
            TIME_STEP_MS / 1000,
        )
        if survey_command.state == SurveyState.DONE:
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

    def _depth_frame(self) -> np.ndarray | None:
        if not VISION_ENABLED or self.depth_camera is None:
            return None
        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        return np.array(self.depth_camera.getRangeImage(), dtype=np.float32).reshape((height, width))

    def _detect_largest_ball(self, frame: np.ndarray) -> BallDetection | None:
        if not VISION_ENABLED or frame is None:
            return None
        return detect_largest_ball(frame)

    def _observation_from_detection(
        self,
        detection: BallDetection | None,
        depth_frame: np.ndarray | None,
    ) -> BallObservationInput:
        if detection is None:
            return BallObservationInput(visible=False)

        observation = self._estimate_observation(detection, depth_frame)
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
            robot_x_m=world_observation.robot_x_m,
            robot_y_m=world_observation.robot_y_m,
            world_x_m=world_observation.world_x_m,
            world_y_m=world_observation.world_y_m,
        )

    def _estimate_observation(
        self,
        detection: BallDetection,
        depth_frame: np.ndarray | None,
    ) -> BallObservation:
        if depth_frame is not None:
            depth_observation = estimate_depth_ball_observation(
                detection,
                depth_frame,
                self.camera.getWidth(),
                self.camera.getHeight(),
                self.camera.getFov(),
            )
            if depth_observation is not None:
                return depth_observation

        return estimate_ball_observation(
            detection,
            self.camera.getWidth(),
            self.camera.getFov(),
        )

    def _supervised_ball_observation(self) -> BallObservationInput:
        nearest: tuple[float, float, float, float, float, float] | None = None
        robot_world_x = self.robot_node.getPosition()[0]
        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None:
                continue
            if self._is_collection_animation_ball(index):
                continue
            ball_position = ball.getPosition()
            if self._across_net(robot_world_x, ball_position[0]):
                continue
            x, y, _z = self._world_to_robot_local(ball_position)
            if x <= 0:
                continue
            distance_m = math.hypot(x, y)
            bearing_rad = math.atan2(y, x)
            if abs(bearing_rad) > SUPERVISED_FOV_RAD / 2 or distance_m > SUPERVISED_MAX_RANGE_M:
                continue
            if nearest is None or distance_m < nearest[0]:
                nearest = (distance_m, bearing_rad, x, y, ball_position[0], ball_position[1])

        if nearest is None:
            return BallObservationInput(visible=False)
        distance_m, bearing_rad, robot_x_m, robot_y_m, world_x_m, world_y_m = nearest
        return BallObservationInput(
            visible=True,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
            confidence=1.0,
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

    def _route_snapshot(self) -> dict[str, object]:
        robot_x, robot_y, _robot_z = self.robot_node.getPosition()
        robot_yaw = self._robot_yaw_rad()
        same_side_balls: list[RouteBall] = []
        ball_rows: list[dict[str, object]] = []

        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None or self._is_collection_animation_ball(index):
                continue

            x_m, y_m, _z_m = ball.getPosition()
            across_net = self._across_net(robot_x, x_m)
            local_x, local_y, _local_z = self._world_to_robot_local([x_m, y_m, _z_m])
            distance_m = math.hypot(local_x, local_y)
            bearing_rad = math.atan2(local_y, local_x)
            visible_candidate = (
                not across_net
                and local_x > 0
                and abs(bearing_rad) <= SUPERVISED_FOV_RAD / 2
                and distance_m <= SUPERVISED_MAX_RANGE_M
            )
            row = {
                "id": index,
                "x_m": x_m,
                "y_m": y_m,
                "side": "across_net" if across_net else "same_side",
                "visible_candidate": visible_candidate,
                "planned": False,
                "order": None,
                "risk": None,
            }
            ball_rows.append(row)
            if ROUTE_PLANNER_AVAILABLE and not across_net:
                same_side_balls.append(RouteBall(x=x_m, y=y_m, id=index))

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
            "updated_at": time.time(),
            "court": {
                "min_x": -11.885,
                "max_x": 11.885,
                "min_y": -5.485,
                "max_y": 5.485,
                "net_x": NET_X_M,
            },
            "active_bounds": bounds_payload,
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
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_ref = self.display.imageNew(rgb.tobytes(), Display.RGB, frame.shape[1], frame.shape[0])
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def _apply_command(self, command: ConceptACommand) -> None:
        self.set_base_command(command.base.linear_speed_m_s, command.base.angular_speed_rad_s)
        self.set_collector_command(command.collector.lift_wheel_speed)

    def _front_range_m(self, depth_frame: np.ndarray | None) -> float | None:
        if depth_frame is None:
            return None
        height, width = depth_frame.shape
        crop = depth_frame[height // 2 - 8 : height // 2 + 8, width // 2 - 8 : width // 2 + 8]
        valid = crop[np.isfinite(crop) & (crop > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _robot_yaw_rad(self) -> float:
        orientation = self.robot_node.getOrientation()
        return math.atan2(orientation[3], orientation[0])

    def _robot_pose_2d(self) -> tuple[float, float, float]:
        x_m, y_m, _z_m = self.robot_node.getPosition()
        return (x_m, y_m, self._robot_yaw_rad())

    def _write_sensor_snapshots(self, depth_frame: np.ndarray | None) -> None:
        now = time.time()
        if now - self.last_sensor_write_s < 1.0:
            return
        self.last_sensor_write_s = now
        self.sensor_store.write(
            {
                "front_camera": self._camera_snapshot(self.camera),
                "collector_camera": self._camera_snapshot(self.collector_camera),
                "front_depth": self._depth_snapshot(depth_frame),
            }
        )

    def _camera_snapshot(self, camera: Camera | None) -> dict[str, object] | None:
        if camera is None:
            return None
        width = camera.getWidth()
        height = camera.getHeight()
        raw = camera.getImage()
        if raw is None:
            return None
        return {
            "width": width,
            "height": height,
            "format": "bgra-bmp",
            "data_url": _bgra_bmp_data_url(bytes(raw), width, height),
        }

    def _depth_snapshot(self, depth_frame: np.ndarray | None) -> dict[str, object] | None:
        if self.depth_camera is None:
            return None
        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        values = (
            depth_frame.reshape(-1).tolist()
            if depth_frame is not None
            else list(self.depth_camera.getRangeImage())
        )
        max_range = float(self.depth_camera.getMaxRange())
        min_range = float(self.depth_camera.getMinRange())
        span = max(0.001, max_range - min_range)
        pixels = bytearray()
        for value in values:
            if not math.isfinite(value) or value <= 0:
                shade = 0
            else:
                normalized = 1.0 - max(0.0, min(1.0, (float(value) - min_range) / span))
                shade = int(normalized * 255)
            pixels.extend((shade, shade, shade, 255))
        return {
            "width": width,
            "height": height,
            "format": "depth-bmp",
            "min_range_m": min_range,
            "max_range_m": max_range,
            "data_url": _bgra_bmp_data_url(bytes(pixels), width, height),
        }

    def _write_status(
        self,
        requested_mode: str,
        command: ConceptACommand,
        observation: BallObservationInput,
        depth_frame: np.ndarray | None,
        inventory: dict[str, float | int | None],
    ) -> None:
        now = time.time()
        if now - self.last_status_write_s < 0.2 and not self.collection_confirmed:
            return
        self.last_status_write_s = now

        x_m, y_m, z_m = self.robot_node.getPosition()
        target = self.survey_behavior.current_target()
        front_range_m = self._front_range_m(depth_frame)
        self.status_store.write(
            {
                "requested_mode": requested_mode,
                "actual_mode": self.control_mode,
                "collector_state": command.state.value,
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
                "observation": {
                    "visible": observation.visible,
                    "distance_m": None if math.isinf(observation.distance_m) else observation.distance_m,
                    "bearing_rad": observation.bearing_rad,
                    "bearing_deg": math.degrees(observation.bearing_rad),
                    "confidence": observation.confidence,
                    "robot_x_m": observation.robot_x_m,
                    "robot_y_m": observation.robot_y_m,
                    "world_x_m": observation.world_x_m,
                    "world_y_m": observation.world_y_m,
                },
                "command": {
                    "linear_speed_m_s": command.base.linear_speed_m_s,
                    "angular_speed_rad_s": command.base.angular_speed_rad_s,
                    "lift_wheel_speed": command.collector.lift_wheel_speed,
                    "intake_enabled": command.collector.intake_enabled,
                },
                "survey": {
                    "state": self.survey_behavior.state.value,
                    "waypoint_index": self.survey_behavior.waypoint_index,
                    "waypoint_count": len(self.survey_behavior.waypoints),
                    "target_x_m": None if target is None else target[0],
                    "target_y_m": None if target is None else target[1],
                    "front_range_m": front_range_m,
                },
                "collection_animation_active": self.collection_animation is not None,
            }
        )

    def _print_status(
        self,
        command: ConceptACommand,
        observation: BallObservationInput,
        depth_frame: np.ndarray | None,
    ) -> None:
        status = (
            f"mode={self.control_mode} "
            f"collector={command.state.value} "
            f"visible={observation.visible} "
            f"distance={observation.distance_m:.2f}m "
            f"bearing={math.degrees(observation.bearing_rad):.1f}deg "
            f"balls={self.collection_count}"
        )
        if observation.world_x_m is not None and observation.world_y_m is not None:
            status += f" ball_world=({observation.world_x_m:.2f},{observation.world_y_m:.2f})"
        if self.control_mode == "survey":
            x_m, y_m, _z_m = self.robot_node.getPosition()
            target = self.survey_behavior.current_target()
            target_text = "none" if target is None else f"({target[0]:.2f},{target[1]:.2f})"
            front_range = self._front_range_m(depth_frame)
            front_range_text = "none" if front_range is None else f"{front_range:.2f}m"
            status += (
                f" survey_state={self.survey_behavior.state.value} "
                f"waypoint={self.survey_behavior.waypoint_index + 1}/{len(self.survey_behavior.waypoints)} "
                f"pos=({x_m:.2f},{y_m:.2f}) "
                f"target={target_text} "
                f"front_range={front_range_text}"
            )
        print(status)

    def _simulate_collection(self, command: ConceptACommand) -> bool:
        if not command.collector.intake_enabled:
            return False
        if self.collection_animation is not None:
            return False

        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None:
                continue
            local_position = self._world_to_robot_local(ball.getPosition())
            if self._in_intake_zone(local_position):
                self._start_collection_animation(index, ball)
                return True
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
