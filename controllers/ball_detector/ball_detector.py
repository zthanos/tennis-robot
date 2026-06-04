"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import os
import time
from typing import TYPE_CHECKING

from ball_map import BallMap, BallMapConfig, across_net
from collect_one_mission import CollectOneMission
from status_reporter import StatusReporter
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
from debug_display import draw_debug as _draw_debug_frame
from lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m
from search import HalfCourtSearchBehavior, SearchState
from survey import CourtSurveyBehavior, SurveyState, SurveyVision
from mapping import LidarSurveyBoundaryProvider, MapLeftSideMission
from route_visualizer import ROUTE_VISUALIZATION_PRESET, WebotsRouteVisualizer
from telemetry import setup_telemetry, TelemetryJournal

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
        build_survey_vision,
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
INTAKE_ZONE_X_M = (0.50, 0.72)
# Wide intake roller accepts a wider lateral pickup envelope than the earlier centered wheel.
INTAKE_HALF_WIDTH_M = 0.16
INTAKE_MAX_HEIGHT_M = -0.08
SUPERVISED_FOV_RAD = 1.05
SUPERVISED_MAX_RANGE_M = 8.0
NET_X_M = 0.0
NET_SIDE_CLEARANCE_M = 0.25
COURT_MAX_X_M = 11.885
COURT_MAX_Y_M = 5.485
COURT_BALL_MARGIN_M = _env_float("COURT_BALL_MARGIN_M", 3.2)
COLLECTION_ANIMATION_S = 0.75
COLLECTION_PATH_LOCAL = (
    (0.66, 0.0, -0.165),
    (0.62, 0.0, -0.135),
    (0.54, 0.0, -0.120),
    (0.36, 0.0, -0.045),
    (0.17, 0.0, 0.120),
)
FRONT_CAMERA_MOUNT = CameraMount(x_m=0.535, y_m=0.0, yaw_rad=0.0) if VISION_ENABLED else None
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
SCAN_SIDE_DURATION_S = 12.0
COLLECT_PATTERN_COLLECTION_TIMEOUT_S = _env_float("COLLECT_PATTERN_COLLECTION_TIMEOUT_S", 35.0)
LIDAR_CANDIDATE_CONFIDENCE = 0.15


def _angle_delta_rad(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi






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
        self.telemetry_journal = TelemetryJournal(self.started_at)
        self.collection_complete_reported = False
        self.ball_map = BallMap(BallMapConfig(court_ball_margin_m=COURT_BALL_MARGIN_M))
        self.active_mapped_target_id: int | None = None
        self.collect_one_mission = CollectOneMission()
        self.scan_side_started_at: float | None = None
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s: float = 0.0
        self.collect_pattern_failures = 0
        self._map_completion_reported = False
        self._search_complete_reported = False
        self._survey_complete_reported = False
        self._last_survey_vision: SurveyVision | None = None
        self._last_debug_detection: BallDetection | None = None
        self._last_debug_observation: BallObservationInput | None = None
        self.status_reporter = StatusReporter(
            status_store=self.status_store,
            sensor_store=self.sensor_store,
            robot_node=self.robot_node,
            robot=self.robot,
            camera=self.camera,
            depth_camera=self.depth_camera,
            lidar=self.lidar,
            ir_intake_left=self.ir_intake_left,
            ir_intake_right=self.ir_intake_right,
            started_at=self.started_at,
            survey_behavior=self.survey_behavior,
            search_behavior=self.search_behavior,
            behavior=self.behavior,
            ball_map=self.ball_map,
            collect_one_mission=self.collect_one_mission,
            map_mission=self._map_mission,
            route_visualizer=self.route_visualizer,
            telemetry=self.telemetry,
            telemetry_journal=self.telemetry_journal,
            lidar_front_index_ratio=LIDAR_FRONT_INDEX_RATIO,
            lidar_front_min_obstacle_range_m=LIDAR_FRONT_MIN_OBSTACLE_RANGE_M,
            ir_intake_trigger_threshold=IR_INTAKE_TRIGGER_THRESHOLD,
        )

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
        self._record_event(
            "controller_started",
            telemetry_enabled=self.telemetry.enabled,
            vision_enabled=VISION_ENABLED,
            route_visualization_enabled=self.route_visualizer.enabled,
        )

    def _record_event(self, event_type: str, severity: str = "info", **fields: object) -> None:
        self.telemetry_journal.record(
            event_type,
            severity,
            mode=self.control_mode,
            loop_count=self.loop_count,
            **fields,
        )

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
                now = time.time()
                mapped_ball_id, is_new_ball = self.ball_map.update(mapped_observation, now)
                if is_new_ball and mapped_observation.world_x_m is not None:
                    self._record_event(
                        "target_observed",
                        target_id=mapped_ball_id,
                        x_m=round(mapped_observation.world_x_m, 3),
                        y_m=round(mapped_observation.world_y_m, 3),
                        distance_m=round(mapped_observation.distance_m, 3),
                        source=mapped_observation.source,
                    )
                if self.loop_count % 90 == 0:
                    self.ball_map.prune_phantoms(now)
                control_command = self.command_store.read()
                inventory = self._ball_inventory()
                effective_mode = self._effective_control_mode(control_command.mode)
                control_observation = self._control_observation_for_mode(
                    effective_mode,
                    mapped_observation,
                    mapped_ball_id,
                )
                if effective_mode == "map_court":
                    command = self._survey_command_for_mode(effective_mode, image)
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
                self._last_debug_detection = detection
                self._last_debug_observation = control_observation
                _draw_debug_frame(
                    image, detection, command, control_observation,
                    self.display, self.collection_count, self.control_mode,
                    self.survey_behavior, self._last_survey_vision,
                )
                self._apply_command(command)
                self.telemetry.add_collector_state(command.state.value)
                self.collection_confirmed = self._check_collection(command)
                if self.collection_confirmed:
                    rx, ry, _ = self.robot_node.getPosition()
                    collected_id = self.ball_map.mark_nearest_collected(rx, ry, time.time())
                    if collected_id is not None:
                        ball = self.ball_map.balls.get(collected_id)
                        self._record_event(
                            "target_collected",
                            target_id=collected_id,
                            x_m=round(ball.x_m, 3) if ball else None,
                            y_m=round(ball.y_m, 3) if ball else None,
                            total_collected=self.collection_count,
                        )
                        if self.active_mapped_target_id == collected_id:
                            self.active_mapped_target_id = None
                    self.route_visualizer.refresh()
                self.status_reporter.write_status(
                    requested_mode=control_command.mode,
                    control_mode=self.control_mode,
                    command=command,
                    observation=control_observation,
                    detection=detection,
                    inventory=inventory,
                    collection_count=self.collection_count,
                    loop_count=self.loop_count,
                    collection_confirmed=self.collection_confirmed,
                    collection_complete_reported=self.collection_complete_reported,
                    collect_pattern_phase=self.collect_pattern_phase,
                    collect_pattern_failures=self.collect_pattern_failures,
                    collect_pattern_collect_elapsed_s=self.collect_pattern_collect_elapsed_s,
                    scan_side_started_at=self.scan_side_started_at,
                    active_mapped_target_id=self.active_mapped_target_id,
                    collection_animation=self.collection_animation,
                    last_survey_vision=self._last_survey_vision,
                    last_debug_detection=self._last_debug_detection,
                    last_debug_observation=self._last_debug_observation,
                    collect_pattern_collection_timeout_s=COLLECT_PATTERN_COLLECTION_TIMEOUT_S,
                    scan_side_duration_s=SCAN_SIDE_DURATION_S,
                    vision_enabled=VISION_ENABLED,
                )
                self.status_reporter.write_sensor_snapshots(
                    lidar_ranges=self._lidar_ranges(),
                    ir_left=self.ir_intake_left.getValue() if self.ir_intake_left is not None else None,
                    ir_right=self.ir_intake_right.getValue() if self.ir_intake_right is not None else None,
                    last_debug_detection=self._last_debug_detection,
                    last_debug_observation=self._last_debug_observation,
                    control_mode=self.control_mode,
                    last_survey_vision=self._last_survey_vision,
                )
                self.last_command = command
                self.loop_count += 1
                if self.loop_count % 60 == 0:
                    self.status_reporter.print_status(
                        control_mode=self.control_mode,
                        command=command,
                        observation=control_observation,
                        collection_count=self.collection_count,
                        collect_pattern_phase=self.collect_pattern_phase,
                    )
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _on_mode_changed(self, new_mode: str) -> bool:
        """Reset all shared behaviors when mode changes. Returns True if mode actually changed."""
        if new_mode == self.control_mode:
            return False
        previous_mode = self.control_mode
        self.behavior.reset()
        self.search_behavior.reset()
        self.survey_behavior.reset()
        if self.control_mode == "map_left_side" and not self._map_mission.complete:
            self._map_mission.reset()
        self.control_mode = new_mode
        self.collect_one_mission.reset()
        self._reset_collect_pattern()
        self.scan_side_started_at = None
        self._collect_start_time = None
        self.active_mapped_target_id = None
        self.route_visualizer.clear()
        print(f"control mode changed to {self.control_mode}")
        self._record_event("mode_changed", previous_mode=previous_mode, requested_mode=new_mode)
        return True

    def _effective_control_mode(self, requested_mode: str) -> str:
        if requested_mode == "collect":
            elapsed = 0.0 if self._collect_start_time is None else time.time() - self._collect_start_time
            min_scan_time = self.behavior.config.scan_full_turn_s
            if self.ball_map.all_collected(time.time()) and elapsed > min_scan_time and self.collection_animation is None:
                if not self.collection_complete_reported:
                    print(f"collection complete; total={self.collection_count}")
                    self.command_store.write("idle", source="webots-complete")
                    self.collection_complete_reported = True
                return "idle"
            if not self.ball_map.all_collected(time.time()):
                self.collection_complete_reported = False
        if requested_mode != "collect_one":
            self.collect_one_mission._complete_reported = False
        return requested_mode

    def _collector_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if self._on_mode_changed(mode):
            if mode == "collect":
                self.ball_map.reset()
                self._collect_start_time = time.time()
                self.route_visualizer.refresh()
                self._record_event("collection_started")
            elif mode == "collect_one":
                self.ball_map.reset()
                self.collect_one_mission.start(self._robot_pose_2d())
                self._collect_start_time = time.time()

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
            cmd = self.collect_one_mission.update(
                observation, self.collection_confirmed, TIME_STEP_MS / 1000,
                self._robot_pose_2d(), self.behavior,
            )
            if self.collect_one_mission.is_done and not self.collect_one_mission._complete_reported:
                print(f"collect one complete; total={self.collection_count}")
                self.command_store.write("idle", source="webots-collect-one-complete")
                self.collect_one_mission._complete_reported = True
            return cmd
        if mode == "scan_side":
            return self._scan_side_command()

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _search_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self._search_complete_reported = False

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
            if not self._search_complete_reported:
                self._record_event("search_complete", coverage_pct=search_command.coverage_pct)
                self._search_complete_reported = True
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
        if self._on_mode_changed(mode):
            self.search_behavior.max_interrupt_distance_m = COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            self.ball_map.reset()
            seeded = self._seed_mapped_balls_from_map_mission()
            if seeded > 0:
                rx, ry, _ = self.robot_node.getPosition()
                self.active_mapped_target_id = self.ball_map.nearest_target_id(rx, ry, time.time())
            self._collect_start_time = time.time()
            self.collection_complete_reported = False
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            self.collect_pattern_failures = 0
            self.route_visualizer.refresh()
            self._record_event("collect_pattern_started", seeded_targets=len(self.ball_map))

        if self.collect_pattern_phase == "collect":
            return self._collect_pattern_collect_command(observation, mapped_ball_id)

        x_m, y_m, _z_m = self.robot_node.getPosition()
        yaw = self._robot_yaw_rad()
        now = time.time()
        mapped_search_id = mapped_ball_id or self.ball_map.nearest_target_id(x_m, y_m, now)
        search_observation = observation
        if not search_observation.visible and mapped_search_id is not None:
            search_observation = self.ball_map.observation_from_target(mapped_search_id, x_m, y_m, yaw, now) or search_observation
        search_command = self.search_behavior.update(
            x_m, y_m, yaw, search_observation, self._front_range_m(), TIME_STEP_MS / 1000,
            target_id=mapped_search_id,
        )
        if search_command.state == SearchState.COMPLETE:
            if not self.collection_complete_reported:
                print(f"collect pattern complete; total={self.collection_count}")
                self.collection_complete_reported = True
                self._record_event(
                    "collect_pattern_complete",
                    collected=self.collection_count,
                    failures=self.collect_pattern_failures,
                )
            self.command_store.write("idle", source="webots-collect-pattern-complete")

        if search_command.state == SearchState.BALL_DETECTED:
            trigger_obs = search_observation if search_observation.visible else self.ball_map.observation_from_target(mapped_search_id, x_m, y_m, yaw, now)
            has_confirmed_target = mapped_search_id is not None
            close_enough = trigger_obs is not None and trigger_obs.distance_m <= COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            if trigger_obs is not None and trigger_obs.visible and (close_enough or has_confirmed_target):
                self.collect_pattern_phase = "collect"
                self.collect_pattern_collect_elapsed_s = 0.0
                self.active_mapped_target_id = mapped_search_id
                self.behavior.reset()
                self.behavior.start_tracking(trigger_obs)
                self._record_event(
                    "target_locked",
                    target_id=mapped_search_id,
                    distance_m=round(trigger_obs.distance_m, 3),
                    source=trigger_obs.source,
                    reason="collect_pattern_ball_detected",
                )
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
            self._record_event("collection_attempt_succeeded", target_id=self.active_mapped_target_id)
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
            self._record_event(
                "collection_attempt_failed",
                "warning",
                target_id=self.active_mapped_target_id,
                reason="timeout",
                elapsed_s=round(self.collect_pattern_collect_elapsed_s, 3),
                failures=self.collect_pattern_failures,
            )
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
            if self.active_mapped_target_id is not None:
                self.ball_map.set_state(self.active_mapped_target_id, "collection_failed")
                print(
                    f"collect pattern gave up on target {self.active_mapped_target_id} "
                    f"after {self.behavior.capture_attempts} attempt(s)"
                )
            self.collect_pattern_failures += 1
            self._record_event(
                "collection_attempt_failed",
                "warning",
                target_id=self.active_mapped_target_id,
                reason="collector_gave_up",
                attempts=self.behavior.capture_attempts,
                failures=self.collect_pattern_failures,
            )
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
        rx, ry, _ = self.robot_node.getPosition()
        yaw = self._robot_yaw_rad()
        now = time.time()
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(rx, ry, now)
        if observation.visible and mapped_ball_id is not None and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self.ball_map.observation_from_target(self.active_mapped_target_id, rx, ry, yaw, now)
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
        if across_net(robot_x_m, observation.world_x_m, NET_X_M, NET_SIDE_CLEARANCE_M):
            return BallObservationInput(visible=False, source="across_net_filtered")
        if (
            abs(observation.world_x_m) > COURT_MAX_X_M + COURT_BALL_MARGIN_M
            or observation.world_y_m is None
            or abs(observation.world_y_m) > COURT_MAX_Y_M + COURT_BALL_MARGIN_M
        ):
            return BallObservationInput(visible=False, source="out_of_court_filtered")
        return observation

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

    def _seed_mapped_balls_from_map_mission(self) -> int:
        if not self._map_mission.complete or not self._map_mission.candidates:
            return 0
        return self.ball_map.seed_from_candidates(self._map_mission.candidates, time.time())

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
        if self._on_mode_changed(mode):
            self._map_completion_reported = False
            robot_x, robot_y, _ = self.robot_node.getPosition()
            self._map_mission.start(robot_x, robot_y, self._robot_yaw_rad())
            print(
                f"map_left_side started; side={self._map_mission.bounds.side}"
                f" center=({self._map_mission.bounds.center_x:.1f}, 0)"
            )
            self._record_event(
                "map_left_side_started",
                side=self._map_mission.bounds.side,
                center_x_m=round(self._map_mission.bounds.center_x, 3),
            )

        robot_x, robot_y, _ = self.robot_node.getPosition()
        command = self._map_mission.update(robot_x, robot_y, self._robot_yaw_rad(), TIME_STEP_MS / 1000)

        if self._map_mission.complete and not self._map_completion_reported:
            grid = self._map_mission.grid
            seeded = self._seed_mapped_balls_from_map_mission()
            if seeded > 0:
                self.active_mapped_target_id = self.ball_map.nearest_target_id(robot_x, robot_y, time.time())
            print(
                f"map_left_side complete; candidates={len(self._map_mission.candidates)}"
                f" seeded={seeded} grid={grid}"
            )
            self._record_event(
                "map_left_side_complete",
                candidates=len(self._map_mission.candidates),
                seeded_targets=seeded,
                grid=grid,
            )
            self.route_visualizer.refresh()
            self.command_store.write("idle", source="webots-map-complete")
            self._map_completion_reported = True

        return command

    def _survey_command_for_mode(self, mode: str, image: np.ndarray | None = None) -> ConceptACommand:
        if not VISION_ENABLED or image is None:
            raise RuntimeError("map_court mode requires RGB vision — set USE_RGB_VISION=true")
        if self._on_mode_changed(mode):
            self._survey_complete_reported = False
            self._record_event("map_court_started")

        x_m, y_m, _z_m = self.robot_node.getPosition()
        self._last_survey_vision = self._survey_vision_summary(image)
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
                status = bounds.get("status", "FAILED")
                if status == "SUCCESS":
                    court = bounds.get("court_geometry") or {}
                    external = bounds.get("external_boundary_map") or {}
                    free_space = bounds.get("free_space_between_court_lines_and_fences") or {}
                    self._record_event(
                        "map_court_complete",
                        status=status,
                        point_count=pt_count,
                        length_m=court.get("length_m"),
                        width_m=court.get("width_m"),
                        external_candidates=len(external.get("candidates") or []),
                    )
                    print(
                        f"map court complete — "
                        f"L={court.get('length_m', 0.0):.2f}m  "
                        f"W={court.get('width_m', 0.0):.2f}m  "
                        f"external={len(external.get('candidates') or [])}  "
                        f"free_avg={free_space.get('avg_m') or 0.0:.2f}m  "
                        f"pts={pt_count}"
                    )
                else:
                    self._record_event(
                        "map_court_failed",
                        "error",
                        status=status,
                        point_count=pt_count,
                        reason=bounds.get("failure_reason") or "survey incomplete",
                    )
                    print(
                        f"map court FAILED — {bounds.get('failure_reason') or 'survey incomplete'} "
                        f"pts={pt_count}"
                    )
                self.command_store.write("idle", source="robot-map-court-complete")
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

    def _survey_vision_summary(self, frame: np.ndarray) -> SurveyVision:
        """Build SurveyVision from camera + depth. Requires VISION_ENABLED."""
        depth = self._depth_frame_m()
        depth_min = float(self.depth_camera.getMinRange()) if self.depth_camera is not None else 0.1
        depth_max = float(self.depth_camera.getMaxRange()) if self.depth_camera is not None else 10.0
        return build_survey_vision(frame, depth, depth_min, depth_max)

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
            if across_net(robot_world_x, ball_position[0], NET_X_M, NET_SIDE_CLEARANCE_M):
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
            if across_net(robot_world_x, ball_position[0], NET_X_M, NET_SIDE_CLEARANCE_M):
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

    def _nearest_lidar_candidate_observation(self) -> BallObservationInput | None:
        """Return the nearest LiDAR ball candidate as a synthetic observation, or None."""
        candidates = extract_ball_candidates(self._lidar_ranges() or [])
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
        rx, ry, _ = self.robot_node.getPosition()
        yaw = self._robot_yaw_rad()
        now = time.time()
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(rx, ry, now)
        # Prefer live camera/depth over stored position when the camera currently sees
        # the active target; the live bearing is fresher and corrects map drift.
        if observation.visible and mapped_ball_id is not None and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self.ball_map.observation_from_target(self.active_mapped_target_id, rx, ry, yaw, now)
        if locked is not None:
            return locked
        self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(rx, ry, now)
        return self.ball_map.observation_from_target(self.active_mapped_target_id, rx, ry, yaw, now) or observation

    def _apply_command(self, command: ConceptACommand) -> None:
        self.set_base_command(command.base.linear_speed_m_s, command.base.angular_speed_rad_s)
        self.set_collector_command(command.collector.lift_wheel_speed)

    def _front_range_m(self) -> float | None:
        return lidar_front_range_m(self._lidar_ranges() or [], LIDAR_FRONT_INDEX_RATIO, LIDAR_FRONT_MIN_OBSTACLE_RANGE_M)

    def _robot_yaw_rad(self) -> float:
        orientation = self.robot_node.getOrientation()
        return math.atan2(orientation[3], orientation[0])

    def _robot_pose_2d(self) -> tuple[float, float, float]:
        x_m, y_m, _z_m = self.robot_node.getPosition()
        return (x_m, y_m, self._robot_yaw_rad())

    def _lidar_ranges(self) -> list[float] | None:
        if self.lidar is None:
            return None
        return [float(value) for value in self.lidar.getRangeImage()]

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
        self._record_event(
            "ball_collected",
            ball_def=f"TENNIS_BALL_{index:02d}",
            total_collected=self.collection_count,
        )

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
