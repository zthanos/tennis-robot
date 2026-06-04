"""ROS 2 controller node: orchestrates all robot behavior.

This node contains all logic previously in BallDetectorController (ball_detector.py),
with every direct Webots API call replaced by a ROS 2 topic read or write.
The underlying behavior modules (collector.py, survey.py, ball_map.py, etc.)
are imported unchanged from the controllers/ tree.

Subscribes:
  /ball/observation  (tennis_robot_msgs/BallObservation)
  /survey/vision     (std_msgs/String, JSON)
  /scan              (sensor_msgs/LaserScan)
  /odom              (nav_msgs/Odometry)
  /ir/readings       (tennis_robot_msgs/IrReadings)
  /robot/command     (tennis_robot_msgs/RobotCommand)
  /sim/balls         (std_msgs/String, JSON) — sim-only ground truth

Publishes:
  /cmd_vel           (geometry_msgs/Twist)
  /collector/cmd     (tennis_robot_msgs/CollectorCmd)
  /robot/status      (std_msgs/String, JSON)
  /ball/collected    (std_msgs/String, ball def name) — triggers sim animation
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import asdict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from tennis_robot import yaw_from_quaternion

sys.path.insert(0, "/workspace/controllers/ball_detector")

from ball_map import BallMap, BallMapConfig, across_net
from collect_one_mission import CollectOneMission
from collector import (
    BallObservationInput,
    BaseCommand,
    CollectorCommand,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptACommand,
    ConceptAConfig,
)
from config_utils import _env_float
from lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m
from mapping import LidarSurveyBoundaryProvider, MapLeftSideMission
from search import HalfCourtSearchBehavior, SearchState
try:
    from tennis_robot.lidar_survey import LidarSurveyState, Ros2LidarCourtSurvey
except ModuleNotFoundError:
    from lidar_survey import LidarSurveyState, Ros2LidarCourtSurvey
from survey import SurveyVision
from tennis_robot_msgs.msg import BallObservation, CollectorCmd, IrReadings, RobotCommand

TIME_STEP_S = 0.032
NET_X_M = 0.0
NET_SIDE_CLEARANCE_M = 0.25
COURT_MAX_X_M = 11.885
COURT_MAX_Y_M = 5.485
COURT_BALL_MARGIN_M = _env_float("COURT_BALL_MARGIN_M", 3.2)
INTAKE_ZONE_X_M = (0.50, 0.72)
INTAKE_HALF_WIDTH_M = 0.16
INTAKE_MAX_HEIGHT_M = -0.08
IR_INTAKE_TRIGGER_THRESHOLD = 500.0
SCAN_SIDE_DURATION_S = 12.0
COLLECT_PATTERN_COLLECTION_TIMEOUT_S = _env_float("COLLECT_PATTERN_COLLECTION_TIMEOUT_S", 35.0)
MAPPED_BALL_MERGE_DISTANCE_M = 0.65
MAPPED_BALL_MAX_MERGE_DISTANCE_M = 1.6
MAPPED_BALL_MIN_SEEN_COUNT = 5
MAPPED_BALL_MAX_CREATE_DISTANCE_M = 3.0
COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M = _env_float(
    "COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M", MAPPED_BALL_MAX_CREATE_DISTANCE_M
)
MAPPED_BALL_STALE_AFTER_S = 45.0
LIDAR_FRONT_INDEX_RATIO = max(0.0, min(1.0, _env_float("LIDAR_FRONT_INDEX_RATIO", 0.5)))
LIDAR_FRONT_MIN_OBSTACLE_RANGE_M = _env_float("LIDAR_FRONT_MIN_OBSTACLE_RANGE_M", 0.18)
LIDAR_CANDIDATE_CONFIDENCE = 0.15


def _angle_delta_rad(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _survey_vision_from_json(payload: str) -> SurveyVision:
    try:
        d = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return SurveyVision()
    return SurveyVision(
        line_detected=bool(d.get("line_detected", False)),
        line_offset_m=d.get("line_offset_m"),
        line_heading_error_rad=d.get("line_heading_error_rad"),
        line_confidence=float(d.get("line_confidence", 0.0)),
        corner_detected=bool(d.get("corner_detected", False)),
        corner_confidence=float(d.get("corner_confidence", 0.0)),
        center_m=d.get("center_m"),
        left_m=d.get("left_m"),
        right_m=d.get("right_m"),
        valid_count=int(d.get("valid_count", 0)),
        obstacle_class=d.get("obstacle_class"),
    )


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("tennis_robot_controller")

        # ── behavior modules (unchanged) ──────────────────────────────────────
        self.behavior = ConceptACollectorBehavior(ConceptAConfig.from_env())
        self.search_behavior = HalfCourtSearchBehavior.from_env()
        self.survey_behavior = Ros2LidarCourtSurvey.from_env()
        self.ball_map = BallMap(BallMapConfig(court_ball_margin_m=COURT_BALL_MARGIN_M))
        self.collect_one_mission = CollectOneMission()
        self._map_mission = MapLeftSideMission(
            LidarSurveyBoundaryProvider(), self._map_supervisor_balls
        )

        # ── state ──────────────────────────────────────────────────────────────
        self.control_mode = "idle"
        self.collection_count = 0
        self.loop_count = 0
        self.started_at = time.time()
        self.active_mapped_target_id: int | None = None
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s = 0.0
        self.collect_pattern_failures = 0
        self.collection_confirmed = False
        self._collect_start_time: float | None = None
        self.scan_side_started_at: float | None = None
        self._collection_complete_reported = False
        self._search_complete_reported = False
        self._survey_complete_reported = False
        self._map_completion_reported = False
        self._last_survey_log_key: tuple[str, str] | None = None

        # ── cached topic values ────────────────────────────────────────────────
        self._latest_obs = BallObservationInput(visible=False, source="startup")
        self._latest_survey_vision: SurveyVision | None = None
        self._lidar_ranges: list[float] | None = None
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._ir_left = 0.0
        self._ir_right = 0.0
        self._control_command_mode = "idle"
        self._control_command_source = "startup"
        self._sim_balls: list[dict] = []

        # ── subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(BallObservation, "/ball/observation", self._on_observation, 1)
        self.create_subscription(String, "/survey/vision", self._on_survey_vision, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 1)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(IrReadings, "/ir/readings", self._on_ir, 10)
        self.create_subscription(RobotCommand, "/robot/command", self._on_command, 10)
        self.create_subscription(String, "/sim/balls", self._on_sim_balls, 1)

        # ── publishers ─────────────────────────────────────────────────────────
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self._pub_collector = self.create_publisher(CollectorCmd, "/collector/cmd", 1)
        self._pub_status = self.create_publisher(String, "/robot/status", 10)
        self._pub_ball_collected = self.create_publisher(String, "/ball/collected", 10)
        self._pub_command = self.create_publisher(RobotCommand, "/robot/command", 10)

        self.create_timer(TIME_STEP_S, self._step)
        self.get_logger().info("tennis_robot_controller started")

    # ── subscription callbacks (cache only) ────────────────────────────────────

    def _on_observation(self, msg: BallObservation) -> None:
        self._latest_obs = BallObservationInput(
            visible=msg.visible,
            bearing_rad=msg.bearing_rad,
            distance_m=msg.distance_m,
            confidence=msg.confidence,
            source=msg.source,
            world_x_m=msg.world_x_m if msg.visible else None,
            world_y_m=msg.world_y_m if msg.visible else None,
            robot_x_m=msg.robot_x_m if msg.visible else None,
            robot_y_m=msg.robot_y_m if msg.visible else None,
        )

    def _on_survey_vision(self, msg: String) -> None:
        self._latest_survey_vision = _survey_vision_from_json(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        self._lidar_ranges = [float(r) for r in msg.ranges]
        self._lidar_angle_min = float(msg.angle_min)
        self._lidar_angle_increment = float(msg.angle_increment)

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_ir(self, msg: IrReadings) -> None:
        self._ir_left = msg.left
        self._ir_right = msg.right

    def _on_command(self, msg: RobotCommand) -> None:
        self._control_command_mode = msg.mode
        self._control_command_source = msg.source

    def _on_sim_balls(self, msg: String) -> None:
        try:
            self._sim_balls = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._sim_balls = []

    # ── main step (runs at TIME_STEP_S Hz) ─────────────────────────────────────

    def _step(self) -> None:
        observation = self._latest_obs
        now = time.time()
        mapped_observation = self._mapping_observation(observation)
        mapped_ball_id, is_new_ball = self.ball_map.update(mapped_observation, now)

        if self.loop_count % 90 == 0:
            self.ball_map.prune_phantoms(now)

        effective_mode = self._effective_control_mode(self._control_command_mode)
        control_observation = self._control_observation_for_mode(
            effective_mode, mapped_observation, mapped_ball_id
        )

        if effective_mode == "map_court":
            command = self._survey_command_for_mode(effective_mode)
        elif effective_mode == "search":
            command = self._search_command_for_mode(
                effective_mode, self._same_side_search_observation(control_observation)
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

        self.collection_confirmed = self._check_collection(command)
        if self.collection_confirmed:
            collected_id = self.ball_map.mark_nearest_collected(
                self._robot_x, self._robot_y, now
            )
            if collected_id == self.active_mapped_target_id:
                self.active_mapped_target_id = None

        self._apply_command(command)
        self._publish_status(command, control_observation)
        self.loop_count += 1

    # ── mode orchestration (logic unchanged from BallDetectorController) ───────

    def _on_mode_changed(self, new_mode: str) -> bool:
        if new_mode == self.control_mode:
            return False
        self.behavior.reset()
        self.search_behavior.reset()
        if not (self.control_mode == "map_court" and new_mode == "idle"):
            self.survey_behavior.reset()
        if self.control_mode == "map_left_side" and not self._map_mission.complete:
            self._map_mission.reset()
        self.control_mode = new_mode
        self.collect_one_mission.reset()
        self._reset_collect_pattern()
        self.scan_side_started_at = None
        self._collect_start_time = None
        self.active_mapped_target_id = None
        self.get_logger().info(f"mode → {new_mode}")
        return True

    def _effective_control_mode(self, requested_mode: str) -> str:
        if requested_mode == "collect":
            elapsed = 0.0 if self._collect_start_time is None else time.time() - self._collect_start_time
            if (
                self.ball_map.all_collected(time.time())
                and elapsed > self.behavior.config.scan_full_turn_s
            ):
                if not self._collection_complete_reported:
                    self.get_logger().info(f"collection complete; total={self.collection_count}")
                    self._publish_command("idle", "controller-complete")
                    self._collection_complete_reported = True
                return "idle"
            if not self.ball_map.all_collected(time.time()):
                self._collection_complete_reported = False
        if requested_mode != "collect_one":
            self.collect_one_mission._complete_reported = False
        return requested_mode

    def _collector_command_for_mode(
        self, mode: str, observation: BallObservationInput
    ) -> ConceptACommand:
        if self._on_mode_changed(mode):
            if mode == "collect":
                self.ball_map.reset()
                self._collect_start_time = time.time()
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
            return self.behavior.update(observation, TIME_STEP_S, self.collection_confirmed)

        if mode == "collect_one":
            cmd = self.collect_one_mission.update(
                observation, self.collection_confirmed, TIME_STEP_S,
                self._robot_pose_2d(), self.behavior,
            )
            if self.collect_one_mission.is_done and not self.collect_one_mission._complete_reported:
                self.get_logger().info(f"collect_one complete; total={self.collection_count}")
                self._publish_command("idle", "controller-collect-one-complete")
                self.collect_one_mission._complete_reported = True
            return cmd

        if mode == "scan_side":
            return self._scan_side_command()

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _search_command_for_mode(
        self, mode: str, observation: BallObservationInput
    ) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self._search_complete_reported = False
        search_command = self.search_behavior.update(
            self._robot_x, self._robot_y, self._robot_yaw,
            observation, self._front_range_m(), TIME_STEP_S,
            target_id=self.active_mapped_target_id,
        )
        if search_command.state == SearchState.COMPLETE:
            if not self._search_complete_reported:
                self._search_complete_reported = True
                self._publish_command("idle", "controller-search-complete")
        return ConceptACommand(
            state=CollectorState.SURVEY
            if search_command.state in {SearchState.SURVEY_VIEWPOINT, SearchState.TRANSIT_TO_ZONE, SearchState.LOCAL_SCAN}
            else CollectorState.SCAN,
            base=search_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _collect_pattern_command_for_mode(
        self, mode: str, observation: BallObservationInput, mapped_ball_id: int | None
    ) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self.search_behavior.max_interrupt_distance_m = COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            self.ball_map.reset()
            seeded = self._seed_mapped_balls_from_map_mission()
            if seeded > 0:
                self.active_mapped_target_id = self.ball_map.nearest_target_id(
                    self._robot_x, self._robot_y, time.time()
                )
            self._collect_start_time = time.time()
            self._collection_complete_reported = False
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            self.collect_pattern_failures = 0

        if self.collect_pattern_phase == "collect":
            return self._collect_pattern_collect_command(observation, mapped_ball_id)

        now = time.time()
        mapped_search_id = mapped_ball_id or self.ball_map.nearest_target_id(
            self._robot_x, self._robot_y, now
        )
        search_observation = observation
        if not search_observation.visible and mapped_search_id is not None:
            search_observation = (
                self.ball_map.observation_from_target(
                    mapped_search_id, self._robot_x, self._robot_y, self._robot_yaw, now
                ) or search_observation
            )
        search_command = self.search_behavior.update(
            self._robot_x, self._robot_y, self._robot_yaw,
            search_observation, self._front_range_m(), TIME_STEP_S,
            target_id=mapped_search_id,
        )
        if search_command.state == SearchState.COMPLETE:
            if not self._collection_complete_reported:
                self.get_logger().info(f"collect_pattern complete; total={self.collection_count}")
                self._collection_complete_reported = True
            self._publish_command("idle", "controller-collect-pattern-complete")

        if search_command.state == SearchState.BALL_DETECTED:
            trigger_obs = search_observation if search_observation.visible else (
                self.ball_map.observation_from_target(
                    mapped_search_id, self._robot_x, self._robot_y, self._robot_yaw, now
                )
            )
            has_confirmed_target = mapped_search_id is not None
            close_enough = trigger_obs is not None and trigger_obs.distance_m <= COLLECT_PATTERN_MAX_APPROACH_DISTANCE_M
            if trigger_obs is not None and trigger_obs.visible and (close_enough or has_confirmed_target):
                self.collect_pattern_phase = "collect"
                self.collect_pattern_collect_elapsed_s = 0.0
                self.active_mapped_target_id = mapped_search_id
                self.behavior.reset()
                self.behavior.start_tracking(trigger_obs)
                return self.behavior.update(trigger_obs, TIME_STEP_S, collection_confirmed=False)

        return ConceptACommand(
            state=CollectorState.SURVEY
            if search_command.state in {SearchState.SURVEY_VIEWPOINT, SearchState.TRANSIT_TO_ZONE, SearchState.LOCAL_SCAN}
            else CollectorState.SCAN,
            base=search_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _collect_pattern_collect_command(
        self, observation: BallObservationInput, mapped_ball_id: int | None
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

        self.collect_pattern_collect_elapsed_s += TIME_STEP_S
        if self.collect_pattern_collect_elapsed_s > COLLECT_PATTERN_COLLECTION_TIMEOUT_S:
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

        target_obs = self._collect_pattern_target_observation(observation, mapped_ball_id)
        if self.behavior.state == CollectorState.SCAN and target_obs.visible:
            self.behavior.start_tracking(target_obs)
        cmd = self.behavior.update(target_obs, TIME_STEP_S, collection_confirmed=False)
        if self.behavior.gave_up:
            if self.active_mapped_target_id is not None:
                self.ball_map.set_state(self.active_mapped_target_id, "collection_failed")
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
        self, observation: BallObservationInput, mapped_ball_id: int | None
    ) -> BallObservationInput:
        now = time.time()
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(
                self._robot_x, self._robot_y, now
            )
        if observation.visible and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self.ball_map.observation_from_target(
            self.active_mapped_target_id, self._robot_x, self._robot_y, self._robot_yaw, now
        )
        if locked is not None:
            return locked
        if observation.visible:
            self.active_mapped_target_id = mapped_ball_id or self.active_mapped_target_id
            return observation
        return BallObservationInput(visible=False, source="collect_pattern_no_target")

    def _survey_command_for_mode(self, mode: str) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self._survey_complete_reported = False

        sv = self._latest_survey_vision or SurveyVision()
        survey_command = self.survey_behavior.update(
            self._robot_x, self._robot_y, self._robot_yaw,
            self._lidar_ranges, TIME_STEP_S, sv,
            lidar_angle_min=self._lidar_angle_min,
            lidar_angle_increment=self._lidar_angle_increment,
        )
        self._log_survey_progress()
        if survey_command.state == LidarSurveyState.DONE:
            if not self._survey_complete_reported:
                self._survey_complete_reported = True
                self._publish_command("idle", "controller-survey-complete")
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

    def _log_survey_progress(self) -> None:
        nav = self.survey_behavior.telemetry()
        key = (str(nav.get("state")), str(nav.get("last_event")))
        if key == self._last_survey_log_key:
            return
        self._last_survey_log_key = key
        self.get_logger().info(
            "map_court "
            f"state={nav.get('state')} event={nav.get('last_event')} "
            f"points={nav.get('map_point_count')} net={'yes' if nav.get('net_boundary') else 'no'} "
            f"target={nav.get('active_target')} dist={nav.get('distance_to_target_m')}"
        )

    def _map_mission_command_for_mode(self, mode: str) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self._map_completion_reported = False
            self._map_mission.start(self._robot_x, self._robot_y, self._robot_yaw)

        command = self._map_mission.update(
            self._robot_x, self._robot_y, self._robot_yaw, TIME_STEP_S
        )
        if self._map_mission.complete and not self._map_completion_reported:
            seeded = self._seed_mapped_balls_from_map_mission()
            if seeded > 0:
                self.active_mapped_target_id = self.ball_map.nearest_target_id(
                    self._robot_x, self._robot_y, time.time()
                )
            self.get_logger().info(
                f"map_left_side complete; candidates={len(self._map_mission.candidates)} seeded={seeded}"
            )
            self._publish_command("idle", "controller-map-complete")
            self._map_completion_reported = True
        return command

    def _scan_side_command(self) -> ConceptACommand:
        now = time.time()
        if self.scan_side_started_at is None:
            self.scan_side_started_at = now
        if now - self.scan_side_started_at >= SCAN_SIDE_DURATION_S:
            self._publish_command("idle", "controller-scan-complete")
            self.scan_side_started_at = None
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    # ── helpers ─────────────────────────────────────────────────────────────────

    def _robot_pose_2d(self) -> tuple[float, float, float]:
        return (self._robot_x, self._robot_y, self._robot_yaw)

    def _front_range_m(self) -> float | None:
        return lidar_front_range_m(
            self._lidar_ranges or [], LIDAR_FRONT_INDEX_RATIO, LIDAR_FRONT_MIN_OBSTACLE_RANGE_M
        )

    def _nearest_lidar_candidate_observation(self) -> BallObservationInput | None:
        candidates = extract_ball_candidates(self._lidar_ranges or [])
        if not candidates:
            return None
        cx, cy = min(candidates, key=lambda c: math.hypot(c[0], c[1]))
        distance_m = math.hypot(cx, cy)
        bearing_rad = math.atan2(cy, cx)
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        return BallObservationInput(
            visible=True,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
            confidence=LIDAR_CANDIDATE_CONFIDENCE,
            source="lidar_candidate",
            robot_x_m=cx,
            robot_y_m=cy,
            world_x_m=self._robot_x + cos_yaw * cx - sin_yaw * cy,
            world_y_m=self._robot_y + sin_yaw * cx + cos_yaw * cy,
        )

    def _mapping_observation(self, observation: BallObservationInput) -> BallObservationInput:
        return observation

    def _same_side_search_observation(
        self, observation: BallObservationInput
    ) -> BallObservationInput:
        if not observation.visible or observation.world_x_m is None:
            return observation
        if across_net(self._robot_x, observation.world_x_m, NET_X_M, NET_SIDE_CLEARANCE_M):
            return BallObservationInput(visible=False, source="across_net_filtered")
        if (
            abs(observation.world_x_m) > COURT_MAX_X_M + COURT_BALL_MARGIN_M
            or observation.world_y_m is None
            or abs(observation.world_y_m) > COURT_MAX_Y_M + COURT_BALL_MARGIN_M
        ):
            return BallObservationInput(visible=False, source="out_of_court_filtered")
        return observation

    def _control_observation_for_mode(
        self, mode: str, observation: BallObservationInput, mapped_ball_id: int | None
    ) -> BallObservationInput:
        if mode not in {"collect", "collect_pattern"}:
            self.active_mapped_target_id = None
            return observation
        if mode == "collect_pattern" and self.collect_pattern_phase != "collect":
            return observation
        if self.behavior.state == CollectorState.SCAN:
            if not observation.visible and self._lidar_ranges is not None:
                lidar_obs = self._nearest_lidar_candidate_observation()
                if lidar_obs is not None:
                    return lidar_obs
            return observation
        now = time.time()
        if self.active_mapped_target_id is None:
            self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(
                self._robot_x, self._robot_y, now
            )
        if observation.visible and mapped_ball_id == self.active_mapped_target_id:
            return observation
        locked = self.ball_map.observation_from_target(
            self.active_mapped_target_id, self._robot_x, self._robot_y, self._robot_yaw, now
        )
        if locked is not None:
            return locked
        self.active_mapped_target_id = mapped_ball_id or self.ball_map.nearest_target_id(
            self._robot_x, self._robot_y, now
        )
        return (
            self.ball_map.observation_from_target(
                self.active_mapped_target_id, self._robot_x, self._robot_y, self._robot_yaw, now
            )
            or observation
        )

    def _check_collection(self, command: ConceptACommand) -> bool:
        if not command.collector.intake_enabled:
            return False
        # Hardware path: IR sensors
        if self._ir_left > IR_INTAKE_TRIGGER_THRESHOLD or self._ir_right > IR_INTAKE_TRIGGER_THRESHOLD:
            return True
        # Simulation path: ground-truth ball positions from /sim/balls
        if not self._sim_balls:
            return False
        ori_cos = math.cos(self._robot_yaw)
        ori_sin = math.sin(self._robot_yaw)
        for ball in self._sim_balls:
            dx = ball["x"] - self._robot_x
            dy = ball["y"] - self._robot_y
            # Approximate rotation matrix for 2D (Webots uses 3D but robot is flat)
            lx = ori_cos * dx + ori_sin * dy
            ly = -ori_sin * dx + ori_cos * dy
            if (
                INTAKE_ZONE_X_M[0] <= lx <= INTAKE_ZONE_X_M[1]
                and abs(ly) <= INTAKE_HALF_WIDTH_M
            ):
                collected_msg = String()
                collected_msg.data = ball["def"]
                self._pub_ball_collected.publish(collected_msg)
                self.collection_count += 1
                return True
        return False

    def _map_supervisor_balls(self) -> list[tuple[float, float]]:
        side = self._map_mission.bounds.side if self._map_mission.bounds else "left"
        result: list[tuple[float, float]] = []
        for ball in self._sim_balls:
            x = ball["x"]
            y = ball["y"]
            if side == "left" and x > -0.25:
                continue
            if side == "right" and x < 0.25:
                continue
            result.append((x, y))
        return result

    def _seed_mapped_balls_from_map_mission(self) -> int:
        if not self._map_mission.complete or not self._map_mission.candidates:
            return 0
        return self.ball_map.seed_from_candidates(self._map_mission.candidates, time.time())

    def _reset_collect_pattern(self) -> None:
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s = 0.0
        self.collect_pattern_failures = 0

    def _apply_command(self, command: ConceptACommand) -> None:
        twist = Twist()
        twist.linear.x = command.base.linear_speed_m_s
        twist.angular.z = command.base.angular_speed_rad_s
        self._pub_cmd_vel.publish(twist)

        col = CollectorCmd()
        col.lift_wheel_speed = float(command.collector.lift_wheel_speed)
        col.intake_enabled = command.collector.intake_enabled
        self._pub_collector.publish(col)

    def _publish_command(self, mode: str, source: str) -> None:
        msg = RobotCommand()
        msg.mode = mode
        msg.source = source
        self._pub_command.publish(msg)
        # Also write to the file store so the web panel stays in sync
        try:
            sys.path.insert(0, "/workspace/controllers/ball_detector")
            from control_bus import RobotCommandStore
            RobotCommandStore.from_env().write(mode, source)
        except Exception:
            pass

    def _publish_status(self, command: ConceptACommand, observation: BallObservationInput) -> None:
        status = {
            "mode": self.control_mode,
            "requested_mode": self._control_command_mode,
            "collector_state": command.state.value,
            "collection_count": self.collection_count,
            "loop_count": self.loop_count,
            "robot_x_m": round(self._robot_x, 3),
            "robot_y_m": round(self._robot_y, 3),
            "robot_yaw_rad": round(self._robot_yaw, 4),
            "cmd_linear_m_s": round(command.base.linear_speed_m_s, 3),
            "cmd_angular_rad_s": round(command.base.angular_speed_rad_s, 3),
            "ball_visible": observation.visible,
            "ball_distance_m": round(observation.distance_m, 3) if observation.visible else None,
            "collect_pattern_phase": self.collect_pattern_phase,
            "survey": {
                "state": self.survey_behavior.state.value,
                "sample_count": self.survey_behavior.sample_count,
                "bounds_saved": self.survey_behavior.court_bounds is not None,
                "bounds": self.survey_behavior.court_bounds,
                "navigation": self.survey_behavior.telemetry(),
            },
            "uptime_s": round(time.time() - self.started_at, 1),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
