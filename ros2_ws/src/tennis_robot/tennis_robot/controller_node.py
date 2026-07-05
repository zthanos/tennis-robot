"""ROS 2 controller node: orchestrates all robot behavior.

This node contains all logic previously in BallDetectorController (ball_detector.py),
with every direct Webots API call replaced by a ROS 2 topic read or write.
The underlying behavior modules (collector.py, survey.py, ball_map.py, etc.)
are imported unchanged from the controllers/ tree.

Subscribes:
  /perception/ball_detections (tennis_robot_msgs/BallDetectionArray)
  /survey/vision     (std_msgs/String, JSON)
  /scan              (sensor_msgs/LaserScan)
  /odom              (nav_msgs/Odometry)
  /ir/readings       (tennis_robot_msgs/IrReadings)
  /robot/command     (tennis_robot_msgs/RobotCommand)
  /sim/balls         (std_msgs/String, JSON) — sim-only ground truth

Publishes:
  /navigation/cmd_vel (geometry_msgs/Twist, consumed by navigation_node)
  /collector/cmd     (tennis_robot_msgs/CollectorCmd)
  /robot/status      (std_msgs/String, JSON)
  /ball/collected    (std_msgs/String, ball def name) — triggers sim animation
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer as TfBuffer, TransformListener as TfListener

from tennis_robot import yaw_from_quaternion

from tennis_robot.ball_map import BallMap, BallMapConfig, across_net
from tennis_robot.collect_one_mission import CollectOneMission
from tennis_robot.collector import (
    BallObservationInput,
    BaseCommand,
    CollectorCommand,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptACommand,
    ConceptAConfig,
)
from tennis_robot.config_utils import _env_float
from tennis_robot.lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m
from tennis_robot.mapping import (
    LidarSurveyBoundaryProvider,
    MapLeftSideMission,
    ServiceLineDistributionScanMission,
)
from tennis_robot.motion_controller import MOTION_COMMAND_TOPIC
try:
    from tennis_robot.nav2_lane_navigator import Nav2LaneNavigator, LaneNavState
    _NAV2_AVAILABLE = True
except Exception:  # nav2_msgs / action deps not present
    Nav2LaneNavigator = None
    LaneNavState = None
    _NAV2_AVAILABLE = False
from tennis_robot.search import HalfCourtSearchBehavior, SearchState
from tennis_robot.lidar_survey import LidarSurveyState
from tennis_robot.lidar_survey_v2 import LidarCourtSurveyV2 as Ros2LidarCourtSurvey
from tennis_robot.survey import SurveyVision
from tennis_robot_msgs.msg import BallDetectionArray, CollectorCmd, IrReadings, RobotCommand

TIME_STEP_S = 0.032
PERCEPTION_OBSERVATION_TIMEOUT_S = float(
    os.getenv("PERCEPTION_OBSERVATION_TIMEOUT_S", "1.0")
)
PERCEPTION_FRAME_ID = os.getenv(
    "PERCEPTION_FRAME_ID", "camera_link_optical_frame"
)
PERCEPTION_CAMERA_X_M = float(os.getenv("PERCEPTION_CAMERA_X_M", "0.535"))
COLLECTION_EVENT_SCHEMA_VERSION = 2
NET_X_M = 0.0
NET_SIDE_CLEARANCE_M = 0.25
COURT_MAX_X_M = 11.885
COURT_MAX_Y_M = 5.485
COURT_BALL_MARGIN_M = _env_float("COURT_BALL_MARGIN_M", 3.2)
# Roller-first intake: the sim now attempts REAL mechanical capture (paddled
# roller is the first contact; channel wraps the ball up into the basket), so
# ground truth counts a ball as collected only once it is physically inside
# the basket volume — local x in [0.0, 0.42], |y| <= 0.16, ball-centre z
# above the basket floor (top 0.128 m; ball centre on floor ~0.16 m). The old
# lip-contact zone (0.48–0.72 at ground level) is gone: it faked collection
# the moment the ball touched the lip and would delete the ball before the
# mechanism was ever exercised. Hardware/sim IR confirmation comes from the
# basket beam pair (see gazebo_extras_node.py).
BASKET_ZONE_X_M = (0.0, 0.42)
BASKET_HALF_WIDTH_M = 0.16
BASKET_MIN_BALL_Z_M = 0.12
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
COLLECTION_LANE_CAPTURE_RANGE_M = _env_float("COLLECTION_LANE_CAPTURE_RANGE_M", 2.5)
MAPPED_BALL_STALE_AFTER_S = 45.0
MANUAL_LINEAR_SPEED_M_S = _env_float("MANUAL_LINEAR_SPEED_M_S", 0.40)
MANUAL_TURN_SPEED_RAD_S = _env_float("MANUAL_TURN_SPEED_RAD_S", 0.80)
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
        self._collection_scan = ServiceLineDistributionScanMission(self._collection_scan_balls)
        self._nav2_requested = os.getenv(
            "COLLECTION_USE_NAV2", "false"
        ).strip().lower() in {"1", "true", "yes", "on"}
        if self._nav2_requested and not _NAV2_AVAILABLE:
            self.get_logger().error(
                "COLLECTION_USE_NAV2 is set but nav2_msgs/rclpy.action are unavailable - "
                "Nav2 lane navigation will NOT run and there is no P-controller fallback. "
                "Source the Nav2 install or set COLLECTION_USE_NAV2=false to use the P-controller deliberately."
            )
        self._use_nav2_lanes = self._nav2_requested and _NAV2_AVAILABLE
        self._nav2_lane = Nav2LaneNavigator(self) if self._use_nav2_lanes else None

        # ── state ──────────────────────────────────────────────────────────────
        self.control_mode = "idle"
        self.collection_count = 0
        self.loop_count = 0
        self.started_at = time.time()
        self.active_mapped_target_id: int | None = None
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s = 0.0
        self.collect_pattern_failures = 0
        self._collection_lane_collecting = False
        self._collection_opportunistic_collecting = False
        self._collection_lane_collect_elapsed_s = 0.0
        self.collection_confirmed = False
        self._collect_start_time: float | None = None
        self.scan_side_started_at: float | None = None
        self._collection_complete_reported = False
        self._search_complete_reported = False
        self._survey_complete_reported = False
        self._map_completion_reported = False
        self._collection_scan_completion_reported = False
        self._last_survey_log_key: tuple[str, str] | None = None
        self._last_status_file_write_s: float = 0.0
        self._collection_events: deque[dict] = deque(maxlen=60)
        self._collection_event_started_at: float | None = None
        self._collection_event_log = Path(
            os.getenv(
                "COLLECTION_EVENT_LOG_FILE",
                "/workspace/runtime/collection_events.jsonl",
            )
        )
        self._run_id = f"{int(self.started_at)}-{os.getpid()}"
        self._last_collection_event_key: tuple | None = None
        self._last_collection_scan_key: tuple | None = None

        # ── cached topic values ────────────────────────────────────────────────
        self._latest_obs = BallObservationInput(visible=False, source="startup")
        self._latest_obs_received_at = 0.0
        self._latest_obs_seq = 0
        self._mapped_obs_seq = 0
        self._latest_survey_vision: SurveyVision | None = None
        self._latest_camera_balls: list[dict] = []
        self._latest_camera_balls_received_at = 0.0
        self._last_bad_perception_frame = ""
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
        self._turn_180_start_yaw: float = 0.0

        # ── pose source: SLAM-corrected TF with /odom fallback ─────────────────
        # Raw wheel odometry mis-estimates in-place turns (4WD skid-steer slips
        # laterally during rotation), so map->base_footprint from slam_toolbox is
        # the authoritative pose. /odom remains the fallback until SLAM publishes.
        self._tf_buffer = TfBuffer()
        self._tf_listener = TfListener(self._tf_buffer, self)
        self._pose_source = "odom"

        # ── subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            BallDetectionArray,
            "/perception/ball_detections",
            self._on_ball_detections,
            1,
        )
        self.create_subscription(String, "/survey/vision", self._on_survey_vision, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 1)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(IrReadings, "/ir/readings", self._on_ir, 10)
        self.create_subscription(RobotCommand, "/robot/command", self._on_command, 10)
        self.create_subscription(String, "/sim/balls", self._on_sim_balls, 1)

        # ── publishers ─────────────────────────────────────────────────────────
        self._pub_motion_cmd = self.create_publisher(Twist, MOTION_COMMAND_TOPIC, 1)
        self._pub_collector = self.create_publisher(CollectorCmd, "/collector/cmd", 1)
        self._pub_status = self.create_publisher(String, "/robot/status", 10)
        self._pub_collection_event = self.create_publisher(
            String, "/telemetry/collection_events", 10
        )
        self._pub_ball_collected = self.create_publisher(String, "/ball/collected", 10)
        self._pub_command = self.create_publisher(RobotCommand, "/robot/command", 10)

        self.create_timer(TIME_STEP_S, self._step)
        self.get_logger().info("tennis_robot_controller started")

    # ── subscription callbacks (cache only) ────────────────────────────────────

    def _on_survey_vision(self, msg: String) -> None:
        self._latest_survey_vision = _survey_vision_from_json(msg.data)

    def _on_ball_detections(self, msg: BallDetectionArray) -> None:
        """Consume the canonical sim/real OAK-D perception contract."""
        if msg.header.frame_id != PERCEPTION_FRAME_ID:
            if msg.header.frame_id != self._last_bad_perception_frame:
                self.get_logger().error(
                    f"rejecting perception frame {msg.header.frame_id!r}; "
                    f"expected {PERCEPTION_FRAME_ID!r}"
                )
                self._last_bad_perception_frame = msg.header.frame_id
            self._latest_obs = BallObservationInput(
                visible=False, source="invalid_perception_frame"
            )
            self._latest_camera_balls = []
            return

        received_at = self._runtime_seconds()
        observations: list[BallObservationInput] = []
        camera_balls: list[dict] = []
        for detection in msg.detections:
            if (
                not detection.has_spatial
                or not math.isfinite(detection.distance_m)
                or detection.distance_m <= 0.0
            ):
                continue

            # Optical XYZ is +right/+down/+forward. Convert it to the robot
            # planar frame (+forward/+left), then through the authoritative
            # controller pose into map/world coordinates.
            local_x = PERCEPTION_CAMERA_X_M + float(detection.position_z)
            local_y = -float(detection.position_x)
            cos_yaw = math.cos(self._robot_yaw)
            sin_yaw = math.sin(self._robot_yaw)
            world_x = self._robot_x + cos_yaw * local_x - sin_yaw * local_y
            world_y = self._robot_y + sin_yaw * local_x + cos_yaw * local_y
            observation = BallObservationInput(
                visible=True,
                bearing_rad=float(detection.bearing_rad),
                distance_m=float(detection.distance_m),
                confidence=float(detection.confidence),
                source="oak_ai_depth",
                world_x_m=world_x,
                world_y_m=world_y,
                robot_x_m=self._robot_x,
                robot_y_m=self._robot_y,
            )
            observations.append(observation)
            camera_balls.append(
                {
                    "bearing_rad": round(observation.bearing_rad, 4),
                    "distance_m": round(observation.distance_m, 3),
                    "source": observation.source,
                    "confidence": round(observation.confidence, 3),
                    "world_x_m": round(world_x, 3),
                    "world_y_m": round(world_y, 3),
                }
            )

        self._latest_obs = (
            min(observations, key=lambda obs: obs.distance_m)
            if observations
            else BallObservationInput(visible=False, source="no_detection")
        )
        self._latest_camera_balls = camera_balls
        self._latest_obs_received_at = received_at
        self._latest_camera_balls_received_at = received_at
        self._latest_obs_seq += 1

    def _on_scan(self, msg: LaserScan) -> None:
        self._lidar_ranges = [float(r) for r in msg.ranges]
        self._lidar_angle_min = float(msg.angle_min)
        self._lidar_angle_increment = float(msg.angle_increment)

    def _on_odom(self, msg: Odometry) -> None:
        if self._pose_source == "slam_tf":
            return  # SLAM TF is authoritative once available
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _update_pose_from_tf(self) -> None:
        """Refresh pose from map->base_footprint (slam_toolbox) when available."""
        try:
            t = self._tf_buffer.lookup_transform("map", "base_footprint", RclpyTime())
        except Exception:
            return  # SLAM not up (yet) — keep odom pose
        self._robot_x = t.transform.translation.x
        self._robot_y = t.transform.translation.y
        self._robot_yaw = yaw_from_quaternion(t.transform.rotation)
        self._pose_source = "slam_tf"

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

    def _runtime_seconds(self) -> float:
        """Mission time: Gazebo /clock in sim, system time on the real robot."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh_perception_observation(
        self, runtime_now: float | None = None
    ) -> BallObservationInput:
        """Return the current observation or an explicit timeout sentinel."""
        now = self._runtime_seconds() if runtime_now is None else runtime_now
        if (
            self._latest_camera_balls_received_at <= 0.0
            or now - self._latest_camera_balls_received_at
            > PERCEPTION_OBSERVATION_TIMEOUT_S
        ):
            self._latest_camera_balls = []
        if (
            self._latest_obs_received_at > 0.0
            and now - self._latest_obs_received_at
            <= PERCEPTION_OBSERVATION_TIMEOUT_S
        ):
            return self._latest_obs
        return BallObservationInput(visible=False, source="observation_timeout")

    def _step(self) -> None:
        self._update_pose_from_tf()
        observation = self._fresh_perception_observation()
        now = self._runtime_seconds()
        mapping_observation = (
            observation
            if self._latest_obs_seq != self._mapped_obs_seq
            else BallObservationInput(visible=False, source="observation_already_mapped")
        )
        mapped_observation = self._mapping_observation(mapping_observation)
        self._mapped_obs_seq = self._latest_obs_seq
        mapped_ball_id, is_new_ball = self.ball_map.update(mapped_observation, now)
        control_mapping_observation = self._mapping_observation(observation)

        if self.loop_count % 90 == 0:
            self.ball_map.prune_phantoms(now)

        effective_mode = self._effective_control_mode(self._control_command_mode)
        control_observation = self._control_observation_for_mode(
            effective_mode, control_mapping_observation, mapped_ball_id
        )

        if effective_mode == "map_court":
            command = self._external_map_court_command(effective_mode)
        elif effective_mode == "search":
            command = self._search_command_for_mode(
                effective_mode, self._same_side_search_observation(control_observation)
            )
        elif effective_mode == "collect_pattern":
            command = self._collect_pattern_command_for_mode(
                effective_mode,
                self._same_side_search_observation(control_mapping_observation),
                mapped_ball_id,
            )
        elif effective_mode == "map_left_side":
            command = self._map_mission_command_for_mode(effective_mode)
        elif effective_mode in self._MANUAL_MODES:
            command = self._manual_move_command(effective_mode)
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

    def _external_map_court_command(self, mode: str) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self.get_logger().info(
                "map_court delegated to court_survey.launch.py; controller drive output idle"
            )
        return ConceptACommand(
            state=CollectorState.SURVEY,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    # ── mode orchestration (logic unchanged from BallDetectorController) ───────

    def _on_mode_changed(self, new_mode: str) -> bool:
        if new_mode == self.control_mode:
            return False
        previous_mode = self.control_mode
        if previous_mode == "collect" and new_mode != "collect" and self._nav2_lane is not None:
            self._record_collection_event("nav2_goal_cancel", reason=f"mode_exit:{new_mode}")
            self._nav2_lane.reset()
        self.behavior.reset()
        self.search_behavior.reset()
        if not (self.control_mode == "map_court" and new_mode == "idle"):
            self.survey_behavior.reset()
        if self.control_mode == "map_left_side" and not self._map_mission.complete:
            self._map_mission.reset()
        if self.control_mode == "collect" and not self._collection_scan.complete:
            self._collection_scan.reset()
        self.control_mode = new_mode
        self.collect_one_mission.reset()
        self._reset_collect_pattern()
        self.scan_side_started_at = None
        self._collect_start_time = None
        self.active_mapped_target_id = None
        self._collection_scan_completion_reported = False
        self.get_logger().info(f"mode → {new_mode}")
        if new_mode == "collect":
            self._collection_events.clear()
            self._collection_event_started_at = self._runtime_seconds()
            if self._nav2_lane is not None:
                self._nav2_lane.reset()
            self._last_collection_event_key = None
            self._last_collection_scan_key = None
            self._record_collection_event("mode_enter", requested=self._control_command_mode)
        return True

    _MANUAL_MODES = frozenset({
        "move_forward", "move_backward", "move_left", "move_right", "turn_180",
        "move_forward_left", "move_forward_right",
        "move_backward_left", "move_backward_right",
    })
    _AUTONOMOUS_MODES = frozenset({"map_court", "map_left_side", "collect_pattern", "collect", "collect_one", "search", "scan_side"})

    def _effective_control_mode(self, requested_mode: str) -> str:
        if requested_mode in self._MANUAL_MODES and self.control_mode in self._AUTONOMOUS_MODES:
            return self.control_mode
        if requested_mode == "collect":
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
                self._collection_scan.reset()
                self._collect_start_time = self._runtime_seconds()
            elif mode == "collect_one":
                self.ball_map.reset()
                self.collect_one_mission.start(self._robot_pose_2d())
                self._collect_start_time = self._runtime_seconds()

        if mode == "collect":
            return self._collection_distribution_scan_command_for_mode(mode, observation)

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

    def _manual_move_command(self, mode: str) -> ConceptACommand:
        if self._on_mode_changed(mode) and mode == "turn_180":
            self._turn_180_start_yaw = self._robot_yaw

        if mode == "turn_180":
            rotated = abs(_angle_delta_rad(self._robot_yaw, self._turn_180_start_yaw))
            if rotated >= math.pi - math.radians(10.0):
                self._publish_command("idle", "controller-turn-180-complete")
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, MANUAL_TURN_SPEED_RAD_S),
                collector=CollectorCommand(0.0, False),
            )

        if mode == "move_forward":
            base = BaseCommand(MANUAL_LINEAR_SPEED_M_S, 0.0)
        elif mode == "move_backward":
            base = BaseCommand(-MANUAL_LINEAR_SPEED_M_S, 0.0)
        elif mode == "move_left":
            base = BaseCommand(0.0, MANUAL_TURN_SPEED_RAD_S)
        elif mode == "move_right":
            base = BaseCommand(0.0, -MANUAL_TURN_SPEED_RAD_S)
        elif mode == "move_forward_left":
            base = BaseCommand(MANUAL_LINEAR_SPEED_M_S, MANUAL_TURN_SPEED_RAD_S)
        elif mode == "move_forward_right":
            base = BaseCommand(MANUAL_LINEAR_SPEED_M_S, -MANUAL_TURN_SPEED_RAD_S)
        elif mode == "move_backward_left":
            base = BaseCommand(-MANUAL_LINEAR_SPEED_M_S, MANUAL_TURN_SPEED_RAD_S)
        else:  # move_backward_right
            base = BaseCommand(-MANUAL_LINEAR_SPEED_M_S, -MANUAL_TURN_SPEED_RAD_S)
        return ConceptACommand(
            state=CollectorState.IDLE,
            base=base,
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
                    self._robot_x, self._robot_y, self._runtime_seconds()
                )
            self._collect_start_time = self._runtime_seconds()
            self._collection_complete_reported = False
            self.collect_pattern_phase = "search"
            self.collect_pattern_collect_elapsed_s = 0.0
            self.collect_pattern_failures = 0

        if self.collect_pattern_phase == "collect":
            return self._collect_pattern_collect_command(observation, mapped_ball_id)

        now = self._runtime_seconds()
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
        now = self._runtime_seconds()
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
                bounds = self.survey_behavior.court_bounds
                if bounds:
                    self.get_logger().info(
                        f"survey complete: type={bounds.get('survey_type')} "
                        f"status={bounds.get('status')} "
                        f"is_doubles={bounds.get('is_doubles')}"
                    )
                    # DB import is handled by the control panel when it reads
                    # court_boundary.json — no direct DuckDB write from here.
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
        extents = (nav.get("scan_coverage") or {}).get("world_extents") or {}
        ext_str = (
            f"x[{extents.get('min_x_m'):.1f}..{extents.get('max_x_m'):.1f}] "
            f"y[{extents.get('min_y_m'):.1f}..{extents.get('max_y_m'):.1f}]"
            if extents else "extents=none"
        )
        dbg = nav.get("net_detection_debug") or {}
        dbg_str = " ".join(f"{k}={v}" for k, v in dbg.items()) if dbg else "none"
        self.get_logger().info(
            "map_court "
            f"state={nav.get('state')} event={nav.get('last_event')} "
            f"points={nav.get('map_point_count')} net={'yes' if nav.get('net_boundary') else 'no'} "
            f"target={nav.get('active_target')} dist={nav.get('distance_to_target_m')} "
            f"{ext_str} net_dbg=[{dbg_str}]"
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
                    self._robot_x, self._robot_y, self._runtime_seconds()
                )
            self.get_logger().info(
                f"map_left_side complete; candidates={len(self._map_mission.candidates)} seeded={seeded}"
            )
            self._publish_command("idle", "controller-map-complete")
            self._map_completion_reported = True
        return command

    def _collection_distribution_scan_command_for_mode(
        self, _mode: str, observation: BallObservationInput
    ) -> ConceptACommand:
        if not self._collection_scan.active and not self._collection_scan.complete:
            try:
                self._collection_scan.start(self._robot_x, self._robot_y, self._robot_yaw)
                scan = self._collection_scan.telemetry()
                self._record_collection_event(
                    "scan_start",
                    side=scan.get("side"),
                    lane_count=scan.get("lane_count"),
                    lane_spacing_m=scan.get("lane_spacing_m"),
                    waypoint_count=scan.get("waypoint_count"),
                )
            except RuntimeError as exc:
                self.get_logger().error(f"collect distribution scan blocked: {exc}")
                self._record_collection_event("scan_blocked", reason=str(exc))
                self._publish_command("idle", "controller-collect-scan-blocked")
                return ConceptACommand(
                    state=CollectorState.IDLE,
                    base=BaseCommand(0.0, 0.0),
                    collector=CollectorCommand(0.0, False),
                )

        # Hard half-court gate. The court is mapped, so any detection beyond the
        # collect half-court bounds (across the net, past the fence, the other
        # half) is ignored outright; it must never start or abort a lane collect.
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and not self._collection_scan.observation_in_half_court(
                observation.world_x_m, observation.world_y_m
            )
        ):
            observation = BallObservationInput(visible=False, source="out_of_half_court")

        if self._collection_lane_collecting:
            return self._collection_lane_collect_command(observation)

        if (
            self._collection_scan.active
            and not self._collection_scan.complete
            and not self._collection_scan.lane_started
            and self._collection_pre_lane_observation_valid(observation)
        ):
            self._collection_lane_collecting = True
            self._collection_opportunistic_collecting = True
            self._collection_lane_collect_elapsed_s = 0.0
            self.behavior.reset()
            self.behavior.start_tracking(observation)
            if self._nav2_lane is not None:
                self._nav2_lane.cancel()
            self._record_collection_event(
                "opportunistic_collect_start",
                ball_distance_m=observation.distance_m,
                ball_x_m=observation.world_x_m,
                ball_y_m=observation.world_y_m,
                source=observation.source,
                confidence=observation.confidence,
            )
            return self.behavior.update(observation, TIME_STEP_S, collection_confirmed=False)

        if (
            self._collection_scan.active
            and not self._collection_scan.complete
            and self._collection_scan.lane_started
            and self._collection_lane_observation_valid(observation)
        ):
            self._collection_lane_collecting = True
            self._collection_lane_collect_elapsed_s = 0.0
            self.behavior.reset()
            self.behavior.start_tracking(observation)
            if self._nav2_lane is not None:
                self._nav2_lane.cancel()
            self._record_collection_event(
                "lane_collect_start",
                ball_distance_m=observation.distance_m,
                ball_x_m=observation.world_x_m,
                ball_y_m=observation.world_y_m,
                source=observation.source,
                confidence=observation.confidence,
            )
            return self.behavior.update(observation, TIME_STEP_S, collection_confirmed=False)

        if self._use_nav2_lanes and self._nav2_lane is not None:
            command = self._collection_nav2_sweep_step()
        elif self._nav2_requested:
            # Nav2 requested but its deps are missing: fail loud, never silently
            # drive with the P-controller while we are debugging the Nav2 path.
            self._record_collection_event(
                "nav2_unavailable",
                detail="nav2_msgs/rclpy.action not importable; robot stopped (no fallback)",
            )
            command = ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        else:
            command = self._collection_scan.update(
                self._robot_x, self._robot_y, self._robot_yaw, TIME_STEP_S
            )
        self._record_collection_scan_snapshot()
        if self._collection_scan.complete and not self._collection_scan_completion_reported:
            seeded = self.ball_map.seed_from_candidates(
                self._collection_scan.local_candidates, self._runtime_seconds()
            )
            self.get_logger().info(
                "collect distribution scan complete; "
                f"side={self._collection_scan.side_id} "
                f"estimate_candidates={len(self._collection_scan.candidates)} "
                f"local_candidates={len(self._collection_scan.local_candidates)} seeded={seeded} "
                f"grid={self._collection_scan.grid}"
            )
            self._record_collection_event(
                "scan_complete",
                local_candidates=len(self._collection_scan.local_candidates),
                seeded=seeded,
                grid=self._collection_scan.grid,
            )
            self._publish_command("idle", "controller-collect-distribution-scan-complete")
            self._collection_scan_completion_reported = True
        return command

    def _collection_nav2_sweep_step(self) -> ConceptACommand:
        """Drive the current lawnmower waypoint via Nav2. The controller emits
        an idle command so the motor adapter goes silent and twist_mux hands the
        wheels to Nav2 (/cmd_vel_nav). There is NO P-controller fallback: if the
        Nav2 action server is not up the robot stops and the reason is logged,
        so a broken Nav2 stack is loud instead of silently masked."""
        target = self._collection_scan.nav2_target(self._robot_x, self._robot_y)
        if target is None:
            self._nav2_lane.reset()
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        tx, ty = target
        goal_yaw = math.atan2(ty - self._robot_y, tx - self._robot_x)
        self._nav2_lane.request(tx, ty, goal_yaw)
        if self._nav2_lane.state == LaneNavState.UNAVAILABLE:
            self._record_collection_event(
                "nav2_unavailable",
                detail="navigate_to_pose action server not up; robot stopped (no fallback)",
                target_x_m=tx,
                target_y_m=ty,
            )
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        return ConceptACommand(
            state=CollectorState.SURVEY,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _collection_lane_collect_command(self, observation: BallObservationInput) -> ConceptACommand:
        if self.collection_confirmed:
            self.behavior.reset()
            self.active_mapped_target_id = None
            self._collection_lane_collecting = False
            self._collection_opportunistic_collecting = False
            self._collection_lane_collect_elapsed_s = 0.0
            self._record_collection_event("lane_collect_confirmed")
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        self._collection_lane_collect_elapsed_s += TIME_STEP_S
        reject_reason = self._collection_lane_reject_reason(observation)
        if observation.visible and reject_reason is not None:
            self.behavior.reset()
            self.active_mapped_target_id = None
            self._collection_lane_collecting = False
            self._collection_opportunistic_collecting = False
            self._collection_lane_collect_elapsed_s = 0.0
            self._record_collection_event(
                "lane_collect_abort",
                reason=reject_reason,
                ball_distance_m=observation.distance_m,
                ball_x_m=observation.world_x_m,
                ball_y_m=observation.world_y_m,
            )
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        if self._collection_lane_collect_elapsed_s > COLLECT_PATTERN_COLLECTION_TIMEOUT_S:
            elapsed_s = self._collection_lane_collect_elapsed_s
            self.behavior.reset()
            self.active_mapped_target_id = None
            self._collection_lane_collecting = False
            self._collection_opportunistic_collecting = False
            self._collection_lane_collect_elapsed_s = 0.0
            self._record_collection_event(
                "lane_collect_timeout",
                elapsed_s=elapsed_s,
            )
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        if self.behavior.state == CollectorState.SCAN and observation.visible:
            self.behavior.start_tracking(observation)
        cmd = self.behavior.update(observation, TIME_STEP_S, collection_confirmed=False)
        if self.behavior.gave_up:
            self.behavior.reset()
            self.active_mapped_target_id = None
            self._collection_lane_collecting = False
            self._collection_opportunistic_collecting = False
            self._collection_lane_collect_elapsed_s = 0.0
            self._record_collection_event("lane_collect_gave_up")
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        return cmd

    def _collection_lane_observation_valid(self, observation: BallObservationInput) -> bool:
        return self._collection_lane_reject_reason(observation) is None

    def _collection_pre_lane_observation_valid(self, observation: BallObservationInput) -> bool:
        if not observation.visible:
            return False
        if observation.distance_m > COLLECTION_LANE_CAPTURE_RANGE_M:
            return False
        if observation.world_x_m is None or observation.world_y_m is None:
            return False
        return self._collection_scan.observation_in_half_court(observation.world_x_m, observation.world_y_m)

    def _collection_lane_reject_reason(self, observation: BallObservationInput) -> str | None:
        if not observation.visible:
            return "no_visible_ball"
        if observation.distance_m > COLLECTION_LANE_CAPTURE_RANGE_M:
            return "out_of_range"
        if self._collection_opportunistic_collecting:
            if not self._collection_scan.observation_in_half_court(observation.world_x_m, observation.world_y_m):
                return "outside_collect_half_court"
            return None
        if not self._collection_scan.observation_in_active_lane(observation.world_x_m, observation.world_y_m):
            return "outside_active_lane"
        return None

    def _record_collection_event(self, event_type: str, **fields: object) -> None:
        now = self._runtime_seconds()
        if self._collection_event_started_at is None:
            self._collection_event_started_at = now
        truth = self._collection_truth()
        event = {
            "schema_version": COLLECTION_EVENT_SCHEMA_VERSION,
            "run_id": self._run_id,
            "t_s": round(max(0.0, now - self._collection_event_started_at), 3),
            "sim_time_s": round(now, 3),
            "recorded_at_s": round(time.time(), 3),
            "type": event_type,
            "mode": self.control_mode,
            "lane": self._collection_scan.telemetry().get("phase_label"),
            "phase": truth["phase"],
            "motion_owner": truth["motion_owner"],
            "motion_path": truth["motion_path"],
            "fallback_mode": truth["fallback_mode"],
            "current_blocker": truth["current_blocker"],
            "nav2_state": truth["nav2_state"],
            "robot_x_m": round(self._robot_x, 3),
            "robot_y_m": round(self._robot_y, 3),
        }
        for key, value in fields.items():
            if isinstance(value, float):
                event[key] = round(value, 3)
            else:
                event[key] = value
        temporal_fields = {"t_s", "sim_time_s", "recorded_at_s", "run_id"}
        key = tuple(
            sorted(
                (
                    k,
                    json.dumps(v, sort_keys=True)
                    if isinstance(v, (dict, list))
                    else v,
                )
                for k, v in event.items()
                if k not in temporal_fields
            )
        )
        if key == self._last_collection_event_key:
            return
        self._last_collection_event_key = key
        self._collection_events.append(event)
        payload = json.dumps(event, separators=(",", ":"), sort_keys=True)
        self._pub_collection_event.publish(String(data=payload))
        try:
            self._collection_event_log.parent.mkdir(parents=True, exist_ok=True)
            with self._collection_event_log.open("a", encoding="utf-8") as handle:
                handle.write(payload + "\n")
        except OSError as exc:
            self.get_logger().warning(
                f"failed to append collection event log: {exc}"
            )

    def _collection_truth(self, command: ConceptACommand | None = None) -> dict[str, object]:
        scan = self._collection_scan.telemetry()
        nav2_state = self._nav2_lane.state.value if self._nav2_lane is not None else "disabled"
        nav2_busy = bool(self._nav2_lane.busy) if self._nav2_lane is not None else False
        nav2_goal = self._nav2_lane.goal_xy if self._nav2_lane is not None else None
        nav2_goal_xy = (
            [round(float(nav2_goal[0]), 3), round(float(nav2_goal[1]), 3)]
            if nav2_goal is not None
            else None
        )

        if self.control_mode in self._MANUAL_MODES:
            phase = "manual"
            motion_owner = "manual"
            motion_path = "manual_teleop"
            fallback_mode = "none"
            current_blocker = "none"
        elif self.control_mode == "collect" and self._collection_lane_collecting:
            phase = "fine_collect"
            motion_owner = "collector_fsm"
            motion_path = "collector_fine_approach"
            fallback_mode = "none"
            current_blocker = "none"
        elif self.control_mode == "collect" and scan.get("active") and not scan.get("complete"):
            phase = "lane_sweep" if scan.get("lane_started") else "transit_to_lane_start"
            if self._use_nav2_lanes:
                motion_owner = "nav2"
                motion_path = "nav2_lawnmower"
                fallback_mode = "none"
                current_blocker = (
                    "nav2_action_unavailable"
                    if nav2_state == "unavailable"
                    else "waiting_for_lane_start"
                    if not scan.get("lane_started")
                    else "none"
                )
            elif self._nav2_requested:
                motion_owner = "none"
                motion_path = "stopped"
                fallback_mode = "nav2_import_missing_stop"
                current_blocker = "nav2_import_missing"
            else:
                motion_owner = "controller_fsm"
                motion_path = "local_lawnmower"
                fallback_mode = "deliberate_local_controller"
                current_blocker = "waiting_for_lane_start" if not scan.get("lane_started") else "none"
        elif self.control_mode == "collect":
            phase = "complete" if scan.get("complete") else "preflight"
            motion_owner = "none"
            motion_path = "stopped"
            fallback_mode = "none"
            current_blocker = "none"
        elif nav2_busy or nav2_goal_xy is not None:
            phase = "external_nav2_goal"
            motion_owner = "nav2"
            motion_path = "nav2_lawnmower"
            fallback_mode = "none"
            current_blocker = "nav2_goal_active_outside_collect"
        elif command is not None and (
            abs(command.base.linear_speed_m_s) > 1e-9
            or abs(command.base.angular_speed_rad_s) > 1e-9
        ):
            phase = self.control_mode
            motion_owner = "controller_fsm"
            motion_path = self.control_mode
            fallback_mode = "none"
            current_blocker = "none"
        else:
            phase = self.control_mode
            motion_owner = "none"
            motion_path = "stopped"
            fallback_mode = "none"
            current_blocker = "none"

        return {
            "schema_version": COLLECTION_EVENT_SCHEMA_VERSION,
            "phase": phase,
            "motion_owner": motion_owner,
            "motion_path": motion_path,
            "fallback_mode": fallback_mode,
            "current_blocker": current_blocker,
            "nav2_requested": self._nav2_requested,
            "nav2_enabled": self._use_nav2_lanes,
            "nav2_state": nav2_state,
            "nav2_busy": nav2_busy,
            "nav2_goal_xy": nav2_goal_xy,
            "lane_started": bool(scan.get("lane_started")),
            "scan_phase": scan.get("phase"),
            "scan_phase_label": scan.get("phase_label"),
            "waypoint_index": scan.get("waypoint_index"),
            "waypoint_count": scan.get("waypoint_count"),
        }

    def _record_collection_scan_snapshot(self) -> None:
        scan = self._collection_scan.telemetry()
        target = scan.get("target_pose_map") or {}
        key = (
            scan.get("phase"),
            scan.get("phase_label"),
            scan.get("waypoint_index"),
            target.get("x_m"),
            target.get("y_m"),
        )
        if key == self._last_collection_scan_key:
            return
        self._last_collection_scan_key = key
        self._record_collection_event(
            "lane_target",
            phase=scan.get("phase"),
            waypoint_index=scan.get("waypoint_index"),
            waypoint_count=scan.get("waypoint_count"),
            target_x_m=target.get("x_m"),
            target_y_m=target.get("y_m"),
            candidates=scan.get("total_candidates"),
            assigned=scan.get("assigned_candidates"),
        )

    def _scan_side_command(self) -> ConceptACommand:
        now = self._runtime_seconds()
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
        now = self._runtime_seconds()
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
            # Approximate rotation matrix for 2D (robot is flat on the court)
            lx = ori_cos * dx + ori_sin * dy
            ly = -ori_sin * dx + ori_cos * dy
            # Ball world z ~= height above court (flat ground, robot z ~ 0).
            bz = float(ball.get("z", 0.0))
            if (
                BASKET_ZONE_X_M[0] <= lx <= BASKET_ZONE_X_M[1]
                and abs(ly) <= BASKET_HALF_WIDTH_M
                and bz >= BASKET_MIN_BALL_Z_M
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

    def _collection_scan_balls(self) -> list[tuple[float, float]]:
        now = self._runtime_seconds()
        result: list[tuple[float, float]] = []
        for ball in self.ball_map.balls.values():
            if ball.state in {"collected", "collection_failed"}:
                continue
            if ball.seen_count < self.ball_map.config.min_seen_count:
                continue
            if now - ball.last_seen_s > self.ball_map.config.stale_after_s:
                continue
            result.append((ball.x_m, ball.y_m))
        return result

    def _seed_mapped_balls_from_map_mission(self) -> int:
        if not self._map_mission.complete or not self._map_mission.candidates:
            return 0
        return self.ball_map.seed_from_candidates(
            self._map_mission.candidates, self._runtime_seconds()
        )

    def _reset_collect_pattern(self) -> None:
        self.collect_pattern_phase = "idle"
        self.collect_pattern_collect_elapsed_s = 0.0
        self.collect_pattern_failures = 0
        self._collection_lane_collecting = False
        self._collection_opportunistic_collecting = False
        self._collection_lane_collect_elapsed_s = 0.0

    def _apply_command(self, command: ConceptACommand) -> None:
        twist = Twist()
        twist.linear.x = command.base.linear_speed_m_s
        twist.angular.z = command.base.angular_speed_rad_s
        self._pub_motion_cmd.publish(twist)

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
            from tennis_robot.control_bus import RobotCommandStore
            RobotCommandStore.from_env().write(mode, source)
        except Exception:
            pass

    def _build_map_payload(self) -> dict:
        """Collection Map payload: recognized balls at their detected world points,
        plus the active target, robot pose and camera cone for the renderer."""
        cfg = self.ball_map.config
        balls = self.ball_map.to_console_balls(
            self._robot_x, self.active_mapped_target_id, now=self._runtime_seconds()
        )
        confirmed = [b for b in balls if b["confirmed"] and b["side"] != "across_net"]
        return {
            "balls": balls,
            "active_target_id": self.active_mapped_target_id,
            "robot": {
                "x_m": round(self._robot_x, 3),
                "y_m": round(self._robot_y, 3),
                "yaw_rad": round(self._robot_yaw, 4),
            },
            "camera_fov_rad": round(cfg.supervised_fov_rad, 4),
            "camera_max_range_m": round(cfg.supervised_max_range_m, 2),
            "metrics": {
                "balls_mapped": len(balls),
                "balls_confirmed": len(confirmed),
                "balls_collectable": len(confirmed),
            },
        }

    def _publish_status(self, command: ConceptACommand, observation: BallObservationInput) -> None:
        status = {
            "mode": self.control_mode,
            "actual_mode": self.control_mode,       # alias for control panel JS compatibility
            "requested_mode": self._control_command_mode,
            "collector_state": command.state.value,
            "collection_count": self.collection_count,
            "balls_collected": self.collection_count,
            "loop_count": self.loop_count,
            "robot_x_m": round(self._robot_x, 3),
            "robot_y_m": round(self._robot_y, 3),
            "robot_yaw_rad": round(self._robot_yaw, 4),
            "pose_source": self._pose_source,
            "cmd_linear_m_s": round(command.base.linear_speed_m_s, 3),
            "cmd_angular_rad_s": round(command.base.angular_speed_rad_s, 3),
            "ball_visible": observation.visible,
            "ball_distance_m": round(observation.distance_m, 3) if observation.visible else None,
            # Camera bearing (rad, +left per ROS) so the console can draw the live
            # detected ball on the map before it is mapped/confirmed.
            "ball_bearing_rad": round(observation.bearing_rad, 4) if observation.visible else None,
            # Recognized world point of the primary ball, so the map can pin it where
            # it was detected instead of reprojecting from the (moving) live pose.
            "ball_world_x_m": round(observation.world_x_m, 3) if observation.visible and observation.world_x_m is not None else None,
            "ball_world_y_m": round(observation.world_y_m, 3) if observation.visible and observation.world_y_m is not None else None,
            # All in-frame camera balls (bearing/distance/world/source) for the map.
            "camera_balls": self._latest_camera_balls,
            # Recognized-ball registry surfaced for the Collection Map: every ball
            # drawn at the world point where it was detected (see ball_map).
            "map": self._build_map_payload(),
            "collect_pattern_phase": self.collect_pattern_phase,
            "collection_lane_collecting": self._collection_lane_collecting,
            "collection_opportunistic_collecting": self._collection_opportunistic_collecting,
            "collection_scan": self._collection_scan.telemetry(),
            "collection_truth": self._collection_truth(command),
            "collection_events": list(self._collection_events),
            "collection_nav2": {
                "enabled": self._use_nav2_lanes,
                "requested": self._nav2_requested,
                "state": self._nav2_lane.state.value if self._nav2_lane is not None else "disabled",
                "goal_xy": self._nav2_lane.goal_xy if self._nav2_lane is not None else None,
            },
            "map_mission": self._map_mission.telemetry(),
            "survey": {
                "state": self.survey_behavior.state.value,
                "sample_count": self.survey_behavior.sample_count,
                "bounds_saved": self.survey_behavior.court_bounds is not None,
                "bounds": self.survey_behavior.court_bounds,
                "navigation": self.survey_behavior.telemetry(),
                "vision": asdict(self._latest_survey_vision) if self._latest_survey_vision is not None else None,
            },
            "uptime_s": round(time.time() - self.started_at, 1),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)

        # Write to file so the web control panel can read it (throttled to ~2 Hz)
        now = time.time()
        if now - self._last_status_file_write_s >= 0.5:
            self._last_status_file_write_s = now
            try:
                from tennis_robot.control_bus import RobotStatusStore
                RobotStatusStore.from_env().write(status)
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
