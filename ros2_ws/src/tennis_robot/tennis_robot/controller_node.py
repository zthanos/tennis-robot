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
  /collector/intake_beam_broken (std_msgs/Bool)
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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer as TfBuffer, TransformListener as TfListener

from tennis_robot import yaw_from_quaternion

from tennis_robot.ball_map import BallMap, BallMapConfig, across_net
from tennis_robot.collect_one_mission import CollectOneMission
from tennis_robot.collect_route_mission import CollectRouteMission
from tennis_robot.collection_route_planner import CourtModel, remaining_route_length_m
from tennis_robot.collection_scoring import (
    CreditReconciler,
    SimRetentionTracker,
    onboard_ball_zone,
    retained_ball_still_in_bin,
)
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
    DEFAULT_BOUNDARY_FILE,
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
SIM_BASKET_RETENTION_DWELL_S = _env_float("SIM_BASKET_RETENTION_DWELL_S", 0.75)
SIM_CAPTURE_PENDING_GRACE_S = _env_float("SIM_CAPTURE_PENDING_GRACE_S", 4.0)
# "beam" (default): collection is confirmed by the SAME basket IR latch the
# hardware uses; sim ground truth only referees (beam-vs-truth reconciliation).
# "truth": legacy ground-truth bin-dwell confirmation (debug fallback).
SIM_COLLECTION_CONFIRM_SOURCE = os.getenv(
    "SIM_COLLECTION_CONFIRM_SOURCE", "beam"
).strip().lower()
# After the basket beam clears, ignore re-breaks for this long: one bouncing
# ball is one collection, not two (run 10 double-counted every crossing).
BEAM_REARM_QUIET_S = _env_float("BEAM_REARM_QUIET_S", 0.6)
BEAM_SYMMETRY_MAX_DELTA = _env_float("BEAM_SYMMETRY_MAX_DELTA", 200.0)
NET_X_M = 0.0
NET_SIDE_CLEARANCE_M = 0.25
COURT_MAX_X_M = 11.885
COURT_MAX_Y_M = 5.485
COURT_BALL_MARGIN_M = _env_float("COURT_BALL_MARGIN_M", 3.2)
# Top-roller launcher intake: collection is credited only when the ball is
# physically inside the basket volume, not when it touches the entry lip. The
# lip/roller contact is just the launch impulse; hardware/sim confirmation
# comes from the basket beam pair (see gazebo_extras_node.py).
# Low-hopper basket (debug-log #41-#46): receiver/deck contact is only an entry
# candidate. Collection is credited after continuous residence behind the
# retention lip, inside the bin. Zone gates and dwell tracking live in
# collection_scoring; the one-shot sim-ball def guard prevents repeated counts.
IR_INTAKE_TRIGGER_THRESHOLD = 500.0
SCAN_SIDE_DURATION_S = 12.0
COLLECT_PATTERN_COLLECTION_TIMEOUT_S = _env_float("COLLECT_PATTERN_COLLECTION_TIMEOUT_S", 35.0)
# Fine-approach forward guard, measured FROM THE LIDAR (mast at x=-0.42): the
# funnel tip sits 1.30 m ahead of it, so 1.45 stops ~15 cm before contact.
COLLECT_ROUTE_FRONT_BLOCK_M = _env_float("COLLECT_ROUTE_FRONT_BLOCK_M", 1.45)
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
        self.collect_route_mission = CollectRouteMission()
        self._court_model: CourtModel | None = None
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
        # The navigator is constructed whenever Nav2 deps are importable:
        # collect_route always drives its legs via Nav2, while the lawnmower
        # sweep keeps honoring COLLECTION_USE_NAV2 via _use_nav2_lanes.
        self._nav2_lane = Nav2LaneNavigator(self) if _NAV2_AVAILABLE else None

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
        self._collect_route_run_start_count = 0
        self._last_collect_route_summary: dict = {}
        self._last_collection_event_key: tuple | None = None
        self._last_collection_scan_key: tuple | None = None
        self._collect_route_last_probe_s: float = 0.0
        self._collect_route_last_block_event_s: float = 0.0
        self._sim_true_pose: tuple[float, float, float] | None = None
        self._pose_frame_offset: tuple[float, float] | None = None
        self._last_pose_divergence_event_s: float = 0.0

        # ── cached topic values ────────────────────────────────────────────────
        self._latest_obs = BallObservationInput(visible=False, source="startup")
        self._latest_observations: list[BallObservationInput] = []
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
        self._intake_beam_broken = False
        self._intake_roller_latched = False
        self._control_command_mode = "idle"
        self._control_command_source = "startup"
        self._sim_balls: list[dict] = []
        self._sim_balls_seen = False
        self._counted_sim_ball_defs: set[str] = set()
        self._sim_retention_tracker = SimRetentionTracker(SIM_BASKET_RETENTION_DWELL_S)
        self._credit_reconciler = CreditReconciler()
        self._sim_bin_candidate_active = False
        self._sim_ball_route_owner: dict[str, int] = {}
        self._sim_ball_route_last_onboard_s: dict[str, float] = {}
        self._confirmed_route_ball_id: int | None = None
        self._lost_retained_sim_ball_defs: set[str] = set()
        self._hardware_collection_latched = False
        self._beam_rearm_at_s = 0.0
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
        self.create_subscription(
            Bool,
            "/collector/intake_beam_broken",
            self._on_intake_beam,
            10,
        )
        self.create_subscription(RobotCommand, "/robot/command", self._on_command, 10)
        self.create_subscription(String, "/sim/balls", self._on_sim_balls, 1)
        self.create_subscription(
            String, "/sim/robot_true_pose", self._on_sim_true_pose, 1
        )

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
            self._latest_observations = []
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
        self._latest_observations = observations
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

    def _on_intake_beam(self, msg: Bool) -> None:
        self._intake_beam_broken = msg.data

    def _on_command(self, msg: RobotCommand) -> None:
        self._control_command_mode = msg.mode
        self._control_command_source = msg.source

    def _on_sim_true_pose(self, msg: String) -> None:
        """Sim-only ground truth from gz pose/info (world frame ≈ map frame:
        the survey map is anchored at the world-origin start pose)."""
        try:
            d = json.loads(msg.data)
            self._sim_true_pose = (float(d["x"]), float(d["y"]), float(d["yaw"]))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._sim_true_pose = None

    def _pose_error_m(self) -> float | None:
        """Believed (SLAM/odom) pose vs sim ground truth, metres.

        The map frame is anchored at the robot's start pose while the Gazebo
        world frame is court-centred (~8.4 m apart), so the raw difference is
        a constant offset. It is calibrated at collect_route start (when
        localization is trustworthy); the reported error is the DEVIATION
        from that initial offset — i.e. actual localization drift."""
        if self._sim_true_pose is None:
            return None
        dx = self._sim_true_pose[0] - self._robot_x
        dy = self._sim_true_pose[1] - self._robot_y
        if self._pose_frame_offset is None:
            return None
        return round(
            math.hypot(dx - self._pose_frame_offset[0], dy - self._pose_frame_offset[1]),
            3,
        )

    def _on_sim_balls(self, msg: String) -> None:
        self._sim_balls_seen = True
        try:
            self._sim_balls = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._sim_balls = []
        current_defs = {
            str(ball.get("def"))
            for ball in self._sim_balls
            if ball.get("def") is not None
        }
        self._counted_sim_ball_defs.intersection_update(current_defs)
        self._lost_retained_sim_ball_defs.intersection_update(current_defs)
        self._sim_ball_route_owner = {
            ball_def: ball_id
            for ball_def, ball_id in self._sim_ball_route_owner.items()
            if ball_def in current_defs
        }
        self._sim_ball_route_last_onboard_s = {
            ball_def: last_onboard_s
            for ball_def, last_onboard_s in self._sim_ball_route_last_onboard_s.items()
            if ball_def in current_defs
        }
        self._sim_retention_tracker.retain_only(current_defs)

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
        self._monitor_retained_sim_balls()
        new_detection_frame = self._latest_obs_seq != self._mapped_obs_seq
        mapping_observation = (
            observation
            if new_detection_frame
            else BallObservationInput(visible=False, source="observation_already_mapped")
        )
        mapped_observation = self._mapping_observation(mapping_observation)
        self._mapped_obs_seq = self._latest_obs_seq
        collect_route_create_allowed = (
            self.control_mode != "collect_route"
            or not self.collect_route_mission.freeze_initial_plan
            or self.collect_route_mission.phase in {"idle", "scan"}
        )
        mapped_ball_id, is_new_ball = self.ball_map.update(
            mapped_observation,
            now,
            allow_create=collect_route_create_allowed,
        )
        control_mapping_observation = self._mapping_observation(observation)
        ignored_new_detections = 0
        if (
            self.control_mode == "collect_route"
            and new_detection_frame
            and not collect_route_create_allowed
            and mapped_ball_id is None
            and mapped_observation.visible
            and mapped_observation.world_x_m is not None
            and mapped_observation.world_y_m is not None
        ):
            ignored_new_detections += 1

        # collect_route maps EVERY detection of the frame, not only the
        # nearest: the 360° scan must register balls behind/beside the closest
        # one. When the initial plan is frozen, detections after the scan may
        # still refine existing entries but cannot add new route candidates.
        if new_detection_frame and self.control_mode == "collect_route":
            for extra in self._latest_observations:
                if extra is not self._latest_obs:
                    extra_mapped = self._mapping_observation(extra)
                    extra_id, _ = self.ball_map.update(
                        extra_mapped,
                        now,
                        allow_create=collect_route_create_allowed,
                    )
                    if (
                        not collect_route_create_allowed
                        and extra_id is None
                        and extra_mapped.visible
                        and extra_mapped.world_x_m is not None
                        and extra_mapped.world_y_m is not None
                    ):
                        ignored_new_detections += 1

        if ignored_new_detections:
            self._record_collection_event(
                "route_detection_ignored",
                reason="frozen_initial_plan",
                ignored_count=ignored_new_detections,
                mission_phase=self.collect_route_mission.phase,
            )

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
        elif effective_mode == "collect_route":
            command = self._collect_route_command_for_mode(
                effective_mode,
                self._collect_route_target_observation(control_mapping_observation),
            )
        elif effective_mode == "map_left_side":
            command = self._map_mission_command_for_mode(effective_mode)
        elif effective_mode in self._MANUAL_MODES:
            command = self._manual_move_command(effective_mode)
        else:
            command = self._collector_command_for_mode(effective_mode, control_observation)

        self.collection_confirmed = self._check_collection(command)
        if self._sim_balls_seen and SIM_COLLECTION_CONFIRM_SOURCE != "truth":
            reconcile = self._credit_reconciler.poll(now)
            if reconcile is not None:
                self._record_collection_event(
                    reconcile.pop("event"), severity="critical", **reconcile
                )
        if self._sim_bin_candidate_active and not self.collection_confirmed:
            # The ball has crossed the lip. Hold the chassis still while the
            # intake remains active and prove that the ball stays in the bin.
            command = ConceptACommand(
                state=command.state,
                base=BaseCommand(0.0, 0.0),
                collector=command.collector,
            )
        if self.collection_confirmed:
            if self._confirmed_route_ball_id is not None:
                collected_id = self._confirmed_route_ball_id
                self.ball_map.set_state(collected_id, "collected")
            elif self.control_mode == "collect_route":
                # The route mission attributes the credit to its own stop and
                # sets the map state itself; marking the nearest entry here as
                # well retired a SECOND ball per capture on the beam path.
                collected_id = None
            else:
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
        if (
            previous_mode in {"collect", "collect_route"}
            and new_mode != previous_mode
            and self._nav2_lane is not None
        ):
            self._record_collection_event("nav2_goal_cancel", reason=f"mode_exit:{new_mode}")
            self._nav2_lane.reset()
        if previous_mode == "collect_route":
            self._last_collect_route_summary = self._build_collect_route_summary(
                status="complete" if self.collect_route_mission.is_done else "stopped"
            )
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
        self.collect_route_mission.reset()
        self.ball_map.max_create_distance_override_m = None
        self._reset_collect_pattern()
        self.scan_side_started_at = None
        self._collect_start_time = None
        self.active_mapped_target_id = None
        self._collection_scan_completion_reported = False
        self.get_logger().info(f"mode → {new_mode}")
        if new_mode in {"collect", "collect_route"}:
            self._collection_events.clear()
            self._collection_event_started_at = self._runtime_seconds()
            if self._nav2_lane is not None:
                self._nav2_lane.reset()
            self._last_collection_event_key = None
            self._last_collection_scan_key = None
            if new_mode == "collect_route":
                self._collect_route_run_start_count = self.collection_count
                self._last_collect_route_summary = {}
                self._sim_ball_route_owner.clear()
                self._sim_ball_route_last_onboard_s.clear()
                self._credit_reconciler = CreditReconciler()
            self._record_collection_event("mode_enter", requested=self._control_command_mode)
        return True

    _MANUAL_MODES = frozenset({
        "move_forward", "move_backward", "move_left", "move_right", "turn_180",
        "move_forward_left", "move_forward_right",
        "move_backward_left", "move_backward_right",
    })
    _AUTONOMOUS_MODES = frozenset({"map_court", "map_left_side", "collect_pattern", "collect", "collect_one", "collect_route", "search", "scan_side"})

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

    def _collect_route_observation(
        self, observation: BallObservationInput
    ) -> BallObservationInput:
        """Same-side/in-court gate against the SURVEYED court geometry.

        The legacy _same_side_search_observation assumes the net at world x=0;
        in the SLAM map frame the net sits wherever the survey found it
        (run-3 incident: x≈8.08), so with a court model loaded the real net
        line and fence polygon are authoritative."""
        if self._court_model is None:
            return self._same_side_search_observation(observation)
        if (
            not observation.visible
            or observation.world_x_m is None
            or observation.world_y_m is None
        ):
            return observation
        if not self._court_model.same_side(
            self._robot_x, self._robot_y, observation.world_x_m, observation.world_y_m
        ):
            return BallObservationInput(visible=False, source="across_net_filtered")
        if not self._court_model.contains(observation.world_x_m, observation.world_y_m):
            return BallObservationInput(visible=False, source="out_of_court_filtered")
        return observation

    def _collect_route_target_observation(
        self, nearest_observation: BallObservationInput
    ) -> BallObservationInput:
        """Prefer the detection nearest the CURRENT TARGET over the detection
        nearest the robot.

        Run-4 finding (archive/collection-route-debug-log-el #8): with ball clusters,
        the nearest-to-robot detection often refers to a DIFFERENT ball, so
        the target 'never gets a live sighting' and real balls were declared
        missing (seen_count 174-757). The controller has the whole frame —
        pick the detection closest to the mission's pursued ball."""
        target = self.collect_route_mission.current_target_xy
        fresh = (
            self._latest_obs_received_at > 0.0
            and self._runtime_seconds() - self._latest_obs_received_at
            <= PERCEPTION_OBSERVATION_TIMEOUT_S
        )
        if target is None or not fresh or not self._latest_observations:
            return self._collect_route_observation(nearest_observation)
        best = min(
            self._latest_observations,
            key=lambda obs: math.hypot(
                (obs.world_x_m or 1e9) - target[0], (obs.world_y_m or 1e9) - target[1]
            ),
        )
        if (
            best.world_x_m is None
            or math.hypot(best.world_x_m - target[0], best.world_y_m - target[1]) > 1.5
        ):
            return self._collect_route_observation(nearest_observation)
        return self._collect_route_observation(best)

    def _collect_route_command_for_mode(
        self, mode: str, observation: BallObservationInput
    ) -> ConceptACommand:
        mission = self.collect_route_mission
        now = self._runtime_seconds()
        if self._on_mode_changed(mode):
            self.ball_map.reset()
            self._collect_start_time = now
            self._court_model = CourtModel.from_boundary_file(DEFAULT_BOUNDARY_FILE)
            if self._court_model is None:
                self.get_logger().warning(
                    f"collect_route: no usable court model at {DEFAULT_BOUNDARY_FILE}; "
                    "approach poses degrade to direct (no lateral fence/net handling)"
                )
            # Register balls out to camera range during the 360° scan.
            self.ball_map.max_create_distance_override_m = mission.planner_cfg.scan_range_m
            # Calibrate the map↔world frame offset while localization is
            # still trustworthy; _pose_error_m measures drift from here.
            if self._sim_true_pose is not None:
                self._pose_frame_offset = (
                    self._sim_true_pose[0] - self._robot_x,
                    self._sim_true_pose[1] - self._robot_y,
                )
            mission.start(self._robot_pose_2d())

        if self._nav2_lane is None:
            # collect_route drives its legs via Nav2 exclusively — fail loud,
            # never silently substitute the P-controller for the legs.
            self._record_collection_event(
                "nav2_unavailable",
                detail="nav2_msgs/rclpy.action not importable; collect_route stopped (no fallback)",
            )
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )

        self._assign_sim_ball_route_owners()
        command = mission.update(
            observation,
            self.collection_confirmed,
            TIME_STEP_S,
            self._robot_pose_2d(),
            self.behavior,
            self.ball_map,
            now,
            self._nav2_lane.state.value,
            self._court_model,
            self._confirmed_route_ball_id if self.collection_confirmed else None,
            self._pending_sim_capture_ball_id(now),
        )

        # LiDAR forward guard for the fine approach: the capture stream owns
        # the wheels here (priority 100, no costmap), and the lidar plane
        # cannot see balls — anything solid dead ahead is net/fence/furniture.
        # Run 3 pushed into the net this way; stop forward motion and let the
        # approach timeout convert the blockage into a retry/skip.
        if command.base.linear_speed_m_s > 0.0 and mission.phase == "approach":
            front_m = self._front_range_m()
            if front_m is not None and front_m < COLLECT_ROUTE_FRONT_BLOCK_M:
                command = ConceptACommand(
                    state=command.state,
                    base=BaseCommand(0.0, command.base.angular_speed_rad_s),
                    collector=command.collector,
                )
                if now - self._collect_route_last_block_event_s >= 2.0:
                    self._collect_route_last_block_event_s = now
                    self._record_collection_event(
                        "route_approach_blocked",
                        ball_id=mission.current_ball_id,
                        front_range_m=front_m,
                    )

        # Scan-range override only lives while the mission scans.
        if not mission.scanning:
            self.ball_map.max_create_distance_override_m = None

        # Nav2 wiring: the mission exposes the goal, the controller owns the
        # navigator (same split as the lawnmower sweep).
        goal = mission.nav_goal
        if goal is not None:
            self._nav2_lane.request(*goal)
            if self._nav2_lane.state == LaneNavState.UNAVAILABLE:
                self._record_collection_event(
                    "nav2_unavailable",
                    detail="navigate_to_pose action server not up; collect_route stopped (no fallback)",
                    target_x_m=goal[0],
                    target_y_m=goal[1],
                )
        elif self._nav2_lane.busy:
            self._nav2_lane.cancel()

        self.active_mapped_target_id = mission.current_ball_id
        for event_type, fields in mission.drain_events():
            self._record_collection_event(event_type, **fields)

        # Ground-truth capture probe (sim only): while a leg or fine approach
        # runs, record where the nearest physical ball actually is in the
        # robot frame every ~2 s. Evidence trail for capture stalls (#2) and
        # for nav freezes (#11: suspected ball wedged under the chassis —
        # Nav2 cannot see balls, so legs plow straight through them).
        if mission.phase in ("approach", "nav") and self._sim_balls_seen:
            if now - self._collect_route_last_probe_s >= 2.0:
                self._collect_route_last_probe_s = now
                probe = self._nearest_sim_ball_local()
                # Lock-vs-truth: distance from the mission's locked world point
                # to the nearest physical ball. Quantifies how far the approach
                # is steering from reality (lateral-capture-error investigation).
                lock_error_m = None
                locked = mission._locked_world
                if locked is not None and self._sim_balls:
                    lock_error_m = round(
                        min(
                            math.hypot(b["x"] - locked[0], b["y"] - locked[1])
                            for b in self._sim_balls
                        ),
                        3,
                    )
                self._record_collection_event(
                    "route_capture_probe",
                    ball_id=mission.current_ball_id,
                    mission_phase=mission.phase,
                    behavior_state=self.behavior.state.value,
                    approach_elapsed_s=mission._approach_elapsed_s,
                    intake_beam_broken=self._intake_beam_broken,
                    intake_roller_latched=self._intake_roller_latched,
                    locked_world=list(locked) if locked else None,
                    lock_error_m=lock_error_m,
                    pose_error_m=self._pose_error_m(),
                    nearest_sim_ball=probe,
                )

        # Localization watchdog (sim only): a believed-vs-truth pose gap of
        # metres invalidates every downstream decision (goals, side checks,
        # ball projections) — run-6 rejected all goals with the robot
        # believed at the fence. Loud event, throttled.
        pose_error = self._pose_error_m()
        if pose_error is not None and pose_error > 1.0:
            if now - self._last_pose_divergence_event_s >= 5.0:
                self._last_pose_divergence_event_s = now
                self._record_collection_event(
                    "pose_divergence",
                    pose_error_m=pose_error,
                    believed=[round(self._robot_x, 2), round(self._robot_y, 2)],
                    true=[round(self._sim_true_pose[0], 2), round(self._sim_true_pose[1], 2)],
                    pose_source=self._pose_source,
                )

        if mission.is_done and not mission._complete_reported:
            mission._complete_reported = True
            self.get_logger().info(
                f"collect_route complete; total={self.collection_count}"
            )
            self._publish_command("idle", "controller-collect-route-complete")
        return command

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
        if self.control_mode == "collect_route" or event_type.startswith("route_"):
            route = self.collect_route_mission.telemetry()
            route_goal = self.collect_route_mission.nav_goal
            event.update(
                {
                    "route_phase": route.get("phase"),
                    "route_current_ball_id": route.get("current_ball_id"),
                    "route_remaining": route.get("remaining"),
                    "route_stop_count": route.get("stop_count"),
                    "route_failed_ball_ids": route.get("failed_ball_ids"),
                    "route_freeze_initial_plan": route.get("freeze_initial_plan"),
                    "route_nav_attempts": route.get("nav_attempts"),
                    "route_nav_goal": (
                        [round(route_goal[0], 3), round(route_goal[1], 3), round(route_goal[2], 3)]
                        if route_goal is not None
                        else None
                    ),
                    "route_stops": route.get("route"),
                }
            )
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
        self._sim_bin_candidate_active = False
        self._confirmed_route_ball_id = None
        if self._sim_balls_seen and SIM_COLLECTION_CONFIRM_SOURCE == "truth":
            # Legacy ground-truth confirmation (debug fallback): credit on the
            # bin retention dwell. NOT gated on intake_enabled — a ball
            # launched at the tail of an aborted capture settles into the bin
            # AFTER the roller stops (run 8) and must still be credited.
            if not self._sim_balls:
                return False
            return self._sim_retention_step(credit=True)
        if self._sim_balls_seen:
            # Beam-primary (default): the confirmation signal below is the
            # SAME basket IR latch hardware uses — the sim beams feed
            # /ir/readings — so a sim run certifies the hardware pipeline.
            # Ground truth only referees: retention events plus beam-vs-truth
            # count reconciliation, never a credit.
            self._sim_retention_step(credit=False)

        if not command.collector.intake_enabled:
            self._hardware_collection_latched = False
            return False

        # Basket IR beam pair: count once per crossing. A ball bouncing down
        # the tray breaks/clears the beam more than once within a fraction of
        # a second (run 10: every real crossing double-counted), so after the
        # beam clears the latch re-arms only after a quiet period.
        now_s = self._runtime_seconds()
        # A real tray crossing is centred by the funnel: BOTH sensors report
        # near-equal ranges (run 12: 631/625, 638/633). One-sided/asymmetric
        # hits are court balls bouncing beside the robot seen through the
        # open wire mesh (869/576, 901/233) — never a collection.
        hardware_triggered = (
            self._ir_left > IR_INTAKE_TRIGGER_THRESHOLD
            and self._ir_right > IR_INTAKE_TRIGGER_THRESHOLD
            and abs(self._ir_left - self._ir_right) <= BEAM_SYMMETRY_MAX_DELTA
        )
        if not hardware_triggered:
            if self._hardware_collection_latched:
                self._beam_rearm_at_s = now_s + BEAM_REARM_QUIET_S
            self._hardware_collection_latched = False
            return False
        if self._hardware_collection_latched:
            return True
        self._hardware_collection_latched = True
        if now_s < self._beam_rearm_at_s:
            # Same physical crossing still settling: re-latch without a count.
            return True
        self.collection_count += 1
        if self._sim_balls_seen:
            self._credit_reconciler.on_beam_credit(now_s)
        self._record_collection_event(
            "beam_collection_credit",
            ir_left=round(self._ir_left, 1),
            ir_right=round(self._ir_right, 1),
        )
        return True

    def _sim_ball_local(self, ball: dict) -> tuple[float, float, float]:
        """Ball position in the robot frame for onboard-zone scoring.

        Prefers the ground-truth local coordinates published by
        gazebo_extras. The fallback subtracts the SLAM map pose from
        odom-anchored ball coordinates; that frame gap grows with odometry
        drift and cost run 8 three uncounted basket balls — kept only for
        replaying older fixtures.
        """
        if "local_x" in ball:
            return (
                float(ball["local_x"]),
                float(ball["local_y"]),
                float(ball.get("local_z", ball.get("z", 0.0))),
            )
        ori_cos = math.cos(self._robot_yaw)
        ori_sin = math.sin(self._robot_yaw)
        dx = ball["x"] - self._robot_x
        dy = ball["y"] - self._robot_y
        return (
            ori_cos * dx + ori_sin * dy,
            -ori_sin * dx + ori_cos * dy,
            # Ball world z ~= height above court (flat ground, robot z ~ 0).
            float(ball.get("z", 0.0)),
        )

    def _sim_retention_step(self, credit: bool) -> bool:
        """Track ground-truth basket retention for every sim ball.

        credit=True (truth mode): a completed dwell IS the collection credit.
        credit=False (beam-primary): retention only feeds the referee — the
        credit comes from the IR beam latch, exactly like hardware.
        """
        now = self._runtime_seconds()
        for ball in self._sim_balls:
            ball_def = str(ball.get("def", ""))
            if ball_def in self._counted_sim_ball_defs:
                continue
            lx, ly, bz = self._sim_ball_local(ball)
            zone = onboard_ball_zone(lx, ly, bz)
            retention = self._sim_retention_tracker.update(ball_def, zone, now)
            if credit and zone == "bin" and not retention.retained:
                self._sim_bin_candidate_active = True
            if retention.event is not None:
                self._record_collection_event(
                    retention.event,
                    ball_def=ball_def,
                    zone=zone,
                    previous_zone=retention.previous_zone,
                    local_x_m=round(lx, 3),
                    local_y_m=round(ly, 3),
                    ball_z_m=round(bz, 3),
                    dwell_s=round(retention.dwell_s, 3),
                    required_dwell_s=SIM_BASKET_RETENTION_DWELL_S,
                    route_ball_id=self._sim_ball_route_owner.get(ball_def),
                )
            if retention.retained:
                owner = self._sim_ball_route_owner.pop(ball_def, None)
                self._sim_ball_route_last_onboard_s.pop(ball_def, None)
                collected_msg = String()
                collected_msg.data = ball_def
                self._pub_ball_collected.publish(collected_msg)
                self._counted_sim_ball_defs.add(ball_def)
                if not credit:
                    self._credit_reconciler.on_truth_retained(now)
                    continue
                self._confirmed_route_ball_id = owner
                self.collection_count += 1
                self._record_collection_event(
                    "sim_collection_credit",
                    ball_def=ball_def,
                    zone="bin",
                    local_x_m=round(lx, 3),
                    local_y_m=round(ly, 3),
                    ball_z_m=round(bz, 3),
                    dwell_s=round(retention.dwell_s, 3),
                    confirmation="stable_behind_retention_lip",
                    route_ball_id=owner,
                )
                return True
        return False

    def _assign_sim_ball_route_owners(self) -> None:
        """Bind an onboard sim ball to its route stop before retention completes."""
        route_ball_id = self.collect_route_mission.capture_ball_id
        if not self._sim_balls_seen or route_ball_id is None:
            return
        now = self._runtime_seconds()
        for ball in self._sim_balls:
            ball_def = str(ball.get("def", ""))
            if (
                not ball_def
                or ball_def in self._counted_sim_ball_defs
            ):
                continue
            lx, ly, bz = self._sim_ball_local(ball)
            zone = onboard_ball_zone(lx, ly, bz)
            if zone not in {"deck", "receiver", "bin"}:
                continue
            if ball_def not in self._sim_ball_route_owner:
                self._sim_ball_route_owner[ball_def] = route_ball_id
                self._record_collection_event(
                    "basket_owner_assigned",
                    ball_def=ball_def,
                    zone=zone,
                    local_x_m=round(lx, 3),
                    local_y_m=round(ly, 3),
                    ball_z_m=round(bz, 3),
                    route_ball_id=route_ball_id,
                )
            # A deck-parked ball (beside the bin, |y| > bin half-width) can
            # never reach the retention zone: it must not keep extending the
            # capture-pending grace, or the missing decision stalls for the
            # full 35 s approach budget instead of the 6 s phantom gate.
            if zone in {"receiver", "bin"}:
                self._sim_ball_route_last_onboard_s[ball_def] = now

    def _pending_sim_capture_ball_id(self, now_s: float) -> int | None:
        """Return the current stop only while its linked ball is still onboard."""
        if SIM_COLLECTION_CONFIRM_SOURCE != "truth":
            # Hardware parity: the real robot has no ground-truth "ball is
            # onboard" signal, so beam-primary runs must not defer the
            # missing decision on one either.
            return None
        route_ball_id = self.collect_route_mission.capture_ball_id
        if route_ball_id is None:
            return None
        for ball_def, owner_ball_id in self._sim_ball_route_owner.items():
            if owner_ball_id != route_ball_id:
                continue
            last_onboard_s = self._sim_ball_route_last_onboard_s.get(ball_def)
            if (
                last_onboard_s is not None
                and now_s - last_onboard_s <= SIM_CAPTURE_PENDING_GRACE_S
            ):
                return route_ball_id
        return None

    def _monitor_retained_sim_balls(self) -> None:
        """Log a critical post-credit escape instead of hiding it as success."""
        if not self._sim_balls_seen or not self._counted_sim_ball_defs:
            return
        for ball in self._sim_balls:
            ball_def = str(ball.get("def", ""))
            if (
                ball_def not in self._counted_sim_ball_defs
                or ball_def in self._lost_retained_sim_ball_defs
            ):
                continue
            lx, ly, bz = self._sim_ball_local(ball)
            zone = onboard_ball_zone(lx, ly, bz)
            if retained_ball_still_in_bin(lx, ly, bz):
                continue
            self._lost_retained_sim_ball_defs.add(ball_def)
            self._record_collection_event(
                "basket_retention_lost",
                severity="critical",
                ball_def=ball_def,
                zone=zone,
                local_x_m=round(lx, 3),
                local_y_m=round(ly, 3),
                ball_z_m=round(bz, 3),
            )

    def _nearest_sim_ball_local(self) -> dict | None:
        """Nearest ground-truth ball in the robot frame (sim only): the same
        transform _sim_retention_step uses, so the numbers are directly
        comparable to the basket volume gates."""
        if not self._sim_balls:
            return None
        best: dict | None = None
        best_dist = math.inf
        for ball in self._sim_balls:
            lx, ly, bz = self._sim_ball_local(ball)
            dist = math.hypot(lx, ly)
            if dist < best_dist:
                best_dist = dist
                best = {
                    "def": str(ball.get("def", "")),
                    "local_x_m": round(lx, 3),
                    "local_y_m": round(ly, 3),
                    "z_m": round(bz, 3),
                    "already_counted": str(ball.get("def", "")) in self._counted_sim_ball_defs,
                }
        return best

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

        requested = command.collector
        if command.state == CollectorState.CAPTURE:
            # Committed ingest: the wheels MUST already spin when the ball
            # meets them. Gating capture on the throat beam deadlocked live
            # (debug-log #44): with the dual-wheel geometry a plowed ball
            # reaches at most x=0.678 and only grazes the 0.670 beam, so the
            # latch never set and the ball was bulldozed by dead wheels.
            collector_enabled = requested.intake_enabled
            self._intake_roller_latched = True
        elif command.state == CollectorState.APPROACH:
            self._intake_roller_latched |= self._intake_beam_broken
            collector_enabled = requested.intake_enabled and self._intake_roller_latched
        else:
            # Reverse-clear is intentionally not gated by the intake beam.
            collector_enabled = requested.intake_enabled
            if command.state != CollectorState.REVERSE_CLEAR:
                self._intake_roller_latched = False

        col = CollectorCmd()
        col.lift_wheel_speed = (
            float(requested.lift_wheel_speed) if collector_enabled else 0.0
        )
        col.intake_enabled = collector_enabled
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
        route: list[dict] = []
        planned_order: dict[int, int] | None = None
        insertions = 0
        if self.control_mode == "collect_route":
            route, planned_order = self.collect_route_mission.route_export(
                (self._robot_x, self._robot_y)
            )
            insertions = self.collect_route_mission.insertion_count
        balls = self.ball_map.to_console_balls(
            self._robot_x,
            self.active_mapped_target_id,
            now=self._runtime_seconds(),
            planned_order=planned_order,
        )
        confirmed = [b for b in balls if b["confirmed"] and b["side"] != "across_net"]
        metrics: dict[str, object] = {
            "balls_mapped": len(balls),
            "balls_confirmed": len(confirmed),
            "balls_collectable": len(confirmed),
        }
        if len(route) > 1:
            metrics["total_distance_m"] = remaining_route_length_m(
                (self._robot_x, self._robot_y), self.collect_route_mission.stops
            )
            metrics["planned_replans"] = insertions
        return {
            "balls": balls,
            "active_target_id": self.active_mapped_target_id,
            "route": route,
            "robot": {
                "x_m": round(self._robot_x, 3),
                "y_m": round(self._robot_y, 3),
                "yaw_rad": round(self._robot_yaw, 4),
            },
            "camera_fov_rad": round(cfg.supervised_fov_rad, 4),
            "camera_max_range_m": round(cfg.supervised_max_range_m, 2),
            "metrics": metrics,
        }

    def _build_collect_route_summary(self, status: str = "running") -> dict:
        route = self.collect_route_mission.telemetry()
        stops = route.get("stops", {})
        missing = int(stops.get("missing", 0))
        skipped = int(stops.get("skipped", 0))
        return {
            "run_id": self._run_id,
            "status": status,
            "planned": int(route.get("planned_total", 0)),
            "basket_retained": max(
                0, self.collection_count - self._collect_route_run_start_count
            ),
            # Sim referee counters: beam credits vs ground-truth retention.
            # Diverging values mean the beam is lying (bounce-outs, blocking).
            "beam_credits": self._credit_reconciler.beam_count,
            "truth_retained": self._credit_reconciler.truth_count,
            "route_collected": int(stops.get("collected", 0)),
            "failed": missing + skipped,
            "missing": missing,
            "skipped": skipped,
            "remaining": int(route.get("remaining", 0)),
            "active_ball_id": route.get("current_ball_id"),
            "failed_ball_ids": list(route.get("failed_ball_ids", [])),
        }

    def _collect_route_summary_for_status(self) -> dict:
        if self.control_mode == "collect_route":
            return self._build_collect_route_summary(
                status="complete" if self.collect_route_mission.is_done else "running"
            )
        return self._last_collect_route_summary

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
            "collect_route": self.collect_route_mission.telemetry(),
            "collection_run": self._collect_route_summary_for_status(),
            "pose_error_m": self._pose_error_m(),
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
