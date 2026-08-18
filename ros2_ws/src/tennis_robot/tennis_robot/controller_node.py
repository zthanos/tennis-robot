"""ROS 2 controller node: orchestrates all robot behavior.

This node contains all logic previously in BallDetectorController (ball_detector.py),
with every direct Webots API call replaced by a ROS 2 topic read or write.
The underlying behavior modules (collector.py, survey.py, ball_map.py, etc.)
are imported unchanged from the controllers/ tree.

Subscribes:
  /perception/ball_detections (tennis_robot_msgs/BallDetectionArray)
  /perception/diagnostics (std_msgs/String, JSON; operator diagnostics only)
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
from tennis_robot.collection_executor_node_factory import (
    CollectionExecutorNodeCache,
    CollectionExecutorNodeFactory,
)
from tennis_robot.collection_route_executor import ExecutorState
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
from tennis_robot.collector_driver import GazeboCollectorDriver
from tennis_robot.collector_interface import CollectorInterface
from tennis_robot.config_utils import _env_float
from tennis_robot.lidar_processor import extract_ball_candidates, front_range_m as lidar_front_range_m
from tennis_robot.mapping import (
    DEFAULT_BOUNDARY_FILE,
    LidarSurveyBoundaryProvider,
    MapLeftSideMission,
    ServiceLineDistributionScanMission,
)
from tennis_robot.motion_controller import MOTION_COMMAND_TOPIC
from tennis_robot.perception_covariance_calibration import PerceptionSpatialValidationConfig, validate_spatial_metadata
from tennis_robot.perception_diagnostics import format_no_targets_diagnostic
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
COLLECTION_ROUTE_MINIMUM_DRAIN_S = _env_float(
    "COLLECTION_ROUTE_MINIMUM_DRAIN_S", 1.5
)
COLLECTION_ROUTE_MAXIMUM_DRAIN_S = _env_float(
    "COLLECTION_ROUTE_MAXIMUM_DRAIN_S", 5.0
)
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
    # Class seam used by the node-wiring unit test; production uses the real
    # Phase 6D.3 factory without a runtime feature flag or fallback.
    collection_executor_factory_type = CollectionExecutorNodeFactory

    def __init__(self) -> None:
        super().__init__("tennis_robot_controller")

        # ── behavior modules (unchanged) ──────────────────────────────────────
        self.behavior = ConceptACollectorBehavior(ConceptAConfig.from_env())
        self.search_behavior = HalfCourtSearchBehavior.from_env()
        self.survey_behavior = Ros2LidarCourtSurvey.from_env()
        self.ball_map = BallMap(BallMapConfig(court_ball_margin_m=COURT_BALL_MARGIN_M))
        self.collect_one_mission = CollectOneMission()
        self._perception_spatial_validation_config: PerceptionSpatialValidationConfig | None = None
        self._last_perception_rejection_reason: str | None = None
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
        # Actual ground speed from /odom twist (valid even while collect_route is
        # hands-off and the Python base command is idle).
        self._robot_speed_mps: float = 0.0
        self._collection_event_log = Path(
            os.getenv(
                "COLLECTION_EVENT_LOG_FILE",
                str(Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace")) / "runtime" / "collection_events.jsonl"),
            )
        )
        self._run_id = f"{int(self.started_at)}-{os.getpid()}"
        self._collect_route_run_start_count = 0
        self._last_collect_route_summary: dict = {}
        self._collect_route_executor = None
        self._collect_route_executor_factory = None
        self._collect_route_execution_truth_snapshot: dict = {}
        self._collect_route_executor_events: list[dict] = []
        self._collect_route_executor_complete_reported = False
        self._collection_executor_cache = CollectionExecutorNodeCache()
        self._last_collection_event_key: tuple | None = None
        self._last_collection_scan_key: tuple | None = None
        self._collect_route_last_probe_s: float = 0.0
        self._collect_route_last_block_event_s: float = 0.0
        self._sim_true_pose: tuple[float, float, float] | None = None
        self._pose_frame_offset: tuple[float, float] | None = None
        self._pose_frame_yaw_offset: float | None = None
        self._last_pose_divergence_event_s: float = 0.0
        self._collect_route_confirmations: list[dict] = []
        self._collect_route_run_history: list[dict] = []

        # ── cached topic values ────────────────────────────────────────────────
        self._latest_obs = BallObservationInput(visible=False, source="startup")
        self._latest_ball_detections_msg = None
        self._latest_perception_diagnostics: dict = {}
        self._latest_observations: list[BallObservationInput] = []
        self._latest_obs_received_at = 0.0
        self._latest_obs_seq = 0
        self._mapped_obs_seq = 0
        self._latest_survey_vision: SurveyVision | None = None
        self._latest_camera_balls: list[dict] = []
        self._latest_camera_balls_received_at = 0.0
        self._last_bad_perception_frame = ""
        self._lidar_ranges: list[float] | None = None
        self._latest_scan_msg = None
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._ir_left = 0.0
        self._ir_right = 0.0
        self._intake_beam_broken = False
        self._entry_beam_previous = False
        self._entry_beam_sequence = 0
        self._last_credited_entry_sequence = 0
        self._confirmed_beam_broken = False
        self._collect_route_collector_active = False
        self._intake_roller_latched = False
        self._control_command_mode = "idle"
        self._control_command_source = "startup"
        self._sim_balls: list[dict] = []
        self._sim_balls_seen = False
        self._counted_sim_ball_defs: set[str] = set()
        self._sim_retention_tracker = SimRetentionTracker(SIM_BASKET_RETENTION_DWELL_S)
        self._credit_reconciler = CreditReconciler()
        self._sim_bin_candidate_active = False
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
        self.create_subscription(
            String, "/perception/diagnostics", self._on_perception_diagnostics, 10
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

    def _on_perception_diagnostics(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict) and payload.get("schema_version") == 1:
            self._latest_perception_diagnostics = payload

    def _on_ball_detections(self, msg: BallDetectionArray) -> None:
        """Consume the canonical sim/real OAK-D perception contract."""
        self._latest_ball_detections_msg = msg
        self._collection_executor_cache.latest_ball_detections = msg
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

        if not msg.spatial_targets_healthy:
            self._latest_obs = BallObservationInput(visible=False, source="spatial_targets_unhealthy")
            self._latest_observations = []
            self._latest_camera_balls = []
            self._latest_obs_seq += 1
            return

        config = self._perception_spatial_validation_config
        if config is None:
            self._latest_obs = BallObservationInput(visible=False, source="perception_metadata_rejected")
            self._last_perception_rejection_reason = "validation_config_invalid"
            self._latest_observations = []; self._latest_camera_balls = []; self._latest_obs_seq += 1
            return

        rgb_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        spatial = [d for d in msg.detections if d.has_spatial]
        for detection in spatial:
            depth_s = float(detection.matched_depth_stamp.sec) + float(detection.matched_depth_stamp.nanosec) * 1e-9
            reason = validate_spatial_metadata(rgb_s, depth_s, tuple(detection.position_covariance), config)
            if reason:
                self._latest_obs = BallObservationInput(visible=False, source="perception_metadata_rejected")
                self._last_perception_rejection_reason = reason
                self._latest_observations = []; self._latest_camera_balls = []; self._latest_obs_seq += 1
                return

        try:
            camera_to_map = self._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, RclpyTime.from_msg(msg.header.stamp)
            )
        except Exception:
            self._latest_obs = BallObservationInput(visible=False, source="perception_tf_rejected")
            self._latest_observations = []
            self._latest_camera_balls = []
            self._latest_obs_seq += 1
            return
        q = camera_to_map.transform.rotation
        tx, ty, tz = camera_to_map.transform.translation.x, camera_to_map.transform.translation.y, camera_to_map.transform.translation.z

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
            # Rotate camera optical XYZ with TF evaluated at the RGB stamp.
            x, y, z = float(detection.position_x), float(detection.position_y), float(detection.position_z)
            xx, yy, zz = q.x*q.x, q.y*q.y, q.z*q.z
            xy, xz, yz, wx, wy, wz = q.x*q.y, q.x*q.z, q.y*q.z, q.w*q.x, q.w*q.y, q.w*q.z
            world_x = tx + (1 - 2*(yy + zz))*x + 2*(xy - wz)*y + 2*(xz + wy)*z
            world_y = ty + 2*(xy + wz)*x + (1 - 2*(xx + zz))*y + 2*(yz - wx)*z
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
        self._latest_scan_msg = msg
        self._collection_executor_cache.latest_scan = msg
        self._lidar_ranges = [float(r) for r in msg.ranges]
        self._lidar_angle_min = float(msg.angle_min)
        self._lidar_angle_increment = float(msg.angle_increment)

    def _on_odom(self, msg: Odometry) -> None:
        # Velocity always comes from /odom regardless of pose source (SLAM TF
        # supplies pose, not twist), so capture it before the pose short-circuit.
        self._robot_speed_mps = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
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
        self._confirmed_beam_broken = (
            self._ir_left > IR_INTAKE_TRIGGER_THRESHOLD
            and self._ir_right > IR_INTAKE_TRIGGER_THRESHOLD
            and abs(self._ir_left - self._ir_right) <= BEAM_SYMMETRY_MAX_DELTA
        )

    def _on_intake_beam(self, msg: Bool) -> None:
        broken = bool(msg.data)
        if broken and not self._entry_beam_previous:
            self._entry_beam_sequence += 1
        self._entry_beam_previous = broken
        self._intake_beam_broken = broken

    def _consume_entry_for_confirmation(self) -> bool:
        """Allow one confirmed-basket credit for each physical intake entry."""
        if self._entry_beam_sequence <= self._last_credited_entry_sequence:
            return False
        self._last_credited_entry_sequence = self._entry_beam_sequence
        return True

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

    def _pose_yaw_error_rad(self) -> float | None:
        if self._sim_true_pose is None or self._pose_frame_yaw_offset is None:
            return None
        raw_error = self._sim_true_pose[2] - self._robot_yaw
        return round(
            math.atan2(
                math.sin(raw_error - self._pose_frame_yaw_offset),
                math.cos(raw_error - self._pose_frame_yaw_offset),
            ),
            4,
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
        mapped_ball_id, is_new_ball = self.ball_map.update(
            mapped_observation,
            now,
            allow_create=True,
        )
        control_mapping_observation = self._mapping_observation(observation)

        # Keep the console ball map populated from every camera detection.  The
        # executor owns its own immutable scan snapshot and never consumes this
        # legacy BallMap registry.
        if new_detection_frame and self.control_mode == "collect_route":
            for extra in self._latest_observations:
                if extra is not self._latest_obs:
                    extra_mapped = self._mapping_observation(extra)
                    self.ball_map.update(
                        extra_mapped,
                        now,
                        allow_create=True,
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
            command = self._collect_route_command_for_mode(effective_mode)
        elif effective_mode == "map_left_side":
            command = self._map_mission_command_for_mode(effective_mode)
        elif effective_mode in self._MANUAL_MODES:
            command = self._manual_move_command(effective_mode)
        else:
            command = self._collector_command_for_mode(effective_mode, control_observation)

        if effective_mode == "collect_route":
            # Route success remains owned exclusively by the executor plan and
            # crossing telemetry. Independently account for the physical
            # CONFIRMED beam while the route collector is running so production
            # telemetry reports what actually entered the basket. This does not
            # mutate route planning results or mark mapped targets collected.
            sensor_command = ConceptACommand(
                state=command.state,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(
                    self.behavior.config.lift_wheel_speed,
                    self._collect_route_collector_active,
                ),
            )
            self.collection_confirmed = self._check_collection(sensor_command)
        else:
            self.collection_confirmed = self._check_collection(command)
        if self._sim_balls_seen and SIM_COLLECTION_CONFIRM_SOURCE != "truth":
            reconcile = self._credit_reconciler.poll(now)
            if reconcile is not None:
                self._record_collection_event(
                    reconcile.pop("event"), severity="critical", **reconcile
                )
        if (
            effective_mode != "collect_route"
            and self._sim_bin_candidate_active
            and not self.collection_confirmed
        ):
            # The ball has crossed the lip. Hold the chassis still while the
            # intake remains active and prove that the ball stays in the bin.
            command = ConceptACommand(
                state=command.state,
                base=BaseCommand(0.0, 0.0),
                collector=command.collector,
            )
        if effective_mode != "collect_route" and self.collection_confirmed:
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
            terminal_state = (
                self._collect_route_executor.state.value
                if previous_mode == "collect_route"
                and self._collect_route_executor is not None
                and self._collect_route_executor.is_terminal
                else None
            )
            cancel_reason = (
                f"terminal_cleanup:{terminal_state}"
                if terminal_state is not None
                else f"mode_exit:{new_mode}"
            )
            self._record_collection_event("nav2_goal_cancel", reason=cancel_reason)
            self._nav2_lane.reset()
        if previous_mode == "collect_route":
            terminal = bool(
                self._collect_route_executor is not None
                and self._collect_route_executor.is_terminal
            )
            self._last_collect_route_summary = self._build_collect_route_summary(
                status=None if terminal else "stopped"
            )
            if self._collect_route_executor_factory is not None and not terminal:
                self._collect_route_executor_factory.stop()
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
                self._collect_route_executor = None
                self._collect_route_executor_factory = None
                self._collect_route_execution_truth_snapshot = {}
                self._collect_route_executor_events = []
                self._collect_route_executor_complete_reported = False
                self._credit_reconciler = CreditReconciler()
                self._collect_route_confirmations = []
                self._collect_route_run_history = []
                self._last_credited_entry_sequence = self._entry_beam_sequence
                if self._sim_true_pose is not None:
                    self._pose_frame_offset = (
                        self._sim_true_pose[0] - self._robot_x,
                        self._sim_true_pose[1] - self._robot_y,
                    )
                    self._pose_frame_yaw_offset = math.atan2(
                        math.sin(self._sim_true_pose[2] - self._robot_yaw),
                        math.cos(self._sim_true_pose[2] - self._robot_yaw),
                    )
                else:
                    self._pose_frame_offset = None
                    self._pose_frame_yaw_offset = None
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

    def _collect_route_command_for_mode(self, mode: str) -> ConceptACommand:
        if self._on_mode_changed(mode):
            self.ball_map.reset()
            self._collect_route_console_collected_positions = []
            self._collect_start_time = self._runtime_seconds()
            self._start_collection_route_executor()

        if self._collect_route_executor is None:
            raise RuntimeError("collect_route executor was not constructed on mode entry")

        self._collection_executor_cache.latest_scan = self._latest_scan_msg
        self._collection_executor_cache.latest_ball_detections = (
            self._latest_ball_detections_msg
        )
        self._collection_executor_cache.robot_x_m = self._robot_x
        self._collection_executor_cache.robot_y_m = self._robot_y
        self._collection_executor_cache.robot_yaw_rad = self._robot_yaw
        state = self._collect_route_executor.tick()

        if (
            self._collect_route_executor.is_terminal
            and not self._collect_route_executor_complete_reported
        ):
            self._collect_route_executor_complete_reported = True
            detail = ""
            session = getattr(self._collect_route_executor, "_scan_session", None)
            if state.value == "aborted_scan":
                reason = getattr(session, "last_failure_detail", None)
                if reason:
                    detail = f" (scan_failure={reason})"
            elif state.value == "completed_no_targets":
                detail = format_no_targets_diagnostic(
                    getattr(self, "_latest_perception_diagnostics", None)
                )
            elif state is ExecutorState.INCOMPLETE_TARGETS:
                plan = self._collect_route_executor.plan
                unresolved = (
                    sum(
                        result.status.value in {"deferred", "unreachable"}
                        for result in plan.ball_results
                    )
                    if plan is not None
                    else 0
                )
                detail = f" (unresolved_targets={unresolved})"
            self.get_logger().info(f"collect_route executor terminal: {state.value}{detail}")
            # Aggregated scan telemetry (rejection histogram + per-track step
            # counts) explains an empty/partial snapshot: cross-half rejections,
            # TF failures, or balls seen from too few scan steps to confirm.
            scan_diag = getattr(session, "scan_diagnostics", None)
            if scan_diag is not None and state.value in ("completed_no_targets", "aborted_scan"):
                self.get_logger().info(f"collect_route scan diagnostics: {scan_diag}")
            self._publish_command("idle", "controller-collect-route-complete")

        # HANDS-OFF: scan rotation and FollowPath own the base through their
        # dedicated mux inputs; the executor collector port owns the collector.
        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _start_collection_route_executor(self) -> None:
        if self._nav2_lane is None:
            raise RuntimeError(
                "collect_route requires Nav2 action dependencies; no fallback is available"
            )
        self._declare_collection_route_parameters()
        calibration_path = os.getenv("COLLECTION_ROUTE_CALIBRATION_ARTIFACT")
        if not calibration_path:
            raise RuntimeError(
                "COLLECTION_ROUTE_CALIBRATION_ARTIFACT is required for collect_route"
            )
        from ament_index_python.packages import get_package_share_directory

        config_dir = Path(get_package_share_directory("tennis_robot")) / "config"
        self._collection_executor_cache.latest_scan = self._latest_scan_msg
        self._collection_executor_cache.latest_ball_detections = (
            self._latest_ball_detections_msg
        )
        self._collection_executor_cache.robot_x_m = self._robot_x
        self._collection_executor_cache.robot_y_m = self._robot_y
        self._collection_executor_cache.robot_yaw_rad = self._robot_yaw

        def publish_collector_speed(speed_rad_s: float) -> None:
            self._collect_route_collector_active = abs(speed_rad_s) > 1e-9
            message = CollectorCmd()
            message.lift_wheel_speed = float(speed_rad_s)
            message.intake_enabled = self._collect_route_collector_active
            self._pub_collector.publish(message)

        collector_interface = CollectorInterface(
            GazeboCollectorDriver(publish_collector_speed),
            default_speed=ConceptAConfig.from_env().lift_wheel_speed,
        )
        factory = self.collection_executor_factory_type(
            node=self,
            tf_buffer=self._tf_buffer,
            cache=self._collection_executor_cache,
            lane_navigator=self._nav2_lane,
            collector_interface=collector_interface,
            court_boundary_path=DEFAULT_BOUNDARY_FILE,
            collection_route_config_path=config_dir / "collection_route.yaml",
            calibration_artifact_path=calibration_path,
            telemetry_sink=self._on_collection_executor_telemetry,
            entry_beam_provider=lambda: self._intake_beam_broken,
            confirmed_beam_provider=lambda: self._confirmed_beam_broken,
            collector_minimum_drain_s=COLLECTION_ROUTE_MINIMUM_DRAIN_S,
            collector_maximum_drain_s=COLLECTION_ROUTE_MAXIMUM_DRAIN_S,
        )
        executor = factory.build()
        executor.start()
        self._collect_route_executor_factory = factory
        self._collect_route_executor = executor

    def _declare_collection_route_parameters(self) -> None:
        from ament_index_python.packages import get_package_share_directory
        import yaml

        params_path = (
            Path(get_package_share_directory("tennis_robot")) / "config" / "nav2_params.yaml"
        )
        source = yaml.safe_load(params_path.read_text(encoding="utf-8"))
        try:
            values = source["collection_route_executor"]["ros__parameters"]
        except (KeyError, TypeError) as exc:
            raise RuntimeError(
                f"missing collection_route_executor parameters in {params_path}"
            ) from exc
        for name, value in values.items():
            if name != "use_sim_time" and not self.has_parameter(name):
                self.declare_parameter(name, value)

    def _on_collection_executor_telemetry(self, event: dict) -> None:
        serialized = dict(event)
        now = self._runtime_seconds()
        started_at = getattr(self, "_collection_event_started_at", None)
        if started_at is not None:
            serialized.setdefault("t_s", round(max(0.0, now - started_at), 3))
        serialized.setdefault("sim_time_s", round(now, 3))
        self._collect_route_executor_events.append(serialized)
        del self._collect_route_executor_events[:-100]
        if serialized.get("code") == "route_outcome":
            self._capture_collect_route_run(serialized.get("state"))

    def _capture_collect_route_run(self, route_outcome: str | None) -> None:
        executor = self._collect_route_executor
        plan = executor.plan if executor is not None else None
        if plan is None:
            return
        serialized = plan.to_dict()
        plan_id = serialized["plan_id"]
        if any(item.get("plan_id") == plan_id for item in self._collect_route_run_history):
            return
        self._write_collect_route_audit(executor, route_outcome)
        crossing_telemetry = (
            self._collect_route_executor_factory.crossing_telemetry
            if self._collect_route_executor_factory is not None
            else []
        )
        outcomes = self._build_collect_route_execution_outcomes(
            serialized["ball_results"], crossing_telemetry, plan_id
        )
        covered = [item for item in outcomes if item["planner_status"] == "covered"]
        record = {
            "run_number": len(self._collect_route_run_history) + 1,
            "plan_id": plan_id,
            "scan_id": serialized.get("scan_id"),
            "planning_status": serialized["planning_status"],
            "route_outcome": route_outcome,
            "ball_results": serialized["ball_results"],
            "execution_outcomes": outcomes,
            "planned": len(covered),
            "confirmed": sum(
                item["execution_status"] == "confirmed" for item in covered
            ),
            "crossed_unconfirmed": sum(
                item["execution_status"] == "crossed_unconfirmed"
                for item in covered
            ),
            "skipped": sum(
                item["planner_status"] in {"deferred", "unreachable"}
                for item in outcomes
            ),
            "route_collected_at_end": max(
                0, self.collection_count - self._collect_route_run_start_count
            ),
        }
        self._collect_route_run_history.append(record)

    def _write_collect_route_audit(self, executor, route_outcome: str | None) -> None:
        """Persist the immutable planner boundary only when explicitly enabled.

        This is regression instrumentation, not a runtime data dependency.
        With ``COLLECTION_ROUTE_AUDIT_DIR`` unset it performs no filesystem
        access. Audit failures are diagnostic-only and cannot alter mission
        state, planning, or execution.
        """
        audit_dir_value = os.getenv("COLLECTION_ROUTE_AUDIT_DIR", "").strip()
        if not audit_dir_value:
            return
        snapshot = getattr(executor, "snapshot", None)
        plan = getattr(executor, "plan", None)
        if snapshot is None or plan is None:
            self.get_logger().warning(
                "collection route audit skipped: snapshot or plan unavailable"
            )
            return
        try:
            artifact = {
                "schema_version": 1,
                "run_id": self._run_id,
                "route_outcome": route_outcome,
                "snapshot": snapshot.to_dict(),
                "plan": plan.to_dict(),
                "execution_frame_diagnostics": (
                    getattr(
                        self._collect_route_executor_factory,
                        "execution_frame_diagnostics",
                        {},
                    )
                    if self._collect_route_executor_factory is not None
                    else {}
                ),
                "execution_truth_snapshot": dict(
                    getattr(
                        self,
                        "_collect_route_execution_truth_snapshot",
                        {},
                    )
                ),
            }
            safe_scan_id = "".join(
                char if char.isalnum() or char in "-_." else "_"
                for char in str(snapshot.scan_id)
            )
            directory = Path(audit_dir_value)
            directory.mkdir(parents=True, exist_ok=True)
            target = directory / f"{safe_scan_id}.json"
            temporary = directory / f".{safe_scan_id}.json.tmp"
            temporary.write_text(
                json.dumps(artifact, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, target)
            self.get_logger().info(f"collection route audit saved: {target}")
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(f"collection route audit write failed: {exc}")

    def _route_confirmation_context(self, now_s: float) -> dict:
        factory = self._collect_route_executor_factory
        state = (
            getattr(factory, "controller_state", None)
            if factory is not None
            else None
        )
        candidate = dict(state) if isinstance(state, dict) else {}
        association = "active_crossing"
        if not candidate.get("has_active_crossing") or not candidate.get("active_ball_id"):
            # The intake mouth is ~0.876 m ahead of base_footprint, so ENTRY and
            # CONFIRMED normally occur before the controller's base-centre
            # crossing window. Associate that physical event with the nearest
            # upcoming immutable crossing instead of reporting it unassigned.
            progress = candidate.get("progress_s")
            plan = (
                getattr(self._collect_route_executor, "plan", None)
                if self._collect_route_executor is not None
                else None
            )
            upcoming = []
            if isinstance(progress, (int, float)) and plan is not None:
                for segment in plan.segments:
                    for crossing in segment.planned_crossings:
                        delta = float(crossing.progress_s) - float(progress)
                        if -0.35 <= delta <= 1.20:
                            upcoming.append((abs(delta - 0.45), delta, segment, crossing))
            if upcoming:
                _, _, segment, crossing = min(
                    upcoming, key=lambda item: (item[0], item[1], item[3].ball_id)
                )
                candidate.update(
                    {
                        "has_active_crossing": True,
                        "active_ball_id": crossing.ball_id,
                        "active_segment_id": segment.id,
                        "active_crossing_progress_s": crossing.progress_s,
                    }
                )
                association = "intake_lead_crossing"
        if not candidate.get("has_active_crossing") or not candidate.get("active_ball_id"):
            candidate = {}
            samples = (
                getattr(factory, "crossing_telemetry", [])
                if factory is not None
                else []
            )
            for sample in reversed(samples):
                observed_at = sample.get("observed_sim_time_s")
                if (
                    observed_at is not None
                    and 0.0 <= now_s - float(observed_at) <= 3.0
                    and sample.get("active_ball_id")
                ):
                    candidate = dict(sample)
                    association = "recent_crossing"
                    break
        if not candidate:
            return {"association": "unassigned", "ball_id": None}
        return {
            "association": association,
            "plan_id": candidate.get("plan_id"),
            "ball_id": candidate.get("active_ball_id"),
            "segment_id": candidate.get("active_segment_id"),
            "progress_s": candidate.get("progress_s"),
            "crossing_progress_s": candidate.get("active_crossing_progress_s"),
            "measured_speed_mps": candidate.get("measured_speed_mps"),
            "lateral_error_m": candidate.get("lateral_error_m"),
            "heading_error_rad": candidate.get("heading_error_rad"),
        }

    def _record_route_confirmation(self, now_s: float) -> dict:
        context = self._route_confirmation_context(now_s)
        started_at = self._collection_event_started_at
        confirmation = {
            "confirmation_id": len(self._collect_route_confirmations) + 1,
            "t_s": (
                round(max(0.0, now_s - started_at), 3)
                if started_at is not None
                else None
            ),
            "sim_time_s": round(now_s, 3),
            **context,
        }
        self._collect_route_confirmations.append(confirmation)
        # The Phase 9 trace records the *attributed* confirmation, not the raw
        # beam: this is the first place the association with a ball exists, so
        # it is where the evidence is captured rather than re-derived offline.
        factory = self._collect_route_executor_factory
        capture = getattr(factory, "trace_capture", None) if factory is not None else None
        if capture is not None:
            try:
                capture.record_confirmation(confirmation)
            except Exception:  # noqa: BLE001 - instrumentation may not disturb a route
                pass
        return confirmation

    def _mark_route_confirmation_on_console_map(
        self,
        confirmation: dict,
        now_s: float,
    ) -> int | None:
        """Hide a physically confirmed ball from the operator-only BallMap.

        The immutable route snapshot and planner results remain untouched.  A
        route-associated confirmation uses the exact snapshot target position;
        an unassigned confirmation falls back to the CONFIRMED sensor plane in
        front of the current base pose.  Both paths require a nearby mapped
        ball, so a weak association cannot remove an unrelated map target.
        """
        target_x = target_y = None
        route_ball_id = confirmation.get("ball_id")
        snapshot = (
            getattr(self._collect_route_executor, "snapshot", None)
            if self._collect_route_executor is not None
            else None
        )
        if route_ball_id and snapshot is not None:
            target = next(
                (
                    ball
                    for ball in snapshot.balls
                    if ball.ball_id == route_ball_id
                ),
                None,
            )
            if target is not None:
                target_x = float(target.position.x_m)
                target_y = float(target.position.y_m)
        if target_x is None or target_y is None:
            confirmed_plane_x_m = 0.35
            target_x = self._robot_x + confirmed_plane_x_m * math.cos(self._robot_yaw)
            target_y = self._robot_y + confirmed_plane_x_m * math.sin(self._robot_yaw)

        active = [
            ball
            for ball in self.ball_map.balls.values()
            if ball.state not in {"collected", "collection_failed"}
        ]
        if not active:
            return None
        nearest = min(
            active,
            key=lambda ball: math.hypot(
                ball.x_m - target_x,
                ball.y_m - target_y,
            ),
        )
        association_limit_m = min(
            1.0,
            float(self.ball_map.config.max_merge_distance_m),
        )
        if math.hypot(nearest.x_m - target_x, nearest.y_m - target_y) > association_limit_m:
            return None
        self.ball_map.set_state(nearest.id, "collected")
        nearest.last_seen_s = now_s
        positions = getattr(
            self,
            "_collect_route_console_collected_positions",
            None,
        )
        if positions is None:
            positions = []
            self._collect_route_console_collected_positions = positions
        positions.append((float(nearest.x_m), float(nearest.y_m)))
        return nearest.id

    def _build_collect_route_execution_outcomes(
        self,
        planner_results: list[dict],
        crossing_samples: list[dict],
        plan_id: str | None = None,
    ) -> list[dict]:
        confirmations_by_ball: dict[str, list[dict]] = {}
        for confirmation in self._collect_route_confirmations:
            if plan_id is not None and confirmation.get("plan_id") not in {None, plan_id}:
                continue
            ball_id = confirmation.get("ball_id")
            if ball_id:
                confirmations_by_ball.setdefault(str(ball_id), []).append(confirmation)
        crossings_by_ball: dict[str, list[dict]] = {}
        for sample in crossing_samples:
            if plan_id is not None and sample.get("plan_id") not in {None, plan_id}:
                continue
            ball_id = sample.get("active_ball_id")
            if ball_id:
                crossings_by_ball.setdefault(str(ball_id), []).append(sample)

        outcomes = []
        for planner_result in planner_results:
            ball_id = str(planner_result.get("ball_id"))
            planner_status = planner_result.get("status")
            confirmations = confirmations_by_ball.get(ball_id, [])
            crossings = crossings_by_ball.get(ball_id, [])
            completed_crossings = [
                sample
                for sample in crossings
                if isinstance(sample.get("progress_s"), (int, float))
                and isinstance(sample.get("active_crossing_progress_s"), (int, float))
                and sample["progress_s"] >= sample["active_crossing_progress_s"]
            ]
            if confirmations:
                execution_status = "confirmed"
            elif completed_crossings:
                execution_status = "crossed_unconfirmed"
            elif crossings:
                execution_status = "executing"
            elif planner_status == "covered":
                execution_status = "planned"
            else:
                execution_status = planner_status
            outcomes.append(
                {
                    "ball_id": ball_id,
                    "planner_status": planner_status,
                    "planner_reason": planner_result.get("reason_code"),
                    "execution_status": execution_status,
                    "crossing_samples": len(crossings),
                    "crossing_completed": bool(completed_crossings),
                    "confirmation_count": len(confirmations),
                    "last_confirmation": confirmations[-1] if confirmations else None,
                }
            )
        return outcomes

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
            route = self._build_collect_route_summary()
            event.update(
                {
                    "route_phase": route.get("state"),
                    "route_outcome": route.get("route_outcome"),
                    "route_plan_id": route.get("plan_id"),
                    "route_planning_status": route.get("planning_status"),
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
        if not self._consume_entry_for_confirmation():
            self._record_collection_event(
                "confirmed_beam_rejected",
                reason="no_new_entry_sequence",
                entry_sequence=self._entry_beam_sequence,
                ir_left=round(self._ir_left, 1),
                ir_right=round(self._ir_right, 1),
            )
            return True
        self.collection_count += 1
        if self._sim_balls_seen:
            self._credit_reconciler.on_beam_credit(now_s)
        route_confirmation = (
            self._record_route_confirmation(now_s)
            if self.control_mode == "collect_route"
            else None
        )
        if route_confirmation is not None:
            self._mark_route_confirmation_on_console_map(
                route_confirmation,
                now_s,
            )
        self._record_collection_event(
            "beam_collection_credit",
            ir_left=round(self._ir_left, 1),
            ir_right=round(self._ir_right, 1),
            route_confirmation=route_confirmation,
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
                    route_ball_id=None,
                )
            if retention.retained:
                collected_msg = String()
                collected_msg.data = ball_def
                self._pub_ball_collected.publish(collected_msg)
                self._counted_sim_ball_defs.add(ball_def)
                if not credit:
                    self._credit_reconciler.on_truth_retained(now)
                    continue
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
                    route_ball_id=None,
                )
                return True
        return False

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
        if self.control_mode == "collect_route":
            # HANDS-OFF: the executor owns base motion (scan rotation ->
            # /cmd_vel_collection, FollowPath -> /cmd_vel_nav) and the collector
            # (via its Collector port).  Publishing a per-tick base twist here
            # would be relayed by the MotionController onto /cmd_vel_collection
            # and fight the scan rotation on the SAME topic — the robot rotates
            # in stutters, never completes the 360 scan within scan_timeout_s,
            # and the run ends as aborted_scan (Phase 7 finding, sim run 1).
            return

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
        if self.control_mode == "collect_route" and self._collect_route_executor is not None:
            plan = self._collect_route_executor.plan
            if plan is not None:
                route = [
                    {
                        "x_m": point.pose.x_m,
                        "y_m": point.pose.y_m,
                        "yaw_rad": point.pose.yaw_rad,
                    }
                    for segment in plan.segments
                    for point in segment.path.points
                ]
        balls = self.ball_map.to_console_balls(
            self._robot_x,
            self.active_mapped_target_id,
            now=self._runtime_seconds(),
            planned_order=planned_order,
        )
        # collect_route uses the immutable scan snapshot pipeline rather than
        # the legacy BallMap registry.  Surface its live tracks directly so the
        # Collection Workspace updates throughout the 360-degree scan (pending
        # after one heading, confirmed after the configured distinct-step gate).
        snapshot_diagnostics = (
            getattr(self._collect_route_executor_factory, "snapshot_diagnostics", {})
            if self._collect_route_executor_factory is not None
            else {}
        )
        snapshot_tracks = snapshot_diagnostics.get("tracks", [])
        if snapshot_tracks and not balls:
            minimum = int(snapshot_diagnostics.get("minimum_confirmation_count", 2))
            balls = [
                {
                    "id": f"route-scan-track-{index + 1}",
                    "x_m": float(track["x_m"]),
                    "y_m": float(track["y_m"]),
                    "side": "same_side",
                    "visible_candidate": len(track.get("steps", ())) >= minimum,
                    "confirmed": bool(
                        track.get(
                            "confirmed",
                            len(track.get("steps", ())) >= minimum,
                        )
                    ),
                    "planned": False,
                    "order": None,
                    "source": "oak_depth",
                }
                for index, track in enumerate(snapshot_tracks)
                if isinstance(track, dict)
                and isinstance(track.get("x_m"), (int, float))
                and isinstance(track.get("y_m"), (int, float))
            ]
        collected_positions = getattr(
            self,
            "_collect_route_console_collected_positions",
            (),
        )
        if collected_positions:
            merge_distance_m = float(
                getattr(self.ball_map.config, "merge_distance_m", 0.65)
            )
            balls = [
                ball
                for ball in balls
                if not any(
                    math.hypot(
                        float(ball["x_m"]) - collected_x,
                        float(ball["y_m"]) - collected_y,
                    )
                    <= merge_distance_m
                    for collected_x, collected_y in collected_positions
                )
            ]
        confirmed = [b for b in balls if b["confirmed"] and b["side"] != "across_net"]
        metrics: dict[str, object] = {
            "balls_mapped": len(balls),
            "balls_confirmed": len(confirmed),
            "balls_collectable": len(confirmed),
        }
        if len(route) > 1:
            metrics["total_distance_m"] = self._collect_route_executor.plan.total_length_m
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

    def _collect_route_elapsed_s(self, state: str) -> float | None:
        """Wall-clock duration of the current collect_route run.

        Ticks live while the run is active; freezes at the last executor-event
        timestamp once the run reaches a terminal state so a finished run shows
        its final duration rather than counting idle time.
        """
        started = getattr(self, "_collection_event_started_at", None)
        if started is None:
            return None
        terminal = {
            ExecutorState.IDLE.value,
            ExecutorState.COMPLETED.value,
            ExecutorState.COMPLETED_NO_TARGETS.value,
            ExecutorState.INCOMPLETE_TARGETS.value,
            ExecutorState.ABORTED_SCAN.value,
            ExecutorState.ABORTED_PLANNING.value,
            ExecutorState.ABORTED_COLLECTOR.value,
            ExecutorState.ABORTED_SAFETY.value,
            ExecutorState.ABORTED_TRACKING.value,
        }
        if state in terminal:
            events = self._collect_route_executor_events
            last = events[-1].get("t_s") if events else None
            return round(float(last), 1) if last is not None else None
        return round(max(0.0, self._runtime_seconds() - started), 1)

    def _build_collect_route_summary(self, status: str | None = None) -> dict:
        executor = self._collect_route_executor
        state = executor.state.value if executor is not None else ExecutorState.IDLE.value
        outcome = (
            executor.route_outcome.value
            if executor is not None and executor.route_outcome is not None
            else None
        )
        crossing_telemetry = (
            self._collect_route_executor_factory.crossing_telemetry
            if self._collect_route_executor_factory is not None
            else []
        )
        controller_state = (
            getattr(self._collect_route_executor_factory, "controller_state", None)
            if self._collect_route_executor_factory is not None
            else None
        )
        execution_frame_diagnostics = (
            getattr(
                self._collect_route_executor_factory,
                "execution_frame_diagnostics",
                {},
            )
            if self._collect_route_executor_factory is not None
            else {}
        )
        execution_truth_snapshot = getattr(
            self, "_collect_route_execution_truth_snapshot", {}
        )
        if (
            execution_frame_diagnostics
            and not execution_truth_snapshot
        ):
            execution_truth_snapshot = {
                "captured_at_s": self._runtime_seconds(),
                "sim_balls_odom": [
                    {
                        key: ball[key]
                        for key in ("def", "x", "y", "z")
                        if key in ball
                    }
                    for ball in self._sim_balls
                    if isinstance(ball, dict)
                ],
                "sim_robot_true_pose": (
                    {
                        "x_m": self._sim_true_pose[0],
                        "y_m": self._sim_true_pose[1],
                        "yaw_rad": self._sim_true_pose[2],
                    }
                    if self._sim_true_pose is not None
                    else None
                ),
                "believed_robot_map_pose": {
                    "x_m": self._robot_x,
                    "y_m": self._robot_y,
                    "yaw_rad": self._robot_yaw,
                },
                "pose_frame_offset": (
                    {
                        "x_m": self._pose_frame_offset[0],
                        "y_m": self._pose_frame_offset[1],
                        "yaw_rad": self._pose_frame_yaw_offset,
                    }
                    if self._pose_frame_offset is not None
                    and self._pose_frame_yaw_offset is not None
                    else None
                ),
                "pose_drift_m": self._pose_error_m(),
                "yaw_drift_rad": self._pose_yaw_error_rad(),
            }
            self._collect_route_execution_truth_snapshot = (
                execution_truth_snapshot
            )
        payload = {
            "run_id": self._run_id,
            "status": status or state,
            "state": state,
            "route_outcome": outcome,
            "failure_reason": (
                getattr(executor, "terminal_reason", None).value
                if getattr(executor, "terminal_reason", None) is not None
                else None
            ),
            "failure_detail": getattr(executor, "terminal_detail", None),
            "elapsed_s": self._collect_route_elapsed_s(state),
            "plan_id": None,
            "planning_status": None,
            "ball_results": [],
            "segments": [],
            "crossings": [],
            "executed_crossing_telemetry": crossing_telemetry,
            "execution_outcomes": [],
            "run_history": list(self._collect_route_run_history),
            "planned": 0,
            "confirmed": 0,
            "crossed_unconfirmed": 0,
            "missing": 0,
            "skipped": 0,
            "remaining": 0,
            "unresolved_targets": 0,
            "failed": 0,
            "failed_ball_ids": [],
            "active_ball_id": (
                controller_state.get("active_ball_id")
                if isinstance(controller_state, dict)
                and controller_state.get("has_active_crossing")
                else None
            ),
            "confirmations": list(self._collect_route_confirmations),
            "unassigned_confirmations": sum(
                confirmation.get("association") == "unassigned"
                for confirmation in self._collect_route_confirmations
            ),
            "pose_drift_m": self._pose_error_m(),
            "yaw_drift_rad": self._pose_yaw_error_rad(),
            "controller_state": controller_state,
            "executor_events": list(self._collect_route_executor_events),
            "perception_diagnostics": dict(
                getattr(self, "_latest_perception_diagnostics", {})
            ),
            "snapshot_diagnostics": (
                getattr(self._collect_route_executor_factory, "snapshot_diagnostics", {})
                if self._collect_route_executor_factory is not None
                else {}
            ),
            "execution_frame_diagnostics": execution_frame_diagnostics,
            "execution_truth_snapshot": dict(execution_truth_snapshot),
            # Physical intake evidence is deliberately separate from route
            # coverage. A pass can be geometrically covered without the ball
            # being transferred, and a retained ball must never be hidden
            # behind a successful navigation outcome.
            "route_collected": max(
                0, self.collection_count - self._collect_route_run_start_count
            ),
            "beam_credits": self._credit_reconciler.beam_count,
            "truth_retained": self._credit_reconciler.truth_count,
            "basket_retained": self._credit_reconciler.truth_count,
        }
        plan = executor.plan if executor is not None else None
        if plan is not None:
            serialized = plan.to_dict()
            payload.update(
                {
                    "plan_id": serialized["plan_id"],
                    "planning_status": serialized["planning_status"],
                    "ball_results": serialized["ball_results"],
                    "segments": serialized["segments"],
                    "crossings": [
                        crossing
                        for segment in serialized["segments"]
                        for crossing in segment["planned_crossings"]
                    ],
                }
            )
            payload["execution_outcomes"] = self._build_collect_route_execution_outcomes(
                serialized["ball_results"], crossing_telemetry, serialized["plan_id"]
            )
            covered = [
                item
                for item in payload["execution_outcomes"]
                if item["planner_status"] == "covered"
            ]
            crossed_unconfirmed = [
                item
                for item in covered
                if item["execution_status"] == "crossed_unconfirmed"
            ]
            unresolved_targets = [
                item
                for item in payload["execution_outcomes"]
                if item["planner_status"] in {"deferred", "unreachable"}
            ]
            payload.update(
                {
                    "planned": len(covered),
                    "confirmed": sum(
                        item["execution_status"] == "confirmed" for item in covered
                    ),
                    "crossed_unconfirmed": len(crossed_unconfirmed),
                    "missing": len(crossed_unconfirmed),
                    "skipped": sum(
                        item["planner_status"] in {"deferred", "unreachable"}
                        for item in payload["execution_outcomes"]
                    ),
                    "remaining": sum(
                        item["execution_status"] == "planned" for item in covered
                    )
                    + len(unresolved_targets),
                    "unresolved_targets": len(unresolved_targets),
                    "failed_ball_ids": [
                        item["ball_id"] for item in crossed_unconfirmed
                    ],
                }
            )
        completed_plan_ids = {
            item.get("plan_id") for item in self._collect_route_run_history
        }
        current_plan_is_new = (
            payload.get("plan_id") is not None
            and payload["plan_id"] not in completed_plan_ids
        )
        history_planned = sum(
            int(item.get("planned", 0)) for item in self._collect_route_run_history
        )
        history_confirmed = sum(
            int(item.get("confirmed", 0)) for item in self._collect_route_run_history
        )
        history_crossed = sum(
            int(item.get("crossed_unconfirmed", 0))
            for item in self._collect_route_run_history
        )
        history_skipped = sum(
            int(item.get("skipped", 0)) for item in self._collect_route_run_history
        )
        if self._collect_route_run_history:
            current_planned = payload["planned"] if current_plan_is_new else 0
            current_confirmed = payload["confirmed"] if current_plan_is_new else 0
            current_crossed = (
                payload["crossed_unconfirmed"] if current_plan_is_new else 0
            )
            current_skipped = payload["skipped"] if current_plan_is_new else 0
            payload.update(
                {
                    "planned": history_planned + current_planned,
                    "confirmed": history_confirmed + current_confirmed,
                    "crossed_unconfirmed": history_crossed + current_crossed,
                    "missing": history_crossed + current_crossed,
                    "skipped": history_skipped + current_skipped,
                    "failed_ball_ids": [
                        item["ball_id"]
                        for run in self._collect_route_run_history
                        for item in run.get("execution_outcomes", [])
                        if item.get("execution_status") == "crossed_unconfirmed"
                    ]
                    + (
                        payload["failed_ball_ids"] if current_plan_is_new else []
                    ),
                }
            )
        return payload

    def _collect_route_summary_for_status(self) -> dict:
        if self.control_mode == "collect_route":
            return self._build_collect_route_summary()
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
            "measured_speed_mps": round(self._robot_speed_mps, 3),
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
            "collect_route": (
                self._build_collect_route_summary()
                if self.control_mode == "collect_route"
                else self._last_collect_route_summary
            ),
            "collection_run": self._collect_route_summary_for_status(),
            "pose_error_m": self._pose_error_m(),
            "pose_yaw_error_rad": self._pose_yaw_error_rad(),
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
