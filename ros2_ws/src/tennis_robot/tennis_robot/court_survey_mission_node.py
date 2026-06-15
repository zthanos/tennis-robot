"""Court Survey Mission Node â€” Python brain for the Court Knowledge Model.

Implements the perimeter survey FSM from the Court Knowledge Model spec using:
  - nav2_simple_commander.BasicNavigator  for Nav2 NavigateToPose goals
  - /court_landmarks (JSON)               for live camera landmark detections
  - /scan (LaserScan)                     for LiDAR front/side range checks
  - /odom (Odometry)                      for current robot pose

State machine (mirrors the spec FSM, adapted for Nav2):
  INIT
  FIND_FIRST_OBSTACLE   drive forward; classify first obstacle (net vs fence)
  APPROACH_NET          navigate to net standoff using landmark bearing+depth
  TURN_LEFT_AT_NET      rotate 90Â° left; record loop-reference pose
  FOLLOW_NET_TO_FENCE   drive a locked straight line toward sideline fence
  TURN_LEFT_AT_FENCE    rotate 90Â° left
  FOLLOW_FENCE          drive a locked straight line until next corner
  CROSS_NET             navigate through right-side net gap
  SECOND_HALF           mirror of FOLLOW_NET_TO_FENCE â€¦ FOLLOW_FENCE for far half
  COMPLETE              write court_boundary.json; publish result

On SUCCESS the node writes runtime/court_boundary.json and shuts down.
On FAILURE it writes a failed status and shuts down with exit code 1.

Environment variables
â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
COURT_SURVEY_NET_STANDOFF_M       float  default 2.00
COURT_SURVEY_FENCE_STOP_M         float  default 0.40
COURT_SURVEY_DRIVE_FORWARD_M      float  default 8.0  (FIND_FIRST_OBSTACLE)
COURT_SURVEY_STATE_TIMEOUT_S      float  default 180.0 (per state)
COURT_SURVEY_LANDMARK_MIN_CONF    float  default 0.25
COURT_SURVEY_BT_XML               str    default "" (uses nav2_params default)
ROBOT_STATUS_FILE / TENNIS_ROBOT_ROOT  (same as controller_node)
"""

from __future__ import annotations

import json
import math
import os
import time
from enum import Enum
from pathlib import Path
from typing import Any

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None  # type: ignore
    TaskResult = None       # type: ignore


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"))
COURT_BOUNDARY_FILE = Path(
    os.getenv("ROBOT_STATUS_FILE", str(PROJECT_ROOT / "runtime" / "robot_status.json"))
).parent / "court_boundary.json"

NET_STANDOFF_M: float = float(os.getenv("COURT_SURVEY_NET_STANDOFF_M", "2.00"))
FENCE_STOP_M: float = float(os.getenv("COURT_SURVEY_FENCE_STOP_M", "0.40"))
DRIVE_FORWARD_M: float = float(os.getenv("COURT_SURVEY_DRIVE_FORWARD_M", "8.0"))
STATE_TIMEOUT_S: float = float(os.getenv("COURT_SURVEY_STATE_TIMEOUT_S", "180.0"))
SECOND_HALF_RETURN_TIMEOUT_S: float = float(os.getenv("COURT_SURVEY_SECOND_HALF_RETURN_TIMEOUT_S", "420.0"))
LANDMARK_MIN_CONF: float = float(os.getenv("COURT_SURVEY_LANDMARK_MIN_CONF", "0.25"))
BT_XML: str = os.getenv("COURT_SURVEY_BT_XML", "")

FRONT_LIDAR_HALF_DEG: float = 20.0   # Â±20Â° sector for front range
COURT_LENGTH_M: float = float(os.getenv("COURT_SURVEY_COURT_LENGTH_M", "23.77"))
COURT_WIDTH_M: float = float(os.getenv("COURT_SURVEY_COURT_WIDTH_M", "10.97"))
COURT_HALF_LENGTH_M: float = COURT_LENGTH_M / 2.0
COURT_HALF_WIDTH_M: float = COURT_WIDTH_M / 2.0
NET_APPROACH_MARGIN_M: float = float(os.getenv("COURT_SURVEY_NET_APPROACH_MARGIN_M", "0.30"))
NET_TO_FENCE_MIN_TRAVEL_M: float = float(
    os.getenv("COURT_SURVEY_NET_TO_FENCE_MIN_TRAVEL_M", str(COURT_HALF_WIDTH_M))
)
FENCE_TURN_STANDOFF_M: float = float(os.getenv("COURT_SURVEY_FENCE_TURN_STANDOFF_M", "2.50"))
CROSS_NET_DISTANCE_M: float = float(os.getenv("COURT_SURVEY_CROSS_NET_DISTANCE_M", "0.90"))
NET_LINE_TRIGGER_TOLERANCE_M: float = float(os.getenv("COURT_SURVEY_NET_LINE_TRIGGER_TOLERANCE_M", "0.35"))
LOOP_CLOSURE_GOAL_STANDOFF_M: float = float(os.getenv("COURT_SURVEY_LOOP_CLOSURE_GOAL_STANDOFF_M", "2.0"))
LOOP_CLOSURE_TOLERANCE_M: float = float(os.getenv("COURT_SURVEY_LOOP_CLOSURE_TOLERANCE_M", "2.3"))
LOOP_CLOSURE_FINAL_TOLERANCE_M: float = float(os.getenv("COURT_SURVEY_LOOP_CLOSURE_FINAL_TOLERANCE_M", "0.50"))
RETURN_CORNER_TOLERANCE_M: float = float(os.getenv("COURT_SURVEY_RETURN_CORNER_TOLERANCE_M", "0.20"))
RETURN_GOAL_M: float = float(os.getenv("COURT_SURVEY_RETURN_GOAL_M", str(COURT_LENGTH_M)))
FENCE_CORNER_MIN_TRAVEL_M: float = float(
    os.getenv("COURT_SURVEY_FENCE_CORNER_MIN_TRAVEL_M", str(COURT_HALF_LENGTH_M))
)
SECOND_HALF_CORNER_MIN_TRAVEL_M: float = float(
    os.getenv("COURT_SURVEY_SECOND_HALF_CORNER_MIN_TRAVEL_M", str(COURT_HALF_WIDTH_M))
)
SURVEY_TURN_TOLERANCE_DEG: float = float(os.getenv("COURT_SURVEY_TURN_TOLERANCE_DEG", "5.0"))
SURVEY_TURN_MAX_RAD_S: float = float(os.getenv("COURT_SURVEY_TURN_MAX_RAD_S", "0.45"))
SURVEY_TURN_MIN_RAD_S: float = float(os.getenv("COURT_SURVEY_TURN_MIN_RAD_S", "0.14"))
CROSS_NET_LINEAR_M_S: float = float(os.getenv("COURT_SURVEY_CROSS_NET_LINEAR_M_S", "0.25"))
SURVEY_STRAIGHT_LINEAR_M_S: float = float(os.getenv("COURT_SURVEY_STRAIGHT_LINEAR_M_S", "0.30"))
SURVEY_STRAIGHT_YAW_KP: float = float(os.getenv("COURT_SURVEY_STRAIGHT_YAW_KP", "1.2"))
SURVEY_STRAIGHT_CROSSTRACK_KP: float = float(os.getenv("COURT_SURVEY_STRAIGHT_CROSSTRACK_KP", "0.25"))


# ---------------------------------------------------------------------------
# Survey FSM states
# ---------------------------------------------------------------------------

class SurveyState(Enum):
    INIT = "init"
    FIND_FIRST_OBSTACLE = "find_first_obstacle"
    APPROACH_NET = "approach_net"
    TURN_LEFT_AT_NET = "turn_left_at_net"
    FOLLOW_NET_TO_FENCE = "follow_net_to_fence"
    TURN_LEFT_AT_FENCE_1 = "turn_left_at_fence_1"
    FOLLOW_FENCE_TO_CORNER = "follow_fence_to_corner"
    TURN_LEFT_AT_CORNER = "turn_left_at_corner"
    FOLLOW_FENCE_TO_NET = "follow_fence_to_net"
    CROSS_NET = "cross_net"
    SECOND_HALF_FOLLOW_FENCE = "second_half_follow_fence"
    SECOND_HALF_TURN = "second_half_turn"
    SECOND_HALF_RETURN = "second_half_return"
    COMPLETE = "complete"
    FAILED = "failed"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _yaw_from_quaternion(q) -> float:  # type: ignore[type-arg]
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _make_pose_stamped(
    x: float, y: float, yaw: float, frame: str = "map"
) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    return ps


def _project(x: float, y: float, heading: float, dist: float) -> tuple[float, float]:
    return x + dist * math.cos(heading), y + dist * math.sin(heading)


def _snap_cardinal_yaw(yaw: float) -> float:
    return round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)


def _front_range_from_scan(ranges: list[float], angle_min: float, angle_inc: float) -> float:
    """30th-pct front range over Â±20Â°."""
    half = math.radians(FRONT_LIDAR_HALF_DEG)
    vals: list[float] = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        angle = (angle_min + i * angle_inc + math.pi) % (2 * math.pi) - math.pi
        if abs(angle) <= half:
            vals.append(r)
    if not vals:
        return math.inf
    vals.sort()
    idx = min(len(vals) - 1, int(round((len(vals) - 1) * 0.30)))
    return vals[idx]


# ---------------------------------------------------------------------------
# Mission node
# ---------------------------------------------------------------------------

class CourtSurveyMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("court_survey_mission_node")

        self._state = SurveyState.INIT
        self._state_entered_at: float = time.time()
        self._last_event: str = "none"
        self._failure_reason: str | None = None
        self._navigation_points: list[dict] = []
        self._locked_net: dict[str, Any] | None = None

        # Sensor caches
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._state_start_x: float = 0.0
        self._state_start_y: float = 0.0
        self._front_range_m: float = math.inf
        self._last_landmarks: dict[str, Any] = {}
        self._last_survey_vision: dict[str, Any] = {}
        self._last_scan_points: list[tuple[float, float, float]] = []
        self._scan_angle_min: float = -math.pi
        self._scan_angle_inc: float = math.radians(1.0)
        self._net_approach_start_x: float | None = None
        self._net_approach_start_y: float | None = None
        self._net_approach_target_m: float | None = None
        self._locked_first_obstacle: str | None = None
        self._locked_net_bearing_rad: float = 0.0
        self._locked_net_distance_m: float | None = None
        self._locked_net_confidence: float = 0.0
        self._locked_net_approach_yaw_rad: float | None = None

        # Loop-closure reference (recorded at first net-left-turn)
        self._loop_ref_x: float | None = None
        self._loop_ref_y: float | None = None
        self._net_follow_heading: float | None = None
        self._fence_follow_heading: float | None = None
        self._fence_to_net_heading: float | None = None
        self._second_half_follow_heading: float | None = None

        # Stored target yaw for turn states (set once on entry, cleared on _enter())
        # Avoids recalculating from self._robot_yaw on every tick while turning.
        self._turn_target_yaw: float | None = None
        self._local_turn_active: bool = False
        self._cross_net_phase: str | None = None
        self._cross_net_yaw: float | None = None
        self._cross_net_resume_yaw: float | None = None
        self._cross_net_start_x: float | None = None
        self._cross_net_start_y: float | None = None
        self._second_half_return_phase: str | None = None
        self._second_half_return_heading: float | None = None
        self._second_half_return_start_x: float | None = None
        self._second_half_return_start_y: float | None = None

        # Nav2
        self._navigator: BasicNavigator | None = None
        self._nav_active: bool = False
        self._nav2_lifecycle_active: bool = False
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

        # TF: get robot pose in map frame (more reliable than /odom topic QoS)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(String, "/court_landmarks", self._landmarks_cb, 10)
        self.create_subscription(String, "/survey/vision", self._survey_vision_cb, 10)

        # Main FSM timer at 5 Hz
        self.create_timer(0.2, self._step)

        self.get_logger().info("court_survey_mission_node started")

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _update_pose_from_tf(self) -> None:
        """Update robot pose from TF (mapâ†’base_link) instead of /odom topic.

        The /odom topic has QoS negotiation issues with diff_drive_controller.
        TF is always available (used by Nav2 itself) and gives pose in map frame,
        which is exactly what we need for computing Nav2 goals.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05),
            )
            self._robot_x = float(t.transform.translation.x)
            self._robot_y = float(t.transform.translation.y)
            self._robot_yaw = _yaw_from_quaternion(t.transform.rotation)
        except (LookupException, ExtrapolationException, ConnectivityException):
            pass  # keep previous values until TF is available

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_inc = float(msg.angle_increment)
        ranges = list(msg.ranges)
        self._front_range_m = _front_range_from_scan(
            ranges, self._scan_angle_min, self._scan_angle_inc
        )
        points: list[tuple[float, float, float]] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = self._scan_angle_min + i * self._scan_angle_inc
            points.append((float(r) * math.cos(angle), float(r) * math.sin(angle), float(r)))
        self._last_scan_points = points

    def _landmarks_cb(self, msg: String) -> None:
        try:
            self._last_landmarks = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _survey_vision_cb(self, msg: String) -> None:
        try:
            self._last_survey_vision = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------------
    # FSM step
    # ------------------------------------------------------------------

    def _step(self) -> None:
        if self._state in (SurveyState.COMPLETE, SurveyState.FAILED):
            return

        # Update robot pose from TF every tick (replaces /odom subscription)
        self._update_pose_from_tf()

        if self._state_timed_out():
            self._fail(f"{self._state.value}_timeout")
            return

        {
            SurveyState.INIT: self._state_init,
            SurveyState.FIND_FIRST_OBSTACLE: self._state_find_first_obstacle,
            SurveyState.APPROACH_NET: self._state_approach_net,
            SurveyState.TURN_LEFT_AT_NET: self._state_turn_left_at_net,
            SurveyState.FOLLOW_NET_TO_FENCE: self._state_follow_net_to_fence,
            SurveyState.TURN_LEFT_AT_FENCE_1: self._state_turn_left_at_fence_1,
            SurveyState.FOLLOW_FENCE_TO_CORNER: self._state_follow_fence_to_corner,
            SurveyState.TURN_LEFT_AT_CORNER: self._state_turn_left_at_corner,
            SurveyState.FOLLOW_FENCE_TO_NET: self._state_follow_fence_to_net,
            SurveyState.CROSS_NET: self._state_cross_net,
            SurveyState.SECOND_HALF_FOLLOW_FENCE: self._state_second_half_follow_fence,
            SurveyState.SECOND_HALF_TURN: self._state_second_half_turn,
            SurveyState.SECOND_HALF_RETURN: self._state_second_half_return,
        }[self._state]()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _state_init(self) -> None:
        # Give Nav2 time to fully activate before sending goals.
        # bt_navigator registers its action server early but rejects goals until ACTIVE.
        # Wait until the navigate_to_pose action server is available (non-blocking).
        if self._navigator is None and BasicNavigator is not None:
            self._navigator = BasicNavigator(node_name="court_survey_navigator")
            self.get_logger().info("Nav2 navigator created, waiting for action server...")
            return
        if not self._navigator.nav_to_pose_client.wait_for_server(timeout_sec=0.0):
            return  # still starting up â€” check again next tick (5 Hz)
        if not self._nav2_lifecycle_active:
            self.get_logger().info("Nav2 action server ready, waiting for lifecycle active...")
            self._navigator._waitForNodeToActivate("bt_navigator")
            self._nav2_lifecycle_active = True
        self.get_logger().info("Nav2 action server ready, starting survey")
        self._enter(SurveyState.FIND_FIRST_OBSTACLE, "survey_started")

    def _state_find_first_obstacle(self) -> None:
        """Drive forward; classify first obstacle as net or fence.

        A net detection can appear several meters before the robot reaches the
        net.  Use that detection to switch into the net approach state, but only
        start the perimeter turn after the robot reaches the configured standoff.
        If we reach 2.5 m without a net detection â†’ it's a fence end-wall.
        """
        # Net detected from far away -> approach it first, do not turn yet.
        net = self._first_visible_net_for_initial_approach()
        if net and net.get("confidence", 0) >= LANDMARK_MIN_CONF:
            net_dist = self._lock_net_detection(net)
            if net_dist is not None:
                self._cancel_nav()
                close_by_landmark = net_dist <= NET_STANDOFF_M + NET_APPROACH_MARGIN_M
                close_by_lidar = self._front_range_m <= NET_STANDOFF_M + 0.10
                if close_by_landmark or close_by_lidar:
                    self._record("first_obstacle_net")
                    self._enter(SurveyState.TURN_LEFT_AT_NET, "net_detected_near")
                else:
                    self._record("first_obstacle_net_detected")
                    self._enter(SurveyState.APPROACH_NET, "net_detected_approach")
                return

        # Reached close range with no net â†’ this survey flow expects the net first.
        if self._front_range_m <= 2.5:
            self._record("first_obstacle_fence")
            self._fail("first_obstacle_fence_without_net_reference")
            return
        # If nav task completed (success or failure), clear the flag so we retry.
        # This handles: planner failure (goal in obstacle), path blocked, etc.
        if self._nav_active and self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"find_first_obstacle nav done ({result}); "
                f"front={self._front_range_m:.2f}m â€” retrying"
            )
        # Keep driving forward.
        # IMPORTANT: never project the goal INTO the detected obstacle â€” the planner
        # will reject a goal that lands on an occupied (net/fence) cell.
        # Place the goal 1.5 m short of the nearest detected obstacle, capped at
        # DRIVE_FORWARD_M.  If no obstacle yet (inf), use DRIVE_FORWARD_M.
        if not self._nav_active:
            if math.isinf(self._front_range_m):
                target_dist = DRIVE_FORWARD_M
            else:
                target_dist = max(0.5, min(DRIVE_FORWARD_M, self._front_range_m - 1.5))
            gx, gy = _project(self._robot_x, self._robot_y, self._robot_yaw, target_dist)
            self.get_logger().info(
                f"find_first_obstacle: front={self._front_range_m:.2f}m "
                f"â†’ goal {gx:.2f},{gy:.2f} (dist={target_dist:.2f}m)"
            )
            self._nav_to(gx, gy, self._robot_yaw)

    def _state_approach_net(self) -> None:
        """Navigate to net standoff using the first net detection as reference."""
        net = self._first_visible_net_for_initial_approach()
        if (
            net
            and self._locked_first_obstacle != "net"
            and net.get("confidence", 0) >= LANDMARK_MIN_CONF
        ):
            if self._lock_net_detection(net) is None:
                return
        net_dist = self._locked_net_distance_m
        travelled = self._distance_from_net_approach_start()

        close_by_initial_detection = (
            self._net_approach_target_m is not None
            and travelled >= self._net_approach_target_m
        )
        close_by_landmark = net_dist is not None and net_dist <= NET_STANDOFF_M + NET_APPROACH_MARGIN_M
        close_by_lidar = self._front_range_m <= NET_STANDOFF_M + 0.10
        if close_by_initial_detection or close_by_landmark or close_by_lidar:
            self._record("first_obstacle_net")
            self._record("net_standoff_reached")
            self._cancel_nav()
            self._enter(SurveyState.TURN_LEFT_AT_NET, "near_net")
            return

        if self._nav_active and self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"approach_net nav done ({result}); front={self._front_range_m:.2f}m "
                f"net_dist={net_dist if net_dist is not None else float('nan'):.2f}m "
                f"travelled={travelled:.2f}m - retrying"
            )

        if not self._nav_active:
            bearing = self._locked_net_bearing_rad
            if self._net_approach_target_m is not None:
                dist = max(0.0, self._net_approach_target_m - travelled)
            elif net_dist is not None and net_dist > NET_STANDOFF_M:
                dist = net_dist - NET_STANDOFF_M
            else:
                dist = self._safe_dist(DRIVE_FORWARD_M, clearance=NET_STANDOFF_M)
            if dist <= NET_APPROACH_MARGIN_M:
                self._record("first_obstacle_net")
                self._record("net_standoff_reached")
                self._enter(SurveyState.TURN_LEFT_AT_NET, "near_net")
                return
            gx, gy = _project(
                self._robot_x, self._robot_y,
                self._robot_yaw + bearing, dist,
            )
            goal_yaw = self._robot_yaw + bearing
            self.get_logger().info(
                f"approach_net: front={self._front_range_m:.2f}m "
                f"net_dist={net_dist if net_dist is not None else float('nan'):.2f}m "
                f"locked={self._locked_first_obstacle or 'none'} "
                f"travelled={travelled:.2f}m -> goal {gx:.2f},{gy:.2f} (dist={dist:.2f}m)"
            )
            self._nav_to(gx, gy, goal_yaw)

    def _state_turn_left_at_net(self) -> None:
        """Rotate 90Â° left; verify with TF; retry corrective spins until Â±8Â°."""
        if self._turn_target_yaw is None:
            if self._locked_net_approach_yaw_rad is None:
                self._fail("missing_locked_net_approach_yaw")
                return
            approach_yaw = self._locked_net_approach_yaw_rad
            self._turn_target_yaw = _snap_cardinal_yaw(approach_yaw + math.pi / 2)
            # Record position before any spinning
            self._loop_ref_x = self._robot_x
            self._loop_ref_y = self._robot_y
            self._net_follow_heading = self._turn_target_yaw
            self._record("first_net_left_turn_reference")
            self.get_logger().info(
                f"turn_left_at_net: approach_yaw={math.degrees(approach_yaw):.1f}deg "
                f"target_parallel_yaw={math.degrees(self._turn_target_yaw):.1f}deg"
            )
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_NET_TO_FENCE,
            "turn_left_at_net_complete",
        )

    def _state_follow_net_to_fence(self) -> None:
        """Drive parallel to net until fence detected ahead."""
        if self._net_follow_heading is None or self._loop_ref_x is None or self._loop_ref_y is None:
            self._fail("missing_net_follow_reference")
            return
        heading = self._net_follow_heading
        ref_x = self._loop_ref_x
        ref_y = self._loop_ref_y
        travelled = self._progress_from_ref(ref_x, ref_y, heading)
        if travelled >= NET_TO_FENCE_MIN_TRAVEL_M and self._front_range_m <= FENCE_TURN_STANDOFF_M:
            self._record("net_to_fence_corner")
            self._publish_stop()
            self.get_logger().info(
                f"follow_net_to_fence: fence turn trigger travelled={travelled:.2f}m "
                f"front={self._front_range_m:.2f}m standoff={FENCE_TURN_STANDOFF_M:.2f}m"
            )
            self._enter(SurveyState.TURN_LEFT_AT_FENCE_1, "near_fence")
            return
        self._drive_locked_heading("follow_net_to_fence", ref_x, ref_y, heading, travelled)

    def _state_turn_left_at_fence_1(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = _snap_cardinal_yaw(self._robot_yaw + math.pi / 2)
            self._fence_follow_heading = self._turn_target_yaw
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_FENCE_TO_CORNER,
            "turn_left_at_fence_1_complete",
        )

    def _state_follow_fence_to_corner(self) -> None:
        """Follow fence until next corner."""
        if self._fence_follow_heading is None:
            self._fail("missing_fence_follow_heading")
            return
        heading = self._fence_follow_heading
        travelled = self._progress_from_ref(self._state_start_x, self._state_start_y, heading)
        corner_standoff = FENCE_TURN_STANDOFF_M if travelled >= FENCE_CORNER_MIN_TRAVEL_M else FENCE_STOP_M + 0.35
        if self._front_range_m <= corner_standoff:
            if travelled < FENCE_CORNER_MIN_TRAVEL_M:
                self.get_logger().info(
                    f"follow_fence_to_corner: ignoring early front obstacle "
                    f"front={self._front_range_m:.2f}m travelled={travelled:.2f}m "
                    f"min={FENCE_CORNER_MIN_TRAVEL_M:.2f}m"
                )
            else:
                self._record("fence_corner")
                self._publish_stop()
                self._enter(SurveyState.TURN_LEFT_AT_CORNER, "corner_detected")
                return
        self._drive_locked_heading("follow_fence_to_corner", self._state_start_x, self._state_start_y, heading, travelled)

    def _state_turn_left_at_corner(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = _snap_cardinal_yaw(self._robot_yaw + math.pi / 2)
            self._fence_to_net_heading = self._turn_target_yaw
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_FENCE_TO_NET,
            "turn_left_at_corner_complete",
        )


    def _state_follow_fence_to_net(self) -> None:
        """Follow fence until crossing the known net line, then use the gap."""
        if (
            self._fence_to_net_heading is None
            or self._loop_ref_x is None
            or self._loop_ref_y is None
        ):
            self._fail("missing_fence_to_net_reference")
            return
        heading = self._fence_to_net_heading
        travelled = self._progress_from_ref(self._state_start_x, self._state_start_y, heading)
        net_line_travel = (
            (self._loop_ref_x - self._state_start_x) * math.cos(heading)
            + (self._loop_ref_y - self._state_start_y) * math.sin(heading)
        )
        if net_line_travel > 0.0 and travelled >= max(0.0, net_line_travel - NET_LINE_TRIGGER_TOLERANCE_M):
            self._record("net_detected_from_far_side")
            self._publish_stop()
            self.get_logger().info(
                f"follow_fence_to_net: net-line trigger travelled={travelled:.2f}m "
                f"target={net_line_travel:.2f}m tolerance={NET_LINE_TRIGGER_TOLERANCE_M:.2f}m"
            )
            self._enter(SurveyState.CROSS_NET, "net_line_reached")
            return
        net = self._last_landmarks.get("net")
        if net and net.get("confidence", 0) >= LANDMARK_MIN_CONF:
            self._record("net_detected_from_far_side")
            self._publish_stop()
            self._enter(SurveyState.CROSS_NET, "net_detected")
            return
        if self._front_range_m <= FENCE_STOP_M + 0.35:
            self._fail("hit_fence_before_net_line")
            return
        self._drive_locked_heading("follow_fence_to_net", self._state_start_x, self._state_start_y, heading, travelled)

    def _state_cross_net(self) -> None:
        """Navigate through the right-side net gap (between net post and fence).

        The robot reaches this state while already travelling perpendicular to
        the net line.  Keep that locked heading and drive a short clearance
        distance through the net line before starting the second half.
        """
        if self._cross_net_phase is None:
            self._cancel_nav()
            self._cross_net_resume_yaw = (
                _snap_cardinal_yaw(self._fence_to_net_heading)
                if self._fence_to_net_heading is not None
                else _snap_cardinal_yaw(self._robot_yaw)
            )
            self._cross_net_yaw = self._cross_net_resume_yaw
            self._cross_net_phase = "turn_to_gap"
            self._record("crossing_net_right_side")
            self.get_logger().info(
                f"cross_net: straight-through crossing_yaw={math.degrees(self._cross_net_yaw):.1f}deg "
                f"clearance={CROSS_NET_DISTANCE_M:.2f}m "
                f"resume_yaw={math.degrees(self._cross_net_resume_yaw):.1f}deg"
            )

        if self._cross_net_phase == "turn_to_gap":
            assert self._cross_net_yaw is not None
            yaw_err = ((self._cross_net_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
            if self._turn_step_done(yaw_err):
                self._publish_stop()
                self._local_turn_active = False
                self._cross_net_start_x = self._robot_x
                self._cross_net_start_y = self._robot_y
                self._cross_net_phase = "drive_gap"
                self.get_logger().info("cross_net: turn_to_gap complete")
            return

        if self._cross_net_phase == "drive_gap":
            if self._cross_net_start_x is None or self._cross_net_start_y is None:
                self._cross_net_start_x = self._robot_x
                self._cross_net_start_y = self._robot_y
            travelled = math.hypot(
                self._robot_x - self._cross_net_start_x,
                self._robot_y - self._cross_net_start_y,
            )
            if travelled >= CROSS_NET_DISTANCE_M:
                self._publish_stop()
                self._cross_net_phase = "resume_heading"
                self._record("cross_net_exit")
                self.get_logger().info(f"cross_net: drive_gap complete travelled={travelled:.2f}m")
                return
            assert self._cross_net_yaw is not None
            yaw_err = ((self._cross_net_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
            twist = Twist()
            twist.linear.x = CROSS_NET_LINEAR_M_S
            twist.angular.z = max(-0.25, min(0.25, yaw_err * 0.8))
            self._cmd_vel_pub.publish(twist)
            return

        if self._cross_net_phase == "resume_heading":
            assert self._cross_net_resume_yaw is not None
            yaw_err = ((self._cross_net_resume_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
            if self._turn_step_done(yaw_err):
                self._publish_stop()
                self._local_turn_active = False
                self._cross_net_phase = None
                self._second_half_follow_heading = self._cross_net_resume_yaw
                self._record("second_half_start")
                self._cross_net_yaw = None
                self._cross_net_resume_yaw = None
                self._cross_net_start_x = None
                self._cross_net_start_y = None
                self._enter(SurveyState.SECOND_HALF_FOLLOW_FENCE, "gap_crossed")
            return

    def _state_second_half_follow_fence(self) -> None:
        """Mirror of FOLLOW_FENCE_TO_CORNER for the second half of the court."""
        if self._second_half_follow_heading is None:
            self._fail("missing_second_half_follow_heading")
            return
        heading = self._second_half_follow_heading
        travelled = self._progress_from_ref(self._state_start_x, self._state_start_y, heading)
        corner_standoff = FENCE_TURN_STANDOFF_M if travelled >= SECOND_HALF_CORNER_MIN_TRAVEL_M else FENCE_STOP_M + 0.35
        if self._front_range_m <= corner_standoff:
            if travelled >= SECOND_HALF_CORNER_MIN_TRAVEL_M:
                self._record("second_half_corner")
                self._publish_stop()
                self._enter(SurveyState.SECOND_HALF_TURN, "corner_detected_second_half")
                return
            self.get_logger().info(
                f"second_half_follow_fence: ignored early obstacle "
                f"front={self._front_range_m:.2f}m travelled={travelled:.2f}m "
                f"min={SECOND_HALF_CORNER_MIN_TRAVEL_M:.2f}m"
            )
        self._drive_locked_heading("second_half_follow_fence", self._state_start_x, self._state_start_y, heading, travelled)

    def _state_second_half_turn(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = _snap_cardinal_yaw(self._robot_yaw + math.pi / 2)
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.SECOND_HALF_RETURN,
            "second_half_turn_complete",
        )

    def _state_second_half_return(self) -> None:
        """Complete the second-half perimeter before closing at the net side."""
        first_fence_corner = self._point_by_label("net_to_fence_corner")
        second_half_corner = self._point_by_label("second_half_corner")
        if first_fence_corner is None:
            self._fail("missing_first_fence_corner_reference")
            return
        if second_half_corner is None:
            self._fail("missing_second_half_corner_reference")
            return

        first_x = float(first_fence_corner["x_m"])
        first_y = float(first_fence_corner["y_m"])
        second_x = float(second_half_corner["x_m"])
        second_y = float(second_half_corner["y_m"])

        if self._second_half_return_phase is None:
            self._cancel_nav()
            self._second_half_return_phase = "bottom_leg"
            self._second_half_return_heading = _snap_cardinal_yaw(self._robot_yaw)
            self._second_half_return_start_x = self._robot_x
            self._second_half_return_start_y = self._robot_y
            self.get_logger().info(
                f"second_half_return: perimeter bottom leg heading="
                f"{math.degrees(self._second_half_return_heading):.1f}deg "
                f"target_full_length={COURT_LENGTH_M:.2f}m "
                f"close_ref=({first_x:.2f},{first_y:.2f})"
            )

        if (
            self._second_half_return_start_x is None
            or self._second_half_return_start_y is None
            or self._second_half_return_heading is None
        ):
            self._fail("missing_second_half_return_reference")
            return

        if self._second_half_return_phase == "bottom_leg":
            heading = self._second_half_return_heading
            target_x, target_y = _project(second_x, second_y, heading, COURT_LENGTH_M)
            travelled = self._progress_from_ref(
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
            )
            target_travel = (
                (target_x - self._second_half_return_start_x) * math.cos(heading)
                + (target_y - self._second_half_return_start_y) * math.sin(heading)
            )
            if target_travel <= 0.0:
                self._fail("invalid_second_half_return_bottom_target")
                return
            near_planned_corner = travelled >= max(0.0, target_travel - RETURN_CORNER_TOLERANCE_M)
            fence_at_full_length = (
                travelled >= max(0.0, target_travel - FENCE_TURN_STANDOFF_M)
                and self._front_range_m <= FENCE_TURN_STANDOFF_M
            )
            if near_planned_corner or fence_at_full_length:
                self._publish_stop()
                self._record("return_side_corner")
                self._second_half_return_phase = "turn_to_far_side"
                self._state_entered_at = time.time()
                self._turn_target_yaw = _snap_cardinal_yaw(heading + math.pi / 2.0)
                self.get_logger().info(
                    f"second_half_return: bottom leg complete travelled={travelled:.2f}m "
                    f"target={target_travel:.2f}m "
                    f"front={self._front_range_m:.2f}m"
                )
                return
            self._drive_locked_heading(
                "second_half_return_bottom",
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
                travelled,
            )
            return

        if self._second_half_return_phase == "turn_to_far_side":
            assert self._turn_target_yaw is not None
            yaw_err = ((self._turn_target_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
            if self._turn_step_done(yaw_err):
                self._publish_stop()
                self._local_turn_active = False
                self._second_half_return_phase = "far_side_leg"
                self._second_half_return_heading = self._turn_target_yaw
                self._second_half_return_start_x = self._robot_x
                self._second_half_return_start_y = self._robot_y
                self._state_entered_at = time.time()
                self.get_logger().info(
                    f"second_half_return: turn_to_far_side complete heading="
                    f"{math.degrees(self._second_half_return_heading):.1f}deg"
                )
            return

        if self._second_half_return_phase == "far_side_leg":
            heading = self._second_half_return_heading
            travelled = self._progress_from_ref(
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
            )
            fence_width_m = math.hypot(first_x - second_x, first_y - second_y)
            target_x, target_y = _project(
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
                fence_width_m,
            )
            target_travel = (
                (target_x - self._second_half_return_start_x) * math.cos(heading)
                + (target_y - self._second_half_return_start_y) * math.sin(heading)
            )
            near_planned_corner = travelled >= max(0.0, target_travel - RETURN_CORNER_TOLERANCE_M)
            fence_at_full_width = (
                travelled >= max(0.0, target_travel - FENCE_TURN_STANDOFF_M)
                and self._front_range_m <= FENCE_TURN_STANDOFF_M
            )
            if near_planned_corner or fence_at_full_width:
                self._publish_stop()
                self._record("far_side_corner")
                self._second_half_return_phase = "turn_to_close_side"
                self._state_entered_at = time.time()
                self._turn_target_yaw = _snap_cardinal_yaw(heading + math.pi / 2.0)
                self.get_logger().info(
                    f"second_half_return: far side leg complete travelled={travelled:.2f}m "
                    f"target={target_travel:.2f}m "
                    f"front={self._front_range_m:.2f}m"
                )
                return
            self.get_logger().info(
                f"second_half_return_far_side: "
                f"travelled={travelled:.2f}m target={target_travel:.2f}m"
            )
            self._drive_locked_heading(
                "second_half_return_far_side",
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
                travelled,
            )
            return

        if self._second_half_return_phase == "turn_to_close_side":
            assert self._turn_target_yaw is not None
            yaw_err = ((self._turn_target_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
            if self._turn_step_done(yaw_err):
                self._publish_stop()
                self._local_turn_active = False
                self._second_half_return_phase = "closing_leg"
                self._second_half_return_heading = self._turn_target_yaw
                self._second_half_return_start_x = self._robot_x
                self._second_half_return_start_y = self._robot_y
                self._state_entered_at = time.time()
                self.get_logger().info(
                    f"second_half_return: turn_to_close_side complete heading="
                    f"{math.degrees(self._second_half_return_heading):.1f}deg"
                )
            return

        if self._second_half_return_phase == "closing_leg":
            heading = self._second_half_return_heading
            travelled = self._progress_from_ref(
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
            )
            target_travel = (
                (first_x - self._second_half_return_start_x) * math.cos(heading)
                + (first_y - self._second_half_return_start_y) * math.sin(heading)
            )
            dist_to_corner = math.hypot(self._robot_x - first_x, self._robot_y - first_y)
            if (
                dist_to_corner <= LOOP_CLOSURE_FINAL_TOLERANCE_M
                or travelled >= max(0.0, target_travel - LOOP_CLOSURE_FINAL_TOLERANCE_M)
            ):
                self._publish_stop()
                self._record("perimeter_closed")
                self._second_half_return_phase = None
                self._second_half_return_heading = None
                self._second_half_return_start_x = None
                self._second_half_return_start_y = None
                self._finalize(success=True)
                return
            self.get_logger().info(
                f"second_half_return_closing: dist_to_corner={dist_to_corner:.2f}m "
                f"travelled={travelled:.2f}m target={target_travel:.2f}m"
            )
            self._drive_locked_heading(
                "second_half_return_closing",
                self._second_half_return_start_x,
                self._second_half_return_start_y,
                heading,
                travelled,
            )
            return

        self._fail(f"unknown_second_half_return_phase_{self._second_half_return_phase}")

    # ------------------------------------------------------------------
    # Nav2 helpers
    # ------------------------------------------------------------------

    def _safe_dist(self, requested: float, clearance: float = 1.5) -> float:
        """Cap a requested forward distance to stay clear of the detected obstacle.

        Prevents the planner from receiving a goal that lands inside an occupied
        (fence / net) costmap cell, which causes immediate plan failure.

        Uses 0.15 m minimum (not 0.5 m) so the robot can make small final
        approach hops when very close to the fence without overshooting into the
        inflation / lethal zone.  The follow-state trigger thresholds are set at
        FENCE_STOP_M + 0.35 = 0.75 m to catch the robot before it would need
        a hop that lands inside the inflated obstacle zone.
        """
        if math.isinf(self._front_range_m):
            return requested
        return max(0.15, min(requested, self._front_range_m - clearance))

    def _choose_cross_net_yaw(self) -> float:
        """Pick the perpendicular heading that points back into the court."""
        left_yaw = self._robot_yaw + math.pi / 2
        right_yaw = self._robot_yaw - math.pi / 2
        if self._loop_ref_x is None or self._loop_ref_y is None:
            return left_yaw

        to_ref_x = self._loop_ref_x - self._robot_x
        to_ref_y = self._loop_ref_y - self._robot_y

        def score(yaw: float) -> float:
            return math.cos(yaw) * to_ref_x + math.sin(yaw) * to_ref_y

        return left_yaw if score(left_yaw) >= score(right_yaw) else right_yaw

    def _distance_from_loop_ref(self) -> float:
        if self._loop_ref_x is None or self._loop_ref_y is None:
            return 0.0
        return math.hypot(self._robot_x - self._loop_ref_x, self._robot_y - self._loop_ref_y)

    def _progress_from_ref(self, ref_x: float, ref_y: float, heading: float) -> float:
        dx = self._robot_x - ref_x
        dy = self._robot_y - ref_y
        return dx * math.cos(heading) + dy * math.sin(heading)

    def _cross_track_from_ref(self, ref_x: float, ref_y: float, heading: float) -> float:
        dx = self._robot_x - ref_x
        dy = self._robot_y - ref_y
        return -dx * math.sin(heading) + dy * math.cos(heading)

    def _drive_locked_heading(
        self,
        label: str,
        ref_x: float,
        ref_y: float,
        heading: float,
        travelled: float,
    ) -> None:
        self._cancel_nav()
        cross_track = self._cross_track_from_ref(ref_x, ref_y, heading)
        correction = max(-0.35, min(0.35, cross_track * SURVEY_STRAIGHT_CROSSTRACK_KP))
        target_yaw = heading - correction
        yaw_err = ((target_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi

        twist = Twist()
        twist.linear.x = SURVEY_STRAIGHT_LINEAR_M_S if abs(yaw_err) < math.radians(35.0) else 0.08
        twist.angular.z = max(-0.35, min(0.35, yaw_err * SURVEY_STRAIGHT_YAW_KP))
        self._cmd_vel_pub.publish(twist)

        if int(time.time() * 2.0) % 4 == 0:
            self.get_logger().info(
                f"{label}: straight travelled={travelled:.2f}m front={self._front_range_m:.2f}m "
                f"cross_track={cross_track:.2f}m heading={math.degrees(heading):.1f}deg"
            )

    def _distance_from_state_start(self) -> float:
        return math.hypot(self._robot_x - self._state_start_x, self._robot_y - self._state_start_y)

    def _distance_from_net_approach_start(self) -> float:
        if self._net_approach_start_x is None or self._net_approach_start_y is None:
            return 0.0
        return math.hypot(
            self._robot_x - self._net_approach_start_x,
            self._robot_y - self._net_approach_start_y,
        )

    def _net_distance(self, net: dict[str, Any] | None) -> float | None:
        if not net:
            return None
        try:
            dist = float(net.get("distance_m", 0.0))
        except (TypeError, ValueError):
            return None
        if dist <= 0.0 or not math.isfinite(dist):
            return None
        return dist

    def _first_visible_net_for_initial_approach(self) -> dict[str, Any] | None:
        if self._state not in (SurveyState.FIND_FIRST_OBSTACLE, SurveyState.APPROACH_NET):
            return None
        survey_net = self._survey_vision_net()
        if survey_net is not None:
            return survey_net
        net = self._last_landmarks.get("net")
        if isinstance(net, dict):
            return net
        return None

    def _survey_vision_net(self) -> dict[str, Any] | None:
        if self._last_survey_vision.get("obstacle_class") != "net":
            return None
        try:
            dist = float(self._last_survey_vision.get("center_m", 0.0))
        except (TypeError, ValueError):
            return None
        if dist <= 0.0 or not math.isfinite(dist):
            return None
        return {
            "label": "net",
            "distance_m": dist,
            "oak_depth_m": dist,
            "bearing_rad": 0.0,
            "confidence": 1.0,
            "source": "survey_vision",
        }

    def _lock_net_detection(self, net: dict[str, Any]) -> float | None:
        """Freeze the first confident net label and use LiDAR as its range."""
        if self._locked_first_obstacle == "net":
            return self._locked_net_distance_m

        try:
            bearing = float(net.get("bearing_rad", 0.0))
        except (TypeError, ValueError):
            bearing = 0.0
        try:
            confidence = float(net.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        source = str(net.get("source", "court_landmarks"))
        lidar_dist = self._front_range_m if math.isfinite(self._front_range_m) else None
        oak_depth = self._net_distance(net)
        selected_range = lidar_dist if lidar_dist is not None else oak_depth
        if selected_range is None:
            return None

        self._locked_first_obstacle = "net"
        self._locked_net_bearing_rad = bearing
        self._locked_net_distance_m = selected_range
        self._locked_net_confidence = confidence
        self._locked_net_approach_yaw_rad = self._robot_yaw + bearing
        self._net_approach_start_x = self._robot_x
        self._net_approach_start_y = self._robot_y
        self._net_approach_target_m = max(0.0, selected_range - NET_STANDOFF_M)
        map_x, map_y = _project(self._robot_x, self._robot_y, self._robot_yaw + bearing, selected_range)
        self._locked_net = {
            "label": "net",
            "range_m": round(selected_range, 3),
            "range_source": "lidar" if lidar_dist is not None else "oak_depth",
            "lidar_range_m": round(lidar_dist, 3) if lidar_dist is not None else None,
            "oak_depth_m": round(oak_depth, 3) if oak_depth is not None else None,
            "bearing_rad": round(bearing, 5),
            "confidence": round(confidence, 3),
            "vision_source": source,
            "robot_x_m": round(self._robot_x, 3),
            "robot_y_m": round(self._robot_y, 3),
            "robot_yaw_rad": round(self._robot_yaw, 5),
            "map_x_m": round(map_x, 3),
            "map_y_m": round(map_y, 3),
        }
        self.get_logger().info(
            f"locked net: range={selected_range:.2f}m "
            f"range_source={self._locked_net['range_source']} "
            f"lidar={lidar_dist if lidar_dist is not None else float('nan'):.2f}m "
            f"oak_depth={oak_depth if oak_depth is not None else float('nan'):.2f}m "
            f"bearing={math.degrees(bearing):.1f}deg confidence={confidence:.2f} "
            f"target_travel={self._net_approach_target_m if self._net_approach_target_m is not None else float('nan'):.2f}m"
        )
        return selected_range

    def _side_lidar_line_diagnostic(self) -> dict[str, Any] | None:
        """Estimate side obstacle line angle relative to robot forward axis.

        0 degrees means the nearest LiDAR line is parallel with the robot's
        forward direction.  This is a diagnostic for the "parallel to net"
        DoD, not a control input. After the first left turn the net should be
        visible mainly in a side sector, so front obstacles are ignored here.
        """
        best: dict[str, Any] | None = None
        for side, min_deg, max_deg in (("left", 55.0, 125.0), ("right", -125.0, -55.0)):
            side_points: list[tuple[float, float, float]] = []
            for x, y, r in self._last_scan_points:
                angle_deg = math.degrees(math.atan2(y, x))
                if min_deg <= angle_deg <= max_deg and r <= 8.0:
                    side_points.append((x, y, r))
            if len(side_points) < 8:
                continue

            nearest = min(p[2] for p in side_points)
            cluster = [(x, y) for x, y, r in side_points if r <= nearest + 1.25]
            if len(cluster) < 8:
                continue

            line_angle = self._fit_line_angle_deg(cluster)
            if line_angle is None:
                continue

            diag = {
                "side": side,
                "angle_deg": line_angle,
                "abs_angle_deg": abs(line_angle),
                "nearest_m": nearest,
                "points": len(side_points),
                "cluster_points": len(cluster),
            }
            if best is None or nearest < float(best["nearest_m"]):
                best = diag
        return best

    def _fit_line_angle_deg(self, cluster: list[tuple[float, float]]) -> float | None:
        mean_x = sum(p[0] for p in cluster) / len(cluster)
        mean_y = sum(p[1] for p in cluster) / len(cluster)
        sxx = sum((x - mean_x) * (x - mean_x) for x, _ in cluster)
        syy = sum((y - mean_y) * (y - mean_y) for _, y in cluster)
        sxy = sum((x - mean_x) * (y - mean_y) for x, y in cluster)
        if sxx + syy <= 1e-9:
            return None

        angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        # A line has no direction, so normalize to [-90, 90] relative to robot x.
        while angle > math.pi / 2:
            angle -= math.pi
        while angle < -math.pi / 2:
            angle += math.pi
        return math.degrees(angle)

    def _spin_to_target(self, target_yaw: float, next_state: SurveyState, event: str) -> None:
        """Turn toward target_yaw using iterative TF-verified spins.

        The Nav2 Spin behavior tracks progress via /odom.  In simulation the
        diff-drive odom over-counts rotation (wheel slip) so the behavior
        declares "success" before the robot has physically completed the full
        angle.  This method checks the ACTUAL yaw from TF after each spin and
        sends a corrective spin for the remaining angle until within 8Â°.

        spin_dist sign: positive = CCW (left), negative = CW (right) â€” the
        Spin behavior respects the sign so we can also correct a small overshoot.
        """
        # yaw_err in [-Ï€, Ï€]: positive â†’ need more CCW, negative â†’ overshot
        yaw_err = ((target_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
        self._drive_local_turn(yaw_err, next_state, event)
        return
        if not self._nav_active:
            if abs(yaw_err) < math.radians(8.0):
                if self._state == SurveyState.TURN_LEFT_AT_NET:
                    line_diag = self._side_lidar_line_diagnostic()
                    if line_diag is None:
                        self.get_logger().info("turn_left_at_net: lidar_parallel_check unavailable")
                    else:
                        self.get_logger().info(
                            "turn_left_at_net: "
                            f"lidar_parallel_angle={line_diag['angle_deg']:.1f}deg "
                            f"abs={line_diag['abs_angle_deg']:.1f}deg "
                            f"side={line_diag['side']} "
                            f"nearest={line_diag['nearest_m']:.2f}m "
                            f"points={line_diag['points']} "
                            f"cluster={line_diag['cluster_points']}"
                        )
                self._enter(next_state, event)
                return
            # spin_dist = yaw_err handles both CCW (positive) and CW correction (negative)
            self.get_logger().info(
                f"{self._state.value}: TF yaw_err={math.degrees(abs(yaw_err)):.1f}Â° "
                f"â†’ spinning {math.degrees(yaw_err):.1f}Â°"
            )
            if self._navigator is not None:
                self._navigator.spin(spin_dist=yaw_err, time_allowance=30)
                self._nav_active = True
        elif self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"{self._state.value}: spin done ({result}), "
                f"TF yaw_err now={math.degrees(abs(yaw_err)):.1f}Â°"
            )
            # Do NOT transition here â€” next tick rechecks yaw_err from TF

    def _drive_local_turn(self, yaw_err: float, next_state: SurveyState, event: str) -> None:
        if abs(yaw_err) < math.radians(SURVEY_TURN_TOLERANCE_DEG):
            self._publish_stop()
            self._local_turn_active = False
            if self._state == SurveyState.TURN_LEFT_AT_NET:
                line_diag = self._side_lidar_line_diagnostic()
                if line_diag is None:
                    self.get_logger().info("turn_left_at_net: lidar_parallel_check unavailable")
                else:
                    self.get_logger().info(
                        "turn_left_at_net: "
                        f"lidar_parallel_angle={line_diag['angle_deg']:.1f}deg "
                        f"abs={line_diag['abs_angle_deg']:.1f}deg "
                        f"side={line_diag['side']} "
                        f"nearest={line_diag['nearest_m']:.2f}m "
                        f"points={line_diag['points']} "
                        f"cluster={line_diag['cluster_points']}"
                    )
            self.get_logger().info(
                f"{self._state.value}: local turn complete, "
                f"TF yaw_err={math.degrees(abs(yaw_err)):.1f}deg"
            )
            self._enter(next_state, event)
            return

        if not self._local_turn_active:
            self.get_logger().info(
                f"{self._state.value}: local turn start "
                f"TF yaw_err={math.degrees(abs(yaw_err)):.1f}deg "
                f"target_delta={math.degrees(yaw_err):.1f}deg"
            )
            self._local_turn_active = True

        twist = Twist()
        speed = min(SURVEY_TURN_MAX_RAD_S, max(SURVEY_TURN_MIN_RAD_S, abs(yaw_err) * 0.9))
        twist.angular.z = math.copysign(speed, yaw_err)
        self._cmd_vel_pub.publish(twist)

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _turn_step_done(self, yaw_err: float) -> bool:
        if abs(yaw_err) < math.radians(SURVEY_TURN_TOLERANCE_DEG):
            return True
        if not self._local_turn_active:
            self._local_turn_active = True
        twist = Twist()
        speed = min(SURVEY_TURN_MAX_RAD_S, max(SURVEY_TURN_MIN_RAD_S, abs(yaw_err) * 0.9))
        twist.angular.z = math.copysign(speed, yaw_err)
        self._cmd_vel_pub.publish(twist)
        return False

    def _nav_to(self, x: float, y: float, yaw: float) -> None:
        if self._navigator is None:
            return
        goal = _make_pose_stamped(x, y, yaw)
        if BT_XML:
            goal.header.frame_id = "map"  # bt_xml passed as parameter to navigator
            self._navigator.goToPose(goal, behavior_tree=BT_XML)
        else:
            self._navigator.goToPose(goal)
        self._nav_active = True

    def _cancel_nav(self) -> None:
        if self._navigator is not None and self._nav_active:
            self._navigator.cancelTask()
        self._nav_active = False

    def _court_model_from_survey(self, success: bool) -> dict[str, Any]:
        points = list(self._navigation_points)
        for maybe_point in (self._locked_net,):
            if maybe_point and maybe_point.get("map_x_m") is not None and maybe_point.get("map_y_m") is not None:
                points.append({
                    "label": "locked_net_map",
                    "x_m": float(maybe_point["map_x_m"]),
                    "y_m": float(maybe_point["map_y_m"]),
                })

        xs = [float(p["x_m"]) for p in points if p.get("x_m") is not None]
        ys = [float(p["y_m"]) for p in points if p.get("y_m") is not None]
        if not xs or not ys:
            return {}

        west_x = min(xs)
        east_x = max(xs)
        south_y = min(ys)
        north_y = max(ys)
        length_m = east_x - west_x
        width_m = north_y - south_y

        loop_error_m = None
        perimeter_closed = self._point_by_label("perimeter_closed")
        first_fence_corner = self._point_by_label("net_to_fence_corner")
        if perimeter_closed and first_fence_corner:
            loop_error_m = math.hypot(
                float(perimeter_closed["x_m"]) - float(first_fence_corner["x_m"]),
                float(perimeter_closed["y_m"]) - float(first_fence_corner["y_m"]),
            )

        canonical = {
            "status": "ESTIMATED" if success else "PARTIAL",
            "source": "court_survey_mission_route",
            "corners": {
                "southwest": {"x_m": round(west_x, 3), "y_m": round(south_y, 3)},
                "northwest": {"x_m": round(west_x, 3), "y_m": round(north_y, 3)},
                "northeast": {"x_m": round(east_x, 3), "y_m": round(north_y, 3)},
                "southeast": {"x_m": round(east_x, 3), "y_m": round(south_y, 3)},
            },
            "extents": {
                "west_x_m": round(west_x, 3),
                "east_x_m": round(east_x, 3),
                "south_y_m": round(south_y, 3),
                "north_y_m": round(north_y, 3),
            },
            "loop_closure_error_m": round(loop_error_m, 3) if loop_error_m is not None else None,
        }

        geometry = {
            "length_m": round(length_m, 3),
            "width_m": round(width_m, 3),
            "method": "survey_route_extents",
            "confidence": "estimated",
        }
        return {
            "canonical_fence_model": canonical,
            "court_geometry": geometry,
            "boundary_distances": {
                "near_baseline_to_fence_m": None,
                "far_baseline_to_fence_m": None,
                "left_sideline_to_fence_m": None,
                "right_sideline_to_fence_m": None,
                "loop_closure_error_m": canonical["loop_closure_error_m"],
            },
            "geometry": {
                "net_world_pos": (
                    {
                        "x_m": self._locked_net.get("map_x_m"),
                        "y_m": self._locked_net.get("map_y_m"),
                    }
                    if self._locked_net else None
                )
            },
            "is_doubles": None,
            "point_count": len(self._navigation_points),
        }

    def _point_by_label(self, label: str) -> dict[str, Any] | None:
        for point in self._navigation_points:
            if point.get("label") == label:
                return point
        return None

    def _check_nav_result_raw(self) -> str | None:
        if not self._nav_active or self._navigator is None:
            return None
        if self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            if result == TaskResult.SUCCEEDED:
                return "succeeded"
            return "failed"
        return None

    # ------------------------------------------------------------------
    # FSM bookkeeping
    # ------------------------------------------------------------------

    def _enter(self, state: SurveyState, event: str) -> None:
        self.get_logger().info(f"survey: {self._state.value} â†’ {state.value} [{event}]")
        self._state = state
        self._last_event = event
        self._state_entered_at = time.time()
        self._state_start_x = self._robot_x
        self._state_start_y = self._robot_y
        self._nav_active = False
        if self._local_turn_active:
            self._publish_stop()
        self._local_turn_active = False
        if state != SurveyState.CROSS_NET:
            self._cross_net_phase = None
            self._cross_net_yaw = None
            self._cross_net_resume_yaw = None
            self._cross_net_start_x = None
            self._cross_net_start_y = None
        if state != SurveyState.SECOND_HALF_RETURN:
            self._second_half_return_phase = None
            self._second_half_return_heading = None
            self._second_half_return_start_x = None
            self._second_half_return_start_y = None
        self._turn_target_yaw = None  # reset so each turn state computes fresh

    def _state_timed_out(self) -> bool:
        timeout_s = SECOND_HALF_RETURN_TIMEOUT_S if self._state == SurveyState.SECOND_HALF_RETURN else STATE_TIMEOUT_S
        return time.time() - self._state_entered_at > timeout_s

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self.get_logger().error(f"survey FAILED: {reason}")
        self._cancel_nav()
        self._publish_stop()
        self._finalize(success=False)

    def _record(self, label: str) -> None:
        if any(p["label"] == label for p in self._navigation_points):
            return
        self._navigation_points.append({
            "label": label,
            "x_m": round(self._robot_x, 3),
            "y_m": round(self._robot_y, 3),
            "state": self._state.value,
        })
        self.get_logger().info(f"survey waypoint recorded: {label} ({self._robot_x:.2f}, {self._robot_y:.2f})")

    # ------------------------------------------------------------------
    # Completion
    # ------------------------------------------------------------------

    def _finalize(self, success: bool) -> None:
        self._publish_stop()
        status = "SUCCESS" if success else "FAILED"
        model = self._court_model_from_survey(success)
        bounds = {
            "surveyed_at": time.time(),
            "status": status,
            "survey_complete": success,
            "survey_type": "court_survey_mission_bt",
            "failure_reason": self._failure_reason,
            "navigation_points": self._navigation_points,
            "navigation_route": [
                {"x_m": p["x_m"], "y_m": p["y_m"], "label": p["label"]}
                for p in self._navigation_points
            ],
            "locked_net": self._locked_net,
            "loop_reference": (
                {"x_m": self._loop_ref_x, "y_m": self._loop_ref_y}
                if self._loop_ref_x is not None else None
            ),
        }
        bounds.update(model)
        COURT_BOUNDARY_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = COURT_BOUNDARY_FILE.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(COURT_BOUNDARY_FILE)
        self.get_logger().info(
            f"survey {status} â€” {len(self._navigation_points)} waypoints â†’ {COURT_BOUNDARY_FILE}"
        )
        self._enter(
            SurveyState.COMPLETE if success else SurveyState.FAILED,
            f"survey_{status.lower()}",
        )
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CourtSurveyMissionNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
