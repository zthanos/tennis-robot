"""Court Survey Mission Node — Python brain for the Court Knowledge Model.

Implements the perimeter survey FSM from the Court Knowledge Model spec using:
  - nav2_simple_commander.BasicNavigator  for Nav2 NavigateToPose goals
  - /court_landmarks (JSON)               for live camera landmark detections
  - /scan (LaserScan)                     for LiDAR front/side range checks
  - /odom (Odometry)                      for current robot pose

State machine (mirrors the spec FSM, adapted for Nav2):
  INIT
  FIND_FIRST_OBSTACLE   drive forward; classify first obstacle (net vs fence)
  APPROACH_NET          navigate to net standoff using landmark bearing+depth
  TURN_LEFT_AT_NET      rotate 90° left; record loop-reference pose
  FOLLOW_NET_TO_FENCE   navigate along net direction toward sideline fence
  TURN_LEFT_AT_FENCE    rotate 90° left
  FOLLOW_FENCE          navigate along fence until next corner
  CROSS_NET             navigate through right-side net gap
  SECOND_HALF           mirror of FOLLOW_NET_TO_FENCE … FOLLOW_FENCE for far half
  COMPLETE              write court_boundary.json; publish result

On SUCCESS the node writes runtime/court_boundary.json and shuts down.
On FAILURE it writes a failed status and shuts down with exit code 1.

Environment variables
─────────────────────
COURT_SURVEY_NET_STANDOFF_M       float  default 0.35
COURT_SURVEY_FENCE_STOP_M         float  default 0.40
COURT_SURVEY_DRIVE_FORWARD_M      float  default 8.0  (FIND_FIRST_OBSTACLE)
COURT_SURVEY_STATE_TIMEOUT_S      float  default 120.0 (per state)
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
from geometry_msgs.msg import PoseStamped
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

NET_STANDOFF_M: float = float(os.getenv("COURT_SURVEY_NET_STANDOFF_M", "0.35"))
FENCE_STOP_M: float = float(os.getenv("COURT_SURVEY_FENCE_STOP_M", "0.40"))
DRIVE_FORWARD_M: float = float(os.getenv("COURT_SURVEY_DRIVE_FORWARD_M", "8.0"))
STATE_TIMEOUT_S: float = float(os.getenv("COURT_SURVEY_STATE_TIMEOUT_S", "120.0"))
LANDMARK_MIN_CONF: float = float(os.getenv("COURT_SURVEY_LANDMARK_MIN_CONF", "0.25"))
BT_XML: str = os.getenv("COURT_SURVEY_BT_XML", "")

FRONT_LIDAR_HALF_DEG: float = 20.0   # ±20° sector for front range


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


def _front_range_from_scan(ranges: list[float], angle_min: float, angle_inc: float) -> float:
    """30th-pct front range over ±20°."""
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

        # Sensor caches
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._front_range_m: float = math.inf
        self._last_landmarks: dict[str, Any] = {}
        self._scan_angle_min: float = -math.pi
        self._scan_angle_inc: float = math.radians(1.0)

        # Loop-closure reference (recorded at first net-left-turn)
        self._loop_ref_x: float | None = None
        self._loop_ref_y: float | None = None

        # Stored target yaw for turn states (set once on entry, cleared on _enter())
        # Avoids recalculating from self._robot_yaw on every tick while turning.
        self._turn_target_yaw: float | None = None

        # Nav2
        self._navigator: BasicNavigator | None = None
        self._nav_active: bool = False
        self._nav2_lifecycle_active: bool = False

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

        # Main FSM timer at 5 Hz
        self.create_timer(0.2, self._step)

        self.get_logger().info("court_survey_mission_node started")

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _update_pose_from_tf(self) -> None:
        """Update robot pose from TF (map→base_link) instead of /odom topic.

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
        self._front_range_m = _front_range_from_scan(
            list(msg.ranges), self._scan_angle_min, self._scan_angle_inc
        )

    def _landmarks_cb(self, msg: String) -> None:
        try:
            self._last_landmarks = json.loads(msg.data)
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
            return  # still starting up — check again next tick (5 Hz)
        if not self._nav2_lifecycle_active:
            self.get_logger().info("Nav2 action server ready, waiting for lifecycle active...")
            self._navigator._waitForNodeToActivate("bt_navigator")
            self._nav2_lifecycle_active = True
        self.get_logger().info("Nav2 action server ready, starting survey")
        self._enter(SurveyState.FIND_FIRST_OBSTACLE, "survey_started")

    def _state_find_first_obstacle(self) -> None:
        """Drive forward; classify first obstacle as net or fence.

        Net is checked at ANY range — as soon as the landmark detector sees the
        net with enough confidence we cancel the approach and start the perimeter
        traverse.  There is no need to drive closer: the net position is recorded
        here and the robot turns immediately.
        If we reach 2.5 m without a net detection → it's a fence end-wall.
        """
        # Net detected at any range → start perimeter immediately (no approach)
        net = self._last_landmarks.get("net")
        if net and net.get("confidence", 0) >= LANDMARK_MIN_CONF:
            self._record("first_obstacle_net")
            self._cancel_nav()
            self._enter(SurveyState.TURN_LEFT_AT_NET, "net_detected")
            return

        # Reached close range with no net → fence end-wall
        if self._front_range_m <= 2.5:
            self._record("first_obstacle_fence")
            self._enter(SurveyState.FOLLOW_FENCE_TO_CORNER, "first_obstacle_fence")
            return
        # If nav task completed (success or failure), clear the flag so we retry.
        # This handles: planner failure (goal in obstacle), path blocked, etc.
        if self._nav_active and self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"find_first_obstacle nav done ({result}); "
                f"front={self._front_range_m:.2f}m — retrying"
            )
        # Keep driving forward.
        # IMPORTANT: never project the goal INTO the detected obstacle — the planner
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
                f"→ goal {gx:.2f},{gy:.2f} (dist={target_dist:.2f}m)"
            )
            self._nav_to(gx, gy, self._robot_yaw)

    def _state_approach_net(self) -> None:
        """Navigate to net standoff using camera bearing + depth."""
        net = self._last_landmarks.get("net")
        if net and net.get("distance_m", 0) and net["distance_m"] > NET_STANDOFF_M:
            # Project goal: along net bearing at (distance - standoff)
            bearing = float(net["bearing_rad"])
            dist = float(net["distance_m"]) - NET_STANDOFF_M
            gx, gy = _project(
                self._robot_x, self._robot_y,
                self._robot_yaw + bearing, dist,
            )
            # Face the net: heading toward net
            goal_yaw = self._robot_yaw + bearing
            if not self._nav_active:
                self._nav_to(gx, gy, goal_yaw)
        # Check arrival: front range ≤ standoff + 0.10
        if self._front_range_m <= NET_STANDOFF_M + 0.10:
            self._record("net_standoff_reached")
            self._cancel_nav()
            self._enter(SurveyState.TURN_LEFT_AT_NET, "near_net")
            return
        self._check_nav_result(SurveyState.TURN_LEFT_AT_NET, "near_net")

    def _state_turn_left_at_net(self) -> None:
        """Rotate 90° left; verify with TF; retry corrective spins until ±8°."""
        if self._turn_target_yaw is None:
            self._turn_target_yaw = self._robot_yaw + math.pi / 2
            # Record position before any spinning
            self._loop_ref_x = self._robot_x
            self._loop_ref_y = self._robot_y
            self._record("first_net_left_turn_reference")
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_NET_TO_FENCE,
            "turn_left_at_net_complete",
        )

    def _state_follow_net_to_fence(self) -> None:
        """Drive parallel to net until fence detected ahead."""
        if self._front_range_m <= FENCE_STOP_M + 0.35:
            self._record("net_to_fence_corner")
            self._cancel_nav()
            self._enter(SurveyState.TURN_LEFT_AT_FENCE_1, "near_fence")
            return
        self._retry_if_complete("follow_net_to_fence")
        if not self._nav_active:
            d = self._safe_dist(15.0)
            gx, gy = _project(self._robot_x, self._robot_y, self._robot_yaw, d)
            self._nav_to(gx, gy, self._robot_yaw)
        self._check_nav_result(SurveyState.TURN_LEFT_AT_FENCE_1, "near_fence")

    def _state_turn_left_at_fence_1(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = self._robot_yaw + math.pi / 2
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_FENCE_TO_CORNER,
            "turn_left_at_fence_1_complete",
        )

    def _state_follow_fence_to_corner(self) -> None:
        """Follow fence until next corner."""
        if self._front_range_m <= FENCE_STOP_M + 0.35:
            self._record("fence_corner")
            self._cancel_nav()
            self._enter(SurveyState.TURN_LEFT_AT_CORNER, "corner_detected")
            return
        self._retry_if_complete("follow_fence_to_corner")
        if not self._nav_active:
            d = self._safe_dist(20.0)
            gx, gy = _project(self._robot_x, self._robot_y, self._robot_yaw, d)
            self._nav_to(gx, gy, self._robot_yaw)
        self._check_nav_result(SurveyState.TURN_LEFT_AT_CORNER, "corner_detected")

    def _state_turn_left_at_corner(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = self._robot_yaw + math.pi / 2
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.FOLLOW_FENCE_TO_NET,
            "turn_left_at_corner_complete",
        )

    def _state_follow_fence_to_net(self) -> None:
        """Follow fence until net detected again → approach right-side gap."""
        net = self._last_landmarks.get("net")
        if net and net.get("confidence", 0) >= LANDMARK_MIN_CONF:
            self._record("net_detected_from_far_side")
            self._cancel_nav()
            self._enter(SurveyState.CROSS_NET, "net_detected")
            return
        if self._front_range_m <= FENCE_STOP_M + 0.35:
            # Hit another fence before seeing net — still transition
            self._record("fence_before_net_far_side")
            self._cancel_nav()
            self._enter(SurveyState.CROSS_NET, "near_fence")
            return
        self._retry_if_complete("follow_fence_to_net")
        if not self._nav_active:
            d = self._safe_dist(20.0)
            gx, gy = _project(self._robot_x, self._robot_y, self._robot_yaw, d)
            self._nav_to(gx, gy, self._robot_yaw)
        self._check_nav_result(SurveyState.CROSS_NET, "nav_result")

    def _state_cross_net(self) -> None:
        """Navigate through the right-side net gap (between net post and fence).

        Strategy: use /court_landmark_poses right-side passage, or dead-reckon
        90° right → forward past net → 90° left to resume perimeter heading.
        """
        # Simplified: turn right 90° → drive past net → turn left 90°
        # A full implementation would use the landmark right-side gap bearing.
        target_yaw = self._robot_yaw - math.pi / 2   # turn right
        if not self._nav_active:
            # Project a goal past the net: forward 3 m after turning right
            gx, gy = _project(self._robot_x, self._robot_y, target_yaw, 3.0)
            self._nav_to(gx, gy, target_yaw + math.pi / 2)  # resume original heading
            self._record("crossing_net_right_side")
        if self._check_nav_result(SurveyState.SECOND_HALF_FOLLOW_FENCE, "gap_crossed"):
            return

    def _state_second_half_follow_fence(self) -> None:
        """Mirror of FOLLOW_FENCE_TO_CORNER for the second half of the court."""
        if self._front_range_m <= FENCE_STOP_M + 0.35:
            self._record("second_half_corner")
            self._cancel_nav()
            self._enter(SurveyState.SECOND_HALF_TURN, "corner_detected_second_half")
            return
        self._retry_if_complete("second_half_follow_fence")
        if not self._nav_active:
            d = self._safe_dist(20.0)
            gx, gy = _project(self._robot_x, self._robot_y, self._robot_yaw, d)
            self._nav_to(gx, gy, self._robot_yaw)
        self._check_nav_result(SurveyState.SECOND_HALF_TURN, "corner_detected_second_half")

    def _state_second_half_turn(self) -> None:
        if self._turn_target_yaw is None:
            self._turn_target_yaw = self._robot_yaw + math.pi / 2
        self._spin_to_target(
            self._turn_target_yaw,
            SurveyState.SECOND_HALF_RETURN,
            "second_half_turn_complete",
        )

    def _state_second_half_return(self) -> None:
        """Drive back toward the loop-reference (first net-left-turn point)."""
        if self._loop_ref_x is None or self._loop_ref_y is None:
            self._fail("no_loop_reference")
            return
        dist_to_ref = math.hypot(
            self._robot_x - self._loop_ref_x,
            self._robot_y - self._loop_ref_y,
        )
        if dist_to_ref <= 1.5:
            self._record("loop_closed")
            self._cancel_nav()
            self._finalize(success=True)
            return
        if not self._nav_active:
            self._nav_to(self._loop_ref_x, self._loop_ref_y, self._robot_yaw)
        if self._check_nav_result_raw():
            if self._check_nav_result_raw() == "succeeded":
                dist_to_ref = math.hypot(
                    self._robot_x - self._loop_ref_x,
                    self._robot_y - self._loop_ref_y,
                )
                if dist_to_ref <= 2.0:
                    self._record("loop_closed")
                    self._finalize(success=True)
                else:
                    self._fail("loop_closure_too_far")

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
        a hop that lands inside the 0.55 m inflation radius.
        """
        if math.isinf(self._front_range_m):
            return requested
        return max(0.15, min(requested, self._front_range_m - clearance))

    def _spin_to_target(self, target_yaw: float, next_state: SurveyState, event: str) -> None:
        """Turn toward target_yaw using iterative TF-verified spins.

        The Nav2 Spin behavior tracks progress via /odom.  In simulation the
        diff-drive odom over-counts rotation (wheel slip) so the behavior
        declares "success" before the robot has physically completed the full
        angle.  This method checks the ACTUAL yaw from TF after each spin and
        sends a corrective spin for the remaining angle until within 8°.

        spin_dist sign: positive = CCW (left), negative = CW (right) — the
        Spin behavior respects the sign so we can also correct a small overshoot.
        """
        # yaw_err in [-π, π]: positive → need more CCW, negative → overshot
        yaw_err = ((target_yaw - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
        if not self._nav_active:
            if abs(yaw_err) < math.radians(8.0):
                self._enter(next_state, event)
                return
            # spin_dist = yaw_err handles both CCW (positive) and CW correction (negative)
            self.get_logger().info(
                f"{self._state.value}: TF yaw_err={math.degrees(abs(yaw_err)):.1f}° "
                f"→ spinning {math.degrees(yaw_err):.1f}°"
            )
            if self._navigator is not None:
                self._navigator.spin(spin_dist=yaw_err, time_allowance=30)
                self._nav_active = True
        elif self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"{self._state.value}: spin done ({result}), "
                f"TF yaw_err now={math.degrees(abs(yaw_err)):.1f}°"
            )
            # Do NOT transition here — next tick rechecks yaw_err from TF

    def _retry_if_complete(self, label: str) -> None:
        """If the current nav task has finished (success or fail), clear _nav_active.

        Call this at the top of any state that waits on a sensor trigger rather
        than a nav-completion event.  Without this, a failed plan leaves
        _nav_active=True forever and the state times out.
        """
        if self._nav_active and self._navigator is not None and self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            self.get_logger().info(
                f"{label} nav done ({result}); front={self._front_range_m:.2f}m — retrying"
            )

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

    def _check_nav_result(
        self, next_state: SurveyState, event: str
    ) -> bool:
        """Return True and transition if navigation succeeded."""
        if not self._nav_active or self._navigator is None:
            return False
        if self._navigator.isTaskComplete():
            result = self._navigator.getResult()
            self._nav_active = False
            if result == TaskResult.SUCCEEDED:
                self._enter(next_state, event)
                return True
            if result == TaskResult.FAILED:
                self._fail(f"nav_failed_in_{self._state.value}")
        return False

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
        self.get_logger().info(f"survey: {self._state.value} → {state.value} [{event}]")
        self._state = state
        self._last_event = event
        self._state_entered_at = time.time()
        self._nav_active = False
        self._turn_target_yaw = None  # reset so each turn state computes fresh

    def _state_timed_out(self) -> bool:
        return time.time() - self._state_entered_at > STATE_TIMEOUT_S

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self.get_logger().error(f"survey FAILED: {reason}")
        self._cancel_nav()
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
        status = "SUCCESS" if success else "FAILED"
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
            "loop_reference": (
                {"x_m": self._loop_ref_x, "y_m": self._loop_ref_y}
                if self._loop_ref_x is not None else None
            ),
        }
        COURT_BOUNDARY_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = COURT_BOUNDARY_FILE.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(COURT_BOUNDARY_FILE)
        self.get_logger().info(
            f"survey {status} — {len(self._navigation_points)} waypoints → {COURT_BOUNDARY_FILE}"
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
