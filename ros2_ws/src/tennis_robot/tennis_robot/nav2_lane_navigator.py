"""Nav2 NavigateToPose bridge for driving collection lawnmower lanes.

The collection mission (`ServiceLineDistributionScanMission`) produces lane
waypoints in the *map* frame. Instead of the built-in P-controller (`_nav_to`),
we hand each waypoint to Nav2's NavigateToPose action so the global planner
(Smac) + Regulated Pure Pursuit controller drive the robot with obstacle
avoidance against the survey map.

Channel arbitration (see config/twist_mux.yaml):
  * Nav2 publishes on /cmd_vel_nav        (priority 50)
  * fine-approach capture / teleop stream  (priority 100)
While a lane leg is in progress the controller emits an *idle* command, so the
motor adapter goes silent (publish-zero-once) and twist_mux hands the wheels to
Nav2. When a ball is found we cancel the goal and let the higher-priority
capture stream drive; the next sweep tick re-requests the current waypoint,
which makes Nav2 replan from the post-capture pose.

All action I/O is non-blocking (futures + callbacks) so it is safe to call
`request()` once per control tick from within the node's executor.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class LaneNavState(str, Enum):
    IDLE = "idle"
    PENDING = "pending"          # goal sent, awaiting acceptance
    ACTIVE = "active"            # accepted, navigating
    REACHED = "reached"          # goal succeeded
    FAILED = "failed"            # rejected / aborted / canceled
    UNAVAILABLE = "unavailable"  # Nav2 action server not up


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Nav2LaneNavigator:
    """Thin, non-blocking wrapper around the NavigateToPose action."""

    def __init__(
        self,
        node: Node,
        action_name: str = "navigate_to_pose",
        frame_id: str = "map",
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._state = LaneNavState.IDLE
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._goal_handle = None
        self._send_future = None
        self._result_future = None

    # ---- observable state ----

    @property
    def state(self) -> LaneNavState:
        return self._state

    @property
    def goal_xy(self) -> Optional[Tuple[float, float]]:
        return self._goal_xy

    @property
    def reached(self) -> bool:
        return self._state == LaneNavState.REACHED

    @property
    def busy(self) -> bool:
        return self._state in (LaneNavState.PENDING, LaneNavState.ACTIVE)

    # ---- control ----

    def reset(self) -> None:
        self.cancel()
        self._state = LaneNavState.IDLE
        self._goal_xy = None

    def request(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        replan_tolerance_m: float = 0.5,
    ) -> None:
        """Ensure Nav2 is heading to (x, y, yaw) in the map frame.

        No-op while a goal toward ~the same point is still in flight; if the
        requested waypoint moved (next lane) the in-flight goal is superseded.
        """
        if (
            self.busy
            and self._goal_xy is not None
            and math.hypot(self._goal_xy[0] - x, self._goal_xy[1] - y) <= replan_tolerance_m
        ):
            return
        if not self._client.server_is_ready():
            self._state = LaneNavState.UNAVAILABLE
            return
        self._send(x, y, yaw)

    def cancel(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - best-effort cancel, never raise in control loop
                pass
        self._goal_handle = None
        if self.busy:
            self._state = LaneNavState.IDLE

    # ---- internals ----

    def _send(self, x: float, y: float, yaw: float) -> None:
        ps = PoseStamped()
        ps.header.frame_id = self._frame_id
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = _quaternion_from_yaw(float(yaw))

        goal = NavigateToPose.Goal()
        goal.pose = ps

        self._goal_xy = (float(x), float(y))
        self._state = LaneNavState.PENDING
        self._send_future = self._client.send_goal_async(goal)
        self._send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception:  # noqa: BLE001
            handle = None
        if handle is None or not handle.accepted:
            self._state = LaneNavState.FAILED
            self._goal_handle = None
            return
        self._goal_handle = handle
        self._state = LaneNavState.ACTIVE
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception:  # noqa: BLE001
            status = None
        self._state = (
            LaneNavState.REACHED
            if status == GoalStatus.STATUS_SUCCEEDED
            else LaneNavState.FAILED
        )
        self._goal_handle = None
