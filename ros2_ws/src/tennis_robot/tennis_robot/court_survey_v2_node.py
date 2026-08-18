#!/usr/bin/env python3
"""Court Survey v2 mission node — LiDAR occupancy → Court Knowledge Model.

Replaces the dead-reckoning perimeter FSM. The court is MEASURED from the
accumulated 360° LiDAR occupancy map, not traced by a fragile perimeter drive.

Flow (see docs/survey/court-survey-v2-spec-el.md):
  INIT → FIND_NET (drive to net, lock it → court frame)
       → COVERAGE (deterministic drive-to-waypoint on the SLAM pose to each
                   vantage point; NOT Nav2 — Nav2 proved too flaky run-to-run)
       → after each vantage: try extraction
            OK         → write court_boundary.json (v2) → DONE
            recoverable→ next vantage
            structural → FAILED (fail-loud)
       → DONE / FAILED

NO FALLBACKS: failures are explicit; we never emit a fabricated boundary.
"""

from __future__ import annotations

import json
import math
import os
import time
from enum import Enum
from pathlib import Path

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None

try:
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
except ImportError:  # graceful: survey still completes, just without a saved map
    SaveMap = None
    SerializePoseGraph = None

try:
    from tennis_robot.court_extraction import (
        extract_court_knowledge_model, CourtExtractionError, CourtSpec,
    )
    from tennis_robot.court_coverage import vantage_points, is_recoverable_failure
    from tennis_robot.court_format import (
        CameraCornerEvidence,
        add_distinct_corner,
        camera_corner_to_map,
        estimate_court_format,
    )
except ModuleNotFoundError:  # running from source tree
    from court_extraction import (
        extract_court_knowledge_model, CourtExtractionError, CourtSpec,
    )
    from court_coverage import vantage_points, is_recoverable_failure
    from court_format import (
        CameraCornerEvidence,
        add_distinct_corner,
        camera_corner_to_map,
        estimate_court_format,
    )


PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"))
RUNTIME_DIR = Path(
    os.getenv("ROBOT_STATUS_FILE", str(PROJECT_ROOT / "runtime" / "robot_status.json"))
).parent
COURT_BOUNDARY_FILE = RUNTIME_DIR / "court_boundary.json"
COURT_SURVEY_LIVE_FILE = RUNTIME_DIR / "court_survey_live.json"

NET_STANDOFF_M = float(os.getenv("COURT_SURVEY_NET_STANDOFF_M", "2.00"))
FIND_NET_SPEED = float(os.getenv("COURT_SURVEY_FIND_NET_SPEED_M_S", "0.45"))
DWELL_S = float(os.getenv("COURT_SURVEY_VANTAGE_DWELL_S", "2.0"))
GOAL_TIMEOUT_S = float(os.getenv("COURT_SURVEY_GOAL_TIMEOUT_S", "90.0"))
FIND_NET_TIMEOUT_S = float(os.getenv("COURT_SURVEY_FIND_NET_TIMEOUT_S", "60.0"))
NET_MIN_CONF = float(os.getenv("COURT_SURVEY_LANDMARK_MIN_CONF", "0.25"))
VISION_MAX_AGE_S = float(os.getenv("COURT_SURVEY_VISION_MAX_AGE_S", "1.0"))
CAMERA_X_M = float(os.getenv("PERCEPTION_CAMERA_X_M", "0.535"))
FORMAT_MIN_CORNER_CONF = float(
    os.getenv("COURT_SURVEY_FORMAT_MIN_CORNER_CONF", "0.35")
)

# Live occupancy map (identical to the proven v1 accumulation).
MAP_VOXEL_M = float(os.getenv("COURT_SURVEY_MAP_VOXEL_M", "0.10"))
MAP_MAX_VOXELS = int(os.getenv("COURT_SURVEY_MAP_MAX_VOXELS", "8000"))
MAP_SAMPLE_MAX = int(os.getenv("COURT_SURVEY_MAP_SAMPLE_MAX", "1500"))
MAP_MIN_RANGE_M = float(os.getenv("COURT_SURVEY_MAP_MIN_RANGE_M", "0.20"))
MAP_MAX_RANGE_M = float(os.getenv("COURT_SURVEY_MAP_MAX_RANGE_M", "15.0"))
FRONT_HALF_DEG = 20.0
# Deterministic drive-to-waypoint (closed-loop on SLAM pose; Nav2 too flaky here).
WAYPOINT_TOL_M = float(os.getenv("COURT_SURVEY_WAYPOINT_TOL_M", "0.6"))
WAYPOINT_TIMEOUT_S = float(os.getenv("COURT_SURVEY_WAYPOINT_TIMEOUT_S", "45.0"))
DRIVE_SPEED = float(os.getenv("COURT_SURVEY_DRIVE_SPEED_M_S", "0.85"))
TURN_SPEED = float(os.getenv("COURT_SURVEY_TURN_SPEED_RAD_S", "1.1"))
OBSTACLE_STOP_M = float(os.getenv("COURT_SURVEY_OBSTACLE_STOP_M", "0.5"))
# Stop this far from a fence when approaching it head-on. Accounts for the robot
# footprint (LiDAR ~0.4 m behind the front) so we map the fence densely WITHOUT
# ramming it — the robot body cannot reach the fence, only the LiDAR sees it.
FENCE_APPROACH_M = float(os.getenv("COURT_SURVEY_FENCE_APPROACH_M", "1.4"))

# Saved SLAM map artifacts (for Nav2 reuse in the collection phase).
MAPS_DIR = RUNTIME_DIR / "maps"
MAP_SAVE_TIMEOUT_S = float(os.getenv("COURT_SURVEY_MAP_SAVE_TIMEOUT_S", "12.0"))


def _yaw_from_quaternion(q) -> float:
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


class V2State(Enum):
    INIT = "init"
    FIND_NET = "find_net"
    COVERAGE = "coverage"
    SAVING_MAP = "saving_map"
    DONE = "done"
    FAILED = "failed"


class CourtSurveyV2Node(Node):
    def __init__(self) -> None:
        super().__init__("court_survey_mission_node")
        self._state = V2State.INIT
        self._entered_at: float | None = None
        self._survey_started_at: float | None = None
        self._phase_started_at: float | None = None
        self._phase_durations_s: dict[str, float] = {}
        self._failure_reason: str | None = None

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._pose_valid = False
        self._scan_frame_id = ""
        self._front_range_m = math.inf
        self._scan_angle_min = -math.pi
        self._scan_angle_inc = 2.0 * math.pi / 360.0
        self._last_scan_points: list[tuple[float, float, float]] = []
        self._vision: dict = {}
        self._vision_received_at: float | None = None
        self._camera_corner_evidence: list[CameraCornerEvidence] = []
        self._events: list[dict] = []
        self._last_event = "initializing"
        self._cmd_linear_m_s = 0.0
        self._cmd_angular_rad_s = 0.0

        # occupancy map
        self._map_voxels: dict[tuple[int, int], tuple[float, float]] = {}
        self._map_error: str | None = "no /scan received yet"

        # court frame / coverage
        self._locked_net: dict | None = None
        self._vantages: list[dict] = []
        self._survey_start_pose: dict | None = None
        self._home: dict | None = None
        self._vantage_i = 0
        self._goal_active = False
        self._goal_started_at = 0.0
        self._dwell_until = 0.0
        self._wp_started = 0.0
        self._spec = CourtSpec()
        # Measurement is locked once (so we know it's not a failure), but we keep
        # driving the full path to complete a clean, loop-closed map. The FINAL
        # result is the last successful extraction on the most complete map.
        self._measured = False
        self._last_model: dict | None = None
        self._final_live_written = False  # write one terminal live update on DONE/FAILED
        # SLAM map serialization (best-effort, for Nav2 reuse in the collection phase)
        self._save_clients: dict = {}
        self._save_futures: dict = {}
        self._map_base: str | None = None
        self._saving_started = 0.0
        if SaveMap is not None and SerializePoseGraph is not None:
            self._save_clients["serialize"] = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
            self._save_clients["occupancy"] = self.create_client(SaveMap, "/slam_toolbox/save_map")

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav = BasicNavigator() if BasicNavigator is not None else None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos)
        self.create_subscription(String, "/survey/vision", self._vision_cb, 10)
        self.create_timer(0.2, self._step)
        self._record_event("survey_node_started", "Waiting for pose and sensors")
        self.get_logger().info("court_survey_v2 node started (LiDAR occupancy mapping)")

    # ── sensors ──────────────────────────────────────────────────────────────
    def _vision_cb(self, msg: String) -> None:
        try:
            self._vision = json.loads(msg.data)
            self._vision_received_at = self._now()
            self._capture_camera_corner(self._vision)
        except (json.JSONDecodeError, TypeError):
            self._vision = {}
            self._vision_received_at = None

    def _capture_camera_corner(self, vision: dict) -> None:
        if not self._pose_valid or str(vision.get("junction_type") or "") != "L":
            return
        confidence = float(vision.get("junction_confidence") or 0.0)
        distance = vision.get("junction_distance_m")
        bearing = vision.get("junction_bearing_rad")
        if confidence < FORMAT_MIN_CORNER_CONF:
            return
        projected = camera_corner_to_map(
            robot_x_m=self._robot_x,
            robot_y_m=self._robot_y,
            robot_yaw_rad=self._robot_yaw,
            bearing_rad=bearing,
            distance_m=distance,
            camera_x_m=CAMERA_X_M,
        )
        if projected is None:
            return
        add_distinct_corner(
            self._camera_corner_evidence,
            CameraCornerEvidence(
                projected.map_x_m,
                projected.map_y_m,
                confidence,
            ),
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_frame_id = msg.header.frame_id
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_inc = float(msg.angle_increment)
        ranges = list(msg.ranges)
        # front range over ±FRONT_HALF_DEG
        half = math.radians(FRONT_HALF_DEG)
        front: list[float] = []
        pts: list[tuple[float, float, float]] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = self._scan_angle_min + i * self._scan_angle_inc
            an = (a + math.pi) % (2 * math.pi) - math.pi
            if abs(an) <= half:
                front.append(r)
            pts.append((r * math.cos(a), r * math.sin(a), r))
        front.sort()
        self._front_range_m = front[int(len(front) * 0.3)] if front else math.inf
        self._last_scan_points = pts

    def _update_pose_from_tf(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05))
            self._robot_x = float(t.transform.translation.x)
            self._robot_y = float(t.transform.translation.y)
            self._robot_yaw = _yaw_from_quaternion(t.transform.rotation)
            self._pose_valid = True
        except (LookupException, ExtrapolationException, ConnectivityException):
            pass

    # ── occupancy map (map frame) ────────────────────────────────────────────
    def _accumulate_map(self) -> None:
        if not self._scan_frame_id:
            self._map_error = "no /scan received yet"
            return
        try:
            t = self._tf_buffer.lookup_transform(
                "map", self._scan_frame_id, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05))
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self._map_error = f"no TF map->{self._scan_frame_id}: {type(exc).__name__}"
            return
        self._map_error = None
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        tyaw = _yaw_from_quaternion(t.transform.rotation)
        cy, sy = math.cos(tyaw), math.sin(tyaw)
        v = self._map_voxels
        full = len(v) >= MAP_MAX_VOXELS
        for sx, syl, r in self._last_scan_points:
            if r < MAP_MIN_RANGE_M or r > MAP_MAX_RANGE_M:
                continue
            mx = tx + sx * cy - syl * sy
            my = ty + sx * sy + syl * cy
            key = (int(math.floor(mx / MAP_VOXEL_M)), int(math.floor(my / MAP_VOXEL_M)))
            if key in v or full:
                continue
            v[key] = (mx, my)
            if len(v) >= MAP_MAX_VOXELS:
                full = True

    def _map_points(self) -> list[dict]:
        pts = list(self._map_voxels.values())
        if len(pts) > MAP_SAMPLE_MAX:
            stride = len(pts) / MAP_SAMPLE_MAX
            pts = [pts[int(k * stride)] for k in range(MAP_SAMPLE_MAX)]
        return [{"x_m": round(px, 3), "y_m": round(py, 3)} for px, py in pts]

    def _write_live(self) -> None:
        vision_age_s = (
            None
            if self._vision_received_at is None
            else max(0.0, self._now() - self._vision_received_at)
        )
        payload = {
            "updated_at": time.time(),
            "running": self._state not in (V2State.DONE, V2State.FAILED),
            "state": self._state.value,
            "result": ("OK" if self._state == V2State.DONE
                       else "FAILED" if self._state == V2State.FAILED else None),
            "failure_reason": self._failure_reason,
            "coverage": {"i": self._vantage_i, "n": len(self._vantages)},
            "timing": self._timing_snapshot(),
            "last_event": self._last_event,
            "events": self._events[-24:],
            "error": self._map_error, "sensor_frame": self._scan_frame_id or None,
            "front_range_m": None if math.isinf(self._front_range_m) else round(self._front_range_m, 3),
            "robot": {"x_m": round(self._robot_x, 3), "y_m": round(self._robot_y, 3),
                      "yaw_rad": round(self._robot_yaw, 4)},
            "map_points": self._map_points(), "map_point_count": len(self._map_voxels),
            "net": self._locked_net, "survey_start_pose": self._survey_start_pose,
            "navigation_points": [],
            "motion": {
                "commanded_linear_m_s": round(self._cmd_linear_m_s, 3),
                "commanded_angular_rad_s": round(self._cmd_angular_rad_s, 3),
            },
            "health": {
                "pose": "healthy" if self._pose_valid else "waiting",
                "lidar": "healthy" if self._scan_frame_id and not self._map_error else "waiting",
                "vision": (
                    "healthy"
                    if vision_age_s is not None and vision_age_s <= VISION_MAX_AGE_S
                    else "stale"
                ),
                "vision_age_s": None if vision_age_s is None else round(vision_age_s, 2),
                "map": "healthy" if self._map_error is None else "waiting",
            },
            "court_format_estimate": self._court_format_estimate(),
        }
        try:
            tmp = COURT_SURVEY_LIVE_FILE.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(payload))
            tmp.replace(COURT_SURVEY_LIVE_FILE)
        except OSError:
            pass

    # ── helpers ──────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _record_event(
        self, code: str, detail: str = "", level: str = "info",
    ) -> None:
        self._last_event = code
        elapsed_s = (
            0.0
            if self._survey_started_at is None
            else max(0.0, self._now() - self._survey_started_at)
        )
        event = {
            "code": code,
            "detail": detail,
            "level": level,
            "elapsed_s": round(elapsed_s, 1),
            "state": self._state.value,
        }
        if self._events and self._events[-1] == event:
            return
        self._events.append(event)
        if len(self._events) > 64:
            del self._events[:-64]

    def _enter(self, state: V2State) -> None:
        now = self._now()
        previous = self._state
        if self._phase_started_at is not None:
            elapsed = max(0.0, now - self._phase_started_at)
            key = self._state.value
            self._phase_durations_s[key] = self._phase_durations_s.get(key, 0.0) + elapsed
        self.get_logger().info(f"survey: {self._state.value} -> {state.value}")
        self._state = state
        self._entered_at = now
        self._phase_started_at = now
        self._record_event(
            f"phase_{state.value}",
            f"{previous.value} → {state.value}",
            "error" if state == V2State.FAILED else "info",
        )

    def _timing_snapshot(self) -> dict:
        now = self._now()
        durations = dict(self._phase_durations_s)
        current_phase_s = 0.0
        if self._phase_started_at is not None:
            current_phase_s = max(0.0, now - self._phase_started_at)
            durations[self._state.value] = durations.get(self._state.value, 0.0) + current_phase_s
        elapsed_s = 0.0 if self._survey_started_at is None else max(0.0, now - self._survey_started_at)
        return {
            "elapsed_s": round(elapsed_s, 1),
            "current_phase": self._state.value,
            "current_phase_s": round(current_phase_s, 1),
            "phase_durations_s": {k: round(v, 1) for k, v in durations.items()},
        }

    def _timed_out(self, limit_s: float) -> bool:
        if self._entered_at is None:
            self._entered_at = self._now()
            return False
        return self._now() - self._entered_at > limit_s

    def _drive(self, lin: float, ang: float = 0.0) -> None:
        self._cmd_linear_m_s = float(lin)
        self._cmd_angular_rad_s = float(ang)
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self._cmd_pub.publish(tw)

    def _stop(self) -> None:
        self._drive(0.0, 0.0)

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self.get_logger().error(f"survey FAILED: {reason}")
        self._stop()
        self._enter(V2State.FAILED)
        self._record_event("survey_failed", reason, "error")
        self._write_result(status="FAILED")
        self._write_live()
        self._final_live_written = True

    # ── net locking → court frame ────────────────────────────────────────────
    def _try_lock_net(self) -> bool:
        v = self._vision or {}
        if (
            self._vision_received_at is None
            or self._now() - self._vision_received_at > VISION_MAX_AGE_S
        ):
            return False
        cls = str(v.get("obstacle_class") or "")
        source = str(v.get("obstacle_source") or "")
        conf = float(v.get("obstacle_confidence") or 0.0)
        net_seen = cls.lower() == "net"
        d = self._front_range_m
        # accept lock when a net is classified ahead and we are at/near standoff,
        # using the accurate front LiDAR range as the net distance.
        if (
            net_seen
            and source == "neural_court_scene"
            and conf >= NET_MIN_CONF
            and math.isfinite(d)
            and d <= NET_STANDOFF_M + 1.0
        ):
            hx, hy = math.cos(self._robot_yaw), math.sin(self._robot_yaw)
            self._locked_net = {
                "map_x_m": round(self._robot_x + hx * d, 3),
                "map_y_m": round(self._robot_y + hy * d, 3),
                "robot_x_m": round(self._robot_x, 3),
                "robot_y_m": round(self._robot_y, 3),
                "robot_yaw_rad": round(self._robot_yaw, 4),
                "range_m": round(d, 3),
                "confidence": conf,
                "source": "lidar+neural_court_scene",
            }
            self.get_logger().info(
                f"net locked at map=({self._locked_net['map_x_m']},"
                f"{self._locked_net['map_y_m']}) range={d:.2f}m")
            self._record_event(
                "net_locked",
                f"{d:.2f} m · confidence {conf:.2f}",
            )
            return True
        return False

    # ── Nav2 ─────────────────────────────────────────────────────────────────
    def _send_goal(self, pose: dict) -> None:
        if self._nav is None:
            self._fail("nav2_unavailable")
            return
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(pose["x_m"])
        g.pose.position.y = float(pose["y_m"])
        yaw = float(pose.get("yaw_rad", 0.0))
        g.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.orientation.w = math.cos(yaw / 2.0)
        self._nav.goToPose(g)
        self._goal_active = True
        self._goal_started_at = self._now()
        self.get_logger().info(f"nav goal -> ({pose['x_m']:.2f},{pose['y_m']:.2f}) court_x={pose.get('court_x')}")

    # ── deterministic drive-to-waypoint ──────────────────────────────────────
    def _drive_to_waypoint(self, target: dict) -> bool:
        """Closed-loop drive toward a map-frame target using the SLAM pose.
        Deterministic (no Nav2 planning). The coverage path never crosses the
        net, so straight-line driving is safe. Returns True when reached."""
        dx = float(target["x_m"]) - self._robot_x
        dy = float(target["y_m"]) - self._robot_y
        dist = math.hypot(dx, dy)
        if dist < WAYPOINT_TOL_M:
            self._stop()
            return True
        desired = math.atan2(dy, dx)
        yaw_err = ((desired - self._robot_yaw + math.pi) % (2 * math.pi)) - math.pi
        # Reached a fence ahead (dense mapping) while still far from the waypoint:
        # the waypoint is set beyond the fence on purpose; stop at the fence.
        # Only for stop_short waypoints (deep fence points) -- NOT for gap-cross
        # points, or the LiDAR-visible net would be mistaken for the fence and
        # the robot would never cross to the far half.
        if target.get("stop_short", True) and dist > 2.0 and self._front_range_m < FENCE_APPROACH_M and abs(yaw_err) < math.radians(30):
            self._stop()
            return True
        tw = Twist()
        if abs(yaw_err) > math.radians(25):
            tw.angular.z = max(-TURN_SPEED, min(TURN_SPEED, yaw_err * 1.5))
        elif self._front_range_m < OBSTACLE_STOP_M:
            # obstacle right ahead while aligned: rotate away rather than ram it
            tw.angular.z = TURN_SPEED if yaw_err >= 0 else -TURN_SPEED
        else:
            tw.linear.x = DRIVE_SPEED
            tw.angular.z = max(-0.4, min(0.4, yaw_err))
        self._cmd_pub.publish(tw)
        self._cmd_linear_m_s = float(tw.linear.x)
        self._cmd_angular_rad_s = float(tw.angular.z)
        return False

    def _court_format_estimate(self) -> dict:
        if self._locked_net is None:
            return {
                "label": "unknown",
                "confidence": 0.0,
                "source": "camera_line_junctions",
                "evidence_count": 0,
                "scores": {"singles": 0.0, "doubles": 0.0},
                "affects_navigation": False,
            }
        try:
            frame = _build_frame(self._locked_net)
        except CourtExtractionError:
            return {
                "label": "unknown",
                "confidence": 0.0,
                "source": "camera_line_junctions",
                "evidence_count": 0,
                "scores": {"singles": 0.0, "doubles": 0.0},
                "affects_navigation": False,
            }
        return estimate_court_format(
            self._camera_corner_evidence,
            net_center_x_m=frame.cx,
            net_center_y_m=frame.cy,
            length_axis_x=frame.ux,
            length_axis_y=frame.uy,
            width_axis_x=frame.vx,
            width_axis_y=frame.vy,
            court_half_length_m=self._spec.half_length_m,
            singles_half_width_m=self._spec.width_singles_m / 2.0,
            doubles_half_width_m=self._spec.width_doubles_m / 2.0,
        )

    # ── extraction ───────────────────────────────────────────────────────────
    def _try_measure(self) -> None:
        """Extract continuously, keeping the LATEST successful model (re-fit on the
        ever-more-complete, loop-closed map). Locks 'measured' on first success.

        Fail-loud ONLY on a structural failure BEFORE any success — once we have a
        valid measurement we never throw it away on a transient later failure
        (e.g. the map momentarily distorting mid loop-closure)."""
        try:
            model = extract_court_knowledge_model(self._map_points(), self._locked_net, self._spec)
        except CourtExtractionError as e:
            if not self._measured and not is_recoverable_failure(e.reason):
                self._fail(e.reason)  # genuine structural failure, never measured → stop
            return
        self._last_model = model
        model["court"]["format_estimate"] = self._court_format_estimate()
        if not self._measured:
            self._measured = True
            self._record_event(
                "court_model_locked",
                f"{len(self._map_voxels)} map points",
            )
            self.get_logger().info(
                f"survey measurable (model locked): doubles={model['court']['is_doubles']} "
                f"dist={model['distances_to_fence_m']} obstacles={len(model['obstacles'])}; "
                f"continuing coverage to complete a clean map")

    def _write_result(self, status: str, model: dict | None = None) -> None:
        if model is None:
            model = {
                "schema": "court_knowledge_model/v2", "status": status,
                "failure_reason": self._failure_reason, "frame": "map",
                "net": self._locked_net, "occupancy": {"point_count": len(self._map_voxels)},
            }
        model["timing"] = self._timing_snapshot()
        model["surveyed_at"] = time.time()
        try:
            tmp = COURT_BOUNDARY_FILE.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(model, indent=2))
            tmp.replace(COURT_BOUNDARY_FILE)
        except OSError as exc:
            self.get_logger().warning(f"could not write court_boundary.json: {exc}")

    # ── SLAM map serialization (best-effort, for Nav2 reuse) ──────────────────
    def _begin_map_save(self) -> None:
        """Serialize the slam_toolbox map (+ occupancy grid) so the collection
        phase can reload it for Nav2. NEVER blocks the survey: missing services or
        a timeout still finishes with the (valid) measurement."""
        if SaveMap is None or not self._save_clients:
            self._finalize_done(map_error="slam_save_services_unavailable")
            return
        try:
            MAPS_DIR.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass
        base = str(MAPS_DIR / f"court_{int(time.time())}")
        self._map_base = base
        started = False
        ser = self._save_clients.get("serialize")
        occ = self._save_clients.get("occupancy")
        try:
            if ser is not None and ser.service_is_ready():
                req = SerializePoseGraph.Request(); req.filename = base
                self._save_futures["serialize"] = ser.call_async(req); started = True
            if occ is not None and occ.service_is_ready():
                req2 = SaveMap.Request(); req2.name = String(data=base)
                self._save_futures["occupancy"] = occ.call_async(req2); started = True
        except Exception as exc:  # best-effort: never break the survey
            self.get_logger().warning(f"map save request error: {exc}")
        if not started:
            self.get_logger().warning("slam_toolbox save services not ready; finishing without map artifact")
            self._finalize_done(map_error="slam_save_services_not_ready")
            return
        self.get_logger().info(f"saving slam map -> {base}.*")
        self._record_event("map_save_started", base)
        self._saving_started = self._now()
        self._enter(V2State.SAVING_MAP)

    def _build_map_artifact(self, map_error: str | None) -> dict:
        if map_error:
            return {"status": "error", "reason": map_error}
        base = self._map_base
        files = {}
        if base:
            for ext in ("posegraph", "data", "yaml", "pgm"):
                if Path(f"{base}.{ext}").exists():
                    files[ext] = f"{base}.{ext}"
        net = (self._last_model or {}).get("net") or {}
        return {
            "status": "saved" if files else "pending",
            "basename": base,
            "files": files,
            "survey_start_pose": self._survey_start_pose,
            # the court frame ties the Court Knowledge Model measurements to the
            # saved occupancy grid -> Nav2 + collection share one coordinate frame.
            "court_frame": {
                "center": net.get("center"),
                "axis_length": net.get("axis_length"),
                "axis_width": net.get("axis_width"),
            },
            "saved_at": time.time(),
        }

    def _finalize_done(self, map_error: str | None = None) -> None:
        model = self._last_model
        if model is not None:
            model["court"]["format_estimate"] = self._court_format_estimate()
            model["map_artifact"] = self._build_map_artifact(map_error)
            model["completed"] = True
            model["survey_start_pose"] = self._survey_start_pose
            model["notice"] = "Court survey complete; boundaries saved, robot returned to survey start pose."
            self._write_result(status="OK", model=model)
            self.get_logger().info(
                "===== SURVEY COMPLETE ===== "
                f"dist={model['distances_to_fence_m']} "
                f"obstacles={len(model['obstacles'])} map={model['map_artifact'].get('status')} "
                "(robot returned to start)")
        self._enter(V2State.DONE)
        if model is not None:
            self._record_event(
                "survey_completed",
                f"{len(self._map_voxels)} points · map {model['map_artifact'].get('status')}",
            )
        self._write_live()
        self._final_live_written = True

    # ── main step ────────────────────────────────────────────────────────────
    def _step(self) -> None:
        if self._state in (V2State.DONE, V2State.FAILED):
            if not self._final_live_written:
                self._write_live()  # persist running=false + result so the UI sees completion
                self._final_live_written = True
            return
        self._update_pose_from_tf()
        self._accumulate_map()
        self._write_live()

        if self._state == V2State.INIT:
            if not self._pose_valid:
                self._stop()
                return
            if self._survey_start_pose is None:
                self._survey_started_at = self._now()
                self._record_event("survey_started", "Start pose captured")
                self._survey_start_pose = {
                    "x_m": round(self._robot_x, 3), "y_m": round(self._robot_y, 3),
                    "yaw_rad": round(self._robot_yaw, 4),
                    "label": "survey_start_pose", "role": "return_anchor",
                    "court_x": None, "court_y": None, "stop_short": False, "is_home": True,
                }
            self._enter(V2State.FIND_NET)
            return

        if self._state == V2State.FIND_NET:
            if self._timed_out(FIND_NET_TIMEOUT_S):
                self._fail("net_not_observed: find_net timeout")
                return
            if self._try_lock_net():
                self._stop()
                self._vantages = vantage_points(_build_frame(self._locked_net), self._spec)
                # Return-home: after the coverage/return path, drive back to where
                # the survey began, not merely to the net-lock standoff pose.
                self._home = self._survey_start_pose or {
                    "x_m": round(self._robot_x, 3), "y_m": round(self._robot_y, 3),
                    "yaw_rad": round(self._robot_yaw, 4),
                    "label": "survey_start_pose", "role": "return_anchor",
                    "court_x": None, "court_y": None, "stop_short": False, "is_home": True,
                }
                self._vantages.append(self._home)
                self._vantage_i = 0
                self._goal_active = False
                self._enter(V2State.COVERAGE)
                return
            # drive forward toward the net, stop short of collision
            self._drive(0.0 if self._front_range_m <= NET_STANDOFF_M else FIND_NET_SPEED)
            return

        if self._state == V2State.SAVING_MAP:
            done = all(fu.done() for fu in self._save_futures.values()) if self._save_futures else True
            if done or (self._now() - self._saving_started) > MAP_SAVE_TIMEOUT_S:
                self._finalize_done()
            return

        if self._state == V2State.COVERAGE:
            # Extract continuously as the map fills (locks the measurement once,
            # keeps refitting on the completing map; fail-loud only if a structural
            # failure happens before we ever measured).
            self._try_measure()
            if self._state in (V2State.DONE, V2State.FAILED):
                return
            if self._vantage_i >= len(self._vantages):
                # Full path (incl. return pass) done. Finalise with the last good
                # model on the most complete, loop-closed map — or fail-loud if we
                # never managed a valid measurement.
                if self._last_model is not None:
                    self._stop()
                    self._begin_map_save()  # -> SAVING_MAP (serialize) -> DONE
                else:
                    self._fail(self._failure_reason or "coverage_incomplete: all vantage points visited")
                return
            target = self._vantages[self._vantage_i]
            if self._wp_started <= 0.0:
                self._wp_started = self._now()
            if self._drive_to_waypoint(target):
                # reached: dwell to accumulate scans, then advance
                if self._dwell_until <= 0.0:
                    self._dwell_until = self._now() + DWELL_S
                    return
                if self._now() < self._dwell_until:
                    return
                self._dwell_until = 0.0
                self._wp_started = 0.0
                self._record_event(
                    "coverage_waypoint_reached",
                    f"{self._vantage_i + 1}/{len(self._vantages)} · {target.get('label', 'waypoint')}",
                )
                self._vantage_i += 1
            elif self._now() - self._wp_started > WAYPOINT_TIMEOUT_S:
                self.get_logger().warning(
                    f"waypoint {self._vantage_i} (court_x={target.get('court_x')}) timed out; advancing")
                self._stop()
                self._record_event(
                    "coverage_waypoint_timeout",
                    f"{self._vantage_i + 1}/{len(self._vantages)}",
                    "warning",
                )
                self._wp_started = 0.0
                self._vantage_i += 1
            return


def _build_frame(locked_net: dict):
    try:
        from tennis_robot.court_extraction import build_court_frame
    except ModuleNotFoundError:
        from court_extraction import build_court_frame
    return build_court_frame(locked_net)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CourtSurveyV2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Launch teardown (the panel SIGINTs us on going idle) — clean exit, the
        # measurement is already written. Swallow so we don't print a scary trace.
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
