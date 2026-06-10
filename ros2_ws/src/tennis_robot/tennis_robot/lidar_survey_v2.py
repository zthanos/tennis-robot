"""Open-loop perimeter survey — heading-only turns, no vision dependency.

Reuses LidarSurveyState, LidarSurveyConfig, LidarSurveyCommand from
lidar_survey.py for drop-in compatibility with controller_node.py.

Key differences from V1 (Ros2LidarCourtSurvey):
- Turns complete on heading + settle-ticks only (no parallel-boundary check).
- Corner stops on front LiDAR range alone (no side-sample stability guard).
- No active-waypoint detour: each state transitions directly on trigger.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

try:
    from tennis_robot.motion import TurnTracker
except ModuleNotFoundError:
    from motion import TurnTracker
try:
    from tennis_robot.survey import SurveyVision
except ModuleNotFoundError:
    from survey import SurveyVision
try:
    from tennis_robot.lidar_survey import (
        BaseCommand,
        LidarSurveyCommand,
        LidarSurveyConfig,
        LidarSurveyState,
    )
except ModuleNotFoundError:
    from lidar_survey import BaseCommand, LidarSurveyCommand, LidarSurveyConfig, LidarSurveyState


PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"))


class LidarCourtSurveyV2:
    """Perimeter survey requiring only LiDAR + odometry.

    Public interface is identical to Ros2LidarCourtSurvey; swap in
    controller_node.py by aliasing this class as Ros2LidarCourtSurvey.
    """

    def __init__(self, config: LidarSurveyConfig | None = None) -> None:
        self.config = config or LidarSurveyConfig()
        self.state = LidarSurveyState.NET_STANDOFF
        self.sample_count = 0

        self._net_approach_yaw: float | None = None
        self._turn_180_target: float | None = None
        self._baseline_heading: float | None = None
        self._sideline_heading: float | None = None
        self._long_side_heading: float | None = None
        self._far_short_heading: float | None = None
        self._return_heading: float | None = None

        self._turn_tracker = TurnTracker()

        self._last_scan_ranges: list[float] = []
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float = 2.0 * math.pi / 360
        self._last_front_range_m: float = math.inf

        self._last_pose: tuple[float, float] | None = None
        self._distance_traveled_m: float = 0.0
        self._state_entry_distance_m: float = 0.0
        self._state_elapsed_s: float = 0.0
        self._started_at: float | None = None
        self._last_event: str = "none"
        self._failure_reason: str | None = None
        self._court_bounds: dict | None = None

        self._navigation_points: list[dict] = []

    @classmethod
    def from_env(cls) -> "LidarCourtSurveyV2":
        return cls(LidarSurveyConfig.from_env())

    def reset(self) -> None:
        self.__init__(self.config)

    @property
    def court_bounds(self) -> dict | None:
        return self._court_bounds

    # ── public update ──────────────────────────────────────────────────────────

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,
        vision: SurveyVision | None,
        lidar_angle_min: float = -math.pi,
        lidar_angle_increment: float | None = None,
    ) -> LidarSurveyCommand:
        if self._started_at is None:
            self._started_at = time.time()
        if self.state == LidarSurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0))

        self._state_elapsed_s += max(0.0, dt_s)
        self._update_distance(x_m, y_m)
        self.sample_count += 1

        if lidar_ranges:
            self._last_scan_ranges = lidar_ranges
            self._lidar_angle_min = lidar_angle_min
            self._lidar_angle_increment = (
                lidar_angle_increment
                if lidar_angle_increment is not None
                else 2.0 * math.pi / max(1, len(lidar_ranges))
            )

        self._last_front_range_m = self._front_range()

        command = self._step(x_m, y_m, yaw_rad)
        return self._cmd(self._apply_safety(command))

    # ── telemetry ──────────────────────────────────────────────────────────────

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        return {
            "state": self.state.value,
            "navigation_source": "v2_open_loop_perimeter",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "sample_count": self.sample_count,
            "front_lidar_range_m": (
                None if math.isinf(self._last_front_range_m)
                else round(self._last_front_range_m, 3)
            ),
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
            "state_elapsed_s": round(self._state_elapsed_s, 1),
            "turn_progress_deg": round(math.degrees(self._turn_tracker.progress_rad), 1),
            "survey_navigation_points": list(self._navigation_points),
            "survey_route": [
                {"x_m": p["x_m"], "y_m": p["y_m"], "label": p["label"]}
                for p in self._navigation_points
            ],
            "survey_navigation_point_count": len(self._navigation_points),
        }

    # ── state machine ──────────────────────────────────────────────────────────

    def _step(self, x_m: float, y_m: float, yaw_rad: float) -> BaseCommand:
        # ① drive to net
        if self.state == LidarSurveyState.NET_STANDOFF:
            if self._net_approach_yaw is None:
                self._net_approach_yaw = yaw_rad
            if not math.isinf(self._last_front_range_m) and self._last_front_range_m <= self.config.net_standoff_m:
                self._record("net_standoff", x_m, y_m)
                self._turn_180_target = yaw_rad + math.pi
                self._baseline_heading = (self._net_approach_yaw or yaw_rad) + math.pi
                self._enter(LidarSurveyState.TURN_180, "net_detected")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(self.config.drive_speed_m_s, 0.0)

        # turn 180° toward baseline
        if self.state == LidarSurveyState.TURN_180:
            if self._state_elapsed_s >= self.config.turn_timeout_s:
                self._finalize("turn_180_timeout")
                return BaseCommand(0.0, 0.0)
            if self._turn_180_target is None:
                self._finalize("turn_180_no_target")
                return BaseCommand(0.0, 0.0)
            if self._heading_reached(yaw_rad, self._turn_180_target, math.pi):
                self._enter(LidarSurveyState.BASELINE_APPROACH, "turn_180_complete")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._turn_180_target, yaw_rad)
            # Near antipode: sign is ambiguous — lock to tracker direction
            if abs(err) > math.pi - math.radians(20.0) and self._turn_tracker.direction != 0.0:
                err = abs(err) * self._turn_tracker.direction
            return BaseCommand(0.0, self._proportional_turn(err))

        # ② drive to near-baseline fence
        if self.state == LidarSurveyState.BASELINE_APPROACH:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize("baseline_timeout")
                return BaseCommand(0.0, 0.0)
            if not math.isinf(self._last_front_range_m) and self._last_front_range_m <= self.config.short_side_stop_range_m:
                self._record("near_baseline_fence", x_m, y_m)
                self._sideline_heading = (self._baseline_heading or yaw_rad) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_SIDELINE, "baseline_reached")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(
                self.config.drive_speed_m_s,
                self._heading_correction(yaw_rad, self._baseline_heading),
            )

        # turn 90° left toward sideline
        if self.state == LidarSurveyState.TURN_TO_SIDELINE:
            return self._turn_step(
                yaw_rad, self._sideline_heading,
                LidarSurveyState.DRIVE_SIDELINE, "sideline_turn_complete",
            )

        # ③ drive short side (near-baseline fence leg)
        if self.state == LidarSurveyState.DRIVE_SIDELINE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize("sideline_timeout")
                return BaseCommand(0.0, 0.0)
            if self._min_drive_cleared() and not math.isinf(self._last_front_range_m) and self._last_front_range_m <= self.config.short_side_stop_range_m:
                self._record("near_left_fence_corner", x_m, y_m)
                self._long_side_heading = (self._sideline_heading or yaw_rad) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_LONG_SIDE, "left_corner_reached")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(
                self.config.drive_speed_m_s,
                self._heading_correction(yaw_rad, self._sideline_heading),
            )

        # turn 90° left toward far baseline
        if self.state == LidarSurveyState.TURN_TO_LONG_SIDE:
            return self._turn_step(
                yaw_rad, self._long_side_heading,
                LidarSurveyState.DRIVE_LONG_SIDE, "long_side_turn_complete",
            )

        # ④ drive long side — 80th-pct range sees through net
        if self.state == LidarSurveyState.DRIVE_LONG_SIDE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize("long_side_timeout")
                return BaseCommand(0.0, 0.0)
            front = self._through_front_range()
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.long_side_stop_range_m:
                self._record("far_left_fence_corner", x_m, y_m)
                self._far_short_heading = (self._long_side_heading or yaw_rad) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_FAR_SHORT, "far_baseline_reached")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(
                self.config.drive_speed_m_s,
                self._heading_correction(yaw_rad, self._long_side_heading),
            )

        # turn 90° left toward far sideline
        if self.state == LidarSurveyState.TURN_TO_FAR_SHORT:
            return self._turn_step(
                yaw_rad, self._far_short_heading,
                LidarSurveyState.DRIVE_FAR_SHORT, "far_short_turn_complete",
            )

        # ⑤ drive far short side
        if self.state == LidarSurveyState.DRIVE_FAR_SHORT:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize("far_short_timeout")
                return BaseCommand(0.0, 0.0)
            if self._min_drive_cleared() and not math.isinf(self._last_front_range_m) and self._last_front_range_m <= self.config.short_side_stop_range_m:
                self._record("far_right_fence_corner", x_m, y_m)
                self._return_heading = (self._far_short_heading or yaw_rad) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_RETURN, "far_right_corner_reached")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(
                self.config.drive_speed_m_s,
                self._heading_correction(yaw_rad, self._far_short_heading),
            )

        # turn 90° left toward return long side
        if self.state == LidarSurveyState.TURN_TO_RETURN:
            return self._turn_step(
                yaw_rad, self._return_heading,
                LidarSurveyState.DRIVE_RETURN, "return_turn_complete",
            )

        # ⑥ drive return long side — 80th-pct range sees through net
        if self.state == LidarSurveyState.DRIVE_RETURN:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize("return_timeout")
                return BaseCommand(0.0, 0.0)
            front = self._through_front_range()
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.long_side_stop_range_m:
                self._record("near_right_fence_corner", x_m, y_m)
                self._finalize(None)
                return BaseCommand(0.0, 0.0)
            return BaseCommand(
                self.config.drive_speed_m_s,
                self._heading_correction(yaw_rad, self._return_heading),
            )

        return BaseCommand(0.0, 0.0)

    # ── turn helper ────────────────────────────────────────────────────────────

    def _turn_step(
        self,
        yaw_rad: float,
        target: float | None,
        next_state: LidarSurveyState,
        event: str,
    ) -> BaseCommand:
        if self._state_elapsed_s >= self.config.turn_timeout_s:
            self._finalize(f"{self.state.value}_timeout")
            return BaseCommand(0.0, 0.0)
        if target is None:
            self._finalize(f"{self.state.value}_no_target")
            return BaseCommand(0.0, 0.0)
        if self._heading_reached(yaw_rad, target, math.radians(90.0)):
            self._enter(next_state, event)
            return BaseCommand(0.0, 0.0)
        err = self._angle_delta(target, yaw_rad)
        return BaseCommand(0.0, self._proportional_turn(err))

    def _heading_reached(self, yaw_rad: float, target: float, total_delta_rad: float) -> bool:
        if self._turn_tracker.target_heading_rad is None:
            required = self._angle_delta(target, yaw_rad)
            if abs(required) > math.pi - math.radians(10.0):
                direction = 1.0
            else:
                direction = math.copysign(1.0, required) if abs(required) > 1e-6 else 0.0
            self._turn_tracker.reset(target, abs(required), direction)
        self._turn_tracker.update(yaw_rad)
        return self._turn_tracker.complete(
            yaw_rad, self.config.heading_tolerance_rad, self.config.turn_settle_ticks
        )

    # ── navigation helpers ─────────────────────────────────────────────────────

    def _heading_correction(self, yaw_rad: float, target: float | None) -> float:
        if target is None:
            return 0.0
        err = self._angle_delta(target, yaw_rad)
        cap = self.config.turn_speed_rad_s * 0.5
        return max(-cap, min(cap, err * 1.8))

    def _proportional_turn(self, err: float) -> float:
        gain = self.config.turn_speed_rad_s / math.radians(10.0)
        speed = max(self.config.turn_min_speed_rad_s, min(self.config.turn_speed_rad_s, abs(err) * gain))
        return math.copysign(speed, err)

    def _min_drive_cleared(self) -> bool:
        return (
            self._distance_traveled_m - self._state_entry_distance_m
        ) >= self.config.min_drive_before_corner_m

    def _front_range(self) -> float:
        if not self._last_scan_ranges:
            return math.inf
        n = len(self._last_scan_ranges)
        center = n // 2
        half = max(1, n // 16)
        values = [
            self._last_scan_ranges[(center + i) % n]
            for i in range(-half, half + 1)
            if math.isfinite(self._last_scan_ranges[(center + i) % n])
        ]
        return min(values) if values else math.inf

    def _through_front_range(self) -> float:
        """80th-percentile front range — passes through sparse net mesh."""
        if not self._last_scan_ranges:
            return math.inf
        cfg = self.config
        vals: list[float] = []
        center_rad = 0.0
        half_rad = math.radians(20.0)
        for i, r in enumerate(self._last_scan_ranges):
            if not math.isfinite(r) or r < cfg.lidar_min_range_m or r > cfg.lidar_max_range_m:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if center_rad - half_rad <= angle <= center_rad + half_rad:
                vals.append(r)
        if not vals:
            return math.inf
        vals.sort()
        idx = min(len(vals) - 1, int(round((len(vals) - 1) * 0.80)))
        return vals[idx]

    def _apply_safety(self, command: BaseCommand) -> BaseCommand:
        if command.linear_speed_m_s <= 0.0:
            return command
        cfg = self.config
        if self._last_front_range_m <= cfg.safety_stop_range_m:
            return BaseCommand(0.0, 0.0)
        if self._last_front_range_m <= cfg.safety_slow_range_m:
            return BaseCommand(command.linear_speed_m_s * 0.35, command.angular_speed_rad_s)
        return command

    # ── output ─────────────────────────────────────────────────────────────────

    def _record(self, label: str, x_m: float, y_m: float) -> None:
        if any(p["label"] == label for p in self._navigation_points):
            return
        self._navigation_points.append({
            "label": label,
            "x_m": round(float(x_m), 3),
            "y_m": round(float(y_m), 3),
            "state": self.state.value,
            "sample_count": self.sample_count,
            "distance_traveled_m": round(self._distance_traveled_m, 3),
        })

    def _finalize(self, failure: str | None) -> None:
        now = time.time()
        elapsed = 0.0 if self._started_at is None else now - self._started_at
        status = "SUCCESS" if failure is None and len(self._navigation_points) >= 6 else "PARTIAL"
        bounds = {
            "surveyed_at": now,
            "status": status,
            "survey_complete": status == "SUCCESS",
            "survey_type": "v2_open_loop_perimeter",
            "failure_reason": failure,
            "navigation_points": list(self._navigation_points),
            "navigation_route": [
                {"x_m": p["x_m"], "y_m": p["y_m"], "label": p["label"]}
                for p in self._navigation_points
            ],
            "elapsed_s": round(elapsed, 1),
            "court_geometry": {
                "length_m": self.config.expected_court_length_m,
                "width_m": self.config.expected_court_width_m,
                "method": "v2_open_loop_perimeter",
            },
        }
        self._court_bounds = bounds
        out = self.config.output_file
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(out)
        self._enter(LidarSurveyState.DONE, f"survey_{status.lower()}_{failure or 'ok'}")

    # ── internal plumbing ──────────────────────────────────────────────────────

    def _enter(self, state: LidarSurveyState, event: str) -> None:
        self.state = state
        self._last_event = event
        self._state_elapsed_s = 0.0
        self._state_entry_distance_m = self._distance_traveled_m
        if state in {
            LidarSurveyState.TURN_180,
            LidarSurveyState.TURN_TO_SIDELINE,
            LidarSurveyState.TURN_TO_LONG_SIDE,
            LidarSurveyState.TURN_TO_FAR_SHORT,
            LidarSurveyState.TURN_TO_RETURN,
        }:
            self._turn_tracker.reset()

    def _cmd(self, base: BaseCommand) -> LidarSurveyCommand:
        return LidarSurveyCommand(self.state, base, self.sample_count)

    def _update_distance(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = math.hypot(x_m - self._last_pose[0], y_m - self._last_pose[1])
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi
