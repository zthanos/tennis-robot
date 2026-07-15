"""collect_route mission: 360° scan → route plan → Nav2 legs → fine capture.

Pure FSM — no rclpy. The controller owns the ConceptACollectorBehavior, the
BallMap and the Nav2LaneNavigator; it passes their state into update() each
tick and wires `nav_goal` to the navigator (same ownership contract as
CollectOneMission / ServiceLineDistributionScanMission.nav2_target).

Flow:
  SCAN_ROTATE  step-rotate 360° while the controller feeds every camera
               detection into the ball map with the scan-range override.
  PLAN         order confirmed same-side balls (greedy NN + 2-opt) and compute
               a boundary-aware approach pose per stop.
  NAV_TO_BALL  expose the approach pose as a Nav2 goal; wheels belong to Nav2
               (the mission returns an idle command, twist_mux arbitration).
  FINE_APPROACH  ConceptACollectorBehavior ALIGN→APPROACH→CAPTURE on the
               locked mapped ball (collect_one's blind-zone lock pattern).
  SETTLE       short post-capture hold, then the next stop.
Balls confirmed mid-route that are not in the plan are added by cheapest
insertion (never reordering the leg in progress or earlier stops).
"""

from __future__ import annotations

import math

from tennis_robot.ball_map import BallMap, across_net
from tennis_robot.collection_route_planner import (
    CourtModel,
    RoutePlannerConfig,
    RouteStop,
    approach_pose_for_ball,
    cheapest_insertion,
    order_route,
    remaining_route_length_m,
    route_polyline,
)
from tennis_robot.collector import (
    BallObservationInput,
    BaseCommand,
    CollectorCommand,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptACommand,
)
from tennis_robot.config_utils import _env_float

# ── Config ─────────────────────────────────────────────────────────────────────

NAV_TIMEOUT_S = _env_float("COLLECT_ROUTE_NAV_TIMEOUT_S", 60.0)
NAV_RETRIES = int(_env_float("COLLECT_ROUTE_NAV_RETRIES", 2))
MAX_BALL_ATTEMPTS = int(_env_float("COLLECT_ROUTE_MAX_BALL_ATTEMPTS", 2))
MISSING_SCAN_S = _env_float("COLLECT_ROUTE_MISSING_SCAN_S", 6.0)
APPROACH_TIMEOUT_S = _env_float("COLLECT_PATTERN_COLLECTION_TIMEOUT_S", 35.0)

_SCAN_STEP_RAD = math.radians(30.0)
_SCAN_STEP_TOLERANCE_RAD = math.radians(2.0)
_SCAN_TURN_SPEED_RAD_S = 0.65
_SCAN_ANGULAR_GAIN = 1.8
_SCAN_SETTLE_S = 0.20
_SETTLE_HOLD_S = 2.0
_SETTLE_INTAKE_S = 0.25
# Max distance between a fresh sighting and the locked mapped ball for the
# lock to be refreshed (same physical ball) — see collect_one debug-log #44.
_RELOCK_GATE_M = 0.6

_IDLE_CMD = ConceptACommand(
    state=CollectorState.IDLE,
    base=BaseCommand(0.0, 0.0),
    collector=CollectorCommand(0.0, False),
)
# Zero SURVEY command: motor adapter goes publish-zero-once silent so
# twist_mux hands the wheels to Nav2 (/cmd_vel_nav).
_NAV_IDLE_CMD = ConceptACommand(
    state=CollectorState.SURVEY,
    base=BaseCommand(0.0, 0.0),
    collector=CollectorCommand(0.0, False),
)
_SCAN_HOLD_CMD = ConceptACommand(
    state=CollectorState.SCAN,
    base=BaseCommand(0.0, 0.0),
    collector=CollectorCommand(0.0, False),
)


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _world_to_robot_obs(
    world_x: float,
    world_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> BallObservationInput | None:
    dx = world_x - robot_x
    dy = world_y - robot_y
    local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
    local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
    if local_x <= -0.1:
        return None
    return BallObservationInput(
        visible=True,
        bearing_rad=math.atan2(local_y, local_x),
        distance_m=math.hypot(local_x, local_y),
        confidence=0.8,
        source="collect_route_locked",
        robot_x_m=local_x,
        robot_y_m=local_y,
        world_x_m=world_x,
        world_y_m=world_y,
    )


# ── Mission ────────────────────────────────────────────────────────────────────

class CollectRouteMission:
    """Fast multi-ball collection: scan once, plan the route, execute it."""

    def __init__(self, planner_cfg: RoutePlannerConfig | None = None) -> None:
        self.planner_cfg = planner_cfg or RoutePlannerConfig.from_env()
        self.phase: str = "idle"  # idle|scan|plan|nav|approach|settle|done
        self.stops: list[RouteStop] = []
        self.current_index: int = 0
        self.insertion_count: int = 0
        self.current_blocker: str | None = None
        self._planned_ids: set[int] = set()
        self._scan_target_yaw: float | None = None
        self._scan_settle_until_s: float = 0.0
        self._scan_steps_taken: int = 0
        self._nav_goal: tuple[float, float, float] | None = None
        self._nav_elapsed_s: float = 0.0
        self._nav_last_state: str = "idle"
        self._nav_attempts: int = 0
        self._approach_elapsed_s: float = 0.0
        self._missing_scan_elapsed_s: float = 0.0
        self._live_seen_in_approach: bool = False
        self._locked_world: tuple[float, float] | None = None
        self._settle_remaining_s: float = 0.0
        self._events: list[tuple[str, dict]] = []
        self._complete_reported = False

    def reset(self) -> None:
        self.__init__(self.planner_cfg)

    def start(self, robot_pose: tuple[float, float, float]) -> None:
        self.reset()
        self.phase = "scan"
        self._emit("route_scan_start")

    # ── Observable state ───────────────────────────────────────────────────────

    @property
    def is_done(self) -> bool:
        return self.phase == "done"

    @property
    def scanning(self) -> bool:
        return self.phase == "scan"

    @property
    def nav_goal(self) -> tuple[float, float, float] | None:
        return self._nav_goal if self.phase == "nav" else None

    @property
    def current_ball_id(self) -> int | None:
        if self.phase in ("nav", "approach", "settle") and self.current_index < len(self.stops):
            return self.stops[self.current_index].ball_id
        return None

    def drain_events(self) -> list[tuple[str, dict]]:
        events, self._events = self._events, []
        return events

    def _emit(self, event_type: str, **fields: object) -> None:
        self._events.append((event_type, fields))

    # ── Console export ─────────────────────────────────────────────────────────

    def route_export(
        self, robot_xy: tuple[float, float]
    ) -> tuple[list[dict], dict[int, int]]:
        """(map.route polyline, {ball_id: display order}) for the console."""
        pending = [s for s in self.stops if s.status in ("pending", "active")]
        polyline = route_polyline(robot_xy, self.stops) if pending else []
        planned_order = {s.ball_id: s.order for s in pending}
        return polyline, planned_order

    def telemetry(self) -> dict:
        counts = {"pending": 0, "active": 0, "collected": 0, "skipped": 0, "missing": 0}
        for stop in self.stops:
            counts[stop.status] = counts.get(stop.status, 0) + 1
        return {
            "phase": self.phase,
            "current_ball_id": self.current_ball_id,
            "stop_count": len(self.stops),
            "stops": counts,
            "insertions": self.insertion_count,
            "scan_steps": self._scan_steps_taken,
            "nav_attempts": self._nav_attempts,
            "current_blocker": self.current_blocker,
            "route": [
                {
                    "ball_id": s.ball_id,
                    "order": s.order,
                    "status": s.status,
                    "approach_mode": s.approach.mode,
                    "risk": s.approach.risk,
                }
                for s in self.stops
            ],
        }

    # ── Tick ───────────────────────────────────────────────────────────────────

    def update(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        now: float,
        nav_state: str,
        court: CourtModel | None,
    ) -> ConceptACommand:
        self.current_blocker = None
        if self.phase == "idle":
            self.start(robot_pose)

        if self.phase in ("nav", "approach", "settle"):
            self._insert_new_balls(ball_map, robot_pose, court, now)

        if self.phase == "scan":
            return self._scan_phase(robot_pose, ball_map, now)
        if self.phase == "plan":
            return self._plan_phase(robot_pose, ball_map, court, now)
        if self.phase == "nav":
            return self._nav_phase(dt_s, robot_pose, behavior, ball_map, nav_state, now)
        if self.phase == "approach":
            return self._approach_phase(
                observation, collection_confirmed, dt_s, robot_pose, behavior, ball_map, now
            )
        if self.phase == "settle":
            return self._settle_phase(dt_s, behavior)
        return _IDLE_CMD

    # ── SCAN_ROTATE ────────────────────────────────────────────────────────────

    def _scan_phase(
        self,
        robot_pose: tuple[float, float, float],
        ball_map: BallMap,
        now: float,
    ) -> ConceptACommand:
        robot_yaw = robot_pose[2]
        if self._scan_settle_until_s > now:
            return _SCAN_HOLD_CMD

        max_steps = int(math.ceil(2 * math.pi / _SCAN_STEP_RAD))
        if self._scan_target_yaw is None:
            if self._scan_steps_taken >= max_steps:
                ball_map.prune_phantoms(now)
                self.phase = "plan"
                return _SCAN_HOLD_CMD
            self._scan_target_yaw = robot_yaw + _SCAN_STEP_RAD

        yaw_err = _angle_delta(self._scan_target_yaw, robot_yaw)
        if abs(yaw_err) <= _SCAN_STEP_TOLERANCE_RAD:
            self._scan_target_yaw = None
            self._scan_steps_taken += 1
            self._scan_settle_until_s = now + _SCAN_SETTLE_S
            return _SCAN_HOLD_CMD

        angular = max(
            -_SCAN_TURN_SPEED_RAD_S,
            min(_SCAN_TURN_SPEED_RAD_S, yaw_err * _SCAN_ANGULAR_GAIN),
        )
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, angular),
            collector=CollectorCommand(0.0, False),
        )

    # ── PLAN ───────────────────────────────────────────────────────────────────

    def _collectable_balls(
        self,
        ball_map: BallMap,
        robot_pose: tuple[float, float, float],
        now: float,
        court: "CourtModel | None",
    ) -> list[tuple[int, float, float]]:
        cfg = ball_map.config
        robot_x, robot_y = robot_pose[0], robot_pose[1]

        def _same_side(ball) -> bool:
            # The surveyed net line is authoritative; the across_net(net_x=0)
            # convention only holds in worlds whose frame is centred on the
            # net (run-3 incident: real net at map x≈8, robot sent across it).
            if court is not None:
                return court.same_side(robot_x, robot_y, ball.x_m, ball.y_m)
            return not across_net(robot_x, ball.x_m, cfg.net_x_m, cfg.net_side_clearance_m)

        return [
            (b.id, b.x_m, b.y_m)
            for b in ball_map.balls.values()
            if b.state not in {"collected", "collection_failed"}
            and b.seen_count >= cfg.min_seen_count
            and now - b.last_seen_s <= cfg.stale_after_s
            and _same_side(b)
        ]

    def _plan_phase(
        self,
        robot_pose: tuple[float, float, float],
        ball_map: BallMap,
        court: CourtModel | None,
        now: float,
    ) -> ConceptACommand:
        robot_x, robot_y, _ = robot_pose
        balls = self._collectable_balls(ball_map, robot_pose, now, court)
        if not balls:
            self._emit("route_planned", stops=0)
            self._finish()
            return _IDLE_CMD

        by_id = {b[0]: (b[1], b[2]) for b in balls}
        ordered_ids = order_route((robot_x, robot_y), balls, self.planner_cfg)
        self.stops = []
        prev_xy = (robot_x, robot_y)
        for order, ball_id in enumerate(ordered_ids, start=1):
            ball_xy = by_id[ball_id]
            approach = approach_pose_for_ball(ball_xy, prev_xy, court, self.planner_cfg)
            self.stops.append(
                RouteStop(
                    ball_id=ball_id,
                    ball_x_m=ball_xy[0],
                    ball_y_m=ball_xy[1],
                    approach=approach,
                    order=order,
                )
            )
            prev_xy = ball_xy
        self._planned_ids = set(ordered_ids)
        self.current_index = 0
        self._emit(
            "route_planned",
            stops=len(self.stops),
            lateral_stops=sum(1 for s in self.stops if s.approach.mode == "lateral"),
            route_length_m=remaining_route_length_m((robot_x, robot_y), self.stops),
        )
        self._enter_nav()
        return _IDLE_CMD

    # ── NAV_TO_BALL ────────────────────────────────────────────────────────────

    def _enter_nav(self) -> None:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return
        stop.status = "active"
        self.phase = "nav"
        self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
        self._nav_elapsed_s = 0.0
        self._nav_last_state = "idle"
        self._emit(
            "route_leg_start",
            ball_id=stop.ball_id,
            order=stop.order,
            approach_mode=stop.approach.mode,
            risk=stop.approach.risk,
            goal_x_m=stop.approach.x_m,
            goal_y_m=stop.approach.y_m,
            goal_yaw_rad=stop.approach.yaw_rad,
        )

    def _nav_phase(
        self,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        nav_state: str,
        now: float,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if nav_state == "unavailable":
            self.current_blocker = "nav2_action_unavailable"
            return _NAV_IDLE_CMD

        if self._nav_goal is None:
            # Previous attempt was cancelled last tick; re-issue the goal so
            # Nav2 replans from the current pose.
            self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
            self._nav_elapsed_s = 0.0
            self._nav_last_state = "idle"
            return _NAV_IDLE_CMD

        self._nav_elapsed_s += dt_s
        failed = nav_state == "failed" and self._nav_last_state != "failed"
        timed_out = self._nav_elapsed_s > NAV_TIMEOUT_S
        self._nav_last_state = nav_state

        if failed or timed_out:
            self._nav_attempts += 1
            self._emit(
                "route_leg_nav_retry" if self._nav_attempts <= NAV_RETRIES else "route_leg_skip",
                ball_id=stop.ball_id,
                attempts=self._nav_attempts,
                reason="nav_timeout" if timed_out else "nav_failed",
            )
            if self._nav_attempts > NAV_RETRIES:
                self._skip_current(ball_map, "skipped")
                return _NAV_IDLE_CMD
            # Drop the goal for one tick so the controller cancels the
            # in-flight goal; the next tick re-issues it (branch above).
            self._nav_goal = None
            return _NAV_IDLE_CMD

        if nav_state == "reached":
            self._enter_approach(robot_pose, behavior, ball_map, now)
        return _NAV_IDLE_CMD

    # ── FINE_APPROACH ──────────────────────────────────────────────────────────

    def _enter_approach(
        self,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        now: float,
    ) -> None:
        stop = self._current_stop()
        self.phase = "approach"
        self._nav_goal = None
        self._approach_elapsed_s = 0.0
        self._missing_scan_elapsed_s = 0.0
        self._live_seen_in_approach = False
        behavior.reset()
        ball = ball_map.balls.get(stop.ball_id) if stop is not None else None
        self._locked_world = (ball.x_m, ball.y_m) if ball is not None else None
        self._emit(
            "route_fine_approach",
            ball_id=stop.ball_id if stop else None,
            locked=self._locked_world is not None,
        )

    def _approach_phase(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        now: float,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if collection_confirmed:
            behavior.reset()
            stop.status = "collected"
            self.phase = "settle"
            self._settle_remaining_s = _SETTLE_HOLD_S
            self._emit("route_ball_collected", ball_id=stop.ball_id, order=stop.order)
            return ConceptACommand(
                state=CollectorState.COLLECTED,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
            )

        robot_x, robot_y, robot_yaw = robot_pose
        self._approach_elapsed_s += dt_s

        # Refresh the lock while the ball stays visible (blind-zone pattern
        # from collect_one, debug-log #44): live sightings within the gate
        # shrink the frozen error before the near-field blind zone takes over.
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and self._locked_world is not None
            and math.hypot(
                observation.world_x_m - self._locked_world[0],
                observation.world_y_m - self._locked_world[1],
            )
            <= _RELOCK_GATE_M
        ):
            self._locked_world = (observation.world_x_m, observation.world_y_m)
            self._live_seen_in_approach = True

        # Hard budget FIRST — before any early-return branch below. Run-3
        # incident (collection-route-debug-log-el #6): the turn-toward-target
        # branch returned before this check ever ran, so a phantom chase
        # pushed into the net for 78 s with the 35 s timeout never firing.
        if behavior.gave_up or self._approach_elapsed_s > APPROACH_TIMEOUT_S:
            return self._approach_failed(
                stop, ball_map, behavior,
                "gave_up" if behavior.gave_up else "approach_timeout",
            )

        # Phantom gate: from the 1.3 m standoff the camera MUST see a real
        # ball within a few seconds (blind zone starts ~0.9 m). No live
        # sighting near the lock by then means the map entry is a phantom —
        # skip it instead of dead-reckoning a capture into whatever stands
        # there (run 3 drove into the net this way, lock_error_m 3.3-4.0).
        if (
            not self._live_seen_in_approach
            and self._approach_elapsed_s > MISSING_SCAN_S
        ):
            self._emit(
                "route_ball_missing",
                ball_id=stop.ball_id,
                reason="no_live_sighting_at_standoff",
            )
            behavior.reset()
            self._skip_current(ball_map, "missing")
            return _IDLE_CMD

        if self._locked_world is None:
            # Mapped entry vanished (stale/pruned): give the camera a short
            # in-place scan before declaring the ball missing.
            if (
                observation.visible
                and observation.world_x_m is not None
                and math.hypot(
                    observation.world_x_m - stop.ball_x_m,
                    observation.world_y_m - stop.ball_y_m,
                )
                <= _RELOCK_GATE_M
            ):
                self._locked_world = (observation.world_x_m, observation.world_y_m)
                self._live_seen_in_approach = True
            else:
                self._missing_scan_elapsed_s += dt_s
                if self._missing_scan_elapsed_s > MISSING_SCAN_S:
                    self._emit("route_ball_missing", ball_id=stop.ball_id)
                    self._skip_current(ball_map, "missing")
                    return _IDLE_CMD
                return ConceptACommand(
                    state=CollectorState.SCAN,
                    base=BaseCommand(0.0, _SCAN_TURN_SPEED_RAD_S * 0.6),
                    collector=CollectorCommand(0.0, False),
                )

        locked_obs = _world_to_robot_obs(*self._locked_world, robot_x, robot_y, robot_yaw)
        if locked_obs is None:
            # The locked ball is BEHIND the robot: Nav2 can report "reached"
            # with a loose final yaw, and the behavior's blind scan-spin then
            # looks like a second 360° (run-2/3 user report). We know exactly
            # where the ball is — turn straight toward it, shortest way.
            dx = self._locked_world[0] - robot_x
            dy = self._locked_world[1] - robot_y
            bearing = _angle_delta(math.atan2(dy, dx), robot_yaw)
            angular = max(
                -_SCAN_TURN_SPEED_RAD_S,
                min(_SCAN_TURN_SPEED_RAD_S, bearing * _SCAN_ANGULAR_GAIN),
            )
            return ConceptACommand(
                state=CollectorState.SCAN,
                base=BaseCommand(0.0, angular),
                collector=CollectorCommand(0.0, False),
            )

        # Track ONLY the locked target. Falling back to the raw camera
        # observation here could silently steal the approach for a different
        # visible ball; the lock refresh above (gated to 0.6 m) is the only
        # place live sightings feed the target.
        tracking_obs = locked_obs
        if behavior.state == CollectorState.SCAN and tracking_obs.visible:
            behavior.start_tracking(tracking_obs)
        cmd = behavior.update(tracking_obs, dt_s, collection_confirmed=False)
        if behavior.gave_up:
            return self._approach_failed(stop, ball_map, behavior, "gave_up")
        return cmd

    def _approach_failed(
        self,
        stop: RouteStop,
        ball_map: BallMap,
        behavior: ConceptACollectorBehavior,
        reason: str,
    ) -> ConceptACommand:
        behavior.reset()
        stop.attempts += 1
        if stop.attempts >= MAX_BALL_ATTEMPTS:
            self._emit("route_leg_skip", ball_id=stop.ball_id, reason=reason)
            self._skip_current(ball_map, "skipped")
        else:
            self._emit("route_leg_retry", ball_id=stop.ball_id, reason=reason)
            self._nav_attempts = 0
            self._enter_nav_retry(stop)
        return _IDLE_CMD

    def _enter_nav_retry(self, stop: RouteStop) -> None:
        self.phase = "nav"
        self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
        self._nav_elapsed_s = 0.0
        self._nav_last_state = "idle"

    # ── SETTLE ─────────────────────────────────────────────────────────────────

    def _settle_phase(
        self, dt_s: float, behavior: ConceptACollectorBehavior
    ) -> ConceptACommand:
        self._settle_remaining_s = max(0.0, self._settle_remaining_s - dt_s)
        if self._settle_remaining_s > 0.0:
            intake_enabled = self._settle_remaining_s > (_SETTLE_HOLD_S - _SETTLE_INTAKE_S)
            return ConceptACommand(
                state=CollectorState.COLLECTED,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(
                    behavior.config.lift_wheel_speed if intake_enabled else 0.0,
                    intake_enabled,
                ),
            )
        self._advance()
        return _IDLE_CMD

    # ── Route bookkeeping ──────────────────────────────────────────────────────

    def _current_stop(self) -> RouteStop | None:
        if self.current_index < len(self.stops):
            return self.stops[self.current_index]
        return None

    def _skip_current(self, ball_map: BallMap, status: str) -> None:
        stop = self._current_stop()
        if stop is not None:
            stop.status = status
            ball_map.set_state(stop.ball_id, "collection_failed")
        self._advance()

    def _advance(self) -> None:
        self._nav_attempts = 0
        self._locked_world = None
        self.current_index += 1
        while self.current_index < len(self.stops) and self.stops[
            self.current_index
        ].status not in ("pending",):
            self.current_index += 1
        if self.current_index >= len(self.stops):
            self._finish()
        else:
            self._enter_nav()

    def _finish(self) -> None:
        self.phase = "done"
        self._nav_goal = None
        collected = sum(1 for s in self.stops if s.status == "collected")
        self._emit(
            "route_complete",
            stops=len(self.stops),
            collected=collected,
            skipped=sum(1 for s in self.stops if s.status == "skipped"),
            missing=sum(1 for s in self.stops if s.status == "missing"),
            insertions=self.insertion_count,
        )

    # ── Dynamic insertion ──────────────────────────────────────────────────────

    def _insert_new_balls(
        self,
        ball_map: BallMap,
        robot_pose: tuple[float, float, float],
        court: CourtModel | None,
        now: float,
    ) -> None:
        robot_x, robot_y, _ = robot_pose
        for ball_id, bx, by in self._collectable_balls(ball_map, robot_pose, now, court):
            if ball_id in self._planned_ids:
                continue
            self._planned_ids.add(ball_id)

            # Route anchor: the ball of the leg in progress, then the pending
            # tail. New balls may only enter after the current leg.
            pending_indices = [
                i
                for i in range(self.current_index + 1, len(self.stops))
                if self.stops[i].status == "pending"
            ]
            current = self._current_stop()
            anchor = (
                (current.ball_x_m, current.ball_y_m)
                if current is not None and current.status == "active"
                else (robot_x, robot_y)
            )
            points = [anchor] + [
                (self.stops[i].ball_x_m, self.stops[i].ball_y_m) for i in pending_indices
            ]
            insert_at, detour_m = cheapest_insertion(points, (bx, by), start_index=1)
            if detour_m > self.planner_cfg.insertion_max_detour_m:
                insert_at = len(points)  # append at route end

            if insert_at >= len(points):
                stop_index = len(self.stops)
                prev_xy = (
                    (self.stops[pending_indices[-1]].ball_x_m, self.stops[pending_indices[-1]].ball_y_m)
                    if pending_indices
                    else anchor
                )
            else:
                stop_index = pending_indices[insert_at - 1]
                prev_xy = points[insert_at - 1]

            approach = approach_pose_for_ball((bx, by), prev_xy, court, self.planner_cfg)
            new_stop = RouteStop(
                ball_id=ball_id,
                ball_x_m=bx,
                ball_y_m=by,
                approach=approach,
                order=0,
            )
            self.stops.insert(stop_index, new_stop)
            # The successor's incoming direction changed; refresh its pose.
            successor_index = stop_index + 1
            if (
                successor_index < len(self.stops)
                and self.stops[successor_index].status == "pending"
            ):
                succ = self.stops[successor_index]
                succ.approach = approach_pose_for_ball(
                    (succ.ball_x_m, succ.ball_y_m), (bx, by), court, self.planner_cfg
                )
            self._renumber()
            self.insertion_count += 1
            self._emit(
                "route_insertion",
                ball_id=ball_id,
                detour_m=detour_m,
                position=new_stop.order,
                approach_mode=approach.mode,
            )

    def _renumber(self) -> None:
        order = 0
        for stop in self.stops:
            if stop.status in ("pending", "active"):
                order += 1
                stop.order = order
