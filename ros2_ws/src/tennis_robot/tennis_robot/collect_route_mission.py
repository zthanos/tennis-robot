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
# Wider gate for the FIRST sighting of an approach: entries created by the
# 9 m 360° scan carry up to ~0.5 m position error (run-3 lock_error_m data),
# so from the standoff the nearest ball within 1 m of the plan is the target.
_INITIAL_ADOPT_GATE_M = 1.0
# Refresh the nav goal when the mapped ball drifted this far from the stop's
# planned position (nudged balls, refined estimates).
_GOAL_REFRESH_DRIFT_M = 0.3
# ...but an entry that wandered this far from the PLAN-time position is not
# the same physical ball any more (run-4 stop 6: chain-merges dragged the
# entry 4+ m across the court and the approach followed it) — drop the stop.
_GOAL_DRIFT_ABANDON_M = 1.5
# Nav failure faster than this = the planner rejected the request outright
# (start pose inside costmap inflation), not a genuine navigation failure.
_NAV_INSTANT_FAIL_S = 2.0
_NAV_MAX_RECOVERIES = 2
_NAV_RECOVER_REVERSE_S = 2.5
_NAV_RECOVER_SPEED_M_S = -0.15
# Opportunistic capture during Nav2 legs: Nav2 cannot see balls (lidar plane
# is above them) so legs plow straight through — runs 5-7 froze with a ball
# suspected under the chassis. A ball visible ahead within this range/bearing
# is collected on the spot, then the leg resumes. PLAN-ONLY (user decision,
# log #13): the ball must belong to a pending/active stop — anything else is
# ignored (confirmed new balls join the plan via insertion anyway).
_OPP_CAPTURE_RANGE_M = 1.2
_OPP_CAPTURE_BEARING_RAD = math.radians(40.0)
_OPP_TIMEOUT_S = 15.0
# An opportunistic ball matches the plan stop whose planned ball lies within
# this distance.
_OPP_STOP_MATCH_M = 0.8
# Blind reverse recoveries are capped PER RUN: repeated reversing under
# persistent rejections walked the robot into the fence (run 6). Failures
# beyond the budget skip the stop and the plan simply continues (user
# decision, log #13: record the failure, move to the next planned ball).
_TOTAL_RECOVERY_BUDGET = 4

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
        self._nav_recoveries: int = 0
        self._total_recoveries: int = 0
        self._recover_until_s: float = 0.0
        self._opp_locked: tuple[float, float] | None = None
        self._opp_stop_id: int | None = None
        self._opp_elapsed_s: float = 0.0
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

    @property
    def current_target_xy(self) -> tuple[float, float] | None:
        """World position of the ball being pursued (lock if approaching).

        None during nav legs on purpose: there the interesting observation is
        whatever ball is nearest ahead (opportunistic capture), not the
        distant target of the leg."""
        if self.phase == "approach" and self._locked_world is not None:
            return self._locked_world
        if self.phase == "opportunistic" and self._opp_locked is not None:
            return self._opp_locked
        if self.phase == "approach" and self.current_index < len(self.stops):
            stop = self.stops[self.current_index]
            return (stop.ball_x_m, stop.ball_y_m)
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
            "planned_total": len(self.stops),
            "remaining": counts.get("pending", 0) + counts.get("active", 0),
            "failed_ball_ids": [
                s.ball_id for s in self.stops if s.status in ("skipped", "missing")
            ],
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
            return self._nav_phase(
                observation, dt_s, robot_pose, behavior, ball_map, nav_state, now, court
            )
        if self.phase == "opportunistic":
            return self._opportunistic_phase(
                observation, collection_confirmed, dt_s, robot_pose, behavior, ball_map
            )
        if self.phase == "recover":
            return self._recover_phase(now)
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
        self._nav_recoveries = 0
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
        observation: BallObservationInput,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        nav_state: str,
        now: float,
        court: "CourtModel | None" = None,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if nav_state == "unavailable":
            self.current_blocker = "nav2_action_unavailable"
            return _NAV_IDLE_CMD

        # A PLANNED ball directly ahead on the leg: collect it instead of
        # plowing over it (Nav2 cannot see balls — runs 5-7 froze with one
        # suspected under the chassis). Plan-only: an unplanned sighting never
        # diverts the leg. The leg resumes right after.
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and observation.distance_m <= _OPP_CAPTURE_RANGE_M
            and abs(observation.bearing_rad) <= _OPP_CAPTURE_BEARING_RAD
        ):
            matched = self._matching_plan_stop(
                observation.world_x_m, observation.world_y_m
            )
            if matched is not None:
                self.phase = "opportunistic"
                self._nav_goal = None  # controller cancels the Nav2 goal
                self._opp_locked = (observation.world_x_m, observation.world_y_m)
                self._opp_stop_id = matched.ball_id
                self._opp_elapsed_s = 0.0
                behavior.reset()
                behavior.start_tracking(observation)
                self._emit(
                    "route_opportunistic_start",
                    ball_id=matched.ball_id,
                    resumed_stop=stop.ball_id,
                    ball_x_m=observation.world_x_m,
                    ball_y_m=observation.world_y_m,
                    distance_m=observation.distance_m,
                )
                return behavior.update(observation, dt_s, collection_confirmed=False)

        # The mapped ball may have drifted since the plan (e.g. nudged by a
        # previous capture attempt — run-4 stops 12/6 retried into the STALE
        # standoff and found nothing). Follow the live map entry: refresh the
        # ball position and approach pose when it moved meaningfully — but an
        # entry far from the PLAN position has chain-merged onto other balls
        # (run-4 stop 6 wandered 4+ m): drop it instead of chasing it.
        entry = ball_map.balls.get(stop.ball_id)
        if entry is not None and math.hypot(
            entry.x_m - stop.planned_x_m, entry.y_m - stop.planned_y_m
        ) > _GOAL_DRIFT_ABANDON_M:
            self._emit(
                "route_ball_lost",
                ball_id=stop.ball_id,
                reason="map_entry_drifted",
                drift_m=math.hypot(
                    entry.x_m - stop.planned_x_m, entry.y_m - stop.planned_y_m
                ),
            )
            self._skip_current(ball_map, "missing")
            return _NAV_IDLE_CMD
        if entry is not None and math.hypot(
            entry.x_m - stop.ball_x_m, entry.y_m - stop.ball_y_m
        ) > _GOAL_REFRESH_DRIFT_M:
            stop.ball_x_m, stop.ball_y_m = entry.x_m, entry.y_m
            stop.approach = approach_pose_for_ball(
                (entry.x_m, entry.y_m),
                (robot_pose[0], robot_pose[1]),
                court,
                self.planner_cfg,
            )
            self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
            self._emit(
                "route_goal_updated",
                ball_id=stop.ball_id,
                ball_x_m=entry.x_m,
                ball_y_m=entry.y_m,
                approach_mode=stop.approach.mode,
            )

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

        # Instant rejection usually means the START pose is inside costmap
        # inflation (run-5: every goal aborted in <0.1 s after the robot
        # ended an approach near the net; the whole route burned through in
        # 1.3 s). Back straight out of the inflated zone, then re-issue.
        if failed and self._nav_elapsed_s < _NAV_INSTANT_FAIL_S:
            if (
                self._nav_recoveries < _NAV_MAX_RECOVERIES
                and self._total_recoveries < _TOTAL_RECOVERY_BUDGET
            ):
                self._nav_recoveries += 1
                self._total_recoveries += 1
                self._emit(
                    "route_nav_recovery",
                    ball_id=stop.ball_id,
                    recovery=self._nav_recoveries,
                    nav_elapsed_s=self._nav_elapsed_s,
                )
                self.phase = "recover"
                self._recover_until_s = now + _NAV_RECOVER_REVERSE_S
                self._nav_goal = None
                return _NAV_IDLE_CMD

        if failed or timed_out:
            self._nav_attempts += 1
            self._emit(
                "route_leg_nav_retry" if self._nav_attempts <= NAV_RETRIES else "route_leg_skip",
                ball_id=stop.ball_id,
                attempts=self._nav_attempts,
                reason="nav_timeout" if timed_out else "nav_failed",
            )
            if self._nav_attempts > NAV_RETRIES:
                # Record the failure and continue the SAME plan from the next
                # planned ball (user decision, log #13) — no route abort.
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
        # The first sighting of the approach gets the wider adoption gate —
        # scan-created entries can be ~0.5 m off; later refreshes use the
        # strict gate so a different ball cannot steal the lock mid-capture.
        refresh_gate_m = (
            _RELOCK_GATE_M if self._live_seen_in_approach else _INITIAL_ADOPT_GATE_M
        )
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and self._locked_world is not None
            and math.hypot(
                observation.world_x_m - self._locked_world[0],
                observation.world_y_m - self._locked_world[1],
            )
            <= refresh_gate_m
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

    def _opportunistic_phase(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if collection_confirmed:
            behavior.reset()
            credited = self._credit_opportunistic_stop()
            self._emit(
                "route_opportunistic_collected",
                ball_id=credited,
                resumed_stop=stop.ball_id,
            )
            self._opp_locked = None
            if stop.status == "collected":
                # The opportunistic ball WAS the current stop's ball.
                self.phase = "settle"
                self._settle_remaining_s = _SETTLE_HOLD_S
            else:
                self._enter_nav_retry(stop)
            return _IDLE_CMD

        self._opp_elapsed_s += dt_s
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and self._opp_locked is not None
            and math.hypot(
                observation.world_x_m - self._opp_locked[0],
                observation.world_y_m - self._opp_locked[1],
            )
            <= _RELOCK_GATE_M
        ):
            self._opp_locked = (observation.world_x_m, observation.world_y_m)

        if behavior.gave_up or self._opp_elapsed_s > _OPP_TIMEOUT_S:
            # Not worth a fight — resume the leg; the ball stays mapped.
            behavior.reset()
            self._emit("route_opportunistic_abort", resumed_stop=stop.ball_id)
            self._opp_locked = None
            self._enter_nav_retry(stop)
            return _NAV_IDLE_CMD

        robot_x, robot_y, robot_yaw = robot_pose
        locked_obs = (
            _world_to_robot_obs(*self._opp_locked, robot_x, robot_y, robot_yaw)
            if self._opp_locked is not None
            else None
        )
        tracking_obs = locked_obs if locked_obs is not None else observation
        if behavior.state == CollectorState.SCAN and tracking_obs.visible:
            behavior.start_tracking(tracking_obs)
        return behavior.update(tracking_obs, dt_s, collection_confirmed=False)

    def _matching_plan_stop(self, x: float, y: float) -> "RouteStop | None":
        """The pending/active stop whose planned ball lies nearest (x, y)."""
        best: RouteStop | None = None
        best_d = _OPP_STOP_MATCH_M
        for s in self.stops:
            if s.status not in ("pending", "active"):
                continue
            d = math.hypot(s.ball_x_m - x, s.ball_y_m - y)
            if d < best_d:
                best, best_d = s, d
        return best

    def _credit_opportunistic_stop(self) -> int | None:
        """Mark the plan stop matched at opportunistic start as collected."""
        for s in self.stops:
            if s.ball_id == self._opp_stop_id and s.status in ("pending", "active"):
                s.status = "collected"
                self._renumber()
                return s.ball_id
        return None

    def _recover_phase(self, now: float) -> ConceptACommand:
        """Reverse straight back out of costmap inflation, then retry the leg."""
        if now < self._recover_until_s:
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(_NAV_RECOVER_SPEED_M_S, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD
        self._enter_nav_retry(stop)
        return _NAV_IDLE_CMD

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
        """The plan ledger is exhausted: every planned ball (initial scan +
        insertions) is accounted for as collected or failed. Completion is
        declared here and only here (user decision, log #13)."""
        self.phase = "done"
        self._nav_goal = None
        failed = [s.ball_id for s in self.stops if s.status in ("skipped", "missing")]
        self._emit(
            "route_complete",
            planned_total=len(self.stops),
            collected=sum(1 for s in self.stops if s.status == "collected"),
            skipped=sum(1 for s in self.stops if s.status == "skipped"),
            missing=sum(1 for s in self.stops if s.status == "missing"),
            failed_ball_ids=failed,
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
