"""collect_route mission: 360° scan → one planned drive-through route.

Pure FSM — no rclpy. The controller owns the ConceptACollectorBehavior, the
BallMap and the Nav2LaneNavigator; it passes their state into update() each
tick and wires `nav_goal` to the navigator (same ownership contract as
CollectOneMission / ServiceLineDistributionScanMission.nav2_target).

Runtime flow:
  SCAN_ROTATE  step-rotate 360° while the controller feeds every camera
               detection into the ball map with the scan-range override.
  PLAN         order confirmed same-side balls once and calculate a straight
               crossing for each: the ball centre must pass through the funnel.
  DRIVE         Nav2 reaches each run-in entry; the mission drives straight
               through the funnel crossing and immediately continues to the
               following crossing.

Collection is deliberately not a mission transition.  A basket/beam
confirmation only credits telemetry for the nearest planned crossing; it never
causes a wait, retry, fine approach, or route replan.  This makes the route a
single fixed traversal even when a ball is missed.
"""

from __future__ import annotations

import math
import os

from tennis_robot.ball_map import BallMap, across_net
from tennis_robot.collection_route_planner import (
    ApproachPose,
    CourtModel,
    RoutePlannerConfig,
    RouteStop,
    approach_pose_for_ball,
    cheapest_insertion,
    order_route,
    remaining_route_length_m,
    route_polyline,
    SWEEP_OVERRUN_M,
    SWEEP_RUN_IN_M,
    sweep_route,
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


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


# ── Config ─────────────────────────────────────────────────────────────────────

NAV_TIMEOUT_S = _env_float("COLLECT_ROUTE_NAV_TIMEOUT_S", 60.0)
# Sweep mode (log #21): one continuous drive-through route, collection fully
# decoupled — no stops, no fine approach, intake always on, beam counts.
_SWEEP_ARRIVE_M = 0.45
_SWEEP_CREDIT_MATCH_M = 1.5
# The crossing itself is driven by the MISSION, dead straight: Nav2's
# continuous path corrections near the ball slapped it away with the funnel
# cheeks (user observation, run 15). Steering is allowed only while further
# than _SWEEP_BLIND_M before the ball; inside that window the heading is
# frozen and the funnel does the centring.
_SWEEP_PASS_SPEED_M_S = _env_float("COLLECT_ROUTE_SWEEP_PASS_SPEED_M_S", 0.35)
_SWEEP_BLIND_M = 0.6
_SWEEP_PASS_TIMEOUT_S = 10.0
_SWEEP_PASS_ANGULAR_GAIN = 1.2
_SWEEP_PASS_MAX_ANGULAR_RAD_S = 0.5
# After the first run-in, links between funnel crossings stay mission-owned.
# Sending another NavigateToPose here made Nav2 stop and perform a final-yaw
# rotation before every ball.  The link controller always has forward motion;
# it bends the route into the following run-in instead.
_SWEEP_LINK_SPEED_M_S = _env_float("COLLECT_ROUTE_SWEEP_LINK_SPEED_M_S", 0.35)
_SWEEP_LINK_ARRIVE_M = 0.18
NAV_RETRIES = int(_env_float("COLLECT_ROUTE_NAV_RETRIES", 0))
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
# Dynamic-plan mode may refresh the nav goal when the mapped ball drifted this
# far from the stop's planned position. Frozen initial-plan mode deliberately
# keeps the scan-time position and goal unchanged for the whole Nav2 leg.
_GOAL_REFRESH_DRIFT_M = 0.3
# ...but an entry that wandered this far from the PLAN-time position is not
# the same physical ball any more (run-4 stop 6: chain-merges dragged the
# entry 4+ m across the court and the approach followed it) — drop the stop.
_GOAL_DRIFT_ABANDON_M = 1.5
# Opportunistic capture during Nav2 legs: Nav2 cannot see balls (lidar plane
# is above them) so legs plow straight through — runs 5-7 froze with a ball
# suspected under the chassis. A ball visible ahead within this range/bearing
# is collected on the spot, then the leg resumes. PLAN-ONLY: the ball must
# belong to a pending/active stop — anything else is ignored while the initial
# 360° plan is frozen.
_OPP_CAPTURE_RANGE_M = 1.2
_OPP_CAPTURE_BEARING_RAD = math.radians(40.0)
_OPP_TIMEOUT_S = 15.0
# An opportunistic ball matches the plan stop whose planned ball lies within
# this distance.
_OPP_STOP_MATCH_M = 0.8
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

    def __init__(
        self,
        planner_cfg: RoutePlannerConfig | None = None,
        *,
        freeze_initial_plan: bool | None = None,
        sweep: bool | None = None,
    ) -> None:
        self.planner_cfg = planner_cfg or RoutePlannerConfig.from_env()
        self.freeze_initial_plan = (
            _env_bool("COLLECT_ROUTE_FREEZE_INITIAL_PLAN", True)
            if freeze_initial_plan is None
            else freeze_initial_plan
        )
        # `collect_route` is a route traversal, not a sequence of capture
        # attempts.  Do not let an environment setting silently restore the
        # old stop-and-wait mission.  The explicit argument remains solely for
        # the legacy pure-FSM tests while that code is being retired.
        self.sweep = True if sweep is None else sweep
        self._sweep_confirm_latched = False
        self._pass_active = False
        self._link_active = False
        self._pass_elapsed_s = 0.0
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
        self._nav_seen_running: bool = False
        self._nav_attempts: int = 0
        self._opp_locked: tuple[float, float] | None = None
        self._opp_stop_id: int | None = None
        self._opp_elapsed_s: float = 0.0
        self._approach_elapsed_s: float = 0.0
        self._missing_scan_elapsed_s: float = 0.0
        self._live_seen_in_approach: bool = False
        self._capture_pending_reported_for: int | None = None
        self._locked_world: tuple[float, float] | None = None
        self._settle_remaining_s: float = 0.0
        self._events: list[tuple[str, dict]] = []
        self._complete_reported = False

    def reset(self) -> None:
        self.__init__(
            self.planner_cfg,
            freeze_initial_plan=self.freeze_initial_plan,
            sweep=self.sweep,
        )

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
    def capture_ball_id(self) -> int | None:
        """Route stop currently feeding the intake, including opportunistic capture.

        Only capture phases own the intake: during a Nav2 leg no capture is
        intended, and binding a stray ball bumped onboard mid-leg to the
        not-yet-reached target stop would credit that stop for the wrong ball.
        """
        if self.phase == "opportunistic":
            return self._opp_stop_id
        if self.phase in ("approach", "settle") and self.current_index < len(self.stops):
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

    def _route_snapshot(self) -> list[dict[str, object]]:
        return [
            {
                "ball_id": s.ball_id,
                "order": s.order,
                "status": s.status,
                "ball_x_m": round(s.ball_x_m, 3),
                "ball_y_m": round(s.ball_y_m, 3),
                "goal_x_m": round(s.approach.x_m, 3),
                "goal_y_m": round(s.approach.y_m, 3),
                "goal_yaw_rad": round(s.approach.yaw_rad, 3),
                "attempts": s.attempts,
            }
            for s in self.stops
        ]

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
                s.ball_id
                for s in self.stops
                if s.status in ("skipped", "missing", "swept")
            ],
            "stops": counts,
            "insertions": self.insertion_count,
            "freeze_initial_plan": self.freeze_initial_plan,
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
                    "ball_x_m": round(s.ball_x_m, 3),
                    "ball_y_m": round(s.ball_y_m, 3),
                    "goal_x_m": round(s.approach.x_m, 3),
                    "goal_y_m": round(s.approach.y_m, 3),
                    "attempts": s.attempts,
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
        confirmed_ball_id: int | None = None,
        capture_pending_ball_id: int | None = None,
    ) -> ConceptACommand:
        self.current_blocker = None
        if self.phase == "idle":
            self.start(robot_pose)

        if (
            not self.freeze_initial_plan
            and self.phase in ("nav", "approach", "settle")
        ):
            self._insert_new_balls(ball_map, robot_pose, court, now)

        if self.phase == "scan":
            return self._scan_phase(robot_pose, ball_map, now)
        if self.phase == "plan":
            return self._plan_phase(robot_pose, ball_map, court, now)
        if self.phase == "nav":
            return self._nav_phase(
                observation,
                dt_s,
                robot_pose,
                behavior,
                ball_map,
                nav_state,
                now,
                court,
                collection_confirmed,
                confirmed_ball_id,
            )
        if self.phase == "opportunistic":
            return self._opportunistic_phase(
                observation,
                collection_confirmed,
                dt_s,
                robot_pose,
                behavior,
                ball_map,
                confirmed_ball_id,
            )
        if self.phase == "approach":
            return self._approach_phase(
                observation,
                collection_confirmed,
                dt_s,
                robot_pose,
                behavior,
                ball_map,
                now,
                confirmed_ball_id,
                capture_pending_ball_id,
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
        if self.sweep:
            # Sweep mode (log #21): the Nav2 goal of every leg is the EXIT
            # pose PAST the ball — the funnel crosses the ball mid-leg. No
            # stop is ever made; collection is counted, never waited for.
            legs = sweep_route((robot_x, robot_y), balls, court, self.planner_cfg)
            ordered_ids = [leg.ball_id for leg in legs]
            self.stops = [
                RouteStop(
                    ball_id=leg.ball_id,
                    ball_x_m=leg.ball_x_m,
                    ball_y_m=leg.ball_y_m,
                    approach=ApproachPose(
                        x_m=leg.exit_x_m,
                        y_m=leg.exit_y_m,
                        yaw_rad=leg.yaw_rad,
                        mode=leg.mode,
                        risk=leg.risk,
                    ),
                    order=order,
                    sweep_entry_x_m=leg.entry_x_m,
                    sweep_entry_y_m=leg.entry_y_m,
                )
                for order, leg in enumerate(legs, start=1)
            ]
        else:
            ordered_ids = order_route((robot_x, robot_y), balls, self.planner_cfg)
            self.stops = []
            prev_xy = (robot_x, robot_y)
            for order, ball_id in enumerate(ordered_ids, start=1):
                ball_xy = by_id[ball_id]
                approach = approach_pose_for_ball(
                    ball_xy, prev_xy, court, self.planner_cfg
                )
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
            frozen=self.freeze_initial_plan,
            sweep=self.sweep,
            planned_order=self._route_snapshot(),
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
        if self.sweep:
            # Nav2 drives only to the ENTRY of the run-in; the crossing
            # itself is mission-driven, dead straight (no cheek slaps).
            ex, ey = self._sweep_entry_xy(stop)
            self._nav_goal = (ex, ey, stop.approach.yaw_rad)
            self._pass_active = False
        else:
            self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
        self._nav_elapsed_s = 0.0
        self._nav_last_state = "idle"
        self._nav_seen_running = False
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
        collection_confirmed: bool = False,
        confirmed_ball_id: int | None = None,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if self.sweep:
            return self._sweep_drive(
                dt_s, robot_pose, behavior, ball_map, nav_state, collection_confirmed
            )

        # A retention that completes mid-leg (ball launched at the tail of an
        # aborted capture settles after the roller stopped — run 8) must still
        # land in the ledger. Only EXPLICIT ground-truth attributions credit
        # here; an ownerless confirm never defaults to the untouched current
        # stop the way the approach-phase confirm does.
        if collection_confirmed and confirmed_ball_id is not None:
            credited = self._credit_confirmed_stop(confirmed_ball_id)
            if credited is not None:
                ball_map.set_state(credited, "collected")
                self._emit(
                    "route_delayed_collection_attributed",
                    ball_id=credited,
                    resumed_stop=stop.ball_id,
                    phase="nav",
                )
                if credited == stop.ball_id:
                    # The current leg's ball is already in the basket: skip
                    # the drive to its ghost standoff and move on.
                    self._advance("collected")
                    return _NAV_IDLE_CMD

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
            if matched is not None and matched is not stop:
                # Route rule R2: NO chase capture. Promote the on-path stop to
                # be the NEXT stop and take it with the normal standoff +
                # straight fine approach. The old opportunistic chase captured
                # at speed: it punted balls metres away (run 9, 5 misses) and
                # bounced launches back out of the basket (run 10, 5 false
                # beam credits).
                self.stops.remove(matched)
                self.stops.insert(self.current_index, matched)
                stop.status = "pending"
                matched.status = "active"
                self._renumber()
                self._emit(
                    "route_on_path_promoted",
                    ball_id=matched.ball_id,
                    postponed_stop=stop.ball_id,
                    distance_m=observation.distance_m,
                )
                self._enter_nav_retry(matched)
                return _NAV_IDLE_CMD

        # Dynamic-plan mode follows map refinements. Frozen initial-plan mode
        # keeps both the scan-time ball position and its Nav2 goal immutable;
        # live observations are adopted later by the fine-approach lock.
        entry = ball_map.balls.get(stop.ball_id)
        if not self.freeze_initial_plan and entry is not None and math.hypot(
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
        if not self.freeze_initial_plan and entry is not None and math.hypot(
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
            self._nav_seen_running = False
            return _NAV_IDLE_CMD

        self._nav_elapsed_s += dt_s
        if nav_state in ("pending", "active"):
            self._nav_seen_running = True
        # A failed result from the previous stop can remain visible for one
        # controller tick after advancing. Accept failure only after this goal
        # has entered Nav2's running states, preventing cascade-skips.
        failed = (
            nav_state == "failed"
            and self._nav_seen_running
            and self._nav_last_state != "failed"
        )
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
        self._capture_pending_reported_for = None
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
        confirmed_ball_id: int | None,
        capture_pending_ball_id: int | None,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if collection_confirmed:
            credited = self._credit_confirmed_stop(confirmed_ball_id or stop.ball_id)
            if credited is not None:
                ball_map.set_state(credited, "collected")
            if credited == stop.ball_id:
                behavior.reset()
                self.phase = "settle"
                self._settle_remaining_s = _SETTLE_HOLD_S
                self._emit("route_ball_collected", ball_id=stop.ball_id, order=stop.order)
                return ConceptACommand(
                    state=CollectorState.COLLECTED,
                    base=BaseCommand(0.0, 0.0),
                    collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
                )
            # Delayed retention of an EARLIER ball settled mid-approach: credit
            # it and keep the current capture untouched — resetting the behavior
            # here restarted ALIGN/APPROACH from scratch against an un-reset
            # 35 s budget and threw away live capture progress.
            self._emit(
                "route_delayed_collection_attributed",
                ball_id=credited,
                resumed_stop=stop.ball_id,
                phase="approach",
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
        # incident (archive/collection-route-debug-log-el #6): the turn-toward-target
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
        if not self._live_seen_in_approach and self._approach_elapsed_s > MISSING_SCAN_S:
            if capture_pending_ball_id == stop.ball_id:
                if self._capture_pending_reported_for != stop.ball_id:
                    self._capture_pending_reported_for = stop.ball_id
                    self._emit(
                        "route_missing_deferred",
                        ball_id=stop.ball_id,
                        reason="onboard_capture_pending_retention",
                    )
            else:
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

    # ── SWEEP DRIVE (log #21: collection decoupled from the route) ─────────────

    def _sweep_cmd(self, behavior: ConceptACollectorBehavior) -> ConceptACommand:
        """Nav2 owns the base; the intake runs for the whole route."""
        return ConceptACommand(
            state=CollectorState.SURVEY,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
        )

    def _credit_nearest_sweep_ball(
        self, robot_pose: tuple[float, float, float], ball_map: BallMap
    ) -> None:
        """A beam crossing belongs to the closest un-collected planned ball.

        Reporting only — the route never waits for or branches on a credit."""
        robot_x, robot_y, _ = robot_pose
        best: RouteStop | None = None
        best_d = _SWEEP_CREDIT_MATCH_M
        for s in self.stops:
            if s.status == "collected":
                continue
            d = math.hypot(s.ball_x_m - robot_x, s.ball_y_m - robot_y)
            if d < best_d:
                best, best_d = s, d
        if best is None:
            return
        best.status = "collected"
        ball_map.set_state(best.ball_id, "collected")
        self._renumber()
        self._emit(
            "route_ball_collected",
            ball_id=best.ball_id,
            order=best.order,
            match_distance_m=best_d,
        )

    def _sweep_drive(
        self,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        nav_state: str,
        collection_confirmed: bool,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        # Rising edge only: the beam latch stays True for the whole crossing.
        if collection_confirmed and not self._sweep_confirm_latched:
            self._credit_nearest_sweep_ball(robot_pose, ball_map)
        self._sweep_confirm_latched = collection_confirmed

        if self._pass_active:
            return self._sweep_pass_tick(dt_s, robot_pose, behavior, ball_map)
        if self._link_active:
            return self._sweep_link_tick(dt_s, robot_pose, behavior, ball_map)

        if nav_state == "unavailable":
            self.current_blocker = "nav2_action_unavailable"
            return self._sweep_cmd(behavior)

        if self._nav_goal is None:
            ex, ey = self._sweep_entry_xy(stop)
            self._nav_goal = (ex, ey, stop.approach.yaw_rad)
            self._nav_elapsed_s = 0.0
            self._nav_last_state = "idle"
            self._nav_seen_running = False
            return self._sweep_cmd(behavior)

        self._nav_elapsed_s += dt_s
        if nav_state in ("pending", "active"):
            self._nav_seen_running = True
        failed = (
            nav_state == "failed"
            and self._nav_seen_running
            and self._nav_last_state != "failed"
        )
        timed_out = self._nav_elapsed_s > NAV_TIMEOUT_S
        self._nav_last_state = nav_state

        robot_x, robot_y, _ = robot_pose
        entry_x, entry_y = self._sweep_entry_xy(stop)
        at_entry = nav_state == "reached" or (
            math.hypot(robot_x - entry_x, robot_y - entry_y) <= _SWEEP_ARRIVE_M
        )
        if at_entry:
            # Hand the base to the mission for the crossing: cancel the Nav2
            # goal and drive the run-in dead straight.
            self._pass_active = True
            self._pass_elapsed_s = 0.0
            self._nav_goal = None
            self._emit("route_pass_start", ball_id=stop.ball_id, order=stop.order)
            return self._sweep_pass_tick(dt_s, robot_pose, behavior, ball_map)

        if failed or timed_out:
            self._emit(
                "route_leg_skip",
                ball_id=stop.ball_id,
                reason="nav_timeout" if timed_out else "nav_failed",
            )
            self._skip_current(ball_map, "skipped")
            return self._sweep_cmd(behavior)

        return self._sweep_cmd(behavior)

    def _sweep_entry_xy(self, stop: RouteStop) -> tuple[float, float]:
        if stop.sweep_entry_x_m is not None and stop.sweep_entry_y_m is not None:
            return stop.sweep_entry_x_m, stop.sweep_entry_y_m
        hx = math.cos(stop.approach.yaw_rad)
        hy = math.sin(stop.approach.yaw_rad)
        return (
            stop.ball_x_m - hx * SWEEP_RUN_IN_M,
            stop.ball_y_m - hy * SWEEP_RUN_IN_M,
        )

    def _sweep_pass_tick(
        self,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
    ) -> ConceptACommand:
        """Mission-driven straight crossing over the ball.

        Nav2 corrections near the ball slapped it away with the funnel cheeks;
        here the heading is corrected only while the ball is still further
        than _SWEEP_BLIND_M ahead, then frozen — the funnel does the rest."""
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD
        robot_x, robot_y, robot_yaw = robot_pose
        hx = math.cos(stop.approach.yaw_rad)
        hy = math.sin(stop.approach.yaw_rad)
        # Signed progress along the crossing axis, zero AT the ball.
        progress = (robot_x - stop.ball_x_m) * hx + (robot_y - stop.ball_y_m) * hy
        self._pass_elapsed_s += dt_s

        if progress >= SWEEP_OVERRUN_M or self._pass_elapsed_s > _SWEEP_PASS_TIMEOUT_S:
            self._pass_active = False
            previous_status = stop.status
            if stop.status == "active":
                stop.status = "swept"
                previous_status = "swept"
                self._emit("route_ball_swept", ball_id=stop.ball_id, order=stop.order)
            self._advance_sweep(previous_status)
            if self._link_active:
                return self._sweep_link_tick(dt_s, robot_pose, behavior, ball_map)
            return self._sweep_cmd(behavior)

        angular = 0.0
        if progress < -_SWEEP_BLIND_M:
            # Still far from the ball: small correction toward the ball point.
            bearing = _angle_delta(
                math.atan2(stop.ball_y_m - robot_y, stop.ball_x_m - robot_x),
                robot_yaw,
            )
            angular = max(
                -_SWEEP_PASS_MAX_ANGULAR_RAD_S,
                min(
                    _SWEEP_PASS_MAX_ANGULAR_RAD_S,
                    bearing * _SWEEP_PASS_ANGULAR_GAIN,
                ),
            )
        return ConceptACommand(
            state=CollectorState.APPROACH,
            base=BaseCommand(_SWEEP_PASS_SPEED_M_S, angular),
            collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
        )

    def _advance_sweep(self, previous_status: str) -> None:
        """Continue the fixed route without handing the next crossing to Nav2.

        The first entry is Nav2-owned for obstacle-aware access.  Afterwards
        the route is a continuous, forward-moving curve from an exit to the
        next run-in, so a new Nav2 final-pose rotation cannot interrupt it.
        """
        previous = self._current_stop()
        self.current_index += 1
        while self.current_index < len(self.stops) and self.stops[
            self.current_index
        ].status not in ("pending",):
            self.current_index += 1
        if self.current_index >= len(self.stops):
            self._finish()
            return

        next_stop = self.stops[self.current_index]
        next_stop.status = "active"
        self.phase = "nav"
        self._nav_goal = None
        self._link_active = True
        self._emit(
            "route_advance",
            previous_ball_id=previous.ball_id if previous is not None else None,
            previous_status=previous_status,
            next_ball_id=next_stop.ball_id,
            next_order=next_stop.order,
            remaining=sum(
                1 for s in self.stops[self.current_index:] if s.status == "pending"
            ),
            continuous=True,
        )

    def _sweep_link_tick(
        self,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
    ) -> ConceptACommand:
        """Forward-only curved link from one crossing to the next run-in."""
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD
        robot_x, robot_y, robot_yaw = robot_pose
        entry_x, entry_y = self._sweep_entry_xy(stop)
        distance = math.hypot(entry_x - robot_x, entry_y - robot_y)
        if distance <= _SWEEP_LINK_ARRIVE_M:
            self._link_active = False
            self._pass_active = True
            self._pass_elapsed_s = 0.0
            self._emit("route_pass_start", ball_id=stop.ball_id, order=stop.order)
            return self._sweep_pass_tick(dt_s, robot_pose, behavior, ball_map)

        bearing = _angle_delta(math.atan2(entry_y - robot_y, entry_x - robot_x), robot_yaw)
        angular = max(
            -_SWEEP_PASS_MAX_ANGULAR_RAD_S,
            min(_SWEEP_PASS_MAX_ANGULAR_RAD_S, bearing * _SWEEP_PASS_ANGULAR_GAIN),
        )
        # Never stop to turn.  Even a sharp join creeps forward while the
        # bounded steering curve brings the funnel onto the next run-in.
        linear = _SWEEP_LINK_SPEED_M_S * max(0.25, math.cos(bearing))
        return ConceptACommand(
            state=CollectorState.APPROACH,
            base=BaseCommand(linear, angular),
            collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
        )

    def _enter_nav_retry(self, stop: RouteStop) -> None:
        self.phase = "nav"
        self._nav_goal = (stop.approach.x_m, stop.approach.y_m, stop.approach.yaw_rad)
        self._nav_elapsed_s = 0.0
        self._nav_last_state = "idle"
        self._nav_seen_running = False

    def _opportunistic_phase(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
        ball_map: BallMap,
        confirmed_ball_id: int | None,
    ) -> ConceptACommand:
        stop = self._current_stop()
        if stop is None:
            self._finish()
            return _IDLE_CMD

        if collection_confirmed:
            delayed = (
                confirmed_ball_id is not None
                and confirmed_ball_id != self._opp_stop_id
            )
            credited = self._credit_confirmed_stop(
                confirmed_ball_id or self._opp_stop_id
            )
            if credited is not None:
                ball_map.set_state(credited, "collected")
            self._emit(
                "route_opportunistic_collected",
                ball_id=credited,
                resumed_stop=stop.ball_id,
                delayed_attribution=delayed,
            )
            if not delayed:
                behavior.reset()
                self._opp_locked = None
                if stop.status == "collected":
                    # The opportunistic ball WAS the current stop's ball.
                    self.phase = "settle"
                    self._settle_remaining_s = _SETTLE_HOLD_S
                else:
                    self._enter_nav_retry(stop)
                return _IDLE_CMD
            # Delayed retention of an EARLIER ball: credit it and keep chasing
            # the opportunistic ball — dropping the lock here re-issued the leg
            # while the live ball sat half-captured in the funnel.

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

    def _credit_confirmed_stop(self, ball_id: int | None) -> int | None:
        """Credit the stop that owned intake entry, even after a delayed settle."""
        for s in self.stops:
            if s.ball_id == ball_id and s.status != "collected":
                s.status = "collected"
                self._renumber()
                return s.ball_id
            if s.ball_id == ball_id:
                return s.ball_id
        return None

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
            self._emit(
                "route_stop_finalized",
                ball_id=stop.ball_id,
                order=stop.order,
                status=status,
                attempts=max(stop.attempts, self._nav_attempts),
                nav_attempts=self._nav_attempts,
                collection_attempts=stop.attempts,
            )
        self._advance(status)

    def _advance(self, previous_status: str = "collected") -> None:
        previous = self.stops[self.current_index] if self.current_index < len(self.stops) else None
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
            next_stop = self.stops[self.current_index]
            self._emit(
                "route_advance",
                previous_ball_id=previous.ball_id if previous is not None else None,
                previous_status=previous_status,
                next_ball_id=next_stop.ball_id,
                next_order=next_stop.order,
                remaining=sum(
                    1 for s in self.stops[self.current_index:] if s.status == "pending"
                ),
            )
            self._enter_nav()

    def _finish(self) -> None:
        """The plan ledger is exhausted: every planned ball is accounted for
        as collected or failed. Completion is declared here and only here."""
        self.phase = "done"
        self._nav_goal = None
        # "swept" = crossed without a beam credit: the ball stayed on court.
        failed = [
            s.ball_id
            for s in self.stops
            if s.status in ("skipped", "missing", "swept")
        ]
        self._emit(
            "route_complete",
            planned_total=len(self.stops),
            collected=sum(1 for s in self.stops if s.status == "collected"),
            skipped=sum(1 for s in self.stops if s.status == "skipped"),
            missing=sum(1 for s in self.stops if s.status == "missing"),
            swept_uncollected=sum(1 for s in self.stops if s.status == "swept"),
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
