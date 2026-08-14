"""Pure, bounded adaptive-approach candidate generator (SHADOW / OFFLINE ONLY).

This module is part of the offline adaptive collection-route analysis.  It is
**deliberately not wired into the live planner**: ``plan_collection_route`` never
imports it, and it never changes the live route, the production tuning, or the
immutable ``CollectionRoutePlan`` contract.  A static regression test
(``tests/test_adaptive_approach_no_live_wiring.py``) enforces that.

For every snapshot ball it can produce, for each already-valid Phase-3A heading:

1. the existing baseline candidate exactly as today (always kept),
2. a small, bounded set of alternative approach *gate* distances (a longer
   straight run-in that may unlock a cheaper connector),
3. optional capture-safe lateral centreline offsets (bounded by the already
   computed effective capture half-width),
4. a mandatory final straight alignment corridor (every candidate here is a
   straight funnel pass, so the corridor is intrinsic; adaptive candidates whose
   run-in is shorter than the calibrated corridor are rejected — no curve is
   ever allowed inside the corridor),
5. the same run-out contract as the baseline (never shorter).

Generation is a discrete cross product of ``{gate distances} x {lateral
offsets}`` on the finite Phase-3A heading set — it is bounded, deterministic, and
contains no continuous/unbounded optimiser.  Domination filtering
(:func:`pareto_filter`) then keeps only non-dominated candidates under explicit
per-heading and per-ball caps.  The baseline is pinned and never pruned; a
candidate is never marked unreachable merely because a cap was hit.

All geometry (collision, effective capture width) is delegated to the existing,
unmodified Phase-3A helpers, so a candidate accepted here obeys the same swept
collision contract as the live planner.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from tennis_robot.collection_capture_geometry import CaptureGeometry
from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    FunnelPassCandidate,
    PerBallFeasibility,
    PlannerInputError,
    _effective_capture_half_width,
    _segment_is_collision_free,
    _segment_polygon_distance,
    analyze_snapshot,
)
from tennis_robot.collection_route_types import (
    BallReasonCode,
    CollectionRouteConfiguration,
    Point2D,
    Pose2D,
    ScanSnapshot,
)


class AdaptiveApproachError(ValueError):
    """The adaptive-approach generator was called with invalid explicit input."""


@dataclass(frozen=True)
class AdaptiveApproachConfiguration:
    """Bounded, explicit adaptive-approach search parameters.

    ``additional_gate_distances_m`` are extra base-frame run-in lengths tried in
    addition to (never replacing) the production ``minimum_run_in_m`` baseline;
    each must be strictly greater than the baseline.  ``lateral_offsets_m`` are
    signed centreline offsets (``0.0`` optional; the baseline already covers the
    zero offset).  The two caps bound how many candidates survive per heading and
    per ball.  There is no continuous range and no optimiser knob.
    """

    additional_gate_distances_m: tuple[float, ...]
    lateral_offsets_m: tuple[float, ...]
    max_candidates_per_heading: int
    max_candidates_per_ball: int

    def __post_init__(self) -> None:
        if not isinstance(self.additional_gate_distances_m, tuple):
            raise AdaptiveApproachError("additional_gate_distances_m must be a tuple")
        for value in self.additional_gate_distances_m:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
                raise AdaptiveApproachError("gate distances must be positive finite numbers")
        if len(set(self.additional_gate_distances_m)) != len(self.additional_gate_distances_m):
            raise AdaptiveApproachError("gate distances must be unique")
        if not isinstance(self.lateral_offsets_m, tuple):
            raise AdaptiveApproachError("lateral_offsets_m must be a tuple")
        for value in self.lateral_offsets_m:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise AdaptiveApproachError("lateral offsets must be finite numbers")
        if len(set(self.lateral_offsets_m)) != len(self.lateral_offsets_m):
            raise AdaptiveApproachError("lateral offsets must be unique")
        for name in ("max_candidates_per_heading", "max_candidates_per_ball"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise AdaptiveApproachError(f"{name} must be a positive int")


@dataclass(frozen=True)
class AdaptiveApproachCandidate:
    """A single adaptive candidate wrapping an unchanged ``FunnelPassCandidate``.

    The wrapper carries the adaptive metadata (gate distance, lateral offset,
    corridor lengths, capture margin, clearance, connector lower bound) used for
    Pareto filtering and reporting.  The embedded ``candidate`` is exactly the
    existing contract, so this never mutates ``CollectionRoutePlan``.
    """

    candidate: FunnelPassCandidate
    ball_id: str
    heading_rad: float
    is_baseline: bool
    approach_gate_distance_m: float
    lateral_offset_m: float
    alignment_corridor_m: float
    minimum_alignment_corridor_m: float
    capture_margin_m: float
    minimum_clearance_m: float
    connector_lower_bound_m: float
    pass_length_m: float

    @property
    def corridor_satisfied(self) -> bool:
        """Whether the straight run-in covers the calibrated alignment corridor."""
        return self.alignment_corridor_m >= self.minimum_alignment_corridor_m - 1e-9

    def dominance_vector(self) -> tuple[float, ...]:
        """Objectives as *lower-is-better* (larger clearance/margin negated)."""
        return (
            self.connector_lower_bound_m,
            self.pass_length_m,
            self.approach_gate_distance_m,
            -self.minimum_clearance_m,
            -self.capture_margin_m,
        )

    def sort_key(self) -> tuple:
        """Deterministic total order (baselines first, then dominance vector)."""
        return (
            0 if self.is_baseline else 1,
            self.connector_lower_bound_m,
            self.pass_length_m,
            self.approach_gate_distance_m,
            -self.minimum_clearance_m,
            -self.capture_margin_m,
            self.heading_rad,
            self.lateral_offset_m,
            self.candidate.ball_id,
        )


@dataclass(frozen=True)
class AdaptivePerBall:
    ball_id: str
    candidates: tuple[AdaptiveApproachCandidate, ...]
    unreachable_reason: BallReasonCode | None
    per_heading_budget_exhausted: bool
    per_ball_budget_exhausted: bool

    def __post_init__(self) -> None:
        if not self.ball_id:
            raise AdaptiveApproachError("ball_id must be non-empty")
        if self.candidates and self.unreachable_reason is not None:
            raise AdaptiveApproachError("reachable ball cannot carry an unreachable reason")
        if not self.candidates and self.unreachable_reason is None:
            raise AdaptiveApproachError("unreachable ball requires a deterministic reason")
        if not any(item.is_baseline for item in self.candidates) and self.candidates:
            raise AdaptiveApproachError("a reachable ball must retain its baseline candidate")


@dataclass(frozen=True)
class AdaptiveApproachResult:
    """Adaptive generation result with unambiguous candidate counters.

    The counters satisfy exact arithmetic invariants (validated in
    ``__post_init__``):

    * ``raw_candidate_count == raw_baseline_candidate_count +
      raw_adaptive_extra_candidate_count``
    * ``pareto_kept_total == pareto_kept_baseline_count +
      pareto_kept_adaptive_count``
    * ``pareto_pruned_count == raw_candidate_count - pareto_kept_total``

    "raw" counts are pre-filtering (every generated candidate); "kept" counts
    are the final per-ball candidates after per-heading and per-ball filtering.
    """

    per_ball: tuple[AdaptivePerBall, ...]
    capture_geometry: CaptureGeometry
    raw_baseline_candidate_count: int
    raw_adaptive_extra_candidate_count: int
    raw_candidate_count: int
    pareto_kept_baseline_count: int
    pareto_kept_adaptive_count: int
    pareto_kept_total: int
    pareto_pruned_count: int
    budget_exhausted: bool

    def __post_init__(self) -> None:
        if self.raw_candidate_count != self.raw_baseline_candidate_count + self.raw_adaptive_extra_candidate_count:
            raise AdaptiveApproachError("raw_candidate_count must equal baseline + adaptive extras")
        if self.pareto_kept_total != self.pareto_kept_baseline_count + self.pareto_kept_adaptive_count:
            raise AdaptiveApproachError("pareto_kept_total must equal kept baseline + kept adaptive")
        if self.pareto_pruned_count != self.raw_candidate_count - self.pareto_kept_total:
            raise AdaptiveApproachError("pareto_pruned_count must equal raw - kept total")


def generate_adaptive_candidates(
    *,
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
    capture_geometry: CaptureGeometry,
    adaptive: AdaptiveApproachConfiguration,
) -> AdaptiveApproachResult:
    """Deterministically produce the bounded, Pareto-filtered adaptive set.

    The baseline candidate for every valid heading is always present.  Unreachable
    balls mirror the Phase-3A deterministic reason exactly.
    """
    if not isinstance(snapshot, ScanSnapshot):
        raise AdaptiveApproachError("snapshot must be ScanSnapshot")
    if not isinstance(court, CourtModel):
        raise AdaptiveApproachError("court must be CourtModel")
    if not isinstance(configuration, CollectionRouteConfiguration):
        raise AdaptiveApproachError("configuration must be CollectionRouteConfiguration")
    if not isinstance(capture_geometry, CaptureGeometry):
        raise AdaptiveApproachError("capture_geometry must be CaptureGeometry")
    if not isinstance(adaptive, AdaptiveApproachConfiguration):
        raise AdaptiveApproachError("adaptive must be AdaptiveApproachConfiguration")
    # Fail loud on uncalibrated required geometry: adaptive generation must not
    # run on a corridor length that has not been established by intake trials.
    uncalibrated = capture_geometry.uncalibrated_fields()
    if uncalibrated:
        raise AdaptiveApproachError(
            "adaptive generation requires calibrated/configured capture geometry; "
            f"uncalibrated fields: {list(uncalibrated)}"
        )
    baseline_run_in = configuration.mechanical.minimum_run_in_m
    for gate in adaptive.additional_gate_distances_m:
        if gate <= baseline_run_in + 1e-12:
            raise AdaptiveApproachError(
                "additional gate distances must be strictly greater than the "
                "production minimum_run_in_m baseline"
            )

    feasibility = analyze_snapshot(snapshot, court, configuration)
    by_id = {ball.ball_id: ball for ball in snapshot.balls}
    corridor = capture_geometry.minimum_alignment_corridor_m

    per_ball: list[AdaptivePerBall] = []
    raw_baseline = 0
    raw_extra = 0
    kept_baseline = 0
    kept_adaptive = 0
    budget_exhausted = False

    for feas in feasibility:
        ball = by_id[feas.ball_id]
        if not feas.reachable:
            per_ball.append(AdaptivePerBall(feas.ball_id, (), feas.unreachable_reason, False, False))
            continue

        kept: list[AdaptiveApproachCandidate] = []
        heading_exhausted = False
        for base_candidate in feas.candidates:
            group = _heading_group(
                snapshot=snapshot,
                court=court,
                configuration=configuration,
                capture_geometry=capture_geometry,
                adaptive=adaptive,
                ball=ball,
                baseline=base_candidate,
                corridor=corridor,
            )
            raw_baseline += sum(1 for item in group if item.is_baseline)
            raw_extra += sum(1 for item in group if not item.is_baseline)
            filtered, exhausted = pareto_filter(group, cap=adaptive.max_candidates_per_heading)
            heading_exhausted = heading_exhausted or exhausted
            kept.extend(filtered)

        # Per-ball cap on the merged, per-heading-filtered set (baseline pinned).
        merged, ball_exhausted = pareto_filter(
            tuple(kept), cap=adaptive.max_candidates_per_ball, pinned_only=True
        )
        kept_baseline += sum(1 for item in merged if item.is_baseline)
        kept_adaptive += sum(1 for item in merged if not item.is_baseline)
        budget_exhausted = budget_exhausted or heading_exhausted or ball_exhausted
        per_ball.append(
            AdaptivePerBall(
                feas.ball_id,
                merged,
                None,
                heading_exhausted,
                ball_exhausted,
            )
        )

    raw_total = raw_baseline + raw_extra
    kept_total = kept_baseline + kept_adaptive
    return AdaptiveApproachResult(
        per_ball=tuple(per_ball),
        capture_geometry=capture_geometry,
        raw_baseline_candidate_count=raw_baseline,
        raw_adaptive_extra_candidate_count=raw_extra,
        raw_candidate_count=raw_total,
        pareto_kept_baseline_count=kept_baseline,
        pareto_kept_adaptive_count=kept_adaptive,
        pareto_kept_total=kept_total,
        pareto_pruned_count=raw_total - kept_total,
        budget_exhausted=budget_exhausted,
    )


def _heading_group(
    *,
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
    capture_geometry: CaptureGeometry,
    adaptive: AdaptiveApproachConfiguration,
    ball,
    baseline: FunnelPassCandidate,
    corridor: float,
) -> tuple[AdaptiveApproachCandidate, ...]:
    """Baseline + all valid (gate, lateral) variants for a single heading."""
    heading = baseline.heading_rad
    effective_width = _effective_capture_half_width(ball, heading, configuration)
    start = snapshot.robot_pose_at_scan
    clearance = configuration.feasibility.footprint_clearance_radius_m
    baseline_run_in = configuration.mechanical.minimum_run_in_m
    run_out = configuration.mechanical.minimum_run_out_m

    group: list[AdaptiveApproachCandidate] = []
    group.append(
        _wrap(
            candidate=baseline,
            ball=ball,
            heading=heading,
            is_baseline=True,
            gate=baseline_run_in,
            lateral=0.0,
            effective_width=effective_width,
            corridor=corridor,
            court=court,
            start=start,
            run_out=run_out,
        )
    )

    # Lateral offsets are an independent axis: they apply at the baseline run-in
    # AND at every additional gate.  The (baseline_run_in, 0.0) pair is the
    # baseline itself and is skipped in the variant loop.
    gates = sorted({baseline_run_in, *adaptive.additional_gate_distances_m})
    for gate in gates:
        # The mandatory final straight alignment corridor: no curve is allowed
        # inside it, so a gate shorter than the calibrated corridor is rejected
        # (only this candidate — never the ball).  The proven baseline run-in is
        # exempt: it is the current live behaviour and is always kept.
        if gate != baseline_run_in and gate < corridor - 1e-9:
            continue
        for lateral in sorted(adaptive.lateral_offsets_m):
            if lateral == 0.0 and gate == baseline_run_in:
                continue  # that is the baseline
            if abs(lateral) > effective_width + 1e-12:
                continue  # capture-safe bound: never exceed the effective corridor
            built = _build_variant(
                ball=ball,
                heading=heading,
                gate=gate,
                lateral=lateral,
                effective_width=effective_width,
                corridor=corridor,
                court=court,
                clearance=clearance,
                start=start,
                run_out=run_out,
            )
            if built is not None:
                group.append(built)
    return tuple(group)


def _build_variant(
    *,
    ball,
    heading: float,
    gate: float,
    lateral: float,
    effective_width: float,
    corridor: float,
    court: CourtModel,
    clearance: float,
    start: Pose2D,
    run_out: float,
) -> AdaptiveApproachCandidate | None:
    direction = (math.cos(heading), math.sin(heading))
    normal = (-math.sin(heading), math.cos(heading))
    # Offset centreline: the crossing centreline point is the ball shifted by the
    # lateral offset along the pass normal, so the ball sits |lateral| off centre.
    crossing_centre = Point2D(
        ball.position.x_m + lateral * normal[0],
        ball.position.y_m + lateral * normal[1],
    )
    entry = Point2D(crossing_centre.x_m - gate * direction[0], crossing_centre.y_m - gate * direction[1])
    exit_point = Point2D(crossing_centre.x_m + run_out * direction[0], crossing_centre.y_m + run_out * direction[1])
    # Whole approach corridor must pass the existing swept collision contract.
    if not _segment_is_collision_free(entry, crossing_centre, court, clearance):
        return None
    if not _segment_is_collision_free(crossing_centre, exit_point, court, clearance):
        return None
    candidate = FunnelPassCandidate(
        ball.ball_id,
        (ball.ball_id,),
        heading,
        Pose2D(entry.x_m, entry.y_m, heading),
        crossing_centre,
        Pose2D(exit_point.x_m, exit_point.y_m, heading),
        effective_width,
        (ball.position,),
    )
    return _wrap(
        candidate=candidate,
        ball=ball,
        heading=heading,
        is_baseline=False,
        gate=gate,
        lateral=lateral,
        effective_width=effective_width,
        corridor=corridor,
        court=court,
        start=start,
        run_out=run_out,
    )


def _wrap(
    *,
    candidate: FunnelPassCandidate,
    ball,
    heading: float,
    is_baseline: bool,
    gate: float,
    lateral: float,
    effective_width: float,
    corridor: float,
    court: CourtModel,
    start: Pose2D,
    run_out: float,
) -> AdaptiveApproachCandidate:
    entry = candidate.entry_pose
    exit_pose = candidate.exit_pose
    connector_lower_bound = math.hypot(entry.x_m - start.x_m, entry.y_m - start.y_m)
    pass_length = math.hypot(
        candidate.crossing.x_m - entry.x_m, candidate.crossing.y_m - entry.y_m
    ) + math.hypot(
        exit_pose.x_m - candidate.crossing.x_m, exit_pose.y_m - candidate.crossing.y_m
    )
    return AdaptiveApproachCandidate(
        candidate=candidate,
        ball_id=ball.ball_id,
        heading_rad=heading,
        is_baseline=is_baseline,
        approach_gate_distance_m=gate,
        lateral_offset_m=lateral,
        alignment_corridor_m=gate,
        minimum_alignment_corridor_m=corridor,
        capture_margin_m=effective_width - abs(lateral),
        minimum_clearance_m=_corridor_clearance(entry, exit_pose, court),
        connector_lower_bound_m=connector_lower_bound,
        pass_length_m=pass_length,
    )


def _corridor_clearance(entry: Pose2D, exit_pose: Pose2D, court: CourtModel) -> float:
    """Minimum distance from the straight approach corridor to any boundary.

    Uses the same segment/polygon distance helper the collision check relies on,
    so the reported clearance and the accept/reject verdict come from one source.
    """
    start = Point2D(entry.x_m, entry.y_m)
    end = Point2D(exit_pose.x_m, exit_pose.y_m)
    distances = [_segment_polygon_distance(start, end, court.navigable_polygon)]
    for obstacle in court.obstacles:
        distances.append(_segment_polygon_distance(start, end, obstacle.polygon))
    return min(distances)


def pareto_filter(
    candidates: tuple[AdaptiveApproachCandidate, ...],
    *,
    cap: int,
    pinned_only: bool = False,
) -> tuple[tuple[AdaptiveApproachCandidate, ...], bool]:
    """Deterministic Pareto filtering with a hard cap; baselines are pinned.

    Returns ``(kept, budget_exhausted)``.  A candidate is dropped only if another
    *strictly dominates* it (never worse on any objective, strictly better on at
    least one).  Baseline candidates are always retained.  If the non-dominated
    set exceeds ``cap`` the extra non-baseline candidates are trimmed by the
    deterministic :meth:`AdaptiveApproachCandidate.sort_key` and
    ``budget_exhausted`` is set — this is a candidate-budget signal only, never a
    reason to call a ball unreachable.

    ``pinned_only`` skips the dominance pass and applies the cap alone; used for
    the per-ball merge of already-per-heading-filtered candidates.
    """
    if cap <= 0:
        raise AdaptiveApproachError("cap must be positive")
    if not candidates:
        return (), False

    ordered = sorted(candidates, key=lambda item: item.sort_key())
    if pinned_only:
        survivors = list(ordered)
    else:
        survivors = []
        for candidate in ordered:
            if candidate.is_baseline:
                survivors.append(candidate)
                continue
            dominated = any(
                _dominates(other, candidate)
                for other in ordered
                if other is not candidate
            )
            if not dominated:
                survivors.append(candidate)

    baselines = [item for item in survivors if item.is_baseline]
    others = [item for item in survivors if not item.is_baseline]
    if len(baselines) >= cap:
        # Never drop a baseline; if baselines alone exceed the cap keep them all.
        return tuple(baselines), len(baselines) > cap or bool(others)
    keep_others = others[: cap - len(baselines)]
    exhausted = len(others) > len(keep_others)
    kept = sorted(baselines + keep_others, key=lambda item: item.sort_key())
    return tuple(kept), exhausted


def _dominates(a: AdaptiveApproachCandidate, b: AdaptiveApproachCandidate) -> bool:
    va, vb = a.dominance_vector(), b.dominance_vector()
    if any(x > y + 1e-12 for x, y in zip(va, vb)):
        return False
    return any(x < y - 1e-12 for x, y in zip(va, vb))


# ── Adaptive shared passes (shadow only) ─────────────────────────────────────
@dataclass(frozen=True)
class AdaptiveSharedPassResult:
    """Adaptive shared-pass candidates that preserve physical ball positions."""

    candidates: tuple[FunnelPassCandidate, ...]
    candidate_budget_exhausted: bool
    rejections: tuple[tuple[tuple[str, ...], str], ...]


def generate_adaptive_shared_passes(
    *,
    single_ball_candidates: tuple[FunnelPassCandidate, ...],
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> AdaptiveSharedPassResult:
    """Shadow-only shared-pass generator that keeps physical ball positions.

    The production ``generate_shared_passes`` reconstructs the shared crossing
    positions from ``member.crossing`` (the pass centreline).  For adaptive
    single-ball candidates the centreline is laterally *shifted* away from the
    ball, so the production reconstruction would report each ball at a synthetic
    shifted location.  This generator instead:

    * takes the physical ball position from ``member.crossing_positions[0]``;
    * uses the members' shifted centrelines only to *propose* candidate shared
      centreline offsets (plus the physical midpoint);
    * tests lateral feasibility against the *physical* positions and each
      member's own effective capture half-width;
    * emits one shared candidate per member combination, at the feasible
      centreline that maximises the minimum capture margin;
    * runs the existing swept collision check;
    * rejects (with an explicit reason) any combination that is not feasible or
      not representable by the ``FunnelPassCandidate`` contract, rather than
      fabricating geometry.

    Deterministic and bounded (combinations up to ``max_shared_pass_balls``,
    capped by ``max_shared_pass_candidates``).  It does NOT touch the production
    shared-pass implementation.
    """
    if not isinstance(court, CourtModel) or not isinstance(configuration, CollectionRouteConfiguration):
        raise AdaptiveApproachError("court and configuration are required")
    from itertools import combinations

    shared_cfg = configuration.shared_pass
    single = [c for c in single_ball_candidates if len(c.covered_ball_ids) == 1]

    by_heading: dict[float, dict[str, list[FunnelPassCandidate]]] = {}
    for candidate in single:
        ball_id = candidate.covered_ball_ids[0]
        by_heading.setdefault(candidate.heading_rad, {}).setdefault(ball_id, []).append(candidate)

    generated: list[FunnelPassCandidate] = []
    rejections: list[tuple[tuple[str, ...], str]] = []
    exhausted = False

    for heading in sorted(by_heading):
        direction = (math.cos(heading), math.sin(heading))
        normal = (-math.sin(heading), math.cos(heading))
        ball_map = by_heading[heading]
        ball_ids = sorted(ball_map)
        info: dict[str, _SharedMemberInfo] = {}
        for ball_id in ball_ids:
            variants = ball_map[ball_id]
            physical = variants[0].crossing_positions[0]
            eff = min(v.effective_capture_half_width_m for v in variants)
            proposed = sorted({_dot(v.crossing, normal) for v in variants})
            info[ball_id] = _SharedMemberInfo(
                ball_id=ball_id,
                physical=physical,
                effective_half_width=eff,
                longitudinal=_dot(physical, direction),
                lateral=_dot(physical, normal),
                proposed_centrelines=tuple(proposed),
            )
        max_size = min(shared_cfg.max_shared_pass_balls, len(ball_ids))
        for size in range(2, max_size + 1):
            for members in combinations(ball_ids, size):
                member_info = [info[ball_id] for ball_id in members]
                built, reason = _best_adaptive_shared_candidate(
                    member_info, direction, normal, court, configuration
                )
                if built is None:
                    rejections.append((members, reason or "infeasible"))
                    continue
                if len(generated) >= shared_cfg.max_shared_pass_candidates:
                    exhausted = True
                    continue
                generated.append(built)

    return AdaptiveSharedPassResult(tuple(generated), exhausted, tuple(rejections))


@dataclass(frozen=True)
class _SharedMemberInfo:
    ball_id: str
    physical: Point2D
    effective_half_width: float
    longitudinal: float
    lateral: float
    proposed_centrelines: tuple[float, ...]


def _best_adaptive_shared_candidate(
    members: list[_SharedMemberInfo],
    direction: tuple[float, float],
    normal: tuple[float, float],
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> tuple[FunnelPassCandidate | None, str | None]:
    ordered = sorted(members, key=lambda m: (m.longitudinal, m.ball_id))
    longitudinals = [m.longitudinal for m in ordered]
    spacing = configuration.shared_pass.minimum_mechanical_ball_spacing_m
    for previous, current in zip(longitudinals, longitudinals[1:]):
        if current - previous < spacing:
            return None, "mechanical_spacing"

    laterals = [m.lateral for m in ordered]
    midpoint = (min(laterals) + max(laterals)) / 2.0
    option_set = {midpoint} | {value for m in ordered for value in m.proposed_centrelines}
    # Width-aware feasibility interval (intersection of each member's
    # [lateral - width, lateral + width]).  Its centre is the margin-maximising
    # centreline; for ASYMMETRIC effective widths this is shifted away from the
    # physical midpoint, so add it explicitly — otherwise a genuinely feasible
    # asymmetric shared pass would be missed.
    lo = max(m.lateral - m.effective_half_width for m in ordered)
    hi = min(m.lateral + m.effective_half_width for m in ordered)
    if hi >= lo - 1e-12:
        option_set.add((lo + hi) / 2.0)
    options = sorted(option_set)

    best_candidate: FunnelPassCandidate | None = None
    best_key: tuple | None = None
    last_reason: str | None = None
    for centreline in options:
        margins = [m.effective_half_width - abs(m.lateral - centreline) for m in ordered]
        if any(margin < -1e-12 for margin in margins):
            last_reason = "lateral_exceeds_capture"
            continue
        candidate = _build_adaptive_shared_candidate(
            ordered, centreline, direction, normal, court, configuration
        )
        if isinstance(candidate, str):
            last_reason = candidate
            continue
        # Prefer the largest minimum margin; deterministic tie-break by |offset|.
        key = (-min(margins), abs(centreline), centreline)
        if best_key is None or key < best_key:
            best_key = key
            best_candidate = candidate
    if best_candidate is None:
        return None, last_reason
    return best_candidate, None


def _build_adaptive_shared_candidate(
    ordered: list[_SharedMemberInfo],
    centreline: float,
    direction: tuple[float, float],
    normal: tuple[float, float],
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> FunnelPassCandidate | str:
    run_in = configuration.mechanical.minimum_run_in_m
    run_out = configuration.mechanical.minimum_run_out_m
    clearance = configuration.feasibility.footprint_clearance_radius_m

    def point(longitudinal: float) -> Point2D:
        return Point2D(
            longitudinal * direction[0] + centreline * normal[0],
            longitudinal * direction[1] + centreline * normal[1],
        )

    first_long = ordered[0].longitudinal
    last_long = ordered[-1].longitudinal
    entry = point(first_long - run_in)
    exit_point = point(last_long + run_out)
    if not _segment_is_collision_free(entry, exit_point, court, clearance):
        return "collision"

    heading = math.atan2(direction[1], direction[0])
    covered = tuple(m.ball_id for m in ordered)
    # crossing_positions are the PHYSICAL ball positions, never the centreline.
    physical_positions = tuple(m.physical for m in ordered)
    crossing_centreline = point(first_long)
    try:
        return FunnelPassCandidate(
            "shared:" + "+".join(covered),
            covered,
            heading,
            Pose2D(entry.x_m, entry.y_m, heading),
            crossing_centreline,
            Pose2D(exit_point.x_m, exit_point.y_m, heading),
            min(m.effective_half_width for m in ordered),
            physical_positions,
        )
    except (PlannerInputError, ValueError):
        return "non_representable"


def _dot(point: Point2D, vector: tuple[float, float]) -> float:
    return point.x_m * vector[0] + point.y_m * vector[1]


# ── Shadow global solve (offline only) ──────────────────────────────────────
@dataclass(frozen=True)
class ShadowSolveResult:
    plan: object  # CollectionRoutePlan; typed loosely to avoid a hard import here
    graph_node_count: int
    graph_edge_count: int
    connector_rejection_histogram: dict[str, int]
    bounded_candidate_count: int
    # Independently known budget signals — these are the only ones we can
    # attribute to a single cause:
    global_candidate_cap_exhausted: bool
    shared_pass_budget_exhausted: bool
    shared_pass_rejections: tuple[tuple[tuple[str, ...], str], ...]
    # The production solver computes planning_search_status from
    # ``search_exhausted OR candidate_budget_exhausted`` (see
    # collection_route_global_solver.solve_global_route), so a value of
    # ``budget_exhausted`` cannot be decomposed into DFS-search vs candidate-cap
    # exhaustion.  It is surfaced as a COMBINED status, never as independent
    # DFS/search exhaustion.
    combined_planner_budget_status: str


def flatten_adaptive_candidates(
    result: AdaptiveApproachResult,
) -> tuple[FunnelPassCandidate, ...]:
    """All single-ball ``FunnelPassCandidate`` objects across the adaptive set."""
    return tuple(
        item.candidate
        for per_ball in result.per_ball
        for item in per_ball.candidates
    )


def shadow_global_solve(
    *,
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
    adaptive_result: AdaptiveApproachResult,
) -> ShadowSolveResult:
    """Run the existing pure graph + solver over the adaptive candidate pool.

    This is the ONLY adaptive path into the global solver, and it is offline:
    it reuses the unmodified pure ``build_directed_candidate_graph`` /
    ``solve_global_route`` functions.  Shared passes are regenerated by the
    shadow ``generate_adaptive_shared_passes`` (which preserves physical ball
    positions), not the production generator.

    The baseline single-ball candidates are *preserved* through candidate
    bounding: the ``maximum_candidate_count`` cap trims only the adaptive extras,
    never a baseline.  Without this, flooding the pool with adaptive variants can
    evict the well-connected baselines at the production cap and make the shadow
    route look worse than baseline for a pure bounding reason — a misleading
    artifact rather than a property of the adaptive candidates.  This mirrors the
    core invariant that the baseline candidate is always retained.
    """
    if not isinstance(adaptive_result, AdaptiveApproachResult):
        raise AdaptiveApproachError("adaptive_result must be AdaptiveApproachResult")
    # Belt-and-braces: an adaptive result can only be produced from calibrated
    # geometry, but never solve over uncalibrated geometry even if constructed
    # directly.
    uncalibrated = adaptive_result.capture_geometry.uncalibrated_fields()
    if uncalibrated:
        raise AdaptiveApproachError(
            "shadow_global_solve requires calibrated capture geometry; "
            f"uncalibrated fields: {list(uncalibrated)}"
        )
    baselines = tuple(
        item.candidate
        for per_ball in adaptive_result.per_ball
        for item in per_ball.candidates
        if item.is_baseline
    )
    extras = tuple(
        item.candidate
        for per_ball in adaptive_result.per_ball
        for item in per_ball.candidates
        if not item.is_baseline
    )
    return _solve_over_single_candidates(
        snapshot=snapshot,
        court=court,
        configuration=configuration,
        preserved=baselines,
        extra=extras,
        use_adaptive_shared=True,
    )


def baseline_shadow_solve(
    *,
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> ShadowSolveResult:
    """Reproduce the live baseline plan and expose its graph telemetry.

    Uses only the baseline Phase-3A single-ball candidates (no adaptive
    variants) with the production shared-pass generator and the same plain
    bounding as ``plan_collection_route``, so the returned plan is byte-identical
    to the live planner while additionally surfacing the connector edge rejection
    histogram and graph size.
    """
    individual = analyze_snapshot(snapshot, court, configuration)
    single = tuple(
        candidate
        for result in individual
        for candidate in result.candidates
        if len(candidate.covered_ball_ids) == 1
    )
    return _solve_over_single_candidates(
        snapshot=snapshot, court=court, configuration=configuration,
        preserved=(), extra=single, use_adaptive_shared=False,
    )


def _candidate_key(candidate: FunnelPassCandidate):
    """Identity key matching ``_merge_candidates`` deduplication."""
    return (
        candidate.covered_ball_ids,
        candidate.heading_rad,
        candidate.entry_pose,
        candidate.crossing,
        candidate.exit_pose,
    )


def _solve_over_single_candidates(
    *,
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
    preserved: tuple[FunnelPassCandidate, ...],
    extra: tuple[FunnelPassCandidate, ...],
    use_adaptive_shared: bool,
) -> ShadowSolveResult:
    from tennis_robot.collection_route_connector_graph import build_directed_candidate_graph
    from tennis_robot.collection_route_global_solver import solve_global_route
    from tennis_robot.collection_route_planner_v2 import (
        _bounded_candidates,
        _merge_candidates,
    )
    from tennis_robot.collection_route_shared_pass import generate_shared_passes

    individual = analyze_snapshot(snapshot, court, configuration)
    all_singles = preserved + extra
    if use_adaptive_shared:
        shared = generate_adaptive_shared_passes(
            single_ball_candidates=all_singles, court=court, configuration=configuration
        )
        shared_rejections = shared.rejections
    else:
        shared = generate_shared_passes(
            snapshot=snapshot, single_ball_candidates=all_singles, court=court, configuration=configuration
        )
        shared_rejections = ()
    pool = _merge_candidates(all_singles + shared.candidates)
    cap = configuration.planning.maximum_candidate_count

    if preserved:
        preserved_pool = _merge_candidates(preserved)
        preserved_keys = {_candidate_key(candidate) for candidate in preserved_pool}
        rest = tuple(candidate for candidate in pool if _candidate_key(candidate) not in preserved_keys)
        if len(preserved_pool) >= cap:
            # Never drop a baseline; keep them all and flag the overflow.
            bounded = preserved_pool
            cap_exhausted = len(preserved_pool) > cap or bool(rest)
        else:
            bounded_rest, cap_exhausted = _bounded_candidates(
                snapshot, rest, cap - len(preserved_pool)
            )
            bounded = _merge_candidates(preserved_pool + bounded_rest)
    else:
        bounded, cap_exhausted = _bounded_candidates(snapshot, pool, cap)
    graph = build_directed_candidate_graph(
        snapshot=snapshot,
        candidates=bounded,
        court=court,
        configuration=configuration,
    )
    plan = solve_global_route(
        snapshot=snapshot,
        feasibility=individual,
        graph=graph,
        court=court,
        configuration=configuration,
        candidate_budget_exhausted=cap_exhausted,
    )
    histogram: dict[str, int] = {}
    for edge in graph.edges:
        key = edge.rejection.value if edge.rejection is not None else "accepted"
        histogram[key] = histogram.get(key, 0) + 1
    return ShadowSolveResult(
        plan=plan,
        graph_node_count=len(graph.pass_nodes),
        graph_edge_count=len(graph.edges),
        connector_rejection_histogram=histogram,
        bounded_candidate_count=len(bounded),
        global_candidate_cap_exhausted=cap_exhausted,
        shared_pass_budget_exhausted=shared.candidate_budget_exhausted,
        shared_pass_rejections=shared_rejections,
        combined_planner_budget_status=plan.planning_search_status.value,
    )
