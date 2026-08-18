"""Bounded anytime search over collection passes.

The primitive here is the *pass*: a stretch of driving whose swept collector
footprint takes one or more balls.  Everything else -- clustering, macros,
expansion order -- exists to find a good sequence of passes sooner, and none of
it may decide which sequences exist.  Its predecessor made clusters atomic
routing units and lost executable routes outright (debug log #57): a tight
cluster whose internal connectors failed dropped every one of its balls, and a
starved candidate cap returned no route at all.

Two invariants are load-bearing and are maintained by construction rather than
by care:

*Best-so-far is never lost.*  Every node offers itself as a finished route the
moment it is created, before any pruning looks at it, and the incumbent is a
running minimum over a total order.  No cap, cluster, macro, dominance rule or
budget can turn a route that was found into no route at all.

*More budget is never worse.*  Expansion order is a pure function of the inputs,
so a smaller budget executes a strict prefix of a larger one, and a running
minimum over a prefix can only be matched or beaten by the full run.  The budget
is therefore allowed to stop the loop and nothing else: no branching, ordering,
pruning threshold or successor set may read it.
"""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
import math
import time

from tennis_robot.collection_route_connector_graph import ConnectorEdge, link_poses
from tennis_robot.collection_route_cost import RouteAccumulators, accumulated_cost, cost_value
from tennis_robot.collection_route_evidence import (
    BallEvidence,
    attribute_ball_results,
)
from tennis_robot.collection_route_plan_builder import (
    assemble_plan,
    connector_segment,
    empty_plan,
    pass_length,
    pass_segment,
    terminal_segment,
    transit_pass_segment,
)
from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    FunnelPassCandidate,
    PerBallFeasibility,
    _segment_is_collision_free,
)
from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    CollectionRoutePlan,
    PlanningSearchStatus,
    PlanningStatus,
    Point2D,
    Pose2D,
    ScanSnapshot,
    SuccessorBatchPolicy,
)

_START = -1
_COST_EPSILON = 1e-9

# Successors evaluated per visit, by how many candidates the problem admits.
# Measured on the 24-layout benchmark at equal wall clock (debug log #61): below
# ~140 candidates the pacing makes no difference to the result because the
# search finishes anyway, while above it a small batch spends the budget
# re-popping the same states -- 134 pops per state at batch 4 against 1.6 when a
# state is expanded in one go, for a coverage-matched length 7 points worse.
# Coverage was identical at every batch size in every group, so this trades
# nothing.
_ADAPTIVE_BATCH_STEPS: tuple[tuple[int, int], ...] = (
    (80, 4),      # small: any pacing works; keep the small batch that reaches
                  # a first route soonest
    (140, 16),    # medium
)
_ADAPTIVE_BATCH_LARGE = 1 << 30  # large: expand the state in one visit


def successor_batch_size(configuration: CollectionRouteConfiguration, candidate_count: int) -> int:
    """How many successors one visit evaluates.

    A pacing decision and nothing else: it may look at the size of the problem
    and at no part of the search.  Not the budget left, not the clock, not the
    incumbent, not which clusters exist -- any of those would make the batch a
    search policy and cost the determinism the rest of the router relies on.
    """
    search = configuration.global_route_search
    if search.successor_batch_policy is SuccessorBatchPolicy.FIXED:
        return search.successor_batch_size
    for limit, size in _ADAPTIVE_BATCH_STEPS:
        if candidate_count <= limit:
            return size
    return _ADAPTIVE_BATCH_LARGE


@dataclass(frozen=True)
class RouterResult:
    """The plan plus what the run itself did, which is not a route property."""

    plan: CollectionRoutePlan
    # One expansion is one successor evaluation, so the budget means the same
    # amount of work whatever the batch size is.
    expansions: int
    search_complete: bool
    wall_clock_truncated: bool
    macro_successors_queued: int
    state_pops: int = 0
    state_resumptions: int = 0
    batch_size: int = 0


@dataclass(frozen=True)
class _Step:
    """One connector followed by one pass, with what each of them declares."""

    edge: ConnectorEdge
    candidate_index: int
    connector_declares: frozenset[str]
    pass_declares: frozenset[str]


@dataclass(frozen=True)
class _Node:
    mask: int
    tail: int
    totals: RouteAccumulators
    steps: tuple[_Step, ...]


@dataclass(frozen=True)
class _Route:
    node: _Node
    terminal_pose: Pose2D
    terminal_length_m: float
    cost: float


@dataclass
class _Visit:
    """A state on the frontier together with how far its stream has been read.

    Mutable on purpose: the cursor is the state's own progress through its
    successors, and it must survive the state being put back on the frontier.
    """

    node: _Node
    stream: tuple | None
    cursor: int


def solve_route(
    *,
    snapshot: ScanSnapshot,
    feasibility: tuple[PerBallFeasibility, ...],
    candidates: tuple[FunnelPassCandidate, ...],
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
    starved_ball_ids: frozenset[str] = frozenset(),
) -> RouterResult:
    if snapshot.configuration_snapshot != configuration:
        raise ValueError("configuration must exactly match snapshot configuration")
    if not snapshot.balls:
        return RouterResult(
            empty_plan(
                snapshot, configuration, PlanningStatus.EMPTY_NO_BALLS,
                PlanningSearchStatus.COMPLETE, (),
            ),
            0, True, False, 0, 0, 0, successor_batch_size(configuration, len(candidates)),
        )
    return _Search(
        snapshot, feasibility, candidates, court, configuration, starved_ball_ids
    ).run()


class _Search:
    def __init__(self, snapshot, feasibility, candidates, court, configuration, starved):
        self.snapshot = snapshot
        self.feasibility = feasibility
        self.court = court
        self.configuration = configuration
        self.starved = starved
        self.search = configuration.global_route_search

        self.ball_ids = tuple(ball.ball_id for ball in snapshot.balls)
        self.bit = {ball_id: 1 << index for index, ball_id in enumerate(self.ball_ids)}
        self.candidates = candidates
        self.candidate_masks = tuple(
            self._mask_of(candidate.covered_ball_ids) for candidate in candidates
        )
        # Only balls that survived per-ball feasibility may be swept in transit.
        # A ball with no pass candidate is keepout or otherwise unreachable, and
        # driving a connector through it is precisely what must not happen.
        eligible = {
            ball_id for candidate in candidates for ball_id in candidate.covered_ball_ids
        }
        self.sweepable = tuple(ball for ball in snapshot.balls if ball.ball_id in eligible)
        self.eligible_mask = self._mask_of(eligible)

        self.link_cache: dict[tuple[int, int], object] = {}
        self.terminal_cache: dict[int, tuple[float, Pose2D] | None] = {}
        self.evidence = {
            ball_id: BallEvidence(ball_id) for ball_id in self.ball_ids
        }
        for item in feasibility:
            if not item.reachable:
                self.evidence[item.ball_id].generation_reason = item.unreachable_reason
        for ball_id in starved:
            self.evidence[ball_id].cap_starved = True
        for index, candidate in enumerate(candidates):
            for ball_id in candidate.covered_ball_ids:
                self.evidence[ball_id].candidate_indices.append(index)

        self.clusters = _cluster_index(snapshot, configuration, candidates)
        self.macros = _macro_chains(self, configuration)
        self.batch_size = successor_batch_size(configuration, len(candidates))
        self.expansions = 0
        self.state_pops = 0
        self.state_resumptions = 0
        self.macro_successors_queued = 0
        self.incumbent: _Route | None = None
        self.wall_clock_truncated = False

    # -- geometry helpers, all memoized ------------------------------------
    def _mask_of(self, ball_ids) -> int:
        mask = 0
        for ball_id in ball_ids:
            mask |= self.bit.get(ball_id, 0)
        return mask

    def _pose_of(self, tail: int) -> Pose2D:
        return (
            self.snapshot.robot_pose_at_scan
            if tail == _START
            else self.candidates[tail].exit_pose
        )

    def link(self, source: int, target: int):
        """Best connector from a tail to a candidate's entry, with evidence."""
        key = (source, target)
        attempt = self.link_cache.get(key)
        if attempt is None:
            attempt = link_poses(
                source_id="start" if source == _START else f"pass-{source}",
                source_pose=self._pose_of(source),
                target_id=f"pass-{target}",
                target_pose=self.candidates[target].entry_pose,
                court=self.court,
                configuration=self.configuration,
                balls=self.sweepable,
            )
            self.link_cache[key] = attempt
        return attempt

    def terminal(self, tail: int):
        if tail not in self.terminal_cache:
            self.terminal_cache[tail] = self._compute_terminal(tail)
        return self.terminal_cache[tail]

    def _compute_terminal(self, tail: int):
        pose = self._pose_of(tail)
        length = self.search.terminal_run_out_m
        terminal = Pose2D(
            pose.x_m + length * math.cos(pose.yaw_rad),
            pose.y_m + length * math.sin(pose.yaw_rad),
            pose.yaw_rad,
        )
        if not _segment_is_collision_free(
            Point2D(pose.x_m, pose.y_m), Point2D(terminal.x_m, terminal.y_m),
            self.court, self.configuration.feasibility.footprint_clearance_radius_m,
        ):
            return None
        return length, terminal

    # -- search -------------------------------------------------------------
    def run(self) -> RouterResult:
        """Best-first over passes, expanding each state a batch at a time.

        A state does not have to be linked against every candidate on the court
        before the search may move: it hands back a bounded batch of successors
        and returns to the frontier holding the rest.  Because a child with more
        coverage outranks its parent, the search follows a promising route to a
        finished incumbent within a few visits instead of after ~200 connector
        solves, which is where the seconds went.

        Nothing is dropped by batching.  A visited state keeps its place and its
        remaining successors, and only leaves the frontier once its stream is
        exhausted -- so an empty frontier still means every route the candidate
        set admits was enumerated.
        """
        start = _Node(0, _START, RouteAccumulators(), ())
        frontier: list = []
        counter = 0
        heappush(frontier, (self._priority(start), counter, _Visit(start, None, 0)))
        visited: dict[tuple[int, int], float] = {(0, _START): 0.0}
        budget = self.search.max_search_expansions
        batch = self.batch_size
        deadline = time.monotonic() + self.configuration.planning.maximum_planning_time_s

        while frontier and self.expansions < budget:
            priority, _, visit = heappop(frontier)
            self.state_pops += 1
            if visit.stream is None:
                visit.stream = self._successor_stream(visit.node)
            produced = 0
            while (
                visit.cursor < len(visit.stream)
                and produced < batch
                and self.expansions < budget
            ):
                move = visit.stream[visit.cursor]
                visit.cursor += 1
                produced += 1
                # One expansion is one successor evaluation, so the budget buys
                # the same amount of work whatever the batch size does with it.
                self.expansions += 1
                successor, is_macro = self._expand(visit.node, move)
                if successor is None:
                    continue
                self._offer(successor)
                key = (successor.mask, successor.tail)
                cost = accumulated_cost(successor.totals, self.configuration)
                known = visited.get(key)
                if known is not None and known <= cost + _COST_EPSILON:
                    continue
                if self._is_pruned(successor):
                    continue
                visited[key] = cost
                counter += 1
                if is_macro:
                    self.macro_successors_queued += 1
                heappush(
                    frontier,
                    (self._priority(successor), counter, _Visit(successor, None, 0)),
                )
            if visit.cursor < len(visit.stream):
                # Unfinished, so it goes back with its own priority: the rest of
                # its successors stay reachable until the search ends.
                counter += 1
                self.state_resumptions += 1
                heappush(frontier, (priority, counter, visit))
            if time.monotonic() > deadline:
                self.wall_clock_truncated = True
                break

        search_complete = not frontier and not self.wall_clock_truncated
        return self._result(search_complete)

    def _successor_stream(self, node: _Node) -> tuple:
        """The full ordered successor stream for a state, computed once.

        Macros come first: they are the precomputed intra-cluster chains, so
        trying them early is the whole point of having them.  Individual passes
        follow in the deterministic candidate order.  Order decides *when* a
        successor is evaluated and never whether it exists.
        """
        return tuple(
            ("macro", chain) for chain in self.macros
        ) + tuple(("pass", index) for index in self._candidate_order(node))

    def _expand(self, node: _Node, move):
        kind, payload = move
        if kind == "macro":
            successor = self._apply_chain(node, payload)
            return successor, True
        step = self._step(node, payload)
        if step is None:
            return None, False
        return self._apply(node, (step,)), False

    def _priority(self, node: _Node):
        """What to look at first.  Never what is allowed to exist.

        Coverage first mirrors the objective, so a high-coverage incumbent
        appears early, and true cost breaks the tie.  There is deliberately no
        cluster term: biasing the frontier towards finishing the group the robot
        is standing in was measured on the real scan and it *delayed* good
        routes (7.0 s to beat the flat baseline with the bias, 2.7 s without;
        debug log #59).  Clustering still pays for itself through macros, which
        is a different mechanism -- offering a good chain in one step rather
        than second-guessing the frontier.
        """
        return (
            -_popcount(node.mask),
            accumulated_cost(node.totals, self.configuration),
            node.tail,
        )

    def _candidate_order(self, node: _Node) -> tuple[int, ...]:
        """Deterministic order over every candidate; nothing is filtered out.

        Passes that would collect more new balls come first, because that is
        what the objective rewards first.  Cluster affinity used to break the
        tie and was measured to cost more than it bought (debug log #59).
        """
        return tuple(
            sorted(
                range(len(self.candidates)),
                key=lambda index: (
                    -_popcount(self.candidate_masks[index] & ~node.mask), index
                ),
            )
        )

    def _step(self, node: _Node, index: int) -> _Step | None:
        """One connector-plus-pass move, or None when it adds nothing or fails."""
        if index == node.tail:
            return None
        attempt = self.link(node.tail, index)
        if not attempt.feasible:
            self._record_link_failure(node.tail, index, attempt)
            return None
        edge = attempt.edge
        swept = self._mask_of(edge.swept_ball_ids)
        covered = self.candidate_masks[index]
        new = (swept | covered) & ~node.mask
        # A pass whose balls are all in the basket already may still be worth
        # driving when the connector into it sweeps something new, so the test
        # is on the move as a whole, never on the pass alone.
        if not new:
            return None
        for ball_id in self.candidates[index].covered_ball_ids:
            self.evidence[ball_id].entered = True
        return _Step(
            edge,
            index,
            frozenset(
                ball_id for ball_id in edge.swept_ball_ids if new & self.bit[ball_id]
            ),
            frozenset(
                ball_id
                for ball_id in self.candidates[index].covered_ball_ids
                if new & self.bit[ball_id] and not swept & self.bit[ball_id]
            ),
        )

    def _apply(self, node: _Node, steps: tuple[_Step, ...]) -> _Node:
        mask = node.mask
        totals = node.totals
        for step in steps:
            candidate = self.candidates[step.candidate_index]
            length = pass_length(candidate)
            mask |= self._mask_of(step.connector_declares | step.pass_declares)
            totals = totals.plus(
                length_m=step.edge.path.length_m + length,
                duration_s=(
                    step.edge.path.length_m / self.search.connector_nominal_speed_m_s
                    + length / self.search.crossing_nominal_speed_m_s
                ),
                curvature_rad=step.edge.path.total_turn_rad,
                pass_count=1 if step.pass_declares else 0,
            )
        return _Node(mask, steps[-1].candidate_index, totals, node.steps + steps)

    def _apply_chain(self, node: _Node, chain: tuple[int, ...]) -> _Node | None:
        """Take a whole precomputed chain in one expansion, or not at all.

        A macro is only ever an accelerator: every pass it contains is also an
        individual successor of this same node, so refusing a macro can never
        remove a route, and taking one can never reach a state the individual
        moves could not.
        """
        current = node
        steps: list[_Step] = []
        for index in chain:
            step = self._step(current, index)
            if step is None:
                return None
            current = self._apply(current, (step,))
            steps.append(step)
        if current.mask == node.mask:
            return None
        return _Node(current.mask, current.tail, current.totals, node.steps + tuple(steps))

    def _record_link_failure(self, source: int, target: int, attempt) -> None:
        for ball_id in self.candidates[target].covered_ball_ids:
            self.evidence[ball_id].inbound_rejections.extend(attempt.rejections)
        if source != _START:
            for ball_id in self.candidates[source].covered_ball_ids:
                self.evidence[ball_id].outbound_rejections.extend(attempt.rejections)

    def _offer(self, node: _Node) -> None:
        """Close the node into a finished route and keep it if it is the best.

        Called before any pruning or dominance decision touches the node, which
        is what makes "a found route is never lost" structural.
        """
        if node.tail == _START:
            return
        terminal = self.terminal(node.tail)
        for ball_id in self.candidates[node.tail].covered_ball_ids:
            self.evidence[ball_id].terminal_checked = True
            if terminal is not None:
                self.evidence[ball_id].terminal_feasible = True
        if terminal is None:
            return
        length, pose = terminal
        totals = node.totals.plus(
            length_m=length, duration_s=length / self.search.connector_nominal_speed_m_s
        )
        route = _Route(node, pose, length, accumulated_cost(totals, self.configuration))
        if self.incumbent is None or self._score(route) < self._score(self.incumbent):
            self.incumbent = route

    def _score(self, route: _Route):
        return (
            -_popcount(route.node.mask),
            route.cost,
            len(route.node.steps),
            tuple(step.candidate_index for step in route.node.steps),
        )

    def _is_pruned(self, node: _Node) -> bool:
        """Admissible bounds only: never discard what could still win.

        Both tests compare against the incumbent, which evolves identically for
        any prefix of the run, so pruning cannot break budget monotonicity.
        """
        if self.incumbent is None:
            return False
        best_coverage = _popcount(self.incumbent.node.mask)
        reachable = _popcount(node.mask | self._uncovered_eligible(node.mask))
        if reachable < best_coverage:
            return True
        if reachable > best_coverage:
            return False
        run_out = self.search.terminal_run_out_m
        lower_bound = cost_value(
            node.totals.length_m + run_out,
            node.totals.duration_s + run_out / self.search.connector_nominal_speed_m_s,
            node.totals.curvature_rad,
            node.totals.pass_count,
            self.configuration,
        )
        return lower_bound > self.incumbent.cost + _COST_EPSILON

    def _uncovered_eligible(self, mask: int) -> int:
        return self.eligible_mask & ~mask

    # -- output --------------------------------------------------------------
    def _result(self, search_complete: bool) -> RouterResult:
        # The candidate cap is a budget too: if it trimmed anything, the run did
        # not examine everything the geometry offers, whatever the frontier did.
        everything_examined = search_complete and not self.starved
        search_status = (
            PlanningSearchStatus.COMPLETE
            if everything_examined
            else PlanningSearchStatus.BUDGET_EXHAUSTED
        )
        if self.incumbent is None:
            results = attribute_ball_results(
                snapshot=self.snapshot, evidence=self.evidence, covered={},
                search_complete=search_complete, route_found=False,
            )
            # "No feasible targets" is the mission-level statement that there was
            # nothing to attempt, so it belongs only to balls that never had a
            # pass at all.  A ball with passes that no route could use is a
            # planning failure and must not be reported as a quiet success.
            status = (
                PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS
                if all(
                    self.evidence[item.ball_id].generation_reason is not None
                    for item in results
                )
                else PlanningStatus.PLANNING_TIMEOUT
            )
            return RouterResult(
                empty_plan(self.snapshot, self.configuration, status, search_status, results),
                self.expansions, search_complete, self.wall_clock_truncated,
                self.macro_successors_queued, self.state_pops, self.state_resumptions,
                self.batch_size,
            )

        segments, covered = self._segments(self.incumbent)
        results = attribute_ball_results(
            snapshot=self.snapshot, evidence=self.evidence, covered=covered,
            search_complete=search_complete, route_found=True,
        )
        plan = assemble_plan(
            snapshot=self.snapshot, configuration=self.configuration, segments=segments,
            terminal_pose=self.incumbent.terminal_pose, ball_results=results,
            search_status=search_status,
        )
        return RouterResult(
            plan, self.expansions, search_complete, self.wall_clock_truncated,
            self.macro_successors_queued, self.state_pops, self.state_resumptions,
            self.batch_size,
        )

    def _segments(self, route: _Route):
        segments = []
        covered: dict[str, str] = {}
        progress = 0.0
        for position, step in enumerate(route.node.steps):
            candidate = self.candidates[step.candidate_index]
            connector_id = f"connector-{position}"
            segments.append(
                connector_segment(
                    connector_id, step.edge, progress, self.configuration,
                    declare_only=step.connector_declares,
                )
            )
            progress = segments[-1].progress_end_m
            for ball_id in segments[-1].covered_ball_ids:
                covered[ball_id] = connector_id
            if step.pass_declares:
                pass_id = f"pass-{position}:{candidate.ball_id}"
                segments.append(
                    pass_segment(
                        pass_id, candidate, progress, self.configuration,
                        declare_only=step.pass_declares,
                    )
                )
                for ball_id in segments[-1].covered_ball_ids:
                    covered[ball_id] = pass_id
            else:
                segments.append(
                    transit_pass_segment(
                        f"transit-{position}:{candidate.ball_id}", candidate, progress,
                        self.configuration,
                    )
                )
            progress = segments[-1].progress_end_m
        segments.append(
            terminal_segment(
                "terminal", self._pose_of(route.node.tail), route.terminal_pose,
                progress, route.terminal_length_m, self.configuration,
            )
        )
        return tuple(segments), covered


def _cluster_index(snapshot, configuration, candidates) -> dict[int, frozenset[str]]:
    """Map each candidate to the cluster it belongs to, for ordering only."""
    clusters = cluster_balls(snapshot.balls, configuration)
    membership: dict[str, frozenset[str]] = {}
    for cluster in clusters:
        for ball_id in cluster:
            membership[ball_id] = cluster
    index: dict[int, frozenset[str]] = {}
    for position, candidate in enumerate(candidates):
        cluster = membership.get(candidate.covered_ball_ids[0])
        if cluster is not None:
            index[position] = cluster
    return index


def cluster_balls(balls, configuration) -> tuple[frozenset[str], ...]:
    """Single-linkage grouping, bounded by ``maximum_clusters``.

    This is a hint and nothing else: it decides what the search looks at first
    and which macros are worth precomputing.  No caller may use it to reject a
    pass, and no ball's reachability depends on which group it lands in.
    """
    heuristics = configuration.cluster_heuristics
    groups = [[ball] for ball in sorted(balls, key=lambda ball: ball.ball_id)]

    def closest(items):
        best = None
        for first in range(len(items)):
            for second in range(first + 1, len(items)):
                separation = min(
                    math.hypot(a.position.x_m - b.position.x_m, a.position.y_m - b.position.y_m)
                    for a in items[first]
                    for b in items[second]
                )
                key = (separation, items[first][0].ball_id, items[second][0].ball_id)
                if best is None or key < best[0]:
                    best = (key, first, second)
        return best

    while len(groups) > 1:
        found = closest(groups)
        if found is None:
            break
        (separation, _, _), first, second = found
        over_cap = len(groups) > heuristics.maximum_clusters
        if separation > heuristics.cluster_threshold_m and not over_cap:
            break
        groups[first] = groups[first] + groups[second]
        del groups[second]
    return tuple(frozenset(ball.ball_id for ball in group) for group in groups)


def _macro_chains(search: "_Search", configuration) -> tuple[tuple[int, ...], ...]:
    """Precompute a bounded set of promising intra-cluster pass chains.

    Greedy, cheap, and entirely optional: each chain is one expansion instead of
    several, which is where the measured speed of cluster-first routing actually
    came from.  Nothing here restricts the search -- a chain is offered as an
    extra successor next to every individual pass.
    """
    heuristics = configuration.cluster_heuristics
    if heuristics.maximum_macro_chains <= 0 or heuristics.maximum_macro_passes < 2:
        return ()
    by_cluster: dict[frozenset[str], list[int]] = {}
    for index, cluster in search.clusters.items():
        if set(search.candidates[index].covered_ball_ids) <= set(cluster):
            by_cluster.setdefault(cluster, []).append(index)

    chains: list[tuple[int, ...]] = []
    for cluster in sorted(by_cluster, key=lambda item: (-len(item), sorted(item))):
        members = sorted(by_cluster[cluster])
        for seed in members:
            if len(chains) >= heuristics.maximum_macro_chains:
                return tuple(chains)
            chain = _greedy_chain(search, seed, members, heuristics.maximum_macro_passes)
            if len(chain) >= 2 and chain not in chains:
                chains.append(chain)
    return tuple(chains)


def _greedy_chain(search: "_Search", seed: int, members, limit: int) -> tuple[int, ...]:
    chain = [seed]
    covered = set(search.candidates[seed].covered_ball_ids)
    while len(chain) < limit:
        best = None
        for index in members:
            if index in chain:
                continue
            gained = set(search.candidates[index].covered_ball_ids) - covered
            if not gained:
                continue
            attempt = search.link(chain[-1], index)
            if not attempt.feasible:
                continue
            key = (-len(gained), attempt.edge.path.length_m, index)
            if best is None or key < best[0]:
                best = (key, index, gained)
        if best is None:
            break
        _, index, gained = best
        chain.append(index)
        covered |= gained
    return tuple(chain)


def _popcount(mask: int) -> int:
    return bin(mask).count("1")
