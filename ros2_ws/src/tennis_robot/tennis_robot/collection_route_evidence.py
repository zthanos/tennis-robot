"""What the planner actually learned about each ball, and what it may claim.

The rule this module exists to enforce: a geometric failure is reported only
when it was *established*.  A ball the search never resolved is reported as
search-limited, because "planning stopped before the answer was known" and
"the geometry forbids it" are different statements and only one of them is
actionable by moving the robot (debug log #57beta).

Evidence is collected by the search as it runs.  Nothing here re-derives
geometry; it only reads what was recorded.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from tennis_robot.collection_route_connector_graph import ConnectorRejectionCode
from tennis_robot.collection_route_types import (
    BallReasonCode,
    BallResult,
    BallStatus,
    ScanSnapshot,
)


@dataclass
class BallEvidence:
    """Everything observed about one ball during candidate generation and search."""

    ball_id: str
    # Set when per-ball feasibility already refused the ball.  This is
    # established without any search at all, so it is always reportable.
    generation_reason: BallReasonCode | None = None
    # The candidate cap trimmed this ball's alternatives, so nothing about the
    # trimmed geometry can be claimed either way.
    cap_starved: bool = False
    candidate_indices: list[int] = field(default_factory=list)
    # A step into one of its candidates was actually constructed: the ball was
    # reachable from somewhere the route could be.
    entered: bool = False
    terminal_checked: bool = False
    terminal_feasible: bool = False
    inbound_rejections: list[ConnectorRejectionCode] = field(default_factory=list)
    outbound_rejections: list[ConnectorRejectionCode] = field(default_factory=list)

    @property
    def has_candidates(self) -> bool:
        return bool(self.candidate_indices)

    def sole_inbound_cause(self) -> ConnectorRejectionCode | None:
        distinct = set(self.inbound_rejections)
        return distinct.pop() if len(distinct) == 1 else None


def attribute_ball_results(
    *,
    snapshot: ScanSnapshot,
    evidence: dict[str, BallEvidence],
    covered: dict[str, str],
    search_complete: bool,
    route_found: bool,
) -> tuple[BallResult, ...]:
    """Turn evidence into one result per snapshot ball.

    ``search_complete`` means the frontier emptied: every route the candidate
    set admits was enumerated.  That is what licenses a claim about geometry,
    because a failure observed under an exhaustive search is a property of the
    problem rather than of when we stopped looking.
    """
    results = []
    for ball in snapshot.balls:
        item = evidence[ball.ball_id]
        if ball.ball_id in covered:
            results.append(
                BallResult(
                    ball.ball_id, BallStatus.COVERED, BallReasonCode.SELECTED,
                    covered[ball.ball_id],
                )
            )
        elif item.generation_reason is not None:
            # Established before any search ran: no pass exists for this ball
            # under the declared heading sampling.
            results.append(
                BallResult(ball.ball_id, BallStatus.UNREACHABLE, item.generation_reason)
            )
        elif not search_complete or item.cap_starved:
            # The honest answer whenever the evidence is incomplete: planning
            # stopped before feasibility could be established.
            results.append(
                BallResult(ball.ball_id, BallStatus.DEFERRED, BallReasonCode.PLANNING_BUDGET)
            )
        else:
            results.append(_exhausted_result(item, route_found))
    return tuple(results)


def _exhausted_result(item: BallEvidence, route_found: bool) -> BallResult:
    """Attribution after an exhaustive search, where claims are permitted."""
    if item.entered:
        if not route_found and item.terminal_checked and not item.terminal_feasible:
            # Reachable, but no route could ever be closed after collecting it.
            return BallResult(item.ball_id, BallStatus.UNREACHABLE, BallReasonCode.NO_TERMINAL)
        # The search had this ball available in a route it could drive and the
        # global objective declined it.  A planner decision, not geometry.
        return BallResult(item.ball_id, BallStatus.DEFERRED, BallReasonCode.ROUTE_CONFLICT)
    cause = item.sole_inbound_cause()
    if cause is ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED:
        return BallResult(item.ball_id, BallStatus.UNREACHABLE, BallReasonCode.TURN_RADIUS)
    if cause is ConnectorRejectionCode.COLLISION_REJECTED:
        return BallResult(item.ball_id, BallStatus.UNREACHABLE, BallReasonCode.CONNECTOR_CLEARANCE)
    # Never entered, but the alternatives failed for mixed reasons or against a
    # configured connector length cap.  No single physical cause was
    # established, so no physical cause is claimed.
    return BallResult(item.ball_id, BallStatus.DEFERRED, BallReasonCode.ROUTE_CONFLICT)
