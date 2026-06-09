#!/usr/bin/env python3
"""Analyze recorded Map Court ticks as PoC-style semantic survey events.

This is the bridge between real/virtual recorded telemetry and the
event-injected survey PoC:

  replay JSONL ticks -> semantic event candidates -> canonical fence model

It intentionally does not drive the robot behavior. It observes what the
recording says happened, translates known Map Court events/points into the
same canonical route vocabulary as the PoC, and reports whether canonical
validation would accept it.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPLAY = ROOT / "runtime" / "survey_replay_current.jsonl"


class EventType(str, Enum):
    NEAR_NET = "near_net"
    NEAR_FENCE = "near_fence"
    CORNER_DETECTED = "corner_detected"
    TURN_COMPLETE = "turn_complete"
    LOOP_CLOSED = "loop_closed"


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float

    def distance_to(self, other: "Point") -> float:
        return math.hypot(self.x_m - other.x_m, self.y_m - other.y_m)


@dataclass(frozen=True)
class SemanticEvent:
    kind: EventType
    label: str
    point: Point
    tick: int
    source: str
    confidence: float = 1.0


@dataclass(frozen=True)
class CanonicalFence:
    label: str
    start_corner: str
    end_corner: str
    start: Point
    end: Point

    @property
    def length_m(self) -> float:
        return self.start.distance_to(self.end)


LEGACY_TO_CANONICAL_LABEL = {
    "near_net_standoff": "near_net_standoff",
    "near_baseline_fence_standoff": "near_baseline_fence_standoff",
    "left_side_fence_corner": "near_left_fence_corner",
    "far_baseline_fence_corner": "far_left_fence_corner",
    "right_side_fence_corner": "far_right_fence_corner",
    "return_baseline_fence_corner": "near_right_fence_corner",
}

EVENT_TO_CANONICAL_LABEL = {
    "net_standoff_reached": ("near_net_standoff", EventType.NEAR_NET),
    "net_found_turning_180_for_baseline": ("near_net_standoff", EventType.NEAR_NET),
    "baseline_fence_reached": ("near_baseline_fence_standoff", EventType.NEAR_FENCE),
    "side_fence_reached": ("near_left_fence_corner", EventType.CORNER_DETECTED),
    "far_baseline_fence_reached": ("far_left_fence_corner", EventType.CORNER_DETECTED),
    "far_side_fence_reached": ("far_right_fence_corner", EventType.CORNER_DETECTED),
    "return_baseline_fence_reached": ("near_right_fence_corner", EventType.CORNER_DETECTED),
    "full_perimeter_survey_success": ("near_left_loop_closure", EventType.LOOP_CLOSED),
}

EVENT_PREFIX_TO_CANONICAL_LABEL = {
    "net_found": ("near_net_standoff", EventType.NEAR_NET),
    "baseline_fence_reached": ("near_baseline_fence_standoff", EventType.NEAR_FENCE),
    "side_fence_reached": ("near_left_fence_corner", EventType.CORNER_DETECTED),
    "far_baseline_fence_reached": ("far_left_fence_corner", EventType.CORNER_DETECTED),
    "far_side_fence_reached": ("far_right_fence_corner", EventType.CORNER_DETECTED),
    "return_baseline_fence_reached": ("near_right_fence_corner", EventType.CORNER_DETECTED),
    "full_perimeter_survey_success": ("near_left_loop_closure", EventType.LOOP_CLOSED),
}


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _point_from_tick(tick: dict[str, Any]) -> Point | None:
    try:
        x = float(tick.get("x_m"))
        y = float(tick.get("y_m"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return Point(round(x, 3), round(y, 3))


def _navigation_points(tick: dict[str, Any]) -> list[dict[str, Any]]:
    nav = tick.get("navigation") or {}
    points = nav.get("survey_navigation_points")
    if isinstance(points, list):
        return points
    bounds = tick.get("bounds") or {}
    points = bounds.get("navigation_points")
    return points if isinstance(points, list) else []


def _canonical_from_runtime_event(raw_event: str) -> tuple[str, EventType] | None:
    direct = EVENT_TO_CANONICAL_LABEL.get(raw_event)
    if direct is not None:
        return direct
    for prefix, canonical in EVENT_PREFIX_TO_CANONICAL_LABEL.items():
        if raw_event.startswith(prefix):
            return canonical
    return None


def extract_semantic_events(ticks: list[dict[str, Any]]) -> list[SemanticEvent]:
    events: list[SemanticEvent] = []
    seen_labels: set[str] = set()
    seen_runtime_events: set[tuple[str, str]] = set()

    for idx, tick in enumerate(ticks):
        for raw_point in _navigation_points(tick):
            raw_label = raw_point.get("label")
            label = LEGACY_TO_CANONICAL_LABEL.get(raw_label, raw_label)
            if not label or label in seen_labels:
                continue
            try:
                point = Point(float(raw_point["x_m"]), float(raw_point["y_m"]))
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(point.x_m) or not math.isfinite(point.y_m):
                continue
            seen_labels.add(label)
            kind = EventType.CORNER_DETECTED
            if label == "near_net_standoff":
                kind = EventType.NEAR_NET
            elif label == "near_baseline_fence_standoff":
                kind = EventType.NEAR_FENCE
            events.append(SemanticEvent(kind, label, point, idx, "navigation_point"))

        raw_event = tick.get("survey_event") or ((tick.get("navigation") or {}).get("last_event"))
        if not raw_event:
            continue
        canonical = _canonical_from_runtime_event(raw_event)
        if canonical is None:
            continue
        label, kind = canonical
        key = (raw_event, label)
        if key in seen_runtime_events or label in seen_labels:
            continue
        point = _point_from_tick(tick)
        if point is None:
            continue
        seen_runtime_events.add(key)
        seen_labels.add(label)
        events.append(SemanticEvent(kind, label, point, idx, f"survey_event:{raw_event}", confidence=0.65))

    events.sort(key=lambda e: e.tick)
    return events


def canonical_model(events: list[SemanticEvent]) -> dict[str, Any]:
    by_label = {event.label: event.point for event in events}
    errors: list[str] = []
    required = [
        "near_net_standoff",
        "near_baseline_fence_standoff",
        "near_left_fence_corner",
        "far_left_fence_corner",
        "far_right_fence_corner",
        "near_right_fence_corner",
    ]
    missing = [label for label in required if label not in by_label]
    if missing:
        errors.append("missing_route_points:" + ",".join(missing))

    corners: dict[str, Point] = {}
    if not missing:
        corners = {
            "near_left": by_label["near_left_fence_corner"],
            "far_left": by_label["far_left_fence_corner"],
            "far_right": by_label["far_right_fence_corner"],
            "near_right": by_label["near_right_fence_corner"],
        }
        errors.extend(_geometry_errors(by_label, corners))

    fences: dict[str, CanonicalFence] = {}
    if corners:
        specs = {
            "near_baseline_fence": ("near_left", "near_right"),
            "left_side_fence": ("near_left", "far_left"),
            "far_baseline_fence": ("far_left", "far_right"),
            "right_side_fence": ("far_right", "near_right"),
        }
        fences = {
            label: CanonicalFence(label, start, end, corners[start], corners[end])
            for label, (start, end) in specs.items()
        }

    return {
        "status": "VALID" if not errors else "PARTIAL",
        "errors": errors,
        "corners": {
            label: {"x_m": round(point.x_m, 3), "y_m": round(point.y_m, 3)}
            for label, point in corners.items()
        },
        "fences": {
            label: {
                "start_corner": fence.start_corner,
                "end_corner": fence.end_corner,
                "length_m": round(fence.length_m, 3),
            }
            for label, fence in fences.items()
        },
    }


def render_svg(events: list[SemanticEvent], model: dict[str, Any], title: str) -> str:
    """Render a dependency-free survey diagnostic drawing."""
    points = [(event.point.x_m, event.point.y_m) for event in events]
    corners = [
        (float(point["x_m"]), float(point["y_m"]))
        for point in (model.get("corners") or {}).values()
    ]
    court_half_width = 10.97 / 2
    court_length = 23.77
    court_points = [
        (-court_half_width, 0.0),
        (court_half_width, 0.0),
        (court_half_width, court_length),
        (-court_half_width, court_length),
    ]
    all_points = points + corners + court_points
    if not all_points:
        all_points = [(-6.0, -6.0), (6.0, 24.0)]

    min_x = min(x for x, _ in all_points) - 2.0
    max_x = max(x for x, _ in all_points) + 2.0
    min_y = min(y for _, y in all_points) - 2.0
    max_y = max(y for _, y in all_points) + 2.0
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)

    width = 920
    height = 760
    margin = 72
    plot_w = width - margin * 2
    plot_h = height - margin * 2
    scale = min(plot_w / span_x, plot_h / span_y)

    def sx(x: float) -> float:
        return margin + (x - min_x) * scale

    def sy(y: float) -> float:
        return height - margin - (y - min_y) * scale

    def polyline(coords: list[tuple[float, float]], close: bool = False) -> str:
        pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in coords)
        if close and coords:
            pts += f" {sx(coords[0][0]):.1f},{sy(coords[0][1]):.1f}"
        return pts

    status = model.get("status", "UNKNOWN")
    escaped_title = html.escape(title)
    escaped_status = html.escape(str(status))
    route_pts = polyline(points)
    court_pts = polyline(court_points, close=True)
    net_y = court_length / 2
    net = ((-court_half_width, net_y), (court_half_width, net_y))

    fence_lines: list[str] = []
    corner_by_name = model.get("corners") or {}
    for fence in (model.get("fences") or {}).values():
        start = corner_by_name.get(fence.get("start_corner"))
        end = corner_by_name.get(fence.get("end_corner"))
        if start and end:
            fence_lines.append(
                f'<line x1="{sx(float(start["x_m"])):.1f}" y1="{sy(float(start["y_m"])):.1f}" '
                f'x2="{sx(float(end["x_m"])):.1f}" y2="{sy(float(end["y_m"])):.1f}" '
                'stroke="#c62828" stroke-width="4" stroke-linecap="round" />'
            )

    event_marks: list[str] = []
    for index, event in enumerate(events, start=1):
        x = sx(event.point.x_m)
        y = sy(event.point.y_m)
        event_marks.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="#1976d2" stroke="#ffffff" stroke-width="2" />'
        )
        event_marks.append(
            f'<text x="{x + 10:.1f}" y="{y - 10:.1f}" class="label">'
            f'{index}. {html.escape(event.label)}</text>'
        )

    corner_marks: list[str] = []
    for label, point in corner_by_name.items():
        x = sx(float(point["x_m"]))
        y = sy(float(point["y_m"]))
        corner_marks.append(
            f'<rect x="{x - 6:.1f}" y="{y - 6:.1f}" width="12" height="12" '
            'fill="#c62828" stroke="#ffffff" stroke-width="2" />'
        )
        corner_marks.append(
            f'<text x="{x + 10:.1f}" y="{y + 18:.1f}" class="corner">{html.escape(label)}</text>'
        )

    errors = model.get("errors") or model.get("validation_errors") or []
    error_text = "; ".join(str(error) for error in errors) or "no validation errors"
    error_text = html.escape(error_text)

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <style>
    .title {{ font: 700 22px Arial, sans-serif; fill: #17202a; }}
    .subtitle {{ font: 14px Arial, sans-serif; fill: #4d5b65; }}
    .label {{ font: 12px Arial, sans-serif; fill: #0d47a1; }}
    .corner {{ font: 12px Arial, sans-serif; fill: #8e1b1b; font-weight: 700; }}
    .legend {{ font: 13px Arial, sans-serif; fill: #263238; }}
  </style>
  <rect width="100%" height="100%" fill="#f8faf7" />
  <text x="36" y="38" class="title">{escaped_title}</text>
  <text x="36" y="61" class="subtitle">canonical status: {escaped_status}</text>
  <text x="36" y="{height - 28}" class="subtitle">{error_text}</text>
  <polyline points="{court_pts}" fill="#e8f5e9" stroke="#6d8f5f" stroke-width="2" stroke-dasharray="8 6" />
  <line x1="{sx(net[0][0]):.1f}" y1="{sy(net[0][1]):.1f}" x2="{sx(net[1][0]):.1f}" y2="{sy(net[1][1]):.1f}" stroke="#111111" stroke-width="3" />
  <text x="{sx(court_half_width) + 10:.1f}" y="{sy(net_y) + 4:.1f}" class="legend">net</text>
  <text x="{sx(court_half_width) + 10:.1f}" y="{sy(court_length) + 4:.1f}" class="legend">playable outer lines</text>
  <polyline points="{route_pts}" fill="none" stroke="#1976d2" stroke-width="2.5" stroke-dasharray="6 5" />
  {''.join(fence_lines)}
  {''.join(event_marks)}
  {''.join(corner_marks)}
</svg>
"""


def _geometry_errors(by_label: dict[str, Point], corners: dict[str, Point]) -> list[str]:
    errors: list[str] = []
    expected_length = 23.77
    expected_width = 10.97
    tolerance = 3.0

    checks = {
        "near_baseline_to_near_left": (
            by_label["near_baseline_fence_standoff"].distance_to(corners["near_left"]),
            (expected_width / 2) - tolerance,
            math.inf,
        ),
        "left_side": (
            corners["near_left"].distance_to(corners["far_left"]),
            expected_length - tolerance * 2,
            expected_length + tolerance * 2,
        ),
        "far_baseline": (
            corners["far_left"].distance_to(corners["far_right"]),
            expected_width - tolerance * 2,
            expected_width + tolerance * 2,
        ),
        "right_side": (
            corners["far_right"].distance_to(corners["near_right"]),
            expected_length - tolerance * 2,
            expected_length + tolerance * 2,
        ),
    }
    for label, (distance, lower, upper) in checks.items():
        if not (lower <= distance <= upper):
            errors.append(f"{label}_distance_invalid:{distance:.2f}m")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("replay", nargs="?", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--svg-out", type=Path)
    args = parser.parse_args()

    ticks = _load_jsonl(args.replay)
    events = extract_semantic_events(ticks)
    model = canonical_model(events)
    summary = {
        "replay": str(args.replay),
        "ticks": len(ticks),
        "semantic_events": [
            {
                "tick": event.tick,
                "kind": event.kind.value,
                "label": event.label,
                "x_m": event.point.x_m,
                "y_m": event.point.y_m,
                "source": event.source,
                "confidence": event.confidence,
            }
            for event in events
        ],
        "canonical_fence_model": model,
    }
    text = json.dumps(summary, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    if args.svg_out:
        args.svg_out.parent.mkdir(parents=True, exist_ok=True)
        args.svg_out.write_text(render_svg(events, model, args.replay.name), encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
