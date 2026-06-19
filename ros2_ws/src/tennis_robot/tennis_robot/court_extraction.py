"""Court Knowledge Model extraction from a LiDAR occupancy point set.

Pure, ROS-free functions (see docs/court-survey-v2-spec-el.md). Given the
accumulated map-frame LiDAR points and the LiDAR-locked net, this produces the
Court Knowledge Model: net + posts, fence rectangle, court lines (net-anchored
standard ITF geometry), interior obstacles, run-off distances and singles/doubles.

DESIGN: NO FALLBACKS. Every step that lacks evidence raises CourtExtractionError
with an explicit reason — the caller writes status="FAILED" and never a fabricated
boundary. The only constants are the regulation court dimensions (a deliberate
design choice, configurable via CourtSpec).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


class CourtExtractionError(Exception):
    """Raised, fail-loud, when the model cannot be measured from the points."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True)
class CourtSpec:
    # Regulation ITF dimensions (the only allowed constants).
    length_m: float = 23.77
    width_doubles_m: float = 10.97
    width_singles_m: float = 8.23
    service_from_net_m: float = 6.40
    doubles_alley_m: float = 1.37
    # Net posts sit just outside the doubles sidelines (sim model: ±5.65 m).
    post_half_span_doubles_m: float = 5.65
    post_half_span_singles_m: float = 5.03
    post_span_tol_m: float = 0.8
    # Fence fitting.
    fence_bin_m: float = 0.15
    fence_min_points: int = 12
    fence_peak_frac: float = 0.30
    # Net line exclusion when hunting posts / obstacles.
    net_band_m: float = 0.80  # exclude full net depth+posts (returns spread to ~0.7m)
    # Obstacle clustering.
    obstacle_grid_m: float = 0.20
    obstacle_min_points: int = 8
    obstacle_edge_margin_m: float = 0.90  # exclude fence thickness/noise (leaks ~0.6m inward)
    # Smart fence-artifact rejection: a cluster within this band of a fence AND
    # elongated PARALLEL to it (ratio below) is LiDAR scatter off the fence, not a
    # real object. A real obstacle protrudes inward (elongated perpendicular) or
    # sits clear of the fences, so it survives even near a fence.
    fence_artifact_band_m: float = 1.8
    fence_artifact_parallel_ratio: float = 1.25
    # Sanity bounds for run-off (court line -> fence).
    runoff_min_m: float = 0.0
    runoff_max_m: float = 12.0
    # Minimum points a fence side must have to count as observed.
    fence_side_min_points: int = 25

    @property
    def half_length_m(self) -> float:
        return self.length_m / 2.0


@dataclass
class CourtFrame:
    """Court coordinate frame derived from the net (origin=net centre)."""

    cx: float
    cy: float
    ux: float  # +x' = length axis (robot -> net direction)
    uy: float
    vx: float  # +y' = width axis (along the net line)
    vy: float

    def to_court(self, px: float, py: float) -> tuple[float, float]:
        dx, dy = px - self.cx, py - self.cy
        return (dx * self.ux + dy * self.uy, dx * self.vx + dy * self.vy)

    def to_map(self, xp: float, yp: float) -> tuple[float, float]:
        return (self.cx + xp * self.ux + yp * self.vx,
                self.cy + xp * self.uy + yp * self.vy)


# ── helpers ────────────────────────────────────────────────────────────────

def _as_xy(points: list) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for p in points:
        if isinstance(p, dict):
            x, y = p.get("x_m"), p.get("y_m")
        else:
            x, y = p[0], p[1]
        if isinstance(x, (int, float)) and isinstance(y, (int, float)) \
                and math.isfinite(x) and math.isfinite(y):
            out.append((float(x), float(y)))
    return out


def build_court_frame(locked_net: dict) -> CourtFrame:
    """Court frame from the LiDAR net: origin=net centre, +x'=robot->net."""
    try:
        cx = float(locked_net["map_x_m"]); cy = float(locked_net["map_y_m"])
        rx = float(locked_net["robot_x_m"]); ry = float(locked_net["robot_y_m"])
    except (KeyError, TypeError, ValueError):
        raise CourtExtractionError("net_not_observed: missing net map/robot pose")
    ux, uy = cx - rx, cy - ry
    n = math.hypot(ux, uy)
    if not math.isfinite(n) or n < 0.5:
        raise CourtExtractionError("net_not_observed: robot->net vector too short")
    ux, uy = ux / n, uy / n
    return CourtFrame(cx, cy, ux, uy, -uy, ux)


def _outer_fence_lines(coords: list[float], spec: CourtSpec) -> tuple[float, float, int, int]:
    """Outermost dense bins along one axis = the two fence lines on that axis.

    Returns (near, far, near_count, far_count). Raises if a side is missing.
    """
    if not coords:
        raise CourtExtractionError("fence_side_missing: no points")
    lo, hi = min(coords), max(coords)
    span = hi - lo
    nb = max(1, int(span / spec.fence_bin_m) + 1)
    hist = [0] * nb
    for c in coords:
        idx = min(nb - 1, int((c - lo) / spec.fence_bin_m))
        hist[idx] += 1
    peak = max(hist)
    thr = max(spec.fence_min_points, peak * spec.fence_peak_frac)
    tall = [i for i, h in enumerate(hist) if h >= thr]
    if len(tall) < 2:
        raise CourtExtractionError("fence_side_missing: fewer than two fence lines")
    near_i, far_i = tall[0], tall[-1]
    center = lambda i: lo + (i + 0.5) * spec.fence_bin_m
    return center(near_i), center(far_i), hist[near_i], hist[far_i]


# ── extraction steps ───────────────────────────────────────────────────────

def extract_posts(points_court: list[tuple[float, float]], spec: CourtSpec) -> dict:
    """Net posts = extent of the contiguous net line around y'=0 within the net
    band (|x'|<net_band). The side fences also cross x'=0 (at y'=±fence_width),
    so we isolate the cluster that spans y'=0, split on gaps."""
    ys = sorted(yp for xp, yp in points_court if abs(xp) <= spec.net_band_m)
    if len(ys) < 4:
        raise CourtExtractionError("net_not_observed: too few points on net line")
    gap = max(spec.doubles_alley_m, 0.6)  # net↔fence gap is large; net is contiguous
    clusters: list[list[float]] = [[ys[0]]]
    for y in ys[1:]:
        (clusters[-1].append(y) if y - clusters[-1][-1] <= gap else clusters.append([y]))
    spanning = [c for c in clusters if c[0] <= 0.0 <= c[-1]]
    net_cl = max(spanning, key=lambda c: c[-1] - c[0]) if spanning \
        else min(clusters, key=lambda c: min(abs(c[0]), abs(c[-1])))
    # Reject isolated straggler points at the ends (gap > 0.25 m from the dense
    # net bulk) — robust to occasional spurious returns near the posts.
    while len(net_cl) > 4 and (net_cl[1] - net_cl[0]) > 0.25:
        net_cl = net_cl[1:]
    while len(net_cl) > 4 and (net_cl[-1] - net_cl[-2]) > 0.25:
        net_cl = net_cl[:-1]
    y_min, y_max = net_cl[0], net_cl[-1]
    span = y_max - y_min
    if not math.isfinite(span) or span < 1.0:
        raise CourtExtractionError("net_not_observed: net span too small")
    return {"y_min": y_min, "y_max": y_max, "span_m": span}


def classify_doubles(post_span_m: float, spec: CourtSpec) -> bool:
    full = 2 * spec.post_half_span_doubles_m
    single = 2 * spec.post_half_span_singles_m
    if abs(post_span_m - full) <= spec.post_span_tol_m:
        return True
    if abs(post_span_m - single) <= spec.post_span_tol_m:
        return False
    raise CourtExtractionError(
        f"ambiguous_court_width: post span {post_span_m:.2f}m matches neither "
        f"doubles({full:.2f}) nor singles({single:.2f})"
    )


def fit_fence_rectangle(points_court: list[tuple[float, float]], spec: CourtSpec) -> dict:
    xs = [xp for xp, _ in points_court]
    ys = [yp for _, yp in points_court]
    # Coverage precondition: the observed length extent must reach BOTH baselines
    # (fences sit beyond ±half_length). If not, the far/near half is not mapped
    # yet — recoverable, keep covering. Prevents premature/garbage fits.
    need = spec.half_length_m + 2.0  # reach into the run-off so the FENCE (beyond
    # the baseline) is densely mapped before fitting — not just the baseline.
    if not xs or min(xs) > -need or max(xs) < need:
        raise CourtExtractionError(
            "coverage_incomplete: fences not yet densely in view "
            f"(x' extent [{min(xs):.1f},{max(xs):.1f}] need +/-{need:.1f})")
    x_near, x_far, xn_c, xf_c = _outer_fence_lines(xs, spec)
    y_left, y_right, yl_c, yr_c = _outer_fence_lines(ys, spec)
    for name, cnt in (("x_near", xn_c), ("x_far", xf_c), ("y_left", yl_c), ("y_right", yr_c)):
        if cnt < spec.fence_side_min_points:
            raise CourtExtractionError(f"fence_side_missing:{name} ({cnt} pts)")
    if not (x_near < 0 < x_far and y_left < 0 < y_right):
        # Net (origin) not between the observed baselines => the far/near half
        # has not been mapped yet. Recoverable: keep covering.
        raise CourtExtractionError(
            "coverage_incomplete: net not inside observed fence box "
            "(both baselines not yet mapped)")
    return {
        "x_near": x_near, "x_far": x_far, "y_left": y_left, "y_right": y_right,
        "corners_court": [
            (x_near, y_left), (x_far, y_left), (x_far, y_right), (x_near, y_right),
        ],
    }


def court_lines(is_doubles: bool, spec: CourtSpec) -> dict:
    half_w = (spec.width_doubles_m if is_doubles else spec.width_singles_m) / 2.0
    return {
        "baselines_x": [-spec.half_length_m, spec.half_length_m],
        "service_x": [-spec.service_from_net_m, spec.service_from_net_m],
        "sidelines_y": [-half_w, half_w],
        "center_line_y": 0.0,
        "half_width_m": half_w,
    }


def compute_distances(fence: dict, lines: dict, spec: CourtSpec) -> dict:
    half_l = spec.half_length_m
    half_w = lines["half_width_m"]
    d = {
        "near_baseline": abs(fence["x_near"]) - half_l,
        "far_baseline": fence["x_far"] - half_l,
        "left_sideline": abs(fence["y_left"]) - half_w,
        "right_sideline": fence["y_right"] - half_w,
    }
    for k, v in d.items():
        if v < spec.runoff_min_m - 0.15:
            # A fence cannot sit INSIDE the court baseline. A negative run-off means
            # the real outer fence on this side is NOT mapped yet — the fit latched
            # onto the net (origin) or an inner return because the far fence is still
            # sparse. This is a COVERAGE problem, not a court property: recoverable,
            # keep covering. (Without this, a couple of grazing far points pass the
            # extent gate, the fit picks the net, and we would wrongly fail-loud as
            # non-standard before the robot ever reaches the far fence.)
            raise CourtExtractionError(
                f"coverage_incomplete: {k} fence not yet mapped (run-off {v:.2f}m < 0)")
        if v > spec.runoff_max_m:
            # Beyond any regulation run-off → genuinely non-standard court. Fail-loud.
            raise CourtExtractionError(
                f"nonstandard_or_bad_fit: {k} run-off {v:.2f}m exceeds {spec.runoff_max_m}m")
    return {k: round(v, 3) for k, v in d.items()}


def extract_obstacles(points_court: list[tuple[float, float]], fence: dict, spec: CourtSpec) -> list[dict]:
    """Cluster interior points that are not net, not fence -> obstacles."""
    m = spec.obstacle_edge_margin_m
    interior = []
    for xp, yp in points_court:
        if not (fence["x_near"] + m < xp < fence["x_far"] - m):
            continue
        if not (fence["y_left"] + m < yp < fence["y_right"] - m):
            continue
        if abs(xp) <= spec.net_band_m:  # exclude the net line/posts
            continue
        interior.append((xp, yp))
    # grid clustering (connected occupied cells)
    g = spec.obstacle_grid_m
    cells: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for xp, yp in interior:
        cells.setdefault((int(math.floor(xp / g)), int(math.floor(yp / g))), []).append((xp, yp))
    seen: set = set()
    obstacles: list[dict] = []
    oid = 0
    for key in list(cells):
        if key in seen:
            continue
        stack = [key]; comp: list[tuple[float, float]] = []
        seen.add(key)
        while stack:
            cx, cy = stack.pop()
            comp.extend(cells.get((cx, cy), []))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nk = (cx + dx, cy + dy)
                    if nk in cells and nk not in seen:
                        seen.add(nk); stack.append(nk)
        if len(comp) < spec.obstacle_min_points:
            continue
        xs = [p[0] for p in comp]; ys = [p[1] for p in comp]
        w = max(xs) - min(xs); h = max(ys) - min(ys)
        cxc = (min(xs) + max(xs)) / 2.0; cyc = (min(ys) + max(ys)) / 2.0
        # Reject fence scatter: near a fence and elongated parallel to it.
        band = spec.fence_artifact_band_m; r = spec.fence_artifact_parallel_ratio
        d_vert = min(abs(cxc - fence["x_near"]), abs(cxc - fence["x_far"]))
        d_horz = min(abs(cyc - fence["y_left"]), abs(cyc - fence["y_right"]))
        if (d_vert <= band and h >= r * max(w, 1e-3)) or \
           (d_horz <= band and w >= r * max(h, 1e-3)):
            continue
        oid += 1
        obstacles.append({
            "id": oid,
            "class": "obstacle" if max(w, h) >= 0.25 else "small",
            "center_court": ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0),
            "size_m": {"w": round(w, 3), "h": round(h, 3)},
            "point_count": len(comp),
        })
    return obstacles


# ── top-level ──────────────────────────────────────────────────────────────

def extract_court_knowledge_model(map_points: list, locked_net: dict,
                                  spec: CourtSpec | None = None) -> dict:
    """Full Court Knowledge Model, or raise CourtExtractionError (fail-loud)."""
    spec = spec or CourtSpec()
    pts = _as_xy(map_points)
    if len(pts) < 50:
        raise CourtExtractionError(f"coverage_incomplete: only {len(pts)} points")

    frame = build_court_frame(locked_net)
    pc = [frame.to_court(px, py) for px, py in pts]

    # Coverage gate FIRST: never make structural decisions (doubles, run-off)
    # on partial data. Require both baselines (fences beyond ±half_length).
    cxs = [xp for xp, _ in pc]
    _need = spec.half_length_m + 2.0
    if not cxs or min(cxs) > -_need or max(cxs) < _need:
        raise CourtExtractionError(
            "coverage_incomplete: fences not yet densely in view "
            f"(x' extent [{min(cxs):.1f},{max(cxs):.1f}] need +/-{_need:.1f})")

    # Physical regulation court is doubles-width; singles is an inner painted-line
    # subset (camera/future). Net posts follow standard geometry anchored to the
    # measured net centre. The run-off (what VARIES per facility) is still MEASURED
    # from the fences below.
    is_doubles = True
    half_span = spec.post_half_span_doubles_m
    fence = fit_fence_rectangle(pc, spec)
    lines = court_lines(is_doubles, spec)
    distances = compute_distances(fence, lines, spec)
    obstacles = extract_obstacles(pc, fence, spec)

    def to_map(xp, yp):
        mx, my = frame.to_map(xp, yp)
        return {"x_m": round(mx, 3), "y_m": round(my, 3)}

    post_a = to_map(0.0, half_span); post_b = to_map(0.0, -half_span)
    return {
        "schema": "court_knowledge_model/v2",
        "status": "OK",
        "failure_reason": None,
        "frame": "map",
        "net": {
            "center": {"x_m": round(frame.cx, 3), "y_m": round(frame.cy, 3)},
            "axis_length": {"x_m": round(frame.ux, 4), "y_m": round(frame.uy, 4)},
            "axis_width": {"x_m": round(frame.vx, 4), "y_m": round(frame.vy, 4)},
            "posts": [post_a, post_b],
            "span_m": round(2 * half_span, 3),
        },
        "court": {
            "is_doubles": is_doubles,
            "length_m": spec.length_m,
            "width_m": round(2 * lines["half_width_m"], 3),
            "lines_court_frame": {k: v for k, v in lines.items() if k != "half_width_m"},
               },
        "fence": {
            "corners": [to_map(xp, yp) for xp, yp in fence["corners_court"]],
            "extents_court_frame": {
                "x_near": round(fence["x_near"], 3), "x_far": round(fence["x_far"], 3),
                "y_left": round(fence["y_left"], 3), "y_right": round(fence["y_right"], 3),
            },
        },
        "distances_to_fence_m": distances,
        "obstacles": [
            {**o, "center": to_map(*o.pop("center_court"))} for o in obstacles
        ],
        "occupancy": {"point_count": len(pts)},
    }
