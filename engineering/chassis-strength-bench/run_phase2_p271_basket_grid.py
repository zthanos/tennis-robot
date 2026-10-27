#!/usr/bin/env python3
"""P2-71 geometry-only basket-grid audit from retained P2-69 traces."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
MATRIX = ROOT / "gazebo_p269/matrix/a_75/r_75"
OUT = ROOT / "p271_basket_grid.json"

BALL_RADIUS_MM = 33.0
REQUIRED_CLEARANCE_MM = 10.0
MAX_CHORD_MM = 0.1

FRONT = {"name": "front_grid_and_closed_door", "x_mm": 225.0,
         "thickness_mm": 4.0, "half_y_mm": 172.0,
         "bottom_z_mm": 46.1, "top_z_mm": 235.0}
REAR_X_MM = -118.0
REAR = {"name": "rear_upright_grid", "x_mm": REAR_X_MM,
        "thickness_mm": 4.0, "half_y_mm": 140.0,
        "bottom_z_mm": None, "top_z_mm": 320.0}

BALL_VOLUME_L = 4.0 / 3.0 * math.pi * 33.0**3 / 1e6
PACKING = 0.56


def floor_z_mm(x_mm: float) -> float:
    """Extend the accepted P2-51 floor line from (-85,70) to (278,42)."""
    return 70.0 + (x_mm + 85.0) * (42.0 - 70.0) / (278.0 + 85.0)


REAR["bottom_z_mm"] = floor_z_mm(REAR_X_MM)


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def ball_contact(row: dict[str, str], token: str) -> bool:
    pair = (row.get("collision_1") or "") + " " + (row.get("collision_2") or "")
    return "capture_ball" in pair and token in pair


def dense_flight(telemetry: list[dict[str, str]], start: float, end: float):
    raw = [
        (1000 * float(row["ball_base_x_m"]),
         1000 * float(row["ball_base_y_m"]),
         1000 * float(row["ball_base_z_m"]),
         float(row["time_s"]))
        for row in telemetry if start <= float(row["time_s"]) <= end
    ]
    dense = []
    for a, b in zip(raw, raw[1:]):
        subdivisions = max(1, math.ceil(math.dist(a[:3], b[:3]) / MAX_CHORD_MM))
        for index in range(subdivisions):
            u = index / subdivisions
            dense.append(tuple(a[k] + u * (b[k] - a[k]) for k in range(4)))
    if raw:
        dense.append(raw[-1])
    return dense


def panel_clearance(point, panel):
    x, y, z, timestamp = point
    dx = max(0.0, abs(x - panel["x_mm"]) - panel["thickness_mm"] / 2)
    dy = max(0.0, abs(y) - panel["half_y_mm"])
    dz = max(panel["bottom_z_mm"] - z, 0.0, z - panel["top_z_mm"])
    centre = math.sqrt(dx * dx + dy * dy + dz * dz)
    return centre - BALL_RADIUS_MM, {"time_s": timestamp, "ball_center_xyz_mm": [x, y, z]}


def solve_fill_plane(ball_count: int, slope: float) -> dict:
    target_mm3 = ball_count * BALL_VOLUME_L * 1e6 / PACKING
    # The fail-closed wall at X225 is the storage boundary. X225..278 is feed
    # handoff space and must not be credited as pile plan area.
    segments = [(-118.0, -15.0, 280.0), (-15.0, 225.0, 344.0)]

    def volume(c: float) -> float:
        total = 0.0
        step = 0.1
        for lo, hi, width in segments:
            x = lo
            while x < hi:
                total += width * max(0.0, c + slope * x - floor_z_mm(x)) * step
                x += step
        return total

    lo, hi = -100.0, 500.0
    for _ in range(80):
        mid = (lo + hi) / 2
        if volume(mid) < target_mm3:
            lo = mid
        else:
            hi = mid
    c = (lo + hi) / 2
    return {
        "surface_intercept_mm": c,
        "one_ball_allowance_mm": 66.0,
        "top_z_at_rear_wall_mm": c + slope * REAR_X_MM + 66.0,
        "top_z_at_front_wall_x225_mm": c + slope * 225.0 + 66.0,
        "maximum_top_z_mm": max(c + slope * x + 66.0 for x in (-118.0, 225.0)),
    }


def main() -> None:
    paths = sorted(MATRIX.glob("v_*/tm_*/y_*/e_0p3/telemetry.csv"))
    if len(paths) != 30:
        raise SystemExit(f"expected 30 retained P2-69 traces, found {len(paths)}")

    current = {"cases": len(paths), "front_wall_contact_cases": 0,
               "feed_door_contact_cases": 0, "rear_backboard_contact_cases": 0,
               "rear_backboard_samples": 0, "backboard_point_z_mm": [],
               "backboard_ball_center_z_mm": [],
               "backboard_begins_after_floor_cases": 0}
    minima = {FRONT["name"]: (math.inf, None, None), REAR["name"]: (math.inf, None, None)}
    case_rows = []

    for telemetry_path in paths:
        directory = telemetry_path.parent
        telemetry = read(telemetry_path)
        native = read(directory / "native_contacts.csv")
        summary = json.loads((directory / "summary.json").read_text())
        wall = [row for row in native if ball_contact(row, "p269_front_wall")]
        door = [row for row in native if ball_contact(row, "p269_feed_door")]
        back = [row for row in native if ball_contact(row, "rear_backboard")]
        floor = [row for row in native if ball_contact(row, "basket_p251_center_floor")]
        current["front_wall_contact_cases"] += bool(wall)
        current["feed_door_contact_cases"] += bool(door)
        current["rear_backboard_contact_cases"] += bool(back)
        current["rear_backboard_samples"] += len(back)
        current["backboard_point_z_mm"] += [1000 * float(row["point_z_m"]) for row in back]
        if back and floor and min(float(r["time_s"]) for r in back) >= min(float(r["time_s"]) for r in floor):
            current["backboard_begins_after_floor_cases"] += 1

        tele_by_time = {round(float(row["time_s"]), 5): row for row in telemetry}
        for row in back:
            match = tele_by_time.get(round(float(row["time_s"]), 5))
            if match:
                current["backboard_ball_center_z_mm"].append(
                    1000 * float(match["ball_base_z_m"]))

        takeoff = float(summary["metrics"]["takeoff_time_s"])
        first_floor = min(float(row["time_s"]) for row in floor if float(row["time_s"]) >= takeoff)
        trace = dense_flight(telemetry, takeoff, first_floor)
        row_out = {"case": str(directory.relative_to(MATRIX)), "takeoff_s": takeoff,
                   "first_floor_s": first_floor, "clearance_mm": {}}
        for panel in (FRONT, REAR):
            values = [panel_clearance(point, panel) for point in trace]
            clearance, state = min(values, key=lambda item: item[0])
            row_out["clearance_mm"][panel["name"]] = clearance
            if clearance < minima[panel["name"]][0]:
                minima[panel["name"]] = (clearance, str(directory), state)
        case_rows.append(row_out)

    for key in ("backboard_point_z_mm", "backboard_ball_center_z_mm"):
        values = current[key]
        current[key] = {"min": min(values), "max": max(values)}

    segments = ((REAR_X_MM, -15.0, 280.0), (-15.0, 225.0, 344.0))
    plan_area_mm2 = sum((hi-lo)*width for lo,hi,width in segments)
    def gross_to(top_z_mm: float) -> float:
        return sum(width*(hi-lo)*(top_z_mm-(floor_z_mm(lo)+floor_z_mm(hi))/2)
                   for lo,hi,width in segments)
    gross_235_mm3 = gross_to(235.0)
    gross_320_mm3 = gross_to(320.0)
    pile = {}
    for count in (40, 45):
        mean_bulk_h = count * BALL_VOLUME_L * 1e6 / PACKING / plan_area_mm2
        static_plane = solve_fill_plane(count, 0.0)
        braking_plane = solve_fill_plane(count, 0.2)
        opposite_plane = solve_fill_plane(count, -0.2)
        pile[str(count)] = {
            "solid_ball_volume_l": count * BALL_VOLUME_L,
            "mean_bulk_height_mm": mean_bulk_h,
            "P251_comparable_conservative_static_top_z_mm": floor_z_mm(REAR_X_MM) + mean_bulk_h + 66.0,
            "volume_conserving_static_plane": static_plane,
            "forward_braking_0p2g_plane": braking_plane,
            "opposite_acceleration_0p2g_plane": opposite_plane,
            "front_wall_margin_static_mm": 235.0-static_plane["top_z_at_front_wall_x225_mm"],
            "front_wall_margin_forward_braking_0p2g_mm": 235.0-braking_plane["top_z_at_front_wall_x225_mm"],
            "rear_wall_margin_opposite_acceleration_0p2g_mm": 320.0-opposite_plane["top_z_at_rear_wall_mm"],
        }

    clearance = {
        name: {"minimum_ball_surface_clearance_mm": value[0],
               "case_path": value[1], "state": value[2],
               "passes_10mm": value[0] >= REQUIRED_CLEARANCE_MM}
        for name, value in minima.items()
    }
    result = {
        "phase": "P2-71",
        "method": "geometry_only_from_retained_P2-69_R75_75_30_traces",
        "new_gazebo_runs": 0,
        "current_native_contact_audit": current,
        "geometry": {
            "front_grid": FRONT,
            "front_grid_max_opening_mm": 40.0,
            "solid_fail_closed_door_mm": [84.0, 84.0],
            "rear_grid": REAR,
            "rear_pod_front_x_mm": -125.0,
            "rear_grid_rear_face_x_mm": -120.0,
            "rear_pod_clearance_mm": 5.0,
            "floor_extension_mm": [[REAR_X_MM, floor_z_mm(REAR_X_MM)], [-85.0, 70.0]],
        },
        "flight_clearance": clearance,
        "basket_capacity": {
            "plan_area_mm2": plan_area_mm2,
            "storage_x_extent_mm": [REAR_X_MM, 225.0],
            "gross_volume_to_front_rim_z235_l": gross_235_mm3 / 1e6,
            "usable_volume_to_front_rim_after_existing_1p7l_reserve_l": gross_235_mm3 / 1e6 - 1.7,
            "gross_volume_to_side_rear_rim_z320_l": gross_320_mm3 / 1e6,
            "usable_volume_to_side_rear_rim_after_existing_1p7l_reserve_l": gross_320_mm3 / 1e6 - 1.7,
            "packing_fraction": PACKING,
            "pile": pile,
            "warning": "0.2g plane is a geometry screen, not a calibrated granular-ball dynamics claim",
        },
        "rows": case_rows,
    }
    OUT.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"native": current, "clearance": clearance,
                      "basket_capacity": result["basket_capacity"]}, indent=2))
    if not all(item["passes_10mm"] for item in clearance.values()):
        raise SystemExit("P2-71 STOP: proposed grid enters the required 10 mm flight clearance")


if __name__ == "__main__":
    main()
