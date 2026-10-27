#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p258")
DX = (0, 35, 70, 100, 150)
DY = (0, 20, 35, 50, 70, 100)


def feasible(dx: int, dy: int) -> bool:
    return math.hypot(dx, dy) >= 66.0 - 1e-9


def orientations(mu: float, dy: int) -> tuple[str, ...]:
    if dy == 0:
        return ("center",)
    return ("lead_plus", "lead_minus") if mu == .6 else ("lead_plus",)


def folder(mu: float, orientation: str, dx: int, dy: int) -> Path:
    return ROOT / "matrix" / f"tm_{str(mu).replace('.', 'p')}" / orientation / f"dx_{dx}" / f"dy_{dy}"


def classify(row: dict) -> str:
    metrics = row["metrics"]
    if metrics["balls_in_basket"] == 2:
        return "PASS"
    return "JAM" if metrics["jammed"] else "REJECT"


def load_all() -> dict[tuple[float, str, int, int], dict]:
    rows = {}
    for mu in (.6, .9):
        for dx in DX:
            for dy in DY:
                if not feasible(dx, dy):
                    continue
                for orientation in orientations(mu, dy):
                    path = folder(mu, orientation, dx, dy) / "summary.json"
                    row = json.loads(path.read_text())
                    row["classification"] = classify(row)
                    row["dx_mm"], row["dy_mm"], row["orientation"] = dx, dy, orientation
                    row["record_present"] = (path.parent / "replay" / "state.tlog").exists()
                    rows[(mu, orientation, dx, dy)] = row
    return rows


def map_cell(rows: dict, mu: float, map_orientation: str, dx: int, dy: int) -> str:
    if not feasible(dx, dy):
        return "N/A"
    orientation = "center" if dy == 0 else map_orientation
    return rows[(mu, orientation, dx, dy)]["classification"]


def maps(rows: dict) -> list[dict]:
    result = []
    for mu, orientation in ((.6, "lead_plus"), (.6, "lead_minus"), (.9, "lead_plus")):
        result.append({
            "tread_mu": mu,
            "lead_side": orientation,
            "columns_dy_mm": list(DY),
            "rows": [{"dx_mm": dx, "classification": [map_cell(rows, mu, orientation, dx, dy) for dy in DY]}
                     for dx in DX],
        })
    return result


def recording_set(rows: dict) -> set[tuple[float, str, int, int]]:
    selected = {key for key, row in rows.items() if row["classification"] != "PASS"}
    for mu, map_orientation in ((.6, "lead_plus"), (.6, "lead_minus"), (.9, "lead_plus")):
        for ix, dx in enumerate(DX):
            for iy, dy in enumerate(DY):
                if map_cell(rows, mu, map_orientation, dx, dy) != "PASS":
                    continue
                marginal = False
                for jx, jy in ((ix-1, iy), (ix+1, iy), (ix, iy-1), (ix, iy+1)):
                    if 0 <= jx < len(DX) and 0 <= jy < len(DY):
                        neighbor = map_cell(rows, mu, map_orientation, DX[jx], DY[jy])
                        if neighbor in ("JAM", "REJECT"):
                            marginal = True
                if marginal:
                    orientation = "center" if dy == 0 else map_orientation
                    selected.add((mu, orientation, dx, dy))
    return selected


def robust_frontier(rows: dict) -> list[dict]:
    result = []
    for dx_min in DX:
        passing_dy = 0
        for dy_limit in DY:
            cells = []
            for mu in (.6, .9):
                for dx in DX:
                    for dy in DY:
                        if dx < dx_min or dy > dy_limit or not feasible(dx, dy):
                            continue
                        for orientation in orientations(mu, dy):
                            cells.append(rows[(mu, orientation, dx, dy)]["classification"])
            if cells and all(value == "PASS" for value in cells):
                passing_dy = dy_limit
            else:
                break
        result.append({"minimum_dx_mm": dx_min, "guaranteed_dy_mm": passing_dy})
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-record-manifest", action="store_true")
    args = ap.parse_args()
    rows = load_all()
    selected = recording_set(rows)
    manifest = ROOT / "record_manifest.txt"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text("".join(f"{dx} {dy} {orientation} {mu}\n"
                                for mu, orientation, dx, dy in sorted(selected)) )
    if args.write_record_manifest:
        print(json.dumps({"screening_runs": len(rows), "record_cases": len(selected),
                          "manifest": str(manifest)}, indent=2))
        return
    map_data = maps(rows)
    classifications = {name: sum(row["classification"] == name for row in rows.values())
                       for name in ("PASS", "JAM", "REJECT")}
    jams = [row for row in rows.values() if row["classification"] == "JAM"]
    detector_hits = sum(row["metrics"]["d5065_detector_triggered"] for row in jams)
    passes = [row for row in rows.values() if row["classification"] == "PASS"]
    pass_detector_triggers = sum(row["metrics"]["d5065_detector_triggered"] for row in passes)
    frontier = robust_frontier(rows)
    # A planner release rule must cover the complete tested DY grid at its
    # minimum DX, not rely on N/A overlapping cells.  The first such row is
    # DX=70 mm (just over the 66 mm bench-ball diameter).
    releasable = [item for item in frontier if item["minimum_dx_mm"] >= 66]
    best = max(releasable, key=lambda item: (item["guaranteed_dy_mm"], -item["minimum_dx_mm"]))
    recorded = [rows[key] for key in selected]
    commands = [f"engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p258_dx{row['dx_mm']}_dy{row['dy_mm']}_{row['orientation']} {row['tread_mu']}"
                for row in recorded]
    report = {
        "status": "COMPLETE",
        "geometry_changed": False,
        "setup": "frozen P2-40 baseline cheeks plus unchanged P2-51/5c-3 calibration",
        "screening_runs": len(rows),
        "classification_counts": classifications,
        "jam_detector": {"jam_cases": len(jams), "triggered_on_jams": detector_hits,
                         "missed_jams": len(jams) - detector_hits,
                         "passing_cases": len(passes),
                         "triggers_on_passing_cases": pass_detector_triggers},
        "maps": map_data,
        "robust_frontier": frontier,
        "recommended_rule": {
            "minimum_dx_mm": best["minimum_dx_mm"],
            "maximum_dy_mm": best["guaranteed_dy_mm"],
            "minimum_physical_centre_distance_mm": 66,
            "text": f"for physical non-overlapping pairs, planner ensures longitudinal stagger dx >= {best['minimum_dx_mm']} mm; every tested |dy| through {best['guaranteed_dy_mm']} mm then passes. If dx is smaller, realign before intake",
        },
        "record_selection": {
            "rule": "every JAM/REJECT plus every edge-adjacent PASS",
            "required_count": len(selected),
            "present_count": sum(row["record_present"] for row in recorded),
            "all_present": all(row["record_present"] for row in recorded),
        },
        "exact_replay_commands": commands,
        "criteria_sha256": "",
    }
    criteria = Path("engineering/shared-flywheel-isolated-bench/criteria_5c3_p258.json")
    import hashlib
    report["criteria_sha256"] = hashlib.sha256(criteria.read_bytes()).hexdigest()
    (ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    (ROOT / "replay_commands.txt").write_text("\n".join(commands) + ("\n" if commands else ""))
    print(json.dumps({"status": report["status"], "screening_runs": len(rows),
                      "classifications": classifications, "jam_detector": report["jam_detector"],
                      "recommended_rule": report["recommended_rule"],
                      "records": report["record_selection"]}, indent=2))


if __name__ == "__main__":
    main()
