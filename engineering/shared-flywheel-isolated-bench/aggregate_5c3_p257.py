#!/usr/bin/env python3
"""Aggregate the complete, predeclared P2-57 matrix and rank variants."""
from __future__ import annotations
import json
from pathlib import Path

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p257")
VARIANTS = ("p257_shield_m90", "p257_shield_m100", "p257_shield_m110",
            "p257_stagger_open", "p257_stagger_shield")
MUS = ("0p6", "0p9")
SIDE = ("side_y35", "side_y50", "side_y100")
SERIES = ("series_gap_0", "series_gap_50", "series_gap_150", "series_gap_300")
OFFSETS = (0, -20, 20, -40, 40, -60, 60, -80, 80, -100, 100)


def single_name(offset: int) -> str:
    if offset == 0:
        return "single_y0"
    return f"single_y{'p' if offset > 0 else 'm'}{abs(offset)}"


def load(variant: str, mu: str, case: str) -> dict:
    folder = ROOT / "matrix" / variant / f"tm_{mu}" / case
    summary = json.loads((folder / "summary.json").read_text())
    summary["record_present"] = (folder / "replay" / "state.tlog").exists()
    return summary


def capture_half_width(singles: dict[int, bool]) -> int:
    width = 0
    for candidate in (20, 40, 60, 80, 100):
        if singles.get(candidate) and singles.get(-candidate):
            width = candidate
        else:
            break
    return width if singles.get(0) else 0


def main() -> None:
    all_runs, variants = [], []
    for variant in VARIANTS:
        runs = [load(variant, mu, case) for mu in MUS
                for case in (*SIDE, *SERIES, *(single_name(value) for value in OFFSETS))]
        all_runs.extend(runs)
        side = [row for row in runs if row["case"] in SIDE]
        series = [row for row in runs if row["case"] in SERIES]
        captures_by_mu = {}
        for mu in (.6, .9):
            singles = {offset: next(row for row in runs
                                    if row["tread_mu"] == mu and row["case"] == single_name(offset))
                       for offset in OFFSETS}
            captures_by_mu[str(mu)] = {
                "retained_by_offset_mm": {str(offset): row["metrics"]["balls_in_basket"] == 1
                                           for offset, row in singles.items()},
                "symmetric_capture_half_width_mm": capture_half_width({
                    offset: row["metrics"]["balls_in_basket"] == 1 for offset, row in singles.items()
                }),
            }
        rejected = [ball for row in runs for ball in row["metrics"]["per_ball"].values()
                    if not ball["retained"]]
        side_rejected = [ball for row in side for ball in row["metrics"]["per_ball"].values()
                         if not ball["retained"]]
        minimum_capture = min(row["symmetric_capture_half_width_mm"] for row in captures_by_mu.values())
        digest = {
            "variant": variant,
            "side_runs": len(side),
            "side_jams": sum(row["metrics"]["jammed"] for row in side),
            "side_balls_in_basket": sum(row["metrics"]["balls_in_basket"] for row in side),
            "side_case_results": [{
                "case": row["case"],
                "tread_mu": row["tread_mu"],
                "jammed": row["metrics"]["jammed"],
                "balls_in_basket": row["metrics"]["balls_in_basket"],
                "balls_rejected": row["metrics"]["balls_rejected"],
                "rejected_free_on_court": row["metrics"]["balls_free_on_court"],
                "maximum_rejected_forward_push_m": row["metrics"]["maximum_rejected_forward_push_m"],
            } for row in side],
            "side_rejected_balls": len(side_rejected),
            "side_rejected_free_on_court": sum(ball["free_on_court"] for ball in side_rejected),
            "side_maximum_rejected_forward_push_m": max(
                (ball["maximum_forward_push_m"] for ball in side_rejected), default=0.0),
            "series_passes": sum(row["metrics"]["balls_in_basket"] == 2 for row in series),
            "series_total": len(series),
            "capture_by_mu": captures_by_mu,
            "minimum_symmetric_capture_half_width_mm": minimum_capture,
            "rejected_balls": len(rejected),
            "rejected_free_on_court": sum(ball["free_on_court"] for ball in rejected),
            "maximum_rejected_forward_push_m": max((ball["maximum_forward_push_m"] for ball in rejected), default=0.0),
            "peak_current_a": max(row["metrics"]["max_current_a"] for row in runs),
            "all_recorded": all(row["record_present"] for row in runs),
        }
        digest["criteria"] = {
            "zero_side_by_side_jams": digest["side_jams"] == 0,
            "series_regression_8_of_8": digest["series_passes"] == 8,
            "rejected_ball_forward_push": (digest["rejected_balls"] == digest["rejected_free_on_court"]
                                             and digest["maximum_rejected_forward_push_m"] <= .300 + 1e-9),
            "capture_half_width": minimum_capture >= 40,
            "all_recorded": digest["all_recorded"],
        }
        digest["pass"] = all(digest["criteria"].values())
        variants.append(digest)
    ranked = sorted(variants, key=lambda row: (
        row["pass"], row["minimum_symmetric_capture_half_width_mm"],
        row["series_passes"], -row["side_jams"], -row["maximum_rejected_forward_push_m"]), reverse=True)
    commands = [f"engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh {row['variant']}__{row['case']} {row['tread_mu']}"
                for row in all_runs]
    report = {
        "status": "COMPLETE" if len(all_runs) == 180 else "INCOMPLETE",
        "setup": "unchanged P2-51/5c-3 contact model and calibration; printed cheeks/covers only",
        "run_count": len(all_runs),
        "variants": variants,
        "recommended_variant": ranked[0]["variant"],
        "recommendation_passes_all_criteria": ranked[0]["pass"],
        "all_recorded": all(row["record_present"] for row in all_runs),
        "exact_replay_pattern": "engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh <variant>__<case> <0.6|0.9>",
        "exact_replay_commands_file": str(ROOT / "replay_commands.txt"),
        "exact_replay_commands": commands,
    }
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    (ROOT / "replay_commands.txt").write_text("\n".join(commands) + "\n")
    print(json.dumps({"status": report["status"], "run_count": report["run_count"],
                      "recommended_variant": report["recommended_variant"],
                      "recommendation_passes_all_criteria": report["recommendation_passes_all_criteria"],
                      "all_recorded": report["all_recorded"]}, indent=2))


if __name__ == "__main__":
    main()
