#!/usr/bin/env python3
"""Decompose selected corrected compact launcher translations with exact CGAL."""

from __future__ import annotations

import argparse
import json
import math
import tempfile
from pathlib import Path

from analyze_corrected_compact_packaging_resolution import ROOT, exact_many


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--z", nargs="+", type=int, default=[33, 35, 38])
    args = parser.parse_args()
    result = {"units": "mm", "status": "ANALYSIS_ONLY", "candidates": {}}
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="launcher-decompose-", dir=runtime) as temp:
        directory = Path(temp)
        for dz in args.z:
            specs = []
            common = dict(launcher_dx=0, launcher_dy=0, launcher_dz=dz)
            for moving in ("motor", "wheel", "direct_drive_hardware"):
                for launcher in ("cradle", "left_flywheel", "right_flywheel"):
                    specs.append((f"raw_{moving}|{launcher}", dict(
                        part="swept_launcher_intersection", component=moving,
                        launcher_component=launcher, selected_side=0, **common)))
            for gap in (0, 2):
                for moving in ("motor", "direct_drive_hardware"):
                    specs.append((f"cradle_relief_{moving}_{gap}", dict(
                        part="cradle_relief_intersection", component=moving,
                        selected_side=0, clearance=gap, **common)))
                for obstacle in ("basket", "hood", "bridge", "chassis", "battery", "lidar"):
                    specs.append((f"combined_gap{gap}_assembly|{obstacle}", dict(
                        part="launcher_combined_environment_intersection",
                        launcher_component="assembly", target=obstacle,
                        clearance=gap, pocket_clearance=2, **common)))
            specs.append(("hard_2mm", dict(
                part="hard_conflict_probe", component="direct_drive_hardware",
                selected_side=0, clearance=2, pocket_clearance=2, **common)))
            records = exact_many(args.openscad, directory, specs)
            outlet_x = 460 + 320 * math.cos(math.radians(20))
            outlet_z = 215 + dz + 320 * math.sin(math.radians(20))
            result["candidates"][f"0,0,{dz}"] = {
                "translation_mm": [0, 0, dz],
                "launcher_nip_world_mm": [460, 0, 215 + dz],
                "launcher_exit_guide_outlet_world_mm": [round(outlet_x, 3), 0, round(outlet_z, 3)],
                "records": records,
            }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value["records"]["hard_2mm"]
                      for key, value in result["candidates"].items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
