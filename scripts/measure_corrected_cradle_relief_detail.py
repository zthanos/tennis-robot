#!/usr/bin/env python3
"""Measure per-side cradle reliefs in world and rigid-launcher coordinates."""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import numpy as np

from analyze_compact_intake_pod_concepts import bbox, intersection_record, stl_vertices
from analyze_corrected_compact_packaging_resolution import ROOT, launcher_local, render


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--z", nargs="+", type=int, default=[33, 35, 38])
    args = parser.parse_args()
    result = {"units": "mm", "plate_local_bounds_mm":
              {"x": [-128, 128], "y": [-47, -39], "z": [223, 537]},
              "candidates": {}}
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="cradle-detail-", dir=runtime) as temp:
        directory = Path(temp)
        for dz in args.z:
            candidate = {}
            for gap in (0, 2):
                for side, label in ((1, "left"), (-1, "right")):
                    name = f"z{dz}_{label}_{gap}"
                    path = render(args.openscad, directory, name,
                                  part="plate_relief_intersection", component="motor",
                                  selected_side=side, selected_plate=-1,
                                  clearance=gap, launcher_dz=dz)
                    record = intersection_record(path)
                    if path is not None:
                        vertices = stl_vertices(path)
                        relative = vertices - np.asarray([0.0, 0.0, dz])
                        record["launcher_local_bbox_mm"] = bbox(launcher_local(relative))
                    candidate[f"{label}_{gap}mm"] = record
            result["candidates"][str(dz)] = candidate
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["candidates"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
