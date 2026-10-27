#!/usr/bin/env python3
"""Run exact analysis-only hard-conflict probes at requested launcher points."""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

from analyze_corrected_compact_packaging_resolution import ROOT, exact_many


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openscad", default="/snap/bin/openscad-nightly")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--x", nargs="+", type=int, required=True)
    parser.add_argument("--z", nargs="+", type=int, required=True)
    args = parser.parse_args()
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="launcher-points-", dir=runtime) as temp:
        specs = []
        for dx in args.x:
            for dz in args.z:
                specs.append((f"x{dx:+d}_z{dz:+d}", dict(
                    part="hard_conflict_probe", component="direct_drive_hardware",
                    selected_side=0, launcher_dx=dx, launcher_dy=0,
                    launcher_dz=dz, clearance=2, pocket_clearance=2)))
        records = exact_many(args.openscad, Path(temp), specs)
    viable = [name for name, record in records.items()
              if record["intersection_volume_mm3"] == 0]
    payload = {"units": "mm", "clearance": 2, "records": records,
               "viable": viable}
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"viable": viable}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
