#!/usr/bin/env python3
"""P2-57 printed-front-part sweep against the frozen P2-53 fixed world."""
from __future__ import annotations
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "cad/modular-test-chassis/shared-flywheel-transition-p252.scad"
OUT = ROOT / "engineering/chassis-strength-bench/results/shared_flywheel_phase2/p257_cheek_clearance.json"
OPENSCAD = "/snap/bin/openscad-nightly"
VARIANTS = ("p257_shield_m90", "p257_shield_m100", "p257_shield_m110",
            "p257_stagger_open", "p257_stagger_shield")


def run(view: str, variant: str, angle: int, output: Path) -> tuple[int, str]:
    cmd = [OPENSCAD, "-o", str(output), "-D", f'p253_view="{view}"',
           "-D", f'p257_variant="{variant}"', "-D", f"p253_angle={angle}", str(CAD)]
    proc = subprocess.run(cmd, text=True, capture_output=True, timeout=180)
    return proc.returncode, proc.stdout + proc.stderr


def main() -> None:
    work = OUT.parent / ".p257_clearance_work"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    report = {}
    try:
        for variant in VARIANTS:
            rows = []
            for angle in range(-10, 21):
                target = work / f"{variant}_{angle}.stl"
                returncode, log = run("qa_p257_fixed_at_angle", variant, angle, target)
                empty = "Current top level object is empty." in log and "ERROR:" not in log
                # OpenSCAD returns 1 when an intentionally empty intersection
                # has no exportable top-level object.  That is the PASS signal.
                if returncode != 0 and not empty:
                    raise RuntimeError(
                        f"OpenSCAD failed for {variant} at {angle} deg "
                        f"(return code {returncode}):\n{log}"
                    )
                rows.append({"angle_deg": angle, "zero_intersection": empty})
            # Analytic lower-edge screen of the exact front-part primitives.
            # z_global = pivot_z + sin(angle)*x + cos(angle)*z for rotate(-angle,Y).
            primitives = [(105, -10), (180, -10), (130, -12)]
            if variant.startswith("p257_stagger"):
                primitives += [(180, 5), (250, 5)]
            import math
            ground = []
            for angle in range(-10, 21):
                a = math.radians(angle)
                z = min(51.837363636 + math.sin(a) * x + math.cos(a) * local_z
                        for x, local_z in primitives)
                ground.append((z, angle))
            worst = min(ground)
            report[variant] = {
                "angles": rows,
                "zero_intersection_all_angles": all(row["zero_intersection"] for row in rows),
                "minimum_ground_clearance_mm": round(worst[0], 3),
                "governing_ground_angle_deg": worst[1],
            }
    finally:
        shutil.rmtree(work, ignore_errors=True)
    result = {
        "status": "PASS" if all(row["zero_intersection_all_angles"] and row["minimum_ground_clearance_mm"] > 0
                                  for row in report.values()) else "FAIL",
        "scope": "P2-57 printed cheeks/shields only; P2-40 core and P2-51 ski/basket frozen",
        "angle_sweep_deg": [-10, 20, 1],
        "variants": report,
        "no_stl_release": True,
    }
    OUT.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
