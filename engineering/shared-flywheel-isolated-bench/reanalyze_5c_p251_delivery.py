#!/usr/bin/env python3
"""Re-run P2-51 analysis from preserved telemetry; does not run Gazebo."""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

BENCH = Path("engineering/shared-flywheel-isolated-bench")
ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_delivery_matrix")
CRITERIA = BENCH / "criteria_5c_p251_delivery_matrix.json"

count = 0
for summary in sorted(ROOT.glob("a_42/v_*/tm_*/rm_*/y_*/summary.json")):
    condition = json.loads(summary.read_text())["condition"]
    directory = summary.parent
    subprocess.run([
        "python3", str(BENCH / "analyze_5c_p251_run.py"),
        str(directory / "telemetry.csv"), str(directory / "ball_contacts.csv"),
        str(directory / "native_contacts.csv"), "--criteria", str(CRITERIA),
        "--angle", "42", "--surface-speed", str(condition["surface_speed_m_s"]),
        "--tread-mu", str(condition["tread_mu"]), "--ramp-mu", str(condition["ramp_mu"]),
        "--lateral", str(condition["lateral_m"]), "--output", str(summary),
    ], check=True, stdout=subprocess.DEVNULL)
    count += 1
print(f"reanalyzed {count} preserved P2-51 runs; Gazebo not invoked")
