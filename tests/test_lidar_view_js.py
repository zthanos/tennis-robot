from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts/control_panel/lidar_view.js"


@pytest.mark.skipif(shutil.which("node") is None, reason="Node.js is unavailable")
def test_lidar_ui_transformation_reports_live_and_stale_without_source_type() -> None:
    program = f"""
const view = require({json.dumps(str(MODULE))});
const sensor = {{
  ranges_m: [1.0, null, 2.0, 0.01, 20.0],
  last_message_at_s: 10.0,
  frame_id: "lidar_link",
  scan_rate_hz: 10.0,
  sample_count: 5,
  valid_sample_count: 2,
  invalid_sample_count: 3,
  angle_min_rad: -Math.PI,
  angle_max_rad: Math.PI,
  angle_increment_rad: Math.PI / 360,
  range_min_m: 0.05,
  range_max_m: 16.0
}};
console.log(JSON.stringify({{
  live: view.derive(sensor, 11000),
  stale: view.derive(sensor, 14000)
}}));
"""
    completed = subprocess.run(
        ["node", "-e", program],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)

    assert result["live"]["state"] == "live"
    assert result["stale"]["state"] == "stale"
    assert result["live"]["age_s"] == 1
    assert result["live"]["nearest_m"] == 1
    assert result["live"]["frame_id"] == "lidar_link"
    assert "source" not in result["live"]
