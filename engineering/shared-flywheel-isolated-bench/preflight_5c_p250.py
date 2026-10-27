#!/usr/bin/env python3
"""Preflight the frozen P2-50 one-ball high-ramp campaign."""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--ball", type=Path, required=True)
    ap.add_argument("--image-digest", required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    errors = []
    criteria = json.loads(a.criteria.read_text())
    allowed_status = {
        "FROZEN_BEFORE_FIRST_P2_50_GAZEBO_RUN",
        "FROZEN_AFTER_PIECEWISE_FAILURE_BEFORE_R125_RERUN",
        "FROZEN_AFTER_R125_LATERAL_ESCAPE_BEFORE_CONTINUOUS_RAIL_RERUN",
    }
    if criteria.get("status") not in allowed_status:
        errors.append("criteria not frozen")
    frozen = criteria.get("frozen_inputs", {})
    if frozen.get("flywheel_surface_speeds_m_s") != [2.8, 3.0, 3.2]: errors.append("speed matrix mismatch")
    if frozen.get("lateral_ball_offsets_m") != [-0.05, 0.0, 0.05]: errors.append("lateral matrix mismatch")
    if frozen.get("ball_tread_mu") != 0.6 or frozen.get("approach_speed_m_s") != 0.6: errors.append("mu/approach mismatch")

    root = ET.parse(a.urdf).getroot()
    names = {node.get("name") for node in root.findall(".//collision") if node.get("name")}
    variant = frozen.get("ramp_variant")
    if variant in ("p249_r125", "p249_r125_continuous_rails"):
        required = {f"intake_fixed_ramp_floor_{i}r125_col" for i in range(11)}
        forbidden = {
            "intake_fixed_ramp_floor_0_col", "intake_fixed_ramp_floor_1_col",
            "intake_fixed_ramp_floor_2_col", "intake_fixed_ramp_floor_11t_col",
            "intake_fixed_ramp_floor_2p249_col", "basket_floor_col",
        }
        if variant == "p249_r125_continuous_rails":
            required |= {f"intake_fixed_ramp_rail_{i}r125_{side}_col"
                         for i in range(11) for side in (-1, 1)}
    else:
        required = {f"intake_fixed_ramp_floor_{i}p249_col" for i in range(2, 8)}
        forbidden = {"intake_fixed_ramp_floor_2_col", "intake_fixed_ramp_floor_11t_col", "basket_floor_col"}
    required |= {"basket_p249_rear_floor_col", "basket_p249_divider_mid_col"}
    if not required <= names:
        errors.append(f"missing P2-49 fixtures: {sorted(required-names)}")
    if forbidden & names:
        errors.append(f"superseded fixtures present: {sorted(forbidden & names)}")
    if not a.image_digest.startswith("sha256:"):
        errors.append("container image digest missing")

    result = {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "criteria_sha256": sha(a.criteria),
        "urdf_sha256": sha(a.urdf),
        "tennis_ball_compliant_sha256": sha(a.ball),
        "container_image_digest": a.image_digest,
        "required_collision_count": len(required),
        "frozen_matrix_conditions": 9,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
