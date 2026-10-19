#!/usr/bin/env python3
"""0a/0b preflight for the P2-51-derived 5c-3 harness."""
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
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--ball", type=Path, required=True)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--world", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    criteria = json.loads(args.criteria.read_text())
    errors = []
    setup = criteria.get("frozen_setup", {})
    if criteria.get("status") != "FROZEN_BEFORE_CORRECTED_TWO_BALL_P2_55_MATRIX":
        errors.append("criteria are not frozen")
    if (setup.get("ski_angle_deg_geometric"), setup.get("ski_radius_mm"), setup.get("wheel_surface_speed_m_s")) != (42, 75, 6.0):
        errors.append("P2-51 geometry or selected speed changed")
    if setup.get("conditions") != 12 or setup.get("tread_mu") != [0.6, 0.9]:
        errors.append("corrected two-ball matrix mismatch")
    urdf = ET.parse(args.urdf).getroot()
    names = {node.get("name") for node in urdf.findall(".//collision") if node.get("name")}
    required = {"intake_fixed_ramp_floor_p251_flat_col", "intake_fixed_ramp_floor_p251_42_col",
                "basket_p251_center_floor_col", "basket_p251_rear_backboard_col", "fixture_camera_body_col"}
    if not required <= names:
        errors.append(f"0a missing P2-51 collisions: {sorted(required - names)}")
    world = ET.parse(args.world).getroot()
    ball_names = sorted(model.get("name") for model in world.findall(".//world/model")
                        if (model.get("name") or "").startswith("capture_ball_"))
    if ball_names != ["capture_ball_00", "capture_ball_01"]:
        errors.append(f"0b two-ball spawn mismatch: {ball_names}")
    camera_pose = world.findtext(".//world/gui/plugin/camera_pose")
    if camera_pose != "0.55 -1.45 0.38 0 0.10 1.5708":
        errors.append("0b side camera configuration changed")
    result = {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "gates": {
            "0a_frozen_p251_geometry_and_criteria": not any(e.startswith("criteria") or e.startswith("P2-51") or e.startswith("0a") for e in errors),
            "0b_two_ball_spawn_observer_and_side_camera": not any(e.startswith("0b") for e in errors),
        },
        "criteria_sha256": sha(args.criteria),
        "ball_sha256": sha(args.ball),
        "urdf_sha256": sha(args.urdf),
        "world_sha256": sha(args.world),
        "ball_names": ball_names,
        "camera_pose_xyz_rpy": camera_pose,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    raise SystemExit(bool(errors))


if __name__ == "__main__":
    main()
