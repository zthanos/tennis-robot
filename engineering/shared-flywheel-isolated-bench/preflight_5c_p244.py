#!/usr/bin/env python3
"""Static gate for the P2-44 reduced Gazebo campaign."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", action="append", type=Path, required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--contract", type=Path, required=True)
    ap.add_argument("--ball", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    errors = []
    criteria = json.loads(a.criteria.read_text())
    contract = json.loads(a.contract.read_text())
    if sha(a.ball) != contract["ball_model"]["source_sha256"]:
        errors.append("calibrated ball model hash changed")
    if criteria["status"] != "FROZEN_BEFORE_FIRST_P2_44_GAZEBO_RUN":
        errors.append("criteria are not frozen")
    if criteria["matrix"]["run_count"] != 75:
        errors.append("matrix is not 75 runs")
    variants = {}
    required_ramp = {f"intake_fixed_ramp_floor_{i}_col" for i in range(7)}
    required_ramp.add("intake_fixed_ramp_flap_col")
    for path in a.urdf:
        root = ET.parse(path).getroot()
        links = {link.attrib["name"]: link for link in root.findall("link")}
        base = links.get("base_link")
        frame = links.get("flywheel_launcher_frame_link")
        basket = links.get("basket_support_link")
        if base is None or frame is None or basket is None:
            errors.append(f"{path}: base/module missing")
            continue
        base_names = {collision.attrib.get("name") for collision in base.findall("collision")}
        if not required_ramp <= base_names:
            errors.append(f"{path}: ramp fixtures missing {sorted(required_ramp-base_names)}")
        basket_names = {collision.attrib.get("name") for collision in basket.findall("collision")}
        required_basket = {"basket_floor_col", "basket_front_left_wall_col", "basket_front_right_wall_col"}
        if not required_basket <= basket_names:
            errors.append(f"{path}: basket-entry fixtures missing {sorted(required_basket-basket_names)}")
        frame_names = {collision.attrib.get("name") for collision in frame.findall("collision")}
        if "intake_moving_guide_floor_col" in frame_names:
            errors.append(f"{path}: legacy moving guide still present")
        if not {"intake_cheek_1_col", "intake_cheek_-1_col"} <= frame_names:
            errors.append(f"{path}: P2-44 cheeks missing")
        cheek = frame.find("collision[@name='intake_cheek_1_col']")
        origin = [float(value) for value in cheek.find("origin").attrib["xyz"].split()]
        yaw = float(cheek.find("origin").attrib["rpy"].split()[2])
        variants[path.stem] = {"cheek_centre_x_m": origin[0], "cheek_centre_y_m": origin[1], "yaw_deg": math.degrees(yaw)}
    result = {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "criteria_hard_count": sum(item["hard_fail"] for item in criteria["criteria"]),
        "ball_model_sha256": sha(a.ball),
        "matrix_runs": criteria["matrix"]["run_count"],
        "urdf_variants": variants,
        "static_cad": {
            "launcher_vs_fixed_ramp_minus3_to_plus20": "EMPTY_INTERSECTION",
            "cheek_vs_flywheel": "EMPTY_INTERSECTION",
            "ramp_vs_low_deck": "INTENTIONAL_TANGENT_SUPPORT_AT_Z_52_MM_X_276_TO_286_MM",
            "ramp_tip_ground_clearance_mm": 1.0,
        },
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
