#!/usr/bin/env python3
"""Static preflight for the frozen P2-45 3/5/8 mm campaign."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
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
    args = ap.parse_args()
    errors = []
    criteria = json.loads(args.criteria.read_text())
    contract = json.loads(args.contract.read_text())
    if criteria["status"] != "FROZEN_BEFORE_FIRST_P2_45_GAZEBO_RUN": errors.append("criteria are not frozen")
    if criteria["matrix"]["run_count"] != 135: errors.append("matrix is not 135 runs")
    if sha(args.ball) != contract["ball_model"]["source_sha256"]: errors.append("calibrated ball model hash changed")
    variants = {}
    required_ramp = {f"intake_fixed_ramp_floor_{i}_col" for i in range(7)} | {"intake_fixed_ramp_flap_col"}
    for path in args.urdf:
        match = re.search(r"tip(3|5|8)_", path.stem)
        if not match:
            errors.append(f"{path}: filename does not encode tip height"); continue
        nominal_mm = float(match.group(1))
        root = ET.parse(path).getroot()
        links = {link.attrib["name"]: link for link in root.findall("link")}
        base, frame, basket = (links.get(name) for name in ("base_link", "flywheel_launcher_frame_link", "basket_support_link"))
        if base is None or frame is None or basket is None:
            errors.append(f"{path}: base/module/basket missing"); continue
        base_names = {item.attrib.get("name") for item in base.findall("collision")}
        if not required_ramp <= base_names: errors.append(f"{path}: ramp fixtures missing")
        frame_names = {item.attrib.get("name") for item in frame.findall("collision")}
        if "intake_moving_guide_floor_col" in frame_names: errors.append(f"{path}: legacy moving guide present")
        first = base.find("collision[@name='intake_fixed_ramp_floor_0_col']")
        origin = [float(x) for x in first.find("origin").attrib["xyz"].split()]
        pitch = float(first.find("origin").attrib["rpy"].split()[1])
        size = [float(x) for x in first.find("geometry/box").attrib["size"].split()]
        collision_aabb_min_mm = 1000*(origin[2] - abs(math.sin(pitch))*size[0]/2 - abs(math.cos(pitch))*size[2]/2)
        # Segment boxes overlap by 2 mm at each end, so their conservative
        # collision AABB sits slightly below the exact CAD hull underside.
        if collision_aabb_min_mm < nominal_mm - 0.6:
            errors.append(f"{path}: ramp collision AABB {collision_aabb_min_mm:.3f} mm too low")
        variants[path.stem] = {"nominal_cad_tip_underside_mm": nominal_mm,
                               "conservative_collision_aabb_min_mm": collision_aabb_min_mm,
                               "urdf_sha256": sha(path)}
    result = {"status": "PASS" if not errors else "FAIL", "errors": errors,
              "criteria_sha256": sha(args.criteria), "ball_model_sha256": sha(args.ball),
              "matrix_runs": criteria["matrix"]["run_count"], "urdf_variants": variants,
              "collision_note": "URDF overlapping segment box AABB is conservative; exact CAD hull underside is the named 3/5/8 mm variable."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if errors: raise SystemExit(1)


if __name__ == "__main__": main()
