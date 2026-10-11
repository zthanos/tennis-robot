#!/usr/bin/env python3
"""CAD solid-volume comparison; preserves historical middle-T measurements."""
import argparse
import json
from pathlib import Path
from measure_middle_t_design import mesh_stats, ROOT, BENCH

PARTS = {
    "motor_left":1, "motor_right":1,
    "front_motor_gamma_left":1, "front_motor_gamma_right":1,
    "rail_extension_left":1, "rail_extension_right":1,
    "corner_left":1, "corner_right":1, "crossbar_half":2,
    "straight_sleeve":5, "ramp_back_half":2, "ramp_back_sleeve":3,
    "electronics_tray_left":1, "electronics_tray_right":1,
}


def measure():
    rows = []
    for name, qty in PARTS.items():
        stats = mesh_stats((ROOT / f"cad/modular-test-chassis/stl-v2/{name}.stl").read_bytes())
        if stats["nonmanifold_edge_count"]:
            raise ValueError(f"Nonmanifold STL: {name}")
        rows.append({"part":name,"quantity":qty,**stats})
    volume = sum(r["quantity"]*r["solid_volume_cm3"] for r in rows)
    old = json.loads((BENCH / "results/middle_t_integration/cad_measurements.json").read_text())
    return {"status":"CAD_MEASURED_ELASTIC_FRAME_VALIDATION_PENDING",
        "parts":rows, "total_solid_volume_cm3":volume,
        "pla_solid_mass_g":volume*1.24,
        "previous_middle_t_pla_solid_mass_g":old["solid_mass_estimates_g"]["PLA"]["complete_design"],
        "geometry":{"brace_width_mm":50,"brace_height_mm":20,"brace_x_mm":294.4,
            "ramp_rear_face_x_mm":319.65,"glue_gap_mm":0.25,
            "brace_top_z_mm":42,"motor_nut_channel_bottom_z_mm":44.3,
            "brace_half_length_mm":160,"sleeve_length_mm":60},
        "limitations":["CAD density estimate, not slicer mass or a measured print",
            "New 43x13 mm sleeve and M3 nut pockets need a physical coupon",
            "Ramp glue pads contact rear sidewalls; adhesive strength is unverified",
            "Relocation does not remove the closed physics loop or validate elastic stiffness"]}


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    result=measure()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ("parts","limitations")},indent=2))
