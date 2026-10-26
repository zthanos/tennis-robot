#!/usr/bin/env python3
"""ASCII-STL volume, mass and P2S planning estimate for the P2-93 release."""
from __future__ import annotations
import json
import math
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])
dest = pathlib.Path(sys.argv[2])
rho_g_mm3 = 1.27e-3

qty = {
    "arch_foot_x240": 2, "arch_foot_x300": 2, "arch_leg_lower": 2,
    "arch_leg_upper": 2, "arch_brace_x240": 2, "arch_brace_x300": 2,
    "arch_joint_sleeve": 4, "sensor_joint_sleeve": 1,
    "arch_crossbar_center": 1, "arch_crossbar_end": 2, "lidar_mast": 2,
    "lidar_crossbar_half": 2, "oak_universal_plate": 1,
    "tilt_fit0186_carrier": 1, "tilt_kfl08_base": 1,
    "tilt_nut_carriage": 1, "tilt_stop": 2, "tilt_as5600_pod": 1,
    "tilt_as5600_cover": 1, "tilt_magnet_cap": 1,
}

# Load-path parts retain approximately 2 mm shell thickness. Hours are P2S
# planning estimates from conservative deposited PETG plus per-part heat/travel
# overhead; final Bambu Studio slicing remains the print gate.
heavy = {"arch_foot_x240", "arch_foot_x300", "arch_leg_lower", "arch_leg_upper",
         "arch_brace_x240", "arch_brace_x300", "arch_crossbar_center",
         "arch_crossbar_end", "arch_joint_sleeve", "tilt_fit0186_carrier", "tilt_kfl08_base",
         "tilt_nut_carriage", "tilt_stop"}

profiles = {
    "0.4/0.28": {"nozzle_mm": 0.4, "layer_mm": 0.28,
                  "heavy_walls": 5, "light_walls": 4,
                  "heavy_infill": 35, "light_infill": 25,
                  "rate_g_h": 22.0, "overhead_h": 0.65, "height_factor_h": 0.45},
    "0.6/0.30": {"nozzle_mm": 0.6, "layer_mm": 0.30,
                  "heavy_walls": 4, "light_walls": 3,
                  "heavy_infill": 35, "light_infill": 25,
                  "rate_g_h": 31.0, "overhead_h": 0.55, "height_factor_h": 0.38},
}

vertex = re.compile(r"^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def volume_and_bbox(path: pathlib.Path):
    tri=[]; vol6=0.0; mins=[math.inf]*3; maxs=[-math.inf]*3
    for line in path.read_text(errors="ignore").splitlines():
        m=vertex.match(line)
        if not m: continue
        p=tuple(float(m.group(i)) for i in range(1,4)); tri.append(p)
        for i in range(3): mins[i]=min(mins[i],p[i]); maxs[i]=max(maxs[i],p[i])
        if len(tri)==3:
            vol6 += sum(tri[0][i]*cross(tri[1],tri[2])[i] for i in range(3)); tri=[]
    return abs(vol6)/6.0, [round(maxs[i]-mins[i],2) for i in range(3)]

rows=[]
for path in sorted(root.glob("*.stl")):
    name=path.stem; vol,bbox=volume_and_bbox(path); mass=vol*rho_g_mm3
    estimates = {}
    for label, p in profiles.items():
        is_heavy = name in heavy
        hours = p["overhead_h"] + mass/p["rate_g_h"] + max(bbox)/250.0*p["height_factor_h"]
        estimates[label] = {
            "p2s_hours_each": round(hours,1),
            "perimeters": p["heavy_walls"] if is_heavy else p["light_walls"],
            "infill_percent": p["heavy_infill"] if is_heavy else p["light_infill"],
        }
    rows.append({"part":name,"qty":qty.get(name,1),"bbox_mm":bbox,
                 "volume_cm3_each":round(vol/1000,2),"mass_g_each":round(mass,1),
                 "profiles":estimates})

summary={"material":"PETG","density_g_cm3":1.27,
         "mass_method":"conservative full-STL-volume envelope; slicer mass should be lower on infilled solids",
         "profile_assumptions":profiles,"rows":rows,
         "first_wave_distinct_stls":len(rows),
         "first_wave_prints":sum(r["qty"] for r in rows),
         "first_wave_mass_g":round(sum(r["mass_g_each"]*r["qty"] for r in rows),1),
         "first_wave_hours_by_profile":{
             label: round(sum(r["profiles"][label]["p2s_hours_each"]*r["qty"] for r in rows),1)
             for label in profiles
         }}
dest.write_text(json.dumps(summary,indent=2)+"\n")
print(f"P2-93 STATS: {summary['first_wave_mass_g']} g, " +
      ", ".join(f"{k}={v} h" for k,v in summary["first_wave_hours_by_profile"].items()))
