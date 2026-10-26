#!/usr/bin/env python3
"""P2-93 conservative hand screens for 8 N.m terminal design load.

Not nonlinear FEA. The checks deliberately put the complete P2-54 terminal
moment into one arch leg/stop and use the repository PETG fatigue limits:
10 MPa in-layer, 5 MPa across layers.
"""
from __future__ import annotations
import json
from math import pi
from pathlib import Path

M = 8_000.0                    # N mm, P2-54 1.5x terminal design moment
F_ACT = 103.6                  # N, worst P2-53 equivalent force for 8 N.m
LIMIT_IN = 10.0
LIMIT_X = 5.0

def rect_ellipse_section(b, h, bi, hi):
    iy = b*h**3/12 - pi*bi*hi**3/64
    z = iy/(h/2)
    return iy, z

Iy_leg, Z_leg = rect_ellipse_section(20,26,13.6,19.6)
Iy_brace, Z_brace = rect_ellipse_section(26,22,19.6,15.6)

rows=[]
def add(name, stress, limit, model):
    rows.append({"check":name,"stress_mpa":round(stress,3),"limit_mpa":limit,
                 "utilization":round(stress/limit,3),"pass":stress<=limit,"model":model})

add("arch leg, full 8 N.m in one member", M/Z_leg, LIMIT_IN,
    "20x26 outer minus 13.6x19.6 elliptical void; open-end print, bending in layer")
add("X240/X300 brace, full 8 N.m sensitivity", M/Z_brace, LIMIT_IN,
    "26x22 outer minus 19.6x15.6 elliptical void; conservative single brace")

# X240--X300 feet form a 60 mm X couple. Put the full couple force into one
# 10 mm PETG foot around one M4 bore; this is deliberately worse than four
# bolts and the triangulated load sharing.
foot_force = M/60
add("foot M4 printed bearing", foot_force/(10*4.6), LIMIT_X,
    "8 N.m / 60 mm X240-X300 couple; one 10 mm foot and one Ø4.6 bore")

# Stop shoulder: 12 mm deep x 38 mm wide root, printed flat so bending is in-layer.
Z_stop = 38*12**2/6
add("+20 stop shoulder root", M/Z_stop, LIMIT_IN,
    "one stop takes complete 8 N.m; 38x12 mm rectangular root")
stop_force = M/40
add("+20 stop M6 bearing", stop_force/(6.6*16), LIMIT_X,
    "200 N at 40 mm effective shoulder; Ø6.6 through bore in 16 mm depth")

# Carrier screen: actuator force at a 42 mm local eccentricity, 10x32 section.
carrier_m = F_ACT*42
Z_carrier = 10*32**2/6
add("FIT/KFL carrier local strip", carrier_m/Z_carrier, LIMIT_IN,
    "103.6 N actuator force, 42 mm eccentricity, conservative 10x32 strip")

result={
    "status":"PASS" if all(r["pass"] for r in rows) else "FAIL",
    "design_load_nm":8.0,
    "source":"P2-11/P2-54 terminal design load; analytical section screens, not FEA",
    "limits_mpa":{"in_layer":LIMIT_IN,"across_layers":LIMIT_X},
    "rows":rows,
    "max_utilization":max(r["utilization"] for r in rows),
    "limitations":[
        "physical proof-load and hardware fit remain mandatory before use",
        "joint slip, fastener preload, creep and impact are not nonlinear-modeled",
        "fit-check parts must be re-exported if user measurements exceed the slots/shims",
    ],
}
out=Path(__file__).with_name("p293-strength-results.json")
out.write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
raise SystemExit(0 if result["status"]=="PASS" else 1)
