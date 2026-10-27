#!/usr/bin/env python3
"""P2-68 Y=0 side section from the existing 30 P2-67 R75/75 traces."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.transforms import Affine2D


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p267/matrix/a_75/r_75")
OUT = Path("cad/modular-test-chassis/p268-upright-wall-side.png")


def ski() -> tuple[list[float], list[float]]:
    points = [(435, 10), (383, 10)]
    for degrees in range(5, 76, 5):
        theta = math.radians(degrees)
        points.append((383 - 75 * math.sin(theta), 10 + 75 * (1 - math.cos(theta))))
    return [p[0] for p in points], [p[1] for p in points]


fig, ax = plt.subplots(figsize=(12.8, 8.0), constrained_layout=True)
paths = sorted(ROOT.glob("v_*/tm_*/y_*/e_0p3/telemetry.csv"))
for index, path in enumerate(paths):
    rows = list(csv.DictReader(path.open()))
    trace = [
        (1000 * float(r["ball_base_x_m"]), 1000 * float(r["ball_base_z_m"]))
        for r in rows
        if 2.45 <= float(r["time_s"]) <= 3.20
    ]
    x, z = zip(*trace)
    ax.plot(x, z, color="#0072B2", lw=0.7, alpha=0.26,
            label="30 existing R75/75 traces" if index == 0 else None)

# Nominal trace is emphasized without implying a new run.
nominal = ROOT / "v_6/tm_0p6/y_0/e_0p3/telemetry.csv"
rows = list(csv.DictReader(nominal.open()))
trace = [(1000*float(r["ball_base_x_m"]),1000*float(r["ball_base_z_m"]))
         for r in rows if 2.45 <= float(r["time_s"]) <= 3.20]
x, z = zip(*trace)
ax.plot(x, z, color="#005B96", lw=2.2, label="nominal existing trace")

# Retained basket floor/back and R75/75 ski.
ax.plot([-85,278],[70,42],color="#4E7D52",lw=3,label="basket floor")
ax.plot([-85,-53],[70,375],color="#4E7D52",lw=3)
sx, sz = ski()
ax.plot(sx, sz, color="#00A0C6", lw=2.4, label="R75 / 75° ski")

# P2-67 arch and independent deflector.
ax.plot([318,390],[52,245],color="#1976D2",lw=18,solid_capstyle="butt",
        label="P2-67 inclined arch")
ax.add_patch(Rectangle((365,230),50,30,color="#1565C0",alpha=.82))
ax.plot([250,360],[363,343],color="#AD1457",lw=9,solid_capstyle="butt",
        label="deflector X250/Z363 → X360/Z343")
ax.add_patch(Rectangle((352,358),76,32,color="#5C6BC0",alpha=.45))
camera=Rectangle((369,291),82,28,color="#8E24AA",alpha=.45)
camera.set_transform(Affine2D().rotate_deg_around(410,305,50)+ax.transData)
ax.add_patch(camera)

# P2-68 selected wall: actual 4 mm slab and R2 top edge.
ax.add_patch(Rectangle((223,46.1),4,186.9,color="#2E7D32",alpha=.90,
                       label="P2-68 upright wall X225, top Z235"))
ax.add_patch(Circle((225,233),2,color="#2E7D32"))
ax.add_patch(Rectangle((222.5,46.1),5,80,color="#F57C00",alpha=.95,
                       label="fail-closed 80×80 door"))
ax.plot([227,278],[46.1,42],color="#2455A4",lw=5,label="fixed feed handoff")
ax.plot([278,329.2],[42,26.1],color="#D19024",lw=5,
        label="+15° selector nose → nip rear mouth")

# Show the R33+10 conservative top-edge box at its limiting trace location.
ax.add_patch(Rectangle((180,235),90,86,fill=False,ls="--",lw=1.2,
                       edgecolor="#C62828",label="R33+10 conservative envelope"))
ax.annotate("selected top: Z235\nbox-envelope clearance 11.45 mm",
            xy=(225,235),xytext=(92,255),fontsize=9,color="#2E7D32",
            arrowprops={"arrowstyle":"->","color":"#2E7D32"})
ax.annotate("door closed at −10…+14.5°\nopens only at +15…+20° / stationary",
            xy=(225,90),xytext=(65,115),fontsize=9,
            arrowprops={"arrowstyle":"->"})

ax.axhline(390,color="#5C6BC0",ls=":",lw=1)
ax.text(10,394,"LiDAR plane Z390",fontsize=9,color="#3949AB")
ax.set(xlabel="X [mm]  (front →)",ylabel="Z [mm]  (+Z up)",
       title="P2-68 — geometry only from 30 existing P2-67 R75/75 trajectories")
ax.set_xlim(-120,480); ax.set_ylim(0,420); ax.set_aspect("equal",adjustable="box")
ax.grid(True,alpha=.2); ax.legend(loc="upper left",fontsize=7.7,ncol=2)
fig.savefig(OUT,dpi=170)
print(OUT)
