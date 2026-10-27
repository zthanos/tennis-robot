#!/usr/bin/env python3
"""Render the P2-69 Y=0 section from all 30 confirmation trajectories."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p269/matrix/a_75/r_75")
OUT = Path("cad/modular-test-chassis/p269-confirmation-side.png")


def ski() -> tuple[list[float], list[float]]:
    points = [(435, 10), (383, 10)]
    for degrees in range(5, 76, 5):
        theta = math.radians(degrees)
        points.append((383 - 75 * math.sin(theta), 10 + 75 * (1 - math.cos(theta))))
    return [point[0] for point in points], [point[1] for point in points]


fig, ax = plt.subplots(figsize=(12.8, 8.0), constrained_layout=True)
paths = sorted(ROOT.glob("v_*/tm_*/y_*/e_0p3/telemetry.csv"))
for index, path in enumerate(paths):
    rows = list(csv.DictReader(path.open()))
    trace = [(1000*float(row["ball_base_x_m"]),
              1000*float(row["ball_base_z_m"]))
             for row in rows if 2.45 <= float(row["time_s"]) <= 3.20]
    x, z = zip(*trace)
    ax.plot(x, z, color="#0072B2", lw=0.8, alpha=.30,
            label="30 P2-69 PASS trajectories" if index == 0 else None)

nominal = ROOT / "v_6/tm_0p6/y_0/e_0p3/telemetry.csv"
rows = list(csv.DictReader(nominal.open()))
trace = [(1000*float(row["ball_base_x_m"]),1000*float(row["ball_base_z_m"]))
         for row in rows if 2.45 <= float(row["time_s"]) <= 3.20]
x,z = zip(*trace)
ax.plot(x,z,color="#004E7C",lw=2.3,label="nominal 6 m/s, μ=0.6, Y=0")

ax.plot([-85,278],[70,42],color="#4E7D52",lw=3,label="basket floor")
ax.plot([-85,-53],[70,375],color="#4E7D52",lw=3)
sx,sz=ski(); ax.plot(sx,sz,color="#00A0C6",lw=2.4,label="R75 / 75° ski")

ax.plot([318,390],[52,245],color="#1976D2",lw=18,solid_capstyle="butt",
        label="inclined arch X390/Z245")
ax.add_patch(Rectangle((365,230),50,30,color="#1565C0",alpha=.82))
ax.plot([250,360],[363,343],color="#AD1457",lw=9,solid_capstyle="butt",
        label="deflector X250/Z363 → X360/Z343")
ax.add_patch(Rectangle((352,358),76,32,color="#5C6BC0",alpha=.45))
camera=Rectangle((369,291),82,28,color="#8E24AA",alpha=.45)
camera.set_transform(Affine2D().rotate_deg_around(410,305,50)+ax.transData)
ax.add_patch(camera)

# Confirmed closed P2-68 wall. Orange is the overlapping 84 mm physical leaf
# around the nominal clear 80x80 feed aperture.
ax.add_patch(Rectangle((223,46.1),4,188.9,color="#2E7D32",alpha=.92,
                       label="upright wall X225 / top Z235"))
ax.add_patch(Rectangle((222.5,44.1),5,84,color="#F57C00",alpha=.96,
                       label="closed fail-closed door (80×80 clear)"))
ax.annotate("0 wall contacts\n0 door contacts",xy=(225,235),xytext=(105,260),
            color="#2E7D32",fontsize=10,
            arrowprops={"arrowstyle":"->","color":"#2E7D32"})

ax.axhline(380,color="#C62828",ls="--",lw=1,label="pre-deflector ball-top limit Z380")
ax.axhline(390,color="#5C6BC0",ls=":",lw=1)
ax.text(3,394,"LiDAR plane Z390",fontsize=9,color="#3949AB")
ax.set(xlabel="X [mm]  (front →)",ylabel="Z [mm]  (+Z up)",
       title="P2-69 — R75/75 confirmation, 30/30 PASS, e=0.3")
ax.set_xlim(-120,480); ax.set_ylim(0,420); ax.set_aspect("equal",adjustable="box")
ax.grid(True,alpha=.2); ax.legend(loc="upper left",fontsize=8,ncol=2)
fig.savefig(OUT,dpi=170)
print(OUT)
