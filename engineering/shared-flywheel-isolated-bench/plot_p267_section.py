#!/usr/bin/env python3
"""Render the requested P2-67 Y=0 full-v3 side section and nominal traces."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib.transforms import Affine2D

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p267/matrix")
OUT = Path("cad/modular-test-chassis/p267-ballistic-side.png")


def ski(angle: int) -> tuple[list[float], list[float]]:
    points = [(435, 10), (383, 10)]
    for degrees in range(5, angle + 1, 5):
        theta = math.radians(degrees)
        points.append((383-75*math.sin(theta), 10+75*(1-math.cos(theta))))
    return [p[0] for p in points], [p[1] for p in points]


fig, ax = plt.subplots(figsize=(12.8, 8.0), constrained_layout=True)
colours = {75: "#0072B2", 80: "#009E73"}
for angle in (75, 80):
    path = ROOT / f"a_{angle}/r_75/v_6/tm_0p6/y_0/e_0p3/telemetry.csv"
    rows = list(csv.DictReader(path.open()))
    trace = [(1000*float(r["ball_base_x_m"]), 1000*float(r["ball_base_z_m"]))
             for r in rows if 2.45 <= float(r["time_s"]) <= 3.30]
    x, z = zip(*trace)
    ax.plot(x, z, color=colours[angle], lw=2.4, label=f"R75 / {angle}°")
    sx, sz = ski(angle)
    ax.plot(sx, sz, color=colours[angle], lw=1.1, alpha=.55)
    peak = max(range(len(z)), key=z.__getitem__)
    ax.add_patch(Circle((x[peak], z[peak]), 33, fill=False,
                        color=colours[angle], lw=1.2, alpha=.55))

# P2-51 basket retained geometry.
ax.plot([-85, 278], [70, 42], color="#4E7D52", lw=3)
ax.plot([-85, -53], [70, 375], color="#4E7D52", lw=3)

# P2-67 inclined arch: base X318/Z52, crossmember centre X390/Z245.
ax.plot([318, 390], [52, 245], color="#1976D2", lw=18,
        solid_capstyle="butt", label="inclined arch 20.5°")
ax.add_patch(Rectangle((365, 230), 50, 30, color="#1565C0", alpha=.82))
ax.annotate("crossmember rear edge X365", xy=(365,245), xytext=(418,226),
            arrowprops={"arrowstyle":"->", "color":"#1565C0"},
            color="#1565C0", fontsize=9)

# LiDAR / camera translated +60 mm, same Z and orientation.
ax.add_patch(Rectangle((352, 358), 76, 32, color="#5C6BC0", alpha=.55))
ax.text(390, 393, "LiDAR X390 / scan Z390", ha="center", fontsize=9)
camera = Rectangle((410-41,305-14),82,28,color="#8E24AA",alpha=.60)
camera.set_transform(Affine2D().rotate_deg_around(410,305,50)+ax.transData)
ax.add_patch(camera)
ax.text(443, 324, "camera X410/Z305", fontsize=9)

# Independent basket-mounted compliant deflector.
ax.plot([250,360],[363,343],color="#AD1457",lw=9,
        solid_capstyle="butt",label="soft deflector 10 mm")
ax.annotate("basket-mounted\n(no rigid arch path)",xy=(255,362),xytext=(125,382),
            arrowprops={"arrowstyle":"->", "color":"#AD1457"},
            color="#AD1457",fontsize=9)

# Rearward-tilted P2-67 wall: geometrically clear but rejected for capacity.
wall_top_x = 278-(90-42)*math.tan(math.radians(27.5))
ax.plot([278,wall_top_x],[42,90],color="#2E7D32",lw=6,
        label="REJECTED wall 27.5° / top Z90")
ax.annotate("rejected: insufficient pile retention",xy=(wall_top_x,90),xytext=(315,102),
            arrowprops={"arrowstyle":"->"},fontsize=9)

ax.axhline(380,color="#C62828",ls="--",lw=1,label="pre-impact ball-top limit Z380")
ax.set(xlabel="X [mm]  (front →)",ylabel="Z [mm]  (+Z up)",
       title="P2-67 — full-robot-v3 Y=0 section, nominal v=6 m/s / μ=0.6 / e=0.3")
ax.set_xlim(-120,480); ax.set_ylim(0,420); ax.set_aspect("equal",adjustable="box")
ax.grid(True,alpha=.2); ax.legend(loc="upper left",fontsize=8,ncol=2)
fig.savefig(OUT,dpi=170)
print(OUT)
