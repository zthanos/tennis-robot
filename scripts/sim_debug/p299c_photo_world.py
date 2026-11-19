#!/usr/bin/env python3
"""P2-99c: build a Gazebo 'photo' world that renders the robot model from fixed views.

The robot (generated SDF) is placed static at the origin (base_footprint),
fixed camera models save PNGs, and an optional translucent disc marks the
LiDAR scan plane. The disc and cameras exist ONLY in this world file; the
robot model and the court world are untouched.

    python3 scripts/sim_debug/p299c_photo_world.py --robot-sdf R.sdf \
        --out-world W.sdf --img-dir DIR [--court]
    gz sim -s -r --iterations 400 W.sdf
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

# name: (eye xyz, target xyz, hfov rad) in the ground frame of base_footprint.
VIEWS = {
    "iso_front_left": ((1.75, 1.45, 1.05), (0.0, 0.0, 0.22), 0.75),
    "iso_rear_right": ((-1.85, -1.45, 1.05), (-0.05, 0.0, 0.22), 0.75),
    "side_y": ((0.0, 9.0, 0.25), (0.0, 0.0, 0.25), 0.20),
    "top_z": ((0.0, 0.0, 9.0), (0.0, 0.0, 0.0), 0.20),
    "close_tower_c1": ((0.25, 0.75, 0.62), (-0.316, 0.11, 0.37), 0.75),
    "close_front_oak": ((1.45, 0.55, 0.62), (0.45, 0.0, 0.30), 0.75),
}


def look_at_pose(eye, target):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))  # gz camera looks along +X
    return f"{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.6f} {yaw:.6f}"


def camera_model(name, eye, target, hfov, img_dir, w=1400, h=1000):
    return f"""
    <model name="cam_{name}"><static>true</static><pose>{look_at_pose(eye, target)}</pose>
      <link name="l"><sensor name="{name}" type="camera">
        <update_rate>2</update_rate><always_on>true</always_on>
        <camera><horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>60</far></clip>
          <save enabled="true"><path>{img_dir}/{name}</path></save>
        </camera></sensor></link></model>"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-sdf", type=Path, required=True)
    ap.add_argument("--out-world", type=Path, required=True)
    ap.add_argument("--img-dir", type=Path, required=True)
    ap.add_argument("--scan-xyz", type=float, nargs=3, default=(-0.316, 0.110, 0.4198))
    ap.add_argument("--disc-r", type=float, default=0.85)
    ap.add_argument("--court", action="store_true", help="include model://tennis_court and only the court view")
    ap.add_argument("--robot-xy", type=float, nargs=2, default=(0.0, 0.0))
    args = ap.parse_args()

    sdf = args.robot_sdf.read_text()
    model = re.search(r"<model\b.*</model>", sdf, re.S).group(0)
    model = re.sub(r"<plugin\b.*?</plugin>", "", model, flags=re.S)
    model = re.sub(r"<sensor\b.*?</sensor>", "", model, flags=re.S)  # robot sensors not needed
    model = model.replace(">", "><static>true</static>", 1)
    rx, ry = args.robot_xy
    model = model.replace("<static>true</static>", f"<static>true</static><pose>{rx} {ry} 0 0 0 0</pose>", 1)
    sx, sy, sz = args.scan_xyz
    disc = f"""
    <model name="scan_plane_marker"><static>true</static><pose>{rx + sx} {ry + sy} {sz} 0 0 0</pose>
      <link name="l"><visual name="v"><cast_shadows>false</cast_shadows><transparency>0.55</transparency>
        <geometry><cylinder><radius>{args.disc_r}</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.1 0.8 1 0.45</ambient><diffuse>0.1 0.8 1 0.45</diffuse></material>
      </visual></link></model>"""
    views = dict(VIEWS)
    if args.court:
        # Straight-down views (eye xy == target xy): image up = +X, right = -Y.
        views = {"court_top_wide": ((rx + 8.0, ry, 32.0), (rx + 8.0, ry, 0.0), 1.2),
                 "court_top_near": ((rx, ry, 6.0), (rx, ry, 0.0), 0.6)}
    img = args.img_dir.resolve()
    cams = "".join(camera_model(n, e, t, f, img) for n, (e, t, f) in views.items())
    ground = "" if args.court else """
    <model name="ground"><static>true</static><link name="l">
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link></model>"""
    court = "<include><uri>model://tennis_court</uri><pose>0 0 0 0 0 0</pose></include>" if args.court else ""
    world = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="p299c_photo">
  <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.88 0.92 1</background><shadows>false</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.3 -0.9</direction></light>
  <light type="directional" name="fill"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>0.4 0.4 0.4 1</diffuse><direction>0.5 -0.4 -0.6</direction></light>
  {ground}{court}
  {model}
  {disc}
  {cams}
</world></sdf>
"""
    args.out_world.write_text(world)
    for n in views:
        (img / n).mkdir(parents=True, exist_ok=True)
    print(f"world {args.out_world} views {list(views)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
