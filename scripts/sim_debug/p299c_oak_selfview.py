#!/usr/bin/env python3
"""P2-99c (b): does any robot body enter the OAK-D field of view?

Builds, per module tilt, a world with ONLY the robot (static, tilt baked into
launcher_tilt_joint), no ground, a flat magenta background and a camera at
camera_link with the sim RGB/depth FOV. Every non-background pixel is robot.
    python3 p299c_oak_selfview.py --robot-sdf-dir D --out-dir O
"""
import argparse, math, re
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-sdf", type=Path, required=True)
    ap.add_argument("--tilt-deg", type=float, required=True)
    ap.add_argument("--out-world", type=Path, required=True)
    ap.add_argument("--img-dir", type=Path, required=True)
    ap.add_argument("--cam", type=float, nargs=4, default=(0.3905, 0.0, 0.300, 0.436332), help="x y z(ground) pitch")
    ap.add_argument("--hfov", type=float, default=1.204)
    a = ap.parse_args()
    model = re.search(r"<model\b.*</model>", a.robot_sdf.read_text(), re.S).group(0)
    model = re.sub(r"<plugin\b.*?</plugin>", "", model, flags=re.S)
    model = re.sub(r"<sensor\b.*?</sensor>", "", model, flags=re.S)
    model = model.replace(">", "><static>true</static>", 1)
    x, y, z, p = a.cam
    img = a.img_dir.resolve(); img.mkdir(parents=True, exist_ok=True)
    a.out_world.write_text(f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="selfview">
  <physics name="p" type="dart"><max_step_size>0.004</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene><ambient>0.7 0.7 0.7 1</ambient><background>1 0 1 1</background><shadows>false</shadows><grid>false</grid></scene>
  <light type="directional" name="sun"><cast_shadows>false</cast_shadows><direction>-0.3 0.2 -0.9</direction><diffuse>0.8 0.8 0.8 1</diffuse></light>
  {model}
  <model name="oak"><static>true</static><pose>{x} {y} {z} 0 {p} 0</pose>
    <link name="l"><sensor name="selfview_tilt{int(a.tilt_deg)}" type="camera"><update_rate>2</update_rate><always_on>true</always_on>
      <camera><horizontal_fov>{a.hfov}</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>50</far></clip><save enabled="true"><path>{img}</path></save></camera></sensor></link></model>
</world></sdf>
""")

if __name__ == "__main__":
    main()
