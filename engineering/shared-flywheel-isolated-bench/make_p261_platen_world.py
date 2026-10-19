#!/usr/bin/env python3
import argparse, math
from pathlib import Path

K = 107309.29404174259
R = 0.033

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True); ap.add_argument('--contacts',required=True)
    ap.add_argument('--platen',required=True); ap.add_argument('--scale',type=float,choices=(1,2),required=True)
    ap.add_argument('--damping',type=float,required=True); ap.add_argument('--dt',type=float,default=.00025)
    a=ap.parse_args()
    delta=(15.57/K)**(1/1.5); p=delta/a.scale
    ball_z=R-p; upper_z=2*R-2*p
    xml=f'''<?xml version="1.0"?>
<sdf version="1.10"><world name="p261_platen">
  <gravity>0 0 0</gravity><physics name="p261" type="ignored"><max_step_size>{a.dt}</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <model name="upper_platen"><static>true</static><pose>0 0 {upper_z} 0 0 0</pose><link name="upper_platen_link"/>
    <plugin filename="v3_platen_p261_system" name="phase2_v3::PlatenP261System"><initial_z>{upper_z}</initial_z><speed_m_s>0.003333</speed_m_s><hold_s>1</hold_s><duration_s>9</duration_s><output_csv>{a.platen}</output_csv></plugin>
  </model>
  <model name="platen_ball"><pose>0 0 {ball_z} 0 0 0</pose><link name="ball_link"><inertial><mass>0.058</mass><inertia><ixx>4.2108e-5</ixx><iyy>4.2108e-5</iyy><izz>4.2108e-5</izz></inertia></inertial></link>
    <plugin filename="tennis_ball_compliant_contact_system" name="tennis_ball_contact_system::TennisBallContactSystem">
      <ball_link>ball_link</ball_link><ball_radius>0.033</ball_radius><enable_ground>true</enable_ground><ground_height>0</ground_height>
      <upper_platen_link>upper_platen_link</upper_platen_link><diametral_compression_scale>{a.scale}</diametral_compression_scale>
      <loading_stiffness>{K}</loading_stiffness><loading_exponent>1.5</loading_exponent><unloading_exponent>1.9866353710127362</unloading_exponent>
      <dynamic_damping>{a.damping}</dynamic_damping><telemetry_csv>{a.contacts}</telemetry_csv>
    </plugin>
  </model>
</world></sdf>'''
    a.output.write_text(xml)
if __name__=='__main__': main()
