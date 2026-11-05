#!/usr/bin/env python3
"""P2-93d analytical knee screen, not FEA or a print release."""
import json
from math import hypot, sqrt, sin, cos, atan2, degrees

M=8000.0                       # N mm; full design moment on ONE side
arm=132.0
F=M/arm
b,h,d=20.0,26.0,4.6
area=b*(h-d)                   # conservative section through a pin bore
Iy=b*(h**3-d**3)/12
Z=Iy/(h/2)
Kt=1.8                         # engineering assumption, R3 root; coupon/FEA pending
stress=Kt*(M/Z+F/area)
E=1800.0                      # repository short-term printed PETG screen
inclined_len=hypot(132,125)
theta=atan2(125,132)
delta_arm=F*inclined_len**3*cos(theta)**2/(3*E*Iy)
delta_post_rotation=arm*M*48/(E*Iy)
delta_post_axial=F*48/(E*area)
pin_pitch=34.0                 # Y-axis M4 at global Z80 / Z114
pin_force=hypot(M/pin_pitch,F/2)
cheek_bearing=pin_force/(2*6*d)
knee_bearing=pin_force/(20*d)
edge_ligament=6-d/2             # nearest foot cheek edge: Z120 - Z114
edge_shear=pin_force/(4*6*edge_ligament)
foot_torsion=2*M/(0.20*36*24**2) # K=2, raised 24x36 root; no load-sharing credit
foot_bending=2*(F*78)/(24*36**2/6)
rows=[
 {"check":"knee combined bend + axial, Kt=1.8","stress_mpa":stress,"limit_mpa":10},
 {"check":"foot clevis M4 bearing, both 6 mm cheeks","stress_mpa":cheek_bearing,"limit_mpa":5},
 {"check":"knee M4 bearing","stress_mpa":knee_bearing,"limit_mpa":5},
 {"check":"foot cheek pin edge shear","stress_mpa":edge_shear,"limit_mpa":5},
 {"check":"raised foot root torsion, K=2","stress_mpa":foot_torsion,"limit_mpa":5},
 {"check":"raised foot root lateral cantilever bending, K=2","stress_mpa":foot_bending,"limit_mpa":10},
]
for r in rows:
 r['utilization']=r['stress_mpa']/r['limit_mpa'];r['pass']=r['utilization']<=1
result={
 'status':'ANALYTICAL_SCREEN_PASS' if all(r['pass'] for r in rows) else 'FAIL',
 'design_moment_nm':8,'one_side_takes_full_load':True,'lever_mm':arm,
 'equivalent_crown_force_n':F,'angle_deg':degrees(theta),'net_Iy_mm4':Iy,'net_Z_mm3':Z,
 'root_radius_mm':3,'knee_section_mm':[26,20], 'knee_print_plane':'XZ load plane on bed; layer normal Y',
 'pin_pitch_mm':pin_pitch,'max_pin_force_n':pin_force,'rows':rows,
 'elastic_knee_deflection_mm':delta_arm+delta_post_rotation+delta_post_axial,
 'deflection_excludes':['foot bridge torsion','joint slip','bolt compliance','creep'],
 'base_fastener_warning':{
  'overturning_x_moment_nmm':F*78,
  'existing_Y_hole_pitch_mm':10,
  'hole_pair_couple_force_n':F*78/10,
  'note':'X240-only base loses the old 60 mm X240-X300 couple. Verify clamp preload, washer/spreader bearing and existing rail; not released by this knee calculation.'},
 'limitations':['not FEA; Kt is assumed, not measured',
  '10/5 MPa are existing repository screening limits, not material certification',
  '100% solid knee or equivalent verified perimeter core; not draft 2 walls/10% infill',
  'foot print orientation, supports and hardware load spreading need physical proof',
  'full-scene F concept and unrelated/tilt collisions remain blocking release']}
print(json.dumps(result,indent=2))
