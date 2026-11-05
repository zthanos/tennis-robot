#!/usr/bin/env python3
"""Conservative, statically admissible bolt/contact load path; NOT FEA/release.

Bolts cannot alone resist this moment: the positive compression-only rail
contact is essential. No friction, equal sharing of close bolts, or balanced
left/right arch loading is credited. Preload is an explicit sensitivity input.
"""
import json
from math import pi

M=8000.0; F=M/132; roll=F*78
pitch_lever=266-230; reverse_pitch_lever=230-196; roll_lever=228-150
Tp=M/pitch_lever; Tp_rev=M/reverse_pitch_lever
Tr=roll/roll_lever; Tr_rev=roll/(220-150)
preload=100.0  # assumed residual force, NOT a tightening torque specification
annulus=lambda od:pi*(od**2-4.3**2)/4
# Add the FULL vertical uplift F to EACH anchor individually as an envelope,
# although the design load is downward; no close-pair load sharing credited.
rail=max(Tp,Tp_rev)+Tr_rev+F+preload; inboard=Tr+F+preload
rows=[
 {"check":"rail top PETG / added OD12 washer", "stress_mpa":rail/annulus(12),"limit_mpa":5},
 {"check":"rail underside PETG / added OD12 washer", "stress_mpa":rail/annulus(12),"limit_mpa":5},
 {"check":"each X240 seat / OD9 washer (no A/B sharing credit)","stress_mpa":inboard/annulus(9),"limit_mpa":5},
 {"check":"rail wall punching screen, t3.2, OD12", "stress_mpa":rail/(pi*12*3.2),"limit_mpa":5},
 # Other anchor preloads close locally under their seats, not at this toe.
 {"check":"each pitch patch (front X262..270 / rear X192..200), Y214..226; full combined envelope", "stress_mpa":rail/(8*12),"limit_mpa":5},
 {"check":"rail roll compression patch X236..244 Y224..232", "stress_mpa":(Tr+preload)/(8*8),"limit_mpa":5},
 {"check":"reverse roll root patch X228..232 Y144..156", "stress_mpa":(Tr_rev+preload)/(4*12),"limit_mpa":5},
 {"check":"foot net root bend, 24-4.6 by20, Kt1.8", "stress_mpa":1.8*roll/((24-4.6)*20**2/6),"limit_mpa":10},
 {"check":"rail pad pitch bend, 28-4.6 by18, Kt1.5", "stress_mpa":1.5*M/((28-4.6)*18**2/6),"limit_mpa":10},
]
for r in rows:r['pass']=r['stress_mpa']<=r['limit_mpa']
print(json.dumps({
 'status':'SCREEN_PASS_NOT_RELEASE' if all(r['pass'] for r in rows) else 'FAIL',
 'one_side_full_pitch_moment_nm':M/1000,'crown_vertical_force_n':F,
 'roll_moment_nm':roll/1000,'pitch_contact_lever_mm':pitch_lever,
 'reverse_pitch_contact_lever_mm':reverse_pitch_lever,
 'reverse_roll_contact_lever_mm':70,
 'roll_contact_lever_mm':roll_lever,'bolt_centres_mm':[[240,150],[240,160],[230,220]],
 'old_close_pair_couple_n':roll/10,'rail_pitch_couple_n':Tp,'reverse_rail_pitch_couple_n':Tp_rev,
 'inboard_roll_couple_n':Tr,'reverse_roll_rail_couple_n':Tr_rev,
 'load_cases_nm':[[p,ro] for p in [-8,8] for ro in [-roll/1000,roll/1000]],
 'preload_assumption_n_per_bolt':preload,
 'worst_rail_bolt_n':rail,'worst_each_inboard_bolt_n':inboard,
 'washer_area_mm2':{'OD9':annulus(9),'OD12':annulus(12)},
 'max_residual_preload_at_5mpa_n':{'rail_washer':5*annulus(12)-max(Tp,Tp_rev)-Tr_rev-F,
 'pitch_contact_patch':5*96-max(Tp,Tp_rev)-Tr_rev-F,'each_inboard':5*annulus(9)-Tr-F},
 'rail_grip_increment_mm':18+1+1,'rows':rows,
 'limitations':['statically admissible load path, not stiffness-resolved bolt reactions',
 'continuous positive sole contact is REQUIRED; bolt-only triangular group is not credited',
 'PETG 10/5 MPa screening assumptions, not material certification or creep allowables',
 '100 N residual preload is assumed: do not apply generic steel-joint M4 torque',
 'no rail wall bending, long-term creep, vibration, fatigue or thread stripping certification',
 'Cut coupon must prove rail/socket seating, washer seats, access and retained thread engagement',
 'not RELEASED: solid foot/knee/clevis and physical coupon required']},indent=2))
raise SystemExit(0 if all(r['pass'] for r in rows) else 1)
