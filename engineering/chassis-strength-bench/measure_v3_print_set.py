#!/usr/bin/env python3
"""Measure the Phase-2 v3 STL set; does not replace slicer estimates."""
from __future__ import annotations

import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE))
from measure_middle_t_design import mesh_stats

DENSITY_G_CM3=0.72
STL=ROOT/'cad/modular-test-chassis/stl-v3'
OUTPUT=HERE/'results/shared_flywheel_phase2/v3_print_set.json'

MANIFEST={
    'sleeve_fit_coupon':(1,'rail bottom on bed','LOCKED','print first with one clamp sleeve; physical fit gate'),
    'clamp_sleeve':(5,'43 mm face on bed','LOCKED','four rail joints plus rear-centre splice; print one with coupon first'),
    'corner_clamp_left':(1,'23 mm broad face on bed','LOCKED','rear-left corner; coupon M4 nut fit applies'),
    'corner_clamp_right':(1,'23 mm broad face on bed','LOCKED','rear-right corner; coupon M4 nut fit applies'),
    'rear_motor_left':(1,'rail bottom on bed','LOCKED','validated FIT0403 land; v3 adds M4 clamp holes'),
    'rear_motor_right':(1,'rail bottom on bed','LOCKED','validated FIT0403 land; v3 adds M4 clamp holes'),
    'rail_extension_left':(1,'rail bottom on bed','LOCKED','M4 grid only outside both 60 mm insertion zones'),
    'rail_extension_right':(1,'rail bottom on bed','LOCKED','M4 grid only outside both 60 mm insertion zones'),
    'front_terminal_left':(1,'rail bottom on bed','LOCKED','P2-78 integrated X240/X300 hollow sockets'),
    'front_terminal_right':(1,'rail bottom on bed','LOCKED','P2-78 integrated X240/X300 hollow sockets'),
    'rear_crossbar_half':(2,'long flat bottom on bed','LOCKED','two halves; M4 accessory grid y75..150'),
    'x240_crossbar_half':(2,'long flat bottom on bed','LOCKED','P2-78: two 130 mm 50x20 halves at X240'),
    'x240_clamp_sleeve':(1,'43 mm face on bed','LOCKED','centre splice for X240'),
    'x240_terminal_sleeve':(2,'43 mm face on bed','LOCKED','P2-78 socket/crossbar splice; 60 mm per side'),
    'x300_crossbar_half':(2,'long flat bottom on bed','LOCKED','P2-78: two 130 mm 20x20 halves; grid retained'),
    'x300_clamp_sleeve':(1,'13 mm face on bed','LOCKED','centre splice for X300'),
    'x300_terminal_sleeve':(2,'13 mm face on bed','LOCKED','P2-78 socket/crossbar splice; 60 mm per side'),
    'torsion_side_brace':(2,'broad face on bed','LOCKED','P2-78 paired short X240/Y150 to X300/Y125 braces'),
    'ballast_tab':(4,'70x30 face on bed','TEMPORARY','bolt plywood/metal ballast decks to M4 grid'),
}

def main():
    rows=[]
    for name,(qty,orientation,state,dependency) in MANIFEST.items():
        path=STL/f'{name}.stl'
        stat=mesh_stats(path.read_bytes())
        if stat['nonmanifold_edge_count']:
            raise RuntimeError(f'non-manifold mesh: {path}')
        mass=stat['solid_volume_cm3']*DENSITY_G_CM3
        rows.append({'part':name,'quantity':qty,'mass_each_g':mass,
            'mass_total_g':mass*qty,'orientation':orientation,'state':state,
            'dependency':dependency,**stat})
    report={'status':'V3_CAD_LOCKED_PHYSICAL_COUPON_FIRST',
        'density_g_cm3':DENSITY_G_CM3,'parts':rows,
        'total_estimated_mass_g':sum(r['mass_total_g'] for r in rows),
        'locked_mass_excluding_coupon_and_temporary_g':sum(
            r['mass_total_g'] for r in rows
            if r['state']=='LOCKED' and r['part']!='sleeve_fit_coupon'),
        'limits':['0.72 g/cm3 is the user slicer-derived effective value; re-slice with the final profile',
          'OpenSCAD/manifold checks do not prove M4 nut fit, clamp preload, layer adhesion or fatigue',
          'Print sleeve_fit_coupon plus one clamp_sleeve before long parts']}
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    OUTPUT.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'parts':len(rows),'total_estimated_mass_g':report['total_estimated_mass_g'],
        'locked_mass_excluding_coupon_and_temporary_g':report['locked_mass_excluding_coupon_and_temporary_g']},indent=2))

if __name__=='__main__': main()
