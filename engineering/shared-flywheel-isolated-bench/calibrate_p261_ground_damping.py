#!/usr/bin/env python3
"""Fit only damping after mapping one-sided rigid penetration p to ITF delta=2p."""
from dataclasses import asdict
import json, math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from tennis_ball_contact_model import CompliantNormalModel, NormalParameters

def rebound(dt, damping):
    prm=NormalParameters(dynamic_damping_n_s_m_pow=damping)
    law=CompliantNormalModel(prm); p=0.; v=math.sqrt(2*prm.gravity_m_s2*2.54); maximum=0.; peak=0.; t=0.
    def deriv(x,u,mx):
        s=law.evaluate(2*x,2*u,2*mx)
        return u,prm.gravity_m_s2-s.force_n/prm.mass_kg,s.force_n
    for _ in range(200000):
        maximum=max(maximum,p)
        k1=deriv(p,v,maximum); k2=deriv(p+.5*dt*k1[0],v+.5*dt*k1[1],max(maximum,p+.5*dt*k1[0])); k3=deriv(p+.5*dt*k2[0],v+.5*dt*k2[1],max(maximum,p+.5*dt*k2[0])); k4=deriv(p+dt*k3[0],v+dt*k3[1],max(maximum,p+dt*k3[0]))
        p += dt*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6
        v += dt*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6
        peak=max(peak,k1[2],k2[2],k3[2],k4[2]); t+=dt; maximum=max(maximum,p)
        s=law.evaluate(2*p,2*v,2*maximum)
        if v<0 and s.force_n<=1e-9 and t>dt: break
    return {'dt_s':dt,'rebound_height_m':(-v)**2/(2*prm.gravity_m_s2)-max(p,0),'rebound_velocity_m_s':-v,'max_one_sided_penetration_m':maximum,'max_diametral_law_compression_m':2*maximum,'peak_force_n':peak,'contact_duration_s':t}

lo,hi=0.,20000.
for _ in range(64):
    mid=(lo+hi)/2
    if rebound(.000025,mid)['rebound_height_m']>1.41: lo=mid
    else: hi=mid
damping=(lo+hi)/2
out={'phase':'P2-61','diametral_compression_scale':2.0,'dynamic_damping':damping,'fit':rebound(.000025,damping),'validation':[rebound(x,damping) for x in (.001,.0005,.00025)]}
path=Path('engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p261_contact_model/damping_fit.json'); path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(out,indent=2)+'\n'); print(json.dumps(out,indent=2))
