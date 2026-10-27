#!/usr/bin/env python3
import argparse,csv,json
from pathlib import Path

def interp(rows, target):
    for a,b in zip(rows,rows[1:]):
        if a[0] <= target <= b[0] and b[0] != a[0]:
            q=(target-a[0])/(b[0]-a[0]); return a[1]+q*(b[1]-a[1])
    raise RuntimeError(f'load {target} N not crossed')
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--contacts',type=Path,required=True); ap.add_argument('--mode',required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args()
    by={2:{},3:{}}
    with a.contacts.open() as f:
        for r in csv.DictReader(f):
            cid=int(float(r['contact_id'])); t=round(float(r['time_s']),7)
            if cid in by: by[cid][t]=(float(r['normal_force_n']),float(r['compression_m']))
    rows=[]
    for t in sorted(set(by[2])&set(by[3])):
        f0,p0=by[2][t]; f1,p1=by[3][t]; rows.append(((f0+f1)/2,p0+p1,t))
    loading=[]
    maxf=-1
    for force,deform,t in rows:
        if force >= maxf: loading.append((force,deform,t)); maxf=force
    d0=interp([(f,d) for f,d,_ in loading],15.57); d1=interp([(f,d) for f,d,_ in loading],95.64)
    result={'phase':'P2-61','mode':a.mode,'preload_total_deformation_mm':1000*d0,'test_total_deformation_mm':1000*d1,'forward_deformation_mm':1000*(d1-d0),'peak_mean_load_n':max(f for f,_,_ in rows)}
    a.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
