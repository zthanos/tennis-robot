#!/usr/bin/env python3
"""Static collision lower-bound audit at the -10 degree launcher limit."""
from __future__ import annotations
import argparse, json, math, struct, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def rpy(text: str | None) -> np.ndarray:
    r,p,y=map(float,(text or "0 0 0").split()); cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def axis_angle(axis, angle):
    a=np.array(axis,dtype=float); a/=np.linalg.norm(a); x,y,z=a; c,s=math.cos(angle),math.sin(angle); C=1-c
    return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]])
def origin(e):
    if e is None:return np.zeros(3),np.eye(3)
    return np.array(list(map(float,e.get("xyz","0 0 0").split()))),rpy(e.get("rpy"))
def stl_vertices(path: Path) -> np.ndarray:
    b=path.read_bytes()
    if len(b)>=84:
        n=struct.unpack_from("<I",b,80)[0]
        if 84+50*n==len(b):
            out=[]
            for i in range(n): out.extend(struct.unpack_from("<9f",b,84+50*i+12))
            return np.array(out).reshape(-1,3)
    vals=[]
    for line in b.decode(errors="ignore").splitlines():
        q=line.strip().split()
        if len(q)==4 and q[0]=="vertex": vals.append(list(map(float,q[1:])))
    return np.array(vals)
def main():
    p=argparse.ArgumentParser(); p.add_argument("urdf",type=Path); p.add_argument("--repo",type=Path,default=Path(".")); p.add_argument("--launcher-angle-deg",type=float,default=-10.0); p.add_argument("--output",type=Path,required=True); a=p.parse_args()
    root=ET.parse(a.urdf).getroot(); links={x.get("name"):x for x in root.findall("link")}; children={}
    for j in root.findall("joint"):
        parent=j.find("parent").get("link"); child=j.find("child").get("link"); t,R=origin(j.find("origin")); angle=math.radians(a.launcher_angle_deg) if j.get("name")=="launcher_tilt_joint" else 0.; axis=list(map(float,j.find("axis").get("xyz").split())) if j.find("axis") is not None else [1,0,0]
        children.setdefault(parent,[]).append((child,t,R@axis_angle(axis,angle)))
    tf={"base_link":(np.zeros(3),np.eye(3))}; stack=["base_link"]
    while stack:
        parent=stack.pop(); pt,pR=tf[parent]
        for child,t,R in children.get(parent,[]): tf[child]=(pt+pR@t,pR@R); stack.append(child)
    records=[]
    for lname,link in links.items():
        if lname not in tf: continue
        lt,lR=tf[lname]
        for i,col in enumerate(link.findall("collision")):
            ct,cR=origin(col.find("origin")); center=lt+lR@ct; R=lR@cR; geom=col.find("geometry"); low=None; kind=""
            if geom.find("box") is not None:
                h=np.array(list(map(float,geom.find("box").get("size").split())))/2; low=center[2]-np.abs(R[2])@h; kind="box"
            elif geom.find("cylinder") is not None:
                q=geom.find("cylinder"); rad=float(q.get("radius")); half=float(q.get("length"))/2; low=center[2]-rad*math.hypot(R[2,0],R[2,1])-half*abs(R[2,2]); kind="cylinder"
            elif geom.find("mesh") is not None:
                q=geom.find("mesh"); name=q.get("filename"); rel=name.replace("package://tennis_robot/",""); path=a.repo/"ros2_ws/src/tennis_robot"/rel; scale=np.array(list(map(float,q.get("scale","1 1 1").split()))); v=stl_vertices(path)*scale; low=float(np.min((v@R.T+center)[:,2])); kind=f"mesh:{rel}"
            if low is not None: records.append({"link":lname,"collision":col.get("name",str(i)),"kind":kind,"min_z_m":float(low),"is_drive_wheel":lname.endswith("_wheel_link") and not lname.startswith("flywheel"),"is_flywheel":lname.startswith("flywheel_left") or lname.startswith("flywheel_right")})
    drive=[r for r in records if r["is_drive_wheel"]]; fly=[r for r in records if r["is_flywheel"]]; other=[r for r in records if not r["is_drive_wheel"] and not r["is_flywheel"]]
    # URDF Z is chassis-local.  Normalize to the settled ground plane using
    # the lowest drive-tyre point; this supports R != axle-Z sensitivity.
    ground_z=min(r["min_z_m"] for r in drive)
    for r in records: r["clearance_from_settled_ground_m"]=r["min_z_m"]-ground_z
    result={"launcher_angle_deg":a.launcher_angle_deg,"chassis_local_ground_z_m":ground_z,"minimum_drive_wheel_z_m":0.0,"minimum_flywheel_z_m":min(r["clearance_from_settled_ground_m"] for r in fly) if fly else None,"minimum_nonwheel_nonflywheel_z_m":min(r["clearance_from_settled_ground_m"] for r in other),"only_drive_wheels_at_ground":min(r["clearance_from_settled_ground_m"] for r in fly+other)>0,"lowest_nonwheel_nonflywheel_collisions":sorted(other,key=lambda x:x["clearance_from_settled_ground_m"])[:10],"all_collisions":records}
    a.output.write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps({k:v for k,v in result.items() if k!="all_collisions"},indent=2))
if __name__=="__main__":main()
