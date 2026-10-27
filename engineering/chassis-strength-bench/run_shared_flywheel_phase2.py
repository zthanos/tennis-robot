#!/usr/bin/env python3
"""Phase-2 front-structure topology and sleeve-compliance FEA screen.

CalculiX B32R centreline model, mm/N/MPa.  This compares structural concepts;
it is not a printed-part strength or fatigue certification.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess

from run_global_frame_fea import ROOT, SOLVER, runtime_env
from run_shared_flywheel_phase1 import mass_model, traction_screen, pivot_loads

TOPOLOGIES = ("open_u", "arch", "x240_triangle", "combined")
CASES = ("vertical_2g", "launch_reaction", "one_wheel_lift", "antisymmetric_racking")
JOINT_MODES = ("clamped_no_play",)
PETG_E_MPA = 1500.0
PETG_NU = 0.38
SLEEVE_CLEARANCE_PER_SIDE_MM = 0.30
SLEEVE_ENGAGEMENT_MM = 60.0


def box_second_moment_mm4(width: float, height: float, wall: float) -> float:
    return (width*height**3-(width-2*wall)*(height-2*wall)**3)/12


def sleeve_model(pivot: dict) -> dict:
    theta = 2*math.atan(SLEEVE_CLEARANCE_PER_SIDE_MM/SLEEVE_ENGAGEMENT_MM)
    bearing_arm=math.dist((300,220,37),(397,234,64.3))
    nip_arms=[]
    for angle in (-10,20):
        t=math.radians(angle)
        nip=(397+math.cos(t)*45,64.3+math.sin(t)*45)
        nip_arms.append(math.dist((300,37),nip))
    return {
        "clearance_per_side_mm":SLEEVE_CLEARANCE_PER_SIDE_MM,
        "engagement_mm":SLEEVE_ENGAGEMENT_MM,
        "angular_deadband_rad":theta,
        "angular_deadband_deg":math.degrees(theta),
        "bearing_lever_arm_mm":bearing_arm,
        "free_bearing_translation_mm":bearing_arm*theta,
        "nip_lever_arm_range_mm":[min(nip_arms),max(nip_arms)],
        "free_nip_translation_range_mm":[min(nip_arms)*theta,max(nip_arms)*theta],
        "method":"pure geometric lost motion before contact; it is not a spring and is not added as an elastic modulus",
        "mitigation":"v3 M4 top-and-outer-side clamp screws preload each sleeve against two socket datum faces"
    }


def build_model(topology: str, joint_mode: str, mesh_mm: float, sleeve_e_mpa: float):
    if topology not in TOPOLOGIES or joint_mode not in JOINT_MODES:
        raise ValueError("invalid topology/joint mode")
    nodes, index, elements, sections = {}, {}, [], []

    def node(p):
        key=tuple(round(v,6) for v in p)
        if key not in index:
            index[key]=len(index)+1; nodes[index[key]]=key
        return index[key]

    def member(a,b,w=50,h=30,t=3.2,kind="BOX",material="PETG"):
        length=math.dist(a,b); count=max(1,math.ceil(length/mesh_mm))
        ids=[node(tuple(a[k]+(b[k]-a[k])*i/(2*count) for k in range(3))) for i in range(2*count+1)]
        eids=[]
        for i in range(count):
            eid=len(elements)+1; elements.append((eid,ids[2*i],ids[2*i+1],ids[2*i+2])); eids.append(eid)
        tangent=[(b[k]-a[k])/length for k in range(3)]
        axis=[0,0,1] if abs(tangent[2])<.95 else [0,1,0]
        dot=sum(axis[k]*tangent[k] for k in range(3)); normal=[axis[k]-dot*tangent[k] for k in range(3)]
        mag=math.sqrt(sum(v*v for v in normal)); normal=[v/mag for v in normal]
        sections.append((eids,w,h,t,kind,normal,material))

    sleeve_material="PETG"
    # Identical segmentation to the corrected Phase-1 model so the open-U
    # control differs only when a Phase-2 topology is deliberately added.
    stations=[-165,-65,35,200,208,240,272,300,320,330]
    for side in (-1,1):
        for a,b in zip(stations,stations[1:]):
            mat=sleeve_material if (a,b)==(240,300) else "PETG"
            member((a,side*220,37),(b,side*220,37),material=mat)
    member((-165,-220,37),(-165,220,37))

    # Retained X240 crossbar and its original local braces in every option.
    ys=[-220,-160,0,160,220]
    for a,b in zip(ys,ys[1:]):
        member((240,a,37 if abs(a)==220 else 32),(240,b,37 if abs(b)==220 else 32),50,20)
    for side in (-1,1):
        member((208,side*220,37),(240,side*160,32),6,20,0,"RECT")
        member((272,side*220,37),(240,side*160,32),6,20,0,"RECT")

    bearing_points=[]
    for side in (-1,1):
        # Exact Phase-1 bearing location and carrier topology are retained for
        # the controlled open-U comparison.  The released compact yoke moves
        # this point only 3 mm inward (Y 237 -> 234); that packaging change is
        # not allowed to masquerade as a stiffness/topology change.
        bearing=(397,side*237,64.3)
        member((300,side*220,37),(330,side*220,52),45,12,0,"RECT")
        member((330,side*220,52),bearing,45,12,0,"RECT")
        member((320,side*220,37),bearing,45,12,0,"RECT")
        bearing_points.append(node(bearing))

    if topology in ("arch","combined"):
        for side in (-1,1):
            # 60 mm base engagement is the explicit sleeve-sensitivity zone.
            member((330,side*220,37),(330,side*220,97),50,30,3.2,"BOX",sleeve_material)
            member((330,side*220,97),(330,side*220,245))
        member((330,-220,245),(330,220,245),50,30)

    if topology in ("x240_triangle","combined"):
        for side in (-1,1):
            # Braces terminate at actual rail centreline nodes.  The earlier
            # Z=52 endpoints were visually adjacent but disconnected from the
            # Z=37 beam mesh, so the solver could not transfer their load.
            for a in ((240,side*220,37),(300,side*220,37)):
                b=(330,side*220,245)
                length=math.dist(a,b); f=60/length
                split=tuple(a[k]+(b[k]-a[k])*f for k in range(3))
                member(a,split,40,25,3.2,"BOX",sleeve_material)
                member(split,b,40,25)

    if topology == "combined":
        # Close the otherwise open X240..X300 drive bay in the horizontal plane.
        # This low torsion deck stays below the intake guide and transfers an
        # asymmetric fore-aft bearing reaction into both longitudinal rails.
        for x in (240,300):
            for side in (-1,1):
                member((x,side*220,37),(x,side*220,32),40,20)
        member((300,-220,32),(300,220,32),20,20)
        # Printable v3 diagonals pin into the crossbar M4 grid at Y=+/-150;
        # crossbars, rather than hidden sleeve zones, carry load to the rails.
        member((240,-150,32),(300,150,32),20,10,0,"RECT")
        member((240,150,32),(300,-150,32),20,10,0,"RECT")

    supports={
        "rear_right":node((-65,-220,37)),"rear_left":node((-65,220,37)),
        "front_right":node((300,-220,37)),"front_left":node((300,220,37)),
    }
    return nodes,elements,sections,supports,bearing_points,{"sleeve_e_mpa":sleeve_e_mpa}


def constraints(supports, lifted=None):
    out=[]
    for name,n in supports.items():
        if name!=lifted: out.append((n,3))
    out += [(supports["rear_left"],1),(supports["rear_left"],2),(supports["front_left"],2)]
    return out


def loads(case_name, model, pivot):
    nodes,_,_,supports,bearings,_=model
    fixed=constraints(supports,"front_right" if case_name=="one_wheel_lift" else None)
    out=[]
    if case_name=="vertical_2g":
        out=[(n,3,-pivot["bearing_vertical_design_load_n_each"]) for n in bearings]
    elif case_name=="launch_reaction":
        for n in bearings:
            out += [(n,1,-pivot["bearing_fore_aft_design_load_n_each"]),
                    (n,5,pivot["launch_ball_reaction_torque_nm"]*1000/2)]
    elif case_name=="one_wheel_lift":
        out=[(n,3,-pivot["module_weight_n"]/2) for n in bearings]
    elif case_name=="antisymmetric_racking":
        for n in bearings:
            side=1 if nodes[n][1]>0 else -1
            out.append((n,1,side*pivot["bearing_fore_aft_design_load_n_each"]))
    return fixed,out


def write_deck(path, model, case_name, pivot, sleeve_e_mpa):
    nodes,elements,sections,supports,bearings,_=model
    fixed,case_loads=loads(case_name,model,pivot)
    lines=["*HEADING","Phase-2 front structure topology screen","*NODE,NSET=ALLN"]
    lines += [f"{n},"+",".join(map(str,p)) for n,p in nodes.items()]
    lines += ["*ELEMENT,TYPE=B32R,ELSET=ALLE"]+[",".join(map(str,e)) for e in elements]
    lines += ["*MATERIAL,NAME=PETG","*ELASTIC",f"{PETG_E_MPA},{PETG_NU}",
              "*MATERIAL,NAME=SLEEVE","*ELASTIC",f"{sleeve_e_mpa},{PETG_NU}"]
    for i,(ids,w,h,t,kind,normal,material) in enumerate(sections):
        lines += [f"*ELSET,ELSET=S{i}"]
        for j in range(0,len(ids),12): lines.append(",".join(map(str,ids[j:j+12])))
        lines += [f"*BEAM SECTION,ELSET=S{i},MATERIAL={material},SECTION={kind}",
                  f"{h},{w},{t},{t},{t},{t}" if kind=="BOX" else f"{h},{w}",
                  ",".join(map(str,normal))]
    measured=bearings+[supports["front_right"],supports["front_left"]]
    lines += ["*NSET,NSET=MEASURE",",".join(map(str,measured)),"*BOUNDARY"]
    lines += [f"{n},{d},{d}" for n,d in fixed]
    lines += ["*STEP","*STATIC","*CLOAD"]+[f"{n},{d},{v}" for n,d,v in case_loads]
    lines += ["*NODE PRINT,NSET=MEASURE","U","*EL PRINT,ELSET=ALLE","S","*END STEP"]
    path.write_text("\n".join(lines)+"\n")


def parse_dat(path, nodes, supports, bearings):
    disp={}; peak=0.; mode=""
    for line in path.read_text().splitlines():
        low=line.lower()
        if "displacements" in low: mode="u"
        elif "stresses" in low: mode="s"
        fields=line.split()
        try:
            if mode=="u" and len(fields)==4 and int(fields[0]) in set(bearings)|set(supports.values()):
                disp[int(fields[0])]=[float(v) for v in fields[1:]]
            elif mode=="s" and len(fields)==8:
                sx,sy,sz,txy,txz,tyz=map(float,fields[2:])
                vm=math.sqrt(((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)/2+3*(txy*txy+txz*txz+tyz*tyz))
                peak=max(peak,vm)
        except ValueError: pass
    expected=set(bearings)|{supports["front_right"],supports["front_left"]}
    if set(disp)!=expected: raise RuntimeError("missing measurement-node displacement")
    bearing_disp={n:disp[n] for n in bearings}
    ordered=sorted(bearing_disp.items())
    local=[]
    for bearing in bearings:
        front=supports["front_left"] if nodes[bearing][1]>0 else supports["front_right"]
        delta=[disp[bearing][i]-disp[front][i] for i in range(3)]
        local.append(math.sqrt(sum(x*x for x in delta)))
    return {"max_bearing_displacement_mm":max(math.sqrt(sum(x*x for x in u)) for u in bearing_disp.values()),
            "max_bearing_relative_to_local_front_mount_mm":max(local),
            "relative_bearing_x_mm":abs(ordered[0][1][0]-ordered[1][1][0]),
            "relative_bearing_z_mm":abs(ordered[0][1][2]-ordered[1][1][2]),
            "beam_integration_point_peak_von_mises_mpa":peak}


def run(work: Path):
    mass=mass_model(); traction=traction_screen(); pivot=pivot_loads(mass,traction)
    sleeve=sleeve_model(pivot); sleeve_e=PETG_E_MPA
    cases=[]
    for mesh in (10.,5.):
        for topology in TOPOLOGIES:
            for joint_mode in JOINT_MODES:
                model=build_model(topology,joint_mode,mesh,sleeve_e)
                for case_name in CASES:
                    stem=f"p2_{topology}_{joint_mode}_{case_name}_{mesh:g}mm"
                    deck=work/f"{stem}.inp"; write_deck(deck,model,case_name,pivot,sleeve_e)
                    proc=subprocess.run([str(SOLVER),"-i",stem],cwd=work,env=runtime_env(),capture_output=True,text=True,timeout=60)
                    (work/f"{stem}.log").write_text(proc.stdout+proc.stderr)
                    if proc.returncode or "*ERROR" in (proc.stdout+proc.stderr).upper():
                        raise RuntimeError(f"CalculiX failed: {stem}")
                    cases.append({"topology":topology,"joint_mode":joint_mode,"load_case":case_name,
                                  "mesh_mm":mesh,"element_count":len(model[1]),
                                  **parse_dat(work/f"{stem}.dat",model[0],model[3],model[4])})
    summary=[]
    for topology in TOPOLOGIES:
        for joint_mode in JOINT_MODES:
            rows=[c for c in cases if c["topology"]==topology and c["joint_mode"]==joint_mode and c["mesh_mm"]==5]
            launch=next(c for c in rows if c["load_case"]=="launch_reaction")
            summary.append({"topology":topology,"joint_mode":joint_mode,
                "launch_response_mm":launch["max_bearing_displacement_mm"],
                "launch_target_le_1mm":launch["max_bearing_displacement_mm"]<=1,
                "launch_local_response_mm":launch["max_bearing_relative_to_local_front_mount_mm"],
                "launch_local_target_le_1mm":launch["max_bearing_relative_to_local_front_mount_mm"]<=1,
                "responses_5mm":{r["load_case"]:{
                    "max_bearing_displacement_mm":r["max_bearing_displacement_mm"],
                    "max_bearing_relative_to_local_front_mount_mm":r["max_bearing_relative_to_local_front_mount_mm"],
                    "relative_bearing_x_mm":r["relative_bearing_x_mm"],
                    "peak_beam_stress_mpa":r["beam_integration_point_peak_von_mises_mpa"]} for r in rows}})
    convergence=[]
    for topology in TOPOLOGIES:
        for joint_mode in JOINT_MODES:
            for case_name in CASES:
                fine=next(c for c in cases if c["topology"]==topology and c["joint_mode"]==joint_mode and c["load_case"]==case_name and c["mesh_mm"]==5)
                coarse=next(c for c in cases if c["topology"]==topology and c["joint_mode"]==joint_mode and c["load_case"]==case_name and c["mesh_mm"]==10)
                convergence.append({"topology":topology,"joint_mode":joint_mode,"load_case":case_name,
                    "relative_change_10_to_5":abs(fine["max_bearing_displacement_mm"]-coarse["max_bearing_displacement_mm"])/fine["max_bearing_displacement_mm"]})
    return {"status":"PHASE_2_CAD_FEA_PRE_XACRO_STOP", "material":"PETG isotropic screen, E=1.5 GPa",
        "sleeve_joint_free_play":sleeve,"support_model":"same minimal two-drive-axle constraints and carrier topology as corrected Phase 1",
        "topology_definitions":{
            "open_u":"Phase-1 open front, X240 retained",
            "arch":"uprights/crossbar at X330 and Z245; actuator carried by arch, bearings remain on local carriers",
            "x240_triangle":"paired side diagonals from X240/X300 to the X330 arch crown, all behind the wheel sweep",
            "combined":"collision-free arch/side triangulation plus low X240..X300 drive-bay torsion deck"},
        "summary":summary,"mesh_convergence":convergence,"cases":cases,
        "limitations":["centreline ideal beam geometry; exact fasteners, print layers, notches and contacts omitted",
            "elastic results assume the v3 sleeve clamps are preloaded with no free play; clamp preload/contact is not solved",
            "unclamped 0.30 mm/side sleeve play is reported separately as geometric lost motion, not stiffness",
            "beam stress is comparative and not a printed-part allowable",
            "Gazebo is not used and cannot certify printed strength"]}


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--work",type=Path,default=ROOT/"fea/work/shared_flywheel_phase2")
    p.add_argument("--output",type=Path,default=ROOT/"results/shared_flywheel_phase2/report.json")
    args=p.parse_args(); args.work.mkdir(parents=True,exist_ok=True); args.output.parent.mkdir(parents=True,exist_ok=True)
    report=run(args.work); args.output.write_text(json.dumps(report,indent=2)+"\n"); print(json.dumps(report,indent=2))


if __name__=="__main__": main()
