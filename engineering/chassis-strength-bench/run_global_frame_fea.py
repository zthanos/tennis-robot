#!/usr/bin/env python3
"""Ideal-joint 3D B32R beam comparison, mm/N/MPa units; NOT a print release."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
SOLVER = Path("/snap/freecad/current/usr/bin/ccx")
BASE_VARIANTS = ("none", "middle", "ramp_back")
FRONT_STUDY_VARIANTS = BASE_VARIANTS + ("ramp_back_50x20", "ramp_back_50x30")


def runtime_env():
    env = os.environ.copy()
    base = Path("/snap/freecad/current/usr/lib")
    env["LD_LIBRARY_PATH"] = ":".join(str(base/p) for p in
        (".", "x86_64-linux-gnu", "x86_64-linux-gnu/blas", "x86_64-linux-gnu/lapack"))
    env["OMP_NUM_THREADS"] = "1"
    return env


def build(variant="none", mesh_mm=20):
    if variant not in FRONT_STUDY_VARIANTS or mesh_mm <= 0:
        raise ValueError("Invalid variant or mesh size")
    nodes, elements, segments, index = {}, [], [], {}
    def node(point):
        key = tuple(round(v, 8) for v in point)
        if key not in index:
            index[key] = len(index)+1
            nodes[index[key]] = key
        return index[key]
    def segment(a, b, width=50, height=30, wall=3.2, kind="BOX"):
        length = math.dist(a, b)
        count = max(1, math.ceil(length/mesh_mm))
        points = [tuple(a[k]+(b[k]-a[k])*i/(2*count) for k in range(3))
                  for i in range(2*count+1)]
        ids = [node(p) for p in points]
        element_ids = []
        for i in range(count):
            eid = len(elements)+1
            elements.append((eid, ids[2*i], ids[2*i+1], ids[2*i+2]))
            element_ids.append(eid)
        # Projection of global Z onto the plane normal to the member axis.
        tangent = [(b[k]-a[k])/length for k in range(3)]
        normal = [-tangent[0]*tangent[2], -tangent[1]*tangent[2], 1-tangent[2]**2]
        magnitude = math.sqrt(sum(v*v for v in normal))
        normal = [v/magnitude for v in normal]
        segments.append((element_ids,width,height,wall,kind,normal))
    # Wide candidates extend rearward: retain the nominal ramp-facing X=319.4.
    wide = variant in ("ramp_back_50x20", "ramp_back_50x30")
    cross_x = 117.5 if variant == "middle" else (294.4 if wide else 306.9)
    reach = 65 if variant == "middle" else 32
    anchors = [cross_x-reach,cross_x,cross_x+reach]
    stations = sorted(set([-165,-65,35,200,300,330,455] + (anchors if variant != "none" else [])))
    for side in (-1,1):
        for a,b in zip(stations,stations[1:]):
            segment((a,side*220,37),(b,side*220,37))
    segment((-165,-220,37),(-165,220,37))
    if variant != "none":
        width,height,rib,socket = (50,30,6,135) if variant == "middle" else (25,20,4,160)
        if wide:
            width = 50
            height = 30 if variant == "ramp_back_50x30" else 20
        z = 22+height/2
        ys = [-220,-socket,0,socket,220]
        for a,b in zip(ys,ys[1:]):
            za = 37 if abs(a)==220 else z
            zb = 37 if abs(b)==220 else z
            segment((cross_x,a,za),(cross_x,b,zb),width,height)
        for side in (-1,1):
            for direction in (-1,1):
                segment((cross_x+direction*reach,side*220,37),
                        (cross_x,side*socket,z),rib,height,0,"RECT")
    fixed = [node((-165,side*220,37)) for side in (-1,1)]
    front = [node((300,side*220,37)) for side in (-1,1)]
    return nodes,elements,segments,fixed,front


def write_deck(path, model, material, force=10):
    nodes,elements,segments,fixed,front = model
    lines = ["*HEADING", "Ideal-joint global chassis beam comparison", "*NODE,NSET=ALLN"]
    lines += [f"{n},"+",".join(map(str,p)) for n,p in nodes.items()]
    lines += ["*ELEMENT,TYPE=B32R,ELSET=ALLE"]
    lines += [",".join(map(str,e)) for e in elements]
    lines += ["*MATERIAL,NAME=POLYMER", "*ELASTIC",
              f"{material['reference_elastic_modulus_gpa']*1000},{material['poisson_ratio']}"]
    for i,(ids,w,h,t,kind,normal) in enumerate(segments):
        lines += [f"*ELSET,ELSET=S{i}"]
        for j in range(0,len(ids),12):
            lines.append(",".join(map(str,ids[j:j+12])))
        lines += [f"*BEAM SECTION,ELSET=S{i},MATERIAL=POLYMER,SECTION={kind}",
                  f"{h},{w},{t},{t},{t},{t}" if kind=="BOX" else f"{h},{w}",
                  ",".join(map(str,normal))]
    lines += ["*NSET,NSET=FRONT", ",".join(map(str,front)), "*BOUNDARY"]
    lines += [f"{n},1,6" for n in fixed]
    lines += ["*STEP", "*STATIC", "*CLOAD"]
    for n in front:
        side = 1 if nodes[n][1]>0 else -1
        lines += [f"{n},1,{side*force}",f"{n},6,{-force*130}"]
    lines += ["*NODE PRINT,NSET=FRONT", "U", "*NODE PRINT,NSET=ALLN", "RF",
              "*EL PRINT,ELSET=ALLE", "S", "*END STEP"]
    path.write_text("\n".join(lines)+"\n")


def parse_dat(path, front):
    displacement = {}
    mode = ""
    peak_vm = 0.0
    for line in path.read_text().splitlines():
        lower = line.lower()
        if "displacements" in lower:
            mode = "u"
        elif "forces" in lower:
            mode = "rf"
        elif "stresses" in lower:
            mode = "s"
        fields = line.split()
        try:
            if mode=="u" and len(fields)==4 and int(fields[0]) in front:
                displacement[int(fields[0])] = [float(v) for v in fields[1:]]
            elif mode=="s" and len(fields)==8:
                sx,sy,sz,txy,txz,tyz = map(float,fields[2:])
                vm=math.sqrt(((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)/2+3*(txy*txy+txz*txz+tyz*tyz))
                peak_vm=max(peak_vm,vm)
        except ValueError:
            continue
    if set(displacement) != set(front):
        raise RuntimeError("Solver did not output both front displacements")
    return {"front_displacement_mm":displacement,
            "max_front_displacement_mm":max(math.sqrt(sum(v*v for v in u)) for u in displacement.values()),
            "relative_front_x_mm":abs(displacement[front[0]][0]-displacement[front[1]][0]),
            "mean_front_y_mm":sum(displacement[n][1] for n in front)/2,
            "beam_integration_point_peak_von_mises_mpa":peak_vm}


def run(work, mesh_sizes=(20,10,5), force=10, variants=BASE_VARIANTS):
    if "none" not in variants or len(set(mesh_sizes)) < 2 or force <= 0:
        raise ValueError("Require unbraced reference, two distinct meshes and positive force")
    materials=json.loads((ROOT/"config/materials.json").read_text())["materials"]
    cases=[]
    for mesh in mesh_sizes:
        for variant in variants:
            model=build(variant,mesh)
            for name,material in materials.items():
                stem=f"{variant}_{name.replace('-','_')}_{mesh:g}mm"
                deck=work/f"{stem}.inp"
                write_deck(deck,model,material,force)
                result=subprocess.run([str(SOLVER),"-i",stem],cwd=work,env=runtime_env(),
                    text=True,capture_output=True,timeout=60)
                (work/f"{stem}.log").write_text(result.stdout+result.stderr)
                if result.returncode or "*ERROR" in result.stdout.upper():
                    raise RuntimeError(f"Solver failed: {stem}; see log")
                stats=parse_dat(work/f"{stem}.dat",model[-1])
                cases.append({"variant":variant,"material":name,"mesh_mm":mesh,
                    "element_count":len(model[1]),**stats})
    iz=(30*50**3-(30-6.4)*(50-6.4)**3)/12
    analytic=force*130*465**2/(2*2300*iz)
    finest=min(mesh_sizes)
    reference=next(c for c in cases if c["variant"]=="none" and c["material"]=="PLA" and c["mesh_mm"]==finest)
    error=abs(abs(reference["mean_front_y_mm"])-analytic)/analytic
    if error>0.03:raise RuntimeError("Unbraced beam failed analytic cantilever check")
    convergence=[]
    for variant in variants:
        for name in materials:
            pair=sorted((c for c in cases if c["variant"]==variant and c["material"]==name),key=lambda c:c["mesh_mm"])
            change=abs(pair[0]["max_front_displacement_mm"]-pair[1]["max_front_displacement_mm"])/pair[0]["max_front_displacement_mm"]
            convergence.append({"variant":variant,"material":name,"relative_change_finest_two_meshes":change})
            if change>0.02:raise RuntimeError("Displacement mesh convergence exceeds 2 percent")
    return {"status":"IDEAL_JOINT_ELASTIC_COMPARISON_NOT_PRINT_CERTIFICATION",
        "solver":"CalculiX B32R, BOX/RECT beam sections, mm/N/MPa",
        "force_per_front_mount_n":force,"moment_per_front_mount_nmm":-force*130,
        "rear_support":"Both rear rail/crossbar nodes clamped in six degrees of freedom",
        "front_study_geometry":{
            "ramp_facing_x_mm":319.4,"wide_crossbar_center_x_mm":294.4,
            "wide_bottom_z_mm":22,"rib_width_mm":4,"rib_reach_mm":32,
            "note":"Wide front candidates preserve ramp-facing plane; CAD fit not verified. 30 mm height obstructs current nut-access envelope."},
        "validation":{"analytic_unbraced_pla_lateral_mm":analytic,"analytic_relative_error":error,
                      "displacement_mesh_convergence":convergence},
        "cases":cases,"limitations":[
            "Ideal rigid joints, no sleeve clearance, slip, bolt preload or glue compliance",
            "Centreline beam model, not exact CAD; W, local roofs, Gamma plates and PCB omitted",
            "T rib ends idealized at member centreline; brace vertical offset included",
            "Isotropic provisional material profiles, no layer separation or fatigue",
            "Reference force is prescribed, not measured wheel load; stress peaks are not hole/notch allowables"]}


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--work",type=Path,default=ROOT/"fea/work/global_frame")
    p.add_argument("--output",type=Path,default=ROOT/"results/global_frame/comparison.json")
    p.add_argument("--force-n",type=float,default=10)
    p.add_argument("--front-width-study",action="store_true",
                   help="Include hypothetical 50x20 / 50x30 ramp-back candidates; no CAD modification")
    args=p.parse_args()
    if args.force_n<=0:p.error("force must be positive")
    args.work.mkdir(parents=True,exist_ok=True)
    report=run(args.work,force=args.force_n,
               variants=FRONT_STUDY_VARIANTS if args.front_width_study else BASE_VARIANTS)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
