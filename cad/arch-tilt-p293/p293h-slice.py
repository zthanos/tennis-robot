#!/usr/bin/env python3
"""PARKED analysis-only slicer probe. Does not modify any input STL/3MF."""
import argparse,json,subprocess,tempfile,zipfile,re,os
import trimesh
from pathlib import Path
H=Path(__file__).resolve().parent;O=H/'p293h-analysis';O.mkdir(exist_ok=True)
ap=argparse.ArgumentParser();ap.add_argument('--bambu',required=True);ap.add_argument('--only');ap.add_argument('--library-path');args=ap.parse_args()
with zipfile.ZipFile(H/'stl/arch-tilt.3mf') as z:cfg=json.loads(z.read('Metadata/project_settings.config'))
cfg.update({'from':'user','name':'P293h analysis solid P2S','type':'process'})
cfg['print_compatible_printers']=[]
cfg.update(sparse_infill_density='100%',sparse_infill_pattern='rectilinear',wall_loops='5',layer_height='0.2',
           enable_support='1',support_type='normal(auto)',support_threshold_angle='30',brim_type='auto_brim',
           filament_density=['1.27'],filament_max_volumetric_speed=['12'],curr_bed_type='Textured PEI Plate')
(O/'slice-settings.json').write_text(json.dumps(cfg,indent=2)+'\n')
machine=dict(cfg);machine.update(type='machine',name='Bambu Lab P2S 0.4 nozzle',compatible_printers=[],compatible_printers_condition='')
(O/'slice-machine.json').write_text(json.dumps(machine,indent=2)+'\n')
cfg.update(compatible_printers=[],compatible_printers_condition='')
(O/'slice-settings.json').write_text(json.dumps(cfg,indent=2)+'\n')
rows=[]
for path in sorted((H/'stl-candidate-p293g').glob('*.stl')):
 if args.only and args.only not in path.stem:continue
 with tempfile.TemporaryDirectory(prefix='p293h-slice-') as work:
  mesh=trimesh.load(path,force='mesh')
  vertices=''.join(f'<vertex x="{x}" y="{y}" z="{z}"/>' for x,y,z in mesh.vertices)
  triangles=''.join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a,b,c in mesh.faces)
  xml=f'<?xml version="1.0"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources><object id="1" type="model"><mesh><vertices>{vertices}</vertices><triangles>{triangles}</triangles></mesh></object></resources><build><item objectid="1"/></build></model>'
  input3mf=Path(work)/'input.3mf';projectcfg=dict(cfg);projectcfg.update({'from':'project','name':'project_settings'})
  with zipfile.ZipFile(input3mf,'w',zipfile.ZIP_DEFLATED) as z,zipfile.ZipFile(H/'stl/arch-tilt.3mf') as original:
   for n in ['[Content_Types].xml','_rels/.rels']:z.writestr(n,original.read(n))
   z.writestr('3D/3dmodel.model',xml)
   z.writestr('Metadata/project_settings.config',json.dumps(projectcfg))
  cmd=[args.bambu,'--datadir',work+'/config',
       '--skip-useless-pick','--min-save','--arrange','1','--orient','0','--ensure-on-bed','--slice','0','--outputdir',work,'--export-3mf','sliced.3mf',str(input3mf)]
  env=dict(os.environ)
  if args.library_path:env['LD_LIBRARY_PATH']=args.library_path
  run=subprocess.run(cmd,cwd=work,env=env,text=True,capture_output=True,timeout=900)
  log=run.stdout+run.stderr;(O/(path.stem+'-slice.log')).write_text(log)
  projects=[Path(work)/'sliced.3mf'];gcode=''
  for project in projects:
   if not project.exists():continue
   with zipfile.ZipFile(project) as z:
    gc=[n for n in z.namelist() if n.endswith('.gcode')]
    if gc:gcode=z.read(gc[0]).decode(errors='replace')
  comments=[l for l in gcode.splitlines() if l.startswith(';') and any(t in l.lower() for t in ['time','filament','weight','volume'])]
  (O/(path.stem+'-gcode-summary.txt')).write_text('\n'.join(comments)+'\n')
  row=dict(STL=path.name,returncode=run.returncode,gcode_found=bool(gcode),summary=comments)
  rows.append(row);print(row,flush=True)
  if run.returncode or not gcode:print(log[-3000:],flush=True)
(O/('slice-results'+('-'+args.only if args.only else '')+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
