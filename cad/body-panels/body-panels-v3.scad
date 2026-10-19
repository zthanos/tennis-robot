// BODY PANELS on the REAL v3 robot (full-robot-v3.scad, 2026-10-17 state):
// P2-65 v3 frame, P2-67 tilted arch + soft deflector, P2-68/P2-71 front grid
// wall + fail-closed door, P2-70 rear pod, P2-71 basket grids.
// Industrial-design CONCEPT "D".  No STL, no print release.  Nothing in
// cad/modular-test-chassis is modified -- everything is read via `use`.
// Units mm.  +X front, +Y left, +Z up.  Ground Z=0.
//
// style = "curved"  : single-curvature surfaces (arcs that lie in one plane),
//                     big fillets, plan outline on the R425 turn circle.
// style = "faceted" : the same layout with chamfers / straight facets, for an
//                     honest side-by-side comparison.
// Double-curvature skins are deliberately avoided: each piece prints on a
// 250x250 bed without supports and stays ~2.2 mm thick.
//
// Hard rules checked by qa-clearance-v3.py:
//   >=10 mm to the -10..+20 deg moving set (P2-40 module, P2-54 actuator/arm/
//   FIT0186/belt, cam, D5065 cables, P2-68 door + selector nose) and to the
//   drive tyres; nothing above Z375; outside camera FOV (P2-67 camera X410),
//   launch cone, R75/75 ski + flight + basket volume; plan radius <= 421 mm
//   about the in-place-turn centre X117 (P2-70 R425 envelope, 4 mm margin).

use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p267-tilted-arch.scad>
use <../modular-test-chassis/shared-flywheel-p268-upright-wall.scad>
use <../modular-test-chassis/shared-rear-pod-p270.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/drive-test-control-console.scad>

style = "curved";       // [curved, faceted, flow]  flow = curved + h3li0-inspired lines
bp_view = "assembly";
bp_angle = 0;
bp_piece = "";
bp_side = 1;
show_context = true;
show_console = false;   // temporary drive-test console (conflicts with the left sill)
show_marks = true;
show_keepouts = false;
show_turn_circle = false;
qa_probe = 0;
$fn = 48;

CURVED = style!="faceted";
FLOW = style=="flow";     // drift character line, pill doors, helix-twist gills (discreet)
T = 2.2;
icr = [117,0];          // in-place turn centre (P2-70)
r_turn = 425;           // existing turn envelope (P2-70)
r_keep = 421;           // panel limit: 4 mm inside R425
tyre_axle_x = [-65,300]; tyre_axle_z = 85; tyre_r = 85;
fender_r_in = tyre_r+12;
ball_r = 33;
cam_c = [410,0,305];    // P2-67 camera
z_ceiling = 375;
function nip(a) = gpoint(a,[55,0,0]);
function throat(a) = gpoint(a,[-50,0,0]);
function arcpts(c,r,a0,a1,n) = [for(i=[0:n]) let(a=a0+(a1-a0)*i/n) [c[0]+r*cos(a),c[1]+r*sin(a)]];
NARC = CURVED ? 24 : 3;   // facets per arc

// ---------------------------------------------------------------- colours
c_top  = [0.94,0.94,0.91];   // ASA white -- upward faces
c_side = [0.28,0.30,0.33];   // graphite
c_acc  = [0.05,0.45,0.78];   // signal blue
c_haz  = [0.96,0.42,0.05];   // RAL2004
c_gill = [0.06,0.07,0.08];

module slab(a,b){ translate([min(a[0],b[0]),min(a[1],b[1]),min(a[2],b[2])])
  cube([abs(b[0]-a[0]),abs(b[1]-a[1]),abs(b[2]-a[2])]); }
module keep_circle(h0=-10,h1=420){ translate([icr[0],icr[1],h0]) cylinder(r=r_keep,h=h1-h0,$fn=360); }
module label_y(s,p,size=10,face=1,h=0.6){
  translate(p) rotate([90,0,face>0?180:0]) linear_extrude(height=h)
    text(s,size=size,halign="center",valign="center",font="Liberation Sans:style=Bold");
}

// ================================================================ POD SKIN
// Proposed outer skin for the P2-70 pod, SAME envelope X-272..-125,
// Y+/-167, Z55..350, same rear louvres, filter hoods, -X drawers and top
// controls.  The 141x91x81 Parkside reservations fill the rear corners, so
// the plan corners keep the P2-70 5 mm break.  The curve goes where there is
// room: a large top fillet on the rear and side edges (contents top <= Z284).
pod_x0=-272; pod_x1=-125; pod_y=167; pod_z0=55; pod_z1=350; pod_t=2.5; // 2.5: 0.5 mm to the P2-70 3-mm-wall reservations
pod_rt = CURVED ? 40 : 30;           // top-edge fillet / chamfer
module pod_footprint(inset_rs=0, inset_all=0){
  // rear & sides inset by inset_rs (fillet layers), all sides by inset_all (wall)
  x0=pod_x0+inset_rs+inset_all; y=pod_y-inset_rs-inset_all; x1=pod_x1-inset_all;
  c = inset_all>0 ? 2 : 5;   // inner corner break 2 mm: clears the Parkside reservation corner
  polygon([[x1,-y],[x0+c,-y],[x0,-y+c],[x0,y-c],[x0+c,y],[x1,y]]);
}
module pod_solid(w=0){
  n = CURVED ? 8 : 1;
  hull(){
    translate([0,0,pod_z0+w]) linear_extrude(0.01) pod_footprint(0,w);
    for(i=[0:n]) let(ph=90*i/n)
      translate([0,0,pod_z1-pod_rt+pod_rt*sin(ph)-w])
        linear_extrude(0.01) pod_footprint(pod_rt*(1-cos(ph)),w);
  }
}
module pod_skin_full(){
  difference(){
    pod_solid(0);
    translate([0,0,-1]) pod_solid(pod_t);
    translate([pod_x1-pod_t-1,-pod_y+pod_t,pod_z0+pod_t]) cube([pod_t+2,2*pod_y-2*pod_t,pod_z1-pod_z0-2*pod_t]); // open front -> P2-70 base/back
    translate([-400,-300,pod_z0-50]) cube([600,600,50+pod_t-0.01]);   // no floor: P2-70 base tray
    for(y=[-55,-25,5,35],z=[155,180,205]) translate([pod_x0-1,y,z]) cube([pod_t+2,18,7]); // P2-70 louvres
    for(s=[-1,1]) translate([pod_x0-1,s*115.5-45.5,58]) cube([pod_t+2,91,81]); // -X Parkside drawer openings
    for(s=[-1,1]) translate([-176,s*pod_y-6,pod_z0-1]) cube([22,12,14]); // notch: P2-70 anti-racking ears
  }
}
module pod_doors(){ // drawer doors + filter hoods (P2-70 positions)
  for(s=[-1,1]){
    color(c_side) translate([pod_x0-2.5,s*115.5-44,59.5])
      if(FLOW) translate([0,0,0]) rotate([90,0,90]) linear_extrude(2.5)
        offset(r=18) offset(delta=-18) square([88,78]);
      else cube([2.5,88,78]);
    if(show_marks) color(c_acc) translate([pod_x0-3.1,s*115.5-30,120]) cube([0.6,60,5]);
    color([0.10,0.35,0.48]) translate([pod_x0-4,s*118,170]) cube([8,72,72],center=true);
  }
}
module pod_split(part){ // print split: Y0 and Z200 -> 4 pieces <=250x250
  if(part=="pod_upper") intersection(){ pod_skin_full(); slab([-400,0,200],[0,400,420]); }
  else if(part=="pod_lower") intersection(){ pod_skin_full(); slab([-400,0,0],[0,400,200]); }
}
module pod_skin(){
  color(c_side) intersection(){ pod_skin_full(); slab([-400,-400,0],[0,400,pod_z1-pod_rt]); }
  color(c_top)  intersection(){ pod_skin_full(); slab([-400,-400,pod_z1-pod_rt],[0,400,420]); }
  pod_doors();
  p270_controls();   // E-stop / covered switch / ARM / LED exactly as P2-70
}

// ================================================================ REAR QUARTERS
// The rear outline IS the turn circle: an R(keep) wall from the pod side to
// just behind the rear tyre (X-162 = tyre X-150 - 12 mm).  Spray guard behind
// the rear tyre, hides crossbar end + corner clamp, protects drawers' flanks.
rq_x_front = -162; rq_y_in = 170; rq_z0 = 68; rq_z_in = 300; rq_z_out = 238;
rq_r = r_keep-1;
function rq_arc_y(x) = sqrt(rq_r*rq_r-(icr[0]-x)*(icr[0]-x));
rq_a0 = 180-asin(rq_arc_y(rq_x_front)/rq_r);       // front end of arc (deg)
rq_a1 = 180-asin((rq_y_in+2)/rq_r);                 // inner end (at the pod)
module rq_plan(off=0){
  pts = arcpts(icr,rq_r-off,rq_a0,rq_a1,NARC);
  polygon(concat(pts,[[rq_x_front+40,rq_y_in-off-30],[rq_x_front+40,pts[0][1]]]));
}
function rq_cap_z(y) = rq_z_in-(y-rq_y_in)*(rq_z_in-rq_z_out)/(rq_arc_y(rq_x_front)-rq_y_in);
module rq_body(off=0,dz=0){
  intersection(){
    translate([0,0,rq_z0+dz]) linear_extrude(rq_z_in+20) rq_plan(off);
    hull(){ // sloped cap plane
      translate([-400,rq_y_in-40,0]) cube([300,1,rq_cap_z(rq_y_in-40)-dz]);
      translate([-400,rq_arc_y(rq_x_front)+40,0]) cube([300,1,rq_cap_z(rq_arc_y(rq_x_front)+40)-dz]);
    }
    slab([-400,rq_y_in,0],[rq_x_front,400,420]);
  }
}
module rear_quarter(){
  difference(){
    rq_body(0,0);
    rq_body(T,T);
    slab([-400,0,-1],[rq_x_front+40,rq_y_in+T,420]);   // open towards the pod
    slab([rq_x_front-T,0,-1],[0,400,420]);               // open towards the tyre
    translate([0,0,-1]) linear_extrude(rq_z0+T+1) rq_plan(T); // open bottom (drain)
  }
}
module rq_cap_band(){ difference(){ rq_body(-5,0); rq_body(-5,T+0.4); } }
module rear_quarter_coloured(){   // white = sloped cap only, graphite = walls
  color(c_side) difference(){ rear_quarter(); rq_cap_band(); }
  color(c_top)  intersection(){ rear_quarter(); rq_cap_band(); }
}

// ================================================================ SIDE SILLS
// Controller housings between the tyres (proposed D5065 controller location;
// the P2-52 envelope still sits inside the basket).  Plan: flat Y244 in the
// tyre zones, bulging to Y250 between them.  YZ profile: vertical inner wall
// at Y179 (5.5 mm from basket grid Y173.5), big outer-top fillet, top slopes
// outward (rain sheds away from the basket).
sl_y_in = 179; sl_z0 = 54.5; sl_z_top_in = 272; sl_z_top_out = 262;
sl_r_top = CURVED ? 42 : 30; sl_r_bot = CURVED ? 14 : 10;
sl_stations = [[-30,244],[25,244],[62,250],[178,250],[210,244],[215,244]];
function sl_prof(yo) =
  CURVED ? concat([[sl_y_in,sl_z0]],
                  arcpts([yo-sl_r_bot,sl_z0+sl_r_bot],sl_r_bot,-90,0,10),
                  arcpts([yo-sl_r_top,sl_z_top_out-sl_r_top],sl_r_top,0,90,16),
                  [[sl_y_in,sl_z_top_in]])
         : [[sl_y_in,sl_z0],[yo-sl_r_bot,sl_z0],[yo,sl_z0+sl_r_bot],
            [yo,sl_z_top_out-sl_r_top],[yo-sl_r_top,sl_z_top_out],[sl_y_in,sl_z_top_in]];
module sl_slice(x,yo,w){ translate([x,0,0]) rotate([90,0,90]) linear_extrude(0.01)
  offset(delta=-w) polygon(sl_prof(yo)); }
module sl_loft(w=0,x0=-30,x1=215){
  st=[for(s=sl_stations) [min(max(s[0],x0),x1),s[1]]];
  for(i=[0:len(st)-2]) if(st[i+1][0]>st[i][0]) hull(){ sl_slice(st[i][0],st[i][1],w); sl_slice(st[i+1][0],st[i+1][1],w); }
}
sl_gx0 = 72; sl_gx1 = 168;
sl_gills_low = [100,113,126]; sl_gills_high = [182,195,208];
module sl_gill_cuts(){ // slots rise inward: outside opening is lower -> no rain path
  if(FLOW) {
    // helix: short dashes whose tilt twists -28..+28 deg along X; each dash
    // still rises inward by 5 mm, so the rain-shedding geometry is unchanged.
    n=8; w=(sl_gx1-sl_gx0)/n;
    for(z=concat(sl_gills_low,sl_gills_high), i=[0:n-1]) let(xc=sl_gx0+(i+0.5)*w, a=-28+56*i/(n-1))
      hull(){
        translate([xc,240.5,z+2]) rotate([0,a,0]) cube([w-3,1,4],center=true);
        translate([xc,253.5,z-3]) rotate([0,a,0]) cube([w-3,1,4],center=true);
      }
  } else
  for(z=concat(sl_gills_low,sl_gills_high)) hull(){
    translate([sl_gx0,240,z]) cube([sl_gx1-sl_gx0,1,4]);
    translate([sl_gx0,253,z-5]) cube([sl_gx1-sl_gx0,1,4]);
  }
}
sl_door_box = [[-27.8,234,74],[212.8,262,218]];
sl_end_r = FLOW ? 40 : CURVED ? 28 : 20;     // plan rounding of the four vertical end edges
module sl_plan(w=0){ // rounded/chamfered plan limit, inset w
  translate([0,0,-1]) linear_extrude(420)
    if(CURVED) offset(r=sl_end_r-w) offset(delta=-sl_end_r) translate([-30,sl_y_in-60]) square([245,260-sl_y_in+60]);
    else offset(delta=sl_end_r-w,chamfer=true) offset(delta=-sl_end_r) translate([-30,sl_y_in-60]) square([245,260-sl_y_in+60]);
}
module sl_shell(){
  difference(){
    intersection(){ sl_loft(0); sl_plan(0); }
    intersection(){ sl_loft(T,-30+T,215-T); sl_plan(T); }
    sl_gill_cuts();
  }
}
module sl_door_region(){ // pill-shaped door outline for FLOW, plain box otherwise
  if(FLOW) translate([0,sl_door_box[1][1],0]) rotate([90,0,0])
    linear_extrude(sl_door_box[1][1]-sl_door_box[0][1])
      offset(r=34) offset(delta=-34)
        translate([sl_door_box[0][0],sl_door_box[0][2]])
          square([sl_door_box[1][0]-sl_door_box[0][0],sl_door_box[1][2]-sl_door_box[0][2]]);
  else slab(sl_door_box[0],sl_door_box[1]);
}
module sill_body(){ difference(){ sl_shell(); sl_door_region(); } }
module sill_door(){ intersection(){ sl_shell(); sl_door_region(); } }
module sill_coloured(){
  color(c_side) intersection(){ sill_body(); slab([-100,0,0],[300,400,sl_z_top_out-sl_r_top*0.6]); }
  color(c_top)  intersection(){ sill_body(); slab([-100,0,sl_z_top_out-sl_r_top*0.6],[300,400,400]); }
  color(c_acc) sill_door();
}
module sill_labels(){ if(show_marks) for(s=[1,-1])
  if(FLOW) color(c_top) label_y("TR-1",[120,s*250.6,86],11,s);   // under the lower gills
  else color(c_top) label_y("TR-1",[120,s*250.6,150],20,s); }

// ================================================================ FENDERS
// Arc R97 (12 mm radial to the Ø170 tyre envelope), Y248..336, trimmed in
// plan by the R421 keep circle -> the outer front/rear corners follow the
// turn circle.  Faceted style: 4 straight facets.
fd_y0=248; fd_y1=336;
module fender_raw(ax){
  // faceted: vertices on r/cos(half-step) so every FLAT face keeps >= r_in
  n = CURVED ? 40 : 4; k = CURVED ? 1 : 1/cos(70/n);
  outer = concat([[0,0]],[for(i=[0:n]) let(a=-70+140*i/n) (fender_r_in+T)*k*[sin(a),cos(a)]]);
  inner = concat([[0,0]],[(fender_r_in*k+5)*[sin(-80),cos(-80)]],
                 [for(i=[0:n]) let(a=-70+140*i/n) fender_r_in*k*[sin(a),cos(a)]],
                 [(fender_r_in*k+5)*[sin(80),cos(80)]]);
  translate([ax,fd_y1,tyre_axle_z]) rotate([90,0,0]) linear_extrude(fd_y1-fd_y0)
    difference(){ polygon(outer); polygon(inner); }
}
module fender(ax){
  intersection(){ fender_raw(ax); keep_circle(); }
}
module fender_coloured(ax){ color(c_top) fender(ax); }

// ================================================================ SHOULDERS
// Outboard of the tilted X318->X390 arch leg, above the front tyre (Z>=186,
// tyre top Z170).  Front edge parallel to the leg (20.5 deg) so the robot
// "leans" with its arch; covers actuator/belt/cable loops from the side.
sh_y = 246;
function leg_x(z) = 318+(z-52)*(390-318)/(245-52);
sh_ztop = 270;
function sh_outline() = CURVED
  ? concat([[244,186],[leg_x(186)+22,186]],
           arcpts([leg_x(sh_ztop)+22-26,sh_ztop-26],26,-20.5+0,90,10),
           arcpts([300,sh_ztop-70],70,90,150,10),
           [[244,212]])
  : [[244,186],[leg_x(186)+22,186],[leg_x(sh_ztop-14)+22,sh_ztop-14],[leg_x(sh_ztop)+8,sh_ztop],
     [292,sh_ztop],[262,sh_ztop-30],[244,212]];
module shoulder(){
  color(c_side) translate([0,sh_y+T,0]) rotate([90,0,0]) linear_extrude(T) polygon(sh_outline());
  color(c_top) intersection(){ // short inward return on top (above crossbar Z260)
    translate([0,222,sh_ztop-T]) linear_extrude(T) polygon([[300,0],[leg_x(sh_ztop)+10,0],[leg_x(sh_ztop)+10,sh_y+T-222],[300,sh_y+T-222]]);
    slab([0,0,0],[600,400,400]);
  }
  if(show_marks) color(c_haz) intersection(){
    for(i=(FLOW?[2:3]:[0:3])) translate([270+i*28,sh_y+T,186]) rotate([0,-35,0]) cube([9,0.8,140]);
    translate([0,sh_y+T+0.8,0]) rotate([90,0,0]) linear_extrude(1) offset(delta=-8) polygon(sh_outline());
  }
}

// ================================================================ BROW
// Cap on the P2-67 crossbar (X365..415, Z230..260).  Ends at X404: the
// camera FOV ray at Z268 is X~421.  Hollow, open underneath (snap cap).  Nose rounded (curved) or chamfered.
// Notch for the LiDAR mast posts X377.5..402.5 Y137.5..162.5.
br_x0=362; br_x1=404; br_z0=260.5;   // X404: >=12 mm to the camera FOV
function br_prof() = CURVED
  ? concat([[br_x0,br_z0],[br_x1,br_z0]],
           arcpts([br_x1-8,br_z0+0.1],8,0,90,8),
           [[br_x0+10,br_z0+7.5],[br_x0,br_z0+4]])
  : [[br_x0,br_z0],[br_x1,br_z0],[br_x1,br_z0+3],[br_x1-6,br_z0+8],[br_x0+10,br_z0+7.5],[br_x0,br_z0+4]];
module brow_half(){
  difference(){
    union(){
      translate([0,238,0]) rotate([90,0,0]) linear_extrude(236) difference(){
        polygon(br_prof());
        offset(delta=-T) polygon(br_prof());
        translate([br_x0+T,br_z0-1]) square([br_x1-br_x0-2*T,T+1.01]);   // open underside
      }
      slab([br_x0,2,250],[br_x0+T,190,br_z0+1]); // rear drip lip, clear of arch braces
    }
    translate([376,136,240]) cube([28,28,40]);
  }
}
module brow(){
  for(s=[1,-1]) mirror([0,s>0?0:1,0]) color(c_top) brow_half();
  if(show_marks) color(c_acc) for(s=[1,-1]) translate([385,s*80,br_z0+7.6]) linear_extrude(0.6)
    polygon([[-10,-8],[-10,8],[12,0]]);
}

// ================================================================ FLOW: drift character line
// One continuous wave read in side view: high at the pod (Z~210), dipping to
// Z158 between the two gill bands of the sill door (X110), rising steeply up
// the tilted shoulder (Z~250).  Two half-parabolas with zero slope at the
// vertex -> smooth.  Printed as a 0.8 mm raised bead (or groove) integral to
// each panel; it jumps the gaps between panels like a car's shoulder line.
function zline(x) = x<110 ? 158+0.00037*(x-110)*(x-110) : 158+0.00164*(x-110)*(x-110);
module bead_run(x0,x1,yfn,col){
  color(col) for(x=[x0:4:x1-4]) hull(){
    translate([x,yfn(x),zline(x)]) cube([0.2,0.8,3],center=true);
    translate([x+4,yfn(x+4),zline(x+4)]) cube([0.2,0.8,3],center=true);
  }
}
module flow_line(){
  bead_run(-262,-166,function(x) rq_arc_y(x)+0.4,c_acc);   // rear quarter (R420 arc)
  bead_run(-160,-130,function(x) pod_y+0.4,c_acc);          // visible strip of pod flank
  bead_run(66,174,function(x) 250.4,c_top);                  // sill door bulge
  bead_run(268,352,function(x) sh_y+T+0.4,c_acc);           // shoulder
}

// ================================================================ assembly
module side_sym(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) children(); }
module panels(){
  pod_skin();
  side_sym(){ rear_quarter_coloured(); sill_coloured(); fender_coloured(tyre_axle_x[0]);
              fender_coloured(tyre_axle_x[1]); shoulder(); if(FLOW) flow_line(); }
  sill_labels(); brow();
}
pieces = ["pod_upper","pod_lower","rear_quarter","sill_body","sill_door",
          "fender_rear","fender_front","shoulder","brow"];
module piece(n){
  if(n=="pod_upper"||n=="pod_lower") pod_split(n);
  else if(n=="rear_quarter") rear_quarter();
  else if(n=="sill_body") sill_body();
  else if(n=="sill_door") sill_door();
  else if(n=="fender_rear") fender(tyre_axle_x[0]);
  else if(n=="fender_front") fender(tyre_axle_x[1]);
  else if(n=="shoulder") union(){ translate([0,sh_y+T,0]) rotate([90,0,0]) linear_extrude(T) polygon(sh_outline());
      translate([0,222,sh_ztop-T]) linear_extrude(T) polygon([[300,0],[leg_x(sh_ztop)+10,0],[leg_x(sh_ztop)+10,sh_y+T-222],[300,sh_y+T-222]]); }
  else if(n=="brow") brow_half();
  else assert(false,str("unknown piece ",n));
}
module sided_piece(n,s){ mirror([0,s>0?0:1,0]) piece(n); }
module all_pieces_qa(){ for(n=pieces) for(s=[1,-1]) sided_piece(n,s); }

// ---------------------------------------------------------------- context (= full-robot-v3 defaults)
module context(a){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors();
  p271_basket_exact(); p267_ski(75); p251_feed_exact();
  d5065_controller_envelopes();
  p253_frozen_launcher(a); p253_actuator(a); p253_as5600(); p253_stops(); p253_cables(a);
  p267_soft_deflector(); p271_front_system(a,true);
  p270_mounts();
  if(show_console) translate([45,220,52]) color([0.9,0.9,0.2]) console();
}
module bp_moving(a){
  p253_frozen_launcher(a);
  module_pose(a) p244_side_cheeks();
  p253_actuator(a); p253_cam_moving(a); p253_cables(a);
  p268_feed_door(a,false); p268_selector_nose(a);
}
module bp_airflow(a){ module_pose(a) for(yy=[-129,129]) d5065_airflow_reservation(yy); }
module bp_fixed(){
  frame_assembly(); pivot_supports(); p267_arch(); p267_sensors();
  p271_basket_exact(); p267_ski(75); p251_feed_exact(); p267_soft_deflector();
  p271_front_grid(); p268_fixed_feed_channel(); p253_as5600(); p253_stops();
  p270_mounts(); drive_hardware_envelopes();
}
module ko_launch_cone(L=1600){
  for(a=[15,17.5,20]) hull(){
    translate(nip(a)) sphere(r=ball_r);
    for(de=[-10,10], sy=[-1,1]){ e=a+de; d=[cos(e),0,sin(e)];
      translate(nip(a)+L*d+[0,sy*L*tan(10),0]) sphere(r=ball_r); }
  }
}
module ko_camera_fov(){ hull(){ translate(cam_c) sphere(d=8);
  for(p=[[605,-120,33],[605,120,33],[490,-70,33],[490,70,33]]) translate(p) sphere(d=8); }}
module ko_lidar(){ translate([-400,-450,z_ceiling]) cube([1100,900,60]); }
module ko_ball_path(){ // intake, nip, R75/75 ski, flight + deflector + basket volume
  hull(){ translate([620,0,ball_r]) sphere(r=ball_r+10); translate(nip(-3)) sphere(r=ball_r+10); }
  pts=concat([[435,10],[383,10]],[for(t=[15:15:75]) [383-75*sin(t),10+75*(1-cos(t))]]);
  for(i=[0:len(pts)-2]) hull() for(p=[pts[i],pts[i+1]],y=[-47,47]) translate([p[0],y,p[1]+ball_r]) sphere(r=ball_r+10);
  // Flight + deflector + basket.  The ball leaves the R75/75 ski at X~311
  // travelling REARWARD (-X), so its surface never passes X~344; box = +10 mm.
  translate([-122,-176,40]) cube([355+122,352,340]);
}
module ko_feed_path(){ for(a=[15,20]) hull() for(y=[-40,40]){
  translate([278,y,42+ball_r]) sphere(r=ball_r+10); translate(throat(a)+[0,y,0]) sphere(r=ball_r+10); }}
module ko_pod_contents(){ p270_reserved_contents(); }
module inflated_panels(){ minkowski(){ all_pieces_qa(); sphere(r=qa_probe/pow(cos(180/16),2),$fn=16); } }
module keepouts(){
  %color([1,0.2,0.2,0.15]) ko_launch_cone();
  %color([0.1,0.75,0.95,0.20]) ko_camera_fov();
}
module turn_circle_mark(){ color([1,1,1]) translate([icr[0],0,1]) difference(){
  cylinder(r=r_turn+1.5,h=1,$fn=240); translate([0,0,-1]) cylinder(r=r_turn-1.5,h=3,$fn=240); } }

module scene(a){
  if(show_context) context(a);
  panels();
  if(show_keepouts) keepouts();
  if(show_turn_circle) turn_circle_mark();
}
module ground_marks(){ color([0.75,0.75,0.75]) translate([150,0,-1.5]) cube([1000,3,3],center=true); }
module annot(t){
  color("white") translate([560,0,30]) rotate([90,0,0]) linear_extrude(1,center=true)
    text("FRONT +X >",size=18,halign="left",valign="center");
  color("white") translate([-300,0,420]) rotate([90,0,0]) linear_extrude(1,center=true)
    text(t,size=18,halign="left");
}

if(bp_view=="assembly") scene(bp_angle);
else if(bp_view=="side"){ scene(bp_angle); ground_marks(); annot(str("D ",style,"  ",bp_angle," deg")); }
else if(bp_view=="side_section"){
  intersection(){ scene(bp_angle); translate([-500,0,-50]) cube([1200,500,600]); }
  ground_marks(); annot(str("D ",style,"  section Y=0  ",bp_angle," deg"));
}
else if(bp_view=="panels") panels();
else if(bp_view=="piece") sided_piece(bp_piece,bp_side);
else if(bp_view=="mesh_piece") sided_piece(bp_piece,bp_side);
else if(bp_view=="mesh_panels") all_pieces_qa();
else if(bp_view=="mesh_moving") bp_moving(bp_angle);
else if(bp_view=="mesh_airflow") bp_airflow(bp_angle);
else if(bp_view=="mesh_tyres") drive_tire_envelopes();
else if(bp_view=="mesh_stops") p253_stops();
else if(bp_view=="mesh_fixed") bp_fixed();
else if(bp_view=="mesh_console") translate([45,220,52]) console();
else if(bp_view=="qa_cone") intersection(){ all_pieces_qa(); ko_launch_cone(); }
else if(bp_view=="qa_fov") intersection(){ all_pieces_qa(); ko_camera_fov(); }
else if(bp_view=="qa_lidar") intersection(){ all_pieces_qa(); ko_lidar(); }
else if(bp_view=="qa_ball") intersection(){ all_pieces_qa(); ko_ball_path(); }
else if(bp_view=="qa_feed") intersection(){ all_pieces_qa(); ko_feed_path(); }
else if(bp_view=="qa_pod_contents") intersection(){ all_pieces_qa(); ko_pod_contents(); }
else if(bp_view=="qa_turn") difference(){ // pod skin = P2-70 envelope (R421.4), checked numerically
  for(n=pieces) if(n!="pod_upper" && n!="pod_lower") for(s=[1,-1]) sided_piece(n,s);
  keep_circle(); }
else if(bp_view=="qa_fixed") intersection(){ all_pieces_qa(); bp_fixed(); }
else if(bp_view=="qa_moving_sweep") intersection(){
  if(qa_probe>0) inflated_panels(); else all_pieces_qa();
  union(){ for(a=[-10:1:20]) bp_moving(a); }
}
else if(bp_view=="qa_tyres") intersection(){
  if(qa_probe>0) inflated_panels(); else all_pieces_qa();
  drive_tire_envelopes();
}
else scene(bp_angle);

echo("BODY_PANELS_V3_CONCEPT_D_NO_STL", style=style, nip15=nip(15), nip20=nip(20),
     rq_arc_front_y=rq_arc_y(rq_x_front));
