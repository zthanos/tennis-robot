// BODY PANELS -- industrial-design CONCEPT study.  NOT a print release, no STL.
// Units mm.  +X front, +Y left, +Z up.  Ground Z=0.
// Context geometry is taken read-only from the frozen P2-40 / P2-51 / P2-54
// assembly via `use` -- nothing in cad/modular-test-chassis is modified.
//
// Concepts:
//   "A" Minimal   -- rear power pod + two compact controller pods only.
//   "B" Pods+     -- (RECOMMENDED) A + full side pods with louvres, four tyre
//                    fenders, front shoulders, arch brow, provisional flywheel
//                    side guards.
//   "C" Shell     -- continuous side skin up to Z300, wedge rear hood, tall
//                    shoulders and larger provisional front guards.
//
// Hard rules encoded here (checked by bp_view="qa_*" exports, see
// docs/design/body-panels/panels-concept-el.md):
//   * every panel >= 10 mm from the swept flywheel module, actuator, arm,
//     belt/FIT0186, cam, D5065 cables (-10..+20 deg) and from the drive tyres;
//   * nothing above Z 375 (LiDAR scan plane Z390, 360 deg);
//   * outside the camera FOV hull, the +15..+20 deg launch cone (+/-10 deg,
//     inflated by the ball radius), the nip->ski->basket ball path, the open
//     basket top/lift-out volume and the temporary X278/Z42 feed path;
//   * |Y| <= drive-tyre outer face (338.5); every piece <= 250x250 mm.

use <../modular-test-chassis/shared-flywheel-transition-p252.scad>

concept = "B";        // [A,B,C]
bp_view = "assembly"; // see dispatcher at the end of the file
bp_angle = 0;         // flywheel-module angle for assembly views
bp_piece = "";        // single-piece name for bp_view="piece"
bp_side = 1;          // +1 left / -1 right for single pieces
show_context = true;
show_marks = true;
show_zones = false;   // temporary (PROSORINES) zones as translucent volumes
show_keepouts = false;
qa_probe = 0;         // inflate panels for clearance gate (mm)
$fn = 32;

// ---------------------------------------------------------------- context data
// Read-only copies of frozen values (sources noted).  pivot from gpoint().
T = 2.2;                       // nominal printed wall
tyre_axle_x = [-65,300];       // phase1 drive_mount_x
tyre_axle_z = 85;              // phase1 drive_axle_z
tyre_r = 85;                   // phase1 drive_tire_envelopes d170
tyre_in_y = 297-83/2;          // 255.5 tyre inner face
tyre_out_y = 297+83/2;         // 338.5 overall half width
fender_r_in = tyre_r+12;       // 12 mm radial clearance
ball_r = 33;
cam_c = [350,0,305];           // phase1 camera_center
lidar_z = 390;                 // phase1 lidar_scan_z
z_ceiling = 375;               // brief: nothing above ~Z375
function pv() = gpoint(0,[0,0,0]);           // [387,0,51.84]
function nip(a) = gpoint(a,[55,0,0]);
function throat(a) = gpoint(a,[-50,0,0]);   // rear nip mouth (local u = nip-105)

// ---------------------------------------------------------------- colours
c_top   = [0.93,0.93,0.90];    // ASA/PETG white  -- all upward faces
c_side  = [0.30,0.32,0.35];    // graphite        -- verticals
c_light = [0.78,0.80,0.80];    // light grey      -- concept A / C variants
c_acc   = [0.05,0.45,0.78];    // signal blue accent / pod doors
c_haz   = [0.96,0.42,0.05];    // RAL2004 pure orange -- launch hazard zone
c_black = [0.06,0.06,0.07];
c_estop = [0.85,0.05,0.05];
c_estop_bg = [0.98,0.82,0.05];
c_temp  = [0.96,0.42,0.05,0.30];

// ---------------------------------------------------------------- primitives
// Convex plate through corner points (each corner a T-cube -> ~T thick).
module plate(pts,t=T){ hull() for(p=pts) translate(p) cube(t,center=true); }
// Axis-aligned slab between two corners.
module slab(a,b){ translate([min(a[0],b[0]),min(a[1],b[1]),min(a[2],b[2])])
  cube([abs(b[0]-a[0]),abs(b[1]-a[1]),abs(b[2]-a[2])]); }
// XZ-plane arc shell about Y axis (fender), angles from +Z towards +X.
module arc_shell(cx,cz,r_in,t,y0,y1,a0,a1){
  translate([cx,y1,cz]) rotate([90,0,0]) linear_extrude(height=y1-y0)
    difference(){
      polygon(concat([[0,0]],[for(a=[a0:2:a1]) [(r_in+t)*sin(a),(r_in+t)*cos(a)]]));
      polygon(concat([[0,0]],[for(a=[a0-1:2:a1+1]) [r_in*sin(a),r_in*cos(a)]]));
    }
}
module label_y(s,p,size=10,face=1,h=0.6){ // text on a +/-Y facing wall
  translate(p) rotate([90,0,face>0?180:0]) linear_extrude(height=h)
    text(s,size=size,halign="center",valign="center",font="Liberation Sans:style=Bold");
}
module label_z(s,p,size=10,rot=0,h=0.6){
  translate(p) rotate([0,0,rot]) linear_extrude(height=h)
    text(s,size=size,halign="center",valign="center",font="Liberation Sans:style=Bold");
}
module label_x(s,p,size=10,h=0.6){ // text on the -X (rear) face, readable from behind
  translate(p) rotate([90,0,-90]) linear_extrude(height=h)
    text(s,size=size,halign="center",valign="center",font="Liberation Sans:style=Bold");
}

// Louvre bank on a +Y-facing wall at y_out: hooded slots, opening faces DOWN.
module louvre_cuts(x0,x1,zs,y_out){
  for(z=zs) translate([x0,y_out-T-1,z-7]) cube([x1-x0,T+2,6]);
}
module louvre_hoods(x0,x1,zs,y_out,d=6){
  for(z=zs) hull(){
    translate([x0,y_out-0.5,z-1]) cube([x1-x0,1,2]);
    translate([x0,y_out+d-1,z-9]) cube([x1-x0,1,1.6]);
  }
}

// ================================================================ REAR POWER POD
// Common to A/B/C (C uses a steeper wedge lid).  Parkside packs X-232.5..-97.5,
// Y67.5..152.5, Z74.5..149.5 (P2-17 placeholder envelope); Ugreen X-158..-112;
// rear electronics X-200..-170 Y+/-60 Z80..200; backboard leans to X-84.7 @Z224.
rp_x_rear = -257;     // inner face of doors
rp_x_front = -100;
rp_y = 240;
function rp_lid_z(x) = concept=="C" ? 200+(x-rp_x_rear)/(rp_x_front-rp_x_rear)*42
                                    : 216+(x-rp_x_rear)/(rp_x_front-rp_x_rear)*8;
con_x0=-264; con_x1=-212; con_y=62;

module rp_lid(){ // one half (+Y).  Lift-off, 2 locating pins + 1 magnet.
  zr=rp_lid_z(rp_x_rear); zf=rp_lid_z(rp_x_front);
  color(c_top) difference(){
    union(){
      plate([[con_x1+2,2,rp_lid_z(con_x1)],[rp_x_front,2,zf],
             [rp_x_front,rp_y,zf],[con_x1+2,rp_y,rp_lid_z(con_x1)]]);
      plate([[rp_x_rear,con_y+2,zr],[con_x1+2,con_y+2,rp_lid_z(con_x1)],
             [con_x1+2,rp_y,rp_lid_z(con_x1)],[rp_x_rear,rp_y,zr]]);
      // drip ridge on front edge (no down-lip: basket backboard is in front)
      plate([[rp_x_front,2,zf+6],[rp_x_front,rp_y,zf+6],[rp_x_front,2,zf],[rp_x_front,rp_y,zf]]);
    }
  }
  color(c_side){ // outer and rear skirts 14 mm
    plate([[rp_x_rear,rp_y-1.1,zr],[rp_x_front,rp_y-1.1,zf],
           [rp_x_front,rp_y-1.1,zf-14],[rp_x_rear,rp_y-1.1,zr-14]]);
    plate([[rp_x_rear+1.1,con_y+2,zr],[rp_x_rear+1.1,rp_y,zr],
           [rp_x_rear+1.1,rp_y,zr-14],[rp_x_rear+1.1,con_y+2,zr-14]]);
    for(y=[90,170]) plate([[rp_x_rear+20,y,zr-1],[rp_x_front-10,y,zf-1],
                           [rp_x_front-10,y,zf-8],[rp_x_rear+20,y,zr-8]],1.6); // ribs
  }
}
module rp_console(){ // fixed centre console: E-stop + main switch + status LED
  z0=58; zf=concept=="C"?262:248; zr=concept=="C"?246:232;
  color(c_side) difference(){
    hull(){
      slab([con_x0,-con_y,z0],[con_x1,con_y,z0+2]);
      translate([con_x1-1,-con_y,zf-1]) cube([1,2*con_y,1]);
      translate([con_x0,-con_y,zr-1]) cube([1,2*con_y,1]);
    }
    hull(){
      slab([con_x0+T,-con_y+T,z0-1],[con_x1-T,con_y-T,z0+1]);
      translate([con_x1-T-1,-con_y+T,zf-T-1]) cube([1,2*con_y-2*T,1]);
      translate([con_x0+T,-con_y+T,zr-T-1]) cube([1,2*con_y-2*T,1]);
    }
    // rear-electronics vent: louvre slots in the rear face, low (open downward)
    for(z=[80:14:120]) translate([con_x0-1,-40,z]) cube([T+2,80,6]);
  }
  color(c_side) for(z=[80:14:120]) hull(){ // hoods over vent slots
    translate([con_x0-0.5,-40,z+6]) cube([1,80,2]);
    translate([con_x0-6,-40,z-1]) cube([1,80,1.6]);
  }
  // E-stop on yellow backing ring, sloped face (rear-up)
  ez=(zf+zr)/2; ex=(con_x0+con_x1)/2; slope=atan((zf-zr)/(con_x1-con_x0));
  translate([ex,0,ez]) rotate([0,-slope,0]) {
    color(c_estop_bg) cylinder(d=62,h=1.4);
    color(c_estop) translate([0,0,1.4]) cylinder(d=22,h=10);
    color(c_estop) translate([0,0,11]) cylinder(d1=40,d2=36,h=12);
  }
  // main key switch + LED on rear face
  color(c_black) translate([con_x0-1,30,175]) rotate([0,-90,0]) cylinder(d=28,h=4);
  color([0.1,0.9,0.3]) translate([con_x0-1,-30,175]) rotate([0,-90,0]) cylinder(d=8,h=3);
  if(show_marks){
    color("white") label_x("STOP",[con_x0-0.7,0,205],9);
    color("white") label_x("PWR",[con_x0-0.7,30,152],7);
  }
}
module rp_door(){ // drop-down battery door (+Y half), hinge on rear crossbar grid
  zt = concept=="C"?rp_lid_z(rp_x_rear)-2:214;
  color(c_side) difference(){
    slab([rp_x_rear-T,con_y+3,60],[rp_x_rear,rp_y,zt]);
    translate([rp_x_rear-T-1,110,zt-22]) cube([T+2,50,10]); // finger pull
  }
  color(c_side) for(y=[110,190]) slab([rp_x_rear,y-0.8,66],[rp_x_rear+7,y+0.8,zt-12]); // ribs
  color([0.5,0.5,0.52]) for(y=[con_y+12,(con_y+rp_y)/2,rp_y-12]) // hinge knuckles
    translate([rp_x_rear-4,y,62]) rotate([90,0,0]) cylinder(d=9,h=18,center=true);
  if(show_marks) color(c_acc) slab([rp_x_rear-T-0.6,con_y+20,118],[rp_x_rear-T,rp_y-20,124]);
}
module rp_door_labels(){ if(show_marks) for(s=[1,-1])
  color(c_top) label_x("BATT",[rp_x_rear-T-0.4,s*(con_y+rp_y)/2,150],16); }
module rp_side(){ // fixed side cheek of the pod (+Y), clamps to corner clamp M4
  zt = concept=="C"?rp_lid_z(rp_x_front):216;
  color(c_side) plate([[rp_x_rear-T,rp_y+1.1,60],[rp_x_front,rp_y+1.1,60],
                       [rp_x_front,rp_y+1.1,rp_lid_z(rp_x_front)-14],
                       [rp_x_rear-T,rp_y+1.1,rp_lid_z(rp_x_rear)-14]]);
}
module rear_pod(){
  for(s=[1,-1]) mirror([0,s>0?0:1,0]) { rp_lid(); rp_door(); if(concept!="C") rp_side(); }
  rp_console(); rp_door_labels();
}

// ================================================================ SIDE PODS
// Controller bay outboard of the basket grid (Y170.5..173.5) and inboard of
// the drive-tyre inner face (Y255.5).  Proposed D5065 controller position:
// vertical, X40..150, Y195..230, Z140..198 (open point for the mechanism agent).
sp_x0 = concept=="A" ? 30 : -30;
sp_x1 = concept=="A" ? 200 : 215;
sp_y_in = 179;  sp_y_out = 244;
sp_z_top_in = concept=="A" ? 250 : 272;
sp_z_top_out = concept=="A" ? 242 : 262;
sp_louvre_low = [100,112,124];
sp_louvre_high= [210,222,234];

module sp_shell(){ // inner wall + sloped top + ends + floor tray (+Y)
  color(c_side){
    slab([sp_x0,sp_y_in,54.5],[sp_x1,sp_y_in+T,sp_z_top_in]);        // inner wall
    slab([sp_x0,sp_y_in,52.5],[sp_x1,sp_y_out-T,54.7]);                // floor tray (0.5 mm pads on rail top)
    for(x=[sp_x0,sp_x1-T]) plate([[x+T/2,sp_y_in,54],[x+T/2,sp_y_out-T,54],
          [x+T/2,sp_y_out-T,sp_z_top_out],[x+T/2,sp_y_in,sp_z_top_in]]);
  }
  color(c_top) plate([[sp_x0,sp_y_in,sp_z_top_in],[sp_x1,sp_y_in,sp_z_top_in],
                      [sp_x1,sp_y_out,sp_z_top_out],[sp_x0,sp_y_out,sp_z_top_out]]);
}
module sp_door(){ // removable outer wall with louvres (+Y) -- 2 thumb screws
  // louvre hoods protrude 6 mm -> only between the tyres (rear tyre X<=20, front X>=215)
  xl0=max(sp_x0+25,40); xl1=min(sp_x1-25,190);
  color(concept=="A"?c_light:c_acc) difference(){
    slab([sp_x0,sp_y_out-T,56],[sp_x1,sp_y_out,sp_z_top_out-2]);
    louvre_cuts(xl0,xl1,sp_louvre_low,sp_y_out);
    louvre_cuts(xl0,xl1,sp_louvre_high,sp_y_out);
  }
  color(concept=="A"?c_light:c_acc){
    louvre_hoods(xl0,xl1,sp_louvre_low,sp_y_out);
    louvre_hoods(xl0,xl1,sp_louvre_high,sp_y_out);
  }
}
module sp_labels(){ if(show_marks && concept!="A") for(s=[1,-1])
  color(c_top) label_y("TR-1",[(sp_x0+sp_x1)/2,s*sp_y_out,165],22,s); }
module controller_proposal(){ // reservation only, not a panel
  color([0.58,0.30,0.78,0.55]) for(s=[1,-1]) translate([95,s*212.5,169]) cube([110,35,58],center=true);
}
module side_pods(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) { sp_shell(); sp_door(); } sp_labels(); }

// ================================================================ C: SIDE SKIN
sk_y_in = 242; sk_z_top = 300;
module skin_panel(x0,x1){
  difference(){
    union(){
      color(c_side) slab([x0,sk_y_in,60],[x1,sk_y_in+T,sk_z_top]);
      color(c_top) plate([[x0,182,sk_z_top+12],[x1,182,sk_z_top+12],
                          [x1,sk_y_in+T,sk_z_top],[x0,sk_y_in+T,sk_z_top]]);
      color(c_side) slab([x0,182,sk_z_top-40],[x1,182+T,sk_z_top+12]);
    }
    for(ax=tyre_axle_x) translate([ax,sk_y_in-5,tyre_axle_z]) rotate([-90,0,0]) cylinder(r=26,h=20);
    louvre_cuts(40,190,sp_louvre_low,sk_y_in+T);
    louvre_cuts(40,190,sp_louvre_high,sk_y_in+T);
  }
  color(c_side) intersection(){
    union(){ louvre_hoods(40,190,sp_louvre_low,sk_y_in+T); louvre_hoods(40,190,sp_louvre_high,sk_y_in+T); }
    slab([x0,0,0],[x1,400,400]);
  }
}
module c_skin(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) {
  skin_panel(-259,-38);  skin_panel(-38,208);
  if(show_marks) color(c_acc) slab([-259,sk_y_in+T,sk_z_top-28],[208,sk_y_in+T+0.6,sk_z_top-18]);
}
  if(show_marks) for(s=[1,-1]) color(c_top) label_y("TR-1",[90,s*(sk_y_in+T),165],26,s);
}

// ================================================================ FENDERS
// Arc shell R97 (12 mm radial to the Ø170 tyre envelope), Y248..336.
fd_y0=248; fd_y1=336;
module fender(ax,front){
  a0 = -70; a1 = 70;
  color(c_top) arc_shell(ax,tyre_axle_z,fender_r_in,T,fd_y0,fd_y1,a0,a1);
  color(c_side) for(y=[fd_y0+1,fd_y1-1]) // edge beads/ribs
    arc_shell(ax,tyre_axle_z,fender_r_in+T,1.6,y-0.8,y+0.8,a0,a1);
  if(front) color(c_side) arc_shell(ax,tyre_axle_z,fender_r_in+T,6,fd_y0,fd_y0+1.6,a0,a1);
}
module fenders(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) {
  fender(tyre_axle_x[0],false); fender(tyre_axle_x[1],true);
}}

// ================================================================ FRONT SHOULDERS
// Above the front fender (Z>=184: tyre top Z170 -> >=14 mm), outboard of
// the X318 arch leg (Y245).  Covers actuator/belt/cable loops from the side.
sh_x0=228; sh_x1=388; sh_y=246;
function sh_ztop() = concept=="C" ? 300 : 258;
module shoulder(){
  color(c_side) plate([[sh_x0,sh_y+1.1,186],[sh_x1,sh_y+1.1,186],
                       [sh_x1-30,sh_y+1.1,sh_ztop()],[sh_x0,sh_y+1.1,sh_ztop()]]);
  color(c_top) plate([[sh_x0,222,sh_ztop()+1],[sh_x1-30,222,sh_ztop()+1],
                      [sh_x1-30,sh_y+T,sh_ztop()],[sh_x0,sh_y+T,sh_ztop()]]);
  if(show_marks) color(c_haz) {
    for(i=[0:3]) intersection(){
      translate([sh_x0+30+i*30,sh_y+T,190]) rotate([0,-35,0]) cube([10,0.8,140]);
      slab([sh_x0+10,sh_y,192],[sh_x1-20,sh_y+5,sh_ztop()-8]);
    }
  }
}
module shoulders(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) shoulder(); }

// ================================================================ ARCH BROW
// Snap cap on the X305..355 / Z230..260 arch crossbar (front 9 mm left bare).  Ends at X346
// (camera FOV ray at Z262 is X362.6).  Mast-post notch X315..345 Y136..164.
module brow(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) color(c_top) difference(){
  union(){
    slab([299.5,2,260.5],[346,238,262.7]);   // top, rests on crossbar via clips
    slab([299.5,2,250],[301.7,200,262.7]);   // rear drip lip, behind crossbar face X305, stops before the arch braces |Y|205..235
  }
  translate([315,136,240]) cube([31.5,28,30]);
}
  if(show_marks) color(c_acc) for(s=[1,-1]) translate([323,s*60,262.7]) linear_extrude(0.6)
    polygon([[-10,-8],[-10,8],[12,0]]); // FRONT arrows
}

// ================================================================ FRONT GUARDS (PROVISIONAL)
// Outboard of the flywheel rim (|Y|229) -> plate at Y241..243.2 (>=12 mm).
// Bottom Z80: a Ø66 ball passes underneath, nothing ploughs.  Inner face is
// modelled vertical (no upward-facing surface towards the nip); a 3..6 deg
// overhang (top inboard) is RECOMMENDED for the detail design so a side-
// ejected ball can only be deflected down, never towards camera/LiDAR.
// Start X430: >=12 mm from the yoke web/trunnion corner at -10 deg.
fg_x0=430; fg_x1=548;
function fg_ztop() = concept=="C" ? 190 : 150;
module front_guard(){
  color(c_side) hull(){
    translate([fg_x0,241,80]) cube([1,T,1]);
    translate([fg_x1-20,241,80]) cube([1,T,1]);
    translate([fg_x1,241,100]) rotate([-90,0,0]) cylinder(r=1,h=T);
    translate([fg_x1-20,241,fg_ztop()-20]) rotate([-90,0,0]) cylinder(r=20,h=T);
    translate([fg_x0,241,fg_ztop()-1]) cube([1,T,1]);
  }
  color(c_side) hull(){ // outward stiffening flange (no inboard lip)
    translate([fg_x0,243,fg_ztop()-2]) cube([fg_x1-fg_x0-30,14,T]);
  }
  if(show_marks){
    color(c_haz) slab([fg_x0+6,243.2,96],[fg_x1-12,243.8,fg_ztop()-26]);
    color(c_black) for(i=[0:3]) intersection(){
      translate([fg_x0+10+i*30,243.6,90]) rotate([0,-35,0]) cube([10,0.6,140]);
      slab([fg_x0+6,243.5,96],[fg_x1-12,244.2,fg_ztop()-26]);
    }
  }
}
module front_guards(){ for(s=[1,-1]) mirror([0,s>0?0:1,0]) front_guard(); }

// ================================================================ concept assembly
module panels(){
  rear_pod();
  if(concept=="C") c_skin(); else side_pods();
  if(concept=="C") for(s=[1,-1]) mirror([0,s>0?0:1,0]) { sp_shell_c(); }
  if(concept!="A"){ fenders(); shoulders(); brow(); front_guards(); }
}
module sp_shell_c(){ // C: inner controller tray wall behind the skin
  color(c_side) slab([-30,sp_y_in,54],[215,sp_y_in+T,200]);
  color(c_side) slab([-30,sp_y_in,52.5],[215,sk_y_in,54.7]);
}

// Piece list -- each one is a separate print, used by QA/BOM.
pieces_A = ["rp_lid","rp_console","rp_door","rp_side","sp_shell","sp_door"];
pieces_B = concat(pieces_A,["fender_rear","fender_front","shoulder","brow","front_guard"]);
pieces_C = ["rp_lid","rp_console","rp_door","skin_rear","skin_mid","sp_shell_c",
            "fender_rear","fender_front","shoulder","brow","front_guard"];
module piece(n){
  if(n=="rp_lid") rp_lid();
  else if(n=="rp_console") rp_console();
  else if(n=="rp_door") rp_door();
  else if(n=="rp_side") rp_side();
  else if(n=="sp_shell") sp_shell();
  else if(n=="sp_door") sp_door();
  else if(n=="sp_shell_c") sp_shell_c();
  else if(n=="skin_rear") skin_panel(-259,-38);
  else if(n=="skin_mid") skin_panel(-38,208);
  else if(n=="fender_rear") fender(tyre_axle_x[0],false);
  else if(n=="fender_front") fender(tyre_axle_x[1],true);
  else if(n=="shoulder") shoulder();
  else if(n=="brow") color(c_top) difference(){
      union(){ slab([299.5,2,260.5],[346,238,262.7]); slab([299.5,2,250],[301.7,200,262.7]); }
      translate([315,136,240]) cube([31.5,28,30]); }
  else if(n=="front_guard") front_guard();
  else assert(false,str("unknown piece ",n));
}
module sided_piece(n,s){ if(n=="rp_console") piece(n); else mirror([0,s>0?0:1,0]) piece(n); }
module all_pieces_qa(){ // panels only, no labels
  pl = concept=="A"?pieces_A:concept=="B"?pieces_B:pieces_C;
  for(n=pl) for(s=[1,-1]) sided_piece(n,s);
}

// ================================================================ context + keep-outs
module context(a){ p253_assembly(a); }

module bp_moving(a){ // everything that moves with / drives the module
  p253_frozen_launcher(a);
  module_pose(a) p244_side_cheeks();
  p253_actuator(a); p253_cam_moving(a); p253_cables(a);
}
module bp_airflow(a){ module_pose(a) for(yy=[-129,129]) d5065_airflow_reservation(yy); }
module bp_tyres(){ drive_tire_envelopes(); }

module ko_launch_cone(L=1600){ // apex at nip, axis at +15 and +20 deg, +/-10 deg, + ball radius
  for(a=[15,17.5,20]) hull(){
    translate(nip(a)) sphere(r=ball_r);
    for(de=[-10,10], sy=[-1,1]){
      e=a+de; d=[cos(e),0,sin(e)];
      translate(nip(a)+L*d+[0,sy*L*tan(10),0]) sphere(r=ball_r);
    }
  }
}
module ko_camera_fov(){ hull(){
  translate(cam_c) sphere(d=8);
  for(p=[[545,-120,33],[545,120,33],[430,-70,33],[430,70,33]]) translate(p) sphere(d=8);
}}
module ko_lidar(){ translate([-400,-450,z_ceiling]) cube([1100,900,60]); }
module ko_ball_path(){
  // intake approach + nip
  hull(){ translate([620,0,ball_r]) sphere(r=ball_r+10); translate(nip(-3)) sphere(r=ball_r+10); }
  hull(){ translate(nip(-3)) sphere(r=ball_r+10); translate([435,0,10+ball_r]) sphere(r=ball_r+10); }
  // ski R75/42 (P2-51 points), Y +/-47 strip width
  pts=[[435,10],[383,10],[369.976,11.139],[357.348,14.523],[345.5,20.048],[332.815,29.264]];
  for(i=[0:len(pts)-2]) hull() for(p=[pts[i],pts[i+1]],y=[-47,47])
    translate([p[0],y,p[1]+ball_r]) sphere(r=ball_r+10);
  // ballistic arc: takeoff 333/62, 38 deg actual, 4.41 m/s (P2-51 realized)
  v=4410; th=38; g=9810;
  for(i=[0:9]) hull() for(k=[i,i+1], y=[-60,60]){ // ends at the backboard (t=0.12 s, X~-84)
    t=k*0.012; translate([332.8-v*cos(th)*t,y,62.3+v*sin(th)*t-g*t*t/2]) sphere(r=ball_r+10);
  }
}
module ko_basket_lift(){ translate([-90,-176,60]) cube([372,352,380]); } // basket + lift-out
module ko_feed_path(){ for(a=[15,20]) hull() for(y=[-40,40]){
  translate([278,y,42+ball_r]) sphere(r=ball_r+10); translate(throat(a)+[0,y,0]) sphere(r=ball_r+10);
}}
module temp_zones(){ color(c_temp){
  translate([330,-240,0]) cube([240,480,200]);       // front cheeks / flywheel
  translate([240,-80,0]) cube([130,160,120]);        // feed port X278/Z42 -> rear nip
  translate([-85,-172,0]) cube([363,344,70]);        // under basket (5d singulator/break-beam)
}}
module keepouts(){
  %color([1,0.2,0.2,0.15]) ko_launch_cone();
  %color([0.1,0.75,0.95,0.20]) ko_camera_fov();
  %color([1,0.85,0.1,0.18]) ko_ball_path();
}

module keepouts_solid(){ // opaque-ish versions usable inside intersection()
  color([1,0.2,0.2,0.35]) ko_launch_cone();
  color([0.1,0.75,0.95,0.35]) ko_camera_fov();
  color([1,0.85,0.1,0.30]) ko_ball_path();
}
// Circumscribed sphere: every facet of the $fn=16 polyhedron is >= qa_probe.
module inflated_panels(){ minkowski(){ all_pieces_qa(); sphere(r=qa_probe/pow(cos(180/16),2),$fn=16); } }
// ================================================================ dispatcher
module scene(a){
  if(show_context) context(a);
  panels();
  if(show_zones) temp_zones();
  if(show_keepouts) keepouts();
}
module ground_marks(){
  color([0.75,0.75,0.75]) translate([150,0,-1.5]) cube([1000,3,3],center=true);
}
module side_annot(a){
  scene(a); ground_marks();
  color("white") translate([560,0,30]) rotate([90,0,0]) linear_extrude(1,center=true)
    text("FRONT +X >",size=18,halign="left",valign="center");
  color("white") translate([-280,0,330]) rotate([90,0,0]) linear_extrude(1,center=true)
    text(str("concept ",concept,"  ",a," deg"),size=18,halign="left");
}

if(bp_view=="assembly") scene(bp_angle);
else if(bp_view=="side") side_annot(bp_angle);
else if(bp_view=="side_section"){ // cut at Y=0, keep +Y half: module tilt visible behind left panels
  intersection(){ union(){ scene(bp_angle); if(show_keepouts) keepouts_solid(); } translate([-500,0,-50]) cube([1200,500,600]); }
  ground_marks();
  color("white") translate([560,0,30]) rotate([90,0,0]) linear_extrude(1,center=true)
    text("FRONT +X >",size=18,halign="left",valign="center");
  color("white") translate([-280,0,330]) rotate([90,0,0]) linear_extrude(1,center=true)
    text(str("concept ",concept,"  section Y=0  ",bp_angle," deg"),size=18,halign="left");
}
else if(bp_view=="panels") panels();
else if(bp_view=="context") context(bp_angle);
else if(bp_view=="piece") sided_piece(bp_piece,bp_side);
else if(bp_view=="zones"){ scene(bp_angle); temp_zones(); controller_proposal(); }
else if(bp_view=="keepouts"){ scene(bp_angle); keepouts(); }
else if(bp_view=="sweep"){ // overlay: panels + module at -10/-3/0/+20
  panels(); p253_fixed_world(); bp_tyres();
  for(a=[-10,-3,0,20]) color([1,0.5,0.1,0.25]) bp_moving(a);
}
// ---- QA exports (each to its own mesh for the distance audit)
else if(bp_view=="mesh_panels") all_pieces_qa();
else if(bp_view=="mesh_piece") sided_piece(bp_piece,bp_side);
else if(bp_view=="mesh_moving") bp_moving(bp_angle);
else if(bp_view=="mesh_airflow") bp_airflow(bp_angle);
else if(bp_view=="mesh_tyres") bp_tyres();
else if(bp_view=="mesh_stops") p253_stops();
else if(bp_view=="mesh_fixed") union(){ p253_fixed_world(); p253_as5600(); p253_stops(); }
else if(bp_view=="qa_cone") intersection(){ all_pieces_qa(); ko_launch_cone(); }
else if(bp_view=="qa_fov") intersection(){ all_pieces_qa(); ko_camera_fov(); }
else if(bp_view=="qa_lidar") intersection(){ all_pieces_qa(); ko_lidar(); }
else if(bp_view=="qa_ball") intersection(){ all_pieces_qa(); ko_ball_path(); }
else if(bp_view=="qa_basket") intersection(){ all_pieces_qa(); ko_basket_lift(); }
else if(bp_view=="qa_feed") intersection(){ all_pieces_qa(); ko_feed_path(); }
else if(bp_view=="qa_fixed") intersection(){ all_pieces_qa(); union(){ p253_fixed_world(); p253_as5600(); p253_stops(); } }
else if(bp_view=="qa_moving_sweep") intersection(){
  if(qa_probe>0) inflated_panels(); else all_pieces_qa();
  union(){ for(a=[-10:1:20]) bp_moving(a); }
}
else if(bp_view=="qa_tyres") intersection(){
  if(qa_probe>0) inflated_panels(); else all_pieces_qa();
  bp_tyres();
}
else scene(bp_angle);

echo("BODY_PANELS_CONCEPT_NO_STL", concept=concept, pivot=pv(), nip_launch15=nip(15),
     nip_launch20=nip(20), throat20=throat(20), tyre_inner_y=tyre_in_y, overall_half_width=tyre_out_y);
