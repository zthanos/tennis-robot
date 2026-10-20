// BODY CONCEPT "F" -- planter tub (direction from the user, 2026-10-17):
// sides hide the basket, not car-like, wheels outside with no fenders,
// object language inspired by a lattice planter in a dark collar with a
// diagonal top edge (h3li0 "povoa" was the reference image -- lines only,
// not a copy).  Concept only: no STL, no print release.
// Context = full-robot-v3.scad defaults, read via `use`; nothing modified.
// Units mm.  +X front, +Y left, +Z up.
//
//   * Plan: one pill outline -- straight sides Y+/-244.5 (11 mm inside the
//     drive-tyre inner face Y255.5), rear end = arc R424.5 about the in-place
//     turn centre X117 (just wraps the P2-70 pod corners at R421.4, stays
//     inside the P2-70 R425 envelope), R60 plan fillets.
//   * Front: OPEN.  The flywheel/ski/arch is the "face"; side walls end on a
//     plane parallel to the tilted P2-67 arch leg, 25 mm behind its axis.
//   * Collar (dark, solid): Z60 up to a diagonal line Z350 (rear) -> Z185
//     (front end).  Lattice (diagonal diamond grid, +/-60 deg bars, prints
//     without supports) from the line up to the Z320 rim -> hides the basket
//     and ventilates the side controller bays.  The collar/lattice seam is
//     the print split.
//   * Decks: side rim Z320 (Y180..wall) closes the controller bays from
//     above; between pod and wall the deck follows the diagonal; the P2-70
//     top panel (E-stop etc.) stays flush at Z350.

use <body-panels-f-support.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p267-tilted-arch.scad>
use <../modular-test-chassis/shared-flywheel-p268-upright-wall.scad>
use <../modular-test-chassis/shared-rear-pod-p270.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>

colorway = "terracotta";  // [terracotta, bone]
bp_view = "assembly";
bp_angle = 0;
bp_piece = "";
bp_side = 1;
show_context = true;
show_turn_circle = false;
crown_on = false;         // REJECTED by the user (module-mounted ring); kept for reference only
front_on = true;          // tub continuation in front of the arch legs (user direction)
cam_fwd = true;           // PROPOSAL: camera X410 -> X470 on a short bracket from the P2-67 crossbar
crown_top = 109;          // local W of the crown top (plate top = 47, D5065 top = 112)
qa_probe = 0;
$fn = 64;

T = 2.5;
icr = [117,0];
r_tub = 424.5;           // rear arc (outer)
y_wall = 244.5;          // outer face of the side walls
z_bot = 60; z_rim = 320; z_top = 350;
x_arc = -230;            // collar starts to fall from Z350 here
z_front = 185;           // collar line at the front end
x_front_top = 393;
function leg_x(z) = 318+(z-52)*(390-318)/(245-52);
function x_end(z) = leg_x(z)-25;          // wall end plane (parallel to the arch leg)
function zc(x) = x<=x_arc ? z_top : z_top-(x-x_arc)*(z_top-z_front)/(x_front_top-x_arc);
function zdeck(x) = max(z_rim, zc(x));

c_collar = [0.11,0.11,0.12];
c_lat = colorway=="bone" ? [0.90,0.87,0.80] : [0.76,0.42,0.29];

module slab(a,b){ translate([min(a[0],b[0]),min(a[1],b[1]),min(a[2],b[2])])
  cube([abs(b[0]-a[0]),abs(b[1]-a[1]),abs(b[2]-a[2])]); }
module xz_extrude(){ rotate([90,0,0]) linear_extrude(1000,center=true) children(); } // 2D (x,z) -> 3D, all Y

// ---------------------------------------------------------------- plan
module plan(inset=0){
  offset(r=60-inset) offset(delta=-60)
    intersection(){
      translate(icr) circle(r=r_tub,$fn=360);
      translate([-500,-y_wall]) square([1100,2*y_wall]);
    }
}
module end_cut(){ // keep everything behind the slanted end plane
  xz_extrude() polygon([[-600,-50],[x_end(-50),-50],[x_end(500),500],[-600,500]]);
}
module below_line(dz=0){ xz_extrude() polygon([[-600,-50],[-600,z_top+dz],[x_arc,z_top+dz],
  [x_front_top,z_front+dz],[700,zc(700)+dz],[700,-50]]); }
module below_deck(dz=0){ xz_extrude() polygon([[-600,-50],[-600,z_top+dz],[x_arc,z_top+dz],
  [x_arc+(z_top-z_rim)*(x_front_top-x_arc)/(z_top-z_front),z_rim+dz],[700,z_rim+dz],[700,-50]]); }

module wall_raw(){
  intersection(){
    translate([0,0,z_bot]) linear_extrude(z_top-z_bot) difference(){ plan(0); plan(T); }
    end_cut(); below_deck();
  }
}
// axle holes (drive axles rotate: >=10 mm radial) and P2-70 rear openings
module wall_cuts(){
  for(ax=[-65,300], s=[-1,1]) translate([ax,s*y_wall,85]) rotate([90,0,0]) cylinder(r=14,h=30,center=true);
  for(s=[-1,1]) translate([-330,s*115.5-45.5,58]) cube([60,91,81]);          // Parkside drawers -X
}
// ---------------------------------------------------------------- collar / lattice
module collar(){ difference(){ intersection(){ wall_raw(); below_line(); } wall_cuts(); rear_grille_cut(); } }
module rear_band(){ slab([-600,-158,138],[-240,158,222]); }   // grille window over the P2-70 louvres/G2 hoods

lat_p = 20; lat_w = 3.6; lat_a = 60;
module lattice2d(x0,x1,z0,z1){
  intersection(){
    translate([x0,z0]) square([x1-x0,z1-z0]);
    union(){
      for(k=[-40:80]) translate([x0+k*lat_p,z0]) rotate(lat_a-90) translate([-lat_w/2,-50]) square([lat_w,600]);
      for(k=[-40:80]) translate([x0+k*lat_p,z0]) rotate(90-lat_a) translate([-lat_w/2,-50]) square([lat_w,600]);
    }
  }
}
module lattice(){
  difference(){
    intersection(){ wall_raw(); difference(){ below_deck(1); below_line(); } }
    // open the diamonds: everything that is not bar / frame
    difference(){
      slab([-600,-400,-10],[700,400,500]);
      xz_extrude() lattice2d(-200,500,150,330);           // bars
      below_line(4);                                       // 4 mm frame above the collar line
      difference(){ slab([-600,-400,z_rim-4],[700,400,500]); }   // 4 mm top rim
      end_cut_frame();                                     // 6 mm frame along the end edge
    }
  }
}
module end_cut_frame(){ difference(){ slab([-600,-400,-10],[700,400,500]);
  xz_extrude() polygon([[-600,-50],[x_end(-50)-6,-50],[x_end(500)-6,500],[-600,500]]); } }

// rear grille over the P2-70 louvres / G2 hoods (curved rear wall, bars in YZ)
module rear_grille_cut(){
  intersection(){
    slab([-340,-154,142],[-240,154,218]);
    difference(){
      slab([-340,-154,142],[-240,154,218]);
      rotate([90,0,90]) linear_extrude(1000,center=true) lattice2d(-160,160,140,218);
    }
  }
}

// ---------------------------------------------------------------- decks
module side_deck(){ // rim deck Z320 (slight outward fall) + sloped deck along the diagonal
  for(s=[1,-1]) mirror([0,s>0?0:1,0]) intersection(){
    union(){
      // sloped part between pod side Y167 and wall (X-230..-116), follows the collar line
      // over the pod (inner edge 0.5 mm off the P2-70 side Y167) ...
      hull(){
        translate([x_arc,167.5,z_top-T]) cube([1,y_wall-167.5,T]);
        translate([-126,167.5,zc(-126)-T]) cube([1,y_wall-167.5,T]);
      }
      // ... then over the basket rear (inner edge Y180, clear of grid/basket volume)
      hull(){
        translate([-126,180,zc(-126)-T]) cube([1,y_wall-180,T]);
        translate([x_arc+(z_top-z_rim)*(x_front_top-x_arc)/(z_top-z_front),180,z_rim-T]) cube([1,y_wall-180,T]);
      }
      // rim deck from the basket rear to the end, inner edge Y180 (5 mm to the
      // P2-67 deflector rail Y175)
      hull(){
        translate([-125,180,z_rim-T]) cube([1,1,T]);
        translate([-125,y_wall-1,z_rim-T-1.5]) cube([1,1,T]);
        translate([x_end(z_rim),180,z_rim-T]) cube([1,1,T]);
        translate([x_end(z_rim),y_wall-1,z_rim-T-1.5]) cube([1,1,T]);
      }
    }
    translate([0,0,-1]) linear_extrude(500) plan(T/2);
    end_cut();
  }
}
module rear_deck(){ // Z350 ring between the tub wall and the P2-70 pod (pod top stays as is)
  intersection(){
    translate([0,0,z_top-T]) linear_extrude(T) difference(){
      plan(T/2);
      translate([-272.5,-167.5]) square([400,335]);
    }
    slab([-600,-400,0],[x_arc,400,500]);
  }
}
// G2 filter liner (non-printed foam/spacer-fabric sheet, ~5 mm) behind the
// side lattice: hides the basket completely (dark interior, like the
// planter), lets the controller bays breathe, stops clay dust and the spray
// of the drive tyres that run right next to the wall.
module liner(){
  intersection(){
    translate([0,0,z_bot]) linear_extrude(z_top-z_bot) difference(){ plan(T+0.5); plan(T+5.5); }
    difference(){ below_deck(-T); below_line(-2); }
    end_cut();
    slab([-125,-400,0],[700,400,500]);
  }
}
module drawer_faces(){ // pill drawer faces flush with the curved rear (P2-70 drawers extended)
  for(s=[-1,1]) intersection(){
    translate([0,s*115.5,98.5]) rotate([90,0,90]) linear_extrude(1000,center=true)
      offset(r=16) offset(delta=-16) square([86,76],center=true);
    translate([0,0,0]) linear_extrude(400) difference(){ plan(0.5); plan(T+0.5); }
  }
}

// ---------------------------------------------------------------- D5065 CROWN
// User direction: the terracotta lattice also crosses the front and hides the
// D5065 motors, starting at the top of the module's upper plate, kept short so
// it never limits the tilt.  It is MOUNTED ON THE MODULE (clips over the 8 mm
// plate side edges, no holes, motor mounts untouched) and tilts with it, so it
// has no relative motion to the motors/plate.  Module-local frame: X=u (along
// the module), Y=v, Z=w; motors at u55, v+/-129.  Plan = pill around both
// motors; inner radius 40 > the 33 mm D5065 airflow reservation; open lattice
// + open top keep the convection path (P2-28 thermal note).
cr_cu = 55; cr_cv = 129; cr_ri = 40; cr_t = 2.5; cr_w0 = 47.5;
cr_rm = cr_ri+cr_t/2;
module cr_stadium(r){ hull(){ for(s=[-1,1]) translate([cr_cu,s*cr_cv]) circle(r=r,$fn=96); } }
module cr_band(){ translate([0,0,cr_w0]) linear_extrude(crown_top-cr_w0) difference(){ cr_stadium(cr_ri+cr_t); cr_stadium(cr_ri); } }
module cr_bar(p,phi,sg){ L=(crown_top-cr_w0)/cos(30)+20;
  translate([p[0],p[1],(cr_w0+crown_top)/2]) rotate([0,0,phi]) rotate([sg*30,0,0]) cube([cr_t+2,lat_w,L],center=true); }
module cr_bars(){
  pitch=lat_p;
  for(sg=[-1,1]){
    for(v=[-cr_cv:pitch:cr_cv]) { cr_bar([cr_cu+cr_rm,v],0,sg); cr_bar([cr_cu-cr_rm,v],180,sg); }
    for(e=[1,-1]) for(ph=[0:pitch/cr_rm*180/PI:180]) let(phi=e>0?ph:180+ph)
      cr_bar([cr_cu+cr_rm*cos(phi),e*cr_cv+cr_rm*sin(phi)],phi,sg);
  }
}
module crown_local(){
  intersection(){ cr_band(); union(){
    cr_bars();
    translate([0,0,crown_top-4]) linear_extrude(4) square(1000,center=true);   // top rim
    translate([0,0,cr_w0]) linear_extrude(5) square(1000,center=true);          // foot rim
  } }
  // two clip hooks per side over the plate edges (plate v+/-157, w39..47)
  for(s=[-1,1], u=[30,80]) translate([u,s*157,0]) {
    translate([0,s*3.3,42.85]) cube([12,6,9.3],center=true);     // 0.3 mm off the plate edge
    translate([0,s*1.8,37.7]) cube([12,9,1.4],center=true);      // lip 0.3 mm under the plate
  }
}
module crown(a){ module_pose(a) crown_local(); }
// camera FOV minus what the module's own upper plate already hides: keep only
// the part of the FOV hull above the line camera -> plate front edge (u170,w47).
module fov_visible(a){
  e = gpoint(a,[170,0,47]);
  intersection(){ ko_camera_fov_f();
    xz_extrude() polygon([[cam_x(),cam_z()],[e[0]+(e[0]-cam_x())*3,e[2]+(e[2]-cam_z())*3],[900,900],[cam_x()-50,900]]); }
}
// launch cone with its axis at the CURRENT module angle (paired check)
module ko_cone_at(a,L=1600){ n=gpoint(a,[55,0,0]); hull(){ translate(n) sphere(r=33);
  for(de=[-10,10], sy=[-1,1]){ e=a+de; translate(n+L*[cos(e),0,sin(e)]+[0,sy*L*tan(10),0]) sphere(r=33); } } }
cam_old = [410,0,305]; cam_new = [500,0,300];   // camera integrated in the front face, hung from the X-beam of the LiDAR mast (Z344)
function cam_x()= cam_fwd ? cam_new[0] : cam_old[0]; function cam_z()= cam_fwd ? cam_new[2] : cam_old[2];
module cam_box(c){ translate(c) rotate([0,50,0]) cube([82,32,28],center=true); }
module ko_camera_fov_f(){ dx = cam_x()-410; hull(){ translate([cam_x(),0,cam_z()]) sphere(d=8);
  for(p=[[605,-120,33],[605,120,33],[490,-70,33],[490,70,33]]) translate(p+[dx,0,0]) sphere(d=8); } }
module camera_proposal(){
  color([0.08,0.09,0.11]) cam_box(cam_new);
  color([0.20,0.50,0.76]) hull(){ translate([cam_new[0]-22,0,343]) cube([30,24,2],center=true); translate([cam_new[0]-22,0,334]) cube([12,24,2],center=true); } // hanger from the mast X-beam
}

// ---------------------------------------------------------------- FRONT CONTINUATION
// The tub continues AFTER the blue P2-67 arch legs so the body reads as one
// object.  Fixed to the chassis (not the module).  Plan: straight sides
// Y244.5, then the same R424.5 turn-circle arc as the rear, then a short
// straight front at X506.  Height: bottom at the module-plate level (Z105,
// above the yoke web that tilts) rising to Z155; top = the collar diagonal
// continued (Z~195 at the leg -> 166) and a slim rim across the front.
// Limits that set the front rim (all angles -10..+20):
//   below: launch cone (+20: ~Z142 at X506) and the plate at +20 (~Z144)
//   above: camera line of sight over the plate edge (0 deg: Z170 at X506)
//   behind: D5065 airflow reservation at -10 (front reaches X~496)
fb_xf = 511;   // inner face X508.5: >=10 mm from the D5065 airflow reservation at -10 deg
// bottom: Z105 along the sides; above the flywheel discs' front (they reach
// |Y|229 and rise to ~Z127 near X500 at +20 deg) the corners start at Z140;
// the front rim (|Y|<=170) at Z155.
function fb_zb(x) = x<=455 ? 115 : x<=470 ? 115+25*(x-455)/15 : x<=500 ? 140 : x>=506 ? 158 : 140+18*(x-500)/6;   // Z158: >=10 mm over the plate at +20
function fb_zt(x) = z_rim;   // same Z320 rim as the rear tub (user direction); camera integrated in the front face
module fb_outline(){ intersection(){ translate(icr) circle(r=r_tub,$fn=720); translate([0,-y_wall]) square([fb_xf,2*y_wall]); } }
module fb_band_xz(dz0=0,dz1=0){ xz_extrude() polygon(concat(
  [for(x=[300:4:700]) [x,fb_zb(x)+dz0]], [for(x=[700:-4:300]) [x,fb_zt(x)-dz1]])); }
module fb_after_leg(d=18){ xz_extrude() polygon([[leg_x(-50)+d,-50],[900,-50],[900,600],[leg_x(600)+d,600]]); }
module fb_wall(){ intersection(){
  linear_extrude(400) difference(){ fb_outline(); offset(delta=-T) fb_outline(); translate([-10,-300]) square([310,600]); }
  fb_band_xz(); fb_after_leg(); } }
module fb_lattice_keep(){ union(){
  xz_extrude() lattice2d(300,480,90,210);
  fb_band_xz(0,0) ; } }
module front_band(){
  // same logic as the rear tub: solid dark collar below the diagonal line,
  // diamond lattice above it up to the Z320 rim; camera "eye" cut out.
  difference(){
    fb_wall();
    cam_eye();
    intersection(){
      difference(){ slab([0,-400,0],[900,400,500]); below_line(); }      // lattice only above the diagonal
      difference(){ slab([0,-400,0],[900,400,500]);
        intersection(){ xz_extrude() lattice2d(300,520,90,330); slab([0,-400,0],[470,400,500]); }
        intersection(){ rotate([90,0,90]) linear_extrude(2000,center=true) lattice2d(-260,260,90,330); slab([470,-400,0],[900,400,500]); }
        below_line(4);                                                      // 4 mm frame above the diagonal
        difference(){ slab([0,-400,0],[900,400,500]); fb_band_xz(4,4); }   // 4 mm top/bottom frame
        difference(){ slab([0,-400,0],[900,400,500]); fb_after_leg(24); } // 6 mm frame at the leg
      }
    }
  }
}
// two slim hangers from the P2-67 arch crossbar front face (X415) to the
// front rim at Y+/-175 (outside the D5065 airflow reservation and the camera FOV)
module fb_hangers(){ for(s=[-1,1]) hull(){
  translate([417,s*172,236]) cube([4,8,10],center=true);     // clip on the crossbar front face X415
  translate([498,s*172,fb_zt(498)-2]) cube([4,8,4],center=true); } }    // lands on the corner arc, inside R424.5
// opening = camera body (82x32x28 at 50 deg) + 3 mm all round, pushed through
// the front face along the lens direction; the lens sits flush in the face.
module cam_eye(){
  translate(cam_new) rotate([0,50,0]) hull(){ cube([88,38,34],center=true); translate([60,0,0]) cube([88,38,34],center=true); }
  // + the FOV mouth where it crosses the face (a few mm below the body)
  intersection(){ minkowski(){ ko_camera_fov_f(); sphere(r=3,$fn=8); } slab([480,-60,200],[530,60,330]); }
}
module front_continuation(){ front_band(); }

// ---------------------------------------------------------------- assembly
module tub(){
  color(c_collar) difference(){ collar(); rear_band(); }
  color(c_lat) intersection(){ collar(); rear_band(); }
  color(c_lat) difference(){ lattice(); wall_cuts(); }
  color([0.07,0.07,0.08]) liner();
  color(c_lat) side_deck();
  color(c_lat) rear_deck();
  color([0.18,0.18,0.19]) drawer_faces();
}
module tub_solid(){ // geometry only (QA): collar + lattice + decks + drawer faces + front continuation
  collar(); difference(){ lattice(); wall_cuts(); } side_deck(); rear_deck(); drawer_faces();
  if(front_on) front_continuation();
}

// print pieces: L/R by mirror; X splits; collar | lattice at the diagonal seam;
// collar split at Z200 where it is taller than 250 mm.
xs = [-400,-150,40,230,500];
pieces = ["rear_lo","rear_hi","c1_lo","c1_hi","c2","c3","l1","l2","l3","deck_rear","drawer_face","deck_s1","deck_s2","deck_s3"];
module half(){ intersection(){ children(); slab([-600,0,-10],[700,400,500]); } }
module piece(n){
  if(n=="rear_lo") half() intersection(){ collar(); slab([-600,-400,0],[xs[1],400,200]); }
  else if(n=="rear_hi") half() intersection(){ collar(); slab([-600,-400,200],[xs[1],400,500]); }
  else if(n=="c1_lo") half() intersection(){ collar(); slab([xs[1],-400,0],[xs[2],400,200]); }
  else if(n=="c1_hi") half() intersection(){ collar(); slab([xs[1],-400,200],[xs[2],400,500]); }
  else if(n=="c2") half() intersection(){ collar(); slab([xs[2],-400,0],[xs[3],400,500]); }
  else if(n=="c3") half() intersection(){ collar(); slab([xs[3],-400,0],[xs[4],400,500]); }
  else if(n=="l1") half() intersection(){ difference(){ lattice(); wall_cuts(); } slab([-126,-400,0],[xs[2],400,500]); }
  else if(n=="l2") half() intersection(){ difference(){ lattice(); wall_cuts(); } slab([xs[2],-400,0],[xs[3],400,500]); }
  else if(n=="l3") half() intersection(){ difference(){ lattice(); wall_cuts(); } slab([xs[3],-400,0],[xs[4],400,500]); }
  else if(n=="deck_rear") half() rear_deck();
  else if(n=="drawer_face") half() drawer_faces();
  else if(n=="deck_s1") half() intersection(){ side_deck(); slab([-400,-400,0],[-20,400,500]); }
  else if(n=="deck_s2") half() intersection(){ side_deck(); slab([-20,-400,0],[190,400,500]); }
  else if(n=="deck_s3") half() intersection(){ side_deck(); slab([190,-400,0],[500,400,500]); }
  else assert(false,str("unknown piece ",n));
}
module sided_piece(n,s){ mirror([0,s>0?0:1,0]) piece(n); }
module all_pieces_qa(){ tub_solid(); }

module context_f(a){
  if(cam_fwd) { difference(){ context(a); translate(cam_old) rotate([0,50,0]) cube([84,34,30],center=true); } camera_proposal(); }
  else context(a);
  p270_shell(); p270_controls(); }
module inflated_panels(){ minkowski(){ all_pieces_qa(); sphere(r=qa_probe/pow(cos(180/16),2),$fn=16); } }
module keep_circle_f(){ translate([icr[0],icr[1],-10]) cylinder(r=r_tub+0.01,h=500,$fn=720); }
module turn_mark(){ color("white") translate([icr[0],0,1]) difference(){ cylinder(r=426.5,h=1,$fn=240); translate([0,0,-1]) cylinder(r=423.5,h=3,$fn=240);} }
module scene(a){ if(show_context) context_f(a); tub();
  if(front_on) { color(c_collar) intersection(){ front_continuation(); below_line(); }
                 color(c_lat) difference(){ front_continuation(); below_line(); } }
  if(crown_on) color(c_lat) crown(a); if(show_turn_circle) turn_mark(); }
module annot(t){
  color([0.75,0.75,0.75]) translate([150,0,-1.5]) cube([1000,3,3],center=true);
  color("white") translate([560,0,30]) rotate([90,0,0]) linear_extrude(1,center=true) text("FRONT +X >",size=18,valign="center");
  color("white") translate([-300,0,430]) rotate([90,0,0]) linear_extrude(1,center=true) text(t,size=18);
}

if(bp_view=="assembly") scene(bp_angle);
else if(bp_view=="side"){ scene(bp_angle); annot(str("F planter  ",bp_angle," deg")); }
else if(bp_view=="side_section"){ intersection(){ scene(bp_angle); translate([-500,0,-50]) cube([1200,500,600]); } annot(str("F planter  section Y=0  ",bp_angle," deg")); }
else if(bp_view=="panels") tub();
else if(bp_view=="mesh_piece") sided_piece(bp_piece,bp_side);
else if(bp_view=="mesh_panels") all_pieces_qa();
else if(bp_view=="qa_moving_sweep") intersection(){ if(qa_probe>0) inflated_panels(); else all_pieces_qa(); union(){ for(a=[-10:1:20]) bp_moving(a); } }
else if(bp_view=="qa_tyres") intersection(){ if(qa_probe>0) inflated_panels(); else all_pieces_qa(); drive_tire_envelopes(); }
else if(bp_view=="qa_fixed") intersection(){ all_pieces_qa(); union(){ bp_fixed(); p270_outer_envelope(); } }
else if(bp_view=="qa_pod_contents") intersection(){ all_pieces_qa(); ko_pod_contents(); }
else if(bp_view=="qa_cone") intersection(){ all_pieces_qa(); ko_launch_cone(); }
else if(bp_view=="qa_fov") intersection(){ all_pieces_qa(); ko_camera_fov(); }
else if(bp_view=="qa_lidar") intersection(){ all_pieces_qa(); ko_lidar(); }
else if(bp_view=="qa_ball") intersection(){ all_pieces_qa(); ko_ball_path(); }
else if(bp_view=="qa_feed") intersection(){ all_pieces_qa(); ko_feed_path(); }
else if(bp_view=="qa_turn") difference(){ all_pieces_qa(); keep_circle_f(); }
else if(bp_view=="qa_console") intersection(){ all_pieces_qa(); translate([45,195,52]) cube([140,50,62]); }
else if(bp_view=="mesh_front") front_continuation();
else if(bp_view=="qa_front_fov") union(){ for(a=[-10:1:20]) intersection(){ front_continuation();
  difference(){ fov_visible(a); translate(cam_new) rotate([0,50,0]) cube([88,38,34],center=true); } } }   // FOV in front of the lens
else if(bp_view=="qa_front_fov_op") union(){ for(a=[-3:1:20]) intersection(){ front_continuation(); fov_visible(a); } }
else if(bp_view=="qa_front_cone") union(){ for(a=[15:1:20]) intersection(){ front_continuation(); ko_cone_at(a); } }
else if(bp_view=="qa_cam_moving") union(){ for(a=[-10:1:20]) intersection(){ camera_proposal(); union(){ bp_moving(a); bp_airflow(a); } } }
else if(bp_view=="qa_cam_fixed") intersection(){ cam_box(cam_new); union(){ tub_solid();
  difference(){ bp_fixed(); translate(cam_old) rotate([0,50,0]) cube([84,34,30],center=true); } } }  // old camera removed
else if(bp_view=="qa_cam_fixed_nobracket") intersection(){ color() cam_box(cam_new); union(){ tub_solid();
  difference(){ bp_fixed(); translate(cam_old) rotate([0,50,0]) cube([84,34,30],center=true); } } }
else if(bp_view=="qa_cam_lidar") intersection(){ camera_proposal(); ko_lidar(); }
else if(bp_view=="qa_front_inflated10") intersection(){ minkowski(){ front_continuation(); sphere(r=10/pow(cos(180/16),2),$fn=16); }
  union(){ for(a=[-10:1:20]) union(){ bp_moving(a); } } }
else if(bp_view=="mesh_cam") camera_proposal();
else if(bp_view=="mesh_crown") crown(bp_angle);
else if(bp_view=="crown_local") crown_local();
else if(bp_view=="qa_crown_fixed") union(){ for(a=[-10:1:20]) intersection(){ crown(a);
  union(){ bp_fixed(); p267_sensors(); p267_arch(); p253_actuator(a); p253_cables(a); tub_solid(); } } }
else if(bp_view=="qa_crown_module") intersection(){ crown(0); union(){ bp_airflow(0); p253_frozen_launcher(0); } }
else if(bp_view=="qa_crown_fov") union(){ for(a=[-10:1:20]) intersection(){ crown(a); fov_visible(a); } }
else if(bp_view=="qa_crown_cone") union(){ for(a=[15:1:20]) intersection(){ crown(a); ko_cone_at(a); } }
else if(bp_view=="qa_crown_lidar") union(){ for(a=[-10:5:20]) intersection(){ crown(a); ko_lidar(); } }
else scene(bp_angle);
