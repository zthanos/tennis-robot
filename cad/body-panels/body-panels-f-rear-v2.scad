// P2-86 — selected concept F rear panels on rear pod v2.
// Release-ready CAD, but NOT APPROVED FOR PRINTING.
// +X front, +Y left, +Z up. Select printable/QA geometry with `part`.

use <body-panels-f.scad>
use <body-panels-f-support.scad>
use <../rear-pod/rear-pod-v2.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad>

part = "assembly";
show_context = true;
door_angle = 0;
$fn = 72;

icr_x = 117;
f_inner_r = 424.5;
f_t = 2.5;
f_outer_r = f_inner_r + f_t;
f_y_end = 244.5;             // P2-87 side-wall seam after the R60 rear corner
f_z0 = 60;
f_z_split0 = 198;
f_z_split1 = 201;
f_z_top = 350;
m3_clear_f = 3.4;
m4_clear_f = 4.5;

function rear_x(y,r=f_inner_r) = icr_x-sqrt(r*r-y*y);
hinge_x = rear_x(164)-3;
hinge_y = 165;                // 0.5 mm positive union to the fixed frame

module side_copy(side=1) { if(side>0) children(); else mirror([0,1,0]) children(); }
module x_cylinder(p,d,h=30) { translate(p) rotate([0,90,0]) cylinder(d=d,h=h,center=true); }
module y_cylinder(p,d,h=30) { translate(p) rotate([90,0,0]) cylinder(d=d,h=h,center=true); }

module half_annulus_2d(r0=f_inner_r,r1=f_outer_r) {
  intersection(){
    difference(){ translate([icr_x,0]) circle(r=r1); translate([icr_x,0]) circle(r=r0); }
    // 0.3 mm installed centre seam; the hidden lap closes it without
    // coplanar solid ambiguity and permits independent service removal.
    translate([-400,0.15]) square([190,f_y_end-0.15]);
  }
}

module curved_patch(z0,z1,y0=0,y1=f_y_end,r0=f_inner_r,r1=f_outer_r) {
  intersection(){
    translate([0,0,z0]) linear_extrude(z1-z0) half_annulus_2d(r0,r1);
    translate([-400,y0,z0-1]) cube([190,y1-y0,z1-z0+2]);
  }
}

// Inward T-ribs: 12 mm total section in the open rear field, 8.5 mm near
// the pod corner. The ribs are clipped away from Parkside and retain 3 mm.
module shell_ribs(z0,z1) {
  for(y=[18,52,100,145,205,237]) let(depth=y<125?9:y<160?5.5:10)
    curved_patch(z0,z1,y-2.5,y+2.5,f_inner_r-depth,f_inner_r+0.4);
  for(z=[z0+8,(z0+z1)/2,z1-8])
    curved_patch(z-2.5,z+2.5,0,151,f_inner_r-8,f_inner_r+0.4);
  for(z=[z0+8,(z0+z1)/2,z1-8])
    curved_patch(z-2.5,z+2.5,174,f_y_end,f_inner_r-10,f_inner_r+0.4);
}

// Positive half owns the inside lap. The negative half is relieved by its
// mirrored geometry, leaving clearance at the Y=0 service seam.
module center_lap(z0,z1,hole_zs=[]) {
  difference(){
    union(){
      // Cross-seam part sits 0.3 mm inside the opposite outer skin.  The
      // second block exists only on +Y and bridges it positively to its owner.
      translate([rear_x(0)+0.3,-5,z0]) cube([4.2,17,z1-z0]);
      translate([rear_x(0)-1,0,z0]) cube([6,12,z1-z0]);
    }
    for(z=hole_zs) x_cylinder([rear_x(0)+1,-0.2,z],m3_clear_f,14);
  }
}
module seam_relief_for_mirrored_half(z0,z1) {
  translate([rear_x(0)-0.3,-0.25,z0-1]) cube([15,5.5,z1-z0+2]);
}

module base_land(y) {
  difference(){
    union(){
      hull(){
        translate([rear_x(y)-1,y-6,y>100?47.5:48]) cube([5,12,y>100?10:14]);
        translate([-272,y-6,47.5]) cube([5,12,10]);
      }
      // The Y145 land sits under the drawer. Route its print connection
      // below Z58 to Y164, then rise outside the Y161 drawer boundary.
      if(y>100) hull(){
        translate([rear_x(y)-1,y-6,47.5]) cube([5,12,10]);
        translate([rear_x(164)-1,155.5,47.5]) cube([5,6,10]);
      }
      if(y>100) translate([rear_x(164)-1,161.4,47.5]) cube([5,8.6,17]);
    }
    x_cylinder([rear_x(y)-2,y,52.5],m3_clear_f,52);
  }
}

module corner_seam_flange(z0,z1) {
  difference(){
    union(){
      curved_patch(z0,z1,231,f_y_end,f_inner_r-7,f_outer_r);
      translate([-237,238,z0]) cube([14,6.5,z1-z0]);
    }
    for(z=[80,176]) y_cylinder([-230,242,z],m4_clear_f,16);
    // P2-87 side datum: two M3 through holes on a 12 mm-wide future land.
    for(z=[112,152]) y_cylinder([-230,242,z],m3_clear_f,16);
  }
}

module rear_lo_positive_raw(with_lap=true) {
  difference(){
    union(){
      difference(){
        curved_patch(f_z0,f_z_split0);
        if(!with_lap) seam_relief_for_mirrored_half(f_z0,f_z_split0);
      }
      shell_ribs(f_z0,f_z_split0);
      base_land(45); base_land(145);
      corner_seam_flange(f_z0,f_z_split0);
      if(with_lap) center_lap(78,182,[105,155]);
      for(zz=[[62,74],[90,110],[140,150]])
        translate([hinge_x,hinge_y,zz[0]]) cylinder(d=8,h=zz[1]-zz[0]);
      for(y=[30,142]) curved_patch(190,200,y-6,y+6,f_inner_r,f_inner_r+7);
      curved_patch(96,114,56,68,f_inner_r-4,f_outer_r);
    }
    translate([-330,68,59]) cube([90,93.5,87]);
    for(zz=[[61,75],[89,111],[139,151]])
      translate([hinge_x,hinge_y,zz[0]]) cylinder(d=m4_clear_f,h=zz[1]-zz[0]);
    for(y=[30,142]) curved_patch(193,201,y-4.3,y+4.3,f_inner_r+1.7,f_inner_r+5.8);
    x_cylinder([rear_x(62)-1,62,105],m4_clear_f,18);
  }
}

module rear_hi_positive_raw(with_lap=true) {
  difference(){
    union(){
      difference(){
        difference(){
          curved_patch(f_z_split1,347);
          if(!with_lap) seam_relief_for_mirrored_half(f_z_split1,347);
        }
        translate([-330,10,217]) cube([90,150,114]);
      }
      intersection(){
        curved_patch(214,334,7,163);
        union(){
          for(k=[-7:12]) hull(){
            translate([-292,-20+k*20,214]) cube([70,4,4],center=true);
            translate([-292,50+k*20,334]) cube([70,4,4],center=true);
          }
          for(k=[-4:15]) hull(){
            translate([-292,-20+k*20,334]) cube([70,4,4],center=true);
            translate([-292,50+k*20,214]) cube([70,4,4],center=true);
          }
        }
      }
      shell_ribs(206,343);
      corner_seam_flange(f_z_split1,347);
      if(with_lap) center_lap(218,334,[250,305]);
      for(y=[30,142]) curved_patch(194,205,y-4,y+4,f_inner_r+2,f_inner_r+5.5);
      // Two independent structural ties per half to the control-panel beam.
      for(y=[20,85]) hull(){
        translate([rear_x(y)-1,y-6,329]) cube([5,12,10]);
        translate([-277,y-6,329]) cube([10,12,10]);
      }
    }
    for(y=[20,85]) x_cylinder([-276,y,334],m3_clear_f,72);
  }
}

module deck_positive_raw(with_lap=true) {
  difference(){
    union(){
      // 4 mm deck with 12 mm total L-section depth (8 mm below its underside),
      // not a flat cantilever.
      intersection(){
        translate([0,0,347]) linear_extrude(4)
          difference(){ translate([icr_x,0]) circle(r=f_outer_r); translate([icr_x,0]) circle(r=387.5); }
        translate([-325,0,346]) cube([65,f_y_end,6]);
      }
      curved_patch(339,351,0,f_y_end,f_inner_r-5,f_inner_r+0.5);
      for(y=[20,85]) translate([-277,y-6,339]) cube([10,12,12]);
      // Integral under-lap crosses Y=0 and supports the opposite half.
      if(with_lap) translate([rear_x(0)+0.3,-5,342])
        cube([-268-(rear_x(0)+0.3),19,5]);
    }
    for(y=[20,85]) x_cylinder([-272,y,344],m3_clear_f,24);
    translate([-299,with_lap?-0.2:0.2,340]) cylinder(d=m3_clear_f,h=14);
  }
}

module parkside_door_positive_raw() {
  difference(){
    union(){
      curved_patch(63,143,71,158,f_inner_r+0.4,f_outer_r-0.4);
      for(zz=[[75,89],[111,139]]) union(){
        translate([hinge_x,hinge_y,zz[0]]) cylinder(d=8,h=zz[1]-zz[0]);
        translate([hinge_x-4,157,zz[0]]) cube([8,8,zz[1]-zz[0]]);
      }
      translate([rear_x(82)-4,82,96]) rotate([0,90,0])
        difference(){ cylinder(d=16,h=5); cylinder(d=7,h=7); }
    }
    for(zz=[[74,90],[110,140]]) translate([hinge_x,hinge_y,zz[0]])
      cylinder(d=m4_clear_f,h=zz[1]-zz[0]);
  }
}

module door_turn_button() {
  difference(){
    hull(){ for(y=[-11,11]) translate([0,y,0]) cylinder(d=12,h=5); }
    translate([0,0,-1]) cylinder(d=m4_clear_f,h=7);
  }
}

// Structural corner stay reuses the top and side M4 of the rear corner clamp.
module corner_stay_positive() {
  difference(){
    union(){
      // 0.2 mm washer/shim datum above the crossbar top.  The side ear is
      // wholly behind X-190, never inside the 50 mm rear crossbar.
      translate([-196,170.5,52.2]) cube([43,14,6.8]);
      translate([-196,170.5,31]) cube([5.8,14,28]);
      hull(){
        translate([-189.8,172,53]) cube([25.8,11,10]);
        translate([-235,233,58]) cube([12,8,10]);
      }
      translate([-236,233,58]) cube([12,8,136]);
      for(z=[80,176]) translate([-236,233,z-7]) cube([12,9,14]);
    }
    translate([-165,177.5,50]) cylinder(d=m4_clear_f,h=18);
    x_cylinder([-193,177.5,37],m4_clear_f,16);
    for(z=[80,176]) y_cylinder([-230,239,z],m4_clear_f,20);
  }
}

module rear_lo(side=1) {
  if(side>0) rear_lo_positive_raw(true);
  else mirror([0,1,0]) difference(){
    rear_lo_positive_raw(false);
    seam_relief_for_mirrored_half(77,183);
  }
}
module rear_hi(side=1) {
  if(side>0) rear_hi_positive_raw(true);
  else mirror([0,1,0]) difference(){
    rear_hi_positive_raw(false);
    seam_relief_for_mirrored_half(217,335);
  }
}
module deck_rear(side=1) {
  if(side>0) deck_positive_raw(true);
  else mirror([0,1,0]) difference(){
    deck_positive_raw(false);
    translate([-312,-0.25,341.7]) cube([46,5.5,5.6]);
  }
}
module corner_stay(side=1) { side_copy(side) corner_stay_positive(); }
module parkside_door(side=1,angle=0) {
  side_copy(side) translate([hinge_x,hinge_y,0]) rotate([0,0,angle])
    translate([-hinge_x,-hinge_y,0]) parkside_door_positive_raw();
}
module turn_button_installed(side=1,open=false) {
  side_copy(side) translate([rear_x(62)-4.3,62,105]) rotate([0,90,0])
    rotate([0,0,open?90:0]) door_turn_button();
}

module all_fixed_f_rear(){
  for(s=[-1,1]) { rear_lo(s); rear_hi(s); deck_rear(s); corner_stay(s); }
}
module all_f_rear(angle=0){ all_fixed_f_rear(); for(s=[-1,1]) parkside_door(s,angle); }
module drawer_sweep(){ for(s=[-1,1]) translate([-364,s*115.5,98.5]) cube([472,91,81],center=true); }
module moving_sweep(){ for(a=[-10:2:20]) bp_moving(a); }
module p286_pod(){ installed_p279_structure(); }

module rounded_keepout(origin,size,r=3){ minkowski(){ translate(origin) cube(size); sphere(r=r,$fn=12); } }
module pod_clearance_3(){
  translate([0,0,48]) linear_extrude(14) offset(delta=3) p276_footprint_2d();
  rounded_keepout([-263,-82,61],[129,164,286]);
  rounded_keepout([-258,-157,125],[126,252,162]);
  for(s=[-1,1]) rounded_keepout([-269,s*115.5-45.5,58],[141,91,81]);
  rounded_keepout([-255.5,16,232],[121,52,52]);
  rounded_keepout([-261,-37,64],[98,74,46]);
  rounded_keepout([-267.5,-95,327],[138,190,24]);
}

module f_skin_without_interfaces(){
  difference(){
    for(s=[-1,1]) { rear_lo(s); rear_hi(s); deck_rear(s); }
    translate([-330,-250,45]) cube([75,500,20]);
    translate([-290,-105,325]) cube([35,210,30]);
    for(s=[-1,1]) translate([-250,s>0?226:-246,50]) cube([35,20,155]);
  }
  for(s=[-1,1]) parkside_door(s,0);
}

module interface_fasteners(){
  for(s=[-1,1],y=[45,145]) color("silver") x_cylinder([rear_x(y)-1,s*y,52.5],3,42);
  for(s=[-1,1],y=[20,85],z=[334,344]) color("silver") x_cylinder([-273,s*y,z],3,18);
  for(s=[-1,1],z=[80,176]) color("silver") y_cylinder([-230,s*242,z],4,16);
  for(s=[-1,1]) color("silver") translate([hinge_x,s*hinge_y,58]) cylinder(d=4,h=96);
}
module interface_head_envelopes(){
  for(s=[-1,1],y=[45,145]) x_cylinder([rear_x(y)-5,s*y,52.5],7,4);
  for(s=[-1,1],y=[20,85],z=[334,344]) x_cylinder([-283,s*y,z],6,4);
  for(s=[-1,1],z=[80,176]) y_cylinder([-230,s*247,z],8,5);
  for(s=[-1,1]) x_cylinder([rear_x(62)-6,s*62,105],8,5);
}

module print_orient(){ rotate([0,-90,0]) children(); }
module corner_print_orient(){ rotate([90,0,0]) children(); }
module assembly_scene(service=false){
  if(show_context){
    color([0.15,0.36,0.62,0.55]) p286_pod();
    color([0.35,0.35,0.38,0.24]) frame_assembly();
    color([0.18,0.18,0.20,0.14]) drive_package_envelopes();
  }
  color([0.12,0.12,0.13]) for(s=[-1,1]) rear_lo(s);
  if(!service) color([0.74,0.40,0.27]) for(s=[-1,1]) rear_hi(s);
  color([0.74,0.40,0.27]) for(s=[-1,1]) deck_rear(s);
  color([0.10,0.35,0.48]) for(s=[-1,1]) corner_stay(s);
  color([0.20,0.20,0.22]) for(s=[-1,1]) parkside_door(s,service?110:door_angle);
  color([0.96,0.42,0.05]) for(s=[-1,1]) turn_button_installed(s,service);
  interface_fasteners();
}

if(part=="rear_lo_left") print_orient() rear_lo(1);
else if(part=="rear_lo_right") print_orient() rear_lo(-1);
else if(part=="rear_hi_left") print_orient() rear_hi(1);
else if(part=="rear_hi_right") print_orient() rear_hi(-1);
else if(part=="deck_rear_left") print_orient() deck_rear(1);
else if(part=="deck_rear_right") print_orient() deck_rear(-1);
else if(part=="parkside_door_left") print_orient() parkside_door(1,0);
else if(part=="parkside_door_right") print_orient() parkside_door(-1,0);
else if(part=="corner_stay_left") corner_print_orient() corner_stay(1);
else if(part=="corner_stay_right") corner_print_orient() corner_stay(-1);
else if(part=="door_turn_button") door_turn_button();
else if(part=="qa_skin_pod_3mm") intersection(){ f_skin_without_interfaces(); pod_clearance_3(); }
else if(part=="qa_chassis") intersection(){ all_f_rear(); frame_assembly(); }
else if(part=="qa_wheels") intersection(){ all_f_rear(); drive_package_envelopes(); }
else if(part=="qa_moving") intersection(){ all_f_rear(); moving_sweep(); }
else if(part=="qa_drawer_fixed") intersection(){ all_fixed_f_rear(); drawer_sweep(); }
else if(part=="qa_drawer_open") intersection(){
  union(){ for(s=[-1,1]) { parkside_door(s,110); turn_button_installed(s,true); } interface_head_envelopes(); } drawer_sweep();
}
else if(part=="qa_ground") intersection(){ all_f_rear(); translate([-400,-300,-100]) cube([800,600,117]); }
else if(part=="qa_panel_seams") intersection(){
  union(){ rear_lo(1); rear_hi(1); deck_rear(1); }
  union(){ rear_lo(-1); rear_hi(-1); deck_rear(-1); }
}
else if(part=="service") assembly_scene(true);
else assembly_scene(false);
