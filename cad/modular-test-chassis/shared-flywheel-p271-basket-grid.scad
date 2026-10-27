// P2-71 simplified basket walls -- CAD/geometry only, no STL release.
// Units mm; +X front, +Y left, +Z up.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>
use <shared-flywheel-p267-tilted-arch.scad>
use <shared-flywheel-p268-upright-wall.scad>
use <shared-rear-pod-p270.scad>

p271_view = "assembly"; // assembly | section | qa_front_flight | qa_rear_pod
p271_angle = 0;
p271_front_x = 225;
p271_front_t = 4;
p271_front_half_y = 172;
p271_front_bottom_z = 46.1;
p271_front_top_z = 235;
p271_door_half_y = 42;       // physical 84 mm leaf; clear opening is 80 mm
p271_door_top_z = 128.1;     // physical leaf includes 2 mm overlap
p271_rear_x = -118;
p271_rear_t = 4;
p271_rear_half_y = 140;
p271_rear_top_z = 320;
p271_grid_rib = 4;
$fn=36;

// Explicit accepted datums. OpenSCAD `use` imports modules/functions but not
// these caller-scope variables; duplicating them here prevents undef geometry
// without changing P2-51/P2-68.
p251_rear_x=-85; p251_front_x=278; p251_half_y=172;
p251_floor_rear_z=70; p251_floor_front_z=42;
p268_wall_x=225; p268_wall_bottom_z=46.1;
p268_door_clear_w=80; p268_door_clear_h=80;
p268_door_hinge_z=p268_wall_bottom_z+p268_door_clear_h;

function p271_floor_z(x)=70+(x+85)*(42-70)/(278+85);
p271_rear_bottom_z=p271_floor_z(p271_rear_x);

module p271_xz_strip(p0,p1,width=280,t=4){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
    rotate([0,-a,0])cube([l+.8,width,t],center=true);
}

module p271_front_grid(){
  color([.32,.58,.36]) {
    // Upper field. Nine bays across 340 mm and three vertically: every clear
    // opening remains below 40 mm with 4 mm ribs.
    upper_z0=p268_door_hinge_z;
    upper_z1=p271_front_top_z;
    for(i=[0:9]) {
      y=-170+i*(340/9);
      translate([p271_front_x,y,(upper_z0+upper_z1)/2])
        cube([p271_front_t,p271_grid_rib,upper_z1-upper_z0],center=true);
    }
    for(i=[0:3]) {
      z=upper_z0+2+i*((upper_z1-upper_z0-4)/3);
      translate([p271_front_x,0,z])
        cube([p271_front_t,2*p271_front_half_y,p271_grid_rib],center=true);
    }

    // Two lower side fields surround the solid fail-closed door.
    for(side=[-1,1]) {
      for(i=[0:4]) {
        y=side*(p271_door_half_y+i*32);
        translate([p271_front_x,y,(p271_front_bottom_z+p268_door_hinge_z)/2])
          cube([p271_front_t,p271_grid_rib,
                p268_door_hinge_z-p271_front_bottom_z],center=true);
      }
      for(z=[48.1,86.1,124.1])
        translate([p271_front_x,side*(p271_door_half_y+65),z])
          cube([p271_front_t,130,p271_grid_rib],center=true);
    }
  }
}

module p271_rear_grid(){
  color([.38,.58,.40]) {
    // Seven bays across 276 mm; clear width about 35.4 mm.
    for(i=[0:7]) {
      y=-138+i*(276/7);
      translate([p271_rear_x,y,(p271_rear_bottom_z+p271_rear_top_z)/2])
        cube([p271_rear_t,p271_grid_rib,
              p271_rear_top_z-p271_rear_bottom_z],center=true);
    }
    // Seven vertical bays; clear height about 30.8 mm.
    for(i=[0:7]) {
      z=p271_rear_bottom_z+2+i*((p271_rear_top_z-p271_rear_bottom_z-4)/7);
      translate([p271_rear_x,0,z])
        cube([p271_rear_t,2*p271_rear_half_y,p271_grid_rib],center=true);
    }
  }
}

module p271_rear_floor_and_side_extension(){
  // Close only the newly gained X-118..-85 rear extension. The accepted
  // P2-51 floor and side grids remain unchanged forward of X=-85.
  color([.48,.70,.50,.75])
    p271_xz_strip([p271_rear_x,p271_rear_bottom_z],[-85,70],280,4);
  color([.35,.62,.38]) for(side=[-1,1]) {
    // Continue the accepted side grid rather than closing the extension.
    for(x=[-116,-93])
      translate([x,side*172,(p271_rear_bottom_z+320)/2])
        cube([4,3,320-p271_rear_bottom_z],center=true);
    for(z=[82:46:312])
      translate([(p271_rear_x-85)/2,side*172,z])
        cube([-85-p271_rear_x,3,7],center=true);
  }
}

module p271_basket_exact(){
  // P2-51 basket without the old 12-degree backboard.
  color([.48,.70,.50,.75])
    xz_strip_p253([p251_rear_x,p251_floor_rear_z],
                   [p251_front_x,p251_floor_front_z],280,4);
  for(y=[-p251_half_y,p251_half_y])p251_grid_side(y);
  for(y=[-156,156])color([.38,.58,.40])
    translate([p251_front_x,y,190])cube([4,32,260],center=true);
  p271_rear_floor_and_side_extension();
  p271_rear_grid();
}

module p271_front_system(angle=p271_angle,servo=true){
  p271_front_grid();
  p268_feed_door(angle,servo);       // only solid wall element
  p268_feed_channel(angle);
}

module p271_assembly(angle=p271_angle){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors(); p271_basket_exact(); p251_feed_exact();
  p267_ski(75); p267_soft_deflector(); d5065_controller_envelopes();
  p253_frozen_launcher(angle); p253_actuator(angle);
  p253_as5600(); p253_stops(); p253_cables(angle);
  p271_front_system(angle,true); p270_rear_pod(true,true);
}

if(p271_view=="assembly")p271_assembly(p271_angle);
else if(p271_view=="section")projection(cut=true)rotate([90,0,0])p271_assembly(p271_angle);
else if(p271_view=="qa_rear_pod")intersection(){
  p271_rear_grid(); p270_outer_envelope();
}
