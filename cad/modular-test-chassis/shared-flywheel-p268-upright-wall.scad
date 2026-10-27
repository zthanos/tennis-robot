// P2-68 dependent CAD / kinematics only. No STL or manufacturing release.
// Units mm; +X front, +Y left, +Z up.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>
use <shared-flywheel-p267-tilted-arch.scad>

p268_view = "assembly"; // assembly | section | door_sweep | qa_wall_sweep | qa_fixed_channel_sweep | qa_closed_selector_sweep
p268_angle = 0;
p268_wall_x = 225;
p268_wall_t = 4;
p268_wall_half_y = 172;
p268_wall_bottom_z = 46.1; // P2-51 sloping floor evaluated at X225
p268_wall_top_z = 235;
p268_top_edge_r = 2;
p268_door_clear_w = 80;
p268_door_clear_h = 80;
p268_door_hinge_z = p268_wall_bottom_z + p268_door_clear_h;
p268_selector_rail_y = 55; // P2-74 accepted: 3.5 mm nominal rail-to-ski clearance per side
$fn = 48;

function p268_gate_open(a) = a < 14.5 ? 0 : a > 20.5 ? 0 : 95;
function p268_mouth_floor(a) = a <= 15 ? [329.2,26.1]
    : a >= 20 ? [331.5,21.1]
    : [329.2+(a-15)*(331.5-329.2)/5,
       26.1+(a-15)*(21.1-26.1)/5];

module p268_xz_strip(p0,p1,width=80,t=4){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
    rotate([0,-a,0])cube([l+.8,width,t],center=true);
}

module p268_xz_rails(p0,p1,width=80,t=3,h=18){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  for(side=[-1,1])translate([(p0[0]+p1[0])/2,side*(width+t)/2,
                             (p0[1]+p1[1])/2+h/2])
    rotate([0,-a,0])cube([l+.8,t,h],center=true);
}

module p268_xz_rails_at_y(p0,p1,rail_y,t=3,h=16){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  for(side=[-1,1])translate([(p0[0]+p1[0])/2,side*rail_y,
                             (p0[1]+p1[1])/2+h/2])
    rotate([0,-a,0])cube([l+.8,t,h],center=true);
}

module p268_fixed_wall(){
  upper_bottom=p268_door_hinge_z;
  upper_top=p268_wall_top_z-2*p268_top_edge_r;
  side_w=p268_wall_half_y-p268_door_clear_w/2;
  color([.32,.58,.36]){
    // Full-width upper panel and two lower cheeks leave a clear 80x80 port.
    translate([p268_wall_x,0,(upper_bottom+upper_top)/2])
      cube([p268_wall_t,2*p268_wall_half_y,upper_top-upper_bottom],center=true);
    for(side=[-1,1])translate([p268_wall_x,
        side*(p268_door_clear_w/2+side_w/2),
        (p268_wall_bottom_z+p268_door_hinge_z)/2])
      cube([p268_wall_t,side_w,p268_door_clear_h],center=true);
    // R2 full-width top cap: physical top remains exactly Z235.
    translate([p268_wall_x,0,p268_wall_top_z-p268_top_edge_r])
      rotate([90,0,0])cylinder(r=p268_top_edge_r,
                               h=2*p268_wall_half_y,center=true);
  }
}

module p268_feed_door(angle=p268_angle,servo=true){
  opening=p268_gate_open(angle);
  // 84x84 leaf overlaps the nominal 80x80 opening by 2 mm on every edge.
  // Positive rotation folds it rearward into the basket; a spring biases 0°.
  translate([p268_wall_x,0,p268_door_hinge_z])rotate([0,opening,0])
    translate([0,0,-p268_door_clear_h/2])
      color([.95,.52,.08])cube([5,p268_door_clear_w+4,
                                p268_door_clear_h+4],center=true);
  color("silver")translate([p268_wall_x,0,p268_door_hinge_z])
    rotate([90,0,0])cylinder(d=8,h=p268_door_clear_w+16,center=true);
  // Cam/PCA9685 envelope. The linkage is conceptual and fail-closed by spring.
  if(servo){
    color([.20,.20,.24])translate([210,-188,108])cube([42,24,40],center=true);
    color([.75,.75,.18])hull(){
      translate([210,-176,108])sphere(d=5);
      translate([p268_wall_x,-48,p268_door_hinge_z])sphere(d=5);
    }
  }
}

module p268_fixed_feed_channel(){
  // Fixed 51 mm handoff to the retained X278/Z42 interface.
  color([.18,.42,.78]){
    p268_xz_strip([227,p268_wall_bottom_z],[278,42],80,4);
    p268_xz_rails([227,p268_wall_bottom_z],[278,42]);
  }
}

module p268_selector_nose(angle=p268_angle){
  // Only the launch state exposes the selector nose to the moving rear mouth.
  if(angle>=14.5 && angle<=20.5){
    mouth=p268_mouth_floor(angle);
    color([.82,.58,.12]){
      p268_xz_strip([278,42],mouth,72,5);
      // P2-74 changes only these two rails. Their inner faces are Y +/-53.5,
      // 3.5 mm outside the R75/75 ski-wall outer faces at about Y +/-50.
      p268_xz_rails_at_y([278,42],mouth,p268_selector_rail_y,3,16);
    }
  } else {
    // Red shutter marks both-closed/collection-safe state at the channel nose.
    color([.80,.10,.10])translate([278,0,82])cube([5,88,80],center=true);
  }
}

module p268_feed_channel(angle=p268_angle){
  p268_fixed_feed_channel(); p268_selector_nose(angle);
}

module p268_wall_system(angle=p268_angle,servo=true){
  p268_fixed_wall(); p268_feed_door(angle,servo); p268_feed_channel(angle);
}

module p268_assembly(angle=p268_angle){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors(); p251_basket_exact(); p251_feed_exact();
  p267_ski(75); p267_soft_deflector();
  p253_power(); p253_frozen_launcher(angle); p253_actuator(angle);
  p253_as5600(); p253_stops(); p253_cables(angle);
  p268_wall_system(angle,true);
}

module p268_moving_at(angle){
  p253_frozen_launcher(angle); p253_actuator(angle); p253_cables(angle);
}

if(p268_view=="assembly")p268_assembly(p268_angle);
else if(p268_view=="section")projection(cut=true)rotate([90,0,0])p268_assembly(p268_angle);
else if(p268_view=="door_sweep")for(a=[-10:1:20])p268_feed_door(a,false);
else if(p268_view=="qa_wall_sweep")union()for(a=[-10:1:20])intersection(){
  union(){p268_fixed_wall();p268_feed_door(a,false);} p268_moving_at(a);
}
else if(p268_view=="qa_fixed_channel_sweep")intersection(){
  p268_fixed_feed_channel(); union()for(a=[-10:1:20])p268_moving_at(a);
}
else if(p268_view=="qa_closed_selector_sweep")union()for(a=[-10:1:14])intersection(){
  p268_selector_nose(a); p268_moving_at(a);
}
