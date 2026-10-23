// P2-90 review views: measured UGREEN, final cradle and declared rear path.
include <rear-pod/params-v2.scad>
use <rear-pod/rear-pod-v2.scad>
use <body-panels/body-panels-f-rear-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

view = "assembly";

module device(x=ugreen_center[0],alpha=0.85) {
  color([0.22,0.25,0.31,alpha])
    translate([x,ugreen_center[1],ugreen_center[2]])
      cube(ugreen_installed_size,center=true);
}

module path_volume() {
  color([0.10,0.78,0.92,0.18]) hull() {
    device(ugreen_service_start_x,0.18);
    device(ugreen_center[0],0.18);
  }
  // +X insertion arrow; removal is the reverse direction.
  color([0.98,0.78,0.10]) {
    translate([-330,ugreen_center[1],ugreen_center[2]+34])
      rotate([0,90,0]) cylinder(d=4,h=72);
    translate([-258,ugreen_center[1],ugreen_center[2]+34])
      rotate([0,90,0]) cylinder(d1=12,d2=0,h=18);
  }
}

if(view=="assembly") {
  assembly_scene(true);
  color([0.96,0.48,0.05]) ugreen_post_holder();
  color([0.95,0.75,0.12]) ugreen_rear_retainer();
  device();
} else {
  color([0.30,0.32,0.35,0.32]) frame_assembly();
  color([0.66,0.52,0.22,0.45]) installed_p279_structure();
  color([0.96,0.48,0.05]) ugreen_post_holder();
  device();
  path_volume();
}
