// P2-90/P2-90b review views: measured UGREEN, rear path and split clamps.
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
  color([0.18,0.72,0.95]) {
    ugreen_clamp_cap_installed(-1); ugreen_clamp_cap_installed(1);
  }
  color([0.95,0.75,0.12]) ugreen_rear_retainer();
  device();
} else if(view=="split_clamps") {
  color([0.62,0.64,0.68,0.52]) {
    panel_post_installed(panel_center_x-58,-1,"rear_negative");
    panel_post_installed(panel_center_x-58,1,"positive");
  }
  color([0.96,0.48,0.05]) ugreen_post_holder();
  color([0.18,0.72,0.95]) {
    ugreen_clamp_cap_installed(-1); ugreen_clamp_cap_installed(1);
    // Exploded copies show the lateral, outside-in installation direction.
    translate([0,-55,0]) ugreen_clamp_cap_installed(-1);
    translate([0,55,0]) ugreen_clamp_cap_installed(1);
  }
  color([0.98,0.78,0.10]) for(s=[-1,1]) {
    py=s>0?118:-82;
    translate([panel_center_x-46,py,241]) rotate([s>0?90:-90,0,0])
      cylinder(d=3.5,h=28);
    translate([panel_center_x-46,s>0?90:-54,241])
      rotate([s>0?90:-90,0,0]) cylinder(d1=10,d2=0,h=10);
  }
  device();
} else {
  color([0.30,0.32,0.35,0.32]) frame_assembly();
  color([0.66,0.52,0.22,0.45]) installed_p279_structure();
  color([0.96,0.48,0.05]) ugreen_post_holder();
  device();
  path_volume();
}
