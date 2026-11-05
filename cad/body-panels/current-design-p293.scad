// Current P2-93 report scene. Render-only composition; no STL release.
// Exact/released context: P2-78 chassis, rear pod v2/P2-90d, P2-93 arch and
// tilt parts, P2-85 ski and P2-86b rear F panels. Concept context: P2-40
// moving launcher pending wheel/D5065 measurements and F side/front panels.

use <body-panels-f.scad>
use <body-panels-f-rear-v2.scad>
use <../rear-pod/rear-pod-v2.scad>
use <../arch-tilt-p293/p293-arch-tilt-release.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/shared-flywheel-p273-bolted-ski.scad>
use <../modular-test-chassis/p277-torsion-node.scad>

view = "hero";          // hero | section | rear_service
angle = 20;             // moving-module operating angle
service = false;
colorway = "terracotta";
show_context = false;   // suppress nested legacy context in P2-86b scene
front_on = true;
cam_fwd = true;
$fn = 48;

module p271_basket_current() {
  // Assembly definition of P2-71: replace the stale X278 tail at X225.
  intersection() {
    p271_basket_exact();
    translate([-250,0,220]) cube([950,500,500],center=true);
  }
}

module exact_current_context(a) {
  color([0.34,0.36,0.39]) frame_assembly();
  drive_package_envelopes();
  color([0.92,0.63,0.14]) rear_pod_v2_assembly();
  color([0.92,0.63,0.14]) {
    for(side=[-1,1]) {
      parkside_dock(side);
      parkside_drawer(side);
      rear_foot_wedge_installed(side);
      ugreen_clamp_cap_installed(side);
    }
    ugreen_post_holder();
    ugreen_rear_retainer();
  }
  color([0.18,0.45,0.72])
    translate([112.5,195,37]) rotate([90,0,0]) mpu6050_rail_side_mount();
  color([0.28,0.67,0.36]) p271_basket_current();
  color([0.28,0.67,0.36]) p271_front_system(a,true);
  color([0.08,0.62,0.82]) p273_ski_mount_assembly();
  color([0.10,0.42,0.86]) arch_release_assembly();
  color([0.95,0.43,0.08]) tilt_release_assembly(a);
  // Pod local Y0..10; cover begins at its outer Y254 face.
  color([0.95,0.43,0.08]) translate([387,254,55.84]) as5600_cover();
  fit0186_installed_hardware();
}

// Hardware only: uses the carrier's actual +30 motor bore and installed
// transform, on the -Y actuator side. Local +Z points towards +X/+Z.
// The 17 mm gearbox station follows the carrier ring; its length is a
// provisional envelope pending hardware measurement. Motor can: D37 x 70.
module fit0186_installed_hardware() {
  multmatrix([
    [-0.150,-0.132,0.980,295.7],
    [ 0.660,-0.740,0.000,-157.2],
    [ 0.737, 0.647,0.200,140.1],
    [ 0,     0,    0,    1]
  ]) translate([30,0,0]) union() {
    color([0.65,0.65,0.68]) cylinder(d=37,h=17);
    color([0.12,0.13,0.15]) translate([0,0,17]) cylinder(d=37,h=70);
  }
}

module moving_module_concept(a) {
  color([0.92,0.48,0.08]) p253_frozen_launcher(a);
}

module current_robot(a=angle,service_open=service) {
  exact_current_context(a);
  moving_module_concept(a);
  f_side_front_concept();
  assembly_scene(service_open,false);
}

module side_section(a) {
  intersection(){
    current_robot(a,false);
    translate([-500,0,-50]) cube([1200,500,650]);
  }
}

if(view=="section") side_section(angle);
else current_robot(angle,view=="rear_service" || service);
