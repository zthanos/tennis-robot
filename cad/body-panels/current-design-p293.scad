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

view = "hero";          // hero | section | rear_service
angle = 20;             // moving-module operating angle
service = false;
colorway = "terracotta";
show_context = false;   // suppress nested legacy context in P2-86b scene
front_on = true;
cam_fwd = true;
$fn = 48;

module exact_current_context(a) {
  color([0.34,0.36,0.39]) frame_assembly();
  drive_package_envelopes();
  color([0.92,0.63,0.14]) rear_pod_v2_assembly();
  color([0.28,0.67,0.36]) {
    p271_basket_exact();
    p271_front_system(a,true);
  }
  color([0.08,0.62,0.82]) p273_ski_mount_assembly();
  color([0.10,0.42,0.86]) arch_release_assembly();
  color([0.95,0.43,0.08]) tilt_release_assembly(a);
}

module moving_module_concept(a) {
  color([0.92,0.48,0.08]) p253_frozen_launcher(a);
}

module current_robot(a=angle,service_open=service) {
  exact_current_context(a);
  moving_module_concept(a);
  f_side_front_concept();
  assembly_scene(service_open);
}

module side_section(a) {
  intersection(){
    current_robot(a,false);
    translate([-500,0,-50]) cube([1200,500,650]);
  }
}

if(view=="section") side_section(angle);
else current_robot(angle,view=="rear_service" || service);
