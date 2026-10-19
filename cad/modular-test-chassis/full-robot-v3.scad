// Whole-robot view on the REAL v3 chassis (2026-10-16).
// The P2-52/53 transition assembly draws a simplified frame envelope
// (chassis_structure in the Phase-1 file, crossbars to the rail centreline).
// This file swaps that envelope for the printed v3 frame (frame_assembly:
// rails, corners, P2-78 socket/sleeve crossbars and paired short side braces)
// and keeps every
// other P2-51..P2-54 modules unchanged. No STL; dependent P2-68 clearance
// checks live in shared-flywheel-p268-upright-wall.scad.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>
use <drive-test-control-console.scad>
use <shared-flywheel-p267-tilted-arch.scad>
use <shared-flywheel-p268-upright-wall.scad>
use <shared-flywheel-p273-bolted-ski.scad>
use <shared-rear-pod-p270.scad>
use <shared-flywheel-p271-basket-grid.scad>

angle = 0;            // flywheel module tilt: -3 collect, 0 transport, +20 launch
show_console = true;  // temporary drive-test button console on the left rail
show_basket = true;
p267_enabled = true;  // P2-67 inclined arch / overhead soft deflector concept
p267_exit_angle_full_robot = 75; // 75 or 80; R75 only
p268_enabled = true;  // upright wall/door + P2-74 selector rails at Y +/-55
p270_enabled = true;  // P2-70 rear pod packaging proposal; no manufacturing release
p270_show_shell = true; // false gives a layout cutaway for review renders
p271_enabled = true;  // simplified <=40 mm basket grids; P2-69 traces only

frame_assembly();
drive_package_envelopes();
pivot_supports();
if (p267_enabled) { p267_arch(); p267_sensors(); }
else { front_arch_and_triangulation(); sensors_and_mounts(); }
if (show_basket) {
    if (p271_enabled) p271_basket_exact(); else p251_basket_exact();
    if (p267_enabled) p273_ski_mount_assembly(); else p251_ski_exact();
    p251_feed_exact();
}
if (!p270_enabled) p253_power(); else d5065_controller_envelopes();
p253_frozen_launcher(angle);
p253_actuator(angle);
p253_as5600();
p253_stops();
p253_cables(angle);
if (p267_enabled) p267_soft_deflector();
if (p271_enabled) p271_front_system(angle,true);
else if (p268_enabled) p268_wall_system(angle,true);
else if (p267_enabled) p267_gate(angle,true); // rejected P2-67 wall, history only
if (show_console) translate([45, 220, 52]) color([0.9,0.9,0.2]) console();
if (p270_enabled) p270_rear_pod(true,p270_show_shell);
