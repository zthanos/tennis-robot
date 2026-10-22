// P2-88 permanent exact-solid tool-access gate.
// A non-empty export is a collision.  Fastener mating volumes are excluded;
// only the external 60 mm driver/wrench approach is tested here.  Hole-axis
// alignment remains in active-chassis-hole-alignment.scad.
use <rear-pod/rear-pod-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

check = "side_frame_front_driver";

module installed_bases_and_frame() {
  translate([0,0,55]) { base_half_v2(-1); base_half_v2(1); }
  frame_assembly();
}

module side_frame_front_driver_collision() {
  intersection() {
    side_frame();
    translate([66.5,0,8]) cylinder(d=6,h=60);
  }
}

module side_frame_rear_wrench_collision() {
  intersection() {
    side_frame();
    translate([-58,0,7.1]) cylinder(d=17.8,h=17.8);
  }
}

module side_frame_front_nut_access_collision() {
  intersection() {
    installed_bases_and_frame();
    for (s=[-1,1])
      // 12 mm nut-driver/socket envelope; fingers are not credited here.
      translate([-132,s*54,-16]) cylinder(d=12,h=60);
  }
}

module side_frame_rear_driver_collision() {
  intersection() {
    installed_bases_and_frame();
    for (s=[-1,1])
      translate([-256.5,s*54,-25]) cylinder(d=6,h=60);
  }
}

module dock_mount_driver_collision() {
  intersection() {
    installed_bases_and_frame();
    for (s=[-1,1],x=[-245,-210])
      translate([x,s*146.5,-14]) cylinder(d=5,h=60);
  }
}

module parkside_lock_driver_collision() {
  intersection() {
    p288_new_parts_installed();
    for (s=[-1,1])
      translate([-333,s*153,76]) rotate([0,90,0]) cylinder(d=5,h=60);
  }
}

module ugreen_driver_collision() {
  intersection() {
    p288_new_parts_installed();
    union() {
      translate([-316.5,70,249]) rotate([0,90,0]) cylinder(d=5,h=45.5);
      translate([-80.5,70,249]) rotate([0,-90,0]) cylinder(d=5,h=45.5);
    }
  }
}

if (check=="side_frame_front_driver") side_frame_front_driver_collision();
else if (check=="side_frame_rear_wrench") side_frame_rear_wrench_collision();
else if (check=="side_frame_front_nut") side_frame_front_nut_access_collision();
else if (check=="side_frame_rear_driver") side_frame_rear_driver_collision();
else if (check=="dock_mount_driver") dock_mount_driver_collision();
else if (check=="parkside_lock_driver") parkside_lock_driver_collision();
else if (check=="ugreen_driver") ugreen_driver_collision();
else union() {
  side_frame_front_driver_collision();
  side_frame_rear_wrench_collision();
  side_frame_front_nut_access_collision();
  side_frame_rear_driver_collision();
  dock_mount_driver_collision();
  parkside_lock_driver_collision();
  ugreen_driver_collision();
}
