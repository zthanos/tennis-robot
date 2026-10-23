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

module side_frame_front_head_install_collision() {
  // Front M4 head plus OD14 washer enter laterally into the low pocket
  // before the Allen approaches from above.  The 0.1 mm Z offset avoids
  // counting the intended contact with the top of the foot pad.
  intersection() {
    side_frame();
    hull() {
      translate([66.5,-24,4.1]) cylinder(d=13.6,h=3.8);
      translate([66.5,0,4.1]) cylinder(d=13.6,h=3.8);
    }
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
      // Four split-clamp screws and two removable rear-retainer screws, all
      // reached from the open rear_hi service aperture with a >=60 mm driver.
      for (p=[[75,241],[105,241],[-69.9,241],[-39.9,241]])
        translate([-327,p[0],p[1]]) rotate([0,90,0]) cylinder(d=5,h=60.4);
      translate([-332,3,242]) rotate([0,90,0]) cylinder(d=5,h=62.3);
      translate([-332,65,242]) rotate([0,90,0]) cylinder(d=5,h=62.3);
    }
  }
}

module post_joint_assembly_stage() {
  // Declared sequence: close the four post M4 joints after bases,
  // carrier and side frames, before electronics, docks and panel.
  translate([0,0,55]) { base_half_v2(-1); base_half_v2(1); }
  for (s=[-1,1]) translate([panel_center_x,s*54,64]) side_frame();
  carrier_installed();
  panel_post_installed(panel_center_x-58,-1,"rear_negative");
  panel_post_installed(panel_center_x+58,-1,"front_negative");
  panel_post_installed(panel_center_x-58,1,"positive");
  panel_post_installed(panel_center_x+58,1,"positive");
}

module post_cup_driver_collision() {
  intersection() {
    post_joint_assembly_stage();
    union() {
      for (s=[-1,1]) {
        // Rear cup: head on -X outer face, 60 mm Allen approach from rear.
        translate([-340.0,s*54,154]) rotate([0,90,0]) cylinder(d=6,h=59.0);
        // Front cup: head on +X outer face, 60 mm Allen approach from front.
        translate([-116.0,s*54,154]) rotate([0,90,0]) cylinder(d=6,h=59.5);
      }
    }
  }
}

module post_cup_nyloc_wrench_collision(which="both") {
  intersection() {
    post_joint_assembly_stage();
    union() {
      for (s=[-1,1]) {
        // Six-millimetre-thick open-ended wrench envelopes approach each
        // inner nyloc from the outward Y side; fingers are not credited.
        if (which=="both" || which=="rear")
          translate([-228.5,s*84,154]) cube([6,60,12],center=true);
        if (which=="both" || which=="front")
          translate([-168.5,s*84,154]) cube([6,60,12],center=true);
      }
    }
  }
}

if (check=="side_frame_front_driver") side_frame_front_driver_collision();
else if (check=="side_frame_front_head_install") side_frame_front_head_install_collision();
else if (check=="side_frame_rear_wrench") side_frame_rear_wrench_collision();
else if (check=="side_frame_front_nut") side_frame_front_nut_access_collision();
else if (check=="side_frame_rear_driver") side_frame_rear_driver_collision();
else if (check=="dock_mount_driver") dock_mount_driver_collision();
else if (check=="parkside_lock_driver") parkside_lock_driver_collision();
else if (check=="ugreen_driver") ugreen_driver_collision();
else if (check=="post_cup_driver") post_cup_driver_collision();
else if (check=="post_cup_nyloc_wrench") post_cup_nyloc_wrench_collision();
else union() {
  side_frame_front_driver_collision();
  side_frame_front_head_install_collision();
  side_frame_rear_wrench_collision();
  side_frame_front_nut_access_collision();
  side_frame_rear_driver_collision();
  dock_mount_driver_collision();
  parkside_lock_driver_collision();
  ugreen_driver_collision();
  post_cup_driver_collision();
  post_cup_nyloc_wrench_collision();
}
