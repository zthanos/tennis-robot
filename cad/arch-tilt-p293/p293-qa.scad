// P2-93 exact-solid and interface gates. Empty top-level objects are PASS.
use <p293-arch-tilt-release.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/shared-flywheel-p273-bolted-ski.scad>
use <../modular-test-chassis/shared-rear-pod-p270.scad>
use <../body-panels/body-panels-f.scad>
use <../modular-test-chassis/drive-test-control-console.scad>

check = "arch_f";
$fn=40;

module moving_sweep() {
  for(a=[-10:1:20]) union() {
    p253_frozen_launcher(a);
    p253_actuator(a);
    p253_cables(a);
  }
}

module mechanism_moving_sweep() {
  // The old p253_actuator surrogate is replaced by the P2-93 hardware and
  // must not be treated as an additional obstacle to that replacement.
  for(a=[-10:1:20]) union() {
    p253_frozen_launcher(a);
    p253_moving_arm(a);
    p253_tr8_only(a);
    p253_cables(a);
  }
}

module basket_removal_prism() {
  // Existing chassis below Z90 is intentionally outside this overhead gate.
  translate([(-118+225)/2,0,(90+520)/2]) cube([225-(-118),344,520-90],center=true);
}

module basket_p271_frozen() {
  // p271_basket_exact() still carries the superseded P2-51 floor/side tail
  // to X278. P2-71 closes at the frozen front grid X225; clip that stale
  // surrogate here and include the actual P2-71 front grid.
  intersection() {
    p271_basket_exact();
    translate([-250,0,220]) cube([950,500,500],center=true);
  }
  p271_front_grid();
}

module fixed_world_without_arch_tilt() {
  frame_assembly();
  basket_p271_frozen();
  p273_ski_mount_assembly();
  p270_outer_envelope();
}

if(check=="arch_frame_forbidden") intersection() {
  // Mating top surfaces at Z52 have zero volume; any penetration fails.
  arch_release_assembly(); frame_assembly();
}
else if(check=="arch_basket") intersection() { arch_release_assembly(); basket_p271_frozen(); }
else if(check=="arch_basket_removal") intersection() { arch_release_assembly(); basket_removal_prism(); }
else if(check=="arch_ski") intersection() { arch_release_assembly(); p273_ski_mount_assembly(); }
else if(check=="arch_rear_pod") intersection() { arch_release_assembly(); p270_outer_envelope(); }
else if(check=="arch_moving") intersection() { arch_release_assembly(); moving_sweep(); }
else if(check=="arch_f") intersection() { arch_release_assembly(); all_pieces_qa(); }
else if(check=="tilt_f") intersection() { tilt_release_assembly(0); all_pieces_qa(); }
else if(check=="tilt_basket") intersection() { tilt_release_assembly(0); basket_p271_frozen(); }
else if(check=="tilt_ski") intersection() { tilt_release_assembly(0); p273_ski_mount_assembly(); }
else if(check=="tilt_rear_pod") intersection() { tilt_release_assembly(0); p270_outer_envelope(); }
else if(check=="tilt_arch") intersection() { tilt_release_assembly(0); arch_release_assembly(); }
else if(check=="debug_fitplate_arch") intersection() { fit_carrier_installed(); arch_release_assembly(); }
else if(check=="debug_kfl_arch") intersection() { kfl08_installed(); arch_release_assembly(); }
else if(check=="debug_kfl_environment") intersection() {
  kfl08_installed();
  union() { arch_release_assembly(); fit_carrier_installed(); mechanism_moving_sweep(); }
}
else if(check=="debug_carrier_kfl") intersection() {
  fit_carrier_installed(); kfl08_installed();
}
else if(check=="debug_carrier_stops") intersection() {
  fit_carrier_installed(); upper_stops_installed();
}
else if(check=="debug_stop_environment") intersection() {
  upper_stops_installed();
  union() { arch_release_assembly(); fit_carrier_installed(); mechanism_moving_sweep(); }
}
else if(check=="debug_carrier_environment") intersection() {
  fit_carrier_installed();
  union() {
    arch_release_assembly();
    kfl08_installed();
    upper_stops_installed();
    mechanism_moving_sweep();
  }
}
else if(check=="debug_nut_arch") intersection() {
  translate(gpoint(0,[50,-205,90])) anti_backlash_carriage(); arch_release_assembly();
}
else if(check=="debug_stops_arch") intersection() {
  upper_stops_installed();
  arch_release_assembly();
}
else if(check=="debug_sensor_arch") intersection() {
  endpoint_sensor_installed();
  arch_release_assembly();
}
else if(check=="fixed_tilt_vs_moving") intersection() {
  // Fixed carrier/bearing/stops/sensors only. Moving nut carriage is omitted.
  union() {
    fit_carrier_installed();
    kfl08_installed();
    upper_stops_installed();
    translate([387,244,55.84]) as5600_pod();
  }
  mechanism_moving_sweep();
}
else if(check=="debug_carrier_moving") intersection() {
  union() {
    fit_carrier_installed();
    kfl08_installed();
  }
  mechanism_moving_sweep();
}
else if(check=="debug_fitplate_moving") intersection() {
  fit_carrier_installed(); mechanism_moving_sweep();
}
else if(check=="debug_kfl_moving") intersection() {
  kfl08_installed(); mechanism_moving_sweep();
}
else if(check=="debug_stops_moving") intersection() {
  upper_stops_installed();
  mechanism_moving_sweep();
}
else if(check=="debug_sensor_moving") intersection() {
  translate([387,244,55.84]) as5600_pod(); mechanism_moving_sweep();
}
else if(check=="console_f") intersection() {
  all_pieces_qa(); translate([45,195,52]) cube([140,50,62]);
}
else if(check=="hole_axes") intersection() {
  // Existing P2-78 screw-axis probes must pass through no released material.
  union() for(x=[240,300],side=[-1,1],y=[150,160])
    translate([x,side*y,38]) cylinder(d=4,h=70);
  arch_release_assembly();
}
else if(check=="printed_joint_axes") {
  // Material on either Ø4.0 probe means that a segment and its sleeve do
  // not share the same through-M4 axis.  Test one assembled leg joint and
  // one assembled crossbar joint; the mirrored copies are identical.
  union() {
    intersection() {
      union() {
        p0=[318,228,67]; p1=[390,228,245]; ps=lerp3(p0,p1,0.52);
        l0=seg_len(p0,ps); l1=seg_len(ps,p1);
        translate([-l0/2,0,0]) arch_leg_lower();
        translate([ l1/2,0,0]) arch_leg_upper();
        joint_sleeve_x([20,26],48);
      }
      for(x=[-12,12]) translate([x,0,-35]) cylinder(d=4.0,h=70,$fn=24);
    }
    intersection() {
      union() {
        translate([-100,0,0]) arch_crossbar_center();
        translate([64,0,0]) arch_crossbar_end();
        joint_sleeve_x([20,26],48);
      }
      for(x=[-12,12]) translate([x,0,-35]) cylinder(d=4.0,h=70,$fn=24);
    }
  }
}
else if(check=="fastener_tool_paths") {
  // Ø10 x 60 mm driver envelopes.  They start at the installed fastener
  // head plane, so any positive-volume intersection is an obstructed tool
  // approach rather than the intended bore itself.
  union() {
    intersection() {
      arch_release_assembly();
      union() for(x=[240,300],side=[-1,1],y=[150,160])
        translate([x,side*y,63]) cylinder(d=10,h=60,$fn=24);
    }
    intersection() {
      union() {
        p0=[318,228,67]; p1=[390,228,245]; ps=lerp3(p0,p1,0.52);
        l0=seg_len(p0,ps); l1=seg_len(ps,p1);
        translate([-l0/2,0,0]) arch_leg_lower();
        translate([ l1/2,0,0]) arch_leg_upper();
        joint_sleeve_x([20,26],48);
      }
      for(x=[-12,12]) translate([x,0,13]) cylinder(d=10,h=60,$fn=24);
    }
    intersection() {
      union() {
        translate([-100,0,0]) arch_crossbar_center();
        translate([64,0,0]) arch_crossbar_end();
        joint_sleeve_x([20,26],48);
      }
      for(x=[-12,12]) translate([x,0,13]) cylinder(d=10,h=60,$fn=24);
    }
  }
}
else if(check=="arch_install_path") intersection() {
  // Conservative vertical drop: every structural arch member is tested
  // together, even though the real sequence uses the open-top clevises
  // one member at a time.  F is intentionally absent: it is installed last.
  union() for(dz=[20:20:120]) translate([0,0,dz]) arch_release_assembly();
  fixed_world_without_arch_tilt();
}
else if(check=="as5600_install_path") intersection() {
  union() for(dy=[20:20:100])
    translate([0,dy,0]) translate([387,244,55.84]) as5600_pod();
  union() { fixed_world_without_arch_tilt(); arch_release_assembly(); }
}
else assert(false,str("unknown P2-93 QA check: ",check));
