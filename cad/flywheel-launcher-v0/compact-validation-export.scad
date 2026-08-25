// Deterministic per-component export for compact CAD <-> URDF validation.
// This is analysis-only: it calls the authoritative study modules and never
// redefines their geometry. Example:
//   openscad -D 'part="bridge"' -o /tmp/bridge.stl compact-validation-export.scad

use <compact-packaging-study.scad>
use <launcher-envelope.scad>
use <../collector-intake-v1/option-a/option-a.scad>
use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/hood.scad>
use <compact-basket-support.scad>
use <compact-parked-reliefs.scad>
include <params.scad>

part = "bridge";
functional_shift_x = -100;
launcher_origin = [560, 0, 0];

module shifted(child_part="") {
    translate([functional_shift_x, 0, 0]) children();
}

// Physical launcher solids only. launcher_cradle() also draws its explicitly
// labelled exit-guide envelope, so call the source wheel/plate modules here
// when auditing manufactured-solid interference.
module physical_launcher() {
    shifted() translate(launcher_origin)
        translate([0, 0, 215])
            rotate([0, -pitch_deg, 0])
                rotate([90, 0, 0])
                    translate([0, 0, -path_z]) {
                        wheel_envelope(lower_wheel_z);
                        wheel_envelope(upper_wheel_z);
                        side_plate(-1);
                        side_plate(1);
                    }
}

module collect_basket() {
    shifted() { compact_relieved_bin(); compact_fixed_hood(); }
}

module raised_basket() {
    shifted() {
        translate([0, 0, 100]) compact_relieved_bin();
        compact_fixed_hood();
    }
}

module launch_basket() {
    shifted()
        translate([0, 0, 100])
            translate([470, 0, 40])
                rotate([0, 12, 0])
                    translate([-470, 0, -40]) {
                        compact_relieved_bin();
                    }
    shifted() compact_fixed_hood();
}

module compact_battery() {
    translate([-255, 0, 52 + 170/2]) rounded_box([166, 198, 170], 6);
}

module compact_lidar() {
    translate([-420, 0, 478]) cylinder(d=95, h=20);
    translate([-420, 0, 498]) cylinder(d=78, h=36);
}

module compact_intake() {
    shifted() {
        curved_cheek(1);
        curved_cheek(-1);
        compact_handoff_ramp();
        for (sy = [-1, 1])
            translate([470, sy * 90, 70])
                rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
    }
}

module prerepair_handoff_ramp() {
    shifted()
        difference() {
            compact_handoff_ramp_unrelieved();
            compact_axis_expanded() { bin(); hood(); }
        }
}

if (part == "chassis")
    chassis_plate_option_a();
else if (part == "bridge")
    shifted() compact_bridge();
else if (part == "cheeks")
    shifted() { curved_cheek(1); curved_cheek(-1); }
else if (part == "handoff_ramp")
    shifted() compact_handoff_ramp();
else if (part == "handoff_ramp_original")
    shifted() compact_handoff_ramp_unrelieved();
else if (part == "handoff_ramp_prerepair")
    prerepair_handoff_ramp();
else if (part == "handoff_ramp_prerepair_removed")
    difference() {
        shifted() compact_handoff_ramp_unrelieved();
        prerepair_handoff_ramp();
    }
else if (part == "handoff_ramp_left_wheel_intersection")
    intersection() {
        shifted() compact_handoff_ramp();
        shifted() translate([470, 90, 70])
            rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
    }
else if (part == "handoff_ramp_right_wheel_intersection")
    intersection() {
        shifted() compact_handoff_ramp();
        shifted() translate([470, -90, 70])
            rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
    }
else if (part == "intake_wheels")
    shifted() {
        // Wheel solids only; pods/motors are a separate assembly envelope.
        for (sy = [-1, 1])
            translate([470, sy * 90, 70])
                rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
    }
else if (part == "launcher")
    shifted() translate(launcher_origin)
        launcher_oriented(orientation="side_by_side", nip_height=215);
else if (part == "launcher_cradle")
    physical_launcher();
else if (part == "basket_collect")
    collect_basket();
else if (part == "basket_bin")
    shifted() compact_relieved_bin();
else if (part == "receiving_chute")
    shifted() bin_receiving_chute();
else if (part == "receiving_chute_repaired")
    shifted() compact_repaired_receiving_chute();
else if (part == "receiving_chute_prerepair")
    shifted() compact_prerepair_receiving_chute();
else if (part == "basket_bin_local")
    compact_relieved_bin();
else if (part == "basket_bin_original_local")
    bin();
else if (part == "basket_bin_prerepair_local")
    compact_prerepair_relieved_bin();
else if (part == "basket_hood")
    shifted() compact_fixed_hood();
else if (part == "basket_hood_local")
    compact_fixed_hood();
else if (part == "basket_hood_original_local")
    hood();
else if (part == "basket_hood_shell")
    shifted() compact_relieved_hood_shell();
else if (part == "hood_supports")
    shifted() compact_rerouted_hood_supports();
else if (part == "basket_launch")
    launch_basket();
else if (part == "basket_launch_moving")
    shifted()
        translate([0, 0, 100])
            translate([470, 0, 40]) rotate([0, 12, 0])
                translate([-470, 0, -40]) compact_relieved_bin();
else if (part == "basket_guides")
    shifted() compact_basket_guides();
else if (part == "basket_holders")
    shifted() compact_raised_basket_holders();
else if (part == "launcher_hood_intersection")
    intersection() {
        shifted() compact_relieved_hood();
        shifted() translate(launcher_origin)
            launcher_oriented(orientation="side_by_side", nip_height=215);
    }
else if (part == "launcher_bridge_intersection")
    intersection() {
        shifted() compact_bridge();
        physical_launcher();
    }
else if (part == "basket_hood_bridge_intersection")
    intersection() { shifted() compact_bridge(); shifted() compact_relieved_hood(); }
else if (part == "basket_collect_bridge_intersection")
    intersection() { shifted() compact_bridge(); collect_basket(); }
else if (part == "basket_raised_bridge_intersection")
    intersection() { shifted() compact_bridge(); raised_basket(); }
else if (part == "basket_launch_bridge_intersection")
    intersection() { shifted() compact_bridge(); launch_basket(); }
else if (part == "bridge_cheeks_intersection")
    intersection() {
        shifted() compact_bridge();
        shifted() { curved_cheek(1); curved_cheek(-1); }
    }
else if (part == "bridge_chassis_intersection")
    intersection() { shifted() compact_bridge(); chassis_plate_option_a(); }
else if (part == "basket_collect_chassis_intersection")
    intersection() { collect_basket(); chassis_plate_option_a(); }
else if (part == "basket_flange_chassis_intersection")
    intersection() { shifted() bin_flange(); chassis_plate_option_a(); }
else if (part == "basket_walls_chassis_intersection")
    intersection() {
        shifted() compact_relieved_bin_walls(); chassis_plate_option_a();
    }
else if (part == "basket_floor_chassis_intersection")
    intersection() { shifted() bin_floor(); chassis_plate_option_a(); }
else if (part == "launcher_basket_hood_intersection")
    intersection() { physical_launcher(); shifted() compact_relieved_hood(); }
else if (part == "launcher_basket_collect_intersection")
    intersection() { physical_launcher(); collect_basket(); }
else if (part == "launcher_basket_raised_intersection")
    intersection() { physical_launcher(); raised_basket(); }
else if (part == "launcher_hood_raised_intersection")
    intersection() {
        physical_launcher();
        shifted() compact_fixed_hood();
    }
else if (part == "launcher_basket_launch_intersection")
    intersection() { physical_launcher(); launch_basket(); }
else if (part == "launcher_hood_launch_intersection")
    intersection() {
        physical_launcher();
        shifted() compact_fixed_hood();
    }
else if (part == "basket_collect_intake_intersection")
    intersection() { collect_basket(); compact_intake(); }
else if (part == "basket_collect_left_wheel_intersection")
    intersection() {
        collect_basket();
        shifted() translate([470, 90, 70]) rotate([0, 35, 0])
            cylinder(d=124, h=73, center=true);
    }
else if (part == "basket_collect_right_wheel_intersection")
    intersection() {
        collect_basket();
        shifted() translate([470, -90, 70]) rotate([0, 35, 0])
            cylinder(d=124, h=73, center=true);
    }
else if (part == "basket_hood_intake_intersection")
    intersection() { shifted() compact_relieved_hood(); compact_intake(); }
else if (part == "basket_bin_intake_intersection")
    intersection() { shifted() compact_relieved_bin(); compact_intake(); }
else if (part == "basket_collect_battery_intersection")
    intersection() { collect_basket(); compact_battery(); }
else if (part == "basket_raised_battery_intersection")
    intersection() { raised_basket(); compact_battery(); }
else if (part == "basket_launch_battery_intersection")
    intersection() { launch_basket(); compact_battery(); }
else if (part == "basket_launch_lidar_intersection")
    intersection() { launch_basket(); compact_lidar(); }
else if (part == "guides_chassis_intersection")
    intersection() { shifted() compact_basket_guides(); chassis_plate_option_a(); }
else if (part == "guides_intake_intersection")
    intersection() { shifted() compact_basket_guides(); compact_intake(); }
else if (part == "guides_bridge_intersection")
    intersection() { shifted() compact_basket_guides(); shifted() compact_bridge(); }
else if (part == "guides_launcher_intersection")
    intersection() { shifted() compact_basket_guides(); physical_launcher(); }
else if (part == "basket_launch_holders_intersection")
    intersection() { launch_basket(); shifted() compact_raised_basket_holders(); }
else if (part == "holders_chassis_intersection")
    intersection() { shifted() compact_raised_basket_holders(); chassis_plate_option_a(); }
else if (part == "holders_bridge_intersection")
    intersection() { shifted() compact_raised_basket_holders(); shifted() compact_bridge(); }
else if (part == "holders_launcher_intersection")
    intersection() { shifted() compact_raised_basket_holders(); physical_launcher(); }
else if (part == "hood_supports_wheels_intersection")
    intersection() {
        shifted() compact_rerouted_hood_supports();
        shifted()
            for (sy = [-1, 1])
                translate([470, sy * 90, 70])
                    rotate([0, 35, 0]) cylinder(d=128, h=77, center=true);
    }
else if (part == "hood_supports_launcher_intersection")
    intersection() { shifted() compact_rerouted_hood_supports(); physical_launcher(); }
else if (part == "hood_supports_bridge_intersection")
    intersection() { shifted() compact_rerouted_hood_supports(); shifted() compact_bridge(); }
else if (part == "hood_supports_basket_intersection")
    intersection() { shifted() compact_rerouted_hood_supports(); shifted() compact_relieved_bin(); }
else if (part == "hood_supports_chassis_intersection")
    intersection() { shifted() compact_rerouted_hood_supports(); chassis_plate_option_a(); }
else
    assert(false, str("Unknown compact validation part: ", part));
