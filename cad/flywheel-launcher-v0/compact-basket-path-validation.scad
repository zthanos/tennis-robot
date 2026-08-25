// Parameterized exact-boolean wrapper for the compact basket guide path.
// Analysis source: no fixed functional datum is redefined here.
// Units: mm, CAD ground frame (+X forward, +Y left, +Z up).

use <compact-packaging-study.scad>
use <launcher-envelope.scad>
use <../collector-intake-v1/option-a/option-a.scad>
use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/hood.scad>
use <compact-basket-support.scad>
include <params.scad>

part = "moving_basket";
path_lift_mm = 100;
path_retraction_mm = 0; // positive means rearward (-X)
path_tilt_deg = 12;
study_opening_extension_mm = 0;
study_relief_clearance_mm = 0;

functional_shift_x = -100;
launcher_origin = [560, 0, 0];
path_pivot = [470, 0, 40];

// Analysis-only derivative of the protected 920 x 580 x 14 mm chassis plate.
// The opening keeps its authoritative front and lateral datums and moves only
// its rear edge from X=10 toward -X. This module is never called by production
// CAD; it exists solely for the permitted opening-extension design study.
module study_chassis_plate(opening_extension_mm=study_opening_extension_mm) {
    translate([0, 0, 45])
        difference() {
            cube([920, 580, 14], center=true);
            opening_rear_x = 10 - opening_extension_mm;
            translate([(opening_rear_x + 460) / 2, 0, 0])
                cube([460 - opening_rear_x, 300, 16], center=true);
        }
}

module shifted() {
    translate([functional_shift_x, 0, 0]) children();
}

module path_transform(lift_mm=path_lift_mm,
                      retraction_mm=path_retraction_mm,
                      tilt_deg=path_tilt_deg) {
    shifted()
        translate([-retraction_mm, 0, lift_mm])
            translate(path_pivot)
                rotate([0, tilt_deg, 0])
                    translate(-path_pivot)
                        children();
}

module moving_basket() {
    path_transform() { bin(); hood(); }
}

module moving_hood() {
    path_transform() hood();
}

module fixed_launcher() {
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

module fixed_launcher_wheels() {
    shifted() translate(launcher_origin)
        translate([0, 0, 215])
            rotate([0, -pitch_deg, 0])
                rotate([90, 0, 0])
                    translate([0, 0, -path_z]) {
                        wheel_envelope(lower_wheel_z);
                        wheel_envelope(upper_wheel_z);
                    }
}

module fixed_launcher_plates() {
    shifted() translate(launcher_origin)
        translate([0, 0, 215])
            rotate([0, -pitch_deg, 0])
                rotate([90, 0, 0])
                    translate([0, 0, -path_z]) {
                        side_plate(-1);
                        side_plate(1);
                    }
}

module fixed_intake() {
    shifted() {
        curved_cheek(1);
        curved_cheek(-1);
        compact_handoff_ramp();
        for (sy = [-1, 1])
            translate([470, sy * 90, 70])
                rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
    }
}

module fixed_intake_wheels() {
    shifted()
        for (sy = [-1, 1])
            translate([470, sy * 90, 70])
                rotate([0, 35, 0]) cylinder(d=124, h=73, center=true);
}

module fixed_intake_cheeks() {
    shifted() { curved_cheek(1); curved_cheek(-1); }
}

module fixed_intake_ramp() {
    shifted() compact_handoff_ramp();
}

module fixed_battery() {
    translate([-255, 0, 52 + 170/2])
        rounded_box([166, 198, 170], 6);
}

module fixed_lidar() {
    translate([-420, 0, 478]) cylinder(d=95, h=20);
    translate([-420, 0, 498]) cylinder(d=78, h=36);
}

module expanded(clearance_mm=study_relief_clearance_mm) {
    if (clearance_mm > 0)
        minkowski() {
            children();
            sphere(r=clearance_mm, $fn=12);
        }
    else
        children();
}

// Robust clearance envelope for complex wire-mesh unions. Six translated
// copies provide the declared axial gap without CGAL Minkowski decomposition
// failures and without filling the basket's intentional open volume.
module axis_expanded(clearance_mm=study_relief_clearance_mm) {
    children();
    if (clearance_mm > 0)
        for (offset = [[clearance_mm,0,0], [-clearance_mm,0,0],
                       [0,clearance_mm,0], [0,-clearance_mm,0],
                       [0,0,clearance_mm], [0,0,-clearance_mm]])
            translate(offset) children();
}

module bin_without_walls() {
    bin_floor();
    bin_management_tray();
    bin_receiving_chute();
    bin_flange();
    bin_handles();
    bin_front_retention();
}

module rerouted_hood_supports() {
    support_x = 430;
    post_y = 184;
    post_size = 8;
    chassis_top_z = 52;
    crossbar_t = 6;
    crossbar_bottom_z = 142;
    crossbar_top_z = crossbar_bottom_z + crossbar_t;
    roof_attach_z = 120 + 6 + (support_x - 380) * tan(atan2(15, 90));

    // Two posts bear on the chassis side strips, outside the basket flange,
    // tyre pockets and bridge uprights.
    for (sy = [-1, 1])
        translate([support_x, sy * post_y,
                   (chassis_top_z + crossbar_top_z) / 2])
            cube([post_size, post_size,
                  crossbar_top_z - chassis_top_z], center=true);

    // Portal crossbar occupies the measured 142..148 mm corridor: above the
    // expanded wheel envelope and 2 mm below the bridge underside.
    translate([support_x, 0, (crossbar_bottom_z + crossbar_top_z) / 2])
        cube([post_size, 2 * post_y + post_size, crossbar_t], center=true);

    // Twin central hangers stay inside the 2 mm-expanded inter-wheel gap and
    // prevent the hood roof from twisting about a single attachment.
    for (sy = [-1, 1])
        translate([support_x, sy * 35,
                   (roof_attach_z + crossbar_bottom_z) / 2])
            cube([post_size, post_size,
                  crossbar_bottom_z - roof_attach_z], center=true);
}

// Analysis-only minimum-relief concept. The functional basket datum and every
// fixed launcher/intake datum remain unchanged. Only material participating in
// a measured PARKED solid conflict is removed.
module relieved_parked_bin(clearance_mm=study_relief_clearance_mm) {
    difference() {
        shifted() bin_without_walls();
        expanded(clearance_mm) fixed_intake_wheels();
    }
    difference() {
        shifted() bin_walls();
        expanded(clearance_mm) fixed_intake_wheels();
        expanded(clearance_mm) study_chassis_plate(0);
    }
}

module relieved_parked_hood(clearance_mm=study_relief_clearance_mm) {
    difference() {
        shifted() { hood_roof(); hood_cheeks(); }
        expanded(clearance_mm) fixed_intake_wheels();
        expanded(clearance_mm) fixed_launcher_plates();
    }
    shifted() rerouted_hood_supports();
}

module relieved_intake_ramp(clearance_mm=study_relief_clearance_mm) {
    difference() {
        fixed_intake_ramp();
        axis_expanded(clearance_mm) shifted() { bin(); hood(); }
    }
}

module relieved_parked_basket(clearance_mm=study_relief_clearance_mm) {
    relieved_parked_bin(clearance_mm);
    relieved_parked_hood(clearance_mm);
}

module relieved_fixed_intake(clearance_mm=study_relief_clearance_mm) {
    fixed_intake_wheels();
    fixed_intake_cheeks();
    relieved_intake_ramp(clearance_mm);
}

if (part == "moving_basket")
    moving_basket();
else if (part == "moving_hood")
    moving_hood();
else if (part == "basket_launcher_intersection")
    intersection() { moving_basket(); fixed_launcher(); }
else if (part == "hood_launcher_intersection")
    intersection() { moving_hood(); fixed_launcher(); }
else if (part == "hood_roof_launcher_wheels_intersection")
    intersection() { path_transform() hood_roof(); fixed_launcher_wheels(); }
else if (part == "hood_roof_launcher_plates_intersection")
    intersection() { path_transform() hood_roof(); fixed_launcher_plates(); }
else if (part == "hood_cheeks_launcher_wheels_intersection")
    intersection() { path_transform() hood_cheeks(); fixed_launcher_wheels(); }
else if (part == "hood_cheeks_launcher_plates_intersection")
    intersection() { path_transform() hood_cheeks(); fixed_launcher_plates(); }
else if (part == "hood_mounts_launcher_intersection")
    intersection() { path_transform() hood_mounts(); fixed_launcher(); }
else if (part == "basket_bridge_intersection")
    intersection() { moving_basket(); shifted() compact_bridge(); }
else if (part == "basket_chassis_intersection")
    intersection() { moving_basket(); study_chassis_plate(); }
else if (part == "basket_walls_chassis_intersection")
    intersection() { path_transform() bin_walls(); study_chassis_plate(); }
else if (part == "basket_flange_chassis_intersection")
    intersection() { path_transform() bin_flange(); study_chassis_plate(); }
else if (part == "basket_floor_chassis_intersection")
    intersection() { path_transform() bin_floor(); study_chassis_plate(); }
else if (part == "study_chassis")
    study_chassis_plate();
else if (part == "basket_intake_intersection")
    intersection() { moving_basket(); fixed_intake(); }
else if (part == "bin_wheels_intersection")
    intersection() { path_transform() bin(); fixed_intake_wheels(); }
else if (part == "bin_floor_wheels_intersection")
    intersection() { path_transform() bin_floor(); fixed_intake_wheels(); }
else if (part == "bin_management_wheels_intersection")
    intersection() { path_transform() bin_management_tray(); fixed_intake_wheels(); }
else if (part == "bin_receiving_wheels_intersection")
    intersection() { path_transform() bin_receiving_chute(); fixed_intake_wheels(); }
else if (part == "bin_walls_wheels_intersection")
    intersection() { path_transform() bin_walls(); fixed_intake_wheels(); }
else if (part == "bin_front_wheels_intersection")
    intersection() { path_transform() bin_front_retention(); fixed_intake_wheels(); }
else if (part == "bin_cheeks_intersection")
    intersection() { path_transform() bin(); fixed_intake_cheeks(); }
else if (part == "bin_ramp_intersection")
    intersection() { path_transform() bin(); fixed_intake_ramp(); }
else if (part == "hood_wheels_intersection")
    intersection() { path_transform() hood(); fixed_intake_wheels(); }
else if (part == "hood_roof_wheels_intersection")
    intersection() { path_transform() hood_roof(); fixed_intake_wheels(); }
else if (part == "hood_cheeks_wheels_intersection")
    intersection() { path_transform() hood_cheeks(); fixed_intake_wheels(); }
else if (part == "hood_mounts_wheels_intersection")
    intersection() { path_transform() hood_mounts(); fixed_intake_wheels(); }
else if (part == "hood_cheeks_intersection")
    intersection() { path_transform() hood(); fixed_intake_cheeks(); }
else if (part == "hood_ramp_intersection")
    intersection() { path_transform() hood(); fixed_intake_ramp(); }
else if (part == "basket_battery_intersection")
    intersection() { moving_basket(); fixed_battery(); }
else if (part == "basket_lidar_intersection")
    intersection() { moving_basket(); fixed_lidar(); }
else if (part == "basket_holders_intersection")
    intersection() { moving_basket(); shifted() compact_raised_basket_holders(); }
else if (part == "relieved_parked_basket")
    relieved_parked_basket();
else if (part == "relieved_intake_ramp")
    relieved_intake_ramp();
else if (part == "relieved_basket_launcher_intersection")
    intersection() { relieved_parked_basket(); fixed_launcher(); }
else if (part == "relieved_basket_intake_intersection")
    intersection() { relieved_parked_basket(); relieved_fixed_intake(); }
else if (part == "relieved_basket_chassis_intersection")
    intersection() { relieved_parked_basket(); study_chassis_plate(0); }
else if (part == "bin_relief_removed")
    difference() { shifted() bin(); relieved_parked_bin(); }
else if (part == "hood_relief_removed")
    difference() { shifted() hood(); relieved_parked_hood(); }
else if (part == "ramp_relief_removed")
    difference() { fixed_intake_ramp(); relieved_intake_ramp(); }
else if (part == "wall_chassis_relief_envelope")
    intersection() {
        shifted() bin_walls();
        expanded(study_relief_clearance_mm) study_chassis_plate(0);
    }
else if (part == "bin_wheel_relief_envelope")
    intersection() {
        shifted() bin();
        expanded(study_relief_clearance_mm) fixed_intake_wheels();
    }
else if (part == "hood_wheel_relief_envelope")
    intersection() {
        shifted() hood();
        expanded(study_relief_clearance_mm) fixed_intake_wheels();
    }
else if (part == "hood_launcher_relief_envelope")
    intersection() {
        shifted() hood();
        expanded(study_relief_clearance_mm) fixed_launcher_plates();
    }
else if (part == "rerouted_hood_supports")
    shifted() rerouted_hood_supports();
else if (part == "rerouted_supports_wheels_intersection")
    intersection() { shifted() rerouted_hood_supports(); expanded(2) fixed_intake_wheels(); }
else if (part == "rerouted_supports_launcher_intersection")
    intersection() { shifted() rerouted_hood_supports(); expanded(2) fixed_launcher(); }
else if (part == "rerouted_supports_bridge_intersection")
    intersection() { shifted() rerouted_hood_supports(); shifted() compact_bridge(); }
else if (part == "rerouted_supports_basket_intersection")
    intersection() { shifted() rerouted_hood_supports(); shifted() bin(); }
else if (part == "rerouted_supports_chassis_intersection")
    intersection() { shifted() rerouted_hood_supports(); study_chassis_plate(0); }
else if (part == "ramp_basket_relief_envelope")
    intersection() {
        fixed_intake_ramp();
        axis_expanded(study_relief_clearance_mm) shifted() { bin(); hood(); }
    }
else
    assert(false, str("Unknown path-validation part: ", part));
