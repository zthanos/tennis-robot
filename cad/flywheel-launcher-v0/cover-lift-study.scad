// Two-position manual basket lift carried by rails, not by the cosmetic cover.
// Packaging/mechanism study only — no manufacturing holes or purchased slides.
// Units: mm, ground frame, robot +X forward, +Y left.

use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/chassis_context.scad>

$fn = 64;

mode = "both"; // "collect", "launch" or "both"
show_chassis = true;
show_battery = true;
show_cover = true;
show_load_path = true;
show_top_pull_handle = true;
show_manual_lever = false; // legacy side-lever comparison only
show_counterbalance = true;
show_future_actuator = true;
show_control_provision = true;
show_actuator_swept_keepout = true;

lift_travel = 100;
launch_tilt_deg = 12;
top_handle_x = 220;
top_handle_post_y = 165;
top_handle_grip_z_low = 328; // 20 mm below the original grip centre
top_handle_rod = 18;

// Existing basket references.
front_pivot = [470, 0, 40];

// Fixed structure.
rail_x = 70;
rail_y = 180;
rail_bottom_z = 52;
rail_top_z = 455;
rail_section = [28, 22];

// Moving carriage and upper-stop geometry.
block_center_z_low = 150;
block_height = 80;
upper_stop_top_z = block_center_z_low + lift_travel - block_height / 2;

// V1 manual actuation references.  These are packaging hardpoints, not a
// solved four-bar linkage: the final lever-arm holes follow measured basket
// weight and handle-force tests on the physical prototype.
lever_side_y = -(rail_y + 70);
lever_pivot = [rail_x - 40, lever_side_y, 120];

// Outer basket envelope, not the 140 mm interior half width: bin.scad carries
// the mesh wall frame out to bin_half_width + frame_d, the flange drop struts
// to bin_half_width + wall_thickness / 2 and the moulded carry handles to
// bin_half_width + 4. The legs must clear the worst of those.
basket_outer_half_width = 146;
top_handle_basket_side_clearance = top_handle_post_y
                                 - top_handle_rod / 2
                                 - basket_outer_half_width;
// Static assembly gap only. The legs and the basket both ride the carriage, so
// there is no relative travel here; the swept leg path at x=211...229 misses
// the fixed rails and their cross-brace at x=56...84 entirely. The pre-existing
// longitudinal carriage beam is tighter still at 7 mm on the same datum.
assert(top_handle_basket_side_clearance >= 10,
       "top-handle legs need at least 10 mm outside the outer basket envelope");

// V2 electric linear-actuator reservation on the opposite side.  No actuator
// stroke or force is selected yet; the translucent volume reserves access and
// lets both clevis brackets survive the manual prototype.
actuator_side_y = rail_y + 55;
actuator_fixed_pin = [rail_x - 55, actuator_side_y, 78];
actuator_moving_pin_low = [rail_x + 155, actuator_side_y, 120];
actuator_body_d = 42;
actuator_keepout_d = 66;

module round_link(p1, p2, d=12, tint="dimgray", alpha=1.0) {
    color(tint, alpha)
        hull() {
            translate(p1) sphere(d=d);
            translate(p2) sphere(d=d);
        }
}

module clevis_bracket(pin, fixed=false) {
    color(fixed ? "goldenrod" : "steelblue")
        translate(pin)
            difference() {
                cube([48, 18, 46], center=true);
                rotate([90, 0, 0]) cylinder(d=12, h=22, center=true);
            }
}

module actuation_fixed_provisions() {
    // Manual lever pivot pedestal transfers user load into the rail foot and
    // chassis frame rather than into the cosmetic cover.
    if (show_manual_lever) {
        color("goldenrod") {
            translate(lever_pivot)
                rotate([90, 0, 0]) cylinder(d=24, h=46, center=true);
            round_link([lever_pivot[0], lever_pivot[1] + 18, 65],
                       [lever_pivot[0], lever_pivot[1] + 18,
                        lever_pivot[2]], 24, "goldenrod");
            translate([lever_pivot[0], lever_pivot[1] + 18, 61])
                cube([82, 56, 12], center=true);
        }
    }

    if (show_future_actuator) {
        clevis_bracket(actuator_fixed_pin, fixed=true);

        // Ghosted swept reservation between the two extreme moving pins.
        if (show_actuator_swept_keepout)
            color("mediumpurple", 0.12)
                hull() {
                    translate(actuator_fixed_pin)
                        sphere(d=actuator_keepout_d);
                    translate(actuator_moving_pin_low)
                        sphere(d=actuator_keepout_d);
                    translate([actuator_moving_pin_low[0],
                               actuator_moving_pin_low[1],
                               actuator_moving_pin_low[2] + lift_travel])
                        sphere(d=actuator_keepout_d);
                }
    }

    if (show_control_provision) {
        // Two fixed limit-switch envelopes: one verifies stowed/locked and the
        // other verifies raised/locked.  Small flags live on the carriage.
        color("orangered")
            for (zz = [block_center_z_low, block_center_z_low + lift_travel])
                translate([rail_x + 26, rail_y + 22, zz])
                    cube([32, 18, 22], center=true);

        // Driver/electrical keepout and protected cable route for V2.
        color("darkorchid", 0.55)
            translate([rail_x - 75, actuator_side_y, 58])
                cube([82, 48, 34], center=true);
        round_link([rail_x - 75, actuator_side_y, 80],
                   [rail_x, actuator_side_y, rail_top_z - 20],
                   9, "darkorchid", 0.55);
    }
}

module manual_lever_actuation(lift=0, launch=false) {
    if (show_manual_lever) {
        handle_end = launch
            ? [lever_pivot[0] + 115, lever_side_y, 346]
            : [lever_pivot[0] - 125, lever_side_y, 310];
        short_arm_end = launch
            ? [lever_pivot[0] - 44, lever_side_y, lever_pivot[2] + 32]
            : [lever_pivot[0] + 38, lever_side_y, lever_pivot[2] + 38];
        carriage_pickup = [rail_x, -(rail_y + 32),
                           block_center_z_low + lift];

        // Long removable handle, short crank arm and adjustable pull link.
        round_link(lever_pivot, handle_end, 22, "firebrick");
        round_link(lever_pivot, short_arm_end, 18, "firebrick");
        round_link(short_arm_end, carriage_pickup, 12, "gold");
        color("black") translate(handle_end) sphere(d=34);
        color("gold") translate(carriage_pickup)
            rotate([90, 0, 0]) cylinder(d=16, h=44, center=true);
    }
}

module counterbalance_for_pose(lift=0) {
    if (show_counterbalance) {
        fixed_eye = [rail_x + 40, -(rail_y + 20), 72];
        moving_eye = [rail_x + 170, -(rail_y + 20), 118 + lift];

        // Gas-spring envelope.  It assists the lift but never replaces the
        // positive locking pins at either end position.
        round_link(fixed_eye,
                   [(fixed_eye[0] + moving_eye[0]) / 2,
                    fixed_eye[1],
                    (fixed_eye[2] + moving_eye[2]) / 2],
                   28, "slategray");
        round_link([(fixed_eye[0] + moving_eye[0]) / 2,
                    fixed_eye[1],
                    (fixed_eye[2] + moving_eye[2]) / 2],
                   moving_eye, 12, "silver");
        color("black") {
            translate(fixed_eye) sphere(d=20);
            translate(moving_eye) sphere(d=20);
        }
    }
}

module future_actuator_for_pose(lift=0) {
    if (show_future_actuator) {
        moving_pin = [actuator_moving_pin_low[0], actuator_side_y,
                      actuator_moving_pin_low[2] + lift];
        clevis_bracket(moving_pin, fixed=false);
        round_link(actuator_fixed_pin, moving_pin, actuator_body_d,
                   "mediumpurple", 0.32);
        color("mediumpurple", 0.5) {
            translate(actuator_fixed_pin) sphere(d=18);
            translate(moving_pin) sphere(d=18);
        }
    }
}

module actuation_for_pose(lift=0, launch=false) {
    manual_lever_actuation(lift, launch);
    counterbalance_for_pose(lift);
    future_actuator_for_pose(lift);

    if (show_control_provision)
        color("orangered")
            translate([rail_x + 4, rail_y + 8,
                       block_center_z_low + lift])
                cube([12, 34, 14], center=true);
}

module fixed_rails() {
    color("silver") {
        for (sy = [-1, 1]) {
            translate([rail_x, sy * rail_y,
                       (rail_bottom_z + rail_top_z) / 2])
                cube([rail_section[0], rail_section[1],
                      rail_top_z - rail_bottom_z], center=true);
            // Wide feet transfer rail loads into the chassis plate/frame.
            translate([rail_x, sy * rail_y, 58])
                cube([90, 70, 12], center=true);
        }
        // Cross-brace stops the two rails racking independently.
        translate([rail_x, 0, rail_top_z - 12])
            cube([28, 2 * rail_y + 28, 24], center=true);
    }

    if (show_load_path)
        color("gold")
            for (sy = [-1, 1])
                translate([rail_x, sy * rail_y, upper_stop_top_z - 8])
                    cube([54, 46, 16], center=true);
}

module sliding_blocks(lift=0) {
    color("steelblue")
        for (sy = [-1, 1])
            translate([rail_x, sy * rail_y, block_center_z_low + lift])
                difference() {
                    cube([66, 48, block_height], center=true);
                    cube([rail_section[0] + 3, rail_section[1] + 3,
                          block_height + 2], center=true);
                }
}

module carriage_frame(lift=0) {
    color("steelblue") {
        // Longitudinal beams carry the basket cradle, not the cover skin.
        for (sy = [-1, 1])
            translate([225, sy * 165, 92 + lift])
                cube([450, 24, 24], center=true);
        for (xx = [15, 435])
            translate([xx, 0, 92 + lift])
                cube([24, 354, 24], center=true);
        // Tie the carriage beams back to both sliding blocks.
        for (sy = [-1, 1])
            hull() {
                translate([rail_x, sy * rail_y, 128 + lift])
                    cube([34, 28, 28], center=true);
                translate([155, sy * 165, 92 + lift])
                    cube([34, 24, 24], center=true);
            }
    }
}

module locking_pins(lift=0, upper=false) {
    if (show_load_path)
        color("gold")
            for (sy = [-1, 1])
                translate([rail_x, sy * (rail_y + 28),
                           block_center_z_low + lift])
                    rotate([90, 0, 0])
                        cylinder(d=16, h=56, center=true);
}

module cosmetic_cover(lift=0) {
    if (show_cover)
        color("lightblue", 0.72) {
            // Cover outline only: the production skin attaches here but does
            // not carry basket load. Frame rendering keeps the load-bearing
            // blue carriage and wire basket visible in OpenSCAD previews.
            for (sy = [-1, 1])
                translate([220, sy * 192, 292 + lift])
                    cube([470, 10, 10], center=true);
            for (xx = [-12, 452])
                translate([xx, 0, 292 + lift])
                    cube([10, 394, 10], center=true);
            for (xx = [-12, 452], sy = [-1, 1])
                translate([xx, sy * 192, 195 + lift])
                    cube([10, 10, 194], center=true);
            // Two light side rails locate removable cosmetic panels.
            for (sy = [-1, 1])
                translate([220, sy * 192, 150 + lift])
                    cube([470, 8, 8], center=true);
        }

}

module top_pull_handle(lift=0) {
    // Original central top handle. It belongs to the moving carriage/load
    // path, not to the removable cosmetic hatch. The hatch must be open for
    // manual operation; the handle remains inside the closed shell envelope.
    //
    // The grip is gripped at the centre but reacted at the legs, so the rod
    // works in bending over a 330 mm unsupported span. It is a purchased metal
    // section, not a printed part: an 18 mm square in PLA reaches about 26 MPa
    // and sags roughly 9 mm under a 300 N pull, against about 0.4 mm in
    // aluminium at the same stress.
    if (show_top_pull_handle)
        color("dimgray") {
            post_bottom_z = 92 + 12; // top of the longitudinal carriage beam
            for (sy = [-1, 1])
                translate([top_handle_x, sy * top_handle_post_y,
                           (post_bottom_z + top_handle_grip_z_low) / 2 + lift])
                    cube([top_handle_rod, top_handle_rod,
                          top_handle_grip_z_low - post_bottom_z], center=true);
            translate([top_handle_x, 0,
                       top_handle_grip_z_low + lift])
                cube([top_handle_rod,
                      2 * top_handle_post_y + top_handle_rod,
                      top_handle_rod], center=true);
        }
}

module basket_launch_pose() {
    translate([0, 0, lift_travel])
        translate(front_pivot)
            rotate([0, launch_tilt_deg, 0])
                translate(-front_pivot)
                    bin();
}

module moving_carriage_and_cover(lift=0, launch=false) {
    sliding_blocks(lift);
    carriage_frame(lift);
    locking_pins(lift, upper=launch);
    cosmetic_cover(lift);
    top_pull_handle(lift);
}

module collect_configuration(alpha=1.0) {
    color("white", alpha) bin();
    moving_carriage_and_cover(lift=0, launch=false);
    actuation_for_pose(lift=0, launch=false);
    // In collect, the existing basket flange/chassis ring carries the load.
    if (show_load_path)
        color("gold")
            for (sy = [-1, 1])
                translate([220, sy * 161, 54])
                    cube([400, 18, 8], center=true);
}

module launch_configuration() {
    basket_launch_pose();
    moving_carriage_and_cover(lift=lift_travel, launch=true);
    actuation_for_pose(lift=lift_travel, launch=true);
}

if (show_chassis) chassis_plate();
if (show_battery) battery();
fixed_rails();
actuation_fixed_provisions();

if (mode == "collect")
    collect_configuration();
else if (mode == "launch")
    launch_configuration();
else if (mode == "both") {
    color("seagreen", 0.18) collect_configuration(alpha=0.20);
    launch_configuration();
} else
    assert(false, str("Unknown mode: ", mode));
