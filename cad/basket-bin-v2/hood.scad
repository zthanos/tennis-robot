// Chassis-mounted entry hood (log #54-#56): inclined mesh roof over the
// receiving chute plus two side cheeks. Stays on the chassis so the bin
// lifts out without touching it. Solved the 45-ball load escapes; rear
// clearance 120 keeps the incoming ball down onto the pile.
include <params.scad>
use <lib.scad>

module hood_roof() {
    len = sqrt(hood_run ^ 2 + hood_rise ^ 2);
    // panel underside = validated clearance plane, so shift one frame_d up
    translate([hood_rear_x, 0, hood_rear_clearance_z + frame_d])
        rotate([0, -hood_angle, 0])
            translate([len / 2, 0, 0])
                mesh_panel(len, 2 * recv_half_width);
}

module hood_cheeks() {
    // close the sides of the launch channel between chute and roof.
    // The sim collision starts at x 420, but that interpenetrates the
    // bin's corner guards (x 420-430) — two separate physical parts.
    // Trim to x 430-470: the leftover 10 mm slot above the guards is
    // far below ball diameter. Documented in README.
    cheek_len = recv_run - wall_thickness;
    for (sy = [-1, 1])
        translate([bin_front_x + wall_thickness + cheek_len / 2,
                   sy * (recv_half_width + hood_cheek_thickness / 2),
                   (recv_mid_z + hood_side_top_z) / 2])
            cube([cheek_len, hood_cheek_thickness,
                  hood_side_top_z - recv_mid_z], center = true);
}

module hood_mounts() {
    // transverse bar over the plate opening, landing on the side strips
    // (|y| >= 150), with two uprights carrying the roof; see README for
    // the alternative funnel-frame mount
    bar_z = hood_rear_clearance_z + frame_d + (ir_x - hood_rear_x) * tan(hood_angle);
    translate([ir_x, 0, plate_top_z + (bar_z - plate_top_z) / 2]) {
        for (sy = [-1, 1])
            translate([0, sy * (recv_half_width + 25), 0])
                cube([frame_d + 2, frame_d + 2, bar_z - plate_top_z],
                     center = true);
    }
    translate([ir_x, 0, bar_z])
        cube([frame_d + 2, 2 * (open_half_wid + flange_width), frame_d + 2],
             center = true);
}

module hood() {
    color("lightsteelblue") {
        hood_roof();
        hood_cheeks();
        hood_mounts();
    }
}

hood();
