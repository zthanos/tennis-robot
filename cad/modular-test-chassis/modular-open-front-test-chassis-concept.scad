// Modular open-front PLA test chassis — packaging/interface concept only.
// Units: mm, ground frame, +X forward, +Y left, +Z up.
//
// This assembly deliberately preserves the current physical intake datums.
// It is NOT a manufacturing release: beam walls, internal ribs, splice bolts,
// drive-pad patterns and fastener access still require physical measurements
// and structural iteration.

$fn = 48;

use <../standalone-intake-fixed-motor/gamma-base-repair-shoe.scad>
use <../standalone-intake-fixed-motor/dual-outboard-gamma-brackets.scad>

part = "assembly"; // [assembly,frame,intake_context,ramp_cradles,support_wheel_interference]

// Test-frame hypothesis.  Intake coordinates remain unchanged.
frame_rear_x = -300;
frame_front_x = 460;
frame_open_rear_x = 10;

rail_center_y = 210;
rail_width = 70;
rail_top_z = 52;
rail_height = 34;
rail_wall = 3.2;

// Four bed-sized pieces.  Splices deliberately stay away from both provisional
// axle/motor-pad stations so skid-steer reactions do not enter a joint first.
// Nominal lengths before the small assembly gaps: 180, 165, 255 and 160 mm.
rail_gap = 0.6;
rail_segments = [
    [frame_rear_x, -120],
    [-120 + rail_gap, 45],
    [45 + rail_gap, 300],
    [300 + rail_gap, frame_front_x]
];
splice_x = [-120, 45, 300];
splice_length = 100;
splice_clearance = 0.30;
splice_wall = 3.0;

crossbar_width_x = 30;
rear_crossbar_x = -270;
opening_crossbar_x = frame_open_rear_x - crossbar_width_x / 2;

// Provisional drive positions.  Their pads remain movable along the rails.
rear_axle_x = -220;
front_axle_x = 220;
drive_axle_y = 350;
drive_axle_z = 85;
drive_wheel_d = 170;
drive_wheel_w = 80;

drive_node_size = [70, 70, 8];
steel_mount_foot_size = [40, 40, 2];
electronics_tray_size = [190, 290, 4];
electronics_tray_x = -115;

// Provisional no-drill ramp retention.  Two independent side cradles reach
// inward from the rails and capture only the rear/high sidewall regions.
// Nothing crosses the 180 mm ball corridor or sits in front of the low lip.
ramp_support_x = 321;
ramp_outer_y = 94;
rail_inner_y = rail_center_y - rail_width / 2;
ramp_arm_inner_y = 102;
ramp_arm_size_x = 10;
ramp_arm_height = 10;
ramp_arm_z = 47;
ramp_clip_outer_y = 98.5;
ramp_clip_inner_y = 87.5;
ramp_clip_outer_w = 7;
ramp_clip_inner_w = 3;
ramp_clip_height = 18;
ramp_clip_z = 42;
ramp_clip_top_z = 54;
ramp_clip_top_t = 4;

frame_color = [0.15, 0.47, 0.78, 0.82];
splice_color = [0.96, 0.55, 0.10, 0.72];
crossbar_color = [0.08, 0.34, 0.62, 0.90];
pad_color = [0.78, 0.80, 0.83, 0.96];
electronics_color = [0.12, 0.55, 0.35, 0.70];
wheel_color = [0.06, 0.06, 0.07, 0.92];

module hollow_box_x(x0, x1, yc, width, z0, height, wall) {
    difference() {
        translate([(x0 + x1) / 2, yc, z0 + height / 2])
            cube([x1 - x0, width, height], center=true);
        translate([(x0 + x1) / 2, yc, z0 + height / 2])
            cube([x1 - x0 + 2,
                  width - 2 * wall,
                  height - 2 * wall], center=true);
    }
}

module hollow_box_y(xc, y0, y1, width_x, z0, height, wall) {
    difference() {
        translate([xc, (y0 + y1) / 2, z0 + height / 2])
            cube([width_x, y1 - y0, height], center=true);
        translate([xc, (y0 + y1) / 2, z0 + height / 2])
            cube([width_x - 2 * wall,
                  y1 - y0 + 2,
                  height - 2 * wall], center=true);
    }
}

module splice_sleeve(yc, xc) {
    cavity_w = rail_width - 2 * rail_wall;
    cavity_h = rail_height - 2 * rail_wall;
    sleeve_w = cavity_w - 2 * splice_clearance;
    sleeve_h = cavity_h - 2 * splice_clearance;

    color(splice_color)
        difference() {
            translate([xc, yc,
                       rail_top_z - rail_height / 2])
                cube([splice_length, sleeve_w, sleeve_h], center=true);
            translate([xc, yc,
                       rail_top_z - rail_height / 2])
                cube([splice_length + 2,
                      sleeve_w - 2 * splice_wall,
                      sleeve_h - 2 * splice_wall], center=true);
        }
}

module splice_bolts(yc, xc) {
    for (dx = [-30, 30])
        color("silver")
            translate([xc + dx, yc, rail_top_z - rail_height - 1])
                cylinder(d=5.5, h=rail_height + 7);
}

module segmented_side_rail(side=1) {
    yc = side * rail_center_y;

    color(frame_color)
        for (segment = rail_segments)
            hollow_box_x(segment[0], segment[1], yc,
                         rail_width, rail_top_z - rail_height,
                         rail_height, rail_wall);

    for (xc = splice_x) {
        splice_sleeve(yc, xc);
        splice_bolts(yc, xc);
    }
}

module rear_and_opening_crossbars() {
    y0 = -rail_center_y + rail_width / 2;
    y1 = rail_center_y - rail_width / 2;

    color(crossbar_color) {
        hollow_box_y(rear_crossbar_x, y0, y1,
                     crossbar_width_x, rail_top_z - rail_height,
                     rail_height, rail_wall);
        // Its front face ends exactly at X=10, preserving the existing
        // X=10..front central opening and leaving no front crossbar.
        hollow_box_y(opening_crossbar_x, y0, y1,
                     crossbar_width_x, rail_top_z - rail_height,
                     rail_height, rail_wall);
    }
}

module provisional_drive_pad(x, side=1) {
    // The photographed steel foot is 40 x 40 mm, but the PLA load-spreading
    // node must be substantially larger because its holes sit near that edge.
    // The node spans the complete 70 mm rail width; the steel foot is shown
    // separately on top and remains provisional until all four mounts are
    // measured.
    color(crossbar_color)
        translate([x, side * rail_center_y,
                   rail_top_z + drive_node_size[2] / 2])
            cube(drive_node_size, center=true);
    color(pad_color)
        translate([x, side * rail_center_y,
                   rail_top_z + drive_node_size[2]
                       + steel_mount_foot_size[2] / 2])
            cube(steel_mount_foot_size, center=true);
}

module drive_wheel(x, side=1) {
    color(wheel_color)
        translate([x, side * drive_axle_y, drive_axle_z])
            rotate([90, 0, 0])
                difference() {
                    cylinder(d=drive_wheel_d, h=drive_wheel_w, center=true);
                    cylinder(d=36, h=drive_wheel_w + 2, center=true);
                }
}

module drive_context() {
    for (x = [rear_axle_x, front_axle_x], side = [-1, 1]) {
        provisional_drive_pad(x, side);
        drive_wheel(x, side);
    }
}

module electronics_tray_context() {
    color(electronics_color)
        translate([electronics_tray_x, 0,
                   rail_top_z + electronics_tray_size[2] / 2])
            cube(electronics_tray_size, center=true);
}

module ramp_side_cradle_positive() {
    // Cantilever arm from the inside face of the side rail to the outside of
    // the ramp wall.  Final geometry needs fillets and diagonal ribs.
    translate([ramp_support_x,
               (rail_inner_y + ramp_arm_inner_y) / 2,
               ramp_arm_z])
        cube([ramp_arm_size_x,
              rail_inner_y - ramp_arm_inner_y,
              ramp_arm_height], center=true);

    // Outer and inner jaws capture the rear/high wall without drilling it.
    translate([ramp_support_x, ramp_clip_outer_y, ramp_clip_z])
        cube([ramp_arm_size_x, ramp_clip_outer_w,
              ramp_clip_height], center=true);
    translate([ramp_support_x, ramp_clip_inner_y, ramp_clip_z])
        cube([ramp_arm_size_x, ramp_clip_inner_w,
              ramp_clip_height], center=true);

    // Removable-cap envelope shown closed in the concept.  The released part
    // must split here so the already printed ramp can be installed/removed.
    translate([ramp_support_x,
               (ramp_clip_inner_y + ramp_clip_outer_y) / 2,
               ramp_clip_top_z])
        cube([ramp_arm_size_x,
              ramp_clip_outer_y - ramp_clip_inner_y + ramp_clip_outer_w,
              ramp_clip_top_t], center=true);

    // Rear stop prevents the short ramp from moving toward the basket while
    // leaving the complete centre exit open.
    translate([316,
               (ramp_clip_inner_y + ramp_clip_outer_y) / 2,
               ramp_clip_z + 2])
        cube([4,
              ramp_clip_outer_y - ramp_clip_inner_y + ramp_clip_outer_w,
              ramp_clip_height + 4], center=true);
}

module ramp_side_cradle(side=1) {
    color([0.58, 0.20, 0.76, 0.94])
        if (side > 0) ramp_side_cradle_positive();
        else mirror([0, 1, 0]) ramp_side_cradle_positive();
}

module ramp_cradles() {
    ramp_side_cradle(1);
    ramp_side_cradle(-1);
}

module frame() {
    segmented_side_rail(1);
    segmented_side_rail(-1);
    rear_and_opening_crossbars();
}

module repaired_gamma_pair() {
    // Actual first-prototype Γ bodies plus the already modelled repair shoes.
    color([0.78, 0.26, 0.08, 0.78]) {
        surviving_gamma(1);
        surviving_gamma(-1);
    }
    color([0.12, 0.55, 0.90, 0.92]) {
        repair_shoe(1);
        repair_shoe(-1);
    }
}

module corrected_ramp() {
    color([0.42, 0.72, 0.32, 0.95])
        import("../standalone-intake-fixed-motor/stl/intake_handoff_ramp_wheel_first.stl");
}

module intake_context() {
    repaired_gamma_pair();
    corrected_ramp();
}

module assembly() {
    frame();
    drive_context();
    electronics_tray_context();
    intake_context();
    ramp_cradles();
}

if (part == "frame") frame();
else if (part == "intake_context") intake_context();
else if (part == "ramp_cradles") ramp_cradles();
else if (part == "support_wheel_interference")
    intersection() {
        ramp_cradles();
        union() {
            wheel_context(1);
            wheel_context(-1);
        }
    }
else assembly();
