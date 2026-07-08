// Curved centre scoop for the tennis-ball collector.
//
// Coordinate system:
//   X = ball travel, from the floor-level front lip toward the collector
//   Y = scoop width
//   Z = height above the floor
//
// The default 180 x 180 x 100 mm scoop fits common hobby printers.
// Set render_part to "left" or "right" to export two 90 mm wide halves.

$fn = 72;

// Main dimensions (mm).
scoop_width = 180;
scoop_length = 180;
scoop_height = 100;
shell_thickness = 4;
front_lip_height = 2;

// Shape controls.
// Values above 1 make the entry shallow and the rear progressively steeper.
curve_exponent = 1.75;
curve_steps = 80;

// Optional edge guides. Keep disabled when this is only the centre section.
edge_guides = false;
guide_height = 20;
guide_thickness = 4;

// Detachable rear mounts. A continuous keyed rail is printed with the scoop;
// two identical ears slide onto it from either side and bolt to the base.
mounting_rail = true;
mount_ear_length = 38;
mount_ear_width = 28;
mount_ear_height = 16;
mount_hole_diameter = 5.5;
mount_hole_edge_offset = 12;
mount_fit_clearance = 0.30; // Per side; tune for the printer/material.

// Roller fit-check dimensions: 120 mm capture width x 45 mm diameter.
roller_diameter = 45;
roller_width = 120;
roller_bore_diameter = 6.35;
roller_shaft_length = 190;
roller_center_x = 135;
roller_ramp_gap = 60;
roller_center_z =
    front_lip_height
    + (scoop_height - front_lip_height)
        * pow(roller_center_x / scoop_length, curve_exponent)
    + roller_diameter / 2
    + roller_ramp_gap;
roller_bracket_thickness = 8;
roller_bracket_width = 38;
roller_shaft_clearance = 0.35;
motor_body_diameter = 37;
motor_body_length = 45;
shaft_coupler_diameter = 16;
shaft_coupler_length = 20;

// "scoop", "left", "right", "ear", "roller", or "assembly".
// Wrapper SCAD files set part_override before including this source.
render_part = is_undef(part_override) ? "scoop" : part_override;

function ramp_z(x) =
    front_lip_height
    + (scoop_height - front_lip_height) * pow(x / scoop_length, curve_exponent);

function inner_z(x) = max(0, ramp_z(x) - shell_thickness);

function shell_profile_points() =
    concat(
        [for (i = [0 : curve_steps])
            let(x = scoop_length * i / curve_steps)
            [x, ramp_z(x)]],
        [for (i = [curve_steps : -1 : 0])
            let(x = scoop_length * i / curve_steps)
            [x, inner_z(x)]]
    );

function guide_profile_points() =
    concat(
        [for (i = [0 : curve_steps])
            let(x = scoop_length * i / curve_steps)
            [x, ramp_z(x)]],
        [for (i = [curve_steps : -1 : 0])
            let(x = scoop_length * i / curve_steps)
            [x, ramp_z(x) + guide_height]]
    );

module curved_shell(width = scoop_width) {
    rotate([90, 0, 0])
        linear_extrude(height = width, center = true, convexity = 10)
            polygon(points = shell_profile_points());
}

module side_guide(y) {
    translate([0, y, 0])
        rotate([90, 0, 0])
            linear_extrude(height = guide_thickness, center = true, convexity = 10)
                polygon(points = guide_profile_points());
}

function rail_profile(clearance = 0) = [
    [scoop_length - 9 - clearance, scoop_height - 7 - clearance],
    [scoop_length + 4 + clearance, scoop_height - 7 - clearance],
    [scoop_length + 10 + clearance, scoop_height - 2 - clearance],
    [scoop_length + 10 + clearance, scoop_height + 2 + clearance],
    [scoop_length - 9 - clearance, scoop_height + 2 + clearance]
];

module rear_mounting_rail() {
    rotate([90, 0, 0])
        linear_extrude(height = scoop_width, center = true, convexity = 10)
            polygon(points = rail_profile());
}

module mounting_ear_at(y = 0) {
    ear_start_x = scoop_length - 12;
    ear_start_z = scoop_height - 10;
    hole_x = scoop_length + mount_ear_length - mount_hole_edge_offset;

    difference() {
        translate([ear_start_x, y - mount_ear_width / 2, ear_start_z])
            cube([
                mount_ear_length + 12,
                mount_ear_width,
                mount_ear_height
            ]);

        // Keyed channel runs through the ear so it can slide along the rail.
        translate([0, y, 0])
            rotate([90, 0, 0])
                linear_extrude(
                    height = mount_ear_width + 2,
                    center = true,
                    convexity = 10
                )
                    polygon(points = rail_profile(mount_fit_clearance));

        translate([hole_x, y, ear_start_z - 0.5])
            cylinder(h = mount_ear_height + 1, d = mount_hole_diameter);
    }
}

module standalone_mounting_ear() {
    // Localized near the origin for straightforward slicer placement.
    translate([
        -(scoop_length - 12),
        mount_ear_width / 2,
        -(scoop_height - 10)
    ])
        mounting_ear_at(0);
}

module collector_roller() {
    difference() {
        rotate([90, 0, 0])
            cylinder(h = roller_width, d = roller_diameter, center = true);

        rotate([90, 0, 0])
            cylinder(
                h = roller_width + 2,
                d = roller_bore_diameter,
                center = true
            );
    }
}

module roller_support(y) {
    support_bottom_z = scoop_height + 2;
    support_height = roller_center_z - support_bottom_z + 25;

    difference() {
        translate([
            roller_center_x - roller_bracket_width / 2,
            y - roller_bracket_thickness / 2,
            support_bottom_z
        ])
            cube([
                roller_bracket_width,
                roller_bracket_thickness,
                support_height
            ]);

        translate([roller_center_x, y, roller_center_z])
            rotate([90, 0, 0])
                cylinder(
                    h = roller_bracket_thickness + 2,
                    d = roller_bore_diameter + roller_shaft_clearance,
                    center = true
                );
    }
}

module roller_drive_assembly() {
    support_y = roller_width / 2 + roller_bracket_thickness / 2 + 3;
    motor_y = support_y + roller_bracket_thickness / 2
        + shaft_coupler_length + motor_body_length / 2;

    // Printable roller.
    color([0.10, 0.10, 0.10])
        translate([roller_center_x, 0, roller_center_z])
            collector_roller();

    // Metal shaft: reference geometry only, not intended for printing.
    color([0.72, 0.72, 0.75])
        translate([roller_center_x, 0, roller_center_z])
            rotate([90, 0, 0])
                cylinder(
                    h = roller_shaft_length,
                    d = roller_bore_diameter,
                    center = true
                );

    color([0.85, 0.68, 0.15]) {
        roller_support(-support_y);
        roller_support( support_y);
    }

    // Shaft coupler and 37 mm gearmotor envelope for clearance checking.
    color([0.55, 0.55, 0.58])
        translate([
            roller_center_x,
            support_y + roller_bracket_thickness / 2
                + shaft_coupler_length / 2,
            roller_center_z
        ])
            rotate([90, 0, 0])
                cylinder(
                    h = shaft_coupler_length,
                    d = shaft_coupler_diameter,
                    center = true
                );

    color([0.25, 0.25, 0.28])
        translate([roller_center_x, motor_y, roller_center_z])
            rotate([90, 0, 0])
                cylinder(
                    h = motor_body_length,
                    d = motor_body_diameter,
                    center = true
                );

    // 67 mm tennis-ball reference showing the intended roller compression.
    color([0.72, 0.90, 0.12, 0.70])
        translate([
            roller_center_x,
            0,
            ramp_z(roller_center_x) + 67 / 2
        ])
            sphere(d = 67);
}

module complete_scoop() {
    union() {
        curved_shell();

        if (edge_guides) {
            side_guide(-scoop_width / 2 + guide_thickness / 2);
            side_guide( scoop_width / 2 - guide_thickness / 2);
        }

        if (mounting_rail)
            rear_mounting_rail();
    }
}

module selected_part() {
    if (render_part == "left") {
        intersection() {
            complete_scoop();
            translate([-1, -scoop_width / 2 - 1, -1])
                cube([
                    scoop_length + 12,
                    scoop_width / 2 + 1,
                    scoop_height + guide_height + 2
                ]);
        }
    } else if (render_part == "right") {
        intersection() {
            complete_scoop();
            translate([-1, 0, -1])
                cube([
                    scoop_length + 12,
                    scoop_width / 2 + 1,
                    scoop_height + guide_height + 2
                ]);
        }
    } else if (render_part == "ear") {
        standalone_mounting_ear();
    } else if (render_part == "roller") {
        collector_roller();
    } else if (render_part == "assembly") {
        complete_scoop();
        mounting_ear_at(-scoop_width / 2 + mount_ear_width / 2);
        mounting_ear_at( scoop_width / 2 - mount_ear_width / 2);
        roller_drive_assembly();
    } else {
        complete_scoop();
    }
}

selected_part();
