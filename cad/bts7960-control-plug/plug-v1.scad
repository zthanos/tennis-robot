// BTS7960 / IBT-2 logic plug, v1 fit prototype (millimetres).
//
// v1: the complete driver-side nose is 1 mm narrower on each x side than v0,
// and fasteners are M3 bolts with external nuts. With an approximately 13 mm
// female-header body, the x-side walls are only about 0.55 mm: fit trial only.
// This is NOT keyed yet. Do not use with an energised motor.

$fn = 48;

part = "fit_gauge"; // [fit_gauge,lower,upper,assembly,exploded]

pitch = 2.54;
pin_hole = 1.35;         // confirmed by the first gauge fit on the driver
front_lip = 1.60;

header_x = 13.00;        // approximate complete 2x4 female-header body
header_y = 4.50;
header_z = 8.50;         // unmeasured; adjust after checking the female header
header_fit = 0.40;       // total clearance, not per side

nose_x = 14.50;          // v0 was 16.50: -1 mm at left and right, all along nose
nose_y = 7.80;
nose_z = front_lip + header_z + 1.00;

rear_x = 25.00;          // retained for adequate M3 flange wall thickness
rear_y = 15.00;
rear_z = 18.00;
rear_corner_r = 2.50;   // external x-z corners only; functional cuts unchanged
total_z = nose_z + rear_z;

solder_x = 12.00;        // smaller than v0, still wider than 7.62 mm pin span
solder_y = 9.20;
solder_depth = 13.00;
cable_exit_d = 5.50;

screw_x = 9.00;
screw_z = nose_z + 7.00;
screw_clearance_d = 3.20; // two M3 through-bolts, external nuts
mark_depth = 0.60;

epsilon = 0.02;

module eight_pin_pattern(h=1, aperture=pin_hole) {
    for (col = [0:3], row = [0:1])
        translate([(col - 1.5)*pitch, (row - 0.5)*pitch, -epsilon])
            linear_extrude(height=h + 2*epsilon)
                square([aperture, aperture], center=true);
}

module fit_gauge() {
    difference() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, front_lip]);
        eight_pin_pattern(front_lip);
    }
}

module outside_shell() {
    union() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, nose_z]);
        // Round the visible outline of the wide rear/upper housing. Cylinders
        // run along y, so screw axes, mating face and header cavity stay put.
        hull()
            for (x = [-rear_x/2 + rear_corner_r,
                       rear_x/2 - rear_corner_r],
                 z = [nose_z + rear_corner_r,
                      total_z - rear_corner_r])
                translate([x, 0, z])
                    rotate([-90, 0, 0])
                        cylinder(r=rear_corner_r, h=rear_y, center=true);
    }
}

module cable_bore() {
    translate([0, 0, nose_z + solder_depth - epsilon])
        cylinder(d=cable_exit_d, h=rear_z - solder_depth + 2*epsilon);
}

module fastener_holes() {
    for (x = [-screw_x, screw_x])
        translate([x, -rear_y/2 - epsilon, screw_z])
            rotate([-90, 0, 0])
                cylinder(d=screw_clearance_d, h=rear_y + 2*epsilon);
}

module orientation_mark() {
    // Recessed on the outside of the y-positive half. When wiring the plug,
    // this side must contain the VCC / R_IS / R_EN / RPWM socket row.
    translate([0, rear_y/2 + epsilon, nose_z + 3.10])
        rotate([90, 0, 0])
            linear_extrude(height=mark_depth + 2*epsilon)
                mirror([1, 0, 0])
                    text("VCC", size=3.80, halign="center", valign="center");
}

module whole_shell() {
    difference() {
        outside_shell();

        translate([-(header_x + header_fit)/2,
                   -(header_y + header_fit)/2,
                   front_lip])
            cube([header_x + header_fit,
                  header_y + header_fit,
                  header_z + 1.10]);

        // Insulated solder tails, five-wire harness and the EN bridge.
        translate([-solder_x/2, -solder_y/2, nose_z - epsilon])
            cube([solder_x, solder_y, solder_depth + 2*epsilon]);

        cable_bore();
        eight_pin_pattern(front_lip);
        fastener_holes();
        orientation_mark();
    }
}

module lower_half() {
    intersection() {
        whole_shell();
        translate([-rear_x, -rear_y, -epsilon])
            cube([2*rear_x, rear_y, total_z + 2*epsilon]);
    }
}

module upper_half() {
    intersection() {
        whole_shell();
        translate([-rear_x, 0, -epsilon])
            cube([2*rear_x, rear_y, total_z + 2*epsilon]);
    }
}

if (part == "fit_gauge") fit_gauge();
else if (part == "lower") rotate([-90, 0, 0]) lower_half();
else if (part == "upper") rotate([90, 0, 0]) upper_half();
else if (part == "assembly") {
    lower_half();
    color([0.40, 0.75, 0.85, 0.65]) upper_half();
} else if (part == "exploded") {
    color([0.40, 0.75, 0.85]) translate([0, 6, 0]) upper_half();
    color([0.45, 0.70, 0.45]) translate([0, -6, 0]) lower_half();
    %translate([-header_x/2, -header_y/2, front_lip])
        cube([header_x, header_y, header_z]);
} else assert(false, str("Unknown part: ", part));
