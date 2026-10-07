// BTS7960 / IBT-2 logic plug, first physical-fit prototype (millimetres).
//
// A standard one-piece 2x4, 2.54 mm PCB female header is soldered directly
// to five logic wires and trapped between two printed halves. No perfboard.
// This is NOT keyed yet. Do not use it on an energised motor until a matching
// driver-side key and the actual pinout have been verified.

$fn = 48;

part = "fit_gauge"; // [fit_gauge,lower,upper,assembly,exploded]

pitch = 2.54;
pin_hole = 1.35;          // test aperture; enlarge only if the gauge binds
front_lip = 1.60;         // printed material before the female header face

// Provisional outside envelope of a 2x4 female PCB header. Adjust from the
// first physical print; different header brands have different plastic bodies.
header_x = 13.00;          // user's approximate overall 2x4 body length
header_y = 4.50;           // user's approximate overall 2x4 body width
header_z = 8.50;
header_fit = 0.40;        // total diametral clearance, not per side

nose_x = 16.50;
nose_y = 7.80;
nose_z = front_lip + header_z + 1.00;

rear_x = 25.00;           // screw flanges are behind the driver-side nose
rear_y = 15.00;
rear_z = 18.00;
total_z = nose_z + rear_z;

solder_x = 14.00;
solder_y = 9.20;
solder_depth = 13.00;
cable_exit_d = 5.50;     // for a bundled, sleeved five-wire harness

screw_x = 9.00;
screw_z = nose_z + 7.00;
screw_clearance_d = 2.30; // two M2 through-bolts with nuts
nut_pocket_d = 4.90;     // hex circumscribed diameter; ~4.2 mm across flats
nut_pocket_depth = 2.30;

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
        translate([-rear_x/2, -rear_y/2, nose_z])
            cube([rear_x, rear_y, rear_z]);
    }
}

module cable_bore() {
    translate([0, 0, nose_z + solder_depth - epsilon])
        cylinder(d=cable_exit_d, h=rear_z - solder_depth + 2*epsilon);
}

module fastener_holes() {
    for (x = [-screw_x, screw_x]) {
        // A through-hole perpendicular to the split plane.
        translate([x, -rear_y/2 - epsilon, screw_z])
            rotate([-90, 0, 0])
                cylinder(d=screw_clearance_d, h=rear_y + 2*epsilon);
        // Captive nut recess in the outside face of the lower half.
        translate([x, -rear_y/2 - epsilon, screw_z])
            rotate([-90, 0, 0])
                cylinder(d=nut_pocket_d, h=nut_pocket_depth + epsilon,
                         $fn=6);
    }
}

module whole_shell() {
    difference() {
        outside_shell();

        // The header slides in from the cable side and stops at the front lip.
        translate([-(header_x + header_fit)/2,
                   -(header_y + header_fit)/2,
                   front_lip])
            cube([header_x + header_fit,
                  header_y + header_fit,
                  header_z + 1.10]);

        // Chamber for the solder tails, insulated joints and the EN bridge.
        translate([-solder_x/2, -solder_y/2, nose_z - epsilon])
            cube([solder_x, solder_y, solder_depth + 2*epsilon]);

        cable_bore();
        eight_pin_pattern(front_lip);
        fastener_holes();
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
// Exported halves sit on their split faces, ready for the slicer bed.
else if (part == "lower") rotate([-90, 0, 0]) lower_half();
else if (part == "upper") rotate([90, 0, 0]) upper_half();
else if (part == "assembly") {
    lower_half();
    color([0.40, 0.75, 0.85, 0.65]) upper_half();
} else if (part == "exploded") {
    color([0.40, 0.75, 0.85]) translate([0, 6, 0]) upper_half();
    color([0.45, 0.70, 0.45]) translate([0, -6, 0]) lower_half();
    // Reference volume only: the physical female header is not printed.
    %translate([-header_x/2, -header_y/2, front_lip])
        cube([header_x, header_y, header_z]);
} else assert(false, str("Unknown part: ", part));
