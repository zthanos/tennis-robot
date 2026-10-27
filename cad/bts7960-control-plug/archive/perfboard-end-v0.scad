// Keyed, separated 2-pin and 3-pin logic connectors for the perfboard end.
// First FIT PROTOTYPE ONLY. All dimensions are millimetres.
//
// The cable plug traps a cut 1xN PCB female header between two printed halves.
// Two small cable ties around the rear grooves keep the halves together.
// A matching shroud must be fixed around the male header on the perfboard;
// the key cannot prevent reverse/cross mating against bare male pins.

$fn = 40;

contacts = 2; // [2,3]
part = "plug_lower"; // [plug_lower,plug_upper,board_shroud,exploded]

pitch = 2.54;
pin_hole = 1.35;
front_lip = 1.20;

// Female-header plastic dimensions are unmeasured. 2.84 mm extra length
// extrapolates from the approximate 13 mm overall 2x4 body already provided.
// Alter these after a physical trial with the actual 1x2 and 1x3 strips.
header_x = contacts*pitch + 2.84;
header_y = 2.70;
header_z = 8.50;
header_fit = 0.40;       // total clearance, not per side

nose_x = header_x + header_fit + 2.20;
nose_y = header_y + header_fit + 2.40;
nose_z = 9.00;

rear_x = nose_x + 2.00;
rear_y = 8.20;
rear_z = 13.00;
total_z = nose_z + rear_z;

solder_x = header_x + header_fit + 0.80;
solder_y = 4.50;
solder_z = 9.50;
cable_exit_d = contacts == 2 ? 3.60 : 4.40;

tie_groove_depth = 0.55;
tie_groove_width = 2.60;
tie_z_positions = [nose_z + 2.00, nose_z + 8.00];

key_width = 1.45;
key_projection = 1.10;
key_x = contacts == 2 ? 0 : pitch;
key_side = contacts == 2 ? 1 : -1;

shroud_clearance = 0.55;
shroud_wall = 2.00;
shroud_h = 7.50;

epsilon = 0.02;

module pin_pattern(h) {
    for (i = [0:contacts-1])
        translate([(i - (contacts-1)/2)*pitch, 0, -epsilon])
            linear_extrude(height=h + 2*epsilon)
                square([pin_hole, pin_hole], center=true);
}

module key_rib() {
    translate([key_x - key_width/2,
               key_side > 0 ? nose_y/2 : -nose_y/2 - key_projection,
               0])
        cube([key_width, key_projection, nose_z]);
}

module rear_segment(z0, zlen, recessed=false) {
    depth = recessed ? tie_groove_depth : 0;
    translate([-rear_x/2 + depth, -rear_y/2 + depth, z0])
        cube([rear_x - 2*depth, rear_y - 2*depth, zlen]);
}

module outer_plug() {
    union() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, nose_z]);
        key_rib();

        rear_segment(nose_z, 2.00);
        rear_segment(tie_z_positions[0], tie_groove_width, true);
        rear_segment(tie_z_positions[0] + tie_groove_width, 3.40);
        rear_segment(tie_z_positions[1], tie_groove_width, true);
        rear_segment(tie_z_positions[1] + tie_groove_width,
                     rear_z - 8.00 - tie_groove_width);
    }
}

module whole_plug() {
    difference() {
        outer_plug();
        // Header body stops against the front lip; the rear chamber provides
        // space for insulated solder tails and a small cable-stop tie.
        translate([-(header_x + header_fit)/2,
                   -(header_y + header_fit)/2, front_lip])
            cube([header_x + header_fit,
                  header_y + header_fit,
                  header_z + 0.80]);
        translate([-solder_x/2, -solder_y/2, nose_z - epsilon])
            cube([solder_x, solder_y, solder_z + 2*epsilon]);
        translate([0, 0, nose_z + solder_z - epsilon])
            cylinder(d=cable_exit_d,
                     h=rear_z - solder_z + 2*epsilon);
        pin_pattern(front_lip);
    }
}

module lower_half() {
    intersection() {
        whole_plug();
        translate([-rear_x, -rear_y, -epsilon])
            cube([2*rear_x, rear_y, total_z + 2*epsilon]);
    }
}

module upper_half() {
    intersection() {
        whole_plug();
        translate([-rear_x, 0, -epsilon])
            cube([2*rear_x, rear_y, total_z + 2*epsilon]);
    }
}

module keyed_opening(h) {
    inner_x = nose_x + shroud_clearance;
    inner_y = nose_y + shroud_clearance;
    // Main nose passage.
    translate([-inner_x/2, -inner_y/2, -epsilon])
        cube([inner_x, inner_y, h + 2*epsilon]);
    // Variant-specific keyway. 2-pin: centred on +y. 3-pin: offset on -y.
    slot_w = key_width + shroud_clearance;
    translate([key_x - slot_w/2,
               key_side > 0 ? nose_y/2 : -nose_y/2 - key_projection
                                            - shroud_clearance/2,
               -epsilon])
        cube([slot_w, key_projection + shroud_clearance/2,
              h + 2*epsilon]);
}

module board_shroud() {
    outer_x = nose_x + shroud_clearance + 2*shroud_wall;
    outer_y = nose_y + shroud_clearance + 2*shroud_wall;
    difference() {
        translate([-outer_x/2, -outer_y/2, 0])
            cube([outer_x, outer_y, shroud_h]);
        keyed_opening(shroud_h);
    }
}

if (part == "plug_lower") rotate([-90, 0, 0]) lower_half();
else if (part == "plug_upper") rotate([90, 0, 0]) upper_half();
else if (part == "board_shroud") board_shroud();
else if (part == "exploded") {
    color([0.30, 0.75, 0.85]) translate([0, 5.0, 12]) upper_half();
    color([0.40, 0.70, 0.45]) translate([0, -5.0, 12]) lower_half();
    color([0.9, 0.55, 0.25]) board_shroud();
    %translate([-header_x/2, -header_y/2, 12 + front_lip])
        cube([header_x, header_y, header_z]);
} else assert(false, str("Unknown part: ", part));
