// Perfboard-side 1x2 / 1x4 holder for individual MALE Dupont jumper ends.
// Matching keyed collar surrounds a soldered FEMALE header on the perfboard.
// Fit prototype only: verify the real jumper housings and pin engagement.
// All dimensions in millimetres.

$fn = 48;

contacts = 2; // [2,4]
part = "plug_lower"; // [plug_lower,plug_upper,board_collar,assembly,exploded]

pitch = 2.54;
pin_aperture = 1.35;
front_lip = 1.20;

// Generic 1P Dupont housing dimensions, NOT verified for the purchased wires.
single_x = 2.54;
single_y = 2.54;
single_z = 14.00;
pack_fit_x = 0.50;
pack_fit_y = 0.50;
pack_fit_z = 0.50;

body_x = contacts*single_x + pack_fit_x;
body_y = single_y + pack_fit_y;
nose_x = body_x + 2.40;
nose_y = body_y + 2.40;
rear_stop = 2.60;
nose_z = front_lip + single_z + pack_fit_z + rear_stop;

rear_x = nose_x + 4.00;
rear_y = 8.00;
rear_z = 10.00;
total_z = nose_z + rear_z;
rear_corner_r = 1.50;

wire_slot_x = 1.55;
wire_slot_y = 2.20;
rear_chamber_x = body_x + 0.80;
rear_chamber_y = 4.40;
rear_chamber_z = 6.00;
cable_exit_d = contacts == 2 ? 3.50 : 5.20;

tie_groove_depth = 0.55;
tie_groove_width = 2.40;
tie_z = nose_z + 4.00;

key_width = 1.40;
key_projection = 1.10;
key_x = contacts == 2 ? 0 : pitch;
key_side = contacts == 2 ? 1 : -1;
collar_clearance = 0.55;
collar_wall = 2.00;
collar_h = 7.50;
pin1_mark_d = 1.20;
pin1_mark_depth = 0.50;

epsilon = 0.02;

assert(contacts == 2 || contacts == 4);
assert(pin_aperture < pitch);

function contact_x(i) = (i-(contacts-1)/2)*pitch;

module pin_pattern(h=front_lip) {
    for (i = [0:contacts-1])
        translate([contact_x(i), 0, -epsilon])
            linear_extrude(height=h+2*epsilon)
                square([pin_aperture, pin_aperture], center=true);
}

module key_rib() {
    translate([key_x-key_width/2,
               key_side > 0 ? nose_y/2 : -nose_y/2-key_projection,
               0])
        cube([key_width, key_projection, nose_z]);
}

module rounded_rear() {
    hull()
        for (x = [-rear_x/2+rear_corner_r,
                   rear_x/2-rear_corner_r],
             z = [nose_z+rear_corner_r,
                  total_z-rear_corner_r])
            translate([x, 0, z])
                rotate([-90, 0, 0])
                    cylinder(r=rear_corner_r, h=rear_y, center=true);
}

module outer_plug() {
    union() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, nose_z]);
        key_rib();
        rounded_rear();
    }
}

module body_pocket() {
    translate([-body_x/2, -body_y/2, front_lip])
        cube([body_x, body_y,
              single_z+pack_fit_z+epsilon]);
}

module wire_comb() {
    // Both half-channels open at the y=0 split so a preterminated cable can
    // be laid in place without threading its other end through a small hole.
    for (i = [0:contacts-1])
        translate([contact_x(i)-wire_slot_x/2, -wire_slot_y/2,
                   front_lip+single_z+pack_fit_z-epsilon])
            cube([wire_slot_x, wire_slot_y,
                  rear_stop+2*epsilon]);
}

module rear_cable_space() {
    translate([-rear_chamber_x/2, -rear_chamber_y/2,
               nose_z-epsilon])
        cube([rear_chamber_x, rear_chamber_y,
              rear_chamber_z+2*epsilon]);
    translate([0, 0, nose_z+rear_chamber_z-epsilon])
        cylinder(d=cable_exit_d,
                 h=rear_z-rear_chamber_z+2*epsilon);
}

module tie_groove() {
    // Shallow external groove for one small cable tie across both halves.
    difference() {
        translate([-rear_x, -rear_y, tie_z])
            cube([2*rear_x, 2*rear_y, tie_groove_width]);
        translate([-rear_x/2+tie_groove_depth,
                   -rear_y/2+tie_groove_depth,
                   tie_z-epsilon])
            cube([rear_x-2*tie_groove_depth,
                  rear_y-2*tie_groove_depth,
                  tie_groove_width+2*epsilon]);
    }
}

module pin1_mark() {
    // Recessed dot over x-negative contact 1, on the y-positive half.
    translate([contact_x(0), nose_y/2+epsilon, nose_z/2])
        rotate([90, 0, 0])
            cylinder(d=pin1_mark_d, h=pin1_mark_depth+2*epsilon);
}

module whole_plug() {
    difference() {
        outer_plug();
        body_pocket();
        wire_comb();
        rear_cable_space();
        pin_pattern();
        tie_groove();
        pin1_mark();
    }
}

module lower_half() {
    intersection() {
        whole_plug();
        translate([-rear_x, -rear_y, -epsilon])
            cube([2*rear_x, rear_y, total_z+2*epsilon]);
    }
}

module upper_half() {
    intersection() {
        whole_plug();
        translate([-rear_x, 0, -epsilon])
            cube([2*rear_x, rear_y, total_z+2*epsilon]);
    }
}

module keyed_opening(h) {
    inner_x = nose_x+collar_clearance;
    inner_y = nose_y+collar_clearance;
    translate([-inner_x/2, -inner_y/2, -epsilon])
        cube([inner_x, inner_y, h+2*epsilon]);
    slot_x = key_width+collar_clearance;
    translate([key_x-slot_x/2,
               key_side > 0 ? nose_y/2 :
                              -nose_y/2-key_projection-collar_clearance/2,
               -epsilon])
        cube([slot_x, key_projection+collar_clearance/2,
              h+2*epsilon]);
}

module board_collar() {
    outer_x = nose_x+collar_clearance+2*collar_wall;
    outer_y = nose_y+collar_clearance+2*collar_wall;
    difference() {
        translate([-outer_x/2, -outer_y/2, 0])
            cube([outer_x, outer_y, collar_h]);
        keyed_opening(collar_h);
    }
}

module mockup_ends() {
    for (i = [0:contacts-1])
        color([0.12, 0.12, 0.12, 0.5])
            translate([contact_x(i)-single_x/2,
                       -single_y/2, front_lip])
                cube([single_x, single_y, single_z]);
}

if (part == "plug_lower") rotate([-90, 0, 0]) lower_half();
else if (part == "plug_upper") rotate([90, 0, 0]) upper_half();
else if (part == "board_collar") board_collar();
else if (part == "assembly") {
    lower_half();
    color([0.40, 0.75, 0.85, 0.45]) upper_half();
    mockup_ends();
} else if (part == "exploded") {
    color([0.40, 0.75, 0.85]) translate([0, 4.0, 12]) upper_half();
    color([0.45, 0.70, 0.45]) translate([0, -4.0, 12]) lower_half();
    color([0.95, 0.55, 0.22]) board_collar();
    mockup_ends();
} else assert(false, str("Unknown part: ", part));
