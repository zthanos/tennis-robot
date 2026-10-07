// BTS7960 driver-side holder for six *individual* female Dupont jumper ends.
// Two printed dummies occupy R_IS and L_IS. Mechanical fit prototype only.
// Dimensions are millimetres. Actual 1P jumper plastic must be measured.

$fn = 48;

part = "fit_gauge"; // [fit_gauge,lower,upper,dummy,assembly,exploded]
side_relief = 0.0; // v3 test uses 1.0 mm extra outward space per half

pitch = 2.54;
pin_aperture = 1.35;   // carried over from the driver gauge that fitted
front_lip = 1.60;

// Provisional dimensions for one separate 1P female Dupont plastic end.
single_x = 2.54;
single_y = 2.54;
single_z = 14.00;
pack_clearance = 0.40; // total clearance around a packed 2x4 block
pack_clearance_y = pack_clearance + 2*side_relief;

nose_x = 14.50;         // keep the v1 driver-side x envelope
nose_y = 7.80 + 2*side_relief;
rear_stop = 3.00;       // plastic behind the female ends
nose_z = front_lip + single_z + rear_stop;

rear_x = 25.00;
rear_y = 15.00;
rear_z = 18.00;
rear_corner_r = 2.50;
total_z = nose_z + rear_z;

wire_channel_x = 1.50; // provisional; verify against jumper insulation
wire_channel_y = 2.20; // open at the split so cables can be laid in place
rear_chamber_x = 13.00;
rear_chamber_y = 8.00;
rear_chamber_depth = 12.50;
cable_exit_d = 6.00;

screw_x = 9.00;
screw_z = nose_z + 7.00;
screw_clearance_d = 3.20; // M3 bolts and external nuts as in v1
mark_depth = 0.60;

dummy_x = single_x;
dummy_y = single_y;
dummy_z = single_z;
dummy_pin_hole = 1.45;

epsilon = 0.02;

function contact_x(col) = (col - 1.5)*pitch;
function contact_y(row) = (row - 0.5)*pitch;
// Column 1 in each row is the unused current-sense (IS) contact.
function used(col) = col != 1;

assert(4*single_x + pack_clearance < nose_x);
assert(2*single_y + pack_clearance_y < nose_y);

module eight_pin_pattern(h=front_lip) {
    for (col = [0:3], row = [0:1])
        translate([contact_x(col), contact_y(row), -epsilon])
            linear_extrude(height=h+2*epsilon)
                square([pin_aperture, pin_aperture], center=true);
}

module fit_gauge() {
    difference() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, front_lip]);
        eight_pin_pattern();
    }
}

module outside_shell() {
    union() {
        translate([-nose_x/2, -nose_y/2, 0])
            cube([nose_x, nose_y, nose_z]);
        hull()
            for (x = [-rear_x/2+rear_corner_r,
                       rear_x/2-rear_corner_r],
                 z = [nose_z+rear_corner_r,
                      total_z-rear_corner_r])
                translate([x, 0, z])
                    rotate([-90, 0, 0])
                        cylinder(r=rear_corner_r, h=rear_y, center=true);
    }
}

module packed_ends_pocket() {
    translate([-(4*single_x+pack_clearance)/2,
               -(2*single_y+pack_clearance_y)/2, front_lip])
        cube([4*single_x+pack_clearance,
              2*single_y+pack_clearance_y, single_z+epsilon]);
}

module wire_channels() {
    // Individual comb slots only behind the six occupied positions. Each
    // channel opens to y=0, allowing preterminated jumper wires to be laid
    // into either clamshell half without passing the male end through a hole.
    for (col = [0:3], row = [0:1]) if (used(col))
        translate([contact_x(col)-wire_channel_x/2,
                   row == 0 ? -wire_channel_y : 0,
                   front_lip+single_z-epsilon])
            cube([wire_channel_x, wire_channel_y,
                  rear_stop+2*epsilon]);
}

module rear_chamber() {
    translate([-rear_chamber_x/2, -rear_chamber_y/2,
               nose_z-epsilon])
        cube([rear_chamber_x, rear_chamber_y,
              rear_chamber_depth+2*epsilon]);
    translate([0, 0, nose_z+rear_chamber_depth-epsilon])
        cylinder(d=cable_exit_d,
                 h=rear_z-rear_chamber_depth+2*epsilon);
}

module fastener_holes() {
    for (x = [-screw_x, screw_x])
        translate([x, -rear_y/2-epsilon, screw_z])
            rotate([-90, 0, 0])
                cylinder(d=screw_clearance_d, h=rear_y+2*epsilon);
}

module orientation_mark() {
    // This is a visual mark only; the bare 2x4 driver header is symmetric.
    translate([0, rear_y/2+epsilon, nose_z+3.10])
        rotate([90, 0, 0])
            linear_extrude(height=mark_depth+2*epsilon)
                mirror([1, 0, 0])
                    text("VCC", size=3.80,
                         halign="center", valign="center");
}

module whole_shell() {
    difference() {
        outside_shell();
        packed_ends_pocket();
        wire_channels();
        rear_chamber();
        eight_pin_pattern();
        fastener_holes();
        orientation_mark();
    }
}

module lower_half() {
    intersection() {
        whole_shell();
        translate([-rear_x, -rear_y, -epsilon])
            cube([2*rear_x, rear_y, total_z+2*epsilon]);
    }
}

module upper_half() {
    intersection() {
        whole_shell();
        translate([-rear_x, 0, -epsilon])
            cube([2*rear_x, rear_y, total_z+2*epsilon]);
    }
}

module unused_is_dummy() {
    difference() {
        translate([-dummy_x/2, -dummy_y/2, 0])
            cube([dummy_x, dummy_y, dummy_z]);
        translate([-dummy_pin_hole/2, -dummy_pin_hole/2, -epsilon])
            cube([dummy_pin_hole, dummy_pin_hole,
                  dummy_z+2*epsilon]);
    }
}

module mockup_ends() {
    for (col = [0:3], row = [0:1])
        translate([contact_x(col), contact_y(row), front_lip])
            if (used(col))
                color([0.12, 0.12, 0.12, 0.6])
                    translate([-single_x/2, -single_y/2, 0])
                        cube([single_x, single_y, single_z]);
            else
                color([0.95, 0.48, 0.18]) unused_is_dummy();
}

if (part == "fit_gauge") fit_gauge();
else if (part == "lower") rotate([-90, 0, 0]) lower_half();
else if (part == "upper") rotate([90, 0, 0]) upper_half();
else if (part == "dummy") unused_is_dummy();
else if (part == "assembly") {
    lower_half();
    color([0.40, 0.75, 0.85, 0.45]) upper_half();
    mockup_ends();
} else if (part == "exploded") {
    color([0.40, 0.75, 0.85]) translate([0, 6, 0]) upper_half();
    color([0.45, 0.70, 0.45]) translate([0, -6, 0]) lower_half();
    mockup_ends();
} else assert(false, str("Unknown part: ", part));
