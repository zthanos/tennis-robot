// TPU tread vs tennis-ball felt friction test (physical μ for P2-60).
// Units mm. part = "plate" | "sled" | "pulley" | "pulley_bracket" | "assembly"
//
// plate          TPU, printed UPRIGHT (long edge on the bed) so the test face
//                is a side wall with layer lines along the pull direction,
//                like the wheel tread. Use the tread's TPU settings.
// sled           PETG. Two balls pressed into the cups, in line, so they slide
//                without rolling. Weights are strapped on the deck.
// pulley(+bracket) PETG. Turns the pull vertical so a water bottle on a string
//                can be weighed on the kitchen scale (hanging-mass method).

part = "assembly";
$fn = 72;

ball_d      = 67;     // nominal tennis ball
cup_d       = 63.5;   // press fit: felt squeezes ~1.5 mm per side
ball_pitch  = 85;     // ball centre spacing along the pull direction (X)
protrude    = 14;     // ball below the sled underside

plate_l = 230; plate_w = 50; plate_t = 4;

sled_l = 170; sled_w = 80; deck_t = 6;

pulley_d = 34; pulley_w = 10; axle_d = 4.4;   // M4 bolt as axle
eye_z   = 6;   // pull-eye height above the sled underside
// string height above the table (plate and bracket base both on the table)
string_z = plate_t + protrude + eye_z;

// ---------------------------------------------------------------- plate
module plate() {
    difference() {
        cube([plate_l, plate_w, plate_t]);
        // two tape/screw tabs' holes at the ends, outside the ball track
        for (x = [8, plate_l-8]) translate([x, plate_w/2, -1]) cylinder(d=4.5, h=plate_t+2);
    }
}
// printed upright: rotate so the 230x4 edge is on the bed
module plate_print() { rotate([90,0,0]) plate(); }

// ---------------------------------------------------------------- sled
// Underside at z=0. Ball centres at z = protrude - ball_d/2 ... i.e. the
// ball bottom sits protrude mm below the sled.
ball_cz = ball_d/2 - protrude;   // ball centre height above sled underside
cup_top = ball_cz + ball_d/2;    // ball pressed in until it touches the ceiling
sled_h  = cup_top + deck_t;      // weight goes straight into the balls

module sled() {
    difference() {
        union() {
            translate([-sled_l/2, -sled_w/2, 0]) cube([sled_l, sled_w, sled_h]);
            // pull eye at the front, low, to limit tipping
            translate([sled_l/2, 0, eye_z]) rotate([90,0,0])
                difference() { cylinder(d=20, h=8, center=true); cylinder(d=8, h=10, center=true); }
        }
        // blind ball cups from below; the ceiling sets the protrusion
        for (x = [-ball_pitch/2, ball_pitch/2]) translate([x, 0, -1]) cylinder(d=cup_d, h=cup_top+1);
        // strap slots for weights
        for (x = [-sled_l/2+8, sled_l/2-8]) translate([x, 0, sled_h-12])
            cube([4, sled_w+2, 8], center=true);
    }
}

// ---------------------------------------------------------------- pulley
module pulley() {
    difference() {
        rotate_extrude() polygon([
            [axle_d/2, 0], [pulley_d/2, 0], [pulley_d/2, 2],
            [pulley_d/2-4, pulley_w/2], [pulley_d/2, pulley_w-2],
            [pulley_d/2, pulley_w], [axle_d/2, pulley_w]]);
    }
}
// Base sits on the table top with its end flush with the table edge; clamp or
// tape it. The pulley top is at string_z, so the string from the sled eye runs
// level, then drops past the edge to the water bottle.
module pulley_bracket() {
    base_l = 90; base_w = 50; base_t = 6; ear_t = 5; gap = pulley_w + 1.5;
    axle_x = base_l + 12; axle_z = string_z - pulley_d/2;
    difference() {
        union() {
            cube([base_l, base_w, base_t]);
            for (y = [base_w/2 - gap/2 - ear_t, base_w/2 + gap/2])
                hull() {
                    translate([base_l-40, y, 0]) cube([40, ear_t, base_t]);
                    translate([axle_x, y, axle_z]) rotate([-90,0,0]) cylinder(d=2*axle_z, h=ear_t);
                }
        }
        translate([axle_x, -1, axle_z]) rotate([-90,0,0]) cylinder(d=axle_d, h=base_w+2);
        for (x = [15, 50]) translate([x, base_w/2, -1]) cylinder(d=6, h=base_t+2); // clamp/screw holes
    }
}

// ---------------------------------------------------------------- views
module assembly() {
    color([0.15,0.15,0.15]) plate();
    translate([plate_l/2, plate_w/2, plate_t + protrude]) {
        color([0.85,0.45,0.10]) sled();
        for (x = [-ball_pitch/2, ball_pitch/2])
            color([0.80,0.95,0.20]) translate([x, 0, ball_cz]) sphere(d=ball_d);
    }
}

if (part == "plate") plate_print();
else if (part == "sled") sled();
else if (part == "pulley") pulley();
else if (part == "pulley_bracket") pulley_bracket();
else assembly();
