// Separate 20 mm spacers for the two BTS7960 boards.
// Print 8 for the installation; the default plate includes 2 spares.

include <params.scad>

$fn = 48;

part = "plate"; // [plate,single]
spacer_od = 10;
spacer_hole_d = 3.4; // clearance for the 2.8 mm printed pin or an M3 screw
spacer_count = 10;
spacer_pitch = 14;

module bts_spacer() {
    difference() {
        cylinder(d=spacer_od, h=bts_spacer_h);
        translate([0, 0, -1])
            cylinder(d=spacer_hole_d, h=bts_spacer_h+2);
    }
}

module spacer_plate() {
    cols = 5;
    for (i = [0:spacer_count-1])
        translate([(i % cols)*spacer_pitch, floor(i/cols)*spacer_pitch, 0])
            bts_spacer();
}

if (part == "single") bts_spacer();
else spacer_plate();

assert(bts_spacer_h == 20, "BTS spacer must be exactly 20 mm high");
