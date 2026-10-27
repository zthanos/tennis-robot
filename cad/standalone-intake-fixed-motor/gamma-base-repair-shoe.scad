// Bolt-on repair shoe for a Γ bracket whose original chassis foot fractured.
// Units: mm; compact robot ground frame (+X forward).
//
// The remaining vertical frame seats in the 8.8 mm channel at its original
// Z=60 datum. Two transverse M4 through-bolts pass through the repair socket
// just above the existing bottom header. They act as retaining cross-pins, so
// the surviving Γ does not need to be drilled or weakened further.

$fn = 64;

use <dual-outboard-gamma-brackets.scad>

part = "left_print"; // [left,right,left_print,right_print,assembly_check,pin_gamma_intersection,base_access_obstruction]

base_x0 = 330;
base_x1 = 455;
base_inner_y = 175;
base_outer_y = 245;
base_z0 = 52;
base_t = 12;

gamma_wall_y = 205;
gamma_wall_t = 8;
channel_clearance = 0.8;
channel_w = gamma_wall_t + channel_clearance;
channel_x0 = 392;
channel_x1 = base_x1 + 2; // open front end; the Γ continues beyond the foot
channel_floor_z = 60;

socket_wall_t = 6;
socket_top_z = 94;
bolt_clear_d = 4.5; // M4 clearance
base_clear_d = 5.5; // existing M5 chassis pattern
base_access_d = 15; // socket/washer access above the intact 5.5 mm land
base_hole_x = [345, 440];
base_hole_y = [188, 232];

// The original lower header occupies Z=60..72. Both bolts pass through the
// open region immediately above it and trap the complete header underneath.
repair_bolts = [
    [430, 76],
    [450, 76]
];

module transverse_hole(x, z, y=gamma_wall_y, length=32) {
    translate([x, y, z])
        rotate([90, 0, 0]) cylinder(d=bolt_clear_d, h=length, center=true);
}

module repair_pin_envelopes() {
    for (position = repair_bolts)
        transverse_hole(position[0], position[1]);
}

module base_access_envelopes() {
    for (xx = base_hole_x, yy = base_hole_y)
        translate([xx, yy, base_z0+base_t-0.01])
            cylinder(d=base_access_d,
                     h=socket_top_z-(base_z0+base_t)+2);
}

module base_access_probes() {
    for (xx = base_hole_x, yy = base_hole_y)
        translate([xx, yy, base_z0+base_t+0.2])
            cylinder(d=base_access_d-0.4,
                     h=socket_top_z-(base_z0+base_t));
}

module socket_side(y_sign=1) {
    side_y = gamma_wall_y
        + y_sign * (channel_w/2 + socket_wall_t/2);
    translate([(channel_x0+base_x1)/2,
               side_y,
               (channel_floor_z+socket_top_z)/2])
        cube([base_x1-channel_x0,
              socket_wall_t,
              socket_top_z-channel_floor_z], center=true);
}

module socket_gusset(x, y_sign=1) {
    side_y = gamma_wall_y
        + y_sign * (channel_w/2 + socket_wall_t);
    hull() {
        translate([x, side_y, base_z0+base_t/2])
            cube([18, 18, base_t], center=true);
        translate([x, side_y-y_sign*3, socket_top_z-5])
            cube([18, 6, 10], center=true);
    }
}

module repair_shoe_positive() {
    union() {
        // Full bearing plate: the M5 holes no longer break out through rails.
        translate([(base_x0+base_x1)/2,
                   (base_inner_y+base_outer_y)/2,
                   base_z0+base_t/2])
            cube([base_x1-base_x0,
                  base_outer_y-base_inner_y,
                  base_t], center=true);

        socket_side(-1);
        socket_side(1);

        // Four load-spreading ribs around the two retaining-pin regions.
        for (xx = [430, 450], sy = [-1, 1])
            socket_gusset(xx, sy);
    }
}

module repair_shoe_positive_side() {
    difference() {
        repair_shoe_positive();

        // Recess the surviving Γ wall 4 mm into the new full base, while
        // preserving its original lower datum at Z=60.
        translate([(channel_x0+channel_x1)/2,
                   gamma_wall_y,
                   (channel_floor_z+socket_top_z+2)/2])
            cube([channel_x1-channel_x0,
                  channel_w,
                  socket_top_z+2-channel_floor_z], center=true);

        for (xx = base_hole_x, yy = base_hole_y)
            translate([xx, yy, base_z0-1])
                cylinder(d=base_clear_d, h=base_t+2);

        // Keep every chassis fastener reachable even where a socket gusset
        // passes above it. The 15 mm well begins at the plate's top surface;
        // the load-bearing plate below retains the original 5.5 mm M5 hole.
        base_access_envelopes();

        repair_pin_envelopes();
    }
}

module repair_shoe(side=1) {
    if (side > 0) repair_shoe_positive_side();
    else mirror([0, 1, 0]) repair_shoe_positive_side();
}

module surviving_gamma(side=1) {
    // Use the actual first prototype STL that was printed and fractured. Undo
    // its left_print placement to recover the compact robot world coordinates.
    intersection() {
        if (side > 0)
            translate([330, 69, 52])
                import("stl/gamma_bracket_left_prototype.stl");
        else
            mirror([0, 1, 0])
                translate([330, 69, 52])
                    import("stl/gamma_bracket_left_prototype.stl");
        translate([300, -270, channel_floor_z])
            cube([240, 540, 170]);
    }
}

module print_oriented(side=1) {
    if (side > 0)
        translate([-base_x0, -base_inner_y, -base_z0]) repair_shoe(1);
    else
        translate([-base_x0, base_outer_y, -base_z0]) repair_shoe(-1);
}

if (part == "left") repair_shoe(1);
else if (part == "right") repair_shoe(-1);
else if (part == "left_print") print_oriented(1);
else if (part == "right_print") print_oriented(-1);
else if (part == "assembly_check") {
    color([0.12, 0.55, 0.90]) repair_shoe(1);
    color([0.78, 0.26, 0.08, 0.55]) surviving_gamma(1);
}
else if (part == "pin_gamma_intersection")
    intersection() {
        repair_pin_envelopes();
        surviving_gamma(1);
    }
else if (part == "base_access_obstruction")
    intersection() {
        repair_shoe(1);
        base_access_probes();
    }

assert(channel_w > gamma_wall_t,
       "repair channel must include positive fit clearance");
assert(channel_floor_z < base_z0 + base_t,
       "repair channel must be recessed into the full base");
assert(base_x1 <= 460,
       "repair foot must stay behind compact chassis X=460");
