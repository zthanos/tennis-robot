// Low-cost calibration print for the approximate GB37 motor measurements.
// Print flat with no supports. This is a gauge, not a working motor mount.

include <params.scad>

ring_wall = 3.5;
ring_height = 10;
ring_opening = 9;
ring_spacing = 48;
label_height = 0.6;

shaft_plate_size = [58, 24, 6];

module c_ring(inner_d) {
    difference() {
        cylinder(d=inner_d + 2 * ring_wall, h=ring_height);
        translate([0, 0, -0.1])
            cylinder(d=inner_d, h=ring_height + 0.2);
        // Opening makes the gauge tolerant and lets it clip around the body.
        translate([0, -ring_opening / 2, -0.1])
            cube([inner_d, ring_opening, ring_height + 0.2]);
    }
    translate([-7, -inner_d / 2 - ring_wall - 5, ring_height])
        linear_extrude(label_height)
            text(str(inner_d), size=4, halign="center", valign="center");
}

module d_shaft_cutter(d, flat_depth, h) {
    intersection() {
        cylinder(d=d, h=h);
        translate([-d, -d, 0])
            cube([d + d / 2 - flat_depth, 2 * d, h]);
    }
}

module shaft_fit_plate() {
    difference() {
        cube(shaft_plate_size);
        for (i = [0 : len(shaft_fit_diameters) - 1]) {
            x = 12 + i * 17;
            translate([x, shaft_plate_size[1] / 2, -0.1])
                d_shaft_cutter(
                    shaft_fit_diameters[i],
                    shaft_flat_depth_provisional,
                    shaft_plate_size[2] + 0.2
                );
        }
    }
    for (i = [0 : len(shaft_fit_diameters) - 1]) {
        x = 12 + i * 17;
        translate([x, 3.2, shaft_plate_size[2]])
            linear_extrude(label_height)
                text(str(shaft_fit_diameters[i]), size=3,
                     halign="center", valign="center");
    }
}

for (i = [0 : len(motor_fit_diameters) - 1])
    translate([i * ring_spacing, 0, 0])
        c_ring(motor_fit_diameters[i]);

translate([55, 36, 0]) shaft_fit_plate();
