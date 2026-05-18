include <common.scad>

// Printable stabilizer foot. Use two at the front of the base.
// It is meant to touch the court during launch and lift during trolley transport.

foot_len = 130;
foot_w = 58;
foot_h = 20;
leg_h = 72;
mount_hole_d = 5.2;
printed_pin_d = 10;

module stabilizer_foot() {
    difference() {
        union() {
            rounded_plate([foot_len, foot_w, foot_h], 14);

            // vertical lug for printed hinge pin or shoulder bolt
            translate([foot_len / 2 - 18, foot_w / 2 - 8, foot_h])
                cube([36, 16, leg_h]);

            // wider top tab
            translate([foot_len / 2 - 36, foot_w / 2 - 18, foot_h + leg_h - 14])
                rounded_plate([72, 36, 14], 8);
        }

        // hinge pin hole
        translate([foot_len / 2, foot_w / 2, foot_h + leg_h - 7])
            rotate([90, 0, 0])
                cylinder(h=foot_w + 8, d=printed_pin_d, center=true);

        // rubber pad screw holes
        for (x = [30, foot_len - 30]) {
            translate([x, foot_w / 2, -1]) screw_hole(mount_hole_d, foot_h + 2);
        }

        // underside shallow pocket for rubber strip
        translate([16, 10, -1]) rounded_plate([foot_len - 32, foot_w - 20, 5], 8);
    }
}

module printed_hinge_pin() {
    cylinder(h=foot_w + 14, d=printed_pin_d - 0.4);
    translate([0, 0, -2]) cylinder(h=2, d=printed_pin_d + 5);
    translate([0, 0, foot_w + 14]) cylinder(h=2, d=printed_pin_d + 5);
}

stabilizer_foot();
// translate([170, 0, 0]) printed_hinge_pin();

