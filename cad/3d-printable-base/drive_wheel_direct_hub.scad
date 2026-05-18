include <common.scad>

// Direct-drive printed wheel.
// Print wheel core in PETG/ASA/nylon-CF.
// Print optional tire sleeve in TPU, or wrap the tread with rubber.

wheel_d = 170;
wheel_w = 42;
hub_d = 62;
bore_d = 8.2;          // motor shaft clearance; adjust to actual motor
bore_flat_depth = 1.2; // set 0 for round bore; >0 approximates D-shaft flat
clamp_gap = 4;
clamp_bolt_d = 3.4;   // M3 clearance
set_screw_d = 3.2;    // optional set screw pilot
tread_grooves = 18;

module d_shaft_bore() {
    difference() {
        cylinder(h=wheel_w + 4, d=bore_d);
        if (bore_flat_depth > 0) {
            translate([bore_d / 2 - bore_flat_depth, -bore_d, -2])
                cube([bore_d, bore_d * 2, wheel_w + 8]);
        }
    }
}

module wheel_core() {
    difference() {
        union() {
            cylinder(h=wheel_w, d=wheel_d);
            translate([0, 0, -4]) cylinder(h=wheel_w + 8, d=hub_d);

            // spokes
            for (a = [0:45:315]) {
                rotate([0, 0, a])
                    translate([hub_d / 2 - 3, -5, 0])
                        cube([wheel_d / 2 - hub_d / 2 - 10, 10, wheel_w]);
            }
        }

        translate([0, 0, -6]) d_shaft_bore();

        // side lightening holes
        for (a = [22.5:45:337.5]) {
            rotate([0, 0, a])
                translate([wheel_d * 0.32, 0, -1])
                    cylinder(h=wheel_w + 2, d=24);
        }

        // clamp split
        translate([-clamp_gap / 2, -hub_d / 2 - 8, -8])
            cube([clamp_gap, hub_d + 16, wheel_w + 16]);

        // clamp bolt cross holes
        for (z = [wheel_w * 0.32, wheel_w * 0.68]) {
            translate([0, hub_d / 2 - 8, z])
                rotate([0, 90, 0])
                    cylinder(h=hub_d + 12, d=clamp_bolt_d, center=true);
        }

        // tread grooves
        for (a = [0:360 / tread_grooves:359]) {
            rotate([0, 0, a])
                translate([wheel_d / 2 - 3, -2, -1])
                    cube([8, 4, wheel_w + 2]);
        }

        // optional set screw pilot, use only if needed
        translate([hub_d / 2 - 6, 0, wheel_w / 2])
            rotate([0, 90, 0])
                cylinder(h=20, d=set_screw_d);
    }
}

module tire_sleeve() {
    difference() {
        cylinder(h=wheel_w + 2, d=wheel_d + 8);
        translate([0, 0, -1]) cylinder(h=wheel_w + 4, d=wheel_d - 2);
    }
}

// Export one module at a time.
wheel_core();
// tire_sleeve();

