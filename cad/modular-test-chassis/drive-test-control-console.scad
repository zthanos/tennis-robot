// Temporary control console for the physical drive test (v3 chassis).
// Holds the 22 mm E-stop (NC -> D33), the covered toggle switch and the
// START/ARM push button (D32). Bolts to the top M4 grid of
// rail_extension_left at global X100/X125 (25 mm pitch, rail centreline).
// Not the final location: the body-panel concept puts these in a rear console.
//
// Local frame: X along the rail (global X = x + 45), Y across the rail
// (+Y outboard), Z up from the rail top. Units mm.
//
// Print: lying on the outer wall (part="print"), inner side up. All walls are
// then vertical, the holes are horizontal teardrops and nothing needs support.
// Bolting: M4x40 up from under the rail, washer + nut on the foot inside the
// console (reach through the open inner side).

part = "assembly"; // [assembly, print, fit_coupon]
$fn = 64;

// Measured by the user 2026-10-16 (body below the panel):
estop_body   = [35, 30];   // contact-block footprint
toggle_body  = [27, 15];
button_body_d = 15;

// Panel holes. Defaults are the usual sizes for these parts; print
// part="fit_coupon" first and adjust here if a part is loose or tight.
estop_hole  = 22.5;
toggle_hole = 12.3;
button_hole = 16.3; // R16-504BD, ordered-parts G7

L = 140; W = 50; H = 62;        // console envelope, H = rail top to panel top
wall = 3; panel_t = 3; foot_t = 4;
foot_x0 = 40; foot_x1 = 95;
grid_x = [55, 80];              // global X100 / X125
m4_clear = 4.5;

// Component centres along X on the panel.
button_x = 22; toggle_x = 60; estop_x = 110;

// Teardrop hole along Z whose point faces -Y (print "up" in part="print").
module teardrop_z(d, h) {
    linear_extrude(height = h, center = true)
        union() {
            circle(d = d);
            rotate(45 + 180) square(d / 2);   // 45 deg roof toward -Y
        }
}

module console() {
    difference() {
        union() {
            translate([0, -W/2, H - panel_t]) cube([L, W, panel_t]);        // panel
            translate([0, W/2 - wall, 0]) cube([L, wall, H]);              // outer wall
            for (x = [0, L - wall]) translate([x, -W/2, 0]) cube([wall, W, H]); // end walls
            translate([foot_x0, -W/2, 0]) cube([foot_x1 - foot_x0, W, foot_t]); // bolt foot
        }
        // Panel holes.
        translate([button_x, 0, H - panel_t/2]) teardrop_z(button_hole, panel_t + 2);
        translate([toggle_x, 0, H - panel_t/2]) teardrop_z(toggle_hole, panel_t + 2);
        translate([estop_x,  0, H - panel_t/2]) teardrop_z(estop_hole,  panel_t + 2);
        // M4 grid bolts.
        for (x = grid_x) translate([x, 0, foot_t/2]) teardrop_z(m4_clear, foot_t + 2);
        // Cable exit through the outer wall near the rail top.
        translate([L/2, W/2 - wall/2, 14]) cube([30, wall + 2, 12], center = true);
    }
}

// Ghost bodies under the panel, to check that nothing collides.
module bodies() {
    %translate([estop_x, 0, H - panel_t - 50/2]) cube([estop_body[0], estop_body[1], 50], center = true);
    %translate([toggle_x, 0, H - panel_t - 30/2]) cube([toggle_body[0], toggle_body[1], 30], center = true);
    %translate([button_x, 0, H - panel_t - 25/2]) cylinder(d = button_body_d, h = 25, center = true);
}

// Small plate with the three panel holes, to check fit before the console.
module fit_coupon() {
    difference() {
        translate([0, -15, 0]) cube([130, 30, panel_t]);
        translate([15, 0, panel_t/2])  teardrop_z(button_hole, panel_t + 2);
        translate([45, 0, panel_t/2])  teardrop_z(toggle_hole, panel_t + 2);
        translate([95, 0, panel_t/2])  teardrop_z(estop_hole,  panel_t + 2);
    }
}

if (part == "print") rotate([-90, 0, 0]) translate([0, -W/2, 0]) console();
else if (part == "fit_coupon") fit_coupon();
else { console(); bodies(); }

// Checks: E-stop body (assumed 50 mm deep) must clear the bolt foot.
assert(H - panel_t - 50 >= foot_t || estop_x - estop_body[0]/2 >= foot_x1,
       "E-stop body hits the bolt foot");
assert(estop_x + 40/2 <= L, "E-stop mushroom overhangs the console end");
