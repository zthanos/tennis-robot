// SUPERSEDED BEFORE PRINT — DO NOT PRINT.
// Unified full-width retainer fit test for the populated electronics tray.
// Replaced by vertical-service-stand-front-retainer-fit-test.scad after the
// installation direction and screw-tab geometry were clarified.

$fn = 32;

tray_w = 180;
tray_t = 4;
upright_t = 8;

wall = 2.4;
width_clearance = 0.8;
depth_clearance = 0.7;
clip_h = 10;

inside_w = tray_w + width_clearance;
outside_w = inside_w + 2 * wall;
inside_depth = tray_t + upright_t + depth_clearance;
outside_depth = inside_depth + 2 * wall;

// Both populated edge bays leave 6 mm between the tray edge and the PCB.
// A 4 mm front tab therefore retains 2 mm nominal PCB clearance.
front_tab = 4;
retention_rib = 0.3;

// Approximate support centres relative to the left tray edge. Rear pads press
// only over the two black uprights; the connecting rear beam distributes load.
support_centres = [20, 160];
support_pad_w = 20;

module left_front_tab() {
    // Angled inner edge acts as a lead-in while the frame is pushed from the
    // rear toward the populated tray.
    linear_extrude(height = clip_h)
        polygon([
            [0, 0],
            [wall + front_tab, 0],
            [wall, wall],
            [0, wall]
        ]);
}

module unified_retainer_fit_test() {
    union() {
        // Full rear beam and two outer side walls form the main U.
        translate([0, wall + inside_depth, 0])
            cube([outside_w, wall, clip_h]);
        cube([wall, outside_depth, clip_h]);
        translate([outside_w - wall, 0, 0])
            cube([wall, outside_depth, clip_h]);

        // Short front tabs touch only the empty 6 mm tray-edge margins.
        left_front_tab();
        translate([outside_w, 0, 0]) mirror([1, 0, 0]) left_front_tab();

        // Front friction ribs stay inside the same clear edge margins.
        translate([wall + 0.8, wall - 0.1, 1.5])
            cube([front_tab - 1.1, retention_rib + 0.1, clip_h - 3]);
        translate([outside_w - wall - front_tab + 0.3,
                   wall - 0.1, 1.5])
            cube([front_tab - 1.1, retention_rib + 0.1, clip_h - 3]);

        // Rear pads remove clearance only above the two support locations.
        for (cx = support_centres)
            translate([wall + width_clearance / 2 + cx - support_pad_w / 2,
                       wall + inside_depth - retention_rib, 1.5])
                cube([support_pad_w, retention_rib + 0.1, clip_h - 3]);
    }
}

unified_retainer_fit_test();
