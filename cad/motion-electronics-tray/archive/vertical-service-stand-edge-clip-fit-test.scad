// One-piece LEFT edge-clip fit test for the populated electronics tray and
// vertical service stand. Print one, verify physically, then mirror in X for
// the right side only after fit is confirmed.

$fn = 32;

tray_t = 4;
upright_t = 8;
fit_clearance = 0.7;

clip_h = 10;
wall = 2.4;
inside_depth = tray_t + upright_t + fit_clearance;
outside_depth = inside_depth + 2 * wall;

// The closest PCB begins 6 mm inboard from the green tray edge. Limit the
// front jaw to 4 mm, leaving 2 mm nominal clearance. The upright begins 8 mm
// inboard, so a 22 mm rear jaw overlaps it by 14 mm.
front_grip = 4;
rear_grip = 22;
retention_rib = 0.35;

module left_edge_clip_fit_test() {
    union() {
        // Closed outer wall rests against the green tray edge.
        cube([wall, outside_depth, clip_h]);

        // Short front jaw stays inside the clear edge margin before the PCB.
        cube([wall + front_grip, wall, clip_h]);

        // Long rear jaw reaches the black upright behind the tray.
        translate([0, wall + inside_depth, 0])
            cube([wall + rear_grip, wall, clip_h]);

        // Shallow ribs give a slight PETG friction fit without relying on a
        // screw, adhesive or access behind the triangular brace.
        translate([wall + 0.8, wall - 0.1, 1.5])
            cube([front_grip - 1.1, retention_rib + 0.1, clip_h - 3]);
        translate([wall + 9, wall + inside_depth - retention_rib, 1.5])
            cube([rear_grip - 9.3, retention_rib + 0.1, clip_h - 3]);
    }
}

left_edge_clip_fit_test();

