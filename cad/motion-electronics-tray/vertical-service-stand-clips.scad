// REJECTED CONCEPT — DO NOT PRINT.
// Side-entry clips cannot reach the stand uprights after the populated tray is
// installed: the continuous tray, driver envelopes, standoffs and wiring block
// the required lateral insertion path. Retained only as design history.

$fn = 36;

tray_t = 4;
upright_t = 8;
fit_clearance = 0.7;

// Only 7 mm high: install directly above the support ledges. The legacy L298N
// footprint begins 10 mm above the tray edge and the BTS footprints begin at
// 15 mm, so this stays below every PCB, standoff and lower wiring guide with
// at least 2 mm nominal vertical clearance in the worst legacy envelope.
clip_h = 7;
clip_w = 32;
wall = 2.4;
captured_stack = tray_t + upright_t;
inside_depth = captured_stack + fit_clearance;
outside_depth = inside_depth + 2 * wall;

// Shallow inner ribs remove most of the clearance after the clip is seated.
retention_rib = 0.35;

module side_entry_clip() {
    union() {
        difference() {
            cube([clip_w, outside_depth, clip_h]);

            // Cavity reaches through the open side so the clip slides on
            // horizontally; the closed end stops against the upright edge.
            translate([wall, wall, -1])
                cube([clip_w - wall + 1, inside_depth, clip_h + 2]);

        }

        // Opposed shallow ribs grip the front of the tray and the rear of the
        // upright without requiring a screw or printed thread.
        translate([wall + 5, wall - 0.1, 1.5])
            cube([clip_w - wall - 8, retention_rib + 0.1, clip_h - 3]);
        translate([wall + 5, wall + inside_depth - retention_rib, 1.5])
            cube([clip_w - wall - 8, retention_rib + 0.1, clip_h - 3]);
    }
}

// A left-opening and a right-opening clip, already arranged for printing.
side_entry_clip();
translate([2 * clip_w + 10, outside_depth, 0])
    rotate([0, 0, 180]) side_entry_clip();
