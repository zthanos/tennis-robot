// Front-loaded, full-width electronics-tray retainer — physical fit test.
// It bolts to the two front-middle slots in the already-printed black stand,
// clears both shelf/upright footprints, and bears on the lower 10 mm of the
// green tray below every driver PCB.

$fn = 48;

show_top_projection = false;

// Coordinates match vertical-service-stand.scad in its installed orientation.
part_x0 = 8;
part_x1 = 202;
rail_y0 = 38;
rail_y1 = 62;
flange_t = 4;

// Existing shelf/upright footprints: centres 35/175, shelf width 34. Add 1 mm
// clearance on every side so the retainer can slide in from the front.
support_cutouts = [[17, 53], [157, 193]];
cutout_y0 = 43;
cutout_y1 = 63;

// Existing centre slots in the stand are at x=68/142, y=22. The tabs accept
// M4 hardware with washer; elongated holes retain assembly tolerance.
screw_x = [68, 142];
screw_y = 22;
ear_w = 18;
ear_y0 = 16;
ear_y1 = rail_y0;
m4_slot_len = 10;
m4_slot_w = 4.6;

// Stand top is z=6. Green tray bottom is z=20. The retaining wall starts at
// the tray bottom and rises exactly 10 mm, ending at z=30 where the lowest
// legacy PCB envelope begins.
wall_y0 = 54;
wall_y1 = 58;
wall_z0 = 14;  // installed z=20 after the part rests on the 6 mm stand
wall_z1 = 24;  // installed z=30

module capsule_2d(length, width, horizontal = true) {
    hull() {
        if (horizontal) {
            translate([-(length - width) / 2, 0]) circle(d = width);
            translate([ (length - width) / 2, 0]) circle(d = width);
        } else {
            translate([0, -(length - width) / 2]) circle(d = width);
            translate([0,  (length - width) / 2]) circle(d = width);
        }
    }
}

module plan_profile() {
    difference() {
        union() {
            // Main front cross-rail.
            square([part_x1 - part_x0, rail_y1 - rail_y0]);

            // Two forward screw ears.
            for (x = screw_x)
                translate([x - part_x0 - ear_w / 2,
                           ear_y0 - rail_y0])
                    square([ear_w, ear_y1 - ear_y0]);
        }

        // Two rear openings wrap around the printed black supports, producing
        // the three-finger outline in the user's top-view sketch.
        for (r = support_cutouts)
            translate([r[0] - part_x0, cutout_y0 - rail_y0])
                square([r[1] - r[0], cutout_y1 - cutout_y0]);

        // M4 adjustment slots in the two ears.
        for (x = screw_x)
            translate([x - part_x0, screw_y - rail_y0])
                capsule_2d(m4_slot_len, m4_slot_w, true);
    }
}

module front_retainer_fit_test() {
    union() {
        // Horizontal flange that slides in from the front and bolts to the
        // stand's existing middle slots.
        linear_extrude(height = flange_t) plan_profile();

        // Three vertical webs connect the flange to the full-width wall while
        // clearing the two support blocks below it.
        for (r = [[part_x0, support_cutouts[0][0]],
                  [support_cutouts[0][1], support_cutouts[1][0]],
                  [support_cutouts[1][1], part_x1]])
            translate([r[0] - part_x0, wall_y0 - rail_y0, flange_t])
                cube([r[1] - r[0], wall_y1 - wall_y0,
                      wall_z0 - flange_t]);

        // Continuous 10 mm retaining face spreads pressure along the tray and
        // remains below the drivers and printed wiring guides.
        translate([0, wall_y0 - rail_y0, wall_z0])
            cube([part_x1 - part_x0, wall_y1 - wall_y0,
                  wall_z1 - wall_z0]);
    }
}

if (show_top_projection)
    projection(cut = false) front_retainer_fit_test();
else
    front_retainer_fit_test();
