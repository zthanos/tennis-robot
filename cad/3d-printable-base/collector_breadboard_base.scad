include <common.scad>

// Raised tray for the collector breadboard prototype.
// User estimate: breadboard footprint 115 x 65 mm, stand height 35 mm.

breadboard_len = 115;
breadboard_w = 65;
breadboard_clearance = 1.0;

wall_t = 2.4;
floor_t = 3.0;
total_h = 35;
lip_h = 8;
corner_r = 5;

outer_len = breadboard_len + 2 * (wall_t + breadboard_clearance);
outer_w = breadboard_w + 2 * (wall_t + breadboard_clearance);
inner_len = breadboard_len + 2 * breadboard_clearance;
inner_w = breadboard_w + 2 * breadboard_clearance;

window_margin = 9;
post_d = 10;
screw_d = 4.2;      // M4 clearance, also usable as pilot for wood screws.
screw_head_d = 8.5;
cable_slot_w = 18;
cable_slot_h = 18;
cable_drop_len = 78;
cable_drop_w = 22;

module rounded_rect_2d(size=[100, 50], r=5) {
    hull() {
        for (x = [r, size[0] - r]) {
            for (y = [r, size[1] - r]) {
                translate([x, y]) circle(r=r);
            }
        }
    }
}

module rounded_prism(size=[100, 50, 10], r=5) {
    linear_extrude(height=size[2])
        rounded_rect_2d([size[0], size[1]], r);
}

module cable_slot_x(y, z=total_h - lip_h / 2) {
    translate([-1, y - cable_slot_w / 2, z - cable_slot_h / 2])
        cube([outer_len + 2, cable_slot_w, cable_slot_h]);
}

module cable_slot_y(x, z=total_h - lip_h / 2) {
    translate([x - cable_slot_w / 2, -1, z - cable_slot_h / 2])
        cube([cable_slot_w, outer_w + 2, cable_slot_h]);
}

module mount_hole(x, y) {
    translate([x, y, -1])
        cylinder(h=total_h + 2, d=screw_d);
    translate([x, y, total_h - 2.2])
        cylinder(h=3.4, d1=screw_head_d, d2=screw_d);
}

module underside_window() {
    translate([window_margin, window_margin, -1])
        rounded_prism(
            [outer_len - 2 * window_margin, outer_w - 2 * window_margin, total_h - floor_t],
            4
        );
}

module cable_drop_window() {
    translate([
        outer_len / 2 - cable_drop_len / 2,
        outer_w / 2 - cable_drop_w / 2,
        total_h - lip_h - 1
    ])
        rounded_prism([cable_drop_len, cable_drop_w, lip_h + 3], 4);
}

module collector_breadboard_base() {
    difference() {
        union() {
            rounded_prism([outer_len, outer_w, total_h], corner_r);

            // Four internal bosses keep screw loads away from the hollow shell.
            for (x = [11, outer_len - 11]) {
                for (y = [11, outer_w - 11]) {
                    translate([x, y, 0])
                        cylinder(h=total_h, d=post_d);
                }
            }
        }

        // Top pocket. The breadboard rests on a flat shelf and is retained by
        // the short lip without covering the jumper holes.
        translate([wall_t + breadboard_clearance, wall_t + breadboard_clearance, total_h - lip_h - 0.4])
            rounded_prism([inner_len, inner_w, lip_h + 2], max(corner_r - 2, 1));

        // Hollow underside to reduce filament while leaving a stiff rim.
        underside_window();

        // Downward cable pass-through into the hollow underside. This keeps
        // jumper bundles exiting below the tray instead of across the top edge.
        cable_drop_window();

        // Cable exits: power at the top, USB/front, and jumper bundles left/right.
        cable_slot_x(outer_w / 2);
        cable_slot_y(outer_len / 2);
        cable_slot_y(outer_len * 0.20);
        cable_slot_y(outer_len * 0.80);

        for (x = [11, outer_len - 11]) {
            for (y = [11, outer_w - 11]) {
                mount_hole(x, y);
            }
        }
    }
}

collector_breadboard_base();
