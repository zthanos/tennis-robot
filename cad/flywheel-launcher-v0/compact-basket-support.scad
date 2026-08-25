// Compact basket support architecture — current mechanical concept.
// Units: mm, compact functional-group frame (+X forward, +Y left, +Z up).
//
// Known: two vertical guide envelopes at the existing guide datums.
// Inferred: hollow aluminium guide members and plywood raised-position support
// envelopes. Purchased guide/follower hardware and the holder engagement /
// retention detail are deliberately not selected here.

guide_x = 70;
guide_y = 180;
guide_bottom_z = 52;
guide_top_z = 455;
guide_size = [28, 22];
guide_foot_size = [90, 70, 12];

// In the current diagnostic launch configuration the real bin flange is first
// lifted 100 mm and then tilted 12 degrees about [470,0,40]. Each holder shelf
// reproduces a 120 x 12 mm patch of that transformed flange underside. Posts
// sit wholly on the intact chassis side strips (|Y| >= 170).
holder_lift = 100;
holder_tilt_deg = 12;
holder_pivot = [470, 0, 40];
holder_x = 220;
holder_length = 120;
holder_flange_y0 = 160;
holder_flange_y1 = 172;
holder_post_inner_y = 172;
holder_post_outer_y = 196;
holder_post_bottom_z = 52;
holder_shelf_thickness = 20;
holder_post_x = holder_pivot[0]
              + cos(holder_tilt_deg) * (holder_x - holder_pivot[0])
              + sin(holder_tilt_deg) * (52 - holder_pivot[2]);
holder_shelf_under_z = holder_pivot[2] + holder_lift
                     - sin(holder_tilt_deg) * (holder_x - holder_pivot[0])
                     + cos(holder_tilt_deg)
                       * (52 - holder_shelf_thickness - holder_pivot[2]);

module compact_basket_guides() {
    color("silver")
        for (sy = [-1, 1]) {
            translate([guide_x, sy * guide_y,
                       (guide_bottom_z + guide_top_z) / 2])
                cube([guide_size[0], guide_size[1],
                      guide_top_z - guide_bottom_z], center=true);
            translate([guide_x, sy * guide_y, 58])
                cube(guide_foot_size, center=true);
        }
}

// Configuration-specific support envelope. These fixed wooden shelves provide
// a real downward load surface in the launch pose, but the still-undesigned
// engagement/latch detail must withdraw or install them during vertical lift.
// Therefore current integration views show them only in the raised/launch
// configuration; this is not a claim of a solved dynamic mechanism.
module compact_raised_basket_holders() {
    color("burlywood")
        for (sy = [-1, 1]) {
            // Chassis-supported post.
            translate([holder_post_x,
                       sy * (holder_post_inner_y + holder_post_outer_y) / 2,
                       (holder_post_bottom_z + holder_shelf_under_z) / 2])
                cube([holder_length,
                      holder_post_outer_y - holder_post_inner_y,
                      holder_shelf_under_z - holder_post_bottom_z],
                     center=true);
            // Inward tilted shelf: top face coincides with the transformed
            // basket-flange underside over the declared 120 x 12 mm patch.
            translate([0, 0, holder_lift])
                translate(holder_pivot)
                    rotate([0, holder_tilt_deg, 0])
                        translate(-holder_pivot)
                            translate([holder_x,
                                       sy * (holder_flange_y0
                                             + holder_post_outer_y) / 2,
                                       52 - holder_shelf_thickness / 2])
                                cube([holder_length,
                                      holder_post_outer_y - holder_flange_y0,
                                      holder_shelf_thickness], center=true);
        }
}
