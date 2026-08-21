// Manual basket lift/tilt v0 — packaging envelopes, not mechanism geometry.
// Robot frame: +X forward, +Z up. Dimensions follow basket-bin-v2.1.

$fn = 64;

show_collect_pose = true;
show_launch_pose = true;
show_chassis = true;
show_ball_load = true;

// Existing validated basket envelope.
bin_rear_x = 20;
bin_front_x = 420;
bin_half_width = 140;
floor_top_z = 25;
wall_top_z = 250;
ball_d = 66;
entry_half_width = 90;

// Baseline side port. Change feed_side to -1 after checking the real cable,
// drive-pod and service layout. The port is basket-integral and fail-closed.
feed_side = 1;
side_port_center_x = 220;
side_port_width_x = 92;
side_port_height_z = 82;

// Exploration inputs. Negative X rotation lowers the +Y side selected above,
// encouraging gravity flow toward the low side port.
launch_lift_z = 430;
launch_side_tilt_deg = -12 * feed_side;
launch_pivot = [side_port_center_x,
                feed_side * bin_half_width,
                floor_top_z];

module chassis_reference() {
    color("burlywood", 0.45)
        translate([0, 0, 45]) cube([920, 580, 14], center=true);
}

module basket_envelope(alpha=0.42) {
    color("seagreen", alpha)
        translate([(bin_rear_x + bin_front_x) / 2,
                   0,
                   (floor_top_z + wall_top_z) / 2])
            difference() {
                cube([bin_front_x - bin_rear_x,
                      2 * bin_half_width,
                      wall_top_z - floor_top_z], center=true);
                translate([0, 0, 6])
                    cube([bin_front_x - bin_rear_x - 12,
                          2 * bin_half_width - 12,
                          wall_top_z - floor_top_z], center=true);
                // Low side port. A real version has a spring-closed integral
                // door; this subtraction only exposes its envelope.
                translate([side_port_center_x
                               - (bin_rear_x + bin_front_x) / 2,
                           feed_side * bin_half_width,
                           -(wall_top_z - floor_top_z) / 2
                               + side_port_height_z / 2])
                    cube([side_port_width_x, 14,
                          side_port_height_z], center=true);
            }
}

module representative_ball_load() {
    // Sparse references only; not a packing/collision simulation.
    for (xx = [80:72:368], yy = [-72, 0, 72])
        color("yellowgreen", 0.75)
            translate([xx, yy, floor_top_z + ball_d / 2]) sphere(d=ball_d);
}

module collect_basket() {
    basket_envelope(0.58);
    // Fixed lift-frame shutter masks the port throughout collection. The real
    // basket still needs its own fail-closed gate for safe removal.
    color("dimgray", 0.85)
        translate([side_port_center_x,
                   feed_side * (bin_half_width + 4),
                   floor_top_z + side_port_height_z / 2])
            cube([side_port_width_x + 16, 8,
                  side_port_height_z + 16], center=true);
    if (show_ball_load) representative_ball_load();
}

module launch_basket() {
    translate([0, 0, launch_lift_z])
        translate(launch_pivot)
            rotate([launch_side_tilt_deg, 0, 0])
                translate(-launch_pivot) {
                    basket_envelope(0.18);
                    if (show_ball_load) representative_ball_load();
                }
}

module side_gravity_dock_keepout() {
    // Stationary chute/singulator dock. A fixed cam opens the basket-integral
    // gate only after the raised basket is locked and fully seated here.
    color("mediumseagreen", 0.30)
        translate([side_port_center_x,
                   feed_side * (bin_half_width + 65),
                   floor_top_z + launch_lift_z + side_port_height_z / 2])
            cube([side_port_width_x, 130,
                  side_port_height_z], center=true);
}

if (show_chassis) chassis_reference();
if (show_collect_pose) collect_basket();
if (show_launch_pose) {
    launch_basket();
    side_gravity_dock_keepout();
}
