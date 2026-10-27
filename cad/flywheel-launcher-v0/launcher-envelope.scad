// Flywheel launcher v0 — non-manufacturing packaging model.
// See README.md and docs/mechanism/standalone-flywheel-launcher.md.

include <params.scad>

show_guard = true;
show_feed_keepout = true;
show_reference_balls = true;

assert(nip_gap < ball_d,
       "nip_gap must compress the nominal ball in this concept study");
assert(nip_gap >= 50,
       "nip_gap below 50 mm is outside the v0 safe exploration range");
assert(feed_clear_d >= ball_d + 15,
       "feed interface needs at least 15 mm nominal diametral clearance");

module wheel_envelope(z_pos) {
    color("darkorange")
        translate([wheel_x, 0, z_pos])
            rotate([90, 0, 0])
                cylinder(d=wheel_d, h=wheel_width, center=true);
}

module wheel_guard_envelope(z_pos) {
    color("crimson", 0.16)
        translate([wheel_x, 0, z_pos])
            rotate([90, 0, 0])
                cylinder(d=guard_d, h=guard_width, center=true);
}

module side_plate(side_sign) {
    plate_height = 2 * (wheel_d / 2 + cradle_margin)
                 + nip_gap;
    color("slategray", 0.72)
        translate([wheel_x,
                   side_sign * side_plate_y,
                   path_z])
            cube([wheel_d + 2 * cradle_margin,
                  side_plate_t,
                  plate_height], center=true);
}

module horizontal_tube(length, diameter) {
    rotate([0, 90, 0]) cylinder(d=diameter, h=length, center=true);
}

module feed_interface_keepout(entry_z=path_z - feed_clear_d / 2) {
    // The singulator must deliver one ball upward into this breech volume.
    // It is a keep-clear envelope, not a proposed tube or elevator.
    color("mediumseagreen", 0.25)
        translate([-feed_keepout_len / 2 - ball_d / 2, 0,
                   entry_z])
            cube([feed_keepout_len, feed_clear_d, feed_clear_d], center=true);
}

module exit_guide_envelope() {
    color("steelblue", 0.22)
        translate([wheel_d / 2 + exit_guide_len / 2, 0, path_z])
            horizontal_tube(exit_guide_len, exit_clear_d);
}

module ball_path_references() {
    for (xx = [-wheel_d / 2 - ball_d,
                0,
                wheel_d / 2 + exit_guide_len * 0.72])
        color("dodgerblue", 0.75)
            translate([xx, 0, path_z]) sphere(d=ball_d);
}

module launcher_cradle(feed_entry_z=path_z - feed_clear_d / 2) {
    wheel_envelope(lower_wheel_z);
    wheel_envelope(upper_wheel_z);

    if (show_guard) {
        wheel_guard_envelope(lower_wheel_z);
        wheel_guard_envelope(upper_wheel_z);
    }

    side_plate(-1);
    side_plate(1);
    exit_guide_envelope();

    if (show_feed_keepout) feed_interface_keepout(feed_entry_z);
    if (show_reference_balls) ball_path_references();
}

// Place and pitch the launcher about the nominal ball centre at the nip.
// `over_under` provides top/back-spin control but is tall. `side_by_side`
// turns the same pair 90 degrees about the launch axis: it is much lower and
// trades top/back-spin authority for differential left/right spin.
module launcher_oriented(orientation="over_under",
                         nip_height=path_z,
                         launch_pitch_deg=pitch_deg) {
    assert(orientation == "over_under" || orientation == "side_by_side",
           str("Unknown launcher orientation: ", orientation));

    translate([0, 0, nip_height])
        rotate([0, -launch_pitch_deg, 0])
            rotate([orientation == "side_by_side" ? 90 : 0, 0, 0])
                translate([0, 0, -path_z])
                    launcher_cradle(
                        feed_entry_z=orientation == "side_by_side"
                            ? path_z
                            : path_z - feed_clear_d / 2);
}

launcher_oriented();
