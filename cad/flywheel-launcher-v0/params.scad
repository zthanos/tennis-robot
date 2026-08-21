// Flywheel launcher v0 — envelope inputs only.
// Units: mm. Local frame: +X is launch direction, +Z is up, wheel axes are Y.

$fn = 96;

ball_d = 66;

wheel_d = 200;
wheel_width = 50;
nip_gap = 58;
wheel_edge_round = 6; // visual hint only; not a tyre profile

pitch_deg = 20;
path_z = 380;
wheel_x = 0;
wheel_center_separation = wheel_d + nip_gap;
lower_wheel_z = path_z - wheel_center_separation / 2;
upper_wheel_z = path_z + wheel_center_separation / 2;

feed_clear_d = 90;
feed_keepout_len = 150;
exit_guide_len = 220;
exit_clear_d = 90;

side_plate_t = 8;
side_plate_y = wheel_width / 2 + 18;
cradle_margin = 28;

guard_radial_clearance = 18;
guard_width_clearance = 18;

// Derived reference values.
nominal_compression = ball_d - nip_gap;
guard_d = wheel_d + 2 * guard_radial_clearance;
guard_width = wheel_width + 2 * guard_width_clearance;
