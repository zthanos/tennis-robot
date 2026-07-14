// Reference-only chassis context: plate with the real opening, battery
// bay, IR sensor bodies and confirmation beam, and balls for scale.
// Nothing in this file is manufactured from this model.
include <params.scad>

module chassis_plate() {
    color("burlywood")
        translate([0, 0, plate_top_z - plate_thickness / 2])
            difference() {
                cube([2 * plate_half_len, 2 * plate_half_wid, plate_thickness],
                     center = true);
                // real opening: x 10..460, y +/-150 (spec §3 — no front bridge)
                translate([(open_rear_x + open_front_x) / 2, 0, 0])
                    cube([open_front_x - open_rear_x, 2 * open_half_wid,
                          plate_thickness + 2], center = true);
            }
}

module battery() {
    color("darkslategray")
        translate([batt_min_x + batt_size[0] / 2, 0,
                   plate_top_z + batt_size[2] / 2])
            cube(batt_size, center = true);
}

module ir_sensors() {
    for (sy = [-1, 1])
        color("crimson")
            translate([ir_x, sy * ir_mount_y, ir_z])
                cube([20, 16, 16], center = true);
    color("red")
        translate([ir_x, 0, ir_z])
            rotate([90, 0, 0])
                cylinder(h = 2 * ir_mount_y, r = 1.5, center = true, $fn = 16);
}

module scale_balls() {
    color("greenyellow") {
        // rolling on the receiving chute, cutting the beam
        translate([ir_x, 0, recv_mid_z + ball_d / 2]) sphere(d = ball_d, $fn = 48);
        // resting on the sunken floor
        translate([80, 0, floor_top_z + ball_d / 2]) sphere(d = ball_d, $fn = 48);
    }
}

module chassis_context(with_balls = true) {
    chassis_plate();
    battery();
    ir_sensors();
    if (with_balls) scale_balls();
}

chassis_context();
