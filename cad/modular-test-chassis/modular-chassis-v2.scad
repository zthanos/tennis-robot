// Modular open-front chassis v2 — unified 50->50 interface integration model.
// Units: mm. Ground frame: +X forward, +Y left, +Z up.
//
// This file is an integration prototype, not a print release. It verifies that
// every structural joint uses the same 50 x 30 mm beam interface while the
// Gamma landing body widens to 70 mm only after that interface.

$fn = 48;

part = "assembly";
// [assembly,frame,straight_sleeve,corner_left,corner_right,crossbar_half,gamma_left,gamma_right,ramp_cradle_left,ramp_cradle_right,electronics_tray_left,electronics_tray_right]

rail_w = 50;
rail_h = 30;
rail_wall = 3.2;
rail_z0 = 22;
rail_top_z = rail_z0 + rail_h;

// Universal internal connector. 0.30 mm nominal clearance on every side.
sleeve_w = 43.0;
sleeve_h = 23.0;
sleeve_len = 120;
sleeve_wall = 3.0;
sleeve_insert = sleeve_len / 2;
lock_clear_d = 3.4;
lock_x = 30;
lock_y = 12.5;

nut_across_flats = 6.2;
nut_channel_h = 3.7;
nut_corner_d = nut_across_flats / cos(30);
nut_entry_w = nut_corner_d + 0.40;
nut_seat_w = nut_corner_d + 0.20;
nut_top_cover = 4.0;
nut_tower_d = 15.0;

// Compact U-frame: two 200 mm motor modules per side.
rear_joint_x = -165;
motor_joint_x = 35;
gamma_joint_x = 235;
rear_motor_x = -65;
front_motor_x = 135;

// The existing Gamma feet occupy X=330..455 and Y=175..245 on the left.
gamma_socket_x0 = gamma_joint_x;
gamma_socket_x1 = 300;
gamma_flare_x1 = 330;
gamma_body_x1 = 455;
gamma_body_w = 70;
gamma_body_center_y = 210;
drive_center_y = 220; // 50 mm rail is flush with the outboard Y=245 face.

rear_crossbar_length = 2 * drive_center_y;
crossbar_half_length = rear_crossbar_length / 2;

wheel_d = 170;
wheel_w = 80;
wheel_y = 350;
wheel_z = 85;

electronics_x = -25;
electronics_size_x = 190;
electronics_half_y = 145;
electronics_plate_t = 4;
electronics_arm_x = 30;

frame_color = [0.16, 0.48, 0.78, 0.92];
gamma_body_color = [0.14, 0.64, 0.46, 0.92];
sleeve_color = [0.96, 0.55, 0.10, 0.92];
intake_color = [0.78, 0.26, 0.08, 0.72];
wheel_color = [0.05, 0.06, 0.07, 0.90];

module hollow_box_x(length, width=rail_w, height=rail_h, wall=rail_wall) {
    difference() {
        cube([length, width, height], center=true);
        cube([length + 2, width - 2*wall, height - 2*wall], center=true);
    }
}

module hollow_box_y(length, width=rail_w, height=rail_h, wall=rail_wall) {
    rotate([0, 0, 90]) hollow_box_x(length, width, height, wall);
}

module vertical_lock_holes_x(joint_x=0, beam_y=0, top_z=rail_h) {
    for (dx = [-lock_x, lock_x], yy = [-lock_y, lock_y])
        translate([joint_x + dx, beam_y + yy, -1])
            cylinder(d=lock_clear_d, h=top_z + 2);
}

module sleeve_nut_channel(x, y) {
    side = y >= 0 ? 1 : -1;
    pocket_z = sleeve_h - nut_top_cover - nut_channel_h;
    translate([0, 0, pocket_z])
        linear_extrude(height=nut_channel_h)
            polygon(side > 0 ? [
                [x - nut_seat_w/2, y - 0.05],
                [x + nut_seat_w/2, y - 0.05],
                [x + nut_entry_w/2, sleeve_w/2 + 1],
                [x - nut_entry_w/2, sleeve_w/2 + 1]
            ] : [
                [x - nut_entry_w/2, -sleeve_w/2 - 1],
                [x + nut_entry_w/2, -sleeve_w/2 - 1],
                [x + nut_seat_w/2, y + 0.05],
                [x - nut_seat_w/2, y + 0.05]
            ]);
}

module straight_sleeve() {
    difference() {
        union() {
            difference() {
                translate([-sleeve_len/2, -sleeve_w/2, 0])
                    cube([sleeve_len, sleeve_w, sleeve_h]);
                translate([-sleeve_len/2 - 1,
                           -sleeve_w/2 + sleeve_wall,
                           sleeve_wall])
                    cube([sleeve_len + 2,
                          sleeve_w - 2*sleeve_wall,
                          sleeve_h - 2*sleeve_wall]);
            }
            for (xx = [-lock_x, lock_x], yy = [-lock_y, lock_y])
                translate([xx, yy, sleeve_wall])
                    cylinder(d=nut_tower_d,
                             h=sleeve_h - sleeve_wall);
        }
        for (xx = [-lock_x, lock_x], yy = [-lock_y, lock_y]) {
            translate([xx, yy, sleeve_h - nut_top_cover - nut_channel_h - 0.1])
                cylinder(d=lock_clear_d,
                         h=nut_channel_h + nut_top_cover + 0.2);
            sleeve_nut_channel(xx, yy);
        }
    }
}

// L connector in its left-rear orientation: +X into the side rail and -Y
// into the rear crossbar. Right is an exact mirror. These are solid around the
// elbow; each arm uses the same 43 x 23 mm insertion envelope as the straight
// sleeve. Lock detail remains a coupon gate before printing the full corner.
module corner_sleeve_positive() {
    union() {
            translate([sleeve_insert/2, 0, sleeve_h/2])
                cube([sleeve_insert, sleeve_w, sleeve_h], center=true);
            translate([0, -sleeve_insert/2, sleeve_h/2])
                cube([sleeve_w, sleeve_insert, sleeve_h], center=true);
            translate([0, 0, sleeve_h/2])
                cube([sleeve_w, sleeve_w, sleeve_h], center=true);
    }
}

module corner_sleeve(side=1) {
    if (side > 0) corner_sleeve_positive();
    else mirror([0, 1, 0]) corner_sleeve_positive();
}

module crossbar_half() {
    difference() {
        hollow_box_y(crossbar_half_length);
        // Holes 30 mm from the centre splice and the outer corner joint.
        for (yy = [-crossbar_half_length/2 + lock_x,
                   crossbar_half_length/2 - lock_x], xx = [-lock_y, lock_y])
            translate([xx, yy, -rail_h/2 - 1])
                cylinder(d=lock_clear_d, h=rail_h + 2);
    }
}

module gamma_body_positive() {
    socket_y = drive_center_y;
    body_y = gamma_body_center_y;
    difference() {
        union() {
            translate([(gamma_socket_x0 + gamma_socket_x1)/2,
                       socket_y, rail_z0 + rail_h/2])
                cube([gamma_socket_x1-gamma_socket_x0,
                      rail_w, rail_h], center=true);
            hull() {
                translate([gamma_socket_x1, socket_y,
                           rail_z0 + rail_h/2])
                    cube([1, rail_w, rail_h], center=true);
                translate([gamma_flare_x1, body_y,
                           rail_z0 + rail_h/2])
                    cube([1, gamma_body_w, rail_h], center=true);
            }
            translate([(gamma_flare_x1 + gamma_body_x1)/2,
                       body_y, rail_z0 + rail_h/2])
                cube([gamma_body_x1-gamma_flare_x1,
                      gamma_body_w, rail_h], center=true);
        }

        // Continuous inner cavity. The first 65 mm is exactly the universal
        // 50 mm socket; the shell then widens only toward the chassis centre.
        union() {
            translate([(gamma_socket_x0 + gamma_socket_x1)/2 - 0.5,
                       socket_y, rail_z0 + rail_h/2])
                cube([gamma_socket_x1-gamma_socket_x0 + 1,
                      rail_w - 2*rail_wall,
                      rail_h - 2*rail_wall], center=true);
            hull() {
                translate([gamma_socket_x1, socket_y,
                           rail_z0 + rail_h/2])
                    cube([1.2, rail_w - 2*rail_wall,
                          rail_h - 2*rail_wall], center=true);
                translate([gamma_flare_x1, body_y,
                           rail_z0 + rail_h/2])
                    cube([1.2, gamma_body_w - 2*rail_wall,
                          rail_h - 2*rail_wall], center=true);
            }
            translate([(gamma_flare_x1 + gamma_body_x1)/2 + 0.5,
                       body_y, rail_z0 + rail_h/2])
                cube([gamma_body_x1-gamma_flare_x1 + 1,
                      gamma_body_w - 2*rail_wall,
                      rail_h - 2*rail_wall], center=true);
        }

        // Universal connector lock, 30 mm inside the Gamma socket.
        for (yy = [socket_y-lock_y, socket_y+lock_y])
            translate([gamma_socket_x0 + lock_x, yy, rail_top_z-rail_wall-0.1])
                cylinder(d=lock_clear_d, h=rail_wall + 0.2);

        // Existing physical Gamma foot pattern: 95 x 44 mm, M5 clearance.
        for (xx = [345, 440], yy = [188, 232])
            translate([xx, yy, rail_z0-1])
                cylinder(d=5.5, h=rail_h+2);
    }
}

module gamma_body(side=1) {
    if (side > 0) gamma_body_positive();
    else mirror([0, 1, 0]) gamma_body_positive();
}

module motor_module(side=1, xc=0) {
    // LEFT/RIGHT file names describe the local motor side. World-left needs
    // the +Y motor-side model and world-right needs its -Y mirror.
    translate([xc, side*drive_center_y, rail_z0])
        if (side > 0)
            import("../../engineering/chassis-strength-bench/coupon/drive-motor-inline-RIGHT-M3-validated-dual-splice.stl");
        else
            import("../../engineering/chassis-strength-bench/coupon/drive-motor-inline-LEFT-M3-validated-dual-splice.stl");
}

module rear_crossbar() {
    // Two identical printable halves; their centre splice uses the universal
    // straight sleeve and their outer ends use mirrored L sleeves.
    translate([rear_joint_x, drive_center_y/2, rail_z0 + rail_h/2])
        crossbar_half();
    translate([rear_joint_x, -drive_center_y/2, rail_z0 + rail_h/2])
        crossbar_half();
}

module frame_structure() {
    color(frame_color) {
        for (side = [-1, 1]) {
            motor_module(side, rear_motor_x);
            motor_module(side, front_motor_x);
            gamma_body(side);
        }
        rear_crossbar();
    }

    color(sleeve_color) {
        // Straight side-rail splices and motor-to-Gamma splices.
        for (side = [-1, 1], xx = [motor_joint_x, gamma_joint_x])
            translate([xx, side*drive_center_y,
                       rail_z0 + (rail_h-sleeve_h)/2])
                straight_sleeve();
        // Centre crossbar splice (straight sleeve rotated to Y).
        translate([rear_joint_x, 0,
                   rail_z0 + (rail_h-sleeve_h)/2])
            rotate([0, 0, 90]) straight_sleeve();
        // Rear corners.
        translate([rear_joint_x, drive_center_y,
                   rail_z0 + (rail_h-sleeve_h)/2])
            corner_sleeve(1);
        translate([rear_joint_x, -drive_center_y,
                   rail_z0 + (rail_h-sleeve_h)/2])
            corner_sleeve(-1);
    }
}

module wheel_context(side=1, xc=0) {
    color(wheel_color)
        translate([xc, side*wheel_y, wheel_z])
            rotate([90, 0, 0])
                difference() {
                    cylinder(d=wheel_d, h=wheel_w, center=true);
                    cylinder(d=36, h=wheel_w+2, center=true);
                }
}

module intake_context() {
    color(intake_color) {
        import("../standalone-intake-fixed-motor/stl/gamma_bracket_left_reinforced_v2.stl");
        import("../standalone-intake-fixed-motor/stl/gamma_bracket_right_reinforced_v2.stl");
    }
    color([0.42, 0.72, 0.32, 0.92])
        import("../standalone-intake-fixed-motor/stl/intake_handoff_ramp_wheel_first.stl");
}

module ramp_cradle_positive() {
    rail_inner_y = drive_center_y - rail_w/2;
    ramp_outer_y = 94;
    arm_x = 321;
    arm_z = 47;
    union() {
        translate([arm_x, (rail_inner_y+ramp_outer_y)/2, arm_z])
            cube([10, rail_inner_y-ramp_outer_y, 10], center=true);
        translate([arm_x, 94, 43]) cube([10, 9, 22], center=true);
        hull() {
            translate([arm_x, rail_inner_y-8, rail_top_z-5])
                cube([10, 12, 8], center=true);
            translate([arm_x, 104, 39])
                cube([10, 12, 8], center=true);
        }
    }
}

module ramp_cradle(side=1) {
    if (side > 0) ramp_cradle_positive();
    else mirror([0, 1, 0]) ramp_cradle_positive();
}

module electronics_tray_positive() {
    // Half tray fits the print bed. Two integral arms reach the side rail;
    // their exact M3 board/rail patterns remain a physical-measurement gate.
    union() {
        translate([electronics_x, electronics_half_y/2,
                   rail_top_z + electronics_plate_t/2])
            cube([electronics_size_x, electronics_half_y,
                  electronics_plate_t], center=true);
        for (xx = [electronics_x-electronics_size_x/2+electronics_arm_x/2,
                   electronics_x+electronics_size_x/2-electronics_arm_x/2])
            translate([xx, (electronics_half_y +
                            drive_center_y + rail_w/2)/2,
                       rail_top_z + electronics_plate_t/2])
                cube([electronics_arm_x,
                      drive_center_y + rail_w/2 - electronics_half_y,
                      electronics_plate_t], center=true);
    }
}

module electronics_tray(side=1) {
    if (side > 0) electronics_tray_positive();
    else mirror([0, 1, 0]) electronics_tray_positive();
}

module assembly() {
    frame_structure();
    color([0.58, 0.20, 0.76, 0.94]) {
        ramp_cradle(1);
        ramp_cradle(-1);
    }
    color([0.12, 0.55, 0.35, 0.62]) {
        electronics_tray(1);
        electronics_tray(-1);
    }
    for (side = [-1, 1], xx = [rear_motor_x, front_motor_x])
        wheel_context(side, xx);
    intake_context();
}

if (part == "frame") frame_structure();
else if (part == "straight_sleeve") straight_sleeve();
else if (part == "corner_left") corner_sleeve(1);
else if (part == "corner_right") corner_sleeve(-1);
else if (part == "crossbar_half")
    translate([0, 0, rail_h/2]) crossbar_half();
else if (part == "gamma_left")
    translate([-gamma_socket_x0, -drive_center_y, -rail_z0]) gamma_body(1);
else if (part == "gamma_right")
    translate([-gamma_socket_x0, drive_center_y, -rail_z0]) gamma_body(-1);
else if (part == "ramp_cradle_left")
    translate([-316, -94, -32]) ramp_cradle(1);
else if (part == "ramp_cradle_right")
    translate([-316, 94, -32]) ramp_cradle(-1);
else if (part == "electronics_tray_left")
    translate([-electronics_x, 0, -rail_top_z]) electronics_tray(1);
else if (part == "electronics_tray_right")
    translate([-electronics_x, 0, -rail_top_z]) electronics_tray(-1);
else assembly();

assert(abs((drive_center_y + rail_w/2) -
           (gamma_body_center_y + gamma_body_w/2)) < 0.001,
       "50 mm interface must stay flush with the outboard face of the 70 mm Gamma body");
assert(gamma_socket_x1 - gamma_socket_x0 >= sleeve_insert,
       "Gamma socket must accept at least half of the universal sleeve");
