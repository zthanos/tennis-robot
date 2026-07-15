// Single-motor geared dual-wheel intake concept.
//
// Visual/mechanical concept only. This file does not feed the robot URDF,
// Gazebo model, controller config, or the validated two-motor intake docs.
// Units: mm. Ground frame: +x forward, +y left, +z up.

$fn = 72;

// ---- View toggles ----
show_ball = true;
show_funnel_context = true;
show_drive_arrows = true;
show_labels = false;
show_pitch_circles = true;
explode_gears = 0; // set to 20..60 to lift the gear train for inspection

// ---- Intake geometry copied from the dual-wheel concept scale ----
ball_d = 66;
wheel_r = 60;
wheel_h = 80;
wheel_gap = 56;
nip_x = 540;
wheel_z = 45;
wheel_tilt_deg = 45;
carriage_travel = 8;

wheel_center_y = wheel_gap / 2 + wheel_r;
wheel_left_y = wheel_center_y;
wheel_right_y = -wheel_center_y;

// ---- Single motor + gear train concept ----
gear_z = wheel_z + wheel_h / 2 + 20;
gear_thickness = 8;
gear_module = 2.4;

motor_x = nip_x + 88;
motor_y = 0;
motor_body_d = 37;
motor_body_l = 72;
motor_shaft_d = 6;

pinion_teeth = 18;
output_teeth = 44;
idler_teeth = 22;

pinion_r = gear_pitch_r(pinion_teeth);
output_r = gear_pitch_r(output_teeth);
idler_r = gear_pitch_r(idler_teeth);

left_output = [nip_x, wheel_left_y, gear_z + explode_gears];
right_output = [nip_x, wheel_right_y, gear_z + explode_gears];
pinion_pos = [motor_x, motor_y, gear_z + explode_gears];

pinion_to_idler = pinion_r + idler_r;
idler_to_idler = idler_r + idler_r;
idler_to_output = idler_r + output_r;

// Idler centers are computed from pitch-circle tangencies.
// Left path: pinion -> idler -> left output, so left output rotates the same
// direction as the pinion. Right path: pinion -> idler -> idler -> right
// output, so right output reverses and the wheel inner faces counter-rotate.
left_idler_pos = tangent_idler(pinion_pos, left_output, pinion_to_idler, idler_to_output, -1);
right_idler_a_pos = tangent_idler(
    pinion_pos,
    right_output,
    pinion_to_idler,
    idler_to_idler + idler_to_output,
    1
);
right_idler_b_pos = point_toward(
    right_idler_a_pos,
    right_output,
    idler_to_idler
);

module color_body() { color([0.76, 0.78, 0.78]) children(); }
module color_mount() { color([0.18, 0.19, 0.20]) children(); }
module color_rubber() { color([0.02, 0.025, 0.025]) children(); }
module color_gear() { color([0.86, 0.62, 0.16]) children(); }
module color_motor() { color([0.18, 0.32, 0.58]) children(); }
module color_context() { color([1.0, 0.45, 0.10, 0.30]) children(); }

function gear_pitch_r(teeth) = teeth * gear_module / 2;
function gear_outer_r(teeth) = gear_pitch_r(teeth) + gear_module;
function gear_root_r(teeth) = max(gear_pitch_r(teeth) - 1.25 * gear_module, 3);
function dist_xy(a, b) = sqrt(pow(b[0] - a[0], 2) + pow(b[1] - a[1], 2));
function unit_xy(a, b) =
    let(d = dist_xy(a, b))
    [(b[0] - a[0]) / d, (b[1] - a[1]) / d];
function tangent_idler(a, b, ra, rb, side) =
    let(
        d = dist_xy(a, b),
        u = unit_xy(a, b),
        p = [-u[1], u[0]],
        along = (pow(ra, 2) - pow(rb, 2) + pow(d, 2)) / (2 * d),
        off = sqrt(max(pow(ra, 2) - pow(along, 2), 0)),
        x = a[0] + along * u[0] + side * off * p[0],
        y = a[1] + along * u[1] + side * off * p[1]
    )
    [x, y, gear_z + explode_gears];
function point_toward(a, b, distance) =
    let(u = unit_xy(a, b))
    [a[0] + distance * u[0], a[1] + distance * u[1], gear_z + explode_gears];

module rounded_box(size, r = 3) {
    hull() {
        for (sx = [-1, 1], sy = [-1, 1], sz = [-1, 1])
            translate([
                sx * (size[0] / 2 - r),
                sy * (size[1] / 2 - r),
                sz * (size[2] / 2 - r)
            ])
                sphere(r = r, $fn = 18);
    }
}

module shaft_z(h = 138, d = 8) {
    color([0.45, 0.45, 0.45])
        cylinder(h = h, r = d / 2, center = true);
}

module spur_gear(teeth, thickness = gear_thickness, bore = 8) {
    root_r = gear_root_r(teeth);
    outer_r = gear_outer_r(teeth);
    tooth_w = 360 / teeth * 0.44;

    difference() {
        union() {
            cylinder(h = thickness, r = root_r, center = true, $fn = teeth * 4);
            for (i = [0 : teeth - 1]) {
                rotate([0, 0, i * 360 / teeth])
                    translate([root_r + gear_module * 0.50, 0, 0])
                        rotate([0, 0, 45])
                            cube([gear_module * 1.8, tooth_w * 0.45, thickness], center = true);
            }
            cylinder(h = thickness + 2, r = bore * 1.55, center = true);
        }
        cylinder(h = thickness + 4, r = bore / 2, center = true);
        // Lightening holes.
        for (a = [0 : 60 : 300])
            rotate([0, 0, a])
                translate([root_r * 0.55, 0, 0])
                    cylinder(h = thickness + 5, r = max(root_r * 0.08, 2), center = true, $fn = 24);
    }
}

module pitch_circle(r, thickness = 1.0) {
    color([0.15, 0.55, 1.0, 0.55])
        difference() {
            cylinder(h = thickness, r = r + 0.7, center = true, $fn = 120);
            cylinder(h = thickness + 1, r = r - 0.7, center = true, $fn = 120);
        }
}

module intake_wheel(side = 1) {
    y = side * wheel_center_y;

    // Outward compliance slot and carriage.
    color_mount()
        translate([nip_x, y + side * carriage_travel / 2, wheel_z + wheel_h / 2 + 6])
            cube([70, carriage_travel + 30, 12], center = true);
    color_body()
        translate([nip_x, y, wheel_z + wheel_h / 2 + 6])
            rounded_box([44, 34, 18], 3);

    // Vertical side wheel, tilted as a concept reference to the current bench.
    color_rubber()
        translate([nip_x, y, wheel_z])
            rotate([0, wheel_tilt_deg, 0])
                cylinder(h = wheel_h, r = wheel_r, center = false);

    // Shaft continues up to the gear train.
    translate([nip_x, y, wheel_z + wheel_h / 2])
        shaft_z(h = gear_z - wheel_z + 35, d = 8);

    // Small orange strip to make spin direction visible.
    color([1.0, 0.35, 0.05])
        translate([nip_x + side * 0, y - side * (wheel_r - 5), wheel_z + wheel_h - 12])
            cube([8, 22, 12], center = true);
}

module motor_assembly() {
    // GB37-style gearmotor stood vertically so its output shaft is coaxial
    // with the pinion. A real build could do this with a vertical bracket or
    // replace it with a right-angle gearbox if height becomes a problem.
    body_center_z = gear_z - gear_thickness / 2 - motor_body_l / 2 - 14;

    color_motor()
        translate([motor_x, motor_y, body_center_z])
            cylinder(h = motor_body_l, r = motor_body_d / 2, center = true);
    color([0.08, 0.08, 0.09])
        translate([motor_x, motor_y, gear_z - 11])
            cylinder(h = 30, r = motor_shaft_d / 2, center = true);

    color_mount()
        translate([motor_x, 0, body_center_z - motor_body_l / 2 - 5])
            rounded_box([74, 74, 10], 3);
    color_mount()
        translate([motor_x + 24, 0, body_center_z])
            rounded_box([8, 72, 66], 2);
}

module gear_at(pos, teeth, bore = 8, phase = 0) {
    translate(pos)
        rotate([0, 0, phase]) {
            color_gear()
                spur_gear(teeth, bore = bore);
            if (show_pitch_circles)
                translate([0, 0, gear_thickness / 2 + 1.1])
                    pitch_circle(gear_pitch_r(teeth));
        }
}

module gear_train() {
    // Output gears are coaxial with the intake wheel shafts.
    gear_at(left_output, output_teeth, phase = 4);
    gear_at(right_output, output_teeth, phase = -4);

    // Motor pinion and idlers. The right side has one extra idler to reverse
    // rotation relative to the left output shaft.
    gear_at(pinion_pos, pinion_teeth, bore = motor_shaft_d, phase = 0);
    gear_at(left_idler_pos, idler_teeth, phase = 7);
    gear_at(right_idler_a_pos, idler_teeth, phase = 9);
    gear_at(right_idler_b_pos, idler_teeth, phase = -6);

    // Gear plate above the throat.
    color([0.12, 0.12, 0.12, 0.45])
        translate([nip_x + 38, 0, gear_z - gear_thickness / 2 - 5 + explode_gears])
            cube([210, 270, 4], center = true);

    // Bearing posts under every idler/output shaft make the axes explicit.
    for (p = [left_output, right_output, left_idler_pos, right_idler_a_pos, right_idler_b_pos])
        color_mount()
            translate([p[0], p[1], gear_z - 20 + explode_gears])
                cylinder(h = 30, r = 7, center = true, $fn = 32);
}

module funnel_and_ramp_context() {
    // Front funnel cheeks only as context; not a full robot/chassis.
    color_context() {
        translate([nip_x + 170, 145, 42])
            rotate([0, 0, -13])
                cube([250, 6, 84], center = true);
        translate([nip_x + 170, -145, 42])
            rotate([0, 0, 13])
                cube([250, 6, 84], center = true);

        // Simple center ramp behind the wheels toward the basket/hopper.
        translate([nip_x - 75, 0, 18])
            rotate([0, -8, 0])
                cube([190, 170, 8], center = true);
    }
}

module tennis_ball_reference() {
    color([0.72, 1.0, 0.13, 0.48])
        translate([nip_x + 30, 0, ball_d / 2])
            sphere(d = ball_d);
}

module direction_arrow(pos, rot = 0, label_scale = 1) {
    translate(pos)
        rotate([0, 0, rot])
            color([0.1, 0.55, 0.95]) {
                cube([42 * label_scale, 5, 5], center = true);
                translate([-24 * label_scale, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(h = 18 * label_scale, r1 = 8 * label_scale, r2 = 0, center = true, $fn = 24);
            }
}

module base_reference() {
    color([0.20, 0.22, 0.22, 0.22])
        translate([nip_x + 28, 0, 2])
            cube([420, 330, 4], center = true);
}

module labels() {
    color([0.02, 0.02, 0.02])
        translate([nip_x + 132, -160, 128])
            rotate([70, 0, 0])
                linear_extrude(1.2)
                    text("1 motor + gear train", size = 14, halign = "center");
    color([0.02, 0.02, 0.02])
        translate([nip_x - 78, 0, 142])
            rotate([70, 0, 0])
                linear_extrude(1.2)
                    text("side-pinch wheels", size = 13, halign = "center");
}

module assembly() {
    base_reference();
    if (show_funnel_context) funnel_and_ramp_context();

    intake_wheel(1);
    intake_wheel(-1);
    motor_assembly();
    gear_train();

    if (show_ball) tennis_ball_reference();

    if (show_drive_arrows) {
        // Blue arrows show ball transport direction through the throat.
        direction_arrow([nip_x + 18, 0, 112], 180, 1.0);
        direction_arrow([nip_x - 40, 0, 86], 180, 0.8);
    }

    if (show_labels) labels();
}

assembly();
