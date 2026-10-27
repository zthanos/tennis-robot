// PRINT STUDY — two independent, outward-anchored Γ brackets replacing the
// full transverse bridge. Units: mm; compact robot ground frame (+X forward).
//
// The frozen intake datums are unchanged:
//   wheel centres = [370, +/-90, 70], axes = 35 deg in X-Z;
//   motor-mount bearing plane = Z 190;
//   compact cheek flange plane = Z 150.
//
// This is a prototype structure, not a strength certification. Each side is
// bolted to the rigid chassis side strip and carries only its own motor mount
// and cheek. The centre remains open. Base-hole positions are provisional and
// must be checked against the physical chassis before drilling.

$fn = 64;

use <gamma-cheek-cable-crossbar.scad>

part = "assembly"; // [assembly,left,right,left_print,right_print,hard_interference,cheek_crossbar_interference]
show_context = true;
show_crossbar = true;
crossbar_interference_target = "all"; // [all,wheel,motor,mount,cheek,ball]

// Frozen compact intake geometry.
wheel_x = 370;
wheel_y = 90;
wheel_z = 70;
wheel_d = 124;
wheel_width = 73;
axis_tilt_deg = 35;

// Existing v6 motor-mount interface, translated from its standalone frame.
motor_pad_x = 440.837;
motor_pad_y = 90;
motor_bearing_z = 190;
motor_pad_size = [50, 42, 8];
motor_hole_pitch = [34, 28];

// Existing compact Option-A cheek flange, after the -100 mm compact shift.
cheek_x0 = 455;
cheek_x1 = 515;
// Physical prototype correction: move the complete cheek interface 10 mm
// forward (+X), away from the neighbouring frame support.
cheek_mount_forward_shift = 10;
cheek_hole_x = [465 + cheek_mount_forward_shift,
                485 + cheek_mount_forward_shift];
cheek_hole_y = 132;
cheek_bearing_z = 150;
// Carry the cheek strips past the hole centre so an M5 washer bears on a
// complete printed land instead of sitting on the inner edge of each strip.
cheek_strip_inner_extension = 22; // original 12 mm + 10 mm longer bearing land

// Existing chassis/outer-side bridge landing envelope.
chassis_top_z = 52;
outer_wall_y = 205;

// Prototype section sizes. The load-carrying diagonals avoid an unbraced
// 90-degree PLA corner and put two separate ribs under each motor platform.
plate_t = 8;
frame_rail = 12;
post_w = 14;
rib_w = 14;
fastener_clear_d = 5.5; // M5 clearance, washer + locknut

// The compact chassis ends at X=460. Keep the complete bolt foot behind that
// edge; the open wall reaches forward to the cheek but returns its load to the
// rearward foot through the X-Z diagonal.
foot_x0 = 330;
foot_x1 = 455;
wall_x0 = 395;
wall_x1 = 515;
base_inner_y = 175;
base_outer_y = 245;
base_z0 = chassis_top_z;
base_t = 10;
base_hole_x = [345, 440];
base_hole_y = [188, 232];

mount_color = [0.78, 0.26, 0.08, 0.96];
context_mount_color = [0.62, 0.18, 0.72, 0.70];
context_cheek_color = [0.95, 0.45, 0.05, 0.38];
context_wheel_color = [0.08, 0.09, 0.10, 0.36];
context_motor_color = [0.22, 0.38, 0.58, 0.42];

module beam_between(p0, p1, section=[12, 8, 12]) {
    hull() {
        translate(p0) cube(section, center=true);
        translate(p1) cube(section, center=true);
    }
}

module base_frame_positive() {
    // Full bearing plate. The earlier open rails left the M5 holes at their
    // edges and concentrated the entire motor/wheel moment into a thin joint.
    translate([(foot_x0+foot_x1)/2,
               (base_inner_y+base_outer_y)/2,
               base_z0+base_t/2])
        cube([foot_x1-foot_x0,
              base_outer_y-base_inner_y,
              base_t], center=true);
}

module outer_wall_frame_positive() {
    // Start at the bottom of the full foot, not on its top face. This gives
    // the wall/header a 10 mm structural overlap with the chassis plate.
    wall_z0 = base_z0;
    wall_z1 = motor_bearing_z + plate_t;

    // Two posts and three headers form an open side frame.
    for (xx = [wall_x0 + post_w/2, wall_x1 - post_w/2])
        translate([xx, outer_wall_y,
                   (wall_z0+wall_z1)/2])
            cube([post_w, plate_t, wall_z1-wall_z0], center=true);
    for (zz = [wall_z0+frame_rail/2,
               cheek_bearing_z+plate_t/2,
               wall_z1-frame_rail/2])
        translate([(wall_x0+wall_x1)/2, outer_wall_y, zz])
            cube([wall_x1-wall_x0, plate_t, frame_rail], center=true);

    // One X-Z diagonal prevents the open wall from racking longitudinally.
    beam_between([wall_x0+18, outer_wall_y, wall_z0+14],
                 [wall_x1-18, outer_wall_y, wall_z1-14],
                 [frame_rail, plate_t, frame_rail]);

    // Opposing load-return diagonal brings the forward motor/cheek moment
    // directly into the full foot inside the X<=455 chassis envelope.
    beam_between([foot_x1-20, outer_wall_y, base_z0+base_t/2],
                 [wall_x1-18, outer_wall_y, wall_z1-18],
                 [16, 12, 16]);

    // Broad local root at the rear wall/foot intersection.
    beam_between([wall_x0+12, outer_wall_y, base_z0+base_t/2],
                 [wall_x0+12, outer_wall_y, base_z0+42],
                 [22, 16, 14]);
}

module motor_platform_positive() {
    // Full local bearing pad plus two narrow rails back to the outer frame.
    translate([motor_pad_x, motor_pad_y,
               motor_bearing_z+plate_t/2])
        cube(motor_pad_size, center=true);

    for (xx = [motor_pad_x-motor_hole_pitch[0]/2,
               motor_pad_x+motor_hole_pitch[0]/2]) {
        translate([xx, (motor_pad_y+outer_wall_y)/2,
                   motor_bearing_z+plate_t/2])
            cube([rib_w, outer_wall_y-motor_pad_y, plate_t], center=true);

        // Y-Z diagonal below each rail carries vertical motor/wheel load into
        // the outer foot instead of bending an unsupported Γ corner.
        beam_between([xx, motor_pad_y+30, motor_bearing_z-8],
                     [xx, outer_wall_y, chassis_top_z+48],
                     [rib_w, 10, 12]);
    }
}

module cheek_platform_positive() {
    // Two strips line up with the existing cheek fasteners. The central ball
    // corridor and the area below the cheek remain open. Each strip extends
    // 22 mm inward past its fastener. The extra 10 mm gives the removable
    // cheek base room to seat without touching the neighbouring support.
    cheek_strip_inner_y = cheek_hole_y-cheek_strip_inner_extension;
    for (xx = cheek_hole_x) {
        translate([xx, (cheek_strip_inner_y+outer_wall_y)/2,
                   cheek_bearing_z+plate_t/2])
            cube([frame_rail, outer_wall_y-cheek_strip_inner_y,
                  plate_t], center=true);

        beam_between([xx, cheek_hole_y+18, cheek_bearing_z-8],
                     [xx, outer_wall_y, chassis_top_z+24],
                     [frame_rail, 10, 12]);
    }
}

module gamma_positive_raw() {
    union() {
        base_frame_positive();
        outer_wall_frame_positive();
        motor_platform_positive();
        cheek_platform_positive();
    }
}

module gamma_positive() {
    difference() {
        gamma_positive_raw();

        // Four broad-foot M5 fasteners into the rigid chassis side strip.
        for (xx = base_hole_x, yy = base_hole_y)
            translate([xx, yy, base_z0-1])
                cylinder(d=fastener_clear_d, h=base_t+2);

        // Existing v6 motor-mount four-hole pattern.
        for (dx = [-motor_hole_pitch[0]/2, motor_hole_pitch[0]/2],
             dy = [-motor_hole_pitch[1]/2, motor_hole_pitch[1]/2])
            translate([motor_pad_x+dx, motor_pad_y+dy,
                       motor_bearing_z-1])
                cylinder(d=4.5, h=plate_t+2);

        // Existing compact cheek-flange holes.
        for (xx = cheek_hole_x)
            translate([xx, cheek_hole_y, cheek_bearing_z-1])
                cylinder(d=5.5, h=plate_t+2);
    }
}

module gamma(side=1) {
    color(mount_color)
        if (side > 0) gamma_positive();
        else mirror([0, 1, 0]) gamma_positive();
}

module wheel_context(side=1) {
    color(context_wheel_color)
        translate([wheel_x, side*wheel_y, wheel_z])
            rotate([0, axis_tilt_deg, 0])
                difference() {
                    cylinder(d=wheel_d, h=wheel_width, center=true);
                    cylinder(d=44, h=wheel_width+2, center=true);
                }
}

module motor_context(side=1) {
    motor_face_s = wheel_width/2 - 8 + 30;
    color(context_motor_color)
        translate([wheel_x, side*wheel_y, wheel_z])
            rotate([0, axis_tilt_deg, 0])
                translate([0, 0, motor_face_s])
                    cylinder(d=30, h=70);
}

module mount_context(side=1) {
    color(context_mount_color)
        translate([wheel_x, 0, 0])
            if (side > 0)
                import("stl/motor_mount_single_test_v6_split_half_ring_34p6mm_M4.stl");
            else
                mirror([0, 1, 0])
                    import("stl/motor_mount_single_test_v6_split_half_ring_34p6mm_M4.stl");
}

module cheek_context(side=1) {
    color(context_cheek_color)
        translate([-100, 0, 0])
            import(side > 0
                ? "../collector-intake-v1/option-a/stl/cheek_left.stl"
                : "../collector-intake-v1/option-a/stl/cheek_right.stl");
}

module context() {
    if (show_context)
        for (side = [-1, 1]) {
            wheel_context(side);
            motor_context(side);
            mount_context(side);
            cheek_context(side);
        }
}

module assembly() {
    gamma(1);
    gamma(-1);
    if (show_crossbar) cheek_crossbar_with_cover_world();
    context();
}

module ball_context() {
    // Nominal 66 mm ball at the frozen first-wheel-contact X datum.
    translate([481.2, 0, 33]) sphere(d=66);
}

module cheek_crossbar_interference() {
    intersection() {
        cheek_crossbar_with_cover_world();
        union() {
            if (crossbar_interference_target == "all" ||
                crossbar_interference_target == "wheel")
                for (side = [-1, 1]) wheel_context(side);
            if (crossbar_interference_target == "all" ||
                crossbar_interference_target == "motor")
                for (side = [-1, 1]) motor_context(side);
            if (crossbar_interference_target == "all" ||
                crossbar_interference_target == "mount")
                for (side = [-1, 1]) mount_context(side);
            if (crossbar_interference_target == "all" ||
                crossbar_interference_target == "cheek")
                for (side = [-1, 1]) cheek_context(side);
            if (crossbar_interference_target == "all" ||
                crossbar_interference_target == "ball") ball_context();
        }
    }
}

module hard_interference() {
    intersection() {
        union() { gamma(1); gamma(-1); }
        union()
            for (side = [-1, 1]) {
                wheel_context(side);
                motor_context(side);
            }
    }
}

// Upright orientation: the open chassis foot is on the bed. The two Y-Z
// diagonals rise at approximately 45 degrees under the cantilever rails, so
// the load path is printed continuously from the foot. Envelope 185 x 176 x
// 146 mm, inside the Bambu Lab P2S 256 mm cube.
module print_oriented(side=1) {
    print_min_y = motor_pad_y-motor_pad_size[1]/2;
    if (side > 0)
        translate([-foot_x0, -print_min_y, -base_z0]) gamma(1);
    else
        // gamma(-1) is already the mirrored/right-hand part. Rotate it on the
        // build plate instead of mirroring it a second time, which would turn
        // the exported right STL back into a left-hand part.
        translate([wall_x1, -print_min_y, -base_z0])
            rotate([0, 0, 180]) gamma(-1);
}

if (part == "left") gamma(1);
else if (part == "right") gamma(-1);
else if (part == "left_print") print_oriented(1);
else if (part == "right_print") print_oriented(-1);
else if (part == "cheek_crossbar_interference") cheek_crossbar_interference();
else if (part == "hard_interference") hard_interference();
else assembly();

assert(foot_x1 <= 460,
       "all chassis-foot material must remain behind compact chassis X=460");
assert(max(base_hole_x) + fastener_clear_d/2 <= 460,
       "base fastener must remain inside the compact chassis front edge");
