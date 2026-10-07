// Removable front IR break-beam hangers for the already printed Γ brackets.
// Units: mm; compact robot ground frame (+X forward).
//
// Each hanger drops onto the clear part of the Γ horizontal header with two
// inverted J-hooks, then carries one provisional 20x16x16 sensor behind the
// cheek at the wheel-first entry datum X=460/Z=40. Integrated smaller J-hooks
// route its cable upward on the outside of the ball corridor.

$fn = 48;

use <dual-outboard-gamma-brackets.scad>

part = "assembly"; // [assembly,left,right,left_print,right_print,left_review,hard_interference,gamma_interference]
show_context = true;

sensor_x = 460;
sensor_y = 110;
sensor_z = 40;
sensor_size = [20, 16, 16];

header_y = 205;
header_z = 154;
mount_x = 447;
mount_plate_size = [34, 5, 30];
mount_plate_y = 198; // 0.5 mm free from header inner face Y=201
hanger_hook_x = [438, 456];
hanger_hook_w = 9;

spine_y = 168; // inside the prototype foot, whose inner edge is Y=175
spine_x = 440.837; // centred in the clear gap between the two motor-support ribs
spine_x_w = 18;
spine_y_t = 6;
shelf_size = [30, 30, 5];
shelf_top_z = sensor_z-sensor_size[2]/2;
zip_slot_w = 3.6;

module beam_yz(p0, p1, section=[12, 8, 12]) {
    hull() {
        translate(p0) cube(section, center=true);
        translate(p1) cube(section, center=true);
    }
}

module horizontal_cable_clip(x, z, side=1) {
    // Horizontal C-clip around a vertical cable. The two arms are in the X-Y
    // plane and remain open on the free side, so the lead pushes in laterally
    // instead of being threaded through a hole or an upward-facing J-hook.
    spine_edge_y = spine_y+side*spine_y_t/2;
    clip_inner_y = spine_edge_y-side*1;
    clip_outer_y = spine_edge_y+side*10;
    clip_mid_y = (clip_inner_y+clip_outer_y)/2;
    clip_len_y = abs(clip_outer_y-clip_inner_y);
    for (dx = [-4.5, 4.5])
        translate([x+dx, clip_mid_y, z])
            cube([2.5, clip_len_y, 3], center=true);
}

module header_hanging_hooks() {
    // Header envelope: Y=201..209, Z=148..160. The 0.5 mm clearances let the
    // add-on drop on from above without flexing or loading the printed Γ.
    for (xx = hanger_hook_x) {
        translate([xx, 205.25, 163])
            cube([hanger_hook_w, 13.5, 5], center=true);
        translate([xx, 211.75, 157])
            cube([hanger_hook_w, 4.5, 11], center=true);
    }
}

module hanger_positive_raw() {
    union() {
        // Flat locating pad plus two gravity-loaded hooks on the clear header
        // segment X=430..464. This avoids the diagonal and the cheek rail.
        translate([mount_x, mount_plate_y, header_z])
            cube(mount_plate_size, center=true);
        header_hanging_hooks();

        // Bring the hanging point inboard before dropping the spine. This
        // clears the broad chassis foot of the already printed prototype Γ.
        beam_yz([spine_x, mount_plate_y, 142],
                [spine_x, spine_y, 142],
                [spine_x_w, 6, 12]);

        // Vertical spine plus a diagonal to the sensor shelf. The diagonal
        // carries the tiny sensor load without a long flexible cantilever.
        translate([spine_x, spine_y, 95])
            cube([spine_x_w, spine_y_t, 110], center=true);
        beam_yz([spine_x, spine_y, 142],
                [sensor_x, sensor_y+12, shelf_top_z+6],
                [12, 8, 12]);

        translate([sensor_x, sensor_y, shelf_top_z-shelf_size[2]/2])
            cube(shelf_size, center=true);

        // Outer backstop squares the module while its zip tie is tightened.
        translate([sensor_x,
                   sensor_y+sensor_size[1]/2+2,
                   sensor_z])
            cube([24, 4, 22], center=true);

        // Three horizontal clips retain the cable while it rises vertically
        // on the free face of the spine, opposite the diagonal.
        for (zz = [68, 96, 124])
            horizontal_cable_clip(spine_x, zz, 1);
    }
}

module hanger_positive() {
    difference() {
        hanger_positive_raw();

        // Sensor-retaining zip tie rises through these shelf slots and passes
        // over the module. A second tie may be fitted for outdoor vibration.
        for (xx = [448.5, 471.5])
            translate([xx, 105,
                       shelf_top_z-shelf_size[2]/2])
                cube([zip_slot_w, 18, shelf_size[2]+2], center=true);

        // Optional upper strain-relief tie through the cable spine.
        translate([spine_x, spine_y, 136])
            cube([9, spine_y_t+2, 3.6], center=true);
    }
}

module hanger(side=1) {
    if (side > 0) hanger_positive();
    else mirror([0, 1, 0]) hanger_positive();
}

module sensor_context(side=1) {
    color([0.90, 0.10, 0.10, 0.65])
        translate([sensor_x, side*sensor_y, sensor_z])
            cube(sensor_size, center=true);
}

module beam_context() {
    color([1.0, 0.05, 0.05, 0.55])
        translate([sensor_x, 0, sensor_z])
            rotate([90, 0, 0]) cylinder(d=3, h=2*sensor_y, center=true);
}

module printed_gamma_context(side=1) {
    // Reconstruct the installed coordinates from the two prototype STLs that
    // are already printed.  Keeping this add-on tied to those meshes avoids a
    // false fit check against the later reinforced-v2 Γ revision.
    color([0.78, 0.26, 0.08, 0.38])
        if (side > 0)
            translate([330, 69, 52])
                import("stl/gamma_bracket_left_prototype.stl", convexity=10);
        else
            rotate([0, 0, -180])
                translate([-515, 69, 52])
                    import("stl/gamma_bracket_right_prototype.stl", convexity=10);
}

module assembly() {
    hanger(1);
    hanger(-1);
    sensor_context(1);
    sensor_context(-1);
    beam_context();
    if (show_context) {
        printed_gamma_context(1);
        printed_gamma_context(-1);
        for (side = [-1, 1]) {
            wheel_context(side);
            motor_context(side);
            cheek_context(side);
        }
    }
}

module left_review() {
    color([0.92, 0.65, 0.05]) hanger(1);
    sensor_context(1);
    color([0.78, 0.26, 0.08, 0.42])
        translate([455, header_y, header_z])
            cube([120, 8, 12], center=true);
    color([1.0, 0.05, 0.05, 0.55])
        translate([sensor_x, sensor_y/2, sensor_z])
            rotate([90, 0, 0]) cylinder(d=3, h=sensor_y, center=true);
}

module hard_interference() {
    intersection() {
        union() { hanger(1); hanger(-1); }
        union()
            for (side = [-1, 1]) {
                wheel_context(side);
                motor_context(side);
                cheek_context(side);
            }
    }
}

module gamma_interference() {
    intersection() {
        union() { hanger(1); hanger(-1); }
        union() {
            printed_gamma_context(1);
            printed_gamma_context(-1);
        }
    }
}

// Lay the broad Y-Z side frame on the bed; maximum print height is the
// 45 mm X envelope rather than the 140 mm installed height.
module print_oriented(side=1) {
    if (side > 0)
        rotate([0, 90, 0]) translate([-475, 0, 0]) hanger(1);
    else
        rotate([0, 90, 0]) translate([-475, 0, 0]) hanger(-1);
}

if (part == "assembly") assembly();
else if (part == "left") hanger(1);
else if (part == "right") hanger(-1);
else if (part == "left_print") print_oriented(1);
else if (part == "right_print") print_oriented(-1);
else if (part == "left_review") left_review();
else if (part == "hard_interference") hard_interference();
else if (part == "gamma_interference") gamma_interference();

assert(sensor_x-sensor_size[0]/2 > 442,
       "IR body must stay forward of the calculated wheel X envelope");
assert(sensor_z > 33,
       "IR beam should cross above the nominal resting ball centre");
