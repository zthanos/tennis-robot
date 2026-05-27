include <common.scad>

// Full-size chassis mounting/drill template for the first tennis robot base.
// This is intended as a CAD reference for a 21 mm birch/marine plywood plate.
// It can also be split into printable base sections later. It shows the mounting holes and
// module envelopes for the collector, motor pods, front casters, stabilizers,
// battery, electronics, handle sockets, and a reserved launcher/feed zone.
// With show_verticals=true it also shows the upright posts, trays, and rails
// that components will bolt to above the base plate.

base_len = 760;
base_w = 430;
plate_t = 21;
corner_r = 16;

hole_d = 5.2;          // M5 clearance, or M4 with washers
slot_len = 26;
slot_w = hole_d;
mark_h = 1.4;
plywood_t = 21;
plywood_rail_h = 45;
frame_tube = plywood_t;
frame_post_h = 155;
electronics_standoff_h = 34;
collector_upright_h = 126;
launcher_upright_h = 235;

motor_pod_len = 150;
motor_pod_w = 76;
motor_pod_flange_len = 174;
motor_pod_flange_w = 100;

caster_mount_x = 92;
caster_mount_y = 78;
front_caster_outboard_gap = 18;
front_caster_w = 32;
front_caster_left_y = -front_caster_w / 2 - front_caster_outboard_gap;
front_caster_right_y = base_w + front_caster_w / 2 + front_caster_outboard_gap;
front_caster_anchor_ys = [34, base_w - 34];

collector_len = 360;
collector_w = 360;
collector_origin_x = base_len - collector_len - 15;
collector_origin_y = base_w / 2;
collector_slot_xs = [45, 135, 225, 315];
collector_slot_ys = [-125, 125];

battery_x = 250;
battery_y = base_w / 2;
battery_len = 170;
battery_w = 150;
battery_h = 58;
battery_clearance = 12;

module material_base() {
    color([0.70, 0.52, 0.32]) children();
}

module material_cutout() {
    color([0.96, 0.96, 0.92]) children();
}

module material_collector() {
    color([0.95, 0.63, 0.13]) children();
}

module material_drive() {
    color([0.15, 0.17, 0.19]) children();
}

module material_caster() {
    color([0.28, 0.40, 0.58]) children();
}

module material_power() {
    color([0.26, 0.38, 0.29]) children();
}

module material_electronics() {
    color([0.42, 0.45, 0.48]) children();
}

module material_launcher() {
    color([0.50, 0.28, 0.60]) children();
}

module material_handle() {
    color([0.95, 0.72, 0.18]) children();
}

module material_frame() {
    color([0.08, 0.08, 0.09]) children();
}

module material_tray() {
    color([0.78, 0.82, 0.84]) children();
}

module material_stabilizer() {
    color([0.36, 0.45, 0.34]) children();
}

module slot_x(len=slot_len, d=slot_w, h=plate_t + 4) {
    hull() {
        translate([-len / 2, 0, -2]) cylinder(h=h, d=d);
        translate([len / 2, 0, -2]) cylinder(h=h, d=d);
    }
}

module slot_y(len=slot_len, d=slot_w, h=plate_t + 4) {
    rotate([0, 0, 90]) slot_x(len, d, h);
}

module drill_slot_marker_x(len=slot_len, d=slot_w) {
    material_cutout()
        translate([0, 0, plate_t + 0.08])
            hull() {
                translate([-len / 2, 0, 0]) cylinder(h=mark_h, d=d + 8);
                translate([len / 2, 0, 0]) cylinder(h=mark_h, d=d + 8);
            }
}

module drill_slot_marker_y(len=slot_len, d=slot_w) {
    rotate([0, 0, 90]) drill_slot_marker_x(len, d);
}

module screw_marker(d=hole_d) {
    material_cutout()
        translate([0, 0, plate_t + 0.08])
            cylinder(h=mark_h, d=d + 8);
}

module envelope(size=[100, 50], r=6, z=plate_t + 0.15) {
    translate([0, 0, z])
        rounded_plate([size[0], size[1], mark_h], r);
}

module label(txt, pos, size=12) {
    color([0.08, 0.08, 0.08])
        translate([pos[0], pos[1], plate_t + 2.0])
            linear_extrude(height=0.8)
                text(txt, size=size, halign="center", valign="center", font="Arial:style=Bold");
}

module vertical_post(pos, h=frame_post_h, size=frame_tube) {
    translate([pos[0] - size / 2, pos[1] - size / 2, plate_t])
        cube([size, size, h]);
}

module vertical_round_standoff(pos, h=electronics_standoff_h, d=14) {
    translate([pos[0], pos[1], plate_t])
        cylinder(h=h, d=d);
}

module upper_rail_x(x0, x1, y, z, size=frame_tube) {
    translate([x0, y - size / 2, z])
        cube([x1 - x0, size, size]);
}

module upper_rail_y(x, y0, y1, z, size=frame_tube) {
    translate([x - size / 2, y0, z])
        cube([size, y1 - y0, size]);
}

module vertical_panel(pos, size=[8, 90, 120]) {
    translate([pos[0] - size[0] / 2, pos[1] - size[1] / 2, plate_t])
        cube(size);
}

module plywood_rail_x(x0, x1, y, h=plywood_rail_h) {
    translate([x0, y - plywood_t / 2, plate_t])
        cube([x1 - x0, plywood_t, h]);
}

module plywood_rail_y(x, y0, y1, h=plywood_rail_h) {
    translate([x - plywood_t / 2, y0, plate_t])
        cube([plywood_t, y1 - y0, h]);
}

module chassis_plate_cutouts() {
    // Motor pod chassis flange holes. Pods sit at the rear/side edges.
    for (x = [110, 170, 230]) {
        for (y = [18, 86]) translate([x, y, 0]) screw_hole(hole_d, plate_t + 4);
        for (y = [base_w - 86, base_w - 18]) translate([x, y, 0]) screw_hole(hole_d, plate_t + 4);
    }

    // Outboard front caster outrigger anchors. The caster wheel/plate sits
    // outside the base edge; these slots fasten the inner outrigger bracket.
    for (y_center = front_caster_anchor_ys) {
        x0 = base_len - 110 - caster_mount_x / 2;
        y0 = y_center - caster_mount_y / 2;
        for (x = [x0 + 22, x0 + caster_mount_x - 22]) {
            translate([x, y0 + 12, 0]) slot_x();
            translate([x, y0 + caster_mount_y - 12, 0]) slot_y();
        }
    }

    // Collector mounting plate slots, matching collector_funnel_bin.scad.
    for (x = collector_slot_xs) {
        for (y = collector_slot_ys) {
            translate([collector_origin_x + x, collector_origin_y + y, 0]) slot_x();
        }
    }

    // Battery strap slots. Use webbing/metal straps through these slots.
    for (x = [205, 295]) {
        translate([x, base_w / 2 - 82, 0]) slot_y(36, hole_d);
        translate([x, base_w / 2 + 82, 0]) slot_y(36, hole_d);
    }

    // Electronics/controller standoffs, kept behind the battery.
    for (x = [110, 230]) {
        for (y = [145, base_w - 145]) {
            translate([x, y, 0]) screw_hole(4.3, plate_t + 4);
        }
    }

    // Motor driver / power board standoffs.
    for (x = [300, 390]) {
        for (y = [145, base_w - 145]) {
            translate([x, y, 0]) screw_hole(4.3, plate_t + 4);
        }
    }

    // Handle socket holes at the rear.
    for (y0 = [62, base_w - 106]) {
        for (x = [22, 72]) {
            for (y = [8, 46]) {
                translate([x, y0 + y, 0]) screw_hole(hole_d, plate_t + 4);
            }
        }
    }

    // Front stabilizer brackets / hinge blocks.
    for (y = [base_w / 2 - 62, base_w / 2 + 62]) {
        translate([base_len - 120, y - 22, 0]) slot_x(34, hole_d);
        translate([base_len - 120, y + 22, 0]) slot_x(34, hole_d);
    }

    // Reserved launcher/feed module inserts. These are pilot holes only.
    for (x = [500, 570, 640]) {
        for (y = [base_w / 2 - 65, base_w / 2 + 65]) {
            translate([x, y, 0]) screw_hole(4.3, plate_t + 4);
        }
    }
}

module chassis_plate_markers() {
    // Centerline and front/rear orientation markers.
    color([0.1, 0.1, 0.1, 0.55]) {
        translate([0, base_w / 2 - 1, plate_t + 0.25])
            cube([base_len, 2, mark_h]);
        translate([base_len - 44, base_w / 2 - 9, plate_t + 0.25])
            cube([34, 18, mark_h]);
    }
    label("FRONT", [base_len - 70, base_w / 2], 16);
    label("REAR", [58, base_w / 2], 14);

    // Module envelopes.
    material_drive() {
        translate([83, -8, 0]) envelope([motor_pod_flange_len, motor_pod_flange_w], 10);
        translate([83, base_w - motor_pod_flange_w + 8, 0]) envelope([motor_pod_flange_len, motor_pod_flange_w], 10);
    }
    label("LEFT MOTOR POD", [170, 47], 10);
    label("RIGHT MOTOR POD", [170, base_w - 47], 10);

    material_caster() {
        for (y_center = [front_caster_left_y, front_caster_right_y]) {
            translate([base_len - 110 - caster_mount_x / 2, y_center - caster_mount_y / 2, 0])
                envelope([caster_mount_x, caster_mount_y], 8);
        }
    }
    label("OUTBOARD CASTER", [base_len - 110, 18], 8);
    label("OUTBOARD CASTER", [base_len - 110, base_w - 18], 8);

    material_collector()
        translate([collector_origin_x, base_w / 2 - collector_w / 2, 0])
            envelope([collector_len, collector_w], 10);
    label("COLLECTOR MOUNT", [collector_origin_x + collector_len / 2, base_w / 2], 14);

    material_power()
        translate([battery_x - battery_len / 2, battery_y - battery_w / 2, 0])
            envelope([battery_len, battery_w], 8);
    label("BATTERY LOW", [250, base_w / 2], 13);

    material_electronics() {
        translate([85, 118, 0]) envelope([170, 54], 7);
        translate([85, base_w - 172, 0]) envelope([170, 54], 7);
        translate([280, 118, 0]) envelope([130, 54], 7);
        translate([280, base_w - 172, 0]) envelope([130, 54], 7);
    }
    label("CTRL", [170, 145], 10);
    label("CTRL", [170, base_w - 145], 10);
    label("DRIVERS", [345, 145], 10);
    label("DRIVERS", [345, base_w - 145], 10);

    material_launcher()
        translate([470, base_w / 2 - 92, 0])
            envelope([210, 184], 9);
    label("RESERVED FEED / LAUNCHER", [575, base_w / 2], 11);

    material_handle() {
        translate([10, 62, 0]) envelope([78, 54], 7);
        translate([10, base_w - 106, 0]) envelope([78, 54], 7);
    }
    label("HANDLE", [49, 89], 8);
    label("HANDLE", [49, base_w - 79], 8);

    material_stabilizer() {
        translate([base_len - 155, base_w / 2 - 88, 0]) envelope([70, 52], 8);
        translate([base_len - 155, base_w / 2 + 36, 0]) envelope([70, 52], 8);
    }
    label("STAB", [base_len - 120, base_w / 2 - 62], 9);
    label("STAB", [base_len - 120, base_w / 2 + 62], 9);

    // Hole/slot markers, repeated above the plate so the drill pattern is easy
    // to inspect even when the through-cut holes are hard to see in preview.
    for (x = [110, 170, 230]) {
        for (y = [18, 86]) translate([x, y, 0]) screw_marker();
        for (y = [base_w - 86, base_w - 18]) translate([x, y, 0]) screw_marker();
    }

    for (y_center = front_caster_anchor_ys) {
        x0 = base_len - 110 - caster_mount_x / 2;
        y0 = y_center - caster_mount_y / 2;
        for (x = [x0 + 22, x0 + caster_mount_x - 22]) {
            translate([x, y0 + 12, 0]) drill_slot_marker_x();
            translate([x, y0 + caster_mount_y - 12, 0]) drill_slot_marker_y();
        }
    }

    for (x = collector_slot_xs) {
        for (y = collector_slot_ys) {
            translate([collector_origin_x + x, collector_origin_y + y, 0]) drill_slot_marker_x();
        }
    }

    for (x = [205, 295]) {
        translate([x, base_w / 2 - 82, 0]) drill_slot_marker_y(36, hole_d);
        translate([x, base_w / 2 + 82, 0]) drill_slot_marker_y(36, hole_d);
    }

    for (x = [110, 230]) {
        for (y = [145, base_w - 145]) translate([x, y, 0]) screw_marker(4.3);
    }
    for (x = [300, 390]) {
        for (y = [145, base_w - 145]) translate([x, y, 0]) screw_marker(4.3);
    }

    for (y0 = [62, base_w - 106]) {
        for (x = [22, 72]) {
            for (y = [8, 46]) translate([x, y0 + y, 0]) screw_marker();
        }
    }

    for (y = [base_w / 2 - 62, base_w / 2 + 62]) {
        translate([base_len - 120, y - 22, 0]) drill_slot_marker_x(34, hole_d);
        translate([base_len - 120, y + 22, 0]) drill_slot_marker_x(34, hole_d);
    }

    for (x = [500, 570, 640]) {
        for (y = [base_w / 2 - 65, base_w / 2 + 65]) translate([x, y, 0]) screw_marker(4.3);
    }
}

module vertical_mounting_elements() {
    top_z = plate_t + frame_post_h;

    // Main plywood-on-edge rails cut from the same 21 mm sheet. These stiffen
    // the lower plate without making the whole chassis much heavier.
    material_frame() {
        plywood_rail_x(35, base_len - 35, 45);
        plywood_rail_x(35, base_len - 35, base_w - 45);
        // Split cross rails leave a removable battery bay in the middle.
        plywood_rail_y(170, 55, battery_y - battery_w / 2 - battery_clearance);
        plywood_rail_y(170, battery_y + battery_w / 2 + battery_clearance, base_w - 55);
        plywood_rail_y(395, 55, battery_y - battery_w / 2 - battery_clearance);
        plywood_rail_y(395, battery_y + battery_w / 2 + battery_clearance, base_w - 55);
        plywood_rail_y(650, 55, base_w - 55);
    }

    // Taller reference posts/upper rails for cover, hopper, and later launcher
    // brackets. These can be aluminum angle or removable plywood uprights.
    color([0.08, 0.08, 0.09, 0.55]) {
        for (x = [70, base_len - 70]) {
            for (y = [45, base_w - 45]) {
                vertical_post([x, y]);
            }
        }

        upper_rail_x(70, base_len - 70, 45, top_z);
        upper_rail_x(70, base_len - 70, base_w - 45, top_z);
        upper_rail_y(70, 45, base_w - 45, top_z);
        upper_rail_y(base_len - 70, 45, base_w - 45, top_z);
    }

    // Electronics trays: low standoffs above the plate, still below the hopper.
    material_tray() {
        for (x = [110, 230]) {
            for (y = [145, base_w - 145]) {
                vertical_round_standoff([x, y], electronics_standoff_h, 13);
            }
        }
        translate([92, 122, plate_t + electronics_standoff_h])
            rounded_plate([156, 46, 4], 6);
        translate([92, base_w - 168, plate_t + electronics_standoff_h])
            rounded_plate([156, 46, 4], 6);

        for (x = [300, 390]) {
            for (y = [145, base_w - 145]) {
                vertical_round_standoff([x, y], electronics_standoff_h, 13);
            }
        }
        translate([285, 122, plate_t + electronics_standoff_h])
            rounded_plate([120, 46, 4], 6);
        translate([285, base_w - 168, plate_t + electronics_standoff_h])
            rounded_plate([120, 46, 4], 6);
    }

    // Removable battery bay. The battery slides out from either side after the
    // top strap/clamp is removed; these stops only prevent fore/aft motion.
    material_power() {
        translate([battery_x - battery_len / 2 - 8, battery_y - battery_w / 2, plate_t])
            cube([8, battery_w, battery_h]);
        translate([battery_x + battery_len / 2, battery_y - battery_w / 2, plate_t])
            cube([8, battery_w, battery_h]);

        // Battery placeholder volume with a little top clearance.
        color([0.18, 0.23, 0.18, 0.42])
            translate([battery_x - battery_len / 2, battery_y - battery_w / 2, plate_t + 2])
                cube([battery_len, battery_w, battery_h]);

        // Removable strap/clamp shown above the battery. Do not glue this part.
        color([0.05, 0.05, 0.05, 0.75])
            translate([battery_x - battery_len / 2 - 12, battery_y - 8, plate_t + battery_h + 8])
                cube([battery_len + 24, 16, 6]);
    }

    // Collector uprights. These are the side plates/back-plate support points
    // that hold the wide roller bracket and hopper mouth above the intake lip.
    material_collector() {
        for (y = [base_w / 2 - 98, base_w / 2 + 98]) {
            vertical_panel([collector_origin_x + 160, y], [plywood_t, 90, collector_upright_h]);
            vertical_panel([collector_origin_x + 240, y], [plywood_t, 90, collector_upright_h]);
        }
        upper_rail_y(collector_origin_x + 160, base_w / 2 - 98, base_w / 2 + 98, plate_t + collector_upright_h, 14);
        upper_rail_y(collector_origin_x + 240, base_w / 2 - 98, base_w / 2 + 98, plate_t + collector_upright_h, 14);
        translate([collector_origin_x + 162, base_w / 2 - 88, plate_t + 70])
            cube([82, 176, 8]);
    }
    label("COLLECTOR UPRIGHTS", [collector_origin_x + 205, base_w / 2], 10);

    // Reserved launcher/feed uprights. Keep them in CAD now so we do not place
    // electronics where the later feed path needs structure.
    material_launcher() {
        for (x = [500, 640]) {
            for (y = [base_w / 2 - 72, base_w / 2 + 72]) {
                vertical_post([x, y], launcher_upright_h, 18);
            }
        }
        upper_rail_y(500, base_w / 2 - 72, base_w / 2 + 72, plate_t + launcher_upright_h, 18);
        upper_rail_y(640, base_w / 2 - 72, base_w / 2 + 72, plate_t + launcher_upright_h, 18);
        upper_rail_x(500, 640, base_w / 2 - 72, plate_t + launcher_upright_h, 18);
        upper_rail_x(500, 640, base_w / 2 + 72, plate_t + launcher_upright_h, 18);
    }
    label("LAUNCHER UPRIGHTS LATER", [570, base_w / 2], 9);

    // Handle rails rise from the rear sockets.
    material_handle() {
        translate([47, 82, plate_t])
            cube([16, 14, 260]);
        translate([47, base_w - 96, plate_t])
            cube([16, 14, 260]);
        translate([20, 82, plate_t + 260])
            cube([82, base_w - 178, 16]);
    }
}

module base_mounting_plate(show_markers=true, show_verticals=true) {
    difference() {
        material_base()
            rounded_plate([base_len, base_w, plate_t], corner_r);
        chassis_plate_cutouts();
    }

    if (show_markers) {
        chassis_plate_markers();
    }

    if (show_verticals) {
        vertical_mounting_elements();
    }
}

base_mounting_plate();
