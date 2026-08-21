include <params.scad>

module rounded_rect_2d(size, r) {
    hull()
        for (x = [r, size[0] - r], y = [r, size[1] - r])
            translate([x, y]) circle(r = r);
}

module capsule_2d(length, width, horizontal = true) {
    hull() {
        if (horizontal) {
            translate([-(length - width) / 2, 0]) circle(d = width);
            translate([ (length - width) / 2, 0]) circle(d = width);
        } else {
            translate([0, -(length - width) / 2]) circle(d = width);
            translate([0,  (length - width) / 2]) circle(d = width);
        }
    }
}

module through_capsule(pos, length, width, horizontal = true, zmax = 20) {
    translate([pos[0], pos[1], -1])
        linear_extrude(height = zmax)
            capsule_2d(length, width, horizontal);
}

module cross_slot(pos, span = driver_adjust_span, width = m3_clearance_d) {
    translate([pos[0], pos[1], -1])
        linear_extrude(height = tray_t + driver_standoff_h + 3)
            union() {
                capsule_2d(span, width, true);
                capsule_2d(span, width, false);
            }
}

module fixed_standoff(pos, height) {
    translate([pos[0], pos[1], tray_t])
        cylinder(d = standoff_d, h = height);
}

module fixed_standoff_hole(pos, height) {
    translate([pos[0], pos[1], -1])
        cylinder(d = m3_clearance_d, h = tray_t + height + 3);
}

module driver_slot_boss(pos) {
    translate([pos[0] - 6, pos[1] - 6, tray_t])
        cube([12, 12, driver_standoff_h]);
}

module driver_mount_positions(origin) {
    for (dx = [driver_nominal_inset, driver_size[0] - driver_nominal_inset],
         dy = [driver_nominal_inset, driver_size[1] - driver_nominal_inset])
        translate([origin[0] + dx, origin[1] + dy]) children();
}

module raised_label(txt, pos, size = label_size, halign = "left") {
    translate([pos[0], pos[1], tray_t])
        linear_extrude(height = label_h)
            text(txt, size = size, halign = halign, valign = "center");
}

module divider_ribs() {
    // Gaps between ribs are deliberate wire crossings. Route encoder/logic
    // through different gaps from B+/B-/M+/M-.
    for (segment = [[6, 42], [58, 42], [110, 42], [162, 72]])
        translate([segment[0], 72, tray_t]) cube([segment[1], 2.5, 4]);
}

module electronics_tray() {
    difference() {
        union() {
            linear_extrude(height = tray_t)
                rounded_rect_2d(tray_size, tray_corner_r);

            for (p = mega_holes)
                fixed_standoff(mega_origin + p, mega_standoff_h);
            for (p = perf_holes)
                fixed_standoff(perf_origin + p, perf_standoff_h);
            for (origin = driver_origins)
                driver_mount_positions(origin) driver_slot_boss([0, 0]);

            divider_ribs();

            raised_label("LEFT BTS", [8, 66]);
            raised_label("RIGHT BTS", [66, 66]);
            raised_label("RELAY", [124, 66]);
            raised_label("FUSE / DIST", [180, 66]);
            raised_label("MEGA 2560", [6, 103]);
            raised_label("PERFBOARD", [114, 78]);
            raised_label("USB", [2, 136], 3);
        }

        // M3 board mounts.
        for (p = mega_holes)
            fixed_standoff_hole(mega_origin + p, mega_standoff_h);
        for (p = perf_holes)
            fixed_standoff_hole(perf_origin + p, perf_standoff_h);
        for (origin = driver_origins)
            driver_mount_positions(origin)
                cross_slot([0, 0]);

        // M5 chassis attachment slots.
        for (p = chassis_slots)
            through_capsule(p, chassis_slot_len, chassis_slot_w, true);

        // Ventilation below each driver heatsink.
        for (origin = driver_origins)
            for (dy = [15, 25, 35])
                through_capsule(origin + [25, dy], 28, 4.5, true);

        // Ventilation under Mega and perfboard; narrow enough to retain a stiff
        // plate while keeping the solder side visible and serviceable.
        for (x = [36, 56, 76])
            through_capsule([mega_origin[0] + x, mega_origin[1] + 27], 34, 4, false);
        for (x = [24, 48, 72, 96])
            through_capsule([perf_origin[0] + x, perf_origin[1] + 40], 50, 4, false);

        // Universal relay bay: two adjustable M4/M5 bolt or cable-tie slots.
        for (x = [relay_bay_origin[0] + 10, relay_bay_origin[0] + relay_bay_size[0] - 10])
            through_capsule([x, relay_bay_origin[1] + relay_bay_size[1] / 2], 32, 5, false);

        // Fuse/distribution reserve: four tie slots accept multiple holder sizes.
        for (x = [fuse_bay_origin[0] + 10, fuse_bay_origin[0] + fuse_bay_size[0] - 10],
             y = [fuse_bay_origin[1] + 12, fuse_bay_origin[1] + fuse_bay_size[1] - 12])
            through_capsule([x, y], 12, 4.5, false);

        // Cable-tie anchors around both wiring domains.
        for (p = [[55, 78], [105, 78], [175, 78], [232, 78],
                  [112, 18], [112, 50], [174, 18], [174, 50]])
            through_capsule(p, 12, 4, false);
    }
}

electronics_tray();
