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

module through_capsule(pos, length, width, horizontal = true, zmax = 30) {
    translate([pos[0], pos[1], -1])
        linear_extrude(height = zmax)
            capsule_2d(length, width, horizontal);
}

module cross_slot(pos, span = board_adjust_span, width = m3_slot_w, zmax = 30) {
    translate([pos[0], pos[1], -1])
        linear_extrude(height = zmax)
            union() {
                capsule_2d(span, width, true);
                capsule_2d(span, width, false);
            }
}

module standoff(pos, height) {
    translate([pos[0], pos[1], tray_t]) cylinder(d = standoff_d, h = height);
}

module raised_label(txt, pos, size = label_size, halign = "left") {
    translate([pos[0], pos[1], tray_t])
        linear_extrude(height = label_h)
            text(txt, size = size, halign = halign, valign = "center");
}

module add_mounts(origin, holes, height) {
    for (p = holes) standoff(origin + p, height);
}

module cut_mounts(origin, holes, height, span = board_adjust_span) {
    for (p = holes)
        cross_slot(origin + p, span, m3_slot_w, tray_t + height + 3);
}

module electronics_tray() {
    difference() {
        union() {
            linear_extrude(height = tray_t)
                rounded_rect_2d(tray_size, tray_corner_r);

            add_mounts(perf_origin, perf_holes, perf_standoff_h);
            add_mounts(mega_case_origin, mega_case_holes, mega_case_standoff_h);
            add_mounts(l298_origin, l298_holes, l298_standoff_h);
            for (origin = bts_origins)
                add_mounts(origin, bts_holes, bts_standoff_h);

            // Separates the lower driver/power wiring zone from upper logic.
            for (segment = [[6, 48], [66, 48], [126, 48]])
                translate([segment[0], 84, tray_t]) cube([segment[1], 2.5, 4]);

            raised_label("L298N INTAKE", [6, 77]);
            raised_label("LEFT BTS", [66, 77]);
            raised_label("RIGHT BTS", [124, 77]);
            raised_label("PERFBOARD", [6, 105]);
            raised_label("MEGA CASE", [108, 113]);
        }

        cut_mounts(perf_origin, perf_holes, perf_standoff_h, 8);
        cut_mounts(mega_case_origin, mega_case_holes, mega_case_standoff_h, 6);
        cut_mounts(l298_origin, l298_holes, l298_standoff_h, 6);
        for (origin = bts_origins)
            cut_mounts(origin, bts_holes, bts_standoff_h, 6);

        for (p = chassis_slots)
            through_capsule(p, chassis_slot_len, chassis_slot_w, true);

        // Driver ventilation.
        for (dy = [18, 30, 42, 54])
            through_capsule(l298_origin + [l298_size[0] / 2, dy], 34, 4.5, true);
        for (origin = bts_origins)
            for (dy = [14, 25, 36])
                through_capsule(origin + [bts_size[0] / 2, dy], 28, 4.5, true);

        // Large open windows preserve airflow and underside access. The Mega
        // window deliberately covers the enclosure vent field rather than
        // copying its individual slots.
        for (x = [22, 42, 62])
            through_capsule([perf_origin[0] + x, perf_origin[1] + 60], 84, 4, false);
        for (x = [15, 30, 45])
            through_capsule([mega_case_origin[0] + x,
                             mega_case_origin[1] + mega_case_size[1] / 2],
                            76, 5, false);

        // Cable-tie anchors around the board groups.
        for (p = [[60, 92], [120, 92], [174, 92],
                  [94, 128], [94, 166], [94, 204]])
            through_capsule(p, 12, 4, false);
    }
}

electronics_tray();
