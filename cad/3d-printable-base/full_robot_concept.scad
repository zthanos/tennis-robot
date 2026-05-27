include <common.scad>

// Full tennis robot concept assembly.
// This is a layout/reference model, not a single print-ready part.
// It combines a wooden base, bolt-on frame, collector intake, receiving bin,
// launcher flywheels, electronics/battery volume, rear drive wheels,
// passive front casters, and a folding transport handle.

// Global dimensions in mm.
base_len = 760;
base_w = 430;
wood_t = 12;
frame_tube = 22;

rear_wheel_d = 190;
rear_wheel_w = 48;
front_caster_d = 85;
front_caster_w = 32;

collector_len = 330;
collector_mouth_w = 310;
collector_throat_w = 86;
collector_wall_h = 78;
collector_lip_h = 14;
collector_roller_d = 80;
collector_roller_w = 280;

bin_len = 230;
bin_w = 300;
bin_h = 360;
bin_tilt = 7;
feed_channel_w = 92;
feed_channel_h = 78;

launcher_wheel_d = 110;
launcher_wheel_w = 38;
launcher_gap = 72;
launcher_panel_t = 8;
cover_panel_t = 3;
cover_z = 510;
cover_h = 310;

module material_wood() {
    color([0.58, 0.40, 0.24]) children();
}

module material_frame() {
    color([0.06, 0.06, 0.065]) children();
}

module material_electronics() {
    color([0.45, 0.45, 0.42]) children();
}

module material_collector() {
    color([0.95, 0.68, 0.18]) children();
}

module material_bin() {
    color([0.05, 0.68, 0.24, 0.65]) children();
}

module material_launcher() {
    color([0.55, 0.20, 0.70]) children();
}

module material_handle() {
    color([0.95, 0.75, 0.12]) children();
}

module material_cover() {
    color([0.72, 0.86, 1.0, 0.28]) children();
}

module wheel(d, w) {
    rotate([90, 0, 0])
        cylinder(h=w, d=d, center=true);
}

module chassis_base() {
    material_wood()
        translate([0, 0, wood_t / 2])
            rounded_plate([base_len, base_w, wood_t], 14);

    // Large upper frame where modules bolt on.
    material_frame() {
        translate([base_len / 2, frame_tube / 2, 170])
            cube([base_len, frame_tube, frame_tube], center=true);
        translate([base_len / 2, base_w - frame_tube / 2, 170])
            cube([base_len, frame_tube, frame_tube], center=true);
        translate([frame_tube / 2, base_w / 2, 170])
            cube([frame_tube, base_w, frame_tube], center=true);
        translate([base_len - frame_tube / 2, base_w / 2, 170])
            cube([frame_tube, base_w, frame_tube], center=true);

        // Vertical posts from wood base to upper frame.
        for (x = [60, base_len - 60]) {
            for (y = [45, base_w - 45]) {
                translate([x, y, 90])
                    cube([frame_tube, frame_tube, 160], center=true);
            }
        }
    }
}

module drive_and_casters() {
    material_frame() {
        translate([170, -rear_wheel_w / 2, rear_wheel_d / 2])
            cube([150, 36, 95], center=true);
        translate([170, base_w + rear_wheel_w / 2, rear_wheel_d / 2])
            cube([150, 36, 95], center=true);
    }

    color([0.02, 0.02, 0.02]) {
        translate([170, -rear_wheel_w / 2 - 18, rear_wheel_d / 2])
            wheel(rear_wheel_d, rear_wheel_w);
        translate([170, base_w + rear_wheel_w / 2 + 18, rear_wheel_d / 2])
            wheel(rear_wheel_d, rear_wheel_w);
        translate([base_len - 110, 85, front_caster_d / 2])
            wheel(front_caster_d, front_caster_w);
        translate([base_len - 110, base_w - 85, front_caster_d / 2])
            wheel(front_caster_d, front_caster_w);
    }

    material_frame() {
        for (y = [85, base_w - 85]) {
            translate([base_len - 110, y, front_caster_d + 18])
                rounded_box([88, 64, 18], 6);
            translate([base_len - 110, y, front_caster_d / 2 + 24])
                cube([24, 18, 58], center=true);
        }
    }
}

module electronics_battery_module() {
    material_electronics() {
        translate([250, base_w / 2, wood_t + 35])
            cube([265, 310, 70], center=true);
        translate([190, base_w / 2, wood_t + 92])
            cube([140, 180, 45], center=true);
    }
}

module receiving_bin() {
    // Flow bin: open intake side, sloped floor, narrow feed channel to launcher.
    x = 355;
    y = base_w / 2;
    z = wood_t + 92;

    material_bin() {
        // Sloped floor sends balls toward the launcher feed channel.
        translate([x, y, z])
            rotate([0, -bin_tilt, 0])
                cube([bin_len, bin_w, 8], center=true);

        // Side walls.
        translate([x, y - bin_w / 2, z + bin_h / 2])
            cube([bin_len, 8, bin_h], center=true);
        translate([x, y + bin_w / 2, z + bin_h / 2])
            cube([bin_len, 8, bin_h], center=true);

        // Back wall, opposite the intake opening.
        translate([x - bin_len / 2, y, z + bin_h / 2])
            cube([8, bin_w, bin_h], center=true);

        // Low front lips leave an open intake window for collected balls.
        translate([x + bin_len / 2, y - bin_w / 2 + 42, z + 60])
            cube([8, 84, 120], center=true);
        translate([x + bin_len / 2, y + bin_w / 2 - 42, z + 60])
            cube([8, 84, 120], center=true);
    }

    // Yellow transfer chute from collector intake roller into the bin opening.
    material_collector()
        translate([base_len - 250, y, z + 34])
            rotate([0, -8, 0])
                cube([180, feed_channel_w, 26], center=true);

    // Narrow feed channel from lower bin outlet into launcher throat.
    color([0.90, 0.84, 0.55])
        translate([480, y, z + 38])
            cube([190, feed_channel_w, feed_channel_h], center=true);

    // Simple gate/metering wheel placeholder at bin exit.
    color([0.08, 0.08, 0.09])
        translate([430, y, z + 45])
            rotate([90, 0, 0])
                cylinder(h=feed_channel_w + 14, d=46, center=true);
}

module collector_intake() {
    x0 = base_len - collector_len + 15;
    y0 = base_w / 2;

    material_collector() {
        // Sloped receiving panel/scoop: low front lip, higher rear edge.
        translate([x0 + collector_len / 2, y0, wood_t + 18])
            rotate([0, 10, 0])
                cube([collector_len, collector_throat_w + 40, collector_lip_h], center=true);

        // Funnel side guides.
        for (side = [-1, 1]) {
            angle = atan((collector_mouth_w - collector_throat_w) / 2 / collector_len);
            translate([x0 + collector_len / 2, y0 + side * collector_throat_w / 2, wood_t + 58])
                rotate([0, 0, side * angle])
                    cube([collector_len, 10, collector_wall_h], center=true);
        }

        // Front side flares, visually matching the sketch's sloped intake panels.
        translate([base_len - 60, y0 - collector_mouth_w / 2, wood_t + 38])
            rotate([0, 0, -25])
                cube([140, 10, 78], center=true);
        translate([base_len - 60, y0 + collector_mouth_w / 2, wood_t + 38])
            rotate([0, 0, 25])
                cube([140, 10, 78], center=true);
    }

    // Full-width compliant intake roller: wider contact patch than the earlier centered wheel.
    color([0.03, 0.03, 0.03])
        translate([x0 + 170, y0, wood_t + 94])
            rotate([90, 0, 0])
                cylinder(h=collector_roller_w, d=collector_roller_d, center=true);
}

module launcher_module() {
    x = 570;
    y = base_w / 2;
    z = wood_t + 260;

    material_frame() {
        translate([x, y - 72, z])
            cube([launcher_panel_t, 18, 260], center=true);
        translate([x, y + 72, z])
            cube([launcher_panel_t, 18, 260], center=true);
        translate([x, y, z + 130])
            cube([launcher_panel_t, 170, 18], center=true);
        translate([x, y, z - 130])
            cube([launcher_panel_t, 170, 18], center=true);
    }

    material_launcher() {
        translate([x - 20, y, z + launcher_gap / 2])
            rotate([90, 0, 0])
                cylinder(h=launcher_wheel_w, d=launcher_wheel_d, center=true);
        translate([x - 20, y, z - launcher_gap / 2])
            rotate([90, 0, 0])
                cylinder(h=launcher_wheel_w, d=launcher_wheel_d, center=true);
    }

    // Guarded launch chute.
    color([0.88, 0.88, 0.82])
        translate([x + 58, y, z])
            rotate([0, 18, 0])
                cube([135, 85, 42], center=true);

    // Feed throat aligned with the channel coming from the bin.
    color([0.90, 0.84, 0.55])
        translate([x - 78, y, z - 92])
            rotate([0, -6, 0])
                cube([125, feed_channel_w, 42], center=true);
}

module transport_handle() {
    material_handle() {
        translate([45, 70, 360])
            cube([20, 14, 620], center=true);
        translate([45, base_w - 70, 360])
            cube([20, 14, 620], center=true);
        translate([45, base_w / 2, 675])
            cube([180, 18, 18], center=true);
    }
}

module cover_mounting_frame() {
    // Rails and standoffs showing where removable cover panels bolt on.
    material_frame() {
        // Lower ledges on the wooden base.
        translate([base_len / 2, 24, 205])
            cube([base_len - 80, 14, 18], center=true);
        translate([base_len / 2, base_w - 24, 205])
            cube([base_len - 80, 14, 18], center=true);

        // Upper ledges for side/top cover panels.
        translate([base_len / 2, 24, cover_z])
            cube([base_len - 80, 14, 18], center=true);
        translate([base_len / 2, base_w - 24, cover_z])
            cube([base_len - 80, 14, 18], center=true);
        translate([base_len / 2, base_w / 2, cover_z + 18])
            cube([base_len - 120, 12, 18], center=true);

        // Extra vertical posts around bin and launcher, where panels need support.
        for (x = [260, 455, 625]) {
            for (y = [24, base_w - 24]) {
                translate([x, y, (205 + cover_z) / 2])
                    cube([16, 16, cover_z - 205], center=true);
            }
        }

        // Small screw bosses / visible fastening points.
        for (x = [120, 260, 400, 540, 660]) {
            for (y = [24, base_w - 24]) {
                translate([x, y, 218])
                    cylinder(h=10, d=18, center=true);
                translate([x, y, cover_z - 10])
                    cylinder(h=10, d=18, center=true);
            }
        }
    }

    // Transparent removable panels. They show how the cover closes without hiding internals.
    material_cover() {
        translate([base_len / 2, 12, 360])
            cube([base_len - 150, cover_panel_t, cover_h], center=true);
        translate([base_len / 2, base_w - 12, 360])
            cube([base_len - 150, cover_panel_t, cover_h], center=true);
        translate([420, base_w / 2, cover_z + 30])
            cube([390, base_w - 80, cover_panel_t], center=true);

        // Short rear cover over electronics; front intake and launch chute stay open.
        translate([155, base_w / 2, 360])
            cube([190, base_w - 80, cover_panel_t], center=true);
    }
}

module full_robot_concept() {
    chassis_base();
    drive_and_casters();
    electronics_battery_module();
    receiving_bin();
    collector_intake();
    launcher_module();
    cover_mounting_frame();
    transport_handle();
}

full_robot_concept();
