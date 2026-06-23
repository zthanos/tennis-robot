include <common.scad>

// Full tennis robot concept assembly.
// This is a layout/reference model, not a single print-ready part.
//
// Architecture shown here:
// Ball -> floor-level funnels -> full-width front roller -> hopper behind the
// roller -> front flywheel launcher opening.
// The OAK-D camera sits low between the roller and flywheel opening with a
// clear, slightly downward-looking view over the intake toward court lines,
// net, fences, and loose tennis balls.

// Global dimensions in mm. X=rear-to-front, Y=left-to-right, Z=ground-up.
base_len = 820;
base_w = 460;
wood_t = 12;
frame_tube = 22;

// 4WD: four identical 190 mm driven wheels, two per side, placed symmetrically
// about the chassis center (no casters, no chassis pitch).
drive_wheel_d = 190;
drive_wheel_w = 48;
wheel_outboard_gap = 18;
rear_axle_x = 150;
front_axle_x = base_len - rear_axle_x;   // symmetric about base_len/2 = 410

roller_d = 120;
roller_w = base_w + 80;
roller_x = base_len + 24;
roller_z = 72;

collector_x = 675;
collector_len = 245;
collector_w = 360;
collector_wall_h = 86;
collector_lip_z = 20;

hopper_x = 500;
hopper_w = 320;
hopper_len = 250;
hopper_h = 235;
hopper_floor_z = 95;

feed_channel_w = 92;
feed_channel_h = 58;

launcher_x = 735;
launcher_z = 345;
launcher_wheel_d = 118;
launcher_wheel_w = 40;
launcher_gap = 74;
launcher_panel_t = 9;

camera_x = 805;
camera_z = 275;
oak_body = [95, 32, 28];

low_lidar_d = 56;
low_lidar_h = 42;
lidar_x = 250;
lidar_z = 520;

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

module material_hopper() {
    color([0.05, 0.68, 0.24, 0.55]) children();
}

module material_launcher() {
    color([0.55, 0.20, 0.70]) children();
}

module material_sensor() {
    color([0.05, 0.12, 0.18]) children();
}

module material_reference() {
    color([0.72, 0.86, 1.0, 0.18]) children();
}

module material_ball() {
    color([0.74, 0.95, 0.12]) children();
}

module wheel(d, w) {
    rotate([90, 0, 0])
        cylinder(h=w, d=d, center=true);
}

module tube_x(x1, x2, y, z, d=frame_tube) {
    translate([(x1 + x2) / 2, y, z])
        cube([abs(x2 - x1), d, d], center=true);
}

module tube_y(x, y1, y2, z, d=frame_tube) {
    translate([x, (y1 + y2) / 2, z])
        cube([d, abs(y2 - y1), d], center=true);
}

module tube_z(x, y, z1, z2, d=frame_tube) {
    translate([x, y, (z1 + z2) / 2])
        cube([d, d, abs(z2 - z1)], center=true);
}

module flow_arrow(start=[0, 0, 0], end=[100, 0, 0], d=12) {
    dx = end[0] - start[0];
    dy = end[1] - start[1];
    dz = end[2] - start[2];
    len = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    yaw = atan2(dy, dx);
    pitch = -atan2(dz, sqrt(pow(dx, 2) + pow(dy, 2)));

    color([0.12, 0.55, 0.95]) {
        translate([(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, (start[2] + end[2]) / 2])
            rotate([0, pitch, yaw])
                rotate([0, 90, 0])
                    cylinder(h=len - 26, d=d, center=true);
        translate(end)
            rotate([0, pitch, yaw])
                rotate([0, 90, 0])
                    cylinder(h=30, d1=d * 2.5, d2=0, center=true);
    }
}

module label(txt, p, size=20) {
    color([0.02, 0.02, 0.025])
        translate(p)
            linear_extrude(height=1.2)
                text(txt, size=size, halign="center", valign="center");
}

module chassis_base() {
    material_wood()
        translate([0, 0, wood_t / 2])
            rounded_plate([base_len, base_w, wood_t], 14);

    material_frame() {
        // Lower perimeter rails.
        tube_x(20, base_len - 20, 18, 78);
        tube_x(20, base_len - 20, base_w - 18, 78);
        tube_y(20, 18, base_w - 18, 78);
        tube_y(base_len - 20, 18, base_w - 18, 78);

        // Upper tubular protective frame surrounding all modules.
        tube_x(-5, base_len + 70, -22, 500);
        tube_x(-5, base_len + 70, base_w + 22, 500);
        tube_y(-5, -22, base_w + 22, 500);
        tube_y(base_len + 70, -22, base_w + 22, 500);

        for (x = [10, 250, 560, base_len + 45]) {
            for (y = [-22, base_w + 22]) {
                tube_z(x, y, 78, 500);
            }
        }

        // Front launcher bracing and front intake guard.
        tube_y(launcher_x, 20, base_w - 20, launcher_z + 125);
        tube_x(launcher_x - 115, launcher_x + 55, 42, launcher_z + 80);
        tube_x(launcher_x - 115, launcher_x + 55, base_w - 42, launcher_z + 80);
        tube_y(roller_x + 4, -20, base_w + 20, roller_z + 72);
    }
}

module drive_wheels_4wd() {
    left_y = -drive_wheel_w / 2 - wheel_outboard_gap;
    right_y = base_w + drive_wheel_w / 2 + wheel_outboard_gap;

    // Motor pods (one per wheel) at all four corners.
    material_frame() {
        for (x = [rear_axle_x, front_axle_x]) {
            translate([x, -drive_wheel_w / 2, drive_wheel_d / 2])
                cube([150, 36, 95], center=true);
            translate([x, base_w + drive_wheel_w / 2, drive_wheel_d / 2])
                cube([150, 36, 95], center=true);
        }
    }

    // Four driven wheels.
    color([0.02, 0.02, 0.02]) {
        for (x = [rear_axle_x, front_axle_x]) {
            translate([x, left_y, drive_wheel_d / 2])
                wheel(drive_wheel_d, drive_wheel_w);
            translate([x, right_y, drive_wheel_d / 2])
                wheel(drive_wheel_d, drive_wheel_w);
        }
    }
}

module electronics_battery_module() {
    material_electronics() {
        translate([465, base_w / 2, wood_t + 32])
            cube([230, 130, 64], center=true);
        translate([300, base_w / 2, wood_t + 28])
            cube([165, 280, 56], center=true);
    }
}

module collector_intake() {
    y = base_w / 2;

    material_collector() {
        // Full-width ground lip and rearward collection channel immediately
        // behind the roller.
        hull() {
            translate([roller_x + 18, y, collector_lip_z])
                cube([24, base_w + 45, 10], center=true);
            translate([collector_x + collector_len / 2, y, wood_t + 28])
                cube([34, collector_w + 40, 14], center=true);
        }

        // Left/right floor-level V funnels that stay low enough to guide balls
        // into the roller before the chassis can push them away.
        for (side = [-1, 1]) {
            hull() {
                translate([roller_x + 10, y + side * (base_w / 2 + 16), 16])
                    cube([24, 34, 12], center=true);
                translate([base_len + 265, y + side * (base_w / 2 + 145), 10])
                    cube([22, 46, 8], center=true);
            }
        }

        translate([collector_x, y, wood_t + 48])
            cube([collector_len, feed_channel_w, 30], center=true);

        for (side = [-1, 1]) {
            hull() {
                translate([roller_x - 28, y + side * (base_w / 2 + 20), wood_t + 75])
                    cube([18, 10, collector_wall_h], center=true);
                translate([collector_x - collector_len / 2, y + side * feed_channel_w / 2, wood_t + 75])
                    cube([18, 10, collector_wall_h], center=true);
            }
        }

        // Visible throat section feeding the hopper.
        translate([560, y, hopper_floor_z + 14])
            rotate([0, -8, 0])
                cube([150, feed_channel_w, 36], center=true);
    }

    color([0.03, 0.03, 0.03])
        translate([roller_x, y, roller_z])
            rotate([90, 0, 0])
                cylinder(h=roller_w, d=roller_d, center=true);

    material_frame() {
        for (side = [-1, 1]) {
            translate([roller_x, y + side * (roller_w / 2 + 16), roller_z])
                cube([34, 22, 120], center=true);
        }
    }

    // IR break-beam pair immediately behind the roller throat.
    material_sensor() {
        for (side = [-1, 1]) {
            translate([roller_x - 76, y + side * 70, 72])
                cube([28, 16, 34], center=true);
            translate([roller_x - 76, y + side * 70, 72])
                rotate([90, 0, 0])
                    cylinder(h=12, d=10, center=true);
        }
    }
    color([0.95, 0.08, 0.04, 0.45])
        translate([roller_x - 76, y, 72])
            rotate([90, 0, 0])
                cylinder(h=140, d=4, center=true);
}

module center_hopper() {
    x = hopper_x;
    y = base_w / 2;

    material_hopper() {
        // Open-top storage hopper with sloped rear floor toward the flywheel feed.
        translate([x, y, hopper_floor_z])
            rotate([0, -7, 0])
                cube([hopper_len, hopper_w, 10], center=true);
        translate([x, y - hopper_w / 2, hopper_floor_z + hopper_h / 2])
            cube([hopper_len, 10, hopper_h], center=true);
        translate([x, y + hopper_w / 2, hopper_floor_z + hopper_h / 2])
            cube([hopper_len, 10, hopper_h], center=true);
        translate([x + hopper_len / 2, y, hopper_floor_z + hopper_h / 2])
            cube([10, hopper_w, hopper_h], center=true);
        translate([x - hopper_len / 2, y - hopper_w / 2 + 48, hopper_floor_z + 72])
            cube([10, 96, 144], center=true);
        translate([x - hopper_len / 2, y + hopper_w / 2 - 48, hopper_floor_z + 72])
            cube([10, 96, 144], center=true);
    }

    // Balls visible in storage, reinforcing that this is a hopper rather than a
    // generic electronics bay.
    material_ball() {
        for (p = [[470, 190, 185], [430, 240, 176], [390, 205, 168], [455, 285, 190]]) {
            translate(p) sphere(d=67);
        }
    }
}

module feed_to_launcher() {
    y = base_w / 2;
    start_x = hopper_x + hopper_len / 2 + 8;
    end_x = launcher_x - 90;
    start_z = hopper_floor_z + 56;
    end_z = launcher_z - 70;
    dx = end_x - start_x;
    dz = end_z - start_z;
    channel_len = sqrt(pow(dx, 2) + pow(dz, 2));
    channel_angle = atan2(dz, dx);

    color([0.90, 0.84, 0.55])
        translate([(start_x + end_x) / 2, y, (start_z + end_z) / 2])
            rotate([0, channel_angle, 0])
                cube([channel_len, feed_channel_w, feed_channel_h], center=true);

    color([0.08, 0.08, 0.09])
        translate([start_x - 18, y, start_z + 8])
            rotate([90, 0, 0])
                cylinder(h=feed_channel_w + 16, d=48, center=true);
}

module launcher_module() {
    x = launcher_x;
    y = base_w / 2;
    z = launcher_z;

    material_frame() {
        // Side walls wide enough to contain Y-separated wheels (centers +/-37, radius 59).
        translate([x, y - 105, z])
            cube([launcher_panel_t, 22, 285], center=true);
        translate([x, y + 105, z])
            cube([launcher_panel_t, 22, 285], center=true);
        translate([x, y, z + 142])
            cube([launcher_panel_t, 232, 20], center=true);
        translate([x, y, z - 142])
            cube([launcher_panel_t, 232, 20], center=true);
        // Back plate closes the rear of the front-facing housing.
        translate([x - 52, y, z])
            cube([launcher_panel_t, 228, 285], center=true);
    }

    material_launcher() {
        translate([x, y - launcher_gap / 2, z])
            rotate([0, 90, 0])
                cylinder(h=launcher_wheel_w, d=launcher_wheel_d, center=true);
        translate([x, y + launcher_gap / 2, z])
            rotate([0, 90, 0])
                cylinder(h=launcher_wheel_w, d=launcher_wheel_d, center=true);
    }

    color([0.88, 0.88, 0.82])
        translate([x + 88, y, z + 8])
            rotate([0, -8, 0])
                cube([145, 88, 44], center=true);

    color([0.90, 0.84, 0.55])
        translate([x - 95, y, z - 70])
            rotate([0, -8, 0])
                cube([130, feed_channel_w, 42], center=true);
}

module sensor_module() {
    y = base_w / 2;

    // Raised 360-degree LiDAR on a mast, clear of hopper, camera, frame, and flywheels.
    material_frame() {
        translate([lidar_x, y, 320])
            cube([20, 20, 300], center=true);
        translate([lidar_x, y, lidar_z - low_lidar_h / 2 - 8])
            rounded_box([100, 100, 12], 5);
    }

    material_sensor() {
        translate([lidar_x, y, lidar_z])
            cylinder(h=low_lidar_h, d=low_lidar_d, center=true);
        translate([lidar_x, y, lidar_z])
            cylinder(h=low_lidar_h + 8, d=low_lidar_d + 18, center=true);
    }

    material_reference()
        translate([lidar_x, y, lidar_z])
            rotate([90, 0, 0])
                cylinder(h=base_w + 210, d=4, center=true);

    // OAK-D mounted low between roller and flywheel opening, tilted slightly
    // downward while looking forward over the intake.
    material_frame() {
        translate([camera_x, y, camera_z - 34])
            cube([26, 78, 68], center=true);
        translate([camera_x + 14, y, camera_z - 3])
            rotate([0, -12, 0])
                cube([120, 42, 10], center=true);
    }

    material_sensor()
        translate([camera_x + 26, y, camera_z])
            rotate([0, -12, 0])
                cube(oak_body, center=true);

    color([0.02, 0.02, 0.025]) {
        for (yy = [-22, 0, 22]) {
            translate([camera_x + 73, y + yy, camera_z - 7])
                rotate([0, 78, 0])
                    cylinder(h=7, d=(yy == 0 ? 18 : 14), center=true);
        }
    }

    // Transparent viewing wedge showing the unobstructed forward field above the
    // roller and channel with only a slight downward pitch.
    color([0.35, 0.70, 1.0, 0.15])
        hull() {
            translate([camera_x + 75, y, camera_z - 8])
                cube([4, 80, 10], center=true);
            translate([base_len + 460, y, camera_z - 130])
                cube([4, base_w + 520, 90], center=true);
        }
}

module ball_flow_reference() {
    y = base_w / 2;

    material_ball()
        translate([base_len + 205, y, 34])
            sphere(d=67);

    flow_arrow([base_len + 165, y, 42], [roller_x + 34, y, roller_z], 10);
    flow_arrow([roller_x - 45, y, roller_z], [collector_x + 55, y, wood_t + 62], 10);
    flow_arrow([collector_x - 90, y, wood_t + 68], [hopper_x + hopper_len / 2 - 25, y, hopper_floor_z + 72], 10);
    flow_arrow([hopper_x + hopper_len / 2 - 30, y, hopper_floor_z + 78], [launcher_x - 92, y, launcher_z - 70], 10);
    flow_arrow([launcher_x, y, launcher_z], [launcher_x + 250, y, launcher_z + 120], 10);

    label("Ball", [base_len + 205, y - 88, 9], 22);
    label("Roller", [roller_x, y - 120, 9], 19);
    label("Collector", [collector_x, y - 116, 9], 19);
    label("Hopper", [hopper_x, y - 122, 9], 19);
    label("Flywheel", [launcher_x, y - 124, 9], 19);
}

module full_robot_concept() {
    drive_wheels_4wd();
    chassis_base();
    electronics_battery_module();
    collector_intake();
    center_hopper();
    feed_to_launcher();
    launcher_module();
    sensor_module();
    ball_flow_reference();
}

full_robot_concept();
