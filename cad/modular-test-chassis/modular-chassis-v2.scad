// Modular open-front chassis v2 — unified 50->50 interface integration model.
// Units: mm. Ground frame: +X forward, +Y left, +Z up.
//
// This file is an integration prototype, not a print release. It verifies that
// every structural joint uses the same 50 x 30 mm beam interface while the
// Gamma landing body widens to 70 mm only after that interface.

$fn = 48;

use <../../engineering/chassis-strength-bench/coupon/drive-motor-inline-module.scad>

part = "assembly"; // [assembly,frame,motor_left,motor_right,motor_section,front_motor_gamma_left,front_motor_gamma_right,rail_extension_left,rail_extension_right,straight_sleeve,corner_left,corner_right,crossbar_half,ramp_back_half,ramp_back_sleeve,front_brace_wheel_interference,front_brace_ramp_interference,middle_crossbar_half,gamma_left,gamma_right,ramp_cradle_left,ramp_cradle_right,electronics_tray_left,electronics_tray_right]

// Purchased metal bracket: preview reference only, excluded from all STL.
show_metal_motor_mounts = true;
show_gamma_mounts = true;
brace_layout = "ramp_back"; // [ramp_back,middle]
front_brace_w = 50;
front_brace_h = 20;
front_brace_x = 294.4; // width grows rearward; front face stays at 319.4
front_brace_socket_y = 160;
front_brace_sleeve_w = 43;
front_brace_sleeve_h = 13;
front_brace_sleeve_len = 60;
metal_mount_foot = 40;
metal_mount_thickness = 2;
metal_mount_height = 47; // approximate measurement; upright envelope only

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

// Open U-frame: rear module, extension, unified front motor/Gamma landing.
rear_joint_x = -165;
motor_joint_x = 35;
gamma_joint_x = 235;
rear_motor_x = -65;
front_motor_x = 300;
front_module_x0 = front_motor_x - 100;
middle_crossbar_x = (motor_joint_x + front_module_x0)/2;
middle_socket_y = 135;
middle_half_length = middle_socket_y;
middle_rib_t = 6;
assert(middle_half_length >= 2*sleeve_insert+10,
       "Middle half must leave room between its two sleeves");

// The existing Gamma feet occupy X=330..455 and Y=175..245 on the left.
gamma_socket_x0 = gamma_joint_x;
gamma_socket_x1 = 300;
gamma_flare_x1 = 330;
gamma_body_x1 = 455;
gamma_body_w = 70;
gamma_body_center_y = 210;
drive_center_y = 220; // 50 mm rail is flush with the outboard Y=245 face.

assert(front_motor_x + metal_mount_foot/2 <= gamma_flare_x1 - 10,
       "Keep at least 10 mm between the metal foot and Gamma landing");
assert(gamma_body_x1 - front_module_x0 <= 256,
       "Unified front part exceeds the 256 mm nominal build plate");

rear_crossbar_length = 2 * drive_center_y;
crossbar_half_length = rear_crossbar_length / 2;

wheel_d = 170;
wheel_w = 80;
wheel_y = 350;
wheel_z = 85;

electronics_x = rear_joint_x; // vertical carrier on the rear crossbar
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
    translate([x,y,pocket_z])
        cylinder(d=nut_seat_w,h=nut_channel_h,$fn=6);
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
            translate([xx, yy, sleeve_h - nut_top_cover - nut_channel_h - 2])
                cylinder(d=lock_clear_d,
                         h=nut_channel_h + nut_top_cover + 2.1);
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
    // Generate from the authoritative source, never from an exported snapshot.
    translate([xc, side*drive_center_y, rail_z0])
        drive_motor_inline(side);
}

module metal_motor_mount_positive() {
    // Foot holes match the physically checked pattern. Local +Y faces the
    // wheel. The upright motor-hole pattern remains omitted until measured.
    difference() {
        translate([-metal_mount_foot/2, -metal_mount_foot/2, 0])
            cube([metal_mount_foot, metal_mount_foot, metal_mount_thickness]);
        for (xx = [-15, 15], yy = [metal_mount_foot/2-34,
                                   metal_mount_foot/2-9])
            translate([xx, yy, -0.1])
                cylinder(d=4, h=metal_mount_thickness+0.2);
    }
    translate([-metal_mount_foot/2,
               metal_mount_foot/2-metal_mount_thickness,
               metal_mount_thickness])
        cube([metal_mount_foot, metal_mount_thickness,
              metal_mount_height-metal_mount_thickness]);
}

module metal_motor_mount_reference(side=1) {
    // Background geometry appears in F5, but is excluded from F6 and STL.
    if (show_metal_motor_mounts)
        %color([0.72, 0.74, 0.78, 0.85])
            if (side > 0) metal_motor_mount_positive();
            else mirror([0, 1, 0]) metal_motor_mount_positive();
}

module motor_part_preview(side=1) {
    drive_motor_inline(side);
    translate([0, 0, rail_h]) metal_motor_mount_reference(side);
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
            front_motor_gamma(side);
            rail_extension(side);
        }
        rear_crossbar();
        if (brace_layout == "middle") for (side = [-1,1])
            translate([middle_crossbar_x,side*middle_half_length/2,
                       rail_z0+rail_h/2]) middle_crossbar_half();
        if (brace_layout == "ramp_back") for (side = [-1,1])
            translate([front_brace_x,side*front_brace_socket_y/2,
                       rail_z0+front_brace_h/2]) ramp_back_half();
    }

    color(sleeve_color) {
        // Rear module -> extension -> unified front motor/Gamma part.
        for (side = [-1, 1], xx = [motor_joint_x, front_module_x0])
            translate([xx, side*drive_center_y,
                       rail_z0 + (rail_h-sleeve_h)/2])
                straight_sleeve();
        // Centre crossbar splice (straight sleeve rotated to Y).
        translate([rear_joint_x, 0,
                   rail_z0 + (rail_h-sleeve_h)/2])
            rotate([0, 0, 90]) straight_sleeve();
        if (brace_layout == "middle") for (yy = [-middle_socket_y,0,middle_socket_y])
            translate([middle_crossbar_x,yy,rail_z0+(rail_h-sleeve_h)/2])
                rotate([0,0,90]) straight_sleeve();
        if (brace_layout == "ramp_back") for (yy = [-front_brace_socket_y,0,front_brace_socket_y])
            translate([front_brace_x,yy,
                       rail_z0+(front_brace_h-front_brace_sleeve_h)/2])
                ramp_back_sleeve();
        // Rear corners.
        translate([rear_joint_x, drive_center_y,
                   rail_z0 + (rail_h-sleeve_h)/2])
            corner_sleeve(1);
        translate([rear_joint_x, -drive_center_y,
                   rail_z0 + (rail_h-sleeve_h)/2])
            corner_sleeve(-1);
    }
}

module rail_extension(side=1) {
    if (brace_layout == "middle") {
        if (side > 0) rail_extension_positive();
        else mirror([0,1,0]) rail_extension_positive();
    } else difference() {
        translate([middle_crossbar_x,side*drive_center_y,rail_z0+rail_h/2])
            hollow_box_x(front_module_x0-motor_joint_x);
        for (xx=[motor_joint_x+30,front_module_x0-30],yy=[-lock_y,lock_y])
            translate([xx,side*drive_center_y+yy,rail_top_z-rail_wall-0.1])
                cylinder(d=lock_clear_d,h=rail_wall+0.2);
    }
}

module ramp_back_half() {
    difference() {
        hollow_box_y(front_brace_socket_y,front_brace_w,front_brace_h);
        for (yy=[-front_brace_socket_y/2+15,front_brace_socket_y/2-15],xx=[-lock_y,lock_y])
            translate([xx,yy,front_brace_h/2-rail_wall-0.1])
                cylinder(d=lock_clear_d,h=rail_wall+0.2);
    }
}

module ramp_back_sleeve() {
    w=front_brace_sleeve_w;
    h=front_brace_sleeve_h;
    length=front_brace_sleeve_len;
    pocket_z=h-nut_top_cover-nut_channel_h;
    difference() {
        union() {
            difference() {
                translate([-w/2,-length/2,0]) cube([w,length,h]);
                translate([-w/2+2.5,-length/2-1,2.5])
                    cube([w-5,length+2,h-5]);
            }
            for (yy=[-15,15],xx=[-lock_y,lock_y])
                translate([xx,yy,2.5]) cylinder(d=11,h=h-2.5);
        }
        for (yy=[-15,15],xx=[-lock_y,lock_y]) {
            translate([xx,yy,pocket_z-2]) cylinder(d=lock_clear_d,h=h-pocket_z+3);
            // Same three-sided, tapered guide as the validated motor pocket,
            // rotated 90 degrees so the nut enters from the nearest X side.
            translate([xx,yy,pocket_z]) rotate([0,0,90])
                cylinder(d=nut_seat_w,h=nut_channel_h,$fn=6);
            translate([0,0,pocket_z]) linear_extrude(height=nut_channel_h)
                polygon(xx>0 ? [
                    [xx-0.05,yy-nut_seat_w/2],[xx-0.05,yy+nut_seat_w/2],
                    [w/2+1,yy+nut_entry_w/2],[w/2+1,yy-nut_entry_w/2]
                ] : [
                    [-w/2-1,yy-nut_entry_w/2],[-w/2-1,yy+nut_entry_w/2],
                    [xx+0.05,yy+nut_seat_w/2],[xx+0.05,yy-nut_seat_w/2]
                ]);
        }
    }
}

module ramp_back_t_positive() {
    socket_end_y=drive_center_y-rail_w/2+rail_wall;
    difference() {
        intersection() {
            union() {
                translate([front_brace_x,(front_brace_socket_y+socket_end_y)/2,
                           rail_z0+front_brace_h/2])
                    hollow_box_y(socket_end_y-front_brace_socket_y,
                                 front_brace_w,front_brace_h);
                for (direction=[-1,1]) hull() {
                    translate([front_brace_x+direction*32,197,
                               rail_z0+front_brace_h/2])
                        cube([4,4,front_brace_h],center=true);
                    translate([front_brace_x+direction*(front_brace_w/2-2),165,
                               rail_z0+front_brace_h/2])
                        cube([4,4,front_brace_h],center=true);
                }
            }
            // Do not grow into the original motor cavity or nut access.
            translate([250,100,rail_z0-1]) cube([120,socket_end_y-100,front_brace_h+2]);
        }
        // Re-cut after adding ribs: their overlap must never narrow the socket.
        translate([front_brace_x, (front_brace_socket_y-1+socket_end_y+1)/2,
                   rail_z0+front_brace_h/2])
            cube([front_brace_w-2*rail_wall,
                  socket_end_y-front_brace_socket_y+2,
                  front_brace_h-2*rail_wall],center=true);
        for (xx=[-lock_y,lock_y])
        translate([front_brace_x+xx,front_brace_socket_y+15,
                   rail_z0+front_brace_h-rail_wall-0.1])
            cylinder(d=lock_clear_d,h=rail_wall+0.2);
    }
}

module ramp_back_structure() {
    for (side=[-1,1]) {
        translate([front_brace_x,side*front_brace_socket_y/2,
                   rail_z0+front_brace_h/2]) ramp_back_half();
        if (side>0) ramp_back_t_positive();
        else mirror([0,1,0]) ramp_back_t_positive();
    }
}

module intake_wheel_envelopes() {
    for (side=[-1,1]) translate([370,side*90,70]) rotate([0,35,0])
        cylinder(d=124,h=73,center=true);
}

module middle_crossbar_half() {
    difference() {
        hollow_box_y(middle_half_length);
        for (yy = [-middle_half_length/2+lock_x,middle_half_length/2-lock_x],
             xx = [-lock_y,lock_y])
            translate([xx,yy,rail_h/2-rail_wall-0.1])
                cylinder(d=lock_clear_d,h=rail_wall+0.2);
    }
}

module rail_extension_positive() {
    // Integral inward T. Main cavity is re-cut so ribs cannot obstruct sleeves.
    socket_end_y = drive_center_y-rail_w/2+rail_wall;
    socket_length = socket_end_y-middle_socket_y;
    difference() {
        union() {
            translate([middle_crossbar_x,drive_center_y,rail_z0+rail_h/2])
                hollow_box_x(front_module_x0-motor_joint_x);
            translate([middle_crossbar_x,(middle_socket_y+socket_end_y)/2,
                       rail_z0+rail_h/2])
                hollow_box_y(socket_length);
            // Two diagonal strips in plan, not a filled triangular plate.
            for (direction = [-1,1]) hull() {
                translate([middle_crossbar_x+direction*65,
                           drive_center_y-rail_w/2+2,rail_z0+rail_h/2])
                    cube([middle_rib_t,middle_rib_t,rail_h],center=true);
                translate([middle_crossbar_x+direction*(rail_w/2-2),
                           middle_socket_y+5,rail_z0+rail_h/2])
                    cube([middle_rib_t,middle_rib_t,rail_h],center=true);
            }
        }
        translate([middle_crossbar_x,drive_center_y,rail_z0+rail_h/2])
            cube([front_module_x0-motor_joint_x+2,rail_w-2*rail_wall,
                  rail_h-2*rail_wall],center=true);
        for (xx = [motor_joint_x+30, front_module_x0-30], yy = [-lock_y,lock_y])
            translate([xx,drive_center_y+yy,rail_top_z-rail_wall-0.1])
                cylinder(d=lock_clear_d,h=rail_wall+0.2);
        for (xx = [-lock_y,lock_y])
            translate([middle_crossbar_x+xx,middle_socket_y+lock_x,
                       rail_top_z-rail_wall-0.1])
                cylinder(d=lock_clear_d,h=rail_wall+0.2);
    }
}

module front_motor_gamma(side=1) {
    // Shared housing avoids overlapping separate sockets below the Gamma.
    difference() {
        union() {
            motor_module(side,front_motor_x);
            if (brace_layout == "ramp_back") {
                if (side > 0) ramp_back_t_positive();
                else mirror([0,1,0]) ramp_back_t_positive();
            }
            intersection() {
                gamma_body(side);
                translate([330,-260,rail_z0-1]) cube([126,520,rail_h+2]);
            }
        }
        // Re-cut through the union: the motor rail also crosses this landing.
        for (xx = [345,440], yy = [188,232])
            translate([xx,side*yy,rail_z0-1]) cylinder(d=5.5,h=rail_h+2);
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

module gamma_mount_reference(side=1) {
    if (show_gamma_mounts) color(intake_color) {
        // Undo the build-plate transforms used by print_oriented().
        if (side > 0) translate([330,69,52])
            import("../standalone-intake-fixed-motor/stl/gamma_bracket_left_reinforced_v2.stl");
        else translate([515,-69,52]) rotate([0,0,180])
            import("../standalone-intake-fixed-motor/stl/gamma_bracket_right_reinforced_v2.stl");
    }
}

module front_motor_gamma_preview(side=1) {
    translate([-front_module_x0,-side*drive_center_y,-rail_z0]) {
        front_motor_gamma(side);
        translate([front_motor_x,side*drive_center_y,rail_top_z])
            metal_motor_mount_reference(side);
        // Existing printed Gamma is for orientation checking, not re-export.
        %gamma_mount_reference(side);
    }
}

module intake_context() {
    gamma_mount_reference(1);
    gamma_mount_reference(-1);
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
    // Vertical Y-Z carrier: centre of chassis remains open for future cargo.
    // Dimensions are a layout placeholder; board/rail holes are not released.
    union() {
        translate([electronics_x, electronics_half_y/2,
                   rail_top_z + electronics_size_x/2])
            cube([electronics_plate_t, electronics_half_y,
                  electronics_size_x], center=true);
        translate([electronics_x,(drive_center_y+rail_w/2)/2,
                   rail_top_z+electronics_plate_t/2])
            cube([rail_w,drive_center_y+rail_w/2,electronics_plate_t],center=true);
        for (yy = [15,electronics_half_y-15])
            hull() {
                translate([electronics_x,yy,rail_top_z+30])
                    cube([electronics_plate_t,8,4],center=true);
                translate([electronics_x+20,yy,rail_top_z+electronics_plate_t/2])
                    cube([4,8,electronics_plate_t],center=true);
            }
    }
}

module electronics_tray(side=1) {
    if (side > 0) electronics_tray_positive();
    else mirror([0, 1, 0]) electronics_tray_positive();
}

module assembly() {
    frame_structure();
    color([0.58, 0.20, 0.76, 0.94]) {
        if (brace_layout == "middle") {
            ramp_cradle(1);
            ramp_cradle(-1);
        }
    }
    color([0.12, 0.55, 0.35, 0.62]) {
        electronics_tray(1);
        electronics_tray(-1);
    }
    for (side = [-1, 1], xx = [rear_motor_x, front_motor_x])
        wheel_context(side, xx);
    for (side = [-1, 1], xx = [rear_motor_x, front_motor_x])
        translate([xx, side*drive_center_y, rail_top_z])
            metal_motor_mount_reference(side);
    intake_context();
}

if (part == "frame") frame_structure();
else if (part == "qa_front_sleeves")
    intersection() {
        union() { front_motor_gamma(1); front_motor_gamma(-1);
                  rail_extension(1); rail_extension(-1); ramp_back_structure(); }
        union() {
            for (side=[-1,1])
                translate([front_module_x0,side*drive_center_y,rail_z0+(rail_h-sleeve_h)/2])
                    straight_sleeve();
            for (yy=[-front_brace_socket_y,0,front_brace_socket_y])
                translate([front_brace_x,yy,rail_z0+(front_brace_h-front_brace_sleeve_h)/2])
                    ramp_back_sleeve();
        }
    }
else if (part == "qa_front_lock_bores")
    intersection() {
        union() { front_motor_gamma(1); front_motor_gamma(-1); ramp_back_structure(); }
        union() {
            for (side=[-1,1],yy=[-lock_y,lock_y])
                translate([front_module_x0+30,side*drive_center_y+yy,rail_top_z-rail_wall])
                    cylinder(d=3.2,h=rail_wall+1);
            for (side=[-1,1],delta=[-15,15],xx=[-lock_y,lock_y])
                translate([front_brace_x+xx,side*front_brace_socket_y+delta,
                           rail_z0+front_brace_h-rail_wall])
                    cylinder(d=3.2,h=rail_wall+1);
        }
    }
else if (part == "qa_gamma_holes")
    intersection() {
        union() { gamma_mount_reference(1); gamma_mount_reference(-1); }
        union() { for (side=[-1,1],xx=[345,440],yy=[188,232])
            translate([xx,side*yy,rail_top_z+0.1]) cylinder(d=5,h=1); }
    }
else if (part == "front_brace_wheel_interference")
    intersection() { ramp_back_structure(); intake_wheel_envelopes(); }
else if (part == "front_brace_ramp_interference")
    intersection() {
        ramp_back_structure();
        import("../standalone-intake-fixed-motor/stl/intake_handoff_ramp_wheel_first.stl");
    }
else if (part == "front_motor_gamma_left")
    front_motor_gamma_preview(1);
else if (part == "front_motor_gamma_right")
    front_motor_gamma_preview(-1);
else if (part == "rail_extension_left")
    translate([-motor_joint_x,-drive_center_y,-rail_z0]) rail_extension(1);
else if (part == "rail_extension_right")
    translate([-motor_joint_x,drive_center_y,-rail_z0]) rail_extension(-1);
else if (part == "motor_left") motor_part_preview(1);
else if (part == "motor_right") motor_part_preview(-1);
else if (part == "motor_section") {
    intersection() {
        drive_motor_inline(1);
        // Longitudinal half-section exposes the reinforced roof and nuts.
        translate([-101, -26, -1]) cube([202, 26, 32]);
    }
    translate([0, 0, rail_h]) metal_motor_mount_reference(1);
}
else if (part == "straight_sleeve") straight_sleeve();
else if (part == "corner_left") corner_sleeve(1);
else if (part == "corner_right") corner_sleeve(-1);
else if (part == "crossbar_half")
    translate([0, 0, rail_h/2]) crossbar_half();
else if (part == "middle_crossbar_half")
    translate([0,0,rail_h/2]) middle_crossbar_half();
else if (part == "ramp_back_half")
    translate([0,0,front_brace_h/2]) ramp_back_half();
else if (part == "ramp_back_sleeve") ramp_back_sleeve();
else if (part == "front_connection_coupon")
    translate([-front_brace_x,-front_brace_socket_y,-rail_z0])
        intersection() {
            ramp_back_t_positive();
            translate([front_brace_x-26,front_brace_socket_y,rail_z0]) cube([52,32,20]);
        }
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
