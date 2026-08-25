// Compact packaging study — preserve the Option A chassis and wheelbase.
//
// IMPORTANT: Option A is imported read-only.  This study moves the complete
// intake -> bridge -> basket -> feeder -> launcher chain as one group; it does
// not edit or silently re-dimension the validated intake source.
//
// All dimensions below are design hypotheses for clearance review, not
// manufacturing dimensions.  Units: mm, ground frame, robot +X forward.

use <../collector-intake-v1/option-a/option-a.scad>
include <../collector-intake-v1/option-a/bridge-params.scad>
use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/hood.scad>
use <launcher-envelope.scad>
use <compact-basket-support.scad>
use <compact-parked-reliefs.scad>
include <../motion-electronics-tray/params.scad>
use <../motion-electronics-tray/tray.scad>

$fn = 56;

pose = "both"; // "collect", "launch" or "both"
show_keepouts = true;
show_basket_keepout = show_keepouts;
show_motor_keepouts = show_keepouts;
show_shell_envelope = true;
show_labels = true;
show_basket_system = true;
show_launcher = true;
show_rear_electronics = true;
show_drive = true;
show_intake_ball_path = false;

// ---- Fixed chassis / drivetrain datums ----
chassis_min_x = -460;
chassis_max_x = 460;
chassis_half_y = 290;
chassis_top_z = 52;
wheel_x = 330;
wheel_y = 350;
wheel_d = 170;
wheel_width = 80;
drive_motor_d = 60;
drive_motor_l = 100;
drive_motor_z = 85;
drive_motor_y = 240;

// ---- One rigid functional group ----
functional_shift_x = -100;
lift_travel = 100;
launch_tilt_deg = 12;
launch_pivot = [470, 0, 40];
launcher_origin = [560, 0, 0];

// Corrected intake handoff.  The Option A ramp lip at local X=520 looks
// recessed behind the wheel's total projected envelope, but that forward-most
// tire point is outboard of a centred ball.  Finite-cylinder/sphere clearance
// gives first centred-ball wheel contact at X~=481.2, while the old 1.5 mm lip
// contacts the same ball at X~=529.8.  This study-only lip moves behind the
// pinch; simulation must validate the resulting short/steep handoff.
ball_d = 66;
ball_center_z = ball_d / 2;
wheel_first_ball_center_x = 481.2;
compact_ramp_front_x = 460;
compact_ramp_rear_x = 420;
compact_ramp_front_z = 1.5;
compact_ramp_rear_z = 35;
compact_ramp_width = 180;
compact_ramp_wall_h = 18;
compact_ramp_first_ball_center_x = compact_ramp_front_x
    + sqrt(pow(ball_d/2, 2)
           - pow(ball_center_z-compact_ramp_front_z, 2));
wheel_before_ramp_margin = wheel_first_ball_center_x
                         - compact_ramp_first_ball_center_x;

// Basket geometry is repeated here only as an external clearance contract.
basket_rear_x = 20;
basket_front_x = 420;
basket_service_front_x = 470; // receiving chute + hood, not only the main bin
basket_outer_half_y = 146;
basket_keepout_margin = 14;
basket_service_half_y = basket_outer_half_y + basket_keepout_margin;
basket_service_top_z = 600;

// The original bridge occupies x 380..600 and has its underside at z=150.
// Its compact derivative receives two strictly subtractive service features:
// an open rear notch for basket removal and motor arches in the wooden legs.
bridge_rear_x = 380;
bridge_front_x = 600;
bridge_half_y = oa_bridge_width / 2;
bridge_t = 18;
bridge_under_z = 150;
bridge_upright_y = 205;
bridge_upright_t = 18;
bridge_notch_front_x = basket_service_front_x + basket_keepout_margin;
bridge_notch_half_y = basket_service_half_y + 5;

// After the common shift, the front drive motor at x=330 appears at x=430
// in the functional group's local frame.  An open-bottom curved arch keeps a
// 10 mm radial service gap around the nominal 60 mm motor envelope.
front_motor_local_x = wheel_x - functional_shift_x;
motor_arch_d = 80;
motor_arch_center_z = drive_motor_z;
motor_arch_radial_clearance = (motor_arch_d - drive_motor_d) / 2;
motor_arch_top_ligament = bridge_under_z
                         - (motor_arch_center_z + motor_arch_d / 2);

// Rear floor packaging.  The battery moves rearward but stays horizontal;
// the motion-control tray is rotated into the YZ plane behind it.
battery_size = [166, 198, 170];
battery_center_x = -255;
battery_bottom_z = chassis_top_z;
battery_rear_x = battery_center_x - battery_size[0] / 2;
battery_front_x = battery_center_x + battery_size[0] / 2;

control_plane_x = -425;
control_base_z = 58;
control_installed_depth = 55;
control_front_x = control_plane_x + control_installed_depth;
control_rear_margin = control_plane_x - chassis_min_x;
control_battery_gap = battery_rear_x - control_front_x;
battery_basket_gap = basket_rear_x + functional_shift_x - battery_front_x;

// Roof access follows the basket group.  It remains a flat sealed hatch; no
// hand-well is allowed to intrude into the moving basket volume.
hatch_center_x = 220 + functional_shift_x;
hatch_size = [440, 190];
roof_z = 463;
lidar_scan_z = 498;

assert(functional_shift_x <= 0,
       "compact study expects a rearward functional-group shift");
assert(wheel_before_ramp_margin >= 10,
       "intake wheels must contact a centred ball at least 10 mm before the ramp lip");
assert(bridge_notch_half_y >= basket_service_half_y,
       "bridge notch must clear the complete basket service corridor");
assert(bridge_notch_front_x >= basket_service_front_x
                                + basket_keepout_margin,
       "bridge notch must clear the basket receiving chute and hood");
assert(motor_arch_radial_clearance >= 10,
       "wooden bridge arch needs at least 10 mm radial motor clearance");
assert(motor_arch_top_ligament >= 25,
       "motor arch needs at least 25 mm wood above the cutout");
assert(control_rear_margin >= 30,
       "vertical control tray needs at least 30 mm from chassis rear");
assert(control_battery_gap >= 20,
       "vertical control tray needs at least 20 mm before the battery");
assert(battery_basket_gap >= 50,
       "battery needs at least 50 mm before the shifted basket envelope");

echo(str("compact datums: shift=", functional_shift_x,
         " mm, overall mechanism length=",
         805 + functional_shift_x - chassis_min_x, " mm"));
echo(str("intake order: wheel contact X=", wheel_first_ball_center_x,
         " mm, ramp contact X=", compact_ramp_first_ball_center_x,
         " mm, wheel-first margin=", wheel_before_ramp_margin, " mm"));
echo(str("bridge: arch radial clearance=", motor_arch_radial_clearance,
         " mm, top ligament=", motor_arch_top_ligament,
         " mm, basket/hood notch x=", bridge_rear_x, "..",
         bridge_notch_front_x, " mm"));
echo(str("rear packaging: chassis/tray=", control_rear_margin,
         " mm, tray/battery=", control_battery_gap,
         " mm, battery/basket=", battery_basket_gap, " mm"));

module rounded_box(size_xyz, radius=8) {
    hull()
        for (xx = [-size_xyz[0]/2 + radius, size_xyz[0]/2 - radius],
             yy = [-size_xyz[1]/2 + radius, size_xyz[1]/2 - radius],
             zz = [-size_xyz[2]/2 + radius, size_xyz[2]/2 - radius])
            translate([xx, yy, zz]) sphere(r=radius);
}

module fixed_chassis_and_drive() {
    chassis_plate_option_a();

    for (xx = [-wheel_x, wheel_x], sy = [-1, 1]) {
        color("#242424")
            translate([xx, sy * wheel_y, wheel_d / 2])
                rotate([90, 0, 0])
                    cylinder(d=wheel_d, h=wheel_width, center=true);
        color("slategray")
            translate([xx, sy * drive_motor_y, drive_motor_z])
                rotate([90, 0, 0])
                    cylinder(d=drive_motor_d, h=drive_motor_l, center=true);
        color("silver")
            translate([xx, sy * 300, drive_motor_z])
                rotate([90, 0, 0])
                    cylinder(d=24, h=40, center=true);
    }
}

module compact_bridge() {
    difference() {
        // Reuse the actual Option A bridge solid, then apply study-only cuts.
        plywood_bridge();

        // Open rear edge: basket and carriage can lift vertically through it.
        translate([(bridge_rear_x + bridge_notch_front_x) / 2,
                   0,
                   bridge_under_z + bridge_t/2])
            cube([bridge_notch_front_x - bridge_rear_x + 2,
                  2 * bridge_notch_half_y,
                  bridge_t + 4], center=true);

        // Open-bottom circular arches through the two wooden side legs.
        for (sy = [-1, 1])
            translate([front_motor_local_x, sy * bridge_upright_y,
                       motor_arch_center_z])
                rotate([90, 0, 0])
                    cylinder(d=motor_arch_d,
                             h=bridge_upright_t + 4, center=true);
    }

    // External angle/doubler references.  They remain wholly outside the
    // basket service corridor and bridge the weakened motor-arch region.
    color("dimgray")
        for (sy = [-1, 1])
            translate([front_motor_local_x, sy * bridge_upright_y,
                       bridge_under_z - 7])
                cube([120, 6, 12], center=true);
}

function compact_ramp_z(x) =
    let(t = max(0, min(1,
        (compact_ramp_front_x-x)
        / (compact_ramp_front_x-compact_ramp_rear_x))))
    compact_ramp_front_z
    + (compact_ramp_rear_z-compact_ramp_front_z) * (t*t*(3-2*t));

module compact_handoff_ramp_unrelieved() {
    // New study geometry; do not export as a replacement Option A part until
    // the loaded-ball Gazebo sweep and a physical rolling check both pass.
    color("goldenrod")
        union()
            for (i = [0:16-1]) {
                x0 = compact_ramp_front_x
                    + (compact_ramp_rear_x-compact_ramp_front_x)*i/16;
                x1 = compact_ramp_front_x
                    + (compact_ramp_rear_x-compact_ramp_front_x)*(i+1)/16;
                hull() {
                    translate([x0, 0, compact_ramp_z(x0)/2])
                        cube([0.7, compact_ramp_width,
                              compact_ramp_z(x0)], center=true);
                    translate([x1, 0, compact_ramp_z(x1)/2])
                        cube([0.7, compact_ramp_width,
                              compact_ramp_z(x1)], center=true);
                }
                for (sy = [-1, 1])
                    hull() {
                        translate([x0, sy*(compact_ramp_width/2+2),
                                   (compact_ramp_z(x0)
                                    + compact_ramp_wall_h)/2])
                            cube([0.7, 4,
                                  compact_ramp_z(x0)
                                  + compact_ramp_wall_h], center=true);
                        translate([x1, sy*(compact_ramp_width/2+2),
                                   (compact_ramp_z(x1)
                                    + compact_ramp_wall_h)/2])
                            cube([0.7, 4,
                                  compact_ramp_z(x1)
                                  + compact_ramp_wall_h], center=true);
                    }
            }
}

module compact_handoff_ramp() {
    compact_parked_ramp_relief()
        compact_handoff_ramp_unrelieved();
}

module shifted_option_a_intake() {
    color("peru", 0.78) compact_bridge();
    curved_cheek(1);
    curved_cheek(-1);
    compact_handoff_ramp();
    tilted_wheel_motor_pod(1);
    tilted_wheel_motor_pod(-1);
    intake_ir_beams();

    if (show_intake_ball_path) {
        // Spaced references show the open mouth and the wheel-first order;
        // they are not simultaneous balls in a dynamic simulation.
        for (xx = [760, 610, wheel_first_ball_center_x])
            color("greenyellow", 0.76)
                translate([xx, 0, ball_center_z]) sphere(d=ball_d);

        color("limegreen", 0.86)
            translate([wheel_first_ball_center_x, 0, 2])
                cube([2, 330, 4], center=true);
        color("crimson", 0.86)
            translate([compact_ramp_first_ball_center_x, 0, 2])
                cube([2, 330, 4], center=true);
    }
}

module basket_collect_pose(alpha=1.0) {
    color("gainsboro", alpha) compact_relieved_bin();
}

module basket_launch_pose_local() {
    translate([0, 0, lift_travel])
        translate(launch_pivot)
            rotate([0, launch_tilt_deg, 0])
                translate(-launch_pivot) {
                    compact_relieved_bin();
                }
}

module shifted_functional_group() {
    translate([functional_shift_x, 0, 0]) {
        shifted_option_a_intake();
        if (show_basket_system) {
            compact_basket_guides();
            color("lightsteelblue") compact_fixed_hood();

            if (pose == "collect")
                basket_collect_pose();
            else if (pose == "launch") {
                compact_raised_basket_holders();
                basket_launch_pose_local();
            }
            else if (pose == "both") {
                basket_collect_pose(alpha=0.22);
                compact_raised_basket_holders();
                basket_launch_pose_local();
            } else
                assert(false, str("Unknown pose: ", pose));
        }

        if (show_launcher)
            translate(launcher_origin)
                launcher_oriented(orientation="side_by_side", nip_height=215);
    }
}

module motion_tray_installed_reference() {
    color("lightgray") electronics_tray();

    color("royalblue", 0.88)
        translate([mega_origin[0], mega_origin[1],
                   tray_t + mega_standoff_h])
            cube([mega_size[0], mega_size[1], 2]);

    for (origin = driver_origins)
        color("darkgreen", 0.86)
            translate([origin[0], origin[1],
                       tray_t + driver_standoff_h])
                cube([driver_size[0], driver_size[1], 39]);

    color("orange", 0.42)
        translate([relay_bay_origin[0], relay_bay_origin[1], tray_t])
            cube([relay_bay_size[0], relay_bay_size[1], 28]);
    color("gold", 0.42)
        translate([fuse_bay_origin[0], fuse_bay_origin[1], tray_t])
            cube([fuse_bay_size[0], fuse_bay_size[1], 22]);
}

module rear_electronics_packaging() {
    color("darkslategray", 0.92)
        translate([battery_center_x, 0,
                   battery_bottom_z + battery_size[2]/2])
            rounded_box(battery_size, 6);

    // Cyclic axis mapping: tray local X -> world Y, local Y -> world Z,
    // local Z -> world X.  Components face forward for service access.
    translate([control_plane_x, 0, control_base_z])
        rotate([90, 0, 90])
            translate([-tray_size[0]/2, 0, 0])
                motion_tray_installed_reference();
}

module basket_service_keepouts() {
    // Hard corridor for all collect/launch movement and vertical extraction.
    if (show_basket_keepout)
        color("mediumseagreen", 0.12)
            translate([functional_shift_x
                       + (basket_rear_x + basket_service_front_x)/2,
                       0,
                       25 + (basket_service_top_z - 25)/2])
                cube([basket_service_front_x - basket_rear_x,
                      2 * basket_service_half_y,
                      basket_service_top_z - 25], center=true);

    // Drive-motor service cylinders used to define the wooden arches.
    if (show_motor_keepouts)
        color("crimson", 0.28)
            for (sy = [-1, 1])
                translate([wheel_x, sy * bridge_upright_y,
                           motor_arch_center_z])
                    rotate([90, 0, 0])
                        cylinder(d=motor_arch_d,
                                 h=bridge_upright_t + 12, center=true);
}

module roof_and_shell_references() {
    if (show_shell_envelope)
        color("lightskyblue", 0.07)
            translate([(chassis_min_x + (790 + functional_shift_x))/2,
                       0,
                       (58 + roof_z)/2])
                cube([(790 + functional_shift_x) - chassis_min_x,
                      564, roof_z - 58], center=true);

    // Flat roof hatch follows the basket; yellow outline is the sealing land.
    color("gold", 0.72)
        translate([hatch_center_x, 0, roof_z])
            difference() {
                cube([hatch_size[0] + 12, hatch_size[1] + 12, 3], center=true);
                cube([hatch_size[0], hatch_size[1], 5], center=true);
            }

    // LiDAR remains on the upper chassis and is not moved with the mechanism.
    color("black") {
        translate([-420, 0, 478]) cylinder(d=95, h=20);
        translate([-420, 0, lidar_scan_z]) cylinder(d=78, h=36);
    }
}

module study_labels() {
    if (show_labels) {
        color("black")
            translate([-452, -286, 245])
                rotate([90, 0, 90])
                    linear_extrude(height=2)
                        text(str("functional shift ", functional_shift_x,
                                 " mm"), size=15);
        color("mediumseagreen")
            translate([functional_shift_x + basket_rear_x,
                       -basket_service_half_y - 8, 570])
                rotate([90, 0, 0])
                    linear_extrude(height=2)
                        text("BASKET SERVICE KEEP-OUT", size=12);
    }
}

// `show_drive=false` is useful for unobstructed bridge/motor-arch inspection.
if (show_drive) fixed_chassis_and_drive();
if (show_rear_electronics) rear_electronics_packaging();
shifted_functional_group();
roof_and_shell_references();
if (show_keepouts) basket_service_keepouts();
study_labels();
