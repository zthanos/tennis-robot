// Full robot packaging study: active Option A intake + basket v2.1 + battery
// + 4WD references + manual lift/tilt + flywheel launcher.
//
// IMPORTANT: Option A is imported read-only with `use`; no file under
// cad/collector-intake-v1/option-a is copied, edited or overridden here.
// Units: mm, ground frame, robot +X forward, +Y left.

use <../collector-intake-v1/option-a/option-a.scad>
use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/hood.scad>
use <../basket-bin-v2/chassis_context.scad>
use <launcher-envelope.scad>
use <compact-basket-support.scad>
use <compact-parked-reliefs.scad>

$fn = 72;

mode = "launch";           // "collect", "launch" or "both"
launcher_layout = "front"; // user requirement; side/rear remain comparisons
front_feed_mode = "opening"; // "opening" (baseline) or "rail"
launcher_orientation = "side_by_side"; // low baseline; "over_under" comparison
lidar_mount_style = "upper_frame"; // "upper_frame" baseline or "base_mast"

show_drive = true;
show_sensors = true;
show_lift_guides = true;
show_feed_dock = true;
show_option_a_ir = true;
show_current_camera_ghost = true;
show_height_datums = false;

// Existing basket and lift study references.
bin_rear_x = 20;
bin_front_x = 420;
bin_half_width = 140;
floor_top_z = 25;
wall_top_z = 250;
side_port_center_x = 180;
side_port_width_x = 92;
side_port_height_z = 82;
feed_side = 1;
launch_lift_z = 100;
launch_side_tilt_deg = -12 * feed_side;
side_launch_pivot = [side_port_center_x,
                     feed_side * bin_half_width,
                     floor_top_z];

// Front-opening baseline: reuse the current 180 mm basket entrance as the
// raised gravity outlet. Pivot at the existing receiving-chute front edge.
front_recv_x = 470;
front_recv_z = 40;
launch_front_tilt_deg = 12;
front_launch_pivot = [front_recv_x, 0, front_recv_z];

// Side launcher comparison: direct feed from the +Y side port.
side_launcher_origin = [side_port_center_x, 280, 20];

// Corrected front baseline: a 215 mm nip keeps the side-by-side launcher below
// both the raised basket rim and the LiDAR scan plane.  The old over/under
// stack remains selectable, but is intentionally shown as a tall comparison.
front_launcher_nip_z = 215;
front_launcher_origin = [560, 0, 0];
front_launcher_park_origin = [700, 0, 0];
rear_launcher_origin = [-330, 0, 0];

// Packaging datums for the selected low launcher baseline.
lidar_scan_z = 498;
feeder_crest_z = 275;
launcher_packaging_ceiling_z = 370;
basket_launch_outlet_z = front_recv_z + launch_lift_z + 2;
basket_launch_rim_max_z = wall_top_z + launch_lift_z
                        + (front_recv_x - bin_rear_x)
                        * sin(launch_front_tilt_deg);

module height_datum(z_pos, label, tint) {
    color(tint, 0.82) {
        translate([120, -395, z_pos])
            cube([1240, 5, 5], center=true);
        translate([-495, -397, z_pos + 7])
            rotate([90, 0, 0])
                linear_extrude(height=2)
                    text(label, size=18, halign="left", valign="bottom");
    }
}

module selected_layout_height_datums() {
    height_datum(basket_launch_outlet_z,
                 str("basket outlet ", basket_launch_outlet_z, " mm"),
                 "seagreen");
    height_datum(front_launcher_nip_z,
                 str("flywheel nip ", front_launcher_nip_z, " mm"),
                 "darkorange");
    height_datum(feeder_crest_z,
                 str("feeder crest ", feeder_crest_z, " mm"),
                 "mediumseagreen");
    height_datum(launcher_packaging_ceiling_z,
                 str("launcher envelope <= ",
                     launcher_packaging_ceiling_z, " mm"),
                 "crimson");
    height_datum(basket_launch_rim_max_z,
                 str("raised basket rim ~",
                     round(basket_launch_rim_max_z), " mm"),
                 "steelblue");
    height_datum(lidar_scan_z,
                 str("LiDAR scan ", lidar_scan_z, " mm"),
                 "deepskyblue");
}

module option_a_read_only_context() {
    chassis_plate_option_a();
    color("peru", 0.72) plywood_bridge();
    curved_cheek(1);
    curved_cheek(-1);
    short_handoff_ramp();
    tilted_wheel_motor_pod(1);
    tilted_wheel_motor_pod(-1);
    if (show_option_a_ir) intake_ir_beams();
}

module drive_wheel(x_pos, side) {
    color("#252525")
        translate([x_pos, side * 350, 85])
            rotate([90, 0, 0])
                cylinder(d=170, h=80, center=true);
    color("steelblue")
        translate([x_pos, side * 240, 85])
            rotate([90, 0, 0])
                cylinder(d=60, h=100, center=true);
    color("silver")
        translate([x_pos, side * 300, 85])
            rotate([90, 0, 0])
                cylinder(d=24, h=40, center=true);
}

module drivetrain_context() {
    for (xx = [-330, 330], sy = [-1, 1]) drive_wheel(xx, sy);
}

module sensor_context() {
    // LiDAR scan position stays unchanged. The selected shell baseline carries
    // it on a short bracket tied to the structural upper rear crossmember,
    // rather than on the earlier long mast rising from the chassis plate.
    if (lidar_mount_style == "upper_frame") {
        color("dimgray") {
            translate([-420, 0, 462]) cube([92, 118, 14], center=true);
            for (sy = [-1, 1])
                hull() {
                    translate([-438, sy * 48, 445])
                        cube([18, 18, 18], center=true);
                    translate([-420, sy * 48, 462])
                        cube([18, 18, 14], center=true);
                }
            translate([-420, 0, 478]) cylinder(d=34, h=26, center=true);
        }
    } else if (lidar_mount_style == "base_mast")
        color("dimgray")
            translate([-420, 0, (52 + 475) / 2])
                cube([25, 25, 475 - 52], center=true);
    else
        assert(false, str("Unknown lidar_mount_style: ", lidar_mount_style));

    color("black")
        translate([-420, 0, lidar_scan_z]) cylinder(d=94, h=45, center=true);

    if (launcher_layout == "front") {
        // The current OAK-D position intersects the front flywheel cradle.
        if (show_current_camera_ghost) {
            color("crimson", 0.28)
                translate([535, 0, 443]) cube([92, 30, 30], center=true);
            color("crimson", 0.22)
                translate([535, 0, (168 + 428) / 2])
                    cube([18, 18, 428 - 168], center=true);
        }
        // Selected packaging candidate: OAK-D mounts externally on the closed
        // front fascia, below the launcher cylinder and above the Option A
        // cheek/bridge top. The crossbar is structural; the cosmetic panel is
        // only a locating/optical surface.
        color("dimgray")
            translate([775, 0, 184]) cube([18, 270, 18], center=true);
        color("midnightblue")
            translate([800, 0, 205]) cube([92, 30, 30], center=true);
        color("dimgray")
            translate([786, 0, 195]) cube([34, 18, 42], center=true);
    } else {
        color("midnightblue")
            translate([535, 0, 443]) cube([92, 30, 30], center=true);
        color("dimgray")
            translate([535, 0, (168 + 428) / 2])
                cube([18, 18, 428 - 168], center=true);
    }
}

module lift_system_context() {
    compact_basket_guides();
    if (mode == "launch" || mode == "both")
        compact_raised_basket_holders();
}

module side_port_cutout() {
    translate([side_port_center_x,
               feed_side * bin_half_width,
               floor_top_z + side_port_height_z / 2])
        cube([side_port_width_x, 24, side_port_height_z], center=true);
}

module side_gate(open=false) {
    // Basket-integral fail-closed door. In launch mode a dock-mounted cam
    // rotates it outward only after the basket reaches the upper hard stop.
    color("crimson", 0.9)
        if (!open)
            translate([side_port_center_x,
                       feed_side * (bin_half_width + 3),
                       floor_top_z + side_port_height_z / 2])
                cube([side_port_width_x + 8, 6,
                      side_port_height_z + 8], center=true);
        else
            translate([side_port_center_x,
                       feed_side * (bin_half_width + 3),
                       floor_top_z + side_port_height_z])
                rotate([feed_side * 80, 0, 0])
                    translate([0, 0, -side_port_height_z / 2])
                        cube([side_port_width_x + 8, 6,
                              side_port_height_z], center=true);
}

module basket_with_side_port(gate_open=false) {
    difference() {
        compact_relieved_bin();
        side_port_cutout();
    }
    side_gate(gate_open);
}

module collect_basket_assembly() {
    if (launcher_layout == "side")
        basket_with_side_port(gate_open=false);
    else
        compact_relieved_bin();
    compact_relieved_hood();
}

module launch_basket_transform() {
    if (launcher_layout == "front")
        translate([0, 0, launch_lift_z])
            translate(front_launch_pivot)
                rotate([0, launch_front_tilt_deg, 0])
                    translate(-front_launch_pivot)
                        children();
    else
        translate([0, 0, launch_lift_z])
            translate(side_launch_pivot)
                rotate([launch_side_tilt_deg, 0, 0])
                    translate(-side_launch_pivot)
                        children();
}

module launch_basket_assembly() {
    launch_basket_transform() {
        if (launcher_layout == "side")
            basket_with_side_port(gate_open=true);
        else
            bin();
    }
}

module side_gravity_dock() {
    // Dock/singulator keepout from the raised side port into the launcher
    // breech. The green hull is not a final chute or printable part.
    color("mediumseagreen", 0.35)
        hull() {
            translate([side_port_center_x, 152,
                       floor_top_z + launch_lift_z + 43])
                cube([92, 12, 82], center=true);
            translate([side_port_center_x, 184, 420])
                rotate([90, 0, 0]) cylinder(d=90, h=12, center=true);
        }
}

module front_gravity_dock() {
    // Funnel/singulator plus spring-assisted transfer. The raised basket outlet
    // is below the 215 mm nip, so the provisional tube rises to a 275 mm crest
    // and then descends into the launcher breech.  This is a keepout, not a
    // final tube profile or a selected spring.
    source = [472, 0, basket_launch_outlet_z];
    crest = [500, 0, feeder_crest_z];
    breech = [520, 0, front_launcher_nip_z];
    color("mediumseagreen", 0.35) {
        hull() {
            translate(source)
                rotate([0, 90, 0]) cylinder(d=90, h=8, center=true);
            translate(crest)
                rotate([0, 90, 0]) cylinder(d=90, h=8, center=true);
        }
        hull() {
            translate(crest)
                rotate([0, 90, 0]) cylinder(d=90, h=8, center=true);
            translate(breech)
                rotate([0, 90, 0]) cylinder(d=90, h=8, center=true);
        }
    }
    color("crimson", 0.85)
        translate([486, 0, 176])
            rotate([90, 0, 0]) cylinder(d=78, h=24, center=true);
}

module front_launcher_rails() {
    // Envelope-only rails for the alternative moving-launcher concept.
    // Docked X=560 in launch; parked X=700 in collect.
    for (sy = [-1, 1])
        color("silver")
            translate([630, sy * 58, 188])
                cube([300, 18, 18], center=true);
}

module placed_launcher() {
    if (launcher_layout == "side")
        translate(side_launcher_origin)
            rotate([0, 0, 90])
                launcher_oriented(orientation=launcher_orientation,
                                  nip_height=front_launcher_nip_z);
    else if (launcher_layout == "front") {
        if (front_feed_mode == "rail" && mode == "collect")
            translate(front_launcher_park_origin)
                launcher_oriented(orientation=launcher_orientation,
                                  nip_height=front_launcher_nip_z);
        else
            translate(front_launcher_origin)
                launcher_oriented(orientation=launcher_orientation,
                                  nip_height=front_launcher_nip_z);
        if (front_feed_mode == "rail") front_launcher_rails();
    }
    else if (launcher_layout == "rear")
        translate(rear_launcher_origin)
            rotate([0, 0, 180])
                launcher_oriented(orientation=launcher_orientation,
                                  nip_height=front_launcher_nip_z);
    else
        assert(false, str("Unknown launcher_layout: ", launcher_layout));
}

module full_robot_context() {
    option_a_read_only_context();
    battery();
    if (show_drive) drivetrain_context();
    if (show_sensors) sensor_context();
    if (show_lift_guides) lift_system_context();

    // Launcher stays mounted in both modes; interlock state changes.
    placed_launcher();

    if (mode == "collect")
        collect_basket_assembly();
    else if (mode == "launch") {
        launch_basket_assembly();
        if (show_feed_dock && launcher_layout == "side") side_gravity_dock();
        if (show_feed_dock && launcher_layout == "front") front_gravity_dock();
    } else if (mode == "both") {
        color("seagreen", 0.28) collect_basket_assembly();
        launch_basket_assembly();
        if (show_feed_dock && launcher_layout == "side") side_gravity_dock();
        if (show_feed_dock && launcher_layout == "front") front_gravity_dock();
    } else
        assert(false, str("Unknown mode: ", mode));

    if (show_height_datums) selected_layout_height_datums();
}

full_robot_context();
