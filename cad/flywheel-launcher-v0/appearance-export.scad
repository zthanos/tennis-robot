// Scaled appearance model for printing. NOT a manufacturing export.
//
// The README rule stands: no manufacturing STLs leave this directory while the
// datums are provisional. This file exists for a different purpose - a solid
// massing model, printed small, so the silhouette and the aperture composition
// can be judged in the hand instead of in a flat OpenSCAD render.
//
// It reuses the study's own 2D outlines through use<>, so the printed shape
// cannot drift from the study: change the study, re-export, they agree.

use <external-panel-study.scad>

$fn = 96; // use<> does not import the study's $fn; set it locally

part = "layout"; // "body_left", "body_right", "wheel", "layout", "assembled"

// Bambu Lab P2S. Confirm the real build volume before slicing; if it differs,
// only model_scale needs to move.
bed = [256, 256, 256];
model_scale = 1 / 6;

// Feature sizes are held at MODEL scale, not scaled down with the body. A 3 mm
// shut gap would become 0.5 mm at 1:6 and disappear under a 0.4 nozzle.
groove_w_model = 0.9;
groove_d_model = 0.7;
pocket_d_model = 1.6;
axle_peg_d_model = 3.0;   // a scaled-down 9 mm shaft would print at 1.5 mm
axle_fit_clear_model = 0.35;

// ---- Datums mirrored from external-panel-study.scad ----
side_skin_y = 282;
fixed_skin_bottom_z = 58;
uniform_shell_top_z = 463;
panel_t = 3;
roof_joint_mid_x = -22;
roof_joint_front_x = 405;
roof_shut_gap = 3;
handle_access_center = [220, 0];
handle_access_size = [110, 170];
handle_access_corner_r = 18;
ball_port_center = [109, 140];
ball_port_size = [180, 62];
basket_window_center = [164, 310];
basket_window_size = [290, 100];
basket_window_corner_r = 18;
nose_front_x = 790;
front_fascia_bottom_z = 168;
front_exit_open_d = 116;
launcher_recess_outer_d = 164;
launcher_recess_depth = 42;
launcher_pitch_ref_deg = 20;
launcher_nip_z_ref = 215;
launcher_origin_x_ref = 560;
wheel_arch_d = 205;
wheel_center_z_ref = 85;
drive_wheel_d_ref = 170;
drive_wheel_w_ref = 80;
wheel_station_x = 330;
wheel_center_y = 350;
lidar_x = -420;
pod_side = 80;
pod_corner_r = 16;
pod_top_z = 490;
sensor_side = 55;
sensor_top_z = 511;

// ---- Derived ----
body_top_z = uniform_shell_top_z + panel_t;
body_h = body_top_z - fixed_skin_bottom_z;
groove_w = groove_w_model / model_scale;
groove_d = groove_d_model / model_scale;
pocket_d = pocket_d_model / model_scale;
axle_peg_d = axle_peg_d_model / model_scale;
axle_hole_d = (axle_peg_d_model + axle_fit_clear_model) / model_scale;
roof_rear_panel_hi = roof_joint_mid_x - roof_shut_gap / 2;
roof_basket_panel_lo = roof_joint_mid_x + roof_shut_gap / 2;
roof_basket_panel_hi = roof_joint_front_x - roof_shut_gap / 2;
front_exit_center_z = launcher_nip_z_ref
                    + (nose_front_x - launcher_origin_x_ref)
                    * tan(launcher_pitch_ref_deg);

module body_mass() {
    translate([0, 0, fixed_skin_bottom_z])
        linear_extrude(height = body_h)
            rounded_body_plan_2d();
}

module intake_open_front() {
    // The nose skin stops at the Option A bridge top; below that the intake
    // mouth is open, and the massing model has to show that.
    translate([(420 + nose_front_x + 60) / 2, 0,
               (front_fascia_bottom_z + fixed_skin_bottom_z - 40) / 2])
        cube([nose_front_x + 60 - 420, 700,
              front_fascia_bottom_z - fixed_skin_bottom_z + 40], center = true);
}

module wheel_arches() {
    for (xx = [-wheel_station_x, wheel_station_x])
        translate([xx, 0, wheel_center_z_ref])
            rotate([90, 0, 0])
                cylinder(d = wheel_arch_d, h = 800, center = true);
}

module outline_groove_2d(w) {
    // Ring of width w following whatever 2D child outline is given.
    difference() {
        offset(delta = w / 2) children();
        offset(delta = -w / 2) children();
    }
}

module roof_grooves() {
    translate([0, 0, body_top_z - groove_d])
        linear_extrude(height = groove_d + 2) {
            outline_groove_2d(groove_w) roof_panel_2d(-700, roof_rear_panel_hi);
            outline_groove_2d(groove_w)
                roof_panel_2d(roof_basket_panel_lo, roof_basket_panel_hi);
            translate(handle_access_center)
                outline_groove_2d(groove_w)
                    rounded_rect_2d(handle_access_size,
                                    handle_access_corner_r);
        }
}

module side_pocket(center_xz, size_xz, r, depth) {
    for (side = [-1, 1])
        translate([center_xz[0], side * (side_skin_y + 1), center_xz[1]])
            // The extrusion must run inboard on both sides; a fixed rotate([90,
            // 0, 0]) sends it outside the body on -Y and cuts nothing there.
            rotate([90 * side, 0, 0])
                linear_extrude(height = depth + 1)
                    rounded_rect_2d(size_xz, r);
}

module launcher_recess_solid() {
    translate([nose_front_x, 0, front_exit_center_z])
        rotate([0, 90 - launcher_pitch_ref_deg, 0])
            translate([0, 0, -launcher_recess_depth])
                cylinder(d1 = front_exit_open_d,
                         d2 = launcher_recess_outer_d * 1.6,
                         h = launcher_recess_depth * 1.6);
}

module lidar_pod_mass() {
    translate([lidar_x, 0, uniform_shell_top_z])
        linear_extrude(height = pod_top_z - uniform_shell_top_z)
            offset(r = pod_corner_r) offset(delta = -pod_corner_r)
                square(pod_side, center = true);
    translate([lidar_x, 0, pod_top_z - 4])
        linear_extrude(height = sensor_top_z - pod_top_z + 4)
            offset(r = 8) offset(delta = -8)
                square(sensor_side, center = true);
}

module axle_pegs() {
    for (xx = [-wheel_station_x, wheel_station_x], sy = [-1, 1])
        translate([xx, sy * (side_skin_y - 20), wheel_center_z_ref])
            rotate([-90 * sy, 0, 0])
                cylinder(d = axle_peg_d, h = wheel_center_y - side_skin_y + 40);
}

module robot_appearance() {
    union() {
        difference() {
            union() {
                body_mass();
                lidar_pod_mass();
            }
            intake_open_front();
            wheel_arches();
            roof_grooves();
            side_pocket(ball_port_center, ball_port_size, 18, pocket_d);
            side_pocket(basket_window_center, basket_window_size,
                        basket_window_corner_r, pocket_d);
            launcher_recess_solid();
        }
        axle_pegs();
    }
}

module half(side) {
    // Split on the centre plane so each half prints on its own cut face: the
    // side surfaces that carry the design never touch a support.
    intersection() {
        robot_appearance();
        translate([0, side * 400, 0]) cube([2600, 800, 1400], center = true);
    }
}

module printable_half(side) {
    // Cut face down on the bed.
    scale(model_scale)
        rotate([side > 0 ? 90 : -90, 0, 0])
            translate([0, 0, -fixed_skin_bottom_z])
                half(side);
}

module printable_wheel() {
    scale(model_scale)
        difference() {
            cylinder(d = drive_wheel_d_ref, h = drive_wheel_w_ref);
            translate([0, 0, -1])
                cylinder(d = axle_hole_d, h = drive_wheel_w_ref * 0.7);
        }
}

module assembled_preview() {
    // Not for export: the glued result, for judging the shape before printing.
    robot_appearance();
    for (xx = [-wheel_station_x, wheel_station_x], sy = [-1, 1])
        translate([xx, sy * (wheel_center_y + drive_wheel_w_ref / 2), wheel_center_z_ref])
            rotate([90 * sy, 0, 0])
                cylinder(d = drive_wheel_d_ref, h = drive_wheel_w_ref);
}

if (part == "assembled") assembled_preview();
else if (part == "body_left") printable_half(1);
else if (part == "body_right") printable_half(-1);
else if (part == "wheel") printable_wheel();
else if (part == "layout") {
    printable_half(1);
    translate([0, 120, 0]) printable_half(-1);
    for (i = [0 : 3])
        translate([-130 + i * 50, -75, 0]) printable_wheel();
} else
    assert(false, str("Unknown part: ", part));
