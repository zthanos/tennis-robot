// Robot electronics packaging comparison.
// Uses the current motion tray geometry and the full robot integration without
// changing either source. Pi-case dimensions remain provisional until measured.
// Units: mm, ground frame, robot +X forward, +Y left.

use <robot-integration.scad>
include <../motion-electronics-tray/params.scad>
use <../motion-electronics-tray/tray.scad>

$fn = 48;

layout = "low_split"; // "low_split" selected or "stacked" comparison
mode = "launch";
show_robot_context = true;
show_packaging_datums = false;
show_service_keepouts = true;
show_labels = true;

// Chassis / existing body references.
chassis_rear_x = -460;
chassis_side_y = 290;
battery_min_x_ref = -226;
battery_max_x_ref = -60;
battery_half_y_ref = 99;
battery_top_z_ref = 222;
side_skin_y_ref = 282;

// The imported tray is 240 x 180 mm. In the selected layout it is rotated 90
// degrees so its shorter dimension occupies robot X behind the battery.
tray_center_low = [-340, 0];
tray_base_z_low = 58;
tray_world_x_min_low = tray_center_low[0] - tray_size[1] / 2;
tray_world_x_max_low = tray_center_low[0] + tray_size[1] / 2;
tray_world_half_y_low = tray_size[0] / 2;
tray_battery_x_gap = battery_min_x_ref - tray_world_x_max_low;

// Provisional external metal-case envelope: replace only after measuring the
// actual Pi 5 + M.2/NVMe + active-cooler case.
pi_case_size = [120, 90, 50];
pi_case_center_low = [-143, -190, 58 + pi_case_size[2] / 2];
pi_case_inner_y_edge = pi_case_center_low[1] + pi_case_size[1] / 2;
pi_case_outer_y_edge = pi_case_center_low[1] - pi_case_size[1] / 2;
pi_battery_lateral_gap = abs(pi_case_inner_y_edge)
                       - battery_half_y_ref;
pi_side_skin_gap = side_skin_y_ref - abs(pi_case_outer_y_edge);

// Vertical DFR0753 carrier keep-out. The actual board is about 18 x 25 mm;
// this larger box includes insulation, holder and local airflow, not cables.
buck_keepout_size = [35, 20, 40];
buck_center_low = [-143, -122, 58 + buck_keepout_size[2] / 2];
buck_battery_gap = abs(buck_center_low[1] + buck_keepout_size[1] / 2)
                 - battery_half_y_ref;
buck_pi_gap = abs(pi_case_inner_y_edge
                - (buck_center_low[1] - buck_keepout_size[1] / 2));

// Stacked comparison blocks the present vertical battery service path and
// raises the local electronics envelope to roughly 291 mm.
tray_center_stacked = [-143, 0];
tray_base_z_stacked = 240;

assert(tray_battery_x_gap >= 20,
       "selected rear tray needs at least 20 mm before the battery");
assert(pi_battery_lateral_gap >= 35,
       "Pi case needs service/air gap from the battery");
assert(pi_side_skin_gap >= 40,
       "Pi case needs cable/air gap from the side skin");
assert(buck_battery_gap >= 10 && buck_pi_gap >= 10,
       "vertical buck carrier needs at least 10 mm on both faces");

module rounded_case(size_xyz, radius=8) {
    // Envelope-only rounded box, not a printable case model.
    hull()
        for (xx = [-size_xyz[0] / 2 + radius,
                   size_xyz[0] / 2 - radius],
             yy = [-size_xyz[1] / 2 + radius,
                   size_xyz[1] / 2 - radius],
             zz = [-size_xyz[2] / 2 + radius,
                   size_xyz[2] / 2 - radius])
            translate([xx, yy, zz]) sphere(r=radius);
}

module pcb_reference(size_xy, z_pos, tint) {
    color(tint, 0.82)
        translate([0, 0, z_pos]) cube([size_xy[0], size_xy[1], 1.6]);
}

module motion_tray_installed_reference() {
    electronics_tray();

    translate(mega_origin) {
        pcb_reference(mega_size, tray_t + mega_standoff_h, "RoyalBlue");
        color("Silver")
            translate([-6, 32, tray_t + mega_standoff_h + 1.6])
                cube([16, 13, 11]);
    }

    translate(perf_origin)
        pcb_reference(perf_size, tray_t + perf_standoff_h, "SeaGreen");

    for (origin = driver_origins)
        translate(origin) {
            pcb_reference(driver_size,
                          tray_t + driver_standoff_h, "DarkGreen");
            color("Silver", 0.90)
                translate([8, 5,
                           tray_t + driver_standoff_h + 1.6])
                    cube([34, 40, 37]);
        }

    color("Orange", 0.30)
        translate([relay_bay_origin[0], relay_bay_origin[1], tray_t])
            cube([relay_bay_size[0], relay_bay_size[1], 28]);
    color("Gold", 0.28)
        translate([fuse_bay_origin[0], fuse_bay_origin[1], tray_t])
            cube([fuse_bay_size[0], fuse_bay_size[1], 22]);
}

module placed_motion_tray(center_xy, base_z, rotate_z=0) {
    translate([center_xy[0], center_xy[1], base_z])
        rotate([0, 0, rotate_z])
            translate([-tray_size[0] / 2, -tray_size[1] / 2, 0])
                motion_tray_installed_reference();
}

module pi_case_reference(center_pos) {
    color("#343A40", 0.92)
        translate(center_pos) rounded_case(pi_case_size, 7);

    // USB/Ethernet cable service volume at the outboard face.
    if (show_service_keepouts)
        color("DodgerBlue", 0.16)
            translate([center_pos[0],
                       center_pos[1] - pi_case_size[1] / 2 - 25,
                       center_pos[2]])
                cube([pi_case_size[0], 50, pi_case_size[2] + 20],
                     center=true);
}

module vertical_buck_reference(center_pos) {
    // Translucent keep-out includes the holder and air gap datum.
    color("MediumPurple", 0.24)
        translate(center_pos) cube(buck_keepout_size, center=true);

    // Small vertical PCB within the keep-out; plane is XZ, long direction X.
    color("MidnightBlue", 0.92)
        translate(center_pos) cube([25, 4, 18], center=true);
    color("DimGray")
        translate([center_pos[0], center_pos[1], center_pos[2] + 2])
            cube([12, 10, 12], center=true);
}

module packaging_datums() {
    // Lightweight datum view for top/service renders where the basket obscures
    // the floor bay. These are collision envelopes, not replacement geometry.
    color("BurlyWood", 0.22)
        translate([0, 0, 50]) cube([920, 580, 4], center=true);
    color("DarkSlateGray", 0.46)
        translate([(battery_min_x_ref + battery_max_x_ref) / 2,
                   0,
                   (52 + battery_top_z_ref) / 2])
            cube([battery_max_x_ref - battery_min_x_ref,
                  2 * battery_half_y_ref,
                  battery_top_z_ref - 52], center=true);
    color("LightSkyBlue", 0.18) {
        translate([0, side_skin_y_ref, 250])
            cube([900, 4, 400], center=true);
        translate([0, -side_skin_y_ref, 250])
            cube([900, 4, 400], center=true);
    }
}

module low_split_layout() {
    placed_motion_tray(tray_center_low, tray_base_z_low, 90);
    pi_case_reference(pi_case_center_low);
    vertical_buck_reference(buck_center_low);

    if (show_service_keepouts) {
        // Existing vertical battery extraction corridor.
        color("Gold", 0.12)
            translate([-143, 0, 365]) cube([205, 240, 285], center=true);
        // Short 5V cable corridor between buck and Pi case.
        color("LimeGreen", 0.30)
            hull() {
                translate([buck_center_low[0], buck_center_low[1] - 10,
                           buck_center_low[2]]) sphere(d=8);
                translate([pi_case_center_low[0], pi_case_inner_y_edge,
                           buck_center_low[2]]) sphere(d=8);
            }
    }
}

module stacked_layout() {
    placed_motion_tray(tray_center_stacked, tray_base_z_stacked, 0);
    pi_case_reference(pi_case_center_low);
    vertical_buck_reference(buck_center_low);

    if (show_service_keepouts)
        color("Crimson", 0.20)
            translate([-143, 0, 365]) cube([205, 240, 285], center=true);
}

module datum_label(pos, label, tint="black") {
    if (show_labels)
        color(tint)
            translate(pos)
                rotate([90, 0, 0])
                    linear_extrude(height=1)
                        text(label, size=15, halign="center");
}

if (show_robot_context) full_robot_context();
if (show_packaging_datums) packaging_datums();

if (layout == "low_split") {
    low_split_layout();
    datum_label([-340, 0, 125], "MOTION TRAY  rotated 90 deg");
    datum_label([-143, -190, 125], "PI CASE  provisional");
    datum_label([-143, -122, 112], "5V BUCK  vertical");
} else if (layout == "stacked") {
    stacked_layout();
    datum_label([-143, 0, 310], "STACKED  blocks battery service", "Crimson");
} else
    assert(false, str("Unknown electronics layout: ", layout));
