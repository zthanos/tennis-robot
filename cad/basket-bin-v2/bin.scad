// Removable wire-mesh bin (one weldment): sunken floor, walls,
// load-management tray, receiving chute, corner guards, centre lip,
// support flange and carry handles. No electrical parts — the IR
// sensors stay on the chassis (spec §2).
include <params.scad>
use <lib.scad>

module bin_floor() {
    // top face at floor_top_z; front underside edge acts as the skid,
    // chamfer noted in README (mesh rod already rounds it in practice)
    translate([(bin_rear_x + bin_front_x) / 2, 0, floor_top_z])
        mesh_panel(bin_length, 2 * bin_half_width);
}

module bin_management_tray() {
    // inclined panel: top 25 @ x280 -> 35 @ x420, full bin width
    len = sqrt(mgmt_run ^ 2 + mgmt_rise ^ 2);
    translate([mgmt_rear_x, 0, floor_top_z])
        rotate([0, -mgmt_angle, 0])
            translate([len / 2, 0, 0])
                mesh_panel(len, 2 * bin_half_width);
}

module bin_receiving_chute() {
    // continues the tray toward the wheels: top 35 @ x420 -> 40 @ x470
    len = sqrt(recv_run ^ 2 + recv_rise ^ 2);
    translate([bin_front_x, 0, recv_rear_top_z])
        rotate([0, -recv_angle, 0])
            translate([len / 2, 0, 0])
                mesh_panel(len, 2 * recv_half_width);
}

module bin_walls() {
    // side walls
    for (sy = [-1, 1])
        translate([(bin_rear_x + bin_front_x) / 2, sy * bin_half_width, floor_top_z])
            mesh_wall(bin_length, wall_height);
    // rear wall
    translate([bin_rear_x, 0, floor_top_z])
        rotate([0, 0, 90])
            mesh_wall(2 * bin_half_width, wall_height);
}

module bin_front_retention() {
    // corner guards close the two 50 mm gaps beside the launch channel
    for (sy = [-1, 1])
        translate([bin_front_x + wall_thickness / 2,
                   sy * (entry_half_width + bin_half_width) / 2,
                   floor_top_z + guard_height / 2])
            cube([wall_thickness, bin_half_width - entry_half_width,
                  guard_height], center = true);
    // low fixed centre lip: stored balls must climb it, the launched
    // ball clears it from above (log #52)
    translate([bin_front_x + wall_thickness / 2, 0,
               floor_top_z + center_lip_height / 2])
        cube([wall_thickness, 2 * entry_half_width, center_lip_height],
             center = true);
}

module bin_flange() {
    // rests on the chassis plate (top 52) on both sides and at the rear;
    // the bin never hangs from its floor (spec §2)
    for (sy = [-1, 1])
        translate([(bin_rear_x + bin_front_x) / 2,
                   sy * (open_half_wid + flange_width / 2),
                   plate_top_z + flange_thickness / 2])
            cube([bin_length, flange_width, flange_thickness], center = true);
    translate([open_rear_x - flange_width / 2, 0,
               plate_top_z + flange_thickness / 2])
        cube([flange_width, 2 * (open_half_wid + flange_width),
              flange_thickness], center = true);
    // drop struts tying the flange to the wall frames
    for (sy = [-1, 1], fx = [bin_rear_x + 20, bin_front_x - 20])
        translate([fx, sy * (bin_half_width + wall_thickness / 2), plate_top_z])
            cube([frame_d, wall_thickness, frame_d], center = true);
}

module bin_handles() {
    for (sy = [-1, 1])
        translate([(bin_rear_x + bin_front_x) / 2, sy * bin_half_width, wall_top_z])
            handle(120, 35);
}

module bin() {
    color("gainsboro") {
        bin_floor();
        bin_management_tray();
        bin_receiving_chute();
        bin_walls();
        bin_flange();
        bin_handles();
    }
    color("dimgray") bin_front_retention();
}

bin();
