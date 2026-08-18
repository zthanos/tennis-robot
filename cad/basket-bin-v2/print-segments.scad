// 220 x 220 mm print-bed segments for basket bin v2.1.
//
// Select one part on the command line, for example:
//   openscad -D 'part="floor_tile"' -o floor_tile.stl print-segments.scad
//
// The quantity required for a complete basket is documented in README.md.
// Panels are exported flat, with their lowest face at z=0. Adjacent perimeter
// frames can be lashed together with zip ties, or with the joiner_plate part.

include <params.scad>
use <lib.scad>

part = "layout";
$fn = 48;

floor_tile_x = bin_length / 2;       // 200 mm
floor_tile_y = bin_half_width;       // 140 mm
tray_tile_x = sqrt(mgmt_run ^ 2 + mgmt_rise ^ 2);
tray_tile_y = bin_half_width;        // 140 mm
side_tile_x = bin_length / 2;        // 200 mm
side_tile_y = wall_height / 2;       // 112.5 mm
rear_tile_x = bin_half_width;        // 140 mm
rear_tile_y = wall_height / 2;       // 112.5 mm
chute_tile_x = sqrt(recv_run ^ 2 + recv_rise ^ 2);
rear_flange_segment_length = (2 * (open_half_wid + flange_width)) / 2;

module printable_panel(lx, ly) {
    // mesh_panel() has its top at z=0; lift it onto the print bed.
    translate([0, 0, frame_d]) mesh_panel(lx, ly);
}

module floor_tile() {
    printable_panel(floor_tile_x, floor_tile_y);
}

module management_tray_tile() {
    printable_panel(tray_tile_x, tray_tile_y);
}

module side_wall_tile() {
    printable_panel(side_tile_x, side_tile_y);
}

module rear_wall_tile() {
    printable_panel(rear_tile_x, rear_tile_y);
}

module receiving_chute_tile() {
    printable_panel(chute_tile_x, 2 * recv_half_width);
}

module side_flange_segment() {
    translate([0, 0, flange_thickness / 2])
        cube([bin_length / 2, flange_width, flange_thickness], center = true);
}

module rear_flange_segment() {
    translate([0, 0, flange_thickness / 2])
        cube([rear_flange_segment_length, flange_width,
              flange_thickness], center = true);
}

module corner_guard() {
    // Lay the original 10 x 50 x 20 mm guard on its broad face.
    translate([0, 0, wall_thickness / 2])
        cube([bin_half_width - entry_half_width, guard_height,
              wall_thickness], center = true);
}

module center_lip() {
    translate([0, 0, center_lip_height / 2])
        cube([2 * entry_half_width, wall_thickness,
              center_lip_height], center = true);
}

module carry_handle() {
    // The handle is modelled in XZ. Rotate it onto the bed for printing.
    translate([0, 0, 4]) rotate([90, 0, 0]) handle(120, 35);
}

module drop_strut() {
    translate([0, 0, frame_d / 2])
        cube([wall_thickness, frame_d, frame_d], center = true);
}

module joiner_plate() {
    // General-purpose lashing plate. Four 4.5 mm holes accept common zip ties
    // or M4 hardware; use one plate on each side for a bolted clamp.
    difference() {
        translate([0, 0, 2]) cube([32, 16, 4], center = true);
        for (x = [-10, 10], y = [-4, 4])
            translate([x, y, -1]) cylinder(d = 4.5, h = 6);
    }
}

module printable_layout() {
    // Visual index only; export named parts separately for slicing.
    translate([-105,  78, 0]) floor_tile();
    translate([  75,  78, 0]) management_tray_tile();
    translate([-105, -65, 0]) side_wall_tile();
    translate([  75, -65, 0]) rear_wall_tile();
    translate([ 175,  45, 0]) receiving_chute_tile();
    translate([ 175, -50, 0]) joiner_plate();
}

if (part == "floor_tile") floor_tile();
else if (part == "management_tray_tile") management_tray_tile();
else if (part == "side_wall_tile") side_wall_tile();
else if (part == "rear_wall_tile") rear_wall_tile();
else if (part == "receiving_chute_tile") receiving_chute_tile();
else if (part == "side_flange_segment") side_flange_segment();
else if (part == "rear_flange_segment") rear_flange_segment();
else if (part == "corner_guard") corner_guard();
else if (part == "center_lip") center_lip();
else if (part == "carry_handle") carry_handle();
else if (part == "drop_strut") drop_strut();
else if (part == "joiner_plate") joiner_plate();
else printable_layout();
