// Wire-mesh panel primitives for the basket weldment.
include <params.scad>

// Flat mesh panel in the local XY plane, centered at the origin.
// Perimeter uses frame_d rod, infill uses wire_d at <= mesh_pitch spacing.
// Top face of the panel sits at z = 0 (rods hang below it) so callers can
// position panels by their working surface.
module mesh_panel(lx, ly) {
    nx = max(1, ceil(lx / mesh_pitch));
    ny = max(1, ceil(ly / mesh_pitch));
    translate([0, 0, -frame_d / 2]) {
        // perimeter frame
        for (sy = [-1, 1])
            translate([0, sy * (ly - frame_d) / 2, 0])
                cube([lx, frame_d, frame_d], center = true);
        for (sx = [-1, 1])
            translate([sx * (lx - frame_d) / 2, 0, 0])
                cube([frame_d, ly, frame_d], center = true);
        // infill wires along x
        for (i = [1 : ny - 1])
            translate([0, -ly / 2 + i * ly / ny, 0])
                cube([lx, wire_d, wire_d], center = true);
        // infill wires along y
        for (i = [1 : nx - 1])
            translate([-lx / 2 + i * lx / nx, 0, 0])
                cube([wire_d, ly, wire_d], center = true);
    }
}

// Vertical mesh wall in the local XZ plane (length lx, height hz),
// centered in x, base at z = 0, panel mid-plane at y = 0.
module mesh_wall(lx, hz) {
    translate([0, 0, hz / 2])
        rotate([90, 0, 0])
            mesh_panel(lx, hz);
}

// U-shaped carry handle in the local XZ plane: span in x, rising +z.
module handle(span, rise, rod = 8) {
    for (sx = [-1, 1])
        translate([sx * span / 2, 0, rise / 2])
            cube([rod, rod, rise], center = true);
    translate([0, 0, rise - rod / 2])
        cube([span + rod, rod, rod], center = true);
}
