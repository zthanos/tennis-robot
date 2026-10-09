// Thin drill-pattern gauge for the purchased 37 mm steel gearmotor mount.
//
// PROVISIONAL MEASURED DATA — 2026-10-09
// Print this inexpensive coupon first and compare it with the real bracket.
// It is not a structural motor mount and must not be used as a drill template
// until all four holes accept the real fasteners without forcing the parts.
//
// Coordinate convention:
//   +X points away from the motor-side upright.
//   Y is across the 40 mm mounting foot.
//   The motor-side edge is X=0 and carries the orientation notch/label.

$fn = 48;

foot_length_x = 40;          // measured approximately
foot_width_y = 40;           // measured approximately
gauge_thickness = 2.0;

first_row_x = 9;             // motor-side edge to first hole centre
row_spacing_x = 25;          // longitudinal centre spacing
column_spacing_y = 30;       // transverse centre spacing

measured_hole_d = 4.0;
print_clearance = 0.2;
gauge_hole_d = measured_hole_d + print_clearance;

orientation_notch_depth = 3;
orientation_notch_width = 8;
label_height = 0.4;

assert(first_row_x > 0);
assert(first_row_x + row_spacing_x < foot_length_x);
assert(column_spacing_y < foot_width_y);

module outline_2d() {
    difference() {
        square([foot_length_x, foot_width_y]);

        // Centred notch marks the edge nearest the motor upright.
        translate([-0.01,
                   (foot_width_y - orientation_notch_width) / 2])
            square([orientation_notch_depth + 0.01,
                    orientation_notch_width]);
    }
}

module mounting_holes() {
    for (x = [first_row_x, first_row_x + row_spacing_x],
         y = [(foot_width_y - column_spacing_y) / 2,
              (foot_width_y + column_spacing_y) / 2])
        translate([x, y, -0.1])
            cylinder(d=gauge_hole_d, h=gauge_thickness + 0.2);
}

difference() {
    linear_extrude(gauge_thickness)
        outline_2d();
    mounting_holes();
}

// Keep the label shallow so the coupon remains a quick, support-free print.
translate([foot_length_x / 2, foot_width_y / 2, gauge_thickness])
    linear_extrude(label_height)
        text("< MOTOR", size=3.2, halign="center", valign="center");
