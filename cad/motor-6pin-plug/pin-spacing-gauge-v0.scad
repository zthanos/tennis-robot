// First printable spacing gauge for a 1x6 motor-side male-pin connector.
// FIT CHECK ONLY: do not wire or power a motor through this part.
// Units: mm. Pin size, contact depth and socket envelope are not measured.

$fn = 48;

positions = 6;
pitch = 2.54;
body_length = 15.0;
body_width = 4.5;       // provisional; measure motor socket before plug design
body_thickness = 3.0;
square_pin_side = 0.64; // provisional individual metal pin cross-section
hole_clearance = 0.18; // total clearance across the square
pin1_mark_d = 0.9;
pin1_mark_depth = 0.5;

assert((positions-1)*pitch < body_length,
       "Contact pitch exceeds overall body length");
assert(square_pin_side + hole_clearance > 0);

difference() {
    translate([-body_length/2, -body_width/2, 0])
        cube([body_length, body_width, body_thickness]);

    for (i = [0:positions-1]) {
        x = (i-(positions-1)/2)*pitch;
        translate([x, 0, -0.02])
            linear_extrude(height=body_thickness+0.04)
                square(square_pin_side+hole_clearance, center=true);
    }

    // Recessed dot beside contact 1. It is an orientation mark, not a key.
    translate([-(positions-1)*pitch/2, body_width/2-0.8,
               body_thickness-pin1_mark_depth])
        cylinder(d=pin1_mark_d, h=pin1_mark_depth+0.02);
}
