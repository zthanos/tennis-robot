// Hand-test ski-jumps (2026-10-16). Same shape family as the P2-51 robot ski:
// flat tangent run-in + circular arc of radius R turning the ball up to the
// geometric exit angle. Roll/throw a tennis ball along the table into it and
// watch the exit angle, height and how much speed it keeps.
//
// The ball travels toward +X. Riding surface = top of the profile; two side
// walls guide the ball. Print LYING ON A SIDE WALL (part="print"): every layer
// is the same 2D profile, so there is no overhang and the layer lines run
// along the ball path. PETG, 3 walls, 15% infill.
//
// Variants: 42 deg R75 (robot baseline), 60 deg R75, 75 deg R75, 75 deg R110.

angle  = 75;      // geometric exit angle, deg
radius = 75;      // arc radius, mm
part   = "print"; // [print, view]

run_in   = 52;    // flat tangent part (as on the robot)
lip      = 8;     // tapered entry so a rolling ball climbs on
t        = 2.5;   // surface height above the table on the flat part
inner_w  = 80;    // between the side walls (ball 67)
wall_t   = 3;
wall_h   = 18;    // side wall height above the riding surface
$fn = 96;

function surf(phi) = [radius*sin(phi), t + radius*(1-cos(phi))];
steps = 60;
surface_pts = [for (i=[0:steps]) surf(angle*i/steps)];
exit_pt = surface_pts[steps];

// Solid profile: table level up to the riding surface.
module profile_solid() {
    polygon(concat(
        [[-run_in, 0], [-run_in + lip, t]],
        surface_pts,
        [[exit_pt[0], 0]]
    ));
}

// Side-wall profile: the solid plus a band wall_h above the surface,
// measured along the surface normal.
function above(phi) = [ (radius - wall_h)*sin(phi), t + radius - (radius - wall_h)*cos(phi) ];
module profile_wall() {
    union() {
        profile_solid();
        polygon(concat(
            [[-run_in, 0], [-run_in, t + wall_h]],
            [for (i=[0:steps]) above(angle*i/steps)],
            [exit_pt, [exit_pt[0], 0]]
        ));
    }
}

// Engraved 0.6 mm into the outer face of the +Y side wall.
module label() {
    translate([-run_in/2 + 4, inner_w/2 + wall_t + 0.4, t + wall_h/2]) rotate([90, 0, 0])
        linear_extrude(height = 1) text(str(angle, " R", radius), size = 6, halign = "center", valign = "center");
}

module ski() {
    // XZ profiles extruded along Y.
    rotate([90, 0, 0]) {
        translate([0, 0, -inner_w/2]) linear_extrude(height = inner_w) profile_solid();
        translate([0, 0, inner_w/2]) linear_extrude(height = wall_t) profile_wall();
        translate([0, 0, -inner_w/2 - wall_t]) linear_extrude(height = wall_t) profile_wall();
    }
}

if (part == "print") rotate([90, 0, 0]) translate([0, inner_w/2 + wall_t, 0]) difference() { ski(); label(); }
else difference() { ski(); label(); }

echo(SKI = str(angle, " deg R", radius), exit_height_mm = exit_pt[1],
     length_mm = run_in + exit_pt[0], ball_centre_exit_z_mm = exit_pt[1] + 33.5 * cos(angle));
