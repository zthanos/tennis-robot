// P2-99e: OAK-D S2 bracket from the P2-93 upper (LiDAR) beam, position B.
// Replaces p293d_oak_mount (50 deg plate at X450). Reuses the existing two
// M4 X-axis bores of the beam at [*, +-35, 354] (pads butt at X402) - no new
// hole in the chassis or the arch. Through-bolt M4 x 40 from behind the beam
// into a captive nut in each pad (project rule: load-path M4 = bolt + nut). Back plate perpendicular to the optical
// axis takes the camera's two VESA M4 (75 mm) from behind (M4 x 12, 6 mm
// plate + ~6 mm thread engagement of the 8 mm camera bores).
// Stiffness: per side one solid triangular web from the pad to the upper and
// lower plate nodes (9 mm thick, depth = full plate height at the plate),
// tapered inward in plan; the beam ties the pads, the plate ties the webs.
// Print: plate face on the bed, struts vertical (export pose below).
include <p299e-oak-s2.scad>

m4d = 4.6;
plate_t = 6;
plate_w = 96;            // along camera Y
plate_z = [-22, 8];      // camera-local Z range (lens axis at 0)
strut = 9;               // square strut section, mm
strut_y = 20;            // strut landing on the plate, +-Y
pad_c = [406, 35, 354];  // existing beam bore axis (p293d xhole), +-Y
pad = [12, 16, 20];      // pad block: contact face on the beam front X402
pad_x = 402 + 12 / 2;    // pad centre X

seat_gap = 0.2;          // nominal CAD clearance camera back <-> plate, closed by the M4 clamp
function plate_x() = -oak_size[0] - seat_gap - plate_t;   // camera-local X of the plate back face

module p299e_plate() {
  oak_frame() difference() {
    translate([plate_x(), -plate_w / 2, plate_z[0]])
      cube([plate_t, plate_w, plate_z[1] - plate_z[0]]);
    for (s = [-1, 1]) translate([0, s * oak_m4_y, -oak_m4_below_axis])
      rotate([0, 90, 0]) cylinder(d=m4d, h=200, center=true, $fn=24);
  }
}

module plate_node(s, z) {   // strut foot on the plate back face (camera frame)
  oak_frame() translate([plate_x() - strut / 2, s * strut_y, z]) cube(strut, center=true);
}

// Pad centre in the camera frame (inverse of oak_frame()).
function pad_cam(s) = let(dx = pad_x - oak_lens[0], dz = pad_c[2] - oak_lens[2], p = oak_pitch_deg)
  [dx * cos(p) - dz * sin(p), s * pad_c[1], dx * sin(p) + dz * cos(p)];

// Pad block hulled to a print-frame-aligned foot 16 mm toward the plate, so
// the 25 deg tilted pad does not start printing from a hanging corner.
module pad_block(s) {
  hull() {
    translate([pad_x, s * pad_c[1], pad_c[2]]) cube(pad, center=true);
    oak_frame() translate(pad_cam(s) + [16, 0, 0]) cube([2, pad[1], pad[2]], center=true);
  }
}

module pad_node(s) { translate([pad_x, s * pad_c[1], pad_c[2]]) cube(strut, center=true); }

module p299e_oak_bracket() {
  difference() {
    union() {
      p299e_plate();
      for (s = [-1, 1]) {
        pad_block(s);
        // one solid triangular web per side: pad + upper + lower plate node
        hull() { pad_block(s); plate_node(s, plate_z[1] - strut / 2); plate_node(s, plate_z[0] + strut / 2); }
      }
    }
    for (s = [-1, 1]) {
      // M4 x 40 from behind the beam (X~378): bore through beam side and pad,
      // blind at X416 so it does not break out of the tilted web.
      translate([396, s * pad_c[1], pad_c[2]])
        rotate([0, 90, 0]) cylinder(d=m4d, h=40, center=true, $fn=24);
      // captive M4 nut (AF7 + 0.3) at X409. The pocket is swept along the
      // print-vertical direction (camera -X) out through the pad face that
      // butts on the beam, so it has vertical walls in the print pose; the
      // nut drops in before the pad is bolted to the beam.
      hull() for (t = [0, 30])
        translate([409, s * pad_c[1], pad_c[2]])
          translate(t * [-cos(oak_pitch_deg), 0, sin(oak_pitch_deg)]) rotate([0, 90, 0])
            cylinder(d=(7 + 0.3) / cos(30), h=3.6, center=true, $fn=6);
    }
    // the camera's own envelope is never cut by the bracket
    oak_s2_envelope();
  }
}

// Hex-key (L) access for the two camera M4 from behind the plate: a clear
// D10 x 25 mm cylinder on each bolt axis must not intersect the bracket.
// (A 70 mm straight screwdriver path is NOT available: the struts converge
// to the pads at Y+-35 and cross it.)
module p299e_tool_access() {
  oak_frame() for (s = [-1, 1])
    translate([plate_x() - 25, s * oak_m4_y, -oak_m4_below_axis])
      rotate([0, 90, 0]) cylinder(d=10, h=25, $fn=24);
}

part = "assembly";
if (part == "assembly") p299e_oak_bracket();
else if (part == "qa_tool") intersection() { p299e_oak_bracket(); p299e_tool_access(); }
// print pose: plate back face on the bed, webs up
else if (part == "p299e_oak_bracket")
  rotate([0, 90, 0]) translate(-[plate_x() + plate_t, 0, 0])
    rotate([0, -oak_pitch_deg, 0]) translate(-oak_lens) p299e_oak_bracket();
