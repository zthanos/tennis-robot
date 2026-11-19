// P2-99e: OAK-D S2 envelope, mounting interface and installed pose.
// Shared by the arch bracket (p299e-oak-bracket.scad) and the body-F camera
// eye (body-panels-f.scad) so the two can never drift apart. Use with
// `include`; no top-level geometry.
//
// Source: official Luxonis enclosure STEP DM9098S2_enclosure.stp
// (https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-S2/DM9098S2_enclosure.stp,
// linked from github.com/luxonis/oak-hardware DM9098_OAK-D-S2/Mechanical;
// STEP FILE_NAME 010-00009-02_ASM 2022-06-03, HTTP Last-Modified 2023-05-11,
// sha256 bedf072b...85cdf) and datasheet OAK-D-S2_Datasheet.pdf (Dec 2021):
//   enclosure 97.1 x 29.7 x 23.4 mm (datasheet 97 x 29.5 x 22.9),
//   three lenses on one line 19.67 mm above the bottom (datasheet 19.51),
//   RGB centred, stereo at +-37.5 mm (75 mm baseline),
//   back: 2 x M4 (tap drill D3.3, datasheet "M4 depth 8") at +-37.5 mm,
//         14.87 mm above the bottom = 4.8 mm below the lens axis,
//   bottom: 1/4"-20 (minor D5.1) under one stereo lens, 6.2 mm from the back.
// Mass: NOT stated in the datasheet/docs -> to be weighed by the user.
//
// Camera-local frame: origin = RGB lens centre on the front face,
// +X = optical axis, +Y = left, +Z = up (enclosure top).

oak_lens = [508, 0, 300];      // CAD mm, approved P2-99e position B
oak_pitch_deg = 25;            // approved default, downward
oak_size = [23.4, 97.1, 29.7]; // depth (X), width (Y), height (Z)
oak_lens_above_bottom = 19.67;
oak_m4_y = 37.5;
oak_m4_below_axis = 4.8;

module oak_frame(pitch=oak_pitch_deg) {
  translate(oak_lens) rotate([0, pitch, 0]) children();
}

// Enclosure envelope, optionally grown by `pad` on every side.
module oak_s2_envelope(pad=0, pitch=oak_pitch_deg) {
  oak_frame(pitch)
    translate([-oak_size[0] - pad, -oak_size[1] / 2 - pad, -oak_lens_above_bottom - pad])
      cube([oak_size[0] + 2 * pad, oak_size[1] + 2 * pad, oak_size[2] + 2 * pad]);
}

// Union of the RGB (69 x 55 deg) and stereo (80 x 55 deg) fields of view as a
// pyramid from the lens, `len` mm long (camera eye / keep-out use).
module oak_fov(len=80, pitch=oak_pitch_deg) {
  oak_frame(pitch) hull() {
    translate([0.5, 0, 0]) cube([1, 2 * oak_m4_y + 10, 10], center=true);  // all three lenses
    for (sy = [-1, 1], sz = [-1, 1])
      translate([len, sy * len * tan(40), sz * len * tan(27.5)]) cube(1, center=true);
  }
}
