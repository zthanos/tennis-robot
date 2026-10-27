// ANALYSIS ONLY — side-by-side flywheel launcher on a printed 20 degree
// adapter module above the standalone fixed-motor intake bridge.
// Units: mm; robot +X forward.
//
// This integration study does not redefine either authoritative component.
// It imports the fixed intake assembly unchanged and places the complete
// launcher as one rigid assembly from the raised bridge top datum.

use <../standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad>
use <launcher-envelope.scad>

$fn = 96;

bridge_top_z = 208;
launcher_side_plate_offset = 43; // wheel_width/2 + 18 in launcher params
launcher_side_plate_t = 8;
launcher_pitch_deg = 20;

// The adapter is one mounting module made as three printable 210 x 150 mm
// wedge sections. No bolt pattern is released until the bridge and cradle
// fasteners are selected.
adapter_x0 = -105;
adapter_x1 = 105;
adapter_half_y = 225;
adapter_sections = 3;
adapter_rear_h = 6;
adapter_front_h = adapter_rear_h
                  + (adapter_x1-adapter_x0)*tan(launcher_pitch_deg);

// With the launcher pitched about its nip, the underside of the lower cradle
// plate is z = x*tan(pitch) + nip - 47/cos(pitch). Align that plane exactly
// with the printed adapter's top surface.
launcher_lower_plate_offset = launcher_side_plate_offset
                              + launcher_side_plate_t/2; // 47 mm
launcher_nip_z = bridge_top_z + adapter_rear_h
                 - adapter_x0*tan(launcher_pitch_deg)
                 + launcher_lower_plate_offset/cos(launcher_pitch_deg);

show_intake = true;
show_launcher = true;
show_tilt_adapter = true;

module fixed_intake_checkpoint() {
    if (show_intake) assembly();
}

module wedge_section(y0, y1, tint) {
    z0 = bridge_top_z;
    zr = bridge_top_z + adapter_rear_h;
    zf = bridge_top_z + adapter_front_h;
    color(tint, 0.88)
        polyhedron(
            points=[
                [adapter_x0, y0, z0], [adapter_x1, y0, z0],
                [adapter_x1, y1, z0], [adapter_x0, y1, z0],
                [adapter_x0, y0, zr], [adapter_x1, y0, zf],
                [adapter_x1, y1, zf], [adapter_x0, y1, zr]
            ],
            faces=[
                [0, 3, 2, 1], [4, 5, 6, 7],
                [0, 1, 5, 4], [1, 2, 6, 5],
                [2, 3, 7, 6], [3, 0, 4, 7]
            ],
            convexity=10
        );
}

module printed_tilt_adapter() {
    if (show_tilt_adapter)
        for (section = [0:adapter_sections-1]) {
            section_w = 2*adapter_half_y/adapter_sections;
            y0 = -adapter_half_y + section*section_w;
            y1 = y0 + section_w;
            wedge_section(y0, y1,
                          section == 1
                              ? [0.48, 0.12, 0.68]
                              : [0.62, 0.22, 0.78]);
        }
}

module pitched_launcher_on_adapter() {
    if (show_launcher)
        // Side-by-side keeps the wheel pair laterally arranged. The complete
        // cradle is then pitched as one rigid assembly, while its lower plate
        // lands on the matching 20 degree printed adapter surface.
        launcher_oriented(
            orientation="side_by_side",
            nip_height=launcher_nip_z,
            launch_pitch_deg=launcher_pitch_deg
        );
}

fixed_intake_checkpoint();
printed_tilt_adapter();
pitched_launcher_on_adapter();
