include <params.scad>
use <tray.scad>

module board(size, origin, z, colour) {
    color(colour, 0.75)
        translate([origin[0], origin[1], z]) cube([size[0], size[1], 1.6]);
}

electronics_tray();

board(perf_size, perf_origin, tray_t + perf_standoff_h, "SeaGreen");

// Transparent acrylic Mega enclosure envelope.
color("LightCyan", 0.28)
    translate([mega_case_origin[0], mega_case_origin[1],
               tray_t + mega_case_standoff_h])
        cube([mega_case_size[0], mega_case_size[1], 22]);

board(l298_size, l298_origin, tray_t + l298_standoff_h, "FireBrick");
color("DimGray", 0.85)
    translate([l298_origin[0] + 15, l298_origin[1] + 15,
               tray_t + l298_standoff_h + 1.6])
        cube([23, 30, l298_heatsink_h]);

for (origin = bts_origins) {
    // Four separate 20 mm spacers sit on the tray's existing low bosses.
    for (p = bts_holes)
        color("DarkSlateGray", 0.85)
            translate([origin[0] + p[0], origin[1] + p[1],
                       tray_t + bts_standoff_h])
                difference() {
                    cylinder(d=10, h=bts_spacer_h);
                    translate([0, 0, -1]) cylinder(d=3.4, h=bts_spacer_h+2);
                }

    board(bts_size, origin,
          tray_t + bts_standoff_h + bts_spacer_h, "SteelBlue");
    color("Silver", 0.8)
        translate([origin[0] + 12, origin[1] + 8,
                   tray_t + bts_standoff_h + bts_spacer_h + 1.6])
            cube([26, 34, bts_heatsink_h]);
}
