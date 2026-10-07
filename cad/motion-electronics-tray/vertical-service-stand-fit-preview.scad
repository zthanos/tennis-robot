// Non-printing collision preview: existing populated tray on the vertical
// service stand, with the low-profile clip envelopes highlighted in orange.

include <params.scad>
use <vertical-service-stand.scad>
use <tray.scad>
use <vertical-service-stand-front-retainer-fit-test.scad>

stand_size_local = [210, 130];
tray_x_local = (stand_size_local[0] - tray_size[0]) / 2;
tray_back_y_local = 62;
tray_bottom_z_local = 20;

module board(size, origin, z, colour) {
    color(colour, 0.82)
        translate([origin[0], origin[1], z])
            cube([size[0], size[1], 1.6]);
}

module populated_tray_reference() {
    translate([tray_x_local, tray_back_y_local, tray_bottom_z_local])
        rotate([90, 0, 0]) {
            // Full printed geometry: base, standoffs, lower wiring guides,
            // labels, slots and component-mount bosses.
            color("SeaGreen", 0.3) electronics_tray();

            board(perf_size, perf_origin, tray_t + perf_standoff_h,
                  "SeaGreen");
            color("LightCyan", 0.45)
                translate([mega_case_origin[0], mega_case_origin[1],
                           tray_t + mega_case_standoff_h])
                    cube([mega_case_size[0], mega_case_size[1], 22]);

            board(l298_size, l298_origin, tray_t + l298_standoff_h,
                  "FireBrick");
            color("DimGray", 0.85)
                translate([l298_origin[0] + 15, l298_origin[1] + 15,
                           tray_t + l298_standoff_h + 1.6])
                    cube([23, 30, l298_heatsink_h]);

            for (origin = bts_origins) {
                board(bts_size, origin,
                      tray_t + bts_standoff_h + bts_spacer_h,
                      "SteelBlue");
                color("Silver", 0.85)
                    translate([origin[0] + 12, origin[1] + 8,
                               tray_t + bts_standoff_h + bts_spacer_h + 1.6])
                        cube([26, 34, bts_heatsink_h]);
            }
        }
}

vertical_service_stand();
populated_tray_reference();

// Current front-loaded fit-test retainer, installed on top of the 6 mm stand.
// The wall covers only the lower 10 mm of the tray and the two screw ears align
// with the existing x=68/142, y=22 centre slots.
color("DarkOrange", 0.78)
    translate([8, 38, 6]) front_retainer_fit_test();
