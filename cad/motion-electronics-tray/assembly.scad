include <params.scad>
use <tray.scad>

module pcb(size, z, color_name) {
    color(color_name, 0.72) translate([0, 0, z]) cube([size[0], size[1], 1.6]);
}

module mega_reference() {
    pcb(mega_size, tray_t + mega_standoff_h, "RoyalBlue");
    // USB-B connector points toward the service edge.
    color("Silver") translate([-6, 32, tray_t + mega_standoff_h + 1.6])
        cube([16, 13, 11]);
    color("DimGray") translate([-2, 5, tray_t + mega_standoff_h + 1.6])
        cube([14, 10, 11]);
}

module perfboard_reference() {
    pcb(perf_size, tray_t + perf_standoff_h, "SeaGreen");
}

module driver_reference() {
    pcb(driver_size, tray_t + driver_standoff_h, "DarkGreen");
    color("Silver", 0.85)
        translate([8, 5, tray_t + driver_standoff_h + 1.6])
            cube([34, 40, 37]);
    color("RoyalBlue")
        translate([10, -3, tray_t + driver_standoff_h + 1.6]) cube([30, 8, 11]);
}

electronics_tray();

translate(mega_origin) mega_reference();
translate(perf_origin) perfboard_reference();
for (origin = driver_origins)
    translate(origin) driver_reference();

// Reference-only future hardware envelopes.
color("Orange", 0.25)
    translate([relay_bay_origin[0], relay_bay_origin[1], tray_t + 0.1])
        cube([relay_bay_size[0], relay_bay_size[1], 28]);
color("Gold", 0.22)
    translate([fuse_bay_origin[0], fuse_bay_origin[1], tray_t + 0.1])
        cube([fuse_bay_size[0], fuse_bay_size[1], 22]);
