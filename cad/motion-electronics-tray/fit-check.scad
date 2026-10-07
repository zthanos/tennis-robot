// Three-layer full-footprint drilling/mount-pattern check at 0.20 mm layers.
// This is not a structural part.
include <params.scad>
use <tray.scad>

fit_check_t = 0.6;

intersection() {
    electronics_tray();
    cube([tray_size[0], tray_size[1], fit_check_t]);
}
