// Motion electronics service tray — all dimensions in mm.

$fn = 48;

// Bambu Lab P2S build volume is 256 x 256 x 256 mm. The tray intentionally
// leaves 8 mm total margin in X and much more in Y.
tray_size = [240, 180];
tray_t = 4;
tray_corner_r = 6;

// Printed-board separation above the tray. Perfboard gets extra clearance for
// the solder joints and copper reinforcement visible on its underside.
mega_standoff_h = 8;
perf_standoff_h = 10;
driver_standoff_h = 8;
standoff_d = 9;
m3_clearance_d = 3.6;

// Arduino Mega 2560 Rev3. Hole coordinates use the official Rev3 mechanical
// pattern, measured from the PCB corner at the USB/power-connector end.
mega_origin = [6, 109];
mega_size = [101.6, 53.3];
mega_holes = [
    [15.24, 2.54],
    [96.52, 2.54],
    [15.24, 50.80],
    [90.17, 50.80]
];

// 80 x 120 mm perfboard installed landscape. The user-derived corner-hole
// pattern is 73.66 x 111.76 mm in portrait orientation; rotating the board
// makes the tray pattern 111.76 x 73.66 mm.
perf_origin = [114, 84];
perf_size = [120, 80];
perf_hole_pattern = [111.76, 73.66];
perf_hole_margin = [
    (perf_size[0] - perf_hole_pattern[0]) / 2,
    (perf_size[1] - perf_hole_pattern[1]) / 2
];
perf_holes = [
    [perf_hole_margin[0], perf_hole_margin[1]],
    [perf_size[0] - perf_hole_margin[0], perf_hole_margin[1]],
    [perf_hole_margin[0], perf_size[1] - perf_hole_margin[1]],
    [perf_size[0] - perf_hole_margin[0], perf_size[1] - perf_hole_margin[1]]
];

// IBT-2 / BTS7960 envelope is confirmed as 50 x 50 mm. Vendor hole patterns
// vary, so each nominal corner gets a two-axis cross-slot instead of a fixed
// hole. Defaults accept roughly 40.5-46.5 mm hole spacing.
driver_size = [50, 50];
driver_origins = [[8, 12], [66, 12]];
driver_nominal_inset = 3.25;
driver_adjust_span = 7;

// Future power-safety hardware. These are universal tie/bolt bays, not claims
// about the final relay or fuse-holder hole patterns.
relay_bay_origin = [124, 12];
relay_bay_size = [44, 50];
fuse_bay_origin = [180, 12];
fuse_bay_size = [52, 50];

// Chassis interface: M5 clearance slots allow final positioning after dry-fit.
chassis_slot_len = 18;
chassis_slot_w = 5.5;
chassis_slots = [[20, 6], [220, 6], [20, 174], [220, 174]];

label_h = 0.6;
label_size = 4;
