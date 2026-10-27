// Motion electronics service tray v2 — all dimensions in mm.
// Measurements are from the user's physical parts (2026-09-06).

$fn = 48;

// Same 180 x 240 footprint seen in the physical fit check. It remains inside
// the Bambu Lab P2S 256 x 256 mm build area.
tray_size = [180, 240];
tray_t = 4;
tray_corner_r = 6;

standoff_d = 10;
m3_slot_w = 3.6;
board_adjust_span = 7;
label_h = 0.6;
label_size = 4;

// Upper-left: 80 x 120 mm perfboard, portrait. The physical board was measured
// as 75 x 115 mm hole-centre spacing with 4 mm PCB holes. Cross-slots absorb
// the ruler tolerance without forcing the board.
perf_origin = [6, 112];
perf_size = [80, 120];
perf_hole_pattern = [75, 115];
perf_standoff_h = 10;
perf_holes = [
    [(perf_size[0] - perf_hole_pattern[0]) / 2,
     (perf_size[1] - perf_hole_pattern[1]) / 2],
    [(perf_size[0] + perf_hole_pattern[0]) / 2,
     (perf_size[1] - perf_hole_pattern[1]) / 2],
    [(perf_size[0] - perf_hole_pattern[0]) / 2,
     (perf_size[1] + perf_hole_pattern[1]) / 2],
    [(perf_size[0] + perf_hole_pattern[0]) / 2,
     (perf_size[1] + perf_hole_pattern[1]) / 2]
];

// Upper-right: the Mega is mounted in its actual acrylic enclosure, not by the
// bare Arduino PCB pattern. Case size is 60 x 110 mm. Coordinates below are
// expressed from the lower-left of the installed case. The upper pair is 50 mm
// apart and 3 mm down from the case top; the middle pair is 37 mm apart and
// 65 mm down from the top. All four enclosure holes were measured at 4 mm.
mega_case_origin = [108, 120];
mega_case_size = [60, 110];
mega_case_standoff_h = 4;
mega_case_holes = [
    [(mega_case_size[0] - 50) / 2, mega_case_size[1] - 3],
    [(mega_case_size[0] + 50) / 2, mega_case_size[1] - 3],
    [(mega_case_size[0] - 37) / 2, mega_case_size[1] - 65],
    [(mega_case_size[0] + 37) / 2, mega_case_size[1] - 65]
];

// Lower-left: red L298N intake driver, measured 53 x 60 mm with a 46 x 53 mm
// rectangular mounting pattern and 4 mm PCB holes.
l298_origin = [6, 10];
l298_size = [53, 60];
l298_hole_pattern = [46, 53];
l298_standoff_h = 8;
l298_heatsink_h = 20;
l298_holes = [
    [(l298_size[0] - l298_hole_pattern[0]) / 2,
     (l298_size[1] - l298_hole_pattern[1]) / 2],
    [(l298_size[0] + l298_hole_pattern[0]) / 2,
     (l298_size[1] - l298_hole_pattern[1]) / 2],
    [(l298_size[0] - l298_hole_pattern[0]) / 2,
     (l298_size[1] + l298_hole_pattern[1]) / 2],
    [(l298_size[0] + l298_hole_pattern[0]) / 2,
     (l298_size[1] + l298_hole_pattern[1]) / 2]
];

// Lower centre/right: two measured 50 x 50 mm HW-039/BTS7960 boards. Both
// have a 40 x 40 mm mounting pattern, 4 mm PCB holes and ~20 mm height.
bts_size = [50, 50];
bts_origins = [[66, 15], [124, 15]];
bts_hole_pattern = [40, 40];
// The printed tray keeps its original low bosses. Each physical BTS receives
// four separate 20 mm spacers above them, matching the successful hole fit.
bts_standoff_h = 8;
bts_spacer_h = 20;
bts_heatsink_h = 20;
bts_holes = [
    [(bts_size[0] - bts_hole_pattern[0]) / 2,
     (bts_size[1] - bts_hole_pattern[1]) / 2],
    [(bts_size[0] + bts_hole_pattern[0]) / 2,
     (bts_size[1] - bts_hole_pattern[1]) / 2],
    [(bts_size[0] - bts_hole_pattern[0]) / 2,
     (bts_size[1] + bts_hole_pattern[1]) / 2],
    [(bts_size[0] + bts_hole_pattern[0]) / 2,
     (bts_size[1] + bts_hole_pattern[1]) / 2]
];

// M5 chassis slots. The power relay and fuse move to a separate safety carrier.
chassis_slot_len = 18;
chassis_slot_w = 5.5;
chassis_slots = [[20, 5], [160, 5], [20, 235], [160, 235]];
