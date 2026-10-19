// P2-72 rear pod v0 — drive-test-only printable parameters.
// Units mm; robot coordinates are +X front, +Y left, +Z up.

$fn = 48;

pod_x_front = -125;
pod_x_rear = -272;
pod_y_half = 167;
pod_rear_chamfer = 5;
pod_z_bottom = 55;
pod_body_top = 350;
pod_absolute_top = 375;

plate_t = 4;
wall = 3;
m4_clear = 4.5;
m3_slot_w = 3.6;
board_slot_span = 8;
rod_xy = [[-258,-58],[-258,58],[-139,-58],[-139,58]];

// Accepted P2-70 future reservation envelopes. These are shown only as ghosts
// in v0; no Parkside/Ugreen/relay/DC-DC/D5065 holder is released here.
parkside_clear = [141,91,81]; // 135x85x75 CAD assumption +3 mm per side
pi_clear = [98,74,46];       // measured case 92x68x40 +3 mm per side
ugreen_clear = [121,52,52];  // maker 115x46x45.5 +3 mm per side

mega_case = [110,60];
perfboard = [120,80];
bts_board = [50,50];
bts_pattern = [40,40];
bts_spacer_h = 20;

// Shelf bottom elevations in the robot assembly.
mega_z = 64;
perf_z = 106;
bts_z = 158;
pi_z = 215;
panel_z = 347;

// User-measured control apertures. LED body/hole is deliberately not guessed:
// it receives a replaceable blank insert until the user measures it.
estop_hole = 22.5;
toggle_hole = 12.3;
arm_hole = 15.5;

