// P2-75 rear pod v1 — drive-test-only printable parameters.
// Units mm; robot coordinates are +X front, +Y left, +Z up.

$fn = 48;

// Frozen P2-70 envelope.
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

// Two alternating pillar grids.  A shelf is supported on one grid and carries
// the pillars for the next shelf on the other; the lower joint therefore never
// competes for the same hole as the upper snap.
pillar_grid_a = [[-58,-56],[-58,56],[58,-56],[58,56]];
pillar_grid_b = [[-40,-56],[-40,56],[40,-56],[40,56]];
pillar_body_d = 16;
pillar_hole_d = 8.6;
pillar_snap_neck_d = 7.6;
pillar_snap_barb_d = 9.2;
pillar_snap_h = 4;
pillar_socket_d = 10.2;
pillar_socket_depth = 3.5;
pillar_screw_pilot_d = 2.7;
pillar_screw_engagement = 7;
coupler_d = 9.8;
coupler_head_d = 14;
coupler_head_t = 2;

// Clear pillar spans from the top of one 4 mm shelf to the underside of the
// next.  The current-equivalent profile is 38/48/53/128 and places the control
// panel at the same Z347 as P2-72.  Taller alternatives are exported for dry
// fitting; README contains the envelope sum check.
mega_spans = [38,48,58];
perf_spans = [48,58,68];
bts_spans = [53,63,73];
pi_spans = [98,113,128];

mega_span_default = 38;
perf_span_default = 48;
bts_span_default = 53;
pi_span_default = 128;

// Current-equivalent assembly elevations.
mega_z = 64;
perf_z = mega_z + plate_t + mega_span_default;
bts_z = perf_z + plate_t + perf_span_default;
pi_z = bts_z + plate_t + bts_span_default;
panel_z = pi_z + plate_t + pi_span_default;

// Accepted P2-70 future reservation envelopes.  Ghosts only in v1.
parkside_clear = [141,91,81];
pi_clear = [98,74,46];
ugreen_clear = [121,52,52];

mega_case = [110,60];
perfboard = [120,80];
bts_board = [50,50];
bts_pattern = [40,40];
bts_spacer_h = 20;

// Measured control apertures.  Both small controls contain their own LED;
// there is deliberately no separate panel LED aperture.
estop_hole = 22.5;
toggle_hole = 12.3;
arm_hole = 16.3; // R16-504BD, ordered-parts G7

assert(panel_z == 347, "default v1 stack must preserve the P2-72 panel datum");
