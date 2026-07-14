// Basket bin v2.1 + entry hood — validated dimensions.
//
// Single source of truth for every file in this directory.
// All values in mm, GROUND frame (z=0 at court surface), robot +x forward.
// Sources: docs/basket-bin-redesign-spec-el.md,
//          ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro,
//          debug log #45-#56 (docs/intake-debug-log-el.md).
// Do not change a value here without re-validating in the Gazebo bench.

// ---- Bin interior (spec §2) ----
bin_rear_x       = 20;    // rear interior plane
bin_front_x      = 420;   // front interior plane = retention boundary
bin_half_width   = 140;   // interior half width (280 total)
floor_top_z      = 25;    // sunken floor top (20 mm ground clearance below)
floor_thickness  = 5;
wall_top_z       = 250;   // rim height (below lidar plane 498)
wall_thickness   = 10;    // effective wall envelope in sim

// ---- Load-management tray (spec §2, log #54 low_transition) ----
mgmt_run         = 140;   // rear edge x = bin_front_x - mgmt_run = 280
mgmt_rise        = 10;    // tray top: 25 @ x280 -> 35 @ x420 (~4.1 deg)

// ---- Receiving chute (spec §2) ----
recv_run         = 50;    // x 420 -> 470
recv_rise        = 5;     // top: 35 @ x420 -> 40 @ x470
recv_half_width  = 90;    // 180 mm launch channel

// ---- Front retention (log #51/#52) ----
entry_half_width = 90;    // launch opening half width
guard_height     = 20;    // corner guards, z 25..45, y 90..140 each side
center_lip_height = 10;   // fixed centre lip, z 25..35 (log #52 -> low-transition)

// ---- Entry hood (log #54-#56, now sim default) ----
hood_rear_overhang    = 40;   // roof rear x = bin_front_x - 40 = 380
hood_rear_clearance_z = 120;  // roof underside @ rear edge
hood_front_clearance_z = 135; // roof underside @ front edge (x 470)
hood_cheek_thickness  = 5;

// ---- Wire mesh construction (spec §2: hopper-style mesh) ----
wire_d     = 4;    // mesh wire
frame_d    = 6;    // panel perimeter rod
mesh_pitch = 40;   // grid opening <= 40 (ball 66 cannot pass)

// ---- Chassis context (reference only, spec §3-§4) ----
plate_top_z     = 52;
plate_thickness = 14;
plate_half_len  = 460;   // 920 x 580 chassis
plate_half_wid  = 290;
open_rear_x     = 10;    // plate opening x 10..460, y +/-150
open_front_x    = 460;
open_half_wid   = 150;
flange_width    = 22;    // bin rests on this ring, on the plate top
flange_thickness = 4;

// ---- Sensors / references ----
ir_x       = 445;   // confirmation beam through the chute midpoint
ir_z       = 70.5;
ir_mount_y = 155;   // sensor bodies on the chassis, outside the bin
ball_d     = 66;

// Battery bay (spec §4): 198 x 166 x 170, x -226..-60, centered in y
batt_size  = [166, 198, 170];
batt_min_x = -226;

// Derived (do not edit)
bin_length   = bin_front_x - bin_rear_x;           // 400
wall_height  = wall_top_z - floor_top_z;           // 225
mgmt_rear_x  = bin_front_x - mgmt_run;             // 280
mgmt_angle   = atan2(mgmt_rise, mgmt_run);         // ~4.09 deg
recv_front_x = bin_front_x + recv_run;             // 470
recv_angle   = atan2(recv_rise, recv_run);         // ~5.71 deg
recv_rear_top_z = floor_top_z + mgmt_rise;         // 35
recv_mid_z   = recv_rear_top_z + recv_rise / 2;    // 37.5
hood_rear_x  = bin_front_x - hood_rear_overhang;   // 380
hood_run     = recv_front_x - hood_rear_x;         // 90
hood_rise    = hood_front_clearance_z - hood_rear_clearance_z; // 15
hood_angle   = atan2(hood_rise, hood_run);         // ~9.46 deg
// roof underside height at the cheek midpoint (x = ir_x), xacro hood_side_top_z
hood_side_top_z = hood_rear_clearance_z
    + hood_rise * (ir_x - hood_rear_x) / hood_run; // ~130.8
