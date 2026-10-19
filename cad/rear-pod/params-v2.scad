// P2-76 rear pod v2 — frozen envelope and upright electronics carrier.
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
// P2-79 structural base: 8 mm nominal, locally 7 mm over the crossbar saddle
// and 6.5 mm below the Parkside reservations.  The top datum remains Z59.
base_t = 8;
base_bottom_local = plate_t-base_t;
base_saddle_bottom_local = -3; // global Z52: rear-crossbar top datum
base_rib_bottom_local = -12;   // global Z43; 41 mm with chassis 2 mm low
post_d = 23;
m4_clear = 4.5;
m3_clear = 3.4;
m3_pilot = 2.7;

rear_crossbar_x = -165;
rear_crossbar_w = 50;
rear_crossbar_top_z = 52;
corner_side_bolt_x = -135;
corner_side_bolt_y = 245;
corner_side_bolt_z = 37;

panel_center_x = -198.5;
panel_z = 347;

// Accepted P2-70 reservations — these coordinates, not the temporary v1
// Ugreen ghost at Y112, are the packaging source of truth.
parkside_clear = [141,91,81];
parkside_center_x = -198.5;
parkside_center_y = 115.5;
parkside_center_z = 98.5;

ugreen_clear = [121,52,52];
ugreen_center = [-195,42,258];

pi_clear = [98,74,46];
pi_center_x = -212;
pi_shelf_z = 64;

// Upright carrier. Rear face is X-181; plate thickness grows toward +X.
carrier_rear_x = -181;
carrier_front_x = -177;
carrier_main_z0 = 149;
carrier_step_z = 222;
carrier_top_z = 284;
carrier_y_min = -157;
carrier_y_lower_max = 91;
carrier_y_upper_max = 6;

// Rear-face solid and cable envelopes for the keyed perfboard interface.
perf_solid_projection = 10 + 1.6 + 8.5 + 36.6; // 56.7
perf_cable_bend = 20;

// V1 control bodies, preserved exactly except for the accepted R16 hole.
estop_hole = 22.5;
toggle_hole = 12.3;
arm_hole = 16.3;

assert(carrier_main_z0 - (parkside_center_z + parkside_clear[2]/2) >= 10,
       "carrier needs 10 mm above Parkside reservations");
assert((ugreen_center[2] - ugreen_clear[2]/2) - carrier_step_z >= 10,
       "carrier lower step needs 10 mm below Ugreen");
assert((ugreen_center[1] - ugreen_clear[1]/2) - carrier_y_upper_max >= 10,
       "carrier upper edge needs 10 mm beside Ugreen");
assert(297 - carrier_top_z >= 10,
       "carrier needs 10 mm below the deepest v1 control body");
assert((carrier_rear_x - perf_solid_projection - perf_cable_bend) - pod_x_rear >= 10,
       "keyed plug plus 20 mm cable bend exceeds rear envelope clearance");
assert(pod_x_front - (-135) >= 10,
       "front-mounted BTS envelope needs 10 mm to X-125");
assert(carrier_y_min - (-pod_y_half) >= 10,
       "carrier needs 10 mm to side envelope");
assert(panel_z + plate_t + 21 <= pod_absolute_top,
       "v1 E-stop top exceeds Z375");
