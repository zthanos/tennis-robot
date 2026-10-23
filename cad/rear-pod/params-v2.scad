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

// P2-88 purchased heat-set insert.  This is the only insert family approved
// for the pod: Ruthex RX-M3x5.7.  M4 joints remain through-bolted.
ruthex_m3_length = 5.7;
ruthex_m3_od = 4.6;
ruthex_m3_hole = 4.0;
ruthex_m3_depth = 6.7;
ruthex_m3_min_wall = 2.0;
ruthex_m3_boss_d = ruthex_m3_od + 2*ruthex_m3_min_wall;

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

// P2-90 physical measurement: 115 x 45 x 45 mm.  The printed panel posts
// leave only about 93 mm between them, so the unit cannot enter from +Y at its
// old Y42 location.  It moves to the centre corridor and slides +X through the
// removable rear_hi opening, passing beside (not between) the post paths.
// Ports face -X and the button faces +Y.  Reservation keeps 3 mm per side.
ugreen_actual = [115,45,45];
ugreen_installed_size = ugreen_actual;
ugreen_clear = [ugreen_installed_size[0]+6,
                ugreen_installed_size[1]+6,
                ugreen_installed_size[2]+6];
ugreen_center = [-209,34,258];
ugreen_service_start_x = -350;

// P2-88 field-drilled front side-frame axis.  The rear axis is an existing
// P2-79 through-hole in the printed base.  The new front Ø4.5 hole is made
// only with side_frame_base_drill_jig.stl.
side_frame_rear_x = panel_center_x-58; // -256.5
side_frame_front_x = -132;
side_frame_mount_y = 54;

// Parkside X20V packaging is provisional until a real battery/contact dock
// is measured.  The drawer remains inside the accepted P2-70 reservation.
parkside_drawer_outer = [133,67,67];
parkside_battery_provisional = [125,59,59];
parkside_drawer_floor_z = 68;
parkside_mount_x = [-245,-210];

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
assert((ugreen_center[2]-ugreen_installed_size[2]/2)-231.5 >= 3,
       "physical UGREEN needs 3 mm above the rear electronics envelope");
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
