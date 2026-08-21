// Collector intake v1 — physical motor measurements and design parameters.
// Units: mm.
//
// These are ruler-level measurements supplied on 2026-08-21. Keep the motor
// mount adjustable until the fit gauge has been printed and checked.

$fn = 96;

motor_length_measured = 70;
motor_body_d_measured = 30;
shaft_projection_measured = 20;
shaft_d_measured = 5;

// User description: "shaft placement approximately 15 mm". This is retained
// as a reference only; it is not used until we know exactly which two features
// the measurement connects.
shaft_mount_reference_measured = 15;

// GB37-family drawings and the archived concept used a larger gearbox
// envelope than the ruler measurement. The first print therefore tests a
// useful range instead of committing the carriage to one diameter.
motor_fit_diameters = [30.5, 32.5, 35.5, 37.5];

// D-flat depth is not yet measured. Test several clearance diameters with a
// conservative nominal 0.5 mm flat; do not use this as a final torque hub.
shaft_fit_diameters = [5.1, 5.3, 5.5];
shaft_flat_depth_provisional = 0.5;

// Active intake baseline from the validated URDF.
intake_wheel_d = 120;
intake_wheel_height = 80;
intake_gap = 56;
intake_axis_tilt_deg = 35;
carriage_outward_travel = 8;

// Chassis and printable intake structure (ground frame, robot +X forward).
chassis_front_x = 460;
chassis_half_width = 290;
chassis_plate_top_z = 52;
chassis_plate_thickness = 14;
chassis_open_half_width = 150;

// Active funnel-cheek geometry from funnel.urdf.xacro iteration 2.
cheek_length = 252.4;
cheek_height = 100;
cheek_print_thickness = 4;
cheek_origin_x = 765.5;
cheek_origin_y = 144;
cheek_origin_ground_z = 90;
cheek_yaw_deg = 28.877;  // 0.504 rad
cheek_pitch_deg = 10;

// Physical launch ramp derived from generate_curved_scoop_mesh.py.
ramp_entry_x = 540;
ramp_exit_x = 465;
ramp_entry_top_z = 1.5;  // printable court clearance at the leading edge
ramp_exit_top_z = 32;
ramp_exit_angle_deg = 35;
ramp_width = 180;
ramp_side_wall_t = 4;
ramp_side_wall_h = 20;
ramp_steps = 24;

// Hybrid support: two 20 x 20 mm aluminium square tubes carry the long
// overhanging intake. Printed parts clamp and locate them on the chassis.
support_rail_size = 20;
support_rail_y = 220;
support_rail_start_x = 350;
support_rail_end_x = 890;
support_rail_bottom_z = 58;

m4_clearance_d = 4.5;
m5_clearance_d = 5.6;
