// PHASE-2 PRE-XACRO PACKAGING STUDY -- not a manufacturing release.
// Shared front flywheels for intake and launch. Units mm.
// Global: +X front, +Y left, +Z up. Ground is Z=0.
// Angle convention: positive theta raises the front/local +U mouth.
// The whole module rotates about global +Y; local W is the wheel shaft axis.

$fn = 48;

view = "intake";
show_ball_paths = true;
show_swept_envelope = false;
show_sensor_fields = false;
qa_angle = -10;
clearance_probe = 0; // mm; expands fixed drive envelopes for clearance search
cheek_variant = "narrow"; // [narrow,wide]
p244_cheek_variant = "exposed_shallow_lowfriction"; // [exposed_short,exposed_shallow,exposed_shallow_lowfriction]
p245_ramp_tip_clearance = 8; // mm underside; P2-45 selected reference
p245_ramp_delta = p245_ramp_tip_clearance-1;
p244_ramp_points = [[402,0,p245_ramp_tip_clearance+2],
                    [377,0,8+0.75*p245_ramp_delta],
                    [352,0,21+0.35*p245_ramp_delta],[328,0,36],
                    [315,0,46],[305,0,52],[298,0,54],[278,0,54]];
p244_ramp_widths = [56,66,78,86,90,93,95,96];

// P2-40 dependent-only packaging candidate; NOT an STL/manufacturing release.
// Stop adjustment is explicitly widened to -10..-3 deg.
theta_intake = -3;
theta_transport = 0;
theta_launch = 20; // P2-40: collision-free with X318 side-leg dogleg
theta_launch_scan_max = 20;
flywheel_variant = "d200"; // [d200,d160]
wheel_d = flywheel_variant == "d160" ? 160 : 200;
wheel_width = flywheel_variant == "d160" ? 44 : 60;
tread_edge_round = 2; // analytical usable-edge reserve; profile not rendered
tread_profile_status = "P2-40_PRINTED_FLAT60_R2_XACRO_VALIDATED_DEPENDENT_CAD";
wheel_y = (wheel_d+58)/2;
ball_d = 66;

// Keep the nip X datum fixed while sweeping the pivot 45..110 mm behind it.
nip_x_transport = 442;
pivot_nip_offset = 55; // accepted P2-39/P2-40 CAD point
nip_u = pivot_nip_offset;
module_u_shift = pivot_nip_offset-45;
pivot_nominal = [nip_x_transport-pivot_nip_offset, 0, 64.30];
pivot_adjust_travel = 23;
// At R85/h29 the settled chassis rises 4 mm.  Set the physical lower tread
// edge to Z=23 mm: ball centre 33 - r2 - usable margin 8.
target_physical_lower_edge = 33-tread_edge_round-8;
pivot_ground_target = target_physical_lower_edge
    - nip_u*sin(theta_intake) + wheel_width/2*cos(theta_intake);
pivot_adjust_down = pivot_nominal[2]-(pivot_ground_target-4);
pivot = [pivot_nominal[0], pivot_nominal[1], pivot_nominal[2]-pivot_adjust_down];

upper_plate_u0 = -25;
upper_plate_u1 = 160+module_u_shift;
plate_y = flywheel_variant == "d160" ? 274 : 314;
plate_t = 8;
upper_w = 43;
yoke_y = 234;
yoke_u0 = -25;
yoke_u1 = 15;

motor_d = 50;
motor_length = 65;
shaft_d = 8;
shaft_projection = 30;
mount_pcd = 30;
mount_hole_d = 4;

power_variant = "parkside_pair"; // [parkside_pair,redodo]
redodo_size = [195, 166, 172];
redodo_center = [-55, 0, 138];
// UNMEASURED placeholder only: replace with the exact purchased Parkside pack
// plus adapter/handle sweep before releasing quick-change dock STLs.
parkside_pack_size = [135, 85, 75];
parkside_pack_centers = [[-165,-110,112],[-165,110,112]];
ugreen_size = [115,46,45.5]; // manufacturer dimensions, oriented along X
ugreen_center = [-100,0,145];
parkside_current_limit_a_each = undef; // mandatory measured parameter
basket_x0 = -15;
basket_x1 = 278;
basket_y = 350;
basket_floor_rear = 42;
basket_floor_front = 54;
basket_top = 215;
basket_upper_x1 = 205;
structural_crossbar_x = 240;
frame_front_x = 330;
drive_mount_x = [-65,300];
drive_mount_y = 220;
drive_axle_y = 297;
drive_axle_z = 85;
drive_tire_width = 83;
trunnion_d = 24;
bearing_housing_d = 40;
bearing_guard_y = 243;
arch_top_z = 245;
// P2-40: retain the LiDAR/crossbar datum at X330, but dogleg the side legs
// rearward.  The 12 mm setback supplies at least 11 mm directional
// moving-module clearance in the exact-solid sweep:
// 3 mm tolerance/elastic budget plus 8 mm residual target.
arch_crossbar_x = 330;
arch_leg_x = 318;
arch_tolerance_elastic_budget = 3;
arch_required_residual_clearance = 8;
arch_required_nominal_clearance = arch_tolerance_elastic_budget + arch_required_residual_clearance;
lidar_scan_z = 390; // lowered option; keeps >10 mm body clearance above camera
actuator_fixed = [320,-190,120];
actuator_moving_local = [50,-190,90];
camera_center = [350,0,305];
cheek_entry_y = cheek_variant == "wide" ? 110 : 72;
cheek_throat_y = 46;
guide_centers = [[329,0,78],[305,0,82],[281,0,94],[257,0,111],
                 [233,0,128],[209,0,136],[187,0,132],[163,0,132],[132,0,132]];
function guide_floor(p) = [p[0],p[1],p[2]-ball_d/2];

function gpoint(theta, p) = [
    pivot[0] + cos(theta)*p[0] - sin(theta)*p[2],
    p[1],
    pivot[2] + sin(theta)*p[0] + cos(theta)*p[2]
];
function floor_z(x) = basket_floor_rear +
    (x-basket_x0)*(basket_floor_front-basket_floor_rear)/(basket_x1-basket_x0);

echo("PHASE2_DEPENDENT_CAD_XACRO_VALIDATED_NO_STL_RELEASE", pivot_mm=pivot, angle_convention="positive raises front mouth",
     theta_intake_deg=theta_intake, theta_launch_deg=theta_launch,
     theta_transport_deg=theta_transport, pivot_nip_offset_mm=pivot_nip_offset,
     pivot_nominal_mm=pivot_nominal, pivot_adjust_down_mm=pivot_adjust_down,
     pivot_adjust_travel_mm=pivot_adjust_travel, tread_profile_status=tread_profile_status,
     wheel_width_mm=wheel_width, tread_edge_round_mm=tread_edge_round,
     arch_leg_x_mm=arch_leg_x, arch_crossbar_x_mm=arch_crossbar_x,
     arch_nominal_clearance_requirement_mm=arch_required_nominal_clearance,
     nip_intake_mm=gpoint(theta_intake,[nip_u,0,0]),
     nip_launch_mm=gpoint(theta_launch,[nip_u,0,0]),
     rear_throat_intake_mm=gpoint(theta_intake,[nip_u-105,0,0]),
     rear_throat_launch_mm=gpoint(theta_launch,[nip_u-105,0,0]),
     power_variant=power_variant, redodo_center_mm=redodo_center,
     redodo_size_mm=redodo_size, parkside_pack_centers_mm=parkside_pack_centers,
     parkside_pack_size_assumed_mm=parkside_pack_size,
     parkside_current_limit_a_each=parkside_current_limit_a_each,
     outboard_trunnion_y_mm=yoke_y, cheek_variant=cheek_variant,
     flywheel_variant=flywheel_variant, drive_axle_y_mm=drive_axle_y,
     overall_width_mm=2*(drive_axle_y+drive_tire_width/2),
     drive_envelope_basis="user tape measurements +/-2..3 mm: tyre width 83, axle Y about 295..298, FIT0403 face-to-encoder 77..78, hub end 47..49; radial cable exit routed downward");

module hollow_box_x(length, width=50, height=30, wall=3.2) {
    difference() {
        cube([length,width,height],center=true);
        cube([length+2,width-2*wall,height-2*wall],center=true);
    }
}
module hollow_box_y(length, width=50, height=30, wall=3.2) {
    rotate([0,0,90]) hollow_box_x(length,width,height,wall);
}

module chassis_structure() {
    // Alternative-only frame context. The old Gamma/intake bridge and ramp are absent.
    // The old X=294.4 member collides with the rotating wheels. Its structural
    // function is retained by a relocated 50x20 crossbar at X=240.
    color([0.16,0.48,0.78,0.92]) {
        for (side=[-1,1])
            translate([(-165+frame_front_x)/2,side*220,37])
                hollow_box_x(frame_front_x-(-165));
        translate([-165,0,37]) hollow_box_y(440);
        translate([structural_crossbar_x,0,32]) hollow_box_y(440,50,20);
        for (side=[-1,1])
            hull() {
                translate([structural_crossbar_x-32,side*220,37]) cube([6,6,20],center=true);
                translate([structural_crossbar_x,side*160,32]) cube([6,6,20],center=true);
            }
    }
    low_torsion_deck();
}

module low_torsion_deck() {
    // Low crossed torsion deck closes the open X240..X300 drive bay while
    // retaining dirt-shedding openings below the high intake guide.
    color([0.16,0.48,0.78,0.92]) {
        translate([300,0,32]) hollow_box_y(440,20,20);
        for (side=[-1,1]) hull() {
            translate([240,side*150,47]) cube([12,12,10],center=true);
            translate([300,-side*150,47]) cube([12,12,10],center=true);
        }
    }
}

module drive_tire_envelopes() {
    color([0.05,0.06,0.07])
    for (side=[-1,1],xx=drive_mount_x)
        translate([xx,side*drive_axle_y,drive_axle_z]) rotate([90,0,0])
            difference() { cylinder(d=170,h=drive_tire_width,center=true); cylinder(d=36,h=drive_tire_width+2,center=true); }
}

module drive_package_envelopes() {
    drive_tire_envelopes();
    drive_hardware_envelopes();
}

module shared_base_frame() {
    chassis_structure();
    drive_package_envelopes();
}

module drive_hardware_envelopes() {
    // Positive collision geometry. MEASURED/FIT-CHECKED: 40x40 foot, 25x30
    // hole rectangle; bracket height is only an approximate 47 mm measurement.
    // MEASURED BY TAPE (+/-2..3 mm): tyre width/track, face-to-encoder stack,
    // and outer hex stack. Detailed diameters, shaft/hub lengths and cable bend
    // envelope remain assumptions; verify all four assemblies before release.
    for (side=[-1,1],xx=drive_mount_x) {
        color([0.72,0.74,0.78]) {
            translate([xx,side*drive_mount_y,53]) cube([40,40,2],center=true);
            translate([xx,side*239,77.5]) cube([40,2,47],center=true);
        }
        // User-measured face-to-encoder length is 77..78 mm inward.
        color([0.28,0.30,0.33])
            translate([xx,side*226.5,drive_axle_z]) rotate([90,0,0])
                cylinder(d=40,h=25,center=true);
        color([0.13,0.14,0.16])
            translate([xx,side*193,drive_axle_z]) rotate([90,0,0])
                cylinder(d=38,h=42,center=true);
        color([0.18,0.20,0.23])
            translate([xx,side*166,drive_axle_z]) rotate([90,0,0])
                cylinder(d=40,h=12,center=true);
        // Measured topology: radial exit near encoder end, then downward and
        // below the rail. There is no axial connector beyond the encoder.
        color([0.92,0.72,0.12])
            translate([xx,side*166,61]) cube([18,14,18],center=true);
        color([0.92,0.72,0.12]) hull() {
            translate([xx,side*166,53]) cube([8,8,8],center=true);
            translate([xx,side*220,16]) cube([8,8,8],center=true);
        }
        // User-measured outer stack ends 47..49 mm beyond the vertical plate.
        color("silver")
            translate([xx,side*264,drive_axle_z]) rotate([90,0,0])
                cylinder(d=6,h=48,center=true);
        color([0.15,0.70,0.85])
            translate([xx,side*280,drive_axle_z]) rotate([90,0,0])
                cylinder(d=30,h=24,center=true,$fn=6);
    }
}

module expanded_drive_hardware(clearance=0) {
    if (clearance <= 0) drive_package_envelopes();
    else minkowski() {
        drive_package_envelopes();
        sphere(r=clearance,$fn=16);
    }
}

module module_pose(theta) {
    translate(pivot) rotate([0,-theta,0]) children();
}

module upper_plate() {
    color([0.52,0.55,0.60]) difference() {
        translate([(upper_plate_u0+upper_plate_u1)/2,0,upper_w])
            cube([upper_plate_u1-upper_plate_u0,plate_y,plate_t],center=true);
        // Central rear service/ball-path relief; leaves both motor lands intact.
        translate([-18+module_u_shift,0,upper_w]) cube([16,92,plate_t+2],center=true);
        for (yy=[-wheel_y,wheel_y]) {
            translate([nip_u,yy,upper_w]) cylinder(d=18,h=plate_t+2,center=true);
            for (a=[0:90:270])
                translate([nip_u+mount_pcd/2*cos(a),yy+mount_pcd/2*sin(a),upper_w])
                    cylinder(d=mount_hole_d,h=plate_t+2,center=true);
        }
    }
}

module wheel_and_motor(yy) {
    color([0.95,0.42,0.06])
        translate([nip_u,yy,0]) cylinder(d=wheel_d,h=wheel_width,center=true);
    color([0.08,0.09,0.11])
        translate([nip_u,yy,47+motor_length/2]) cylinder(d=motor_d,h=motor_length,center=true);
    color("silver")
        translate([nip_u,yy,47-shaft_projection/2]) cylinder(d=shaft_d,h=shaft_projection,center=true);
    // Cyan is a controlled metal clamp-hub envelope, not a released part.
    color([0.15,0.70,0.85])
        translate([nip_u,yy,30]) difference() {
            cylinder(d=46,h=18,center=true); cylinder(d=8.2,h=20,center=true);
        }
}

module d5065_airflow_reservation(yy) {
    // Reserved keep-out, not a printed duct.  The 8 mm radial annulus and
    // 15 mm axial outlet are design allowances pending thermal measurement.
    // No continuous-torque credit above the guide's 30% unventilated value is
    // allowed unless this path has measured cross/forced flow and temperature.
    color([0.20,0.80,0.95,0.18])
        translate([nip_u,yy,47+motor_length/2])
            cylinder(d=motor_d+16,h=motor_length+30,center=true);
}

module trunnions_and_standoffs() {
    color([0.66,0.38,0.72]) {
        for (side=[-1,1]) {
            // Over-wheel flange remains at W=+43, 17 mm above the wheel face,
            // then drops at the forward/elevated trunnion, clear of the
            // shortened rail in X (the trunnion edge reaches |Y|=247 mm).
            translate([(yoke_u0+yoke_u1)/2,
                       side*(plate_y/2+(yoke_y-plate_y/2)/2),upper_w])
                cube([yoke_u1-yoke_u0,yoke_y-plate_y/2,plate_t],center=true);
            translate([(yoke_u0+yoke_u1)/2,side*yoke_y,upper_w/2])
                cube([yoke_u1-yoke_u0,plate_t,upper_w],center=true);
            // Compact outboard trunnions: housing outer face is |Y|=240 mm;
            // measured tyre inner face is |Y|=255.5 mm (15.5 mm nominal).
            translate([0,side*yoke_y,0]) rotate([90,0,0])
                cylinder(d=trunnion_d,h=12,center=true);
        }
    }
}

module wheel_pair() {
    for (yy=[-wheel_y,wheel_y])
        translate([nip_u,yy,0]) cylinder(d=wheel_d,h=wheel_width,center=true);
}

module yoke_structure() {
    upper_plate(); trunnions_and_standoffs();
}

module moving_throat_and_guide() {
    // The guide is behind the nip only; nothing projects ahead of the first wheel contact.
    color([0.24,0.68,0.36,0.75]) {
        for (side=[-1,1]) hull() {
            translate([-34+module_u_shift,side*44,-13]) cube([5,5,5],center=true);
            translate([-78+module_u_shift,side*43,-22]) cube([5,5,5],center=true);
        }
        translate([-77+module_u_shift,0,-24]) cube([12,88,4],center=true);
    }
    // Gate blade is closed during rotation; actuator and detailed bearings remain unresolved.
    color([0.88,0.14,0.12,0.80]) translate([-61+module_u_shift,0,0]) cube([5,82,72],center=true);
}

module side_cheeks() {
    // Vertical lateral funnels only. They do not create a front ground-contacting lip.
    color([0.20,0.62,0.78,0.45]) for (side=[-1,1]) hull() {
        translate([155+module_u_shift,side*cheek_entry_y,15]) cube([10,4,54],center=true);
        translate([78+module_u_shift,side*cheek_throat_y,12]) cube([10,4,60],center=true);
    }
}

module p244_side_cheeks() {
    front_x = p244_cheek_variant == "exposed_short" ? 180 : 220;
    color([0.20,0.62,0.78,0.55]) for (side=[-1,1]) hull() {
        // Both candidates end at X=+105 mm relative to the nip, outside the
        // Ø200 tyre envelope.  The front-inner tyre quadrant is unobstructed.
        translate([nip_u+front_x,side*125,10]) cube([4,4,52],center=true);
        translate([nip_u+105,side*80,10]) cube([4,4,52],center=true);
    }
}

module p244_fixed_open_ramp() {
    color([0.24,0.68,0.36,0.80]) {
        for(i=[0:len(p244_ramp_points)-2]) hull() {
            translate(p244_ramp_points[i]-[0,0,1]) cube([4,p244_ramp_widths[i],2],center=true);
            translate(p244_ramp_points[i+1]-[0,0,1]) cube([4,p244_ramp_widths[i+1],2],center=true);
        }
        // Low lateral rails start only after the tyre zone; the top remains open.
        for(side=[-1,1],i=[2:len(p244_ramp_points)-2]) hull() {
            translate(p244_ramp_points[i]+[0,side*(p244_ramp_widths[i]/2+1.5),10]) cube([4,3,20],center=true);
            translate(p244_ramp_points[i+1]+[0,side*(p244_ramp_widths[i+1]/2+1.5),10]) cube([4,3,20],center=true);
        }
    }
    // Short compliant anti-rebound flap; not a closed roof.
    color([0.92,0.50,0.08,0.75]) translate([296,0,128]) rotate([0,11.46,0]) cube([40,100,2],center=true);
}

module p244_moving_hard(theta) {
    module_pose(theta) {
        upper_plate();
        for(yy=[-wheel_y,wheel_y]) wheel_and_motor(yy);
        trunnions_and_standoffs(); p244_side_cheeks();
        actuator_moving_bracket(); cable_service_point();
    }
}

module moving_module(theta) {
    module_pose(theta) {
        upper_plate();
        for (yy=[-wheel_y,wheel_y]) {
            wheel_and_motor(yy);
            %d5065_airflow_reservation(yy);
        }
        trunnions_and_standoffs(); moving_throat_and_guide(); side_cheeks();
        actuator_moving_bracket(); cable_service_point();
    }
}

module moving_hard_clearance(theta) {
    // Excludes only the fixed bearing block and dock/guide interfaces.
    module_pose(theta) {
        upper_plate();
        for (yy=[-wheel_y,wheel_y]) wheel_and_motor(yy);
        trunnions_and_standoffs(); side_cheeks(); actuator_moving_bracket();
    }
}

module moving_frame_drive_clearance(theta) {
    module_pose(theta) {
        upper_plate();
        for (yy=[-wheel_y,wheel_y]) wheel_and_motor(yy);
        trunnions_and_standoffs(); moving_throat_and_guide(); side_cheeks();
    }
}

module vertical_slot_y(x,y,z0,z1,d=6.6,depth=16) {
    hull() for(z=[z0,z1])
        translate([x,y,z]) rotate([90,0,0]) cylinder(d=d,h=depth,center=true);
}

module pivot_bearing_running_envelopes() {
    // Solid running/guard envelopes only; excludes the intentional carrier
    // interface so pivot-X sweeps can test tyres, beams, arch, basket/guide.
    for(side=[-1,1]) {
        translate([pivot[0],side*yoke_y,pivot[2]]) rotate([90,0,0])
            cylinder(d=bearing_housing_d,h=12,center=true);
        translate([pivot[0],side*bearing_guard_y,pivot[2]]) rotate([90,0,0])
            cylinder(d=48,h=3,center=true);
    }
}

module pivot_supports() {
    color([0.55,0.34,0.68]) for (side=[-1,1]) {
        // Fixed plate reserves the full 23 mm downward travel.  The two M6
        // slots position the cassette; the measured shim stack above it is
        // the positive vertical reaction stop, so load is not carried by
        // friction in smooth slots.  Dependent CAD only: bolt edge distances,
        // layer direction and local FEA remain open before STL release.
        difference() {
            translate([pivot_nominal[0],side*220,pivot_nominal[2]-pivot_adjust_travel/2])
                cube([64,12,pivot_adjust_travel+42],center=true);
            for(dx=[-20,20])
                vertical_slot_y(pivot_nominal[0]+dx,side*220,
                    pivot_nominal[2]-pivot_adjust_travel,pivot_nominal[2],6.6,16);
        }
        // Moving bearing cassette and replaceable 1/2/5/10 mm compression
        // shims under the fixed upper datum.
        translate([pivot[0],side*227,pivot[2]]) cube([58,10,46],center=true);
        if(pivot_adjust_down>0)
            color([0.95,0.74,0.10])
                translate([pivot[0],side*227,pivot[2]+23+pivot_adjust_down/2])
                    cube([26,10,pivot_adjust_down],center=true);
        // Forward, elevated bearing arm: stays inboard of the measured drive
        // tyre inner face and starts at the shortened rail end.
        hull() {
            translate([325,side*220,52]) cube([20,30,8],center=true);
            translate([pivot[0],side*yoke_y,pivot[2]]) rotate([90,0,0]) cylinder(d=bearing_housing_d,h=12,center=true);
        }
        translate([pivot[0],side*yoke_y,pivot[2]]) rotate([90,0,0])
            difference() { cylinder(d=bearing_housing_d,h=12,center=true); cylinder(d=24.5,h=14,center=true); }
        // Replaceable sacrificial guard remains mandatory until caliper and
        // dynamic runout measurements validate the nominal clearance.
        color([0.95,0.74,0.10]) translate([pivot[0],side*bearing_guard_y,pivot[2]])
            rotate([90,0,0]) difference() {
                cylinder(d=48,h=3,center=true); cylinder(d=31,h=5,center=true);
            }
    }
}

module front_arch_and_triangulation() {
    color([0.15,0.48,0.78,0.92]) {
        for(side=[-1,1]) {
            translate([arch_leg_x,side*220,(52+arch_top_z)/2])
                rotate([0,-90,0]) hollow_box_x(arch_top_z-52,50,30);
            // Both braces remain behind the swept Ø200 rim.  The short top
            // dogleg returns to X330 only above the moving-module envelope.
            // The arch carries the actuator; bearings remain on the compact
            // local carriers so no fixed member crosses the wheel arc.
            hull() {
                translate([240,side*220,52]) cube([20,30,20],center=true);
                translate([arch_leg_x,side*220,arch_top_z]) cube([30,30,20],center=true);
            }
            hull() {
                translate([300,side*220,52]) cube([20,30,20],center=true);
                translate([arch_leg_x,side*220,arch_top_z]) cube([30,30,20],center=true);
            }
            hull() {
                translate([arch_leg_x,side*220,arch_top_z]) cube([30,30,20],center=true);
                translate([arch_crossbar_x,side*220,arch_top_z]) cube([30,30,20],center=true);
            }
        }
        translate([arch_crossbar_x,0,arch_top_z]) hollow_box_y(440,50,30);
    }
}

module actuator_moving_bracket() {
    color([0.70,0.35,0.70]) hull() {
        translate([10,-190,45]) cube([12,20,12],center=true);
        translate(actuator_moving_local) cube([18,20,18],center=true);
    }
}

module actuator_between(theta) {
    moving = gpoint(theta,actuator_moving_local);
    // Actuonix P16 envelope plus clevis allowance.  P2-39 selected geometry
    // gives 123.1 mm at -3 deg, 119.0 mm at transport 0 deg and 89.7 mm at
    // +20 deg: 33.4 mm travel.  Absolute installed length/brackets remain
    // drawing-gated even though the prior 40 mm stroke screen is satisfied.
    color([0.78,0.79,0.82]) hull() {
        translate(actuator_fixed) sphere(d=18);
        translate(moving) sphere(d=18);
    }
    color([0.15,0.16,0.18]) {
        translate(actuator_fixed) sphere(d=10);
        translate(moving) sphere(d=10);
    }
}

module fixed_stops_and_sensors() {
    // Replaceable seating stops at both commanded positions.  P2-39 widens
    // the indexed lower-stop rail to -10..-3 deg and selects -3 deg for the
    // current operating-position candidate.
    // A keyed stop block bears on a socket; the bolts clamp it but do not
    // define the reaction position by friction alone.
    lower_stop_min=-10;
    lower_stop_max=-3;
    color([0.58,0.34,0.10]) difference() {
        hull() for(a=[lower_stop_min,lower_stop_max]) {
            p=gpoint(a,[68,-205,0]); translate(p) cube([28,30,18],center=true);
        }
        for(a=[lower_stop_min:1:lower_stop_max]) {
            p=gpoint(a,[68,-205,0]); translate(p) cylinder(d=5.2,h=30,center=true);
        }
    }
    color([0.95,0.62,0.12]) {
        p=gpoint(theta_intake,[68,-205,0]); translate(p) cube([20,24,20],center=true);
    }
    // At +20 deg the
    // P16 keeps a small compressive preload so launch reaction goes directly
    // from the moving yoke into the arch-mounted stop, not into the screw.
    color([0.82,0.45,0.12]) {
        p=gpoint(theta_launch,[68,-205,0]); translate(p) cube([20,24,20],center=true);
    }
    // Independent energy-absorbing emergency stops remain beyond travel.
    color([0.65,0.08,0.06]) for(a=[-12,22]) {
        p=gpoint(a,[76,-220,0]);
        translate(p) cube([20,18,20],center=true);
    }
    color([0.10,0.75,0.95]) for(a=[theta_intake,theta_launch]) {
        p=gpoint(a,[82,-198,0]);
        translate(p) cube([14,10,8],center=true); // independent Hall/limit sensor
    }
}

module cable_service_point() {
    color([0.95,0.72,0.10]) translate([40,-170,112]) cube([18,16,16],center=true);
}

module cable_route(theta) {
    p0=gpoint(theta,[40,-170,112]);
    color([0.95,0.72,0.10]) {
        hull() { translate(p0) sphere(d=12); translate([365,-185,215]) sphere(d=12); }
        hull() { translate([365,-185,215]) sphere(d=12); translate([300,-185,205]) sphere(d=12); }
    }
}

module sensors_and_mounts() {
    // Lower scan-plane alternative: enough to clear battery, basket, arch,
    // flywheel and camera, while shortening the vibration-sensitive mast.
    mast_crossbar_z=lidar_scan_z-36;
    color([0.20,0.50,0.76]) for(side=[-1,1])
        translate([330,side*150,(arch_top_z+mast_crossbar_z)/2]) cube([25,25,mast_crossbar_z-arch_top_z],center=true);
    color([0.20,0.50,0.76]) translate([330,0,mast_crossbar_z]) cube([320,30,20],center=true);
    color([0.08,0.09,0.11]) translate([330,0,lidar_scan_z-16]) cylinder(d=76,h=32,center=true);
    if(show_sensor_fields)
        %color([1,0.15,0.15,0.25]) translate([330,0,lidar_scan_z]) cube([900,900,1],center=true);
    color([0.08,0.09,0.11]) translate(camera_center) rotate([0,50,0]) cube([82,32,28],center=true);
}

module camera_sight_envelope() {
    // Packaging rays only: H=69 deg from the repo; V=54 deg is provisional.
    if(show_sensor_fields)
        %color([0.10,0.75,0.95,0.20]) hull() {
            translate(camera_center) sphere(d=8);
            translate([545,-120,33]) sphere(d=8);
            translate([545,120,33]) sphere(d=8);
            translate([430,-70,33]) sphere(d=8);
            translate([430,70,33]) sphere(d=8);
        }
}

module d5065_controller_envelopes() {
    // Reserved packaging only; controller choice and wiring remain out of scope.
    color([0.58,0.30,0.78,0.45]) for(side=[-1,1])
        translate([80,side*145,245]) cube([110,58,35],center=true);
}

module rear_electronics_envelope() {
    // Narrow centre bay clears the two rear Parkside cassettes.  With Redodo
    // selected it remains behind the upright battery.
    color([0.30,0.32,0.36,0.45]) translate([-185,0,140]) cube([30,120,120],center=true);
}

module electronics_controller_envelopes() {
    d5065_controller_envelopes(); rear_electronics_envelope();
}

module battery() {
    if(power_variant=="redodo")
        color([0.15,0.34,0.74,0.60]) translate(redodo_center) cube(redodo_size,center=true);
    else {
        for(p=parkside_pack_centers)
            color([0.12,0.58,0.22,0.72]) translate(p) cube(parkside_pack_size,center=true);
        color([0.12,0.18,0.24,0.72]) translate(ugreen_center) cube(ugreen_size,center=true);
    }
}
module battery_tray() {
    if(power_variant=="redodo") color([0.45,0.47,0.50]) {
        for (xx=[redodo_center[0]-72,redodo_center[0]+72])
            translate([xx,0,49]) cube([22,190,6],center=true);
        for (side=[-1,1]) translate([redodo_center[0],side*91,105]) cube([210,6,112],center=true);
    } else color([0.45,0.47,0.50]) {
        // Reserved external-access quick-change docks, not released STLs.
        for(side=[-1,1]) translate([-165,side*110,70]) cube([150,96,8],center=true);
    }
}

module high_guide_basket_cutout() {
    // Removes only material crossed by the contained ball channel. In empty
    // basket volume this subtraction has no effect; at the tapered front wall
    // it creates a deep, smooth entry rather than an impact face.
    for(i=[0:len(guide_centers)-2]) hull() {
        translate(guide_centers[i]) cube([10,96,76],center=true);
        translate(guide_centers[i+1]) cube([10,96,76],center=true);
    }
}

module basket() {
    color([0.25,0.66,0.34,0.45]) difference() {
        hull() {
            translate([basket_x0,0,basket_floor_rear]) cube([3,basket_y,3],center=true);
            translate([basket_x1,0,basket_floor_front]) cube([3,basket_y,3],center=true);
            translate([(basket_x0+basket_upper_x1)/2,0,basket_top])
                cube([basket_upper_x1-basket_x0,basket_y,3],center=true);
        }
        hull() {
            translate([basket_x0+3,0,basket_floor_rear+4]) cube([3,basket_y-6,6],center=true);
            translate([basket_x1-3,0,basket_floor_front+4]) cube([3,basket_y-6,6],center=true);
            translate([(basket_x0+basket_upper_x1)/2,0,basket_top+4])
                cube([basket_upper_x1-basket_x0-6,basket_y-6,8],center=true);
        }
        // Separate low launch opening; basket floor remains continuous.
        translate([basket_x1,0,55]) cube([20,88,46],center=true);
        high_guide_basket_cutout();
    }
}

module fixed_dock_and_singulator() {
    color([0.18,0.62,0.43,0.72]) {
        // Basket-to-low launch port chute; exact ball escapement is not released.
        hull() {
            translate([250,0,floor_z(250)+4]) cube([48,86,4],center=true);
            translate(gpoint(theta_launch,[-62,0,-8])) cube([12,82,4],center=true);
        }
    }
    color([0.10,0.65,0.95,0.75])
        translate([258,-65,72]) rotate([90,0,0]) cylinder(d=46,h=26,center=true);
}

module high_intake_guide() {
    // Intake-only fixed guide. First tangent follows the real +10 degree rear
    // exit, then direction changes in small steps before a gentle final drop.
    color([0.16,0.70,0.42,0.72]) {
        // Floor rails/slats: no broad shelf across the basket floor.
        for(i=[0:len(guide_centers)-2]) hull() {
            translate(guide_floor(guide_centers[i])) cube([8,82,4],center=true);
            translate(guide_floor(guide_centers[i+1])) cube([8,82,4],center=true);
        }
        // 72 mm internal height and 86 mm internal width contain a 66 mm ball.
        for(side=[-1,1],i=[0:len(guide_centers)-2]) hull() {
            p0=guide_floor(guide_centers[i]); p1=guide_floor(guide_centers[i+1]);
            translate([p0[0],side*44,p0[2]+36]) cube([8,4,76],center=true);
            translate([p1[0],side*44,p1[2]+36]) cube([8,4,76],center=true);
        }
        // Openable/soft roof envelope for rebound containment and jam access.
        for(i=[0:len(guide_centers)-2]) hull() {
            p0=guide_floor(guide_centers[i]); p1=guide_floor(guide_centers[i+1]);
            translate([p0[0],0,p0[2]+72]) cube([8,90,4],center=true);
            translate([p1[0],0,p1[2]+72]) cube([8,90,4],center=true);
        }
    }
    %color([1.0,0.75,0.08,0.45]) for(i=[0:len(guide_centers)-2]) hull() {
        translate(guide_centers[i]) sphere(d=ball_d);
        translate(guide_centers[i+1]) sphere(d=ball_d);
    }
}

module ball_paths(theta) {
    if (show_ball_paths) {
        // Ground approach to first contact and nip.
        %color([1.0,0.50,0.05,0.45]) hull() {
            translate([520,0,ball_d/2]) sphere(d=ball_d);
            translate(gpoint(theta,[nip_u+32,0,0])) sphere(d=ball_d);
        }
        %color([1.0,0.80,0.10,0.40]) hull() {
            translate(gpoint(theta,[nip_u+32,0,0])) sphere(d=ball_d);
            translate(gpoint(theta,[nip_u-32,0,0])) sphere(d=ball_d);
        }
    }
}

module swept_module() {
    %color([0.95,0.20,0.15,0.12]) hull() {
        moving_module(theta_intake); moving_module(theta_launch);
    }
}

module assembly(theta) {
    shared_base_frame(); pivot_supports(); front_arch_and_triangulation();
    fixed_stops_and_sensors(); actuator_between(theta); cable_route(theta);
    sensors_and_mounts(); camera_sight_envelope(); electronics_controller_envelopes();
    battery_tray(); battery(); basket();
    fixed_dock_and_singulator(); high_intake_guide();
    moving_module(theta); ball_paths(theta);
    if (show_swept_envelope) swept_module();
}

if (is_undef(p253_include_guard)) {
if (view=="launch") assembly(theta_launch);
else if (view=="overlay") {
    shared_base_frame(); pivot_supports(); front_arch_and_triangulation();
    fixed_stops_and_sensors(); sensors_and_mounts(); camera_sight_envelope();
    electronics_controller_envelopes(); battery_tray(); battery(); basket(); fixed_dock_and_singulator();
    high_intake_guide();
    color([1,0.55,0.05,0.45]) moving_module(theta_intake);
    color([0.2,0.65,0.95,0.45]) moving_module(theta_launch);
    actuator_between(theta_intake); actuator_between(theta_launch);
    cable_route(theta_intake); cable_route(theta_launch);
}
else if (view=="side_intake") intersection() { assembly(theta_intake); translate([-300,-700,-10]) cube([1000,700,500]); }
else if (view=="side_launch") intersection() { assembly(theta_launch); translate([-300,-700,-10]) cube([1000,700,500]); }
else if (view=="top") assembly(theta_intake);
else if (view=="actuator") { assembly(theta_intake); actuator_between(theta_launch); }
else if (view=="structure") { chassis_structure(); pivot_supports(); front_arch_and_triangulation(); }
else if (view=="sensors") { sensors_and_mounts(); camera_sight_envelope(); basket(); moving_module(theta_intake); }
else if (view=="drive_collision_context") {
    assembly(qa_angle);
    color([1,0,0,1]) intersection() {
        moving_hard_clearance(qa_angle);
        drive_package_envelopes();
    }
}
else if (view=="qa_wheel_yoke") intersection() { wheel_pair(); yoke_structure(); }
else if (view=="qa_sweep_frame") intersection() { union(){for(a=[theta_intake:2:theta_launch]) moving_frame_drive_clearance(a); moving_frame_drive_clearance(theta_launch);} chassis_structure(); }
else if (view=="qa_sweep_drive") intersection() { union(){for(a=[theta_intake:2:theta_launch]) moving_frame_drive_clearance(a); moving_frame_drive_clearance(theta_launch);} expanded_drive_hardware(clearance_probe); }
else if (view=="qa_drive_at_angle") intersection() { moving_frame_drive_clearance(qa_angle); expanded_drive_hardware(clearance_probe); }
else if (view=="qa_sweep_basket") intersection() { union(){for(a=[theta_intake:2:theta_launch]) moving_hard_clearance(a); moving_hard_clearance(theta_launch);} union(){basket();battery();} }
else if (view=="qa_sweep_high_guide") intersection() { union(){for(a=[theta_intake:2:theta_launch]) moving_hard_clearance(a); moving_hard_clearance(theta_launch);} high_intake_guide(); }
else if (view=="qa_sweep_arch") intersection() { union(){for(a=[theta_intake:2:theta_launch]) moving_frame_drive_clearance(a); moving_frame_drive_clearance(theta_launch);} front_arch_and_triangulation(); }
else if (view=="qa_sweep_arch_margin") intersection() {
    // Directional clearance screen consistent with the P2-39 pivot-offset
    // method: shift the fixed arch toward the moving module by the required
    // nominal X clearance and require an empty exact-solid intersection.
    union(){for(a=[theta_intake:2:theta_launch]) moving_frame_drive_clearance(a); moving_frame_drive_clearance(theta_launch);}
    translate([arch_required_nominal_clearance,0,0]) front_arch_and_triangulation();
}
else if (view=="qa_arch_at_angle") intersection() { moving_frame_drive_clearance(qa_angle); front_arch_and_triangulation(); }
else if (view=="qa_sweep_actuator") intersection() { union(){for(a=[theta_intake:2:theta_launch]) actuator_between(a); actuator_between(theta_launch);} union(){basket();battery();drive_package_envelopes();} }
else if (view=="qa_lidar_scan") intersection() {
    translate([330,0,lidar_scan_z]) cube([900,900,1],center=true);
    union(){ basket(); high_intake_guide(); moving_module(theta_intake); moving_module(theta_launch); front_arch_and_triangulation(); }
}
else if (view=="qa_camera_sight") { assembly(theta_intake); camera_sight_envelope(); }
else if (view=="qa_battery_frame") intersection() { battery(); union(){shared_base_frame();pivot_supports();} }
else if (view=="qa_battery_controllers") intersection() {
    battery(); d5065_controller_envelopes();
}
else if (view=="qa_battery_rear_electronics") intersection() {
    battery(); rear_electronics_envelope();
}
else if (view=="qa_basket_drive") intersection() { basket(); drive_hardware_envelopes(); }
else if (view=="qa_basket_controllers") intersection() { basket(); d5065_controller_envelopes(); }
else if (view=="qa_pivot_obstacles") intersection() {
    pivot_bearing_running_envelopes();
    union(){ drive_package_envelopes(); chassis_structure(); front_arch_and_triangulation(); basket(); high_intake_guide(); }
}
else if (view=="qa_pivot_drive") intersection() { pivot_bearing_running_envelopes(); drive_package_envelopes(); }
else if (view=="qa_pivot_frame") intersection() { pivot_bearing_running_envelopes(); chassis_structure(); }
else if (view=="qa_pivot_arch") intersection() { pivot_bearing_running_envelopes(); front_arch_and_triangulation(); }
else if (view=="qa_pivot_basket") intersection() { pivot_bearing_running_envelopes(); basket(); }
else if (view=="qa_pivot_guide") intersection() { pivot_bearing_running_envelopes(); high_intake_guide(); }
else if (view=="qa_guide_low_frame_at_angle") intersection() {
    module_pose(qa_angle) moving_throat_and_guide();
    if(clearance_probe<=0) low_torsion_deck();
    else minkowski(){ low_torsion_deck(); sphere(r=clearance_probe,$fn=16); }
}
else if (view=="p244_intake") {
    shared_base_frame(); pivot_supports(); front_arch_and_triangulation();
    sensors_and_mounts(); battery_tray(); battery(); basket();
    p244_fixed_open_ramp(); p244_moving_hard(theta_intake); ball_paths(theta_intake);
}
else if (view=="p244_side") intersection() {
    union(){ shared_base_frame(); pivot_supports(); front_arch_and_triangulation(); basket(); p244_fixed_open_ramp(); p244_moving_hard(theta_intake); }
    translate([-300,-700,-10]) cube([1000,700,500]);
}
else if (view=="p244_qa_ramp_sweep") intersection() {
    union(){ for(a=[theta_intake:1:theta_launch]) p244_moving_hard(a); p244_moving_hard(theta_launch); }
    p244_fixed_open_ramp();
}
else if (view=="p244_qa_ramp_frame") intersection() {
    p244_fixed_open_ramp();
    union(){ chassis_structure(); front_arch_and_triangulation(); }
}
else if (view=="p244_qa_cheek_wheel") module_pose(theta_intake) intersection() {
    p244_side_cheeks(); wheel_pair();
}
else assembly(theta_intake);
}
