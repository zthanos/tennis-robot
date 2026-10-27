// P2-53 geometry re-audit of P2-52. Frozen P2-40 geometry; no STL release.
// Units mm, +X front, +Y left, +Z up.
p253_include_guard=true;
include <shared-flywheel-intake-launcher-phase1.scad>

p253_view="assembly"; p253_angle=0; clearance_probe_p253=0; $fn=40;
p257_variant="p257_shield_m100";
p253_fixed=[310,-205,120]; p253_moving_local=[50,-205,90];
fit_body_d=37; fit_body_l=70; tr8_d=8;
// P2-54 dependent-only reroute.  P2-40 pivot/module geometry is unchanged.
// Raise the belt-side FIT0186 carrier and arch the moving arm above the
// launcher so the three marginal P2-53 nominal gaps gain tolerance budget.
p254_fit_offset=[0,40.5,60];
p254_arm_root=[-10,-160,68];
p254_arm_high=[-10,-205,135];

// Exact P2-51 basket/ski values.
p251_side_rim_z=320; p251_backboard_top_z=375; p251_front_x=278;
p251_rear_x=-85; p251_half_y=172; p251_rear_half_y=140;
p251_floor_front_z=42; p251_floor_rear_z=70;
p251_ski_pts=[[435,10],[383,10],[376.463,10.285],[369.976,11.139],
 [363.589,12.556],[357.348,14.523],[351.304,17.027],
 [345.500,20.048],[339.982,23.564],[334.791,27.547],[332.815,29.264]];

function vsub(a,b)=[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
function vadd(a,b)=[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
function vmul(a,s)=[a[0]*s,a[1]*s,a[2]*s];
function vlen(a)=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
function vunit(a)=vmul(a,1/vlen(a));
function p253_moving(a)=gpoint(a,p253_moving_local);
module segment(a,b,d){hull(){translate(a)sphere(d=d);translate(b)sphere(d=d);}}
module expanded_p253(c){if(c<=0)children();else minkowski(){children();sphere(r=c,$fn=12);}}
module xz_strip_p253(p0,p1,width=94,t=3){
 dx=p1[0]-p0[0];dz=p1[1]-p0[1];l=sqrt(dx*dx+dz*dz);a=atan2(dz,dx);
 translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])rotate([0,-a,0])cube([l+.8,width,t],center=true);
}

// Frozen launcher: true vertical axes, centres +/-129, D200x60, top plate,
// yokes, trunnions and D5065. Legacy actuator surrogate is intentionally omitted.
module p253_frozen_launcher(a){module_pose(a){
 upper_plate();for(yy=[-wheel_y,wheel_y])wheel_and_motor(yy);
 trunnions_and_standoffs();moving_throat_and_guide();side_cheeks();
}}
module p253_launcher_without_trunnion(a){module_pose(a){
 upper_plate();for(yy=[-wheel_y,wheel_y])wheel_and_motor(yy);
 moving_throat_and_guide();side_cheeks();
}}
module p251_ski_exact(){
 color([.08,.62,.82])for(i=[0:len(p251_ski_pts)-2])xz_strip_p253(p251_ski_pts[i],p251_ski_pts[i+1]);
 for(side=[-1,1])color([.04,.42,.62])for(i=[0:len(p251_ski_pts)-2]){
  p0=p251_ski_pts[i];p1=p251_ski_pts[i+1];dx=p1[0]-p0[0];dz=p1[1]-p0[1];l=sqrt(dx*dx+dz*dz);a=atan2(dz,dx);
  translate([(p0[0]+p1[0])/2,side*48.5,(p0[1]+p1[1])/2+10])rotate([0,-a,0])cube([l+.8,3,20],center=true);
 }}
module p251_grid_side(y){color([.35,.62,.38]){
 for(x=[-70:46:252])translate([x,y,(p251_floor_rear_z+p251_side_rim_z)/2])cube([7,3,p251_side_rim_z-p251_floor_rear_z],center=true);
 for(z=[82:46:312])translate([(p251_rear_x+p251_front_x)/2,y,z])cube([p251_front_x-p251_rear_x,3,7],center=true);
}}
module p251_basket_exact(){
 color([.48,.70,.50,.75])xz_strip_p253([p251_rear_x,p251_floor_rear_z],[p251_front_x,p251_floor_front_z],280,4);
 for(y=[-p251_half_y,p251_half_y])p251_grid_side(y);
 color([.38,.58,.40])translate([p251_rear_x,0,(p251_floor_rear_z+p251_backboard_top_z)/2])rotate([0,12,0])cube([4,2*p251_rear_half_y,p251_backboard_top_z-p251_floor_rear_z],center=true);
 for(y=[-156,156])color([.38,.58,.40])translate([p251_front_x,y,190])cube([4,32,260],center=true);
}
module p251_feed_exact(){color([.18,.42,.78])xz_strip_p253([p251_rear_x,58],[225,38],80,4);}
module p253_power(){
 for(p=parkside_pack_centers)color([.1,.58,.22,.72])translate(p)cube(parkside_pack_size,center=true);
 color([.08,.12,.20,.8])translate([-135,0,145])cube([46,115,45.5],center=true);
 rear_electronics_envelope();d5065_controller_envelopes();
}

module p253_moving_arm(a){color([.7,.35,.7])module_pose(a){
 // The old straight root at local Z45 crossed the true tread. Route from
 // the upper-plate edge upward/outboard, then remain above the wheel.
 hull(){translate(p254_arm_root)cube([14,16,14],center=true);translate(p254_arm_high)cube([14,18,14],center=true);}
 hull(){translate(p254_arm_high)cube([14,18,14],center=true);translate(p253_moving_local)cube([18,18,18],center=true);}
 // P2-55 0b: mandatory symmetric passive stop tongues.  The actuator remains
 // only at -Y; +Y is a stop, not a second actuator.
 for(side=[-1,1])translate([20,side*205,115])cube([18,18,16],center=true);
}}
module p253_screw_and_clevises(a){m=p253_moving(a);
 color("silver")segment(p253_fixed,m,tr8_d);
 color([.65,.65,.68]){translate(p253_fixed)sphere(d=18);translate(m)sphere(d=18);}
 color([.45,.28,.58])translate(p253_fixed)cube([20,20,28],center=true);
}
module p253_fit_motor(a){m=p253_moving(a);u=vunit(vsub(m,p253_fixed));
 // P2-53 correction: a coaxial 37x70 body behind the clevis intersects the
 // X318 arch and X300 drive package. The motor rides on the same rocking
 // carrier, parallel to the screw.  P2-54 raises/reroutes that carrier to
 // +40.5 Y/+60 Z. Its output face drives Tr8 through a short 1:1 toothed belt.
 face=vadd(p253_fixed,p254_fit_offset);tip=vadd(face,vmul(u,fit_body_l));
 color([.12,.13,.15])segment(face,tip,fit_body_d);
 color([.92,.72,.12])translate(tip)sphere(d=fit_body_d+2);
 color([.18,.18,.20])segment(p253_fixed,face,8); // belt/carrier keep-out
 color([.35,.35,.38])hull(){translate(p253_fixed)sphere(d=14);translate(face)sphere(d=14);}
}
module p253_tr8_only(a){m=p253_moving(a);u=vunit(vsub(m,p253_fixed));
 segment(vadd(p253_fixed,vmul(u,12)),vadd(m,vmul(u,-12)),tr8_d);
}
module p253_actuator(a){p253_screw_and_clevises(a);p253_fit_motor(a);p253_moving_arm(a);}
module p253_p16_reference(a){color([.6,.6,.65,.45])segment(p253_fixed,p253_moving(a),20);}

module p253_as5600(){
 color([.48,.12,.64])translate([pivot[0],244,pivot[2]])cube([28,5,22],center=true);
 color("gold")translate([pivot[0],240.5,pivot[2]])rotate([90,0,0])cylinder(d=6,h=2.5,center=true);
}
module p253_cam_moving(a){color([.92,.48,.08])module_pose(a)hull(){
 rotate([90,0,0])cylinder(d=38,h=12,center=true);
 translate([8,0,5])rotate([90,0,0])cylinder(d=30,h=12,center=true);
}}
module p253_followers(){color([.82,.82,.22]){
 translate([365,-13,72])rotate([90,0,0])cylinder(d=12,h=10,center=true);
 translate([405,13,72])rotate([90,0,0])cylinder(d=12,h=10,center=true);
}}
module p253_stops(){
 for(side=[-1,1]){
  color([.58,.34,.10])hull()for(a=[-10,-3]){p=gpoint(a,[20,side*205,115]);translate(p)cube([28,24,18],center=true);}
  color([.58,.34,.10])hull()for(a=[15,20]){p=gpoint(a,[20,side*205,115]);translate(p)cube([28,24,18],center=true);}
 }
}
module p253_cables(a){for(side=[-1,1]){
 p0=gpoint(a,[30,side*172,125]);p1=[365,side*180,215];p2=[300,side*180,205];
 color([.95,.72,.10]){segment(p0,p1,8);segment(p1,p2,8);}
}}

module p253_fixed_world(){
 chassis_structure();drive_package_envelopes();pivot_supports();front_arch_and_triangulation();
 sensors_and_mounts();p251_basket_exact();p251_ski_exact();p251_feed_exact();p253_power();
}
module p253_assembly(a){p253_fixed_world();p253_frozen_launcher(a);p253_actuator(a);
 p253_as5600();p253_cam_moving(a);p253_followers();p253_stops();p253_cables(a);
}
module p253_sweep_launcher(){union(){for(a=[-10:1:20])p253_frozen_launcher(a);}}
module p253_sweep_actuator(){union(){for(a=[-10:1:20])p253_actuator(a);}}
module p253_sweep_motor(){union(){for(a=[-10:1:20])p253_fit_motor(a);}}
module p253_sweep_tr8(){union(){for(a=[-10:1:20])p253_tr8_only(a);}}
module p253_sweep_arm(){union(){for(a=[-10:1:20])p253_moving_arm(a);}}
module p253_sweep_launcher_without_trunnion(){union(){for(a=[-10:1:20])p253_launcher_without_trunnion(a);}}
module p253_launcher_for_arm_qa(a){difference(){
 p253_frozen_launcher(a);
 module_pose(a)translate(p254_arm_root)cube([30,30,30],center=true);
}}
module p253_sweep_launcher_for_arm_qa(){union(){for(a=[-10:1:20])p253_launcher_for_arm_qa(a);}}
module p253_sweep_cam(){union(){for(a=[-10:1:20])p253_cam_moving(a);}}
module p253_sweep_cables(){union(){for(a=[-10:1:20])p253_cables(a);}}
module p253_arch_without_mount(){difference(){front_arch_and_triangulation();translate(p253_fixed)cube([70,70,90],center=true);}}

// P2-57 replaceable printed cheeks/shields only.  Frozen P2-40 wheel/nip/rail
// core and P2-51 ski/basket are not changed.
function p257_mouth(v)=p257_stagger(v)?140:v=="p257_shield_m90"?90:v=="p257_shield_m110"?110:100;
function p257_stagger(v)=v=="p257_stagger_open"||v=="p257_stagger_shield";
function p257_shielded(v)=v!="p257_stagger_open";
module p257_cheek_segment(side,mouth){
 half=mouth/2;rear=[105,side*46,20];front=[180,side*half,20];
 color([.15,.65,.82])hull(){translate(rear)cube([6,6,60],center=true);translate(front)cylinder(r=6,h=60,center=true);}
}
module p257_front_parts_local(v){
 mouth=p257_mouth(v);
 for(side=[-1,1])p257_cheek_segment(side,mouth);
 if(p257_stagger(v))color([.15,.65,.82])hull(){
  translate([180,55,35])cube([6,6,60],center=true);
  translate([250,100,35])cylinder(r=3,h=60,center=true);
 }
 if(p257_shielded(v))for(side=[-1,1])color([.12,.48,.72]){
  translate([130,side*118,18])cube([6,144,60],center=true);
  translate([130,side*46,18])cylinder(r=6,h=60,center=true);
 }
}
module p257_front_parts(a,v){module_pose(a)p257_front_parts_local(v);}
module p257_fixed_collision_target(){
 p253_fixed_world();
 // Ground half-space surrogate: any front-part material below Z=0 intersects.
 translate([100,0,-25])cube([1000,700,50],center=true);
}
module p257_launcher_core(a){module_pose(a){
 upper_plate();for(yy=[-wheel_y,wheel_y])wheel_and_motor(yy);
 trunnions_and_standoffs();moving_throat_and_guide();
}}
module p257_assembly(a,v){p253_fixed_world();p257_launcher_core(a);p257_front_parts(a,v);}

module ground_and_marks(){
 color([.05,.05,.05])translate([100,0,0])cube([800,3,1.5],center=true);
 color([.05,.05,.05])segment([430,0,18],[510,0,18],4);
 color([.05,.05,.05]){segment([510,0,18],[496,0,28],4);segment([510,0,18],[496,0,8],4);}
}
module label_xz(s,p,size=14){color([.05,.05,.05])translate(p)rotate([90,0,0])linear_extrude(height=1,center=true)text(s,size=size,halign="center",valign="center");}
module annotated(a){p253_assembly(a);ground_and_marks();label_xz(str(a," deg"),[385,0,345],16);label_xz("GROUND",[-185,0,14],12);label_xz("FRONT +X",[470,0,42],12);}
module section_y0(a){intersection(){annotated(a);translate([100,0,210])cube([900,2,440],center=true);}}

if(p253_view=="assembly")p253_assembly(p253_angle);
else if(p253_view=="side")projection(cut=false)rotate([-90,0,0])annotated(p253_angle);
else if(p253_view=="top")projection(cut=false)p253_assembly(p253_angle);
else if(p253_view=="section")projection(cut=false)rotate([-90,0,0])section_y0(p253_angle);
// Tessellated nominal solids used only by the P2-53 VTK distance audit.
else if(p253_view=="mesh_launcher_at_angle")p253_frozen_launcher(p253_angle);
else if(p253_view=="mesh_launcher_arm_target_at_angle")p253_launcher_for_arm_qa(p253_angle);
else if(p253_view=="mesh_motor_at_angle")p253_fit_motor(p253_angle);
else if(p253_view=="mesh_tr8_at_angle")p253_tr8_only(p253_angle);
else if(p253_view=="mesh_arm_at_angle")p253_moving_arm(p253_angle);
else if(p253_view=="mesh_cam_at_angle")p253_cam_moving(p253_angle);
else if(p253_view=="mesh_cables_at_angle")p253_cables(p253_angle);
else if(p253_view=="mesh_sweep_actuator")p253_sweep_actuator();
else if(p253_view=="mesh_sweep_motor")p253_sweep_motor();
else if(p253_view=="mesh_sweep_tr8")p253_sweep_tr8();
else if(p253_view=="mesh_sweep_arm")p253_sweep_arm();
else if(p253_view=="mesh_sweep_cam")p253_sweep_cam();
else if(p253_view=="mesh_sweep_cables")p253_sweep_cables();
else if(p253_view=="mesh_sweep_launcher")p253_sweep_launcher();
else if(p253_view=="mesh_sweep_launcher_without_trunnion")p253_sweep_launcher_without_trunnion();
else if(p253_view=="mesh_frame")chassis_structure();
else if(p253_view=="mesh_drive")drive_package_envelopes();
else if(p253_view=="mesh_arch")p253_arch_without_mount();
else if(p253_view=="mesh_sensors")sensors_and_mounts();
else if(p253_view=="mesh_basket_ski")union(){p251_basket_exact();p251_ski_exact();p251_feed_exact();}
else if(p253_view=="mesh_power")p253_power();
else if(p253_view=="mesh_as5600")p253_as5600();
else if(p253_view=="mesh_cam_fixed_target")union(){chassis_structure();p251_ski_exact();p251_feed_exact();}
else if(p253_view=="mesh_stops")p253_stops();
else if(p253_view=="mesh_cables_fixed_target")union(){chassis_structure();front_arch_and_triangulation();sensors_and_mounts();p251_basket_exact();p253_power();}
else if(p253_view=="mesh_p257_at_angle")p257_front_parts(p253_angle,p257_variant);
else if(p253_view=="mesh_p257_fixed_target")p257_fixed_collision_target();
else if(p253_view=="qa_p257_fixed_at_angle")intersection(){p257_front_parts(p253_angle,p257_variant);p257_fixed_collision_target();}
else if(p253_view=="p257_assembly")p257_assembly(p253_angle,p257_variant);
else if(p253_view=="qa_actuator_launcher")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_actuator(a);p253_frozen_launcher(a);}}
else if(p253_view=="qa_actuator_frame")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();chassis_structure();}
else if(p253_view=="qa_actuator_drive")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();drive_package_envelopes();}
else if(p253_view=="qa_actuator_arch")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();p253_arch_without_mount();}
else if(p253_view=="qa_actuator_sensors")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();sensors_and_mounts();}
else if(p253_view=="qa_actuator_basket_ski")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();union(){p251_basket_exact();p251_ski_exact();p251_feed_exact();}}
else if(p253_view=="qa_actuator_power")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_actuator();p253_power();}
else if(p253_view=="qa_motor_launcher")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_fit_motor(a);p253_frozen_launcher(a);}}
else if(p253_view=="qa_motor_drive")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_motor();drive_package_envelopes();}
else if(p253_view=="qa_motor_arch")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_motor();p253_arch_without_mount();}
else if(p253_view=="qa_motor_drive_at_angle")intersection(){expanded_p253(clearance_probe_p253)p253_fit_motor(p253_angle);drive_package_envelopes();}
else if(p253_view=="qa_motor_arch_at_angle")intersection(){expanded_p253(clearance_probe_p253)p253_fit_motor(p253_angle);p253_arch_without_mount();}
else if(p253_view=="qa_tr8_launcher")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_tr8_only(a);p253_frozen_launcher(a);}}
else if(p253_view=="qa_tr8_drive")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_tr8();drive_package_envelopes();}
else if(p253_view=="qa_arm_drive")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_arm();drive_package_envelopes();}
else if(p253_view=="qa_arm_launcher_forbidden")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_moving_arm(a);p253_launcher_for_arm_qa(a);}}
else if(p253_view=="qa_as5600_launcher")intersection(){expanded_p253(clearance_probe_p253)p253_as5600();p253_sweep_launcher_without_trunnion();}
else if(p253_view=="qa_as5600_drive")intersection(){expanded_p253(clearance_probe_p253)p253_as5600();drive_package_envelopes();}
else if(p253_view=="qa_cam_launcher")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_cam_moving(a);p253_frozen_launcher(a);}}
else if(p253_view=="qa_cam_fixed")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_cam();union(){chassis_structure();p251_ski_exact();p251_feed_exact();}}
else if(p253_view=="qa_stops_launcher")intersection(){expanded_p253(clearance_probe_p253)p253_stops();p253_sweep_launcher();}
else if(p253_view=="qa_stops_launcher_at_angle")intersection(){expanded_p253(clearance_probe_p253)p253_stops();p253_frozen_launcher(p253_angle);}
else if(p253_view=="qa_cables_launcher")union(){for(a=[-10:1:20])intersection(){expanded_p253(clearance_probe_p253)p253_cables(a);p253_frozen_launcher(a);}}
else if(p253_view=="qa_cables_fixed")intersection(){expanded_p253(clearance_probe_p253)p253_sweep_cables();union(){chassis_structure();front_arch_and_triangulation();sensors_and_mounts();p251_basket_exact();p253_power();}}
else p253_assembly(p253_angle);

echo("P2_53_FROZEN_GEOMETRY_REAUDIT_NO_STL",angle=p253_angle,rails_y=[-220,220],rail_width=50,
 wheel_centres_y=[-wheel_y,wheel_y],wheel_axis="local_Z",wheel_d=wheel_d,wheel_width=wheel_width,
 pivot=pivot,actuator_fixed=p253_fixed,actuator_moving_local=p253_moving_local,
 p254_fit_offset=p254_fit_offset,p254_arm_root=p254_arm_root,p254_arm_high=p254_arm_high,
 stop_y=[-205,205],stop_ranges=[[-10,-3],[15,20]],
 ugreen_center=[-135,0,145],ugreen_size=[46,115,45.5]);
