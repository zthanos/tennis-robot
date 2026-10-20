// Shared full-robot context and exact keep-outs for the selected body concept F.
// P2-86 extracted these neutral helpers from the superseded A--E concept file.
// No panel geometry lives here.

use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p267-tilted-arch.scad>
use <../modular-test-chassis/shared-flywheel-p268-upright-wall.scad>
use <../modular-test-chassis/shared-rear-pod-p270.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/drive-test-control-console.scad>

bp_support_view = "none";
bp_angle = 0;
show_console = false;
ball_r = 33;
cam_c = [410,0,305];
z_ceiling = 375;

function nip(a) = gpoint(a,[55,0,0]);
function throat(a) = gpoint(a,[-50,0,0]);

module context(a){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors();
  p271_basket_exact(); p267_ski(75); p251_feed_exact();
  d5065_controller_envelopes();
  p253_frozen_launcher(a); p253_actuator(a); p253_as5600(); p253_stops(); p253_cables(a);
  p267_soft_deflector(); p271_front_system(a,true);
  p270_mounts();
  if(show_console) translate([45,220,52]) color([0.9,0.9,0.2]) console();
}

module bp_moving(a){
  p253_frozen_launcher(a);
  module_pose(a) p244_side_cheeks();
  p253_actuator(a); p253_cam_moving(a); p253_cables(a);
  p268_feed_door(a,false); p268_selector_nose(a);
}

module bp_airflow(a){ module_pose(a) for(yy=[-129,129]) d5065_airflow_reservation(yy); }

module bp_fixed(){
  frame_assembly(); pivot_supports(); p267_arch(); p267_sensors();
  p271_basket_exact(); p267_ski(75); p251_feed_exact(); p267_soft_deflector();
  p271_front_grid(); p268_fixed_feed_channel(); p253_as5600(); p253_stops();
  p270_mounts(); drive_hardware_envelopes();
}

module ko_launch_cone(L=1600){
  for(a=[15,17.5,20]) hull(){
    translate(nip(a)) sphere(r=ball_r);
    for(de=[-10,10],sy=[-1,1]) let(e=a+de,d=[cos(e),0,sin(e)])
      translate(nip(a)+L*d+[0,sy*L*tan(10),0]) sphere(r=ball_r);
  }
}

module ko_camera_fov(){ hull(){
  translate(cam_c) sphere(d=8);
  for(p=[[605,-120,33],[605,120,33],[490,-70,33],[490,70,33]]) translate(p) sphere(d=8);
} }

module ko_lidar(){ translate([-400,-450,z_ceiling]) cube([1100,900,60]); }

module ko_ball_path(){
  hull(){ translate([620,0,ball_r]) sphere(r=ball_r+10); translate(nip(-3)) sphere(r=ball_r+10); }
  pts=concat([[435,10],[383,10]],[for(t=[15:15:75]) [383-75*sin(t),10+75*(1-cos(t))]]);
  for(i=[0:len(pts)-2]) hull() for(p=[pts[i],pts[i+1]],y=[-47,47])
    translate([p[0],y,p[1]+ball_r]) sphere(r=ball_r+10);
  translate([-122,-176,40]) cube([477,352,340]);
}

module ko_feed_path(){ for(a=[15,20]) hull() for(y=[-40,40]){
  translate([278,y,42+ball_r]) sphere(r=ball_r+10);
  translate(throat(a)+[0,y,0]) sphere(r=ball_r+10);
} }

module ko_pod_contents(){ p270_reserved_contents(); }

// Standalone target dispatcher used by qa-clearance-f.py after P2-86.
if(bp_support_view=="mesh_moving") bp_moving(bp_angle);
else if(bp_support_view=="mesh_airflow") bp_airflow(bp_angle);
else if(bp_support_view=="mesh_tyres") drive_tire_envelopes();
else if(bp_support_view=="mesh_stops") p253_stops();
else if(bp_support_view=="mesh_fixed") bp_fixed();
else if(bp_support_view=="mesh_console") translate([45,195,52]) cube([140,50,62]);
