// P2-67 dependent CAD / kinematics only. No STL release.
// Units mm; +X front, +Y left, +Z up.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>

p267_view = "assembly"; // assembly | section | gate_sweep | qa_arch_sweep | qa_basket_sweep
p267_angle = -3;
p267_exit_angle = 75;
p267_arch_base_x = 318;
p267_arch_top_x = 390;       // 50 mm crossbar => rear edge X365
p267_arch_base_z = 52;
p267_arch_top_z = 245;
p267_lidar_x = 390;
p267_camera = [410,0,305];
p267_wall_tilt = 27.5;
p267_wall_bottom = [278,42];
p267_wall_top_z = 90;        // trajectory-safe, but REJECTED: insufficient pile retention
p267_wall_top_x = p267_wall_bottom[0]
    -(p267_wall_top_z-p267_wall_bottom[1])*tan(p267_wall_tilt);
p267_deflector_rear = [250,363];
p267_deflector_front = [360,343];
$fn=48;

function p267_gate_open(a)=a<=0?0:a>=15?95:95*a/15;
function p267_arc_point(a)=[383-75*sin(a),10+75*(1-cos(a))];
function p267_ski_points(a)=concat([[435,10],[383,10]],
    [for(t=[5:5:a])p267_arc_point(t)]);

module p267_xz_strip(p0,p1,width=94,t=3){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
    rotate([0,-a,0])cube([l+.8,width,t],center=true);
}

module p267_ski(a=p267_exit_angle){
  pts=p267_ski_points(a);
  color([.08,.62,.82])for(i=[0:len(pts)-2])
    p267_xz_strip(pts[i],pts[i+1],min(94,61+i*6),2);
}

module p267_arch(){
  color([.15,.48,.78,.92]){
    for(side=[-1,1]){
      hull(){
        translate([p267_arch_base_x,side*220,p267_arch_base_z])cube([30,50,10],center=true);
        translate([p267_arch_top_x,side*220,p267_arch_top_z])cube([30,50,10],center=true);
      }
      // Existing chassis feet remain at X318. Two triangular braces deliver
      // the inclined-leg load to the X240/X300 frame nodes.
      hull(){
        translate([240,side*220,52])cube([20,30,20],center=true);
        translate([p267_arch_top_x,side*220,p267_arch_top_z])cube([30,30,20],center=true);
      }
      hull(){
        translate([300,side*220,52])cube([20,30,20],center=true);
        translate([p267_arch_top_x,side*220,p267_arch_top_z])cube([30,30,20],center=true);
      }
    }
    translate([p267_arch_top_x,0,p267_arch_top_z])hollow_box_y(440,50,30);
  }
  // The P16 remains at its frozen point. This short clevis bracket carries
  // the actuator load to the right inclined leg without body interference.
  color([.58,.34,.10])hull(){
    translate([320,-190,120])sphere(d=18);
    translate([343.4,-203,120])sphere(d=18);
  }
}

module p267_sensors(){
  mast_crossbar_z=354;
  color([.20,.50,.76])for(side=[-1,1])
    translate([p267_lidar_x,side*150,(p267_arch_top_z+mast_crossbar_z)/2])
      cube([25,25,mast_crossbar_z-p267_arch_top_z],center=true);
  color([.20,.50,.76])translate([p267_lidar_x,0,mast_crossbar_z])cube([320,30,20],center=true);
  color([.08,.09,.11])translate([p267_lidar_x,0,374])cylinder(d=76,h=32,center=true);
  color([.08,.09,.11])translate(p267_camera)rotate([0,50,0])cube([82,32,28],center=true);
}

module p267_soft_deflector(){
  color([.82,.25,.62,.72])
    p267_xz_strip(p267_deflector_rear,p267_deflector_front,340,10);
  // Independent basket-side elastomer mounts: no rigid load path to the
  // sensor arch.  The yellow blocks are the compliant isolator envelopes.
  for(side=[-1,1]){
    color([.95,.72,.10])for(p=[p267_deflector_rear,p267_deflector_front])
      hull(){
        translate([p[0],side*170,p[1]])cube([18,8,8],center=true);
        translate([p[0],side*170,320])cube([18,8,8],center=true);
      }
    color([.18,.42,.28])translate([305,side*172,320])cube([130,6,12],center=true);
  }
}

module p267_gate(angle=p267_angle,servo=false){
  open=p267_gate_open(angle);
  translate([p267_wall_top_x,0,p267_wall_top_z])rotate([0,open,0])
    translate([-p267_wall_top_x,0,-p267_wall_top_z]){
      color([.32,.58,.36])
        p267_xz_strip(p267_wall_bottom,[p267_wall_top_x,p267_wall_top_z],344,4);
      color([.95,.52,.08])
        p267_xz_strip(p267_wall_bottom,[p267_wall_top_x,p267_wall_top_z],96,5);
    }
  color("silver")translate([p267_wall_top_x,0,p267_wall_top_z])
    rotate([90,0,0])cylinder(d=8,h=356,center=true);
  color([.75,.75,.18])translate([p267_wall_top_x,166,p267_wall_top_z])
    rotate([90,0,0])cylinder(d=18,h=12,center=true);
  if(servo)color([.20,.20,.24])translate([235,-188,115])cube([42,24,40],center=true);
}

module p267_fixed(angle=p267_angle){
  p267_arch(); p267_sensors(); p267_ski(); p267_soft_deflector(); p267_gate(angle,true);
}

module p267_assembly(angle=p267_angle){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors();
  p251_basket_exact(); p251_feed_exact(); p267_ski();
  p253_power(); p253_frozen_launcher(angle); p253_actuator(angle);
  p253_as5600(); p253_stops(); p253_cables(angle);
  p267_soft_deflector(); p267_gate(angle,true);
}

module p267_moving_sweep(){
  for(a=[-10:1:20])union(){
    p253_frozen_launcher(a); p253_actuator(a); p253_cables(a);
  }
  p253_as5600(); p253_stops(); p253_power(); pivot_supports();
}

if(p267_view=="assembly")p267_assembly(p267_angle);
else if(p267_view=="section")projection(cut=true)rotate([90,0,0])p267_assembly(p267_angle);
else if(p267_view=="gate_sweep")for(a=[-10:1:20])p267_gate(a,false);
else if(p267_view=="qa_arch_sweep")intersection(){
  union(){p267_arch();p267_sensors();} p267_moving_sweep();
}
else if(p267_view=="qa_basket_sweep")intersection(){
  union(){p267_soft_deflector();for(a=[-10:1:20])p267_gate(a,false);}
  p267_moving_sweep();
}
