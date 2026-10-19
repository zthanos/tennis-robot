// P2-66 concept CAD / kinematics only. No STL release.
// Units mm; +X front, +Y left, +Z up. P2-40 core remains frozen.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>

p266_view = "section";          // section | assembly | gate_sweep | launcher_sweep_qa
p266_angle = -3;                // launcher: -3 collect, 0 transport, +15..20 launch
p266_radius = 75;               // 75 or 110
p266_exit_angle = 80;           // 70, 75 or 80
p266_wall_top = 240;
p266_door_w = 96;
p266_door_bottom = 42;
p266_door_top = 122;
$fn = 48;

function p266_arc_point(r,a)=[383-r*sin(a),10+r*(1-cos(a))];
function p266_ski_points(r,a)=concat([[435,10],[383,10]],
  [for(t=[5:5:a])p266_arc_point(r,t)]);

module p266_xz_strip(p0,p1,width=94,t=3){
  dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
    rotate([0,-a,0])cube([l+.8,width,t],center=true);
}

module p266_ski(r=p266_radius,a=p266_exit_angle){
  pts=p266_ski_points(r,a);
  color([.08,.62,.82])for(i=[0:len(pts)-2])p266_xz_strip(pts[i],pts[i+1]);
}

// The complete front panel is the gate: leaving fixed side panels intersected
// the frozen ±129 mm wheel/module sweep. A top hinge, torsion springs and cam
// open it into the basket only after 0°; loss of linkage/power returns it to
// the continuous closed plane. The orange central patch marks the X278/Z42
// feed-door datum used by the D36 beam, not a permanent aperture.
module p266_front_wall_fixed(){color([.32,.58,.36]){
  for(y=[-170,170])translate([278,y,(42+p266_wall_top)/2])cube([6,4,p266_wall_top-42],center=true);
  translate([278,0,p266_wall_top])cube([8,344,8],center=true);
}}
function p266_gate_open(angle)=angle<=0?0:angle>=15?95:95*angle/15;
module p266_gate(angle=p266_angle,servo=false){
  open=p266_gate_open(angle);
  color(servo?[.35,.66,.38]:[.32,.58,.36])translate([278,0,p266_wall_top])
    rotate([0,open,0])translate([0,0,-(p266_wall_top-p266_door_bottom)/2])
      cube([4,344,p266_wall_top-p266_door_bottom],center=true);
  color([.95,.52,.08])translate([278,0,p266_wall_top])
    rotate([0,open,0])translate([0,0,-(p266_wall_top-(p266_door_bottom+p266_door_top)/2)])
      cube([5,p266_door_w,p266_door_top-p266_door_bottom],center=true);
  // Hinge, spring keep-out and optional PCA9685-driven servo/cam envelope.
  color("silver")translate([278,0,p266_wall_top])rotate([90,0,0])cylinder(d=8,h=356,center=true);
  color([.75,.75,.18])translate([278,166,p266_wall_top])rotate([90,0,0])cylinder(d=18,h=12,center=true);
  if(servo)color([.20,.20,.24])translate([258,-188,218])cube([42,24,40],center=true);
}

// 10 mm compliant face proxy, front edge Z334, rear edge Z370, top Z375.
module p266_soft_deflector(){color([.82,.25,.62,.72])
  p266_xz_strip([274,334],[82,370],340,10);
}

module p266_basket_without_p251_front_posts(){
  // Exact P2-51 floor/sides/rear are provided by the full-robot assembly.
  p251_basket_exact();
}

module p266_fixed(angle=p266_angle,servo=false){
  p266_ski(); p266_front_wall_fixed(); p266_gate(angle,servo); p266_soft_deflector();
}

module p266_assembly(angle=p266_angle,servo=false){
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  front_arch_and_triangulation(); sensors_and_mounts();
  p266_basket_without_p251_front_posts(); p251_feed_exact();
  p253_power(); p253_frozen_launcher(angle); p253_actuator(angle);
  p253_as5600(); p253_stops(); p253_cables(angle); p266_fixed(angle,servo);
}

module p266_gate_sweep(){
  p266_front_wall_fixed();
  for(a=[0:10:90])color([1,.35,.05,.16])translate([278,0,p266_wall_top])
    rotate([0,a,0])translate([0,0,-(p266_wall_top-p266_door_bottom)/2])
      cube([4,344,p266_wall_top-p266_door_bottom],center=true);
}

if(p266_view=="assembly")p266_assembly(p266_angle,true);
else if(p266_view=="gate_sweep")p266_gate_sweep();
else if(p266_view=="launcher_sweep_qa")union(){for(a=[-10:1:20])intersection(){
  p253_frozen_launcher(a);
  union(){p266_front_wall_fixed();p266_soft_deflector();p266_gate(a,false);}
}}
else projection(cut=true)rotate([90,0,0])p266_assembly(p266_angle,true);
