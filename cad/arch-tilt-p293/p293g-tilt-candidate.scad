// P2-93g candidates. No QA mating exclusions. All units mm.
// Purchased hardware dimensions are provisional fit-check envelopes.
use <p293d-arch-candidate.scad>
use <p293-arch-tilt-release.scad>
part="assembly";
angle=0;
centres=60;
PZ=51.837363636;
B=[260,-205,220];
function gp(a,p)=[387+p[0]*cos(a)-p[2]*sin(a),p[1],PZ+p[0]*sin(a)+p[2]*cos(a)];
function vlen(v)=sqrt(v*v);
function uu(a)=let(d=gp(a,[30,-205,110])-B)d/vlen(d);
function nn(a)=let(u=uu(a))[-u[2],0,u[0]];
function dd(c)=[c*sqrt(11)/6,-c*5/6,0];
function max_L()=max([for(a=[-10:.1:20])vlen(gp(a,[30,-205,110])-B)]);
module screw_pose(a,p=B){
 u=uu(a);n=nn(a);
 multmatrix([[n[0],0,u[0],p[0]],[0,-1,0,p[1]],[n[2],0,u[2],p[2]],[0,0,0,1]])children();
}
module moving_pose(a){translate([387,0,PZ])rotate([0,-a,0])children();}
module yh(p,d=4.6,l=100){translate(p)rotate([90,0,0])cylinder(d=d,h=l,center=true,$fn=48);}
module zh(p,d=4.6,l=100){translate(p)cylinder(d=d,h=l,center=true,$fn=48);}
module rod(a,b,d=12){hull(){translate(a)sphere(d=d,$fn=32);translate(b)sphere(d=d,$fn=32);}}
// Actual R3 re-entrant root fillet, NOT a blanket stress multiplier on stock.
module root_fillet(p,a,b,width,r=3){
 n=cross(a,b);
 multmatrix([[a[0],b[0],n[0],p[0]],[a[1],b[1],n[1],p[1]],[a[2],b[2],n[2],p[2]],[0,0,0,1]])
 linear_extrude(width,center=true)difference(){square([r,r]);translate([r,r])circle(r=r,$fn=24);}
}

// Base hole centres/template retained; outer sole edge clipped atY240.
// Pad widening begins above Z52 with45deg sides.
// Seat stays Z70, so18mm PETG +2x1mm washers still fits M4x60.
module p293g_foot(){
 $fn=48;
 intersection(){
 translate([250,0,180])cube([500,480,360],center=true);
 difference(){
  union(){
   intersection(){p293d_foot();translate([250,200,7])cube([500,400,130],center=true);}
   hull(){
    translate([218,220,52.05])cube([52,28,.1],center=true);
    translate([218,220,65])cube([52,40,14],center=true);
   }
   //8.2mm cheeks,0.2mm per-side socket clearance; same36.8mm outside span.
   for(s=[-1,1])translate([258,221.6+s*14.3,92])cube([26,8.2,56],center=true);
  }
  zh([230,220,62]);
  translate([230,220,70])cylinder(d=14.6,h=35,$fn=64);
  translate([258,221.6,100])cube([26.8,20.4,56],center=true);
  // M4 axes unchanged. Steel OD8 spacers carry shear into the cheeks.
  // Outboard5mm spacer below3.2mm button-head/washer recess; inboard8mm.
  for(z=[80,114]){
   yh([258,221.6,z],8.2);
   yh([258,238.4,z],10.6,3.2);
  }
 }
 }
}
// F's sloping front return is locally inboard of its main Y242 face.
// Replace the low outboard crown lug with a12x26mm inboard-only lug.
// Crown pins:10.9 button-head M4; threaded-root single-sided bending screened.
module p293g_crown(){
 $fn=48;
 difference(){
  union(){
   difference(){
    p293d_crown();
    translate([275,60.4,200])cube([200,100,200],center=true);
   }
   translate([0,-16.4,0])along_local(116.79,156.79,12);
  }
  // Same original knee/crown pin axes; L=sqrt(132^2+125^2).
  for(s=[sqrt(132*132+125*125)-55,sqrt(132*132+125*125)-35])
   yh([258+s*132/sqrt(132*132+125*125),0,120+s*125/sqrt(132*132+125*125)]);
  // PHYSICAL printed lower-edge rod relief, exported in STL. Not a QA mask.
  // Mirrored actuator plane is localY-16.6; d12.6 gives >2mm to actual d8.
  for(a=[-10:1:20])screw_pose(a,[260,-16.6,220])
   translate([0,0,-45])cylinder(d=12.6,h=max_L()+75,$fn=48);
 }
}
module along_local(s0,s1,w){
 t=atan2(125,132);
 translate([258+(s0+s1)/2*cos(t),0,120+(s0+s1)/2*sin(t)])rotate([0,-t,0])cube([s1-s0,w,26],center=true);
}
// Same knee solid, installed6.4mm inboard. Crown follows it unchanged.
// Only the end beam is shortened6.4mm; its end bolt follows the spigot.
module p293g_crossbar_end(){
 $fn=48;
 difference(){
  union(){
   translate([390,152.6,245])rotate([0,0,90])tube_x(105.2,[20,26]);
   for(y=[146,154])translate([390,y,245])cube([20,9,26],center=true);
  }
  for(y=[112,195.2,146,154])zh([390,y,245]);
 }
}
module p293g_arch_assembly(){
 $fn=48;
 for(side=[-1,1]){
  translate([0,side*221.6,0])p293d_knee();
  translate([0,side*221.6,0])if(side>0)p293g_crown();else mirror([0,1,0])p293g_crown();
  if(side>0){p293g_crossbar_end();p293d_mast();p293d_lidar_half();p293d_mast_cap();}
  else mirror([0,1,0]){p293g_crossbar_end();p293d_mast();p293d_lidar_half();p293d_mast_cap();}
 }
 translate([390,0,245])rotate([0,0,90])arch_crossbar_center();
 for(side=[-1,1])translate([390,side*100,245])rotate([0,0,90])joint_sleeve_x([20,26],48);
 translate([390,0,354])rotate([0,0,90])joint_sleeve_x([24,20],44);
 p293d_oak_mount();
 p293g_foot();mirror([0,1,0])p293g_foot();
}

// Keyed two-piece saddle around existing knee bend. No new knee holes.
// Print cavityY20.5; closing each half0.25 gives actual flat contactY20.
// XZ cavity has0.25 clearance, retained as real positive keyed shoulders.
module knee_pocket(){
 $fn=48;
 rotate([90,0,0])linear_extrude(20.5,center=true)
 offset(delta=.25)projection(cut=false)rotate([-90,0,0])p293d_knee();
}
module saddle_half(s=1){
 translate([0,-s*.25,0])intersection(){
  difference(){
   translate([275.5,0,146])cube([77,36.5,52],center=true);
   knee_pocket();
   // Four bolts outside the actual solid member, alongY.
   for(p=[[242,131],[242,164],[304,128],[280,166]])yh([p[0],0,p[1]]);
  }
  translate([275.5,s*50.25,146])cube([100,100,80],center=true);
 }
}
module fixed_fork(){
 difference(){
  union(){
   translate([260,-205,179])cube([24,20,34],center=true);
   translate([260,-205,193])cube([24,56.8,10],center=true);
   for(y=[-230.4,-179.6]){
    translate([260,y,205])cube([20,6,24],center=true);
    translate([260,y,220])rotate([90,0,0])cylinder(d=24,h=6,center=true,$fn=48);
   }
  }
  for(y=[-230.4,-179.6])yh([260,y,220],4.6,8);
  // Button-head M4x16 + washer0.8 recessed within fork ears, away from F.
  for(y=[-231.8,-178.2])yh([260,y,220],9.6,3.2);
 }
}
module p293g_console_inboard(){
 union(){
  translate([0,-221.6,0])saddle_half(1);
  fixed_fork();
 }
}
module p293g_console_outboard(){
 difference(){
  translate([0,-221.6,0])saddle_half(-1);
  // Recess clamp screw heads within outerY239.6, not into F.
  for(p=[[242,131],[242,164],[304,128],[280,166]])yh([p[0],-237.6,p[1]],9.6,4);
 }
}

// Cradle local axes: X=n, Y=-worldY, Z=Tr8 axis u.
// Motor face u-20, KFL face u-17.5, floating608 centre u40.
module p293g_cradle(){
 $fn=48;
 difference(){
  union(){
   hull(){
    translate([0,0,-21.6])cube([32,56,8],center=true);
    for(c=[57,63])translate(dd(c)+[0,0,-24.1])cylinder(d=50,h=8,center=true);
   }
   // Positive-side longitudinal spines leave the fork/screw path open.
   for(v=[-18,18]){
    translate([26,v,8])cube([16,8,65],center=true);
    translate([18,v,0])cube([24,8,16],center=true);
    translate([0,v,0])rotate([90,0,0])cylinder(d=20,h=8,center=true);
   }
   translate([4.5,0,33])cube([59,48,6],center=true);
   translate([0,0,37])cylinder(d=42,h=14,center=true);
  }
  // Real metal clearance cavities, not QA volume exclusions.
  translate([0,0,-20])cylinder(d=18,h=45,center=true);
  for(v=[-1,1])hull()for(pitch=[36,48])translate([0,v*pitch/2,-22])cylinder(d=5.4,h=16,center=true);
  hull()for(c=[57,63])translate(dd(c)+[0,0,-25])cylinder(d=18.5,h=24,center=true);
  hull()for(c=[57,63])translate(dd(c)+[0,0,-20.1])cylinder(d=37.6,h=100);
  // Four of the six FIT M3 face holes; swept slots do not merge with boss bore.
  for(a=[0,60,180,240])hull()for(c=[57,63])translate(dd(c)+[15.5*cos(a),15.5*sin(a),-24])cylinder(d=3.4,h=16,center=true);
  for(v=[-18,18])yh([0,v,0],4.6,12);
  translate([0,0,29])cylinder(d=8.6,h=20);
  translate([0,0,36])cylinder(d=22.4,h=14);
  for(a=[45,135,225,315])translate([16*cos(a),16*sin(a),29])cylinder(d=3.4,h=20);
 }
}
module p293g_float_cap(){
 difference(){
  translate([0,0,46.1])cylinder(d=42,h=4,center=true,$fn=48);
  translate([0,0,43])cylinder(d=18,h=8,$fn=48);
  for(a=[45,135,225,315])translate([16*cos(a),16*sin(a),43])cylinder(d=3.4,h=8,$fn=48);
 }
}

module p293g_nut_trunnion(){
 $fn=48;
 difference(){
  union(){
   translate([0,0,-6.1])cube([32,38,8],center=true);
   for(v=[-16.8,16.8])translate([0,v,0])rotate([90,0,0])cylinder(d=14,h=4,center=true);
  }
  translate([0,0,-15])cylinder(d=10.6,h=30);
  for(a=[45,135,225,315])translate([8*cos(a),8*sin(a),-15])cylinder(d=3.4,h=20);
  for(v=[-16.8,16.8])yh([0,v,0],4.6,5);
  // Insert inward-facing button-head stub screws BEFORE fitting the T8 nut.
  // Real head pockets in plate shoulders; nut flangeR11 stays outside them.
  // Continuous head/tool insertion channel from central bore to both pockets.
  yh([0,0,0],8.4,29.6);
 }
}

// Rigid moving arm attaches to actual yoke topW47, not into its solid.
// Low supported U-clevis: crossbar below the nut, open above for installation.
module p293g_nut_arm(){
 difference(){
  union(){
   //0.1mm nominal seating clearance; no transformed coplanar mesh overlap.
   // Four through-fasteners close the interface during the mandatory coupon.
   translate([-5,-205,53.1])cube([40,52,12],center=true);
   // Clear OD10 mounting washers; local U/W biaxial forces are screened.
   for(y=[-195,-213])translate([-10,y,68])cube([30,10,24],center=true);
   translate([10,-205,80])cube([40,30,16],center=true);
   translate([30,-205,80])cube([24,46.4,14],center=true);
   for(y=[-226.2,-183.8]){
    translate([30,y,95])cube([24,4,30],center=true);
    translate([30,y,110])rotate([90,0,0])cylinder(d=20,h=4,center=true,$fn=48);
   }
   for(y=[-195,-213]){
    root_fillet([5,y,59.1],[1,0,0],[0,0,1],10);
    root_fillet([5,y,72],[1,0,0],[0,0,-1],10);
    root_fillet([-10,y,80],[-1,0,0],[0,0,1],10);
   }
   for(y=[-226.2,-183.8])root_fillet([18,y,87],[-1,0,0],[0,0,1],4);
   root_fillet([30,-185.8,87],[0,-1,0],[0,0,1],24);
   root_fillet([30,-224.2,87],[0,1,0],[0,0,1],24);
  }
  for(u=[-15,5],y=[-224,-184])zh([u,y,52],4.6,24);
  for(y=[-226.2,-183.8])yh([30,y,110],4.6,6);
 }
}
module p293g_yoke_mount_bores(a){moving_pose(a)for(u=[-15,5],y=[-224,-184])zh([u,y,43],4.6,10);}

// Purchased hardware envelopes, internal threads not manufacturing geometry.
module p293g_fit_hardware(){
 $fn=48;
 translate(dd(centres)){
  difference(){
   union(){translate([0,0,-20])cylinder(d=37,h=17);translate([0,0,-3])cylinder(d=37,h=70);}
   for(a=[0:60:300])translate([15.5*cos(a),15.5*sin(a),-20.1])cylinder(d=3,h=3.1);
  }
  translate([0,0,-35.5])cylinder(d=6,h=15.5);
 }
}
module p293g_kfl08_hardware(){
 $fn=48;
 difference(){
  union(){
   hull()for(v=[-19,19])translate([0,v,-17.5])cylinder(d=10,h=4);
   translate([0,0,-17.5])cylinder(d=27,h=12);
  }
  translate([0,0,-18])cylinder(d=8.05,h=14);
  for(v=[-18,18])translate([0,v,-18])cylinder(d=5,h=6);
 }
}
module p293g_608_hardware(){
 difference(){translate([0,0,36.5])cylinder(d=22,h=7,$fn=48);translate([0,0,36])cylinder(d=8.05,h=8,$fn=48);}
}
module p293g_tr8_hardware(){translate([0,0,-45])cylinder(d=8,h=max_L()+75,$fn=48);}
module p293g_t8_nut_hardware(){
 $fn=48;
 difference(){
  union(){translate([0,0,-12])cylinder(d=10,h=30);translate([0,0,-2])cylinder(d=22,h=4);}
  translate([0,0,-13])cylinder(d=8.2,h=32);
  for(a=[45,135,225,315])translate([8*cos(a),8*sin(a),-3])cylinder(d=3.2,h=6);
 }
}
module p293g_collar_hardware(s=1){
 z=s<0?-24:-5;
 difference(){translate([0,0,z])cylinder(d=16,h=6,$fn=48);translate([0,0,z-1])cylinder(d=8.1,h=8,$fn=48);}
}
module p293g_thrust_washer_hardware(s=1){
 z=s<0?-18:-5.5;
 difference(){translate([0,0,z])cylinder(d=10.5,h=.5,$fn=48);translate([0,0,z-.1])cylinder(d=8.2,h=.7,$fn=48);}
}
module p293g_pulley_hardware(motor=false){
 $fn=48;
 translate(motor?dd(centres):[0,0,0])difference(){
  union(){
   translate([0,0,-41])cylinder(d=12,h=8);
   for(z=[-42,-33])translate([0,0,z])cylinder(d=18,h=1);
   translate([0,0,-32])cylinder(d=16,h=7);
  }
  translate([0,0,-42])cylinder(d=motor?6.2:8.2,h=20);
 }
}
module p293g_belt_hardware(){
 $fn=48;
 difference(){
  hull()for(p=[[0,0,0],dd(centres)])translate(p+[0,0,-40])cylinder(r=7.5,h=6);
  hull()for(p=[[0,0,0],dd(centres)])translate(p+[0,0,-41])cylinder(r=6.3,h=8);
 }
}

module p293g_tilt_assembly(a=angle){
 p293g_console_inboard();p293g_console_outboard();
 screw_pose(a)p293g_cradle();screw_pose(a)p293g_float_cap();
 moving_pose(a)p293g_nut_arm();
 screw_pose(a,gp(a,[30,-205,110]))p293g_nut_trunnion();
 // Separate call sites: purchased components are not merged into one QA body.
 screw_pose(a)p293g_fit_hardware();screw_pose(a)p293g_kfl08_hardware();
 screw_pose(a)p293g_608_hardware();screw_pose(a)p293g_tr8_hardware();
 screw_pose(a,gp(a,[30,-205,110]))p293g_t8_nut_hardware();
 for(s=[-1,1])screw_pose(a)p293g_collar_hardware(s);
 for(s=[-1,1])screw_pose(a)p293g_thrust_washer_hardware(s);
 for(motor=[false,true])screw_pose(a)p293g_pulley_hardware(motor);
 screw_pose(a)p293g_belt_hardware();
}

// Candidate exports, never overwrite the historical p293d/e STL or templates.
if(part=="p293g_foot")translate([-192,-133,-42])p293g_foot();
else if(part=="p293g_crossbar_end")translate([-380,-100,-232])p293g_crossbar_end();
else if(part=="p293g_crown")translate([-340,200,36.4])rotate([90,0,0])p293g_crown();
else if(part=="p293g_console_inboard")translate([-237,250,-120])p293g_console_inboard();
else if(part=="p293g_console_outboard")translate([-237,247,-120])p293g_console_outboard();
else if(part=="p293g_cradle")translate([30,80,35])p293g_cradle();
else if(part=="p293g_float_cap")translate([25,25,-44])p293g_float_cap();
else if(part=="p293g_nut_trunnion")translate([20,25,12])p293g_nut_trunnion();
else if(part=="p293g_nut_arm")translate([25,240,-47])p293g_nut_arm();
else if(part=="assembly"){p293g_arch_assembly();p293g_tilt_assembly(angle);}
else assert(false,str("unknown P2-93g part: ",part));
