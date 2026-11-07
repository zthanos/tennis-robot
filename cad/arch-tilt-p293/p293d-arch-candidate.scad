// P2-93e refinement of P2-93d. Existing chassis holes; tilt unchanged.
use <p293-arch-tilt-release.scad>
part="assembly";
$fn=48;
m4d=4.6;
K=[258,120]; C=[390,245];
theta=atan2(C[1]-K[1],C[0]-K[0]);
L=sqrt(pow(C[0]-K[0],2)+pow(C[1]-K[1],2));
function station(s)=[K[0]+s*cos(theta),0,K[1]+s*sin(theta)];

module yhole(p,d=m4d,l=80){translate(p) rotate([90,0,0]) cylinder(d=d,h=l,center=true);}
module zhole(p,d=m4d,l=80){translate(p) cylinder(d=d,h=l,center=true);}
module xhole(p,d=m4d,l=80){translate(p) rotate([0,90,0]) cylinder(d=d,h=l,center=true);}
module along(s0,s1,w=20,h=26){
  translate(station((s0+s1)/2)) rotate([0,-theta,0]) cube([s1-s0,w,h],center=true);
}
module xzpoly(points,w){rotate([90,0,0]) linear_extrude(height=w,center=true) polygon(points);}

// Positive side installed coordinates. Mirrored as a complete part on -Y.
module p293d_foot(){
 difference(){
  union(){
   hull(){
    translate([240,150,70]) cube([24,34,36],center=true);
    translate([258,228,62]) cube([26,28,20],center=true);
   }
   // X240 crossbar/socket top is Z42, ten below the rail top Z52.
   translate([240,150,47]) cube([24,34,10],center=true);
   // Existing THROUGH rail/sleeve lock: X230/Y220, NOT the two blind
   // 3.4 mm legacy holes at Y207.5/232.5. +20 mm TOTAL grip for M4x60:
   // 18 mm PETG + two added 12x4.3x1 steel washers; old washers retained.
   hull(){
    // Rear sole reaches X192 for reverse pitch compression at X196.
    // Above the existing rail only, outside basket Y172; no new bolt.
    translate([218,220,62]) cube([52,28,20],center=true);
    translate([258,228,62]) cube([26,28,20],center=true);
   }
   for(s=[-1,1]) translate([258,228+s*13.4,92]) cube([26,6,56],center=true);
  }
  for(y=[150,160]) zhole([240,y,62]);
  // ISO 7089 OD9 washers fit the 10 mm pitch. Recess bottom Z62:
  // 20 mm foot at each old crossbar hole, M4x30 -> M4x50.
  for(y=[150,160]) translate([240,y,62]) cylinder(d=9.6,h=40);
  zhole([230,220,62]);
  translate([230,220,70]) cylinder(d=13,h=35);
  // Real open socket, not a QA exclusion: the knee bottoms at Z72.
  translate([258,228,100]) cube([26.8,20.8,56],center=true);
  for(z=[80,114]) yhole([258,228,z]);
 }
}

// Solid 26 x 20 knee/arm, printed with the XZ load plane on the bed.
// Separate crown ports: 0.2 mm end clearance; two through-M4 carry the joint.
module p293d_knee(){
 difference(){
  xzpoly_knee();
  for(z=[80,114]) yhole([258,0,z]);
  for(s=[L-55,L-35]) yhole(station(s));
 }
}

// Closing + opening preserves the envelope and rounds root/external corners
// with R3 in the XZ load plane; bolt bores are cut afterwards.
module xzpoly_knee(){
 rotate([90,0,0]) linear_extrude(height=20,center=true)
 offset(r=3) offset(delta=-3) offset(delta=-3) offset(r=3)
 union(){
  translate([245,72]) square([26,48]);
  translate(K) rotate(theta) translate([-4,-13]) square([L-26,26]);
  polygon([[245,90],[271,112],[290,143],[245,130]]);
 }
}

module p293d_crown(){
 difference(){
  union(){
   translate([390,0,245]) cube([26,32.8,26],center=true);
   along(L-29.8,L,32.8);
   for(y=[-13.4,13.4]) translate([0,y,0]) along(L-65,L-25,6);
   // Inboard spigot fits the end-beam void, not its material.
   translate([390,-26.4,245]) rotate([0,0,90])
    difference(){
     scale([1,13/2,19/2]) rotate([0,90,0]) cylinder(d=2,h=20,center=true,$fn=47);
     scale([1,7/2,13/2]) rotate([0,90,0]) cylinder(d=2,h=22,center=true,$fn=47);
    }
  }
  for(s=[L-55,L-35]) yhole(station(s));
  // Two access-through holes for a future crown/end clamp; no hidden screw.
  for(z=[239,251]) yhole([390,0,z]);
  zhole([390,-26.4,245]);
 }
}

// Y100..211.6 ends against the crown face, not inside the crown.
module p293d_crossbar_end(){
 difference(){
  union(){
   translate([390,155.8,245]) rotate([0,0,90]) tube_x(111.6,[20,26]);
   for(y=[146,154]) translate([390,y,245]) cube([20,9,26],center=true);
  }
  zhole([390,112,245]);
  zhole([390,201.6,245]);
  for(y=[146,154]) zhole([390,y,245]);
 }
}

// Mast starts above the beam at Z258; integral shoe is bolted through it.
module p293d_mast(){
 difference(){
  union(){
   translate([390,150,262]) cube([30,30,8],center=true);
   translate([390,150,305]) rotate([0,90,0]) tube_x(78,[24,24],3);
  }
  // Socket-head screws fit inside the open 18 mm mast lumen; no top washer.
  for(y=[146,154]) zhole([390,y,262]);
  xhole([390,150,336]);
 }
}

module p293d_lidar_half(){
 difference(){
  translate([390,69,354]) rotate([0,0,90]) tube_x(138,[24,20],3);
  zhole([390,11,354]);
  zhole([390,128,354]);
  xhole([390,35,354]);
 }
}

module p293d_mast_cap(){
 difference(){
  union(){
   translate([390,150,354]) cube([24,24,20],center=true);
   // Bridge both outside cheeks above the mast's Z344 end face.
   translate([390,150,348]) cube([32,24,8],center=true);
   for(x=[375.8,404.2]) translate([x,150,341]) cube([3.6,16,22],center=true);
   // Elliptical spigot occupies the bar's real void, with 0.5 mm gap.
   translate([390,128,354]) rotate([0,0,90])
     difference(){
      scale([1,17/2,13/2]) rotate([0,90,0]) cylinder(d=2,h=20,center=true,$fn=47);
      scale([1,10/2,6/2]) rotate([0,90,0]) cylinder(d=2,h=22,center=true,$fn=47);
     }
  }
  xhole([390,150,336]);
  zhole([390,128,354]);
 }
}

// OAK plate + integral forward struts. Pads butt at the LiDAR beam X402.
module p293d_oak_mount(){
 difference(){
  union(){
   translate([450,0,305]) rotate([0,50,0]) oak_universal_plate();
   for(s=[-1,1]){
    translate([406,s*35,354]) cube([8,12,16],center=true);
    hull(){
     translate([406,s*35,354]) sphere(r=4);
     translate([448,s*14,303]) sphere(r=4);
    }
   }
  }
  for(s=[-1,1]) xhole([406,s*35,354]);
 }
}

module p293d_arch_assembly(include_feet=true){
 for(side=[-1,1]){
  if(include_feet) if(side>0) p293d_foot(); else mirror([0,1,0]) p293d_foot();
  translate([0,side*228,0]) p293d_knee();
  translate([0,side*228,0]) if(side>0) p293d_crown(); else mirror([0,1,0]) p293d_crown();
  if(side>0){p293d_crossbar_end();p293d_mast();p293d_lidar_half();p293d_mast_cap();}
  else mirror([0,1,0]){p293d_crossbar_end();p293d_mast();p293d_lidar_half();p293d_mast_cap();}
 }
 translate([390,0,245]) rotate([0,0,90]) arch_crossbar_center();
 for(side=[-1,1]) translate([390,side*100,245]) rotate([0,0,90]) joint_sleeve_x([20,26],48);
 translate([390,0,354]) rotate([0,0,90]) joint_sleeve_x([24,20],44);
 p293d_oak_mount();
}

if(part=="p293d_foot") translate([-192,-133,-42]) p293d_foot();
else if(part=="p293d_knee") translate([-245,72,10]) rotate([90,0,0]) p293d_knee();
else if(part=="p293d_crown") translate([-340,200,36.4]) rotate([90,0,0]) p293d_crown();
else if(part=="p293d_crossbar_end") translate([-380,-100,-232]) p293d_crossbar_end();
else if(part=="p293d_mast") translate([-375,-135,-258]) p293d_mast();
else if(part=="p293d_lidar_half") translate([-378,0,-344]) p293d_lidar_half();
else if(part=="p293d_mast_cap") translate([-374,-118,-330]) p293d_mast_cap();
else if(part=="p293d_oak_mount") translate([-402,41,-260]) p293d_oak_mount();
else p293d_arch_assembly();
