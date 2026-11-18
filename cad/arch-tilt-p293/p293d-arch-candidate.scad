// P2-93e refinement of P2-93d. Existing chassis holes; tilt unchanged.
use <p293-arch-tilt-release.scad>
part="assembly";
side_mark="";
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

// P2-98 handed-part engraving.  The R glyph is pre-mirrored because the
// export pipeline mirrors the complete positive-hand mesh afterwards.
// A 0.7 mm cutter starting 0.1 mm outside the face leaves a 0.6 mm engraving.
module p298_mark_2d(label=side_mark){
 if(label=="R") mirror([1,0,0])
  text(label,size=8,halign="center",valign="center",font="Liberation Sans:style=Bold");
 else if(label=="L")
  text(label,size=8,halign="center",valign="center",font="Liberation Sans:style=Bold");
}
module p298_mark_pos_y(p,angle=0,label=side_mark){
 if(label=="L"||label=="R") translate(p) rotate([0,-angle,0]) rotate([90,0,0])
  linear_extrude(height=0.7) p298_mark_2d(label);
}
module p298_mark_pos_x(p,label=side_mark){
 if(label=="L"||label=="R") translate(p) rotate([0,-90,0]) rotate([0,0,90])
  linear_extrude(height=0.7) p298_mark_2d(label);
}

// Positive side installed coordinates. Mirrored as a complete part on -Y.
module p293d_foot(mark=side_mark){
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
  // Outer cheek, away from every bore/socket/contact face.
  p298_mark_pos_y([258,244.5,92],0,mark);
 }
}

// Solid 26 x 20 knee/arm, printed with the XZ load plane on the bed.
// Separate crown ports: 0.2 mm end clearance; two through-M4 carry the joint.
module p293d_knee(mark=side_mark){
 difference(){
  xzpoly_knee();
  for(z=[80,114]) yhole([258,0,z]);
  for(s=[L-55,L-35]) yhole(station(s));
  // Exposed side of the sloping arm, outside both pinned ends.
  p298_mark_pos_y(station(90)+[0,10.1,0],theta,mark);
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

module p293d_crown(mark=side_mark){
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
  // These clamp holes belong only to the 32.8 mm crown body.  The former
  // generic 80 mm bore continued through the inboard spigot and left a
  // constant 1.18 mm shell there.
  for(z=[239,251]) yhole([390,0,z],l=34.8);
  zhole([390,-26.4,245]);
  // Outer solid face; X382 clears the two central clamp bores.
  p298_mark_pos_y([382,16.5,245],0,mark);
 }
}

// Y100..211.6 ends against the crown face, not inside the crown.
module p293d_crossbar_end(mark=side_mark){
 difference(){
  union(){
   translate([390,155.8,245]) rotate([0,0,90]) tube_x(111.6,[20,26]);
   for(y=[146,154]) translate([390,y,245]) cube([20,9,26],center=true);
  }
  zhole([390,112,245]);
  zhole([390,201.6,245]);
  for(y=[146,154]) zhole([390,y,245]);
  // Front tube wall between the bolt stations.
  p298_mark_pos_x([400.1,175,245],mark);
 }
}

// Mast starts above the beam at Z258; integral shoe is bolted through it.
module p293d_mast(mark=side_mark){
 difference(){
  union(){
   translate([390,150,262]) cube([30,30,8],center=true);
   translate([390,150,305]) rotate([0,90,0]) tube_x(78,[24,24],3);
  }
  // Socket-head screws fit inside the open 18 mm mast lumen; no top washer.
  for(y=[146,154]) zhole([390,y,262]);
  xhole([390,150,336]);
  // Front tube wall, clear of the shoe and cap pin.
  p298_mark_pos_x([402.1,150,305],mark);
 }
}

module p293d_lidar_half(mark=side_mark){
 difference(){
  translate([390,69,354]) rotate([0,0,90]) tube_x(138,[24,20],3);
  zhole([390,11,354]);
  zhole([390,128,354]);
  xhole([390,35,354]);
  // Front wall, clear of the OAK hole at Y35 and both sleeve pins.
  p298_mark_pos_x([402.1,78,354],mark);
 }
}

module p293d_mast_cap(mark=side_mark){
 difference(){
  union(){
   translate([390,150,354]) cube([24,24,20],center=true);
   // Bridge both outside cheeks above the mast's Z344 end face.
   translate([390,150,348]) cube([32,24,8],center=true);
   for(x=[375.8,404.2]) translate([x,150,341]) cube([3.6,16,22],center=true);
   // Elliptical spigot occupies the bar's real void, with 0.5 mm gap.
   // Keep the released insertion end at Y118, but overlap the cap body by
   // 2 mm instead of ending tangent at Y138.  The tangent ellipse created a
   // detached first slice at print Z17.7 even though the final STL was one body.
   translate([390,129,354]) rotate([0,0,90])
     difference(){
      scale([1,17/2,13/2]) rotate([0,90,0]) cylinder(d=2,h=22,center=true,$fn=47);
      scale([1,10/2,6/2]) rotate([0,90,0]) cylinder(d=2,h=22,center=true,$fn=47);
     }
  }
  xhole([390,150,336]);
  zhole([390,128,354]);
  // The vertical bore completely split the lowest elliptical chord.  Remove
  // the free underside up to Z348.5; two printable rails below are added
  // outside this difference and support the replacement chord.
  translate([390,128,343.1]) cube([20,20,10.8],center=true);
  // Outboard body face, not the mast/spigot contact surfaces.
  p298_mark_pos_y([390,162.1,354],0,mark);
 }
 // Two 1.6 mm rails sit just outside the Ø4.6 bore (inner edges at X+/-2.4),
 // remain inside the ellipse at Z348.3, and overlap the cap over Y138..140.
 // Each starts one layer before the replacement chord and is layer-supported
 // through that cap overlap, so the nose never starts as a floating region.
 for(side=[-1,1])
  translate([390+side*3.2,129,348.5]) cube([1.6,22,0.4],center=true);
}

// OAK plate + integral forward struts. Pads butt at the LiDAR beam X402.
module p293d_oak_plate(){
 difference(){
  hull()for(x=[-48,48],y=[-18,18])translate([x,y,3])cylinder(d=12,h=6,center=true);
  // Keep the released 4.6 mm M4 clearance.  Shortening the vertical slot
  // travel by 0.2 mm at each end raises its ligament to the horizontal slots
  // from the former constant 1.4 mm to 1.6 mm.
  for(y=[-14,14])hull()for(x=[-40,40])translate([x,y,3])cylinder(d=m4d,h=10,center=true);
  for(x=[-30,30])hull()for(y=[-7.8,7.8])translate([x,y,3])cylinder(d=m4d,h=10,center=true);
 }
}

module p293d_oak_mount(){
 difference(){
  union(){
   translate([450,0,305]) rotate([0,50,0]) p293d_oak_plate();
   for(s=[-1,1]){
    translate([406,s*35,354]) cube([8,12,16],center=true);
    hull(){
     // A finite outer-face ring, rather than a tangent sphere at the bore,
     // leaves 4.2-2.3 = 1.9 mm radial material around the M4 hole.
     translate([410,s*35,354])rotate([0,90,0])cylinder(r=4.2,h=4,center=true);
     // Enter the plate outside the Y=+/-14 horizontal slots.  Local Y=22
     // leaves 1.7 mm between the D8 strut foot and the D4.6 slot edge.
     translate([450,s*22,305])sphere(r=4);
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
 translate([390,0,354]) rotate([0,0,90])
  joint_sleeve_x([24,20],44,[-11,11]);
 p293d_oak_mount();
}

if(part=="p293d_foot") translate([-192,-133,-42]) p293d_foot();
else if(part=="p293d_knee") translate([-245,72,10]) rotate([90,0,0]) p293d_knee();
else if(part=="p293d_crown") translate([-340,200,36.4]) rotate([90,0,0]) p293d_crown();
else if(part=="p293d_crossbar_end") translate([-380,-100,-232]) p293d_crossbar_end();
else if(part=="p293d_mast") translate([-375,-135,-258]) p293d_mast();
else if(part=="p293d_lidar_half") translate([-378,0,-344]) p293d_lidar_half();
else if(part=="p293d_mast_cap") translate([-374,-118,-330]) p293d_mast_cap();
// Put the universal plate on the bed.  The old installed-angle export began
// on a line and produced 0.2 mm-layer knife edges; the complete mount is
// rotated as one rigid part, so its interfaces are unchanged.
else if(part=="p293d_oak_mount")
 union(){
  translate([128.519,0,16.416]) rotate([0,180,0])
   translate([78.137,0,-55.280]) rotate([0,-50,0])
    translate([-402,41,-260]) p293d_oak_mount();
  // The inclined struts otherwise begin as two detached bed islands before
  // they meet the plate.  Permanent 2.4 mm print-plane gussets join each ear
  // to the plate through the first 3.4 mm; they are part geometry, not support.
  for(side=[-1,1])hull(){
   translate([104,41+side*21,1.8])cylinder(d=4,h=3.6,center=true,$fn=24);
   translate([122,41+side*35,1.8])cylinder(d=4,h=3.6,center=true,$fn=24);
   // The rotated strut first appears near X110.5, Y41+/-33.3 at Z1.1.
   // Include that footprint in the same bed gusset instead of letting a
   // 4 mm2 strip begin independently in mid-air.
   translate([110.5,41+side*33.3,1.8])cylinder(d=5,h=3.6,center=true,$fn=24);
  }
 }
else p293d_arch_assembly();
