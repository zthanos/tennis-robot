// P2-93 first release wave: printable P2-67 arch core and P2-53/54 tilt hardware.
// Units mm; +X front, +Y left, +Z up. PETG, through fasteners only.
// Hardware dimensions are manufacturer nominal until the user's fit check.

use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-transition-p252.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/shared-flywheel-p273-bolted-ski.scad>
use <../rear-pod/rear-pod-v2.scad>
use <../body-panels/body-panels-f.scad>

part = "assembly";
angle = 0;
$fn = 48;

m4 = 4.6;
m3 = 3.4;
m6 = 6.6;
fit_clear = 0.35;

arch_base = [318,228,67];              // 2 mm socket floor over chassis top
arch_top = [390,228,245];
arch_split_t = 0.52;
arch_section = [20,26];              // faces Y218/Y238: margin to moving and F
brace_section = [26,22];
crossbar_half = 220;
mast_z0 = 245;
mast_z1 = 354;

// Manufacturer nominal / fit-check parameters.
fit_body_d = 37;
fit_face_pcd = 31;                   // adjacent M3 holes 15.5 on regular hexagon
fit_shaft_d = 6;
belt_centres_nominal = 60;
belt_adjust = 3;                     // 57..63 mm; tensioning included
kfl08_hole_pitch = 48;
kfl08_bore = 8;
nut_flange_d = 22;
nut_hole_pitch = 16;
as5600_board = [23,23,3];
magnet = [6,2.5];
magnet_gap = 1.5;
pivot_p293 = [387,0,55.84];            // frozen P2-40/P2-54 hinge datum

function lerp3(a,b,t) = [for(i=[0:2]) a[i]+(b[i]-a[i])*t];
function vlen3(v) = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
function sub3(a,b) = [for(i=[0:2]) a[i]-b[i]];

module axis_hole(axis="z", d=m4, l=100) {
  if(axis=="z") cylinder(d=d,h=l,center=true);
  else if(axis=="y") rotate([90,0,0]) cylinder(d=d,h=l,center=true);
  else rotate([0,90,0]) cylinder(d=d,h=l,center=true);
}

// Rectangular outside, smooth elliptical roof inside: no flat bridge and no
// pointed aperture crown. Odd tessellation avoids a horizontal roof facet.
module tube_x(len, yz=[34,26], wall=3.2) {
  difference() {
    cube([len,yz[0],yz[1]],center=true);
    scale([1,(yz[0]-2*wall)/2,(yz[1]-2*wall)/2])
      rotate([0,90,0]) cylinder(d=2,h=len+2,center=true,$fn=47);
  }
}

module joint_tongue_x(x0, dir=1, yz=[34,26], len=24) {
  translate([x0+dir*len/2,0,-yz[1]/2+3.2+3.2])
    cube([len,yz[0]-7.2,6.4],center=true);
}

module joint_sleeve_x(yz=[34,26], len=48) {
  difference() {
    scale([1,(yz[0]-7.0)/2,(yz[1]-7.0)/2])
      rotate([0,90,0]) cylinder(d=2,h=len,center=true,$fn=47);
    scale([1,(yz[0]-13.0)/2,(yz[1]-13.0)/2])
      rotate([0,90,0]) cylinder(d=2,h=len+2,center=true,$fn=47);
    joint_holes_x([-12,12],yz);
  }
}

module joint_holes_x(xs, yz=[34,26]) {
  for(x=xs) translate([x,0,0]) axis_hole("z",m4,yz[1]+4);
}

module place_xz(p0,p1) {
  d=sub3(p1,p0); len=vlen3(d); a=atan2(d[2],d[0]);
  translate([(p0[0]+p1[0])/2,(p0[1]+p1[1])/2,(p0[2]+p1[2])/2])
    rotate([0,-a,0]) children();
}

module arch_foot_x240() {
  difference() {
    union() {
      // Broad plate spreads the brace force to both existing Y150/Y160 bores.
      hull() {
        // X228..252 clears the frozen P2-71 front grid at X225.
        translate([0,0,5]) cube([24,34,10],center=true);
        translate([0,78,5]) cube([24,24,10],center=true);
      }
      hull() {
        translate([0,50,8]) cube([24,20,12],center=true);
        translate([0,78,24]) cube([26,22,34],center=true);
      }
    }
    for(y=[0,10]) translate([0,y,5]) axis_hole("z",m4,14);
    translate([0,78,24]) rotate([0,90,0])
      scale([1,(brace_section[0]+0.6)/2,(brace_section[1]+0.6)/2])
        cylinder(d=2,h=60,center=true,$fn=47);
    // Open-top clevis: no hidden horizontal socket roof.
    translate([0,78,40]) cube([62,brace_section[0]+1,32],center=true);
    translate([0,78,24]) axis_hole("z",m4,60);
  }
}

module arch_foot_x300() {
  difference() {
    union() {
      hull() {
        // Entire foot is forward of the frozen P2-71 front grid at X225.
        translate([0,0,5]) cube([38,34,10],center=true);
        translate([18,78,5]) cube([48,24,10],center=true);
      }
      hull() {
        translate([8,48,8]) cube([34,22,12],center=true);
        translate([18,78,28]) cube([38,24,44],center=true);
      }
      // Continuous rear spine keeps both open clevis cheeks in one body.
      translate([8,60,15]) cube([76,8,30],center=true);
    }
    for(y=[0,10]) translate([0,y,5]) axis_hole("z",m4,14);
    // Separate sockets for the inclined leg and the X300 crown brace.
    translate([18,78,15]) rotate([0,90,0])
      scale([1,(arch_section[0]+0.6)/2,(arch_section[1]+0.6)/2])
        cylinder(d=2,h=70,center=true,$fn=47);
    translate([-2,78,25]) rotate([0,90,0])
      scale([1,(brace_section[0]+0.6)/2,(brace_section[1]+0.6)/2])
        cylinder(d=2,h=70,center=true,$fn=47);
    translate([18,78,35]) cube([72,arch_section[0]+1,44],center=true);
    translate([-2,78,42]) cube([72,brace_section[0]+1,34],center=true);
    // Remove the non-structural outer cusp left where the two open clevis
    // reliefs meet; otherwise it exports as a separate 0.11 cm3 sliver.
    translate([40,92,38]) cube([20,10,32],center=true);
    translate([18,78,15]) axis_hole("z",m4,60);
    translate([-2,78,25]) axis_hole("z",m4,60);
  }
}

function leg_split() = lerp3(arch_base,arch_top,arch_split_t);
function seg_len(a,b) = vlen3(sub3(b,a));

module arch_leg_lower() {
  l=seg_len(arch_base,leg_split());
  difference() {
    tube_x(l,arch_section);
    // One bolt on the lower side of the joint; it shares the -12 mm
    // sleeve station after the two segments are assembled end-to-end.
    joint_holes_x([l/2-12],arch_section);
    translate([-l/2+10,0,0]) axis_hole("z",m4,arch_section[1]+4);
  }
}

module arch_leg_upper() {
  l=seg_len(leg_split(),arch_top);
  difference() {
    union() {
      tube_x(l,arch_section);
      // Two integral S4 lands on the outboard face; M4 remains through-bolted.
      for(x=[-18,25]) hull() {
        translate([x,arch_section[0]/2-1,0]) cube([18,4,18],center=true);
        translate([x,arch_section[0]/2+2.5,0]) cube([18,3,18],center=true);
      }
      // In the open-end print pose local X is vertical. These 45-degree
      // roots support the lower edge of each S4 land without a flat bridge.
      for(x=[-18,25]) hull() {
        translate([x-13,arch_section[0]/2-1,0]) cube([2,4,18],center=true);
        translate([x-9,arch_section[0]/2+2.5,0]) cube([2,3,18],center=true);
      }
    }
    // One bolt on the upper side of the joint; it shares the +12 mm
    // sleeve station after the two segments are assembled end-to-end.
    joint_holes_x([-l/2+12],arch_section);
    for(x=[-18,25]) translate([x,arch_section[0]/2+2.5,0]) axis_hole("y",m4,18);
    translate([l/2-10,0,0]) axis_hole("z",m4,arch_section[1]+4);
  }
}

module arch_brace(len) {
  difference() {
    tube_x(len,brace_section,3.0);
    for(x=[-len/2+10,len/2-10]) translate([x,0,0]) axis_hole("z",m4,brace_section[1]+4);
  }
}

module arch_crossbar_center() {
  l=200;
  difference() {
    tube_x(l,arch_section);
    joint_holes_x([-l/2+12,l/2-12],arch_section);
  }
}

module arch_crossbar_end() {
  l=128;
  difference() {
    tube_x(l,arch_section);
    joint_holes_x([-l/2+12],arch_section);
    translate([l/2-10,0,0]) axis_hole("z",m4,arch_section[1]+4);
  }
}

module lidar_mast() {
  l=mast_z1-mast_z0;
  difference() {
    union() {
      tube_x(l,[24,24],3.0);
      // Two front-half lands per side, directed toward +X when installed.
      for(x=[-28,28]) hull() {
        translate([x,0,10]) cube([18,18,4],center=true);
        translate([x,0,18]) cube([18,18,8],center=true);
      }
      for(x=[-28,28]) hull() {
        translate([x-17,0,10]) cube([2,18,4],center=true);
        translate([x-9,0,18]) cube([2,18,8],center=true);
      }
    }
    for(x=[-28,28]) translate([x,0,18]) axis_hole("z",m4,12);
    for(x=[-l/2+10,l/2-10]) translate([x,0,0]) axis_hole("z",m4,28);
  }
}

module lidar_crossbar_half() {
  l=160;
  difference() {
    tube_x(l,[24,20],3.0);
    joint_holes_x([-l/2+11],[24,20]);
    translate([l/2-10,0,0]) axis_hole("z",m4,24);
  }
}

module oak_universal_plate() {
  difference() {
    hull() for(x=[-48,48],y=[-18,18]) translate([x,y,3]) cylinder(d=12,h=6,center=true);
    // Universal 20..80 mm horizontal and 20..32 mm vertical patterns.
    for(y=[-14,14]) hull() for(x=[-40,40]) translate([x,y,3]) cylinder(d=m4,h=10,center=true);
    for(x=[-30,30]) hull() for(y=[-8,8]) translate([x,y,3]) cylinder(d=m4,h=10,center=true);
  }
}

module fit0186_carrier() {
  difference() {
    union() {
      hull() for(x=[-42,42],y=[-18,18]) translate([x,y,5]) cylinder(d=12,h=10,center=true);
      // Raised rings spread motor/KFL loads; no thin mounting tabs.
      translate([30,0,8.5]) cylinder(d=52,h=17,center=true);
      translate([-30,0,8.5]) cylinder(d=46,h=17,center=true);
    }
    translate([30,0,8.5]) cylinder(d=fit_body_d+1.0,h=40,center=true);
    translate([-30,0,8.5]) cylinder(d=18,h=40,center=true);
    // Six 6 mm-long radial slots allow 57..63 mm centre distance.
    for(a=[0:60:300]) hull() for(dx=[-belt_adjust,belt_adjust])
      translate([30+dx+(fit_face_pcd/2)*cos(a),(fit_face_pcd/2)*sin(a),11])
        cylinder(d=m3,h=30,center=true);
    for(x=[-42,42],y=[-18,18]) translate([x,y,8.5]) cylinder(d=m4,h=40,center=true);
  }
}

module kfl08_base() {
  difference() {
    union() {
      hull() for(x=[-28,28],y=[-25,25]) translate([x,y,6]) cylinder(d=12,h=12,center=true);
      translate([0,0,13]) cylinder(d=38,h=14,center=true);
    }
    translate([0,0,10]) cylinder(d=28,h=30,center=true);
    for(x=[-kfl08_hole_pitch/2,kfl08_hole_pitch/2])
      hull() for(y=[-2,2]) translate([x,y,6]) cylinder(d=7,h=20,center=true);
    for(x=[-27,27],y=[-19,19]) translate([x,y,6]) cylinder(d=m4,h=20,center=true);
  }
}

module anti_backlash_carriage() {
  difference() {
    union() {
      translate([0,0,10]) cube([48,42,20],center=true);
      for(y=[-25,25]) translate([0,y,8]) cube([34,12,16],center=true);
    }
    translate([0,0,10]) cylinder(d=nut_flange_d+0.8,h=24,center=true);
    for(x=[-nut_hole_pitch/2,nut_hole_pitch/2],y=[-nut_hole_pitch/2,nut_hole_pitch/2])
      translate([x,y,10]) cylinder(d=m3,h=24,center=true);
    // Anti-rotation guide slots, adjustable with shims after real nut measurement.
    for(y=[-25,25]) translate([0,y,8]) cube([22,5.2,22],center=true);
  }
}

module upper_stop() {
  difference() {
    union() {
      translate([0,0,8]) cube([58,38,16],center=true);
      hull() {
        translate([10,0,16]) cube([30,38,2],center=true);
        translate([24,0,27]) cube([12,38,4],center=true);
      }
      // Non-load-bearing striker interface. A future replaceable tab bolts
      // here; hard-stop reaction remains on the shoulder at X=24.
      translate([-18,0,19]) cube([18,14,10],center=true);
    }
    for(x=[-18,18]) translate([x,0,8]) axis_hole("z",m6,40);
    // Replaceable 8 mm shoulder-pin seat; reaction bypasses the actuator.
    translate([24,0,25]) axis_hole("y",8.3,46);
    for(x=[-22,-14]) translate([x,0,20]) axis_hole("y",m3,20);
  }
}

module as5600_pod() {
  difference() {
    union() {
      hull() for(x=[-18,18],z=[-18,18]) translate([x,5,z+20]) rotate([90,0,0]) cylinder(d=10,h=10,center=true);
      translate([0,5,20]) rotate([90,0,0]) cylinder(d=36,h=10,center=true);
    }
    // Through-open board window; the separate cover closes it after fit check.
    translate([0,5,20]) cube([as5600_board[0]+0.8,12,as5600_board[1]+0.8],center=true);
    translate([0,5,20]) axis_hole("y",14,20);
    for(x=[-16,16]) hull() for(z=[-3,3]) translate([x,5,20+z]) axis_hole("y",m4,20);
  }
}

module as5600_cover() {
  difference() {
    hull() for(x=[-18,18],z=[-18,18]) translate([x,2,z+20]) rotate([90,0,0]) cylinder(d=10,h=4,center=true);
    for(x=[-16,16]) hull() for(z=[-3,3]) translate([x,2,20+z]) axis_hole("y",m4,10);
    translate([0,2,20]) axis_hole("y",12,10);
  }
}

module magnet_cap() {
  difference() {
    cylinder(d=18,h=8);
    translate([0,0,8-magnet[1]-magnet_gap]) cylinder(d=magnet[0]+0.25,h=magnet[1]+magnet_gap+1);
    cylinder(d=8.3,h=10);
  }
}

// The local plate X axis follows the 60 mm belt-centre line; local Z follows
// the nominal Tr8 axis. The two axes are perpendicular, so the pulleys are
// coplanar rather than merely 60 mm apart in plan view.
module fit_carrier_installed() {
  multmatrix([
    [-0.150,-0.132,0.980,295.7],
    [ 0.660,-0.740,0.000,-185.2],
    [ 0.737, 0.647,0.200,140.1],
    [ 0,     0,    0,    1]
  ]) fit0186_carrier();
}

module kfl08_installed() {
  multmatrix([
    [0,-0.200,0.980,310],
    [1, 0.000,0.000,-205],
    [0, 0.980,0.200,120],
    [0, 0,    0,    1]
  ]) kfl08_base();
}

module arch_release_assembly(include_lands=true) {
  for(side=[-1,1]) {
    // Feet use the existing X240/X300, Y150/Y160 top bores only.
    translate([240,side*150,52]) if(side>0) arch_foot_x240(); else mirror([0,1,0]) arch_foot_x240();
    translate([300,side*150,52]) if(side>0) arch_foot_x300(); else mirror([0,1,0]) arch_foot_x300();
    p0=[arch_base[0],side*arch_base[1],arch_base[2]];
    ps=[leg_split()[0],side*leg_split()[1],leg_split()[2]];
    p1=[arch_top[0],side*arch_top[1],arch_top[2]];
    place_xz(p0,ps) arch_leg_lower();
    place_xz(ps,p1) if(side>0) arch_leg_upper(); else mirror([0,1,0]) arch_leg_upper();
    place_xz([ps[0]-24*cos(atan2(p1[2]-ps[2],p1[0]-ps[0])),side*228,ps[2]-24*sin(atan2(p1[2]-ps[2],p1[0]-ps[0]))],
             [ps[0]+24*cos(atan2(p1[2]-ps[2],p1[0]-ps[0])),side*228,ps[2]+24*sin(atan2(p1[2]-ps[2],p1[0]-ps[0]))])
      joint_sleeve_x(arch_section,48);
    place_xz([240,side*228,76],[390,side*228,245]) arch_brace(seg_len([240,side*228,76],[390,side*228,245]));
    place_xz([300,side*228,77],[390,side*228,245]) arch_brace(seg_len([300,side*228,77],[390,side*228,245]));
    // Mast local X becomes world Z. Front lands point +X.
    translate([390,side*150,(mast_z0+mast_z1)/2]) rotate([0,90,0]) lidar_mast();
  }
  translate([390,0,245]) rotate([0,0,90]) arch_crossbar_center();
  for(side=[-1,1]) translate([390,side*100,245]) rotate([0,0,90]) joint_sleeve_x(arch_section,48);
  for(side=[-1,1]) translate([390,side*164,245]) rotate([0,0,side>0?90:-90]) arch_crossbar_end();
  for(side=[-1,1]) translate([390,side*80,mast_z1]) rotate([0,0,side>0?90:-90]) lidar_crossbar_half();
  translate([390,0,mast_z1]) rotate([0,0,90]) joint_sleeve_x([24,20],44);
  translate([408,0,305]) rotate([0,50,0]) oak_universal_plate();
}

module tilt_release_assembly(a=angle) {
  // Detailed parts remain inside the P2-54 validated actuator envelope.
  // Final local poses are shimmed after the user's hardware fit check.
  fit_carrier_installed();
  kfl08_installed();
  translate(gpoint(a,[50,-205,90])) anti_backlash_carriage();
  for(side=[-1,1]) translate(gpoint(20,[20,side*205,115])+[-72,0,-7])
    rotate([0,0,side>0?180:0]) upper_stop();
  translate([pivot_p293[0],244,pivot_p293[2]]) as5600_pod();
  translate([pivot_p293[0],238,pivot_p293[2]]) rotate([90,0,0]) magnet_cap();
}

module assembly() {
  color([0.16,0.48,0.78]) arch_release_assembly();
  color([0.70,0.35,0.18]) tilt_release_assembly(angle);
}

// Print orientation helpers. All long pieces fit <=250 mm in their longest axis.
module on_bed(z) { translate([0,0,z]) children(); }
// Hollow structural members print on an open end. This removes every long
// internal bridge; P2S Z=256 still contains the largest 248 mm tongue stack.
module on_open_end(len, extra=0) { translate([0,0,len/2+extra]) rotate([0,-90,0]) children(); }

if(part=="assembly") assembly();
else if(part=="arch_foot_x240") arch_foot_x240();
else if(part=="arch_foot_x300") arch_foot_x300();
else if(part=="arch_leg_lower") on_open_end(seg_len(arch_base,leg_split())) arch_leg_lower();
else if(part=="arch_leg_upper") on_open_end(seg_len(leg_split(),arch_top)) arch_leg_upper();
else if(part=="arch_joint_sleeve") on_open_end(48) joint_sleeve_x(arch_section,48);
else if(part=="sensor_joint_sleeve") on_open_end(44) joint_sleeve_x([24,20],44);
else if(part=="arch_brace_x240") on_open_end(seg_len([240,228,76],[390,228,245])) arch_brace(seg_len([240,228,76],[390,228,245]));
else if(part=="arch_brace_x300") on_open_end(seg_len([300,228,77],[390,228,245])) arch_brace(seg_len([300,228,77],[390,228,245]));
else if(part=="arch_crossbar_center") on_open_end(200) arch_crossbar_center();
else if(part=="arch_crossbar_end") on_open_end(128) arch_crossbar_end();
else if(part=="lidar_mast") on_open_end(mast_z1-mast_z0) lidar_mast();
else if(part=="lidar_crossbar_half") on_open_end(160) lidar_crossbar_half();
else if(part=="oak_universal_plate") oak_universal_plate();
else if(part=="tilt_fit0186_carrier") fit0186_carrier();
else if(part=="tilt_kfl08_base") kfl08_base();
else if(part=="tilt_nut_carriage") anti_backlash_carriage();
else if(part=="tilt_stop") upper_stop();
else if(part=="tilt_as5600_pod") rotate([90,0,0]) translate([0,0,5]) as5600_pod();
else if(part=="tilt_as5600_cover") rotate([90,0,0]) translate([0,0,2]) as5600_cover();
else if(part=="tilt_magnet_cap") magnet_cap();
else assert(false,str("unknown P2-93 part: ",part));

assert(seg_len(arch_base,leg_split()) < 250 && seg_len(leg_split(),arch_top) < 250,
       "P2-93 leg segments must fit a 250 mm bed");
assert(seg_len([240,228,76],[390,228,245]) < 250,
       "P2-93 X240 brace must fit a 250 mm bed");
assert(belt_centres_nominal-belt_adjust <= 60 && belt_centres_nominal+belt_adjust >= 60,
       "P2-93 FIT carrier must include the GT2-160 60 mm centre distance");
