// P2-86b — schematic perimeter/support map only; NOT printable panel CAD.
// Render `map_view="top"` and `map_view="side"` for the design note.

use <body-panels-f-rear-v2.scad>
use <body-panels-f-support.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-p267-tilted-arch.scad>

map_view = "top";
$fn = 48;

c_rear=[0.72,0.25,0.12];
c_s1=[0.95,0.55,0.08];
c_s2=[0.92,0.78,0.12];
c_s3=[0.25,0.70,0.36];
c_open=[0.88,0.12,0.18];
c_support=[0.05,0.80,0.95];

module top_bar(x0,x1,y,c) {
  color(c) translate([(x0+x1)/2,y,362]) cube([x1-x0,8,8],center=true);
}
module top_support(x,y,label="") {
  color(c_support) translate([x,y,360]) cylinder(d=13,h=14);
  color([0.02,0.28,0.38]) translate([x+7,y+4,369]) linear_extrude(1)
    text(label,size=10,halign="left",valign="bottom");
}
module top_label(x,y,t,c=[0.12,0.12,0.12]) {
  color(c) translate([x,y,369]) linear_extrude(1)
    text(t,size=12,halign="center",valign="center");
}
module top_scene() {
  color([0.25,0.30,0.34,0.24]) frame_assembly();
  // Actual released rear pieces include the R424.5 -> tangent R60 -> side.
  color(c_rear) for(s=[-1,1]) {
    rear_lo(s); rear_hi(s); deck_rear(s);
  }
  for(s=[-1,1]) {
    top_bar(-185,40,s*244.5,c_s1);
    top_bar(40,230,s*244.5,c_s2);
    top_bar(230,400,s*244.5,c_s3);
    top_bar(400,511,s*244.5,c_open);
  }
  // Front halves are separate <=164 mm spans and remain open pending arch fix.
  color(c_open) translate([511,0,362]) cube([8,328,8],center=true);

  for(s=[-1,1]) {
    top_support(-165,s*165,"R165");
    for(x=[-135,5]) top_support(x,s*220,str("S1@",x));
    for(x=[65,125,170,230]) top_support(x,s*220,str("S2@",x));
    top_support(240,s*160,"X240");
    top_support(300,s*160,"X300");
    top_support(390,s*220,"ARCH OPEN");
  }
  top_label(-250,0,"R-L / R-R");
  top_label(-70,265,"S1"); top_label(135,265,"S2");
  top_label(315,265,"S3"); top_label(455,265,"S4 OPEN",c_open);
  top_label(535,0,"FRONT OPEN",c_open);
}

module side_rect(x0,x1,z0,z1,c) {
  color(c) translate([(x0+x1)/2,0,(z0+z1)/2])
    cube([x1-x0,5,z1-z0],center=true);
}
module side_support(x,z,label) {
  color(c_support) translate([x,-5,z]) rotate([90,0,0]) cylinder(d=13,h=8);
  color([0.02,0.28,0.38]) translate([x+5,-10,z+7]) rotate([90,0,0]) linear_extrude(1)
    text(label,size=10,halign="left");
}
module side_scene() {
  color([0.25,0.30,0.34,0.22]) {
    frame_assembly(); bp_moving(-10); bp_moving(20); p267_arch();
  }
  side_rect(-307,-185,60,350,c_rear);
  side_rect(-185,40,105,320,c_s1);
  side_rect(40,230,105,320,c_s2);
  side_rect(230,400,115,320,c_s3);
  side_rect(400,511,140,320,c_open);
  for(x=[-135,5,65,125,170,230]) side_support(x,52,str("M4 ",x));
  side_support(240,42,"X240 socket");
  side_support(300,42,"X300 socket");
  side_support(390,245,"ARCH: OPEN");
  color([0.12,0.12,0.12]) translate([300,-10,335]) rotate([90,0,0]) linear_extrude(1)
    text("rim Z320 / diagonal & decks depend on basket mockup",size=12,halign="center");
}

if(map_view=="top") top_scene(); else side_scene();
