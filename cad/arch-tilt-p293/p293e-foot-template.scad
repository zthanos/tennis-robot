// Non-load-bearing drill-free fit gauge. L=+Y, R=-Y. Print flat, no supports.
use <p293d-arch-candidate.scad>
side=1;
$fn=64;
module outline(){projection(cut=true) translate([0,0,-52.001]) p293d_foot();}
module label(s,size){if(side>0)text(s,size=size);else mirror([0,1,0])text(s,size=size);}
module gauge(){
 difference(){
  // Fill structural clearance bores from the section, then cut gauge Ø4.5.
  linear_extrude(2) union(){outline();for(p=[[240,150],[240,160],[230,220]])translate(p)circle(d=6);}
  for(p=[[240,150],[240,160],[230,220]])translate([p[0],p[1],-1])cylinder(d=4.5,h=4);
  // Reference notch in the X271 forward edge (not a motor clearance reduction).
  translate([270.2,226,-1])cube([2,4,4]);
  translate([233,138,1.5])linear_extrude(1)label(side>0?"L":"R",5);
  translate([235,209,1.5])linear_extrude(1)label("FRONT +X",2.5);
  translate([246,216,1.5])linear_extrude(1)polygon([[0,-0.5],[9,-0.5],[9,-1.5],[12,0],[9,1.5],[9,0.5],[0,0.5]]);
 }
}
// Reflect footprint, not glyphs: reflect geometry first, engrave R afterwards
// by the compensating mirror of text inside the negative-side difference.
if(side>0)translate([-192,-133,0])gauge();
else translate([-192,242,0])mirror([0,1,0])gauge();
