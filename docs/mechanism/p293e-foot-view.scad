use <../../cad/arch-tilt-p293/p293d-arch-candidate.scad>
use <../../cad/modular-test-chassis/modular-chassis-v3.scad>
$fn=48;
module washer(x,y,z,od=9){color("silver") translate([x,y,z]) difference(){cylinder(d=od,h=0.8);translate([0,0,-0.1]) cylinder(d=4.3,h=1);}}
module bolt(x,y,top,len,od=9,bottom=22){
 color("silver") translate([x,y,top+0.8-len]) cylinder(d=4,h=len);
 washer(x,y,top,od);
 color("silver") translate([x,y,top+0.8]) cylinder(d=7,h=4);
 washer(x,y,bottom-0.8,od);
 color("silver") translate([x,y,bottom-4]) cylinder(d=8,h=3.2,$fn=6);
}
color([0.45,0.5,0.55]) translate([300,220,22]) front_terminal_v3(1);
color([0.25,0.3,0.35]) translate([200,220,37]) clamp_sleeve_x();
// Display-only cut above the recessed X240 seats, to expose both heads.
color([0.9,0.55,0.15]) difference(){p293d_foot();translate([228,133,62])cube([24,34,40]);}
color([0.3,0.65,0.7]) intersection(){translate([0,228,0])p293d_knee();translate([200,190,20])cube([80,70,115]);}
bolt(240,150,62,50);bolt(240,160,62,50);
// Rail +18 PETG, +1 upper washer, +1 lower washer: total +20 grip.
bolt(230,220,71,60,bottom=21);
color("silver")translate([230,220,70])difference(){cylinder(d=12,h=1);cylinder(d=4.3,h=1);}
color("silver")translate([230,220,21])difference(){cylinder(d=12,h=1);cylinder(d=4.3,h=1);}
// Actual drive conservative boundary, no motor geometry changed.
color([0.8,0.2,0.2,0.25]) translate([280,195,52])cube([40,50,53]);
