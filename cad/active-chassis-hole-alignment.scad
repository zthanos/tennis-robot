// P2-86b — permanent alignment probes for every active printed part that
// shares an existing chassis M4.  Every selectable QA result must be EMPTY.
// A thin Ø2 cylinder follows the intended screw axis and is intersected with
// the exact frame_assembly(); any solid means the printed chassis has no bore
// on that axis (or the interface datum has drifted).

use <modular-test-chassis/modular-chassis-v3.scad>

interface = "all";
$fn = 32;
probe_d = 2;

module z_probe(x,y,z0=15,z1=65) {
  translate([x,y,z0]) cylinder(d=probe_d,h=z1-z0);
}
module x_probe(y,z,x0=-205,x1=-125) {
  translate([x0,y,z]) rotate([0,90,0]) cylinder(d=probe_d,h=x1-x0);
}
module y_probe(x,z,y0=185,y1=255) {
  translate([x,y0,z]) rotate([-90,0,0]) cylinder(d=probe_d,h=y1-y0);
}

module pod_base_probes() {
  for(side=[-1,1],y=[75,125]) z_probe(-165,side*y);
}

module corner_tie_probes() {
  // P2-86b tie and stay share the existing rear-crossbar transverse bore.
  for(side=[-1,1]) x_probe(side*165,37);
}

module corner_stay_probes() {
  // P2-86b stay reuses the rear-crossbar end-lock pair, not Y±177.5.
  for(side=[-1,1]) {
    z_probe(-165,side*165);
    x_probe(side*165,37);
  }
}

module ski_bracket_probes() {
  for(side=[-1,1],y=[75,100]) z_probe(300,side*y);
}

module mpu_probes() {
  // P2-77 plate holes X100/X125 align with the real side grid at Z37.
  for(x=[100,125]) y_probe(x,37,185,255);
}

module perimeter_side_rear_probes() {
  for(side=[-1,1],x=[-135,5]) z_probe(x,side*220);
}
module perimeter_side_mid_probes() {
  for(side=[-1,1],x=[65,100,125,170,230]) z_probe(x,side*220);
}
module perimeter_x240_probes() {
  for(side=[-1,1],y=[150,160]) z_probe(240,side*y,15,50);
}
module perimeter_x300_probes() {
  for(side=[-1,1],y=[150,160]) z_probe(300,side*y,15,50);
}
module perimeter_map_probes() {
  perimeter_side_rear_probes(); perimeter_side_mid_probes();
  perimeter_x240_probes(); perimeter_x300_probes();
}

module selected_probes() {
  if(interface=="pod_base") pod_base_probes();
  else if(interface=="corner_ties") corner_tie_probes();
  else if(interface=="corner_stays") corner_stay_probes();
  else if(interface=="ski_brackets") ski_bracket_probes();
  else if(interface=="mpu") mpu_probes();
  else if(interface=="perimeter_side_rear") perimeter_side_rear_probes();
  else if(interface=="perimeter_side_mid") perimeter_side_mid_probes();
  else if(interface=="perimeter_x240") perimeter_x240_probes();
  else if(interface=="perimeter_x300") perimeter_x300_probes();
  else if(interface=="perimeter_map") perimeter_map_probes();
  else if(interface=="all") {
    pod_base_probes(); corner_tie_probes(); corner_stay_probes();
    ski_bracket_probes(); mpu_probes(); perimeter_map_probes();
  }
  else assert(false,str("unknown active chassis interface: ",interface));
}

intersection() { selected_probes(); frame_assembly(); }
