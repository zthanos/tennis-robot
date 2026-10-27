// P2-70 rear electronics / energy pod -- packaging proposal only.
// No manufacturing geometry and no STL release.
// Units mm; +X front, +Y left, +Z up.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>
use <shared-flywheel-p267-tilted-arch.scad>

p270_view = "assembly"; // assembly | top | side | qa_basket | qa_drive | qa_frame
p270_angle = 0;
$fn = 36;

// The 147 mm depth is the minimum that contains a 141 mm clear Parkside
// reservation (135 mm assumption + 3 mm each side) behind 3 mm walls.
// The rear corner chamfers preserve the existing R=425 mm turn envelope.
p270_x_front = -125;
p270_x_rear = -272;
p270_y_half = 167;
p270_rear_chamfer = 5;
p270_z_bottom = 55;
p270_z_top = 350;
p270_wall = 3;

// Clear reservation envelopes. Parkside is a CAD assumption pending purchase.
p270_parkside_clear = [141,91,81];       // 135x85x75 + 3 mm/side
p270_pi_clear = [98,74,46];              // measured 92x68x40 + 3 mm/side
p270_ugreen_clear = [121,52,52];         // maker 115x46x45.5 + 3 mm/side
p270_bts_bay = [58,58,35];               // MEASUREMENT USER before detail design

module p270_footprint_2d(inset=0) {
  polygon(points=[
    [p270_x_front+inset,-p270_y_half+inset],
    [p270_x_rear+p270_rear_chamfer+inset,-p270_y_half+inset],
    [p270_x_rear+inset,-p270_y_half+p270_rear_chamfer+inset],
    [p270_x_rear+inset,p270_y_half-p270_rear_chamfer-inset],
    [p270_x_rear+p270_rear_chamfer+inset,p270_y_half-inset],
    [p270_x_front+inset,p270_y_half-inset]
  ]);
}

module p270_outer_envelope() {
  translate([0,0,p270_z_bottom])
    linear_extrude(height=p270_z_top-p270_z_bottom)
      translate([0,0]) p270_footprint_2d();
}

module p270_shell() {
  color([.16,.20,.25,.70]) difference() {
    p270_outer_envelope();
    translate([0,0,p270_z_bottom+p270_wall])
      linear_extrude(height=p270_z_top-p270_z_bottom-2*p270_wall)
        p270_footprint_2d(p270_wall);
    // Rear/downward-facing louvres only; no openings on the horizontal top.
    for(y=[-55,-25,5,35],z=[155,180,205])
      translate([p270_x_rear-1,y,z]) cube([p270_wall+2,18,7]);
  }
  // Two removable G2-filter hoods over the lower intake zones.
  color([.10,.35,.48]) for(side=[-1,1])
    translate([-267,side*118,170]) cube([8,72,72],center=true);
}

module p270_mounts() {
  // Four M4 feet use the existing rear-crossbar top grid at Y +/-75,+/-125.
  // The two anti-racking ears reach the existing corner clamps; no new chassis holes.
  color([.82,.50,.12]) {
    for(y=[-125,-75,75,125]) hull() {
      translate([-165,y,52]) cylinder(d=14,h=4);
      translate([-155,y,61]) cube([28,14,12],center=true);
    }
    for(side=[-1,1]) hull() {
      translate([-165,side*125,58]) cube([16,12,12],center=true);
      translate([-165,side*210,58]) cube([16,12,12],center=true);
    }
  }
}

module p270_reserved_contents() {
  // Side battery drawers; insertion/removal is toward -X, never downward.
  for(side=[-1,1]) {
    color([.10,.58,.22,.48])
      translate([-198.5,side*115.5,98.5])
        cube(p270_parkside_clear,center=true);
    color([.85,.55,.10,.65])
      translate([-198.5,side*115.5,56.5]) cube([141,97,3],center=true);
  }

  // Central board stack. Mega/perfboard sizes are provided holder footprints;
  // shown heights and all component connector/bend zones need user measurement.
  color([.18,.55,.75,.55])
    translate([-198.5,-22,80]) cube([126,86,18],center=true); // perfboard clear
  color([.12,.42,.72,.65])
    translate([-198.5,0,116]) cube([116,66,18],center=true);  // Mega clear

  // Four ventilated BTS bays, one deliberately spare.
  for(x=[-230,-167],y=[-31,31])
    color([.74,.18,.12,.55]) translate([x,y,158]) cube(p270_bts_bay,center=true);

  // Fuse, E-stop power relay, FIT0186 12 V DC-DC and two ideal-diode bays.
  // These are zoning envelopes, not measured part bodies.
  color([.72,.62,.12,.58]) {
    translate([-239,-45,203]) cube([45,34,28],center=true);
    translate([-190,-45,203]) cube([45,34,28],center=true);
    translate([-239,45,203]) cube([45,34,28],center=true);
    translate([-190,45,203]) cube([45,34,28],center=true);
  }

  // Upper service shelf. Ugreen is rear-sliding and strap/foam retained.
  color([.10,.14,.20,.62])
    translate([-192,-48,258]) cube(p270_pi_clear,center=true);
  color([.16,.18,.25,.62])
    translate([-195,42,258]) cube(p270_ugreen_clear,center=true);
}

module p270_controls() {
  // Under-panel bodies are shown at their measured plan envelopes. Holes:
  // E-stop 22.5, covered switch 12.3, ARM 15.5 mm; LED remains a user measure.
  color([.95,.76,.05]) translate([-153,-92,p270_z_top]) cylinder(d=48,h=3);
  color([.86,.08,.08]) translate([-153,-92,p270_z_top+3]) cylinder(d=38,h=21);
  color([.28,.28,.30]) translate([-153,-92,p270_z_top-15]) cube([35,30,30],center=true);
  color([.08,.08,.09]) translate([-153,-25,p270_z_top]) cylinder(d=18,h=8);
  color([.28,.28,.30]) translate([-153,-25,p270_z_top-7.5]) cube([27,15,15],center=true);
  color([.18,.70,.25]) translate([-153,35,p270_z_top]) cylinder(d=18,h=8);
  color([.28,.28,.30]) translate([-153,35,p270_z_top-7.5]) cylinder(d=15,h=15);
  color([.10,.85,.92]) translate([-153,82,p270_z_top]) cylinder(d=9,h=7);
}

module p270_cutaway_outline() {
  pts = [
    [p270_x_front,-p270_y_half],
    [p270_x_rear+p270_rear_chamfer,-p270_y_half],
    [p270_x_rear,-p270_y_half+p270_rear_chamfer],
    [p270_x_rear,p270_y_half-p270_rear_chamfer],
    [p270_x_rear+p270_rear_chamfer,p270_y_half],
    [p270_x_front,p270_y_half]
  ];
  color([.95,.55,.08]) {
    for(z=[p270_z_bottom,p270_z_top],i=[0:len(pts)-1]) hull() {
      translate([pts[i][0],pts[i][1],z]) sphere(d=2.5);
      translate([pts[(i+1)%len(pts)][0],pts[(i+1)%len(pts)][1],z]) sphere(d=2.5);
    }
    for(p=pts) hull() {
      translate([p[0],p[1],p270_z_bottom]) sphere(d=2.5);
      translate([p[0],p[1],p270_z_top]) sphere(d=2.5);
    }
  }
}

module p270_rear_pod(show_contents=true,show_shell=true) {
  if(show_shell) p270_shell(); else p270_cutaway_outline();
  p270_mounts();
  if(show_contents) p270_reserved_contents();
  p270_controls();
}

module p270_fixed_world_without_old_power() {
  frame_assembly(); drive_package_envelopes(); pivot_supports();
  p267_arch(); p267_sensors(); p251_basket_exact(); p251_feed_exact();
  p267_ski(75); p267_soft_deflector();
  d5065_controller_envelopes();
}

module p270_assembly(angle=p270_angle) {
  p270_fixed_world_without_old_power();
  p253_frozen_launcher(angle); p253_actuator(angle);
  p253_as5600(); p253_stops(); p253_cables(angle);
  p270_rear_pod(true,true);
}

if(p270_view=="assembly") p270_assembly(p270_angle);
else if(p270_view=="top") projection(cut=false) p270_assembly(p270_angle);
else if(p270_view=="side") projection(cut=false) rotate([90,0,0]) p270_assembly(p270_angle);
else if(p270_view=="qa_basket") intersection(){
  p270_outer_envelope(); union(){p251_basket_exact();p251_feed_exact();p267_soft_deflector();}
}
else if(p270_view=="qa_drive") intersection(){
  p270_outer_envelope(); drive_package_envelopes();
}
else if(p270_view=="qa_frame") intersection(){
  // Body only. Mount feet intentionally mate with the rear crossbar and are excluded.
  p270_outer_envelope(); frame_assembly();
}
