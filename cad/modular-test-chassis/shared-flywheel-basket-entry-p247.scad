// P2-47 dependent basket-entry comparison.  Concept/reference geometry only;
// no STL release.  Coordinates are millimetres in the v3 base frame.
variant = "all"; // [all,tall_open_drop,high_hood,one_way_flap,lip_cover]
$fn = 48;

floor_z = 54;
basket_front_x = 278;
basket_rear_x = -15;
basket_half_y = 175;
basket_top_z = 215;
entry_half_y = 48;

module basket_shell() {
  color([.65,.65,.68,.25]) {
    translate([(basket_front_x+basket_rear_x)/2,0,floor_z-1.5]) cube([basket_front_x-basket_rear_x,350,3],center=true);
    for(y=[-173.5,173.5]) translate([131.5,y,(floor_z+basket_top_z)/2]) cube([293,3,basket_top_z-floor_z],center=true);
    translate([basket_rear_x+1.5,0,(floor_z+basket_top_z)/2]) cube([3,350,basket_top_z-floor_z],center=true);
    for(y=[-111.5,111.5]) translate([basket_front_x-1.5,y,(floor_z+basket_top_z)/2]) cube([3,127,basket_top_z-floor_z],center=true);
  }
}

module panel_between(x0,z0,x1,z1,width=110,t=3,c=[.2,.55,.9,1]) {
  dx=x1-x0; dz=z1-z0; len=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
  color(c) translate([(x0+x1)/2,0,(z0+z1)/2]) rotate([0,-a,0]) cube([len,width,t],center=true);
}

module high_hood() {
  // Controlled-rise entry then 40+ mm rear-overhung hood.
  panel_between(352,23.5,330,30,82,2);
  panel_between(330,30,305,40,88,2);
  panel_between(305,40,280,52,94,2);
  panel_between(280,52,250,65,96,2);
  panel_between(210,160,312,178,110,3,[1,.55,.1,1]);
  for(y=[-56.5,56.5]) color([1,.55,.1,1]) translate([261,y,126]) cube([104,3,90],center=true);
}

module one_way_flap(angle=0) {
  // Physical concept: top hinge, positive rotation only into -X; positive
  // latch/stop required at 0 degrees.  Shown closed.
  color([.95,.35,.08,1]) translate([305,0,165]) rotate([0,angle,0]) translate([0,0,-42.5]) cube([2,96,85],center=true);
  color([.25,.25,.28,1]) translate([305,0,165]) rotate([90,0,0]) cylinder(d=8,h=112,center=true);
}

module lip_cover() {
  panel_between(305,52,290,56,94,2);
  panel_between(290,56,278,60,95.5,2);
  color([.95,.35,.08,1]) translate([272.5,0,57]) cube([5,96,6],center=true);
  panel_between(210,149,312,164,110,3,[1,.75,.15,1]);
  for(y=[-56.5,56.5]) color([1,.75,.15,1]) translate([261,y,111]) cube([104,3,96],center=true);
}

module tall_open_drop() {
  // P2-47 simplest candidate: 335 mm rim, uncovered bridge and rear drop.
  // No flap, hood or liner; the lower feed/singulator interface remains open.
  for(y=[-173.5,173.5]) color([.35,.72,.42,.55])
    translate([131.5,y,275]) cube([293,3,120],center=true);
  color([.35,.72,.42,.55]) translate([-13.5,0,275]) cube([3,350,120],center=true);
  color([.35,.72,.42,.55]) translate([273.5,0,275]) cube([3,350,120],center=true);
  pts=[[352,23.5],[330,31],[305,48],[280,69],[250,95],[220,121],[190,147],[160,173],[130,199],[110,218],[90,247]];
  for(i=[0:len(pts)-2]) panel_between(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1],i==0?82:96,2,[.10,.62,.92,1]);
}

if (variant=="all") {
  for(i=[0:3]) translate([0,i*430,0]) { basket_shell(); if(i==0) tall_open_drop(); if(i==1) high_hood(); if(i==2) one_way_flap(); if(i==3) lip_cover(); }
} else {
  basket_shell();
  if(variant=="tall_open_drop") tall_open_drop();
  if(variant=="high_hood") high_hood();
  if(variant=="one_way_flap") one_way_flap();
  if(variant=="lip_cover") lip_cover();
}
