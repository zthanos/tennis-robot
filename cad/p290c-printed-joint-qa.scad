// P2-90c exported-STL joint regression.
// Every selector must export EMPTY.  Unlike module-only assembly checks this
// imports the actual release/temporary STL so a capped socket or stale export
// cannot pass accidentally.

side_frame_stl = "rear-pod/stl-v2/side_frame.stl";
panel_posts_stl = "rear-pod/stl-v2/panel_posts.stl";
check = "pair_clearance";

panel_center_x = -198.5;

module exported_side_frame_installed(side=1) {
  // The release STL is deliberately laid on its broad face (+90 deg about X).
  // Undo that print transform before applying the installed coordinates.
  translate([panel_center_x,side*54,64]) rotate([-90,0,0])
    import(side_frame_stl,convexity=20);
}

module exported_side_frame_design_pose() {
  rotate([-90,0,0]) import(side_frame_stl,convexity=20);
}

// Isolate one body from the four-body panel_posts print plate, then restore
// its module-local origin. Offsets are the frozen P2-81 export positions.
module exported_post(offset=0) {
  translate([-offset,0,0]) intersection() {
    import(panel_posts_stl,convexity=20);
    translate([offset-12.4,-60,-10]) cube([24.8,120,230]);
  }
}

module exported_posts_installed(dz=0) {
  translate([panel_center_x-58,-54,149+dz]) exported_post(0);   // rear_negative
  translate([panel_center_x+58,-54,149+dz]) exported_post(26); // front_negative
  translate([panel_center_x-58, 54,149+dz]) exported_post(60); // positive
  translate([panel_center_x+58, 54,149+dz]) exported_post(94); // positive
}

module exported_frames_installed() {
  exported_side_frame_installed(-1); exported_side_frame_installed(1);
}

module post_bolt_axes() {
  for(side=[-1,1],x=[panel_center_x-58,panel_center_x+58])
    // Thin centreline probe: exported polygon facets must not turn a valid
    // Ø4.5 clearance hole into a false full-diameter interference report.
    translate([x-24,side*54,154]) rotate([0,90,0]) cylinder(d=2.0,h=48);
}

module qa_pair_clearance() {
  intersection() { exported_frames_installed(); exported_posts_installed(); }
}

module qa_post_bolt_axes() {
  intersection() {
    post_bolt_axes();
    union(){ exported_frames_installed(); exported_posts_installed(); }
  }
}

module qa_post_insertion() {
  intersection() {
    union() { for(dz=[0:5:100]) exported_posts_installed(dz); }
    exported_frames_installed();
  }
}

module cup_open_probes_local() {
  for(x=[-58,58]) translate([x,0,79.3]) cylinder(d=23.4,h=45);
}

module qa_cups_open() {
  intersection() {
    exported_side_frame_design_pose();
    cup_open_probes_local();
  }
}

module required_ligament_local() {
  // Cardinal probes avoid false annular wedges caused by comparing the
  // faceted Ø4.5 STL bore with a newly tessellated cylinder.  They verify
  // 1.9 mm of the nominal 2.0 mm minimum (0.1 mm numeric tolerance) over
  // the complete flat land, 0.2 mm inside each axial face.
  for(x=[-58,58],sx=[-1,1]) {
    xc=x+sx*20;
    for(sz=[-1,1]) translate([xc,0,90+sz*3.25])
      cube([7.6,2.0,1.9],center=true);
    for(sy=[-1,1]) translate([xc,sy*3.25,90])
      cube([7.6,1.9,2.0],center=true);
  }
}

module qa_cup_ligament() {
  difference() {
    required_ligament_local();
    exported_side_frame_design_pose();
  }
}
if(check=="pair_clearance") qa_pair_clearance();
else if(check=="post_bolt_axes") qa_post_bolt_axes();
else if(check=="post_insertion") qa_post_insertion();
else if(check=="cups_open") qa_cups_open();
else if(check=="cup_ligament") qa_cup_ligament();
else union(){ qa_pair_clearance(); qa_post_bolt_axes(); qa_post_insertion();
              qa_cups_open(); qa_cup_ligament(); }
