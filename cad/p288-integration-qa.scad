// P2-88 integration exact-solid gates.  Any non-empty export is a failure.
use <rear-pod/rear-pod-v2.scad>
use <body-panels/body-panels-f-rear-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

check = "docks_pod_frame";

module docks_and_drawers() {
  for (s=[-1,1]) { parkside_dock(s); parkside_drawer(s); }
}

module qa_docks_pod_frame() {
  intersection() {
    docks_and_drawers();
    union() {
      frame_assembly();
      carrier_installed();
      electronics_ghosts();
      translate([-212,0,64]) pi_shelf_v2();
      for (s=[-1,1]) translate([-198.5,s*54,64]) side_frame();
    }
  }
}

module qa_docks_panels_closed() {
  intersection() { docks_and_drawers(); all_f_rear(0); }
}

module qa_docks_panels_service() {
  intersection() { docks_and_drawers(); all_f_rear(110); }
}

module qa_ugreen_surroundings() {
  intersection() {
    ugreen_post_holder();
    union() {
      carrier_installed();
      electronics_ghosts();
      control_body_solids();
      translate([-198.5,0,347]) control_panel_v2();
    }
  }
}

module qa_new_parts_ground() {
  intersection() {
    p288_new_parts_installed();
    translate([-400,-300,-100]) cube([800,600,115]); // physical ground <15
  }
}

if (check=="docks_pod_frame") qa_docks_pod_frame();
else if (check=="docks_panels_closed") qa_docks_panels_closed();
else if (check=="docks_panels_service") qa_docks_panels_service();
else if (check=="ugreen_surroundings") qa_ugreen_surroundings();
else if (check=="new_parts_ground") qa_new_parts_ground();
else union() {
  qa_docks_pod_frame(); qa_docks_panels_closed();
  qa_docks_panels_service(); qa_ugreen_surroundings(); qa_new_parts_ground();
}
