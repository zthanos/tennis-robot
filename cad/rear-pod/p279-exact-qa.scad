// P2-79 exact-solid regression probes. Every selectable QA view must be empty.
use <rear-pod-v2.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>

qa = "chassis";

module p279_drawer_sweeps() {
    for(side=[-1,1])
        translate([-364,side*115.5,98.5]) cube([472,91,81],center=true);
}

module p279_chassis_fastener_axes() {
    for(side=[-1,1],y=[75,125])
        translate([-165,side*y,45]) cylinder(d=4.5,h=22);
    for(side=[-1,1])
        translate([-135,side*235,37]) rotate([-side*90,0,0]) cylinder(d=4.5,h=20);
}

module p279_pod_fastener_axes() {
    for(side=[-1,1],y=[75,125])
        translate([-165,side*y,45]) cylinder(d=4.5,h=22);
    for(side=[-1,1])
        translate([-135,side*235,37]) rotate([-side*90,0,0]) cylinder(d=4.5,h=20);
}

if(qa=="chassis") difference(){
    intersection(){ installed_p279_structure(); frame_assembly(); }
    // Approved mating/contact zones only: the full rear-crossbar saddle and
    // the two outer corner-clamp faces. Any penetration elsewhere survives.
    translate([-191,-196,44]) cube([52,392,8.2]);
    for(side=[-1,1]) translate([-146,side>0?244.9:-251,24]) cube([22,6.1,32]);
}
else if(qa=="reserved") qa_all_reserved_collision();
else if(qa=="drawers") intersection(){ installed_p279_structure(); p279_drawer_sweeps(); }
else if(qa=="hole_difference") union(){
    difference(){ p279_chassis_fastener_axes(); p279_pod_fastener_axes(); }
    difference(){ p279_pod_fastener_axes(); p279_chassis_fastener_axes(); }
}
else assert(false,str("unknown P2-79 QA: ",qa));
