// P2-77 concept-only packaging view for the post-drive-test rear pod.
// This file does not alter rear-pod-v2.scad or any v2 STL.
$fn=36;
view="assembly";

module box(c,s,col=[.5,.5,.5,.6]) { color(col) translate(c) cube(s,center=true); }
module rear_pod_proposal() {
    // Frozen envelope ghost.
    %box([-198.5,0,215],[147,334,320],[.2,.2,.2,.12]);
    // Reserved battery drawers unchanged.
    box([-198.5,-115.5,98.5],[141,91,81],[.15,.35,.8,.5]);
    box([-198.5, 115.5,98.5],[141,91,81],[.15,.35,.8,.5]);
    // Ugreen moved to the free upper +Y side; drawer still opens to -X.
    box([-198.5,112,264],[121,52,52],[.1,.65,.35,.55]);
    // One rear-service carrier; components project toward -X.
    box([-181,-39,216.5],[4,236,135],[.75,.75,.78,.75]);
    box([-207,-96.5,191],[52,115,80],[.85,.45,.12,.75]); // perf + plug/bend
    box([-195,23.5,181],[28,105,60],[.85,.75,.15,.75]);  // Mega
    for(y=[-130,-74,-18,38])
        box([-204,y,255],[42,44,40],[.7,.15,.12,.8]);      // rear-facing BTS
    // Four straight posts and a direct 12 mm local E-stop reaction rib.
    for(x=[-256.5,-140.5],y=[-54,54])
        color([.92,.58,.08]) translate([x,y,149]) cylinder(d=12,h=198);
    box([-198.5,-54,341],[116,12,12],[.95,.58,.05,.9]);
    box([-198.5,0,349],[116,138,4],[.12,.12,.14,.9]);
    // Rear extraction/service direction.
    color([.85,.1,.75]) for(y=[-130,-74,-18,38])
        translate([-250,y,255]) rotate([0,-90,0]) cylinder(d1=0,d2=10,h=18);
}
rear_pod_proposal();
