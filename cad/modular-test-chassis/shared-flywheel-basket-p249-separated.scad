// P2-49 DEPENDENT concept: collection ramp OUTSIDE storage, launch feed below.
// Static/CAD study only.  No STL or manufacturing release.
view = "section_collect"; // [section_collect,section_launch,top_flow,assembly]
show_labels = true;
$fn = 48;

ball_d = 66;
basket_rear_x = -15;
basket_side_front_x = 278;
basket_half_y = 172;
divider_x = 100;
bay_half_y = 50;
bay_clear_half_y = 40;
rim_z = 360;
lidar_z = 390;
high_sill_z = 285;
wall_t = 3;
grid_open = 38;
floor_long_deg = 4.5;
floor_cross_deg = 2;
pivot = [387,0,49.83736];

storage_outline = [
    [basket_rear_x,-basket_half_y], [basket_side_front_x,-basket_half_y],
    [basket_side_front_x,-bay_half_y], [145,-bay_half_y],
    [divider_x,-bay_clear_half_y], [divider_x,bay_clear_half_y],
    [145,bay_half_y], [basket_side_front_x,bay_half_y],
    [basket_side_front_x,basket_half_y], [basket_rear_x,basket_half_y]
];

// P2-50 r1 proved that the piecewise 22→39 deg transition impacts and returns
// the ball.  The dependent ramp now starts with an R125 mm circular transition
// (8 deg chords) tangent to an approximately 48 deg climb to the sill.
collection_pts = [[402,10],[384.603,11.216],[367.545,14.842],[351.158,20.807],
                  [335.760,28.994],[321.652,39.244],[309.107,51.359],
                  [270,95.10],[220,151.05],[170,207.00],[120,262.95],[100,285]];
feed_pts = [[100,42],[180,42],[225,40],[250,38]];

function mouth_floor(a) = [
    pivot[0]-62*cos(a)+8*sin(a),
    pivot[2]-62*sin(a)-8*cos(a)
];

module xz_strip(p0,p1,width=80,t=3) {
    dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; len=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
        rotate([0,-a,0]) cube([len+1,width,t],center=true);
}

module xz_path(points,width=80,t=3) {
    for(i=[0:len(points)-2]) xz_strip(points[i],points[i+1],width,t);
}

module xz_rail(p0,p1,y,rail_t=3,rail_h=20) {
    dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; len=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    translate([(p0[0]+p1[0])/2,y,(p0[1]+p1[1])/2+rail_h/2])
        rotate([0,-a,0]) cube([len+1,rail_t,rail_h],center=true);
}

module collection_ramp() {
    color([.10,.62,.80]) xz_path(collection_pts,80,4);
    // Continuous rails are part of the dependent ramp after P2-50 r2:
    // isolated rail facets let a centred ball escape during rollback.
    for(i=[0:len(collection_pts)-2]) for(side=[-1,1])
        color([.08,.48,.68]) xz_rail(collection_pts[i],collection_pts[i+1],side*41.5);
}

module section_label(s,x,z,size=8,col=[0.05,0.05,0.05]) {
    if(show_labels) color(col) translate([x,-3,z]) rotate([90,0,0])
        linear_extrude(0.8) text(s,size=size,halign="center",valign="center");
}

module pile_envelope(top_z,col) {
    color(col) translate([(basket_rear_x+divider_x)/2,0,(51+top_z)/2])
        cube([divider_x-basket_rear_x-8,8,top_z-51],center=true);
}

module divider_section() {
    // Low feed opening Z42..122; solid separation; high opening Z285..351.
    color([.35,.55,.38]) {
        translate([divider_x,0,25]) cube([wall_t,10,34],center=true);
        translate([divider_x,0,(122+high_sill_z)/2]) cube([wall_t,10,high_sill_z-122],center=true);
        translate([divider_x,0,(351+rim_z)/2]) cube([wall_t,10,rim_z-351],center=true);
    }
}

module common_moving_throat(angle) {
    m=mouth_floor(angle);
    // Module-mounted nose follows the only rear mouth and overlaps the selector.
    p=[[250,angle<0?72:48],[285,angle<0?62:40],m];
    color([.82,.58,.12]) xz_path(p,72,5);
    color([.75,.25,.15]) translate([m[0],0,m[1]+33]) sphere(d=ball_d,$fn=36);
}

module selector_state(mode="collect") {
    // Reserved two-position selector envelope X225..335/Z32..130.
    %color([.8,.55,.15,.10]) translate([280,0,81]) cube([110,82,98],center=true);
    if(mode=="collect") {
        // Close low feed branch and turn the common throat upward.
        color([.9,.1,.1]) translate([224,0,77]) cube([5,88,76],center=true);
        color([.95,.75,.1]) xz_strip([238,52],[258,91],72,5);
        section_label("FEED CLOSED",190,72,7,[.7,0,0]);
        section_label("COLLECT OPEN",208,132,7,[0,.35,0]);
    } else {
        // Close upper collection branch and connect floor feed to mouth.
        color([.9,.1,.1]) xz_strip([224,112],[250,86],88,5);
        color([.95,.75,.1]) xz_strip([238,42],[260,50],72,5);
        section_label("FEED OPEN",188,68,7,[0,.35,0]);
        section_label("RAMP CLOSED",194,135,7,[.7,0,0]);
    }
}

module base_section() {
    // Storage floor falls 4.5 degrees to the low feed opening at X100.
    color([.35,.55,.38]) xz_strip([basket_rear_x,51.05],[divider_x,42],10,4);
    color([.35,.55,.38]) translate([basket_rear_x,0,(51+rim_z)/2])
        cube([wall_t,10,rim_z-51],center=true);
    divider_section();
    // Feed channel is below and physically separated from collection ramp.
    color([.15,.45,.78]) xz_path(feed_pts,80,4);
    color([.15,.45,.78]) xz_path([[100,116],[180,116],[225,114]],88,3);
    // Collection ramp is entirely in the external central bay.
    collection_ramp();
    pile_envelope(263.76,[.25,.75,.25,.25]);
    pile_envelope(280.91,[.95,.65,.15,.18]);
    %color([.8,0,0,.25]) translate([150,0,lidar_z]) cube([500,6,1],center=true);
    section_label("40 worst Z263.8",34,248,7,[0,.35,0]);
    section_label("45 worst Z280.9",34,275,7,[.65,.35,0]);
    section_label("HIGH SILL Z285",132,301,7,[0,.3,.5]);
    section_label("SEPARATING WALL",110,196,7);
    section_label("LAUNCH FEED BELOW",171,25,7,[0,.2,.65]);
    section_label("COLLECTION RAMP OUTSIDE",218,218,7,[0,.4,.65]);
    section_label("LiDAR Z390",10,382,7,[.6,0,0]);
}

module section(mode="collect") {
    base_section();
    if(mode=="collect") {
        common_moving_throat(-3);
        selector_state("collect");
        section_label("-3 deg COLLECTION",300,155,10,[0,.3,0]);
    } else {
        // Show both frozen launch endpoints; common nose follows the module.
        common_moving_throat(15);
        %common_moving_throat(20);
        selector_state("launch");
        section_label("+15..+20 deg LAUNCH",298,155,10,[.55,.2,0]);
    }
}

module arrow2d(p0,p1,w=7) {
    dx=p1[0]-p0[0]; dy=p1[1]-p0[1]; l=sqrt(dx*dx+dy*dy); a=atan2(dy,dx);
    translate(p0) rotate(a) polygon([[0,-w/2],[l-14,-w/2],[l-14,-w],[l,0],[l-14,w],[l-14,w/2],[0,w/2]]);
}

module top_flow() {
    color([.55,.72,.58]) linear_extrude(3) polygon(storage_outline);
    color([.15,.45,.78]) translate([0,0,4]) linear_extrude(2)
        polygon([[100,-40],[278,-40],[335,-34],[335,34],[278,40],[100,40]]);
    color([.1,.65,.8,.65]) translate([0,0,8]) linear_extrude(2)
        polygon([[100,-40],[145,-55],[278,-55],[335,-40],[335,40],[278,55],[145,55],[100,40]]);
    for(pair=[[[250,-125],[125,-48]],[[250,125],[125,48]],[[20,-120],[92,-35]],[[20,120],[92,35]]])
        color([.95,.55,.05]) translate([0,0,12]) linear_extrude(2) arrow2d(pair[0],pair[1]);
    section_label("80 mm CLEAR THROAT",100,0,8);
}

module grid_wall_x(x,h=300,w=344) {
    color([.35,.55,.38]) {
        translate([x,-w/2,rim_z-h/2]) cube([3,8,h],center=true);
        translate([x,w/2,rim_z-h/2]) cube([3,8,h],center=true);
        for(z=[80:46:rim_z]) translate([x,0,z]) cube([3,w,7],center=true);
        for(y=[-150:46:150]) translate([x,y,rim_z-h/2]) cube([3,7,h],center=true);
    }
}

module assembly() {
    color([.55,.72,.58,.65]) linear_extrude(3) polygon(storage_outline);
    grid_wall_x(basket_rear_x,309,344);
    // Side-wing front grids only; central notch remains open for both routes.
    for(y=[-113.5,113.5]) color([.35,.55,.38])
        translate([basket_side_front_x,y,210]) cube([3,117,300],center=true);
    collection_ramp();
    color([.15,.45,.78]) xz_path(feed_pts,80,4);
}

if(view=="section_collect") section("collect");
else if(view=="section_launch") section("launch");
else if(view=="top_flow") top_flow();
else if(view=="assembly") assembly();

echo("P2_49_DEPENDENT_NO_STL_RELEASE",divider_x=divider_x,
     external_bay_clear=2*bay_clear_half_y,external_bay_outer=2*bay_half_y,
     high_sill_z=high_sill_z,rim_z=rim_z,lidar_clearance=lidar_z-rim_z,
     mouth_minus3=mouth_floor(-3),mouth_plus15=mouth_floor(15),mouth_plus20=mouth_floor(20));
