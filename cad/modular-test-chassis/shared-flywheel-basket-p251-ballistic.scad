// P2-51 DEPENDENT concept: passive R75 ski-jump and open ballistic basket.
// Static/Gazebo study only. No STL or manufacturing release.
view = "side"; // [side,top,assembly]
show_envelopes = true;
$fn = 40;

ball_d = 66;
lidar_z = 390;
side_rim_z = 320;
backboard_top_z = 375;
basket_front_x = 278;
basket_rear_x = -85;
basket_half_y = 172;
rear_half_y = 140;
floor_front_z = 42;
floor_rear_z = 70;
ski_pts = [[435,10],[383,10],[376.463,10.285],[369.976,11.139],
            [363.589,12.556],[357.348,14.523],[351.304,17.027],
            [345.500,20.048],[339.982,23.564],[334.791,27.547],
            [332.815,29.264]];

module xz_strip(p0,p1,width=94,t=3) {
    dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
        rotate([0,-a,0]) cube([l+0.8,width,t],center=true);
}

module ski() {
    color([.08,.62,.82]) for(i=[0:len(ski_pts)-2]) xz_strip(ski_pts[i],ski_pts[i+1]);
    for(side=[-1,1]) color([.04,.42,.62])
        for(i=[0:len(ski_pts)-2]) {
            p0=ski_pts[i]; p1=ski_pts[i+1];
            dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
            translate([(p0[0]+p1[0])/2,side*48.5,(p0[1]+p1[1])/2+10])
                rotate([0,-a,0]) cube([l+0.8,3,20],center=true);
        }
}

module grid_side(y) {
    color([.35,.62,.38]) {
        for(x=[-70:46:252]) translate([x,y,(floor_rear_z+side_rim_z)/2]) cube([7,3,side_rim_z-floor_rear_z],center=true);
        for(z=[82:46:312]) translate([(basket_rear_x+basket_front_x)/2,y,z]) cube([basket_front_x-basket_rear_x,3,7],center=true);
    }
}

module basket() {
    // 4.4 degree gravity floor toward the forward low feed channel.
    color([.48,.70,.50,.75]) xz_strip([basket_rear_x,floor_rear_z],[basket_front_x,floor_front_z],280,4);
    for(y=[-basket_half_y,basket_half_y]) grid_side(y);
    // Rear backboard is local and tall; central width avoids rear FIT0403 corners.
    color([.38,.58,.40]) translate([basket_rear_x,0,(floor_rear_z+backboard_top_z)/2])
        rotate([0,12,0]) cube([4,2*rear_half_y,backboard_top_z-floor_rear_z],center=true);
    // Front is open in the central 280 mm flight aperture; only side posts remain.
    for(y=[-156,156]) color([.38,.58,.40]) translate([basket_front_x,y,190]) cube([4,32,260],center=true);
}

module feed_and_selector() {
    // Reserved interface only: low channel and common-throat diverter remain 5d.
    color([.18,.42,.78]) xz_strip([basket_rear_x,58],[225,38],80,4);
    color([.95,.55,.10,.25]) translate([280,0,80]) cube([110,82,96],center=true);
}

module pile(top_z,col) {
    color(col) translate([80,0,(floor_front_z+top_z)/2])
        cube([300,270,top_z-floor_front_z],center=true);
}

module trajectory(v=3.68,a=41) {
    // Visual envelope from the measured 6 m/s matrix mean, not a CAD claim.
    x0=332.815; z0=62.264;
    color([.95,.35,.08]) for(t=[0:.035:.28]) {
        x=x0-1000*v*cos(a)*t;
        z=z0+1000*(v*sin(a)*t-0.5*9.80665*t*t);
        translate([x,0,z]) sphere(d=12);
    }
}

module label_xz_p251(s,p,size=13) {
    color([.05,.05,.05]) translate(p) rotate([90,0,0])
        linear_extrude(height=1,center=true)
            text(s,size=size,halign="center",valign="center");
}

module labels_side() {
    // Ground datum plus orientation marks. Text is made in the XZ plane so
    // the corrected -90 deg projection keeps +Z upward.
    color([.08,.08,.08]) translate([150,0,0]) cube([700,3,1.5],center=true);
    color([.08,.08,.08]) hull() {
        translate([430,0,18]) sphere(d=4);
        translate([500,0,18]) sphere(d=4);
    }
    color([.08,.08,.08]) hull() {
        translate([500,0,18]) sphere(d=4);
        translate([487,0,27]) sphere(d=4);
    }
    color([.08,.08,.08]) hull() {
        translate([500,0,18]) sphere(d=4);
        translate([487,0,9]) sphere(d=4);
    }
    label_xz_p251("GROUND",[-175,0,14],12);
    label_xz_p251("FRONT +X",[465,0,42],12);
    label_xz_p251("LiDAR Z390",[75,0,lidar_z+12],12);
    color([.7,0,0]) translate([80,-4,lidar_z]) cube([520,8,1],center=true);
    if(show_envelopes) {
        pile(225.3,[.15,.75,.20,.20]);
        pile(236.5,[.95,.60,.05,.16]);
    }
}

module top_view() {
    basket(); ski(); feed_and_selector();
    // Rear motor keep-outs and externally accessible battery zone.
    for(y=[-180,180]) color([.85,.15,.15,.25]) translate([-65,y,85]) cube([36,40,38],center=true);
    color([.2,.2,.2,.18]) translate([-100,0,90]) cube([70,260,70],center=true);
}

module assembly() {
    basket(); ski(); feed_and_selector(); trajectory(); labels_side();
    color([.15,.15,.15,.25]) translate([300,0,32]) cube([20,440,20],center=true);
    color([.20,.20,.20,.25]) translate([350,0,305]) rotate([0,50,0]) cube([82,32,28],center=true);
    color([.3,.1,.7,.25]) translate([330,0,374]) cylinder(r=38,h=32,center=true);
}

if(view=="side") projection(cut=false) rotate([-90,0,0]) assembly();
else if(view=="top") projection(cut=false) top_view();
else assembly();

echo("P2_51_DEPENDENT_NO_STL_RELEASE",ski_radius=75,ski_geometric_angle=42,
     ski_arc_plus_tangent=106.98,basket_x=[basket_rear_x,basket_front_x],
     basket_y=2*basket_half_y,side_rim_z=side_rim_z,backboard_top_z=backboard_top_z,
     lidar_margin_backboard=lidar_z-backboard_top_z);
