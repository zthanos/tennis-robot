// P2-48 DEPENDENT basket panel study.  No STL/manufacturing release.
// All part selectors fit a nominal 256 x 256 mm print bed.
part = "assembly"; // [assembly,side_panel,end_panel,front_lower_left,front_lower_right,floor_panel,bridge_lower,bridge_lower_side,bridge_tail_247,bridge_tail_260,bridge_tail_275,bridge_tail_side_247,bridge_tail_side_260,bridge_tail_side_275,bridge_deck_slat,blind_plate,port_frame,seam_strap,corner_angle,mount_ear]
bridge_height = 260; // [247,260,275]
show_keepouts = false;
$fn = 36;

ball_d = 66;
basket_x0 = -15;
basket_x1 = 278;
basket_y = 350;
basket_rim_z = 335;
floor_rear_z = 54;
floor_front_z = 42;
wall_t = 3;
wall_frame = 10;
wall_rib = 8;
wall_open_max = 40;
floor_rib = 5;
floor_open = 18;
feed_lane_w = 86;
m4_clear = 4.5;

side_w = (basket_x1-basket_x0)/2; // 146.5
side_h = (basket_rim_z-floor_front_z)/2; // 146.5, conservative front datum
end_w = basket_y/2; // 175
end_h = side_h;
floor_w = side_w;
floor_h = end_w;

function floor_z(x) = floor_rear_z + (x-basket_x0)*(floor_front_z-floor_rear_z)/(basket_x1-basket_x0);
function tail_z(x,h) = 95 + (250-x)*(h-95)/(250-90);

module panel_holes(w,h,t=wall_t) {
    for(x=[-w/4,w/4],y=[-h/2+5,h/2-5]) translate([x,y,-1]) cylinder(d=m4_clear,h=t+2);
    for(x=[-w/2+5,w/2-5],y=[-h/4,h/4]) translate([x,y,-1]) cylinder(d=m4_clear,h=t+2);
}

module grid_panel(w,h,t=wall_t,opening=38,rib=wall_rib,frame=wall_frame) {
    inner_w=w-2*frame; inner_h=h-2*frame;
    nx=ceil(inner_w/(opening+rib)); ny=ceil(inner_h/(opening+rib));
    cell_x=inner_w/nx; cell_y=inner_h/ny;
    assert(w<=256 && h<=256, "panel exceeds 256 mm bed");
    assert(cell_x-rib<=wall_open_max && cell_y-rib<=wall_open_max, "wall opening exceeds 40 mm");
    difference() {
        union() {
            for(y=[-h/2+frame/2,h/2-frame/2]) translate([0,y,t/2]) cube([w,frame,t],center=true);
            for(x=[-w/2+frame/2,w/2-frame/2]) translate([x,0,t/2]) cube([frame,h,t],center=true);
            for(i=[1:nx-1]) translate([-inner_w/2+i*cell_x,0,t/2]) cube([rib,inner_h,t],center=true);
            for(i=[1:ny-1]) translate([0,-inner_h/2+i*cell_y,t/2]) cube([inner_w,rib,t],center=true);
        }
        panel_holes(w,h,t);
    }
}

module floor_panel(y_sign=1) {
    w=floor_w; h=floor_h; inner_h=h-2*wall_frame;
    ny=ceil(inner_h/(floor_open+floor_rib)); cell=inner_h/ny;
    assert(w<=256 && h<=256);
    difference() {
        union() {
            // Perimeter and longitudinal ribs are parallel to +X rolling.
            for(y=[-h/2+wall_frame/2,h/2-wall_frame/2]) translate([0,y,wall_t/2]) cube([w,wall_frame,wall_t],center=true);
            for(x=[-w/2+wall_frame/2,w/2-wall_frame/2]) translate([x,0,wall_t/2]) cube([wall_frame,h,wall_t],center=true);
            for(i=[1:ny-1]) translate([0,-inner_h/2+i*cell,wall_t/2]) cube([w-2*wall_frame,floor_rib,wall_t],center=true);
            // Half of the continuous 86 mm solid feed lane at the centre seam.
            translate([0,y_sign*(-h/2+feed_lane_w/4),wall_t/2]) cube([w,feed_lane_w/2,wall_t],center=true);
        }
        panel_holes(w,h,wall_t);
    }
}

module strip_between(x0,z0,x1,z1,width=96,t=3) {
    dx=x1-x0; dz=z1-z0; len=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    translate([(x0+x1)/2,0,(z0+z1)/2]) rotate([0,-a,0]) cube([len+1,width,t],center=true);
}

module bridge_lower() {
    pts=[[402,10],[377,13.25],[352,23.45],[330,31],[305,48],[280,69],[250,95]];
    for(i=[0:len(pts)-2]) strip_between(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1],56+(96-56)*i/(len(pts)-2));
}

module bridge_tail(h=260) {
    pts=[[250,95],[220,tail_z(220,h)],[190,tail_z(190,h)],[160,tail_z(160,h)],[130,tail_z(130,h)],[110,tail_z(110,h)],[90,h]];
    for(i=[0:len(pts)-2]) strip_between(pts[i][0],pts[i][1],pts[i+1][0],pts[i+1][1]);
    // Positive identification; no friction-only shim stack.
    translate([170,0,tail_z(170,h)+5]) linear_extrude(1) text(str(h),size=10,halign="center");
}

module profile_rail_2d(points,d=10) {
    for(i=[0:len(points)-2]) hull() {
        translate(points[i]) circle(d=d);
        translate(points[i+1]) circle(d=d);
    }
}

module bridge_lower_side() {
    p=[[0,0],[25,3.25],[50,13.45],[72,21],[97,38],[122,59],[152,85]];
    linear_extrude(5) profile_rail_2d(p);
}

module bridge_tail_side(h=260) {
    p=[[0,0],[30,tail_z(220,h)-95],[60,tail_z(190,h)-95],
       [90,tail_z(160,h)-95],[120,tail_z(130,h)-95],
       [140,tail_z(110,h)-95],[160,h-95]];
    assert(max([for(q=p) q[0]])<=256 && max([for(q=p) q[1]])<=256);
    linear_extrude(5) profile_rail_2d(p);
}

module bridge_deck_slat() {
    difference() {
        cube([34,96,3],center=true);
        for(x=[-12,12],y=[-40,40]) translate([x,y,-3]) cylinder(d=m4_clear,h=6);
    }
}

module seam_strap() {
    difference() {
        cube([100,18,5],center=true);
        for(x=[-37.5,-12.5,12.5,37.5]) translate([x,0,-4]) cylinder(d=m4_clear,h=8);
    }
}

module corner_angle() {
    difference() {
        union() {
            translate([0,8,70]) cube([4,16,140],center=true);
            translate([8,0,70]) cube([16,4,140],center=true);
        }
        for(z=[20,45,70,95,120]) {
            translate([-3,8,z]) rotate([0,90,0]) cylinder(d=m4_clear,h=10);
            translate([8,-3,z]) rotate([90,0,0]) cylinder(d=m4_clear,h=10);
        }
    }
}

module mount_ear() {
    difference() {
        hull() { translate([0,0,3]) cube([36,22,6],center=true); translate([0,0,20]) cube([18,22,6],center=true); }
        translate([0,0,-1]) cylinder(d=m4_clear,h=9);
        translate([0,0,12]) rotate([90,0,0]) cylinder(d=m4_clear,h=28,center=true);
    }
}

module blind_plate() {
    difference() {
        union() { cube([3,120,120],center=true); translate([-2,0,0]) cube([3,84,84],center=true); }
        for(y=[-52,52],z=[-52,52]) translate([-3,y,z]) rotate([0,90,0]) cylinder(d=m4_clear,h=10);
    }
}

module port_frame() {
    difference() {
        cube([6,120,120],center=true);
        cube([10,80,80],center=true);
        for(y=[-52,52],z=[-52,52]) translate([-3,y,z]) rotate([0,90,0]) cylinder(d=m4_clear,h=12);
    }
}

module side_quadrants(side=1) {
    for(ix=[0:1],iz=[0:1]) {
        xc=basket_x0+side_w/2+ix*side_w;
        zc=floor_front_z+side_h/2+iz*side_h;
        translate([xc,side*basket_y/2,zc]) rotate([90,0,0]) grid_panel(side_w,side_h);
    }
}

module end_quadrants(xpos,front=false) {
    difference() {
        union() for(iy=[0:1],iz=[0:1]) {
            yc=-basket_y/2+end_w/2+iy*end_w;
            zc=floor_front_z+end_h/2+iz*end_h;
            translate([xpos,yc,zc]) rotate([90,0,90]) grid_panel(end_w,end_h);
        }
        if(front) translate([xpos,0,floor_front_z+40]) cube([12,80,80],center=true);
    }
}

// Printable lower-front halves.  The opening touches the bottom and centre
// edges, so the assembled pair produces the same 80 x 80 port as assembly().
module front_lower_panel(left=true) {
    difference() {
        grid_panel(end_w,end_h);
        translate([(left ? 1 : -1)*(end_w/2-20),-end_h/2+40,wall_t/2])
            cube([40.2,80.2,wall_t+2],center=true);
    }
}

module sloped_floor() {
    a=atan2(floor_rear_z-floor_front_z,basket_x1-basket_x0);
    for(ix=[0:1],iy=[0:1]) {
        xc=basket_x0+floor_w/2+ix*floor_w;
        yc=-basket_y/2+floor_h/2+iy*floor_h;
        translate([xc,yc,floor_z(xc)]) rotate([0,a,0]) floor_panel(iy==0 ? 1 : -1);
    }
}

module port_frame_and_paths() {
    // 80 x 80 clear opening: one Ø66 ball, never two side-by-side.
    color([.95,.45,.08]) translate([281,0,floor_front_z+40]) port_frame();
    // Ball-centre keep-outs to the frozen P2-40 rear-mouth floor at +15/+20.
    if(show_keepouts) for(p=[[329.183,59.063],[331.475,54.115]])
        %color([1,.7,.05,.35]) hull() {
            translate([278,0,floor_front_z+33]) sphere(d=ball_d);
            translate([p[0],0,p[1]]) sphere(d=ball_d);
        }
}

module chassis_mounts() {
    // Actual X240/X300 top-grid holes, Y=±100/±150 mm.
    for(x=[240,300],y=[-150,-100,100,150])
        color([.6,.3,.8]) translate([x,y,35]) mount_ear();
}

module assembly() {
    color([.55,.72,.58]) { side_quadrants(1); side_quadrants(-1); end_quadrants(basket_x0); end_quadrants(basket_x1,true); sloped_floor(); }
    color([.10,.55,.9]) { bridge_lower(); bridge_tail(bridge_height); }
    port_frame_and_paths(); chassis_mounts();
    if(show_keepouts) %color([1,0,0,.25]) translate([330,0,390]) cube([900,900,1],center=true);
}

if(part=="assembly") assembly();
else if(part=="side_panel") grid_panel(side_w,side_h);
else if(part=="end_panel") grid_panel(end_w,end_h);
else if(part=="front_lower_left") front_lower_panel(true);
else if(part=="front_lower_right") front_lower_panel(false);
else if(part=="floor_panel") floor_panel(1);
else if(part=="bridge_lower") bridge_lower();
else if(part=="bridge_lower_side") bridge_lower_side();
else if(part=="bridge_tail_247") bridge_tail(247);
else if(part=="bridge_tail_260") bridge_tail(260);
else if(part=="bridge_tail_275") bridge_tail(275);
else if(part=="bridge_tail_side_247") bridge_tail_side(247);
else if(part=="bridge_tail_side_260") bridge_tail_side(260);
else if(part=="bridge_tail_side_275") bridge_tail_side(275);
else if(part=="bridge_deck_slat") bridge_deck_slat();
else if(part=="blind_plate") rotate([0,90,0]) blind_plate();
else if(part=="port_frame") rotate([0,90,0]) port_frame();
else if(part=="seam_strap") seam_strap();
else if(part=="corner_angle") corner_angle();
else if(part=="mount_ear") mount_ear();

echo("P2_48_DEPENDENT_NO_STL_RELEASE",required_capacity=40,best_effort_capacity=45,
     bridge_height=bridge_height,largest_panel=[end_w,end_h],feed_opening=[80,80],
     floor_slope_deg=atan2(floor_rear_z-floor_front_z,basket_x1-basket_x0),
     launch15_mouth_floor=[329.183,0,26.063],launch20_mouth_floor=[331.475,0,21.115]);
