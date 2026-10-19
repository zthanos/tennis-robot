// Third ALTERNATIVE ONLY: low L basket, battery outside it, rear corner extensions.
// mm +X forward. No released joints, STL, load rating or solved ball feeder.
$fn=32;
use <modular-chassis-v2.scad>
use <height-infrastructure-study.scad>
use <rear-low-launcher-study.scad>

view="assembly"; // [assembly,top,side,qa_battery_frame,qa_battery_basket,qa_launcher_battery,qa_launcher_basket,qa_feed_battery,qa_feed_launcher,qa_entry_battery,qa_entry_basket,qa_exit_frame,qa_chute_launcher,qa_basket_frame,basket_only]
show_paths=true;
battery_position="front"; // [front,rear] packaging comparison only; rear collector too narrow
pitch=20;
nip_x=-260;
nip_z=140;
extension_x0=-365;
extension_x1=-165;
rail_y=220;
rail_bottom=22;
rail_top=52;
battery_size=[198,166,170]; // upright: long axis along robot X, narrow axis across Y
battery_mass_kg=5.59;
battery_x=battery_position=="rear" ? 40 : 155;
battery_center=[battery_x,162,137]; // bottom Z52, outer edge Y245: 50 mm contact on side rail
battery_min=[for(i=[0:2]) battery_center[i]-battery_size[i]/2];
battery_max=[for(i=[0:2]) battery_center[i]+battery_size[i]/2];
bay_x0=battery_min[0]-10;
bay_x1=battery_max[0]+10;
bay_y0=battery_min[1]-10;
assert(battery_max[1]<=245,"Keep battery inside outer side-rail edge");
assert(battery_max[0]<=269.4-15,"Keep 15 mm before fixed front brace");
strip_bottom=20;
strip_t=3; // space reservation, NOT validated for 5.59 kg
floor_rear=48; // upper surface, close to chassis top52
floor_front=52;
basket_top=220;
intake_exit_z=35;
feed_local_x=-185;
function pose(p)=[nip_x-cos(pitch)*p[0]+sin(pitch)*p[2],-p[1],
                  nip_z+sin(pitch)*p[0]+cos(pitch)*p[2]];
feed=pose([feed_local_x,0,0]);
function floor_z(x)=floor_rear+(x+120)*(floor_front-floor_rear)/430;
// Widen rear collector leg and reopen a full-width front mouth before the bay.
outer=[[-120,-185],[310,-185],[310,185],[bay_x1,185],
       [bay_x1,bay_y0],[bay_x0,bay_y0],[bay_x0,185],[-120,185]];
inner=[[-117,-182],[307,-182],[307,182],[bay_x1+3,182],
       [bay_x1+3,bay_y0-3],[bay_x0-3,bay_y0-3],[bay_x0-3,182],[-117,182]];
echo("STUDY_ONLY",nip_mm=[nip_x,0,nip_z],pitch_deg=pitch,
     battery_mass_kg=battery_mass_kg,battery_bounds_mm=[battery_min,battery_max],
     basket_floor_mm=[floor_rear,floor_front],intake_surface_step_mm=floor_front-intake_exit_z,
     feed_center_mm=feed,rear_extension_mm=extension_x1-extension_x0);

module launcher_pose_new() {
    translate([nip_x,0,nip_z]) rotate([0,0,180]) rotate([0,-pitch,0]) children();
}
module rear_launcher() {
    // Reuse old visual assembly at (-70,0,175), translated as one whole unit.
    translate([nip_x+70,0,nip_z-175]) launcher_hardware();
}
module rear_extensions() {
    color([0.60,0.35,0.65]) for(side=[-1,1]) {
        translate([(extension_x0+extension_x1)/2,side*rail_y,37])
            horizontal_x(extension_x1-extension_x0,50,30);
        // 70 mm flare / corner landing analogous in purpose to intake Gamma foot.
        translate([extension_x0+35,side*rail_y,37]) cube([70,70,30],center=true);
        // Joint reservation only. Actual overlap/bolt/nut access must be designed.
        %translate([extension_x1,side*rail_y,37]) cube([50,50,30],center=true);
    }
}
module rear_cradle() {
    color([0.60,0.35,0.65]) {
        launcher_pose_new() for(side=[-1,1]) lower_plate_lock_rail(side);
        for(xx=[-100,70]) {
            p=pose([xx,0,-57]);
            assert(p[0]>=extension_x0 && p[0]<=extension_x1);
            // Crossbars seat into corner-extension landing nodes; local overlap
            // denotes a new integrated joint, NOT a press-fit or released socket.
            translate([p[0],0,p[2]]) rotate([0,0,90]) horizontal_x(440);
            for(yy=[-220,220]) {
                if(p[2]>rail_top+5) translate([p[0],yy,rail_top]) tube_z(p[2]-rail_top);
                else translate([p[0],yy,47]) cube([50,50,10],center=true);
            }
        }
    }
}
module battery() { translate(battery_center) cube(battery_size,center=true); }
module battery_strips() {
    // Second contact rail and two cantilever ties: packaging reservations ONLY.
    // No released joints, retention or strength approval for the 5.59 kg battery.
    color([0.55,0.55,0.55]) {
        translate([battery_center[0],95,47]) cube([battery_size[0],20,10],center=true);
        for(xx=[battery_center[0]-70,battery_center[0]+70]) {
            translate([xx,165,47]) cube([20,160,10],center=true);
        }
    }
}
module prism(points,inner_volume=false) {
    n=len(points);
    vertices=concat([for(p=points) [p[0],p[1],floor_z(p[0])+(inner_volume ? 0 : -3)]],
                    [for(p=points) [p[0],p[1],basket_top+(inner_volume ? 5 : 0)]]);
    faces=concat([[for(i=[n-1:-1:0]) i]],[[for(i=[0:n-1]) i+n]],
        [for(i=[0:n-1]) [i,(i+1)%n,(i+1)%n+n,i+n]]);
    polyhedron(points=vertices,faces=[for(f=faces) [for(j=[len(f)-1:-1:0]) f[j]]]);
}
module low_basket() {
    color([0.25,0.65,0.35,0.55]) difference() {
        prism(outer); prism(inner,true);
        // Rear branch widened by the battery move; multi-ball flow NOT validated.
        translate([-79.5,0,50]) cube([72,80,25],center=true);
        // Open intake mouth at centre; then turn into side channel before battery.
        translate([308,0,85]) cube([12,190,90],center=true);
    }
}
module feed_funnel() {
    color([0.25,0.65,0.35,0.55]) difference() {
        hull() {
            translate([-79.5,0,floor_z(-79.5)]) cube([74,84,3],center=true);
            translate([feed[0],0,feed[2]-31]) cube([74,74,3],center=true);
        }
        hull() {
            translate([-79.5,0,floor_z(-79.5)+4]) cube([68,78,12],center=true);
            translate([feed[0],0,feed[2]-29]) cube([68,68,12],center=true);
        }
    }
    color([0.25,0.65,0.35]) launcher_pose_new()
        translate([-159,0,-35]) cube([62,82,4],center=true);
    // Space for a one-ball escapement/pusher remains unresolved; 20° entry uphill.
    %color([0.1,0.65,0.95]) translate([feed[0],-140,feed[2]-15]) cube([45,70,40],center=true);
}
module feed_ball_sweep() {
    hull() {
        translate([-79.5,0,floor_z(-79.5)+33]) sphere(d=66);
        translate(feed) sphere(d=66);
    }
}
module entry_ball_sweep() {
    // Candidate transfer/deflection from intake centre towards left channel.
    // 17 mm uphill step remains: not proof that intake drives balls across it.
    hull() {
        translate([319,0,intake_exit_z+33]) sphere(d=66);
        translate([275,-50,floor_front+33]) sphere(d=66);
    }
}
module exit_path() {
    launcher_pose_new() translate([190,0,0]) rotate([0,90,0]) cylinder(d=76,h=340,center=true);
}
module basket_posts() {
    color([0.15,0.60,0.65]) for(xx=[-110,300],yy=[-195,195]) {
        translate([xx,yy,52]) tube_z(basket_top-52);
        translate([xx,yy>0 ? 210 : -210,50]) cube([20,30,4],center=true);
    }
}
module fixed_frame() { frame_structure(); intake_context(); }
module assembly() {
    fixed_frame();
    for(side=[-1,1],xx=[-65,300]) wheel_context(side,xx);
    rear_extensions(); rear_cradle(); rear_launcher();
    battery_strips(); color([0.15,0.35,0.75,0.6]) battery();
    low_basket(); feed_funnel(); basket_posts();
    %color([1,0.45,0.08,0.4]) entry_ball_sweep();
    if(show_paths) {
        %color([0.1,0.65,0.95,0.25]) exit_path();
        %color([0.1,0.65,0.95,0.25]) feed_ball_sweep();
    }
}
if(view=="qa_battery_frame") intersection() { battery(); union(){fixed_frame();rear_extensions();rear_cradle();} }
else if(view=="qa_battery_basket") intersection() { battery(); prism(outer); }
else if(view=="qa_launcher_battery") intersection() { rear_launcher(); battery(); }
else if(view=="qa_launcher_basket") intersection() { rear_launcher(); low_basket(); }
else if(view=="qa_feed_battery") intersection() { feed_ball_sweep(); battery(); }
else if(view=="qa_feed_launcher") intersection() { feed_ball_sweep(); rear_launcher(); }
else if(view=="qa_entry_battery") intersection() { entry_ball_sweep(); battery(); }
else if(view=="qa_entry_basket") intersection() { entry_ball_sweep(); low_basket(); }
else if(view=="qa_exit_frame") intersection() { exit_path(); union(){fixed_frame();rear_extensions();rear_cradle();basket_posts();battery_strips();} }
else if(view=="qa_chute_launcher") intersection() { feed_funnel(); rear_launcher(); }
else if(view=="qa_basket_frame") intersection() { low_basket(); fixed_frame(); }
else if(view=="basket_only") low_basket();
else if(view=="side") intersection() { assembly(); translate([-800,-700,-10]) cube([1400,700,650]); }
else assembly();
