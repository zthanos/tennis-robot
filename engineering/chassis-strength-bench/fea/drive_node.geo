// Local solid model: 160 mm of 50x30 rail, flush internal roof reinforcement,
// oriented steel-foot pattern and four M3 clearance holes. Units are metres.
SetFactory("OpenCASCADE");

rail_length = 0.160;
rail_width = 0.050;
rail_height = 0.030;
rail_wall = 0.0032;
node_length = 0.070;
node_width = 0.048;
local_roof_thickness = 0.014;
hole_radius = 0.0017;
hole_row_x = 0.015;
hole_y_inner = -0.014;
hole_y_motor = 0.011;
nut_tower_radius = 0.0085;
nut_pocket_height = 0.0037;
nut_top_cover = 0.0040;
nut_pocket_z = rail_height - nut_top_cover - nut_pocket_height;
nut_access_width = 0.0082;
mount_screw_length = 0.0120;
mount_foot_thickness = 0.0020;
mount_washer_thickness = 0.0005;
screw_tip_clearance = 0.0015;
blind_hole_bottom_z = rail_height + mount_foot_thickness
                    + mount_washer_thickness - mount_screw_length
                    - screw_tip_clearance;
web_thickness = 0.0024;
inner_half_width = rail_width/2 - rail_wall;
inner_height = rail_height - 2*rail_wall;
web_dy = 2*inner_half_width/3;
web_length = Sqrt(web_dy*web_dy + inner_height*inner_height);
web_outer_angle = -Atan2(web_dy, inner_height);
web_middle_angle = -Atan2(web_dy, -inner_height);

Box(1) = {-rail_length/2, -rail_width/2, 0,
          rail_length, rail_width, rail_height};
// Extend the void beyond both cut faces so the rail is open-ended.
Box(2) = {-rail_length/2 - 0.001, -rail_width/2 + rail_wall, rail_wall,
          rail_length + 0.002, rail_width - 2*rail_wall,
          rail_height - 2*rail_wall};
rail[] = BooleanDifference{ Volume{1}; Delete; }{ Volume{2}; Delete; };

// The top remains flush. This block thickens the roof downward from 3.2 mm
// to 8 mm only inside the 60x44 drive zone.
Box(3) = {-node_length/2, -node_width/2,
          rail_height - local_roof_thickness,
          node_length, node_width, local_roof_thickness};
Cylinder(4) = {-hole_row_x, hole_y_inner, rail_wall - 0.0001,
                0, 0, rail_height - 2*rail_wall + 0.0002, nut_tower_radius};
Cylinder(5) = {-hole_row_x, hole_y_motor, rail_wall - 0.0001,
                0, 0, rail_height - 2*rail_wall + 0.0002, nut_tower_radius};
Cylinder(6) = { hole_row_x, hole_y_inner, rail_wall - 0.0001,
                0, 0, rail_height - 2*rail_wall + 0.0002, nut_tower_radius};
Cylinder(7) = { hole_row_x, hole_y_motor, rail_wall - 0.0001,
                0, 0, rail_height - 2*rail_wall + 0.0002, nut_tower_radius};

// Three longitudinal diagonal plates reproduce the printable W cross-section.
// They support the roof and divide the shell into torsion-resistant cells.
Box(40) = {-rail_length/2, -inner_half_width - web_thickness/2, rail_wall,
           rail_length, web_thickness, web_length};
Rotate {{1,0,0}, {0,-inner_half_width,rail_wall}, web_outer_angle}
       { Volume{40}; }
Box(41) = {-rail_length/2, -inner_half_width/3 - web_thickness/2,
           rail_height - rail_wall,
           rail_length, web_thickness, web_length};
Rotate {{1,0,0}, {0,-inner_half_width/3,rail_height-rail_wall},
        web_middle_angle} { Volume{41}; }
Box(42) = {-rail_length/2, inner_half_width/3 - web_thickness/2, rail_wall,
           rail_length, web_thickness, web_length};
Rotate {{1,0,0}, {0,inner_half_width/3,rail_wall}, web_outer_angle}
       { Volume{42}; }
solid[] = BooleanUnion{ Volume{rail[]}; Delete; }
                      { Volume{3,4,5,6,7,40,41,42}; Delete; };

Cylinder(10) = {-hole_row_x, hole_y_inner, blind_hole_bottom_z,
                 0, 0, rail_height - blind_hole_bottom_z + 0.002, hole_radius};
Cylinder(11) = {-hole_row_x, hole_y_motor, blind_hole_bottom_z,
                 0, 0, rail_height - blind_hole_bottom_z + 0.002, hole_radius};
Cylinder(12) = { hole_row_x, hole_y_inner, blind_hole_bottom_z,
                 0, 0, rail_height - blind_hole_bottom_z + 0.002, hole_radius};
Cylinder(13) = { hole_row_x, hole_y_motor, blind_hole_bottom_z,
                 0, 0, rail_height - blind_hole_bottom_z + 0.002, hole_radius};

// Open U-seats: a full-width approach reaches the screw axis and an
// equal-diameter round end provides a smooth far stop.
// Conservative FEA envelope for the real open three-sided hex seat: these
// circular cuts remove slightly more material than its three flat faces.
Cylinder(20) = {-hole_row_x, hole_y_inner, nut_pocket_z,
                0, 0, nut_pocket_height, nut_access_width/2};
Cylinder(21) = {-hole_row_x, hole_y_motor, nut_pocket_z,
                0, 0, nut_pocket_height, nut_access_width/2};
Cylinder(22) = { hole_row_x, hole_y_inner, nut_pocket_z,
                0, 0, nut_pocket_height, nut_access_width/2};
Cylinder(23) = { hole_row_x, hole_y_motor, nut_pocket_z,
                0, 0, nut_pocket_height, nut_access_width/2};
Box(24) = {-hole_row_x - nut_access_width/2, -rail_width/2 - 0.001,
           nut_pocket_z, nut_access_width,
           rail_width/2 + hole_y_inner + 0.001,
           nut_pocket_height};
Box(25) = { hole_row_x - nut_access_width/2, -rail_width/2 - 0.001,
           nut_pocket_z, nut_access_width,
           rail_width/2 + hole_y_inner + 0.001,
           nut_pocket_height};
Box(26) = {-hole_row_x - nut_access_width/2,
           hole_y_motor,
           nut_pocket_z, nut_access_width,
           rail_width/2 - hole_y_motor + 0.001,
           nut_pocket_height};
Box(27) = { hole_row_x - nut_access_width/2,
           hole_y_motor,
           nut_pocket_z, nut_access_width,
           rail_width/2 - hole_y_motor + 0.001,
           nut_pocket_height};
final[] = BooleanDifference{ Volume{solid[]}; Delete; }
                          { Volume{10,11,12,13,20,21,22,23,24,25,26,27}; Delete; };

Physical Volume("SOLID") = {final[]};

Mesh.MeshSizeMin = 0.0012;
Mesh.MeshSizeMax = 0.0040;
Mesh.MeshSizeFromCurvature = 18;
// First-order tetrahedra keep the three-material screening run below a minute.
// A second-order mesh-convergence case is required before manufacturing release.
Mesh.ElementOrder = 1;
Mesh.SaveGroupsOfNodes = 1;
