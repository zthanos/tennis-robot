use <rear-pod/rear-pod-v2.scad>
use <body-panels/body-panels-f-rear-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

view = "iso";

color([.30,.32,.35]) frame_assembly();
assembly();
color([.95,.47,.08]) p288_new_parts_installed();
p288_battery_ghosts();
color([.12,.42,.58,.28]) all_f_rear(view=="service"?110:0);
color([.82,.82,.85,.72]) p288_access_hardware();
