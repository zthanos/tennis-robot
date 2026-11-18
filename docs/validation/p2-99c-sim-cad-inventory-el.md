# P2-99c — Sim robot vs τρέχον CAD (body F): Φάση 1, απογραφή

Κατάσταση: **ΑΠΟΓΡΑΦΗ ΜΟΝΟ — 2026-11-18.** Καμία αλλαγή σε xacro, launch,
params ή κώδικα. Δεν έτρεξε sim, ROS ή Gazebo.

## τι διαφέρει από το αναμενόμενο

1. **Το `lidar_xyz` της P2-99b δεν μπαίνει αυτούσιο στο σημερινό URDF.** Η
   P2-99b (§6) δίνει `[-0.1985, +0.110, 0.345]`, δηλαδή υποθέτει ότι
   `base_link X = CAD X`. Στο sim όμως το `base_link` είναι στο **μέσο των
   αξόνων** (τροχοί ±0,330), ενώ στο CAD το μέσο των αξόνων είναι στο
   **CAD X+117,5** (άξονες −65/+300, `icr=[117,0]` του body F). Αν κρατήσουμε τη
   σημερινή σύμβαση (`base_link` = κέντρο περιστροφής skid-steer, που χρειάζεται
   το `diff_drive_controller`), τότε **`lidar_x = −0,316`**, όχι −0,1985. Το
   isolated v3 bench URDF χρησιμοποιεί την άλλη σύμβαση (`base_link` = CAD
   origin στο έδαφος). **Απόφαση D1 πριν από κάθε βήμα.**
2. **Αν μετακινηθεί μόνο ο LiDAR, το sim τυφλώνεται σχεδόν 360°.** Το σημερινό
   κέλυφος έχει οροφή στα 0,494 m. Με scan στα 0,4198 m, το κέλυφος και οι παλιές
   ράγες του καλαθιού κόβουν τη ζώνη ±2° (§2). Το βήμα (α) πρέπει να αλλάξει
   LiDAR, κέλυφος και ράγες σε ένα ατομικό βήμα.
3. **Η OAK-D του CAD δεν βλέπει μακριά.** Η P2-07 και το `ko_camera_fov` δίνουν
   pitch **50°** προς τα κάτω, στο ύψος Z305. Με VFOV 54,5° η κάμερα βλέπει
   έδαφος μόνο από **0,07 έως 0,73 m** μπροστά της και δεν βλέπει ορίζοντα. Το sim
   σήμερα έχει 15,6° και βλέπει από 0,52 m ως τον ορίζοντα. Το 360° scan του
   `collect_route` και η αναζήτηση μπαλών προϋποθέτουν ανίχνευση σε μεγάλη
   απόσταση, οπότε αυτό δεν είναι διόρθωση URDF. Είναι **σχεδιαστική απόφαση D2**.
   Επιπλέον, ο προσανατολισμός της OAK πάνω στο universal plate δεν τεκμηριώνεται
   ρητά στο CAD: το 50° προκύπτει από την P2-07 και από το
   `rotate([0,50,0])`. Χρειάζεται **ΕΠΙΒΕΒΑΙΩΣΗ ΧΡΗΣΤΗ**.
4. **Ο μηχανισμός του sim δεν έχει κανένα κοινό στοιχείο με το CAD.** Σήμερα
   μοντελοποιούνται: dual side-wheel intake 35°, cheeks/ramp, sunken bin v2 με
   **prismatic lift** 100 mm, ξεχωριστός σταθερός launcher +20° και plywood
   bridge. Το CAD έχει ένα κοινό flywheel με **revolute tilt** −3/0/+20°, σταθερό
   καλάθι P2-71, ski R75/75 και αψίδα P2-93. Υπάρχει ήδη **isolated v3 bench URDF**
   (`engineering/shared-flywheel-isolated-bench/v3_chassis_flywheel_bench.urdf.xacro`)
   με σασί v3, `launcher_tilt_joint` και flywheels σε CAD frame, που μπορεί να
   επαναχρησιμοποιηθεί. Όμως: (α) ο actuator του είναι το παλιό P16, όχι το
   FIT0186+Tr8· (β) οι τροχοί είναι D200×60, ενώ οι παραγγελμένοι είναι
   D192×48 και η απόσταση των αξόνων μένει ανοικτή.
5. **Η OAK στη θέση του CAD, πάνω στο σημερινό URDF, θα έβλεπε μόνο τον παλιό
   μηχανισμό.** Στο FOV μπαίνουν 45 visuals: τροχοί intake, cheeks, flywheels,
   bridge. Το βήμα (β) δεν περνά gate πριν από το (δ1).
6. **Μάζα: sim 32,2 kg, CAD bench ≈20,6 kg** (με 45 μπάλες). Η μπαταρία 6,26 kg
   του sim δεν υπάρχει πια στο design: το CAD έχει δύο Parkside των 0,65 kg
   (placeholder) και UGREEN. Το CoG X όμως συμπίπτει σχεδόν (+45 έναντι +39 mm
   από το μέσο των αξόνων).
7. Βρέθηκαν **ήδη υπάρχουσες ασυνέπειες**, ανεξάρτητες από το F:
   - Το Nav2 footprint `±0,46 × ±0,29` δεν περιλαμβάνει τους τροχούς του sim
     (|Y| ως 0,39) ούτε τα cheeks μπροστά (X 0,705).
   - Τα `PERCEPTION_CAMERA_X_M` στο `controller_node.py` και στο
     `court_survey_v2_node.py` έχουν default 0,535, ενώ το URDF έχει 0,690.
   - Το σχόλιο `range_min` του C1 λέει 0,10, ενώ η επίσημη τιμή είναι 0,05 (ήδη
     από την P2-99a).
8. Ο host `xacro` δεν τρέχει (Python 3.14, Ubuntu 26.04). Η απογραφή βασίστηκε
   στο `runtime/tennis_robot.urdf` (generated 2026-10-11, μετά το τελευταίο
   commit του xacro στις 2026-10-07) και στα xacro sources. Η επαλήθευση
   με render μέσα από `docker/jazzy/run.sh` είναι μέρος του πρώτου gate.
9. Η θέση του C1 (P2-99b) είναι ακόμη **FIT-CHECK**: λείπουν οι δύο μετρήσεις του
   dummy. Το βήμα (α) πρέπει να την κρατήσει παραμετρική.

## 0. Πηγές και σύστημα αναφοράς

- Sim: `runtime/tennis_robot.urdf` (39 links, το output του
  `scripts/generate_robot_urdf.py`) και τα
  `ros2_ws/src/tennis_robot/urdf/{tennis_robot.urdf.xacro,components/*.xacro}`.
- CAD: `cad/body-panels/body-panels-f*.scad`,
  `cad/modular-test-chassis/{modular-chassis-v3,shared-flywheel-intake-launcher-phase1,shared-flywheel-p271-basket-grid}.scad`,
  `cad/arch-tilt-p293/p293d-arch-candidate.scad` (`p293d_oak_mount`, όπως
  χρησιμοποιείται από το `p293g-tilt-candidate.scad`),
  `docs/hardware/p2-99b-panel-collar-rplidar-c1-pedestal-el.md`, το v3 bench
  URDF και τα `engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation/*.validation.json`.
- Ο πίνακας χρησιμοποιεί την προτεινόμενη σύμβαση D1:
  **`base = CAD − [0.1175, 0, 0.045]`** (`base_link` στο μέσο των αξόνων, 45 mm
  πάνω από το έδαφος, όπως σήμερα). «Ground» σημαίνει ύψος από το έδαφος.

## 1. Πίνακας URDF ↔ CAD

| Link / joint | URDF σήμερα (base_link, m) | CAD πηγή | CAD σε base_link (m) | Απόκλιση (mm) | Επίπτωση στο sim |
| --- | --- | --- | --- | --- | --- |
| `base_footprint→base_link` | Z 0,045, origin στο μέσο αξόνων ±0,33 | v3: άξονες X−65/+300, icr X117 | origin = CAD X117,5 | σύμβαση (D1) | Όλα τα παρακάτω εξαρτώνται από αυτή |
| `base_link` (σασί) | plywood 0,92×0,58×0,014, 7,2 kg, άνοιγμα bin, τοίχοι 0,14 (visual) | `modular-chassis-v3`: ράγες 50×30 στο Y±220, X−165…330, crossbars | ράγες X −0,2825…+0,2125 | μήκος −425, πλάτος −80 | Collision του plate/βάσης· mass lump |
| 4× `*_wheel_joint` | X ±0,330, Y ±0,350, r 0,085, w 0,080, 0,25 kg | v3: X−65/+300, Y±297, Z85, r85 ref (0,082–0,090 unmeasured), w 83 | X ±0,1825, Y ±0,297 | wheelbase **660→365 (−295)**, track **700→594 (−106)** | Skid-steer turning, `wheel_separation: 1.0` (εμπειρικό ×1,4) πρέπει να ξαναρυθμιστεί, footprint, odometry |
| 4× `*_motor_link` | κύλινδροι Y ±0,24, 0,65 kg | FIT0403 inline στις ράγες ±220 | Y ±0,22 | ~20 | Μάζα/inertia· η μάζα FIT0403 δεν είναι μετρημένη |
| `panels_link` (κέλυφος) | **μόνο visual**, 2,5 kg· X −0,485…+0,46, Y ±0,29, οροφή ground **0,494** | body F: X −307,5…+511, Y ±244,5, Z60…350 (πίσω deck), rim Z320, εμπρός band Z105…320· `ko_lidar` = ό,τι είναι κάτω από Z375 | X −0,425…+0,3935, Y ±0,2445, οροφή ground 0,350 | πίσω +60, εμπρός −66, πλάγια −45,5/πλευρά, οροφή **−144** | Το gpu_lidar και η κάμερα κάνουν render visuals → occlusion· το κέλυφος δεν έχει collision (ούτε σήμερα ούτε στο CAD context) |
| `lidar_joint` / `front_lidar` | link [−0,420, 0, 0,4735], scan ground **0,5535**, head Ø94×45 centred, offset 0,035, `range_min` 0,15, 500 δείγματα, 31,25 Hz | P2-99b: κέντρο C1 CAD [−198,5, +110], βάση Z390, scan **Z419,8**, σώμα 55,6×55,6×41,3, offset 29,8 | link [**−0,316, +0,110, 0,345**], scan ground 0,4198 | X +104, Y +110, scan **−133,7** | SLAM/survey σε νέο ύψος σάρωσης· ασύμμετρο Y· self-hits (§2)· το 31,25 Hz δεν είναι ρυθμός C1 (10 Hz typ.) — να ελεγχθεί χωριστά |
| P2-97 tower + `panel_collar` + pedestal | **δεν υπάρχει** (μόνο ιστός 25×25 του lidar) | panel 138×190 με κέντρο CAD [−198,5, 0], top Z323· pedestal 58×68,75×67 ως Z390· E-stop [0,−58] | κέντρο [−0,316, 0], top ground 0,323 | νέο στοιχείο | Φορέας του C1· visual/collision· οτιδήποτε ως Z390 μένει 29,8 mm κάτω από το scan |
| `camera_joint` (OAK-D) | [0,690, 0, 0,443], ground 0,488, pitch **0,273 (15,6°)**, HFOV 1,204 | `p293d_oak_mount`: plate [450, 0, 305], `rotate([0,50,0])`· P2-07: pitch **50°** | [0,3325, 0, 0,260], ground 0,305, pitch 0,873 | X **−357,5**, Z **−183**, pitch **+34,4°** | Έδαφος 0,07–0,73 m αντί για 0,52 m–ορίζοντα (D2)· `PERCEPTION_CAMERA_X_M` hardcoded |
| Αψίδα P2-93 (legs, crossbar, masts, top beam) | **δεν υπάρχει** | πόδια X318→390, crossbar X390 Z245, top beam ~Z364 | X 0,20…0,27, ground ως 0,364 | νέο στοιχείο | Occlusion της κάμερας (σήμερα μηδέν)· μένει κάτω από το scan με περιθώριο ≥35 mm (P2-99b) |
| `imu_link` | [−0,05, 0, 0,012] | P2-77 rail-side mount CAD [112,5, 195, 37] | [−0,005, +0,195, −0,008] | Y +195 | Μικρή (μόνο lever arm επιταχύνσεων) |
| `ir_left/right` (entry beam) | X 0,62, Y ±0,105, ground 0,040 | Μόνο Mega D36/D37 — ακριβής θέση στο ενεργό CAD: **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ** | — | — | Δεμένο με το παλιό intake |
| `basket_ir_*` (επιβεβαίωση) | X 0,25, Y ±0,105, ground 0,075 | ίδιο κενό | — | — | Η επιβεβαίωση συλλογής του `collect_route` εξαρτάται από αυτό |
| Intake: `intake_wheel_*`, `compact_intake_cheeks`, `compact_handoff_ramp` | 2 τροχοί r0,062 με κλίση 35°, nip base 0,37, cheeks ως X 0,705· μάζα ≈1,0 kg | **καταργήθηκε**: συλλογή με το κοινό flywheel στις −3° και παθητικό ski R75/75 | — | ολόκληρο το υποσύστημα | Η συλλογή στο sim δεν αντιστοιχεί στο design (ήδη 0 Gazebo captures μετά το #63/#64) |
| `compact_bridge_link`, `basket_rails_link`, `compact_fixed_entry_hood` | plywood bridge 1,19 kg, ράγες lift 0,82 kg ως ground **0,455**, hood | **καμία αντιστοιχία** (στη θέση τους: αψίδα P2-93 + pivot supports) | — | — | Οι ράγες κόβουν το νέο scan (§2) |
| `flywheel_launcher_frame` + `flywheel_{l,r}_joint` | **fixed** στο bridge [0,46, 0, 0,17], +20°, 3,83 kg + 2×0,40 kg, άξονες ±0,129 | P2-40: **revolute** pivot CAD [387, 0, 51,8], nip 55 mm μπροστά, D200×60 στο ±129, module ≈4,61 kg· −3/0/+20 (μηχανικά stops −12/+22) | pivot [0,2695, 0, 0,0068] | ο τύπος joint διαφέρει· pivot ~190 mm πίσω | Δεν υπάρχει tilt joint/controller· οι πραγματικοί τροχοί D192×48 και η απόσταση των αξόνων είναι ανοικτά |
| Tilt actuator + AS5600 + stops | — | FIT0186 + Tr8×2 με ιμάντα, AS5600 στο hinge, switches −3/+20 | — | νέο | ros2_control: position joint· στο bench μοντελοποιείται ως P16 (παλιό) |
| `basket_joint` (prismatic) + `basket_link` | bin v2, lift 0…0,100, 1,012 + 2,565 kg, X base −0,08…0,32, ±0,14, τοίχος ground 0,25 | P2-71: **σταθερό**, floor X−85…278 (Z70→42), Y±172, front wall X225 ως Z235 με θύρα 80×80, rear wall X−118 ως Z320 | X −0,2355…+0,1605, ±0,172, ground ως 0,320 | λειτουργία (lift → fixed) | `basket_velocity_controller` χωρίς αντικείμενο· η χωρητικότητα και οι θέσεις των beams αλλάζουν |
| `battery_link` | 6,26 kg, 0,166×0,198×0,170 στο X −0,255 | 2× Parkside 135×85×75 στο [−165, ±110, 112] (**provisional**, 0,65 kg/τεμάχιο placeholder) + UGREEN 115×45×45 στο [−100, 0, 145], 0,309 kg | Parkside [−0,2825, ±0,110, 0,067] | **−4,96 kg** | CoG, inertia, δυναμική πέδησης/στροφής |
| `motion_electronics_tray`, `pi_case`, `pi_buck` | κάθετος δίσκος στο X −0,3975, 0,85 kg· Pi [−0,255, −0,19] | rear pod v2 / P2-97 stack (Mega, perfboard, 4×BTS7960, Pi 5) | γύρω από [−0,316, 0] | ~60–100 | Μάζα· το tower είναι και visual κοντά στο C1 |
| `intake_debug_camera` (sim-only) | [0,68, 0, 0,06], κοιτά πίσω στον παλιό διάδρομο | — | — | — | Πρέπει να στοχεύσει ξανά στο nip [0,3245, 0, ~0,01] |
| **Σύνολο μάζας** | **32,22 kg**, CoG base [+0,045, −0,002], ground Z 0,109 | bench v3 full45: **20,58 kg**, CoG CAD [0,1567, −0,0009, 0,1098]· empty 18,02 kg | CoG base [+0,039, −0,001], ground Z 0,110 | **−11,6 kg (−36 %)**, CoG X −6 mm | Το base 10,0 kg του bench είναι lumped placeholder· καμία ζυγισμένη μάζα: **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ** |
| Nav2 footprint | `±0,46 × ±0,29` | body F + τροχοί + flywheel discs (X542 στο 0°) | X −0,425…+0,4245, Y ±0,3385 | ήδη λάθος σήμερα (§«τι διαφέρει» 7) | Costmap/collision checking |
| `collection_route.yaml` | robot 0,8×0,6, funnel 0,4×0,34, capture ½ 0,17 | είσοδος cheeks Y±72 (narrow) / ±110 (wide), nip gap 58 | capture ½ ≈0,072–0,110 | −60…−98 | Γεωμετρία capture του planner (config, όχι URDF) |

## 2. Τι κόβει το FOV κάθε αισθητήρα

Μέθοδος: δειγματοληψία στα AABB των visuals του `runtime/tennis_robot.urdf` στη
μηδενική στάση. Για τον LiDAR: κάθε visual του οποίου το εύρος Z τέμνει το
`Zscan ± r·tan 2°`. Για την κάμερα: σημεία μέσα σε HFOV 69° × VFOV 54,5°. Είναι
γεωμετρικό screen, όχι render. Το script μένει στο scratchpad και θα γίνει
μόνιμο gate στο βήμα (α).

### LiDAR

| Σενάριο | Visuals μέσα στη ζώνη ±2° | Τυφλές γωνίες (az, 0 = εμπρός, + = αριστερά) |
| --- | --- | --- |
| Σήμερα [−0,42, 0], scan 0,5535 | **κανένα** | 0°. Ελάχιστο περιθώριο: κορυφή κεφαλής OAK 0,5127 έναντι κάτω ορίου ζώνης ≈0,5147 στα r 1,11 m → **≈2 mm** |
| Νέα θέση P2-99b [−0,1985, +0,110], scan 0,4198, **στο σημερινό URDF** | `panels_link`: left/right side, rear curve, front housings, front brow· `basket_rails_link` (ως 0,455)· `camera_link/lower_mount_stem` | ≈ **−121…+239°**, δηλαδή σχεδόν 360°: rear 143…239, αριστερή πλευρά 16…144, δεξιά −121…−32, εμπρός −33…+16, ράγες 18…28 και −63…−57 |
| Νέα θέση στο CAD (P2-99b §5) | κανένα, στις −10/−3/0/15/20° | 0°· δυσμενέστερο περιθώριο η top beam της αψίδας: 35,2 mm |

Στόχος του βήματος (α): το sim να αναπαράγει το αποτέλεσμα του CAD, δηλαδή 0°
και self-hits = 0. Το βήμα (δ1) πρέπει να το επαναλάβει στις πέντε γωνίες tilt.

### OAK-D

| Σενάριο | URDF visuals μέσα στο FOV |
| --- | --- |
| Σήμερα [0,690, ground 0,488], 15,6° | **κανένα**· το κάτω όριο του FOV φτάνει στο έδαφος 0,52 m μπροστά από την κάμερα |
| Θέση/κλίση CAD [0,3325, ground 0,305], 50°, **στο σημερινό URDF** | **45 visuals**: τροχοί και μοτέρ intake (az ±6…34°), 16 τμήματα cheeks, flywheels (dist 0,09 m, az ±8…30°), lower plate του flywheel cradle, bridge top, chassis cheeks, front housings, IR sensors, intake debug camera, beam line |
| Θέση CAD στο CAD | ελέγχεται από `ko_camera_fov` / `qa_cam_lidar`· ο F σχεδιάστηκε με «line of sight over the plate edge»· η απόφραξη από flywheel/cheeks στις −3…+20° **δεν** έχει ελεγχθεί σε sim |

Η κάμερα στη θέση του CAD προϋποθέτει την αφαίρεση του παλιού μηχανισμού (δ1)
και το μοντέλο της αψίδας.

## 3. Πρόταση σε μικρά βήματα με gate

**Αποφάσεις πριν από το (α):**

- **D1 — datum του `base_link`.** Πρόταση: μένει στο μέσο των αξόνων και στα
  45 mm πάνω από το έδαφος, με μία σταθερά `cad_to_base = [−0,1175, 0, −0,045]`
  που αντικαθιστά τη σημασιολογία του `functional_shift_x`. Εναλλακτικά: CAD
  origin, όπως στο bench, με κόστος σφάλματος odometry έως ≈0,235 m σε
  περιστροφή επί τόπου.
- **D2 — OAK pitch.** Επιλογές: 50° του CAD (κοντινή όραση, αλλάζει το scan/search
  του `collect_route`), μικρότερη κλίση (αλλαγή στο CAD mount) ή δεύτερο μοντέλο
  χρήσης.
- **D3 — μηχανισμός στο ενδιάμεσο.** Πρόταση: ως το (δ) μένει ο υπάρχων
  μηχανισμός ως γνωστό κενό, και μετά αφαιρείται εξ ολοκλήρου. Κανένα μεικτό
  μοντέλο.

Προτεινόμενη σειρά: **α → γ → δ1 → β**, γιατί το (β) δεν περνά gate πριν από το
(δ1) (§2).

| Βήμα | Περιεχόμενο | Αρχεία | Gate | Κίνδυνος για τα περασμένα τεστ |
| --- | --- | --- | --- | --- |
| **α** LiDAR + κέλυφος F | C1 [−0,316, +0,110] (παραμετρικό ως τις μετρήσεις του dummy), offset 0,0298, head box 55,6×55,6×41,3, `range_min` (0,15 ή μικρότερο με gate)· body F ως απλοποιημένα visuals ή mesh από το `body-panels-f.scad`· tower/pedestal visual· clip των `basket_rails` κάτω από 0,40 | `tennis_robot.urdf.xacro` (datum block), `components/lidar.urdf.xacro`, `components/body_panels.urdf.xacro`, `compact_mechanics.urdf.xacro` (μόνο rails), `tests/test_mechanical_variants.py` (0,5535 → 0,4198) | render μέσω docker + `check_urdf`· TF: scan στο ground 0,4198· offline occlusion = 0°· headless Gazebo: `/scan` χωρίς returns < 0,30 m και χωρίς τυφλό τομέα· `court_survey_v2` ως DONE με λογικό `court_boundary.json`· SLAM map χωρίς ghost· Nav2 costmap χωρίς ψευδο-εμπόδια γύρω από το robot | Οι παράμετροι survey/SLAM έχουν ρυθμιστεί στα 0,5535 (ανίχνευση δικτύου σε άλλο ύψος)· `collect_route` regression· ο πραγματικός C1 στο Pi παίρνει το ίδιο TF (θετικό) |
| **γ** frame/τροχοί/μάζα/CoG | Τροχοί ±0,1825/±0,297, ράγες v3 αντί για plate, Parkside×2 + UGREEN αντί για μπαταρία 6,26 kg, tower mass, footprint, `wheel_separation` ξανά από ≈0,594 με calibration, `collection_route` dims | `base.urdf.xacro`, `drivetrain.urdf.xacro`, `battery.urdf.xacro`, `electronics_packaging.urdf.xacro`, `generate_robot_urdf.py`, `controllers.yaml`, `nav2_params.yaml`, `collection_route.yaml` | Audit μάζας/CoG ±5 % έναντι του bench `full45.validation.json`· `diagnose_motion` (ευθεία/στροφή, odom έναντι ground truth)· SLAM χωρίς pinwheel· Nav2 goal· `collect_route` smoke (route completes) | **Υψηλός**: τα gates heading/curvature (#29/#34/#45) ρυθμίστηκαν στην παλιά δυναμική· planner tests με διαστάσεις robot· το bin v2 «βυθίζεται» μέσα από το plate opening |
| **δ1** μηχανισμός, κινηματικός | Αφαίρεση intake/bridge/rails/hood/bin lift· port από το v3 bench: `launcher_tilt_joint` (position, −3/0/+20), flywheels, καλάθι P2-71 σταθερό, ski, θύρα, αψίδα (visual/collision)· D200×60 ως frozen baseline, με σημείωση για τους D192×48 | `tennis_robot.urdf.xacro`, νέο `components/shared_flywheel.urdf.xacro`, `basket.urdf.xacro`, `ros2_control.urdf.xacro`, `controllers.yaml`, `controller_node.py` (collector commands), `gazebo_extras_node.py` (beams) | Render/TF στις 5 γωνίες· LiDAR occlusion 0° στις −10/−3/0/15/20· tilt joint ακολουθεί εντολή· `collect_route` smoke: navigation completes (confirmed = γνωστό κενό) | **Υψηλός**: τα `test_compact_mechanical_model`, `test_cad_transcription`, `test_intake_frame_alignment`, `test_throwing_mode`, `test_flywheel_exit_corridor` και `test_mechanical_variants` ελέγχουν την παλιά γεωμετρία → αποσύρονται ή αντικαθίστανται ρητά |
| **β** OAK-D | Θέση [0,3325, 0, 0,260] και pitch κατά D2· `PERCEPTION_CAMERA_X_M` από το TF ή ίδιο default· re-aim της intake debug camera | `tennis_robot.urdf.xacro`, `oak_d.urdf.xacro`, `controller_node.py`, `court_survey_v2_node.py` | FOV occlusion screen στις 5 γωνίες· perception: πίνακας ανίχνευσης έναντι απόστασης για γνωστές μπάλες· coverage του 360° scan· `collect_route` scan → plan | Με 50°: μηδενικό ή μικρό plan στο `collect_route` → σπάει το περασμένο σενάριο 10/10· perception covariance calibration |
| **δ2** ball physics | Contact flywheel–μπάλα, ski, feed | — | — | **Γνωστό κενό**: χωρίς μετρημένους D192, TPU μ ή contact model, η συλλογή μένει plan-level |

## 4. STOP

Δεν άλλαξε κανένα xacro, launch ή param. Το επόμενο βήμα θέλει απαντήσεις στα
D1, D2 και D3 και έγκριση για το (α), με render και headless Gazebo μέσα από
`docker/jazzy/run.sh`.
