# Ενεργό σύνολο εκτύπωσης — P2-93e / P2-95 gate

## P2-93e — ενεργή candidate αψίδα, όχι RELEASED

**Το παλιό σετ `arch-tilt-p293/stl/` της αψίδας: ΜΗΝ ΤΥΠΩΣΕΙΣ.**
Οι παλιές οδηγίες «μπορούν να τυπωθούν» είναι ιστορικές και ακυρώνονται.
Το tilt δεν αλλάζει και παραμένει χωριστό επόμενο βήμα.
Τα τρία retained upper interfaces της P2-93d assembly (κεντρική τραβέρσα,
δύο τύποι sleeves) δεν αποτελούν άδεια εκτύπωσης του παλιού σετ.

Όλα τα παρακάτω είναι στον `arch-tilt-p293/stl-candidate-p293d/`:

| STL | Κατάσταση | Ποσότητα / οδηγία |
| --- | --- | --- |
| `p293d_foot.stl` | **FIT-CHECK** | 2, καθρέφτισμα Y για R· φέρον πέλμα και ενσωματωμένο arch clevis **συμπαγή** |
| `p293d_knee.stl` | **FIT-CHECK** | 2, **συμπαγή**, XZ load plane στο bed |
| `p293d_crown.stl` | **FIT-CHECK** | 2, φέρουσες παρειές/clevis **συμπαγή** |
| `p293d_crossbar_end.stl` | **FIT-CHECK** | 2, πρώτα joint coupon |
| `p293d_mast.stl` | **FIT-CHECK** | 2, πρώτα shoe coupon |
| `p293d_lidar_half.stl` | **FIT-CHECK** | 2, πρώτα joint coupon |
| `p293d_mast_cap.stl` | **FIT-CHECK** | 2, πρώτα joint coupon |
| `p293d_oak_mount.stl` | **FIT-CHECK** | 1, πρώτα sensor-interface coupon |
| `p293e_foot_template.stl` | **FIT-CHECK** | 1 L (+Y), πλήρες επίπεδο gauge 2 mm, χωρίς supports |
| `p293e_foot_template_R.stl` | **FIT-CHECK** | 1 R (−Y), ήδη καθρεφτισμένο, όχι δεύτερο mirror |

Τα templates είναι προσωρινά εργαλεία μέτρησης, όχι installed robot parts:
δεν προστίθενται στο `current_robot()` ούτε μπορούν να πάρουν RELEASED ως
φέροντα εξαρτήματα. Χαλαρές M4 και στις τρεις οπές ταυτόχρονα, χωρίς ζόρι.
Το gauge ακουμπά στη ράγα Z52 και γεφυρώνει το χαμηλότερο X240/Z42· **δεν
σφίγγεται** προς τη χαμηλή τραβέρσα. Η εγκοπή έχει όριο X271 και βάθος
0,8 mm: μέτρησε από την άθικτη παρειά δίπλα της, όχι από τον πυθμένα της.

Για κάθε φέρον print >4 h προηγείται Cut coupon στο Bambu Studio. Η οδηγία
draft 2 walls/10% αφορά μόνο μη φέροντα δοκιμαστικά μέρη, **όχι** foot,
knee ή clevis. FIT-CHECK δεν είναι έγκριση πλήρους φέροντος print.

### P2-93g — υποχρεωτικό physical proof-load foot+knee πριν το RELEASED

**Tilt: FIT-CHECK — ΑΠΑΓΟΡΕΥΣΗ ΛΕΙΤΟΥΡΓΙΑΣ μέχρι να εγκατασταθούν και
να επαληθευτούν τα νέα stops/strikers του P2-93h.** Τα παλιά upper stops
αποσύρθηκαν από το `current_robot` με ρητή έγκριση. Τα παλιά AS5600
pod/cover/magnet cap παραμένουν γνωστά ανοικτά θέματα του P2-93h.
Νέα candidate solids: knee installed |Y|221,6, όριο |Y|≤240 στη ζώνη
X220–420. [P2-93g debug / hardware / assembly](arch-tilt-p293/P2-93g-debug.md),
[LC1/LC2, πραγματικές μάζες/τομές και κενά](arch-tilt-p293/P2-93g-solid-tables.md).
Δεν υπάρχει φυσικό proof-load PASS και κανένα νέο STL δεν είναι RELEASED.

| STL στο arch-tilt-p293/stl-candidate-p293g/ | Ποσότητα | Κατάσταση |
|---|---:|---|
| p293g_foot.stl | 2 mirrored L/R | **FIT-CHECK — συμπαγή**, steel sleeves/recessed pin heads |
| p293g_crown.stl | 2 mirrored L/R | **FIT-CHECK — συμπαγή**, class10.9 button-head pins /inboard12×26mm lug /physical Tr8 relief |
| p293g_crossbar_end.stl | 2 mirrored L/R | **FIT-CHECK**, νέο end length/bolt |
| p293g_console_inboard.stl | 1 | **FIT-CHECK — συμπαγές** |
| p293g_console_outboard.stl | 1 | **FIT-CHECK — συμπαγές** |
| p293g_cradle.stl | 1 | **FIT-CHECK — συμπαγές** |
| p293g_float_cap.stl | 1 | **FIT-CHECK** |
| p293g_nut_trunnion.stl | 1 | **FIT-CHECK — συμπαγές** |
| p293g_nut_arm.stl | 1 | **FIT-CHECK — συμπαγές** |

Στις exported print coordinates: foot/end-beam **MirrorY** για το άλλο
χέρι· crown **MirrorZ και Drop-to-bed**, επειδή έχει εξαχθεί rotatedXZ-on-bed.

**Παλιά p293d_foot / p293d_crown / p293d_crossbar_end: ΜΗΝ ΤΥΠΩΣΕΙΣ**
για τη νέα διάταξη. Το knee STL παραμένει ίδιο και **FIT-CHECK, συμπαγές**,
μόνο installed pose±221,6. Τα δύο p293e templates μένουν αμετάβλητα
**FIT-CHECK**, ελέγχουν ίδιες base holes και X271 datum, όχι όλες τις νέες
τοπικές επιφάνειες του foot. Το ιστορικό inventory παρακάτω δεν αναιρεί
αυτές τις νεότερες αντικαταστάσεις.

Το analytical screening χρησιμοποιεί χωριστές LC1 actuator / LC2 hard-stop,
με όρια10/5 MPa και περιθώριο≥1,0· **δεν** δίνει μόνο του RELEASED.
Η παλιά άθροιση8 N·m + equivalent Tr8 και ο στόχος1,5 έχουν αποσυρθεί.
Αποτελέσματα: [διορθωμένο preflight](arch-tilt-p293/P2-93g-corrected-load-cases.md).

Foot και knee τυπώνονται **συμπαγή** και βιδώνονται στο πραγματικό σασί,
με το τελικό hardware/ροδέλες/ροπές σύσφιγξης καταγεγραμμένα. Για το
proof fixture επιτρέπεται ελεγχόμενη πλήρης φέρουσα εκτύπωση αφού κλείσουν
fit/coupon και screening, παραμένει όμως **FIT-CHECK**, όχι άδεια χρήσης
στο ρομπότ. Μόνο ο υπολογισμός δεν παρακάμπτει κανένα άλλο gate.

- Εφαρμογή **150 N στο B=[260,−205,220] για24 ώρες**, μέσω της τελικής
  κονσόλας ή άκαμπτου ισοδύναμου fixture. Κατεύθυνση παράλληλη στον Tr8
  στη δυσμενέστερη γωνία/φορά του διορθωμένου LC1· καταγραφή της γωνίας
  και της φοράς, όχι αυθαίρετη κατακόρυφη ανάρτηση.
- Αρχική μέτρηση χωρίς φορτίο, **βέλος στην αρχή υπό150 N**, βέλος στο
  τέλος των24 h υπό το ίδιο150 N, και residual μετά την εκφόρτιση.
  Σταθερό datum πάνω στο σασί· καταγράφονται δX/δY/δZ ή ρητά η μετρούμενη
  συνιστώσα, ακρίβεια οργάνου, θερμοκρασία, timestamps και φωτογραφίες.
- Ασφαλισμένο σασί/fixture, μηχανική συγκράτηση του φορτίου σε αστοχία,
  ζώνη αποκλεισμού, μοτέρ εκτός τροφοδοσίας. Δεν χρησιμοποιείται κινούμενο
  ρομπότ ως δοκιμαστήριο. Σταμάτημα σε ρωγμή, ολίσθηση ή προοδευτική αστάθεια.
- Καταγραφή **Δβέλους=|δ24h|−|δαρχής|**, στην ίδια loaded συνιστώσα:
  αποδεκτή αύξηση στις24 h
  **≤20% του αρχικού βέλους υπό φορτίο**. Residual μετά την εκφόρτιση
  **≤0,5 mm**. Καμία ρωγμή, λεύκανση ή ολίσθηση, και καμία
  washer indentation **>0,2 mm**. Καταγράφεται επίσης χαλάρωση βιδών.

**RELEASED μόνο μετά καταγεγραμμένο PASS της φυσικής δοκιμής**, μαζί με
coupon/installed-pose/raw scene gates που εξακολουθούν να ισχύουν.
Η πίσω διαγώνιος προς X65 **απορρίφθηκε** και δεν αποτελεί ενεργό
στήριγμα ή μέρος του proof fixture. Η ανάλυσή της διατηρείται ως ιστορικό
αρνητικό αποτέλεσμα. Το PASS άλλης διάταξης δεν μεταφέρεται στη σημερινή.
Το150 N στο B ελέγχει foot+knee με τη διαδρομή του actuator· δεν
πιστοποιεί από μόνο του τις τοπικές παρειές των hard stops/crown ή όλο
το KFL08/Tr8 bearing stack. Καμία δοκιμή24 h δεν έχει εκτελεστεί ακόμη.

Έλεγχος P2-93e: 86 global collisions +3 contained, μόνο έξι F-concept
τομές στην αψίδα, 9,00 mm drive gap σε −10/−3/0/15/20°. Καμία εξαίρεση
όγκου. Hardware, όρια και coupon: [P2-93e log](arch-tilt-p293/P2-93e-debug.md).

Η παρούσα λίστα είναι το μοναδικό ευρετήριο για κομμάτια που είναι ενεργά
στη σημερινή φυσική κατασκευή. Αρχεία κάτω από `archive/` ή με σήμανση
«concept» **δεν τυπώνονται**. Η κατάσταση αποτυπώνει όσα είναι γνωστά στις
2026-11-05 (P2-95)· όταν ένα φυσικό part αλλάξει, ενημερώνεται πρώτα αυτό το
αρχείο.

## Υποχρεωτικό gate πριν από κάθε εκτύπωση

Οι μόνες επιτρεπτές καταστάσεις STL είναι:

- **CONCEPT** — δεν επιτρέπεται πλήρης εκτύπωση. Η γεωμετρία, η installed
  θέση ή/και το collision set δεν έχουν κλείσει.
- **FIT-CHECK** — επιτρέπεται μόνο coupon/δοκιμαστικό τμήμα της διεπαφής.
  Δεν είναι άδεια για πλήρη εκτύπωση του part.
- **RELEASED** — το STL βρίσκεται στο `current_robot()` στην
  πραγματική θέση συναρμολόγησης, το εφαρμοστέο raw exact-solid gate δίνει
  μηδέν συγκρούσεις και υπάρχει καταγεγραμμένο PASS από τυπωμένο coupon ή
  από την ήδη συναρμολογημένη ίδια διεπαφή.

Για κάθε νέο ή αλλαγμένο robot-installed STL, πριν από slicing:

1. προστίθεται ή ενημερώνεται η αντίστοιχη installed-pose κλήση στο
   `body-panels/current-design-p293.scad` (`current_robot()`),
2. εκτελείται το `independent-check/scene_check.py` στις −10/−3/0/15/20°
   μέσω `bash cad/check-current-scene.sh`, χωρίς αφαίρεση
   mating/fastener/interface όγκων,
3. κάθε θετικός όγκος είναι FAIL· η σήμανση «σκόπιμη» τεκμηριώνει την
   πρόθεση αλλά **δεν** αποτελεί εξαίρεση και δεν επιτρέπει RELEASED,
4. τυπώνεται και περνά coupon της διεπαφής,
5. μόνο τότε η κατάσταση αλλάζει σε RELEASED.

Το P2-95 gate χρησιμοποιεί το default `--min-vol 0.0001` του
`scene_check.py`. Το `collide.py`/`parts.py` παραμένει ιστορικό διαγνωστικό,
όχι release gate. Η τρέχουσα αναφορά βρίσκεται στο
`independent-check/P2-95-report.md`. Οι προηγούμενες 22 ενδείξεις RELEASED
έγιναν FIT-CHECK: το ιστορικό φυσικής συναρμολόγησης διατηρείται παρακάτω,
αλλά δεν αποτελεί P2-95 PASS. Καμία νέα πλήρης εκτύπωση δεν εγκρίνεται όσο
το full-scene gate αποτυγχάνει.

Για εκτύπωση με εκτιμώμενο χρόνο πάνω από 4 ώρες προηγείται interface
coupon με **Cut στο Bambu Studio**. Όπου η αντοχή δεν είναι αντικείμενο της
δοκιμής, η πρώτη εκτύπωση γίνεται draft με **2 τοιχώματα και 10% infill**.
Coupon και fixtures δεν έχουν robot-installed pose· παραμένουν FIT-CHECK και
ελέγχονται στο δικό τους fixture context, όχι ως ψεύτικα σώματα στο robot.

## Κανονικός πίνακας STL → κατάσταση

Ο παρακάτω πίνακας υπερισχύει από παλαιότερες φράσεις όπως
«release-ready», «προς εκτύπωση» ή «τυπώνεται τώρα». Οι διαδρομές είναι
σχετικές με `cad/`.

| STL | Κατάσταση | Τεκμήριο / blocker |
|---|---|---|
| `modular-test-chassis/stl-v3/sleeve_fit_coupon.stl` | FIT-CHECK | Coupon, όχι installed part |
| `modular-test-chassis/stl-v3/clamp_sleeve.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/corner_clamp_left.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/corner_clamp_right.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/rear_motor_left.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/rear_motor_right.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/rail_extension_left.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/rail_extension_right.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/front_terminal_left.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/front_terminal_right.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/rear_crossbar_half.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x240_crossbar_half.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x240_clamp_sleeve.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x240_terminal_sleeve.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x300_crossbar_half.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x300_clamp_sleeve.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/x300_terminal_sleeve.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-v3/torsion_side_brace.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `modular-test-chassis/stl-p277/mpu6050_rail_side_mount.stl` | FIT-CHECK | Δεν έχει περάσει τυπωμένο interface coupon |
| `modular-test-chassis/stl-ski-mount/r75-75-ski.stl` | FIT-CHECK | Τυπωμένο, αλλά το πλήρες νέο bracket/fork interface δεν έχει καταγεγραμμένο coupon PASS |
| `modular-test-chassis/stl-ski-mount/x300-ski-bracket-left.stl` | FIT-CHECK | Πρώτα Cut coupon του X300/fork interface |
| `modular-test-chassis/stl-ski-mount/x300-ski-bracket-right.stl` | FIT-CHECK | Πρώτα Cut coupon του X300/fork interface |
| `modular-test-chassis/stl-ski-mount/p285-ski-wall-fork-left.stl` | FIT-CHECK | Πρώτα Cut coupon του ski-wall interface |
| `modular-test-chassis/stl-ski-mount/p285-ski-wall-fork-right.stl` | FIT-CHECK | Πρώτα Cut coupon του ski-wall interface |
| `arch-tilt-p293/stl/arch_foot_x240.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/arch_foot_x300.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — καταργήθηκε |
| `arch-tilt-p293/stl/arch_leg_lower.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/arch_leg_upper.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/arch_brace_x240.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/arch_brace_x300.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — καταργήθηκε |
| `arch-tilt-p293/stl/arch_joint_sleeve.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** παλιό σετ· retained interface δεν αποτελεί release |
| `arch-tilt-p293/stl/arch_crossbar_center.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** παλιό σετ· retained interface δεν αποτελεί release |
| `arch-tilt-p293/stl/arch_crossbar_end.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/lidar_mast.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/lidar_crossbar_half.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/sensor_joint_sleeve.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** παλιό σετ· retained interface δεν αποτελεί release |
| `arch-tilt-p293/stl/oak_universal_plate.stl` | CONCEPT | **ΜΗΝ ΤΥΠΩΣΕΙΣ** — παλιό σετ |
| `arch-tilt-p293/stl/tilt_stop.stl` | FIT-CHECK | Raw P2-93b PASS, αλλά dry/coupon fit δεν έχει καταγραφεί |
| `arch-tilt-p293/stl/tilt_fit0186_carrier.stl` | FIT-CHECK | Raw P2-93b PASS· εκκρεμούν πραγματικές διαστάσεις και coupon |
| `arch-tilt-p293/stl/tilt_kfl08_base.stl` | FIT-CHECK | Raw P2-93b PASS· εκκρεμούν πραγματικές διαστάσεις και coupon |
| `arch-tilt-p293/stl/tilt_nut_carriage.stl` | FIT-CHECK | Raw P2-93b PASS· εκκρεμούν πραγματικές διαστάσεις και coupon |
| `arch-tilt-p293/stl/tilt_as5600_pod.stl` | FIT-CHECK | Εκκρεμεί AS5600 coupon |
| `arch-tilt-p293/stl/tilt_as5600_cover.stl` | FIT-CHECK | Εκκρεμεί AS5600 coupon |
| `arch-tilt-p293/stl/tilt_magnet_cap.stl` | CONCEPT | Συγκρούεται με το launcher surrogate και δεν έχει κλειστή installed geometry |
| `rear-pod/stl-v2/base_left.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `rear-pod/stl-v2/base_right.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `rear-pod/stl-v2/corner_tie.stl` | FIT-CHECK | Εκκρεμεί τυπωμένο tie/base coupon |
| `rear-pod/stl-v2/pi_shelf.stl` | FIT-CHECK | Raw CAD χωρίς hit, αλλά δεν υπάρχει τυπωμένο coupon PASS |
| `rear-pod/stl-v2/vertical_electronics_plate.stl` | FIT-CHECK | Raw CAD χωρίς hit, αλλά δεν υπάρχει τυπωμένο coupon PASS |
| `rear-pod/stl-v2/side_frame.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `rear-pod/stl-v2/side_frame_base_drill_jig.stl` | FIT-CHECK | Fixture, όχι installed robot part |
| `rear-pod/stl-v2/rear_foot_wedge_washer.stl` | CONCEPT | 12,5 mm³/πλευρά raw overlap με base· blocking |
| `rear-pod/stl-v2/parkside_dock_left.stl` | CONCEPT | 3.282,9 mm³ overlap με drawer· provisional hardware |
| `rear-pod/stl-v2/parkside_dock_right.stl` | CONCEPT | 3.282,9 mm³ overlap με drawer· provisional hardware |
| `rear-pod/stl-v2/parkside_drawer_left.stl` | CONCEPT | 3.282,9 mm³ overlap με dock· provisional hardware |
| `rear-pod/stl-v2/parkside_drawer_right.stl` | CONCEPT | 3.282,9 mm³ overlap με dock· provisional hardware |
| `rear-pod/stl-v2/ugreen_post_holder.stl` | CONCEPT | 172,6 mm³ overlap με rear retainer· blocking |
| `rear-pod/stl-v2/ugreen_clamp_caps.stl` | FIT-CHECK | Εκκρεμεί post/clamp coupon |
| `rear-pod/stl-v2/ugreen_rear_retainer.stl` | CONCEPT | 172,6 mm³ overlap με holder· blocking |
| `rear-pod/stl-v2/panel_posts.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `rear-pod/stl-v2/control_panel.stl` | FIT-CHECK | Ιστορικά τυπωμένο/συναρμολογημένο· P2-95 full-scene FAIL, δεν υπάρχει νέο release |
| `rear-pod/stl-v2/component_pins.stl` | FIT-CHECK | Μη φέρον fit part· πρώτα draft/coupon |
| `rear-pod/stl-v2/component_clips.stl` | FIT-CHECK | Μη φέρον fit part· πρώτα draft/coupon |
| `rear-pod/stl-v2/bts_spacers.stl` | FIT-CHECK | Μη φέρον fit part· πρώτα draft/coupon |
| `rear-pod/stl-v2/cable_bulkhead.stl` | CONCEPT | Παραμένει ενεργό v1 part στον v2 exporter/scene· raw συγκρούσεις με frames/bases, απαγόρευση εκτύπωσης |
| `rear-pod/stl-v2/bulkhead_foot.stl` | CONCEPT | Ενεργό support του blocking cable bulkhead· δεν έχει P2-95 release |
| `rear-pod/stl-v2/upright_mount_fit_coupon.stl` | FIT-CHECK | Coupon PASS· το ίδιο δεν είναι installed release part |
| `body-panels/stl-f-rear/rear_lo_left.stl` | CONCEPT | Raw collisions με stay/base/door/splice |
| `body-panels/stl-f-rear/rear_lo_right.stl` | CONCEPT | Raw collisions με stay/base/door/splice |
| `body-panels/stl-f-rear/rear_hi_left.stl` | CONCEPT | Raw collisions με deck/control/splice |
| `body-panels/stl-f-rear/rear_hi_right.stl` | CONCEPT | Raw collisions με decks/control/splice |
| `body-panels/stl-f-rear/deck_rear_left.stl` | CONCEPT | Raw collisions με rear_hi/control |
| `body-panels/stl-f-rear/deck_rear_right.stl` | CONCEPT | Raw collisions με rear_hi/control |
| `body-panels/stl-f-rear/parkside_door_left.stl` | CONCEPT | Raw 503,4 mm³ hinge/skin collision |
| `body-panels/stl-f-rear/parkside_door_right.stl` | CONCEPT | Raw 503,4 mm³ hinge/skin collision |
| `body-panels/stl-f-rear/corner_stay_left.stl` | CONCEPT | Raw 12.247,7 mm³ mating overlap |
| `body-panels/stl-f-rear/corner_stay_right.stl` | CONCEPT | Raw 12.247,7 mm³ mating overlap |
| `body-panels/stl-f-rear/center_splice_lo.stl` | CONCEPT | Raw 0,00021…0,00030 mm³ lap overlap |
| `body-panels/stl-f-rear/center_splice_hi.stl` | CONCEPT | Raw 0,00020…0,00030 mm³ lap overlap |
| `body-panels/stl-f-rear/door_turn_button.stl` | CONCEPT | Το F subset παραμένει blocking και χωρίς coupon |
| `test-fixtures/tpu-friction/plate.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/tpu-friction/sled.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/tpu-friction/pulley.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/tpu-friction/pulley_bracket.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/ski-jump-hand-test/ski_42deg_R75.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/ski-jump-hand-test/ski_60deg_R75.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/ski-jump-hand-test/ski_75deg_R75.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `test-fixtures/ski-jump-hand-test/ski_75deg_R110.stl` | FIT-CHECK | Fixture· πρώτη εκτύπωση draft |
| `drive-motor-mount/37mm-mount-hole-gauge.stl` | FIT-CHECK | Gauge· fit checked μόνο σε 1/4 mounts |
| `drive-motor-mount/nut-keeper-stl/sleeve.stl` | FIT-CHECK | Retention test εκκρεμεί |

## Σασί v3 — P2-78 sockets και πλευρικές διαγώνιες

Κύρια πηγή: `modular-test-chassis/modular-chassis-v3.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-v3/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Fit coupon μανικιού | `stl-v3/sleeve_fit_coupon.stl` ×1 | pre-P2-78 fit gate | M4 δοκιμής | Τυπωμένο· μόνο δοκιμή |
| Κύρια μανίκια 50×30 | `stl-v3/clamp_sleeve.stl` ×5 | v3/P2-78 retained | M4×40 top, M4×60 side, ροδέλες/παξιμάδια | Τυπωμένα |
| Πίσω γωνίες | `stl-v3/corner_clamp_left.stl` ×1, `corner_clamp_right.stl` ×1 | P2-78 retained/fixed | M4×40 top, M4×60 side | Τυπωμένα |
| Πίσω δοκοί κινητήρων | `stl-v3/rear_motor_left.stl` ×1, `rear_motor_right.stl` ×1 | v3/P2-78 retained | Hardware FIT0403 + joint M4 | Τυπωμένα |
| Μεσαίες πλευρικές δοκοί | `stl-v3/rail_extension_left.stl` ×1, `rail_extension_right.stl` ×1 | v3/P2-78 retained | Joint M4×40/60 | Τυπωμένα |
| Εμπρός terminal δοκοί με sockets | `stl-v3/front_terminal_left.stl` ×1, `front_terminal_right.stl` ×1 | P2-78 | Joint M4 + socket locks | Τυπωμένα |
| Πίσω τραβέρσα | `stl-v3/rear_crossbar_half.stl` ×2 | v3 corner fix/P2-78 retained | M4×40/60 | Τυπωμένη διορθωμένη έκδοση |
| Τραβέρσα X240 | `stl-v3/x240_crossbar_half.stl` ×2 | P2-78 | 4×M4×30 top + 4×M4×60 transverse συνολικά | Τυπωμένη |
| Κεντρικό μανίκι X240 | `stl-v3/x240_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30/60 | Τυπωμένο |
| Terminal μανίκια X240 | `stl-v3/x240_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y75/100/125/150 | Τυπωμένα |
| Τραβέρσα X300 | `stl-v3/x300_crossbar_half.stl` ×2 | P2-78 | 2×M4×30 top + 4×M4×30 transverse συνολικά | Τυπωμένη |
| Κεντρικό μανίκι X300 | `stl-v3/x300_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30 | Τυπωμένο |
| Terminal μανίκια X300 | `stl-v3/x300_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y100/160 | Τυπωμένα |
| Πλευρικές torsion διαγώνιες | `stl-v3/torsion_side_brace.stl` ×2 | P2-78 | ανά brace M4×65 στο X240/Y150 + M4×35 στο X300/Y125, μεγάλες ροδέλες/παξιμάδια | Τυπωμένες |

Το `stl-v3/ballast_tab.stl` δεν είναι ενεργό: ανήκε στην καταργημένη
plywood/metal ballast plate. Τα P2-77 `brace_drill_jig.stl` και
`torsion_centre_node.stl` έχουν αντικατασταθεί πλήρως από τα δύο P2-78
`torsion_side_brace.stl` και αρχειοθετούνται.

### Ενεργό MPU

| Φυσικό κομμάτι | Ενεργό STL | Πηγή | P2 | Βίδες | Κατάσταση |
|---|---|---|---:|---|---|
| Πλευρική βάση MPU6050 στη +Y ράγα | `modular-test-chassis/stl-p277/mpu6050_rail_side_mount.stl` | `modular-test-chassis/p277-torsion-node.scad` | P2-77 | 2×M4×55/60 στις X100/X125, ροδέλες/nyloc· MPU με μη φέρον hardware της πλακέτας | **FIT-CHECK** — πρώτα coupon |

Αυτό είναι το μόνο ενεργό MPU mount. Το `rear-pod/stl-v2/mpu6050_mount.stl`
δεν χρησιμοποιείται.

## Βατήρας R75/75° — P2-85

Πηγή: `modular-test-chassis/shared-flywheel-p273-bolted-ski.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-ski-mount/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Βατήρας R75/75° | `stl-ski-mount/r75-75-ski.stl` ×1 | P2-73, P2-85 χωρίς bosses | 2×M4×30 χαμηλής κεφαλής στο X315 (κεφαλή μέσα) | Ήδη τυπωμένος — ο ενεργός STL είναι πλέον γεωμετρικά ο ίδιος· δεν ξανατυπώνεται |
| Αριστερό/δεξί bracket X300 | `stl-ski-mount/x300-ski-bracket-left.stl` ×1, `x300-ski-bracket-right.stl` ×1 | P2-85 | 4×M4×35 στο X300 Y±75/±100, ροδέλες/nyloc | **FIT-CHECK** — πρώτα Cut coupon· το P2-73 bracket καταργείται (έσπασε) |
| Διχάλα τοιχώματος βατήρα | `stl-ski-mount/p285-ski-wall-fork-left.stl` ×1, `p285-ski-wall-fork-right.stl` ×1 | P2-85 | Hub στη M4×30 του X315· κλείδωμα 2×M4×30 DIN 933 (κεφαλή στο hex pocket) + ροδέλα Ø12/nyloc | **FIT-CHECK** — πρώτα Cut coupon· αντικαθιστούν τους `ski-boss-spacer` |

Το `ski-boss-spacer.stl` (P2-73 fix 2026-10-19) είναι superseded από το hub
της διχάλας P2-85 και αφαιρέθηκε από το ενεργό release. Το full ski dry-run
αναμένει πλέον 5 STL αντί για 4, όλα ακριβώς ένα σώμα.

## Ιστορικό αψίδας/κλίσης — P2-93b (δεν εγκρίνει εκτύπωση)

Πηγή: `arch-tilt-p293/p293-arch-tilt-release.scad`. Ενεργός κατάλογος:
`arch-tilt-p293/stl/`. Αναλυτική λίστα ποσοτήτων, μάζας, χρόνου, hardware
και σειράς συναρμολόγησης: `arch-tilt-p293/README.md`.

| Υποσύνολο | Ενεργά STL | Κατάσταση |
|---|---|---|
| Πόδια/σκέλη/διαγώνιες | `arch_foot_x240/x300`, `arch_leg_lower/upper`, `arch_brace_x240/x300`, `arch_joint_sleeve` | **FIT-CHECK** — πρώτα Cut coupon. Μόνο υπάρχουσες P2-78 οπές, through-M4 |
| Άνω αψίδα/sensors | `arch_crossbar_center/end`, `lidar_mast`, `lidar_crossbar_half`, `sensor_joint_sleeve`, `oak_universal_plate` | **FIT-CHECK** — πρώτα interface coupon. Το OAK part είναι universal interface, όχι cradle |
| Φέροντα tilt stops | `tilt_stop` | **FIT-CHECK** — dry assembly και coupon εκκρεμούν· συμμετρική διαδρομή φορτίου προς αψίδα |
| FIT/KFL/Tr8 interfaces | `tilt_fit0186_carrier`, `tilt_kfl08_base`, `tilt_nut_carriage` | **Fit-check μόνο** μέχρι `[ΜΕΤΡΗΣΕΙΣ ΧΡΗΣΤΗ: ...]`· μην ξεκινήσεις τελικό print |
| Encoder | `tilt_as5600_pod/cover`, `tilt_magnet_cap` | **Fit-check μόνο** μέχρι μετρήσεις AS5600· AZ1 παραγγέλθηκε 2026-10-25, 3,3 V board με αφαίρεση R1 σε ένα sample για 5 V |

P2-93b exact-solid status: αφαιρέθηκαν τα QA masks και διορθώθηκαν μόνο οι
installed poses carrier/KFL08/stops. **Κανένα printable STL δεν άλλαξε.** Και
τα 13 STL της αψίδας είναι byte-identical με το P2-93 release. Αυτό δεν
παρακάμπτει το P2-94 coupon gate: μη ολοκληρωμένες εκτυπώσεις παγώνουν σε
FIT-CHECK μέχρι physical PASS. Τα tilt interfaces παραμένουν fit-check μέχρι
τις φυσικές μετρήσεις hardware.

Τα endpoint `tilt_switch_lower/upper` αφαιρέθηκαν από το P2-93. Η cassette
ανήκει στο release της κινούμενης μονάδας και παραμένουν ανοικτά gates:

- ενεργοποίηση roller lever ~1° πριν τα hard stops −3°/+20°,
- υπερδιαδρομή ώστε reaction/impact να περνά αποκλειστικά stop→αψίδα και
  ποτέ από τον microswitch,
- καλώδιο μέσα στο D5065 service loop με strain relief και στα δύο άκρα,
- δύο switches ανά άκρο: Mega sense + NC hardware cutoff R_EN/L_EN.

Basket floor και rear/side grids, καθώς και saddles/guides/locks, **δεν
ανήκουν στο πρώτο κύμα**: περιμένουν τα Δ1–Δ5 του mockup. Η προσωρινή
drive-test console συγκρούεται με το F και αφαιρείται πριν το F.

Το release έχει **20 one-body STL / 33 prints**, planning upper-bound
1.609,7 g PETG και 100,8 h με 0,4/0,28 ή 75,5 h με 0,6/0,30 στον P2S.
Export, P2-90e roofs και exact-solid regression:

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/arch-tilt-p293/export-p293-stls.sh
```

## Rear pod v2 — κατάσταση P2-90d

> **ΠΡΟΣΟΧΗ — ΜΗΝ ΤΥΠΩΣΕΙΣ ΠΑΛΙΑ EXPORT:** στο P2-86b άλλαξαν ξανά τα
> `base_left.stl`, `base_right.stl` και `corner_tie.stl`. Τα P2-86 base STL
> είναι superseded. Χρησιμοποίησε μόνο τα τρέχοντα αρχεία του `stl-v2/`.

Κύριες πηγές: `rear-pod/rear-pod-v2.scad`, `rear-pod/params-v2.scad`.
Ενεργός κατάλογος: `rear-pod/stl-v2/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Ενισχυμένες βάσεις | `stl-v2/base_left.stl` ×1, `base_right.stl` ×1 | P2-79/P2-83/P2-86b | 4×M4×45 προς rear crossbar· 2×M4×20 countersunk προς ties· 4×M3 rear lands· τοπική flush-stay εγκοπή Y±165 | **ΤΥΠΩΜΕΝΕΣ P2-86b — ΚΛΕΙΔΩΜΕΝΕΣ, δεν επανεξήχθησαν στο P2-88** |
| Corner ties | `stl-v2/corner_tie.stl` ×2 | P2-83/P2-86b | 2×M4×20 countersunk προς βάση· κοινή μακρύτερη through-M4 tie→rear crossbar→stay στο `Y±165/Z37` | **ΝΕΟ P2-86b export — προς εκτύπωση** |
| Pi shelf | `stl-v2/pi_shelf.stl` ×1 | P2-76 | 4×M3, αφρός + 2 straps | **FIT-CHECK** — πρώτα coupon |
| Όρθια electronics plate | `stl-v2/vertical_electronics_plate.stl` ×1 | P2-76 | 2×M4×25 στις σιαγόνες, ροδέλες/nyloc | **FIT-CHECK** — πρώτα coupon |
| Side frames | `stl-v2/side_frame.stl` ×2 | P2-88/P2-90d | εμπρός 2×M4×25 στο X−132, πίσω 2×M4×35 με wedge, ροδέλες OD14· ανοικτά post cups 4×M4×60 στον υπάρχοντα άξονα X, ροδέλες/nyloc | **ΤΥΠΩΜΕΝΑ P2-90d (επιβεβαίωση χρήστη 2026-10-26) — συναρμολογημένα με panel posts και control panel**· όρθιο export, 36×24 load spines, ίδιο χαμηλό hardware pocket στα δύο base feet, support-free οροφές 45°· όλα τα προηγούμενα side-frame STL superseded |
| Jig εμπρός οπής βάσης | `stl-v2/side_frame_base_drill_jig.stl` ×1 | P2-88 | εντοπισμός στις δύο παλιές Ø4,5· τρυπάνι Ø4,5 | **FIT-CHECK** fixture |
| Σφηνοειδείς πίσω ροδέλες | `stl-v2/rear_foot_wedge_washer.stl` ×2 | P2-88 | κάτω από την πίσω M4 | **CONCEPT** — raw overlap 12,5 mm³/πλευρά |
| Parkside docks/drawers | `stl-v2/parkside_dock_left/right.stl`, `parkside_drawer_left/right.stl` ×1 | P2-88 provisional | 4×M3×12 προς υπάρχουσες slots + 2×M3 rear locks, Ruthex RX-M3x5.7 | **Σχεδιασμένο· αναμονή πραγματικής μπαταρίας/dock — ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ** |
| Θήκη UGREEN + split caps + πίσω retainer | `stl-v2/ugreen_post_holder.stl`, `ugreen_clamp_caps.stl` ×1 print plate / 2 caps, `ugreen_rear_retainer.stl` ×1 | P2-90/P2-90b, πραγματικό 115×45×45 | 4×M3×30 caps + 2×M3×12 retainer, 6× Ruthex RX-M3x5.7, προαιρετικό λουράκι | **ΝΕΟ P2-90b export holder+caps — πλάγια τοποθέτηση caps, κεντρική εισαγωγή συσκευής από −X· κλειστά P2-90 collars superseded** |
| Panel posts | `stl-v2/panel_posts.stl` ×1 print plate / 4 posts | P2-81/P2-90d interface | κάτω/cup 4×M4×60, επάνω 4×M4×20, ροδέλες/nyloc | **ΤΥΠΩΜΕΝΑ — κλειδωμένα, καμία μεταβολή STL στο P2-90d** |
| Νευρωμένο control panel | `stl-v2/control_panel.stl` ×1 | P2-81/P2-84/P2-86 | 4×M4×20, ροδέλες/nyloc· νέα συνεχής rear beam: 4×M3 rear_hi + 4×M3 deck· οπή LED Ø8 | **ΤΥΠΩΜΕΝΟ — κλειδωμένο στο P2-90· συναρμολογημένο στα posts με E-stop, toggle και START/ARM (2026-10-26)**· physical proof-load 30 N πριν τη χρήση |
| Component pins/clips | `stl-v2/component_pins.stl` ×1, `component_clips.stl` ×1 | P2-76 | Τυπωμένα pin/C-clips, χωρίς βίδα | **FIT-CHECK** — πρώτα draft coupon |
| BTS spacers | `stl-v2/bts_spacers.stl` ×1 | P2-76 | Σε συνδυασμό με pins/C-clips | **FIT-CHECK** — πρώτα draft coupon |
| Cable bulkhead + feet | `stl-v2/cable_bulkhead.stl` ×1, `bulkhead_foot.stl` ×2 | P2-75 retained in v2 | Υπάρχουσες M4, ροδέλες/παξιμάδια | **FIT-CHECK** — πρώτα coupon |
| Upright jaw coupon | `stl-v2/upright_mount_fit_coupon.stl` ×1 | P2-76 | M4 δοκιμής | **Τυπωμένο, fit PASS**· η σιαγόνα 4,8 mm/γλωττίδα 4 mm/M4 θέση κλειδώνουν |

Τα `rear-pod/stl-v2/mpu6050_mount.stl` και
`rear-pod/stl-v2/chassis_spacers.stl` δεν είναι ενεργά και αρχειοθετούνται.
Δεν υπάρχει M3 self-tap σε φέρον σημείο.

Τα παλιά P2-79 side frames έχουν τυπωθεί αλλά είναι **superseded**: οι τέσσερις
M4 δεν συναρμολογούνται. Δεν χρησιμοποιούνται· τα νέα όρθια P2-90d έχουν
τυπωθεί και φέρουν τα posts και το control panel.

Το P2-83 `corner_tie.stl` είναι ακριβώς ένα σώμα. Βρίσκεται κάτω από τη
βάση (`Z≤51`), στην εσωτερική παρειά της ράγας, και δεν περνά πάνω από τη
ράγα ή μέσα στο Parkside sweep. Τα print plates έχουν δηλωμένο αναμενόμενο
πλήθος: posts 4, pins 27, clips 32, BTS spacers 18 και jaw coupon 2.

## Ενεργά test fixtures

| Fixture / φυσικό κομμάτι | Ενεργό STL ή οδηγία | Πηγή | Βίδες | Κατάσταση |
|---|---|---|---|---|
| TPU friction plate | `test-fixtures/tpu-friction/plate.stl` | `tpu-friction-test.scad` | Ταινία στο τραπέζι | Μόνο δοκιμή· μέτρηση μ εκκρεμεί |
| TPU friction sled | `test-fixtures/tpu-friction/sled.stl` | `tpu-friction-test.scad` | Νήμα/βάρη, χωρίς φέρουσα βίδα | Μόνο δοκιμή |
| TPU pulley + bracket | `test-fixtures/tpu-friction/pulley.stl`, `pulley_bracket.stl` | `tpu-friction-test.scad` | M4×25 + παξιμάδι ως άξονας | Μόνο δοκιμή |
| Ski hand-test, τέσσερις παραλλαγές | `test-fixtures/ski-jump-hand-test/ski_42deg_R75.stl`, `ski_60deg_R75.stl`, `ski_75deg_R75.stl`, `ski_75deg_R110.stl` | `ski-jump-test.scad` | Ταινία/σφιγκτήρας στο τραπέζι | Μόνο χειροκίνητη δοκιμή |
| Εκτοξευτής εισόδου βατήρα (λάστιχα) | `test-fixtures/ramp-entry-ball-launcher/stl/ramp_entry_launcher_base.stl`, `ramp_entry_launcher_pusher.stl` | `ramp-entry-ball-launcher.scad` | Πείρος ατσάλινος Ø4×135–145, 4×M5×25/30 με λείο σωληνάκι, 2 ίδια λάστιχα· ταινία/Velcro/δεματικά | FIT-CHECK δοκιμαστικό· βάση 40%, έμβολο 25% infill· `ramp_surface_z`=11 επιβεβαιωμένο από χρήστη 2026-11-08 |
| Basket mockup | Δεν έχει STL· διαστάσεις στο `test-fixtures/basket-mockup/README.md` | P2-71 cardboard fixture | Ταινία/χαρτόνι | Μόνο δοκιμή· pile retention 40/45 μπαλών εκκρεμεί |

Το P2-71 καλάθι έχει τέσσερις πλευρές πλέγματος. Μόνο η fail-closed θύρα
`80×80 mm` είναι συμπαγής δίοδος τροφοδοσίας βολών· η τελική front-wall
λεπτομέρεια μένει υπό mockup.
| Drive-motor hole gauge | `drive-motor-mount/37mm-mount-hole-gauge.stl` | `drive-motor-mount/37mm-mount-hole-gauge.scad` | Πραγματικές βίδες steel mount μόνο για fit | Τυπωμένο· μόνο δοκιμή, fit-checked σε 1/4 mounts |
| M3 nut-keeper sleeve | `drive-motor-mount/nut-keeper-stl/sleeve.stl` | `drive-motor-mount/m3-side-nut-keeper.scad` | Δεν φέρει preload· συγκρατεί μόνο M3 nut | Μόνο δοκιμή εφαρμογής· physical retention εκκρεμεί |

## Panels F rear — P2-86b, αναμονή / ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ

Πηγή: `body-panels/body-panels-f-rear-v2.scad`. Release-candidate κατάλογος:
`body-panels/stl-f-rear/`. Τα δεκατρία STL (`rear_lo` L/R, `rear_hi` L/R,
`deck_rear` L/R, Parkside doors L/R, corner stays L/R, δύο εσωτερικά
center splices και turn-button που τυπώνεται ×2) είναι manifold, ένα σώμα και με print footprint έως
250×250 mm, αλλά μένουν σε κατάσταση **σχεδιασμένο, αναμονή** μέχρι το
μελλοντικό βήμα πλαϊνών/εμπρός και το basket mockup. Βίδες και σειρά
συναρμολόγησης βρίσκονται στο
`body-panels/stl-f-rear/README.md`.
Το πραγματικό R60 είναι εφαπτόμενο στο R424,5 και στη Y±244,5, με P2-87
ραφή μετά το fillet στο X−185. Τα `rear_lo`, `rear_hi`, `deck_rear` και
`corner_stay` και `center_splice` STL είναι νέα P2-86b exports. Τα skin
halves είναι 246,85 mm στη μεγάλη print διάσταση και τα deck halves
δίνονται επίπεδα σε 45° orientation (≤185×185 mm). Ο περιμετρικός χάρτης βρίσκεται
στο `docs/design/body-panels/p286b-perimeter-map-el.md`.

## Concepts — F επιλεγμένο, χωρίς ενεργό STL πλαϊνών/εμπρός

- **P2-80 rear cage:** μελλοντικό πλήρως τυπωμένο stackable PETG cage,
  χωρίς αλουμίνιο. Box sections περίπου 35×35 mm δεν χωρούν στη λωρίδα
  `Y161…167`, άρα θέσεις/διατομές κολονών θα ξανακριθούν με τα panels F.
- **Body panels F πλαϊνά/εμπρός:** `body-panels/body-panels-f.scad` και τα
  F renders/QA παραμένουν concept. Το rear subset έχει πλέον ξεχωριστή
  release-candidate γεωμετρία παραπάνω, αλλά δεν έχει εγκριθεί για εκτύπωση.
- **Concepts A–E:** superseded από F στο P2-86 και μεταφέρθηκαν σε
  `body-panels/archive/concepts-a-e/` με σήμανση **ΜΗΝ ΤΥΠΩΣΕΙΣ**.
- **Full robot v3:** `modular-test-chassis/full-robot-v3.scad` είναι assembly
  concept/check, όχι πηγή ενεργών STL.

## Ανοικτές φυσικές δοκιμές

Drive test με PSU, μέτρηση TPU `μ`, basket mockup/pile retention, ski hand
test και τελικό proof-load του P2-81 control panel. Καμία από αυτές τις
εκκρεμότητες δεν μετατρέπει concept CAD σε ενεργό STL.
