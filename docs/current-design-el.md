# Τρέχον design του tennis robot

Κατάσταση: **P2-89b — 2026-10-22**. Αυτό είναι το κεντρικό σημείο εκκίνησης
για τη σημερινή αρχιτεκτονική. Το φυσικό print status παραμένει στο
[`cad/ACTIVE-PRINT-SET.md`](../cad/ACTIVE-PRINT-SET.md), ενώ το πλήρες ιστορικό
πειραμάτων βρίσκεται στο
[`shared-flywheel-intake-launcher-debug-log-el.md`](mechanism/shared-flywheel-intake-launcher-debug-log-el.md).

## Αρχιτεκτονική με μία ματιά

- 4WD skid-steer βάση με modular v3 PETG σασί και τέσσερις FIT0403.
- Ένα κοινό περιστρεφόμενο ζεύγος flywheels για συλλογή και εκτόξευση.
- Σταθερό καλάθι, παθητικός βατήρας R75/75°, γερμένη εμπρός αψίδα και
  fail-closed θύρα τροφοδοσίας.
- Rear pod v2 για Mega, perfboard, BTS7960, Pi 5, UGREEN και τις μελλοντικές
  Parkside μπαταρίες.
- Body concept F. Μόνο τα πίσω panels είναι σχεδιασμένα και παραμένουν σε
  αναμονή· τα πλαϊνά/εμπρός περιμένουν το basket mockup.
- Canonical runtime: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic. Στο Ubuntu
  26.04 development PC χρησιμοποιείται το `docker/jazzy/run.sh`.

![Πλήρες robot στη θέση εκτόξευσης](../cad/modular-test-chassis/full-robot-v3-launch20.png)

## Κοινό περιστρεφόμενο flywheel

Δύο counter-rotating τροχοί σχηματίζουν το nip και οδηγούνται απευθείας από
δύο D5065-270KV. Περιστρέφεται ολόκληρη η μονάδα, όχι κάθε μοτέρ χωριστά:

- συλλογή: `−3°`,
- μεταφορά: `0°`,
- εκτόξευση με ακίνητο όχημα: `+15…+20°`.

Η γεωμετρία P2-40 είναι παγωμένη ως CAD baseline. Αυτό **δεν** αποτελεί
πιστοποίηση φυσικής πρόσφυσης, ταχύτητας εξόδου ή αντοχής. Η απόδοση παραμένει
παγωμένη μέχρι να επιβεβαιωθεί η φυσική κίνηση του robot και τα physical
traction/contact gates.

Η επιλογή κοινού μηχανισμού έγινε από τον χρήστη για συμπαγέστερη
αρχιτεκτονική και μικρότερο κόστος αγορών: κάθε D5065 αναφέρεται περίπου
`99 €`, ενώ ξεχωριστό dual-wheel intake και dual-wheel launcher θα
διπλασίαζαν μοτέρ και drivers. Πλήρης ποσοτική σύγκριση κόστους:
**ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ**.

![Κοινό flywheel στη συλλογή](../cad/modular-test-chassis/shared-flywheel-phase2_intake.png)

### Τροχοί flywheel

Στις 2026-10-22 παραγγέλθηκαν δύο pneumatic AliExpress τροχοί, nominal
`200×50 mm`, από την ίδια παρτίδα. Τα στοιχεία πωλητή —όχι μετρήσεις— είναι:

- πραγματική δηλωμένη εξωτερική διάμετρος `192 mm`,
- πλάτος πέλματος `48 mm`,
- μάζα `760 g` ανά τροχό,
- άξονας/έδρα `10 mm` με 6200 bearings.

Το frozen CAD έχει D5065 axes `Y±129 mm`, απόσταση `258 mm`, για D200 και nip
`58 mm`. Με D192 το ίδιο datum δίνει περίπου `66 mm`, σχεδόν χωρίς συμπίεση
μπάλας. Η λύση κατευθύνεται σε **ρυθμιζόμενη απόσταση αξόνων**, αλλά μένει
ΑΝΟΙΧΤΗ μέχρι να μετρηθούν και οι δύο τροχοί στην παραλαβή. Το `Y≈±125 mm`
είναι μόνο πρώτη γεωμετρική εκτίμηση, όχι release dimension.

## Μηχανισμός κλίσης

Η αποφασισμένη baseline μετά τα P2-53/P2-54 είναι ένας υπάρχων
**FIT0186 με μονόσπειρη Tr8×2**, offset 1:1 toothed-belt drive και anti-rotation
nut carriage. Το παλιότερο Actuonix `P16-50-256-12-S` των P2-11/P2-12
παραμένει compact direct fallback, όχι current actuator.

Η λειτουργική διαδρομή είναι `−3°` συλλογή → `0°` μεταφορά → `+15…+20°`
εκτόξευση. Για το FIT0186/Tr8 η υπολογισμένη διαδρομή `−3→+20°` είναι
33,976 mm: 4,20 s ideal/no-load και **4,5…6 s design range** μέχρι να
μετρηθούν continuous torque, θερμοκρασία, πραγματική απόδοση ιμάντα,
backdrive και backlash. Ένας actuator αρκεί στο analytical screen.

Στο `+20°` ο actuator κρατά μικρή ελεγχόμενη προφόρτιση πάνω στα seating
stops. Το δεύτερο συμμετρικό stop είναι υποχρεωτικό: η αντίδραση εκτόξευσης
περνά `yoke → stops → arch`, όχι από Tr8/actuator. Δύο εξωτερικά endpoint
switches στα `−3°/+20°` παρέχουν homing και επιβεβαίωση θέσης. Ανεξάρτητα
μηχανικά emergency stops περιορίζουν τη διαδρομή στα `−12°/+22°`.

Επιτρέπεται manual, spring-retained transport pin μόνο με τη μονάδα ακίνητη
και χωρίς τροφοδοσία· δεν είναι runtime lock. Έχουν παραληφθεί δύο FIT0186,
αλλά η αγορά/παραλαβή Tr8, belt hardware, switches και τελικών stops απαιτεί
**ΕΠΙΒΕΒΑΙΩΣΗ ΧΡΗΣΤΗ**.

## Διαδρομή μπάλας και καλάθι

- P2-73/P2-85: τυπωμένος βατήρας R75/75° με νέα brackets και wall forks.
- P2-74: selector rails στα `Y±55 mm`.
- P2-67: γερμένη αψίδα `X390/Z245` και deflector· η R75/75 baseline πέρασε
  90/90 cases και το συνολικό P2-67 matrix 180/180.
- P2-68/P2-69: όρθιο εμπρός τοίχωμα `X225`, top `Z235`, και fail-closed
  θύρα clear `80×80 mm`, επιβεβαιωμένα 30/30 στο σχετικό matrix.
- P2-71: το καλάθι σχεδιάζεται για 45 μπάλες, αλλά το φυσικό acceptance είναι
  **40 υποχρεωτικές / 41–45 best-effort** μέχρι basket mockup και pile test.

Το cardboard basket mockup, η συγκράτηση σωρού και το ski hand test παραμένουν
υποχρεωτικά πριν κλειδώσουν τα πλαϊνά panels και οι εσωτερικές επιφάνειες.

## Σασί και κίνηση

Το modular v3/P2-78 σασί είναι πλήρως τυπωμένο, μαζί με sockets, terminal
sleeves, side braces, corners και crossbars. Τα εύθραυστα T-clamps και το
P2-77 centre torsion node έχουν καταργηθεί. Το μόνο ενεργό MPU mount είναι το
P2-77 rail-side mount.

Η βάση είναι 4WD skid-steer με τέσσερις FIT0403. Έχει επιβεβαιωθεί **ένα
ζευγάρι / δύο τεμάχια** HPI Baja rear `170×80 mm`. Η ταυτότητα/διάσταση των
άλλων δύο drive wheels απαιτεί **ΕΠΙΒΕΒΑΙΩΣΗ ΧΡΗΣΤΗ**· δεν θεωρούνται αυτόματα
ίδια.

Δεν ανοίγεται νέα οπή στο ήδη τυπωμένο σασί. Οι κοινές συνδέσεις πρέπει να
χρησιμοποιούν υπάρχουσες οπές και να περνούν τους μόνιμους ελέγχους:

- hole-axis probe έναντι του πραγματικού `frame_assembly`,
- ακριβώς ένα σώμα ανά ενεργό STL, εκτός δηλωμένων print plates,
- πλήρες hardware και ≥60 mm tool-access cylinder στη σειρά συναρμολόγησης.

## Rear pod v2 και ηλεκτρονικά

Το rear pod v2 καλύπτει τα P2-75…P2-88. Οι P2-86b βάσεις έχουν τυπωθεί και
είναι κλειδωμένες. Τα παλιά side frames τυπώθηκαν αλλά καταργούνται λόγω
μη συναρμολογήσιμων M4· αντικαθίστανται από τα P2-88 side frames. Το upright
jaw coupon τυπώθηκε και πέρασε, άρα η σιαγόνα `4,8 mm`, η γλωττίδα `4 mm`
και η αντίστοιχη M4 θέση δεν αλλάζουν.

Η σημερινή, **μη τροποποιημένη** Rev-C perfboard και το επαληθευμένο firmware
`motion_intake_mega.ino` εξακολουθούν να αντιστοιχούν τα τέσσερα BTS7960 ως:

1. drive left — `D5/D6`, enable `D30`,
2. drive right — `D9/D10`, enable `D31`,
3. intake left — `D44/D45`, enable `D40`,
4. intake right — `D46/D11`, enable `D42`.

Τα δύο intake BTS οδηγούν από ένα FIT0186 στο σημερινό bring-up firmware.
Το tilt actuator και οι endpoint sensors **δεν υπάρχουν** στη Rev-C netlist.

Η αποφασισμένη τελική κατανομή καναλιών, της οποίας η υλοποίηση εκκρεμεί,
είναι:

1. BTS #1 — drive left, `D5/D6`, EN `D30`, ζεύγος μοτέρ αριστερής πλευράς,
2. BTS #2 — drive right, `D9/D10`, EN `D31`, ζεύγος μοτέρ δεξιάς πλευράς,
3. BTS #3 — tilt FIT0186+Tr8×2, `D44/D45`, EN `D40`,
4. BTS #4 — ρεζέρβα, σημερινά `D46/D11`, EN `D42`.

Οι απαιτήσεις της μελλοντικής netlist/firmware υλοποίησης είναι:

- quadrature encoder του FIT0186 σε pin-change pins· τα `A8…A15` είναι
  `PCINT16…23`. Προτιμώμενος υποψήφιος είναι `A10/A11`, ήδη δρομολογημένος
  ως `J_ENC_IL`. Τα `A8/A9` επίσης απελευθερώνονται· τα `A12…A15` είναι
  ελεύθερα στο firmware αλλά δεν υπάρχουν στο σημερινό 34-pin harness,
- δύο endpoint switches `−3°/+20°` και deterministic homing,
- direction-specific hardware limit: κάθε endpoint αποκόπτει μόνο το
  `R_EN` ή `L_EN` της κίνησης προς το αντίστοιχο όριο. Η σημερινή Rev-C
  γεφυρώνει R_EN/L_EN, άρα αυτό απαιτεί νέα, ελεγμένη netlist,
- stall detection από τα BTS `R_IS/L_IS`, τα οποία επίσης δεν δρομολογούνται
  στο σημερινό control harness· ADC pins και thresholds μένουν να οριστούν,
- αρχικό PWM cap περίπου 60% με Parkside 18–20 V. Αυτό είναι safety/design
  requirement, όχι ισοδύναμο ρυθμισμένης 12 V τροφοδοσίας, και απαιτεί
  current/thermal validation πριν release.

Firmware και Rev-C/netlist δεν αλλάζουν πριν ολοκληρωθεί το drive test και
δοθεί νέα ρητή έγκριση χρήστη. Οι D5065 controllers παραμένουν ξεχωριστή,
μη επιλεγμένη αγορά και δεν ταυτίζονται με BTS7960.

Άλλα βασικά στοιχεία: Mega 2560, keyed perfboard, Pi 5, UGREEN power bank,
MPU6050, PCA9685, E-stop `D33`, START `D32`, ARMED LED `D34` και break beams
`D36/D37`. Τα δύο Parkside X20V packs/docks είναι provisional και δεν έχουν
αγοραστεί.

![Rear pod v2 P2-88](../cad/rear-pod/p288-assembly-isometric.png)

Τα Ruthex RX-M3x5.7 heat-set inserts επιτρέπονται για καλύμματα, θήκες και
docks μέτριου φορτίου. Δεν χρησιμοποιούνται στις κύριες M4 φέρουσες
συνδέσεις, οι οποίες παραμένουν διαμπερείς με ροδέλα/nyloc.

## Body concept F

Το F είναι το μόνο ενεργό body concept. Τα A–E και v3 είναι archived. Τα
πίσω `rear_lo`, `rear_hi`, `deck_rear`, Parkside doors, stays και splices
έχουν σχεδιαστεί στο P2-84…P2-86b αλλά παραμένουν **ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ**.
Ο περιμετρικός χάρτης P2-86b ορίζει το R60 fillet και τις μελλοντικές ραφές.

![Concept F και περιμετρικά σημεία](../cad/body-panels/p286b-perimeter-top.png)

Τα πλαϊνά και εμπρός panels περιμένουν το basket mockup. Οι S4/front
στηρίξεις κοντά στην αψίδα παραμένουν ανοικτές επειδή δεν υπάρχει υφιστάμενη
οπή· δεν επιτρέπεται να προστεθεί οπή στο τυπωμένο σασί.

## Runtime

- Κανονικό περιβάλλον: native Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
  είσοδος `./run_native.sh`.
- Ubuntu 26.04 development PC: `docker/jazzy/run.sh`, δηλαδή το ίδιο
  Ubuntu 24.04/Jazzy runtime μέσα σε container.
- Το παλιό Docker/Humble είναι obsolete και δεν αποτελεί fallback.

## Τι είναι τυπωμένο και τι εκκρεμεί

Τυπωμένα/επιβεβαιωμένα:

- ολόκληρο modular chassis v3/P2-78,
- R75/75 ski,
- rear-pod P2-86b base left/right,
- παλιά side frames — **superseded, δεν χρησιμοποιούνται**,
- upright jaw coupon — fit PASS.

Σε εκτύπωση τώρα: P2-85 X300 brackets και wall forks.

Κύρια ανοικτά φυσικά gates:

- drive test με PSU,
- μέτρηση τριβής TPU `μ`,
- basket mockup και pile retention 40/45,
- ski hand test,
- proof-load 30 N του control panel,
- tilt implementation: νέα netlist/harness, firmware, homing/endpoints,
  direction-specific hardware limits, IS stall thresholds και FIT0186/Tr8
  current/thermal/backdrive test,
- μετρήσεις των AliExpress flywheel wheels και των μελλοντικών Parkside.

Με απόφαση χρήστη, δεν γίνονται άλλες κύριες αγορές πριν επιβεβαιωθεί η φυσική
κίνηση. Εξαιρούνται φθηνά samples που αγοράζονται αποκλειστικά για μέτρηση.

## Πηγές αλήθειας

- Physical print status: [`cad/ACTIVE-PRINT-SET.md`](../cad/ACTIVE-PRINT-SET.md)
- P2 history: [`shared-flywheel-intake-launcher-debug-log-el.md`](mechanism/shared-flywheel-intake-launcher-debug-log-el.md)
- Decisions: [`decisions/decision-log-el.md`](decisions/decision-log-el.md)
- Parts actually ordered: [`hardware/ordered-parts.md`](hardware/ordered-parts.md)
- Rev-C wiring: [`hardware/unified-keyed-perfboard-wiring-el.md`](hardware/unified-keyed-perfboard-wiring-el.md)
- Body F perimeter: [`design/body-panels/p286b-perimeter-map-el.md`](design/body-panels/p286b-perimeter-map-el.md)
