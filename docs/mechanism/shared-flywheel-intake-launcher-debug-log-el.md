# Debug log — κοινό flywheel intake/launcher

## 2026-10-11 — Φάση 1 CAD και analytical/FEA screen

### Baseline

- Branch/checkpoint: `codex/shared-flywheel-intake-launcher` / `5196885`.
- Το working tree ήταν καθαρό στην έναρξη.
- Διαβάστηκαν `AGENTS.md` και
  `shared-flywheel-intake-launcher-handoff-el.md`.
- Δεν εκτελέστηκε ROS, xacro ή Gazebo, σύμφωνα με το υποχρεωτικό stop.

### Πείραμα P1-01 — αρχικό Ø200 × 50 mm

- Παράμετροι: κέντρα `±129 mm`, διάκενο `58 mm`, pivot `[340,0,57,82] mm`,
  nip local `U=45 mm`, γωνίες `−10/+20°`.
- Αποτέλεσμα: ground-clearance screen εφικτό μόνο με πολύ μικρή κάτω γέφυρα,
  αλλά ο τροχός έφτανε σε `|Y|=229 mm` και συγκρουόταν με τις δοκούς που
  αρχίζουν στο `|Y|=195 mm`.
- Απόφαση: REJECTED για τη συγκεκριμένη διάταξη σασί. Δεν κόπηκαν οι βασικές
  διαμήκεις δοκοί για να «περάσει» ο τροχός.

### Πείραμα P1-02 — Ø160 × 44 mm

- Παράμετροι: κέντρα `±109 mm`, διάκενο `58 mm`, ίδιο pivot/γωνίες.
- Αποτέλεσμα: πλευρικό ονομαστικό διάκενο `6 mm`, wheel-ground minimum
  `14,45 mm`.
- Περιορισμός: δεν περιλαμβάνονται ανοχές, runout, elastic deflection ή guard.

### Πείραμα P1-03 — πλήρης κάτω πλάκα

- Παράμετρος: παλιά plane θέση `W=-43 mm`.
- Αποτέλεσμα: δεν υπάρχει πλήρες ωφέλιμο μήκος που να καθαρίζει το έδαφος και
  στα δύο endpoints. Η αρχική στενή γέφυρα `U=-25…-5 mm` δοκιμάστηκε αλλά
  αργότερα απορρίφθηκε στο P1-08 λόγω wheel/yoke intersection.

### Πείραμα P1-04 — δομική τραβέρσα

- Αρχική θέση: X=`294,4 mm`.
- Αποτέλεσμα: πραγματική τομή με το sweep των τροχών.
- Διόρθωση: νέα εναλλακτική θέση X=`240 mm`, με την ίδια 50×20 λειτουργία
  και πλαϊνές διαγώνιες λωρίδες. Η δομική λειτουργία διατηρείται.

### Πείραμα P1-05 — OpenSCAD collision sweep

**Αντικαταστάθηκε εν μέρει από P1-12.** Το αποτέλεσμα ήταν σωστό μόνο για το
τότε μοντέλο γυμνού σασί· τα FIT0403 και τα cable keep-outs έλειπαν.

Εντολές (μία για κάθε view):

```bash
/snap/bin/openscad-nightly \
  -D 'view="qa_sweep_frame"' \
  -o /tmp/shared-qa-frame.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad

/snap/bin/openscad-nightly \
  -D 'view="qa_sweep_basket"' \
  -o /tmp/shared-qa-basket.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad

/snap/bin/openscad-nightly \
  -D 'view="qa_battery_frame"' \
  -o /tmp/shared-qa-battery.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

- Sweep: `−10…+20°`, βήμα `2°`.
- Αποτέλεσμα: και τα τρία μη σκόπιμα intersection views είναι κενά.
- Μόνο η τελική trunnion/bearing εφαρμογή εξαιρείται ως σχεδιασμένη διεπαφή.

### Πείραμα P1-06 — analytical και FEA

**Αντικαταστάθηκε από P1-10/P1-11/P1-14.** Η πάκτωση μόνο των πίσω κόμβων
X=`−165 mm`, το ενιαίο combined load και η χωρητικότητα 12 μπαλών δεν
χρησιμοποιούνται πλέον ως τελικό συμπέρασμα.

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_shared_flywheel_phase1.py
```

- Output: `engineering/chassis-strength-bench/results/shared_flywheel_phase1/report.json`.
- Meshes: `20/10/5 mm`, υλικά PLA/PETG/PETG-CF, με/χωρίς νέα τραβέρσα X240.
- Loads ανά bearing: `Fx=24,435 N`, `Fz=37,740 N`, `My=2653,7 Nmm` περίπου
  ανά πλευρά, από το ανανεωμένο mass/contact screen.
- Finest retained-crossbar αποτέλεσμα: displacement `12,92/19,77/16,51 mm`
  για PLA/PETG/PETG-CF, peak ideal beam stress `4,47/4,50/4,47 MPa`.
- Περιορισμοί: bonded ideal joints, rear clamp boundary, χωρίς bolts, bearing
  housing, layer interfaces, hole/notch stress, creep, fatigue ή μετρημένα
  δυναμικά loads. Δεν αποτελεί print approval.

### Πείραμα P1-07 — ψηλή παράδοση intake

- Αίτημα: να παραμείνει ανοικτός ο χαμηλός μπροστινός όγκος του καλαθιού όταν
  γεμίζει.
- Αλλαγή: το χαμηλό fixed receiver αντικαταστάθηκε από κλειστό intake-only
  οδηγό με ball-centre path `[274,0,78] → [244,0,158] → [205,0,148] mm`.
- Αποτέλεσμα: **απορρίφθηκε μετά από οπτικό review**. Το πρώτο τμήμα ανέβαινε
  80 mm σε 30 mm οριζόντιας διαδρομής (περίπου 70°), άρα η μπάλα θα χτυπούσε
  την είσοδο με μεγάλη κάθετη συνιστώσα και υπήρχε σοβαρός κίνδυνος rebound.
- Περιορισμοί: η πραγματική τριβή, οι απώλειες κύλισης, το rebound, το jam
  access και ο full-basket sensor παραμένουν μη επικυρωμένα.

### Πείραμα P1-08 — διόρθωση wheel/yoke intersection

- Παρατήρηση χρήστη: η σύνδεση πλάκας φαινόταν να περνά μέσα από τον τροχό.
- Επιβεβαίωση: η άνω πλάκα είχε 17 mm axial clearance, αλλά οι δύο εσωτερικοί
  ορθοστάτες και τα trunnions βρίσκονταν μέσα στο wheel cylinder. Η εξαίρεση
  trunnion/bearing του γενικού QA είχε κρύψει το λάθος.
- Διόρθωση: αφαιρέθηκαν κάτω γέφυρα και εσωτερικοί ορθοστάτες. Δύο άνω
  βραχίονες στο `W=+43 mm` περνούν πάνω από τους τροχούς και τις δοκούς και
  κατεβαίνουν μόνο στο `|Y|=260 mm`, έξω από το σασί `|Y|=245 mm`.
- Νέος ρητός έλεγχος:

```bash
/snap/bin/openscad-nightly -D 'view="qa_wheel_yoke"' \
  -o /tmp/shared-wheel-yoke-qa.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

- Αποτέλεσμα: `Current top level object is empty.` Το διακριτό
  `qa_sweep_frame` παραμένει επίσης κενό.
- Ονομαστικά διακένa: 17 mm πάνω από wheel face και 61 mm από το wheel outer-Y
  έως την εσωτερική παρειά του trunnion.
- Περιορισμός: τα νέα cantilever yokes χρειάζονται local plate/bracket FEA,
  fastener definition και φυσικό stiffness coupon.

### Πείραμα P1-09 — βαθύτερη ομαλή είσοδος και κοντύτερο καλάθι

- Παρατήρηση χρήστη: η απότομη ανύψωση θα λειτουργούσε σαν εμπόδιο και θα
  επέστρεφε την μπάλα.
- Διόρθωση: και οι δύο προτεινόμενες κατευθύνσεις συνδυάστηκαν. Ο οδηγός έγινε
  βαθύτερος/ομαλότερος και το επάνω μισό του καλαθιού μεταφέρθηκε προς τα μέσα
  και χαμήλωσε.
- Νέα ball-centre path: `[274,0,78] → [250,0,82] → [226,0,94] →
  [202,0,111] → [178,0,128] → [154,0,136] → [132,0,132] mm`.
- Χαρακτηριστικά: μήκος περίπου 142 mm, αρχική εφαπτομένη `9,5°`, μέγιστη
  τμηματική κλίση `35,3°`, peak lift 58 mm και τελική κάθοδος 4 mm.
- Καλάθι: κάτω μπροστινό άκρο X278/Z54 mm, πάνω μπροστινό άκρο X205/Z195 mm.
  Έτσι διατηρείται το χαμηλό άνοιγμα, αλλά δημιουργείται χώρος για συλλογή από
  ψηλά.
- Ιδανικό κατώφλι ανύψωσης `1,07 m/s`, συντηρητικό κατώφλι με 50% μεταφορά
  ενέργειας `1,51 m/s`. Σάρωση `1,5/1,7/1,9 m/s` (`179/203/227 rpm`).
- Τα `qa_sweep_high_guide` και `qa_sweep_basket` επέστρεψαν κενή τομή.
- Περιορισμός: η συνέχεια της πραγματικής καμπύλης, το rebound και η ροή με
  μερικώς γεμάτο καλάθι πρέπει να επικυρωθούν δυναμικά πριν παγώσει το CAD.

### Πείραμα P1-10 — FEA με στηρίξεις στους δύο άξονες

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_shared_flywheel_phase1.py
```

- Στηρίξεις: U3 στα τέσσερα drive mounts X=`−65/300 mm`, U1/U2 πίσω αριστερά
  και U2 εμπρός αριστερά. Οι στροφές δεν πακτώνονται. Στο wheel-lift αφαιρείται
  το U3 εμπρός δεξιά.
- Mesh/material sweep: `20/10/5 mm`, PLA/PETG/PETG-CF, με/χωρίς X240.
- Χωριστές φορτίσεις: symmetric vertical 2g, launch reaction, 1g με έναν τροχό
  σηκωμένο και antisymmetric racking.
- Finest 5 mm με X240, PLA/PETG/PETG-CF:
  - vertical 2g: `0,293/0,449/0,374 mm`·
  - launch reaction: `0,709/1,084/0,906 mm`·
  - one-wheel lift: `4,93/7,62/6,30 mm`·
  - racking relative bearing-X: `0,745/1,139/0,952 mm`.
- Χωρίς X240 το racking είναι `6,27/9,60/8,01 mm`. Η τραβέρσα μειώνει το
  racking περίπου 88%, το launch response 93% και το wheel-lift response 73%.
- Mesh change 10→5 mm: μέγιστο `0,85%`.
- Συμπέρασμα: τα παλιά `12,9…19,8 mm` ήταν artefact του cantilever support και
  αποσύρονται. Η X240 είναι δομικά ουσιαστική. Η torsional compliance στο
  wheel-lift δεν είναι αμελητέα.
- Περιορισμός: ideal bonded B32R beams. Τοπικό yoke/bearing/trunnion, bolts,
  layer interfaces, notches, creep και fatigue δεν περιλαμβάνονται και
  παραμένουν χωριστό gate.

### Πείραμα P1-11 — nip force, TPU compliance και slip limit

- Γεωμετρία: Ø66 mm μπάλα σε 58 mm gap, άρα 8 mm diametral closure και 4 mm
  ανά contact.
- Νόμος μπάλας: `F=107309·δ^1,5` σε SI. Το TPU μοντελοποιήθηκε ως άκαμπτο,
  γραμμικό `10 N/mm` και γραμμικό `3 N/mm`.
- Λυμένα ζεύγη ball/TPU compression και Fn ανά τροχό:
  `4,00/0/27,15 N`, `2,59/1,41/14,12 N`, `1,64/2,36/7,09 N`.
- Για κάθε compliance case εκτελέστηκε `μ=0,3/0,6/0,9`. Τα πλήρη Ft και
  ideal-energy Δv γράφονται στο `report.json`; το εύρος Δv είναι
  `2,98…10,08 m/s`.
- D5065 theoretical no-load στα 12,8 V: `3456 rpm`. Ø160=`28,95 m/s`,
  Ø200=`36,19 m/s`. Με μόνο συγκριτική εφαρμογή του προηγούμενου
  30→8,25 m/s ratio, προκύπτουν `7,96` και `9,95 m/s`: απώλεια Ø160 20% ή
  `1,99 m/s` έναντι Ø200.
- Περιορισμός: οι stiffness τιμές TPU δεν είναι μετρήσεις και ο λόγος 0,275
  δεν είναι μοντέλο πρόβλεψης· το root-cause test ήταν traction-saturated.

### Πείραμα P1-12 — θετικά FIT0403 envelopes και sweep

- Προστέθηκαν solids για steel foot/upright, gearbox, motor, encoder, shaft,
  hub και connector/cable-bend keep-out στα X=`−65/300 mm`.
- Μετρημένα/fit-checked: foot `40×40 mm`, holes `25×30 mm`. Το περίπου 47 mm
  bracket height είναι προσεγγιστική μέτρηση. Όλα τα μήκη του coaxial stack
  και του καλωδίου είναι δηλωμένες συντηρητικές υποθέσεις.
- Εντολή ανά γωνία:

```bash
/snap/bin/openscad-nightly \
  -D 'view="qa_drive_at_angle"' -D 'qa_angle=-10' \
  -o /tmp/shared-drive-minus10.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

- Εκτελέστηκε για `−10…+20°` ανά 2°, με τα δύο endpoints.
- Αποτέλεσμα: hard intersection και στα 16 δείγματα, minimum clearance
  `0 mm`. Representative `−10°` intersection bounds:
  X=`312,25…314`, `|Y|=136…140,62`, Z=`74…83,92 mm`, μέσα στο assumed front
  connector/cable bend keep-out.
- Συμπέρασμα: **PACKAGING STOP**. Δεν αποδεικνύεται σύγκρουση του πραγματικού
  connector μέχρι να μετρηθεί, αλλά το conservative envelope δεν επιτρέπει
  έγκριση ή θετικό clearance claim.

### Πείραμα P1-13 — lateral budget, χώμα και μάγουλα

- Το ονομαστικό wheel–rail clearance 6 mm φορτώθηκε με `0,8 mm` ανοχή,
  `1,0 mm` runout, `2,0 mm` flex και `2,0 mm` guard: απομένει `0,2 mm`, άρα
  `INSUFFICIENT_FOR_RELEASE`.
- Επιλογές χωρίς εφαρμογή: μικρότερο wheel spacing με νέο nip check, τοπική
  εσοχή δοκού με strength check ή μετατόπιση της μονάδας κατά X.
- Το launch port έχει 4,3 mm ground clearance. Σε clay χρειάζεται elevated
  feed shoe και brush/labyrinth flap ή υψηλότερο dock.
- Τα μάγουλα έγιναν παραμετρικά: `cheek_variant="narrow"` δίνει
  `±72→±46 mm`, ενώ `"wide"` δίνει `±110→±46 mm`. Το πρώτο προστατεύει από
  side strikes, το δεύτερο αυξάνει capture width και αξιοποιεί self-centring.
  Η wide εκδοχή παραμένει clear από structure/basket/high-guide στο ίδιο
  διακριτό sweep· μοιράζεται όμως το FIT0403 connector stop. Η επιλογή
  αναβάλλεται για τις πλευρικές δυναμικές δοκιμές της Φάσης 3.

### Πείραμα P1-14 — χωρητικότητα και full-basket loads

- Gross cavity από την εσωτερική CAD διατομή: περίπου `8,15 L`.
- Πρακτικό range με packing και κεντρικό guide: 24–28 μπάλες.
- Το structural/CG screen άλλαξε από 12 σε 28 μπάλες (`1,616 kg`).
- Full case: `22,55 kg`, CG `[115,8,0,102,9] mm`, rear/front load ανά τροχό
  `55,82/54,77 N`.
- Περιορισμός: η πραγματική χωρητικότητα εξαρτάται από bridge/jam formation
  και πρέπει να μετρηθεί με φυσικό bin ή multi-ball dynamics.

### Παραδοτέα εικόνων

```bash
/snap/bin/openscad-nightly -D 'view="intake"' --render --imgsize=1400,900 \
  --viewall --autocenter -o cad/modular-test-chassis/shared-flywheel-phase1-intake.png \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

Η ίδια εντολή εκτελέστηκε για `view="launch"` και `view="overlay"`.

### Κατάσταση

`PHASE_1_REVIEWED_WITH_PACKAGING_COLLISION_AND_OPEN_LOCAL_STRUCTURAL_GATES`.
Παύση πριν από Φάση 2 μέχρι ρητή έγκριση.

## 2026-10-11 — Αναθεώρηση με πραγματικές μετρήσεις

Τα P1-01…P1-14 δεν διαγράφονται. Τα ακόλουθα αποτελέσματα
**αντικαθίστανται ρητά**:

- P1-01/P1-02: η απόρριψη Ø200 και η επιλογή Ø160 αφορούσαν μακριές δοκούς
  έως X455 και παλιά θέση άρθρωσης·
- P1-09: ο οδηγός μήκους 142 mm αντικαθίσταται από βαθύτερο 197 mm·
- P1-10: η μεθοδολογία στηρίξεων διατηρείται, αλλά αριθμοί/γεωμετρία/φορτία
  αντικαθίστανται από P1-19·
- P1-11: η συμπίεση είχε διαιρεθεί λανθασμένα ανά contact και ο λόγος 0,275
  χρησιμοποιήθηκε συγκριτικά· αντικαθίσταται πλήρως από P1-17·
- P1-12: το hard collision προερχόταν από υποθετικό αξονικό connector που
  δεν υπάρχει στο μετρημένο FIT0403· αντικαθίσταται από P1-15/P1-16·
- P1-13: το budget 6 mm αφορούσε Ø160 δίπλα στη μακριά δοκό·
- P1-14: η χωρητικότητα 24–28 παραμένει, αλλά mass/CG αντικαθίστανται.

### Πείραμα P1-15 — drive measurements και envelope topology

Παράμετροι: tyre 83 mm, drive axle CAD Y=297 mm, overall width 677 mm,
face-to-encoder 77–78 mm inward, outer hex 47–49 mm και αβεβαιότητα ±2–3 mm.
Το cable exit έγινε radial κοντά στον encoder και οδηγείται προς τα κάτω.

Αλλαγές CAD: αφαιρέθηκε το αξονικό connector keep-out, προστέθηκαν radial
exit/downward cable solids και πραγματικές θέσεις tyres/shafts/hubs. Το κύριο
xacro ελέγχθηκε read-only: `wheel_y=0.35`, wheel separation 0,70 m έναντι
πραγματικού περίπου 0,58…0,59 m. Δεν τροποποιήθηκε.

Περιορισμός: οι μετρήσεις είναι με μεζούρα, όχι παχύμετρο.

### Πείραμα P1-16 — Ø200 μπροστά από κοντύτερο frame

Παράμετροι: Ø200×50, κέντρα ±129, pivot `[397,0,64,30] mm`, rails έως X330,
trunnions `|Y|=237`, angles −10…+20°. Το Ø160×44 παραμένει με
`flywheel_variant="d160"`.

Εντολές QA:

```bash
for v in d200 d160; do
  for q in qa_sweep_frame qa_sweep_drive qa_sweep_basket \
           qa_sweep_high_guide qa_wheel_yoke; do
    /snap/bin/openscad-nightly -D "flywheel_variant=\"$v\"" \
      -D "view=\"$q\"" -o /tmp/${v}_${q}.stl \
      cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
  done
done
```

Αποτέλεσμα: όλα τα intersection views κενά. Minkowski drive clearance:
Ø200 καθαρό 8,625 mm / πρώτη τομή 8,75 mm, Ø160 καθαρό 8,00 mm / πρώτη τομή
8,25 mm. Ø200 endpoint dimensions: 709,14×677×222 mm intake και
706,81×677×222 mm launch. Minimum rear wheel–rail-end 6,77 mm, ground 14,50
mm, flywheel outer-Y 229 mm και tyre-inner lateral gap 26,5 mm.

Ο οδηγός έγινε `[329,0,78]…[132,0,132] mm`, περίπου 197 mm, 55 mm βαθύτερος.
Τα spherical ή locating/floating bearings έγιναν απαίτηση λόγω frame twist.

Περιορισμός: το measured-clearance bracket δεν περιλαμβάνει αβεβαιότητα
μεζούρας ή τελικό cable bend radius.

### Πείραμα P1-17 — σωστή διαμετρική συμπίεση

Εξίσωση: `8 mm = δ_ball + 2δ_tread`,
`107309·δ_ball^1,5 = k_tread·δ_tread`. Αποτελέσματα Fn/τροχό:

- rigid 76,78 N·
- TPU-order k=250/1000/5000 N/mm: 68,98/74,64/76,34 N·
- legacy foam reference k=3 N/mm: 9,10 N (όχι candidate)·
- pneumatic unknown k=10/30/100 N/mm: 22,40/41,01/60,14 N.

Για κάθε case εκτελέστηκαν μ=0,3/0,6/0,9 και Ø200/Ø160. Το 30→8,25 m/s
κρατήθηκε μόνο ως traction-saturated observation· δεν έγινε πρόβλεψη με
λόγο 0,275. D5065 no-load 3456 rpm: Ø200 χρειάζεται 2865 rpm για 30 m/s,
Ø160 3581 rpm και άρα υπερβαίνει το no-load κατά 3,6%.

### Πείραμα P1-18 — μάζα, αδράνεια και 28 μπάλες

Printed Ø200: 0,838 kg και inertia bounds 0,00419…0,00838 kg·m². Pneumatic:
seller 0,900 kg, μη μετρημένο, συν assumed hub 0,0268 kg και tyre inertia
0,0045…0,0090 kg·m². Module 4,527/4,706 kg αντίστοιχα.

Με 28 μπάλες: printed total 23,233 kg, CG X=135,1 mm και rear/front ανά
τροχό 51,45/62,47 N. Pneumatic total 23,411 kg, CG X=137,4 mm και
51,15/63,64 N. Η χωρητικότητα παραμένει packaging estimate 24–28.

### Πείραμα P1-19 — νέα global FEA και local arm screen

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_shared_flywheel_phase1.py \
  --output engineering/chassis-strength-bench/results/shared_flywheel_phase1/report.json \
  --work engineering/chassis-strength-bench/fea/work/shared_flywheel_phase1
```

Global supports: drive axles X=−65/300, minimal constraints και free
rotations. Geometry: rails X330, bearing arms έως `[397,±237,64,3]`. Loads:
vertical 2g, launch reaction, one-wheel lift, antisymmetric racking. Finest
5 mm με X240, PLA/PETG/PETG-CF:

- vertical 0,609/0,925/0,778 mm·
- launch 1,793/2,741/2,291 mm·
- wheel-lift 7,756/11,981/9,910 mm·
- racking relative-X 1,936/2,959/2,473 mm.

Χωρίς X240: launch έως 45,61 mm, wheel-lift έως 45,65 mm και racking έως
24,32 mm. Η X240 είναι υποχρεωτική. Mesh 10→5 mm change έως 1,34%.

Separate local ideal cantilever: Al 70×45×12 mm, 83,10 N, 5,39 MPa και
0,021 mm. Bearing fit/rating, holes, bolts, shoulder και fatigue παραμένουν
ανοικτά.

### Πείραμα P1-20 — D5065 purchase gates

- Spin-up στα 8 A/motor και 30 m/s: printed 5,07…10,13 s, pneumatic
  5,44…10,89 s, acceptance ≤12 s.
- Droop ≤5%, recovery ≤1 s.
- Intake Ø200 143…181 rpm, acceptance ≤10 A/motor χωρίς stall/rebound.
- 4×FIT0403 stall≈28 A και 2×D5065 provisional≈40 A έναντι BMS≥20 A:
  measured total peak≤20 A / continuous≤16 A ή αναβάθμιση και interlock.
- Braking energy δύο τροχών: printed 377…754 J, pneumatic 405…810 J.
- Ένας shaft encoder ανά D5065, AMT-102 class ή equivalent. Απαιτούνται dual
  speed/current loops, telemetry, safe disable και bus over-voltage handling.

Δεν επιλέχθηκε controller και δεν απελευθερώθηκε αγορά.

### Νέα κατάσταση

`PHASE_1_REVIEWED_WITH_REAL_MEASUREMENTS_AND_OPEN_PHYSICAL_GATES`.
Παύση πριν από xacro/Gazebo. Δεν έγινε commit, push ή PR.

## Φάση 2 — CAD/FEA πριν από xacro/Gazebo

### Πείραμα P2-01 — baseline και όριο εργασίας

Επιβεβαιώθηκαν branch/worktree, τα Phase-1 artifacts και το datum LiDAR
Z=0,498 m του commit `f9224f2`. Κρατήθηκε η ρητή εξαίρεση του χρήστη:
native Ubuntu 24.04 + ROS 2 Jazzy μόνο και ερώτηση πριν από κάθε ROS/Gazebo.
Στα P2-01…P2-08 **δεν εκτελέστηκε ROS ή Gazebo** και δεν τροποποιήθηκε ο
πλήρης robot stack, Arduino, ηλεκτρονικά ή καλωδίωση.

### Πείραμα P2-02 — topology FEA και σωστή μετρική

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_shared_flywheel_phase2.py \
  --output engineering/chassis-strength-bench/results/shared_flywheel_phase2/report.json \
  --work engineering/chassis-strength-bench/fea/work/shared_flywheel_phase2
```

PETG `E=1500 MPa`, B32R, mesh 10/5 mm, supports X=−65/300, load cases
vertical 2g / launch / one-wheel lift / antisymmetric racking. Προστέθηκε
μέτρηση bearing ως προς το αντίστοιχο εμπρός drive mount, επειδή η απόλυτη
μετατόπιση του minimal-support model περιλαμβάνει τη μετατόπιση όλης της
βάσης και οδηγεί όλο το Fx σε ένα μόνο longitudinal restraint.

Αρχική αψίδα με βαθιούς hangers έδωσε καλή FEA αλλά απέτυχε στο CAD wheel
sweep και απορρίφθηκε. Η τελική collision-free αψίδα τελειώνει στο X330 και
φέρει actuator, όχι bearing hangers.

### Πείραμα P2-03 — sleeve sensitivity

`clearance=0,30 mm/side`, engagement 60 mm:
`theta=2 atan(0,30/60)=0,573°`. Με launch moment 3,110 N·m/πλευρά,
`kθ=311 N·m/rad`. Για 50×30×3,2 mm δόθηκε secant equivalent
`E=288 MPa`. Περιορισμός: γραμμικοποίηση στο συγκεκριμένο φορτίο, όχι
nonlinear contact/backlash.

### Πείραμα P2-04 — τελικά FEA αποτελέσματα

Finest mesh 5 mm, σειρά: absolute launch / local launch / vertical 2g /
wheel-lift / racking relative-X, σε mm:

- open U bonded: `2,752 / 0,587 / 2,092 / 12,608 / 3,295`·
- arch bonded: `2,871 / 0,876 / 2,068 / 8,112 / 3,035`·
- triangulation-only bonded: ουσιαστικά open U·
- combined bonded: `1,803 / 0,770 / 2,055 / 6,946 / 2,286`·
- combined sleeve: `1,897 / 0,922 / 2,788 / 8,201 / 2,511`.

Max mesh change 10→5 mm: 1,178%. Ο τοπικός στόχος ≤1 mm περνά μόνο από τον
combined design, ακόμη και με sleeve sensitivity. Η απόλυτη απόκριση δεν
περνά και παραμένει gate για tyre-ground stiffness/physical measurement.
Δεν μετακινήθηκε η μονάδα εμπρός ως «λύση» ακαμψίας.

### Πείραμα P2-05 — compact bearing και guard

`yoke_y 237→234 mm`, trunnion `Ø30×20→Ø24×12 mm`, fixed housing
`Ø40×12/bore24,5`. Housing outer face |Y|=240 έναντι tyre inner face 255,5:
15,5 mm nominal. Προστέθηκε replaceable sacrificial guard. Με ±2…3 mm
αβεβαιότητα απαιτούνται caliper/runout measurements· δεν είναι release.

### Πείραμα P2-06 — actuator screen και linkage

CAD baseline P16-50-256-12-S: fixed pin `[320,-190,120]`, moving local pin
`[50,-190,90]`. Pin distance 143,9→103,9 mm, stroke 40,0 mm. Effective arm
67,9…84,0 mm και στα 150 N 10,2…12,6 N·m. Transition ~8,3 s. Προστέθηκαν
locks −10/+20°, hard stops −12/+22°, independent position sensors και cable
service loop. Launch reaction bypasses actuator through the +20° pin/stop.

Σύγκριση: Makermotor PN01007-100 3 N·m και 100 rpm, άρα 2:1 για 6 N·m αλλά
με πρόσθετα feedback/locks· XH540-W270-R 9,9 N·m stall αλλά μόνο 1,98 N·m
estimated continuous, άρα απορρίφθηκε χωρίς reduction. Τιμές/διαθεσιμότητα
είναι snapshot 2026-10-11 και όχι purchase release.

### Πείραμα P2-07 — integration αισθητήρων και payload

LiDAR scan datum Z=498 mm, arch structural top περίπου Z260 και sensor
supports έως Z472. Camera `[350,0,305] mm`, pitch 50°, packaging ground view
X≈430…545. Horizontal FOV 1,204 rad από repo· vertical 54° provisional.
Battery: 5,59 kg, 198×166×170 mm upright, 12,8 V/50 Ah και BMS limits
παραμετρικά/εκκρεμή. Δεσμεύθηκαν μόνο envelopes δύο D5065 controllers και
rear electronics bay. Basket estimate παραμένει 24–28 μπάλες.

### Πείραμα P2-08 — τελικό OpenSCAD collision QA

Εντολή ανά view:

```bash
/snap/bin/openscad-nightly -D 'view="qa_sweep_frame"' \
  -o engineering/chassis-strength-bench/results/shared_flywheel_phase2/qa_sweep_frame.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

Αντίστοιχα εκτελέστηκαν `qa_sweep_drive`, `qa_sweep_basket`,
`qa_sweep_high_guide`, `qa_sweep_arch`, `qa_sweep_actuator` και
`qa_lidar_scan`, με −10…+20° ανά 2°. Όλα τα τελικά intersection views ήταν
κενά. Η πρώτη arch εκδοχή και η πρώτη τραβέρσα deck X330 απέτυχαν, οπότε οι
hangers αφαιρέθηκαν και το deck έκλεισε στο X300 με 20 mm εμπρός τραβέρσα.

Περιορισμοί: τα cable envelopes δεν αποδεικνύουν bend radius/abrasion, το
camera cone δεν είναι optical simulation, και το CAD δεν πιστοποιεί printed
strength, bearing fit ή hard-stop impact.

### Νέα κατάσταση

`PHASE_2_POINTS_1_TO_3_COMPLETE_PRE_XACRO_APPROVAL_STOP`.
Παύση πριν από isolated xacro/Gazebo και νέα ρητή έγκριση. Δεν έγινε commit,
push ή PR.

## Φάση 2 — συνέχεια μετά την έγκριση σημείων 1–3

### Πείραμα P2-09 — διόρθωση sleeve play και printable clamp

Το P2-03 αντικαθίσταται.  Για 0,30 mm/πλευρά και engagement 60 mm η τιμή
`0,573°` κρατείται μόνο ως γεωμετρική νεκρή ζώνη.  Δεν δημιουργείται
equivalent modulus ή spring.  Οι ελεύθερες μετατοπίσεις πριν την επαφή είναι
1,017 mm στην έδραση και 1,427…1,457 mm στο nip.

Στο `modular-chassis-v3.scad` σχεδιάστηκε sleeve με M4 clamp από πάνω και από
την εξωτερική πλευρά, ώστε να πατά σε δύο datum faces.  Προστέθηκαν
`sleeve_fit_coupon.stl` και `clamp_sleeve.stl`.  Περιορισμός: το CAD ελέγχει
πρόσβαση και manifold, όχι πραγματικό fit/preload· το coupon τυπώνεται πρώτο.

### Πείραμα P2-10 — κοινό baseline Φάσης 1/2 και corrected FEA

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_shared_flywheel_phase2.py \
  --output engineering/chassis-strength-bench/results/shared_flywheel_phase2/report.json
```

Το Phase-2 open-U έγινε ακριβώς ίδιο με το Phase-1 carrier, Y=237 mm, beam
segmentation, masses/loads και supports X=−65/300.  Η τιμή P2-04 vertical
2g `2,092 mm` ήταν modeling regression και αντικαθίσταται από `0,925 mm`,
ίδια με P1-19.  Διορθώθηκαν επίσης οι brace endpoints από disconnected Z52
σε πραγματικούς rail nodes Z37.

Finest 5 mm, PETG, σειρά 2g / absolute launch / local launch / wheel-lift /
racking:

- open U: `0,925 / 2,741 / 0,375 / 11,981 / 2,939 mm`·
- arch: `0,776 / 2,884 / 0,597 / 7,344 / 2,663 mm`·
- triangle: `0,870 / 2,752 / 0,359 / 11,933 / 2,929 mm`·
- combined: `0,644 / 1,940 / 0,444 / 5,760 / 1,981 mm`.

Max mesh change 10→5 mm `1,383%`.  Elastic values assume preloaded/no-play
v3 clamps.  Absolute launch still exceeds 1 mm and remains a physical
tyre-ground/frame measurement gate; local launch passes.

### Πείραμα P2-11 — lock χωρίς ενεργό πείρο

Αφαιρέθηκαν από το CAD sector/pin bodies.  Το P16-256 προφορτίζει τη μονάδα
πάνω σε replaceable seating stop στα +20°, με ξεχωριστά endpoint sensors και
emergency stops −12°/+22°.  Η launch reaction περνά stop→arch.  Backdrive
500 N × arm 67,9…84,0 mm δίνει 34…42 N·m έναντι απαίτησης 5,3 N·m, χωρίς να
θεωρείται safety brake.  Ενεργός πείρος δεν απαιτείται και συνεπώς δεν
δεσμεύτηκαν actuator/driver/pins.  Επιτρέπεται μόνο manual transport pin.

### Πείραμα P2-12 — χρόνος actuator και γρήγορη εναλλακτική

P16-50-256-12-S: 40 mm / 4,8 mm/s = `8,3 s` no-load, μεγαλύτερο υπό φορτίο.
P16-50-64-12-S: 18 mm/s, `2,22 s` no-load, 40 N peak-efficiency / 90 N max,
άρα 2,72/6,11 N·m στον ελάχιστο arm.  Δεν καλύπτει 5,3 N·m με λειτουργικό
margin χωρίς counterbalance/gas spring και νέο bracket.  Baseline μένει το
256:1.  Τιμές/διαθεσιμότητα είναι snapshot, όχι purchase instruction.

### Πείραμα P2-13 — χαμηλότερο LiDAR και mast vibration screen

Το scan plane κατέβηκε παραμετρικά από Z498 σε Z390 mm.  Το free post length
μειώνεται 217→109 mm.  Για δύο PETG 25×25×3,2 mm posts, E=1,5 GPa και top mass
0,25…0,45 kg, simple cantilever first mode `126…94 Hz`, αντί `45…34 Hz` στο
παλιό ύψος.  Το παλιό range περιλάμβανε τα 47,8 Hz των 2865 rpm.

Εντολή QA:

```bash
/snap/bin/openscad-nightly -D 'view="qa_lidar_scan"' \
  -o engineering/chassis-strength-bench/results/shared_flywheel_phase2/qa_lidar_scan.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

Αποτέλεσμα: empty intersection με basket, high guide, endpoints flywheel,
arch και camera.  Περιορισμός: analytical mode screen μόνο· πραγματικά print
joints, damping, accelerometer και scan quality δεν μετρήθηκαν.

### Πείραμα P2-14 — αριθμητικό guide/deck clearance

View `qa_guide_low_frame_at_angle`: Minkowski expansion του πραγματικού v3
low deck έναντι του moving throat/guide.  Coarse sweep −10…+20° ανά 2° έδωσε
πρώτη τομή 4 mm κοντά στις −4°/−2°.  Fine sweep −6…0° ανά 0,25°, probe ανά
0,125 mm, έδωσε minimum στο −3,0°: empty στα 2,750 mm και non-empty στα
2,875 mm.

Συμπέρασμα: clearance bracket `2,75…2,88 mm`.  Δεν υπάρχει hard collision,
αλλά δεν είναι release margin για ανοχές/flex/χώμα.  Guide STL αναβάλλεται
μέχρι τοπική ανύψωση/εσοχή και στόχο ≥8 mm.

### Πείραμα P2-15 — export και έλεγχος print set v3

Εντολές:

```bash
cd cad/modular-test-chassis
./export-v3-stls.sh
python3 ../../engineering/chassis-strength-bench/measure_v3_print_set.py
```

Παρήχθησαν 18 μοναδικά STL στο ξεχωριστό `stl-v3`, χωρίς αντικατάσταση του
`stl-v2`.  Όλα έχουν `nonmanifold_edge_count=0`.  Με 0,72 g/cm³: locked frame
χωρίς coupon/temporary tabs `1398,3 g`, σύνολο με coupon και 4 tabs
`1482,5 g`.  Το grid είναι M4/25 mm μόνο σε επιτρεπτές πάνω/εξωτερικές
επιφάνειες.  Οι διαγώνιες συνδέονται σε πραγματικές M4 θέσεις Y=±150 mm και
με κεντρικούς joiners, όχι σε sleeve zones.  Conservative local top-hole ΔI
≤5%, χωρίς credit στα nut towers.

Το υπάρχον `front_motor_gamma_left` είναι μόνο προσωρινό drive-test end και
χρειάζεται το πραγματικό mirrored `front_motor_gamma_right`; δύο left δεν
δίνουν σωστή πρόσβαση/handed motor land.  Δεν χρησιμοποιούνται με flywheel.

### Πείραμα P2-16 — Redodo packaging, loads και Mega pin audit

Battery envelope: 195×166×172 mm, 5,29 kg official listing, 12,8 V/50 Ah,
BMS 50 A continuous charge/discharge.  D5065 controller lateral clearance
33 mm.  Το rear bay μετακινήθηκε στο X−185 και έχει 17,5 mm clearance.  Τα
δύο intersection views είναι empty.

Για battery 5,29/3,5/2,5 kg, empty total mass 21,496/19,706/18,706 kg και CG
X=138,0/155,5/166,8 mm.  Front wheel load μένει 58,61/58,37/58,24 N ανά
τροχό, ενώ rear πέφτει 46,79/38,25/33,48 N.

Read-only firmware audit: Serial2 D16/17 και Serial3 D14/15 είναι διαθέσιμα,
Serial1 D18/19 χρησιμοποιείται από encoders.  D11 και D44–D46 δεν είναι
ελεύθερα σήμερα: ανήκουν στα δύο intake BTS channels.  Προτάθηκαν D26/D27
controller enables, D28/D29 faults, D4/D7/D8 P16 H-bridge και D12/D13 endpoint
sensors.  Δεν τροποποιήθηκε Arduino code.

### Νέα κατάσταση

`PHASE_2_V3_FRAME_PRINT_SET_READY_COUPON_FIRST_PRE_XACRO_STOP`.
Δεν εκτελέστηκε ROS/Gazebo, δεν δημιουργήθηκε xacro και δεν άλλαξε robot
stack, Arduino, ηλεκτρονικά ή καλωδίωση.  Παύση για έγκριση.  Δεν έγινε
commit, push ή PR.

### Πείραμα P2-17 — Parkside/Ugreen power baseline και axle-load screen

Το P2-16 Redodo baseline αντικαθίσταται ως βασική επιλογή, αλλά διατηρείται
ως παραμετρική σύγκριση.  Νέο baseline: δύο Parkside X20V Team 4 Ah
`0,65 kg/pack` σε X=−165 mm και Ugreen 12000 mAh/100 W `0,309 kg` σε
X=−100 mm.  Τα Parkside envelope dimensions 135×85×75 mm είναι προσωρινή
υπόθεση και όχι μετρημένα δεδομένα.

Εντολή:

```bash
python3 engineering/chassis-strength-bench/run_phase2_power_layout.py
```

Worst-case module 4,706 kg και basket/support 1,50 kg:

- Parkside + Ugreen empty: `17,815 kg`, CG X=`169,0 mm`, front axle
  `64,12%`, rear/front `31,34/56,01 N` ανά τροχό·
- Parkside + Ugreen +28 balls: `19,411 kg`, CG X=`168,7 mm`, front
  `64,03%`, rear/front `34,23/60,94 N`·
- Redodo empty: `21,496 kg`, CG X=`138,0 mm`, front `55,61%`, rear/front
  `46,79/58,61 N`·
- Redodo +28 balls: `23,092 kg`, CG X=`139,8 mm`, front `56,12%`, rear/front
  `49,68/63,55 N`.

Parkside rear/front friction force ανά τροχό για μ=0,3/0,6/0,9:
`9,40/16,80`, `18,80/33,61`, `28,21/50,41 N`.  Το simple Coulomb
available-yaw/scrub ratio είναι 1,77 έναντι 1,65 Redodo, αλλά ο ελαφρύτερος
πίσω άξονας ολισθαίνει πρώτος.  Περιορισμός: quasi-static comparison χωρίς
tyre compliance, surface anisotropy ή dynamic transfer· απαιτείται φυσικό
pivot-turn test με encoder/current logging.

OpenSCAD QA εκτελέστηκε για `power_variant="parkside_pair"` και `"redodo"`
σε `qa_battery_controllers`, `qa_battery_rear_electronics` και
`qa_battery_frame`.  Με καθαρισμό παλιών output files, και οι έξι τομές είναι
empty.  Οι D5065 controller envelopes μετακινήθηκαν στο X=40 mm και το rear
bay έμεινε X=−185 mm/Y±60 mm.

Ηλεκτρικές απαιτήσεις: fuse + ideal diode ανά pack πριν XT60 common bus,
unknown continuous/peak current ανά pack ως παράμετρος, 12 V-equivalent
FIT0403 PWM ceilings `80,0/66,7/57,1%` στα `15/18/21 V`, current limit/ramp,
12 V buck για P16, LVC με hysteresis και master disconnect.  Ugreen/USB-PD
μένει χωριστός κλάδος Pi/sensors/Mega· Mega ποτέ σε 15…21 V.  Δεν έγινε
αλλαγή firmware ή καλωδίωσης.

### Ενημερωμένη κατάσταση

`PHASE_2_V3_FRAME_READY_PARKSIDE_POWER_LAYOUT_PARAMETRIC_PRE_XACRO_STOP`.
Τα quick-change battery docks είναι DEPENDENT μέχρι πραγματικές διαστάσεις,
adapter/release sweep και μέτρηση ρεύματος.  Δεν εκτελέστηκε ROS/Gazebo και
δεν έγινε commit, push ή PR.

### Πείραμα P2-18 — έρμα χωρίς Gamma και διόρθωση export X300

Το P2-15 προσωρινό Gamma ballast αντικαθίσταται.  Η ισοδύναμη μάζα στο
X=360 mm υπολογίστηκε από ίση ροπή ως προς τον πίσω άξονα X=−65 mm:
`4,706·492/425 = 5,448 kg`, μαζί με πλάκα/βίδες.  Με basket/support 1,70 kg
και P16 0,095 kg στο X320, το module case και το ballast case δίνουν ακριβώς
το ίδιο εμπρός axle load `112,188 N`.  Το ballast case είναι 0,742 kg
βαρύτερο, CG X=156,5 αντί 165,6 mm και σημειακό inertia term ως προς τον πίσω
άξονα 13,6% μικρότερο.

Σχεδιάστηκε μη εκτυπωμένη πλάκα 160×480×12 mm στο X=360: οκτώ M4 στο
X300/Y=±75/100/125/150 με 10 mm rigid spacers, τέσσερα M6 στα
X310/330,Y±185 για δύο inboard C-hook terminal clamps και τέσσερα M8
tie-downs γύρω από X360.  Δεν τρυπιούνται τα front terminals, δεν
χρησιμοποιούνται motor bolts και το clamp δεν μπαίνει στο tyre gap.  Αν λείπουν οι πηγές,
μπαίνουν dummy 0,65 kg ανά Parkside στο X−165/Y±110 και 0,309 kg UGREEN στο
X−100/Y0.

Κατά τον έλεγχο βρέθηκε ότι το assembly χρησιμοποιούσε M4 grid στο X300 αλλά
ο selector `part="x300_crossbar_half"` καλούσε `grid=false`.  Διορθώθηκε σε
`true` και επανεξήχθη όλο το v3 set:

```bash
cd cad/modular-test-chassis
./export-v3-stls.sh
python3 ../../engineering/chassis-strength-bench/measure_v3_print_set.py
```

Αποτέλεσμα: 18/18 manifold, X300 half 38,146 g, locked frame 1406,931 g,
σύνολο με coupon/tabs 1491,116 g.  Περιορισμός: η πλάκα/saddle είναι σχέδιο
fabrication και όχι printed strength release· πρέπει να ζυγιστεί και να
μετρηθεί το πραγματικό X-CG.  Το Gamma pair δεν απαιτείται πλέον.

### Πείραμα P2-19 — καλάθι 45 μπαλών και packaging QA

Παράμετροι CAD: πίσω τοίχωμα X=−15 mm, εμπρός X=278, πλάτος 350, δάπεδο
Z=42→54 mm, άνω Z=215 και upper-front X=205.  Οι D5065 controller envelopes
μετακινήθηκαν σε `[80,±145,245] mm`.  Το πίσω τοίχωμα απέχει 32 mm από το
FIT0403 envelope X=−47, άρα δεν χρειάστηκε narrow/raised tail.

Εντολή υπολογισμού:

```bash
python3 engineering/chassis-strength-bench/run_phase2_ballast_basket.py
```

Αποτελέσματα: gross εσωτερικό 14,47 L, assumed keepout 1,30 L, usable
13,17 L.  Capacity για packing 0,55/0,58/0,60/0,64:
48,1/50,7/52,5/56,0 μπάλες.  Με 45 μπάλες 2,565 kg, provisional CG X105 και
P16 0,095 kg, σύνολο 20,675 kg, CG X158,1, front fraction 61,11%, rear/front
load ανά τροχό 39,42/61,95 N.

Uniform-fill screen της σημερινής εξόδου: lower lip Z94 ≈14,8 μπάλες,
centre Z132 ≈28,0 και upper lip Z168 ≈39,7.  Άρα το volume περνά τον στόχο,
αλλά η είσοδος δεν μπορεί να θεωρηθεί λειτουργική στις 45 χωρίς higher
over-pile chute ή agitator/anti-bridge test.  Το feed screen με 10-ball local
column δίνει 5,59 N, 0,129 N·m gravity torque και ≥0,39 N·m με factor 3,
μαζί με ≥15 N tangential design force.

CAD-only εντολές:

```bash
/snap/bin/openscad-nightly -D 'view="qa_basket_drive"' \
  -o engineering/chassis-strength-bench/results/shared_flywheel_phase2/qa_basket_drive.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
/snap/bin/openscad-nightly -D 'view="qa_basket_controllers"' \
  -o engineering/chassis-strength-bench/results/shared_flywheel_phase2/qa_basket_controllers.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
/snap/bin/openscad-nightly -D 'view="qa_sweep_basket"' \
  -o engineering/chassis-strength-bench/results/shared_flywheel_phase2/qa_sweep_basket_45.stl \
  cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad
```

Και οι τρεις top-level τομές ήταν empty.  Το OpenSCAD επιστρέφει exit 1 όταν
δεν υπάρχει STL για empty object· το κείμενο `Current top level object is
empty` είναι το αναμενόμενο PASS.  Περιορισμοί: basket/support mass 1,70 kg,
ball CG και keepout είναι εκτιμήσεις.  Δεν έγινε discrete-ball simulation ή
φυσική δοκιμή.  Basket/guide παραμένουν ΕΞΑΡΤΩΜΕΝΑ.

### Πείραμα P2-20 — isolated xacro προετοιμασμένο, όχι εκτελεσμένο

Δημιουργήθηκε ξεχωριστός φάκελος
`engineering/shared-flywheel-isolated-bench/` με xacro, controller YAML,
frozen input/hash contract JSON και README.  Περιλαμβάνει track 0,594 m,
axle Y±0,297 m, Parkside/Ugreen,
empty/45-ball payload, P16 effort joint και −10°/+20° stops, 4,7055 kg module
και 5,4479 kg ballast alternative.  Το module χρησιμοποιεί Phase-1 wheel
mass/inertia και τις αμετάβλητες D5065 παραμέτρους 270 KV, 0,031 N·m/A,
0,039 Ω, 20 A/0,62 N·m.  Η μπάλα παραμένει το υπάρχον
`tennis_ball_compliant/model.sdf` χωρίς νέα προσαρμογή.

Εκτελέστηκε μόνο parser check, όχι xacro:

```bash
python3 -c "import xml.etree.ElementTree as ET; ET.parse('engineering/shared-flywheel-isolated-bench/v3_chassis_flywheel_bench.urdf.xacro')"
python3 -c "import yaml; yaml.safe_load(open('engineering/shared-flywheel-isolated-bench/v3_controllers.yaml'))"
```

Αποτέλεσμα: XML well-formed και YAML parsed.  **Δεν** εκτελέστηκε xacro,
ROS build/test/launch ή Gazebo.  Η semantic xacro/plugin validation μένει
για το Ubuntu 24.04/Jazzy container μετά από νέα ρητή άδεια.

### Ενημερωμένη κατάσταση

`PHASE_2_V3_BALLAST_AND_45_BALL_BASKET_PREPARED_XACRO_FILES_READY_EXECUTION_STOP`.
Παύση πριν από κάθε xacro/ROS/Gazebo execution.  Με μελλοντική έγκριση:
container validation και μόνο Gazebo 5a, έπειτα δεύτερη παύση πριν 5b–5d.
Δεν έγινε commit, push ή PR και δεν άλλαξε production robot stack, Arduino,
ηλεκτρονικά ή καλωδίωση.

## Φάση 2 — εγκεκριμένη xacro validation και Gazebo 5a

### Πείραμα P2-21 — κριτήρια 5a παγωμένα πριν από το πρώτο run

Τα κριτήρια γράφτηκαν πριν ξεκινήσει Gazebo και αποθηκεύτηκαν επίσης στο
`engineering/shared-flywheel-isolated-bench/criteria_5a.json`:

- straight 0,3/0,6 m/s: steady speed error ≤10%, cross-track ≤50 mm,
  heading error ≤3°, mean/peak longitudinal slip ≤0,20/0,50,
- spin 0,5/1,0 rad/s: steady yaw-rate error ≤15%, centre drift ≤100 mm·
  rear free-spin σημαίνει rear wheel >115% target ενώ yaw <85% target,
- moving turn 0,4 m/s + 0,6 rad/s: steady v και ω error ≤15%,
- brake 0,6→<0,05 m/s: ≤1,5 s, ≤0,45 m και heading change ≤5°,
- stability σε όλα: |roll|/|pitch| ≤3° και base rise από settled ≤5 mm,
- κάθε FIT0403 command ≤3,73 N·m και steady saturation fraction ≤10%
  (`|τ|≥0,99·3,73`),
- wheel-joint/tilt-joint transmitted wrenches και inertial force/moment
  εξάγονται ως FEA inputs.  Δεν αποτελούν Gazebo strength proof.

Η σάρωση είναι υποχρεωτικά μ=0,5/0,7/0,9 για empty/full45/ballast και όλα τα
αποτελέσματα αναφέρονται χωρίς επιλογή ευνοϊκής τριβής.  Το μ είναι ισότροπη
Coulomb sensitivity, όχι βαθμονομημένο χώμα.  Ο απομονωμένος controller είναι
speed servo με πραγματική εφαρμογή torque capped στα 3,73 N·m, όχι μοντέλο
των production electronics.

### Πείραμα P2-22 — xacro/check_urdf validation στο Jazzy container

Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Εντολή:

```bash
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
  "timeout 120 engineering/shared-flywheel-isolated-bench/validate_v3_xacro.sh"
```

Και οι τρεις περιπτώσεις πέρασαν `check_urdf`, positive-definite inertia,
contract mass/geometry, collisions, joint limits και `ros2_control` interface
checks.  Τελικές μάζες: empty 18,11553 kg, full45 20,68053 kg, ballast
18,85788 kg.  SHA-256 των expanded URDF:

- empty: `985f6b4fe153df3eba72538d99510c5aaddbf2f1ee4505dbfb9f0eb351e382a9`,
- full45: `011cdc46290686f46b2d1e7f1d8e41b5dee840f94ecc4984d700b41ed0406d9c`,
- ballast: `acc9d89bce5a0142c54abe5dffe85c907fba1b06a223bd1fdc2ddde218f517b2`.

Διόρθωση πριν από την τελική validation: η translational wheel mass μένει
μέσα στο 10 kg base datum, αλλά προστέθηκε 0,0025 kg·m² περιστροφική αδράνεια
ανά τροχό ώστε τα torque/slip transients να μην είναι τεχνητά στιγμιαία.
Μετά το run, `docker ps` κενό.

### Πείραμα P2-23 — μη έγκυρο πρώτο load attempt, αντικαταστημένο

Το πρώτο `empty, μ=0,7` attempt σταμάτησε πριν φορτωθεί world: τα
`model://tennis_robot/meshes/...` δεν επιλύονταν.  Δεν παρήχθησαν physics
results.  Προστέθηκε στο isolated runner μόνο
`GZ_SIM_RESOURCE_PATH=$(pwd)/ros2_ws/src`; το production stack δεν άλλαξε.
Το attempt είναι **ΑΝΤΙΚΑΤΑΣΤΗΜΕΝΟ** από το P2-24.  `docker ps` κενό.

### Πείραμα P2-24 — πλήρης Gazebo 5a καμπάνια

Build/run pattern:

```bash
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
  "timeout 180 cmake --build engineering/shared-flywheel-isolated-bench/build_5a -j2"
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
  "timeout 180 engineering/shared-flywheel-isolated-bench/run_5a_case.sh CASE MU"
```

Εκτελέστηκαν τελικά 9 runs: `CASE=empty/full45/ballast` ×
`MU=0.5/0.7/0.9`, headless, DART, ROS_DOMAIN_ID=88, 40 simulated seconds και
εξωτερικό timeout.  Μετά από **κάθε** run το `docker ps` ήταν κενό.
Προγενέστερα diagnostic runs όπου τα δύο env vars δεν είχαν περαστεί ρητά
στο wrapper χαρακτηρίστηκαν **ΑΝΤΙΚΑΤΑΣΤΗΜΕΝΑ** και τα artifacts τους
αντικαταστάθηκαν από την πλήρη env-compliant 9άδα που αναφέρεται εδώ.

Αποτέλεσμα: 27/54 scenario rows PASS.  Όλες οι ευθείες και όλα τα braking
cases PASS· όλα τα spin και moving-turn cases FAIL λόγω yaw tracking.  Τα
spin yaw errors είναι 36,7–99,9%, τα moving-turn yaw errors 40,2–88,4%.
Μέγιστο motor command 3,73 N·m μόνο ως braking transient, χωρίς αποτυχία του
steady saturation criterion.  Δεν μετρήθηκε wheel lift: max wheel-centre rise
0,1 μm.  Η αρχική χρήση joint Fz ως lift detector απορρίφθηκε επειδή είχε
solver dropouts και αντικαταστάθηκε από άμεση wheel-centre pose measurement.

FEA exports: wheel joint έως 94,66 N / 5,90 N·m, tilt joint έως
59,24 N / 5,97 N·m, p99 inertial force/yaw moment 39,74 N / 1,17 N·m.
Τα raw one-step maxima 148,91 N / 20,00 N·m κρατούνται ως sensitivity, όχι
nominal load.  Το rigid Gazebo δεν αποτελεί strength proof.

Artifacts:

- `engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a/scenario_results.csv`,
- `engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a/campaign_summary.json`,
- ανά run: generated URDF/SDF/world, telemetry CSV, Gazebo log και summary.

### Πείραμα P2-25 — gyro και ερμηνεία στροφής

Ο χρήστης επιβεβαίωσε ότι υπάρχει gyro.  Η P2-24 παραμένει σκόπιμα
wheel-speed closed loop / yaw open loop, επειδή τα κριτήρια και ο controller
είχαν παγώσει πριν από το πρώτο run.  Το gyro πρέπει να κλείσει εξωτερικό
yaw-rate loop στη φυσική δοκιμή και σε μελλοντικό ελεγχόμενο variant.  Μπορεί
να διορθώσει tracking και ασυμμετρίες, όχι να εξαφανίσει scrub/traction loads.
Δεν άλλαξε Arduino ή production controller code.

### Πείραμα P2-26 — παύση μετά το 5a

Κατάσταση:
`PHASE_2_GAZEBO_5A_COMPLETE_PAUSED_BEFORE_5B_5C_5D`.
Δεν εκτελέστηκε actuator transition, collection, multi-ball/jam ή launch.
Δεν έγινε commit, push ή PR.  Απαιτείται νέα ρητή έγκριση πριν από κάθε
εκτέλεση 5b–5d.

### Πείραμα P2-27 — παγωμένα κριτήρια διαγνωστικής επανάληψης στροφών

Η ανασκόπηση δέχεται τις ευθείες και το φρενάρισμα του 5a, αλλά **όχι** ακόμη
τις στροφές ως φυσικό συμπέρασμα. Πριν από οποιοδήποτε νέο Gazebo run
παγώθηκε το
`engineering/shared-flywheel-isolated-bench/criteria_turn_diagnostic.json`.
Η αρχική καμπάνια/open-yaw και τα αποτελέσματά της δεν τροποποιούνται.

- Μόνο `spin_0p5`, `spin_1p0`, `moving_turn`, για empty/full45/ballast και
  μ=0,5/0,7/0,9.
- Νέοι τροχοί 0,650 kg έκαστος, R=85 mm, πλάτος 83 mm. Από τη βάση αφαιρούνται
  2,596 kg: βάση 7,404 kg, CG=(0,093864; 0; 0,132272) m και tensor
  Ixx/Iyy/Izz/Ixz=(0,386525/0,358776/0,543284/−0,052148) kg·m². Η σύνθεση
  βάση+4 τροχοί διατηρεί ακριβώς την παλιά μάζα 10,004 kg, το CG και τη
  συνολική αδράνεια· άρα αλλάζει μόνο η κατανομή σε δυναμικά links.
- Παραλλαγή Α: αμετάβλητο open-yaw πάνω από τους PI wheel-speed loops.
- Παραλλαγή Β: gyro/body-yaw-rate PI εξωτερικός βρόχος, Kp=2,0, Ki=0,8,
  integral clamp ±1,5 και yaw correction clamp ±1,5 rad/s. Το gyro είναι η
  body angular velocity του isolated model, όχι αλλαγή στον robot stack.
- Κριτήρια: yaw error ≤15%, moving linear error ≤15%, spin drift ≤0,10 m,
  |roll|/|pitch|≤3°, wheel rise≤5 mm, torque≤3,73 N·m και steady saturation
  ≤10%. Όλα τα μ αναφέρονται, χωρίς επιλογή «βολικής» τιμής.
- Για κάθε run καταγράφονται επίσης Coulomb available/required yaw moment,
  τοπικό όριο μ·Nwheel·R, slip, ροπές/δυνάμεις αρθρώσεων και θέση άρθρωσης.

Προγραμματισμένη εντολή (μόνο στο εγκεκριμένο Jazzy container):
`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 120 engineering/shared-flywheel-isolated-bench/run_turn_diagnostic_case.sh <case> <mu> <open_yaw|gyro_yaw>"`.
Μετά από κάθε run απαιτείται κενό `docker ps`.

### Πείραμα P2-28 — ενημέρωση πηγής D5065 και θερμικό όριο

Πηγή: ODrive Motor Guide, tabs `Motor Guide`, `Torque derating`, `Motor
Inertia`. Η γραμμή D5065-270KV δίνει 65 A max, 32 V max και 420 g. Το φύλλο
derating ορίζει συνεχή ροπή 30% του maximum χωρίς αέρα και 50% με καλή ροή.
Το `Red 5065` inertia sheet υπολογίζει 9,259·10⁻⁵ kg·m² από γεωμετρικό
μοντέλο· **δεν είναι μέτρηση** του συγκεκριμένου μοτέρ.

Διαφορά από το προηγούμενο repo: 65 A αντί 85 A/3 s και 420 g αντί 490 g.
Μέχρι ζύγιση/μέτρηση κρατούνται οι συντηρητικές τιμές 65 A και 490 g. Το
20 A της υπάρχουσας προσομοίωσης παραμένει χωριστό provisional όριο
controller/τροφοδοσίας. Οι Parkside 15–21 V είναι κάτω από τα 32 V, χωρίς αυτό
να επιβεβαιώνει ρεύμα pack/controller. Στο CAD προστέθηκε μη στερεό keep-out
ροής: 8 mm ακτινικά και 15 mm αξονικά ανά D5065. Το 50% continuous δεν
πιστώνεται χωρίς μετρημένη ροή και θερμοκρασία· κλειστό κάλυμμα απαγορεύεται.

### Πείραμα P2-29 — μη έγκυρο πρώτο turn attempt, αντικαταστημένο

Το πρώτο `empty, μ=0,7, open_yaw` αποκάλυψε ότι το ros2_control
`initial_value` δεν εφαρμόζεται όταν το isolated custom plugin τρέχει με
`sim_mode=false`: η άρθρωση ισορρόπησε στο +5,84° αντί −10°. Άρα τα φορτία
του attempt δεν αντιστοιχούν στη ζητούμενη θέση. Τα artifacts κρατήθηκαν σε
φάκελο με κατάληξη `invalid_free_tilt` και δεν θα μπουν στη σύνοψη.

Διόρθωση μόνο στο isolated bench: αρχική `ResetPosition(−10°)` και
PD συγκράτηση θέσης με Kp=80 N·m/rad, Kd=8 N·m·s/rad, όριο ±12,6 N·m. Η
εντολή actuator καταγράφεται χωριστά. Αυτό προσομοιώνει τη δυσμενή θέση
συλλογής χωρίς μηχανικό lock/preloaded upper terminal και επιτρέπει να
ελεγχθεί αν το P16 αναγκάζεται να παραλάβει το φορτίο. Δεν αλλάζει production
controller ή Arduino.

### Πείραμα P2-30 — διορθωμένη 18άδα turn diagnostic και παύση

Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Corrected xacro hashes: empty `7912f7d4…399ea`, full45
`77714792…b8199`, ballast `7c60fca0…a8a5`. Και οι τρεις περιπτώσεις πέρασαν
`check_urdf`, contract mass/CG/inertia, positive-definite inertia,
collisions, limits και ros2_control checks.

Εκτελέστηκαν 18 headless DART runs: δύο variants × empty/full45/ballast ×
μ=0,5/0,7/0,9, μόνο spin 0,5/1,0 και moving turn. Μετά από κάθε run και στο
τέλος το `docker ps` ήταν κενό. Δεν βρέθηκαν joint/plugin load errors.

- open-yaw: 0/27 scenarios PASS, yaw error 35,6–88,3%.
- gyro-yaw: 9/27 PASS — όλα τα moving turns. Yaw error 3,4–14,1%, αλλά τα
  spins αποτυγχάνουν επειδή η μετάθεση κέντρου είναι 0,464–0,566 m.
- Coulomb available/required=1,646–1,748 PASS σε όλα. Maximum motor command
  3,73 N·m, μέγιστο steady saturation fraction 8,75%.
- Διορθωμένο hinge maximum 2,858 N·m και actuator hold maximum 0,541 N·m
  στη θέση περίπου −9,62°. Το προηγούμενο 5,97 N·m αντικαθίσταται για τη
  συγκεκριμένη turn case· δεν αυξάνει την απαίτηση 5,3 N·m.
- Στατικό non-wheel clearance 3,601 mm στο −10° και conservative dynamic
  lower bound 3,57 mm. Μόνο οι τέσσερις drive wheels φτάνουν το ground
  plane στο μοντέλο.

Απαίτηση: actuator ≥5,3 N·m και άρθρωση/άνω+κάτω terminal design load
8,0 N·m (1,5×). Στο −10° προστίθεται κάτω seat/terminal ή lock ώστε το P16
να μη δέχεται κρουστικό φορτίο. Η πλήρης ανάλυση είναι στο
`shared-flywheel-phase2-turn-diagnostic-el.md` και τα 54 rows στο
`gazebo_5a_turn_diagnostic/scenario_results.csv`.

Κατάσταση: `TURN_DIAGNOSTIC_COMPLETE_PAUSED_BEFORE_5B_5C_5D`. Δεν έγινε
commit/push/PR και δεν άλλαξε robot stack, Arduino ή ηλεκτρονικά.

### Πείραμα P2-31 — CAD-aligned isolated launcher, σχέδιο ελέγχου

Η απόσταση 3,601 mm εντοπίστηκε στο legacy lower-panel mesh του
`authoritative_flywheel_launcher_module`, όχι στο εγκεκριμένο CAD. Δημιουργήθηκε
χωριστό isolated macro `cad_v3_flywheel_launcher_module`: άνω πλάκα
185×314×8 mm στο W=43 mm, δύο yokes, Ø24 trunnions, δύο Ø200×50 flywheels
και D5065 envelopes, **χωρίς κάτω πλάκα**. Το παλιό authoritative xacro δεν
τροποποιείται.

Mass policy πριν από validation: διατηρούνται ακριβώς το προηγούμενο fixed
mass 2,85200336 kg, fixed CG και tensor, καθώς και οι δύο rotating masses.
Η allowance της παλιάς κάτω πλάκας θεωρείται ανακατανεμημένη στο άνω
plate/yoke lump· δεν αποτελεί component-level mass claim. Άρα total module
mass/CG/inertia δεν αλλάζουν και δεν θα επαναληφθούν dynamics runs, εκτός αν
η semantic validation αποδείξει απόκλιση.

Οι μη μετρημένες παράμετροι γίνονται ρητές: drive-wheel radius R=80…85 mm
και axle height πάνω από mounting-foot h=28…38 mm. Nominal R=85, h=33 mm.
Το local axle datum είναι `Zaxle=52 mm+h` και, μετά το settling στο έδαφος,
η κάτω πλευρά της δοκού είναι `R−h−30 mm` (nominal 22 mm). Θα γίνει sweep
launcher −10…+20° και sensitivity R/h χωρίς Gazebo.

Ο πρώτος static sweep αποκάλυψε επιπλέον ότι το legacy bench είχε axis `+Y`,
ενώ το CAD χρησιμοποιεί `rotate([0,-theta,0])`: το URDF +20° κατέβαζε το
στόμιο. Ο isolated axis διορθώθηκε σε `−Y`. Επειδή αυτό αλλάζει orientation
και joint loads, ενεργοποιείται η ρητή συνθήκη επανάληψης των turn runs με
τα ίδια, ήδη παγωμένα P2-27 κριτήρια. Ο λανθασμένος πρώτος sweep
αντικαθίσταται και δεν χρησιμοποιείται ως αποτέλεσμα.

### Πείραμα P2-32 — frozen rerun gate μετά τη διόρθωση axis

Τα κριτήρια `criteria_turn_diagnostic.json`, τα gyro gains, οι μάζες, το
CG/tensor, τα μ=0,5/0,7/0,9 και τα τρία load cases παραμένουν αμετάβλητα.
Η προηγούμενη 18άδα διατηρείται χωριστά ως
`gazebo_5a_turn_diagnostic_superseded_plus_y_axis` και χαρακτηρίζεται
**ΑΝΤΙΚΑΤΑΣΤΗΜΕΝΗ**, επειδή joint coordinate −10° αντιστοιχούσε σε CAD +10°.
Η νέα 18άδα είναι αναγκαία μόνο επειδή η διόρθωση `axis=0 −1 0` αλλάζει τον
φυσικό προσανατολισμό και τα joint loads. Το CAD-aligned macro διατηρεί
ακριβώς mass/CG/inertia και από μόνο του δεν θα απαιτούσε rerun.

### Πείραμα P2-33 — μερική τελική CSV γραμμή, αντικαταστημένο attempt

Το πρώτο νέο `open_yaw/ballast/μ=0,5` ολοκλήρωσε 24 s physics, αλλά ο runner
έκλεισε τον server αμέσως μετά το `.done` ενώ είχε ξεκινήσει το επόμενο
PostUpdate sample. Προέκυψε μία μερική τελευταία γραμμή 13/83 πεδίων και ο
analyzer σταμάτησε όπως έπρεπε. Το attempt κρατείται ως
`invalid_partial_csv` και δεν χρησιμοποιείται. Στο isolated logger προστέθηκε
early return όταν `done=true`, ώστε να μην ξεκινά νέο CSV row μετά το marker.
Δεν αλλάζει controller, physics ή telemetry πριν από το τέλος του run.

### Πείραμα P2-34 — CAD-correct clearance, ICR reanalysis και παύση

Η semantic validation πέρασε και στα τρία cases με αμετάβλητες
mass/CG/inertia. Νέα URDF hashes: empty `3fbe1a4a…8234b`, full45
`f2bf99bf…2d88`, ballast `5b91b604…5b6cf`. Ο sweep −10…+20°/2° έδωσε:

- Ø200 flywheel minimum 14,501 mm στο −10° (CAD Φάσης 1: 14,50 mm),
- δοκός και minimum άλλο-εκτός-τροχών 22 mm raw,
- 17 mm μετά από allowance 5 mm για dynamic pitch/ανωμαλία: PASS ≥12 mm.

Sensitivity R=80…85 mm και h=28…38 mm: rail/other raw 12…27 mm και
7…22 mm μετά allowance. Άρα το full tolerance box δεν περνά· αναμένονται
μετρήσεις loaded radius και axle-to-foot height. Flywheel raw minimum
4,501…19,501 mm στα corners.

Η CAD-correct 18άδα ολοκληρώθηκε στο ίδιο image/container. Μετά από κάθε run
και στο τέλος `docker ps` κενό· 18 summaries/hashes και μηδέν plugin/joint
load errors. Frozen αποτέλεσμα: open-yaw 0/27 PASS, gyro-yaw 9/27 PASS,
όπως πριν. Μέγιστο hinge 2,993 N·m. Actuator command≈0 στη −10° θέση επειδή
το lower limit/seat παραλαμβάνει το φορτίο· το πραγματικό κάτω terminal και
το design load 8,0 N·m παραμένουν υποχρεωτικά.

Το frozen `centre_translation_m` μετρά το base_link origin. Στα gyro spins:
expected load-centre ICR X=0,1565…0,1655 m (front load 60,68…63,16%), ενώ το
median instantaneous ICR είναι X=0,267…0,274 m, Y=−0,017…−0,006 m. Το
base-origin drift 0,465…0,566 m αναλύεται κυρίως σε geometric chord
0,450…0,544 m γύρω από το actual ICR. Το actual ICR world maximum spread
είναι 0,050…0,090 m. Άρα το παγωμένο base-origin criterion μένει FAIL, αλλά
το νέο πραγματικό-slip μέτρο ICR περνά αριθμητικά τα 0,10 m χωρίς αλλαγή
κριτηρίου.

Πλήρης αναφορά:
`shared-flywheel-phase2-clearance-icr-review-el.md`. Κατάσταση:
`CAD_CLEARANCE_AND_ICR_REVIEW_COMPLETE_PAUSED_BEFORE_5B`. Δεν εκτελέστηκε
5b–5d, ούτε commit/push/PR, ούτε αλλαγή robot stack/Arduino/ηλεκτρονικών.

### Πείραμα P2-35 — νέα μέτρηση axle-to-foot και radius uncertainty

Η παλιά εκτίμηση h=21…25 mm αποσύρεται ρητά ως λανθασμένη. Νέα μέτρηση με
μεζούρα από την πάνω επιφάνεια της δοκού: άξονας περίπου 28…30 mm πάνω από
το πέλμα, uncertainty ±3 mm. Για υπολογισμούς τίθεται h=29 mm και sweep
26…33 mm. Το isolated local axle datum αλλάζει από Z=85 σε Z=81 mm
(`52+h`).

Η loaded radius δεν έχει μετρηθεί. Η λίστα αγορών αναφέρει Baja 5B 170×80,
ενώ το `CLAUDE.md` αναφέρει 180 mm· καμία τιμή δεν θεωρείται ταυτότητα ή
loaded diameter του εγκατεστημένου τροχού. Νέο sweep R=82…90 mm. Το R=85
mm κρατείται μόνο ως reference σύγκρισης, όχι nominal/released τιμή.

Το clearance model κανονικοποιείται πλέον στο settled ground plane από το
χαμηλότερο drive-tyre point. Η σχέση παραμένει
`rail_clearance=R−h−30 mm`. Θα ενημερωθεί ο static sweep χωρίς Gazebo. Η
αλλαγή axle Z μεταβάλλει ελαφρά mass distribution/loads, αλλά νέα turn runs
δεν επιτρέπονται χωρίς νέα έγκριση και είναι προτιμότερο να περιμένουν την
loaded-radius μέτρηση ώστε να μη γίνει τρίτη προσωρινή καμπάνια.

### Πείραμα P2-36 — επικύρωση h=29 και νέο clearance box

Εκτελέστηκε μόνο xacro expansion/`check_urdf`/semantic validation μέσα στο
εγκεκριμένο Jazzy container, όχι Gazebo:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 120 engineering/shared-flywheel-isolated-bench/validate_turn_diagnostic.sh"`

Και τα τρία cases πέρασαν. Νέα total mass/CG-Z:

- empty: 18,11553065 kg / 0,10867238 m,
- full45: 20,68053065 kg / 0,11193779 m,
- ballast: 18,85788235 kg / 0,11585802 m.

Container image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.

Η συνολική μάζα δεν άλλαξε. Το CG κατέβηκε περίπου 0,50…0,57 mm σε σχέση με
το `h=33 mm`, άρα οι προηγούμενες turn runs παραμένουν ιστορική/ποιοτική
διάγνωση και όχι τελική επικύρωση φορτίων της νέας γεωμετρίας. URDF SHA-256:

- empty: `603813b6f45b6598ae497067dfc61b7319a0a8ee3efb83e0d68b9b6916938747`,
- full45: `45a262b79d74954c68d123556a8a2894e283fb8899bbdd226bce37e2eab4578a`,
- ballast: `1e28279c123adcabde59d1efecec224d3ea8aa2ff7a89d489b0686961e3c5ad2`.

Ο host-only sweep −10…+20°/2° και το sensitivity grid έδωσαν στο reference
`R=85,h=29 mm`: non-wheel 26,000 mm raw / 21,000 mm μετά allowance 5 mm,
flywheel minimum 18,501 mm στο −10°. Στο πλήρες `R=82…90,h=26…33 mm`:

- non-wheel/rail raw 19…34 mm και 14…29 mm μετά allowance: όλα PASS ≥12 mm,
- flywheel raw 11,501…26,501 mm και 6,501…21,501 mm μετά το ίδιο allowance.

Το δυσμενέστερο corner είναι `R=82,h=33 mm`. Το non-wheel clearance περνά,
αλλά το flywheel μένει 11,501 mm raw, οπότε η loaded-radius μέτρηση στο
συναρμολογημένο πλαίσιο παραμένει υποχρεωτική. Μετά το run επιβεβαιώθηκε
κενό `docker ps`. Δεν εκτελέστηκε Gazebo και δεν έγινε 5b–5d. Κατάσταση:
`GROUND_CLEARANCE_SENSITIVITY_UPDATED_AWAITING_LOADED_RADIUS_AND_GAZEBO_APPROVAL`.

### Πείραμα P2-37 — static nip/contact-band gate

Χωρίς Gazebo, επεκτάθηκε το ίδιο sensitivity script και ξαναγράφτηκαν τα
`ground-clearance-sensitivity.json/.csv`. Παράμετροι: `R=82/85/90 mm`,
`h=26/29/33 mm`, συλλογή `−10°`, nip local U=45 mm, flywheel Ø200×50,
κέντρο μπάλας εδάφους Z=33 mm και μη χρησιμοποιήσιμο edge inset 8 mm.
Το inset είναι συντηρητικό proxy για το στρογγύλεμα, όχι μετρημένο tyre
profile. Εντολή:

`python3 engineering/shared-flywheel-isolated-bench/analyze_ground_clearance_sensitivity.py engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation_turn_diagnostic/ground_clearance_sweep --output-json engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation_turn_diagnostic/ground-clearance-sensitivity.json --output-csv engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation_turn_diagnostic/ground-clearance-sensitivity.csv`

Αποτελέσματα για όλο το grid:

- ύψος nip 53,486…68,486 mm,
- απόσταση κέντρου μπάλας από κάτω ακμή −10,866…+4,134 mm: 0/9 PASS στο
  κριτήριο ≥8 mm,
- απλή κάθοδος pivot 3,866…18,866 mm, reference 10,866 mm, αλλά αφήνει μόνο
  7,635 mm raw κάτω από τον flywheel,
- angle-only λύση −15,79…−56,18°: σε όλα τα cases ο Ø200 τέμνει το έδαφος.

Η geometrically compatible συνδυασμένη πρόβλεψη είναι: για 12 mm raw
clearance, `θ=−7,470°` και pivot down 5,662…20,662 mm. Για 12 mm clearance
μετά allowance 5 mm, `θ=−4,589°` και pivot down 7,780…22,780 mm (reference
14,780 mm). Συνεπώς δεσμεύονται τουλάχιστον 23 mm downward adjustment,
positive locking/shims και ρυθμιζόμενο lower stop −10…−4,5°, αλλά καμία
τελική ρύθμιση δεν κλειδώνει πριν από loaded-R/tyre-profile μέτρηση και νέο
guide/mouth collision sweep. Δεν εκτελέστηκε xacro, ROS ή Gazebo. Κατάσταση:
`STATIC_NIP_GATE_FAIL_ADJUSTMENT_RESERVED_PAUSED`.

### Πείραμα P2-38 — tread profile, full-arc clearance και dependent CAD

Χωρίς ROS/Gazebo, εκτελέστηκε:

`python3 engineering/chassis-strength-bench/run_phase2_tread_pivot_static.py`

Το edge rounding διαχωρίστηκε από το κριτήριο: για τυπωμένο flat TPU
ελέγχθηκαν W=50/55/60 mm, r=2/3 mm, με απαίτηση ball centre 8 mm πάνω από τη
χρήσιμη ακμή και flywheel clearance 12 mm μετά allowance 5 mm σε όλο το
R=82…90/h=26…33. Ο πνευματικός τροχός καταγράφηκε χωριστά ως rounded και
μη μετρημένος· δεν του αποδόθηκε μάζα/αδράνεια τυπωμένου τροχού.

Καλύτερο intake candidate: printed flat W60/r2, θ=−3,4398°, pivot down
5,654…20,654 mm. Εκτίμηση ανά τροχό: 0,8790 kg και 0,005090 kg·m², δηλαδή
+41,4 g και +8,47% πολική αδράνεια ως προς το υπάρχον 46 mm tread model.
Το nip 58 mm δεν αλλάζει· το envelope μεγαλώνει 5 mm ανά αξονική πλευρά.
Το candidate δεν περνά το αποδεκτό stop −10…−4,5°: στο −4,5° λείπουν
1,846 mm raw clearance. Στο +20° δίνει 8,644 mm raw / 3,644 mm μετά
allowance, άρα FAIL full arc.

Το OpenSCAD ενημερώθηκε ως **dependent packaging CAD only**: reference down
12,654 mm, travel 23 mm, δύο M6 slots ανά πλευρά, compression shim stack ως
positive vertical stop και indexed replaceable lower-stop rail. Δεν έγινε
STL export/release. Ενδεικτική compile εντολή:

`/snap/bin/openscad-nightly -o /tmp/p2_38_structure.stl -D 'view="structure"' cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad`

Με ρητό +20° endpoint, τα intersection views `qa_sweep_frame`,
`qa_sweep_drive`, `qa_sweep_basket`, `qa_sweep_high_guide`, `qa_sweep_arch`
και `qa_sweep_actuator` ήταν κενά. Το `qa_guide_low_frame_at_angle` ήταν
κενό σε 13 γωνίες −3,4398…+20°. Πρόκειται για nominal collision QA και όχι
απόδειξη ανοχών· το ground-clearance FAIL παραμένει.

Στο χαμηλότερο intake angle, το handoff ball centre είναι 59,246 mm από το
έδαφος και η υπάρχουσα κορυφή guide 133…148 mm. Απαιτείται lift
73,75…88,75 mm, 0,0412…0,0496 J για μπάλα 57 g, ή 1,701…1,866 m/s με 50%
μεταφορά. Το 1,9 m/s είναι οριακό. Σε free flight 80 mm, 0,6/0,9/1,2 m/s
χάνουν 82,7/34,1/17,1 mm ύψους: contained, αναμορφωμένη είσοδος οδηγού
παραμένει υποχρεωτική.

Artifacts: `results/shared_flywheel_phase2/tread_pivot_static.json` και
`tread_pivot_profiles.csv`. Απόφαση:
`NO_FULL_ARC_SOLUTION_DEPENDENT_CAD_ONLY_PAUSED`. Χρειάζεται αλλαγή
αρχιτεκτονικής ή νέος συμβιβασμός launch angle/pivot motion· καμία τελική
θέση και κανένα STL δεν αποδεσμεύεται. Δεν έγινε commit/push/PR.

### Πείραμα P2-39 — operating positions, pivot X/Z και χαμηλότερη παράδοση

Χωρίς ROS/Gazebo εκτελέστηκε ο νέος static sweep:

`python3 engineering/chassis-strength-bench/run_phase2_pivot_operating_positions.py`

Ο κανόνας λειτουργίας πάγωσε πριν από την αξιολόγηση: συλλογή με το ρομπότ
σε κίνηση και allowance 5 mm, μεταφορά μόνο σε 0…+5° με το ίδιο allowance,
και εκτόξευση μόνο με το ρομπότ ακίνητο, όπου ισχύει static όριο 8 mm χωρίς
το allowance. Το indexed κάτω τερματικό διευρύνθηκε σε −10…−3°.

Στο grid `R=82…90 mm`, `h=26…33 mm`, για printed TPU 60 mm/r2 επιλέχθηκαν:

- nip X=442 mm, άρθρωση 55 mm πίσω από το nip (`X=387 mm`),
- συλλογή −3°, βέλτιστη θέση μεταφοράς 0° και εκτόξευση +15…+20°,
- pivot Z=55,837 mm ως ground-coordinate design target, με απαίτηση καθόδου
  5,463…20,463 mm ως προς το nominal chassis datum· διατηρούνται travel
  23 mm, M6 slots, shims και θετικό κλείδωμα.

Οι ελάχιστες αποστάσεις από το έδαφος είναι 17,766 mm raw / 12,766 mm
με allowance στη συλλογή, 25,837 / 20,837 mm στη μεταφορά 0° και 22,030 /
17,030 mm στη μεταφορά +5°. Στην ακίνητη εκτόξευση είναι 15,213 mm στα
+15°, 14,596 στα +16°, 13,992 στα +17°, 13,400 στα +18°, 12,821 στα +19°
και 12,256 mm στα +20°: όλα PASS έναντι 8 mm. Τα +15° κερδίζουν 2,957 mm
απόσταση έναντι +20°, αλλά έχουν χαμηλότερη τροχιά· βεληνεκές, κορυφή και
ύψος πάνω από το φιλέ παραμένουν για το 5d.

Το nominal OpenSCAD collision QA έγινε με εντολές της μορφής:

`/snap/bin/openscad-nightly -o /tmp/p2_39_<view>.stl -D 'view="<qa view>"' cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad`

Στο επιλεγμένο offset 55 mm, τα full-sweep views `qa_sweep_frame`,
`qa_sweep_drive`, `qa_sweep_basket`, `qa_sweep_high_guide`, `qa_sweep_arch`,
`qa_sweep_actuator`, `qa_pivot_obstacles` και το low-guide/deck check ήταν
κενά για −3…+20°. Το bearing/guard envelope μένει κενό έως 73 mm offset και
ακουμπά πρώτο την αψίδα X330 στα 74 mm. Αυτή δεν είναι όμως η ενεργή
περιοριστική περίπτωση: η κινούμενη μονάδα στα +20° είναι κενή έως 57 mm
και τέμνει την αψίδα στα 58 mm, άρα το επιλεγμένο 55 mm έχει μόνο 3 mm
παραμετρικό περιθώριο πριν από ανοχές/runout. Οι αποστάσεις pin-to-pin του
P16 είναι περίπου 123,13 mm στη συλλογή, 119,02 mm στη μεταφορά και
89,70 mm στα +20°: απαιτούμενη διαδρομή 33,43 mm, μέσα στο προηγούμενο
envelope 40 mm αλλά όχι τελική bracket release.

Η χαμηλωμένη είσοδος οδηγού πρέπει να φέρει τις πρώτες δύο θέσεις της στα
75…90 mm, δηλαδή step 16,55…31,55 mm από το handoff centre 58,45 mm, με
ρύθμιση 20…25 mm μέσω shims. Με 50% ενεργειακή μεταφορά και lift έως
89,55 mm, η ελάχιστη ταχύτητα είναι 1,875 m/s. Περιθώριο 25% απαιτεί
τουλάχιστον 2,343 m/s: 2,0 m/s (~191 rpm) και 2,25 m/s (~215 rpm)
αποτυγχάνουν, ενώ 2,35 m/s (~224 rpm) και 2,5 m/s (~239 rpm) περνούν.
Σύσταση 2,35…2,5 m/s, με contained roof, compliant πίσω deflector και
anti-return flap, επειδή η αυξημένη ταχύτητα μεγαλώνει τον κίνδυνο
αναπήδησης στο καλάθι.

Το τετράζυγο δεν ενεργοποιείται: η σταθερή άρθρωση περνά τον νέο κανόνα.
Παραμένει fallback αν οι πραγματικές ανοχές καταναλώσουν τα 3 mm προς την
αψίδα. Artifacts: `results/shared_flywheel_phase2/pivot_operating_positions.json`
και `.csv`. Δεν έγινε STL release, xacro, ROS, Gazebo, commit, push ή PR.
Κατάσταση: `STATIC_CAD_OPERATING_POSITIONS_SELECTED_PAUSED`.

### Πείραμα P2-40 — selected edge grid, arch clearance και xacro validation

Το static grid εκτελέστηκε χωρίς ROS/Gazebo:

`python3 engineering/chassis-strength-bench/run_phase2_p40_selected_geometry.py`

Για W60/r2, pivot 55 mm πίσω από το nip και συλλογή −3°, η αντίστοιχη
positive-lock κάθοδος ανά R/h είναι:

- R82: 12,463 / 9,463 / 5,463 mm για h=26/29/33,
- R85: 15,463 / 12,463 / 8,463 mm,
- R90: 20,463 / 17,463 / 13,463 mm.

Σε όλα τα 9 cases η φυσική κάτω ακμή είναι Z=23 mm, η χρήσιμη ακμή μετά το
r2 reserve είναι Z=25 mm και το κέντρο μπάλας Z=33 mm απέχει **8,000 mm**:
9/9 PASS. Η μέγιστη κάθοδος 20,463 mm περνά το travel 23 mm.

Η αψίδα άλλαξε μόνο στο dependent CAD: crossbar και LiDAR έμειναν X330,
ενώ οι πλευρικοί ορθοστάτες πήγαν X318 και συνδέονται με άνω dogleg. Τα
OpenSCAD checks εκτελέστηκαν με εντολές της μορφής:

`/snap/bin/openscad-nightly -o /tmp/p2_40_<view>.stl -D 'view="<view>"' cad/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad`

Τα exact intersections `qa_sweep_frame`, `qa_sweep_drive`,
`qa_sweep_basket`, `qa_sweep_high_guide`, `qa_sweep_arch`,
`qa_sweep_actuator` και `qa_pivot_obstacles` ήταν κενά στο −3…+20°. Το
directional `qa_sweep_arch_margin`, που μετακινεί την αψίδα προς το πλήρες
swept envelope, ήταν κενό στα 11/12/13/14 mm και έδωσε πρώτη τομή στα
15 mm. Καταγράφεται lower bound 14 mm nominal: με budget 3 mm για ανοχές
εκτύπωσης/συναρμολόγησης και ελαστική μετατόπιση μένουν ≥11 mm, PASS στον
στόχο ≥8 mm. Είναι directional exact-solid QA, όχι φυσική μέτρηση Euclidean
minimum distance.

Το isolated xacro ενημερώθηκε σε pivot X=0,387 m, Z=0,051837 m στο reference
R85/h29, lower joint limit −3°, upper +20°, indexed lower rail −10…−3°,
transport 0° και launch adjustment +15…+20°. Το CAD-only module παραμένει
χωριστό από το legacy authoritative xacro. Για το printed W60/r2 μπήκαν ανά
τροχό, μαζί με hub: mass 0,878977 kg, polar inertia 0,00508982 kg·m²,
transverse inertia 0,00267808 kg·m² και width 0,060 m. Νέα module mass:
4,609958 kg. Πρόκειται για primitive-volume estimate, όχι μέτρηση.

Με ρητή έγκριση εκτελέστηκε μόνο validation, όχι Gazebo:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 180 engineering/shared-flywheel-isolated-bench/validate_v3_xacro.sh"`

Container image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Το source module SHA-256 είναι
`48e1c16a3fb74dd1ea97f5a90f52e0fca3d7333b7a857799abd4f8511191058d`.

Και τα τρία cases πέρασαν xacro expansion, `check_urdf`, positive-definite
inertia, exact mass assertion, pivot/limits, wheel width/mass/inertia,
collision presence και ros2_control interfaces:

- empty: 18,019957627 kg, URDF
  `6301c905c5d74901bcedb45dca9059ebb4759b73228a883f4ee9c41811ada986`,
- full45: 20,584957627 kg, URDF
  `9eca51df73486e2451078f21cdc3c6f14844a368aa7126fbc0ec9ffe4f45d942`,
- ballast: 18,857882353 kg, URDF
  `8cb0c4b175ec080156fd52b1fe99ded8090c5de5c7db2302c13d2b88239e83dd`.

Μετά από κάθε container run το `docker ps` ήταν κενό. Δεν εκτελέστηκε
Gazebo, 5b–5d ή STL release και δεν έγινε commit/push/PR. Κατάσταση:
`P2_40_XACRO_VALIDATED_PAUSED_BEFORE_GAZEBO`.

### Πείραμα P2-41 — Gazebo 5b preflight, BLOCKED πριν την εκτέλεση

Έγινε read-only επιθεώρηση του isolated bench, του παραγόμενου P2-40 URDF,
του 5a world/runner/plugin και του υπάρχοντος phase plan. Δεν ξεκίνησε
Gazebo, επειδή ενεργοποιήθηκε το ρητό STOP condition περί ασαφούς 5b
specification/model.

Εντολές επιθεώρησης:

`rg -n "5b|transition|tilt|P16|endpoint|terminal|joint sweep|settling|contact" engineering/shared-flywheel-isolated-bench docs/mechanism`

`rg -n "arch|lidar|p16|launcher_endpoint|launcher_tilt|plugin|contact" engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation/empty.urdf engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a/empty/mu_0p7/model.sdf`

Blocking findings:

1. Το authoritative isolated xacro/URDF δεν έχει collision link για side legs
   X318, crossmember X330 ή LiDAR. Άρα το υποχρεωτικό arch-contact check δεν
   είναι παρατηρήσιμο στο Gazebo.
2. Το P16 είναι μόνο fixed envelope 95 g. Δεν υπάρχει linkage, κινούμενο
   clevis, stroke/force-speed/backdrive model. Η κίνηση είναι direct-effort
   revolute joint.
3. Υπάρχουν fixed endpoint blocks, αλλά όχι το αντίστοιχο moving stop/bracket
   collision πάνω στο yoke. Δεν μπορεί να αποδειχθεί terminal contact/load
   bypass.
4. Δεν υπάρχει `run_5b`, transition plugin/system, contact recorder, analyzer
   ή `criteria_5b`. Το 5a plugin κάνει μόνο fixed-target tilt hold και drive
   scenarios.
5. Το `drive_test_ballast` αφαιρεί τη μονάδα και βάζει fixed ballast στο
   base link. Συνεπώς ballast tilt sweep δεν ελέγχει module/P16/clearance και
   η σημασία του ζητούμενου τρίτου load case χρειάζεται αποσαφήνιση.
6. Δεν υπάρχουν παγωμένα αριθμητικά όρια για tracking error, overshoot,
   settling, oscillation, chassis displacement ή contact impulse. PASS/FAIL
   μετά από run θα απαιτούσε επινόηση κριτηρίου.

Η παύση έγινε πριν από οποιοδήποτε baseline run, όπως απαιτεί η failure
discipline. Δεν άλλαξε CAD/xacro/controller και το P2-40 static PASS παραμένει
σε ισχύ. Machine-readable evidence:
`results/shared_flywheel_phase2/gazebo_5b/preflight.json`.

Απαιτούμενο επόμενο gate: ρητή έγκριση ενός 5b harness/criteria package που
θα (α) παγώσει αριθμητικά κριτήρια πριν το πρώτο run, (β) προσθέσει στο
isolated xacro μόνο CAD-faithful collision envelopes της παγωμένης αψίδας,
(γ) μοντελοποιήσει ή ρητά εξαιρέσει τη φυσική κινηματική P16/stop bracket,
(δ) καταγράφει contact owners και (ε) ορίσει αν το ballast είναι μόνο
spawn/stability case ή moving inertial surrogate. Κατάσταση:
`GAZEBO_5B_BLOCKED_PREFLIGHT_MODEL_AND_CRITERIA_INCOMPLETE`.

### Πείραμα P2-42 — ελαφρύ harness και Gazebo 5b

Με ρητή έγκριση υλοποιήθηκε και εκτελέστηκε το isolated 5b στο Ubuntu
24.04/Jazzy/Harmonic container. Το παγωμένο CAD δεν άλλαξε. Το isolated
reference της άρθρωσης κατέβηκε μόνο κατά 2 mm, από Z=51,837 σε 49,837 mm,
ώστε το W60/r2 grid R=82…90/h=26…33 να στοχεύει 10,000 mm από το κέντρο
μπάλας ως τη χρήσιμη ακμή. Η απαιτούμενη positive-lock κάθοδος είναι
7,463…22,463 mm, μέσα στη διαθέσιμη ρύθμιση 23 mm. Το φυσικό minimum
παραμένει 8 mm και το P2-40 static geometry PASS παραμένει έγκυρο.

Πριν το πρώτο run παγώθηκε το `criteria_5b.json`: error μετά την παραμονή
≤0,5°, μηδενικές επαφές launcher με arch/crossmember/LiDAR/chassis/ground,
μηδενικά NaN/Inf και joint-limit violations, roll/pitch ≤3°, base rise
≤5 mm και command ≤12,6 N·m. Contact force, settling/overshoot και οριζόντια
μετατόπιση μένουν diagnostic όταν δεν υπάρχει φυσικό requirement. Ο P16
είναι ρητά scope B: abstract joint actuator, 23°/8,3 s = 0,048361 rad/s,
χωρίς ισχυρισμό force/current/link stress/singularity. Η κρούση terminal
είναι εκτός 5b και τα joint limits είναι τα stops. Τα `empty/full45` είναι
dynamic, ενώ το `ballast` είναι μόνο spawn/stability.

Προστέθηκαν collision-only outer envelopes, traceable στο παγωμένο CAD:
δύο legs X318 30×50×193 mm, δύο doglegs X318→X330 42×30×20 mm, crossmember
X330 50×440×30 mm και LiDAR Ø76×32 mm στο `[330,0,374] mm`. Για το 5b το
παλιό coarse base box αντικαθίσταται από τα πραγματικά rail/crossmember
primitives, ώστε επαφή σε κενό χώρο να μη δώσει false positive. Η πλήρης
mapping βρίσκεται στο `p2_40_collision_fixture_mapping.json`.

Το harness καταγράφει ξεχωριστά commanded/measured tilt, base/module poses,
contact owners/force/depth και Gazebo log. Η ακολουθία είναι
`−3→0→+15→+20→+15→0→−3°`, dwell 3 s, ακίνητο chassis και stopped flywheels.
Τα contact sensors επιβεβαίωσαν μόνο τις τέσσερις αναμενόμενες επαφές
drive-wheel/ground (διπλές owner directions στο raw log), καμία επαφή της
μονάδας με fixture/chassis/ground.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 180 engineering/shared-flywheel-isolated-bench/run_5b_case.sh empty"`

και αντίστοιχα `full45`, `ballast` (90 s timeout στο ballast). Το image
digest ήταν `sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.

Το πρώτο PD-only empty diagnostic απέτυχε tracking (1,969°) λόγω static
gravity offset, χωρίς forbidden contact. Προστέθηκε integral action μόνο
στο test controller. Δεύτερη ρύθμιση έδωσε 0,979°. Το τελικό isolated PID
`Kp/Ki/Kd=240/80/12`, με 0,01° initialization inset για να αποφεύγεται το
γνωστό πρώτο DART constraint projection, πέρασε χωρίς αλλαγή κριτηρίων.
Η ανοχή σύγκρισης joint limit είναι 1e−5 rad και δηλώνεται ως αριθμητική,
όχι πρόσθετη φυσική διαδρομή.

Τελικός πίνακας:

- empty dynamic: PASS, max post-dwell error 0,221786°, forbidden contacts 0,
  limit violations 0, roll/pitch ≈0/0,000009°, base rise 2,503910 mm,
  max command 2,83268 N·m, diagnostic contact force 32,3374 N·
- full45 dynamic: PASS, error 0,221668°, forbidden contacts 0, violations 0,
  roll/pitch ≈0/0,000008°, rise 2,503911 mm, command 2,83143 N·m,
  diagnostic contact force 36,0518 N·
- ballast spawn/stability-only: PASS, NaN/Inf 0, roll/pitch
  ≈0/0,000006°, rise 2,503912 mm, command 0, contact force 36,4553 N.

URDF SHA-256: empty
`7f4e5805438b0f1d7e60b201c7212fe3f90f44ab510f65aa288e168b269ce6a9`,
full45 `95cbcdac2a73e02ec01316a7e5858f7377011983f61aed21022354a8acf8dce9`,
ballast `9f65f0a21445c481bfd0e8e9ad5616158b01dc0b04e342657067833d0ea662a6`.
Μετά από κάθε run το `docker ps` ήταν άδειο. Machine-readable aggregate:
`results/shared_flywheel_phase2/gazebo_5b/summary.json`. Δεν έγινε STL,
commit, push ή PR. Κατάσταση:
`P2_42_GAZEBO_5B_PASS_PAUSED_BEFORE_5C_5D`.

**Διόρθωση datum P2-42:** το αρχικά αναφερόμενο κοινό `base rise =
2,504 mm` ήταν spawn-relative artefact του analyzer. Το world έκανε spawn το
`base_link` στο Z=1 mm, με τα ελαστικά αρχικά διεισδυμένα στο ground, και ο
DART constraint solver το μετέφερε ομαλά στο settled Z≈4 mm. Ο παλιός
analyzer αφαιρούσε τον μέσο Z του πρώτου 1 s (≈1,496 mm), όχι settled datum,
οπότε έδινε 4,000−1,496=2,504 mm και στα τρία cases. Το frozen 5b summary
επανυπολογίστηκε με settled datum τον μέσο Z στο 3,5…4,0 s και αξιολόγηση
μόνο μετά τα 4,0 s: μεταγενέστερο rise `0,00016 mm` (≤1 μm), ενώ η
spawn-to-settled διόρθωση ≈`2,999 mm` καταγράφεται χωριστά ως diagnostic.
Δεν άλλαξε το PASS/FAIL ούτε προέκυψε wheel lift.

### Πείραμα P2-43 — Gazebo 5c-1, σύλληψη μίας μπάλας έως την είσοδο οδηγού

Με ρητή έγκριση εκτελέστηκε μόνο το στάδιο 5c-1 στο Ubuntu 24.04 / ROS 2
Jazzy / Gazebo Harmonic container. Το παγωμένο P2-40 CAD δεν άλλαξε και δεν
εκτελέστηκαν 5c-2, 5c-3 ή 5d. Πριν από το πρώτο run παγώθηκε το
`engineering/shared-flywheel-isolated-bench/criteria_5c.json` με 180
συνθήκες και 5 επαναλήψεις ανά συνθήκη (900 runs): προσέγγιση
0,35/0,60/0,80 m/s, πλευρική απόκλιση 0/±50/±100 mm, narrow/wide cheeks,
μπάλα–πέλμα μ=0,3/0,6/0,9 και περιφερειακή ταχύτητα 2,35/2,50 m/s. Η τριβή
εδάφους ήταν 0,7.

Τα hard criteria ήταν: επιτυχία ≥4/5, πλήρης διέλευση του nip όταν το κέντρο
περάσει X=−33 mm στο launcher frame, είσοδος οδηγού στο παγωμένο rear-throat
datum X=−105 mm, πίσω ταχύτητα στο nip ≥1,7 m/s (το ≥1,9 m/s αναφέρεται
χωριστά), forward plough ≤100 mm, συνεχές low-speed jam ≤0,5 s, ρεύμα ανά
D5065 ≤20 A, ροπή ≤0,62 N·m, finite state και μηδενική επαφή της κινούμενης
μονάδας με έδαφος/σασί/αψίδα/LiDAR. Το 100 mm είναι μεγαλύτερο από τρεις
ακτίνες μπάλας και από όλο το taper 77 mm· το 0,5 s είναι πολύ μεγαλύτερο
από τον ιδανικό χρόνο 105/1700=0,062 s. Δεν επιβλήθηκε ταχύτητα ή τεχνητή
μεταφορά στην μπάλα.

Το isolated harness κράτησε την άρθρωση στις −3°, χρησιμοποίησε το reference
καθόδου με περιθώριο useful-edge 10 mm, Ø200×60/r2 τροχούς με αντίστροφη
φορά συλλογής και το υφιστάμενο αναλυτικό D5065 back-EMF/current model. Η
μπάλα φορτώθηκε από το αμετάβλητο `tennis_ball_compliant`· προστέθηκαν μόνο
οι ήδη παγωμένοι collision surrogates narrow/wide cheeks και moving guide
floor. Το preflight ήταν PASS, με SHA-256 ball model
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`.
Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Τα δύο μοναδικά URDF hashes ήταν narrow
`5ce1c474788b1be197a422624d3b307b69a7125da6662798eaa385fc0bac9c99`
και wide
`673bd7f140c1c88a6dc2e16f674d128f2af310eae448d4ef9388749224858795`.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 120 engineering/shared-flywheel-isolated-bench/run_5c1_case.sh 0.6 0.0 narrow 0.6 2.35 1"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 3600 env P2_43_PARALLELISM=6 engineering/shared-flywheel-isolated-bench/run_5c1_campaign.sh"`

Τελικό αποτέλεσμα: **FAIL, 0/180 συνθήκες PASS**, 900/900 runs ολοκληρωμένα,
χωρίς NaN/Inf και με 0 forbidden contacts. Η πλήρης nip διέλευση έγινε σε
560/900 runs και το όριο 1,7 m/s πέρασε σε 550/900. Καμία μπάλα δεν έφτασε
το guide-entry plane. Και στα 560 nip-crossing runs η μπάλα ακούμπησε την
τετραγωνισμένη εμπρός όψη του moving guide floor και σταμάτησε στο launcher
X=−43,709…−42,283 mm. Η όψη αρχίζει περίπου στο X=−76 mm, άρα με ακτίνα
μπάλας 33 mm η αναμενόμενη θέση επαφής κέντρου είναι περίπου −43 mm. Αυτό
είναι εσωτερική επαφή μπάλα–οδηγός, όχι επαφή της μονάδας με το έδαφος ή το
σασί και συνεπώς δεν αντιφάσκει με το P2-40 clearance gate. Η μέγιστη δύναμη
επαφής παραμένει diagnostic επειδή δεν υπάρχει βαθμονομημένη δυσκαμψία
οδηγού.

Σύνοψη ανά μεταβλητή (nip / speed≥1,7, κατόπιν failed repeats
plough/jam):

- v=0,35: 160/300, 150/300, 200/155· v=0,60: 200/300, 200/300,
  200/200· v=0,80: 200/300, 200/300, 200/200.
- y=0: 180/180, 180/180, 0/180· y=±50 mm: 140/180, 135/180,
  120/140 ανά πλευρά· y=−100/+100 mm: 50/180, 50/180, 180/50 και
  180/45 αντίστοιχα.
- narrow: 250/450, 250/450, 360/250· wide: 310/450, 300/450, 240/305.
- μ=0,3: 100/300, 100/300, 240/100· μ=0,6: 220/300, 220/300,
  180/220· μ=0,9: 240/300, 230/300, 180/235.
- surface 2,35: 280/450, 280/450, 300/280· surface 2,50: 280/450,
  270/450, 300/275.

Το ρεύμα έφτασε ακριβώς 20,0 A και η ροπή 0,62 N·m σε όλα τα runs: δεν
υπήρξε αριθμητική υπέρβαση, αλλά το περιθώριο είναι μηδενικό και το 5c-1
δεν αποδεικνύει θερμική ή ηλεκτρική επάρκεια. Τα 5 repeats είναι
ντετερμινιστικές επαναλήψεις του ίδιου physics setup, όχι Monte Carlo
ανοχών. Ένα trailing, μερικώς γραμμένο contact CSV row κατά το orderly
termination αγνοείται πλέον ρητά· όλα τα summaries αναγεννήθηκαν από τα raw
telemetry/contact logs με πλήρη ownership για cheeks και guide.

Ο πλήρης πίνακας των 180 συνθηκών είναι
`results/shared_flywheel_phase2/gazebo_5c1/condition_matrix.csv` και το
machine-readable aggregate είναι `gazebo_5c1/summary.json`. Η αποτυχία είναι
σαφής geometry/transition finding: η τετραγωνισμένη ακμή εισόδου του οδηγού
δεν επιτρέπει στην μπάλα που έχει ήδη περάσει το nip να ανέβει στη ράμπα.
Δεν εφαρμόστηκε αλλαγή CAD επειδή το P2-40 είναι παγωμένο. Κατάσταση:
`P2_43_GAZEBO_5C1_FAIL_PAUSED_BEFORE_5C2_5C3_5D`.

### Πείραμα P2-44 — ανοικτή ράμπα, εκτεθειμένοι τροχοί και basket-entry subset

Το P2-43 έγινε δεκτό ως diagnostic και επιτράπηκε αλλαγή μόνο στα
ΕΞΑΡΤΩΜΕΝΑ guide/cheeks. Η άρθρωση X387/Z49,837, οι γωνίες −3/0/+15…+20°
και η αψίδα X318/X330 έμειναν παγωμένα. Πριν από το πρώτο physics run
γράφτηκε το `criteria_5c_p244.json` (SHA-256
`25a58a467b02e360fb242a7a9a089cb78726d48d1415d10e31d0029d0cc123b2`):
μόνο v=0,6 m/s, flywheel surface=2,5 m/s, ball–tread μ=0,6,
y=0/±50/±100 mm, τρεις cheek variants και 5 deterministic repeats, δηλαδή
15 συνθήκες/75 runs. Επιτυχία συνθήκης απαιτεί ≥4/5 πλήρη PASS και πλέον το
κέντρο της μπάλας πρέπει να περάσει τη φυσική είσοδο του καλαθιού στο base
frame X≤278 mm, |Y|≤15 mm και Z=75…125 mm. Διατηρήθηκαν ως hard gates το
nip, plough≤100 mm, jam≤0,75 s, 20 A/0,62 N·m, finite state και μηδενική
launcher επαφή με fixed ramp/chassis/ground/arch.

Η παλιά κινούμενη guide floor αφαιρέθηκε μόνο από το P2-44 alternative. Η
νέα κύρια ράμπα είναι **σταθερή στο base/chassis**, ανοικτή από πάνω, με
top path `[X,Z]` mm:
`[402,3]→[377,8]→[352,21]→[328,36]→[315,46]→[305,52]→[298,54]→[278,54]`.
Το πλάτος αυξάνει 56→96 mm, χαμηλά rails αρχίζουν μετά τη ζώνη τροχών και
υπάρχει μόνο μικρό anti-rebound flap, όχι κλειστή οροφή. Η εμπρός γλώσσα
έχει underside περίπου 1 mm από το έδαφος και, επειδή είναι chassis-fixed,
η τιμή είναι ίδια σε συλλογή/μεταφορά/εκτόξευση. Αυτό λύνει το πρόβλημα
όπου μια moving ramp θα κατέβαινε κάτω από το έδαφος στο +20°, αλλά απαιτεί
εύκαμπτη/αναλώσιμη μύτη και dirt-shedding. Η ράμπα ακουμπά σκόπιμα στο
επάνω επίπεδο Z=52 mm του low deck κοντά στο X=276…286 mm ως support/mount.

OpenSCAD exact-solid QA πριν το Gazebo:

- κινούμενη παγωμένη μονάδα έναντι fixed ramp, −3…+20° ανά 1°: empty,
- νέα cheeks έναντι Ø200 flywheels: empty,
- ramp έναντι frame: μόνο μηδενικού πάχους tangent support στο Z=52 mm,
  χωρίς διείσδυση όγκου.

Οι τρεις εκδοχές αφήνουν το εμπρός-εσωτερικό τεταρτημόριο του τροχού
ελεύθερο, τερματίζοντας 5 mm μπροστά από το ονομαστικό Ø200 envelope:

- `exposed_short`: X=180/Y=125 προς X=105/Y=80 mm, γωνία 30,96°,
  cheek μ=0,35,
- `exposed_shallow`: X=220/Y=125 προς X=105/Y=80 mm, γωνία 21,37°,
  cheek μ=0,35,
- `exposed_shallow_lowfriction`: ίδια γεωμετρία 21,37°, cheek μ=0,10.

Το xacro preflight/check_urdf ήταν PASS και για τις τρεις εκδοχές. Το
calibrated ball hash έμεινε
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`.
Container image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Τα τελικά URDF hashes είναι `ea68daa8781b8dd9c624a026a852341edc10868de6c1f4d4f7a4db2717820e02`
για short και `982dea7e64e842097aae06ae2ec6ab457eeb24b66f493f5605123f4d69e72676`
για τις δύο shallow εκδοχές· η τριβή cheek εφαρμόζεται στο world SDF, άρα
δεν αλλάζει το URDF hash.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 150 engineering/shared-flywheel-isolated-bench/run_5c_p244_case.sh exposed_shallow 0.0 1"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 1200 env P2_44_PARALLELISM=5 engineering/shared-flywheel-isolated-bench/run_5c_p244_campaign.sh"`

Το πρώτο 75-run campaign έδωσε 9/15 PASS, αλλά σημάνθηκε **SUPERSEDED**
πριν την αναφορά: το numerical basket criterion είχε σωστό άνοιγμα 96 mm,
όμως το collision fixture είχε αφαιρέσει όλο το front wall. Προστέθηκαν τα
δύο front-wall panels έξω από το κεντρικό άνοιγμα και επαναλήφθηκαν και τα
75 runs, με τα ίδια παγωμένα criteria, στον φάκελο `gazebo_5c_p244_r2`.

Τελικό replacement αποτέλεσμα: **9/15 conditions PASS**, 75/75 runs, 0
forbidden contacts, 0 NaN/Inf. Και τα 75 πέρασαν πλήρως το nip και έφτασαν
τη φυσική είσοδο καλαθιού· δεν υπήρξε jam. Όλες οι y=0/±50 mm συνθήκες
πέρασαν 5/5 και για τις τρεις εκδοχές. Και οι έξι y=±100 mm συνθήκες
απέτυχαν 0/5 αποκλειστικά από το παγωμένο plough gate: forward displacement
130,616…199,127 mm αντί ≤100 mm. Το nip exit ήταν 1,935…2,251 m/s. Ρεύμα
και ροπή έφτασαν ακριβώς 20,0 A και 0,62 N·m σε όλα τα runs, χωρίς
περιθώριο. Οι short και shallow-lowfriction εκδοχές ακούμπησαν τα side
panels της εισόδου στα ±100 mm (20 runs συνολικά) πριν μπουν· το κανονικό
`exposed_shallow` μπήκε χωρίς αυτή την επαφή αλλά είχε το μεγαλύτερο plough
≈199 mm. Οι contact forces 0,53…73,93 N παραμένουν diagnostic επειδή δεν
υπάρχει βαθμονομημένη guide/flap stiffness.

Ο πλήρης τελικός πίνακας είναι
`results/shared_flywheel_phase2/gazebo_5c_p244_r2/condition_matrix.csv` και
το aggregate `gazebo_5c_p244_r2/summary.json`. Συμπέρασμα: η σύλληψη, η
εφαπτομενική ράμπα και η είσοδος καλαθιού λειτουργούν στο model έως ±50 mm.
Στα ±100 mm το ανοιχτό wheel quadrant εξασφαλίζει τελικά nip/basket entry,
αλλά όχι χωρίς υπερβολική προώθηση της μπάλας. Επόμενη geometry iteration
πρέπει να ανοίξει/επιμηκύνει την αρχή του funnel ή να καμπυλώσει το cheek,
χωρίς νέο run πριν από έγκριση. Δεν έγινε 5c-2/5c-3/5d, STL, commit, push ή
PR. Κατάσταση: `P2_44_PARTIAL_PASS_PAUSED_BEFORE_NEXT_GUIDE_ITERATION_OR_5C2`.

### Πείραμα P2-45 — παράθυρα ρεύματος και σάρωση γλώσσας 3/5/8 mm

Πριν από το πρώτο νέο run πάγωσε το `criteria_5c_p245.json` (SHA-256
`6e0be24495ca3aa2e93f8273dbb5027af838af0ee6eebe6cc6f54ed7361ca547`).
Η μήτρα κράτησε v=0,6 m/s, surface speed=2,5 m/s, ball–tread μ=0,6,
ground μ=0,7, y=0/±50 mm και τις τρεις P2-44 cheek variants. Πρόσθεσε
nominal underside clearance 3/5/8 mm και 5 deterministic repeats: 27
συνθήκες/135 runs. Hard gate: ακριβώς 5/5 nip crossing και basket entry,
χωρίς jam, electrical-limit violation, nonfinite state ή forbidden contact.
Το plough καταγράφεται πλέον ως diagnostic διαδρομής, όχι hard failure.

#### Επανανάλυση υπάρχοντος ρεύματος — χωρίς νέο physics run

Το `analyze_p245_current_windows.py` ανέλυσε τα raw telemetry των 75
P2-44-r2 runs και χωριστά το αναφερόμενο P2-43 run
`v_0p35/y_0p0/narrow/mu_0p9/surf_2p35/r1`. Παράθυρα: spin-up t=0…2,0 s·
ball–wheel contact από πρώτο έως τελευταίο contact-id 0/1 με normal force
>0,01 N· saturation όταν |I|≥19,999 A, με χρόνο ολοκληρωμένο στα telemetry
sample intervals. Όλα τα ακόλουθα ρεύματα είναι απόλυτες τιμές ανά μοτέρ.

Στα P2-44-r2, το spin-up είχε peak 20,000 A, mean 3,597 A και saturation
0,185 s και στα δύο μοτέρ. Το contact window ήταν 0,038…0,208 s (mean
0,137 s): left peak 7,327…11,309 A (mean run-peak 9,192 A), window-mean
2,808…4,954 A (mean 3,854 A)· right peak 7,331…11,313 A (mean 9,225 A),
window-mean 2,808…4,954 A (mean 3,856 A). Contact saturation: **0,000 s
σε όλα τα runs και στα δύο μοτέρ**.

Στο συγκεκριμένο P2-43 run, το spin-up είχε peak 20,000 A, mean 3,387 A
και saturation 0,170 s ανά μοτέρ. Στο contact window 0,041 s, peak 8,010 A,
window-mean 5,034 A και saturation 0,000 s ανά μοτέρ. Άρα το «20 A/0,62
N·m σε κάθε run» ήταν startup peak, όχι συνεχές φορτίο διέλευσης μπάλας.
Τα αποτελέσματα παραμένουν του αναλυτικού motor model, όχι πραγματική
μέτρηση. Ανά-run δεδομένα: `gazebo_5c_p244_r2/current_windows_p245.csv`·
aggregate: `current_windows_p245.json`.

#### Γεωμετρία, preflight και εκτέλεση

Η chassis-fixed ράμπα έγινε παραμετρική. Τα πρώτα top-Z σημεία είναι
`[clearance+2, 8+0,75(clearance−1), 21+0,35(clearance−1)]` mm· η υπόλοιπη
καμπύλη/είσοδος δεν άλλαξε. Exact-solid OpenSCAD sweep κινούμενης μονάδας
έναντι fixed ramp σε −3…+20° ήταν empty και στα τρία ύψη. Δεν έγινε STL.

Το Jazzy preflight ήταν PASS και στα 9 height×cheek xacro variants:
`check_urdf`, fixtures, απουσία legacy moving floor και αμετάβλητο ball hash
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`.
Λόγω επικάλυψης των URDF segment boxes, η συντηρητική κάτω AABB είναι
2,662/4,697/7,751 mm για exact-CAD nominal 3/5/8 mm. Δεν προέκυψε ground
contact. Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Τα URDF hashes βρίσκονται στο `gazebo_5c_p245/preflight/preflight.json`.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 180 engineering/shared-flywheel-isolated-bench/validate_5c_p245.sh"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 150 engineering/shared-flywheel-isolated-bench/run_5c_p245_case.sh 8 exposed_shallow 0.0 1"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 1800 env P2_45_PARALLELISM=5 engineering/shared-flywheel-isolated-bench/run_5c_p245_campaign.sh"`

#### Αποτελέσματα και απόφαση

Αποτέλεσμα: **135/135 runs, 27/27 συνθήκες PASS**, όλες ακριβώς 5/5, με
0 basket-entry failures, 0 nip failures, 0 jams, 0 forbidden contacts και
0 plough diagnostics.

- 3 mm: 9/9 PASS, nip-exit 1,831…2,342 m/s.
- 5 mm: 9/9 PASS, nip-exit 1,776…2,250 m/s.
- 8 mm: 9/9 PASS, nip-exit 1,704…1,904 m/s.

Η μέγιστη pre-nip εμπρός μετατόπιση ήταν 44,63 mm και η μέγιστη απόλυτη
πλευρική 63,37 mm· για y=0 η εμπρός μετατόπιση ήταν μηδενική. Το whole-run
peak παρέμεινε 20 A μόνο λόγω spin-up. Πλήρης πίνακας:
`gazebo_5c_p245/condition_matrix.csv`· aggregate:
`gazebo_5c_p245/summary.json`. Μετά την καμπάνια το `docker ps` ήταν άδειο.

Σύσταση: **8 mm nominal exact-CAD clearance**, το μεγαλύτερο δοκιμασμένο
που πέρασε 5/5 σε όλη τη μήτρα. Είναι simulation geometry result, όχι
απόδειξη για χαλίκι/χώμα, ανοχές ή κάμψη· απαιτείται φυσικός έλεγχος debris
passage και loaded clearance.

Για επόμενες καμπάνιες ορίζονται ανεξάρτητα: (α) `collection_success`, hard,
με nip crossing και basket entry, και (β) diagnostics
`pre_nip_forward_displacement_m`, `pre_nip_lateral_displacement_m` και
front-wall/cheek contacts. Έτσι η συλλογή στα ±100 mm μπορεί να είναι
επιτυχής ενώ η απόσταση σπρωξίματος παραμένει μετρήσιμη πληροφορία
διαδρομής. Τα παλιά P2-43/P2-44 αποτελέσματα δεν αλλάζουν αναδρομικά.

Δεν έγινε 5c-2, 5c-3, 5d, STL, commit, push ή PR. Κατάσταση:
`P2_45_PASS_PAUSED_BEFORE_5C2_5C3_5D`.

### P2-46 — Gazebo 5c-2: παράδοση και συγκράτηση στο καλάθι

#### Παγωμένα κριτήρια και reference

Πριν από το πρώτο run γράφτηκε το
`engineering/shared-flywheel-isolated-bench/criteria_5c2_p246.json`, SHA-256
`322a3036b6be63362943b4b6c206236dfd1e6f38508f4aea71614f56ed468673`.
Το reference της chassis-fixed ράμπας ενημερώθηκε σε **8 mm nominal
underside clearance** (exact CAD), με shallow exposed cheeks, μονάδα −3°,
προσέγγιση 0,6 m/s, surface speed 2,5 m/s, ball–tread μ=0,6 και ground
μ=0,7. Το κριτήριο delivery ήταν ακριβώς 5/5 ανά preload, χωρίς bounce-out,
μόνιμο κλείσιμο της εισόδου ή απώλεια προφορτωμένης μπάλας. Στην κίνηση το
κριτήριο ήταν μηδενική απώλεια από 45 μπάλες.

Οι προφορτωμένες μπάλες αφέθηκαν να πέσουν δυναμικά από deterministic
low-discrepancy θέσεις πάνω από το ανοικτό καλάθι· δεν τοποθετήθηκαν σε
τελικό grid. Η εισερχόμενη μπάλα κινήθηκε μόνο από ground/robot/flywheel
contacts, χωρίς forced velocity ή artificial transfer. Η επαφή της με
έδαφος/flywheel διατηρεί το αμετάβλητο calibrated
`tennis_ball_compliant`. Basket/ramp και ball–ball χρησιμοποιούν native
DART rigid contact. Επειδή το ball–ball compliance/restitution **δεν είναι
βαθμονομημένο**, ακριβή rebound amplitudes, contact forces και η φυσική
χωρητικότητα δεν θεωρούνται επικυρωμένα φυσικά αποτελέσματα.

#### Harness και preflight

Προστέθηκαν μόνο στο isolated bench: δυναμικό basket preload, παρατηρητής
retention/entry, analyzer, aggregate και compact drive-retention sequence.
Δεν άλλαξε το παγωμένο CAD P2-40, ο πλήρης robot stack, Arduino ή
ηλεκτρονικά. Το preflight πέρασε με 37 αναμενόμενα runs, αμετάβλητο ball
hash `8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`,
delivery URDF hash
`b62448d91244495286aa5188d956cbfe4af26f5fd55b806bcaf5e88df53e6ed0`
και motion URDF hash
`d848c7012715391c2fe4ae0647183d786a2b6a15a541563a570cd1f7cc3e077d`.
Η conservative ramp AABB ήταν 7,751 mm έναντι 8 mm exact CAD. Container
image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 engineering/shared-flywheel-isolated-bench/validate_5c2_p246.sh"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 180 engineering/shared-flywheel-isolated-bench/run_5c2_p246_case.sh delivery 0 1"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 3600 env P2_46_PARALLELISM=3 engineering/shared-flywheel-isolated-bench/run_5c2_p246_campaign.sh"`

#### Αποτελέσματα delivery/capacity

Η καμπάνια ολοκληρώθηκε 37/37 runs, χωρίς NaN/Inf ή forbidden launcher
contacts, αλλά το frozen 5c-2 criterion **απέτυχε**:

- preload 0 → σύνολο 1: 5/5 PASS, χωρίς spill/block/bounce.
- preload 10 → σύνολο 11: 4/5 PASS. Στο αποτυχημένο repeat είχε ήδη
  διαφύγει 1/10 preload ball πριν την παράδοση και η νέα μπάλα δεν έφτασε
  στο nip.
- preload 25 → σύνολο 26: 0/5 PASS. Η είσοδος ήταν επίμονα κατειλημμένη
  5/5 και υπήρξε preload spill σε 4/5. Σε ένα repeat η νέα μπάλα πέρασε nip
  και basket entrance, αλλά έμεινε στην entrance zone και δεν
  συγκρατήθηκε ως νέα αποθηκευμένη μπάλα.
- preload 40 → σύνολο 41: 0/5 PASS, entrance blocked 5/5 και preload spill
  4/5.
- capacity probes 44/49/54/59 preload (σύνολα 45/50/55/60): 0/3 PASS σε
  κάθε σημείο. Όλα είχαν preload spill· το πρώτο παγωμένο failed probe
  είναι επομένως 45, με resolution 5 μπάλες, αλλά **δεν υπάρχει έγκυρο
  lower passing capacity probe** από αυτή τη σειρά.

Άρα ο στόχος 45 **δεν αποδεικνύεται**. Το harness αποδεικνύει μόνο 1 μπάλα
με 5/5 και δίνει 4/5 στα 11· δεν επιτρέπεται να παρουσιαστεί αυτό ως
πρακτική χωρητικότητα 11. Η αστοχία εμφανίζεται ως spill προς το ανοικτό
front/entry και μόνιμη κατάληψη της εισόδου, όχι ως bounce-out της νέας
μπάλας. Είναι ισχυρή ένδειξη ότι χρειάζεται retention lip/flap ή διαφορετική
διαμόρφωση pile/entry, αλλά η τελική γεωμετρική αλλαγή πρέπει να επαληθευτεί
με φυσική δοκιμή και/ή βαθμονομημένο ball–ball contact.

Στα runs όπου υπήρξε πραγματική ball–wheel επαφή, το μέγιστο contact-window
ρεύμα ήταν 10,108…10,572 A και το μέσο 1,840…6,387 A, χωρίς saturation.
Τα μηδενικά aggregate current στα υψηλά preload σημαίνουν ότι η νέα μπάλα
δεν έφτασε στους τροχούς, όχι μηδενικό απαιτούμενο ρεύμα. Το whole-run
20 A παραμένει αποκλειστικά spin-up peak.

#### Κίνηση με 45 μπάλες

Και τα 5 repeats απέτυχαν. Μετά το δυναμικό γέμισμα υπήρχαν μόνο 41…44 από
τις 45 μπάλες στο settled baseline. Κατά τη sequence settle → ευθεία
0,6 m/s → φρενάρισμα → moving turn → stop καταγράφηκαν αντίστοιχα
3/4/3/2/1 μοναδικές απώλειες (13 συνολικά ανά-run events), με τελικό
retained count 41…44. Roll/pitch ήταν πρακτικά μηδενικά, άρα η αποτυχία
είναι containment και όχι ανατροπή σασί.

Πλήρης μήτρα:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246/condition_matrix.csv`·
aggregate:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246/summary.json`·
preflight:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246/preflight/preflight.json`.
Μετά την καμπάνια επιβεβαιώθηκε κενό `docker ps`.

Δεν έγινε 5c-3, 5d, STL, commit, push ή PR. Κατάσταση:
`P2_46_5C2_FAIL_PAUSED_BEFORE_5C3_5D`.

### P2-47 — basket-v2 transfer, τέσσερις είσοδοι και tall-open-drop

#### Πηγές και παγωμένα κριτήρια

Διαβάστηκαν πρώτα τα #54–56 του
`docs/archive/mechanism/intake/intake-debug-log-el.md` και το
`cad/basket-bin-v2/`. Το παλιό αποτέλεσμα ήταν: hood/rear overhang 40 mm
συγκρατεί το pile, αλλά loaded delivery χρειάστηκε rear clearance 120 mm
και controlled launch profile (5/5 αντί wheel-only 2/5). Αυτές οι αρχές
μεταφέρθηκαν χωρίς αλλαγή στο παγωμένο P2-40.

Τα r1/r2/r3 κριτήρια γράφτηκαν πριν από τα αντίστοιχα runs. Για τη νέα
tall-open-drop διατηρούνται επίσης τα αποτυχημένα diagnostics r4/r5:
r4 έφτασε στην ψηλή έξοδο αλλά σφήνωσε μεταξύ X50 και rear wall· r5
σταμάτησε στην απότομη πρώτη κλίση X352. Το τελικό r6 κράτησε αμετάβλητα
τα pass/fail όρια και χρησιμοποίησε ομαλότερη dependent ράμπα.
Frozen r6 SHA-256:
`7577c1ba24f7434ec6a78edd8285208f648af4858e3460906b0fb78a3ff7f239`.

Static calculation: εσωτερικό 287×344 mm, 45 sphere volume 6,774 L,
packing 0,56/0,60/0,64 → mean pile 122,5/114,4/107,2 mm. Με allowance
μίας μπάλας, worst local top Z=242,5 mm. Επιλέχθηκαν open bridge Z=247,
drop X=90 και rim Z=335 mm. LiDAR scan clearance 55 mm. Camera-body to
basket-front X gap 34,9 mm· το forward ground envelope X430…545,
Y±70…120 δεν τέμνεται. Για rise 237 mm: `sqrt(2gh)=2,156 m/s`, άρα
2,695 m/s για 25% velocity margin· run reference 2,70 m/s ή 258 rpm Ø200.

#### Harness, preflight και εντολές

Προστέθηκε μόνο στο isolated bench η `tall_open_drop`, με 335 mm walls και
96 mm uncovered bridge. Δεν υπάρχει entry flap, hood ή liner. Το preload
πέφτει δυναμικά στις δύο ανοικτές πλευρικές λωρίδες, όχι σε final grid.
Το low floor/launch-feed interface παραμένει εξαρτώμενο. Το r6 preflight
ήταν PASS: criteria hash όπως παραπάνω, ball hash
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`,
delivery URDF
`270a1b4b471ff21b6b6b4a2aa3a11ac687aba25b5e28b600afb08bafc6b30175`
και motion URDF
`7a4abe9134be14396263e1e16d89f28d26061646c0add9c3b1df0e4e423c3d48`.
Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 engineering/shared-flywheel-isolated-bench/validate_5c2_p247_r6.sh"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 600 env P2_47_RESULTS_TAG=gazebo_5c2_p247_r6 P2_47_CRITERIA=engineering/shared-flywheel-isolated-bench/criteria_5c2_p247_r6.json P2_47_SURFACE_SPEED=2.7 engineering/shared-flywheel-isolated-bench/run_5c2_p247_case.sh tall_open_drop delivery 0 1"`

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 3600 engineering/shared-flywheel-isolated-bench/run_5c2_p247_r6_campaign.sh"`

#### Αποτελέσματα και σύγκριση

- Tall-open-drop: unloaded 3/3, preload-25 2/3, preload-40 0/3,
  preload-44 0/3. Συνολικά delivery 5/12. Δεν υπήρξε persistent entry
  blockage ή bounce-out. Οι αποτυχίες προήλθαν κυρίως από preload spill
  κατά το uncalibrated dynamic settle και από μη άφιξη της νέας μπάλας στα
  υψηλά φορτία. Motion-45: 0/3, baseline 42…43/45, 8 loss events.
- Ball-contact max 13,4288 A και μέγιστο run mean 6,8046 A, χωρίς contact
  saturation. Whole-run 20 A παραμένει spin-up.
- High hood: 3/15, μόνο unloaded· motion loss 1 ανά repeat. Lip/light
  cover: 3/12, μόνο unloaded, blocked 3/3 στα 40/44. Το επιμολυσμένο
  lip preload-40/r3 μετακινήθηκε σε `_contaminated_overlap` και
  επαναλήφθηκε καθαρά: FAIL. One-way flap: 0/6 και απορρίφθηκε.

Καμία εκδοχή δεν περνά το πλήρες frozen gate. Η tall-open-drop είναι η
απλούστερη και σχετικώς καλύτερη, άρα προηγείται μόνο για το 1:1 mock-up.
Δεν υπάρχει print release ούτε ισχυρισμός πραγματικής χωρητικότητας 45,
επειδή basket/ball–ball contacts είναι native DART και αβαθμονόμητα.
Η πλήρης φυσική προδιαγραφή βρίσκεται στο
`docs/mechanism/shared-flywheel-basket-p247-el.md`.

Αποτελέσματα:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p247_r6/condition_matrix.csv`
και `summary.json`. Μετά από κάθε σειρά επιβεβαιώθηκε κενό `docker ps`.

Δεν έγινε 5c-3, 5d, STL, commit, push ή PR. Κατάσταση:
`P2_47_DIAGNOSTIC_FAIL_PAUSED_BEFORE_5C3_5D`.

### P2-48 — 40 υποχρεωτικές / 45 best-effort και τυπωμένο panel basket

#### Παγωμένη απόφαση και μέθοδος

Με απόφαση χρήστη, το καλάθι σχεδιάζεται για 45 μπάλες αλλά η φυσική
αποδοχή απαιτεί 40. Οι 41–45 είναι best-effort και η απώλειά τους δεν είναι
FAIL εφόσον παραμένουν ≥40, δεν υπάρχει βλάβη και η είσοδος συνεχίζει να
λειτουργεί. Επιλέχθηκε η απλή P2-47 `tall-open-drop`, χωρίς flap/liner. Δεν
έγινε νέο Gazebo: η ball–ball επαφή DART δεν είναι βαθμονομημένη και το
καλάθι θα είναι το ίδιο το φυσικό δοκίμιο.

Εντολή static sizing:

`python3 engineering/chassis-strength-bench/run_phase2_p48_basket.py`

Αποτέλεσμα:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/p248_basket_static.json`.
Η εσωτερική κάτοψη είναι 287×344 mm, η μπάλα Ø66 και η σάρωση packing
0,56/0,60/0,64. Για 40 μπάλες ο solid volume είναι 6,021 L και το mean pile
108,9/101,6/95,3 mm· με μία επιπλέον διάμετρο και floor-high Z54 η δυσμενής
κορυφή είναι Z228,9 mm. Για 45: 6,774 L, 122,5/114,4/107,2 mm και Z242,5 mm.

Τα εναλλάξιμα bridge surfaces είναι:

- Z247: margin 40/45 = 18,1/4,5 mm, άρα FAIL του υποχρεωτικού 20 mm.
  Απαίτηση με 25% velocity margin 2,695 m/s ή 257,4 rpm Ø200.
- Z260: 31,1/17,5 mm, PASS και **reference**. Απαίτηση 2,768 m/s ή
  264,3 rpm. Surface/ball-top clearance προς rim Z335: 75/9 mm· προς
  LiDAR Z390: 130/64 mm.
- Z275: 46,1/32,5 mm, 2,850 m/s ή 272,1 rpm. Η κορυφή της διερχόμενης
  μπάλας είναι 6 mm πάνω από το rim, άρα μένει διαγνωστικό fallback με
  αυξημένο rebound/spill risk.

#### Εξαρτώμενο CAD και printability

Προστέθηκε το
`cad/modular-test-chassis/shared-flywheel-basket-p248-panels.scad`, χωρίς
STL export. Το `part` selector καλύπτει assembly, wall/floor quadrants,
common bridge, tails 247/260/275, seam straps, corner angles, eight M4
mount ears, port frame και blind plate. Στον τελικό έλεγχο προστέθηκαν δύο
ρητοί printable selectors `front_lower_left/right`, ώστε η assembled θύρα
να μη χρειάζεται χειροκίνητη αφαίρεση από generic end panel. Και οι 20
τελικοί selectors μεταγλωττίστηκαν σε προσωρινά CSG χωρίς OpenSCAD error
(κανένα STL δεν διατηρήθηκε ή εκδόθηκε). Η μεγαλύτερη κύρια πλάκα είναι
175×146,5 mm, άρα χωρά σε bed 256 mm.

CAD syntax/selector check (το `<selector>` επαναλήφθηκε για τους 20
δηλωμένους selectors):

`/snap/bin/openscad-nightly -D 'part="<selector>"' -o /tmp/p248-<selector>.csg cad/modular-test-chassis/shared-flywheel-basket-p248-panels.scad`

Πλαϊνά: clear grid opening 38 mm ≤40. Βάση: διαμήκη νεύρα στη φορά +X,
clear opening 18 mm ≤20 και συμπαγής κεντρική λωρίδα 86 mm. Η βάση πέφτει
από Z54 πίσω σε Z42 εμπρός (2,345°). Ο κάνναβος εκτιμάται 692 g έναντι
1.319 g solid, εξοικονόμηση 626 g/47,5%, με ±15% μέχρι slicing και χωρίς
μεταλλικά fasteners. Τα renders είναι
`p248-basket-panels-assembly.png` και `p248-basket-panels-side.png`.

#### Feed interface και φυσικό gate

Η χαμηλή θύρα είναι X278/Y0/Z42, clear 80×80 mm, με frame 120×120 και
τέσσερις M4 θέσεις ±52 mm. Περνά μία Ø66, όχι δύο δίπλα-δίπλα. Η removable
blind plate μπαίνει στις capacity/retention δοκιμές και αφαιρείται μόνο για
flow. Από κέντρο μπάλας `[278,0,75]` προς το frozen moving rear mouth:
+15° → `[329,2,0,59,1]`, μήκος 53,6 mm/κλίση 17,3°· +20° →
`[331,5,0,54,1]`, 57,4 mm/21,3°. Είναι interface/keep-out, όχι feeder ή
singulator· αυτά μένουν στη 5d.

Η φυσική δοκιμή χρησιμοποιεί το ίδιο το τυπωμένο καλάθι, tails Z260/275
(Z247 μόνο διαγνωστικά), preloads 25/35/39/44 και ≥5 νέες παραδόσεις ανά
συνθήκη. Τα 25→26, 35→36, 39→40 απαιτούν 5/5 και μηδενική απώλεια από τις
πρώτες 40. Το 44→45 είναι diagnostic με αποδεκτό τελικό count ≥40. Με blind
plate ακολουθεί rocking και drive/brake/turn με 40 και 45. Χωρίς blind
plate ελέγχεται gravity flow 1/5/10 μπαλών προς τη θύρα, χωρίς ισχυρισμό
singulation.

Για περίπου 2,7 m/s σε κεκλιμένη σανίδα, thin-shell energy δίνει 0,619 m
lossless κατακόρυφο ύψος. Αρχικό setup: 0,70 m σε μήκος 1,20 m (35,7°),
μέτρηση video στα τελευταία 300 mm και ρύθμιση ύψους. Η πλήρης προδιαγραφή,
BOM και πίνακας βρίσκονται στο
`docs/mechanism/shared-flywheel-basket-p248-el.md`.

Δεν έγινε Gazebo, STL release, commit, push ή PR. Κατάσταση:
`P2_48_STATIC_CAD_COMPLETE_BASKET_DEPENDENT_PENDING_PHYSICAL_TEST`.

### P2-49 — απόρριψη εσωτερικής ράμπας και πλήρης διαχωρισμός διαδρομών

#### Διόρθωση προηγούμενου συμπεράσματος

Η P2-48 διάταξη απορρίφθηκε λειτουργικά: η ράμπα μέσα στον εμπρός χαμηλό
όγκο μπορεί να θαφτεί πρώτη, ο όγκος/νεκρός χώρος της δεν είχε αφαιρεθεί
και συνέπιπτε με τη launch-feed θύρα. Αυτό είναι πιθανή κύρια αιτία της
μόνιμης κατάληψης εισόδου στα P2-46/47. Διατηρούνται μόνο οι κανόνες
panel/grid, όχι η λειτουργική γεωμετρία P2-48.

Static command:

`python3 engineering/chassis-strength-bench/run_phase2_p49_separated_basket.py`

Αποτέλεσμα:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/p249_separated_basket_static.json`.

Νέα storage polygon: πίσω X−15, πλαϊνά έως X278/Y±172, αλλά κεντρική
εσοχή outer 100 mm από X278 και tapered clear throat 80 mm στο divider
X100. Η ράμπα βρίσκεται μέσα σε αυτή την εσοχή, εξ ολοκλήρου έξω από τον
αποθηκευτικό όγκο. Το feed channel βρίσκεται κάτω από τη ράμπα. Ο πάτος
είναι piecewise funnel 4,5° κατά X και 2° προς το κέντρο: το πίσω τμήμα
πέφτει προς +X, ενώ τα δύο εμπρός wings προς −X και προς Y0. Τα λοξά
fairings X145→100 δεν δημιουργούν re-entrant corner· η ελάχιστη δίοδος
80 mm αφήνει 14 mm έναντι Ø66.

#### Καθαρός όγκος και packaging

Reference full rectangle 31,217 L. Η εξωτερική ramp/channel bay αφαιρεί
ρητά 5,393 L και μένουν 25,824 L gross. Αφαιρέθηκαν επιπλέον 0,55 L
panel/seams/fasteners, 0,25 L fairings και 0,35 L gate keep-out:
**24,674 L clean net**. Στο packing 0,56 οι 40/45 απαιτούν 10,752/12,096 L.
Η μεγάλη θεωρητική volumetric capacity δεν είναι functional claim.

Με additional projected-area penalty 3.500 mm², worst local top είναι
Z261,1 για 40 και Z277,9 για 45. Το high collection sill Z285 αφήνει
23,9 mm — PASS του υποχρεωτικού ≥20 — και 7,1 mm αντίστοιχα. Rim Z360
μένει 30 mm κάτω από το LiDAR Z390. Δεν απαιτείται rear extension: rear
FIT0403/Ugreen/Parkside gaps παραμένουν 32/27,5/82,5 mm και η εξωτερική
πρόσβαση Parkside δεν αλλάζει.

#### Μοναδικό rear mouth και selector

Το module-mounted common nose ακολουθεί το μοναδικό rear mouth:
−3° floor `[324,7,0,45,1]`, +15° `[329,2,0,26,1]`, +20°
`[331,5,0,21,1]` mm. Η παλιά ramp τιμή X330/Z≈31 αντικαθίσταται τοπικά,
ώστε να μην υπάρχει step. Το selector envelope είναι X225…335/Z32…130.

- Collect: low-feed shutter κλειστό, upper branch ανοικτό, mouth→ramp.
- Transport: και τα δύο shutters κλειστά.
- Launch: upper-ramp shutter κλειστό, low feed ανοικτό, channel→mouth.

Η collection surface ανεβαίνει 45,1→285 mm. Lossless απαιτεί 2,169 m/s·
με 25% margin 2,711 m/s ή 259 rpm Ø200. Actuator, positive stops, sensing,
timing/interlock και singulation μένουν 5d.

CAD:
`cad/modular-test-chassis/shared-flywheel-basket-p249-separated.scad`.
Οι τέσσερις όψεις `section_collect`, `section_launch`, `top_flow`,
`assembly` μεταγλωττίστηκαν σε προσωρινά CSG χωρίς error. Παράχθηκαν μόνο
τα PNG `p249-section-collection.png`, `p249-section-launch.png` και
`p249-top-flow.png`, όχι STL.

CAD check template, με `<view>` μία από τις τέσσερις τιμές:

`/snap/bin/openscad-nightly -D 'view="<view>"' -o /tmp/p249-<view>.csg cad/modular-test-chassis/shared-flywheel-basket-p249-separated.scad`

Πλήρης αναφορά:
`docs/mechanism/shared-flywheel-basket-p249-el.md`.

Δεν έγινε Gazebo, STL release, commit, push ή PR. Κατάσταση:
`P2_49_STATIC_CAD_SEPARATED_PATHS_PASS_PENDING_PHYSICAL_TEST_AND_5D_GATE_MECHANISM`.

### P2-50 — one-ball common-throat → sill Z285

#### Frozen criteria πριν από run

Γράφτηκε πρώτα το
`engineering/shared-flywheel-isolated-bench/criteria_5c_p250.json`
(SHA-256 `10b58d931766507790a533f31e5e318108085c2eadb68ad69737b0bc78e79bcb`).
Matrix: surface speed 2,8/3,0/3,2 m/s, Y=0/±50 mm, approach 0,6 m/s,
ball–tread μ=0,6, ground μ=0,7, μία calibrated μπάλα και μία επανάληψη.
Hard gates: sill X≤110/Zcenter≥305/|Y|≤20 mm, sill speed ≥0,30 m/s,
transition gap ≤20 mm, ramp |Y|≤40 mm, current ≤20 A, torque ≤0,62 N·m,
μηδενική launcher–frame/ground επαφή και finite state.

Container image:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.
Ball model hash:
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`.

Αρχικό preflight command:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "export P2_50_IMAGE_DIGEST=<digest>; timeout 300 engineering/shared-flywheel-isolated-bench/validate_5c_p250.sh"`

#### P2-50/r1 — polyline FAIL και αιτία

Η πρώτη συνθήκη 2,8 m/s/Y0 πέρασε το nip στο t=2,561 s αλλά δεν έφτασε
στο τότε ορισμένο transition-exit gate X≤325 ούτε στο sill. Η μπάλα
επέστρεψε στο X≈410/Z42 mm. Η
piecewise μετάβαση έδωσε diagnostic DART peak 2.304 N. Επειδή η τιμή δεν
είναι βαθμονομημένο structural load, χρησιμοποιήθηκε μόνο για root cause:
η απότομη αλλαγή κλίσης χτυπούσε και επέστρεφε τη μπάλα. Η αρχική matrix
σταμάτησε όπως απαιτήθηκε.

#### P2-50/r2 — R125 και εντοπισμός κενού πλευρικών οδηγών

Πριν την επανάληψη γράφτηκε το `criteria_5c_p250_r2.json`, SHA-256
`808515d406443303f518fa7821f2127ef7461bab8f6d4c04ea28429fc20d2def`.
Η κάτω μετάβαση έγινε τόξο R125, chords 8°, εφαπτόμενο σε περίπου 48°.
Το κεντρικό 2,8 m/s είχε transition peak 16,7 N diagnostic και bounce gap
17,25 mm, αλλά σταμάτησε πριν το sill. Η πλήρης 3×3 σειρά έδωσε 0/9. Οι
±50 μπάλες έφταναν στην αρχή του R125 ήδη κεντραρισμένες σε περίπου ±4 mm,
αλλά στην επιστροφή έφευγαν από τα μεγάλα κενά μεταξύ των τριών μόνο rail
fixtures έως |Y|≈143 mm. Αυτό ήταν ασυνέπεια bench/CAD.

#### P2-50/r3 — τελικό R125 με συνεχείς οδηγούς

Πριν το run γράφτηκε το `criteria_5c_p250_r3.json`, SHA-256
`d3e963b428148a2e90b2a93a5dccf286511b34ec7ed25d6ff62c9501c63497e1`.
Preflight PASS: 35 required collisions, URDF SHA-256
`a259bce4cb17af401ed16738b5d6d93a3791641138569e4ec8600183399cc261`.

Case command template:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 engineering/shared-flywheel-isolated-bench/run_5c_p250_r3_case.sh <speed> <lateral>"`

Μετά από κάθε run το `docker ps` ήταν κενό. Αποτελέσματα:

| surface | Y | ramp-base | R125-exit | apex X/Z | bounce | max |Y| | Imax | gate |
|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 2,8 | −50 | 1,696 | 1,181 | 234,9/184,4 | 17,1 | 11,6 | 13,71 | FAIL |
| 2,8 | 0 | 1,874 | 1,538 | 168,2/259,0 | 17,2 | 0,0 | 11,16 | FAIL |
| 2,8 | +50 | 1,696 | 1,171 | 234,1/185,3 | 17,1 | 11,6 | 13,71 | FAIL |
| 3,0 | −50 | 1,549 | 0,710 | 307,3/103,4 | 17,7 | 12,3 | 13,67 | FAIL |
| 3,0 | 0 | 1,949 | 1,601 | 150,1/279,3 | 17,1 | 0,0 | 11,50 | FAIL |
| 3,0 | +50 | 1,549 | 0,710 | 307,3/103,4 | 17,7 | 12,3 | 13,66 | FAIL |
| 3,2 | −50 | 1,292 | 0,599 | 300,2/111,3 | 17,0 | 4,0 | 14,04 | FAIL |
| 3,2 | 0 | 1,998 | 1,643 | 144,1/286,0 | 17,2 | 0,0 | 11,36 | FAIL |
| 3,2 | +50 | 1,292 | 0,599 | 300,2/111,3 | 17,0 | 4,0 | 14,04 | FAIL |

Η βάση μετριέται στο X≈390 mm αμέσως μετά το common throat και το exit
στο X≈325 mm. Sill speed δεν υπάρχει επειδή καμία μπάλα δεν έφτασε στο
gate. Όλα τα bounce/lateral/current/torque/contact/finite sub-gates
πέρασαν· απέτυχαν sill reached και sill speed. Contact saturation 0 s,
peak torque 0,435 N·m. Τα primary contact means είναι 1,81…6,68 A· το
analyzer πλέον χωρίζει τις μεταγενέστερες επαφές επιστροφής σε episodes.

Η μη μονοτονική απώλεια των ±50 πάνω στους συνεχείς οδηγούς είναι
contact-model sensitivity του DART και όχι φυσικό συμπέρασμα rpm. Η
κεντρική 3,2 m/s φτάνει μόνο X144/Z286. Η παλιά static τιμή 2,711 m/s
σημαίνεται ως μη επαρκής για wheel-surface acceptance, επειδή δεν
περιλάμβανε τις μετρημένες απώλειες throat/R125.

Ενημερώθηκε το εξαρτώμενο
`cad/modular-test-chassis/shared-flywheel-basket-p249-separated.scad` με
R125 και continuous 20 mm rails. OpenSCAD CSG PASS και render
`p250-section-collection-r125.png`. Δεν έγινε STL.

5d option: cam/έκκεντρο που συνδέει selector με tilt, με collect/transport/
launch detents και spring fail-safe «και οι δύο κλάδοι κλειστοί», ώστε να
μη χρειάζεται δεύτερος actuator. Μετά τη φυσική δοκιμή μπορεί να εξεταστεί
stepped side/rear rim Z≈320, αλλά όχι το local entry tower: πλήρες Z320
αφήνει μόνο 35 mm πάνω από sill Z285 και δεν περνά Ø66.

Πλήρης αναφορά:
`docs/mechanism/shared-flywheel-basket-p250-el.md`. Κατάσταση:
`P2_50_GAZEBO_SINGLE_BALL_FAIL_PAUSED_BEFORE_5C3_5D`.

Δεν έγινε σωρός, 5c-3, 5d, STL, commit, push ή PR. ΠΑΥΣΗ.

### P2-51 — παθητικός ski-jump και βαλλιστική παράδοση

#### Κριτήρια και διορθώσεις γεωμετρίας πριν από τη μήτρα

Η κινούμενη ράμπα αποκλείστηκε. Προστέθηκε παθητικός βατήρας που αρχίζει
στο common throat και αφήνει τη μπάλα να πετά ελεύθερα. Το πρώτο frozen
R125/35° run (`criteria_5c_p251_calibration.json`, SHA-256 `7a13727e…`)
σταμάτησε: επαφή calibrated ball–παγωμένης X300 στο t=2,566 s. Το πρώτο
R75/35° run (`criteria_5c_p251_calibration_r2.json`, `4be1f8ad…`) απέτυχε
επίσης: 3,872 m/s, πραγματική έξοδος 25,98° και contact depth 1,50 mm.

Πριν από κάθε νέο run γράφτηκε ξεχωριστό frozen criteria. Το mapping
35/45/55° (`criteria_5c_p251_geometry_diagnostic.json`, `3646aa4f…`) έδωσε
25,98/39,35/51,00°. Το realized-angle mapping 42/50/60°
(`criteria_5c_p251_realized_angle_diagnostic.json`, `b0ec9ab0…`) έδωσε:

| geometric | actual | speed | LiDAR margin | delivery |
|---:|---:|---:|---:|:---:|
| 42° | 37,98° | 4,409 m/s | 45,3 mm | PASS |
| 50° | 46,28° | 3,837 m/s | 1,9 mm | FAIL |
| 60° | 55,77° | 3,578 m/s | −218,1 mm | FAIL |

Άρα η εξαρτώμενη γεωμετρία κλειδώνει για τη δοκιμή σε 52 mm tangent +
R75/42°, μήκος 107 mm. Το P2-40 δεν άλλαξε. Η κλίση συλλογής παραμένει −3°:
οι −5/−7° αποτυγχάνουν το ήδη παγωμένο moving-ground-clearance gate.

#### Frozen 90-case delivery matrix

Criteria γράφτηκαν πριν το πρώτο run:
`criteria_5c_p251_delivery_matrix.json`, SHA-256
`3f98e17d1748b1ec635467e5d66b2cf9145e80eab5dc8161e7ef568c046ff66d`.
Image digest `sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`,
URDF SHA-256 `025dae6095b486341bddf2cd3959f8c9b397782d40052cef1b7028ac3ba9a271`,
ball SHA-256 `8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`.
Matrix: surface 6/10/14 m/s, tread μ=0,6/0,9, ski μ=0,15/0,25/0,4,
Y=0/±50/±100 mm, approach 0,6 m/s. Hard criteria περιλάμβαναν nip,
takeoff, basket entry, empty-basket retention, LiDAR ≥10 mm, current/torque,
forbidden contacts και finite state.

Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 env P2_51_IMAGE_DIGEST=<digest> engineering/shared-flywheel-isolated-bench/validate_5c_p251_delivery_matrix.sh"`

`engineering/shared-flywheel-isolated-bench/run_5c_p251_delivery_matrix_campaign.sh`

Μετά από κάθε case το campaign απαίτησε κενό `docker ps`. Αποτέλεσμα:
6 m/s 30/30 PASS, 10 m/s 14/30, 14 m/s 9/30, σύνολο 53/90. Οι αποτυχίες
ήταν retention 36, LiDAR 10 και X300 contacts 12, με επικαλύψεις. Καμία
επαφή με κάμερα/αψίδα/LiDAR. Στα 6 m/s η έξοδος ήταν
2,749/3,677/4,556 m/s min/mean/max και το ελάχιστο LiDAR margin 18,88 mm.

Χωρίς νέο Gazebo, τα διατηρημένα telemetry επαναναλύθηκαν με
`reanalyze_5c_p251_delivery.py`. Στα 6 m/s η βάση του βατήρα είχε
4,400–5,082 m/s, η απώλεια ήταν 0,500–1,652 m/s και speed retention
0,625–0,897 (mean 0,783). Contact-window mean current 4,68–12,05 A,
peak 5,54–20,0 A, saturation 0–0,020 s. Η αύξηση σε 10/14 m/s ήταν
μη μονοτονική και χειρότερη· δεν επιλέγεται «βολικό» μ.

#### Εξαρτώμενο CAD, καλάθι και απόφαση

Προστέθηκε χωρίς STL το
`cad/modular-test-chassis/shared-flywheel-basket-p251-ballistic.scad`.
Το καλάθι X−85…278/Y±172 ανακτά τον όγκο της βαθιάς ράμπας, έχει πάτο
περίπου 4,4° προς το ανεξάρτητο χαμηλό feed channel, side rim Z320 και
τοπικό backboard Z375 (15 mm κάτω από LiDAR Z390). Static net 30,20 L,
pile top Z225,3/236,5 για 40/45. Αυτά είναι sizing, όχι capacity claim.
Το mandatory φυσικό gate είναι 40, οι 41–45 best-effort.

Σύγκριση: η πλήρης λεία ράμπα παραμένει 0/9 στο P2-50, χαμηλότερο sill
απαιτεί repackaging, gap 56 mm χρειάζεται TPU/nip physical coupon και η
ηπιότερη βαθιά ράμπα δεν αλλάζει την απαιτούμενη potential energy. Άρα η
R75/42° στα 6 m/s (573 rpm Ø200) είναι η απλούστερη λύση που περνά one-ball.

Πλήρης πίνακας και φυσικό πρωτόκολλο:
`docs/mechanism/shared-flywheel-basket-p251-el.md`. Δεν έγινε loaded pile,
5c-3, 5d, STL, commit, push ή PR. Κατάσταση:
`P2_51_PASSIVE_BALLISTIC_ONE_BALL_PASS_BASKET_DEPENDENT_PENDING_PHYSICAL_TEST`.
ΠΑΥΣΗ.

### P2-52 — static/CAD μηχανική μετάβαση και repackaging Ugreen

Σκοπός: actuator/feedback/stops/selector/singulator interface χωρίς αλλαγή
του παγωμένου P2-40, χωρίς Gazebo, STL, Arduino, robot stack ή electronics.

Εντολές:

`python3 engineering/chassis-strength-bench/run_phase2_p52_transition.py`

`/snap/bin/openscad-nightly -o cad/modular-test-chassis/p252-transition-assembly.png --imgsize=1400,900 --viewall --autocenter --camera=0,0,0,55,0,35,0 -D 'view="assembly"' cad/modular-test-chassis/shared-flywheel-transition-p252.scad`

Παράμετροι: pivot `[387,0,45.837]`, fixed pin `[310,-205,120]`, moving
local `[50,-205,90]`, γωνίες −10/−3/0/+15/+20°, requirement 5,3 N·m και
stop design 8,0 N·m. Αποτελέσματα: effective arm 77,0–91,4 mm, axial
57,0–68,6 N για 5,3 N·m και 86,1–103,6 N ισοδύναμο 8 N·m. −3→+20 travel
35,11 mm. Το X310 διορθώνει την ασυμβατότητα του X320 (87,6 mm στο +20°):
όλο το −10…+20° είναι 97,2…142,0 mm μέσα στο P16-50 97…147 mm.
FIT0186/Tr8×2: 8,37 mm/s theoretical, 4,20 s no-load, 4,5–6 s
design, 12.290 encoder counts. Στο worst force χρειάζονται
0,063/0,088/0,146 N·m για screw efficiency 35/25/15%. Το 1,77 N·m είναι
stall· continuous/thermal rating παραμένει measurement gate. Η γραμμή
actuator μετακινήθηκε από Y−190 σε Y−205: η πρώτη είχε 0,5 mm overlap με
το basket side Y−172, ενώ η νέα δίνει 14,5 mm basket, 32 mm drive-tyre,
24 mm arch και 26,5 mm flywheel nominal clearance χωρίς αλλαγή X/Z leverage.

Από Actuonix documentation επιβεβαιώθηκε: `-S`=internal end switches,
`-P`=potentiometer feedback. Με AS5600 0x36 στο hinge και MPU6050
0x68/0x69 στον D20/D21, το `-P` δεν χρειάζεται για τρεις θέσεις. Baseline
driver είναι το υπάρχον BTS7960 D44/D45/D40 από regulated 12 V rail.

Ugreen: παλιά X−157,5…−42,5 όντως επικαλυπτόταν με basket ως X−85.
Νέα θέση `[-135,0,145]`, size `[46,115,45.5]`: nominal gaps basket 27 mm,
Parkside 10 mm/πλευρά, rear electronics 12 mm. Parkside outboard access
μένει ανοικτή. CG shift −0,607 mm empty και −0,523 mm full45.

Το dependent CAD περιλαμβάνει FIT0186/Tr8 envelope, AS5600 pod, indexed
stops, cam selector state −3 collect / 0 both closed / +15…20 feed και
singulator reserve. OpenSCAD compile PASS. Artifacts:
`results/shared_flywheel_phase2/p252_transition_static.json`,
`cad/modular-test-chassis/shared-flywheel-transition-p252.scad`,
`docs/mechanism/shared-flywheel-transition-p252-el.md`.

Περιορισμοί: nominal-envelope CAD, όχι tolerance proof. Ανοιχτά FIT0186
thermal/current, Tr8 backdrive/backlash, measured Parkside adapter, cam
contamination/timing, cable bend radius, 5d singulator και φυσικό basket
gate. Κατάσταση `P2_52_STATIC_CAD_TRANSITION_PASS_WITH_MEASUREMENT_GATES`.
ΠΑΥΣΗ.

### P2-53 — επανέλεγχος transition με την πραγματική παγωμένη γεωμετρία

Αιτία: το P2-52 CAD είχε τροχούς με άξονα Y και rails Y±235/30 mm, ενώ το
παγωμένο launcher έχει τροχούς Ø200×60 στα Y±129 με άξονες local Z και
rails Y±220/50 mm. Οι πλευρικές προβολές είχαν επίσης +Z προς τα κάτω.
Οι παλιές P2-52 εικόνες, η θέση AS5600 Y216 και όλα τα clearance claims
σημειώνονται **ΑΝΤΙΚΑΤΕΣΤΗΜΕΝΑ**.

Το `shared-flywheel-transition-p252.scad` ξαναχτίστηκε με include των
πραγματικών Phase 1/2 modules: upper plate, wheel/motor, yokes/trunnions,
D5065, frame, X318/X330 arch, LiDAR/camera και τις ακριβείς παραμέτρους
P2-51 basket/R75 ski/feed. Το include guard δεν αλλάζει την αυτόνομη έξοδο
του Phase 1 SCAD. Δεν δημιουργήθηκε νέο envelope για τη μονάδα.

Η πρώτη exact-geometry δοκιμή έδειξε hard intersection του coaxial
FIT0186 Ø37×70 με drive package και arch σε όλο το τόξο. Η baseline
διορθώθηκε χωρίς αλλαγή του P2-40: FIT0186 offset +37 mm Y/+50 mm Z πάνω
σε rocking carrier και short 1:1 toothed belt προς Tr8, moving arm πάνω
από το tread, stop tongue local `[20,−205,115]`, AS5600 board/magnet στα
Y244/Y240,5 και D5065 service-loop anchors Y±180. Η μετάδοση προσθέτει
δύο pulleys, tensioner/guard και measurement gate απόδοσης· το P16 μένει
direct fallback.

Εντολές:

`python3 engineering/chassis-strength-bench/run_phase2_p53_geometry_reaudit.py`

`/snap/bin/openscad-nightly -o cad/modular-test-chassis/p253-transition-side.png --imgsize=1400,900 --viewall --autocenter --camera=0,0,0,0,0,0,1000 -D 'p253_view="side"' -D 'p253_angle=0' cad/modular-test-chassis/shared-flywheel-transition-p252.scad`

`/snap/bin/openscad-nightly -o cad/modular-test-chassis/p253-transition-top.png --imgsize=1400,900 --viewall --autocenter --camera=0,0,0,0,0,0,1000 -D 'p253_view="top"' -D 'p253_angle=0' cad/modular-test-chassis/shared-flywheel-transition-p252.scad`

Οι τομές Y=0 αποδόθηκαν αντίστοιχα με `p253_view="section"` και
`p253_angle=-3/0/20`. Η P2-51 side διορθώθηκε σε `rotate([-90,0,0])`.
Όλες οι side εικόνες έχουν ground datum, labels και `FRONT +X`.

Παράμετροι sweep: −10…+20° ανά 1°, pivot `[387,0,51,837]`, fixed pin
`[310,−205,120]`, moving local `[50,−205,90]`. Μέθοδος: zero-intersection
exact solids και συμμετρική VTK vertex-to-triangulated-surface απόσταση,
με reported numerical uncertainty ±0,5 mm.

Κινηματική: −3→+20 travel 33,976 mm, full P16 spacing
99,044…142,358 mm. Effective arm 74,304/78,511/80,225/87,949/90,195 mm
στις −10/−3/0/+15/+20°. Οι αντίστοιχες δυνάμεις 5,3 N·m είναι
71,33/67,51/66,06/60,26/58,76 N και για 8,0 N·m
107,67/101,90/99,72/90,96/88,70 N.

Και οι 18 nominal zero-intersection έλεγχοι PASS. Ελάχιστα διακένια:
FIT–launcher 11,222 mm (+20°), FIT–drive 8,426 mm, FIT–arch 8,557 mm,
Tr8–launcher/drive 32,708/19,222 mm, arm–launcher 8,000 mm (+6°),
arm–drive 33,423 mm, whole actuator–frame/sensors 54,000/41,352 mm,
actuator–basket/ski/feed 11,557 mm, actuator–power 165,763 mm,
AS5600–launcher/drive 18,818/9,000 mm, cam–launcher/fixed 19,000/21,337
mm, stops–launcher 39,000 mm, cables–launcher/fixed 24,004/11,414 mm.

Αποτέλεσμα: `P2_53_EXACT_GEOMETRY_REAUDIT_PASS_WITH_ARM_FIT_GATE`.
Το 8,000 mm arm–launcher είναι ακριβώς nominal gate και δεν περιλαμβάνει
print tolerance, belt deflection ή cable stiffness· απαιτεί physical fit
check/αύξηση πριν release του εξαρτώμενου arm. Output:
`results/shared_flywheel_phase2/p253_geometry_reaudit.json`. Δεν έγινε STL,
Gazebo, ROS, Arduino/electronics change, commit, push ή PR. ΠΑΥΣΗ.

### P2-54 — clearance margin και ασύμμετρη στρέψη

Σκοπός: dependent CAD/static calculation μόνο, χωρίς αλλαγή του P2-40.
Το moving arm αναδρομολογήθηκε στα local `[−10,−160,68]` και
`[−10,−205,135]`. Το FIT0186 1:1 baseline μεταφέρθηκε σε offset
`[0,+40,5,+60] mm` από το fixed pin και ο belt carrier έγινε Ø14/8 mm.

Εντολή:

`python3 engineering/chassis-strength-bench/run_phase2_p54_margin_torsion.py`

Ο runner εκτελεί μόνο πέντε exact-solid ζεύγη στο −10…+20° ανά 1° και
διαγράφει τα προσωρινά meshes. Αποτελέσματα: arm–launcher 14,000 mm στο
+2°, arm–drive 33,432 mm, FIT–launcher 17,405 mm στο +20°, FIT–drive
10,426 mm και FIT–arch 10,932 mm. Άρα περνούν οι στόχοι 12/10/10 mm.

Το torsion screen χρησιμοποιεί plate-only PETG strip 195×8 mm ανάμεσα στα
trunnions Y±234, `E=1,5 GPa`, χωρίς stiffness credit σε yoke webs/motors.
Μάζα μονάδας 4,60996 kg, actuator Y−205, explicit printed-PETG strength
assumption 30 MPa και 50% gate 15 MPa. Στα 8 N·m:

- −10°: F=107,67 N, trunnions −Y/+Y=105,31/24,09 N μαζί με μισό βάρος,
  twist-X 0,0158°, Δnip 0,071 mm.
- +20°: F=88,70 N, trunnions 93,31/25,01 N, worst twist-X 0,0530°,
  worst Δnip 0,2388 mm.
- Worst conservative combined plate stress 0,4373 MPa, 2,92% του 15 MPa
  gate. PASS για Δnip≤1 mm και stress≤50% assumed PETG strength.

Stop-impact sensitivity: isolated-bench `Iyy=0,0335675 kg·m²`, rate
`23°/8,3 s=0,048365 rad/s`, energy `3,926e−5 J`. Για linear compression
0,1/0,25/0,5 mm, kinetic peak torque 0,09165/0,03666/0,01833 N·m. Worst
combined με 8 N·m preload: Δnip 0,2415 mm και stress 0,4423 MPa. Το preload
κυβερνά· απαιτούνται positive stop και current cut-off. Zero-compliance
impact δεν χρησιμοποιείται ως πεπερασμένο structural load.

Απόφαση: ένας actuator αρκεί στο analytical screen. Παρότι δεν απαιτείται
από το κριτήριο, προτείνεται δεύτερο συμμετρικό stop Y=+205 πριν εξεταστεί
δεύτερος actuator, ώστε στα endpoints να μοιράζεται το preload και να
ακυρώνεται πρώτης τάξης η X-twist. Measurement gates: PETG coupon/layers,
joint compliance, φυσικό Δnip υπό preload και stop compression/cut-off.

Output: `results/shared_flywheel_phase2/p254_margin_torsion.json`.
Κατάσταση `P2_54_CLEARANCE_PASS_TORSION_PASS_SECOND_STOP_RECOMMENDED`.
Δεν έγινε Gazebo, STL, commit, push ή PR. ΠΑΥΣΗ πριν από 5c-3.

### P2-55 — 0α/0β, Gazebo 5c-3 και recorded οπτική επιβεβαίωση

Το 5c-3 χτίστηκε αποκλειστικά πάνω στο frozen P2-51 delivery-matrix setup:
ίδιο P2-40 launcher, P2-51 basket και R75/42° ski-jump, surface speed
6,0 m/s, tread μ=0,6, ramp μ=0,15, approach 0,6 m/s, ground μ=0,7 και
αμετάβλητο `tennis_ball_compliant`. Δεν άλλαξε CAD/envelope/contact fit.

Πριν από physics γράφτηκε το `criteria_5c3_p255.json`, SHA-256
`73437c1ca732536ea6ed3702e84c1c221e7d4acb94d83574718e1abdb62de5ca`.
Το 0α πάγωσε και επαλήθευσε τη γεωμετρία/κριτήρια P2-51. Το 0β επαλήθευσε
δύο ανεξάρτητα calibrated ball models, observer και πλευρική κάμερα με
pose XYZ/RPY `0.55 −1.45 0.38 / 0 0.10 1.5708`, δηλαδή από εμπρός δεξιά,
με world +Z πάνω. Και τα δύο gates PASS. Ball SHA-256
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`,
URDF SHA-256
`025dae6095b486341bddf2cd3959f8c9b397782d40052cef1b7028ac3ba9a271`,
τελικό preflight-world SHA-256
`c424c760d348e948119208f1160724f90114bd23d65e21f051162ebe10d86b78`.

Η πρώτη `empty_y0` προσπάθεια δεν αξιολογήθηκε ως product run: ο observer
φορτώθηκε πριν από τα ξεχωριστά ball entities και έγραψε μόνο CSV header.
Διορθώθηκε μόνο η σειρά SDF insertion στο ήδη δοκιμασμένο 5c-2 pattern
`balls → robot`, επαναλήφθηκε το 0β PASS και μετά ξεκίνησε η τελική
καμπάνια. Το αρχικό state log αντικαταστάθηκε από το τελικό valid run.

Τα hard gates ανά μπάλα ήταν nip crossing, ski takeoff, basket entry,
retention με τελική mean speed ≤0,25 m/s, top Z≤380 mm, modeled current
≤20 A, torque ≤0,62 N·m, finite state και μηδενικό forbidden contact.
Εντολές:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 engineering/shared-flywheel-isolated-bench/validate_5c3_p255.sh"`

`engineering/shared-flywheel-isolated-bench/run_5c3_p255_campaign.sh`

Το campaign επιβάλλει πάντα `BENCH_GUI=false BENCH_RECORD=true` και τρέχει
κάθε acceptance case μέσω `docker/jazzy/run.sh`. Αποτέλεσμα: 2/3 PASS.

- `empty_y0`: PASS. Nip/takeoff/entry στα 2,556/2,576/2,601 s, τελικό
  `[−0,04293,0,00000,0,07963] m`, πρακτικά μηδενική τελική ταχύτητα.
- `empty_y50`: PASS. Nip/takeoff/entry στα 2,581/2,601/2,621 s, τελικό
  `[−0,04293,0,04528,0,07963] m`, mean speed 0,01191 m/s.
- `side_by_side_y100`: FAIL. Και οι δύο ταυτόχρονες μπάλες Y±100 mm δεν
  πέρασαν το nip, δεν έφτασαν ski/basket και κατέληξαν εμπρός στο
  X=1,03219/1,03155 m, Y=−0,03590/+0,03506 m. Δεν υπήρξε forbidden contact,
  nonfinite state ή electrical-limit violation. Η αποτυχία είναι collection
  interaction δύο ταυτόχρονων μπαλών, όχι P2-51 one-ball regression.

Και τα τρία headless cases γράφτηκαν με `gz sim --record-path` στον
αντίστοιχο `<case>/replay/state.tlog` (30,3–31,8 MB). Ακριβείς εντολές:

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh empty_y0`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh empty_y50`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100`

Το `replay_5c3_case.sh` ανοίγει playback μέσω `docker/jazzy/run.sh`, με την
ίδια πλευρική κάμερα σε nip/ski-jump/basket και +Z πάνω. Playback του
`empty_y0` άνοιξε επιτυχώς στο host `DISPLAY=:0` με το `XAUTHORITY` που
περνά το `run.sh`. Εμφανίστηκαν μόνο μη-fatal Qt QuickStart width warnings.
Το test έκλεισε με timeout και το container cleanup ολοκληρώθηκε.

Με `BENCH_GUI=true` το ίδιο case τρέχει live και γράφει αποκλειστικά στο
`live_gui/`, χωρίς summary/criteria inclusion. Η live εντολή ελέγχου ήταν:

`ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=false docker/jazzy/run.sh exec "timeout 240 env BENCH_GUI=true BENCH_RECORD=false engineering/shared-flywheel-isolated-bench/run_5c3_p255_case.sh side_by_side_y100"`

Το live GUI άνοιξε και έδωσε ακριβώς την ίδια αποτυχία/τελικές θέσεις με το
headless run· δεν παρατηρήθηκε GUI/headless απόκλιση ή χαμένη μπάλα μόνο σε
μία από τις δύο λειτουργίες. Aggregate:
`results/shared_flywheel_phase2/gazebo_5c3_p255/summary.json`. Δεν έγινε STL,
commit, push ή PR. Κατάσταση: `P2_55_5C3_TWO_OF_THREE_PASS_SIDE_BY_SIDE_FAIL_RECORDED`.
ΠΑΥΣΗ.

### P2-55 correction — πραγματικά 0α/0β και πλήρες two-ball 5c-3

Η προηγούμενη ερμηνεία των 0α/0β ως preflight και τα one-ball
`empty_y0/empty_y50` αποτελέσματα παραπάνω είναι **ΑΝΤΙΚΑΤΕΣΤΗΜΕΝΑ** και
δεν συμμετέχουν στην τελική P2-55 απόφαση. Η frozen P2-40/P2-51 γεωμετρία,
το contact model και η βαθμονόμηση δεν άλλαξαν.

#### 0α — St-Venant torsion γύρω από τον hinge Y

Το `run_phase2_p54_margin_torsion.py` αντικατέστησε το λανθασμένο
transverse beam-bending screen με St-Venant torsion της PETG upper plate
195×8 mm. Χρησιμοποιήθηκαν
`J=(1/3)b t^3(1−0,63t/b)=3,241984e−8 m^4`, `E=1,5 GPa`, `ν=0,38` και
`G=E/[2(1+ν)]=0,543478 GPa`. Η ροπή βάρους του +Y μισού προς τον actuator
στο Y−205 είναι 0,905939/0,893619/0,751214 N·m στις −3/0/+20°.

Χωρίς το δεύτερο stop, η διαφορική γωνία μεταξύ των wheel planes Y±129
είναι 0,760060/0,749725/0,630250° και το αντίστοιχο Δnip για μοχλό 55 mm
είναι 0,729606/0,719684/0,604997 mm. Και οι τρεις τιμές περνούν το
κριτήριο ≤1 mm. Με ιδανικό συμμετρικό stop Y+205 δεν περνά first-order
torque μεταξύ των wheel planes, οπότε το analytical differential και το
Δnip είναι 0. Αυτό δεν πιστώνει ανοχές, άνιση κατανομή φορτίου ή joint
compliance· παραμένουν physical measurement gates. Νέα κατάσταση:
`P2_54_CLEARANCE_PASS_ST_VENANT_TORSION_PASS_SECOND_STOP_MANDATORY`.

#### 0β — υποχρεωτικό συμμετρικό stop και πλήρες geometry sweep

Στο `shared-flywheel-transition-p252.scad` υπάρχουν πλέον moving/fixed
stop στοιχεία στα Y−205 και Y+205, και στα δύο endpoint ranges −10…−3°
και +15…+20°. Είναι mandatory baseline, όχι δεύτερος actuator. Μετά τις
P2-54 αλλαγές moving arm/FIT0186 ξαναέτρεξε όλο το −10…+20° sweep και όλα
τα 18 exact-solid pairs του `run_phase2_p53_geometry_reaudit.py`.
Όλα έχουν μηδενική τομή. Τα νέα ελάχιστα είναι: arm–launcher 14,000 mm,
arm–drive 33,432 mm, FIT–launcher 17,405 mm, FIT–drive 10,426 mm,
FIT–arch 10,932 mm, Tr8–launcher/drive 32,708/19,222 mm,
actuator–frame/sensors 54,000/31,733 mm, actuator–basket 11,557 mm,
AS5600–launcher/drive 18,818/9,000 mm, stops–launcher 39,000 mm και
cables–launcher/fixed 24,004/11,414 mm. Και οι πλευρικές όψεις κρατούν
+Z πάνω. Δεν παρήχθη STL.

#### Corrected 5c-3 — δύο μπάλες σε κάθε acceptance case

Τα criteria παγώθηκαν πριν το run με status
`FROZEN_BEFORE_CORRECTED_TWO_BALL_P2_55_MATRIX` και SHA-256
`1e05c0d67316babab89b915bc8237a839e68298ef17b82cafdf9d68df4a397d1`.
Το preflight PASS έδωσε ball/URDF/world SHA-256
`8715ce705508cc8c6428126a4063e1846f43d46c780eef5274780ee1cbb7356f`,
`025dae6095b486341bddf2cd3959f8c9b397782d40052cef1b7028ac3ba9a271`
και `c6af82828043d4b449044433b741918373aca4767e1597b6ae9395bca48c2b26`.

Το headless acceptance matrix είχε 12 conditions: σε σειρά στο Y0 με
clear gap 0/50/150/300 mm και δίπλα-δίπλα Y±50/Y±100 mm, όλα σε tread
μ=0,6 και 0,9. Αποτέλεσμα 8/12 PASS:

- Και τα οκτώ σε σειρά πέρασαν nip, ski-jump, basket entry και retention.
  Στο μ=0,6 τα rpm drops ανά μπάλα για gaps 0/50/150/300 ήταν αντίστοιχα
  35,093/35,960, 35,093/35,368, 35,093/34,387 και 35,093/35,199 rpm· οι
  ανακτήσεις στο 95% ήταν 8/12, 14/12, 8/9 και 8/8 ms. Οι exit speeds
  ήταν 3,187/2,443, 3,187/3,215, 3,187/3,118 και 3,187/4,145 m/s.
- Στο μ=0,9 τα αντίστοιχα drops ήταν 37,982/40,993, 37,982/39,783,
  37,982/38,756 και 37,982/39,180 rpm· recovery 12/13, 12/13, 12/10 και
  12/13 ms· exit speeds 4,149/4,108, 4,149/3,792, 4,149/4,195 και
  4,149/3,863 m/s. Η ελάχιστη ταχύτητα εξόδου δεύτερης μπάλας σε passing
  case ήταν 2,443327 m/s (gap 0, μ=0,6).
- Και τα τέσσερα side-by-side conditions απέτυχαν πριν από nip/ski/basket.
  Το ±50 δεν λύνει το πρόβλημα: το pair contact γίνεται νωρίτερα
  (2,426 s, mean base-X ≈0,535 m) από το ±100 (2,566 s, ≈0,533 m) και
  δίνει μεγαλύτερο κοινό rpm drop: 18,539 έναντι 17,041 rpm στο μ=0,6
  και 44,654 έναντι 41,914 rpm στο μ=0,9.

Το governing ελεύθερο πλάτος του wheel nip είναι
`258−2×100=58 mm`, έναντι `2×67=134 mm` για δύο μπάλες: έλλειμμα 76 mm.
Η ανίχνευση pair contact περίπου 146–148 mm εμπρός από το plane X387
δείχνει ότι το ζεύγος κλειδώνει ήδη στην προσέγγιση του στενού λαιμού·
δεν έγινε αλλαγή γεωμετρίας.

#### Functional jam diagnostics και record/replay

Στο `side_by_side_y100`, τόσο η στάση οχήματος για 1 s όσο και η μικρή
όπισθεν και νέα κίνηση εμπρός απέτυχαν να περάσουν οποιαδήποτε μπάλα, σε
μ=0,6 και 0,9. Η measured reverse displacement ήταν 61,032/61,016 mm,
εντός του ζητούμενου 50–100 mm. Άρα το σφήνωμα δεν λύνεται λειτουργικά
από κανένα από τα δύο δοκιμασμένα profiles.

Υποχρεωτικά records υπάρχουν για gaps 0/50, για όλες τις αποτυχίες και
για όλα τα diagnostics. Ακριβείς playback εντολές:

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh series_gap_0 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh series_gap_50 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y50 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh series_gap_0 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh series_gap_50 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y50 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.6 stop_1s`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.9 stop_1s`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.6 reverse_75mm`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh side_by_side_y100 0.9 reverse_75mm`

Το playback smoke test του corrected `series_gap_0/μ0,6` αρχικοποίησε
Gazebo GUI/Qt στο host display μέσω των DISPLAY/XAUTHORITY mounts του
`docker/jazzy/run.sh`, με την πλευρική camera (+Z πάνω, εμπρός δεξιά).
Μετά τον ελεγχόμενο timeout ολοκληρώθηκε το `run.sh` stop interval και το
`docker ps` ήταν κενό. Το `BENCH_GUI=true` παραμένει visual-only σε
ξεχωριστό `live_gui/` path και δεν γράφει acceptance summary.

Aggregate report:
`results/shared_flywheel_phase2/gazebo_5c3_p255/summary.json`.
Τελική κατάσταση:
`P2_55_CORRECTED_TWO_BALL_8_OF_12_PASS_SIDE_BY_SIDE_JAM_NOT_RECOVERED_RECORDED`.
Δεν έγινε STL, commit, push ή PR. ΠΑΥΣΗ.

### P2-56 — D5065-only ανίχνευση και side-by-side απεμπλοκή

Scope: αμετάβλητη P2-40/P2-51 γεωμετρία, ίδιο corrected 5c-3 contact
model/calibration, μόνο instrumentation και control profiles. Όλα τα runs
ήταν headless με record. Δεν τροποποιήθηκε το `5c3_side_gui.config`.

#### Αιτία και Y forces

Προστέθηκαν contact sensors στις ήδη υπάρχουσες ball collisions και
vector force columns στο diagnostic CSV, χωρίς αλλαγή collision geometry,
material ή solver. Στο sample μέγιστης ταυτόχρονης inward wheel load το
πρόσημο είναι σταθερό: η μπάλα στο −Y δέχεται +Y και η μπάλα στο +Y
δέχεται −Y. Άρα επιβεβαιώνεται ότι οι εμπρός επιφάνειες των δύο
κατακόρυφων τροχών σπρώχνουν το side-by-side ζεύγος προς την κεντρική
γραμμή και το φορτίζουν το ένα πάνω στο άλλο.

- Y±50, μ=0,6: wheel→ball `+16,75/−16,76 N`, ball–ball αντίδραση στο ίδιο
  sampled instant `−17,11/+17,11 N`.
- Y±100, μ=0,6: `+16,05/−16,05 N`, ball–ball `−16,24/+16,24 N`.
- Y±50, μ=0,9: `+56,35/−56,35 N`, ball–ball `−7,94/+7,94 N` στο ίδιο
  1 ms sample. Η επαφή είναι δυναμική και το ξεχωριστό ball–ball peak
  εμφανίζεται σε άλλο sample, άρα δεν ερμηνεύεται ως static equilibrium.
- Y±100, μ=0,9: `+52,44/−52,45 N`, ball–ball `−48,51/+48,51 N`.

#### Mega-only detector

Η υπογραφή χρησιμοποιεί μόνο rpm/current ανά D5065. Ανοίγει event όταν η
μέση κοινή πτώση των δύο τροχών είναι >5 rpm για ≥50 ms, περιμένει 15 ms
επιστροφής κάτω από 5 rpm και δηλώνει jam όταν είτε
`peak(|I_L|+|I_R|)<30 A` είτε το ολοκλήρωμα κοινής πτώσης είναι
`>1,70 rpm·s`. Η πρώτη συνθήκη ανιχνεύει τα μ=0,6 jams
(25,62–27,65 A, 0,590–0,691 rpm·s) και η δεύτερη τα μ=0,9 jams
(40 A, 1,785–1,834 rpm·s). Trigger latency από την αρχή του event είναι
περίπου 70–104 ms.

Offline replay των οκτώ accepted sequential cases περιέχει 16 ξεχωριστά
ball-passage events: current sum 40 A και deficit impulse 1,395–1,619
rpm·s. Αποτέλεσμα: 0/8 cases και 0/16 passages false positive. Το όριο
1,70 έχει μικρό simulation margin 0,081 rpm·s και παραμένει physical
calibration gate για Mega sampling, πραγματικά motors και tyre wear.

#### Recorded recovery matrix

Έτρεξαν 12 conditions: τρία profiles × Y±50/Y±100 × μ=0,6/0,9.
Στα wheel-only profiles το vehicle controller πρώτα επιβεβαιώνει
`|v|<0,02 m/s` και μετά αρχίζει το maneuver. Η κίνηση κατά το 0,3/0,4 s
unjam interval ήταν μόνο 0,618–0,701 mm.

- Και οι δύο τροχοί −2 m/s για 0,3 s, μετά 6 m/s: 0/4 runs με μπάλα στο
  καλάθι, 8/8 μπάλες έξω. Τελική απόσταση από nip 1,088–5,269 m,
  detector→end 7,320–7,486 s, peak post-detection current 20,0 A.
- Ασύμμετρα, αριστερός −1 m/s και δεξιός 6 m/s για 0,4 s, μετά και οι
  δύο 6 m/s: 0/4 runs με μπάλα στο καλάθι, 8/8 έξω. Απόσταση
  1,148–5,328 m, 7,320–7,486 s, peak current 20,0 A.
- Vehicle reverse/yaw/re-approach: measured reverse 149,63–152,34 mm και
  yaw 11,956–12,013°. Αποτέλεσμα 0/4 με μπάλα στο καλάθι, 8/8 έξω,
  απόσταση 2,394–6,526 m, 7,320–7,486 s, peak D5065 current
  5,341–5,477 A μετά το trigger.

Άρα ο detector δουλεύει στο συγκεκριμένο bench χωρίς false positive,
αλλά καμία από τις τρεις επιτρεπόμενες functional recovery στρατηγικές
δεν επανεισάγει μπάλα στο P2-51 basket. Δεν έγινε geometry change.

Το `replay_5c3_case.sh` αναγνωρίζει τα έξι νέα case names και κρατά το
playback paused. Ακριβείς εντολές:

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse_both_y50 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse_both_y100 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_asym_left_y50 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_asym_left_y100 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse150_yaw12_y50 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse150_yaw12_y100 0.6`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse_both_y50 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse_both_y100 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_asym_left_y50 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_asym_left_y100 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse150_yaw12_y50 0.9`

`engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p256_reverse150_yaw12_y100 0.9`

Report:
`results/shared_flywheel_phase2/gazebo_5c3_p256/summary.json`.
Κατάσταση:
`P2_56_DETECTOR_PASS_ZERO_FALSE_POSITIVES_RECOVERY_ZERO_OF_TWELVE`.
Δεν έγινε αλλαγή γεωμετρίας, STL, commit, push ή PR. ΠΑΥΣΗ.

### P2-57 — geometric prevention με τυπωμένα cheeks/covers

Scope: το P2-40 core και το P2-51 ski/καλάθι παρέμειναν παγωμένα. Στο
isolated xacro/CAD προστέθηκαν μόνο πέντε replaceable front-part variants:
shield mouths 90/100/110 mm, stagger-open και stagger+shield. Τα stagger
έχουν mouth 140 mm, +Y smooth cheek από X180/Y55 έως X250/Y100 mm, 70 mm
lead και round nose R3. Όλες οι νέες contact επιφάνειες έχουν bench
`mu=0,05`. Το `5c3_side_gui.config` δεν άλλαξε.

Το νέο `run_phase2_p57_cheek_clearance.py` έτρεξε −10…+20° ανά 1° για
όλες τις παραλλαγές: 155/155 μηδενικές exact-solid τομές και ελάχιστο
ground clearance 10,733 mm στο −10°. Δεν παρήχθη STL.

Τα criteria παγώθηκαν πριν από το τελικό campaign με SHA-256
`8234c62f388ff94ac357633cdeb5dd8d2b09bdf1da2784fc2cff981860dbfefe`.
Έτρεξαν headless και recorded 180/180 cases στο ίδιο P2-51/5c-3 contact
model: side Y±35/±50/±100, τα οκτώ sequential regressions και singles
Y0/±20/±40/±60/±80/±100. Όλα έχουν replay και το `docker ps` έμεινε κενό.

- Όλες οι παραλλαγές: side jams 0/6 και sequential 8/8.
- shield m90: capture ±20, side collection 0/12, max rejected +X 4,66579 m.
- shield m100: capture ±40, side collection 0/12, max +X 4,66768 m.
- shield m110: capture ±40, side collection 0/12, max +X 4,66090 m.
- stagger-open: capture ±60, side collection 4/12, max +X 4,93918 m.
- stagger+shield: capture ±40, side collection 4/12, max +X 4,93918 m.

Άρα τα covers καταργούν το P2-56 wheel-face jam, αλλά καμία παραλλαγή δεν
αφήνει την απορριφθείσα μπάλα ελεύθερη με +X≤0,30 m. Καλύτερη συγκριτικά
είναι η `p257_stagger_open`, χωρίς όμως release recommendation επειδή
αποτυγχάνει το rejection gate. Το `replay_5c3_case.sh` αναγνωρίζει τα νέα
`<variant>__<case>` names και παραμένει paused playback. Οι 180 ακριβείς
εντολές είναι στο `gazebo_5c3_p257/replay_commands.txt`.

Αναλυτικό report: `docs/mechanism/shared-flywheel-cheeks-p257-el.md`.
Aggregate: `results/shared_flywheel_phase2/gazebo_5c3_p257/summary.json`.
Κατάσταση:
`P2_57_COMPLETE_NO_VARIANT_PASSES_REJECTION_GATE_STAGGER_OPEN_BEST_DIAGNOSTIC`.
Δεν έγινε STL, commit, push ή PR. ΠΑΥΣΗ.

#### P2-57 τελική απόφαση — archive, όχι baseline

Η P2-57 γίνεται δεκτή μόνο ως συγκριτική δοκιμή. Τα επίπεδα covers στο
X130 και Y±46…±190 mm καταργούν το wheel-face σφήνωμα, αλλά είναι κάθετα
στην κίνηση και λειτουργούν σαν λεπίδα εκχιονιστικού, μεταφέροντας την
απορριφθείσα μπάλα προς +X. Εξωτερική κλίση του cover θα έριχνε τη μπάλα
μπροστά από τον κινητήριο τροχό, του οποίου η εσωτερική ακμή βρίσκεται
περίπου στο Y±250 mm· ασφαλής εκτροπή θα απαιτούσε deflector πλατύτερο από
το όχημα. Παράλληλα, το measured capture μειώθηκε από το baseline ±100 mm
σε ±20…±60 mm.

Απόφαση: καμία P2-57 γεωμετρία δεν υιοθετείται. Οι παραλλαγές
αρχειοθετούνται ως diagnostic artifacts και baseline παραμένουν τα P2-40
cheeks με capture ±100 mm. Η πρόληψη μεταφέρεται στον planner: τα ζεύγη
πρέπει να παρουσιάζονται στο intake σε σειρά. Ο P2-56 detector και η
αναστροφή τροχών παραμένουν fallback μετά από ανίχνευση, όχι γεωμετρική
πρόληψη.

### P2-58 — ανοχή planner για διαγώνια ζεύγη

Scope: baseline P2-40 cheeks (`p244/exposed_shallow`) με το αμετάβλητο
P2-51/5c-3 contact model. Δεν χρησιμοποιήθηκαν οι αρχειοθετημένες P2-57
παραλλαγές και δεν έγινε αλλαγή CAD/xacro geometry. Τα criteria γράφτηκαν
πριν από το πρώτο run με SHA-256
`1d48fd5e755b661c0323665979b4d41ba0d545123493d6413ba3a70a1a15fdfe`.

Τα dx/dy είναι συνιστώσες κέντρο-κέντρο. Από τα 30 grid cells, όσα έχουν
`sqrt(dx²+dy²)<66 mm` είναι φυσικά αδύνατα για τις Ø66 mm bench balls και
σημειώθηκαν N/A αντί να ξεκινήσουν με αλληλοδιείσδυση. Εκτελέστηκαν 63
headless cases: μ=0,6/0,9 με την πρώτη μπάλα στο +dy, η κατοπτρική φορά
στο μ=0,6, και ένα κοινό center case όταν dy=0.

Αποτέλεσμα: 57 PASS, 4 JAM, 2 REJECT.

- `dx=0, dy=70`: REJECT και στις δύο φορές στο μ=0,6, JAM στο μ=0,9.
- `dx=0, dy=100`: JAM και στα δύο μ και στις δύο δοκιμασμένες φορές.
- `dx=35, dy=70/100`: PASS σε μ=0,6/0,9 και στις δύο φορές όπου
  δοκιμάστηκαν.
- `dx=70/100/150`: PASS για ολόκληρο το dy=0/20/35/50/70/100 grid.

Ο P2-56 detector ενεργοποιήθηκε σε 4/4 physical jams και σε κανένα από τα
δύο REJECT του μ=0,6. Όμως ενεργοποιήθηκε επίσης σε 38/57 passing diagonal
cases. Άρα παραμένει χρήσιμος ως fallback, αλλά η αυτόματη αναστροφή πρέπει
να είναι planner-qualified: να καταστέλλεται κατά το αναμενόμενο two-ball
passage window και να επιτρέπεται μόνο αν δεν ολοκληρωθεί η προγραμματισμένη
διπλή διέλευση.

Planner rule: για release χρησιμοποιείται το πρώτο πλήρες, μη-N/A grid row:
`dx≥70 mm` και `|dy|≤100 mm` (όριο του δοκιμασμένου envelope). Και τα 51
εκτελεσμένα σχετικά cases σε μ/φορά πέρασαν. Τα `dx35/dy70…100` είναι χρήσιμο
diagnostic margin, όχι release rule, επειδή δεν υπάρχει πλήρες dy sweep στο
dx35 χωρίς μη-φυσική επικάλυψη. Αν το projected dx είναι <70 mm, ο planner
ευθυγραμμίζει την προσέγγιση ώστε οι μπάλες να μπουν σε σειρά.

Έγιναν recorded επαναλήψεις για όλα τα 6 JAM/REJECT και τα 6 γειτονικά
boundary PASS: 12/12 logs, 580169728 bytes. Το `replay_5c3_case.sh`
αναγνωρίζει τα νέα P2-58 names και κρατά paused playback. Οι ακριβείς
εντολές είναι στο `gazebo_5c3_p258/replay_commands.txt`.

Report: `docs/mechanism/shared-flywheel-planner-tolerance-p258-el.md`.
Aggregate: `results/shared_flywheel_phase2/gazebo_5c3_p258/summary.json`.
Κατάσταση:
`P2_58_PASS_PLANNER_DX70_DY100_ENVELOPE_DETECTOR_NEEDS_PLANNER_QUALIFICATION`.
Δεν έγινε STL, commit, push ή PR. ΠΑΥΣΗ.

#### P2-58 τελική απόφαση — accepted planner envelope

Η P2-58 γίνεται δεκτή. Λειτουργική ερμηνεία: το ζεύγος περνά όταν η
δεύτερη μπάλα βρίσκεται περίπου 35–70 mm ή περισσότερο πίσω από την πρώτη,
ενώ η αποτυχία περιορίζεται στο σχεδόν τέλειο side-by-side (`dx=0`). Για
release ο συντηρητικός κανόνας παραμένει `dx≥70 mm`, με το `dx35` ως
μετρημένο diagnostic margin. Ζεύγη με |dy|>100 mm και εφαπτόμενες μπάλες
σε 30/45/60° μεταφέρονται στη μελλοντική εργασία collection route.

Τα 38/57 D5065-only false triggers απορρίπτουν τα motor signals ως μοναδικό
jam discriminator. Νέο baseline sensing: IR break-beam. Jam ορίζεται ως
D5065 rpm event χωρίς beam crossing μέσα σε βαθμονομημένο χρονικό παράθυρο
`T`; η αναστροφή τροχών παραμένει fallback μετά από αυτή τη συνδυασμένη
επιβεβαίωση.

### P2-59 — 5d-1 μονή εκτόξευση, τροφοδοσία και break-beam

Scope: αμετάβλητο P2-40/P2-51 geometry και ίδιο calibrated 5c-3 contact
model. Τα criteria παγώθηκαν πριν από το πρώτο run με SHA-256
`b3fe914bd63f5570ec4d6752f7d7b93bee8ae1d02f5e29274dc29d9628b8877b`.

Το downstream break-beam επιλέχθηκε στο `base_link` X320/Y±140/Z50 mm.
Offline replay των 87 διαθέσιμων P2-55/56/58 folders έδωσε ξεχωριστούς
pulses σε όλα τα 65 passing two-ball cases, περιλαμβανομένου του gap 0.
Ελάχιστο pulse-to-pulse clear interval 105 ms και rpm-event→beam 30…60 ms.
Με rpm event = κοινό mean deficit >5 rpm για ≥50 ms, το timeout ορίστηκε
T=100 ms: 0/65 false positives και 20/20 logged jams detected. Τα δύο
P2-58 REJECT δεν είναι jams.

Το υπάρχον Mega pin map ήδη διαθέτει D36/D37 στα δύο IR beams. Το D37
ανατίθεται στο X320 downstream beam και το D36 σε feed beam στη θύρα X278,
περίπου Z75 mm. Το AS5600 μοιράζεται D20/D21 I²C χωρίς address conflict με
το MPU6050. Το D7 είναι ελεύθερο στο integrated sketch και εκτός J_MEGA,
αλλά το Mega `Servo.h` απαγορεύεται επειδή ο Timer5 καταργεί PWM στα ήδη
χρησιμοποιούμενα D44/D45/D46. Προτείνεται PCA9685 στο I²C και προαιρετικό
D7 servo-power enable. Firmware δεν άλλαξε.

Προτάθηκε spring-return, fail-closed two-finger escapement 72 mm μέσα στο
P2-52 envelope, με ένα servo/cam και περίπου 60 mm handoff. Release μόνο με
AS5600 14,5…20,5°, όχημα ακίνητο, δύο D5065 ≥95% setpoint για ≥200 ms και
feed beam active. Το σημερινό xacro δεν έχει χωριστό low-feed collision
channel από X278 ως το nip, οπότε το physics launch ξεκινά 50 mm πίσω από
το nip. Το chute/singulator παραμένει physical design gate.

Έτρεξαν headless 48/48 cases: +15/+20°, μ=0,6/0,9, surface speed
12/16/20/24 m/s και τρία repeats, με αντίστροφη φορά τροχών και ακίνητο
όχημα. Όλα πέρασαν τα mechanical checks, χωρίς cheek/arch/sensor contacts.
Peak current/torque ήταν 20 A/0,62 N·m και rpm drop 28,55…39,11 rpm. Όμως
η exit speed κορέστηκε στα 6,23…6,27 m/s για μ=0,6 και 7,66…7,69 m/s για
μ=0,9. Το gain 20→24 m/s ήταν μόλις 0,0024…0,0052 m/s, άρα surface speed
για 12–14 m/s δεν μπορεί να εξαχθεί και το performance target δεν περνά.

Το static screen δίνει 1146…2292 rpm, thin-ring TPU stress
0,173…0,691 MPa και free expansion έως 6,912/2,765/1,382 mm για
E=10/25/50 MPa στα 24 m/s. Ο D5065 έχει υπολογιστικό voltage margin 5,73 V
ακόμη και στα 15 V/20 A, αλλά δεν υπάρχει thermal release. Η πραγματική
μεταλλική hub/retention/balance δεν έχει οριστεί και απαγορεύεται printed
torque hub. Το primitive aluminium-hub envelope είναι 78,20 g και στα
24 m/s κάθε 1 g imbalance στο R100 δίνει 5,76 N (12,96 N στο 1,5× rpm),
ενώ η διεπαφή μεταφέρει έως 0,62 N·m. Χωρίς controlled drawing δεν μπορεί
να δοθεί hub stress/expansion, συνεπώς απαιτείται physical 1,5× overspeed
gate.

Recorded ως οριακά τα τέσσερα surface-24/repeat-1 cases. Playback:

`engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 15 0.6 24 1`

`engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 15 0.9 24 1`

`engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 20 0.6 24 1`

`engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 20 0.9 24 1`

Το playback ξεκινά paused μέσω `docker/jazzy/run.sh` και χρησιμοποιεί χωρίς
μεταβολή το διορθωμένο `5c3_side_gui.config`.

Report: `docs/mechanism/shared-flywheel-launcher-p259-el.md`.
Artifacts: `results/shared_flywheel_phase2/gazebo_5d1_p259/`.
Κατάσταση:
`P2_59_BEAM_DETECTOR_PASS_MECHANICAL_RUNS_48_OF_48_LAUNCH_TARGET_NOT_REACHED_HUB_AND_FEED_PHYSICAL_GATES_OPEN`.
Δεν έγινε αλλαγή frozen geometry, firmware, STL, commit, push ή PR. ΠΑΥΣΗ.

#### P2-59 τελική απόφαση — baseline accepted

Η P2-59 γίνεται δεκτή. Baseline κλειδώνουν: downstream IR break-beam στο
`base_link` X320/Y±140/Z50 mm, combined jam timeout T=100 ms με 0/65 false
positives και 20/20 logged jams, D36/D37 για feed/downstream beams, PCA9685
αντί Mega `Servo.h`, και spring-return fail-closed two-finger escapement.
Το launch requirement 12–14 m/s παραμένει δεσμευτικό και δεν έκλεισε στο
P2-59: το 6,2/7,7 m/s plateau, μαζί με drop μόνο 30…40 rpm, μεταφέρει το
root cause από motor energy σε nip traction/contact length.

### P2-60 — diagnostic launch saturation

Το P2-60 έμεινε diagnostic-only: καμία αλλαγή στο frozen CAD/xacro
baseline. Τα generated worlds μετέφεραν μόνο τους wheel joints για
compression 8/11/14 mm. Σταθερά +20°, surface 20 m/s, μ=0,6/0,9/1,2 και
feed=0/2/4 m/s. Η μπάλα τοποθετήθηκε 0,5 mm μέσα από το first-contact plane
μόνο κατά το release, χωρίς pre-spin contact load. Criteria SHA-256:
`83cb9604d6596eb87613ee0ea462761f89bbac2563ed9a0f18130b893be346b7`.

Αναλυτικά, με `N=107309,294·δ^1,5`, το 8 mm nip έχει contact length
64,746 mm, peak N 27,147 N/τροχό και `∫Ndx=1,0314 N·m`/τροχό. Από
`m·dv/dt≤2μN` προκύπτει
`vout²≤vin²+(4μ/m)∫Ndx`: 6,533 m/s για μ=0,6 και 8,001 m/s για μ=0,9
από ηρεμία. Το Gazebo έδωσε 6,209/7,648, μόνο 4,96/4,41% χαμηλότερα.
Στο 8 mm/μ=0,9 μετρήθηκαν friction-capacity impulse 0,5038 N·s, actual
tangential 0,4993 N·s, opposing normal −0,0437 N·s και Δp 0,4432 N·s.
Άρα DART point-contact/damping/normal-vector effects εξηγούν τη μικρή
διαφορά και το Coulomb budget χρησιμοποιείται σχεδόν ολόκληρο.

Ολοκληρώθηκαν 27/27 headless exits. Mean main-effect ranges:
compression +5,289 m/s, μ +3,416 m/s, feed +0,755 m/s. Αποτελέσματα από
ηρεμία:

- 8 mm: 6,209 / 7,648 / 8,803 m/s για μ=0,6/0,9/1,2,
- 11 mm: 8,519 / 10,416 / 12,034 m/s,
- 14 mm: 10,765 / 13,128 / 15,146 m/s.

Εννέα cases έφτασαν ≥12 m/s. Το ελάχιστο αριθμητικό είναι 11 mm/μ=1,2,
αλλά το μ=1,2 δεν είναι calibrated. Το ελάχιστο μέσα στο ήδη χρησιμοποιημένο
μ≤0,9 envelope είναι 14 mm/μ=0,9: 13,128/13,262/13,713 m/s για feed
0/2/4, rpm drop 76,17 rpm στο feed0. Το standalone 58 mm/μ=0,9/feed3
plateau 8,2475 m/s συμφωνεί με παρεμβολή P2-60 ≈8,23 m/s, επιβεβαιώνοντας
κοινό traction/contact-duration root cause.

Για collection, το 14 mm αυξάνει peak N 2,315×, contact length 1,315× και
ideal friction work 3,036×. Με κοινό nip αναμένονται μεγαλύτερο capture/stall
load, περίπου 1,8–2× rpm drop και takeoff προς το 6 m/s wheel-speed cap,
οπότε θα χρειαζόταν collection surface περίπου 3,5…4,5 m/s και πλήρες 5c
rerun. Δεν έγιναν νέα 5c runs.

Δύο επιλογές καταγράφηκαν:

- κοινό gap 52 mm: άξονες Y±126, η απλούστερη γεωμετρική αλλαγή, αλλά
  αλλάζει P2-40 σε όλους τους modes και ξανανοίγει P2-51/όλο το 5c,
- mode-adjustable 58→52 mm: συμμετρικά pods ±3 mm με conjugate cam στον
  υπάρχοντα P2-52/P16 hinge, hard launch stops και spring-open fail-safe,
  χωρίς δεύτερο actuator. Διατηρεί το accepted collection nip.

Προτείνεται το adjustable nip ως μικρότερη system-level αλλαγή για το
δεσμευτικό 12–14 m/s. Απαιτεί νέο local CAD/strength/cam gate, P2-53/P2-54,
collection return-to-58 regressions P2-55/56/58, νέο P2-59/P2-60 με repeats
και έπειτα 5d-2. Release προϋποθέτει measured dynamic μ≥0,9. Αν το φυσικό
μ είναι ≈0,6, κανένας δοκιμασμένος έως 14 mm συνδυασμός δεν περνά και το
compression sweep πρέπει να επεκταθεί πριν σχεδιαστεί hardware.

Recorded boundary cases: 5/5. Replay μέσω
`replay_5d1_p260_case.sh <compression> <mu> <feed>`· το
`5c3_side_gui.config` δεν άλλαξε.

Report: `docs/mechanism/shared-flywheel-launch-saturation-p260-el.md`.
Aggregate: `results/shared_flywheel_phase2/gazebo_5d1_p260/summary.json`.
Κατάσταση:
`P2_60_DIAGNOSTIC_COMPLETE_TRACTION_BOUND_CONFIRMED_ADJUSTABLE_52MM_LAUNCH_NIP_RECOMMENDED_PHYSICAL_MU_GATE_OPEN`.
Δεν έγινε αλλαγή frozen geometry, STL, commit, push ή PR. ΠΑΥΣΗ.

### P2-61 — διόρθωση διαμετρικής παραμόρφωσης ανά επαφή

Το suspected calibration mismatch επιβεβαιώθηκε σε πραγματικό Gazebo
two-platen bench με το ίδιο plugin και στις δύο επαφές. Το historical
`law(p)` έδωσε 13,000001 mm forward deformation από preload 15,57 N σε
95,64 N. Το corrected `law(2p)` έδωσε 6,500001 mm. Στο frozen nip 58 mm,
`p=4 mm` και η δύναμη ανά τροχό διορθώνεται από 27,147 σε 76,784 N.

Το plugin απέκτησε ρητό `diametral_compression_scale=2.0` για wheel, ground
και platen, ενώ η telemetry κρατά χωριστά physical penetration/rate και law
compression/rate. Δεν άλλαξαν K, exponents ή platen fit. Επαναβαθμονομήθηκε
μόνο το ground damping σε 2356,929199 N·s/m^1.5. Gazebo rebound 0,25 ms:
1,410390 m. Το historical 1/0,5/0,25 ms numerical gate αποτυγχάνει (2,74%
spread) λόγω ~4,63 ms contact και διατηρείται ως failed evidence. Το
predeclared 0,25/0,125/0,0625 ms gate περνά με 0,043% spread. Production
timestep κλειδώνει σε 0,25 ms.

Corrected launch στο σημερινό 58 mm nip, +20°, feed 0/2:

- μ=0,6: 10,418…10,608 m/s σε surface 12…24 m/s, άρα δεν κλείνει 12 m/s,
- μ=0,9: 11,528…11,654 στα 12 m/s και 12,776…12,929 στα 16 m/s· στα
  20/24 m/s παραμένει ~12,93 m/s.

Άρα ο δεσμευτικός στόχος 12–14 m/s κλείνει χωρίς αλλαγή geometry με
μ=0,9/surface16. Η P2-60 σύσταση για 52 mm/ρυθμιζόμενο nip
**ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ** και αποσύρεται. Παραμένει physical gate η επιβεβαίωση
dynamic tread μ≥0,9.

Στο corrected P2-51 delivery matrix, surface6 έδωσε 11/30. Η πρώτη
προδηλωμένη χαμηλότερη command, 5 m/s, έδωσε 30/30 με ίδιο frozen ski/basket.
Με αυτήν την collection speed, οι 8 σειρές έμειναν 8/8. Τα P2-58 boundary
dx35/dy70/100 πέρασαν 8/8 και dx0/dy70/100 απορρίφθηκαν 8/8. Τα
side-by-side ±50 έμειναν 0/2 basket και παρήγαγαν rpm event χωρίς beam.

Το corrected offline detector score είναι 0/16 false positives και 2/2
side-by-side detections, με beam X320/Z45, event→beam 30…50 ms, ελάχιστο
clear pulse interval 110 ms και T=100 ms. Γράφτηκαν headless records για
τα δύο side-by-side failures, και για όλα τα 16 P2-58 boundary cases, καθώς
και τέσσερα launch boundary cases. Το `replay_5c3_case.sh` επεκτάθηκε μόνο
με τα νέα P2-61 names· το `5c3_side_gui.config` δεν άλλαξε.

Report: `docs/mechanism/shared-flywheel-contact-correction-p261-el.md`.
Κατάσταση:
`P2_61_CONTACT_MODEL_CORRECTED_PLATEN_REBOUND_CONVERGENCE_PASS_COLLECTION_30_OF_30_SERIES_8_OF_8_LAUNCH_TARGET_MET_AT_MU_0P9_NIP_CHANGE_WITHDRAWN`.
Δεν έγινε αλλαγή frozen geometry, STL, commit, push ή PR. ΠΑΥΣΗ.

### Απόφαση μετά το P2-61 — freeze απόδοσης flywheel

Το P2-61 γίνεται δεκτό. Baseline: corrected contact law `law(2p)`, platen
6,5 mm, recalibrated ground damping, timestep 0,25 ms, launch στο υπάρχον
58 mm nip με μ=0,9/surface16, collection surface 5 m/s και physical gate
dynamic tread μ≥0,9. Το adjustable nip αποσύρεται.

Με απόφαση χρήστη παγώνει εδώ η δουλειά απόδοσης flywheel: δεν ξεκινούν
5d-2, 5e ή σχεδίαση τροχού πριν επιβεβαιωθεί η φυσική κίνηση του chassis.

### P2-62 — frozen physical-drive protocol

Γράφτηκε το `physical-drive-test-protocol-el.md` και παγώθηκε το πλήρες sweep
προβλέψεων 5a για μ=0,5/0,7/0,9 και loaded radius 82/84/85 mm. Το κύριο τεστ
είναι μόνο με PSU 12 V, Mega από USB και κοινό GND. Το μέγιστο PSU current
μένει πεδίο φυσικής συμπλήρωσης. Τα 18 V/Parkside μετατέθηκαν μετά την
επιβεβαίωση κίνησης· τότε απαιτείται PWM ceiling 66,7% για τους 12 V FIT0403,
χωρίς firmware change τώρα.

Το predicted total current (τέσσερα μοτέρ) είναι: straight0,3 peak
19,25…20,76 A, straight/accel0,6 και braking peak 28 A, open spin
10,45…15,99 A, gyro spin 26,80…28 A και gyro arcs 18,37…19,76 A. Τα means,
όλες οι τελικές πόζες, yaw rates, centre translations και slips βρίσκονται
στο immutable JSON. Το current είναι torque-command conversion με το
stall-label secant `3,73 Nm / 7 A`, όχι BTS7960 electrical model.

Firmware audit χωρίς αλλαγή: το `motion_intake_mega.ino` εκπέμπει 10 Hz duty,
τέσσερα raw encoder counts, beam/E-stop και raw `imu.gz` (IMU sampling 20 Hz).
Τα `driveCurrentLeft/Right` είναι duty, όχι ampere. Δεν εκπέμπει V/A,
counts/rev, wheel speed ή integrated yaw και δεν έχει τον 5a gyro controller.
Άρα total physical current μετριέται από το PSU, per-motor current δηλώνεται
`NOT_MEASURED`, και ο 5a controller πρέπει να τρέχει εξωτερικά για έγκυρη
σύγκριση.

Προδηλώθηκαν acceptance limits και calibration mapping. Holdout είναι το
gyro semicircle R1,0 m και δεν χρησιμοποιείται σε fit. Read-only CAD audit:
επιβεβαιώθηκαν οι δύο διορθώσεις χρήστη και δεν βρέθηκε άλλη ίδια αριθμητική
αστοχία. Rail-extension/crossbar-half holes είναι συμμετρικά 30 mm από άκρα
στον κώδικα, αλλά παραμένουν μη φυσικά probed κατά το README, οπότε ζητείται
μόνο έλεγχος sleeve insertion 60 mm/ευθυγράμμισης πριν την κίνηση.

Artifact:
`results/shared_flywheel_phase2/physical_drive_p262/drive_predictions.json`
με SHA-256
`c2fc03c7baa97764d5d31bdf8de1f4e3f22555285200922f22910c3f01144a20`
στο sibling file και Git base
`5196885912057f5df8cdf0e444006d1d9ac3c03c`. Δεν χρησιμοποιήθηκε καμία
φυσική μέτρηση. Δεν έγινε αλλαγή CAD, STL ή firmware, ούτε commit/push/PR.
Κατάσταση: `P2_62_PROTOCOL_AND_PREDICTIONS_FROZEN_AWAITING_12V_PHYSICAL_RUN`.
ΠΑΥΣΗ.

### P2-62 acceptance και P2-63 — physical drive control stack

Το P2-62 γίνεται δεκτό με frozen prediction SHA
`c2fc03c7baa97764d5d31bdf8de1f4e3f22555285200922f22910c3f01144a20`.
Το άγνωστο-current 12 V PSU χρησιμοποιείται για λειτουργική επιβεβαίωση· η
ποσοτική σύγκριση με sim μεταφέρεται στο μεταγενέστερο Parkside βήμα. Στο
protocol προστέθηκε ότι measured gyro-spin centre translation 0,35…0,37 m
αποτελεί λειτουργικό collection-route risk, όχι μόνο sim discrepancy.

Προστέθηκε μόνο νέο `arduino/motion/drive_test_mega/drive_test_mega.ino`, με
ίδια pins και χωρίς αλλαγή υπάρχοντος sketch: 100 Hz 5a-structured wheel PI,
optional gyro yaw PI, configurable ramp/PWM ceiling, 300 ms host timeout,
E-stop, per-run time/distance gates και 50 Hz telemetry. Το ενεργό repo δεν
τεκμηριώνει FIT0403 CPR (το 700 αφορά αρχειοθετημένο FIT0186), άρα `ARM`
απορρίπτεται μέχρι measured `CFG CPR`. Compile για Mega πέρασε: 18.420 bytes
flash και 832 bytes global RAM. Δεν έγινε upload.

Το Pi runner `scripts/run_drive_test_pi.py` περιέχει όλες τις P2-62 scripted
ακολουθίες, immediate Space/q/Esc STOP, signal cleanup, dry-run, local CSV και
metadata. Απώλεια script/SSH καλύπτεται επιπλέον από το firmware timeout. Το
PC `fetch_drive_test_logs.py` κάνει scp και σύγκριση με το frozen JSON, με
optional tape/PSU measurements.

Προστέθηκε `base_serial_bridge`: παίρνει το stamped τελικό output μετά το
physical twist_mux, περιορίζει v/ω, στέλνει CMD, δημοσιεύει `/odom`,
`/imu/data_raw`, `/drive/telemetry` και odom TF. Runner και bridge μοιράζονται
exclusive lock και δεν μπορούν να ανοίξουν μαζί το Mega. Το ξεχωριστό
`physical_drive_bringup.launch.py` ξεκινά controller, τον αναγκαίο navigation
adapter, command bridge, stamp relay, mux, serial bridge και το υπάρχον panel,
χωρίς Nav2/SLAM/perception και χωρίς αλλαγή `run_pi.sh` ή console code.

Validation: runner dry-run + PC summary pass, Jazzy protocol/bridge-node fake
serial 4/4 pass, `tennis_robot` package build pass και launch source parse
7 entities.
Arduino compile έγινε με προσωρινό official CLI/core στο `/tmp`. Κανένα Pi ή
Mega δεν προσεγγίστηκε.

Η 2026-10-15 frame baseline ενημερώθηκε read-only: όλες οι πλαϊνές joint/grid
οπές είναι πλέον through-bolts από δύο τοίχους με εξωτερικές ροδέλες/παξιμάδια,
όλα τα STL ξαναβγήκαν και το `modular-chassis-v3.scad` δεν αλλάχθηκε.

Κατάσταση: `P2_63_NEW_SKETCH_PI_RUNNER_ROS_BRIDGE_COMPILED_DRY_RUN_PASS_AWAITING_PHYSICAL_BRINGUP`.
Δεν έγινε commit, push ή PR. ΠΑΥΣΗ.

### P2-64 — voltage-domain controller, serial timing και fail-safe wiring

Η P2-63 δομή έγινε δεκτή, αλλά η παλιά αντιστοίχιση
`duty=torque_command/3,73` **ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ** πριν από οποιοδήποτε upload.
Ο BTS7960 είναι voltage/PWM actuator. Το νέο drive-test sketch χρησιμοποιεί
feed-forward `ω_target/ω_noload(12 V)` και PI duty correction με
back-calculation anti-windup σε όλο το επιτρεπτό PWM. Η αρχική no-load τιμή
12,776 rad/s είναι μόνο η ονομαστική μετατροπή των 122 rpm και απαιτεί
lifted-wheel μέτρηση/`CFG NOLOAD` στο bring-up. Η x1 encoder speed εκτιμάται
σε παράθυρο 40 ms, ενώ το control tick μένει 100 Hz. Ο physical controller
είναι δομικά voltage-domain και ο 5a torque-domain, άρα τα transients τους
συγκρίνονται μόνο ποιοτικά.

Firmware, Pi runner και ROS bridge πέρασαν σε 500000 baud. Το runner serial
timeout μειώθηκε από 50 σε 5 ms. Για συντηρητική γραμμή 220 χαρακτήρων ο
wire-time bound είναι 4,4 ms. Προστέθηκαν telemetry πεδία
`max_loop_gap_us`/`max_telem_write_us`: dry-run `spin_90` έδωσε 155 γραμμές,
50 Hz synthetic logging και μέγιστο host-loop gap 5,20 ms. Αυτό κλείνει το
software/transport gate, αλλά το STOP <20 ms παραμένει υποχρεωτικό φυσικό
gate με τροχούς στον αέρα· compile/dry-run δεν το παρουσιάζουν ως hardware
μέτρηση.

Το E-stop D33 έγινε fail-safe NC προς GND με `INPUT_PULLUP`: LOW=OK,
HIGH=πατημένο/αποσυνδεδεμένο/κομμένο καλώδιο. HIGH στο boot θέτει αμέσως
`ESTOPPED`, κρατά τα BTS disables LOW και απαγορεύει ARM. Στη δοκιμή PSU το
πλήκτρο Output του τροφοδοτικού είναι το κύριο E-stop ισχύος. Στη μελλοντική
φάση μπαταρίας απαιτείται E-stop που κόβει φυσικά motor power· το D33 είναι
μόνο λογικό κανάλι.

Τα `drive_test_mega.ino`, `base_serial_protocol.py`,
`base_serial_bridge.py`, `physical_drive_bringup.launch.py` και ο Pi runner
ξαναγράφτηκαν σε κανονικό αναγνώσιμο στυλ. Ο bridge/runner συνεχίζουν να
μοιράζονται exclusive port lock, ο watchdog μένει 0,25 s και το launch μένει
χωριστό από `run_pi.sh`. Προστέθηκε ο pin-by-pin οδηγός
`docs/hardware/drive-test-wiring-el.md`: PSU 12 V, δύο drive BTS7960, δύο
παράλληλα FIT0403 ανά πλευρά, 2,5 mm² power wiring, 0,25–0,5 mm² signal,
20 A initial repository fuse baseline, Mega/BTS/PSU common GND, four
encoders, MPU6050, NC E-stop, LED, Pi USB/Ugreen, ASCII diagram και pre-power
checklist. Άγνωστα FIT0403/BTS wire colors/pin orders σημειώθηκαν
`ΕΠΙΒΕΒΑΙΩΣΗ ΑΠΟ ΤΟ DATASHEET/ΕΤΙΚΕΤΑ` χωρίς εικασία.

Validation: Mega compile PASS, 19.020 bytes flash και 882 bytes global RAM.
Pi runner dry-run PASS. Jazzy `colcon --packages-up-to tennis_robot` PASS
για 3 packages, fake-serial protocol/bridge 4/4 PASS και launch parse 7
entities. Το αρχικό package-only build απορρίφθηκε επειδή δεν περιλάμβανε τις
δύο ROS package dependencies· το πλήρες packages-up-to run είναι το τελικό
αποτέλεσμα. Δεν συνδέθηκε Pi ή Mega, δεν έγινε upload, δεν άλλαξε
`motion_intake_mega.ino`, CAD ή STL και δεν έγινε commit/push/PR.

Κατάσταση:
`P2_64_COMPILED_DRY_RUN_AND_FAKE_SERIAL_PASS_AWAITING_NC_ESTOP_WIRING_AND_LIFTED_WHEEL_GATE`.
ΠΑΥΣΗ.

### P2-64 διόρθωση — εργοστασιακά χρώματα FIT0403

Η δήλωση του νέου drive-test wiring guide ότι τα χρώματα του FIT0403 δεν
τεκμηριώνονται ήταν λανθασμένη και **ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ**. Τα προηγούμενα motion
documents περιείχαν τη χρωματική πληροφορία και έγινε διασταύρωση με το
επίσημο διάγραμμα DFRobot FIT0403: κόκκινο=Motor+, μαύρο=Motor−,
μπλε=Hall GND, πράσινο=Hall VCC 5 V, κίτρινο=Hall A Vout και λευκό=Hall B
Vout. Ο πίνακας προστέθηκε στο `docs/hardware/drive-test-wiring-el.md` μαζί με
τις ακριβείς BTS/header συνδέσεις.

Η παλιά πρόταση χρωμάτων στο archived motion-perfboard document αφορούσε τα
δικά μας extension harnesses και όχι το εργοστασιακό εξαπολικό pigtail. Για
αυτό τα extensions συνεχίζουν να απαιτούν labels/continuity map. Το
`ΕΠΙΒΕΒΑΙΩΣΗ ΑΠΟ ΤΟ DATASHEET/ΕΤΙΚΕΤΑ` διατηρείται μόνο για το pin order των
συγκεκριμένων BTS7960 clones και άλλων μη τεκμηριωμένων modules. Δεν άλλαξε
firmware, CAD ή STL και δεν έγινε upload/commit/push/PR.

### P2-65 — T-σύνδεση των X240/X300 με τα ήδη τυπωμένα terminals

Επιβεβαιώθηκε το εύρημα 2026-10-16: τα παλιά 220 mm halves έφταναν από το
κέντρο έως Y=±220 και είχαν μη μηδενική τομή 25 mm με τα
`front_terminal_left/right`. Η λύση ακολουθεί την αρχή της διορθωμένης πίσω
γωνίας χωρίς να αλλάζει κανένα ήδη τυπωμένο κομμάτι: X240 και X300 halves
μήκους 195 mm έως την εσωτερική παρειά Y=±195, συν ξεχωριστό T-clamp ανά άκρο.
Ο βραχίονας του clamp μπαίνει 60 mm στην τραβέρσα, περνά τις υπάρχουσες
top/side M4 στα 30 mm από το άκρο και κρατά ανοικτό το grid Y150. Το tab
βιδώνει με field-drilled through M4 στα terminals.

Οι τέσσερις νέες οπές είναι Ø4,5 mm, άξονας Y, X240/Z46 και X300/Z46 σε κάθε
πλευρά. Οι κινητήρες/καλώδια αφαιρούνται και η διάτρηση γίνεται με dry-assembled
T-clamp ως οδηγό. Στο X240 τρυπιέται μαζί και το longitudinal sleeve του joint
X200. Hardware: M4×65, χαμηλή κεφαλή+ροδέλα outboard, ροδέλα+εξωτερικό
παξιμάδι inboard. Με μέγιστη προεξοχή κεφαλής 4 mm και tyre inner face
Y255,5 μένουν 6,5 mm.

Ο collision audit έδωσε:

- crossbar/terminal: μηδενικός όγκος, μόνο κοινή επιφάνεια Y=±195·
- ballast plate: μηδενική τομή, μόνο επαφή στην Z52·
- Baja tyre: μηδενική τομή·
- control console/rail extension X100/X125: τελειώνει X185, 30 mm πριν από το
  πλησιέστερο X240 tube envelope·
- FIT0403 σώμα/encoder: κάτω ακμή Z65 έναντι clamp top Z52. Η παλιά ευθεία
  κάθοδος καλωδίου στο X300 δεν είναι αποδεκτή· η checked διαδρομή κάνει jog
  στο X275 πάνω από το deck, κατεβαίνει inboard στο Y185 και περνά κάτω από
  Z22, με μηδενική exact τομή.

Βρέθηκε και διορθώθηκε δεύτερη παλιά σύγκρουση: το M6 C-hook της πλάκας στο
X310/Y±185 έτεμνε την ακμή του 20 mm X300 tube. Η νέα, ήδη αλλαγμένη X300
πήρε συμμετρικό relief Ø9 mm. Το exact probe έγινε κενό και το nominal
ακτινικό διάκενο γύρω από το Ø6,5 envelope είναι 1,25 mm. Το hook X330 ήταν
ήδη καθαρό. Δεν άλλαξε η πλάκα ή κανένα τυπωμένο terminal.

Ο πλάγιος assembly έλεγχος βρήκε επίσης transform-only λάθος σε όλες τις
παλιές centered sleeves/corner clamps: η έκφραση
`rail_z+(rail_h-sleeve_h)/2` έβαζε το κέντρο τους στο Z25,5 αντί Z37 και τις
πλάγιες οπές 11,5 mm χαμηλά. Αντίστοιχα τα low sleeves ήταν στο Z25,5 αντί
Z32. Διορθώθηκαν μόνο οι transforms του `frame_assembly`, όχι τα ήδη τυπωμένα
STL/modules. Η νέα πλάγια preview επιβεβαιώνει ότι τα sleeves βρίσκονται μέσα
στους σωλήνες και οι άξονες των οπών συμπίπτουν.

Η combined B32R FEA ξανατρέχτηκε στα 10/5 mm με 195 mm crossbars και
συντηρητικά PETG connector sections 24×6 mm (X240), 11×6 mm (X300), χωρίς
πίστωση ακαμψίας χαλύβδινης βίδας:

- vertical 2g: 0,644 → **0,646 mm** (+0,17%)·
- launch: 1,940 → **2,212 mm** (+14,01%), local 0,474 mm·
- antisymmetric racking: 1,981 → **2,195 mm** (+10,81%), local 0,415 mm.

Η mesh μεταβολή είναι ≤0,90%. Τα αποτελέσματα είναι comparative elastic
screen και όχι πιστοποίηση hole bearing/preload/layers/fatigue.

Εξήχθησαν μόνο τα αλλαγμένα/νέα
`x240_crossbar_half`, `x300_crossbar_half`, `x240_t_clamp`,
`x300_t_clamp`. Όλα έχουν `nonmanifold_edge_count=0`. Μάζες 0,72 g/cm³:
71,091 / 33,448 / 24,288 / 7,367 g ανά τεμάχιο αντίστοιχα. Το set είναι πλέον
20 unique meshes, locked frame 1,477 kg και συνολικό table 1,551 kg. Το ⚠
αφαιρέθηκε μόνο μετά τα collision, FEA και manifold gates. Ενημερώθηκαν
README και v3 print-release report. Δεν έγινε firmware/STL άλλου part,
commit, push ή PR. Κατάσταση: `P2_65_T_CONNECTION_RELEASE_PASS`.

### P2-66 — βατήρας 70–80°, μαλακό ταμπλό, κλειστό μπροστινό καλάθι

Έγινε πλήρης one-ball/empty-basket μήτρα 540 συνθηκών στο πραγματικό v3 με το
διορθωμένο contact model P2-61: γωνίες 70/75/80°, R75/R110, surface
5/6/8 m/s, μ πέλματος 0,6/0,9, Y=0/±50/±100 και restitution ταμπλό
0,1/0,3/0,5. Αποτέλεσμα **0/540 PASS**. Καμία τροχιά δεν έφτασε στο ταμπλό.

Η τραβέρσα της αψίδας X305…355/Z230…260 δέχθηκε τη μπάλα σε 515/540
συνθήκες. Για R75/75 η πραγματική έξοδος ήταν 57,54…70,30° και για R75/80
62,50…75,06°, όμως οι απαγορευμένες επαφές ήταν αντίστοιχα 84/90 και 90/90.
Στο κρίσιμο Z κέντρου 187 mm η μπάλα βρισκόταν στο X312,5…317,6, ενώ για
R33+10 mm απαιτείται X≤262: λείπουν 50,5…55,6 mm οπισθοχώρησης. Το R110
αποδέσμευσε νωρίς και έδωσε μόνο 32,76…52,75°.

Το κατακόρυφο μπροστινό τοίχωμα Z240 είχε επαφή σε 449/540. Για wall-only
clearance 15 mm η πρώτης τάξης κλίση προς τα πίσω εκτιμάται περίπου 25° για
R75/75, 30° για R75/70 και 55° για R110. Δεν είναι συνολική λύση: το R75/80
δεν άγγιξε καθόλου τοίχωμα και παρ' όλα αυτά χτύπησε την αψίδα 90/90.

Μοντελοποιήθηκε compliant ταμπλό X274/Z334 → X82/Z370, 340×10 mm, top Z375,
και fail-closed άνω αρθρωμένο πλήρες εμπρόσθιο πάνελ X278/Z42…240. Η πύλη
ανοίγει προς το καλάθι μόνο στις +15…+20°, μέσω cam κλίσης ή προαιρετικού
PCA9685 servo με τα interlocks P2-52/P2-59. Το paired-angle intersection με
το launcher sweep −10…+20° είναι κενό. Το e EVA/διχτυού δεν μπορεί να
συναχθεί, επειδή δεν υπήρξε κρούση· απαιτείται φυσική μέτρηση. Δεν έγινε pile
simulation ή feed test.

Όλες οι αποτυχίες έχουν summary, telemetry, contacts και Gazebo log. Έξι
ονομαστικές γεωμετρίες έχουν επιπλέον full state replay. Αναλυτικό report:
`docs/mechanism/shared-flywheel-deflector-p266-el.md`. Δεν παράχθηκε STL και
δεν έγινε commit/push/PR. Κατάσταση:
`P2_66_NO_GO_FROZEN_ARCH_0_OF_540`. ΠΑΥΣΗ.

### P2-67 — γερμένη αψίδα και ταμπλό επάνω από τη στήλη

Με απόφαση χρήστη η διάγνωση P2-66 έγινε δεκτή και άλλαξε μόνο η αψίδα
αισθητήρων: οι βάσεις έμειναν X318/Z52, ο επάνω κόμβος πήγε X390/Z245
(+60 mm) και η πίσω ακμή της τραβέρσας είναι X365. Η κλίση είναι 20,46°
από την κατακόρυφο. LiDAR X390/scan Z390 και κάμερα X410/Z305/50°
μετακινήθηκαν μαζί· το camera ground footprint μετατοπίζεται περίπου 60 mm
εμπρός χωρίς αλλαγή γωνιακού FOV. Το ταμπλό είναι ανεξάρτητα στηριγμένο στο
καλάθι, X250/Z363→X360/Z343, πλάτος 340 και πάχος 10 mm.

Η ολοκληρωμένη υπάρχουσα μήτρα R75/75 και R75/80 έδωσε **180/180 PASS**.
R75/75: πραγματική έξοδος 56,00…70,17°, 3,572…5,607 m/s, 90/90. R75/80:
61,48…74,18°, 3,547…5,606 m/s, 90/90. Καμία επαφή με αψίδα/αισθητήρες/
τοίχωμα, ball-top ≤356,1 mm και τελική μέση ταχύτητα ≤0,0194 m/s. Η baseline
είναι R75/75, επειδή οι 80° αφήνουν πολύ λιγότερο χώρο για μπροστινό
τοίχωμα.

Τα e=0,1/0,3/0,5 έδωσαν ακριβώς τα ίδια αποτελέσματα ανά speed/μ/Y: το
`restitution_coefficient` δεν επηρεάζει το DART εδώ. Συνεπώς το «μαλακό»
ταμπλό **δεν μοντελοποιήθηκε**. Ο solver αφήνει τη μπάλα με χαμηλή ταχύτητα
αμέσως μετά την επαφή και σχεδόν μηδενική τελικά, άρα η ανάκτηση μάλλον δεν
κυριαρχεί, αλλά EVA/δίχτυ και e κρίνονται υποχρεωτικά στη φυσική μακέτα.

Το γερμένο τοίχωμα στην X278 απορρίφθηκε: ασφαλές top μόνο περίπου Z90–100,
έναντι απαιτούμενου περίπου Z190 ακίνητο και Z225 σε ήπιο φρενάρισμα για
40–45 μπάλες. Το rerun Z90 σταμάτησε και δεν επαναλήφθηκε. Ο beam screen
της γερμένης αψίδας έδωσε 3,03 MPa, factor 6,60 ως προς 20 MPa και 0,814 mm
tip displacement, αλλά παραμένουν φυσικά joint/layer/fatigue/vibration
gates. Αναλυτικό report:
`docs/mechanism/shared-flywheel-tilted-arch-deflector-p267-el.md`.
Κατάσταση: `P2_67_ACCEPTED_FLIGHT_180_OF_180_OLD_WALL_REJECTED`.

### P2-68 — όρθιο τοίχωμα ανάμεσα σε άνοδο και κάθοδο

Χωρίς νέο Gazebo, αναλύθηκαν γεωμετρικά οι 30 μοναδικές υπάρχουσες
τροχιές R75/75 (ένα e) με interpolation chord ≤0,1 mm. Με R33+10 mm και
panel 4 mm, το X225 δίνει exact circular top limit Z266,09 και
συντηρητικό box limit Z238,37. Επιλέχθηκε X225, Y±172, bottom Z46,1,
**top Z235**, ακμή R2. Το πλήρες panel έχει exact surface clearance
25,00 mm και συντηρητικό box surface clearance 11,45 mm. Max |Y| κοντά στο
wall 33,93 mm και lateral surface-to-edge clearance 105,07 mm.

Η βάση έχει clear θύρα 80×80 mm και φύλλο 84×84 με άρθρωση Z126,1. Ελατήριο
την κρατά κλειστή· ανοίγει προς τα πίσω μόνο +15…+20°/όχημα ακίνητο μέσω
P2-52 cam ή PCA9685, με τα P2-52/P2-59 interlocks. Σταθερό κανάλι
X227/Z46,1→X278/Z42 και launch-only selector nose οδηγούν στο rear-mouth
floor X329,2/Z26,1 (+15°) ή X331,5/Z21,1 (+20°). Δεν έγινε feed, pile ή
singulation test.

Το `full-robot-v3.scad` ενημερώθηκε. Paired-angle OpenSCAD intersection
wall/door έναντι launcher/D5065/actuator/arm/cables −10…+20°: empty. Και το
σταθερό κανάλι X227…278 έναντι όλου του sweep και το κλειστό shutter στις
−10…+14°: empty. Το σκόπιμο nose/mouth overlap εξαιρείται. Machine-readable audit:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/p268_wall_geometry.json`.
Αναλυτικό report:
`docs/mechanism/shared-flywheel-upright-wall-p268-el.md`. Δεν παράχθηκε STL,
δεν έγινε commit/push/PR. Η γεωμετρία έγινε δεκτή ως baseline και
επιβεβαιώθηκε ακολούθως στο P2-69. Κατάσταση:
`P2_68_ACCEPTED_X225_Z235`.
ΠΑΥΣΗ.

### P2-69 — confirmation με όρθιο τοίχωμα και κλειστή θύρα

Τα P2-67/P2-68 έγιναν δεκτά ως baseline: R75/75°, αψίδα X390/Z245,
ταμπλό X250/Z363→X360/Z343, όρθιο τοίχωμα X225/top Z235 και fail-closed
θύρα clear 80×80 mm. Εκτελέστηκε μόνο η εγκεκριμένη μήτρα 30 συνθηκών:
surface 5/6/8 m/s × μ 0,6/0,9 × Y=0/±50/±100, με ένα e=0,3.

Αποτέλεσμα **30/30 PASS**. Πραγματική έξοδος 56,00…70,17°,
3,572…5,607 m/s. Max pre-deflector ball-top 356,05 mm, max τελική μέση
ταχύτητα 0,01835 m/s. Front-wall contacts 0, closed-door contacts 0,
forbidden arch/sensor/chassis/actuator contacts 0. Και οι 30 είχαν
deflector contact, basket entry και retained empty-basket. Δεν άλλαξε η
γεωμετρία.

Firmware requirement: με γεμάτο καλάθι επιβάλλεται deceleration cap
**≤~0,2 g**. Ο σωρός 45 μπαλών εκτιμάται ότι χρειάζεται περίπου Z236 σε
φρενάρισμα, ενώ το τοίχωμα είναι Z235. Αυτό είναι hard motion requirement
και απαιτεί φυσική δοκιμή 40/45 μπαλών· το one-ball PASS δεν αποδεικνύει
pile retention. Δεν έγινε pile/feed simulation.

Aggregate:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p269/summary.json`.
Report: `docs/mechanism/shared-flywheel-confirmation-p269-el.md`.
Δεν παράχθηκε STL και δεν έγινε commit/push/PR. Κατάσταση:
`P2_69_R75_75_CLOSED_WALL_DOOR_30_OF_30_PASS`. ΠΑΥΣΗ.

### P2-70 — rear electronics/energy pod, layout proposal only

Πρώτη απόκλιση από την προσδοκία: με ICR X117 και εμπρός datum X542 ο
υφιστάμενος κύκλος είναι R425. Στο X−300 επιτρέπεται μόνο |Y|≤82,1 mm,
άρα full-width pod έως εκεί απορρίπτεται. Χωρίς αλλαγή παγωμένης
γεωμετρίας προτάθηκε λοξοκομμένο envelope X−272…−125, Y±167, σώμα
Z55…350 και highest control Z374. Worst rear corner R421,38, margin 3,62
mm εντός του υφιστάμενου κύκλου.

Exact-solid probes: pod body έναντι basket/feed/deflector, drive
tyres/FIT0403 και v3 frame = empty. Ελάχιστα αναλυτικά gaps: backboard
6,3 mm, basket side 5,0 mm, rear tyre inner face 88,5 mm, rear FIT0403
40,0 mm, body–crossbar 3,0 mm. Τα M4 feet εξαιρούνται επειδή εδράζονται
σκόπιμα στην πίσω τραβέρσα Y±75/±125.

Η ισχύουσα mass ledger baseline είναι 18,110 kg, CG X165,57 και
front/rear 63,17/36,83%, όχι ≈68%. Με μεταφορά Parkside/Ugreen και proxy
0,770 kg παλαιότερου rear-pod CAD προκύπτουν 18,880 kg, CG X146,86 και
58,04/41,96%. Με επιπλέον προσωρινή υπόθεση 1,010 kg για όλα τα αζύγιστα
ηλεκτρονικά, sensitivity result 53,25/46,75%. Άρα το ≈65% δεν
τεκμηριώνεται· μάζες/τελικό four-wheel weighing = ΜΕΤΡΗΣΗ ΧΡΗΣΤΗ.

Οι δύο Parkside έχουν clear reserve 141×91×81 και rear drawer direction,
αλλά dock/rail/latch αναβάλλονται μέχρι αγορά. Pi clear 98×74×46, Ugreen
121×52×52 με αφρό/λουράκι. BTS/fuse/relay/DC-DC/ideal-diode/LED bodies και
connector bend zones σημειώθηκαν ΜΕΤΡΗΣΗ ΧΡΗΣΤΗ. MPU6050 μένει στο κέντρο
του σασί. Cable budgets: front FIT0403 0,9 m/πλευρά, rear 0,5 m/πλευρά,
LiDAR/camera 1,1 m, FIT0186 0,9 m, D5065 controller 0,6 m/πλευρά.

Για PSU motion test απαιτούνται shell/mounts, central tray, rack με δύο
BTS, control panel/buttons, inlet/fuse/relay και strain relief· batteries
και docks μπορούν να περιμένουν. Αναφορά:
`docs/mechanism/rear-pod-p270-el.md`. Audit:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/p270_rear_pod.json`.
Δεν έγινε STL/firmware/Gazebo/commit/push/PR. Κατάσταση:
`P2_70_REAR_POD_LAYOUT_PROPOSED_MEASUREMENT_GATED`. ΠΑΥΣΗ.

### P2-71 — απλοποίηση τοιχωμάτων καλαθιού

Πρώτα επαναναλύθηκαν τα 30 raw P2-69 `native_contacts.csv`: front wall
0/30, closed door 0/30, παλιό rear backboard 30/30. Όλα τα 14.812
backboard samples είναι Z90,004…90,495 mm και σε 30/30 το backboard contact
αρχίζει αφού έχει ήδη αρχίσει floor contact. Άρα πρόκειται για μπάλα στον
πάτο, όχι για επαφή πτήσης.

Το προτεινόμενο front grid κράτησε X225/Z46,1…235/t4 και openings ≤40 mm,
με μοναδικό solid την fail-closed leaf 84×84 γύρω από clear 80×80. Το νέο
rear grid είναι όρθιο X−118/t4/Y±140/Z72,545…320, openings ≤35,4 mm. Η
πίσω παρειά X−120 αφήνει 5,0 mm από το accepted pod X−125. Το παλιό 12°
backboard αφαιρέθηκε. Ο πάτος και τα side grids επεκτάθηκαν X−118…−85.

Geometry-only audit από τις υπάρχουσες 30 P2-69 τροχιές, chords ≤0,1 mm,
με συμπαγή panel envelopes: min ball-surface clearance front 25,005 mm,
rear 106,949 mm. Και τα δύο PASS του ≥10 mm. Exact rear-grid/pod
intersection empty. Δεν έγινε νέο Gazebo.

Νέο storage X−118…225: plan 111.400 mm², usable volume 17,932 L έως
front rim Z235 και 27,401 L έως side/rear Z320, μετά το ίδιο 1,7 L
reserve. Volume-conserving static top 40/45: Z221,24/Z233,29. Ρητό 0,2 g
free-surface screen: front top Z254,12/Z266,18, άρα wall Z235 FAIL κατά
19,12/31,18 mm. Στο rear grid η αντίθετη 0,2 g κλίση μένει
Z256,95/Z269,01 και PASS. Το παλιό «≈Z236 με braking» ανακαλείται ως μη
τεκμηριωμένο· pile dynamics παραμένει physical gate.

Αναφορά: `docs/mechanism/shared-flywheel-basket-grid-p271-el.md`. Audit:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/p271_basket_grid.json`.
Το `full-robot-v3.scad` ενημερώθηκε. Χωρίς STL/commit/push/PR. Κατάσταση:
`P2_71_FLIGHT_GEOMETRY_PASS_BRAKING_PILE_FAIL_PAUSED`. ΠΑΥΣΗ.

### P2-72 — printable rear pod v0 για PSU drive test

τι διαφέρει από το αναμενόμενο: το παλιό ενιαίο motion-electronics tray
180×240 mm εισέρχεται στους εγκεκριμένους μελλοντικούς πλευρικούς όγκους
Parkside. Χωρίς αλλαγή του accepted P2-70 envelope X−272…−125/Y±167 έγινε
κεντρική ανοικτή ραφιέρα σε τέσσερις M4 ντίζες: Mega Z64, perfboard Z106,
δύο BTS Z158 με spacer 20 mm και Pi Z215. Το control panel είναι Z347…351
και το προβλεπόμενο E-stop top Z372 < Z375.

Παράχθηκαν 13 manifold STL στο ιστορικό
`cad/rear-pod/archive/stl-v0/`. Max XY bounding box:
138×190 mm για control panel· τα δύο base halves είναι 147×167 mm. Όλα
PASS του ορίου 250×250 mm. Οι δύο BTS έχουν ανεξάρτητο αερισμό και οι
μελλοντικοί Parkside όγκοι αρχίζουν στα |Y|70, έναντι rack edge |Y|62:
ονομαστικό πλευρικό gap 8 mm. Η καλωδιοπλάτη X−272…−268 κρατά τέσσερις
motor-power εξόδους έξω και encoders/USB/MPU στο κέντρο. Το MPU6050 έχει
ξεχωριστή βιδωτή βάση στην υπάρχουσα M4 διάταξη X240/Y±30.

LED, inline fuse holder, 12 V inlet/glands, populated board heights και
connector bend zones παραμένουν ρητά `ΜΕΤΡΗΣΗ ΧΡΗΣΤΗ`. Δεν απελευθερώθηκε
holder για Parkside/Ugreen/relay/DC-DC/D5065. Αναφορά, εικόνες, BOM,
συναρμολόγηση και καλωδίωση: `cad/rear-pod/README.md`. Δεν έγινε
firmware/Gazebo/commit/push/PR. Κατάσταση:
`P2_72_PRINTABLE_PSU_DRIVE_POD_V0_EXPORTED_MEASUREMENT_GATED`. ΠΑΥΣΗ.

### P2-73 — βιδωτός R75/75° ski, πρώτο geometry gate

τι διαφέρει από το αναμενόμενο: πριν σχεδιαστεί οποιαδήποτε στήριξη,
ο παγωμένος `p267_ski(75)` στη θέση X435…310,56/Z10…65,59 τέμνει τα
παγωμένα side rails του P2-68 selector nose και στα δύο launch endpoints.
Στα +15° το exact-solid intersection έχει 120 facets και στα +20° 160.
Ο χωριστός έλεγχος του κεντρικού selector floor είναι empty και στις δύο
γωνίες· όλη η τομή προέρχεται από τα rails Y±37,5 μέσα στο ski width Y±47.

Σύμφωνα με τον stop-on-first-failure κανόνα δεν σχεδιάστηκαν brackets,
height slots ή printable sidewalls και δεν δημιουργήθηκε STL directory.
Δεν εκτελέστηκαν οι μεταγενέστεροι sweep/wheel/support checks. Αναφορά και
εικόνες: `docs/mechanism/bolted-ski-mount-p273-el.md`. QA source:
`cad/modular-test-chassis/p273-existing-ski-feed-qa.scad`. Χωρίς
firmware/Gazebo/commit/push/PR. Κατάσταση:
`P2_73_FAIL_FROZEN_SKI_INTERSECTS_SELECTOR_RAILS_PAUSED`. ΠΑΥΣΗ.

### P2-74 — selector rails έξω από το ski

τι διαφέρει από το αναμενόμενο: δεν απαιτείται Y±55. Το μικρότερο
δοκιμασμένο θετικό clearance είναι με rail centres Y±52: inner faces
Y±50,5 έναντι ski-wall outer faces Y±50, δηλαδή 0,5 mm ανά πλευρά.
Άλλαξαν μόνο τα δύο launch-selector rails του P2-68.

Exact-solid +15/+20: rails και αμετάβλητο floor έναντι complete ski empty.
Rails/floor έναντι frozen upper plate και D200 wheel/motor sweeps −10…+20°
όλα empty. Rails/floor έναντι fail-closed door empty· rails έναντι fixed
channel empty. Η μοναδική channel τομή είναι το σκόπιμο floor seam X278
(8 vertices/12 facets), ενώ η πλήρης-module τομή του floor είναι η σκόπιμη
σύνδεση με το moving throat.

Clear width νέων rails 101 mm: Ø67 μπάλα έχει 17 mm/πλευρά. Ski clear 94 mm:
13,5 mm/πλευρά. Rail-to-wall slot 0,5 mm με 18,6…20,9 mm διαμήκη
επικάλυψη, άρα δεν υπάρχει γεωμετρική έξοδος ή σφήνωμα· compliance/print
tolerance μένει physical gate.

Ενημερώθηκαν `shared-flywheel-p268-upright-wall.scad` και
`full-robot-v3.scad`. Report:
`docs/mechanism/outboard-selector-rails-p274-el.md`. QA:
`cad/modular-test-chassis/shared-flywheel-p274-outboard-selector.scad`.
Χωρίς STL/Gazebo/firmware/commit/push/PR. Κατάσταση:
`P2_74_SELECTOR_RAILS_Y52_EXACT_SOLID_PASS`. ΠΑΥΣΗ.

### P2-74 revision Y±55 / P2-73 printable mount stop

τι διαφέρει από το αναμενόμενο: το προηγούμενο rail-to-wall κενό 0,5 mm
δεν έγινε δεκτό. Τα δύο launch-selector rails μετακινήθηκαν μόνο από
Y±52 σε **Y±55**. Με rail t3 και ski-wall outer faces Y±50, το νέο
ονομαστικό κενό είναι 3,5 mm ανά πλευρά.

Η περιορισμένη επανάληψη των P2-74 exact-solid probes στα +15/+20 έδωσε
empty για rail/floor έναντι ski, frozen plate, wheel/motor sweep −10…+20,
closed door και fixed channel. Η μετατόπιση Y±55 επομένως PASS.

Στο επόμενο conditional βήμα σχεδιάστηκε υποψήφιο shell/mount χωρίς να
μετακινηθεί η παγωμένη τροχιά R75/75. Η πρώτη δοκιμή στο +15° απέτυχε:
το πίσω M4 boss του ski στο X327/Y±56/Z≈44,11 τέμνει τα selector rails.
Exact intersection 72 vertices / 136 facets. Boss X315 empty, κύρια
brackets empty· το αντίστοιχο fastener envelope επίσης τέμνει. Έγινε
άμεσο stop χωρίς διόρθωση boss ή άλλης γεωμετρίας. Το failing candidate
δεν ενεργοποιήθηκε στο `full-robot-v3.scad` και δεν εξήχθη κανένα STL.

Αναφορά/εικόνες:
`docs/mechanism/p274-y55-p273-ski-mount-stop-el.md`. Diagnostic source:
`cad/modular-test-chassis/shared-flywheel-p273-bolted-ski.scad`.
Κατάσταση: `P2_74_Y55_PASS_P2_73_MOUNT_X327_BOSS_FAIL_PAUSED`. ΠΑΥΣΗ.

### P2-73 δεύτερη προσπάθεια — X315 + flat-section X400 support

τι διαφέρει από το αναμενόμενο: αφαιρέθηκε το failing boss X327/Y±56.
Κρατήθηκε το exact-clear X315 για πίσω συγκράτηση και προστέθηκε κύριο
boss X400 στο επίπεδο τμήμα, με εξωτερικές νευρώσεις Y±64 από το X300.
Τα κλειδωμένα P2-74 rails μένουν Y±55.

Το πρώτο gate, πλήρες mount έναντι selector floor/rails, είναι empty στα
+15° και +20°. Η νεύρωση έχει inner face Y61 έναντι rail outer face
Y56,5: 4,5 mm nominal clearance.

Το αμέσως επόμενο full-launcher sweep απέτυχε στην πρώτη γωνία, −10°.
Απομόνωση: printable ski path έναντι D200 wheels SOLID, 41 vertices / 74
facets· path έναντι upper plate EMPTY. Η αιτία είναι ότι το candidate
shell έχει σταθερό width 94 mm στο flat, ενώ το frozen `p267_ski(75)`
εκεί ξεκινά width 61 mm και ανοίγει σταδιακά. Έγινε stop χωρίς αλλαγή.

Αναφορά/εικόνες: `docs/mechanism/p273-ski-mount-r2-stop-el.md`.
Δεν ενεργοποιήθηκε στο full robot και δεν δημιουργήθηκαν STL/README.
Κατάσταση: `P2_73_R2_SELECTOR_PASS_D200_AT_MINUS10_FAIL_PAUSED`. ΠΑΥΣΗ.

### P2-73 final — exact frozen-width printable ski

τι διαφέρει από το αναμενόμενο: το candidate shell διορθώθηκε ώστε να
είναι ακριβώς το frozen `p267_ski(75)`, όχι constant-width. Symmetric
solid differences και προς τις δύο κατευθύνσεις είναι empty: R75/75°,
tangent 52, t2, segment widths 61→67→73→79→85→91→94 mm.

Αφαιρέθηκαν ως βοηθητικά στοιχεία το X400 boss/strut και τα sidewalls στο
flat και στα arc segments 3–5. Μένει μόνο X315 boss ανά πλευρά και δύο
X300 brackets. Το frozen floor δεν άλλαξε. Το full launcher intersection
που απομένει είναι αποκλειστικά το ήδη εγκεκριμένο intentional
floor–moving-throat handoff.

Exact-solid PASS: selector Y±55 στα +15/+20, hard launcher −10…+20,
sidewalls/throat, D200/plate/cheeks/trunnions, drive hardware/FIT0403,
frame, closed door/channel και ground. Η ρύθμιση 0…+20 PASS όταν ski και
μονάδα κινούνται μαζί 1:1. Ασύζευκτη ανύψωση ski απαγορεύεται. Nominal
ground clearance ≈9,0 mm· μετά την πτώση chassis ≈8 mm ανεβαίνουν και τα
δύο ≈8 mm.

Εξήχθησαν 3 manifold STL στο
`cad/modular-test-chassis/stl-ski-mount/`: ski
133,55×124,00×68,69 mm και δύο brackets 35×49×38,36 mm. README με PETG,
προσανατολισμό, supports, βίδες και ρύθμιση. Αναφορά:
`docs/mechanism/p273-bolted-ski-final-el.md`. Το full robot ενημερώθηκε.
Χωρίς Gazebo/firmware/commit/push/PR. Κατάσταση:
`P2_73_PRINTABLE_R75_75_SKI_MOUNT_PASS_PAUSED`. ΠΑΥΣΗ.

### P2-75 — rear pod v1 με τυπωμένη snap-stack

τι διαφέρει από το αναμενόμενο: μετά τα σχόλια στο P2-72 καταργήθηκαν οι
τέσσερις μεταλλικές M4 ντίζες και αντικαταστάθηκαν από τυπωμένους πυλώνες
Ø16 με αρσενικό split-snap, θηλυκή άνω φωλιά και μία M3 ανά πυλώνα. Δύο
εναλλάξ grids επιτρέπουν την αφαίρεση κάθε ραφιού μαζί με τα ανώτερα χωρίς
λύσιμο των κατώτερων. Ο οπτικός έλεγχος βρήκε αρχική επικάλυψη 5 mm μεταξύ
coupler Ø14 και γειτονικού πυλώνα· το εσωτερικό grid μετακινήθηκε μόνο προς
το κέντρο και το τελικό ονομαστικό κενό είναι 3 mm. Το frozen P2-70 envelope
X−272…−125/Y±167/Z55…375 δεν άλλαξε.

Δόθηκαν τρεις επιλογές καθαρού μήκους ανά επίπεδο: Mega→perf 38/48/58 mm,
perf→BTS 48/58/68 mm, BTS→Pi 53/63/73 mm και Pi→panel 98/113/128 mm. Το
current-equivalent άθροισμα 267 mm κρατά Z64/106/158/215/347. Το fit coupon
περιέχει δύο θέσεις component pin και έναν πλήρη male/female πυλώνα. Mega,
perfboard και 2×BTS7960 κρατιούνται με προσαρμοσμένα grooved Ø2,8 pins και
C-clips· οι BTS κρατούν spacer 20 mm. Το Pi παραμένει με αφρό/λουριά.

Η καλωδιοπλάτη μεταφέρθηκε από την πίσω ακμή X−272 στη χαμηλή παρειά προς
το σασί X−125…−130. Η χωριστή οπή LED και το `led_blank_insert`
καταργήθηκαν. Ο illuminated toggle 12 V και το illuminated START/ARM
τεκμηριώθηκαν με user-measurement gate για pinout/τάση LED· το START/ARM LED
δείχνει ARMED από D34, απευθείας με αντίσταση αν είναι 5 V ή μέσω
transistor/MOSFET στο perfboard αν είναι 12 V.

Με την τελική αποδοχή διορθώθηκε η οπή START/ARM από Ø15,5 σε **Ø16,3 mm**
για το παραγγελμένο R16-504BD (`ordered-parts` G7), τόσο στο rear-pod
`control_panel.stl` όσο και στο drive-test `control_console_fit_coupon.stl`.
Παράχθηκαν 26 manifold STL στο ιστορικό
`cad/rear-pod/archive/stl-v1/`, όλα με XY≤250 mm·
μεγαλύτερα είναι οι βάσεις 147×167 mm και το panel 138×190 mm. Το panel
παραμένει Z347…351 και το E-stop top Z372<375. Η αντοχή πραγματικού
πατήματος 20–30 N παραμένει physical coupon/load gate πριν από κίνηση.

README/CAD/STL: `cad/rear-pod/`. Διορθώθηκε επίσης η οδηγία του υπάρχοντος
ski: FIT0403 με άξονα επάνω, ski στη χαμηλή ονομαστική θέση και πραγματικός
έλεγχος underside 9–10 mm από το πάτωμα. Δεν έγινε firmware, Gazebo, commit,
push ή PR. Κατάσταση:
`P2_75_REAR_POD_V1_ACCEPTED_PRINTABLE_MEASUREMENT_GATED_PAUSED`. ΠΑΥΣΗ.

### P2-76 — rear pod v2 με όρθια πλακέτα ηλεκτρονικών

τι διαφέρει από το αναμενόμενο: το πραγματικό δεσμευμένο P2-70 Ugreen
είναι στο `[-195,42,258]` (clear 121×52×52), όχι στο προσωρινό v1 ghost
`Y112`. Μία ορθογώνια μονόπλευρη πλακέτα με Mega, perfboard και 4×BTS δεν
χωρά μαζί με αυτόν τον όγκο και το όριο εκτύπωσης 250×250. Χωρίς αλλαγή
του frozen envelope έγινε μία όρθια πλάκα με εγκοπή: Mega/perfboard και
keyed faceplate v2 πίσω `−X`, 4×BTS7960 εμπρός σε 2×2, με spacer 20 mm και
κατακόρυφες ψύκτρες. Δεν υπάρχει L298N· οι θέσεις είναι δύο drive, μία
FIT0186 tilt και μία spare.

Η κύρια πλάκα είναι 248×146 mm, 4 mm, με ονομαστικά gaps: Parkside 10 mm,
Ugreen 10 mm κάτω και πλάγια, χαμηλότερο control body 13 mm, πίσω keyed
plug+cable bend 14,3 mm και front BTS envelope 10 mm. Το Pi μετακινήθηκε
μόνο εντός του ελεύθερου κάτω κέντρου σε shelf 100×80 με clear
98×74×46· το top του είναι Z114 και ο καλωδιοδιάδρομος αρχίζει Z124.
Τα βύσματα πίσω κατεβαίνουν με καμπύλη 20 mm κάτω από την πλάκα και
ενώνονται με τα BTS στον διάδρομο X−260…−130/Y−35…19/Z124…149 προς την
υφιστάμενη εμπρός καλωδιοπλάτη.

Οι πλατιές v1 rack sockets αντικαταστάθηκαν από μικρά M3 bosses χωρίς
αλλαγή εξωτερικής βάσης ή οπών σασί. Δύο side frames συγκρατούν την πλάκα
με 2×M4. Οι θετικοί panel posts παρακάμπτουν το Ugreen στο Y84 και ο
αρνητικός εμπρός post παρακάμπτει το front BTS field. Ως μικροδιορθώσεις
του οπτικού ελέγχου διορθώθηκαν η φορά αυτού του αρνητικού detour και η
συνέχεια του βοηθητικού καλωδιοδιαδρόμου έως X−130.

Κρατήθηκαν byte-identical από v1: control panel Ø22,5/12,3/16,3,
cable bulkhead, bulkhead foot, MPU mount και chassis spacers. Κρατήθηκε η
γεωμετρία pins/C-clips. Καταργήθηκαν mega/perf/BTS shelves, όλοι οι
stacking pillars/couplers, το παλιό Pi shelf, το v1 coupon και το pin pack
δύο BTS.

Παράχθηκαν 15 manifold STL στο `cad/rear-pod/stl-v2/`, όλα με printable
XY≤250 mm. Το μοναδικό coupon αφορά τη νέα σιαγόνα 4,8 mm/πλάκα 4 mm.
Τα exact-solid `qa_priority_collision` και `qa_pi_collision` είναι empty.
README και εικόνες: `cad/rear-pod/stl-v2/README.md`. Πραγματικά plugs,
καμπύλες καλωδίων, ψύκτρες και fit coupon παραμένουν
`ΜΕΤΡΗΣΗ/ΕΠΙΒΕΒΑΙΩΣΗ ΧΡΗΣΤΗ`. Δεν έγινε firmware, Gazebo, commit, push ή
PR. Κατάσταση: `P2_76_UPRIGHT_BOARD_PRINTABLE_MEASUREMENT_GATED_PAUSED`.
ΠΑΥΣΗ.

### P2-77 — επισκευή torsion deck, MPU και τελική πρόταση rear pod

τι διαφέρει από το αναμενόμενο: τα τέσσερα τυπωμένα μισά δεν μπορούν να
σχηματίσουν δύο πλήρεις διαγωνίους στο ίδιο Z42…52, ενώ η λύση offset
+10/+16 mm τέμνει την πλατιά X240 και τον μελλοντικό πάτο. Επιλέχθηκε κοινός
χαμηλός κόμβος Z12…22: τα τέσσερα μισά επαναχρησιμοποιούνται μετά από κοπή
στα 129,0 mm και μία εγκάρσια οπή με τυπωμένο jig. Οι παλιοί joiners
καταργούνται. Exact-solid P2-73 bracket probe empty/PASS, clearance ≥18 mm
προς P2-68/πάτο, ≥30 mm προς reserved ballast-base Z≥52 και 2–3 mm πάνω από
την ονομαστική κάτω επιφάνεια του ski.

FEA 5 mm, με συντηρητική έκκεντρη μεταφορά endpoint: P2-77
0,6456/2,3894/2,3597 mm έναντι P2-65 0,6455/2,2118/2,1954 mm, δηλαδή
+0,01/+8,03/+7,49%. Μία μόνο διαγώνιος δίνει
0,6454/2,3754/2,3542 mm και −0,01/+7,40/+7,23%.

Το MPU μεταφέρθηκε σε νέα άκαμπτη κατακόρυφη βάση στην εσωτερική παρειά
της +Y ράγας, στις υπάρχουσες οπές X100/X125. Για την τελική rear-pod
έκδοση προτάθηκε Ugreen άνω +Y, όλα τα electronics connectors προς −X,
τέσσερις ευθείς στύλοι και 12 mm τοπικό E-stop rib. Στα 30 N οι λυγισμένοι
στύλοι δίνουν 0,035 mm κατακόρυφα/0,089 mm πλάγια με άκαμπτο panel, οι
ευθείς 0,019/0,051 mm. Η πλάκα 4 mm μόνη της δεν εγγυάται ≤1 mm· το 12 mm
rib δίνει συνολικό screen 0,132 mm.

Παράχθηκαν μόνο τρία νέα manifold STL στο `stl-p277/`. Η rear-pod v2 δεν
άλλαξε. Δεν έγινε firmware, Gazebo, commit, push ή PR. Κατάσταση:
`P2_77_UNDERDECK_NODE_PRINTABLE_PROOF_LOAD_GATED_PAUSED`. ΠΑΥΣΗ.

### P2-78 — socket/sleeve torsion deck χωρίς εύθραυστα T-clamps

τι διαφέρει από το αναμενόμενο: τα `x240_t_clamp`/`x300_t_clamp`
καταργήθηκαν από το active set. Τα front terminals απέκτησαν μονολιθικά
κούφια sockets, τα crossbar halves έγιναν 130 mm και τα terminal sleeves
120 mm με 60 mm εμπλοκή ανά μέρος. Η διεπαφή μετακινήθηκε από Y135 σε Y130,
ώστε η Y100 να είναι ταυτόχρονα grid και lock και να μη μείνει ligament
0,5 mm ανάμεσα σε δύο Ø4,5 οπές.

Το torsion deck έγινε δύο κοντές 20×15 πλευρικές διαγώνιες
X240/Y±150→X300/Y±125. Η FEA-optimal αντίστροφη φορά απορρίφθηκε επειδή
έτεμνε τον future basket floor 0,08…0,45 mm. Η επιλεγμένη φορά δίνει στα
5 mm mesh 0,6427/2,1770/2,1840 mm για vertical 2g / launch / racking,
έναντι 0,6454/2,3754/2,3542 mm του P2-77 single diagonal. Mesh delta
10→5 mm ≤0,90%.

Και τα 13 exact-solid gates είναι empty: socket insertion, hole axes
Y75/100/125/150/160, braces έναντι frame/ski/feed/floor/ballast/MPU/cable,
terminal sockets έναντι FIT0403/encoder/tyres και physical ground zone.
Η κάτω παρειά brace είναι CAD Z20,5 / περίπου physical Z18,5. Εξήχθησαν
μόνο τα 7 αλλαγμένα manifold STL, περίπου 372,9 g. Αναφορά:
`docs/mechanism/torsion-deck-sleeve-p278-el.md`. Χωρίς firmware, Gazebo,
commit, push ή PR. Κατάσταση:
`P2_78_SOCKET_SLEEVE_SHORT_BRACES_PRINTABLE_PAUSED`. ΠΑΥΣΗ.

### P2-79 — ακαμψία στήριξης rear pod v2

τι διαφέρει από το αναμενόμενο: η βάση έγινε 8 mm nominal, 7 mm στη
full-width σέλα `X−190…−140` της πίσω τραβέρσας και 6,5 mm κάτω από το
Parkside drawer sweep. Τέσσερις rear-only νευρώσεις ανά μισό κατεβαίνουν
έως CAD Z43 / περίπου physical Z41. Οι τέσσερις 3 mm spacers δεν είναι
πλέον φέρουσα διεπαφή.

Δύο νέα corner ties χρησιμοποιούν τις υπάρχουσες πλευρικές M4
X−135/Z37 με μακρύτερες βίδες. Η πρώτη διαγώνια διαδρομή έμπαινε στη ράγα·
διορθώθηκε ώστε να περνά πάνω από Z52 και να κατεβαίνει μόνο έξω από
Y±245. Η Parkside pocket επεκτάθηκε μέχρι X−272 και κατά 0,5 mm πέρα από
τα Y boundaries, επειδή ο πλήρης drawer sweep —όχι μόνο η στατική battery
θέση— αποκάλυψε τοπική τομή 1 mm.

Side frames, κάτω post clevises και panel retention έγιναν διαμπερή M4 με
ροδέλα/nyloc. Οι posts έγιναν Ø23 και το positive detour Y90. Δεν υπάρχει
τυπωμένο σπείρωμα σε φέρον σημείο. Το envelope και όλες οι electronics/
control θέσεις έμειναν αμετάβλητα.

Linear PETG screen: στα 30 N vertical 0,886 mm με full-panel effective
width, αλλά 3,051 mm στο 40 mm strip sensitivity· στα 20 N lateral και
fore/aft 0,628 mm. Post bending stress 0,829 MPa. Στα 20 N fore/aft το
M4 couple είναι 194,7 N, περίπου 97,3 N ανά tension side. Η εκτιμώμενη
φέρουσα μάζα είναι 1,43 kg.

Exact-solid empty/PASS: chassis εκτός των δηλωμένων saddle/lug mating
faces, πλήρες Parkside drawer sweep, Parkside/Ugreen/controls, Pi και
symmetric hole-axis difference. Ξαναεξήχθησαν μόνο 5 manifold STL:
`base_left/right`, `side_frame`, `panel_posts`, `corner_tie`. README/εικόνες:
`cad/rear-pod/stl-v2/README.md`. Χωρίς firmware, Gazebo, commit, push ή PR.
Κατάσταση: `P2_79_STIFFENED_BASE_M4_JOINTS_PRINTABLE_PANEL_WIDTH_GATED_PAUSED`.
ΠΑΥΣΗ.

### P2-80 — κοινός πίσω φέρων σκελετός rear pod v2 / panels F

τι διαφέρει από το αναμενόμενο: η αρχική ανάλυση επέλεξε υβριδικό cage με
αλουμινένια flats, αλλά το αλουμίνιο δεν είναι διαθέσιμο/εύχρηστο για την
κατασκευή και αφαιρέθηκε από την κατεύθυνση release. Το P2-80 μένει μόνο ως
μελλοντικό πλήρως τυπωμένο, stackable PETG cage με διαμπερείς M4.

Box sections περίπου 35×35 mm, που θα χρειάζονταν για να πλησιάσουν την
ακαμψία των μεταλλικών στοιχείων, δεν χωρούν στην ελεύθερη λωρίδα
`Y161…167`. Οι θέσεις και διατομές των κολονών θα ξανακριθούν μαζί με τα
panels F· δεν παγώνουν τώρα. Η παλιά υβριδική ανάλυση διατηρείται μόνο ως
benchmark, όχι ως κατασκευαστική πρόταση.

Linear screen με electronics + μελλοντικές 2×Parkside: κορυφή 0,217 mm στα
30 N κατακόρυφα, 0,359 mm στα 20 N πλάγια και 0,280 mm στα 20 N εμπρός/πίσω.
Η εκτιμώμενη φέρουσα μάζα είναι 0,84 kg. Τα exact-solid checks έναντι chassis,
κινούμενου μηχανισμού −10…+20°, electronics/Parkside και panels F είναι
empty/PASS για το τότε concept. Δεν παράχθηκε STL. Αναφορά:
`docs/mechanism/rear-cage-p280-el.md`. Κατάσταση:
`P2_80_FUTURE_PRINTED_STACKABLE_PETG_CAGE_COLUMN_LAYOUT_DEFERRED_NO_STL_PAUSED`.
ΠΑΥΣΗ.

### P2-81 — νεύρωση control panel και επίπεδη κορυφή posts

τι διαφέρει από το αναμενόμενο: το νέο 12 mm συνολικό section περνά τον
συντηρητικό υπολογισμό 40 mm effective width: 0,1882 mm κάμψη panel +
0,0024 mm αξονική βράχυνση τεσσάρων posts = 0,1906 mm στα 30 N, έναντι
ορίου 0,300 mm. Τα νέα ribs τελειώνουν στο global Z347 και καθαρίζουν τα
control bodies, Ugreen και carrier.

Ο exact-solid έλεγχος αποκάλυψε την κληρονομημένη τομή του P2-79: τα Ø23
σφαιρικά άνω shoulders εισχωρούσαν στο παλιό skin στο Z348…351. Οι κορυφές
έγιναν επίπεδοι Ø23 ώμοι ακριβώς στο local Z198/global Z347. Μόνο ο Ø8,2
οδηγός συνεχίζει στην αμετάβλητη Ø8,6 οπή και η σύσφιξη είναι διαμπερής M4
με ροδέλα/nyloc.

Exact-solid διείσδυση έναντι control bodies, Ugreen, carrier και posts:
empty/PASS. Η μοναδική post–panel επαφή είναι οι τέσσερις συνεπίπεδοι ώμοι
στο Z347. Εξήχθησαν μόνο τα manifold `control_panel.stl` και
`panel_posts.stl`. README/εικόνα: `cad/rear-pod/stl-v2/README.md`. Χωρίς
firmware, Gazebo, commit, push ή PR. Κατάσταση:
`P2_81_RIBBED_PANEL_FLAT_POST_SHOULDERS_EXACT_SOLID_PASS_PAUSED`.
ΠΑΥΣΗ.

### P2-82 — active print manifest και recoverable archive

τι διαφέρει από το αναμενόμενο: το παλιό `modular-chassis-v2.scad` δεν
μπορούσε να αρχειοθετηθεί πλήρως, επειδή το ενεργό test-only
`drive-motor-mount/m3-side-nut-keeper.scad` το χρησιμοποιεί για QA envelopes.
Έμεινε στη θέση του μόνο ως compatibility dependency, όχι ως print release.
Παρομοίως τα δύο παλιά Gamma STL έμειναν στο `stl-v2/` επειδή το v3 τα
εισάγει ως προαιρετικό visual context· δεν μπήκαν στο active manifest.

Δημιουργήθηκε το `cad/ACTIVE-PRINT-SET.md`. Το P2-77 centre node και drill
jig κρίθηκαν superseded από τα δύο P2-78 side braces· μόνο το P2-77
rail-side MPU mount μένει ενεργό. Το παλιό ballast tab αρχειοθετήθηκε μαζί με
την ακυρωμένη plywood/metal λύση. Τα rear-pod v0/v1, τα ανενεργά rear-pod
MPU/chassis spacers, παλιά exports και unreferenced review images μεταφέρθηκαν
σε recoverable `archive/` με σήμανση «ΜΗΝ ΤΥΠΩΣΕΙΣ». Δεν διαγράφηκε αρχείο.

Το ενεργό rear-pod README κρατά μόνο τις βίδες P2-79/P2-81· οι M3×16/M3×10
του P2-76 αφαιρέθηκαν. Το P2-80 καταγράφεται μόνο ως μελλοντικό πλήρως
τυπωμένο PETG cage, με νέα κρίση θέσεων κολονών μαζί με τα panels F.

Dependency/link audit: 0 missing. Dry-run exports: 18 chassis v3, 1 MPU,
4 ski-mount και 12 rear-pod meshes PASS χωρίς αλλαγή release αρχείων. Render
smoke-check: 13/13 ενεργά CAD/concept sources PASS. Δεν βρέθηκε matching
ανοιχτό GitHub issue. Το GitHub Projects query δεν ήταν διαθέσιμο επειδή το
token δεν έχει `read:project`, άρα δεν δημοσιεύτηκε status comment σε άσχετο
issue. Χωρίς firmware upload, Gazebo, PR, merge ή force push. Κατάσταση:
`P2_82_ACTIVE_PRINT_MANIFEST_ARCHIVE_RENDER_EXPORT_PASS`. ΠΑΥΣΗ.

### P2-83 — επισκευή corner tie και μόνιμος έλεγχος STL bodies

τι διαφέρει από το αναμενόμενο: το release `corner_tie.stl` του P2-79
επιβεβαιώθηκε ότι είχε τρία θετικά, ασύνδετα σώματα. Η επάνω διαδρομή του
έτεμνε τη βάση και η κεφαλή της κατακόρυφης M4 βρισκόταν μέσα στο Parkside
reservation. Η πρώτη εσωτερική L εκδοχή αποκάλυψε επίσης exact-solid τομές
με το crossbar arm του corner clamp και με το άκρο της πίσω τραβέρσας.

Η τελική μονοκόμματη L βρίσκεται κάτω από τη βάση, `X−139,5…−126,5`,
`Z29…51`, και χρησιμοποιεί την υπάρχουσα πλευρική M4 `X−135/Z37`. Ένα
τυπικό M4 washer shim 1 mm γεφυρώνει το tolerance gap μέχρι την εσωτερική
παρειά `Y±195`. Η νέα M4 βάσης είναι στο `X−134/Y±145`, με 90° countersink
flush στο τοπικό top `Z57,5`, 0,5 mm κάτω από το Parkside sweep. Δεν
προστέθηκε οπή στο σασί. Physical ground clearance είναι περίπου 27 mm.

Exact-solid `tie_base`, `tie_chassis` με πλήρες frame και rear
FIT0403/motor/cable/wheel envelopes, `drawers` με κεφαλές/ροδέλες/nyloc και
`ground Z<15`: empty/PASS. Επανεξήχθησαν μόνο `base_left.stl`,
`base_right.stl`, `corner_tie.stl`, όλα ένα σώμα. Προστέθηκε κοινός
`cad/check-stl-components.py` στα rear-pod v2, chassis v3, ski-mount και
P2-77 MPU exports. Full dry-run: rear pod 12 generated + 2 retained, chassis
18, ski 4, MPU 1 PASS. Όλα τα πρόσθετα ενεργά fixtures του manifest PASS.
Τα δηλωμένα print plates είναι 4/27/32/18/2 bodies· κάθε άλλο ενεργό STL
είναι ένα body. Χωρίς firmware, Gazebo ή PR. Κατάσταση:
`P2_83_ONE_BODY_INNER_CORNER_TIE_EXACT_SOLID_AND_EXPORT_GATE_PASS`. ΠΑΥΣΗ.

### P2-84 — πίσω panels F πάνω στο rear pod v2

τι διαφέρει από το αναμενόμενο: το παλιό F δεν ήταν release geometry για το
v2. Το `rear_lo` περίμενε τα καταργημένα P2-70 feet/ears και το ονομαστικό
`R424,5` με πάχος προς τα μέσα άφηνε μόνο περίπου 0,6 mm στις γωνίες. Το
P2-84 κρατά το `R424,5` ως εσωτερικό datum και χτίζει τα 2,5 mm PETG προς τα
έξω (`R427` ορατή εξωτερική ακτίνα), χωρίς καμία αλλαγή στο frozen envelope
του pod. Το exact-solid 3 mm keepout είναι empty/PASS.

Παρήχθησαν οκτώ release-candidate, single-body STL κάτω από 250×250 mm:
`rear_lo` L/R με ενσωματωμένα Parkside frames, `rear_hi` L/R με παράθυρο και
πλέγμα ±60°, `deck_rear` L/R και ανοιγόμενες Parkside doors L/R. Οι πρώτοι
μεντεσέδες βγήκαν ως τρία σώματα και το σταθερό knuckle ακουμπούσε
συνεπίπεδα το drawer boundary `Y161`. Διορθώθηκαν σε ενιαίο φύλλο/knuckles,
με άξονα `Y±165,5`, radial offset και θετικό κενό 0,5 mm από το exact sweep.
Τα `rear_hi` έχουν δύο κάτω locating tongues ανά μισό και τέσσερις συνολικά
top-access M3 σε captive nuts κάτω από το deck· αφαιρούνται χωρίς λύσιμο του
pod και αποκαλύπτουν Mega/perfboard.

Στο pod άλλαξαν μόνο τα επιτρεπόμενα interfaces: δύο rear-facing captive M3
ανά `base_left/right`, δύο lands κάτω από το πίσω χείλος του control panel,
και οπή status LED Ø8 στο local `X+42/Y+70`. Οι Ø22,5/12,3/16,3 και οι
τέσσερις οπές στήριξης έμειναν αμετάβλητες. Επανεξήχθησαν μόνο
`base_left.stl`, `base_right.stl`, `control_panel.stl`, όλα ένα σώμα.

Exact-solid empty/PASS: F↔pod 3 mm, frame/chassis, FIT0403/wheels, moving
mechanism `−10…+20°`, fixed drawer sweep, doors ανοικτές 110°, ground `Z<15`,
control bodies/LED, Ugreen, posts, carrier και base interfaces έναντι
Parkside. Το γενικό `qa_all_reserved_collision` δεν χρησιμοποιείται ως
P2-84 gate επειδή περιλαμβάνει την εκ προθέσεως επαφή των Pi support bosses
με το Pi reservation· ο ειδικός νέος `qa_base_parkside_collision` είναι
empty/PASS. Οι εικόνες assembly, rear και service βρίσκονται στο
`cad/body-panels/p284-rear-f-*.png`.

Τα STL σημειώνονται **σχεδιασμένο, αναμονή / ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ**. Για
το μελλοντικό βήμα πλαϊνών/εμπρός χρειάζεται νέα στήριξη του `c2`, επειδή το
T-clamp `X240` καταργήθηκε στο P2-78, και περιμένει basket mockup και
pile-retention test.
Χωρίς firmware, Gazebo ή PR. Κατάσταση:
`P2_84_REAR_F_RELEASE_CANDIDATE_EXACT_SOLID_PASS_NOT_FOR_PRINT_YET`.
ΠΑΥΣΗ.

### P2-85 — στήριξη βατήρα: κλειστή σχισμή και διχάλα τοιχώματος

τι διαφέρει από το αναμενόμενο: η ζητούμενη δεύτερη βίδα ανά πλευρά μέσα
από τον βατήρα δεν χωρά. Δοκιμή X≈347/Z≈26 (t32°): η κεφαλή μέσα στον βατήρα
τέμνει τους οδηγούς throat (Y41,5…46,5, 1104 mm³) και το nyloc τέμνει το
D200 sweep (83 mm³). Πιο πίσω, το boss πέφτει στα rails Y±55. Ο οδηγός throat
απέχει 0,5 mm από το τοίχωμα σε όλο το τόξο.

Αιτία της φυσικής θραύσης του P2-73 bracket: η σχισμή `mount_z+10±10`, Ø4,5,
έφτανε στο +22,25 πάνω από web που τελείωνε στο +16. Έμεναν δόντια
2,75×6 mm τυπωμένα όρθια (στρώσεις κάθετα), και μόνο μία M4 ανά πλευρά, που
λειτουργούσε ως άξονας περιστροφής.

Λύση: bracket με κλειστή σχισμή (κέντρα Z64,3…98,3, ≥6 mm πάνω/κάτω,
10/12 mm πίσω/εμπρός), web 8 mm (Y61 κρατήθηκε), R2/R4 ακτίνες, νευρώσεις
πέλμα→web. Τυπώνεται στην παρειά Y61. Συν μία διχάλα ανά πλευρά: hub στη M4
X315 (αντικαθιστά τον retrofit spacer), χείλη στις δύο ακμές του τοιχώματος
στο t26…35° (διάκενο 0,3 mm, μοχλοβραχίονας 48,1 mm), κλείδωμα με M4×30
DIN 933 στην ίδια σχισμή, 14 mm πάνω. Ο ήδη τυπωμένος βατήρας χωρίς bosses
δουλεύει χωρίς τρύπημα. Ο ενεργός βατήρας είναι πλέον χωρίς bosses.

Exact-solid, ονομαστικό και coupled h 0…20 / −10…+20°: selector Y±55
(σταθερό και κινούμενο), hard sweep, throat, πλήρης launcher, drive, feed,
frame, ground, P2-78 braces: EMPTY. Περιθώρια ≥3 mm (hard/drive/feed),
≥2 mm (selector), 1,5 mm (throat, εκ σχεδιασμού). Αντοχή 50 N μπάλα (t=0) +
30 N στη μύτη: 3,67 N·m ανά πλευρά. Bracket in-layer ≤10,4 MPa (έδραση
συντηρητικά), across-layers 0,9 MPa. Διχάλα 6,8 / 1,8 MPa. Τριβή Z SF≈13.

Επανεξήχθησαν ski, brackets L/R και νέες διχάλες L/R, όλα ένα σώμα.
`ski-boss-spacer` καταργείται. Αναφορά: `docs/mechanism/p285-ski-bracket-el.md`.
Χωρίς firmware, Gazebo ή PR. Κατάσταση:
`P2_85_CLOSED_SLOT_WALL_FORK_EXACT_SOLID_AND_STRENGTH_PASS_PAUSED`. ΠΑΥΣΗ.

#### Ενσωμάτωση P2-85

Ο ενεργός exporter αντικαταστάθηκε από το P2-85 script και το
`ski-boss-spacer.stl` αφαιρέθηκε. Export/component gate: 5/5 STL ακριβώς
ένα σώμα. Το `p285-qa.sh` πέρασε όλα τα nominal/coupled probes και τα νέα
integration probes: brackets/nyloc envelope (bracket έως Z106,6, nyloc έως
Y82) έναντι P2-84 rear panels και πλήρους F concept front/side/rear:
EMPTY/PASS. Τα δύο 3MF slicer projects του χρήστη έμειναν εκτός commit.
