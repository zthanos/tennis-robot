# Flywheel launcher v0 — διερευνητικό packaging study

Ημερομηνία baseline: 2026-08-21

## Σκοπός

Το παρόν ξεκινά τη διερεύνηση του launcher που θα συνεργάζεται με το ενεργό
Option A intake και το basket v2.1. Δεν είναι σχέδιο παραγωγής ή λίστα αγοράς.
Ο στόχος του v0 είναι να καθορίσει το μηχανικό envelope που επηρεάζει την τελική
μορφή του ρομπότ και να απομονώσει τις αποφάσεις που χρειάζονται bench tests.

Η πλήρης ροή παραμένει:

```text
Option A intake -> basket v2.1 -> manual lift/tilt -> gravity singulator
                -> spring-assisted transfer -> flywheel nip -> guarded guide
```

Το intake τελειώνει στο basket. Δεν επιτρέπεται να χρησιμοποιηθεί ως launcher ή
να αλλάξει η bench-proven γεωμετρία του για να λύσει πρόβλημα τροφοδοσίας.

## Αρχιτεκτονική v0 — επιλεγμένη χαμηλή baseline

Απόφαση χρήστη (2026-08-21): η κύρια κατεύθυνση είναι δύο flywheels
**δίπλα-δίπλα**, με κατακόρυφους άξονες και οριζόντια τροχιά μπάλας. Η παλιά
διάταξη ενός τροχού πάνω από τον άλλο (`over_under`) παραμένει μόνο ως CAD
comparison.

- Ίδιες ταχύτητες αριστερού/δεξιού τροχού δίνουν την αρχική ευθεία βολή.
- Διαφορά ταχύτητας επιτρέπει sidespin. Δεν υπάρχει άμεσος έλεγχος
  topspin/backspin στην επιλεγμένη διάταξη.
- Το skid-steer του ρομπότ αναλαμβάνει το οριζόντιο aiming· δεν προστίθεται yaw
  turret στο πρώτο prototype.
- Ολόκληρο το flywheel cradle παίρνει ρυθμιζόμενο pitch. Το v0 εξετάζει
  `10–35°`, όχι ακόμη actuator.
- Το breech δέχεται ακριβώς μία μπάλα από ξεχωριστό singulator.
- Μετά το nip υπάρχει κοντός, πλήρως guarded οδηγός και όχι μακρύ barrel.

Η χαμηλή διάταξη δεν μειώνει από μόνη της τη θεωρητική εμβέλεια: αυτή
καθορίζεται κυρίως από wheel speed, compression, motor droop και pitch. Ο
συμβιβασμός είναι ότι η `over_under` διάταξη θα μπορούσε να χρησιμοποιήσει
backspin για πρόσθετο carry. Η baseline κερδίζει χαμηλό envelope, καθαρό LiDAR
scan plane και πολύ ευκολότερη σύνδεση με τον feeder.

## Provisional envelope — όχι manufacturing dimensions

| Παράμετρος | v0 | Διερευνητικό εύρος |
|---|---:|---:|
| Διάμετρος μπάλας αναφοράς | 66 mm | 65.4–68.6 mm |
| Διάμετρος flywheel | 200 mm | 180–250 mm |
| Πλάτος flywheel | 50 mm | 40–70 mm |
| Nip gap | 58 mm | 54–62 mm |
| Ονομαστική συμπίεση μπάλας | 8 mm | 4–12 mm |
| Καθαρό feed/guard channel | 90 mm | >= 85 mm |
| Pitch | 20° | 10–35° |

Με wheel radius `0.10 m`, η θεωρητική περιφερειακή ταχύτητα είναι:

```text
1500 RPM -> 15.7 m/s
2500 RPM -> 26.2 m/s
3500 RPM -> 36.7 m/s
```

Αυτές δεν είναι ταχύτητες εξόδου της μπάλας. Slip, compression, wheel inertia,
motor droop και spin τις μειώνουν. Δεν επιλέγεται μοτέρ ή ESC πριν οριστεί το
shot envelope και μετρηθεί bench prototype με chronograph/high-speed video.

## Manual lift/tilt basket — νέα κύρια κατεύθυνση

Απόφαση διερεύνησης χρήστη (2026-08-21): κρατάμε το ίδιο αφαιρούμενο basket
και, πριν από τη λειτουργία launch, ο χειριστής το μετακινεί σε ψηλότερη θέση
ώστε η τροφοδοσία να γίνεται με βαρύτητα.

Η λύση είναι ελκυστική επειδή καταργεί τον ενεργό elevator όλου του φορτίου.
Με περίπου 50 μπάλες το φορτίο μπαλών είναι περίπου 3 kg· μαζί με το
wire-mesh basket το κινούμενο σύνολο αναμένεται περίπου 3.5–4 kg πριν από τα
πραγματικά ζυγίσματα. Αυτό είναι λογικό για υποβοηθούμενο χειροκίνητο
μηχανισμό.

Η launch θέση πρέπει να έχει **ανύψωση και κλίση**. Μόνο η ανύψωση ενός
σχεδόν επίπεδου πατώματος δεν δημιουργεί αξιόπιστη ροή προς μία έξοδο.

```text
COLLECT position
  basket χαμηλά και οριζόντιο
  intake opening ενεργό
  launcher mechanically/electrically inhibited

LAUNCH position
  intake disabled
  hood ανοικτό ή αποδεσμευμένο
  basket ανυψωμένο 100 mm και κεκλιμένο 12° στην CAD baseline
  gated outlet docked σε funnel/singulator και spring-assisted feeder
```

### Αρχική side-port διερεύνηση — superseded

Ο πρώτος υποψήφιος ήταν ένα **χαμηλό πλευρικό άνοιγμα που γίνεται
λειτουργικό μόνο στην ανυψωμένη θέση**. Στη θέση collect το port καλύπτεται και
η basket-integral πόρτα παραμένει spring-closed. Στο τέλος της ανύψωσης:

1. το basket κλειδώνει σε θετικό mechanical stop,
2. το port ευθυγραμμίζεται με stationary feed dock,
3. το dock στηρίζει περιμετρικά το άνοιγμα,
4. fixed cam ανοίγει την πόρτα,
5. μόνο τότε επιτρέπεται ο singulator.

Η πόρτα δεν βασίζεται στο chassis ως το μοναδικό κλείσιμο: πρέπει να μένει
fail-closed όταν το basket αφαιρείται γεμάτο. Η κλίση launch μπορεί να είναι
μικρή πλευρική κλίση προς το port αντί μεγάλης διαμήκους ανατροπής.

Το **bottom outlet** κρατιέται ως εναλλακτική B, αλλά όχι ως πρώτο prototype.
Το σημερινό basket έχει μόλις `20 mm` ground clearance κάτω από το πάτωμα. Ένα
κάτω gate θα βρίσκεται στη χειρότερη θέση για χώμα/νερό/χτυπήματα, θα αλλάξει
τη validated skid επιφάνεια και θα μπορούσε να αδειάσει όλο το φορτίο με μία
αστοχία. Θα επανεξεταστεί μόνο αν το side-port gravity test δείξει bridging που
δεν λύνεται με κλίση και απλό agitator.

Ο μηχανισμός ανύψωσης πρέπει να έχει δύο οδηγούς ή four-bar ώστε να μην
στρεβλώνει, gas springs/constant-force springs ή αντίβαρο, και θετικά μηχανικά
locks και στις δύο θέσεις. Συρματόσχοινο ή gas spring δεν θεωρείται μόνο του
structural stop. Το basket πρέπει να παραμένει αφαιρούμενο χωρίς εργαλεία.

## Το πραγματικό packaging bottleneck

Το flywheel pair είναι σχετικά εύκολο να τοποθετηθεί ως ανεξάρτητο module. Το
δύσκολο μέρος είναι η εξαγωγή μίας μπάλας από το επίπεδο, αφαιρούμενο basket
χωρίς να δημιουργούνται bridges/jams μέσα σε φορτίο έως περίπου 50 μπάλες.

Το τρέχον chassis αφήνει:

- basket `x=20…420 mm`, `y=±140 mm`, rim `z=250 mm`,
- battery `x=-226…-60 mm`, κεντραρισμένη,
- περιορισμένο κεντρικό κενό πίσω από το basket,
- ελεύθερες πλευρικές λωρίδες που πρέπει όμως να ελεγχθούν έναντι drive pods.

Η manual lift/tilt απόφαση αφαιρεί την ανάγκη για powered ball elevator. Με τη
νεότερη απαίτηση ότι ο launcher βρίσκεται μπροστά, συγκρίνονται:

1. **Existing front opening (baseline):** το σημερινό intake entrance γίνεται
   outlet μόνο στην ανυψωμένη/κεκλιμένη θέση. Funnel/singulator αφήνει μία
   μπάλα σε spring-assisted σωλήνα που ανεβαίνει σε crest και κατεβαίνει στο
   flywheel nip.
2. **Sliding front launcher (fallback):** το flywheel cradle παρκάρει εμπρός
   στη συλλογή και κυλά σε rails προς το feed για launch, με positive locks.
3. **Bottom gated outlet:** παραμένει δυσμενές λόγω ground clearance,
   contamination και fail-safe απαιτήσεων.

Το v0 CAD μοντελοποιεί το κοινό flywheel cradle, το feed-interface envelope και
τις δύο ακραίες θέσεις του manual lift/tilt basket. Η ακριβής άρθρωση και τα
locks θα σχεδιαστούν αφού επιβεβαιωθούν η πλευρά του port, η απαιτούμενη κλίση
και η θέση του launcher.

## Shot envelope που πρέπει να παγώσει πριν από αγορά

Χρειάζονται τέσσερις λειτουργικές τιμές:

- ελάχιστη και μέγιστη οριζόντια απόσταση βολής,
- επιθυμητό ύψος περάσματος πάνω από το φιλέ,
- επιτρεπτό sidespin και αν απαιτείται τελικά topspin/backspin,
- ρυθμός βολών (balls/min) και μέγεθος burst.

Από αυτές θα προκύψουν wheel diameter/inertia, RPM, motor power, pitch range,
guard envelope, τροφοδοσία και θερμικό φορτίο. Μέχρι τότε οι αριθμοί του v0
είναι αποκλειστικά για collision/shape exploration.

## Safety constraints από την πρώτη έκδοση

- Πλήρες fixed guard γύρω από flywheels, belts/couplers και exit pinch points.
- Ανεξάρτητο launcher enable, hard emergency stop και ασφαλής εκφόρτιση ισχύος.
- Lid/door interlock στο service opening.
- Spin-up επιτρέπεται μόνο με έγκυρο pitch, κλειστό guard και καθαρό exit zone.
- Feed gate fail-closed: καμία μπάλα δεν μπορεί να πέσει στο nip χωρίς εντολή.
- Ξεχωριστός launcher MCU παραμένει η προτιμώμενη control boundary.

## Gates εξέλιξης

```text
L0  Envelope CAD
    No collisions, service access, guarded ball path.

L1  Passive bench rig
    Hand-fed ball passes through adjustable unpowered nip without trapping.

L2  Powered single-shot rig
    Repeatable exit speed and spin; current, RPM droop and temperature logged.

L3  Pitch cradle + guards
    Repeatable trajectories at the agreed shot envelope.

L4  Manual lift/tilt + singulator bench
    Safe two-position lock, acceptable hand force, 50-ball gravity feed,
    no double-feed and representative jam recovery.

L5  Robot integration
    CoG, battery sag, interlocks and full-system safety validation.
```

Το άμεσο επόμενο mechanical deliverable είναι bench geometry για
funnel/singulator, spring stroke και σωλήνα transfer. Το rail layout του
launcher μένει μόνο ως A/B εναλλακτική μέχρι να αποδειχθεί ότι χρειάζεται.

## Gazebo packaging variant

Η προσαρμογή στη φυσική προσομοίωση είναι selectable και δεν αντικαθιστά το
bench-proven baseline:

```bash
ROBOT_PACKAGING_VARIANT=baseline ros2 launch tennis_robot sim.launch.py headless:=true
ROBOT_PACKAGING_VARIANT=option-a-collect ros2 launch tennis_robot sim.launch.py headless:=true
ROBOT_PACKAGING_VARIANT=compact  ros2 launch tennis_robot sim.launch.py headless:=true
```

Το `option-a-collect` είναι το ενδιάμεσο physics gate πριν από την προσομοίωση
του launcher. Διατηρεί τα bench-proven datums του Option A (`intake nip
x=540 mm` και segmented ramp περίπου `x=524...450 mm`), αλλά χρησιμοποιεί την
πίσω θέση μπαταρίας, το κατακόρυφο motion tray, το Pi case, τον ξεχωριστό buck
converter και το full basket load του compact packaging. Το flywheel είναι
απενεργοποιημένο εξ ορισμού σε αυτή την παραλλαγή, ώστε το collection result να
μην επηρεάζεται από provisional launcher collisions.

Στις 2026-08-23 το generated model πέρασε `check_urdf` με συνολική μάζα
`26.200 kg` και CoM `x=-28.7 mm`, `y=-2.8 mm`, `z=93.0 mm`. Deterministic
native Gazebo sweep πέντε επαναλήψεων, με gap `56 mm`, wheel radius `60 mm`,
tilt `35 deg` και wheel speed `25 rad/s`, έδωσε:

- `5/5` runs με `6/6` required intake/release criteria,
- `5/5` runs με `8/8` basket-entry/retention criteria,
- μηδενικό stall σε όλα τα runs,
- peak inward transport `0.745...0.919 m/s`,
- την target μπάλα παρούσα στο basket στο τελευταίο pose sample κάθε run.

Το launch-ballistics gate δεν αξιολογείται σε αυτή τη φάση: η αποτυχία του με
flywheel disabled είναι αναμενόμενη και δεν αποτελεί collection failure.

Το `compact` μεταφέρει το functional group κατά `-100 mm`, ευθυγραμμίζει τα
intake wheels/cheeks με τα πραγματικά CAD datums, μετακινεί την μπαταρία πίσω
και προσθέτει τα mass/envelope models του κατακόρυφου motion tray, του Pi case,
του ξεχωριστού buck converter και του εμπρός launcher. Το basket περιλαμβάνει
φορτίο `45 x 57 g = 2.565 kg`. Το launcher έχει δύο ανεξάρτητα flywheel joints
και controller `flywheel_velocity_controller`; το nominal Gazebo model
χρησιμοποιεί `R=100 mm`, nip `58 mm`, pitch `20 deg` και όριο `320 rad/s`.

Το mass audit στο zero-joint pose δίνει:

- baseline: `20.385 kg`, CoM `x=-29.9 mm`, `z=83.7 mm`,
- compact/full-load: `29.800 kg`, CoM `x=+11.3 mm`, `z=107.8 mm`.

Άρα το compact hypothesis προσθέτει `9.415 kg`, μετακινεί το CoM περίπου
`41 mm` εμπρός και `24 mm` ψηλότερα, χωρίς αλλαγή wheelbase ή chassis. Σε
ίδιο controller command (`0.3 m/s`, `+/-0.8 rad/s`) το πρώτο A/B probe έδωσε
`0.447 m` έναντι `0.450 m` στην ευθεία και συμμετρική απόκριση στροφής
`+63.3/-64.6 deg` έναντι `+65.1/-66.0 deg` από wheel odometry. Η μικρή διαφορά
δεν αποτελεί ακόμη acceptance της πρόσφυσης: απαιτείται επανάληψη με Gazebo
ground truth και μετά loaded physical skid-steer test.

Η compact handoff collision και η ορατή επιφάνειά της είναι πλέον το ίδιο
segmented ramp, world `x~=360...320 mm`, πίσω από το wheel nip `x=370 mm`.
Έτσι το Gazebo δεν εμφανίζει ούτε προσομοιώνει την παλιά ράμπα μπροστά από τους
rollers. Η γεωμετρία παραμένει study-only και δεν αλλάζει κανένα αρχείο Option A.

## Πρώτο full-robot placement study

Το `cad/flywheel-launcher-v0/robot-integration.scad` εισάγει read-only την
ενεργή γεωμετρία Option A και τη συνδυάζει με basket v2.1, battery, 4WD wheel
envelopes, OAK-D/LiDAR references, lift guides και launcher. Τα αρχεία του
Option A και του basket δεν τροποποιούνται· όλα τα feed overlays υπάρχουν μόνο
στο integration study.

Απόφαση χρήστη (2026-08-21): ο launcher βρίσκεται **μπροστά** και χρησιμοποιεί
τη χαμηλή `side_by_side` διάταξη. Το side layout και η ψηλή `over_under`
διάταξη παραμένουν μόνο ως ιστορικές συγκρίσεις. Η baseline επαναχρησιμοποιεί
το υπάρχον εμπρός άνοιγμα του basket και spring-assisted transfer.

- Η μπαταρία και το πίσω LiDAR μένουν αμετάβλητα.
- Η σημερινή κεντρική OAK-D θέση συγκρούεται με το flywheel cradle και χρειάζεται
  νέα θέση· το v0 δείχνει προσωρινά την παλιά θέση ως κόκκινο ghost.
- Το Option A μένει αμετάβλητο.
- Το υπάρχον front opening τροφοδοτεί funnel/singulator χωρίς powered elevator.

Η καταγεγραμμένη baseline έχει:

- basket lift `100 mm` και launch tilt `12°`,
- basket outlet περίπου `z=142 mm`,
- flywheel nip `z=215 mm`,
- feeder crest `z=275 mm`,
- provisional launcher/exit envelope κάτω από περίπου `z=370 mm`,
- μέγιστο πίσω rim του ανυψωμένου basket περίπου `z=444 mm`,
- LiDAR scan datum `z=498 mm`.

Η σειρά ύψους είναι επομένως `launcher < raised basket rim < LiDAR`. Οι τιμές
είναι packaging datums και όχι manufacturing tolerances. Ο σωλήνας χρειάζεται
ενεργειακή ώθηση επειδή η έξοδος του basket παραμένει χαμηλότερα από το nip.

## Two-position cover/rail lift study

Το `cad/flywheel-launcher-v0/cover-lift-study.scad` αποτυπώνει τη συμφωνημένη
χειροκίνητη λειτουργία με `100 mm` travel και `12°` launch tilt:

- το basket παραμένει το πραγματικό basket v2.1,
- δύο fixed rails βιδώνονται/δένονται στο chassis,
- μεταλλικό carriage κινείται πάνω στα rails και φέρει το basket cradle,
- στην κάτω θέση το basket flange πατά απευθείας στο chassis,
- στην πάνω θέση τα carriage blocks πατούν σε fixed upper stops και
  ασφαλίζονται με transverse pins,
- το cosmetic cover και η χειρολαβή συνδέονται στο carriage αλλά δεν βρίσκονται
  στη load path του basket.

Η επιλεγμένη manual διεπαφή επιστρέφει στην αρχική κεντρική λαβή πάνω από το
basket. Η λαβή δένει στο moving carriage και όχι στο cosmetic hatch· το
removable basket hatch ανοίγει κατά τον χειρισμό. Ο εξωτερικός πλαϊνός lever και
το adjustable link αφαιρούνται από την κύρια CAD baseline και παραμένουν μόνο
ως απενεργοποιημένο legacy comparison. Το gas-spring envelope, τα positive
locks, καθώς και τα clevis hardpoints, swept keep-out, limit switches, driver
keep-out και cable route για μελλοντικό electric linear actuator διατηρούνται.
Rail section, bearings, pin diameters, spring force, hole patterns και material
thicknesses παραμένουν provisional μέχρι να ζυγιστεί το γεμάτο basket και να
μετρηθεί η πραγματική δύναμη άμεσης ανύψωσης από τη λαβή.

Η τελική packaging διόρθωση της λαβής βασίζεται στην εσωτερική οροφή και όχι
στο LiDAR: η αρχική raised λαβή έφτανε `Z=457 mm`, μόλις `3 mm` κάτω από την
εσωτερική επιφάνεια `Z=460 mm`. Η τραβέρσα χαμηλώνει κατά `20 mm`, οπότε οι δύο
θέσεις της κορυφής γίνονται `Z=337/437 mm` και το raised clearance γίνεται
`23 mm`. Η λαβή μετακινείται στο `X=220 mm`, πάνω από το κεντροειδές του basket
και κοντά στη carriage beam `X=225 mm`, μειώνοντας το ασύμμετρο racking.

Τα σκέλη δεν παραμένουν μέσα στο ball volume στο `Y=+/-55 mm`: μεταφέρονται
στις longitudinal carriage beams στο `Y=+/-165 mm`. Το clearance είναι `10 mm`
και μετριέται στο **εξωτερικό** basket envelope `Y=146 mm` (mesh wall frame,
flange drop struts, χυτές χειρολαβές) και όχι στο εσωτερικό ημι-πλάτος
`Y=140 mm`. Είναι στατικό κενό συναρμολόγησης: σκέλη και basket ανεβαίνουν μαζί
στο carriage, ενώ η σαρωμένη διαδρομή των σκελών στο `X=211...229 mm` δεν
συναντά τα σταθερά rails και το cross-brace στο `X=56...84 mm`. Η προϋπάρχουσα
longitudinal carriage beam είναι ακόμα πιο στενή, `7 mm` στο ίδιο datum.

Η τραβέρσα πιάνεται στο κέντρο αλλά εδράζεται στα σκέλη, οπότε δουλεύει σε
κάμψη σε ανυποστήρικτο άνοιγμα `330 mm`. Είναι **αγοραστή μεταλλική διατομή**,
όχι τυπωμένο μέρος: τετράγωνο `18 mm` σε PLA φτάνει περίπου `26 MPa` και
βυθίζεται περίπου `9 mm` σε τράβηγμα `300 N`, έναντι περίπου `0.4 mm` σε
αλουμίνιο στην ίδια τάση. Η κεντρική περιοχή της τραβέρσας παραμένει
προσβάσιμη από επίπεδο gasketed hatch `110 x 170 mm`, με τη διάσταση `170 mm`
κατά Y. Δεν χρησιμοποιείται recessed well ή collar. Το reach στην κάτω θέση
είναι `126 mm` από το outer roof datum, ενώ το hatch παραμένει one-piece print
σε bed `220 x 220 mm`.

Το hatch διπλώνει **επίπεδα** πάνω στο basket hatch, όχι όρθιο. Η ελεύθερη ακμή
βρίσκεται στο `hatch_z + 110 * sin(theta)`, οπότε το όριο `Z<=478 mm` απέναντι
στο scan plane `Z=498 mm` επιτρέπει μόνο `theta <= 6.8` ή `theta >= 173.2`
μοίρες. Κάθε ενδιάμεση γωνία σηκώνει το πάνελ μέσα από το επίπεδο σάρωσης, άρα
δεν υπάρχει χρήσιμη μισάνοιχτη θέση — το προηγούμενο `-100` περνούσε την ακμή
στο `Z=573 mm`, δηλαδή `75 mm` πάνω από το scan plane. Το
`handle_access_hinge_open_deg = 175` δίνει ελεύθερη ακμή `Z=474.6 mm`, δηλαδή
`23.4 mm` κάτω από το scan plane, με το πάνελ να ακουμπά στο `X=55...165 mm`.
Το στοπ πρέπει να κρατά αυτή τη διπλωμένη θέση και όχι ενδιάμεση γωνία, και
επειδή η τελειωμένη όψη ακουμπά προς τα κάτω χρειάζονται bumpers στην οροφή.

## External panels / shell study

Απόφαση διερεύνησης (2026-08-21): μετά το κλείδωμα της χαμηλής launcher
baseline μπορεί να ξεκινήσει η εξωτερική εμφάνιση, αλλά ως ανεξάρτητο
panel-envelope και όχι ως manufacturing shell. Το
`cad/flywheel-launcher-v0/external-panel-study.scad` χωρίζει το bodywork σε:

- σταθερό χαμηλό perimeter subframe πάνω στο chassis,
- αφαιρούμενα lower side panels με wheel arches,
- αφαιρούμενο πίσω hatch για μπαταρία/ηλεκτρονικά,
- ενιαίο fixed upper edge και μεγάλο αφαιρούμενο top hatch,
- προαιρετικό moving cowl μόνο ως stepped comparison,
- χαμηλά εμπρός side fairings με ανοικτό κέντρο για intake, OAK-D και launcher.

Το shell δεν επιτρέπεται να μεταφέρει το βάρος του basket. Νεότερη απόφαση
χρήστη: η κύρια εξωτερική baseline έχει ενιαίο ύψος `z=463 mm`, δηλαδή `35 mm`
κάτω από το LiDAR scan datum. Έτσι κρύβονται rails/launcher και η σιλουέτα
μένει ομαλή σε collect και launch. Το μεγάλο επάνω hatch αφαιρείται για basket
service. Στο CAD υπάρχουν προαιρετικά keep-out
volumes για κατακόρυφη αφαίρεση basket και μπαταρίας. Πριν από DXF/STL ή κοπή
πρέπει να μετρηθούν πραγματικά motor mounts, panel gaps, hinges/quarter-turn
fasteners, αερισμός και το τελικό optical window της OAK-D.

Η πρώτη faceted εξωτερική baseline έχει fixed body περίπου `900 × 564 mm`
πάνω στην πλάκα `920 × 580 mm`, χωρίς να αυξάνει το συνολικό πλάτος που ήδη
ορίζουν οι τροχοί (~`780 mm`). Τα wheel arches κρατούν ονομαστικό radial
clearance `17.5 mm`, τα nose side panels `27 mm` από το flywheel guard envelope
και το uniform shell top `35 mm` κάτω από το LiDAR scan datum. Με panel `3 mm`
μένουν μόνο περίπου `16 mm` ονομαστικού εσωτερικού κενού πάνω από το raised
basket rim· αυτό απαιτεί full-load/flex validation και το hatch μπορεί να
παραμένει ανοικτό στη ρίψη αν οι μπάλες προεξέχουν. Το nose φτάνει περίπου
στο `x=790 mm`, αλλά παραμένει ανοικτό στον κεντρικό άξονα και επομένως δεν
λειτουργεί ως κλειστό front wall γύρω από intake/camera/exit guide.
Δύο top shoulders συνεχίζουν τη γραμμή `z=463 mm` δεξιά και αριστερά από αυτό
το κεντρικό κανάλι, ενώ το rear hatch έχει πραγματικό cutout `70 mm` για τον
ιστό LiDAR.

Νεότερη αισθητική απόφαση χρήστη: η κύρια μορφή γίνεται `rounded`. Το rounded skin
χρησιμοποιεί plan corner radius `55 mm`, ομαλή στένωση προς το nose,
στρογγυλεμένα battery/service doors και rounded top hatches. Η προηγούμενη
`faceted` μορφή παραμένει διαθέσιμη ως CAD comparison και όχι ως baseline.

Νεότερη mounting απόφαση χρήστη: το LiDAR δεν στηρίζεται πλέον με μακρύ ιστό
από την πλάκα του chassis. Η θέση/scan datum παραμένει `(-420, 0, 498 mm)`, αλλά
ένα κοντό bracket δένει στην επάνω πίσω structural crossmember περίπου στο
`z=445 mm`. Το cosmetic hatch έχει μόνο cutout για bracket/cables και δεν
παραλαμβάνει το φορτίο του αισθητήρα. Η μικρότερη προεξοχή αναμένεται να
μειώσει κάμψη και κραδασμούς, αλλά το πραγματικό bracket χρειάζεται modal/
vibration check μετά την επιλογή υλικού.

Το rounded rear δεν είναι πλέον επίπεδο: η κάτοψη γίνεται ήπια κυρτή και
εκτείνεται κατά `45 mm` πίσω από το παλιό rear datum. Το αντίστοιχο rear hatch
μεγαλώνει και στρογγυλεύει ώστε να ακολουθεί τη νέα καμπύλη χωρίς απότομη
οπτική διακοπή.

Οι πίσω βάσεις των panels υποχωρούν κατά `30 mm` προς το κέντρο και τα επάνω/
κάτω διαμήκη rails συγκλίνουν σταδιακά από `x=-260 mm` μέχρι την πίσω τραβέρσα.
Η υποχώρηση αφαιρεί τον τεχνητό «ώμο» του κελύφους, χωρίς να μετακινεί τη
μπαταρία, το service hatch ή τον κεντρικό άξονα στήριξης του LiDAR.

Η αισθητική κατεύθυνση δεν μετατρέπει όλο το robot σε οργανικό concept. Η
ορθογώνια αρχιτεκτονική, ο διαθέσιμος εσωτερικός όγκος και τα αφαιρούμενα panels
παραμένουν. Το `appearance_mode` εφαρμόζει μόνο τρεις παρεμβάσεις υψηλής
απόδοσης: ανοιχτό ψυχρό γκρι rounded upper shell, κρυφές εσωτερικές βάσεις και
συνεχή σκούρα ζώνη chassis μέχρι `z=190 mm`, ευθυγραμμισμένη με την κάτω ακμή
της OAK-D. Στόχος είναι η μετάβαση από
industrial prototype σε commercial sports robot χωρίς μηχανικό redesign.

Νεότερη front-fascia απόφαση χρήστη: το εξωτερικό nose δεν κατεβαίνει μέχρι τη
βάση. Τα Option A cheeks τελειώνουν στο `z=150 mm` και το bridge στο
`z=168 mm`, επομένως η κάτω ακμή του bodywork ορίζεται στο `z=168 mm`. Το
intake mouth και η πλήρης περιοχή κάτω από αυτή τη στάθμη παραμένουν ανοικτά.

Από `z=168…463 mm` το front fascia κλείνει πάνω και κάτω από το flywheel. Το
μοναδικό launcher opening είναι κεκλιμένη κυκλική οπή `116 mm` γύρω από τον
exit guide `90 mm`, με ονομαστικό radial clearance `13 mm`. Στο front plane το
κέντρο της βρίσκεται περίπου στο `z=299 mm` πάνω στον άξονα pitch `20°`.
Η OAK-D μεταφέρεται εξωτερικά κάτω από τον κύλινδρο, περίπου στο
`(x=800, y=0, z=205 mm)`, και δένει σε structural crossbar πίσω από το fascia·
το cosmetic panel δεν παραλαμβάνει το φορτίο της κάμερας.

### Performance shell v1

Η πρώτη ελεγχόμενη `performance_v1` παραλλαγή περιορίζεται σκόπιμα σε τέσσερις
αισθητικές αλλαγές: εμπρός ακμή `10°` στο smoked basket window, shaped/recessed
launcher face με tennis-yellow projected ring, κοντό tapered yellow accent που
δείχνει προς το launcher και graphite lower belt με δύο τοπικά steps μετά το
εμπρός wheel arch. Η λογική είναι `window = λειτουργία`, `accent = κατεύθυνση`,
`launcher face = τελικό σημείο της λειτουργίας`.

Το ring έχει αρχική ονομαστική ακτινική διάσταση `8 mm`, αλλά παραμένει visual
hypothesis και όχι hard dimension. Η εσωτερική έλλειψη ακολουθεί την προβολή
της υπάρχουσας οπής `116 mm` στον άξονα `20°`, ώστε να μη μειώνεται το launcher
clearance. Wheel-arch surrounds, LiDAR pod και roof rake μεταφέρονται ρητά σε
`v2`. Η οροφή δεν αλλάζει στο v1, επειδή το σημερινό nominal clearance πάνω από
το raised basket rim είναι μόνο περίπου `16 mm`.

## LiDAR pod v1 — envelope study

Το `cad/flywheel-launcher-v0/lidar-pod-study.scad` αντικαθιστά το placeholder
`Ø94 x 45 mm` που κουβαλούσαν `robot-integration.scad` και `lidar.urdf.xacro`.
Το πραγματικό εξάρτημα είναι **Slamtec RPLIDAR C1** (επιβεβαιωμένο με serial,
`real-lidar-bringup.md`): τετράγωνη βάση `55 x 55 mm`, βάση `23 mm`, συνολικό
ύψος `43 mm`, τέσσερα brass inserts κοντά στις γωνίες, οριζόντια έξοδος
καλωδίου. Μισή διαγώνιος `38.9 mm`. Οι μετρήσεις είναι χειρωνακτικές, περίπου
`+/- 1 mm`.

**Το κρίσιμο κατώφλι:** το `498 mm` παραμένει το datum, άρα η βάση κάθεται στο
`498 - sensor_scan_h` ενώ η οροφή είναι στο `466 mm`. Πάνω από
`sensor_scan_h = 32 mm` η βάση πέφτει **κάτω** από την οροφή και το pod παύει να
είναι ανυψωμένο fairing — γίνεται βυθισμένη φωλιά. Με βάση `23` και κεφαλή `20`,
η αναμενόμενη τιμή είναι `30...34`, δηλαδή ακριβώς πάνω στο κατώφλι.

Δύο παραλλαγές:

- `pod_variant="open"` — μόνο fairing. Κρύβει πλάκα, bracket, διέλευση οροφής
  και καλωδίωση, με **μηδέν τυφλό τομέα**.
- `pod_variant="caged"` — **απορρίφθηκε**, διατηρείται μόνο ως σύγκριση.
  fairing + καπάκι σε τρεις κολώνες στο `post_r=45 mm`.
  Το `45 < 50 = range_min` (μετρημένο στη συσκευή) σημαίνει ότι οι επιστροφές
  των κολωνών πέφτουν μέσα στο ελάχιστο εύρος και απορρίπτονται **χωρίς
  γωνιακό φίλτρο**. Αζιμούθια `90/210/330` μοίρες, σκόπιμα μακριά από τις
  διαγωνίους της βάσης όπου η μισή διαγώνιος `38.9` αφήνει το λιγότερο περιθώριο·
  χειρότερο κενό `10.7 mm` με κολώνα `5 mm`. Τυφλό σύνολο `19.1` μοίρες, δηλαδή
  `38/720` δείγματα (`5.3%`).

Παραμένουν `TBD` και είναι εκτεθειμένα ως παράμετροι το `sensor_scan_h` και το
`insert_pitch_x/y`. Χωρίς αυτά δεν σχεδιάζεται flange που να εδράζεται στα
σπειρώματα του αισθητήρα αντί να σφίγγει το κέλυφός του.

**Απόφαση χρήστη: επιλέγεται το `open`, χωρίς καπάκι.** Το καπάκι δεν προσφέρει
οπτικό όφελος: το επίπεδο σάρωσης είναι οριζόντιο, άρα ο δέκτης κοιτάζει
οριζόντια μέσα από στενό κατακόρυφο άνοιγμα, και ο ήλιος που φτάνει πραγματικά
σε αυτόν έρχεται σχεδόν οριζόντια — μέσα από το ίδιο κενό που το καπάκι είναι
υποχρεωμένο να αφήσει ανοιχτό. Ένα οριζόντιο καπάκι κόβει απότομες ακτίνες από
πάνω, δηλαδή ακριβώς αυτές που το στενό άνοιγμα ήδη απορρίπτει. Η μόνη
πραγματική δικαιολόγησή του ήταν προστασία από χτύπημα μπάλας, που δεν
επιλέχθηκε. Το `outdoor sunlight` άλλωστε είναι ρητά `NOT TESTED` στο
`docs/hardware/real-lidar-bringup.md`, οπότε το καπάκι θα ήταν λύση σε
αμέτρητο πρόβλημα· αν ποτέ φανεί θέμα σε γήπεδο, η απάντηση είναι κατακόρυφο
χείλος γύρω από το οπτικό παράθυρο ή φιλτράρισμα, όχι το cage.

Χωρίς καπάκι το fairing σταματά `8 mm` κάτω από το επίπεδο σάρωσης και καμία
επιφάνεια του pod δεν βρίσκεται μέσα στη δέσμη, οπότε η ανάκλαση κοντινού πεδίου
παύει να απασχολεί πέρα από το ίδιο το πάνω χείλος.

## Roof grid — ευθυγράμμιση με τον σκελετό

Οι ακμές των πάνελ οροφής προκύπτουν πλέον από τον `fixed_panel_subframe()` και
όχι από αισθητική επιλογή: διαμήκη rails στο `Y=+/-268 mm`, εγκάρσια μέλη στο
`X=-438` και `X=405`. Το `roof_inset = 14 mm` φέρνει κάθε ακμή πάνω στα rails
και τους δύο αρμούς πάνω από εγκάρσια μέλη.

Διορθώθηκαν τρία μετρημένα ελαττώματα:

- το παλιό rear hatch (`450 x 426` στο `X=-270`) **προεξείχε από το περίγραμμα
  του αμαξώματος** έως `15.5 mm` στις δύο πίσω γωνίες, `Y=+/-142...209`
  (επιβεβαιωμένο με 2D boolean `hatch - plan` που επέστρεφε μη κενή γεωμετρία)·
- basket hatch (`0...440`) και nose roof (`420...790`) **αλληλοδιείσδυαν κατά
  20 mm** σχεδόν συνεπίπεδα, δηλαδή εκείνος ο αρμός δεν υπήρχε καθόλου·
- η λωρίδα `45 mm` ανάμεσα στα δύο hatches **δεν είχε μέλος από κάτω**, άρα δύο
  ελεύθερες ακμές πάνω από κενό.

Επιπλέον δεν υπήρχε **καθόλου σταθερή οροφή**: τα δύο αφαιρούμενα πάνελ ήταν η
οροφή και άφηναν ανοιχτή λωρίδα `62...69 mm` σε κάθε πλευρά. Το
`rounded_top_system()` σχεδιάζει τώρα πρώτα σταθερό περίγραμμα, και περίγραμμα
και ακμές πάνελ μοιράζονται το ίδιο rail.

Τα πάνελ βγαίνουν από `roof_panel_2d(x_lo, x_hi)`: inset του body plan, κομμένο
σε ζώνη X. Έτσι το περιθώριο μένει σταθερό, το πίσω πάνελ **ακολουθεί** την
κυρτή ουρά αντί να προεξέχει, και κληρονομεί την ακτίνα του αμαξώματος αντί να
προσθέτει άλλη μία στην οικογένεια. Τα ανοίγματα είναι το ίδιο περίγραμμα
μεγαλωμένο κατά `roof_shut_gap = 3 mm` — η τιμή που ήδη χρησιμοποιούσε το
handle hatch, τώρα σε **όλους** τους αρμούς.

Προστέθηκε εγκάρσιο μέλος στο `roof_joint_mid_x = -22 mm`, `Z=445 mm`.

Η οπή του LiDAR παραμένει `Ø70` στο πίσω πάνελ. Με την ουρά του πάνελ στο
`X=-487` και την οπή να φτάνει `X=-455`, μένουν `32 mm` υλικού· το
`lidar-pod-study.scad` τη μειώνει σε μοτίβο `penetration_reach = 26.5 mm`.

**Απόφαση χρήστη:** η ιεραρχία χρήσης στην οροφή δηλώνεται από τον σκούρο
δακτύλιο φλάντζας του handle hatch και όχι από τόνο πάνελ. Το `upper_hatch_color`
δεν εφαρμόζεται σε κανένα πάνελ οροφής και κρατιέται για μελλοντικό τόνο
χειριστηρίου.

## Tennis-trainer identity pass

Αφετηρία μία μέτρηση: γεμάτο φορτίο 45 μπαλών φτάνει μόλις στο `Z~150 mm`
(πάτωμα `25`, δύο στρώσεις Ø66, ~24 ανά στρώση σε `400 x 280`), ενώ το smoked
basket window είναι στο `Z=221...419 mm`. **Το παράθυρο ξεκινά 71 mm πάνω από
γεμάτο φορτίο και δεν μπορεί ποτέ να δείξει μπάλα** — δείχνει πλέγμα, carriage
και gas spring. Αυτό είναι που κάνει το κέλυφος να διαβάζεται ως prototype αντί
για εξοπλισμό γηπέδου.

- **Εσοχή launcher.** Το `performance_launcher_details()` σχεδίαζε επίπεδους
  δακτυλίους στο fascia (`X=794`, `796.5`), μηδέν βάθος. Τώρα είναι πραγματικός
  κώνος στον άξονα 20°: λαιμός ίσος με το `front_exit_open_d`, διεύρυνση μόνο
  προς τα έξω, άρα το clearance του launcher δεν αγγίζεται. Ο κώνος
  υπερεκτείνεται και **κόβεται από το fascia**, και ο δακτύλιος accent είναι η
  *τομή* του κελύφους με λωρίδα `launcher_ring_w` στο fascia. Η κατασκευή έχει
  σημασία: το στόμιο του κώνου είναι κάθετο στον άξονα 20° ενώ το fascia είναι
  κατακόρυφο, οπότε σχεδιασμένη έλλειψη βγαίνει **μισοφέγγαρο** αντί για
  δακτύλιο — αυτό ακριβώς συνέβη στην πρώτη προσπάθεια.
- **Θυρίδες στάθμης μπαλών** (`show_ball_ports`), `180 x 62 mm` στο
  `X=115, Z=140`, μέσα στη ζώνη γραφίτη. Με asserts: `13 mm` ως την κορυφή της
  ζώνης, `45 mm` πάνω από την κάτω ακμή, `16.5 mm` ως το εμπρός wheel arch στο
  `X=227.5`. Η θυρίδα κόβει τη γραμμή πλήρωσης, άρα λειτουργεί και ως ένδειξη.
- **Συρρίκνωση του basket window** από `390 x 184` σε `290 x 100`. Στο παλιό
  μέγεθος ήταν το μεγαλύτερο μεμονωμένο στοιχείο του ρομπότ και έδειχνε ακριβώς
  ό,τι άξιζε να κρυφτεί. Τα δύο πλευρικά ανοίγματα είναι πλέον μία οικογένεια:
  ίδια αναλογία `~2.9:1` και κοινό πίσω datum, με **αμφότερα τα καθαρά ανοίγματα
  να ξεκινούν στο `X=19 mm`** (γι' αυτό το `ball_port_center` X πήγε στο `109`).
  Ακτίνα `18 mm`, στην οικογένεια ανοιγμάτων. Τα τζάμια είναι σκόπιμα
  αντεστραμμένα: η θυρίδα σχεδόν διαφανής ώστε να διαβάζεται κίτρινο φορτίο, το
  παράθυρο σκούρο (`basket_glazing_alpha = 0.66`) ώστε carriage και gas spring
  να **μη** διαβάζονται. Clearances: `96 mm` ως την οροφή, `63 mm` ως τη ζώνη.
- **Απόσυρση του side accent** (`show_side_accent = false`). Η λοξή λεπίδα
  διάβαζε ως ρίγα οχήματος — λάθος κατηγορία προϊόντος.

**Έλεγχος οπτικής γραμμής:** στο `-Y` μόνο το gas spring κόβει τα κάτω ~9 mm.
Στο `+Y` το swept keepout του V2 actuator (`Y=202...268`, `X=-18...258`,
`Z=45...253`) **περικλείει ολόκληρο το άνοιγμα** ανάμεσα στο skin `282` και το
τοίχωμα καλαθιού `146`. Αν τοποθετηθεί ο actuator, η θυρίδα `+Y` βλέπει υλικό,
όχι μπάλες. Είναι ο **τρίτος** ανεξάρτητος λόγος να μετακινηθεί ο actuator προς
τα μέσα, μετά το tumblehome και το όριο των `14 mm`.

## Αρχιτεκτονική κελύφους — διερεύνηση και απόφαση (2026-08-22)

**Απόφαση χρήστη: το ενιαίο design στα `463 mm` παραμένει.** Η εναλλακτική
profiled αρχιτεκτονική επανεξετάζεται **αφού δουλέψουν όλα τα μηχανικά μέρη**.
Τα ευρήματα της διερεύνησης καταγράφονται εδώ ώστε να μη χρειαστεί να ξαναγίνει.

### Η αλυσίδα ύψους

```
250  wall_top_z (χείλος καλαθιού)
+100  lift
+ 89  συνεισφορά tilt 12 μοιρών γύρω από το [470, 0, 40]
────
 439  πίσω χείλος ανυψωμένου καλαθιού  (x~73)
 356  εμπρός χείλος ανυψωμένου καλαθιού (x~465)
```

Η οροφή `463` υπάρχει **αποκλειστικά** για να περικλείει αυτό το `439`. Κανένα
άλλο στοιχείο δεν τη χρειάζεται.

### Διόρθωση: ο launcher είναι το ψηλό μέρος

```
κορυφή exit guide             ~332
κορυφή ανοίγματος Ø116         357   -> η μύτη χρειάζεται ~370
χείλος καλαθιού σε collect      250   -> η ουρά χρειάζεται ~270
```

Η σιλουέτα που προκύπτει από τη λειτουργία είναι **χαμηλό hopper με ψηλή
κεφαλή εκτόξευσης**, όχι χαμηλή μύτη. Μια προηγούμενη πρόταση για χαμηλή μύτη
ήταν λανθασμένη ως προς αυτό.

### Οι τέσσερις αρχιτεκτονικές

| | Οροφή | LiDAR | Ύψος | 360° σε launch |
|---|---|---|---|---|
| A ενιαίο (επιλεγμένο) | 463 | πίσω 498, κοντό bracket | ~511 | ναι |
| B προφίλ, καλάθι κλειστό | μύτη 370, μέση 463 | πίσω 498 | ~511 | ναι |
| C χαμηλό, καλάθι εξέχει | μύτη 370, ουρά 270 | ιστός ~190 mm | ~475 | μόνο με ιστό |
| D = C + κλίση 6 μοιρών | μύτη 370, ουρά 270 | στη μύτη, `z=420` | **~433** | ναι |

Το B δεν κερδίζει ύψος. Το C ξαναφέρνει τον ιστό που είχε απορριφθεί για
κάμψη/κραδασμούς. Το D είναι το μόνο που σπάει την αλυσίδα: με κλίση 6 μοιρών
το πίσω χείλος πέφτει σε `396`, επίπεδο σάρωσης `420` το καθαρίζει με `24 mm`,
και επειδή η μύτη είναι ήδη στο `370` το LiDAR κάθεται εκεί με bracket `50 mm`.

**Το D εξαρτάται από έναν ανεπιβεβαίωτο αριθμό.** Η κλίση είναι ρητά
καταγεγραμμένη ως εκκρεμής («αφού επιβεβαιωθούν η πλευρά του port, η απαιτούμενη
κλίση και η θέση του launcher») και τα ύψη είναι packaging datums. Επομένως:

> **Το gate `L4 Manual lift/tilt + singulator bench` δεν κρίνει πλέον μόνο τον
> μηχανισμό — κρίνει και την αρχιτεκτονική του κελύφους.** Αν οι 6 μοίρες
> ταΐζουν αξιόπιστα με γεμάτο καλάθι, το D γίνεται διαθέσιμο· αν όχι,
> καταρρέει στο C και το ενιαίο παραμένει η σωστή επιλογή.

### Ζωντανό εύρημα, ανεξάρτητο από την απόφαση

Οι **εκτοξευόμενες μπάλες διασχίζουν το επίπεδο σάρωσης του LiDAR** μπροστά από
το ρομπότ:

```
επίπεδο 498 (σημερινό) -> διασταύρωση ~547 mm μπροστά από το fascia
επίπεδο 420 (D)        -> διασταύρωση ~333 mm
```

Η καταγεγραμμένη στρατηγική αποφυγής εμποδίων είναι **reactive LiDAR e-stop**.
Άρα, με το **σημερινό** design, κάθε εκτόξευση περνά μέσα από το `/scan` περίπου
μισό μέτρο μπροστά. Χρειάζεται γωνιακή μάσκα ή αναστολή του e-stop σε launch
mode — αντίστοιχη με το υπάρχον interlock που αναστέλλει τον launcher σε
collect. **Δεν το εισάγει η αλλαγή αρχιτεκτονικής· υπάρχει ήδη.**

### Προσοχή στα datums της οροφής

Το `roof_inset = 14` και ο αρμός `x=405` προκύπτουν από τον
`fixed_panel_subframe()`, ο οποίος **δεν έχει οριστικοποιηθεί** πέρα από τη
γέφυρα του Option A. Ο κανόνας (ακμές πάνελ πάνω στον σκελετό, αρμοί πάνω από
εγκάρσια μέλη, μία τιμή shut-gap) επιβιώνει σε κάθε αναθεώρηση· οι **αριθμοί**
πρέπει να ξαναβγούν όταν κλειδώσει ο σκελετός. Στέρεα σήμερα είναι μόνο: πλάκα
chassis `920 x 580`, Option A (cheeks `150`, γέφυρα `168`), basket v2.1 και τα
rails ανύψωσης στο `y=180`.

### Σειρά δοκιμών (απόφαση χρήστη, 2026-08-22)

**Πρώτα δοκιμή χωρίς panels· τα panels δοκιμάζονται μετά, πάνω στο working
model.** Το `external-panel-study.scad` παγώνει ως έχει.

Αυτό σημαίνει ότι η γυμνή δοκιμή είναι η μόνη ευκαιρία να επιβεβαιωθούν τα
νούμερα πάνω στα οποία στέκεται το κέλυφος. Τι αξίζει να μετρηθεί όσο τρέχει
χωρίς panels:

| # | Μέτρηση | Τι κρίνει στο κέλυφος |
|---|---|---|
| 1 | Πραγματική κλίση/ανύψωση που ταΐζει με **γεμάτο** καλάθι (`L4`) | Ολόκληρη την αρχιτεκτονική: `439` -> οροφή `463` |
| 2 | Ύψος φορτίου 45 μπαλών στο πραγματικό καλάθι | Ύψος και θέση της θυρίδας στάθμης (`z 103...177`) |
| 3 | Πραγματικό ύψος ανυψωμένου χείλους **με φορτίο**, με flex | Το nominal `16 mm` clearance κάτω από την οροφή |
| 4 | Δύναμη χειρός στη λαβή με γεμάτο καλάθι | Διατομή τραβέρσας, gas spring, μελλοντικός actuator |
| 5 | Πού μπαίνει πραγματικά δομή γύρω από chassis/rails | `fixed_panel_subframe`, άρα `roof_inset` και οι αρμοί |
| 6 | Εξωτερική προεξοχή μοτέρ/άξονα/τροχού με τα πραγματικά εξαρτήματα | Wheel pods (`y 190...290`, `280...320`, `310...390`) |
| 7 | Αν το `/scan` βλέπει τις εκτοξευόμενες μπάλες | Γωνιακή μάσκα ή αναστολή e-stop σε launch mode |
| 8 | `sensor_scan_h` του RPLIDAR C1 (δεν χρειάζεται το ρομπότ) | Αν το pod είναι ανυψωμένο fairing ή βυθισμένη φωλιά |

Χωρίς τα 1-3 το κέλυφος παραμένει study. Με αυτά, τα περισσότερα provisional
datums του `external-panel-study.scad` γίνονται μετρημένα.

## Gazebo flywheel-only gate (2026-08-23)

Το intake και το flywheel αντιμετωπίζονται ως ανεξάρτητοι μηχανισμοί με
διαφορετικά joints, controllers και software. Το ενδιάμεσο feed είναι τρίτο
interface και δεν επιτρέπεται να αλλάξει τα frozen datums ή το tuning του
Option A intake.

Η ηλεκτρομηχανική αλυσίδα του launcher πέρασε το πρώτο μέρος του gate:

- ο `flywheel_velocity_controller` ενεργοποιήθηκε και το joint feedback
  ακολούθησε ακριβώς `+100/-100 rad/s`·
- τα intake joints παρέμειναν `0/0 rad/s` σε όλη τη flywheel-only δοκιμή·
- το `FLYWHEEL_NIP_GAP_M` εκτέθηκε ως ανεξάρτητη simulation/calibration
  παράμετρος, με αμετάβλητο default `58 mm`.

Η single-ball εκτόξευση **δεν πέρασε ακόμη**. Με rigid μπάλα Ø66 και nominal
gap `58 mm` η διπλή επαφή την εκτινάσσει πλευρικά. Στα `64 mm` μειώνεται η
αριθμητική διείσδυση, αλλά το συγχρονισμένο A/B με ίδιο one-step
spring-feeder-equivalent impulse έδωσε:

- flywheels off: `max_vx ~0.597 m/s`·
- flywheels `+100/-100 rad/s`: `max_vx ~0.605 m/s`, πλευρική απόκλιση
  `~277 mm`, χωρίς αύξηση του μέγιστου Z.

Άρα δεν ξεκινά ακόμη calibration εμβέλειας/RPM. Το επόμενο gate είναι
flywheel-only με κεντράρισμα της μπάλας (guide/feeder fixture) ή compliant
tennis-ball contact model. Το `64 mm` είναι simulation hypothesis, όχι
αποφασισμένη μηχανική διάσταση.

Ως regression guard εκτελέστηκε full `option-a-collect` sweep με
`ROBOT_ENABLE_FLYWHEEL=false`: bilateral contact `45/45`, wheel feedback
`25/25 rad/s` και `6` collected balls. Δεν υπάρχει diff κάτω από
`cad/collector-intake-v1/option-a/`.


## Μηχανικές configurations COLLECT / LAUNCH στο Gazebo (2026-08-23)

Το intake assembly και ο launcher **διεκδικούν τον ίδιο μπροστινό όγκο**:

```text
intake wheels + funnel + ramp + deflector   x 446...884 mm, z 0...281 mm
flywheel wheels                             x 459...661 mm, z 162...268 mm
flywheel frame plate (280x508x35, pitch 20) x 469...744 mm, z  24...152 mm
```

Μετρημένες διεισδύσεις OBB/SAT όταν συνυπάρχουν: frame vs intake wheels
`84 mm`, vs funnel cheeks `67 mm`, flywheel wheels vs intake deflector
`36.5 mm`. Δεν είναι σφάλμα τοποθέτησης — είναι δύο **εναλλακτικές μηχανικές
διατάξεις της ίδιας μηχανής**. Ο μηχανισμός μετάβασης (rail/slide) **δεν έχει
επιλεγεί**, άρα δεν μοντελοποιείται κανένας.

Οι variants του `scripts/generate_robot_urdf.py --packaging-variant`:

| variant | intake | launcher | basket entry hood |
|---|---|---|---|
| `baseline` | ναι | **όχι** | ναι |
| `option-a-collect` | ναι | **όχι** | ναι |
| `option-a-launch` | **όχι** | ναι | **όχι** (θέση LAUNCH: hood ανοικτό) |
| `compact` | ναι | ναι | ναι — provisional study, **διατηρεί τις συγκρούσεις** |

Το hood αφαιρείται μέσω του **υπάρχοντος** switch `hood_rear_overhang > 0`, όχι
νέας γεωμετρίας. Επαληθευμένο: `baseline`, `option-a-collect` και
`option-a-launch` δίνουν **0 διεισδύσεις** σε lift `0/25/50/75/100 mm`.

**Ο flywheel είναι πλέον `false` εξ ορισμού σε κάθε collection variant.** Σκέτο
`./run_native.sh` δεν σηκώνει launcher. Ζωντανή απόδειξη της ανάγκης: ίδιο
sweep case, `contact samples 0` με flywheel on έναντι `90` (bilateral `45/45`)
με flywheel off.

## Διόρθωση datum LiDAR (2026-08-23)

Η αλυσίδα έχει **τρία** σκέλη, όχι δύο:

```text
base_footprint (έδαφος)                    0.000
  + base_link_height                       +0.045 -> base_link  0.045
  + lidar_xyz.z                            +0.498 -> lidar_link 0.543
  + front_lidar <pose> (sim-only raise)    +0.035 -> ΕΠΙΠΕΔΟ ΣΑΡΩΣΗΣ 0.578
```

Το `0.498` ήταν γραμμένο ως local z, οπότε το πραγματικό optical centre ήταν
`578 mm` — απόκλιση `80 mm`, όχι 45. Το `+0.035` υπάρχει επειδή το gpu_lidar
κάνει raycast σε **visuals** και ο sensor μέσα στο κέλυφός του τυφλώνεται· είναι
μέρος του datum. Τώρα το CAD ύψος είναι η πηγή αλήθειας και το mount προκύπτει:
`lidar_mount_z = 0.498 - 0.045 - 0.035 = 0.418`. Μετρημένο ζωντανά:
TF `base_footprint->lidar_link = 0.463`, sensor pose στο spawned SDF `0.498`.
Ο ιστός παράγεται πλέον από το `mount_z`, ώστε να μη διαπερνά ποτέ το chassis.

**Ανοιχτό, ανεξάρτητο από αυτή την αλλαγή:** το `net_lidar_strand` στο
`gazebo/models/tennis_court/model.sdf` είναι στο `z=0.713` με σχόλιο «AT the
LiDAR scan height». Δεν ίσχυε ούτε στα `578` ούτε στα `498`. Στα `498` η σάρωση
πέφτει μέσα στο πλέγμα `net_h` του `0.50` (`0.497...0.503`), δηλαδή περιθώριο
`~2 mm`. Χρειάζεται ευθυγράμμιση του strand με το datum πριν εμπιστευτούμε
ξανά το net lock.

## Basket lift: 100 mm, χωρίς tilt

Το simulated travel ήταν `450 mm` — 4.5x το CAD baseline, και έστελνε το χείλος
στα `700 mm`, πάνω από το scan plane. Τώρα `BASKET_LIFT_TRAVEL_M=0.100`
παντού (xacro arg, macro default, ros2_control, generator, console supervisor).

**Το `RAISED` σημαίνει ΜΟΝΟ ότι ο άξονας ανύψωσης έφτασε στο άκρο του.** Η
μηχανική στάση εκτόξευσης είναι `100 mm lift + ~12 deg tilt`· ο μηχανισμός tilt
**δεν είναι επικυρωμένος**, άρα δεν μοντελοποιείται άξονας. Η ετοιμότητα
εκφράζεται ήδη ως `lift_confirmed AND tilt_confirmed` (`TiltState`,
`RobotReadiness.throwing_pose_confirmed`), με το tilt σε
`MECHANICAL_VALIDATION_PENDING`· όταν υπάρξει επικυρωμένος μηχανισμός γράφει
`CONFIRMED/FAULT` εκεί χωρίς αλλαγή στο state machine.


## Throwing Mode: πρώτη ζωντανή E2E orchestration (2026-08-24)

`ROBOT_PACKAGING_VARIANT=option-a-launch`, `ROS_DOMAIN_ID=123`, headless,
`SLAM_MODE=localization`, bench gate PASSED πριν από κάθε run.

Πλήρης διαδρομή, session `0f935a4a` (μετρημένα timestamps):

```text
t= 0.0   POSITIONING     pose (0.000, 0.000, 0.003), basket -3.17 mm
t=37.2   RAISING_BASKET  pose (-2.995, -0.074, 0.083)  <- XY err 0.09 m, yaw err 4.6 deg
t=43.3   ARMING          basket 97.41 mm, API RAISED
t=48.6   READY           flywheels μετρημένα +180 / -180 rad/s
t=50.6   THROWING        thrown=1
t=79.3   COMPLETED       thrown=3
t=91.3   settle          flywheels 0.0/0.0, basket 97.41 mm RAISED
```

Consumer (`gazebo_extras_node`): **3/3 feed events accepted, με σειρά, 0 duplicates**.
Test Throw: **ακριβώς 1**, κανένα δεύτερο μετά από 5x το interval.
Pause/Resume: **0 feeds σε 20 s pause**, καμία ριπή ανάκτησης, ίδιο session, 4/4.
Interlocks: `collect_route` **409** όσο το Throwing Mode κατέχει τον μηχανισμό,
collector start απορρίπτεται, collector stop και `idle` πάντα επιτρεπτά.

### Ελαττώματα που βρήκε το ζωντανό run

1. **Καμία τελική κατεύθυνση από το Nav2.** Ο κοινός `general_goal_checker`
   έχει `yaw_goal_tolerance: 3.14` σκόπιμα (οι lanes συλλογής δεν πρέπει να
   στριφογυρίζουν), και ο RPP δεν κάνει τελική περιστροφή. Το NavigateToPose
   επέστρεφε SUCCEEDED με το ρομπότ **159 μοίρες** λάθος. Το Nav2 `Spin`
   ABORT-άρει με `COLLISION_AHEAD (703)` ακόμη και με τοπικό costmap μετρημένο
   εντελώς καθαρό, και το Jazzy goal δεν έχει `disable_collision_checks`.
   → `tennis_robot.heading_aligner`: κλειστού βρόχου περιστροφή στο
   `/cmd_vel_teleop` (mux priority 100), φραγμένη σε γωνία και σε φρεσκάδα
   feedback. Το gate ετοιμότητας παραμένει ο κριτής.
2. **5x διπλά feed requests.** Το burst publish (σωστό για idempotent setpoints)
   είναι λάθος για διακριτά γεγονότα. → `reliable_event_publish` (ένα μήνυμα,
   αφού ταιριάξουν οι subscribers) + **de-duplication στον consumer με throw_id**.
3. **Basket οδηγήθηκε ΜΕΣΑ στο κάτω stop** (97.4 → −10.0 mm) και ξανακόλλησε.
   Διόρθωση του #61: δεν φταίει μόνο η *στάθμευση* πάνω στο όριο — **η άφιξη
   στο stop υπό εντολή** κλειδώνει το joint. → ο mover φράσσεται από την
   απόσταση και ακυρώνει σε stale feedback.
4. **Falsy-zero bug**: `self.position or start` μετέτρεπε τη θέση 0.0 (το κάτω
   άκρο!) σε start. Το έπιασε behavioural test.

### Ανοιχτά

- **Throw interval**: μετρημένο `7.9 / 8.6 s` για ρύθμιση `4.0 s`. Το
  `interval_s` είναι **κενό μεταξύ ρίψεων**, και το κόστος του feed publish
  (~4 s) προστίθεται. Χρειάζεται απόφαση: period ή gap.
- **Feed delivery με >1 subscriber**: ένας βραχύβιος publisher ανά γεγονός δεν
  εγγυάται παράδοση σε πολλούς subscribers. Στην παραγωγική τοπολογία (ένας
  consumer) μετρήθηκε 3/3· με δεύτερο παρατηρητή χάνονταν ρίψεις. Η οριστική
  λύση είναι persistent publisher, δηλαδή rclpy μέσα στην console.
- **Αστάθεια Nav2**: μετά από πολλά runs το Nav2 ABORT-άρει κάθε goal. Αιτία
  που εντοπίστηκε: **stale FastDDS shared memory** (`/dev/shm/fastrtps_*`,
  `open_and_lock_file failed`) από επανειλημμένα SIGKILL. Καθαρισμός του
  `/dev/shm` πριν από κάθε καθαρή εκτέλεση.
- **BALL_LAUNCH_PHYSICS_NOT_VALIDATED**: τα αποδεκτά feed events ΔΕΝ σημαίνουν
  ότι εκτοξεύτηκε μπάλα. Το `balls_thrown` μετρά **αποδεκτά feed requests**.


## Throwing Mode E2E: τα δύο τελευταία gates (2026-08-24)

### Interval — σημασιολογία περιόδου

Το `Interval Between Throws` είναι **περίοδος launch-to-launch**, όχι ανάπαυση
μετά το feed. Η υλοποίηση κοιμόταν ολόκληρο το interval **μετά** την εργασία του
feed, οπότε η μετρημένη καδέντζα ήταν `feed work + interval` = `7.92 / 8.56 s`
για ρύθμιση `4.0 s`.

Δύο διορθώσεις:

1. **Deadline-based scheduling** με `time.monotonic()`: το επόμενο throw
   προγραμματίζεται στο `previous_launch + interval`, και η αναμονή γίνεται σε
   φέτες `0.1 s` ώστε Pause/Stop να παραμένουν responsive. Το rebasing γίνεται
   από την **πραγματική** στιγμή εκτόξευσης, άρα χαμένο deadline μετατοπίζει την
   καδέντζα — ποτέ δεν συσσωρεύει χρέος που θα έβγαινε ως catch-up burst.
2. **Ντετερμινιστική στιγμή εκπομπής**: το κόστος discovery του βραχύβιου
   publisher μετρήθηκε `2.0–3.6 s` **και μεταβλητό**, οπότε έμπαινε στη στιγμή
   εκπομπής και εμφανιζόταν ως jitter στον consumer (`5.81 / 2.23 / 5.70 s`). Ο
   publisher δέχεται πλέον `--publish-at <unix_ts>`: κάνει το discovery νωρίς,
   κρατά τη matched σύνδεση και εκπέμπει **ακριβώς** τη στιγμή-στόχο. Ο service
   του παραδίδει το αίτημα `FEED_EMISSION_LEAD_S` νωρίτερα.

Μετρημένο στον ίδιο authoritative consumer (`gazebo_extras_node`), ανοχή
**±0.5 s ορισμένη ΠΡΙΝ τη μέτρηση**: gaps `4.093 s` και `4.095 s`
(σφάλμα `+0.093 / +0.095`). **PASS.**

### Pause/Resume — το ρολόι του interval αναστέλλεται

Ο χρόνος σε PAUSED δεν είναι χρέος: κρατείται το υπόλοιπο του interval τη στιγμή
του Pause και αποκαθίσταται στο Resume. Ούτε burst, ούτε αναμονή ολόκληρου νέου
interval.

**Race που βρέθηκε**: Pause που φτάνει **κατά τη διάρκεια** ενός feed. Το event
είχε ήδη εκπεμφθεί, αλλά το `confirm_successful_throw` απέρριπτε την κατάσταση
PAUSED → η ρίψη χανόταν και η session πήγαινε FAULT (προϋπάρχον, όχι νέο). Πλέον
το PAUSED γίνεται δεκτό **μόνο** στο confirm· το `prepare_throw` εξακολουθεί να
αρνείται να **ξεκινήσει** ρίψη σε pause.

### Stop σε ενεργή session

Stop εκδόθηκε σε κατάσταση `THROWING` μετά από επιβεβαιωμένα feed events.
Μετρημένα: αποδοχή HTTP 200· `THROWING → COMPLETED`· flywheels
`180/-180 → 0.0/0.0 rad/s`· basket `97.41 mm RAISED`, υπολειπόμενη ταχύτητα
`-0.0 mm/s`· pose αμετάβλητο (καμία επανεκκίνηση πλοήγησης)· collector
`running=False`· `/api/throwing` απάντησε σε `0.018 s` (κανένα deadlock)· και
**καμία επιπλέον ρίψη** μετά από αναμονή `8 s` (>1 interval). **PASS.**

### Σφάλμα εργαλείου που άξιζε guard

Ένα scripted slice-replace **διπλασίασε** μπλοκ μεθόδων στο `ros_service.py`
αντί να το αντικαταστήσει. Η Python κρατά την **τελευταία** definition, οπότε
μεταγενέστερη επεξεργασία της πρώτης δεν είχε καμία επίδραση στο runtime ενώ το
`grep` έβρισκε το νέο κείμενο και όλα τα tests περνούσαν (χρησιμοποιούν fake ros
port). Εμφανίστηκε ζωντανά ως
`unexpected keyword argument 'publish_at_unix'`. Προστέθηκε
`tests/test_no_shadowed_definitions.py` (AST) που απαγορεύει διπλούς ορισμούς σε
module/class scope. Δεν επηρεάζει προηγούμενα αποτελέσματα: οι διπλές αντιγραφές
ήταν ταυτόσημες μέχρι εκείνη την επεξεργασία.

## Distributed PC↔Pi: η εντολή actuator δεν έφτανε ποτέ (2026-08-24)

Πρώτο ζωντανό Throwing Mode run στην **κατανεμημένη** τοπολογία (Gazebo GUI στο
PC, brain στο Pi, `ROS_DOMAIN_ID=42`, `option-a-launch`). Η session πήγε
`FAULT` με `flywheel readiness was not confirmed`, ενώ οι flywheels **δεν
γύρισαν καθόλου** — μετρημένα `0.0 / 0.0 rad/s` σε όλο το ARMING.

Δεν ήταν timeout. Η εντολή **δεν παραδόθηκε ποτέ**.

Το `RosService._publish_command` στέλνει `ros2 topic pub --times 5 -r 10`. Το
`--times` κάνει ήδη wait για matching subscription, άρα η ριπή γράφεται — αλλά
η διεργασία τερματίζει αμέσως μετά, και πάνω από το LAN το reliable handshake
δεν έχει ολοκληρωθεί, οπότε **όλα τα samples χάνονται**. Είναι η κατανεμημένη
μορφή του ίδιου σφάλματος που το σχόλιο της μεθόδου περιγράφει για το `--once`:
η αρχική μέτρηση («πέντε αντίγραφα οδήγησαν το basket_joint σε πλήρη διαδρομή»)
έγινε **σε ένα μηχάνημα**, όπου το discovery είναι υποδευτερόλεπτο.

Μετρημένα Pi → PC, ζωντανός `flywheel_velocity_controller`:

```text
--times 5 -r 10                   flywheel 0.0 rad/s      (τίποτα δεν έφτασε)
--times 5 -r 10 --keep-alive 3    flywheel 55.0 rad/s
keep-alive sweep: 0.25 s έχανε εντολές· 0.5 s και 1.0 s παρέδωσαν 3/3
```

→ `COMMAND_PUBLISH_KEEP_ALIVE_S = 1.0`: ο writer μένει ζωντανός μετά το
τελευταίο μήνυμα. Είναι εγγύηση παράδοσης, όχι timeout — χωρίς αυτό η εντολή
δεν φτάνει καθόλου και καμία αναμονή στην πλευρά του feedback δεν τη σώζει.

Αφορά **κάθε** actuator εντολή της κονσόλας, όχι μόνο τον launcher: flywheel
speed, `stop_basket`, collector manual control. Το `stop_basket` είναι το
σοβαρό — ένα stop που σιωπηλά δεν φτάνει. Το basket lift δεν επηρεάστηκε γιατί
περνά από τον `basket_lift_mover`, μια μακρόβια rclpy διεργασία.

Regression: `tests/test_actuator_command_delivery.py` (καρφώνει το argv — το
`ros2 topic pub` επιστρέφει 0 είτε παραδόθηκε είτε όχι).

## Ο launcher του URDF ΔΕΝ είναι ο launcher του CAD (2026-08-24)

Η παραδοχή «intake και launcher δεν συνυπάρχουν» **δεν προκύπτει από το CAD**.

`cad/flywheel-launcher-v0/robot-integration.scad`, `full_robot_context()`
σχεδιάζει πάντα ολόκληρο το Option A intake (`option_a_read_only_context()`)
και αμέσως μετά τον launcher, με ρητό σχόλιο:

```text
// Launcher stays mounted in both modes; interlock state changes.
```

Το `mode="both"` δεν σημαίνει «οι δύο μηχανισμοί μαζί» — είναι το καλάθι στις
**δύο θέσεις** (κάτω ghost / πάνω). Ο intake σχεδιάζεται σε κάθε mode.

Η απόκλιση είναι στο **frame**, όχι στη θέση. Μετρημένα στο ίδιο πλαίσιο
(nip 215 mm, pitch 20°, x 560 mm):

| | y | χαμηλότερο z |
|---|---|---|
| CAD cradle (δύο `side_plate` 8 mm, `side_plate_y = ±43`) | −157 … +157 | **127 mm** |
| URDF `flywheel_launcher_frame_link` (280×508×35) | −254 … +254 | **24 mm** |

Το URDF αντικατέστησε τα δύο λεπτά ελάσματα με μία πλάκα **1.6× πιο φαρδιά**
που κρέμεται **103 mm χαμηλότερα**, μέσα στο στόμιο του intake στο ύψος του
δαπέδου — εκεί που η δομή του CAD δεν φτάνει ποτέ. Ο nip στα 215 mm
(`0.17 + 0.045`) ταιριάζει με το CAD· μόνο το frame όχι.

Άρα οι μετρήσεις «frame vs intake wheels 84 mm, vs funnel cheeks 67 mm» που
παρήγαγαν το συμπέρασμα του αποκλεισμού **δεν μέτρησαν τον launcher του CAD**.
Το `option-a-launch` αφαιρεί το intake για λόγο που δεν προκύπτει από το
σχέδιο. **ΑΝΟΙΧΤΟ — απόφαση χρήστη:** είτε το URDF frame ευθυγραμμίζεται με το
CAD cradle (και τότε επανεξετάζεται αν χρειάζεται ξεχωριστό variant), είτε
τεκμηριώνεται γιατί η πλάκα είναι η σωστή αναπαράσταση.
