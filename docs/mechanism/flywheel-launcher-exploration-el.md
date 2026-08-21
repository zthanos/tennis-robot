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

Το model περιλαμβάνει πλέον manual lever, adjustable link, gas-spring envelope,
positive locks, καθώς και διατηρημένα clevis hardpoints, swept keep-out, limit
switches, driver keep-out και cable route για μελλοντικό electric linear
actuator. Rail section, bearings, pin diameters, spring force, hole patterns και
material thicknesses παραμένουν provisional μέχρι να ζυγιστεί το γεμάτο
basket και να μετρηθεί η πραγματική δύναμη χειρισμού.
