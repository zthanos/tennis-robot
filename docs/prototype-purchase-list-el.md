# Λίστα Παραγγελίας Πρώτου Prototype

Σκοπός: αγορά υλικών για το πρώτο φυσικό prototype του tennis robot:

```text
ξύλινη βάση + κινητήρια βάση + front collector + βασική ασφάλεια
```

Πάρε αυτή τη λίστα σε φυσικό κατάστημα ξυλείας/σιδηρικών/ηλεκτρονικών. Οι
διαστάσεις είναι σε mm όπου δεν αναφέρεται κάτι άλλο.

## 1. Ξύλινη Βάση

Υλικό επιλογής:

```text
Κόντρα πλακέ θαλάσσης σημύδα 12-15 mm, φύλλο περίπου 70 x 100 cm
```

Προτίμηση:

```text
15 mm αν υπάρχει.
12 mm αν θέλουμε πιο ελαφριά βάση.
21 mm μόνο αν θέλουμε πολύ στιβαρό αλλά βαρύ prototype.
```

Κοπή:

| Qty | Διαστάσεις | Περιγραφή |
|---:|---|---|
| 1 | 760 x 430 | Κύρια κάτω βάση |
| 2 | 760 x 45 | Μακριά πλαϊνά rails/νευρώσεις |
| 3 | 390 x 45 | Εγκάρσιες νευρώσεις |
| 2 | 180 x 90 | Collector upright plates |
| 2 | 220 x 80 | Hopper/back-plate supports |
| 2 | 120 x 80 | Motor pod reinforcement plates |
| 2 | 100 x 80 | Front caster reinforcement plates |
| 2 | 130 x 60 | Stabilizer bracket blocks |
| 2 | 100 x 60 | Handle socket backing plates |

Να υπολογιστεί απώλεια κοπής 3-5 mm ανά πέρασμα δίσκου.

## 2. Κίνηση Βάσης

### Drive Motors

Ζητάμε:

```text
2 τεμάχια DC gear motor 12V με encoder
```

Ιδανικά χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Τάση | 12 V DC |
| Ταχύτητα | 80-120 RPM |
| Encoder | Ναι, quadrature αν γίνεται |
| Άξονας | 6 mm ή 8 mm D-shaft |
| Rated torque | τουλάχιστον 7-10 kg.cm |
| Stall current | να αναγράφεται στο datasheet |
| Gearbox | μεταλλικό |

Αν δεν υπάρχει με encoder:

```text
Μπορούμε να πάρουμε 2x 12V 100RPM gear motors χωρίς encoder μόνο για rolling
test, αλλά για αυτόνομη κίνηση θα χρειαστούμε encoder αργότερα.
```

Να ρωτήσουμε στο μαγαζί:

- Έχει encoder; πόσα pulses per revolution;
- Ποια είναι η διάμετρος και το σχήμα του shaft;
- Ποιο είναι το rated current και stall current;
- Υπάρχει matching wheel hub/coupler για τον άξονα;

### Driven Wheels

Ζητάμε:

```text
2 τεμάχια driven wheels 150-180 mm διάμετρο
```

Προτίμηση:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Διάμετρος | 150-180 mm |
| Πλάτος | 35-55 mm |
| Πάτημα | rubber / PU / TPU, όχι σκληρό πλαστικό |
| Hub | να ταιριάζει με 6 mm ή 8 mm D-shaft |
| Στήριξη | set screw, clamp hub, ή adapter hub |

Αν δεν βρεθεί έτοιμη ρόδα:

```text
Παίρνουμε hubs/couplers για τον άξονα και κρατάμε το CAD για printed wheel core
με rubber/TPU sleeve.
```

### Front Casters

Ζητάμε:

```text
2 τεμάχια περιστρεφόμενες ρόδες / swivel casters με πλάκα
```

Χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Διάμετρος | 75-85 mm |
| Αντοχή | τουλάχιστον 40-60 kg ανά caster |
| Τροχός | TPE / PU / rubber |
| Βάση | μεταλλική πλάκα με 4 τρύπες |
| Φρένο | όχι απαραίτητο |

## 3. Motor Drivers Και Τροφοδοσία Κίνησης

Ζητάμε:

```text
2 τεμάχια motor driver για brushed DC motor
```

Budget επιλογή:

```text
BTS7960 / IBT-2 driver, ένας ανά μοτέρ
```

Χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Motor voltage | 12 V ή 6-27 V range |
| Control | PWM + direction |
| Logic | 3.3 V / 5 V compatible |
| Current | αρκετό για stall current του μοτέρ |
| Cooling | heatsink, να αερίζεται |

Προσοχή: τα “43A” στα BTS7960 είναι συνήθως peak/marketing. Θέλουμε να ξέρουμε
το πραγματικό stall current των μοτέρ και να βάλουμε ασφάλεια.

## 4. Collector Module

### Funnel / Collector Body

Υλικά:

| Qty | Υλικό | Περιγραφή |
|---:|---|---|
| 1 set | PETG/ASA print ή πλαστικό φύλλο 2-3 mm | Funnel side plates και intake guides |
| 1 | πλαστικό/plywood plate | Adjustable back plate |
| 1 | διάφανο πλαστικό φύλλο 2-3 mm | Hopper/bin, για να βλέπουμε τις μπάλες |
| 1 set | μικρές γωνίες ή brackets | Για ρυθμιζόμενη σύνδεση funnel/back plate |

Διαστάσεις στόχοι:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Funnel mouth width | 220-300 mm |
| Throat width | 75-85 mm |
| Bottom lip height | 5-12 mm από το έδαφος |
| Hopper capacity | 3-6 μπάλες |

### Lift Wheel / Roller

Ζητάμε:

```text
1 τεμάχιο rubber/PU/TPU roller ή wheel, διάμετρος 60-100 mm
```

Χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Διάμετρος | 60-100 mm |
| Πλάτος | 120-180 mm αν είναι roller, ή κεντραρισμένη wheel διάταξη |
| Υλικό | μαλακό rubber/PU/TPU, όχι σκληρό πλαστικό |
| Άξονας | να ταιριάζει με motor shaft ή coupler |

### Collector Motor

Ζητάμε:

```text
1 τεμάχιο DC gear motor 12V για lift wheel
```

Χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Τάση | 12 V DC |
| Ταχύτητα | περίπου 100-300 RPM |
| Torque | αρκετό για να πιέζει/σηκώνει μπάλα tennis |
| Gearbox | μεταλλικό |
| Encoder | προαιρετικό για collector |

Για collector motor driver:

```text
1 μικρός H-bridge driver ή 1 BTS7960 αν πάρουμε ίδιο driver παντού.
```

## 5. Ασφάλεια Και Ηλεκτρικά

Απαραίτητα:

| Qty | Είδος | Σημειώσεις |
|---:|---|---|
| 1 | Emergency stop switch | Να κόβει τροφοδοσία στα μοτέρ |
| 1 | Fuse holder | Για την κύρια γραμμή μπαταρίας |
| 2-4 | Ασφάλειες αυτοκινήτου | Τιμή ανάλογα με τα μοτέρ |
| 1 | Κεντρικός διακόπτης | Battery on/off |
| 1 set | Καλώδια σιλικόνης ή automotive | Για ρεύμα μοτέρ/μπαταρίας |
| 1 set | Dupont/JST/terminal connectors | Για logic και sensors |
| 1 set | Heat shrink tubing | Μόνωση συνδέσεων |
| 1 set | Cable ties + αυτοκόλλητες βάσεις | Cable management |

Για μπαταρία, αν αγοράσουμε τώρα:

```text
12V battery pack ή 3S Li-ion/LiPo με BMS/charger, 5-10Ah για πρώτες δοκιμές.
```

Αν δεν είμαστε έτοιμοι για μπαταρία:

```text
Μπορούμε να κάνουμε πρώτα bench tests με 12V τροφοδοτικό επαρκούς ρεύματος.
```

## 6. Βίδες, Ροδέλες, Inserts

Πάρε αρκετά, γιατί στο prototype θα λυθούν/δεθούν πολλές φορές.

| Qty | Είδος | Χρήση |
|---:|---|---|
| 30-50 | M5 bolts, διάφορα μήκη | motor pods, caster, collector, handle |
| 30-50 | M5 washers | κάτω από όλες τις κεφαλές |
| 20-30 | M5 nyloc nuts ή T-nuts | αφαιρούμενες συνδέσεις |
| 20-30 | M4 bolts + washers | electronics, μικρά brackets |
| 20 | threaded inserts ή T-nuts | σημεία που θα βγαίνουν συχνά |
| 1 set | ξυλόβιδες 3.5x30 ή 4x30 | ξύλινα rails/νευρώσεις |
| 1 | D4 wood glue | μόνιμες ξύλινες ενισχύσεις |

## 7. Να Μην Αγοραστούν Ακόμα

Μην κλειδώσουμε ακόμα:

- launch flywheel motors
- expensive launcher wheels
- large battery pack για launcher
- pan/tilt μηχανισμούς
- LiDAR

Πρώτα θέλουμε:

```text
να κυλάει η βάση,
να πλησιάζει αργά την μπάλα,
να τη βάζει στο funnel,
και ο collector να την ανεβάζει στο hopper.
```

## 8. Σύντομη Λίστα Για Το Ταμείο

Αν ο πωλητής θέλει μόνο τη σύντομη εκδοχή:

```text
1 φύλλο κόντρα πλακέ θαλάσσης σημύδα 12-15 mm, 70x100 cm, κομμένο σύμφωνα με λίστα
2 DC gear motors 12V, 80-120RPM, με encoder, 6/8mm D-shaft
2 motor drivers για brushed DC motors, π.χ. BTS7960/IBT-2
2 driven wheels 150-180mm rubber/PU ή hubs για τους άξονες
2 swivel casters 75-85mm με πλάκα
1 DC gear motor 12V 100-300RPM για collector lift wheel
1 rubber/PU/TPU roller ή wheel 60-100mm για collector
1 μικρός H-bridge driver για collector ή τρίτο BTS7960
1 emergency stop switch
1 fuse holder + ασφάλειες
M5/M4 βίδες, ροδέλες, T-nuts/threaded inserts
D4 ξυλόκολλα, ξυλόβιδες, heat shrink, καλώδια, connectors
```
