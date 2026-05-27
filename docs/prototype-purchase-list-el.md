# Λίστα Παραγγελίας Πρώτου Prototype

Σκοπός: αγορά υλικών για το πρώτο φυσικό prototype του tennis robot:

```text
ξύλινη βάση + κινητήρια βάση + front collector + βασική ασφάλεια
```

## Sensor Baseline Update

Η νέα baseline επιλογή αισθητήρων είναι:

```text
OAK-D S2 camera -> ball detection / ball distance / ball map
Slamtec RPLIDAR C1 -> obstacle scan / wall-net-fence clearance / route costmap
Encoders + IMU -> odometry / robot pose
```

Το RPLIDAR C1 μπαίνει συμπληρωματικά στην OAK-D S2. Δεν αντικαθιστά την κάμερα για την ανίχνευση μπάλας, αλλά κάνει την πλοήγηση και το edge/defer policy πολύ πιο αξιόπιστα.

Προσθήκη στη λίστα αγοράς electronics/sensors:

| Qty | Είδος | Χρήση |
|---:|---|---|
| 1 | Slamtec RPLIDAR C1 | 2D LiDAR για costmap, εμπόδια, φιλέ/τοίχο/φράχτη και ROS 2 navigation |
| 1 | USB/serial cable ή adapter που απαιτεί το C1 kit | Σύνδεση στο SBC / mini PC |
| 1 set | M3/M4 vibration-isolated mounting bracket | Σταθερή οριζόντια τοποθέτηση LiDAR στο robot |

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

Προτιμώμενη αγορά μετά τη συναρμολόγηση της βάσης:

```text
Cytron motor driver candidate:
https://www.amazon.de/-/en/gp/product/B0GV4LQ1VP/ref=ox_sc_act_title_1?smid=A2PMBAYX6G8IJ&th=1
```

Να αγοραστεί αφού:

1. κουμπώσει η ξύλινη βάση,
2. τοποθετηθούν motor pods / τροχοί / casters,
3. ξέρουμε το πραγματικό ρεύμα των drive motors,
4. επιβεβαιώσουμε ότι ο driver καλύπτει το stall/peak current των μοτέρ.

Αν τελικά πάρουμε Cytron drivers, κρατάμε πάλι έναν driver ανά drive motor.
Ο collector μπορεί να χρησιμοποιήσει μικρότερο H-bridge ή τρίτο αντίστοιχο
driver αν θέλουμε κοινό hardware.

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

### Wide Intake Roller / Cylinder

Ζητάμε:

```text
1 τεμάχιο wide compliant rubber/PU/TPU roller/cylinder, μήκος 240-300 mm, διάμετρος 60-90 mm
```

Χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Διάμετρος | 60-90 mm |
| Πλάτος / μήκος κυλίνδρου | 240-300 mm ενεργό πλάτος συλλογής |
| Υλικό | μαλακό rubber/PU/TPU, όχι σκληρό πλαστικό |
| Άξονας | μακρύς άξονας με στήριξη/ρουλεμάν και στις δύο πλευρές |

### Collector Motor

Ζητάμε:

```text
1 τεμάχιο DC gear motor 12V για intake roller
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

Προτεινόμενη on-board μπαταρία για τη βάση:

```text
ECO-WORTHY Lithium Battery 12 V 20 Ah with BMS Protection
LiFePO4 12 V 20 Ah for boats/caravans
```

Χαρακτηριστικά που κρατάμε ως baseline:

| Χαρακτηριστικό | Τιμή / στόχος |
|---|---|
| Χημεία | LiFePO4 |
| Ονομαστική τάση | 12.8 V |
| Χωρητικότητα | 20 Ah |
| BMS | built-in protection |
| Βάρος | περίπου 2.6 kg |
| Διαστάσεις | περίπου 182 x 77 x 170 mm |
| Κύκλοι | 3000+ cycles, έως 15000 ανάλογα χρήση |
| Χρήση | boat, caravan, motorhome, off-grid, όχι starter battery |

Πριν την τελική αγορά επιβεβαιώνουμε:

```text
Maximum continuous discharge current >= 20A
LiFePO4 charger voltage: 14.6V
Terminal type και κατάλληλους ακροδέκτες καλωδίων
```

Προτεινόμενος φορτιστής:

```text
14.6V LiFePO4 charger, 5A προτιμώμενο για ήπια φόρτιση
10A μόνο αν το επιτρέπει ρητά ο κατασκευαστής της μπαταρίας
```

Αν δεν είμαστε έτοιμοι για μπαταρία:

```text
Μπορούμε να κάνουμε πρώτα bench tests με 12V τροφοδοτικό επαρκούς ρεύματος.
```

### Σταθεροποιητές Τάσης / Power Rails

Για τα electronics θέλουμε ξεχωριστό σταθεροποιημένο κλάδο από την ίδια
μπαταρία:

```text
1x 12V DC-DC buck-boost regulator για σταθερό 12V rail, αν χρειαστεί
1x 12V -> 5V buck converter, 5A minimum, για SBC/camera/sensors
```

Ο buck-boost regulator 12V είναι χρήσιμος όταν κάποιο υποσύστημα χρειάζεται
σταθερά 12V, γιατί η LiFePO4 δεν μένει πάντα ακριβώς στα 12V. Το drive motor
power δεν πρέπει να περνάει από μικρό regulator.

### Βασική Καλωδίωση Ισχύος

Η κύρια σύνδεση για τα μοτέρ:

```text
Μπαταρία +
  -> κύρια ασφάλεια Fuse
  -> E-Stop / μανιτάρι ασφαλείας
  -> κεντρικός διακόπτης
  -> Motor Drivers, π.χ. Cytron
  -> Drive motors / collector motor

Μπαταρία -
  -> κοινό ground προς motor drivers, buck converters, controller
```

Για τον ξεχωριστό κλάδο electronics:

```text
Μπαταρία +
  -> μικρότερη ασφάλεια Fuse
  -> buck converter 12V -> 5V
  -> SBC / camera / sensors

Μπαταρία -
  -> κοινό ground
```

Αν χρησιμοποιηθεί 12V buck-boost regulator:

```text
Μπαταρία +
  -> ξεχωριστή ασφάλεια Fuse
  -> 12V buck-boost regulator
  -> σταθερό 12V accessory rail
```

Σημείωση: ο κλάδος που τροφοδοτεί τους motor drivers πρέπει να περνάει από Fuse
και E-Stop. Ο κλάδος electronics μπορεί να έχει δική του μικρότερη ασφάλεια,
ώστε να μην πέφτει όλο το σύστημα από θόρυβο/αιχμές των μοτέρ.

### Καλώδια Και Ακροδέκτες

Για την πρώτη κατασκευή πάρε:

| Qty | Είδος | Χρήση |
|---:|---|---|
| 2-3 m | κόκκινο/μαύρο καλώδιο 2.5 mm² ή 12-14 AWG | μπαταρία, fuse, E-Stop, motor drivers |
| 2-3 m | κόκκινο/μαύρο καλώδιο 1.0-1.5 mm² ή 16-18 AWG | collector motor, accessory 12V, μικρότερα φορτία |
| 2-3 m | καλώδιο 0.25-0.5 mm² ή 22-24 AWG | encoder, PWM, direction, sensors |
| 1 set | ring/spade terminals για τους πόλους της μπαταρίας | σύνδεση στη LiFePO4 |
| 1 set | fork/ring terminals M4/M5 | fuse holder, E-Stop, distribution points |
| 1 set | ferrules | καθαρές συνδέσεις σε screw terminals |
| 1 set | XT60 ή Anderson-style connector | γρήγορη αποσύνδεση μπαταρίας |
| 1 set | WAGO/terminal blocks ή power distribution block | διανομή 12V/GND |
| 1 set | heat shrink tubing | μόνωση |
| 1 set | spiral wrap ή cable sleeve | προστασία καλωδίων |
| 1 set | cable ties + adhesive mounts | cable management |

Προτεινόμενες ασφάλειες για αρχή:

```text
Main motor fuse: 20A ή 25A, ανάλογα με το BMS/motor current
Electronics fuse: 3A-5A
Accessory 12V fuse: 5A-10A, αν μπει buck-boost/accessory rail
```

## 6. Υπολογιστής / On-Board Compute

Candidate για development και πιθανό on-board computer:

```text
Lenovo ThinkCentre M710q Mini PC
Intel Core i5-7400T, 4 cores, έως 3.6 GHz
16 GB DDR4 RAM dual channel
256 GB NVMe SSD
Intel HD Graphics / DirectX 12
HDMI + DisplayPort
4x USB 3.0 + 2x USB 2.0
Gigabit Ethernet
Wi-Fi μέσω μικρού USB adapter
Windows 11 Pro 64-bit activated
Case: 18 x 18 x 3.5 cm
External PSU: περίπου 90 W
```

Χρήση στο project:

| Ρόλος | Απόφαση |
|---|---|
| Bench development | Πολύ καλό |
| Webots/OpenSCAD/general CAD support | Καλό για basic χρήση |
| Python/OpenCV/DepthAI host | Καλό candidate |
| On-board robot computer | Πιθανό, μετά από δοκιμή κατανάλωσης/στήριξης |
| Τελικό OS για robot | Προτίμηση Ubuntu Linux ή dual boot Windows + Ubuntu |

Σημαντική σημείωση τροφοδοσίας:

```text
Το M710q δεν τροφοδοτείται απευθείας από 12V μπαταρία χωρίς κατάλληλο DC-DC.
Πριν μπει πάνω στο robot, πρέπει να επιβεβαιωθεί η είσοδος του original PSU
και να διαλεχθεί ασφαλής τρόπος τροφοδοσίας.
```

Προτιμώμενες επιλογές τροφοδοσίας για on-board χρήση:

1. DC-DC boost/buck-boost από LiFePO4 battery προς την τάση που χρειάζεται το
   Lenovo, με αρκετό wattage και σωστό βύσμα.
2. Ξεχωριστή power bank/USB-C PD λύση μόνο αν υπάρχει αξιόπιστος adapter.
3. Inverter 230V μόνο για πάγκο/δοκιμές, όχι ως πρώτη on-board επιλογή λόγω
   απωλειών και περιττής πολυπλοκότητας.

Τι να επιβεβαιώσουμε πριν την αγορά/τοποθέτηση:

- πραγματική τάση/ρεύμα του Lenovo power input,
- μέγιστη κατανάλωση με camera + USB devices,
- αν χωράει στο electronics bay χωρίς να εμποδίζει την αφαίρεση μπαταρίας,
- αν αντέχει κραδασμούς ή χρειάζεται rubber isolation,
- αν το Wi-Fi USB adapter είναι αρκετό ή θέλουμε καλύτερο antenna/USB extension.

## 7. Βίδες, Ροδέλες, Inserts

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

## 8. Να Μην Αγοραστούν Ακόμα

Μην κλειδώσουμε ακόμα:

- launch flywheel motors
- expensive launcher wheels
- large battery pack για launcher
- pan/tilt μηχανισμούς

Πρώτα θέλουμε:

```text
να κυλάει η βάση,
να πλησιάζει αργά την μπάλα,
να τη βάζει στο funnel,
και ο collector να την ανεβάζει στο hopper.
```

## 9. Σύντομη Λίστα Για Το Ταμείο

Αν ο πωλητής θέλει μόνο τη σύντομη εκδοχή:

```text
1 φύλλο κόντρα πλακέ θαλάσσης σημύδα 12-15 mm, 70x100 cm, κομμένο σύμφωνα με λίστα
2 DC gear motors 12V, 80-120RPM, με encoder, 6/8mm D-shaft
2 motor drivers για brushed DC motors, π.χ. BTS7960/IBT-2
2 driven wheels 150-180mm rubber/PU ή hubs για τους άξονες
2 swivel casters 75-85mm με πλάκα
1 DC gear motor 12V 100-300RPM για collector intake roller
1 wide rubber/PU/TPU roller/cylinder 240-300mm x 60-90mm για collector
1 μικρός H-bridge driver για collector ή τρίτος αντίστοιχος driver
2 Cytron motor drivers ή ισοδύναμοι brushed DC drivers, αγορά μετά το μηχανικό fit της βάσης
1 ECO-WORTHY 12V 20Ah LiFePO4 battery με BMS ή ισοδύναμη 12.8V 20Ah >=20A discharge
1 LiFePO4 charger 14.6V 5A
1 12V DC-DC buck-boost regulator για σταθερό accessory 12V rail, αν χρειαστεί
1 12V -> 5V buck converter 5A+ για SBC/camera/sensors
1 Lenovo ThinkCentre M710q i5-7400T / 16GB / 256GB candidate για bench/on-board compute
1 emergency stop switch
1 fuse holder + ασφάλειες
κόκκινο/μαύρο καλώδιο 2.5mm² για μπαταρία/motors
καλώδιο 1.0-1.5mm² για collector/accessories
λεπτό καλώδιο 22-24AWG για encoder/control/sensors
ring/spade terminals, ferrules, XT60 ή Anderson connector, heat shrink
M5/M4 βίδες, ροδέλες, T-nuts/threaded inserts
D4 ξυλόκολλα, ξυλόβιδες, heat shrink, καλώδια, connectors
```
