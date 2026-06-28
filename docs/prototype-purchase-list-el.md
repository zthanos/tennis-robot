# Λίστα Παραγγελίας Πρώτου Prototype

Σκοπός: αγορά υλικών για το πρώτο φυσικό prototype του tennis robot:

```text
ξύλινη βάση + κινητήρια βάση + front collector + βασική ασφάλεια
```

## Sensor Baseline Update

Η νέα baseline επιλογή αισθητήρων είναι:

```text
Waveshare/Slamtec RPLIDAR C1 low 360 LiDAR -> obstacle scan / shadow zones / wall-net-fence clearance / route costmap
OAK-D S2 top camera -> ball detection / hidden ball recovery / ball distance / ball map
Encoders + IMU -> odometry / robot pose
```

Το Waveshare/Slamtec RPLIDAR C1 μπαίνει χαμηλά στο robot για real-time χάρτη εμποδίων, όρια γηπέδου, shadow zones και costmap. Δεν το χρησιμοποιούμε ως αξιόπιστο ball detector, γιατί η μπάλα 6.7 cm δίνει αδύναμο/ασταθές 2D return. Η OAK-D S2 μένει ψηλά ως ο κύριος αισθητήρας για ball detection, depth και targeted look σε περιοχές που το LiDAR δείχνει κρυμμένες ή μπλοκαρισμένες.

Προσθήκη στη λίστα αγοράς electronics/sensors:

| Qty | Είδος | Χρήση |
|---:|---|---|
| 1 | Waveshare/Slamtec RPLIDAR C1 από Amazon.de | Χαμηλό 360° 2D LiDAR για costmap, εμπόδια, shadow zones, φιλέ/τοίχο/φράχτη και ROS 2 navigation |
| 1 | USB/serial cable ή adapter που απαιτεί το C1 kit | Σύνδεση στο SBC / mini PC |
| 1 set | M3/M4 guarded, vibration-isolated lower mounting bracket | Σταθερή χαμηλή οριζόντια τοποθέτηση LiDAR, προστατευμένη από χτυπήματα και χωρίς να κόβεται το 360° scan από ρόδες/funnel |

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
| 4 | 120 x 80 | Motor pod reinforcement plates (4WD: one per pod) |
| 2 | 130 x 60 | Stabilizer bracket blocks |
| 2 | 100 x 60 | Handle socket backing plates |

Να υπολογιστεί απώλεια κοπής 3-5 mm ανά πέρασμα δίσκου.

## 2. Κίνηση Βάσης

### Drive Motors

Ζητάμε:

```text
4 τεμάχια DC gear motor 12V με encoder
```

Ιδανικά χαρακτηριστικά:

| Χαρακτηριστικό | Στόχος |
|---|---|
| Τάση | 12 V DC |
| Ταχύτητα | περίπου 120 RPM με τροχό 180 mm |
| Encoder | Ναι, quadrature αν γίνεται |
| Άξονας | 6 mm ή 8 mm D-shaft |
| Rated torque | τουλάχιστον 15-18 kg.cm |
| Stall current | να αναγράφεται στο datasheet |
| Gearbox | μεταλλικό |

Προτεινόμενο συγκεκριμένο μοντέλο για το prototype:

```text
4 τεμάχια Metal DC Geared Motor w/Encoder - 12V 122RPM 38Kg.cm
Link: https://grobotronics.com/metal-dc-geared-motor-w-encoder-12v-122rpm-38kg.cm.html
```

Χαρακτηριστικά του συγκεκριμένου μοτέρ:

| Χαρακτηριστικό | Τιμή |
|---|---|
| Motor type | Brushed DC gear motor με integrated quadrature encoder |
| Rated voltage | 12 V DC |
| No-load speed | 122 RPM |
| Rated torque | 38 kg.cm |
| Encoder | Ναι |
| Wheel target | 170 mm driven wheels (HPI Dirt Buster Block, αγοράστηκαν) |

Αυτό το setup αλλάζει τη βάση σε 4WD differential/skid-steer, χωρίς μπροστινούς
casters. Με τροχό 180 mm και 122 RPM η θεωρητική ταχύτητα είναι περίπου
1.15 m/s χωρίς φορτίο. Το χαμηλότερο RPM και η μεγαλύτερη ροπή είναι πιο
κατάλληλα για σταθερό πρώτο prototype, ειδικά με 4WD skid-steer σε γήπεδο.

Αν δεν υπάρχει με encoder:

```text
Δεν το προτιμάμε για τη βάση. Για αυτόνομη κίνηση και odometry θέλουμε encoder
σε κάθε drive motor ή τουλάχιστον αξιόπιστο encoder ανά πλευρά.
```

Specs επιβεβαιωμένα (DFRobot FIT0403):

- Encoder: ναι, quadrature· gearbox 90:1.
- **Shaft: 6 mm D-shaft** (επιβεβαιωμένο από datasheet + μέτρηση).
- Rated/stall current: ~7 A stall @ 12 V.
- Hub: **απαιτείται adapter 6 mm D-shaft → 24 mm hex** (βλ. ordered-parts.md → Parts To Order).

### Driven Wheels

**ΑΓΟΡΑΣΤΗΚΑΝ:** 4 × HPI Racing Dirt Buster Block Tire S Compound on Black Wheel
(Baja 5B Rear, 170x80 mm, μονταρισμένα με foam inserts). 2 ζευγάρια, μεταχειρισμένα
από RC offroad (8 kg, 90+ χλμ/ώ τελική) — άφθονη αντοχή για συλλέκτη μπαλών.

| Χαρακτηριστικό | Πραγματικό |
|---|---|
| Διάμετρος | 170 mm (radius 0.085 → ενημερώθηκε στο URDF + controllers.yaml) |
| Πλάτος | 80 mm |
| Πάτημα | rubber, S (soft) compound — καλό grip σε clay |
| Hub | **24 mm hex** (πρότυπο Baja 5B) |
| Στήριξη | **24 mm hex adapter, 6 mm bore (D-shaft)** προς τον FIT0403 (βλ. ordered-parts.md → Parts To Order) |

> Επόμενο βήμα: παραγγελία 4× hub adapters **6 mm D-shaft → 24 mm hex**.

### Front Casters

Δεν ζητάμε πλέον front casters για την κύρια κινητήρια βάση:

```text
Νέα baseline: 4 driven wheels, χωρίς casters, για καλύτερη πρόσφυση και πιο
προβλέψιμο έλεγχο σε ταχύτητα/στροφή.
```

Σημείωση CAD redesign:

```text
Το CAD πρέπει να ενημερωθεί από 2WD + front casters σε 4WD skid-steer:
δύο αριστεροί και δύο δεξιοί κινητήριοι τροχοί, με αντίστοιχα motor pods και
χώρο για 4 drivers / καλωδίωση / ασφάλειες.
```

## 3. Motor Drivers Και Τροφοδοσία Κίνησης

Ζητάμε:

```text
2 τεμάχια motor driver για brushed DC motor στο πρώτο prototype
```

Budget επιλογή:

```text
BTS7960 / IBT-2 driver, ένας ανά πλευρά
Link: https://grobotronics.com/high-power-dc-motor-driver-dual-bts7960-half-bridge-43a.html
```

Στο πρώτο prototype κάθε BTS7960 οδηγεί τα δύο μοτέρ της ίδιας πλευράς
παράλληλα:

```text
Left BTS7960  -> left front motor + left rear motor
Right BTS7960 -> right front motor + right rear motor
```

Αν δούμε θερμοκρασίες, υπερβολικό ρεύμα, άνισο τράβηγμα ή κακή απόκριση στα
encoders, προσθέτουμε άλλα 2 BTS7960 ώστε να πάμε σε ένα driver ανά μοτέρ.

Prototype drive cost snapshot, 2026-06-21:

| Qty | Είδος | Link | Τιμή μονάδας | Σύνολο |
|---:|---|---|---:|---:|
| 4 | Metal DC Geared Motor w/Encoder - 12V 122RPM 38Kg.cm | https://grobotronics.com/metal-dc-geared-motor-w-encoder-12v-122rpm-38kg.cm.html | 36,00 € | 144,00 € |
| 2 | High-Power DC Motor Driver Dual BTS7960 Half-bridge 43A | https://grobotronics.com/high-power-dc-motor-driver-dual-bts7960-half-bridge-43a.html | 15,90 € | 31,80 € |
| 1 | Arduino Mega 2560 Rev3 | https://grobotronics.com/arduino-mega-2560-rev3.html | 39,00 € | 39,00 € |
|  | **Subtotal κίνησης χωρίς τροχούς/μεταφορικά** |  |  | **214,80 €** |

Το Arduino Mega μπαίνει ως ξεχωριστό motion MCU για encoders, PID ταχύτητας,
acceleration ramps και watchdog. Teensy/ESP32 παραμένουν καλύτερες/εναλλακτικές
επιλογές, αλλά το Mega είναι απλό και διαθέσιμο για πρώτο bring-up.

Να αγοραστεί αφού:

1. κουμπώσει η ξύλινη βάση,
2. τοποθετηθούν motor pods / τροχοί,
3. ξέρουμε το πραγματικό ρεύμα των drive motors,
4. επιβεβαιώσουμε ότι ο driver καλύπτει το stall/peak current των μοτέρ.

Για production-like έκδοση, αντικαθιστούμε τα BTS7960 με 2 ποιοτικούς
dual-channel DC motor drivers ή 4 ποιοτικούς single-channel drivers με
τεκμηριωμένο continuous current, προστασίες και ιδανικά current sensing.
Ο collector μπορεί να χρησιμοποιήσει μικρότερο H-bridge ή άλλο BTS7960 αν
θέλουμε κοινό hardware.

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
| Θέση στο σασί | όσο πιο μπροστά γίνεται, ώστε ο κύλινδρος να πιάνει τη μπάλα πριν τη σπρώξει η βάση |

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
| 1 | Emergency stop switch | Normally-closed, latching mushroom, να κόβει φυσικά την τροφοδοσία στα μοτέρ |
| 1 | Fuse holder | Για την κύρια γραμμή μπαταρίας |
| 2-4 | Ασφάλειες αυτοκινήτου | Main motor fuse 20A αρχικά, electronics fuse 3A-5A, accessory fuse αν χρειαστεί |
| 1 | Κεντρικός διακόπτης | Battery on/off |
| 1 | Start/arm push button | Logic input προς Arduino Mega, δεν αντικαθιστά το E-stop |
| 1 | MPU6050 IMU module | 3-axis gyroscope + 3-axis accelerometer, I2C, Arduino/ROS support, για odometry diagnostics |
| 1 | Power distribution block / terminal blocks | Διανομή +12V/GND προς BTS7960 χωρίς να περνάει ρεύμα από perfboard |
| 1 | Motor-power relay/contactor, optional | Μόνο αν δεν περνάμε το motor current απευθείας από το E-stop/main switch |
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

**Απόφαση: Raspberry Pi 5 (16GB) ως on-board computer** — αντικαθιστά το παλιό
candidate Lenovo i5 mini-PC. Πλήρες σκεπτικό και κόστος στο
[`hardware-bom-el.md`](hardware-bom-el.md).

Σύνοψη επιλεγμένου compute:

```text
Raspberry Pi 5, 16GB LPDDR4X
Broadcom BCM2712, quad-core Cortex-A76 @ 2.4 GHz
PCIe M.2 NVMe board + Silicon Power P34A60 256GB 2280
(`SP256GBP34A60M28AY`) ως υποψήφιο boot drive
Active cooler (υποχρεωτικός σε αυτό το φορτίο)
Τροφοδοσία: 5V/5A — από την μπαταρία μέσω 12V->5V buck converter 5A+
```

Γιατί Pi 5 αντί για i5 mini-PC (το ρομπότ είναι μπαταριοκίνητο/κινούμενο):

| Κριτήριο | Pi 5 16GB | i5 mini-PC | Νικητής |
|---|---|---|---|
| Κατανάλωση | ~5-10W | ~25-45W | **Pi** (3-5x λιγότερο → μικρότερη μπαταρία/μεγαλύτερη αυτονομία) |
| Τροφοδοσία | 5V, απευθείας από buck | 12-19V, θέλει ειδικό DC-DC | **Pi** |
| Μέγεθος/βάρος | μικροσκοπικό | μεγαλύτερο | **Pi** |
| Σύνδεση MCU (Mega/Nano) | USB serial | USB serial | ισοπαλία (USB CDC) |
| Θερμότητα σε κλειστό σασί | εύκολη ψύξη | πιο δύσκολη | **Pi** |
| Compute headroom | αρκετό (+OAK-D offload) | περισσότερο | i5 |
| Dev: Gazebo onboard | όχι (δεν χρειάζεται onboard) | ναι | i5 (μόνο για bench) |

Το μόνο πλεονέκτημα του i5 (compute headroom) ακυρώνεται επειδή το βαρύ vision
(stereo depth + ανίχνευση) γίνεται offload στην **OAK-D** (Myriad X VPU). Δες
`docs/telemetry-architecture-el.md` και την ανάλυση compute budget.

Σημαντική σημείωση τροφοδοσίας:

```text
Το Pi 5 θέλει σταθερά 5V/5A. Πάνω στο robot τροφοδοτείται από την LiFePO4
μέσω του 12V->5V buck converter 5A+ (ήδη στη λίστα). Καλό 5V/5A rail είναι
ΑΠΑΡΑΙΤΗΤΟ για σταθερό NVMe — υποτροφοδοσία προκαλεί throttle/αστάθεια δίσκου.
Το επίσημο 27W PSU του kit είναι μόνο για bench/development.
```

Τι να επιβεβαιώσουμε πριν την τοποθέτηση:

- ότι ο buck converter δίνει σταθερά 5V υπό φορτίο (Pi + OAK-D + LiDAR USB),
- ότι το metal case του kit αφήνει πρόσβαση στις **4 θύρες USB** (OAK-D, LiDAR,
  Mega, Nano), αλλιώς βγάζουμε τη board από το case για το on-robot mount,
  (ο IMU MPU6050 είναι στο I2C του Mega, όχι στα GPIO του Pi — δες hardware-bom-el.md §5),
- ότι ο Silicon Power P34A60 αναγνωρίζεται και περνά boot/reboot/SMART/stress
  test με το συγκεκριμένο PCIe M.2 board (δες validation steps στο BOM),
- αν αντέχει κραδασμούς ή χρειάζεται rubber isolation/standoffs,
- επαρκές Wi-Fi για το telemetry link (αλλιώς external antenna/USB).

## 7. Βίδες, Ροδέλες, Inserts

Πάρε αρκετά, γιατί στο prototype θα λυθούν/δεθούν πολλές φορές.

| Qty | Είδος | Χρήση |
|---:|---|---|
| 30-50 | M5 bolts, διάφορα μήκη | motor pods (x4), collector, handle |
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
4 Metal DC Geared Motor w/Encoder - 12V 122RPM 38Kg.cm: https://grobotronics.com/metal-dc-geared-motor-w-encoder-12v-122rpm-38kg.cm.html
2 BTS7960/IBT-2 motor drivers για brushed DC motors, ένας ανά πλευρά στο πρώτο prototype: https://grobotronics.com/high-power-dc-motor-driver-dual-bts7960-half-bridge-43a.html
προαιρετικά +2 BTS7960 αν χρειαστεί ένα driver ανά μοτέρ
1 Arduino Mega 2560 Rev3 για motion MCU: https://grobotronics.com/arduino-mega-2560-rev3.html
4 driven wheels 180mm rubber/PU ή hubs για τους άξονες
1 DC gear motor 12V 100-300RPM για collector intake roller
1 wide rubber/PU/TPU roller/cylinder 240-300mm x 60-90mm για collector
1 μικρός H-bridge driver για collector ή τρίτος αντίστοιχος driver
production-like εναλλακτική: 2 dual-channel quality drivers ή 4 single-channel quality drivers
1 ECO-WORTHY 12V 20Ah LiFePO4 battery με BMS ή ισοδύναμη 12.8V 20Ah >=20A discharge
1 LiFePO4 charger 14.6V 5A
1 12V DC-DC buck-boost regulator για σταθερό accessory 12V rail, αν χρειαστεί
1 12V -> 5V buck converter 5A+ για SBC/camera/sensors
1 db-tronic Raspberry Pi 5 16GB PCIe M.2 NVMe Set, 64GB Edition
  (M.2 board + active cooler + 27W PSU + metal case), Amazon basket €385,54
1 Silicon Power P34A60 256GB NVMe M.2 PCIe Gen3x4 2280
  (SP256GBP34A60M28AY), Amazon basket €50,99 — boot candidate, ΟΧΙ ακόμη validated
1 emergency stop switch
1 fuse holder + ασφάλειες
κόκκινο/μαύρο καλώδιο 2.5mm² για μπαταρία/motors
καλώδιο 1.0-1.5mm² για collector/accessories
λεπτό καλώδιο 22-24AWG για encoder/control/sensors
ring/spade terminals, ferrules, XT60 ή Anderson connector, heat shrink
M5/M4 βίδες, ροδέλες, T-nuts/threaded inserts
D4 ξυλόκολλα, ξυλόβιδες, heat shrink, καλώδια, connectors
```
