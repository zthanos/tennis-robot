# Motion Perfboard Wiring Reference

Αυτό το έγγραφο περιγράφει την πλακέτα perfboard για το πρώτο 4WD motion
prototype του tennis robot.

Στόχος: να φτιαχτεί μια καθαρή logic/distribution πλακέτα για:

- Arduino Mega 2560 Rev3 ως motion MCU
- 2x BTS7960 / IBT-2 motor drivers, ένας ανά πλευρά
- 4x DC geared motors με encoder, δύο μοτέρ παράλληλα ανά πλευρά
- encoder headers για odometry/diagnostics
- MPU6050 IMU για yaw-rate/acceleration diagnostics μέσω I2C
- κοινό logic ground, 5V encoder rail και καθαρά control signals

## Διαγράμματα

Επισκόπηση συστήματος (power domain εκτός perfboard vs logic domain):

![Motion control wiring](images/motion-control-wiring.svg)

Pinout των headers J1-J9 — πώς ενώνεται κάθε component στην perfboard:

![Motion perfboard header pinout](images/motion-perfboard-pinout.svg)

## 1. Σημαντικό Όριο Της Perfboard

Η perfboard **δεν μεταφέρει ρεύμα μοτέρ**.

Πάνω στην perfboard περνάνε μόνο:

```text
Arduino Mega control signals
BTS7960 logic pins
encoder signals
5V encoder supply
common GND reference
```

Το 12V high-current κομμάτι περνάει εκτός perfboard:

```text
Battery +
  -> main fuse
  -> emergency stop
  -> main switch
  -> motor power relay/contact or manual armed feed
  -> power distribution / terminal block
  -> BTS7960 B+ terminals

Battery -
  -> power distribution / terminal block
  -> BTS7960 B- terminals
  -> Arduino/perfboard GND reference
```

Μη βάλεις ποτέ τα ρεύματα μοτέρ μέσα από λεπτές perfboard διαδρομές.

## 2. Safety Chain: Fuse, Start, E-Stop

Η πρώτη κίνηση του robot πρέπει να έχει φυσική ασφάλεια, όχι μόνο software stop.

### High-current motor power chain

```text
Battery +
  -> main motor fuse, 20A αρχικά
  -> emergency stop, normally-closed, latching
  -> main power switch
  -> optional motor-power relay/contactor
  -> distribution block
  -> Left BTS7960 B+
  -> Right BTS7960 B+

Battery -
  -> distribution block
  -> Left BTS7960 B-
  -> Right BTS7960 B-
```

Προτεινόμενες αρχικές ασφάλειες:

| Fuse | Προτεινόμενη τιμή | Κόβει | Σημείωση |
|---|---:|---|---|
| Main motor fuse | 20A αρχικά, 25A μόνο αν χρειαστεί | Drive motor branch | Να είναι κοντά στη μπαταρία |
| Electronics fuse | 3A-5A | Arduino/SBC/sensors branch | Ξεχωριστή από τα μοτέρ |
| Accessory 12V fuse | 5A-10A | Buck-boost/accessories | Μόνο αν υπάρχει accessory rail |

Η τιμή της main fuse θα οριστικοποιηθεί μετά από μέτρηση ρεύματος. Ξεκινάμε
συντηρητικά, γιατί στο πρώτο prototype θέλουμε να βρούμε λάθη καλωδίωσης πριν
γίνουν ακριβά.

### Emergency stop

Το emergency stop πρέπει να κόβει **φυσικά** την τροφοδοσία των motor drivers.

```text
E-stop pressed
  -> no +12V at BTS7960 B+
  -> motors cannot move even if Arduino/PWM fails
```

Προτίμηση:

- normally-closed contact,
- latching mushroom button,
- rated για DC current πάνω από την επιλεγμένη ασφάλεια ή να οδηγεί relay/contactor
  που έχει το κατάλληλο DC rating.

Μην βασιστείς μόνο σε κουμπί που πάει σε Arduino input για emergency stop.
Μπορούμε να το διαβάζουμε και από το Arduino για status, αλλά η πραγματική
ασφάλεια πρέπει να κόβει το power path.

### Start / Arm button

Το start button δεν είναι emergency stop. Είναι logic input προς το Arduino Mega
για να οπλίζει την κίνηση μετά από reset ή μετά από E-stop release.

Προτεινόμενη λογική:

```text
Power on
  -> Arduino boots DISARMED
  -> BTS7960 enables LOW
  -> user presses START/ARM
  -> Arduino checks command timeout + encoder sanity
  -> Arduino enables BTS7960

E-stop pressed or command timeout
  -> Arduino disables BTS7960
  -> motor power is also cut by E-stop chain
```

Προτεινόμενο start button wiring:

| Start button | Arduino Mega | Σκοπός |
|---|---|---|
| One side | D32 | `START_ARM` input |
| Other side | `LOGIC_GND` | Active-low button |

Στο firmware:

```cpp
pinMode(32, INPUT_PULLUP);
```

Με αυτό:

```text
HIGH = not pressed
LOW  = pressed
```

### Optional E-stop status input

Αν το E-stop έχει δεύτερο βοηθητικό contact, διάβασέ το στο Arduino:

| E-stop auxiliary contact | Arduino Mega | Σκοπός |
|---|---|---|
| One side | D33 | `ESTOP_STATUS` input |
| Other side | `LOGIC_GND` | Active-low status |

Αυτό είναι μόνο telemetry/status. Δεν αντικαθιστά το φυσικό κόψιμο του 12V motor
power.

## 3. System Topology

```text
Arduino Mega 2560
    |
    | PWM / enable signals
    v
Motion perfboard
    |
    +--> Left BTS7960  -> left front motor + left rear motor
    |
    +--> Right BTS7960 -> right front motor + right rear motor
    |
    +<-- 4x encoder A/B signals
```

Για το πρώτο prototype, κάθε BTS7960 οδηγεί δύο μοτέρ παράλληλα στην ίδια πλευρά.
Αν δούμε υπερβολικό ρεύμα, ζέσταμα ή άνισο τράβηγμα, προσθέτουμε άλλα 2 BTS7960
ώστε να πάμε σε ένα driver ανά μοτέρ.

### Wiring Diagram

```mermaid
flowchart LR
    BAT["12V Battery"]
    FUSE["Main Motor Fuse<br/>20A initial"]
    ESTOP["Emergency Stop<br/>NC latching"]
    MSW["Main Switch"]
    DIST["12V/GND Distribution<br/>terminal block"]
    LDRV["Left BTS7960<br/>driver"]
    RDRV["Right BTS7960<br/>driver"]
    LFM["Left Front<br/>Motor"]
    LRM["Left Rear<br/>Motor"]
    RFM["Right Front<br/>Motor"]
    RRM["Right Rear<br/>Motor"]

    MEGA["Arduino Mega<br/>Motion MCU"]
    PERF["Motion Perfboard<br/>logic + encoder headers"]
    START["Start / Arm<br/>button"]
    AUX["Optional E-stop<br/>status contact"]
    IMU["MPU6050 IMU<br/>I2C"]
    LFENC["LF Encoder"]
    LRENC["LR Encoder"]
    RFENC["RF Encoder"]
    RRENC["RR Encoder"]

    BAT --> FUSE --> ESTOP --> MSW --> DIST
    DIST --> LDRV
    DIST --> RDRV
    LDRV --> LFM
    LDRV --> LRM
    RDRV --> RFM
    RDRV --> RRM

    MEGA <-->|"5V, GND, PWM, EN, encoder signals"| PERF
    PERF -->|"RPWM, LPWM, EN"| LDRV
    PERF -->|"RPWM, LPWM, EN"| RDRV
    START --> PERF
    AUX --> PERF
    IMU -->|"SDA, SCL, 5V, GND"| PERF
    LFENC --> PERF
    LRENC --> PERF
    RFENC --> PERF
    RRENC --> PERF
```

Στο διάγραμμα, η επάνω διαδρομή είναι το high-current motor power. Η κάτω
διαδρομή είναι logic/control/encoder και περνάει από την perfboard.

## 4. Arduino Mega Pin Map

### BTS7960 Control Pins

| Arduino Mega | Perfboard net | BTS7960 pin | Σκοπός |
|---|---|---|---|
| D5 PWM | `LEFT_RPWM` | Left driver `RPWM` | Αριστερή πλευρά, φορά A |
| D6 PWM | `LEFT_LPWM` | Left driver `LPWM` | Αριστερή πλευρά, φορά B |
| D30 | `LEFT_EN` | Left driver `R_EN` + `L_EN` | Enable αριστερού driver |
| D9 PWM | `RIGHT_RPWM` | Right driver `RPWM` | Δεξιά πλευρά, φορά A |
| D10 PWM | `RIGHT_LPWM` | Right driver `LPWM` | Δεξιά πλευρά, φορά B |
| D31 | `RIGHT_EN` | Right driver `R_EN` + `L_EN` | Enable δεξιού driver |
| 5V | `LOGIC_5V` | Both drivers `VCC` | Logic τροφοδοσία |
| GND | `LOGIC_GND` | Both drivers `GND` | Κοινή γείωση |

Σημείωση: στα περισσότερα BTS7960 modules τα `R_EN` και `L_EN` μπορούν να
δεθούν μαζί και να οδηγούνται από ένα enable pin ανά driver. Αν το συγκεκριμένο
module απαιτήσει ξεχωριστό έλεγχο, κρατάμε χώρο στην perfboard για να τα
χωρίσουμε αργότερα.

### Encoder Pins

| Motor | Encoder A | Encoder B | Encoder VCC | Encoder GND |
|---|---|---|---|---|
| Left front | D2 interrupt | D22 | `ENC_5V` | `LOGIC_GND` |
| Left rear | D3 interrupt | D23 | `ENC_5V` | `LOGIC_GND` |
| Right front | D18 interrupt | D24 | `ENC_5V` | `LOGIC_GND` |
| Right rear | D19 interrupt | D25 | `ENC_5V` | `LOGIC_GND` |

### Safety / User Inputs

| Arduino Mega | Net | Χρήση |
|---|---|---|
| D32 | `START_ARM` | Start/arm button, active-low with `INPUT_PULLUP` |
| D33 | `ESTOP_STATUS` | Optional E-stop auxiliary status, active-low with `INPUT_PULLUP` |

### MPU6050 IMU / I2C

| Arduino Mega | Net | MPU6050 pin | Χρήση |
|---|---|---|---|
| D20 / SDA | `I2C_SDA` | `SDA` | I2C data |
| D21 / SCL | `I2C_SCL` | `SCL` | I2C clock |
| 5V or 3.3V | `IMU_VCC` | `VCC` | Τροφοδοσία module |
| GND | `LOGIC_GND` | `GND` | Κοινή γείωση |

Τα περισσότερα MPU6050 breakout modules δέχονται 5V επειδή έχουν regulator,
αλλά αυτό πρέπει να επιβεβαιωθεί στο συγκεκριμένο module. Αν το module γράφει
μόνο 3.3V, χρησιμοποίησε 3.3V και έλεγξε αν χρειάζεται level shifting στο I2C.

Στο prototype το MPU6050 χρησιμοποιείται για yaw-rate/acceleration sanity checks
και βελτίωση odometry diagnostics. Δεν το αντιμετωπίζουμε ως απόλυτη πυξίδα.

Για αρχικό bring-up μετράμε interrupt στο `A` και διαβάζουμε το `B` για φορά.
Αργότερα, αν θέλουμε πλήρες quadrature decoding και στα 8 edges, μπορούμε να
περάσουμε σε Teensy/ESP32 ή σε pin-change interrupt library.

## 5. Perfboard Headers

### Header J1 - Arduino Mega Control

| Pin | Net | Προς |
|---:|---|---|
| 1 | `LEFT_RPWM` | Mega D5 |
| 2 | `LEFT_LPWM` | Mega D6 |
| 3 | `RIGHT_RPWM` | Mega D9 |
| 4 | `RIGHT_LPWM` | Mega D10 |
| 5 | `LEFT_EN` | Mega D30 |
| 6 | `RIGHT_EN` | Mega D31 |
| 7 | `START_ARM` | Mega D32 |
| 8 | `ESTOP_STATUS` | Mega D33, optional |
| 9 | `I2C_SDA` | Mega D20 / SDA |
| 10 | `I2C_SCL` | Mega D21 / SCL |
| 11 | `LOGIC_5V` | Mega 5V |
| 12 | `LOGIC_GND` | Mega GND |

### Header J2 - Left BTS7960 Logic

| Pin | Net | BTS7960 pin |
|---:|---|---|
| 1 | `LEFT_RPWM` | `RPWM` |
| 2 | `LEFT_LPWM` | `LPWM` |
| 3 | `LEFT_EN` | `R_EN` |
| 4 | `LEFT_EN` | `L_EN` |
| 5 | `LOGIC_5V` | `VCC` |
| 6 | `LOGIC_GND` | `GND` |

### Header J3 - Right BTS7960 Logic

| Pin | Net | BTS7960 pin |
|---:|---|---|
| 1 | `RIGHT_RPWM` | `RPWM` |
| 2 | `RIGHT_LPWM` | `LPWM` |
| 3 | `RIGHT_EN` | `R_EN` |
| 4 | `RIGHT_EN` | `L_EN` |
| 5 | `LOGIC_5V` | `VCC` |
| 6 | `LOGIC_GND` | `GND` |

### Headers J4-J7 - Motor Encoders

Χρησιμοποίησε 4-pin headers, ένα ανά motor encoder:

| Header | Motor | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
|---|---|---|---|---|---|
| J4 | Left front encoder | `ENC_5V` | `LOGIC_GND` | D2 / `LF_ENC_A` | D22 / `LF_ENC_B` |
| J5 | Left rear encoder | `ENC_5V` | `LOGIC_GND` | D3 / `LR_ENC_A` | D23 / `LR_ENC_B` |
| J6 | Right front encoder | `ENC_5V` | `LOGIC_GND` | D18 / `RF_ENC_A` | D24 / `RF_ENC_B` |
| J7 | Right rear encoder | `ENC_5V` | `LOGIC_GND` | D19 / `RR_ENC_A` | D25 / `RR_ENC_B` |

Τα χρώματα καλωδίων των encoders να επιβεβαιωθούν από το datasheet/label του
μοτέρ πριν κολληθούν. Μην υποθέσεις ότι είναι ίδια με άλλο motor model.

### Header J8 - User Controls

| Pin | Net | Σύνδεση |
|---:|---|---|
| 1 | `START_ARM` | Start/arm button side A |
| 2 | `LOGIC_GND` | Start/arm button side B |
| 3 | `ESTOP_STATUS` | Optional E-stop auxiliary side A |
| 4 | `LOGIC_GND` | Optional E-stop auxiliary side B |

### Header J9 - MPU6050 IMU

| Pin | Net | MPU6050 pin |
|---:|---|---|
| 1 | `IMU_VCC` | `VCC` |
| 2 | `LOGIC_GND` | `GND` |
| 3 | `I2C_SDA` | `SDA` |
| 4 | `I2C_SCL` | `SCL` |

Το `XDA`, `XCL`, `AD0` και `INT` του MPU6050 μένουν ασύνδετα στο πρώτο bring-up,
εκτός αν η βιβλιοθήκη/firmware που θα χρησιμοποιήσουμε ζητήσει interrupt pin.

## 6. Motor Power Wiring

Η ισχύς δεν περνάει από την perfboard.

### Left Side Motors

| Left BTS7960 terminal | Σύνδεση |
|---|---|
| `B+` | +12V μετά από fuse/E-stop/switch |
| `B-` | Battery GND / power ground |
| `M+` | Left front motor lead A + left rear motor lead A |
| `M-` | Left front motor lead B + left rear motor lead B |

### Right Side Motors

| Right BTS7960 terminal | Σύνδεση |
|---|---|
| `B+` | +12V μετά από fuse/E-stop/switch |
| `B-` | Battery GND / power ground |
| `M+` | Right front motor lead A + right rear motor lead A |
| `M-` | Right front motor lead B + right rear motor lead B |

Αν μία πλευρά γυρίζει ανάποδα, διόρθωσε πρώτα στο software. Αν χρειαστεί,
αντάλλαξε `M+`/`M-` στη συγκεκριμένη πλευρά. Μην αλλάξεις τα encoder wires για
να διορθώσεις φορά μοτέρ.

## 7. Grounding Plan

Όλα πρέπει να έχουν κοινή αναφορά γείωσης:

```text
Battery GND
  -> BTS7960 B-
  -> BTS7960 logic GND
  -> perfboard LOGIC_GND
  -> Arduino Mega GND
  -> encoder GND
```

Χωρίς κοινό ground, τα PWM και encoder signals μπορεί να φαίνονται τυχαία ή να
μην λειτουργούν καθόλου.

## 8. 5V Plan

Για το πρώτο prototype:

```text
Arduino Mega 5V -> perfboard LOGIC_5V / ENC_5V
```

Αυτό τροφοδοτεί:

- BTS7960 logic `VCC`
- encoder `VCC`

Μη χρησιμοποιήσεις το 5V rail για μοτέρ, relay coils ή άλλα φορτία. Αν αργότερα
μπουν πολλοί αισθητήρες, βάλε ξεχωριστό 5V buck converter και κράτα κοινό GND.

## 9. Recommended Perfboard Layout

Πρακτική διάταξη:

```text
[J1 Arduino Mega header]      [J2 Left BTS7960 logic]   [J3 Right BTS7960 logic]

[J4 LF encoder] [J5 LR encoder] [J6 RF encoder] [J7 RR encoder]

5V rail along top
GND rail along bottom
Signal wires short and labelled
```

Πρότεινε χρώματα:

| Χρώμα | Χρήση |
|---|---|
| Κόκκινο | `LOGIC_5V` / `ENC_5V` |
| Μαύρο | `LOGIC_GND` |
| Κίτρινο | PWM signals |
| Πορτοκαλί | Enable signals |
| Μπλε/Πράσινο | Encoder A/B |
| Λευκό | Start/E-stop status inputs |
| Μωβ | I2C SDA/SCL προς MPU6050 |

## 10. First Power-On Checklist

Πριν συνδέσεις τα μοτέρ στους drivers:

1. Μέτρα `LOGIC_5V` προς `LOGIC_GND`: πρέπει να είναι περίπου 5V.
2. Μέτρα συνέχεια `Arduino GND` προς `BTS7960 logic GND`: πρέπει να είναι σχεδόν 0 ohm.
3. Επιβεβαίωσε ότι δεν υπάρχει 12V πάνω στην perfboard.
4. Επιβεβαίωσε ότι κάθε encoder παίρνει 5V και GND στη σωστή πολικότητα.
5. Επιβεβαίωσε ότι το MPU6050 VCC ταιριάζει με το module: 5V tolerant ή 3.3V only.
6. Επιβεβαίωσε ότι το E-stop κόβει το +12V στα BTS7960 `B+`.
7. Επιβεβαίωσε ότι το main fuse είναι κοντά στη μπαταρία.
8. Επιβεβαίωσε ότι το start button διαβάζεται `HIGH` idle και `LOW` pressed.
9. Σήκωσε `LEFT_EN` / `RIGHT_EN` μόνο από software, όχι με μόνιμο jumper στο 5V.
10. Δοκίμασε πρώτα encoder hand test, γυρίζοντας κάθε τροχό με το χέρι.
11. Δοκίμασε MPU6050 I2C scan και stationary gyro bias check.
12. Δοκίμασε κάθε πλευρά με χαμηλό PWM χωρίς βάρος στο robot.
13. Μετά από 10-20 δευτερόλεπτα, έλεγξε θερμοκρασία BTS7960 και καλωδίων.

## 11. First Motion Test Limits

Για τις πρώτες δοκιμές:

```text
PWM limit: 40-80 / 255
test duration: 2-5 seconds
robot lifted or wheels off ground first
then very slow floor test
```

Σταμάτα αμέσως αν:

- ζεσταίνεται γρήγορα driver ή καλώδιο,
- μυρίζει πλαστικό/μονωτικό,
- κάποια πλευρά τραβάει πολύ πιο δυνατά,
- οι encoder counts πάνε ανάποδα ή μηδενίζουν,
- πέφτει/κάνει reset το Arduino Mega.

## 12. Upgrade Path Σε 4 Drivers

Αν χρειαστούν 4 BTS7960, κρατάμε την ίδια λογική αλλά χωρίζουμε τα motor outputs:

```text
Left front BTS7960  -> left front motor
Left rear BTS7960   -> left rear motor
Right front BTS7960 -> right front motor
Right rear BTS7960  -> right rear motor
```

Η perfboard μπορεί να επεκταθεί με δύο επιπλέον BTS7960 logic headers. Τα encoder
headers δεν αλλάζουν.
