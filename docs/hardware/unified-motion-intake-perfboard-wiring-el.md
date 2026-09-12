# Ενιαία perfboard motion + intake — σχέδιο κόλλησης Rev A

Το παρόν είναι το σημείο αναφοράς για την κοινή perfboard 120 × 80 mm του
Arduino Mega. Αν κάποιο παλιότερο διάγραμμα δείχνει ξεχωριστά headers, δεν
χρησιμοποιείται για αυτή την κατασκευή.

## 1. Επιβεβαιωμένη πλακέτα και προσανατολισμός

- FR-4, 120 × 80 × 1,6 mm, double-sided plated-through.
- Βήμα 2,54 mm.
- Πραγματικό πλέγμα από φωτογραφία: **42 στήλες × 30 σειρές**.
- Σε component-side οριζόντια όψη: `C1` αριστερά, `C42` δεξιά, `R1` κάτω,
  `R30` πάνω.
- Σε solder-side όψη, κρατώντας την ίδια κάτω ακμή προς τον τεχνικό, οι
  στήλες καθρεφτίζονται: `C42` αριστερά και `C1` δεξιά. Οι σειρές δεν αλλάζουν.
- Το κόκκινο σημάδι στο 3D label δείχνει πάντα το pin 1.

Οι μεγάλες ακριανές επαφές δεν είναι εργοστασιακά ενωμένες μεταξύ τους.
Γίνονται ράγες μόνο αφού γεφυρωθούν στην κάτω όψη:

- αριστερή πλευρά: `LOGIC_5V`,
- δεξιά πλευρά: `LOGIC_GND`.

Χρησιμοποιείται συνεχές επικασσιτερωμένο χάλκινο σύρμα 0,8–1,0 mm. Κολλάμε
κάθε ακριανό pad, χωρίς να πλησιάσουμε τις οπές στερέωσης. Οι ράγες είναι μόνο
για logic και αισθητήρες· **ποτέ 12 V ή ρεύμα μοτέρ**.

## 2. Θέσεις headers στην πάνω όψη

Οι συντεταγμένες είναι φυσικές θέσεις οπών της component side.

| Header | Θέση | Pin 1 |
|---|---|---|
| `J_MEGA`, IDC 2×17 | C3–C4, R6–R22 | C4/R6 |
| `J_MOTION`, IDC 2×14 | C16–C29, R6–R7 | C16/R6 |
| `J_INTAKE`, IDC 2×8 | C17–C24, R13–R14 | C17/R13 |
| `J_IR`, IDC 2×5 | C31–C35, R13–R14 | C31/R13 |
| `J_USER`, IDC 2×3 | C17–C19, R20–R21 | C17/R20 |
| `J_IMU`, label `GYRO`, IDC 2×2 | C24–C25, R20–R21 | C24/R20 |
| `JP_PWR`, 1×3 | C32–C34, R20 | C32/R20 |
| `PWR_IN`, 1×2 | C37–C38, R20 | C37/R20 |

Για τα οριζόντια IDC, τα odd pins είναι στην κάτω σειρά και τα even στην
επάνω. Στο κάθετο `J_MEGA`, τα odd pins είναι στη C4 και τα even στη C3.

### Τι περιλαμβάνει πραγματικά η perfboard

Η perfboard περιλαμβάνει όλες τις **logic συνδέσεις** του motion και intake:

- δύο BTS7960 logic interfaces και τέσσερις drive encoders,
- dual intake driver interface και δύο intake encoders,
- δύο IR break beams,
- START, E-stop status και armed LED,
- GY-521 / MPU6050 gyro + accelerometer,
- επιλογή τροφοδοσίας 5V, εξωτερική είσοδο 5V και κοινή γείωση.

Δεν τοποθετούνται πάνω της τα BTS7960, ο intake driver, το Mega, τα μοτέρ, το
relay, οι ασφάλειες, η μπαταρία ή η 12V διανομή. Συνδέονται με harness και τα
ρεύματα κινητήρων παραμένουν εντελώς εκτός perfboard.

### Υλικά που απαιτούνται για να ολοκληρωθεί

- headers συνολικά 103 ηλεκτρικών θέσεων σύμφωνα με τον παραπάνω πίνακα,
- 1× jumper για το `JP_PWR` και 1× πολωμένο 2-pin `PWR_IN`,
- 11× κεραμικοί πυκνωτές 100 nF,
- 1× ηλεκτρολυτικός 220 µF / τουλάχιστον 10 V,
- 1× 470 Ω και 2× 10 kΩ,
- σύρμα χαλκού 0,8–1,0 mm για τις δύο rails,
- μονωμένο πολύκλωνο 26–28 AWG για τα signals,
- συμβατά βύσματα/housings και strain relief για όλα τα harnesses.

Τα 4× female strips 1×20 που έχουν καταγραφεί στη λίστα παραγγελίας δίνουν 80
θέσεις, άρα **δεν επαρκούν** για το νέο ενιαίο layout των 103 θέσεων. Αν
χρησιμοποιηθούν κομμένες απλές σειρές αντί για keyed IDC, χρειάζονται
τουλάχιστον δύο ακόμη 1×20 strips και αυστηρή σήμανση pin 1. Για πραγματικά
keyed harnesses πρέπει να αγοραστούν τα αντίστοιχα male box headers και plugs.

## 3. Pinout J_MEGA

| Pins | Mega nets | Pins | Mega nets |
|---:|---|---:|---|
| 1 / 2 | D5 / D6 | 3 / 4 | D30 / D9 |
| 5 / 6 | D10 / D31 | 7 / 8 | D2 / D22 |
| 9 / 10 | D3 / D23 | 11 / 12 | D18 / D24 |
| 13 / 14 | D19 / D25 | 15 / 16 | D44 / D40 |
| 17 / 18 | D41 / D45 | 19 / 20 | D42 / D43 |
| 21 / 22 | A8 / A9 | 23 / 24 | A10 / A11 |
| 25 / 26 | D36 / D37 | 27 / 28 | D32 / D33 |
| 29 / 30 | D34 / D20-SDA | 31 / 32 | D21-SCL / MEGA_5V |
| 33 / 34 | MEGA_GND / KEY-NC |  |  |

Το pin 34 δεν συνδέεται. Χρησιμοποιείται ως key μόνο αν το αντίστοιχο βύσμα
έχει φραγμένη θέση.

## 4. Point-to-point signal netlist

Χρησιμοποιούνται μονωμένα stranded καλώδια 26–28 AWG. Κάθε γραμμή παρακάτω
είναι μία ηλεκτρική σύνδεση. Τα crossings είναι αποδεκτά μόνο με άθικτη μόνωση.

### Motion driver και drive encoders

| Από J_MEGA | Προς J_MOTION | Net |
|---:|---:|---|
| 1 | 1 | LEFT_RPWM |
| 2 | 2 | LEFT_LPWM |
| 3 | 3 και 4 | LEFT_EN προς R_EN και L_EN |
| 4 | 7 | RIGHT_RPWM |
| 5 | 8 | RIGHT_LPWM |
| 6 | 9 και 10 | RIGHT_EN προς R_EN και L_EN |
| 7 | 15 | LF_ENCODER_A |
| 8 | 16 | LF_ENCODER_B |
| 9 | 19 | LR_ENCODER_A |
| 10 | 20 | LR_ENCODER_B |
| 11 | 23 | RF_ENCODER_A |
| 12 | 24 | RF_ENCODER_B |
| 13 | 27 | RR_ENCODER_A |
| 14 | 28 | RR_ENCODER_B |

### Intake driver και intake encoders

| Από J_MEGA | Προς J_INTAKE | Net |
|---:|---:|---|
| 15 | 1 | INTAKE_LEFT_PWM / ENA |
| 16 | 2 | INTAKE_LEFT_IN1 |
| 17 | 3 | INTAKE_LEFT_IN2 |
| 18 | 4 | INTAKE_RIGHT_PWM / ENB |
| 19 | 5 | INTAKE_RIGHT_IN3 |
| 20 | 6 | INTAKE_RIGHT_IN4 |
| 21 | 11 | INTAKE_LEFT_ENCODER_A |
| 22 | 12 | INTAKE_LEFT_ENCODER_B |
| 23 | 15 | INTAKE_RIGHT_ENCODER_A |
| 24 | 16 | INTAKE_RIGHT_ENCODER_B |

Το `J_INTAKE` 7 είναι logic GND του driver και το 8 μένει `KEY-NC`. Η επάνω
κλέμμα `5V Optional` του συγκεκριμένου L298N δεν συνδέεται στην perfboard. Ο
L298N παραμένει μόνο για σύντομα, current-limited bench tests και όχι για stall,
jam ή τελική λειτουργία των FIT0186.

### IR, χειριστήρια και IMU

| Από J_MEGA | Προς | Net |
|---:|---|---|
| 25 | J_IR 5 | IR_ENTRY_SIGNAL |
| 26 | J_IR 6 | IR_EXIT_SIGNAL |
| 27 | J_USER 1 | START_ARM |
| 28 | J_USER 3 | ESTOP_STATUS |
| 29 | R1 470Ω → J_USER 5 | ARMED_LED, αντίσταση σε σειρά |
| 30 | J_IMU 3 | I2C_SDA |
| 31 | J_IMU 4 | I2C_SCL |

`J_USER` pins 2, 4 και 6 πάνε στη ράγα GND. Το J_USER 5 είναι η άνοδος του
εξωτερικού LED **μετά** την R1· η κάθοδος επιστρέφει από το pin 6.

Το παραγγελμένο module είναι το Grobotronics `19-00010173`, GY-521 / MPU6050,
και ο κατασκευαστής του module δίνει τροφοδοσία 3–5V. Επομένως το `J_IMU` pin 1
(`IMU_VCC`) συνδέεται στη ράγα 5V. Τα pins 2/3/4 είναι αντίστοιχα GND/SDA/SCL.

Το header και το pin allocation του gyro περιλαμβάνονται στην πλακέτα. Η
ανάγνωση MPU6050 δεν έχει ακόμη υλοποιηθεί στο σημερινό `motion_mega.ino`.

## 5. Διανομή 5V και GND

### Επιλογέας τροφοδοσίας JP_PWR

| Pin | Net |
|---:|---|
| 1 | J_MEGA 32 / MEGA_5V |
| 2 | αριστερή ράγα LOGIC_5V |
| 3 | PWR_IN 1 / EXT_REG_5V |

`PWR_IN` pin 2 πηγαίνει απευθείας στη δεξιά ράγα GND. Το `J_MEGA` pin 33
πηγαίνει επίσης απευθείας στη GND rail ώστε τα σήματα να έχουν κοινή αναφορά.

- Jumper 1–2: bench τροφοδοσία της rail από το Mega/USB.
- Jumper 2–3: rail από εξωτερικό ρυθμισμένο 5V στο PWR_IN.
- Ποτέ δύο jumpers μαζί.
- Στη θέση 2–3 το εξωτερικό 5V δεν τροφοδοτεί το Mega από το 5V pin.
- Η πολικότητα του PWR_IN ελέγχεται με πολύμετρο πριν συνδεθεί το Mega.

### Pins προς +5V rail

`J_MOTION`: 5, 11, 13, 17, 21, 25

`J_INTAKE`: 9, 13

`J_IR`: 1, 3, 7, 9

`J_IMU`: 1

### Pins προς GND rail

`J_MOTION`: 6, 12, 14, 18, 22, 26

`J_INTAKE`: 7, 10, 14

`J_IR`: 2, 4, 8, 10

`J_USER`: 2, 4, 6

`J_IMU`: 2

`PWR_IN`: 2

`J_MEGA`: 33

## 6. Παθητικά εξαρτήματα

Τα 100 nF είναι κεραμικά και μπαίνουν στην κάτω όψη, με όσο γίνεται μικρότερα
άκρα, απευθείας ανάμεσα στα αντίστοιχα VCC/GND pins:

| Ref | Σύνδεση |
|---|---|
| C1 | J_MOTION 5–6, left BTS logic |
| C2 | J_MOTION 11–12, right BTS logic |
| C3 | J_MOTION 13–14, LF encoder |
| C4 | J_MOTION 17–18, LR encoder |
| C5 | J_MOTION 21–22, RF encoder |
| C6 | J_MOTION 25–26, RR encoder |
| C7 | J_INTAKE 9–10, left intake encoder |
| C8 | J_INTAKE 13–14, right intake encoder |
| C9 | J_IR 3–4, entry receiver |
| C10 | J_IR 9–10, exit receiver |

Επιπλέον:

- `C11`: 220 µF / ≥10 V μεταξύ LOGIC_5V και GND δίπλα στο PWR_IN. Η λωρίδα
  του ηλεκτρολυτικού δείχνει προς GND.
- `R1`: 470 Ω σε σειρά μεταξύ J_MEGA 29 και J_USER 5.
- `R2`: 10 kΩ από J_IR 5 προς LOGIC_5V, εξωτερικό pull-up του entry receiver.
- `R3`: 10 kΩ από J_IR 6 προς LOGIC_5V, εξωτερικό pull-up του exit receiver.
- `C_IMU`: 100 nF στο J_IMU 1–2.

Τα firmware `INPUT_PULLUP` στα D36/D37 μπορούν να παραμείνουν ενεργά μαζί
με τα εξωτερικά 10 kΩ pull-ups.

## 7. Σειρά κόλλησης

1. Χωρίς κανένα module συνδεδεμένο, μαρκάρισε component side, solder side,
   C1/C42, R1/R30 και όλα τα pin 1.
2. Κάνε dry-fit όλων των headers και του 3D label. Μη ζορίσεις IDC σώμα ή βίδες.
3. Κόλλησε μόνο τις δύο ακριανές rails και μέτρησε ότι κάθε rail είναι συνεχής.
4. Επιβεβαίωσε ότι 5V rail προς GND rail είναι ανοικτό κύκλωμα.
5. Κόλλησε headers πρώτα από ένα ακριανό pin, έλεγξε καθετότητα και μετά τα
   υπόλοιπα pins.
6. Κόλλησε C1–C11 και R1–R3.
7. Πέρασε ένα group μονωμένων signal wires κάθε φορά και κάνε continuity test
   αμέσως μετά από κάθε group.
8. Καθάρισε flux και έλεγξε με μεγεθυντικό φακό για bridges/ψυχρές κολλήσεις.

## 8. Έλεγχος πριν δοθεί τάση

- Καμία συνέχεια μεταξύ LOGIC_5V και GND.
- JP_PWR αφαιρεμένο κατά τον πρώτο continuity έλεγχο.
- Κάθε signal pin έχει συνέχεια μόνο προς τον προορισμό του.
- Τα KEY-NC pins J_MEGA 34 και J_INTAKE 8 δεν έχουν συνέχεια πουθενά.
- D30 έχει συνέχεια και προς J_MOTION 3 και προς 4, χωρίς επαφή με 5V/GND.
- D31 έχει συνέχεια και προς J_MOTION 9 και προς 10, χωρίς επαφή με 5V/GND.
- J_USER 5 φτάνει στο D34 μόνο μέσα από περίπου 470 Ω.
- Οι D36/D37 μετρούν περίπου 10 kΩ προς 5V όταν το Mega είναι αποσυνδεδεμένο.
- Ο ηλεκτρολυτικός C11 έχει σωστή πολικότητα.
- Στο bench mode 1–2, τροφοδοτούμε πρώτα μόνο από USB και ελέγχουμε 5V/GND.
- Στο external mode 2–3, ελέγχουμε πρώτα το PWR_IN χωρίς Mega ή αισθητήρες.
- Δεν υπάρχει πουθενά 12 V πάνω στην perfboard.

Το `motion_mega.ino` ελέγχει σήμερα μόνο το drive motion. Τα intake pins είναι
δεσμευμένα και ελεγμένα ως προς συγκρούσεις, αλλά το κοινό motion+intake
firmware είναι ξεχωριστό επόμενο βήμα.
