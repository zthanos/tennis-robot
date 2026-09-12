# Δοκιμαστική καλωδίωση intake — Mega, L298N και 2× FIT0186

Η καλωδίωση αυτή προορίζεται για bench bring-up των δύο intake motors
`FIT0186 / GB37Y3530-12V-251R`. Δεν αποτελεί έγκριση του L298N για το τελικό
λειτουργικό intake: το breakout δηλώνεται για έως 2 A ανά κανάλι, ενώ κάθε
μοτέρ έχει stall current 7 A.

## Τροφοδοσία

```text
12 V bench supply (+), με ενεργό περιορισμό ρεύματος
  -> L298N +12V / VMS

12 V bench supply (-)
  -> L298N GND
  -> Arduino Mega GND (κοινή γείωση)

Arduino Mega 5V
  -> κίτρινο καλώδιο encoder και των δύο FIT0186
  -> τροφοδοσία των δύο IR break beams

Arduino Mega GND
  -> λευκό καλώδιο encoder και των δύο FIT0186

L298N επάνω κλέμμα "5 Volts (Optional)"
  -> ΚΑΜΙΑ ΣΥΝΔΕΣΗ όσο το Mega τροφοδοτείται από USB
```

Στη συγκεκριμένη κόκκινη πλακέτα, άφησε το κάτω αριστερό `5 Volt Select`
jumper τοποθετημένο: ο onboard regulator τροφοδοτεί το logic του L298N από το
`VMS`. Το Mega τροφοδοτείται αρχικά μόνο από USB και συνδέεται με τον driver
μόνο μέσω των control signals και του κοινού `GND`. Η επάνω κλέμμα `5 Volts
(Optional)` μένει τελείως ασύνδετη, ώστε να μην ενωθούν μεταξύ τους η έξοδος
του regulator και το USB 5 V του Mega.

Με τις επάνω κλέμμες προς τα πάνω, η σειρά από αριστερά προς τα δεξιά είναι:

```text
Motor A OUT1/OUT2 | VMS +12V | GND | 5V OPTIONAL (ασύνδετο) | Motor B OUT3/OUT4
```

Έλεγξε και τα τυπωμένα labels της πλακέτας πριν την τροφοδοσία· τα χρώματα
των κλεμμών δεν αποτελούν ασφαλή αναγνώριση ακροδέκτη. Μην εφαρμόσεις ποτέ
12 V στην κλέμμα `5V`.

## Pin map

| Λειτουργία | Arduino Mega | L298N / μοτέρ |
|---|---:|---|
| PWM αριστερού μοτέρ | D44 | ENA, με αφαιρεμένο το ENA jumper |
| Φορά αριστερού 1 | D40 | IN1 |
| Φορά αριστερού 2 | D41 | IN2 |
| Αριστερό μοτέρ | — | OUT1/OUT2 → κόκκινο/μαύρο motor leads |
| PWM δεξιού μοτέρ | D45 | ENB, με αφαιρεμένο το ENB jumper |
| Φορά δεξιού 1 | D42 | IN3 |
| Φορά δεξιού 2 | D43 | IN4 |
| Δεξί μοτέρ | — | OUT3/OUT4 → κόκκινο/μαύρο motor leads |
| Encoder αριστερού A | A8 / D62 | μπλε |
| Encoder αριστερού B | A9 / D63 | πράσινο |
| Encoder δεξιού A | A10 / D64 | μπλε |
| Encoder δεξιού B | A11 / D65 | πράσινο |
| Encoder VCC και των δύο | 5V | κίτρινο |
| Encoder GND και των δύο | GND | λευκό |
| IR entry signal | D36 | receiver signal, active LOW |
| IR exit signal | D37 | receiver signal, active LOW |
| IR VCC/GND | 5V/GND | κοινή τροφοδοσία αισθητήρων |

Για κάθε IR break beam: receiver κόκκινο → `5V`, μαύρο → `GND` και
λευκό/κίτρινο signal → `D36` ή `D37`. Το emitter τροφοδοτείται από `5V/GND`.
Το `INPUT_PULLUP` του Mega δίνει `LOW` όταν η δέσμη κόβεται και `HIGH` όταν
είναι ελεύθερη.

Τα A8-A11 είναι pin-change-interrupt pins του ATmega2560 και απαιτούν αντίστοιχο
PCINT ISR/library στο ενιαίο firmware. Δεν λειτουργούν με απλό
`attachInterrupt()`. Η επιλογή τους κρατά ελεύθερα τα D20/D21 για I2C/MPU6050
και δεν συγκρούεται με τα τέσσερα drive encoders.

## Έλεγχος έναντι motion pin map

Το αρχικό προσωρινό intake map D5/D6, D22-D25 και D2/D3/D18/D19 απορρίφθηκε,
επειδή τα pins χρησιμοποιούνται ήδη από τους δύο BTS7960 και τους τέσσερις
drive encoders. Η δεσμευμένη περιοχή intake είναι πλέον:

```text
L298N control: D40-D45
Intake encoders: A8-A11 (D62-D65, PCINT)
```

Τα D46, A12-A15 και τα SPI pins D50-D53 παραμένουν αδέσμευτα για μελλοντική
επέκταση. Το υπάρχον `motion_mega.ino` δεν ελέγχει ακόμη το intake· το pin map
είναι reservation για την επόμενη ενοποίηση firmware.

## Mega bench sketch

Το dedicated sketch βρίσκεται στο:

```text
arduino/collector/07_dual_intake_mega_bench/07_dual_intake_mega_bench.ino
```

Χρησιμοποιεί `115200 baud` και `Newline`. Ξεκινά πάντα `DISARMED`.

```text
ARM
PULSE L 45 250
PULSE R 45 250
RUN 60 1000
AUTO 60 4000
STOP
DISARM
STATUS
ZERO
DIST <μετρημένη απόσταση δεσμών σε mm>
```

Το `AUTO 60 4000` περιμένει το entry beam. Μόλις μπει μπαλάκι, ενεργοποιεί και
τους δύο τροχούς προς τα μέσα, καταγράφει τον χρόνο μέχρι το exit beam και
σταματά μόλις κοπεί το exit beam ή στα 4000 ms. Τα χειροκίνητα `RUN`/`PULSE`
παραμένουν περιορισμένα στα 1500 ms. Ο encoder jam έλεγχος παραμένει ενεργός.

Στο Raspberry Pi, αφού ανέβει στο Mega η τρέχουσα έκδοση του sketch, ο
επαναλαμβανόμενος αυτόματος κύκλος ξεκινά με:

```bash
python3 scripts/run_intake_ir_cycle.py --port /dev/ttyACM0 --pwm 60
```

Το script δεν χειρίζεται απευθείας GPIO. Οπλίζει το Mega μέσω USB serial και το
Mega κρατά το τοπικό όριο των 4 s, ώστε το stop να μην εξαρτάται από latency του
Pi. Με `--once` εκτελείται ένας μόνο κύκλος.

Η έξοδος `DATA` είναι CSV-compatible:

```text
DATA,ms,state,left_pwm,right_pwm,left_count,right_count,left_rpm,right_rpm,entry,exit
```

Τα events `BALL_ENTRY`, `BALL_EXIT_REACHED` και `BALL_CYCLE_COMPLETE` δίνουν
τον πραγματικό χρόνο διέλευσης. Η μέτρηση με L298N χαρακτηρίζει μόνο το
προσωρινό, driver-limited bench setup· δεν είναι ακόμη η τελική επίδοση του
μηχανισμού.

Τοποθέτησε το entry beam λίγο πριν από το σημείο επαφής των τροχών και το exit
beam αμέσως μετά το nip, προς το basket. Μέτρησε την απόσταση ανάμεσα στα δύο
επίπεδα των δεσμών και δώσε την πριν από το `AUTO`, π.χ. `DIST 180` μόνο αν η
πραγματική μετρημένη απόσταση είναι 180 mm. Το event εξόδου έχει μορφή:

```text
BALL_EXIT_REACHED,ms_since_boot,transit_ms,average_speed_m_s
```

Χωρίς εντολή `DIST`, η ταχύτητα αναφέρεται ως `NA` ώστε να μη χρησιμοποιηθεί
υποθετική διάσταση.

## Σειρά ασφαλούς bring-up

1. Αφαίρεσε τροχούς και μπαλάκι από το test.
2. Βγάλε μόνο τα `ENA` και `ENB` jumpers, ώστε τα D44/D45 να δίνουν PWM.
   Άφησε το κάτω αριστερό `5 Volt Select` και τα δεξιά `CSA`, `CSB`,
   `U1`-`U4` jumpers τοποθετημένα.
3. Κάνε πρώτα continuity check των GND χωρίς 12 V.
4. Τροφοδότησε το Mega από USB. Επιβεβαίωσε ότι η επάνω κλέμμα `5V` του
   L298N δεν έχει κανένα καλώδιο και ότι δεν υπάρχει βραχυκύκλωμα VMS-GND.
5. Σύνδεσε 12 V από τροφοδοτικό με ενεργό current limit και δοκίμασε ένα μοτέρ
   κάθε φορά με σύντομο, χαμηλό PWM pulse.
6. Μετά δοκίμασε και τα δύο χωρίς μηχανικό φορτίο. Σταμάτα αν ο L298N ή η
   ψύκτρα θερμαίνεται γρήγορα, αν πέφτει η τάση ή αν γίνεται reset το Mega.
7. Επιβεβαίωσε από τους encoders ότι οι δύο πλευρές αναφέρουν σωστό πρόσημο.

Για να τραβούν το μπαλάκι προς το εσωτερικό, τα δύο μοτέρ θα πάρουν αντίθετες
εντολές φοράς στο software. Αν μία πλευρά είναι ανάποδη, αλλάζουμε τη λογική
IN1/IN2 ή IN3/IN4· δεν αλλάζουμε τα καλώδια του encoder για διόρθωση φοράς.

## Όριο χρήσης

Με L298N επιτρέπεται μόνο σύντομη, επιτηρούμενη δοκιμή κανονικής διέλευσης μίας
μπάλας, αφού περάσουν τα wheel-off και free-wheel tests. Δεν εκτελούμε σκόπιμο
jam/stall, παρατεταμένη πίεση μπάλας ή endurance test. Πριν από τέτοια δοκιμή
επιλέγεται driver με κατάλληλο συνεχές/peak ρεύμα, current limiting και θερμική
προστασία.
