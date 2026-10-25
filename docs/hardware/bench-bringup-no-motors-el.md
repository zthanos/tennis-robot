# Bring-up πάγκου χωρίς μοτέρ — συνδέσεις και δοκιμές από το Pi

Σκοπός: να ελεγχθούν **όλες οι συνδέσεις του Mega** (κουμπιά, φωτισμός,
E-stop, encoders, MPU6050, BTS7960) από το Pi **πριν συνδεθεί οποιοδήποτε
μοτέρ στα BTS**. Συμπληρώνει τα:

- `docs/hardware/drive-test-wiring-el.md` — πλήρης πίνακας pins/χρωμάτων και
  κανόνες ισχύος (πηγή αλήθειας για την καλωδίωση),
- `docs/mechanism/physical-drive-test-protocol-el.md` — η επόμενη φάση
  (μοτέρ, τροχοί στον αέρα, μετά πάτωμα).

Firmware: `arduino/motion/drive_test_mega/drive_test_mega.ino` (500000 baud).
Βοηθητικό script στο Pi: `scripts/bench_check_mega.py`. Χρειάζεται επειδή
το firmware κάνει `FAULT,HOST_TIMEOUT` αν περάσουν 300 ms χωρίς μήνυμα, άρα
ένα απλό serial terminal δεν μπορεί να το κρατήσει ARMED.

> **Γενικός κανόνας:** σε όλα τα στάδια A–Γ **κανένα μοτέρ δεν είναι
> συνδεδεμένο** στις εξόδους M+/M− των BTS. Τα encoders των FIT0403 μπορούν
> να συνδεθούν (παίρνουν 5 V από το Mega) και να γυρίζουν με το χέρι.

## Τι χρειάζεται

- Pi με το repo, `arduino-cli` (core `arduino:avr`) και `pyserial`.
- Mega στο USB του Pi (`/dev/ttyACM0`).
- Πολύμετρο. Τροφοδοτικό πάγκου 12 V με όριο ρεύματος (στάδιο Γ).
- Το πάνελ με E-stop, toggle, START/ARM, ή τα κουμπιά χαλαρά στον πάγκο.

## Πίνακας συνδέσεων που ελέγχονται εδώ

| Λειτουργία | Mega pin | Σύνδεση | Πώς ελέγχεται |
|---|---|---|---|
| E-stop (NC) | D33 | NC επαφή μεταξύ D33 και GND | στάδιο A2: `ESTOP=OK/OPEN` στο monitor |
| START/ARM (στιγμιαίο, OFF-(ON) SPST-NO) | D32 | επαφή μεταξύ D32 και GND, χωρίς αντίσταση (εσωτερικό pull-up) | στάδιο A6: όπλιση από το κουμπί |
| Φωτισμός START/ARM = ARMED LED | D34 | D34 → **330 Ω** → LED(+) → LED(−) → GND. R16-504BD: σκέτο LED ~2,2 V (προδιαγραφή πωλητή, 220–1000 Ω)· 330 Ω ≈ 8,5 mA, χωρίς transistor | στάδιο A5: ανάβει με ARM |
| Φωτισμός toggle | — | από το switched 12 V του toggle | στάδιο Γ1 |
| LED κατάστασης Ø8 στο πάνελ | — | **δεν έχει pin ακόμα** | εκτός bring-up |
| Encoders A/B | LF D2/D22, LR D3/D23, RF D18/D24, RR D19/D25 | κίτρινο=A, λευκό=B, πράσινο=5 V, μπλε=GND | στάδιο A3 |
| MPU6050 | D20 SDA, D21 SCL, 5 V, GND | GY-521 με regulator | στάδιο A4 |
| BTS αριστερό | RPWM D5, LPWM D6, R_EN+L_EN D30 | logic VCC/GND κατά την ετικέτα του module | στάδιο B |
| BTS δεξί | RPWM D9, LPWM D10, R_EN+L_EN D31 | όπως πάνω | στάδιο B |

> **Πώς δουλεύει το START (D32), απόφαση 2026-10-23:** η όπλιση γίνεται
> **τοπικά στο Mega**. Με ένα καθαρό πάτημα (debounce 50 ms) το Mega οπλίζει
> μόνο του, με τους ίδιους ελέγχους με το `ARM` από το Pi (E-stop κλειστό,
> CPR ρυθμισμένο), ανάβει το LED και στέλνει στο Pi `EVENT,START_PRESSED` και
> `OK,ARMED,BUTTON`. **Δεν κινεί τίποτα**: η κίνηση ξεκινά μόνο όταν το Pi
> στείλει εντολές. Αν το Pi δεν στέλνει τίποτα, σε 0,3 s γίνεται
> `FAULT,HOST_TIMEOUT` και αφοπλίζεται. Κουμπί πατημένο κατά την εκκίνηση
> δεν μετρά ως πάτημα.

## Στάδιο 0 — έλεγχοι χωρίς ρεύμα (πολύμετρο)

1. **E-stop:** συνέχεια μεταξύ των δύο καλωδίων προς D33/GND όταν **δεν** είναι
   πατημένο, ανοιχτό όταν πατηθεί. Αν είναι ανάποδα, είσαι στην NO επαφή:
   άλλαξε στην NC.
2. **START/ARM:** βρες ποιοι δύο ακροδέκτες είναι η **επαφή** (συνέχεια μόνο
   όσο το πατάς) και ποιοι είναι το **LED** (σε δίοδο, η μία φορά δίνει
   ~1,8–3 V). Σημείωσε αν το LED έχει ενσωματωμένη αντίσταση: στη θέση
   διόδου, ένα σκέτο LED δείχνει ~1,8–3 V, ενώ ένα LED με ενσωματωμένη
   αντίσταση 12 V συνήθως δεν δείχνει καθαρή τιμή. Στη δεύτερη περίπτωση
   **δεν** συνδέεται απευθείας στο D34: χρειάζεται transistor, βλ.
   `drive-test-wiring-el.md` §3.
3. **Toggle:** βρες IN / switched OUT / LED GND (ετικέτα ή συνέχεια). Το LED του
   ανάβει από το switched OUT και δεν συνδέεται στο Mega.
4. Κανένα 12 V σε 5 V ή σε pin του Mega. Κοινό GND Mega–BTS–PSU.

## Στάδιο A — μόνο USB (το Mega παίρνει ρεύμα από το Pi, τα BTS δεν συνδέονται)

### A1. Ανέβασμα firmware (μόνο όταν το αποφασίσεις)

```bash
cd ~/tennis-robot   # το checkout του repo στο Pi
arduino-cli compile --fqbn arduino:avr:mega arduino/motion/drive_test_mega
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino/motion/drive_test_mega
```

### A2. E-stop

```bash
python3 scripts/bench_check_mega.py monitor
```

Αναμενόμενο:

- στην αρχή: `READY,DRIVE_TEST_MEGA,CPR_REQUIRED,ESTOP=OK,IMU=OK`,
- E-stop απάτητο → `DISARMED ESTOP=OK`,
- πάτημα E-stop → `ESTOP,OPEN_CIRCUIT` και `ESTOPPED ESTOP=OPEN`,
- αποσύνδεση του βύσματος του E-stop → επίσης `ESTOPPED` (fail-safe).

Μετά το ξεκλείδωμα του E-stop, το firmware μένει `ESTOPPED` μέχρι το επόμενο
`ARM`. Αυτό είναι σωστό: δεν ξαναοπλίζει μόνο του.

### A3. Encoders (γύρισμα με το χέρι)

```bash
python3 scripts/bench_check_mega.py monitor --zero
```

- Γύρνα κάθε τροχό/άξονα με το χέρι: πρέπει να αλλάζει **μόνο** ο δικός του
  μετρητής (LF/LR/RF/RR). Αν αλλάζει άλλος, είναι μπερδεμένα καλώδια.
- Γύρνα **μπροστά**: πρόσημο + και στις δύο πλευρές. Σημείωσε όποιον
  βγαίνει αρνητικός. Διορθώνεται με αντιμετάθεση A/B στο βύσμα ή αργότερα στο
  firmware, που θέλει έγκριση.
- **Μέτρηση CPR:** κάνε ένα σημάδι στον τροχό, `--zero`, γύρνα ακριβώς
  **10 περιστροφές**. CPR = counts / 10. Κράτα την τιμή για κάθε τροχό· θα
  τη δώσεις ως `--cpr` στο στάδιο B και ως `--counts-per-rev` στο drive test.

### A4. MPU6050

Με το `monitor --zero` τρέξει, άφησε το chassis ακίνητο: `gz` ≈ 0 (±0,02).
Στρίψε το chassis αριστερά με το χέρι: `gz` θετικό και το `yaw` αυξάνεται.
Αν βγει `IMU=MISSING` στο READY, έλεγξε SDA/SCL και 5 V.

### A5. ARMED LED (φωτισμός START/ARM) — οπλισμός με μηδενική έξοδο

```bash
python3 scripts/bench_check_mega.py pwm --cpr <CPR> --pwm-ceiling 0 --seconds 5
```

Με `--pwm-ceiling 0` το firmware οπλίζει αλλά δεν δίνει καθόλου PWM.

- Κατά τα 5 s: `ARMED` και **ανάβει το LED του START** (D34).
- Μετά: `STOP + DISARM` → το LED σβήνει.
- Ξανατρέξε το και πάτα το E-stop στη μέση: `ESTOP,OPEN_CIRCUIT`, το LED
  σβήνει αμέσως.
- Αν πατήσεις Ctrl+C ή βγάλεις το USB, το firmware κάνει `FAULT,HOST_TIMEOUT`
  σε 0,3 s: το LED σβήνει.

### A6. START (D32) — όπλιση από το κουμπί

1. Ηλεκτρικά: πολύμετρο μεταξύ D32 και GND: ~5 V (pull-up) όταν δεν
   πατιέται, ~0 V όσο πατιέται.
2. Λειτουργικά (PWM 0, καμία έξοδος):

   ```bash
   python3 scripts/bench_check_mega.py monitor --cpr <CPR>
   ```

   - Πάτα START: `EVENT,START_PRESSED`, `OK,ARMED,BUTTON`, κατάσταση `ARMED`,
     **ανάβει το LED** του START.
   - Ξαναπάτα START: μόνο `EVENT,START_PRESSED`· δεν αλλάζει κάτι.
   - Πάτα E-stop: `ESTOP,OPEN_CIRCUIT`, το LED σβήνει. Ξεκλείδωσε το E-stop και
     πάτα START: ξαναοπλίζει.
   - Ctrl+C: το script στέλνει DISARM και το LED σβήνει.
   - Χωρίς `--cpr`, το πάτημα δίνει `ERR,CPR_REQUIRED` (σωστή άρνηση).

## Στάδιο B — BTS logic συνδεδεμένα, B+ χωρίς ρεύμα, χωρίς μοτέρ

Σύνδεσε RPWM/LPWM/EN/VCC/GND των δύο BTS. **Μην** τροφοδοτήσεις B+.

```bash
python3 scripts/bench_check_mega.py pwm --cpr <CPR> --v 0.1 --pwm-ceiling 0.2 --seconds 5
```

Χωρίς μοτέρ οι encoders δεν κινούνται, οπότε ο ελεγκτής ανεβάζει το duty
μέχρι την οροφή 0,2. Αυτό ακριβώς θέλουμε για να το μετρήσουμε.

| Μέτρηση (DC, πολύμετρο) | Αναμενόμενο κατά τα 5 s | Μετά το DISARM |
|---|---|---|
| D30 και D31 (R_EN/L_EN) | ~5 V | 0 V |
| RPWM αριστερού (D5) και δεξιού (D9) | ~1 V (0,2 × 5 V, μέσος όρος PWM) | 0 V |
| LPWM (D6, D10) | 0 V | 0 V |

- `--v -0.1`: τα ~1 V περνούν στα **LPWM**, και τα RPWM πάνε στο 0.
- `--v 0 --w 0.3` (στροφή αριστερά): η αριστερή πλευρά πάει ανάποδα, η δεξιά
  μπροστά. Η κάθε πλευρά δείχνει τάση στο αντίθετο pin από την άλλη.
- E-stop στη μέση της μέτρησης: τα EN πέφτουν στο 0 αμέσως.

## Στάδιο Γ — B+ με ρεύμα, ακόμα χωρίς μοτέρ

1. PSU **12,0 V**, όριο ρεύματος **0,5 A**, Output OFF. Σύνδεσε B+/B− στα BTS
   μέσω της ασφάλειας. Αν ο toggle είναι στη διαδρομή του 12 V, γύρνα τον:
   **το LED του toggle ανάβει** μόνο στη θέση ON (έλεγχος φωτισμού toggle).
2. Output ON. Το PSU πρέπει να δείχνει σχεδόν μηδέν ρεύμα (μόνο ηρεμίας).
3. Τρέξε ξανά το `pwm --v 0.1 --pwm-ceiling 0.2`. Πολύμετρο DC στις εξόδους
   M+/M− κάθε BTS: **~2,4 V** (0,2 × 12 V). Με `--v -0.1` αντιστρέφεται το
   πρόσημο. Μετά το DISARM: 0 V.
4. Το ρεύμα του PSU μένει κοντά στο μηδέν. Αν ανέβει, κάτι είναι βραχυκυκλωμένο:
   Output OFF και έλεγχος.

Αν όλα τα παραπάνω περάσουν, συνεχίζεις στο `drive-test-wiring-el.md` §4
βήμα 8 (μοτέρ, τροχοί στον αέρα, όριο 5 A) και μετά στο
`physical-drive-test-protocol-el.md`.

## Φύλλο αποτελεσμάτων

| Δοκιμή | Αποτέλεσμα | Σημείωση |
|---|---|---|
| Στάδιο 0 E-stop NC | ☐ | |
| Στάδιο 0 START επαφή / LED ταυτοποίηση (τάση LED: ___) | ☐ | |
| Στάδιο 0 toggle IN/OUT/LED | ☐ | |
| A2 E-stop OK / OPEN / αποσύνδεση | ☐ | |
| A3 encoders σωστό κανάλι + πρόσημο (LF ☐ LR ☐ RF ☐ RR ☐) | ☐ | |
| A3 CPR: LF ___ LR ___ RF ___ RR ___ | ☐ | |
| A4 IMU gz ≈ 0 ακίνητο, + αριστερά | ☐ | |
| A5 ARMED LED ανάβει / σβήνει με DISARM, E-stop, timeout | ☐ | |
| A6 START D32 5 V ↔ 0 V· όπλιση από κουμπί, LED, ξαναόπλιση μετά το E-stop | ☐ | |
| B EN, RPWM/LPWM ανά φορά και πλευρά | ☐ | |
| Γ1 LED toggle στη θέση ON | ☐ | |
| Γ3 M+/M− ±2,4 V, 0 V μετά το DISARM, ρεύμα PSU ≈ 0 | ☐ | |
