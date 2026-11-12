# Tennis robot — Ηλεκτρονικά: κατάσταση δοκιμών (bench bring-up, χωρίς μοτέρ)

Τελευταία ενημέρωση: 2026-11-12 · branch `codex/shared-flywheel-intake-launcher`.
Ενημερώνεται μετά από κάθε συνεδρία πάγκου. Διαδικασία σταδίων 0/A/B/Γ:
[bench-bringup-no-motors-el.md](bench-bringup-no-motors-el.md). Καλωδίωση:
[drive-test-wiring-el.md](drive-test-wiring-el.md).

## Υλικό & πρόσβαση

- Arduino Mega 2560, firmware `arduino/motion/drive_test_mega/drive_test_mega.ino`,
  σειριακή 500000 baud.
- Raspberry Pi 5 `tennisserver`, user `thanos`. Wi-Fi AP «TennisRobot» → Pi στο
  10.42.0.1 (κωδικός στο Pi: `~/field_wifi_credentials.txt`). SSH από PC:
  `ssh -o IdentityAgent=none -i ~/.ssh/id_pi thanos@10.42.0.1`. Με Ethernet:
  192.168.31.x (DHCP, αλλάζει).
- Mega στο Pi: `/dev/ttyACM0`. Το repo στο Pi **δεν** κάνει `git pull` — τα αρχεία
  πάνε με `rsync` από το PC.
- Upload firmware (στο Pi, αφού σταματήσει όποιο πρόγραμμα κρατά τη θύρα):

  ```bash
  arduino-cli compile --fqbn arduino:avr:mega arduino/motion/drive_test_mega
  arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino/motion/drive_test_mega
  ```

- Τροφοδοτικό Pi: δεν δίνει 5 A → περιορισμός περιφερειακών· πριν από OAK-D/LiDAR
  χρειάζεται τροφοδοτικό 27 W.

## Pinout (Mega)

| Λειτουργία | Pin | Σύνδεση |
|---|---|---|
| START (διακόπτης NO: μπιπ μόνο όταν πατιέται) | D32 | D32 ↔ GND, εσωτερικό pull-up, debounce 50 ms |
| E-stop (επαφή NC: μπιπ όταν είναι ελεύθερο) | D33 | D33 ↔ GND· ανοιχτό = ESTOPPED |
| LED ARMED (μέσα στο START) | D34 | D34 → 470 Ω → LED+ · LED− → GND· ON μόνο σε ARMED |
| BTS αριστερά (LF+LR) | RPWM D5, LPWM D6, EN D30 | R_EN+L_EN ενωμένα, 10 kΩ pull-down στο καλώδιο |
| BTS δεξιά (RF+RR) | RPWM D9, LPWM D10, EN D31 | ίδιο |
| BTS #3 (tilt FIT0186) | D44/D45, EN D40 | — |
| BTS #4 (εφεδρικό) | D46/D11, EN D42 | — |
| Encoders A/B | LF D2/D22, LR D3/D23, RF D18/D24, RR D19/D25 | — |
| MPU6050 | I²C D20/D21, 0x68 | — |
| AS5600 (tilt) | I²C, 0x36 | πλάνο: αφαίρεση R1, DIR→GND |

- BTS: IS ασύνδετα, VCC 5 V, κοινό GND, B+ με ασφάλεια 20 A.
- Encoders: σύνδεση **κατά θέση, όχι χρώμα**: 1 M+, 2 M−, 3 GND, 4 5 V, 5 A, 6 B.
  Από πίσω, με τα καλώδια κάτω, η θέση 1 είναι αριστερά. Έλεγχος με 5 V / όριο
  30 mA πριν από την πρώτη τροφοδοσία. Τα εμπρός καλώδια περνούν **κάτω** από τη
  ράγα (εκεί πατά το πέλμα της αψίδας).
- E-stop YJ139-LAY37: το NC μπλοκ είναι τα δύο μαύρα. Έχει LED 12 V (X1/X2) —
  **ποτέ** στο Mega ή στα 5 V.

## Λογική firmware

- Καταστάσεις DISARMED / ARMED / ESTOPPED / FAULTED. Τηλεμετρία `T,...` με 24
  πεδία (σταθερή για `run_drive_test_pi.py`).
- Όπλιση: START (τοπικά, `OK,ARMED,BUTTON`) ή host `ARM`. Απαιτεί E-stop κλειστό
  **και** CPR > 0 (`CFG CPR x`). START πατημένο κατά την εκκίνηση αγνοείται.
- Host heartbeat (PING/CMD) κάθε < 300 ms όσο είναι ARMED, αλλιώς
  `FAULT,HOST_TIMEOUT`.

## Εργαλεία

- Σελίδα ελέγχου: στο Pi `cd ~/tennis-robot && python3 scripts/mega_buttons_panel.py`
  → `http://10.42.0.1:8083`. Στέλνει PING μόνο όσο είναι ανοιχτή σε browser·
  ορίζει `CFG CPR 1000` + `CFG PWM 0`, άρα οι έξοδοι μένουν μηδέν.
- CLI: `python3 scripts/bench_check_mega.py monitor --cpr 1` και
  `pwm --cpr X --pwm-ceiling ≤0.3`.

## Αποτελέσματα

| Ημ/νία | Έλεγχος | Αποτέλεσμα |
|---|---|---|
| 2026-11-09 | USB / ράγα 5 V | ✅ 4,9 V |
| 2026-11-11 | E-stop D33 | ✅ OK ελεύθερο / OPEN πατημένο· αρχική αποτυχία = Dupont χωρίς επαφή, το NO ζευγάρι απορρίφθηκε |
| 2026-11-11 | START D32 | ✅ `EVENT,START_PRESSED` → `OK,ARMED,BUTTON`· E-stop σε ARMED → ESTOPPED |
| 2026-11-11 | Firmware CMD/CFG/LIMIT | ✅ διορθώθηκε (a342ce1c, ανεβασμένο): το avr-libc `sscanf("%f")` δεν υποστηρίζεται, όλες έδιναν ERR,UNKNOWN· τώρα `parseFloats` με strtod. Επαληθεύτηκαν CFG CPR/PWM/ACCEL, LIMIT, CMD |
| 2026-11-11 | Scripts πάγκου | ✅ διορθώθηκαν: κολλούσαν στην πρώτη εντολή (μη διαβασμένη τηλεμετρία γέμιζε το 16U2) → `reset_input_buffer` + `write_timeout` |

## Ανοιχτά (σειρά προτεραιότητας)

1. **LED START (D34)** δεν ανάβει σε ARMED. Πριν άναβε αμυδρά από το pull-up του D32
   (λάθος καλωδίωση). Μέτρα D34 ≈ 5 V σε ARMED, μετά πολικότητα/κολλήσεις/αντίσταση.
2. **Κρίσιμο πριν από το drive test:** στα 500000 baud τα PING φτάνουν αλλοιωμένα
   (ERR,UNKNOWN, «PIN» μέσα σε γραμμές T) → τυχαίο `FAULT,HOST_TIMEOUT` μετά από
   λίγα δευτερόλεπτα ARMED. Υποψία: baud πολύ υψηλό ενώ το Mega στέλνει τηλεμετρία
   (~11,6 ms ανά γραμμή). Λύση υπό εξέταση: 250000/115200 ή λιγότερη τηλεμετρία
   (αλλαγή firmware και host scripts μαζί).
3. Encoders ×4 (μέτρηση CPR με το χέρι), MPU6050 (σήμερα `IMU=MISSING`), AS5600 (δοκιμή R1).
4. Στάδιο B (λογική BTS) και Γ (BTS στα 12 V / 0,5 A χωρίς μοτέρ), μετά πρώτο
   drive test με PSU.

## Κανόνες

- Αλλαγές firmware μόνο με έγκριση του χρήστη· commit/push μετά από επαλήθευση.
- Χωρίς μοτέρ: PWM ceiling ≤ 0,3. Καλώδια αλλάζουν μόνο με το USB βγαλμένο.
