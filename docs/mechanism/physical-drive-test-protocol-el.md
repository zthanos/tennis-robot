# P2-62 — πρωτόκολλο φυσικού τεστ κίνησης

Κατάσταση: **προδηλωμένο / χωρίς φυσικές μετρήσεις**. Οι αριθμητικές προβλέψεις
παγώνουν στο `drive_predictions.json`. Δεν επιτρέπεται αλλαγή τους αφού αρχίσει
η μέτρηση· τυχόν νέα fit γράφεται σε άλλο αρχείο και το holdout μένει ανέγγιχτο.

## 1. Διαμόρφωση και όρια

- Τροφοδοσία μοτέρ μόνο από τροφοδοτικό πάγκου **12,0 V**, μέγιστο διαθέσιμο
  ρεύμα: **____ A**. Mega από USB, χωριστά από τα μοτέρ, με κοινό GND.
- Τυπωμένο chassis v3, 4× FIT0403, 4× Baja 170×80, track 0,585 m.
- Πλάκα 5,448 kg με κέντρο X360 και όλες οι dummy μάζες μπαταριών ακριβώς στις
  θέσεις του `stl-v3/README.md`. Δεν χρησιμοποιείται «μπροστινή μπαταρία» αντί
  της πλάκας.
- Ακτίνα υπό φορτίο άγνωστη: οι παγωμένες προβλέψεις καλύπτουν 82/84/85 mm.
- BTS7960 + Mega με wheel-speed PI και προαιρετικό gyro yaw outer loop. Ο
  φυσικός inner controller είναι αναγκαστικά **voltage/PWM-domain**: no-load
  speed feed-forward + PI duty. Ο 5a controller είναι torque-domain. Επομένως
  steady endpoints/slip συγκρίνονται ποσοτικά, αλλά οι μεταβατικές αποκρίσεις
  (rise/brake/current peaks) συγκρίνονται μόνο ποιοτικά ώσπου να βαθμονομηθεί
  το φυσικό motor/voltage model.
- Πρώτα λείο πάτωμα σπιτιού. Χώμα μόνο ως μεταγενέστερο, προαιρετικό dataset.

Τα 18 V/Parkside δεν ανήκουν στο P2-62. Σε μεταγενέστερη δοκιμή οι 12 V
FIT0403 χρειάζονται firmware PWM ceiling `12/18 = 0,667` (και ramp/current
protection). Δεν αλλάζει firmware τώρα.

## 2. Προέλεγχος και σειρά ασφάλειας

1. Στήριξε το όχημα με τους τροχούς στον αέρα, έλεγξε πολικότητα, NC E-stop,
   κοινό GND, φορά τεσσάρων encoders και ότι STOP μηδενίζει PWM.
2. Συμπλήρωσε το πραγματικό `____ A`. Ξεκίνα current limit 5 A μόνο για
   no-load/πολύ αργό bring-up. Μην το θεωρήσεις validation run.
3. Κάνε rollout 10 περιστροφών ανά τροχό υπό το τελικό φορτίο. Μέτρα απόσταση,
   counts/rev και ακτίνα. Διάλεξε 82/84/85 mm μόνο αφού γίνει αυτή η μέτρηση.
4. Το πλήρες frozen run απαιτεί limit **28 A** (τέσσερα stall-label 7 A) και
   τροφοδοτικό που το αντέχει. Ρύθμιση: `min(28 A, ονομαστικό μέγιστο PSU)`.
   Αν το όριο χτυπήσει, σημείωσε `CURRENT_LIMITED`; το run είναι ασφαλές
   bring-up αλλά όχι ποσοτική επικύρωση του 5a.

Προβλεπόμενο συνολικό ρεύμα 12 V (άθροισμα τεσσάρων μοτέρ, sweep μ=0,5…0,9):

| Σενάριο | Μέσο A | Peak A |
|---|---:|---:|
| ευθεία 0,3 m/s | 0,88 | 19,25…20,76 |
| ευθεία / accel 0,6 m/s | 1,61 | 28,00 |
| spin open-loop 0,5 rad/s | 9,11…15,14 | 10,45…15,99 |
| spin με gyro | 9,46…16,07 | 26,80…28,00 |
| τόξο με gyro | 9,22…16,04 | 18,37…19,76 |
| φρενάρισμα από 0,6 | 0,86…0,91 | 28,00 |

Αυτά είναι `Σ|torque command|/(3,73/7)` από το 5a, όχι ηλεκτρικό μοντέλο
BTS7960: δεν περιλαμβάνουν no-load current, losses ή supply transients. Αν το
PSU είναι κάτω από 28 A, κάνε πρώτα straight 0,3 με ήπιο ramp. Με διαθέσιμα
τουλάχιστον 21 A μπορούν να γίνουν straight 0,3 και τόξα χωρίς προβλεπόμενο
clipping· open-loop spin απαιτεί έως 16 A αλλά ξεκινά μόνο μετά την ευθεία.
Τα 0,6, gyro spins και full braking περιμένουν 28 A. Αν ούτε τα 21 A είναι
διαθέσιμα, χρησιμοποίησε μειωμένο acceleration και βάλε το έρμα σε φάσεις
μόνο για λειτουργικό bring-up· δεν βαθμονομείς ούτε αποδέχεσαι το μοντέλο από
αυτά τα runs.

## 3. Σενάρια και παγωμένες προβλέψεις

Σημάδεψε άξονα X 3,5 m, εγκάρσιες γραμμές ανά 0,5 m και σημεία αναφοράς στο
πάτωμα. Για στροφές σημείωσε το αρχικό κέντρο chassis και κύκλους R0,5/R1,0.

1. Ευθεία 3 m στα 0,3 και 0,6 m/s. Με R85 προβλέπονται X=2,991 και 2,994…2,995
   m, Y≈0, yaw≈0°. Το radius sweep δίνει περίπου 2,886…2,995 m.
2. Επιτόπια 90/180/360°, target 0,5 rad/s, πρώτα χωρίς και μετά με gyro.
   Στα 90°/R85 open-loop προβλέπονται μόλις 16,2…43,1° και μετατόπιση
   0,079…0,194 m. Με gyro: 78,9…86,0°, 0,348…0,371 m και
   0,438…0,478 rad/s. Η μεγάλη μετάθεση είναι παγωμένη γνωστή πρόβλεψη,
   όχι λόγος να «διορθωθεί» πριν μετρηθεί. Αν η μετατόπιση 0,35…0,37 m
   επιβεβαιωθεί φυσικά, είναι λειτουργικό ρίσκο του collection route: μία
   επιτόπια διόρθωση heading μπορεί να μεταφέρει το intake έξω από τη γραμμή
   προσέγγισης ή προς μπάλα/εμπόδιο, όχι απλώς sim discrepancy.
3. Αριστερά ημικύκλια 180° στα 0,3 m/s, R0,5 και R1,0, με gyro. Για R85 το
   R0,5 προβλέπει yaw 160,3…173,9°, endpoint error 0,054…0,184 m. Όλες οι
   ακριβείς τελικές πόζες και για τα δύο R βρίσκονται στο JSON.
4. Επιτάχυνση και φρενάρισμα 0→0,6→0. Πρόβλεψη 95% speed σε 0,15…0,18 s
   και 0,061…0,068 m, έπειτα stop 0,12…0,14 s και 0,030…0,038 m (R85),
   με peak 28 A.

Η ολίσθηση στο JSON είναι encoder/wheel έναντι πραγματικής κίνησης: ευθεία
≈0, spin 0,20…0,29 περίπου και τόξα εξαρτώμενα από μ. Κάθε run επαναλαμβάνεται
τρεις φορές χωρίς αλλαγή setup. Το `arc_R1p0_180deg_gyro` είναι holdout: δεν
χρησιμοποιείται σε καμία βαθμονόμηση.

## 4. Προδηλωμένα όρια αποδοχής

- Ευθεία: endpoint ±0,15 m, |Y|≤0,05 m, |yaw|≤3°.
- Gyro spin: τελική γωνία ±15%· μετατόπιση κέντρου ≤0,55 m. Open-loop spin
  είναι diagnostic και δεν αποτελεί gate γωνίας.
- Τόξο: endpoint error ≤0,20 m και yaw error ≤15°.
- Φρένο: ≤1,5 s, ≤0,45 m και heading change ≤5°.
- Mean encoder/actual slip ≤0,20. Peak ή current-limit event καταγράφεται,
  αλλά δεν κρύβεται με averaging.
- Τρία repeats: ο μέσος περνά και κανένα repeat δεν υπερβαίνει 1,5× το όριο.

Αν υπάρξει απόκλιση, αλλάζει μία μόνο φυσική παράμετρος: κοινό scale απόστασης
στην ευθεία → loaded radius/counts-per-rev· spin/arc slip και yaw rate → μ
εδάφους· σωστή κίνηση αλλά λάθος ρεύμα → torque/current map και BTS7960 loss·
σωστή γωνία αλλά λάθος κέντρο στροφής → CG X/Y και κατανομή φορτίου. Μετά το
fit επαναλαμβάνονται τα calibration cases και μόνο τότε εκτελείται το holdout.

## 5. Μετρήσεις και logs

- Μετροταινία: τελικό X/Y του κέντρου chassis, μέγιστη πλάγια απόκλιση και
  chord μετατόπισης σε spin. Σημάδεψε μπροστά/πίσω κέντρο ώστε να βγαίνει yaw.
- Βίντεο από πάνω ή σταθερή φωτογραφία πριν/μετά βοηθά, αλλά δεν απαιτείται.
- Mega serial σε αρχείο από 2 s πριν έως 2 s μετά: ολόκληρες γραμμές `T` στα
  50 Hz, events, εντολές controller και timestamps. Κράτα LF/LR/RF/RR raw
  counts και raw `imu.gz`; μη μηδενίσεις μέσα σε run.
- MPU6050: πάρε 5 s ακίνητο bias πριν από κάθε batch. Στο default ±250°/s,
  `gz/131` δίνει °/s· αφαίρεσε bias και ολοκλήρωσε με τα timestamps των 20 Hz.
- Κατέγραψε V και total A από το PSU ως ξεχωριστό log, ιδανικά ≥100 Hz για
  peak. Αν υπάρχει μόνο ένδειξη οθόνης, γράψε μέγιστο και αν άναψε CC.

Το `motion_intake_mega.ino` ήδη δίνει time, state, **applied duty** αριστερά/
δεξιά, τέσσερα drive counts, beams, E-stop, IMU-present και raw gz. Τα πεδία
`driveCurrentLeft/Right` είναι ramped duty, όχι ampere. Δεν δίνει τάση,
ηλεκτρικό ρεύμα, counts/rev, wheel velocity ή ολοκληρωμένο yaw. Επίσης δεν
υλοποιεί το gyro loop του 5a. Χωρίς αλλαγή firmware, per-motor physical current
δεν μπορεί να βαθμονομηθεί· το P2-62 συγκρίνει το συνολικό PSU current και
δηλώνει το per-motor current `NOT_MEASURED`. Δεν προτείνεται αγορά.

## 6. Freeze και ίχνος

Πηγή: υπάρχοντα `gazebo_5a` και `gazebo_5a_turn_diagnostic`, ballast case,
μ=0,5/0,7/0,9. Generator:
`engineering/chassis-strength-bench/run_phase2_p62_drive_predictions.py`.
Git base SHA `5196885912057f5df8cdf0e444006d1d9ac3c03c` (dirty tree δηλωμένο μέσα
στο artifact). Frozen JSON SHA-256:
`c2fc03c7baa97764d5d31bdf8de1f4e3f22555285200922f22910c3f01144a20`,
επίσης στο διπλανό `.sha256` αρχείο.

Read-only έλεγχος v3: τα front-terminal holes είναι πλέον 30 mm από τη διεπαφή
και το sleeve coupon δεν έχει boss, όπως ζήτησε ο χρήστης. Δεν βρέθηκε δεύτερη
ίδια αριθμητική αστοχία. Τα rail-extension και crossbar-half ζεύγη έχουν
συμμετρικές οπές 30 mm από τα άκρα στον κώδικα, αλλά το README λέει ότι δεν
έχουν ακόμη φυσικά probed· πριν την κίνηση ελέγχεται απλώς ότι sleeve και οι
δύο οπές ευθυγραμμίζονται και ότι η εισαγωγή είναι 60 mm. Η νεότερη
through-bolt revision έχει ήδη probed τις πλαϊνές οπές μέσα από τους δύο
τοίχους· αυτό δεν αντικαθιστά τον παραπάνω έλεγχο joint alignment. Καμία
αλλαγή CAD/STL.

## 7. P2-64 firmware και Pi runner

Χρησιμοποιείται αποκλειστικά το νέο `arduino/motion/drive_test_mega/`· το
`motion_intake_mega.ino` μένει ανέπαφο. Το νέο sketch κρατά τα ίδια drive,
encoder, E-stop, LED και MPU6050 pins και υλοποιεί 100 Hz wheel PI
με 40 ms encoder-speed window. Το duty είναι
`ω_target/ω_noload(12 V) + PI correction`, με back-calculation anti-windup
και integral range που καλύπτει ολόκληρο το επιτρεπτό PWM. Η αρχική
`ω_noload=12,776 rad/s` προέρχεται μόνο από τα ονομαστικά 122 rpm και πρέπει
να αντικατασταθεί με lifted-wheel μέτρηση (`CFG NOLOAD` ή
`--no-load-rad-s`) πριν από floor run. Το gyro outer PI παραμένει
`Kp=2,0`, `Ki=0,8`, correction ±1,5 rad/s. Υπάρχουν configurable
ramp/PWM/radius/track/CPR, 300 ms timeout, run time/distance limits και
telemetry 50 Hz.

Αυτό **αντικαθιστά** την P2-63 περιγραφή `duty=torque/3,73`: BTS7960 ελέγχει
τάση και όχι ροπή. Οι transient αποκρίσεις του φυσικού controller έναντι 5a
είναι μόνο ποιοτικές. Τα σταθερά geometric endpoints και slip gates μένουν
παγωμένα.

Η serial σύνδεση είναι 500000 baud σε firmware, runner και ROS bridge. Μια
συντηρητική γραμμή 220 χαρακτήρων χρειάζεται 4,4 ms στο wire, κάτω από το
όριο STOP 20 ms. Το firmware δημοσιεύει `max_loop_gap_us` και
`max_telem_write_us`. Πριν το floor bring-up απαιτείται πραγματικό
`max_loop_gap_us < 20000` και χειροκίνητη δοκιμή STOP με τροχούς στον αέρα·
το compile/dry-run αποδεικνύει format και host timing, όχι hardware latency.
P2-64 pre-upload evidence: Mega compile 19.020 bytes flash/882 bytes RAM,
`spin_90 --dry-run` 155 samples με max host-loop gap 5,20 ms και υπολογισμένο
wire time 4,4 ms, Jazzy build PASS, fake serial 4/4 PASS. Η πραγματική
telemetry/STOP latency δεν έχει ακόμη μετρηθεί.

Δεν βρέθηκε ενεργό repository evidence για counts/rev του FIT0403. Το
αρχειοθετημένο `700 CPR` αφορά FIT0186 και δεν χρησιμοποιείται. Το firmware
αρνείται `ARM` μέχρι να δοθεί θετικό `CFG CPR`, με τιμή από το rollout δέκα
περιστροφών.

### Sync, compile και upload στο Pi

Από PC στο AP `TennisRobot`:

```bash
rsync -av arduino/motion/drive_test_mega/ \
  tennisserver@tennisserver.local:~/tennis-robot/arduino/motion/drive_test_mega/
rsync -av scripts/run_drive_test_pi.py \
  tennisserver@tennisserver.local:~/tennis-robot/scripts/
rsync -av --delete ros2_ws/src/tennis_robot/ \
  tennisserver@tennisserver.local:~/tennis-robot/ros2_ws/src/tennis_robot/
```

Στο Pi:

```bash
cd ~/tennis-robot
python3 -m pip install --user pyserial
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:mega arduino/motion/drive_test_mega
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino/motion/drive_test_mega
cd ros2_ws
colcon build --packages-up-to tennis_robot
source install/setup.bash
```

Πριν το upload κλείνουν runner και bridge. Runner/bridge χρησιμοποιούν το ίδιο
exclusive `flock`, `/tmp/tennis_robot_ttyACM0.lock`, άρα δεν μπορούν να
ανοίξουν ταυτόχρονα το `/dev/ttyACM0`.

### Scripted runs και ασφάλεια SSH

```bash
cd ~/tennis-robot
python3 scripts/run_drive_test_pi.py straight_0p3 --counts-per-rev CPR --dry-run
python3 scripts/run_drive_test_pi.py straight_0p3 --counts-per-rev CPR \
  --no-load-rad-s MEASURED_RAD_S --mode open
python3 scripts/run_drive_test_pi.py spin_90 --counts-per-rev CPR --mode gyro
python3 scripts/run_drive_test_pi.py arc_R0p5 --counts-per-rev CPR --mode gyro
```

Names: `straight_0p3`, `straight_0p6`, `spin_90`, `spin_180`, `spin_360`,
`arc_R0p5`, `arc_R1p0`, `accel_brake`. Space, `q` ή Esc στέλνει STOP αμέσως.
SIGINT/SIGTERM/SIGHUP στέλνουν STOP+DISARM. Σε βίαιο θάνατο ή απώλεια SSH,
το Mega σταματά μόνο του σε 300 ms. Κάθε scenario γράφει CSV και metadata JSON
στο `~/tennis-robot-logs/`. Το D-pad δεν χρησιμοποιείται για sim comparison.

Ανάκτηση/σύνοψη από PC:

```bash
python3 scripts/fetch_drive_test_logs.py \
  --remote tennisserver@tennisserver.local:~/tennis-robot-logs \
  --output-dir results/physical_drive_p263
```

Προαιρετικό `--measurements measurements.json` προσθέτει tape X/Y/yaw και PSU
mean/peak current ανά run stem. Χωρίς αυτά, η σύνοψη δηλώνει ότι encoder odom
μόνο του δεν επικυρώνει slip.

```json
{
  "20261015T120000Z_straight_0p3_open_r85": {
    "x_m": 2.98, "y_m": 0.03, "yaw_deg": 1.2,
    "total_current_mean_a": 1.1, "total_current_peak_a": 18.7
  }
}
```

### Bring-up

1. Τροχοί στον αέρα, PSU limit 5 A: NC E-stop D33→GND, φορά κάθε
   τροχού/encoder, μέτρηση `ω_noload`, STOP,
   απώλεια SSH και timeout. Ελέγχονται όλες οι νέες διαμπερείς πλαϊνές βίδες
   με εξωτερικές ροδέλες/παξιμάδια· δεν υπάρχουν εσωτερικά nut pockets.
   Στη φάση PSU το πλήκτρο Output του τροφοδοτικού είναι το κύριο E-stop
   ισχύος. Το D33 είναι fail-safe λογικό STOP: LOW=OK, HIGH=πατημένο ή
   κομμένο καλώδιο, και στο boot HIGH απαγορεύει ARM.
2. Πάτωμα χωρίς έρμα: rollout CPR/radius, 0,1 και μετά 0,2 m/s, open mode.
3. Πάτωμα με πλάκα 5,448 kg και battery dummies: P2-62 σειρά, πρώτα straight
   0,3, έπειτα open spin/arcs και τελευταία τα 28 A scenarios αν επαρκεί το PSU.
4. Parkside quantitative comparison μόνο αφού κλείσει το PSU bring-up, με
   PWM ceiling 0,667. Στη μελλοντική χρήση μπαταρίας απαιτείται ανεξάρτητο
   E-stop που κόβει την ισχύ μοτέρ· το D33 μόνο του δεν επαρκεί.

Η pin-by-pin καλωδίωση, οι διατομές, η ασφάλεια, το διάγραμμα και το checklist
πριν από το πρώτο ρεύμα βρίσκονται στο
`docs/hardware/drive-test-wiring-el.md`.

### D-pad μέσω ROS

Το ξεχωριστό `physical_drive_bringup.launch.py` ξεκινά controller, τον
αναγκαίο navigation adapter, command bridge, Twist-stamp relay, twist_mux,
`base_serial_bridge` και το αμετάβλητο control panel. Δεν ξεκινά Nav2, SLAM ή
perception και δεν αλλάζει το `run_pi.sh`.

```bash
cd ~/tennis-robot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TENNIS_ROBOT_ROOT=$HOME/tennis-robot
export DRIVE_COUNTS_PER_REV=CPR
export MANUAL_LINEAR_SPEED_M_S=0.20
export MANUAL_TURN_SPEED_RAD_S=0.40
ros2 launch tennis_robot physical_drive_bringup.launch.py
```

Από κινητό/PC στο `TennisRobot`: `http://tennisserver.local:8081`. Αν δεν
λειτουργεί mDNS, τρέξε `hostname -I` μέσω SSH και άνοιξε
`http://<διεύθυνση-AP>:8081`. Το panel ακούει στο `0.0.0.0`. Η άφεση του
D-pad οδηγεί σε timeout controller/mux/bridge και το φυσικό E-stop παραμένει
ο ανεξάρτητος τελικός φραγμός.
