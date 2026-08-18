# Dual-wheel intake — design spec

Ημερομηνία: 2026-07-10
Branch: `feat/dual-wheel-intake-concept`

Αυτό το έγγραφο ορίζει τη γεωμετρία, τη φυσική και τα κριτήρια αποδοχής του
dual-wheel intake που αντικαθιστά το single top-roller concept. Η απόφαση
εγκατάλειψης του top roller τεκμηριώνεται στο
`docs/mechanism/intake-concept-decision-el.md` και στο
`docs/mechanism/intake-bench-sweep-report-el.md`.

## Τρέχουσα as-built baseline (2026-08-16)

Η αρχική nominal γεωμετρία παρακάτω παραμένει ως design rationale και sweep
history. Η ενεργή default γεωμετρία του generated robot είναι:

```text
δύο ανεξάρτητα intake motors, ένα ανά wheel
wheel radius       0.060 m
wheel height       0.080 m
gap                0.056 m
nip x               0.540 m
wheel tilt          35 deg
carriage travel     0.008 m προς τα έξω
max angular speed  26.3 rad/s
effort limit        1.77 N·m
```

Πηγή εκτέλεσης είναι το
`ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro` και τα `INTAKE_WHEEL_*`
arguments του generator. Το συνολικό ενεργό layout φαίνεται στο
`docs/hardware/chassis-layout-4wd-dual-intake-el.md`.

## Αποφάσεις χρήστη (2026-07-10)

```text
1. Διάταξη: οριζόντιο (πλευρικό) pinch — δύο τροχοί σε ΚΑΤΑΚΟΡΥΦΟΥΣ άξονες,
   ένας αριστερά και ένας δεξιά ενός κεντρικού διαδρόμου.
2. Καταμερισμός ρόλων:
   - funnel  -> centering (οδηγεί τη μπάλα στο κέντρο του διαδρόμου)
   - wheels  -> capture + transport (πιάνουν τη μπάλα και τη στέλνουν πίσω)
   - ramp    -> elevation + guidance προς το basket
3. Κίνηση: ΔΥΟ πανομοιότυπα μοτέρ (ίδιο μοντέλο με του παλιού top roller),
   ένα ανά τροχό, αντίθετης φοράς. Αντικαθιστά την αρχική ιδέα
   "ένα μοτέρ + γρανάζια". Συνέπεια BOM: +1 μοτέρ.
```

## Τι αφαιρείται / τι μένει

Αφαιρούνται (γεωμετρία που υπήρχε ΜΟΝΟ για τον top roller):

- `intake_roller` macro / `lift_wheel_link` ως οριζόντιος roller
  (drivetrain.urdf.xacro)
- το bite lip ως backstop του roller (το impulse δίνεται πλέον από το pinch,
  όχι από αντίσταση lip-roller)
- κάθε spiral/paddle/carriage κατασκευή του παλιού debugging trail

Μένουν:

- funnel cheeks (centering) — `funnel.urdf.xacro`
- ramp/scoop mesh (elevation) — `generate_curved_scoop_mesh.py`, με
  προσαρμογές entry (βλ. παρακάτω)
- basket, IR beams, intake debug camera
- το deterministic bench + release-criteria tooling

## Γεωμετρία (nominal — όλα sweep-able)

Σταθερές αναφοράς: μπάλα ⌀66 mm (r=33), rest z=33 mm (μετά το court fix),
funnel mouth συγκλίνει σε 180 mm στο intake, cheeks στα y=±145 mm,
funnel_link frame: ground = funnel_z − 0.038 m.

```text
intake_wheel_left_link / intake_wheel_right_link
  Σχήμα           κύλινδρος, κατακόρυφος άξονας (revolute joint στο z)
  Ακτίνα Rw       0.060 m            (sweep 0.050-0.070· βλ. motor ισοζύγιο)
  Ύψος            0.080 m, z 0.005-0.085 ground
                   (καλύπτει τον ισημερινό της μπάλας z=33)
  Nip x           0.590 m ground     (sweep ±0.02 — κοντά στο σημερινό lip)
  Κέντρα y        ±(gap/2 + Rw) = ±0.090 m για gap 60 mm
  Gap g           0.060 m            (3 mm interference/πλευρά, sweep 0.056-0.066)
  Υλικό επαφής    μ=2.5 (λάστιχο/αφρός, όπως τα paddles)
```

### Λατερική ενδοτικότητα (υποχρεωτική — δίδαγμα test1/test2)

Μπάλα και τροχοί είναι rigid bodies στο DART: interference = σφήνωμα,
clearance = μηδενική πρόσφυση (αποδεδειγμένο στα collect_test1/2). Άρα το
"3mm interference/πλευρά" ΔΕΝ μπορεί να είναι rigid — κάθε τροχός κρεμιέται
σε **prismatic y-carriage με ελατήριο** (το αποδεδειγμένο SDF
`<spring_stiffness>` patch του log #9):

```text
Carriage joint   prismatic, axis 0 1 0 (left) / 0 -1 0 (right)
Travel           0..+8 mm προς τα έξω (κάτω stop στο rest gap)
Spring k         ~1000 N/m αρχικά (sweep axis μαζί με το rest gap)
Rest gap         0.060 m (3 mm interference/πλευρά που γίνεται grip force)
```

Γιατί εδώ δουλεύει ενώ στο top roller απέτυχε (log #13): εκεί η απαιτούμενη
διεύθυνση υποχώρησης ΠΕΡΙΣΤΡΕΦΟΤΑΝ κατά μήκος της τροχιάς (ακτινική γύρω
από τον roller) και το 1-DOF κατακόρυφο ελατήριο δεν την κάλυπτε· εδώ η
διαδρομή είναι ευθεία και η απαιτούμενη υποχώρηση είναι ΣΤΑΘΕΡΑ ±y σε όλο
το πέρασμα — ακριβώς ο βαθμός ελευθερίας του carriage.

Έλεγχοι χωροθέτησης που πρέπει να περάσουν στη Φάση 1 (offline, στο
γεννημένο SDF):

```text
- wheel bottom (z=0.005) > ramp top στο nip x            (ramp εκεί ~0.001-0.004)
- wheel outer edge (y=±0.150) δεν τέμνει τα funnel cheeks (y=±0.145 —
  ΟΡΙΑΚΟ: πιθανώς τα cheeks κοντύνουν ή μετατοπίζονται μπροστά από το nip)
- wheel κύλινδροι δεν τέμνουν chassis/ramp/deflector
- nip centerline ευθυγραμμισμένο με ramp centerline (y=0)
```

## Λειτουργική φιλοσοφία

```text
capture -> transport -> guide -> hopper
```

Η μπάλα δεν εκτοξεύεται (το launch model ήταν του παλιού concept)· οι τροχοί
τη συλλαμβάνουν και τη μεταφέρουν ενεργά μέσα από το throat, και η ράμπα την
καθοδηγεί στο hopper. Κάθε στάδιο επικυρώνεται χωριστά (βλ. Concept
Validation Plan στο decision doc).

## Φυσική: γιατί το pinch δουλεύει εκεί που ο top roller απέτυχε

Ο top roller απέτυχε γιατί ζητούσε ταυτόχρονα αντίσταση για bite και
απελευθέρωση για launch από ΜΙΑ ενεργή επιφάνεια (βλ. decision doc). Στο
πλευρικό pinch:

- Δύο ενεργές επιφάνειες οδηγούν ΚΑΙ οι δύο προς τα πίσω (−x): η μπάλα
  παίρνει καθαρή rearward ταχύτητα ~ίση με το surface speed, χωρίς να
  χρειάζεται τρίτη επιφάνεια αντίστασης.
- Η ενδοτικότητα που έλειπε (radial compliance, log #13) εδώ είναι εγγενής:
  η συμπίεση είναι συμμετρική στο οριζόντιο επίπεδο και διαρκεί μόνο όσο
  η μπάλα διασχίζει το nip (~2·√(Rw²−(Rw−δ)²) ≈ 40 mm διαδρομή).
- Η κατεύθυνση εξόδου ορίζεται από τη γεωμετρία (centerline), όχι από
  λεπτή ισορροπία τριβών.

## Transport speed budget (με το πραγματικό μοτέρ — προς μέτρηση στη Phase 3)

Πραγματικό μοτέρ (επιβεβαιωμένο από specs χρήστη, 2026-07-10):

```text
GB37Y3530-12V-251R, gear 43.8:1, encoder Hall 16/700 CPR
No-load speed    251 RPM ±10%  = 26.3 rad/s
Stall torque     18 kg·cm      = 1.77 N·m
Stall current    7 A (driver: BTS7960/TB6612 ανά κανάλι)
```

Η ράμπα καταλήγει στο basket floor top z=0.128 m:

```text
v_min χωρίς απώλειες      √(2·9.81·0.128) ≈ 1.59 m/s
v_target με απώλειες      ~2.0 m/s

Surface speed = ω · Rw (ω_load = ω0·(1 − T/T_stall)):
  26.3 rad/s × 0.060 m = 1.58 m/s   free-run — οριακά στο v_min
  ~20  rad/s × 0.060 m = 1.20 m/s   υπό φορτίο λαβής (~7.5 N traction)
  μεγαλύτερο Rw ΔΕΝ σώζει: +Rw ⇒ +torque demand ⇒ droop, και ⌀>120
  συγκρούεται με τα cheeks
```

**Συμπέρασμα**: το momentum-climb στο z=0.128 ΔΕΝ είναι εφικτό με αυτό το
μοτέρ σε λογικό μέγεθος τροχού. Αποδεκτό υπό την transport φιλοσοφία — η
ανύψωση δεν είναι ευθύνη του release. Η Phase 3 μετρά το πραγματικό
έλλειμμα και επιλέγει από τα mitigations, με σειρά προτίμησης:

```text
1. Feed-onto-ramp: nip x τοποθετημένο ώστε η επαφή των τροχών να
   συνεχίζεται πάνω από την αρχή της ράμπας (η μπάλα σπρώχνεται, δεν
   εκτοξεύεται· ~40 mm ενεργής διαδρομής επαφής).
2. Χαμήλωμα hopper entry / basket handoff (αλλαγή ράμπας).
3. Rw 0.070 + μετατόπιση cheeks (τελευταία επιλογή — αγγίζει το funnel).
```

Nominal: **Rw=0.060 m, ω setpoint ±25 rad/s, joint effort limit 1.77 N·m,
ω cap 26.3 rad/s** — έτσι το stall/jam στο sim αναπαράγει το πραγματικό
droop του μοτέρ.

### Rejected-for-now high-speed option: JGB37-3530-1000

Το JGB37-3530-1000 (12V, 10:1, ~1000 RPM no-load / ~800 RPM rated) δίνει
πολύ υψηλότερη ταχύτητα αλλά κόβει δραματικά το torque:

```text
Rw=0.060 m

251 RPM motor:
  26.3 rad/s × 0.060 = 1.58 m/s
  stall 18 kg·cm = 1.77 N·m ≈ 29 N tangential / wheel

1000 RPM / 10:1 motor:
  800 RPM rated ≈ 83.8 rad/s × 0.060 = 5.0 m/s
  rated 0.38 kg·cm = 0.037 N·m ≈ 0.62 N tangential / wheel
  stall 1.52 kg·cm = 0.149 N·m ≈ 2.5 N tangential / wheel
```

Για `capture -> transport -> guide -> hopper`, το ζητούμενο είναι ελεγχόμενη
λαβή με torque reserve, όχι launcher-like surface speed. Άρα το 1000 RPM
μοτέρ **δεν μπαίνει ως default intake motor**. Μπορεί να μείνει ως μελλοντικό
high-speed experiment μόνο αν η Phase 3 δείξει ότι χρειαζόμαστε διαφορετικό
handoff μηχανισμό και όχι απλώς feed-onto-ramp.

## Actuation

```text
Joints      intake_wheel_left_joint, intake_wheel_right_joint
            (continuous, axis 0 0 1, ένα ανά τροχό)
Έλεγχος     δύο velocity interfaces στο ros2_control, κοινό setpoint
            αντίθετου πρόσημου: ω_left = -ω, ω_right = +ω ώστε οι
            εσωτερικές όψεις να κινούνται προς -x (rearward)
Env         COLLECTOR_INTAKE_WHEEL_SPEED (rad/s, default 25)
Hardware    2 × GB37Y3530-12V-251R gear motor με encoder (ίδιο με τον
            παλιό lift wheel) — ο controller στέλνει ένα κοινό μέτρο
            ταχύτητας, όπως ο πραγματικός driver με δύο κανάλια
Effort/ω    joint effort limit 1.77 N·m (stall torque), velocity limit
            26.3 rad/s (no-load) — το stall/jam στο sim γίνεται φυσικά
            ρεαλιστικό (βλ. Transport speed budget)
Jam metric  per-wheel: |joint velocity| καταρρέει ενώ υπάρχει ενεργό
            contact → stall/jam στο analyzer (αντιστοιχεί στο encoder-based
            jam detection του πραγματικού collector)
```

## Ramp entry προσαρμογές

- `INTAKE_LIP_RAISE_M` → 0 (το lip-as-backstop δεν χρειάζεται· κρατάμε τη
  μεταβλητή για sweep — μη-μηδενικό lip ίσως βοηθά το πρώτο άγγιγμα).
- Το ramp profile (clear zone → knee → end) μένει ως έχει· το entry πρέπει
  να ξεκινά ΠΙΣΩ από το nip plane ώστε η μπάλα να πατά στη ράμπα τη στιγμή
  που δέχεται την ώθηση.
- Ramp πλάτος 180 mm: πρέπει να στενέψει ή να κοπεί τοπικά γύρω από τους
  τροχούς αν υπάρχει τομή (έλεγχος Φάσης 1).

## Instrumentation & acceptance

Contact sensors: ένα ανά τροχό, δεμένα στα υπάρχοντα bridges
`roller_contact_0` (left) και `roller_contact_1` (right), με το δυναμικό
fail-loud binding του `generate_robot_urdf.py`.

Bench: `scripts/sim_debug/run_native_intake_sweep.sh` με νέους sweep axes:

```text
INTAKE_WHEEL_GAP_M, INTAKE_WHEEL_RADIUS_M, INTAKE_NIP_X_M,
INTAKE_SWEEP_WHEEL_SPEEDS, INTAKE_SWEEP_DRIVE_SPEEDS
```

Criteria (από `docs/mechanism/intake-concept-decision-el.md` — transport concept):

```text
Required:
- confirmed contact with both rollers (roller_contact_0 ΚΑΙ _1)
- successful capture through the wheel throat
- positive inward transport through the intake
- no stall or jam
- ramp-entry crossing
- hopper-entry or ramp-crest crossing
- repeatable success in at least 4/5 runs

Preferred:
- transport speed >= selected target
- contact duration within expected range
- force_p95 below selected threshold
- successful collection from lateral offsets
- successful collection across drive-speed variations
```

Το "positive vertical velocity at release" του παλιού concept ΔΕΝ
μεταφέρεται — ήταν απαίτηση του launch model.

Απόδειξη αντικατάστασης: head-to-head στο ίδιο bench απέναντι στο καλύτερο
καταγεγραμμένο top-roller baseline (7/7 required case του impulse sweep,
release 0.133 m/s, no crest) — το dual-wheel πρέπει να περνά τα νέα required
συν το transport-speed target.

## Φάσεις υλοποίησης (ακολουθούν το Concept Validation Plan)

```text
0. Γεωμετρία + actuation: xacro/generate scripts, 2 velocity joints,
   offline SDF verification, counter-rotation σε /joint_states.
   Υλοποίηση με xacro args (enable_funnel / enable_ramp) ώστε οι
   validation phases να τρέχουν με επιλεκτικά ενεργή γεωμετρία.
1. Phase 1 — dual-wheel throat only (funnel/ramp off):
   capture + powered transport.
2. Phase 2 — funnel + wheels: centering, off-axis capture.
3. Phase 3 — wheels + ramp: transport onto the ramp, μέτρηση του
   πραγματικού transport-speed target.
4. Phase 4 — full intake: collection στο hopper (basket beams).
5. Operating envelope: lateral offset, approach speed, gap tolerance,
   ball compression, friction — 4/5 ανά συνθήκη.
```

Κάθε phase χρησιμοποιεί το υπάρχον bench harness και γράφει
`release_criteria.json` ανά run· αποτυχία phase σταματά την πρόοδο στην
επόμενη (fail-fast, όπως το stop/continue gate του bench report).

Κάθε βήμα καταγράφεται στο `docs/mechanism/intake-debug-log-el.md` (hypothesis /
result / status) το ίδιο turn με την αλλαγή.
