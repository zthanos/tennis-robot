# P2-60 — διάγνωση κορεσμού εκτόξευσης

> **ΙΣΤΟΡΙΚΟ — δεν περιγράφει το τρέχον design.** Διατηρείται ως evidence
> της απόφασης/δοκιμής της εποχής του. Δες
> [το τρέχον design](../current-design-el.md).

> **ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ από P2-61:** όλα τα force/exit-speed συμπεράσματα αυτού
> του diagnostic χρησιμοποίησαν μονοπλευρικό `p` αντί `delta=2p`. Η σύσταση
> για 52 mm/ρυθμιζόμενο nip αποσύρεται. Με corrected model, το frozen 58 mm
> nip και μ=0,9 δίνουν 12,78–12,93 m/s ήδη στα 16 m/s surface speed. Το
> παρόν κείμενο διατηρείται μόνο ως ιστορικό.

## Απόφαση

Το 12–14 m/s είναι δεσμευτικό requirement. Το P2-60 επιβεβαιώνει ότι ο
περιορισμός του frozen 58 mm nip είναι traction impulse, όχι διαθέσιμη
ενέργεια ή D5065 droop. Με surface speed 20 m/s και +20°:

- 8 mm compression: 6,21 m/s στο μ=0,6 και 7,65 m/s στο μ=0,9 από ηρεμία,
- 11 mm: 12,03 m/s μόνο στο μη βαθμονομημένο μ=1,2,
- 14 mm: 13,13 m/s στο μ=0,9 από ηρεμία.

Η μικρότερη λύση μέσα στο ήδη χρησιμοποιημένο sensitivity envelope
μ≤0,9 είναι συνεπώς **14 mm launch compression**, δηλαδή launch gap 52 mm
και wheel centres Y±126 mm. Προτείνεται mode-linked adjustable nip:
collection 58 mm / 8 mm compression και launch 52 mm / 14 mm compression,
κινούμενο από cam του υπάρχοντος P16/hinge mechanism χωρίς δεύτερο actuator.
Η αλλαγή δεν υιοθετήθηκε στο P2-60· είναι πρόταση για επόμενο gated redesign.

## Αναλυτικό traction bound

Για wheel radius `R=0,100 m`, ball radius `r=0,033 m`, diametral
compression `c` και gap `g=0,066−c`, η απόσταση κάθε wheel centre από τη
mid-plane είναι:

`a = R + g/2`.

Κατά μήκος της επαφής:

`δ(x) = R+r−sqrt(a²+x²)`,

`N(x) = 107309,294·δ(x)^1,5`,

για `|x|≤sqrt((R+r)²−a²)`. Το N είναι ανά τροχό. Το impulse inequality
`m·dv/dt ≤ 2μN(t)` με `dt=dx/v` δίνει το ισοδύναμο work-form bound:

`v_out² ≤ v_in² + (4μ/m)∫N(x)dx`, με `m=0,058 kg`.

| Compression | Gap | Contact length | Peak N/τροχό | ∫Ndx/τροχό |
|---:|---:|---:|---:|---:|
| 8 mm | 58 mm | 64,746 mm | 27,147 N | 1,0314 N·m |
| 11 mm | 55 mm | 75,703 mm | 43,770 N | 1,9415 N·m |
| 14 mm | 52 mm | 85,159 mm | 62,847 N | 3,1312 N·m |

Για 8 mm και `v_in=0`, το bound είναι 6,533 m/s στο μ=0,6 και
8,001 m/s στο μ=0,9. Το Gazebo έδωσε 6,209 και 7,648 m/s αντίστοιχα,
δηλαδή 4,96% και 4,41% χαμηλότερα.

Η διαφορά είναι αναμενόμενη. Το ιδανικό όριο θεωρεί συνεχή συμμετρική
επαφή, το αναλυτικό N(x), friction πάντα ακριβώς στο `μN` και όλο το
tangential force ευθυγραμμισμένο με +X. Το Gazebo/DART έχει discrete
1 ms point contacts, damping, μεταβαλλόμενη penetration, κάθετη/πλάγια
συνιστώσα και μικρό normal impulse αντίθετο στην έξοδο. Στο 8 mm/μ=0,9:

- `∫μΣNdt = 0,5038 N·s`,
- actual forward tangential impulse `0,4993 N·s`,
- forward normal impulse `−0,0437 N·s`,
- measured Δp `0,4432 kg·m/s`.

Άρα το solver χρησιμοποιεί σχεδόν ολόκληρο το Coulomb budget. Δεν υπάρχει
κρυμμένη motor-energy έλλειψη που θα διορθωνόταν με περισσότερες rpm.

## Gazebo sensitivity matrix

Το matrix ήταν diagnostic-only. Η πηγαία P2-40/P2-51 γεωμετρία έμεινε
αμετάβλητη· μόνο τα generated worlds μετέφεραν συμμετρικά τους wheel joints.
Η μπάλα τοποθετήθηκε 0,5 mm μέσα από το nominal first-contact plane κατά το
release, ώστε το `v_in=0` να μην είναι μαθηματικό zero-force equilibrium και
να μη φορτίζει τους τροχούς κατά το spin-up. Criteria SHA-256:
`83cb9604d6596eb87613ee0ea462761f89bbac2563ed9a0f18130b893be346b7`.

Exit speed σε m/s, ως `feed 0 / 2 / 4 m/s`:

| Compression | μ=0,6 | μ=0,9 | μ=1,2 |
|---:|---:|---:|---:|
| 8 mm | 6,209 / 6,496 / 7,347 | 7,648 / 7,869 / 8,584 | 8,803 / 9,029 / 9,655 |
| 11 mm | 8,519 / 8,701 / 9,343 | 10,416 / 10,593 / 11,128 | **12,034 / 12,166 / 12,648** |
| 14 mm | 10,765 / 10,897 / 11,392 | **13,128 / 13,262 / 13,713** | 15,146 / 15,284 / 15,652 |

Κύρια effects, από mean όλων των άλλων levels:

- compression 8→14 mm: **+5,289 m/s**,
- μ 0,6→1,2: **+3,416 m/s**,
- feed 0→4 m/s: **+0,755 m/s**.

Κυριαρχεί η συμπίεση, ακολουθεί η τριβή, ενώ η feed speed είναι τρίτης
τάξης. Εννέα από τα 27 cases πέρασαν τα 12 m/s. Το 11 mm/μ=1,2 είναι η
αριθμητικά μικρότερη συμπίεση, αλλά δεν αποτελεί release επειδή μ=1,2 δεν
έχει φυσική βαθμονόμηση. Το 14 mm/μ=0,9 είναι ο πρώτος συνδυασμός μέσα στο
υπάρχον μ≤0,9 envelope και μένει μέσα στο 12–14 για feed 0/2/4.

Στο 14 mm/μ=0,9/v_in=0, το analytical bound είναι 13,941 m/s, το Gazebo
13,128 m/s, peak ΣN 125,65 N, contact duration 63 ms και rpm drop
76,17 rpm/τροχό. Το drop είναι ακόμη μόνο 4,0% των 1909,9 rpm.

Machine-readable aggregate:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p260/summary.json`.

## Σύγκριση με standalone launcher

Το standalone είχε ίδιο 58 mm nip, ίδιο ball law και μ=0,9, αλλά 3 m/s
deterministic feed και πολύ μεγαλύτερη wheel speed. Κορέστηκε στα
8,2475 m/s. Το P2-60 δίνει 7,869 m/s με feed 2 και 8,584 m/s με feed 4·
η παρεμβολή στο feed 3 είναι περίπου 8,23 m/s. Η συμφωνία με το standalone
είναι σχεδόν ακριβής και επιβεβαιώνει ότι και τα δύο benches βλέπουν το
ίδιο Coulomb/contact-length plateau. Περισσότερη surface speed πάνω από το
slip threshold δεν αυξάνει το `∫μNdt`.

## Επίπτωση στη συλλογή — χωρίς νέα 5c runs

Το accepted collection baseline έχει 8 mm compression, surface speed
6 m/s, sequential takeoff 2,44…4,19 m/s, rpm drop περίπου 34…41 rpm/μπάλα,
recovery 8…14 ms και ήδη στιγμιαίο όριο 20 A/0,62 N·m.

Στα 14 mm το peak normal load αυξάνεται 2,315×, η contact length 1,315×
και το ideal friction work 3,036×. Επομένως ένα κοινό 52 mm nip πιθανότατα:

- βελτιώνει bite/capture αλλά αυξάνει squeeze, stall load και ball wear,
- κρατά το controller στο 20 A για περισσότερο χρόνο και μπορεί να κάνει
  rpm drop περίπου 1,8–2× μεγαλύτερο, όπως δείχνει το launch diagnostic,
- σπρώχνει τη συλλεγόμενη μπάλα προς το wheel-speed cap των 6 m/s, πάνω από
  το σημερινό 2,44…4,19 m/s,
- αλλάζει το P2-51 ski takeoff και μπορεί να προκαλέσει overshoot/impact στο
  καλάθι.

Αν επιλεγεί κοινό nip, η πρώτη εκτίμηση είναι να μειωθεί collection surface
speed από 6 σε περίπου 3,5…4,5 m/s ώστε η μπάλα να μείνει στο παλιό
ballistic envelope. Αυτό δεν αποτελεί validation και απαιτεί πλήρες 5c rerun.

## Επιλογή Α — κοινό nip 52 mm

Είναι η μικρότερη καθαρά γεωμετρική αλλαγή: και οι δύο άξονες μετακινούνται
3 mm προς τη mid-plane, από Y±129 σε Y±126. Pivot X387, wheel diameter,
rails, ski-jump και καλάθι μένουν στις ίδιες θέσεις.

Αλλάζει όμως το P2-40 core σε όλες τις λειτουργίες: motor holes/service
cutouts, wheel axes, upper-plate load path και nominal throat. Το P2-51
σχήμα δεν αλλάζει, αλλά αλλάζουν η είσοδος, η takeoff speed και το loading
του ski/καλαθιού. Απαιτούνται ξανά:

1. P2-53 πλήρες −10…+20° exact-solid/ground/arch/stop sweep και P2-54
   torsion/load path με Y±126.
2. Static wheel/hub/plate και actuator/stop load checks στα νέα 125,7 N
   peak pair normal loads.
3. Όλο το 5c baseline: single capture width, P2-55 8 sequential cases,
   P2-56 side-by-side detector/recovery, P2-58 diagonal matrix, beam timing.
4. P2-51 ski/basket retention με νέο collection surface-speed sweep.
5. P2-59 launch matrix, P2-60 confirmation με repeats και μελλοντικό 5d-2.

## Επιλογή Β — ρυθμιζόμενο nip ανά mode, προτεινόμενη

Collection/transport μένουν στα Y±129 και gap 58 mm. Στα +15…+20° δύο
συμμετρικά sliding motor/wheel pods μετακινούνται κατά 3 mm το καθένα προς
Y=0, με θετικά hard stops στα 52 mm. Ένα conjugate cam στον υπάρχοντα
P2-52 hinge/selector shaft εκτελεί τη μετατόπιση καθώς ο P16 ανεβάζει το
module· δεν προστίθεται actuator. Ελατήρια ανοίγουν fail-safe το nip στα
58 mm όταν αποφορτίζεται το cam. Το gap κρατιέται από mechanical stops,
όχι από τη βίδα P16.

Αυτό αλλάζει τοπικά την P2-40 upper plate, τα motor pods, slots/guides,
cam/followers και gap stops. Δεν αλλάζει pivot, wheel envelope, rails,
cheeks ή P2-51 ski/καλάθι, και διατηρεί ακριβώς την accepted collection
γεωμετρία. Χρειάζονται:

1. Νέο CAD/strength gate για pods, guides, hard stops, backlash, parallelism,
   125,7 N nip reaction και cam/P16 torque σε όλο το −10…+20° sweep.
2. P2-53/P2-54 ξανά, επειδή η launch θέση και η upper-plate load path
   αλλάζουν, συν endurance/fail-safe cycling του cam.
3. Σύντομο αλλά πλήρες collection regression P2-55/56/58 για να αποδειχθεί
   ότι το stop επιστρέφει επαναλήψιμα στα 58 mm· όχι retuning του P2-51 αν
   η επιστροφή είναι εντός tolerance.
4. P2-59/P2-60 ξανά στα 52 mm με repeats, measured gap tolerance, beam
   timing και πραγματικά tread coupons, έπειτα 5d-2 feed-rate tests.

Η Επιλογή Α έχει λιγότερα εξαρτήματα αλλά ξανανοίγει ολόκληρη τη συλλογή.
Η Επιλογή Β είναι η μικρότερη **system-level** αλλαγή που ικανοποιεί τον
δεσμευτικό στόχο και προστατεύει τα accepted P2-40 collection/P2-51
ballistics. Προτείνεται η Β, υπό hard gate ότι το πραγματικό dynamic μ είναι
τουλάχιστον 0,9. Αν το coupon/physical nip βγάλει μ≈0,6, κανένας δοκιμασμένος
P2-60 συνδυασμός δεν περνά 12 m/s και πρέπει να επεκταθεί το diagnostic πάνω
από 14 mm πριν κλειδώσει hardware.

## Recordings

Καταγράφηκαν τα πλησιέστερα cases κάτω/πάνω από 12 m/s ανά μ:

```sh
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 8 1.2 4
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 11 0.9 4
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 11 1.2 0
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 14 0.6 4
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 14 0.9 0
```

Το playback ξεκινά paused και επαναχρησιμοποιεί χωρίς αλλαγή το
`5c3_side_gui.config`.

## Κατάσταση

`P2_60_DIAGNOSTIC_COMPLETE_TRACTION_BOUND_CONFIRMED_ADJUSTABLE_52MM_LAUNCH_NIP_RECOMMENDED_PHYSICAL_MU_GATE_OPEN`
