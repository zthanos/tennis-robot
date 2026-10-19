# P2-61 — διόρθωση contact model και επανέλεγχος

Η P2-61 επιβεβαιώνει το suspected factor-of-two bug και αντικαθιστά τα
Gazebo performance numbers που παρήχθησαν με αυτό. Δεν άλλαξε κανένα frozen
στοιχείο P2-40/P2-51: pivot X387, τροχοί Ø200×60 σε Y±129, nip 58 mm, rails,
R75/42° ski-jump και καλάθι παραμένουν ίδια. Δεν προτείνεται πλέον
ρυθμιζόμενο nip: ο δεσμευτικός στόχος 12–14 m/s επιτυγχάνεται στο σημερινό
nip με μ=0,9 και surface speed 16 m/s.

## Αιτία και calibration gates

Ο ITF νόμος `F=K*delta^1.5`, `K=107309,294`, βαθμονομήθηκε σε συνολική
διαμετρική παραμόρφωση `delta`. Το plugin έδινε στον νόμο τη μονοπλευρική
βύθιση `p`. Η σωστή διεπαφή είναι `delta=2p`, ρητά μέσω
`diametral_compression_scale=2.0` για wheel, ground και platen.

- historical Gazebo two-platen: 13,000001 mm από 15,57 σε 95,64 N,
- corrected Gazebo two-platen: 6,500001 mm,
- nip 58 mm: `p=4 mm` ανά τροχό, άρα 27,147 N historical και 76,784 N
  corrected ανά τροχό (λόγος 2,828427),
- damping μόνο: 4692,375891 → 2356,929199 N·s/m^1.5,
- Gazebo rebound στα 0,25 ms: 1,410390 m,
- fine Gazebo convergence 0,25/0,125/0,0625 ms:
  1,410390/1,411000/1,410510 m, spread 0,043%,
- το παλιό 1/0,5/0,25 ms screen αποτυγχάνει με spread 2,74%, επειδή η
  corrected επαφή διαρκεί μόνο ~4,63 ms. Production timestep: 0,25 ms.

Το platen μέρος, K, loading/unloading exponents και ITF deformation target
δεν επαναπροσαρμόστηκαν. Η canonical standalone calibration και το Gazebo
model ενημερώθηκαν στην ίδια σύμβαση `delta=2p`.

## 5d-1, σημερινό nip 58 mm, +20°

Όλες οι 16 περιπτώσεις είχαν κανονική bilateral έξοδο και μηδενικές
απαγορευμένες επαφές. Οι τιμές είναι εύρος feed 0/2 m/s:

| μ | surface (m/s) | exit speed (m/s) |
|---:|---:|---:|
| 0,6 | 12 | 10,418–10,606 |
| 0,6 | 16 | 10,422–10,607 |
| 0,6 | 20 | 10,424–10,607 |
| 0,6 | 24 | 10,426–10,608 |
| 0,9 | 12 | 11,528–11,654 |
| 0,9 | 16 | **12,776–12,929** |
| 0,9 | 20 | 12,778–12,929 |
| 0,9 | 24 | 12,776–12,930 |

Άρα μ=0,9 αρκεί και η μικρότερη command που πιάνει 12–14 m/s είναι
16 m/s (1528 rpm σε Ø200). Το μ=0,6 δεν αρκεί και κορέζεται ~10,6 m/s.
Το rpm drop είναι 60–71 rpm στα selected μ=0,9/surface16 cases και recovery
≤0,75 ms στο sampled trace· peak modeled current παραμένει στο 20 A limit.

## Collection και δύο μπάλες

Η διορθωμένη επαφή αλλάζει το collection setpoint, όχι τη γεωμετρία:

- surface 6,0 m/s: 11/30 P2-51 delivery matrix,
- surface 5,0 m/s: **30/30**, με takeoff 3,819–4,380 m/s, γωνία
  39,795–40,115° και LiDAR margin 42,10–62,90 mm.

Στα 5c-3/P2-58 regressions με 5,0 m/s και 0,25 ms:

- σειρές: 8/8 PASS,
- P2-58 boundary dx35/dy70 ή 100: 8/8 PASS και στις δύο lead πλευρές,
- dx0/dy70 ή 100: 8/8 REJECT,
- side-by-side ±50: 0/2 basket. Παράγεται wheel rpm event χωρίς downstream
  beam· λειτουργικά παραμένει περίπτωση fallback, παρότι μετά την αρχική
  εμπλοκή οι μπάλες τελικά απορρίπτονται αντί να μείνουν ακίνητες.

Το offline combined detector στο ίδιο corrected dataset κράτησε beam
`base_link X=0,320 m, Z=0,045 m`, T=100 ms: 0/16 false positives στα PASS
cases και 2/2 detections στα side-by-side failures. Event→beam ήταν
30–50 ms και το ελάχιστο καθαρό διάστημα μεταξύ pulses 110 ms.

## Απόφαση

Η P2-60 σύσταση για 52 mm/ρυθμιζόμενο nip **ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ**. Η μικρότερη
αλλαγή που πετυχαίνει τον δεσμευτικό στόχο είναι καμία γεωμετρική αλλαγή:
κρατάμε κοινό frozen nip 58 mm, collection surface 5 m/s και launch surface
16 m/s με measured/qualified tread μ≥0,9. Παραμένει physical gate η μέτρηση
του πραγματικού dynamic μ· αν το πραγματικό πέλμα είναι κοντά στο 0,6, το
12 m/s requirement δεν κλείνει με το σημερινό μοντέλο.

## Recorded playback

Launch boundaries:

```text
engineering/shared-flywheel-isolated-bench/replay_5d1_p261_case.sh 0.6 24 2
engineering/shared-flywheel-isolated-bench/replay_5d1_p261_case.sh 0.9 12 2
engineering/shared-flywheel-isolated-bench/replay_5d1_p261_case.sh 0.9 16 0
engineering/shared-flywheel-isolated-bench/replay_5d1_p261_case.sh 0.9 16 2
```

Side-by-side failures:

```text
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_side_by_side_y50 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_side_by_side_y50 0.9
```

```text
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy70_lead_plus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy100_lead_plus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy70_lead_plus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy100_lead_plus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy70_lead_minus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy100_lead_minus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy70_lead_minus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy100_lead_minus 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy70_lead_plus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy100_lead_plus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy70_lead_plus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy100_lead_plus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy70_lead_minus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx0_dy100_lead_minus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy70_lead_minus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p261_p258_dx35_dy100_lead_minus 0.9
```

Όλα τα playback scripts χρησιμοποιούν `docker/jazzy/run.sh`, ανοίγουν
paused και δεν άλλαξαν το `5c3_side_gui.config`.

Artifacts: `gazebo_p261_contact_model/`, `gazebo_5d1_p261/`,
`gazebo_5c_p261_delivery*`, `gazebo_5c3_p261_*` κάτω από
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/`.
