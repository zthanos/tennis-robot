# Gazebo perception covariance C2 v3 — activation report

> Ημερομηνία: 2026-07-22 — **ACTIVE / PASS για Gazebo perception**

## Artifact και evidence

- Artifact: `calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v3.json`
- Identity: `gazebo-range-depth-quality-diagonal-v1-20260722-v3` / `gazebo-v3`
- SHA-256: `338adb895e764422e51ddde549726514815539a8ddcf3dbab82c17c2563b7027`
- Evidence: 18/18 trials, 540 accepted target samples, 0% target outliers σε
  κάθε trial.
- Domain: range `1.0218417–6.7652887 m`, depth quality `0.8888889–1.0`.
- Conservation: PASS ανά axis και ανά range×quality bin.

## Διορθώσεις που προηγήθηκαν

1. RGB και depth Gazebo cameras ευθυγραμμίστηκαν στο ίδιο HFOV `1.204 rad`.
2. Το depth pixel αναγνωρίζεται ως optical-axis `Z` και μετατρέπεται σωστά σε
   ray XYZ/range.
3. Η REP-103 optical θέση `(right, down, forward)` μετασχηματίζεται με πλήρη
   3D quaternion και πλήρες 3×3 covariance προς map XY.
4. Neural zoom tiles κρατούν το confidence threshold `0.35` και επεκτείνουν την
   ανίχνευση μικρών balls· δεν υπάρχει HSV fallback.
5. Η σάρωση χρησιμοποιεί 18 headings και το association περιλαμβάνει μία φορά
   το shared localization budget, διατηρώντας απαίτηση δύο distinct steps.

## Live αποτέλεσμα

Το τελικό collect-route scan παρήγαγε 30 accepted observations, 13 tracks και
10 confirmed snapshot targets (7 με 3 confirmations και 3 με 2). Άρα το αρχικό
`completed_no_targets` δεν οφείλεται πλέον σε perception. Ο planner επέστρεψε
`empty_no_feasible_targets`: 1 target `keepout` και 9 `no_candidate_found`.
Αυτό καταγράφεται ως επόμενο planner/geometry task.

Το pilot στα `8.263 m` απέτυχε στο neural target-detection gate. Το ονομαστικό
depth range 9 m δεν δηλώνεται ως operational ball-perception range.

## Νεότερη end-to-end επιβεβαίωση

Μετά τις διορθώσεις planner/controller, νεότερη live εκτέλεση ολοκλήρωσε 18/18
scan headings με 29 accepted observations, 11 tracks και 9 confirmed targets.
Ο bounded planner επέλεξε 2 targets και η διαδρομή ολοκληρώθηκε ως
`route_completed`: και οι δύο crossings ήταν hard-compliant στα `0.35 m/s`.
Επομένως το παλιό planner αποτέλεσμα δεν είναι πλέον ενεργό blocker. Τα 7
deferred targets παραμένουν θέμα planning budget/follow-up, όχι perception.
