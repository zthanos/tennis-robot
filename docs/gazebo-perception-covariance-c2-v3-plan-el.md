# C2 Gazebo v3 plan — επέκταση spatial-target range

> Κατάσταση: **evidence PASS και ενεργοποιημένο για Gazebo perception** στις
> 2026-07-22. Operational domain: `1.0218–6.7653 m`, quality `0.8889–1.0`.

## Αφορμή

Στο collect-route run της 2026-07-22 ο YOLO δημοσίευσε sports-ball detection με
confidence περίπου `0.707`. Το timestamp-matched depth ROI ήταν πλήρες
(`depth_quality=1.0`) και έδωσε range περίπου `3.984 m`. Το v2 covariance domain
τελειώνει στα `2.979947 m`, άρα ο producer ορθά επέστρεψε
`calibration_out_of_domain`, δημοσίευσε `has_spatial=false` και το route
ολοκληρώθηκε ως `empty_no_balls` / `completed_no_targets`.

Η διερεύνηση βρήκε επιπλέον δύο γεωμετρικά σφάλματα: διαφορετικό RGB/depth FOV
και χρήση του optical-axis `Z` ως slant range. Και τα δύο διορθώθηκαν πριν από
τη νέα συλλογή evidence.

## Στόχος και μη στόχοι

- Design target: διερεύνηση calibrated Gazebo spatial operation από περίπου
  `0.2 m` έως `9 m`.
- Δεν θεωρείται δεδομένο ότι το neural detector θα είναι αξιόπιστο σε όλο το
  depth range. Το τελικό declared domain θα είναι μόνο η συνεχής περιοχή που
  περνά detection, association, coverage και conservation gates.
- Δεν τροποποιείται το v2 artifact και δεν επιτρέπεται covariance extrapolation.
- Το Gazebo v3 δεν αποτελεί evidence για physical OAK-D· το hardware χρειάζεται
  ξεχωριστό `platform=oak_d` artifact.

## Candidate sampling grid

Το pilot πρέπει πρώτα να επιβεβαιώσει ότι οι fixed poses είναι collision-free,
ότι ο target βρίσκεται εντός FOV και ότι ο detector δίνει αρκετές target
detections. Μετά το pilot, το manifest μπορεί να δηλώσει τα bins:

| Range bin (m) | Σκοπός |
| --- | --- |
| `[0.75, 1.25)` | measured near limit / approach |
| `[1.25, 2.00)` | existing-domain overlap |
| `[2.00, 3.00)` | v2 overlap/regression |
| `[3.00, 4.50)` | current failing scan case |
| `[4.50, 6.00)` | medium/far scan |
| `[6.00, 7.50)` | far detection feasibility |
| `[7.50, 9.00]` | pilot only — excluded, unreliable stock-YOLO detection |

Για κάθε declared range×quality bin απαιτούνται τουλάχιστον 30 accepted,
stationary, timestamp-matched target samples, όπως στο ενεργό C2 scenario.
Range bin που δεν συγκεντρώνει evidence δεν εντάσσεται στο artifact.

## Παραδοτέα

1. Νέο manifest, χωρίς αλλαγή του v2 manifest:
   `config/gazebo_covariance_c2_v3_trials.json`.
2. Raw evidence σε νέο directory (runtime, git-ignored):
   `runtime/c2_v3_geometry_fixed_coverage/`.
3. Coverage-only report πριν από οποιοδήποτε artifact write.
4. Νέο immutable artifact:
   `calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v3.json`.
5. Evidence και activation reports:
   `docs/gazebo-perception-covariance-c2-v3-artifact-report-el.md` και
   `docs/gazebo-perception-covariance-c2-v3-activation-report-el.md`.

## Activation gates

- Strict schema/hash/platform/identity validation.
- Coverage και target-outlier gates του C2 scenario.
- Conservative covariance ανά axis και ανά declared range×quality bin.
- Boundary tests ακριβώς κάτω/πάνω από το νέο range domain.
- Live `has_spatial=true` σε αντιπροσωπευτικό near, current-failure (`~4 m`)
  και far point, με matched RGB/depth timestamps.
- Live out-of-domain detection παραμένει `has_spatial=false`, χωρίς fallback.
- Collect-route με γνωστή μπάλα στη σωστή μισή παράγει non-empty snapshot/plan.
- Empty πραγματική σκηνή εξακολουθεί να παράγει `completed_no_targets`.

## Απόφαση ενεργοποίησης

Τα coverage, outlier, conservation, identity και boundary gates πέρασαν. Το
`sim.launch.py` και το collection-route runtime φορτώνουν πλέον αποκλειστικά το
v3 artifact. Το live scan παρήγαγε 30 accepted observations, 13 tracks και 10
confirmed snapshot targets. Ο planner τα χαρακτήρισε μη εφικτά (`keepout` /
`no_candidate_found`), ξεχωριστό planning/geometry θέμα και όχι perception
failure. Το άνω bin 7.5–9 m αποκλείστηκε ρητά.
