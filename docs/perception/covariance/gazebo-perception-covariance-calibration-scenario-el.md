# C2 scenario: Gazebo perception measurement covariance calibration

## Σκοπός και όρια

Το scenario αυτό παράγει **evidence**, όχι downstream spatial target. Κατά την
εκτέλεσή του ο κανονικός producer παραμένει C1-unhealthy και δεν δημοσιεύει
`has_spatial=true`. Ο recorder εκτελεί το ίδιο neural-detector + depth-fusion
path πάνω στα Gazebo RGB/depth acquisitions και καταγράφει τις raw optical-XYZ
μετρήσεις αποκλειστικά για calibration.

Η ground truth πηγή είναι το `/sim/balls` του `gazebo_extras_node`. Για κάθε
sample, η odom-anchored θέση (`x`, `y`, `z`) του ball μετασχηματίζεται στο
`camera_link_optical_frame` με το TF του ίδιου RGB timestamp. Δεν
χρησιμοποιείται current pose, latest TF ή projected 2D ground truth.

## Σταθερό scenario

- World: `gazebo/worlds/tennis_court.sdf`.
- Ball ground truth: η odom-anchored θέση `x`, `y`, `z` του `ball_02` από
  `/sim/balls`, με required reference `/sim/balls:ball_02`.
- Camera / robot condition: robot stationary σε κάθε trial, level ground,
  yaw τέτοιο ώστε ο επιλεγμένος ball να βρίσκεται εντός του RGB FOV. Δεν
  συνδυάζονται samples ενώ ο robot ή ο ball κινείται.
- RGB/depth: timestamp-matched pair από τα canonical `/camera/image_raw` και
  `/camera/depth`; το RGB timestamp είναι το sample timestamp.
- Matching: η detection αντιστοιχίζεται μόνο στο nearest visible GT ball
  στο optical frame, με 3D gate `<= 0.20 m`. Μη αντιστοιχισμένες detections
  δεν είναι calibration samples.

## Sampling grid και sample policy

Το evidence manifest δηλώνει κάθε trial, robot pose και GT ball. Το planned
grid είναι:

| Dimension | Bins |
| --- | --- |
| optical range (m) | `[0.75, 1.25)`, `[1.25, 2.00)`, `[2.00, 3.00]` |
| depth quality | q90: `[0.85, 0.95)`, q98: `[0.95, 1.00)`, q100/native: `{1.00}` |
| lateral bearing | `[-20°, -7°)`, `[-7°, 7°]`, `(7°, 20°]` |

`depth_quality` ορίζεται μετρήσιμα ως το fraction των finite, positive depth
pixels μέσα στο ίδιο depth ROI που χρησιμοποιεί η fusion. Το range του είναι
`[0, 1]`; η calibration δεν θα επεκταθεί σε bin/domain χωρίς επαρκή samples.

Απαιτούνται τουλάχιστον 30 accepted, stationary samples **ανά κάθε** declared
range×quality bin. Το `config/gazebo_covariance_c2_trials.json` ορίζει τα εννέα
fixed robot-pose trials γύρω από το `ball_02`; το 6.49 m/3-sample result δεν
είναι evidence και δεν μπορεί να επεκτείνει domain.

Το Gazebo depth είναι συνήθως πλήρες (`quality=1.0`). Για το low-quality bin,
ο C2 recorder μόνο αντιγράφει το depth frame και γράφει `NaN` σε deterministic
subset των valid pixels του **ίδιου fusion ROI**: 10% (manifest seed) για
`[0.85,0.95)` και 2% για `[0.95,1.00]`. Το original camera frame δεν
μεταβάλλεται και αυτό δεν είναι producer feature, fallback ή runtime model.
Η quality μετράται ξανά με `depth_roi_quality`, δηλαδή το κοινό metric που θα
χρησιμοποιεί ο producer. Κάθε evidence row καταγράφει ratio, seed και trial ID.

Το q100/native χρησιμοποιεί `missing_pixel_ratio=0.0`: δεν εφαρμόζεται mask ή
άλλη input αλλαγή. Η ίδια `depth_roi_quality` πρέπει να καταγράψει ακριβώς
`1.0`; διαφορετική τιμή αποτυγχάνει το declared `{1.00}` bin και δεν είναι
native-q100 evidence.

Τα q90 και q98 είναι supported calibrated domains του artifact, όχι native
live Gazebo producer modes: το native depth frame έχει quality `1.0`. Η
κάλυψή τους διατηρείται από controlled calibration evidence και pure
producer-model adapter tests με deterministic quality inputs απευθείας στο
adapter boundary. Δεν υπάρχει ROS/Gazebo runtime injection ή mutation του
native depth για activation verification.

## Outlier και acceptance policy

Ένα target sample απορρίπτεται μόνο με καταγεγραμμένο reason: missing/invalid
depth, missing TF at RGB timestamp, non-stationary pose ή target-association
rejection. Detection που συσχετίζεται με άλλη μπάλα καταγράφεται ως
non-target και δεν επηρεάζει target residual/outlier metrics. Ambiguous ή
unmatched detection καταγράφεται χωριστά ως association rejection και δεν
γίνεται ούτε target sample ούτε target outlier. Δεν απορρίπτονται residuals
για να μειωθεί τεχνητά η covariance.

Πριν από fitter/artifact generation, `assert_c2_coverage` ελέγχει κάθε
range×quality bin, ≥30 accepted samples, `/sim/balls:ball_02`, και
`target_outlier_rate = target_outliers / target_samples` ≤ 0.35. Ο
numerator/denominator προέρχεται αποκλειστικά από associated target samples.
Η coverage report command είναι report-only και δεν γράφει artifact.

Για κάθε axis, οι fitted non-negative coefficients και floor προέρχονται από
τα observed squared residuals και στρογγυλοποιούνται προς τα πάνω στο επόμενο
`1e-6 m²`. Τα coefficients του `range_depth_quality_diagonal_v1`
επιτρέπονται μόνο αν το προκύπτον μοντέλο είναι >= κάθε observed squared error
ανά axis σε κάθε declared range×quality bin. Το evidence report καταγράφει
sample counts, association metrics, max/RMSE axis error και το per-bin
conservation check. Αποτυχία οποιουδήποτε check αποτυγχάνει το C2 artifact
gate.

## Artifact activation

Μόνο artifact που αναφέρει αυτό το scenario/evidence report, περνά strict
parser validation και περνά το conservation check μπορεί να δοθεί στο Gazebo
producer. Τότε και μόνο τότε επιτρέπονται `spatial_targets_healthy=true` και
`has_spatial=true` για in-domain, sufficient-quality detections. Εκτός domain
ή όταν η quality είναι ανεπαρκής, ο producer παραμένει non-spatial χωρίς
fallback covariance.
