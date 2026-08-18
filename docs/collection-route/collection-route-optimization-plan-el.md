# Βελτιστοποίηση πλήρους collection route

## Σκοπός

Το ζητούμενο δεν είναι απλώς «η μικρότερη γραμμή». Είναι η συντομότερη
**εκτελέσιμη και πλήρης** αποστολή:

1. καλύπτει κάθε επιβεβαιωμένη και μη πραγματικά unreachable μπάλα,
2. δεν παραβιάζει το πραγματικό swept envelope του robot/intake,
3. περνά κάθε μπάλα μέσα από το μετρημένο capture corridor,
4. είναι κινηματικά και δυναμικά εκτελέσιμη χωρίς tracking abort,
5. και μόνο τότε ελαχιστοποιεί χρόνο, μήκος και περιττές στροφές.

Δεν αλλάζουμε runtime tuning από αυτό το έγγραφο. Πρώτα ενοποιούμε τα φυσικά
όρια, μετά βελτιώνουμε τον planner και τέλος κάνουμε controlled execution.

## Σημερινή objective semantics

Ο solver ήδη βαθμολογεί λεξικογραφικά:

```text
μέγιστος αριθμός covered balls
→ ελάχιστο weighted cost
→ λιγότερα passes
→ deterministic id
```

Άρα δεν επιλέγει συνειδητά μικρότερη αλλά ελλιπή route. Τα `route_conflict`
σημαίνουν ότι το candidate/connector graph δεν περιέχει πλήρη συνδεδεμένη
διαδρομή μέσα στους σημερινούς κανόνες.

Το δευτερεύον weighted cost δεν είναι ακόμη παραγωγικός ορισμός «γρηγορότερης
αποστολής». Προσθέτει με ίσα βάρη μέτρα, δευτερόλεπτα, ακτίνια, energy proxy
και αριθμό passes, ενώ length/time/energy ξαναχρεώνουν την ίδια απόσταση.

## Μετρημένο constraint bottleneck

Frozen pristine scan:

`runtime/route_audit/clean_reverted_20260728_1324/collection-scan-28672000000.json`

- 10/10 targets είναι γεωμετρικά reachable.
- Κάθε target έχει 16 valid single-ball approach candidates.
- 160 single + 8 shared candidates, χωρίς candidate-budget exhaustion.
- 112.896 directed CSC connector edges:
  - 100.981 turning rejected (`89,45%`),
  - 9.741 length rejected (`8,63%`),
  - 2.174 accepted (`1,93%`),
  - 0 collision rejected μετά τα analytic gates.
- Τελικό baseline: 8/10, `40,795885 m`, search complete.

Μονοπαραμετρικό frozen replay:

| Αλλαγή | Coverage | Μήκος | Search | Συμπέρασμα |
|---|---:|---:|---|---|
| baseline radius `1,25 m` | 8/10 | 40,796 m | complete | σημερινό |
| planning radius `1,00 m` | 9/10 | 44,194 m | exhausted | turning connectivity |
| planning radius `0,80 m` | 10/10 | 51,943 m | exhausted | full graph υπάρχει |
| connector arcs `1,5 → π rad` | 10/10 | 52,939 m | exhausted | arc caps επίσης κόβουν connectivity |
| max connector length `20 → 40 m` | 8/10 | 40,796 m | complete | δεν είναι ο ενεργός κόφτης |
| heading samples `16 → 32` | 9/10 | 49,320 m | exhausted | βοηθά, αλλά αυξάνει graph/search |
| clearance circle `0,50 → 0,45 m` | 8/10 | 40,796 m | complete | δεν εξηγεί τα δύο conflicts |
| length-only cost | 8/10 | 40,796 m | complete | το objective δεν προκαλεί την απώλεια |

Δεύτερο frozen scan
`clean_current_20260728_1315/collection-scan-39264000000.json`, με run-in
επαναφερμένο στο `1,0 m`:

| Planning radius | Coverage | Μήκος | Search |
|---|---:|---:|---|
| `1,25 m` | 9/10 | 63,472 m | complete |
| `1,00 m` | 10/10 | 47,026 m | complete |
| `0,80 m` | 10/10 | 46,429 m | exhausted |

Το κύριο σημερινό εμπόδιο είναι επομένως το forward connector model και τα
turning gates. Δεν δικαιολογείται τυφλή μείωση radius: πρώτα χρειάζεται ενιαίο
και μετρημένο κινηματικό contract.

## Constraint register

### A. Φυσικοί κανόνες — δεν χαλαρώνουν χωρίς μέτρηση

| Κανόνας | Σημερινή αναπαράσταση | Πρόβλημα / απαιτούμενη απόδειξη |
|---|---|---|
| Robot envelope | planner circle `0,50 m` | Πρέπει να περιλαμβάνει chassis, τροχούς και intake σε κάθε yaw. |
| Chassis | URDF `0,92 × 0,58 m` | Το route config δηλώνει παλιό `0,80 × 0,60 m`. |
| Intake | mouth tips περίπου `x=0,876 m`, width `0,410 m` | Δεν περιλαμβάνεται στο Nav2 rectangle `±0,46 × ±0,29 m`. |
| Capture corridor | nominal half-width `0,17 m` | Με ball radius, tracking budget και margin μένουν περίπου `0,047 m` πριν από residual covariance. Πρέπει να βαθμονομηθεί από intake trials. |
| Stable turn | άγνωστο ως συνάρτηση ταχύτητας/εδάφους | Χρειάζεται measured `κ_max(v)` με pose/tracking/slip, όχι μία αυθαίρετη σταθερά. |
| Fence/net/fixtures | surveyed polygons + keepout | Παραμένουν hard collision constraints. |
| Capture approach | forward straight pass, base-frame run-in `1,0 m`, run-out `0,3 m` | Το run-in δεν είναι 1 m ελεύθερης προσέγγισης για το intake: το mouth είναι περίπου `0,876 m` μπροστά από τη βάση. Πρέπει να οριστεί ως intake-frame alignment και να προκύψει από trials. |
| Safety/tracking | tube, heading, curvature, stop distance | Δεν χρησιμοποιούνται για να κρύβουν κακή route geometry. |

### B. Σημερινές planner πολιτικές — επιτρέπεται redesign

| Κανόνας | Τρέχουσα τιμή/συμπεριφορά | Επίδραση |
|---|---|---|
| Planning turn radius | `1,25 m` (`κ=0,8 1/m`) | Αποδεδειγμένα κόβει full coverage στα frozen scans. |
| Connector family | forward CSC μόνο: LSL/RSR/LSR/RSL | Δεν υπάρχουν CCC, ελεγχόμενο pivot ή reverse transit. |
| Arc gates | κάθε arc `≤1,5 rad`, total turn `≤3,0 rad` | Κόβουν ασφαλείς αλλά μεγαλύτερες αλλαγές προσανατολισμού. |
| Heading discretization | 16 headings | Περιορίζει entry/exit connectivity· 32 βοηθούν αλλά μεγαλώνουν το graph. |
| Approach gate | ακριβώς `1,0 m` πίσω από τη μπάλα, πάνω στην sampled heading | Δεν βελτιστοποιούνται tangent/join point, τελικό straight length ή μικρό capture-safe lateral offset. |
| Candidate cap | 200 | Στα δύο captures δεν είναι exhausted, αλλά γίνεται όριο με περισσότερα headings. |
| Search cap | 3.000 expansions | Οι full-coverage παραλλαγές συχνά επιστρέφουν budget-exhausted, άρα δεν αποδεικνύουν shortest route. |
| Shared pass | έως 3 balls, ίδιο discrete heading, spacing `≥0,5 m` | Πολύ περιορισμένη συγχώνευση· μόνο 8 shared candidates στο μετρημένο scan. |
| Circle collision model | isotropic radius `0,50 m` | Υπερσυντηρητικό στα πλάγια, αλλά μικρότερο από το chassis corner radius `~0,544 m` και αγνοεί το εμπρός intake. |
| Terminal run-out | `0,5 m` επιπλέον | Προσθέτει σταθερό μήκος· δεν ευθύνεται για route conflicts. |
| Follow-up | έως 2 runs | Recovery policy, όχι λύση για planner που αφήνει feasible targets deferred. |
| Connector/crossing speed | και τα δύο `0,35 m/s` | Δεν αξιοποιείται γρηγορό transit μεταξύ capture passes. |

### C. Ασυνέπειες που πρέπει να κλείσουν πριν από tuning

1. **Τρεις curvature αλήθειες:**
   - planner: radius `1,25 m` → `0,8 1/m`,
   - δηλωμένο mechanical max: `1,25 1/m` → radius `0,8 m`,
   - execution profile hard gate: `2,5 1/m` → radius `0,4 m`.

   Το mechanical πεδίο δεν οδηγεί σήμερα τον planner/controller. Δεν είναι
   ασφαλές να θεωρήσουμε οποιαδήποτε από τις τρεις τιμές πραγματική χωρίς test.

2. **Τρεις envelope αλήθειες:**
   - route config `0,80 × 0,60 m`,
   - URDF/Nav2 chassis `0,92 × 0,58 m`,
   - πραγματικό intake μέχρι περίπου `x=0,876 m`.

3. **Το run-in χρησιμοποιεί λάθος νοητικό reference αν διαβάζεται ως
   απόσταση intake από την μπάλα.** Στο entry pose:

   ```text
   base-to-ball distance             = 1,000 m
   intake mouth offset from base     ≈ 0,876 m
   mouth-to-ball distance at entry   ≈ 0,124 m
   roller nip offset from base       ≈ 0,590 m
   nip-to-ball distance at entry     ≈ 0,410 m
   ```

   Άρα το `1,0 m` είναι μεγάλο για το connector graph, αλλά όχι κατ' ανάγκη
   μεγάλο ως πραγματικό straight alignment πριν αρχίσει η επαφή με το funnel.
   Το canonical contract πρέπει να ορίζει:

   ```text
   base-frame run-in =
       capture-reference offset από τη βάση
       + απαιτούμενη straight απόσταση πριν από την πρώτη επαφή
   ```

   Το capture reference (mouth, throat/entry beam ή roller nip) και η
   απαιτούμενη προ-επαφή θα επιλεγούν από intake trials, όχι από το map.
   Το route model πρέπει επίσης να ξεχωρίζει τα physical progress planes:
   `mouth contact → entry beam → confirmed beam → retained`, αντί να θεωρεί
   μοναδικό crossing τη στιγμή που το `base_footprint` περνά από τη θέση της
   μπάλας. Αυτό μπορεί να αφαιρέσει περιττό post-capture μήκος και να δώσει
   σωστότερα approach/terminal costs.

4. Τα `mechanical.robot_*`, `funnel_*`, `safety_margin_m`,
   `maximum_curvature_per_m`, `minimum_entry_m`, τα planning `cost_weight_*`
   και `maximum_planning_time_s` υπάρχουν στο configuration contract αλλά δεν
   επηρεάζουν τον σημερινό planner. Αυτό δημιουργεί ψευδή αίσθηση ασφάλειας.

5. Το `maximum_planning_time_s=1,0` δεν είναι wall-clock stop. Τα πραγματικά
   bounds είναι candidate/search counts.

## Σχεδιαστικό πλάνο

### Φάση 0 — Planner observability και frozen corpus

- Διατηρούμε planner-boundary artifacts για επαναλήψιμο offline replay.
- Προσθέτουμε ανά run:
  - candidate counts ανά ball/heading,
  - edge rejection histogram και source/target,
  - search expansions/prunes/incumbent coverage,
  - objective breakdown (time, length, turn, pass count),
  - lower bound και optimality/search-complete status.
- Corpus: τα δύο σημερινά pristine scans και layouts για fence, net, corner,
  clusters, aligned balls και adversarial headings.

**Gate:** κάθε αλλαγή συγκρίνεται πάνω στο ίδιο corpus χωρίς κίνηση robot.

### Φάση 1 — Ενιαίο physical envelope και curvature contract

- Εξάγουμε μία canonical 2D polygon από URDF + intake envelope.
- Χρησιμοποιούμε orientation-aware swept polygon/capsules αντί για έναν κύκλο.
- Μετράμε σε Gazebo και αργότερα στο πραγματικό robot:
  - ελάχιστο σταθερό radius ανά speed,
  - slip, lateral/heading error και stopping envelope,
  - connector transition curvature.
- Ορίζουμε ένα speed-dependent `κ_safe(v)` με σαφές margin.
- Planner, execution context, controller και Nav2 footprint παράγονται από την
  ίδια πηγή.

**Gate:** καμία route δεν είναι planner-safe αλλά controller-invalid ή
αντίστροφα.

### Φάση 2 — Connector model με πλήρη ασφαλή συνδεσιμότητα

- Πρώτο controlled βήμα: αξιολόγηση radius `1,0 m` με το νέο physical contract.
- Αν επιβεβαιωθεί, αξιολόγηση μέχρι το measured safe radius, όχι απευθείας
  config tuning.
- Αντικατάσταση των αυθαίρετων arc caps με:
  - collision-free swept path,
  - curvature/speed feasibility,
  - bounded total path cost.
- Εξέταση CCC Dubins για κοντινά poses.
- Pivot/reverse επιτρέπονται μόνο ως **transit connectors**, ποτέ μέσα σε
  funnel pass, και μόνο αν αποδειχθούν ασφαλή για skid-steer και intake.
- Adaptive heading candidates γύρω από χρήσιμες tangents/γειτονικούς στόχους
  αντί για καθολικό διπλασιασμό 16→32.

**Gate:** full connected route για κάθε physically reachable target του frozen
corpus, χωρίς search-budget exhaustion.

#### Adaptive approach manifold

Η σημερινή γεωμετρία είναι:

```text
previous pass exit
→ CSC curved connector
→ fixed entry pose ακριβώς 1,0 m πίσω από τη μπάλα
→ straight crossing από το κέντρο της μπάλας
→ fixed run-out
```

Το προτεινόμενο μοντέλο δημιουργεί μικρό, bounded σύνολο εναλλακτικών:

```text
previous pass exit
→ curvature-continuous transit
→ variable approach/tangent gate
→ υποχρεωτικό straight alignment corridor
→ capture-safe crossing
→ run-out
```

Για κάθε ball/heading επιτρέπεται να βελτιστοποιούνται:

- η απόσταση του approach gate από τη μπάλα μέσα σε βαθμονομημένο
  `[minimum_base_run_in, maximum_useful_base_run_in]`, όπου το minimum
  περιλαμβάνει υποχρεωτικά το intake reference offset,
- το σημείο όπου η καμπύλη γίνεται εφαπτόμενη στην τελική ευθεία,
- μικρό lateral centerline offset μόνο μέσα στο πραγματικό effective capture
  half-width,
- adaptive headings που προκύπτουν από τον προηγούμενο/επόμενο στόχο και τα
  εμπόδια.

Ένα πιο μακρινό approach point μπορεί πράγματι να δώσει μικρότερη συνολική
route: προσθέτει λίγο straight length, αλλά μπορεί να αφαιρέσει μεγάλη Dubins
παράκαμψη ή δύο ακριβά τόξα και να ξεκλειδώσει ασφαλή σύνδεση. Δεν είναι όμως
πάντα καλύτερο. Το προηγούμενο **global** run-in `1,0 → 1,9 m` μείωσε coverage,
επειδή υποχρέωσε κάθε candidate να χρησιμοποιεί τη μακρινή είσοδο.

Η σωστή αλλαγή είναι να διατηρούμε το αποδεδειγμένο `1,0 m` candidate και να
προσθέτουμε επιλεκτικά εναλλακτικά gates. Έτσι το geometric candidate set δεν
χάνει την παλιά λύση. Για να μην επιστρέψει το candidate/search explosion:

- κρατάμε μόνο Pareto candidates ως προς coverage, total connector+pass cost,
  minimum clearance και capture margin,
- θέτουμε μικρό per-ball/per-heading cap,
- η τελική επιλογή γίνεται από τον global solver, όχι greedily ανά μπάλα,
- ολόκληρο curve + straight corridor ελέγχεται με το canonical swept envelope.

Το τελικό τμήμα πριν από τη μπάλα παραμένει ευθύ και aligned. Δεν επιτρέπουμε
καμπύλη μέσα στο ελάχιστο capture alignment corridor, επειδή τότε το intake
θα είχε lateral/yaw velocity πάνω στη crossing και το θεωρητικό πλεονέκτημα
διαδρομής θα γινόταν πραγματικό miss.

### Φάση 3 — Search και objective για πραγματικά shortest complete route

Νέα λεξικογραφική score:

```text
uncovered reachable targets
→ predicted mission time
→ route length
→ capture/tracking risk
→ turn effort
→ deterministic tie-break
```

- Το predicted time χρησιμοποιεί διαφορετικό speed profile για connector,
  turn και funnel pass.
- Τα hard safety/capture constraints δεν γίνονται soft weights.
- Καταργούνται διπλές/ανενεργές cost ρυθμίσεις.
- Το search επιστρέφει:
  - `optimal_on_graph`, ή
  - ρητό optimality gap/budget exhaustion.
- Coverage-first incumbent και admissible lower bounds προσαρμόζονται στο
  time objective ώστε full coverage να μη σημαίνει αυθαίρετα μακρύ route.

### Φάση 4 — Μείωση passes με capture-aware grouping

- Shared pass generation από continuous corridor fitting, όχι μόνο από ακριβώς
  ίδιο sampled heading.
- Βελτιστοποίηση κοινής centerline για 2+ μπάλες μέσα στο μετρημένο effective
  capture width.
- Clustering/order κατά προβολή πάνω στη γραμμή και έλεγχος πραγματικού spacing.
- Δεν αυξάνουμε τεχνητά capture width για να κερδίσουμε route length.

**Gate:** κάθε shared pass περνά intake acceptance σε lateral offsets και
διαδοχικές μπάλες.

### Φάση 5 — Γρήγορη αλλά ασφαλής εκτέλεση

- Capture passes κρατούν τη βαθμονομημένη intake speed.
- Connectors αποκτούν υψηλότερο nominal speed όπου curvature/clearance το
  επιτρέπουν.
- Speed profile μειώνεται πριν από crossing και sharp turns, όχι με ομοιόμορφο
  `0,35 m/s` σε όλη τη διαδρομή.
- Η predicted duration του planner συγκρίνεται με το measured elapsed time.

## Acceptance metrics

Για κάθε frozen και Gazebo scenario:

- `100%` coverage όλων των physically reachable targets.
- `search_status=complete` για τα acceptance layouts.
- Καμία collision, reverse/rotate ή capture-corridor παραβίαση.
- Καμία `heading/curvature/tube/non_monotonic_progress` διακοπή.
- Route length και predicted time όχι χειρότερα από το προηγούμενο
  full-coverage baseline του ίδιου artifact.
- Αναφορά:
  - total/connector/pass length,
  - predicted/measured duration,
  - turns και passes,
  - balls per pass,
  - minimum clearance,
  - capture lateral error,
  - planning wall time και expansions.
- Τρία συνεχόμενα pristine distributed runs ολοκληρώνουν route και intake
  reconciliation χωρίς restart PC/Pi.

## Προτεινόμενη άμεση εργασία

Δεν αλλάζουμε τώρα radius ή arc caps στο production config. Το επόμενο μικρό,
αναστρέψιμο βήμα είναι η **Φάση 0 + Φάση 1**:

1. edge/search diagnostics στο υπάρχον opt-in audit,
2. canonical footprint/envelope report από URDF,
3. curvature sweep ανά speed,
4. offline prototype για adaptive approach gates/lateral offsets πάνω στα δύο
   frozen scans,
5. offline comparison `R=1,25 / 1,00 / measured-safe`,
6. μόνο μετά controlled no-intake route execution.
