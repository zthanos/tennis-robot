# Intake roller/lip debugging — log

Σκοπός: αρχείο-καταγραφή για κάθε ενέργεια/διόρθωση γύρω από το intake
(roller, lip, channel, debug camera) ώστε να μην ξαναγυρνάμε στον ίδιο κύκλο.
Νέα εγγραφή ανά αλλαγή: τι δοκιμάστηκε, τι αποτέλεσμα είχε, τι επόμενο βήμα.

## 2026-07-08

### 1. Root cause: channel geometry δεν ακολουθούσε τα intake offsets
- **Βρέθηκε**: `scripts/generate_curved_scoop_mesh.py` και το `_patch_sdf_contacts`
  στο `scripts/generate_robot_urdf.py` είχαν hardcoded `ROLLER_X=0.615`,
  `ROLLER_Z=0.112` — αγνοούσαν τα `INTAKE_ROLLER_X_OFFSET_M` /
  `INTAKE_ROLLER_Z_OFFSET_M` που μετακινούν τον πραγματικό roller μέσω
  `tennis_robot.urdf.xacro`. Επαληθεύτηκε αριθμητικά μέσα από το γεννημένο
  SDF: πριν τη διόρθωση, roller στο `x=0.615,z=0.107` αλλά το channel arc
  center στο `x=0.600,z=0.112` (mismatch 15mm X / 5mm Z).
- **Fix**: και τα δύο scripts διαβάζουν τώρα τα ίδια env vars και μετατοπίζουν
  το channel/mesh μαζί με τον roller.
- **Επαλήθευση**: μετά τη διόρθωση, roller και channel center ταυτίζονται
  ακριβώς (`x=0.615,z=0.107` και τα δύο) στο γεννημένο SDF.
- **Status**: ✅ εφαρμόστηκε, επαληθεύτηκε αριθμητικά.

### 2. Πρώτο `collect_one` test μετά τη διόρθωση
- Στάλθηκε νέα εντολή collect_one (`runtime/robot_command.json`, sequence 60).
- Screenshot από τον χρήστη: μπάλα ακριβώς στο άκρο του intake, στη φάση
  `capture`.
- Λίγο μετά: `collector_state` γύρισε σε `approach`, `collection_count: 0` —
  η προσπάθεια capture ΔΕΝ πέτυχε ακόμα, παρά τη διόρθωση γεωμετρίας.
- **Status**: ⚠️ ανοιχτό — χρειάζεται οπτική/telemetry επιβεβαίωση γιατί
  αποτυγχάνει ακόμα το capture.

### 3. Debug camera (sim-only) — προσπάθεια οπτικής επιβεβαίωσης
- Προστέθηκε `intake_debug_camera.urdf.xacro`: κάμερα mounted στο
  `base_link` (x=0.45, z=-0.03 rel. δηλ. ~15mm πάνω από το έδαφος), pitch
  -0.51 rad (~29° πάνω), gated πίσω από `sim_mode` (μόνο sim). Bridge added
  σε `gazebo/bridge_config.yaml` (`/gz/intake_debug_camera` →
  `/camera/intake_debug/image_raw`).
- **Attempt 1**: πρώτο frame εντελώς μαύρο (min=max=mean=0). Front/oak-d
  camera δούλευε κανονικά (δοκιμάστηκε για σύγκριση) — άρα δεν είναι γενικό
  πρόβλημα rendering pipeline.
- **Υπόθεση A**: η κάμερα κοιτάζει τη σκιασμένη κάτω πλευρά του σασί (ο
  ήλιος της σκηνής δεν φτάνει εκεί) → **Fix**: προστέθηκε spot light
  (`intake_debug_camera_link_spotlight`) στο ίδιο link, ίδια θέση/κατεύθυνση
  με την κάμερα. **Αποτέλεσμα: ΑΚΟΜΑ μαύρο** (frame3.png, min=max=mean=0).
- **Υπόθεση B**: το `<direction>` του spot light στο gz-sim ΔΕΝ περιστρέφεται
  μαζί με το `<pose>` του (γνωστό quirk) — δηλ. direction="1 0 0" σήμαινε
  οριζόντια δέσμη, όχι προς τα πάνω. **Fix**: explicit world-frame direction
  vector (`0.8736 0 0.4870`, υπολογισμένο ώστε να δείχνει ακριβώς στον
  roller target). **Αποτέλεσμα: ΑΚΟΜΑ μαύρο** (frame4.png, min=max=mean=0).
- **Status**: ❌ ανοιχτό μετά από 2 αποτυχημένες υποθέσεις/διορθώσεις.
  Frame παραμένει *ακριβώς* μηδέν (όχι απλά σκοτεινό) σε κάθε προσπάθεια —
  ένδειξη ότι μπορεί να μην είναι θέμα φωτισμού αλλά κάτι πιο δομικό (π.χ.
  τίποτα μέσα στο frustum πριν το far clip, near-clip clipping μέσα σε στερεό
  σώμα, ή κάποιο πρόβλημα στο ίδιο sensor config).
- **Attempt 3 (depth diagnostic)**: προστέθηκε companion depth camera στο ίδιο
  link/γωνία (`/gz/intake_debug_depth` → `/camera/intake_debug/depth`). Το
  depth δεν εξαρτάται από φωτισμό. **Αποτέλεσμα (καθοριστικό)**: και τα 307200
  pixels διάβασαν **0.080–0.087 m** — δηλαδή ένα επίπεδο στερεό ~8cm μπροστά
  γεμίζει ΟΛΟ το κάδρο. Άρα το μαύρο RGB **ΔΕΝ ήταν ποτέ θέμα φωτισμού** — η
  κάμερα ήταν **μπλοκαρισμένη**, κοιτούσε μέσα στη γεωμετρία (πιθανότατα την
  κάτω/πίσω πλευρά του ίδιου του scoop channel) αντί για τον roller στα ~0.19m.
  → Τα δύο light fixes (#3 attempts 1-2) ήταν **λάθος διάγνωση**· δεν χρειάζονταν.
- **Fix (position + light)**:
  - Μετακίνηση κάμερας από head-on underside (`0.45 0 -0.03`) σε **πλάι-κάτω**
    (`base_link 0.5 0.13 -0.02`, ~25mm πάνω από έδαφος, μόλις μέσα από το
    funnel cheek στο y=0.145), με διαγώνια στόχευση inward-up-forward
    (`rpy 0 -0.441 -0.847`) ώστε η γραμμή θέασης να μπαίνει από το πλάι του
    channel όπου δεν υπάρχει εμπόδιο. Απόσταση στον roller 0.192m
    (υπολογισμένη, γραμμή θέασης ελεγμένη ότι δεν τέμνει cheek/chassis).
  - Το spot light έγινε **point light** (omnidirectional) — παρακάμπτει
    τελείως το gz-sim `<direction>` world-frame quirk.
- **Αποτέλεσμα (restart #8)**: ✅ **ΔΟΥΛΕΥΕΙ**. depth τώρα δομημένο
  (0.034–0.218m, 258802/307200 finite, median 0.099) — καθαρή θέαση, όχι
  μπλοκάρισμα. RGB (frame6.png) φωτισμένο (min 0 / max 255 / mean 58). Η
  εικόνα δείχνει καθαρά τον roller (σκούρος κύλινδρος αριστερά-κέντρο) + μια
  κίτρινη μπάλα (πάνω-αριστερά) + φράχτη στο φόντο.
- **Status**: ✅ camera λειτουργεί. **Εκκρεμεί βελτίωση framing**: ο roller
  γεμίζει το κάδρο (η κάμερα είναι ~4cm από το κοντινό άκρο του roller στο
  y=0.09), η μπάλα είναι στη γωνία, το δεξί μισό μαύρο. Για καθαρό "nip
  contact" shot θέλει είτε ευρύτερο FOV (1.0→~1.5) είτε λίγο πιο πίσω/έξω
  τραβηγμένη κάμερα ώστε να κεντραριστεί το σημείο επαφής ball-roller-lip.

### 4. Παράπλευρο fix: RViz markers ποτέ δεν εμφανίζονταν (ξεχωριστό, real bug)
- **Βρέθηκε**: `gazebo_extras_node.py`'s `_on_pose_info` έθετε `self._robot_pose`
  μόνο όταν `child_frame_id.split("::")[-1] == "tennis_robot"` — αλλά το
  `/gz/pose_info` bridge δίνει `child_frame_id=''` για ΚΑΘΕ transform σε αυτή
  την έκδοση του Gazebo (επιβεβαιώθηκε ήδη νωρίτερα ίδια session). Άρα
  `self._robot_pose` έμενε πάντα `None`, και οι publishers
  `/sim/roller_contact_markers` + `/sim/ball_markers` έκαναν σιωπηλά no-op —
  το `RollerContacts` display στο RViz (ήδη υπήρχε στο config) ποτέ δεν
  έδειχνε τίποτα.
- **Fix**: robot pose τώρα έρχεται από `/odom` (nav_msgs/Odometry) αντί για
  το σπασμένο pose_info name-matching.
- **Status**: ✅ εφαρμόστηκε. **Δεν έχει επαληθευτεί ακόμα** ότι τα markers
  όντως εμφανίζονται τώρα στο RViz (δεν έγινε ακόμα δοκιμή με πραγματική
  επαφή μετά τη διόρθωση).

### 5. RViz config
- Προστέθηκε `Image` display για `/camera/intake_debug/image_raw` στο
  `docker/ros2/tennis_robot.rviz` (`IntakeDebugCamera`). Επιβεβαιώθηκε ότι
  εμφανίζεται στο panel (screenshot χρήστη) — αλλά δείχνει μαύρο γιατί η ίδια
  η κάμερα ακόμα δεν βγάζει εικόνα (βλ. #3).

### 6. Framing refinement της debug camera
- **Πρόβλημα**: στο frame6.png ο roller γεμίζει το κάδρο (κάμερα ~4cm από το
  κοντινό του άκρο), η μπάλα πέφτει στη γωνία, το δεξί μισό μαύρο — δεν
  φαίνεται καθαρά το σημείο επαφής ball/roller/lip.
- **Αλλαγή** (θέση ΔΕΝ αλλάζει — είναι επιβεβαιωμένα καθαρή):
  - FOV 1.0 → 1.5 rad (~86°) και στα δύο sensors (RGB + depth).
  - Στόχευση από roller centre (0.615,0,0.107) → **nip midpoint**
    (0.607,0,0.075): `rpy 0 -0.441 -0.847` → `0 -0.289 -0.882`
    (dist 0.176m).
- **Αποτέλεσμα (restart #9)**: ✅ frame7.png δείχνει πλήρες πλάνο: roller
  (πάνω-αριστερά), scoop channel σε σιλουέτα με το lip να ακουμπά το έδαφος,
  court + γραμμή στο φόντο, front-left τροχός. Depth 0.028–1.702m
  (δομημένο, με ορίζοντα). Το nip visible. Η κάμερα-εργαλείο είναι έτοιμη.
- **Status**: ✅ ολοκληρώθηκε.

### 7. Rosbag capture-failure test (instrumented collect_one)
- **Στόχος**: ποσοτική απάντηση στο "αγγίζει η μπάλα τον roller;" — timeline
  απόστασης μπάλας-roller + contact events + joint vel + beam + camera frames.
- **Σημείωση**: το `ros2 bag play` δεν ξανατρέχει φυσική· το test είναι
  record → offline ανάλυση.
- **Εύρημα κατά το setup**: το raw gz `/world/tennis_court/pose/info` ΕΧΕΙ
  entity names — το ros_gz bridge (Pose_V→TFMessage) είναι που τα πετάει
  (child_frame_id=''). Ground truth λοιπόν μέσω `gz topic --json-output` CLI.
- **Εύρημα #2**: μετά το restart #9 ο controller μπήκε ΜΟΝΟΣ του σε
  collect_one (το robot_command.json είχε μείνει σε collect_one) και η
  ball_02 βρέθηκε σπρωγμένη από (-6.4,0) στο (-0.06,0.00) χωρίς capture —
  live αναπαραγωγή του συμπτώματος. → Πριν από restart τεστ, γράφε idle.
- **Εργαλεία** (scripts/sim_debug/): `log_gz_poses.py` (ground-truth
  JSONL ~20Hz), `analyze_collect_bag.py` (bag+JSONL → closest approach,
  contact count, camera frames γύρω από το κρίσιμο σημείο).
- **Bag topics**: /gz/roller_contact_0, /joint_states,
  /collector/intake_beam_broken, /odom, /cmd_vel, /sim/roller_contact,
  /camera/intake_debug/image_raw.
- **Αποτελέσματα (run: collect_test1, bags στο runtime/bags/)**:
  - Ροή: idle → approach (19s) → capture (6s) → reverse_clear, **0 collected**.
  - **Η μπάλα ΑΓΓΙΖΕΙ τον roller**: 496 ball-contact messages στο
    `/gz/roller_contact_0` επί **2.24s συνεχόμενα**. Η υπόθεση "δεν κάνει
    επαφή" **καταρρίπτεται** από τα δεδομένα.
  - Closest approach: κέντρο-κέντρο 69mm → surface gap **-9mm**
    (διείσδυση — συμπίεση από τον solver). Ball κεντραρισμένη τέλεια
    (lateral **0mm**), κέντρο +25mm μπροστά από τον άξονα roller, στο έδαφος
    (z=43mm = rest height).
  - Beam broken 73 δείγματα (η μπάλα στο nip), roller |vel| max 30 rad/s
    (γύριζε κανονικά).
  - Frames: η μπάλα κάθεται στο lip, πιεσμένη στο μπροστινό-κάτω τεταρτημόριο
    του roller, και μένει εκεί — **σφηνωμένη στο λαιμό**, μέχρι που ο robot
    κάνει reverse_clear.
- **Νέα διάγνωση (root cause candidate): ο λαιμός είναι στενότερος από τη
  μπάλα για rigid bodies.**
  - Κάτω διάκενο roller-έδαφος: 107-45 = **62mm** < 66mm μπάλα (το probe το
    τύπωνε ήδη: "ball fit margin=-4.0mm").
  - Το tuning `INTAKE_ROLLER_Z_OFFSET_M=-0.005` ("χαμήλωσε τον roller") το
    προκάλεσε: στο baseline z=112 το διάκενο ήταν 67mm (+1mm ελεύθερο).
  - Κανάλι-roller: 108-45 = 63mm σχεδιαστικά ("3mm nominal interference,
    μοντελοποιεί συμπίεση αφρού") — αλλά o DART με rigid μπάλα ΔΕΝ συμπιέζει:
    η μπάλα απλά δεν χωράει, σφηνώνει (γι' αυτό το 9mm penetration spike), ο
    solver τη σπρώχνει έξω.
  - Το lip raise +2mm στενεύει την είσοδο επιπλέον.
- **Status**: ✅ διάγνωση ολοκληρώθηκε. Επόμενο: A/B rerun με ανοιχτό λαιμό.

### 8. A/B test: άνοιγμα λαιμού (undo roller lowering)
- **Αλλαγή** (ΜΙΑ μεταβλητή): `INTAKE_ROLLER_Z_OFFSET_M=-0.005` → `0.0`
  (roller πίσω στο baseline z=112 → κάτω διάκενο 67mm, +1mm ελεύθερο για
  66mm μπάλα). Κρατάμε `INTAKE_ROLLER_X_OFFSET_M=0.015` και
  `INTAKE_LIP_RAISE_M=0.002` ως έχουν. Το κανάλι ακολουθεί αυτόματα τον
  roller (fix #1), άρα το arc μένει ομόκεντρο.
- **Διαδικασία**: ίδιο instrumented test (collect_test2) για άμεση σύγκριση
  με collect_test1.
- **Κριτήριο επιτυχίας**: μπάλα περνάει το nip (beam → basket beams,
  balls_collected>0), ή τουλάχιστον σαφώς βαθύτερη πρόοδος στο κανάλι αντί
  για static jam 2.2s + reverse_clear.
- **Αποτελέσματα (collect_test2)**: πάλι 0 collected, αλλά με ΑΝΤΙΘΕΤΟ
  μηχανισμό αποτυχίας:
  - test1 (z=-5mm, διάκενο 62mm): μπάλα σταμάτησε +25mm ΜΠΡΟΣΤΑ από τον
    άξονα, 496 contacts επί 2.2s → **σφηνωμένη** στο nip.
  - test2 (z=0, διάκενο 67mm): μπάλα έφτασε +5mm από τον άξονα (σχεδόν από
    κάτω του), beam 57 hits, **0 contacts** — πέρασε ΚΑΤΩ από τον roller με
    ~1mm αέρα **χωρίς να τον αγγίξει ποτέ** → μηδενική πρόσφυση, ο roller
    γύριζε στον αέρα.
  - Σημ.: το court surface είναι ~10mm πάνω από το world z=0 (η μπάλα
    "rest" z=43mm = 33mm ακτίνα + 10mm court) — τα per-frame "surface gap"
    του analyzer έχουν αυτό το offset· τα contact counts του sensor είναι η
    αλήθεια.
- **Συμπέρασμα (διπλό μπλόκο, και τα δύο αποδεδειγμένα)**:
  1. **Grip**: με rigid σώματα, interference = σφήνωμα (test1) και clearance
     = καμία πρόσφυση (test2). Δεν υπάρχει ενδιάμεσο σημείο για σταθερό
     fixed-axis rigid roller — χρειάζεται ΕΝΔΟΤΙΚΟΤΗΤΑ (όπως ο πραγματικός
     αφρός): είτε spring-suspended roller (prismatic joint + spring —
     υποστηρίζεται από DART) είτε soft-contact params (αβέβαιη υποστήριξη
     στο dartsim).
  2. **Πέρασμα**: ακόμα κι αν ο roller έπιανε τη μπάλα, το κανάλι-arc έχει
     διάκενο 108-45=63mm < 66mm μπάλα σε ΟΛΟ το μήκος του ("3mm nominal
     interference" by design) — η μπάλα δεν χωράει πουθενά στη διαδρομή.
     CHANNEL_R πρέπει ≥ 113mm (45+66+2mm slack).
- **Status**: ✅ διάγνωση πλήρης. Επόμενο: fix και των δύο + rerun.

### 9. Fix: spring-suspended roller + κανάλι 113mm (απόφαση χρήστη: μαζί)
- **Αλλαγή A (πέρασμα)**: CHANNEL_R 0.108 → **0.113** και στα δύο σημεία
  (generate_curved_scoop_mesh.py + _patch_sdf_contacts) → διάκενο
  roller-καναλιού 68mm, μπάλα 66mm περνά με 2mm slack.
- **Αλλαγή B (grip)**: ο roller κρεμιέται σε παθητικό prismatic z-joint
  (carriage) με ελατήριο (SDF `<dynamics><spring_stiffness>` — patch στο
  SDF αφού το URDF δεν έχει springs). Rest position με ~8mm interference
  (`INTAKE_ROLLER_Z_OFFSET_M=-0.009` → κέντρο 103mm, κάτω άκρο 58mm για
  μπάλα-top 66mm), ελατήριο k≈300 N/m ώστε η μπάλα να το σηκώνει
  συμπιέζοντας «αφρό» και να δέχεται συνεχή δύναμη πρόσφυσης χωρίς σφήνωμα.
  Travel: 0..+15mm (κάτω stop στο rest, δεν πέφτει από βαρύτητα).
- **Προσοχή**: το contact sensor δείχνει σε συγκεκριμένο lumped collision
  name — με τη νέα δομή (carriage) το όνομα μπορεί να αλλάξει· το patch
  γίνεται δυναμικό (βρίσκει το πραγματικό roller_col collision στο SDF)
  αλλιώς θα μετράμε σιωπηλά 0 contacts.
- **Σφάλμα διαδικασίας (διορθώθηκε)**: το πρώτο collect_test3 δεν έστειλε
  ποτέ collect_one — το runtime/robot_command.json ήταν root-owned (ο
  controller στο container το ξαναγράφει ως root) και το host-side write
  απέτυχε με EACCES. Ο orchestrator στέλνει πλέον την εντολή ΜΕΣΑ από το
  container. Μάθημα: εντολές στο robot_command.json πάντα μέσω
  `docker compose exec`.
- **Αποτελέσματα (collect_test3, spring+113)**: 0 collected ΑΛΛΑ τεράστια
  πρόοδος — **το grip δουλεύει**:
  - t=23.4s: ο roller ΑΡΠΑΞΕ τη μπάλα: fwd +30 → **-64mm** (πίσω από τον
    άξονα), z 43→65mm. Το ελατήριο υποχώρησε (~11mm) όπως σχεδιάστηκε.
  - t=24-35s: η μπάλα ΣΤΑΣΙΜΗ στο fwd≈-43mm, z=46 επί 11s — μέσα στο
    κανάλι, contact μόνο σποραδικό (193 msgs/13s ≈ 15%).
  - t=36s: reverse_clear την άφησε πίσω στο έδαφος.
- **Διάγνωση νεκρής ζώνης**: το z=-9mm κατέβασε το τόξο ώστε πίσω από τον
  άξονα να πέφτει κάτω από το floor clamp (3mm) → επίπεδη τσέπη με κενό
  10+mm από τον roller. Η μπάλα ισορροπεί στο dx=-43 με το κέντρο της
  1.6mm έξω από το grip radius (79.6 vs 78mm) — ο roller γυρνά από πάνω
  της χωρίς επαφή, το τόξο την ξανακυλά πίσω.
- **Refinement (test4)**: z offset -0.009 → **-0.002** (1mm entry bite,
  4mm under-axis squeeze) και CHANNEL_R 0.113 → **0.109** (το τόξο κρατά
  τη μπάλα κολλημένη στον roller πίσω από τον άξονα· max coast gap ~1mm).
  Το ελατήριο μετατρέπει το interference σε grip force αντί για σφήνωμα.
- **Αποτελέσματα (collect_test4, z=-0.002 / R=0.109)**: 0 collected, αλλά
  το τρίτο και βαθύτερο στάδιο μέχρι τώρα:
  - Grip άμεσο και **συνεχής επαφή** (2561 msgs επί 14.4s, vs 193 σποραδικά
    στο test3) — το refinement του τόξου δούλεψε.
  - Η μπάλα **σκαρφάλωσε όλο το τόξο**: fwd +13→-75mm, z 43→**113mm**
    (ύψος άξονα) σε 0.8s.
  - Στάσιμη στο (fwd -75, z 112) επί 13s: σφηνωμένη ανάμεσα σε roller και
    πίσω τοίχο, **γυρνούσε επιτόπου σαν ρουλεμάν** — roller μ=2.5 της δίνει
    spin, τοίχος μ=0.15 δεν το μετατρέπει σε αναρρίχηση (0.15×N ≈
    0.15-0.45N < βάρος 0.57N).

### 10. Fix: τριβή καναλιού 0.15 → 0.8 (test5)
- Το 0.15 ήταν σχεδίαση της rigid εποχής ("μη καρφώνει τη μπάλα") — με το
  sprung roller το pinch είναι πλέον feature.
- **Αποτέλεσμα (collect_test5)**: ΙΔΙΟ stall (fwd -75, z 112, 13s).
  Επαληθεύτηκε ότι το μ=0.8 ΗΤΑΝ live στο SDF — το fix μπήκε, απλά δεν
  έφτανε.

### 11. Διορθωμένο ισοζύγιο + fix: μ=1.4 και spring k=500 (test6)
- **Ποσοτικό ισοζύγιο στο stall point** (fwd -75, z 103, squeeze 2.6mm):
  - Κάτω: βάρος 0.57N + κάθετη συνιστώσα roller-normal 0.08N = 0.65N.
  - Πάνω (τριβή τοίχου, μπάλα-ρουλεμάν): μ_wall × N, με N = k×lift =
    300×2.6mm ≈ 0.78N.
  - μ=0.15 → 0.12N (κολλημένη ✓ ταιριάζει με test4)
  - μ=0.8 → 0.62N vs 0.65N (ΟΡΙΑΚΑ κολλημένη ✓ ταιριάζει με test5!)
  - Το μοντέλο προβλέπει σωστά και τα δύο αποτελέσματα.
- **Fix**: μ_wall 0.8→**1.4** (λάστιχο, όπως οι πραγματικοί συλλέκτες) ΚΑΙ
  spring k 300→**500** N/m → ανοδική ≈ 1.4×1.3 = 1.8N vs 0.65N (2.7×
  περιθώριο). Δύο αλλαγές μαζί σκόπιμα: πολλαπλασιαστικά μέλη του ίδιου
  όρου (μ×N), το ισοζύγιο είναι πλέον μετρημένο.
- **Αποτελέσματα (collect_test6b· το πρώτο test6 χάθηκε από restart-race
  των recorders — μπήκε uptime guard στον orchestrator)**: ΙΔΙΟ stall
  (fwd -75, z 113) και με μ=1.4 + k=500. Δύο διαψεύσεις → το μοντέλο
  «friction-αναρρίχηση» ήταν λάθος επίπεδο ανάλυσης.

### 12. Η πραγματική γεωμετρική αιτία του stall: νεκρή ζώνη #2 (test7)
- **Απόδειξη**: για να περάσει η μπάλα πρέπει το κέντρο της να φτάσει
  ~z=188 (πάνω από το χείλος τοίχου z=155). Η εμβέλεια του roller
  (78mm από άξονα) τελειώνει στο z≈130 στην τροχιά αναρρίχησης → 58mm
  ΧΩΡΙΣ καμία ώθηση. Το design υπέθετε βαλλιστική εκτόξευση, αλλά η
  quasi-static μεταφορά δίνει ~0.67 m/s → άνοδος μόνο 23mm. Καμία τριβή
  δεν γεφυρώνει 58mm κενό — γι' αυτό απέτυχαν το #10 και το #11.
- **Fix (δομικό, όπως οι πραγματικοί συλλέκτες)**: ο κατακόρυφος τοίχος
  αντικαταστάθηκε από **συνέχιση του ομόκεντρου τόξου γύρω από το πίσω
  μέρος του roller** μέχρι z=155 (guide angle έως asin((155-110)/109)≈24°).
  Με ομόκεντρο οδηγό ο roller οδηγεί τη μπάλα ΕΦΑΠΤΟΜΕΝΙΚΑ σε όλη τη
  διαδρομή (η επαφή διατηρείται by construction) και τη ρίχνει από την
  κορυφή με φορά πάνω-πίσω προς deflector/basket.
- Λεπτομέρεια: το underside του collision polygon στο guide τμήμα παίρνει
  ΑΚΤΙΝΙΚΟ offset (R+2mm) — κατακόρυφο offset θα στένευε το πέρασμα 2mm.
- Αρχεία: generate_curved_scoop_mesh.py (νέο profile με GUIDE_STEPS),
  generate_robot_urdf.py (νέο polyline top+underside).
- **Αποτελέσματα (collect_test7)**: ΙΔΙΟ stall (fwd -75, z 113, 26s) και με
  τον ομόκεντρο οδηγό. 4η διαμόρφωση με πανομοιότυπο σημείο καθηλώματος.

### 13. ΤΕΛΙΚΗ ρίζα: 1-DOF κατακόρυφη ανάρτηση ≠ ακτινική ενδοτικότητα
- **Γεωμετρική απόδειξη**: το ελατήριο κινεί τον roller μόνο κατακόρυφα,
  αλλά η μπάλα σε τροχιά γύρω του χρειάζεται ακτινική υποχώρηση ~2mm
  παντού (guide R=109 → πέρασμα 64mm < 66mm μπάλα). Απαιτούμενο vertical
  lift για 2mm ακτινικά: κάτω=2mm ✓· στο fwd -75 = √(78²-75²) =
  **21.4mm > 15mm travel** → μπλόκο ΑΚΡΙΒΩΣ εκεί που κολλάει κάθε test·
  πίσω-οριζόντια=17.7mm· πάνω από τον άξονα: αδύνατον (θα έπρεπε να
  κατέβει κάτω από το rest). Εξηγεί το πανομοιότυπο fwd=-75 των tests
  4/5/6b/7 ανεξαρτήτως τοίχου/τριβής/ελατηρίου/οδηγού.
- Ο αληθινός αφρός συμπιέζεται ΑΚΤΙΝΙΚΑ (υλική ενδοτικότητα), όχι σαν
  ανάρτηση άξονα.
- **Επόμενο fix (μεγάλο αλλά το πρώτο γεωμετρικά ικανό): paddle roller**
  — hub + 6 πτερύγια σε revolute joints με ελατήρια (SDF patch όπως το
  carriage). Κάθε πτερύγιο λυγίζει ακτινικά → πραγματική ενδοτικότητα σε
  όλη την τροχιά. Ιστορικό εύρημα: τα bridges roller_contact_0..7 και το
  σχόλιο "Real physics contacts from each intake paddle" δείχνουν ότι το
  ΑΡΧΙΚΟ design ήταν paddle wheel — η απλοποίηση σε συμπαγή κύλινδρο
  είναι που αχρήστεψε τη φυσική της σύλληψης.
- **Status**: εγκρίθηκε από χρήστη → υλοποίηση στο #14.

### 14. Υλοποίηση paddle roller (test8)
- **Δομή**: `lift_wheel_link` γίνεται hub (κύλινδρος r=25mm, το ros2_control
  joint `lift_wheel_joint` μένει ίδιο, ξανά απευθείας στο base_link) + 8
  πτερύγια (boxes 25×180×4mm) το καθένα σε δικό του revolute joint
  (±1.2 rad) με SDF-patched spring (k=0.25 N·m/rad, damping 0.02,
  reference 0). Tips στα 50mm — ελαφρώς μεγαλύτερο envelope από τον παλιό
  κύλινδρο (45), συμπιέσιμο ακτινικά έως ~hub.
- **Το κατακόρυφο carriage (#9) ΑΦΑΙΡΕΘΗΚΕ** — με ακτινική ενδοτικότητα
  ανά πτερύγιο είναι περιττό, και το travel-limit του ήταν το μπλόκο του #13.
- **Sensors**: ένα contact sensor ανά πτερύγιο → roller_contact_0..7 — τα
  8 bridges που υπήρχαν ήδη ξαναγεμίζουν όλα. Το SDF patch δένει δυναμικά
  κάθε sensor στο collision του αντίστοιχου paddle link (fail-loud αν
  λείπει κάποιο από τα 8).
- **Τριβές**: paddles μ=2.5 (λάστιχο), hub μ=0.8. Κανάλι μένει μ=1.4,
  guide R=109 (πέρασμα 59mm vs tips → τα πτερύγια λυγίζουν ~40° στο nip,
  δύναμη λαβής ~7N).
- **Αποτελέσματα (collect_test8)**: 0 collected αλλά **το paddle roller
  λύνει το radial-compliance πρόβλημα**:
  - Η μπάλα μεταφέρθηκε σε ΟΛΟ το τόξο: fwd +15→-76, z 43→**171mm** σε
    1.6s — πέρασε το μοιραίο fwd -75 (πρώτη φορά σε 8 tests) ΚΑΙ το
    guide release (155).
  - Μετά: **juggle** στην κορυφή του τροχού (fwd -66, z≈167) επί 12s — στην
    κορυφή τα πτερύγια κινούνται ΜΠΡΟΣΤΑ (φυσική του τροχού: πάνω=+x),
    οπότε δεν μπορούν να σπρώξουν τη μπάλα πίσω προς το καλάθι.
  - Σημ. ανάλυσης: το bag γράφει μόνο roller_contact_0 (1 από 8 πτερύγια)
    — στο επόμενο test προσθήκη και των 8 στο record list.
- **Εναπομένον πρόβλημα (το τελευταίο): handoff στο καλάθι.**
  - Ο υπάρχων deflector (ground z 204-266) σχεδιάστηκε για βαλλιστική
    εκτόξευση· η paddle-μεταφορά δίνει peak μόλις ~204 (οριακό άγγιγμα).
  - Γεωμετρικός περιορισμός: το πάνω μέρος του τροχού πάντα οδηγεί ΜΠΡΟΣΤΑ
    — η μπάλα πρέπει να φύγει από τον τροχό και να ταξιδέψει πίσω είτε
    βαλλιστικά είτε από στατική επιφάνεια. Επιπλέον το basket floor top
    (ground 128, front edge x=0.42) απαιτεί κέντρο μπάλας ~161 για να
    πατήσει — το guide exit δίνει 141 (το juggle έδινε 167).
  - Υποψήφιο πακέτο (test9): (α) οροφή-deflector ΠΙΣΩ-ψηλή (front-low) πάνω
    από το juggle ώστε η ανοδική-εμπρός κρούση να ανακλάται πίσω-κάτω,
    (β) επέκταση basket floor μπροστά (x→~0.50) ώστε η μπάλα που πέφτει
    πίσω από το guide sheet να προσγειώνεται σε πάτωμα και όχι στο κενό
    (x 0.42-0.50 σήμερα δεν έχει τίποτα από κάτω), (γ) πιθανό μέτριο speed
    bump για ενέργεια. Θέλει σχεδιασμό με τα juggle kinematics από το bag.
- **Status**: paddle roller ✅ λειτουργεί· handoff → #15.
- Commit checkpoint: 4cc7886 (paddle roller + εργαλεία + log).

### 15. Handoff πακέτο (test9): spin-walk ceiling + επέκταση basket floor
- **Physics insight**: στο juggle η μπάλα έχει BACKSPIN (τα πτερύγια οδηγούν
  το κάτω μέρος της μπροστά → το πάνω γυρνά πίσω). Οροφή σε επαφή από πάνω
  → η τριβή τη «περπατάει» ΠΙΣΩ κατά μήκος της οροφής (όπως μπαλάκι με
  φάλτσο κάτω από τραπέζι). Δεν χρειάζεται βαλλιστική.
- **Αλλαγή Α (deflector → spin-walk ceiling)**: από front-high (0.425,0,
  funnel 0.197, rpy -0.43) σε **front-LOW/rear-HIGH** (0.52,0,funnel 0.167,
  rpy +0.43, box 0.16×0.20×0.006). Underside στο juggle x=0.549: ground
  z≈0.192 = μέσα στη ζώνη ball-top (0.195-0.204) → συνεχής επαφή. Front
  edge (0.593, 0.172): κενό από paddle tips 15.8mm < μπάλα → μπλοκάρει
  εμπρός διαφυγή. Τριβή deflector μ=1.2 (νέο patch match στο SDF script).
- **Αλλαγή Β (basket floor forward)**: floor top 0.128 επεκτείνεται από
  x=0.42 σε **x=0.50** (box 0.54→0.62, centre 0.15→0.19) — κλείνει το κενό
  x 0.42-0.5045 όπου η μπάλα έπεφτε στο court. Έλεγχοι: 4.5mm από το guide
  sheet (μπάλα δεν χωράει να πέσει)· floor corner 116mm από άξονα > 50
  paddle sweep ✓· μπάλα πάνω στο floor εκτός εμβέλειας πτερυγίων ✓.
- **Ταχύτητα roller αμετάβλητη** (30 rad/s) — μία μεταβλητή τη φορά· το
  spin-walk δεν θέλει ενέργεια.
- **Αναμενόμενη ροή**: juggle → επαφή οροφής → spin-walk πίσω → κύλισμα
  πάνω από το guide edge (fulcrum) → πτώση στο extended floor →
  basket beams → collected=1.
- **Αποτελέσματα (collect_test9)**: 0 collected, αλλά η οροφή δουλεύει
  εν μέρει: το juggle ΕΞΑΛΕΙΦΘΗΚΕ — η μπάλα κρατιέται ΣΤΑΘΕΡΗ στο
  (fwd -66, z 161-162) επί 13s (πριν: αναπήδηση 162-171). Δεν έγινε όμως
  spin-walk προς τα πίσω.
- **Μετρημένη αιτία**: η μπάλα καρφώνεται στο όριο εμβέλειας πτερυγίων
  (dist 84mm ≈ 83mm tips+μπάλα = οριακό άγγιγμα, όχι οδήγηση). Η οριζόντια
  εμβέλεια των πτερυγίων τελειώνει στο x≈0.532 — το χείλος του guide είναι
  στο x=0.5045: **20mm κενό χωρίς καμία ώθηση** που κανένα ύψος οροφής δεν
  γεφυρώνει.
- Commit checkpoint: 73533d2 (handoff package), pushed στο origin.

### 16. Επόμενο (test10): ύψωση guide exit κάτω από το pinned spot
- **Ιδέα (μία παράμετρος)**: guide_top_z 0.155 → **0.175**. Το χείλος
  απελευθέρωσης μετακινείται εμπρός στο x=0.5275 (sinφ=(0.175-0.110)/0.109
  → φ=36.6°) — ακριβώς κάτω από το σημείο όπου η μπάλα κάθεται σταθερά
  (0.549, 0.162). Η μπάλα, πιεσμένη οροφή-πτερύγια, γέρνει πάνω από το
  χείλος (fulcrum) και πέφτει πίσω του στο extended floor (0.128) →
  basket beams.
- Αλλαγές: WALL_TOP_Z/guide_top_z και στα δύο scripts (mesh + SDF patch).
  Έλεγχος: το ανυψωμένο χείλος να μην μπλοκάρει την άφιξη της μπάλας από
  το τόξο (περνά από κάτω/μέσα του με το ceiling από πάνω).
- Έλεγχος άφιξης (υπολογισμένο): μπάλα στο εσωτερικό του τόξου φτάνει με
  κέντρο (0.554, 0.155) → μόλις κάτω από την οροφή (0.190) ✓ και ήδη σε
  επαφή με το ανυψωμένο χείλος (dist 33.2 ≈ 33) → pivot πάνω του. Πίσω από
  το χείλος, η εξωτερική επιφάνεια του guide (R+2mm) σχηματίζει φυσική
  τσουλήθρα ως το floor (0.128) ✓.
- **Αποτελέσματα (collect_test10)**: 0 collected — παρκάρισμα στο ΙΔΙΟ
  σημείο (fwd -64, z 161, 13s). Το ανυψωμένο χείλος (0.5275) μένει ΕΚΤΟΣ
  του κύκλου εμβέλειας πτερυγίων (83mm = tips 50 + μπάλα 33): το κέντρο
  της μπάλας δεν μπορεί να οδηγηθεί πίσω από το x≈0.548 σε κανένα ύψος —
  sqrt(dx²+dz²)≤83 είναι σκληρό όριο, τα 87.5mm του χείλους δεν
  γεφυρώνονται. Ο ΟΜΟΚΕΝΤΡΟΣ οδηγός δεν μπορεί να έχει έξοδο μέσα στη
  ζώνη οδήγησης.

### 17. Σπειροειδής οδηγός (spiral guide) — test11
- **Αρχή (όπως οι πραγματικοί συλλέκτες)**: η ακτίνα του οδηγού μικραίνει
  προοδευτικά R(φ)=109−14·(φ/φmax), φmax=50° → έξοδος R=95. Χείλος εξόδου
  στο (0.554, 0.183): το κέντρο της μπάλας εκεί απέχει 62mm από τον άξονα
  — ΒΑΘΙΑ μέσα στην εμβέλεια 83mm → τα πτερύγια τη σπρώχνουν ΘΕΤΙΚΑ πάνω
  από το χείλος (προοδευτική σύσφιξη 16-21mm την οποία απορροφούν τα
  πτερύγια λυγίζοντας ~57° < όριο 69°).
- Πίσω από το χείλος: η εξωτερική πλάτη του σπιράλ (R(φ)+2mm) κατεβαίνει
  down-back ως το z=0.128/x≈0.505 → φυσική τσουλήθρα στο extended floor.
- Ceiling ανεβαίνει +40mm (funnel 0.167→0.207) — γίνεται δίχτυ ασφαλείας,
  εκτός τροχιάς εξόδου.
- **Αποτελέσματα (collect_test11, taper 14mm)**: 0 collected — ΝΕΑ
  συμπεριφορά: η μπάλα ΑΝΑΚΥΚΛΩΝΕΤΑΙ (3 κύκλοι grab→άνοδος έως z≈130→
  απόρριψη→πτώση→re-grab) και τελικά διαφεύγει μπροστά. Η στένωση 14mm
  ήταν υπερβολική: απαιτούσε λύγισμα πτερυγίων ~57° και η μπάλα γλιστρούσε
  ανάμεσα στα πλήρως λυγισμένα πτερύγια πριν την έξοδο.
- **Refinement (test12)**: taper 14→**8mm** (λύγισμα ~37°, έξοδος R=101,
  χείλος (0.550, 0.187), κέντρο μπάλας εκεί 68mm < 83mm — ακόμα μέσα στη
  ζώνη θετικής οδήγησης).
- **Αποτελέσματα (collect_test12, taper 8mm + court fix live)**: 0 collected
  αλλά η ΜΕΤΑΦΟΡΑ ΕΙΝΑΙ ΠΛΕΟΝ ΤΕΛΕΙΑ: 33→135mm σε 1.6s, καμία ανακύκλωση,
  πέρασε την έξοδο του σπιράλ — και η μπάλα ΣΥΝΕΧΙΣΕ με τα πτερύγια ΠΑΝΩ
  από την κορυφή του τροχού (z=177 στο fwd -8, πάνω από τον άξονα!) και
  εκτοξεύτηκε ΜΠΡΟΣΤΑ έξω από το robot (fwd +70→120+, εν πτήσει, z 174).
  Νέο failure mode: over-the-top forward launch. (Σημ.: ball rest z=33
  πλέον — court fix #18 ενεργό, νούμερα χωρίς το +10 offset.)

### 19. Επόμενο: stripper plate (test13 — ΔΕΝ έχει υλοποιηθεί)
- Όπως οι πραγματικοί συλλέκτες: «χτενάκι» που ξεκολλάει τη μπάλα από τα
  πτερύγια στην έξοδο του σπιράλ, μπλοκάροντας τη συνέχιση over-the-top.
- **Προδιαγραφή (υπολογισμένη)**: πλάκα με κάτω-εμπρός ακμή στο
  (0.59, 0.185) — απόσταση από άξονα 79mm < 83mm (τέμνει την τροχιά του
  κέντρου της μπάλας) και > 50mm paddle sweep (δεν χτυπούν τα πτερύγια) —
  κεκλιμένη πάνω-πίσω έως ~(0.49, 0.245). Έλεγχος: στο x=0.545 αφήνει
  ~0.21 ελεύθερο για το pivot της μπάλας πάνω από το χείλος του σπιράλ
  (0.550, 0.187). Η υπάρχουσα ανυψωμένη οροφή (0.245) είναι πολύ ψηλά για
  να παίξει αυτό το ρόλο — είτε επανατοποθετείται είτε προστίθεται νέο
  λεπτό stripper στο funnel.urdf.xacro.
- **Status**: ⏳ σχεδιασμένο, για επόμενη συνεδρία.

### 18. Court visual/collision mismatch (οι μπάλες «αιωρούνται»)
- **Ερώτημα χρήστη**: οπτικό ή δεν ακουμπούν το έδαφος; → **Οπτικό.**
- **Αιτία**: στο tennis_court/model.sdf το ground collision (box 0.02) ήταν
  κεντραρισμένο στο z=0 (φυσική επιφάνεια z=+0.01) ενώ το visual είχε pose
  -0.01 (ορατή επιφάνεια z=0). Μπάλες/robot πατούσαν σε αόρατο πάτωμα
  10mm ψηλότερα → φαινομενική αιώρηση. Εξηγεί ΚΑΙ το "court +10mm" quirk
  (ball rest z=43 αντί 33) ΚΑΙ το αρχικό popping των μπαλών στο spawn.
- **Fix**: pose -0.01 και στο collision → φυσική = ορατή επιφάνεια (z=0).
  Ισχύει από το επόμενο restart. Μετά από αυτό: ball rest z=33, το
  court-offset σχόλιο του analyzer παύει να ισχύει (τα gaps βγαίνουν πλέον
  σωστά χωρίς νοητό +10).
- **Status**: ✅ εφαρμόστηκε (θα φανεί στο restart μετά το test11).

## 2026-07-10

### 20. Concept pivot: dual-wheel side pinch (design phase)
- **Context**: το deterministic bench εξάντλησε το single top-roller concept
  (stop/continue gate: "No, reconsider concept" — βλ.
  `intake-concept-decision-el.md`, `intake-bench-sweep-report-el.md`).
- **Αποφάσεις χρήστη (Q&A αυτής της session, ΠΡΙΝ από κάθε αλλαγή
  γεωμετρίας)**: (α) οριζόντιο pinch — δύο τροχοί σε κατακόρυφους άξονες
  αριστερά/δεξιά διαδρόμου· (β) funnel=centering, wheels=capture+transport,
  ramp=elevation προς basket· (γ) actuation με ΔΥΟ πανομοιότυπα μοτέρ (ίδιο
  μοντέλο με του top roller), ένα ανά τροχό, αντίθετης φοράς —
  αντικαθιστά το αρχικό "ένα μοτέρ + γρανάζια" (+1 μοτέρ στο BOM).
- **Spec**: γράφτηκε `docs/dual-wheel-intake-design-el.md` — nominal
  Rw=45mm, gap=60mm (3mm interference/πλευρά), nip x≈0.59, τροχοί
  z 0.005-0.085 (καλύπτουν ισημερινό μπάλας z=33), ω=±45 rad/s.
- **Κρίσιμος υπολογισμός**: ramp end z=0.128 ⇒ v_release ≥1.59 m/s (ιδανικά
  ~2.0 με απώλειες). Το παλιό ζεύγος (30 rad/s × r30) δίνει 0.9 m/s =
  ανεπαρκές· nominal 45 rad/s × Rw45 = 2.03 m/s. Rw και ω = πρώτοι sweep
  axes.
- **Απόδειξη αντικατάστασης**: ίδιο bench, required criteria του decision
  doc με both-wheel contact, 4/5 repeatability, head-to-head vs το
  καλύτερο top-roller case (release 0.133 m/s, no crest).
- **Status**: ✅ spec γραμμένο· εκκρεμεί έγκριση χρήστη πριν την υλοποίηση
  γεωμετρίας (Φάση 1).

### 21. Design review χρήστη: launch → transport φιλοσοφία
- **Αλλαγή πλαισίου (από χρήστη)**: το νέο concept περιγράφεται ως
  `capture → transport → guide → hopper` — ΟΧΙ ως launcher. Συνέπειες:
  - Το required "positive vertical velocity at release" ΚΑΤΑΡΓΗΘΗΚΕ
    (ήταν απαίτηση του launch model)· η ανύψωση είναι ευθύνη του
    guide/ramp σταδίου.
  - Νέα required: both-roller contact, capture through throat, positive
    inward transport, no stall/jam, ramp-entry, hopper-entry ή ramp-crest,
    4/5 repeatability (πλέον required, όχι preferred).
  - Νέα preferred: transport speed >= target, contact duration in range,
    force_p95, lateral offsets, drive-speed variations.
- **Νέες ενότητες στο decision doc**: Initial Dual-Wheel Architecture
  (2 μοτέρ, όχι γρανάζια αρχικά — απλοποίηση drivetrain εξετάζεται ΜΕΤΑ
  την απόδειξη της αρχής), Concept Validation Plan (4 phases: throat only →
  +funnel → +ramp → full intake), Operating Envelope Validation
  (5 representative variations × 4/5 — ρητό αντίδοτο στο narrow-sweet-spot
  failure του top roller).
- **Συνέπεια υλοποίησης**: η γεωμετρία θα μπει με xacro args
  (enable_funnel/enable_ramp) ώστε η Phase 1 να τρέχει ΜΟΝΟ το throat.
  Το transport-speed target (υπολογισμός ~2.0 m/s για ramp z=0.128) γίνεται
  preferred και θα οριστικοποιηθεί με μέτρηση στη Phase 3.
- **Status**: ✅ decision doc + design spec ευθυγραμμισμένα.

### 22. Pre-implementation review: 2 κενά κλείνουν στο spec
- **Λατερική ενδοτικότητα (ΚΡΙΣΙΜΟ)**: το spec έλεγε rigid τροχούς με 3mm
  interference/πλευρά — αλλά rigid+rigid = σφήνωμα (δίδαγμα test1/test2).
  Fix στο spec: κάθε τροχός σε prismatic y-carriage με ελατήριο (SDF patch
  του #9), travel 0..8mm, k~1000 N/m (sweep). Γιατί εδώ δουλεύει ενώ στο
  #13 απέτυχε: η απαιτούμενη υποχώρηση είναι ΣΤΑΘΕΡΑ ±y σε όλη την ευθεία
  διαδρομή (όχι περιστρεφόμενη ακτινική όπως στην τροχιά του roller).
- **Ρεαλιστικό μοτέρ**: πραγματικό hardware = GB37Y3530 12V με encoder
  (collector-wiring-reference). Στο spec μπήκε: effort limit = rated
  torque, ω cap = no-load RPM, jam metric = κατάρρευση joint velocity με
  ενεργό contact (καθρέφτης του encoder jam detection). ⏳ Εκκρεμεί από
  χρήστη το RPM variant του GB37 (το ω=45 rad/s = 430 RPM ίσως υπερβαίνει
  το no-load RPM· τότε Rw → ~0.060).
- Λοιπά: wheels στο base_link (funnel off στη Phase 1)· Phase 1 success =
  διέλευση nip plane με inward velocity· IR beam μετατόπιση στο throat στη
  Phase 4· BOM +1 μοτέρ.
- **Status**: spec πλήρες για υλοποίηση· μόνη ανοιχτή είσοδος το GB37
  variant (δεν μπλοκάρει τη Φάση 0 — μπαίνει placeholder 45 rad/s cap).

### 23. Motor specs επιβεβαιωμένα (GB37Y3530-12V-251R) — recalc ισοζυγίου
- **Specs (χρήστης)**: gear 43.8:1, no-load 251 RPM = 26.3 rad/s, stall
  18 kg·cm = 1.77 N·m, stall 7 A, encoder 16/700 CPR.
- **Συνέπεια**: το placeholder ω=45 rad/s ήταν αδύνατο. Free-run surface
  speed max = 26.3×0.060 = 1.58 m/s ≈ οριακά το v_min (1.59) για momentum
  climb στο z=0.128· υπό φορτίο λαβής droop σε ~1.2 m/s. Μεγαλύτερο Rw
  δεν βοηθά (+torque demand ⇒ +droop, ⌀>120 χτυπά cheeks στο y=±0.145).
- **Απόφαση**: momentum-climb ΔΕΝ είναι ο μηχανισμός ανύψωσης (συνεπές με
  την transport φιλοσοφία #21). Phase 3 μετρά το έλλειμμα· mitigations με
  σειρά: (1) feed-onto-ramp (συνεχής επαφή τροχών πάνω από την αρχή της
  ράμπας), (2) χαμήλωμα hopper entry, (3) Rw 0.070 + μετατόπιση cheeks.
- **Νέα nominal**: Rw=0.060 (⌀120), κέντρα y=±0.090, ω setpoint 25 rad/s,
  effort limit 1.77 N·m, velocity limit 26.3 rad/s,
  COLLECTOR_INTAKE_WHEEL_SPEED default 25. Cheek clearance: wheel outer
  edge y=±0.150 vs cheeks ±0.145 = ΟΡΙΑΚΟ → έλεγχος/προσαρμογή στη Φάση 0.
- **Status**: ✅ spec ενημερωμένο· όλα τα inputs πλήρη — ξεκινά η Φάση 0.

### 24. Υλοποίηση Φάσης 0: dual-wheel γεωμετρία + actuation (offline verified)
- **Γεωμετρία (drivetrain/tennis_robot xacro)**: το `intake_roller`
  αντικαταστάθηκε από `intake_side_wheel` ×2 — carriage (prismatic ±y,
  travel 0..8mm, outward) + κατακόρυφος τροχός (r=0.060, h=0.080) +
  contact sensor ανά τροχό. Nominal: nip (0.590, ±0.090), τροχός σε ground
  z 0.030-0.110 — **ανέβηκε από 0.005** γιατί το κάτω άκρο θα έτεμνε τη
  ράμπα στο πίσω τμήμα των τροχών (ramp z≈0.021 στο x=0.53)· η ζώνη
  εξακολουθεί να καλύπτει τον ισημερινό της μπάλας (0.033).
- **Funnel**: cheeks μετατοπίστηκαν (0.60→0.76, y ±0.145→±0.175) — τα παλιά
  έτεμναν τους τροχούς (51mm < 65mm)· νέο rear-tip handoff (0.647, ±0.146),
  απόσταση 77mm > 68mm (r+travel) ✓. Gating `enable_cheeks`/`enable_ramp`
  στο macro (Validation Phases μέσω INTAKE_ENABLE_FUNNEL/_RAMP envs).
- **Ramp**: entry δένεται στο nip (INTAKE_RAMP_ENTRY_X_M default nip+20mm
  = 0.610) αντί των παλιών roller offsets· lip default 0. Mesh script +
  SDF polyline συγχρονισμένα.
- **Actuation**: `intake_wheel_left/right_joint` (axis z, effort 1.77,
  vel 26.3), controller rename `lift_wheel_velocity_controller` →
  `intake_wheel_velocity_controller` (2 joints)· collector_logic στέλνει
  [-v,+v] (left CW / right CCW top-view → εσωτερικές όψεις προς -x)·
  collector default speed 30→25 rad/s, env COLLECTOR_INTAKE_WHEEL_SPEED
  (fallback στο παλιό όνομα). Launches/bridges/start_sim ενημερώθηκαν·
  bridge roller_contact_0 (left) + **νέο** roller_contact_1 (right).
- **SDF patch (generator)**: fail-loud rebind ΚΑΙ των δύο sensors στα
  lumped collision names ✓, spring patch ΚΑΙ στα δύο carriage joints
  (k=1000 env INTAKE_WHEEL_SPRING_K, ref=0) ✓, μ=2.5 + soft contact στους
  τροχούς ✓.
- **IR throat beam**: x 0.620→0.670 — μέσα στο wheel span (0.53-0.65) θα
  διάβαζε μόνιμα broken. Debug camera: μπροστά-κέντρο (0.78, 0, ground
  0.105) κοιτά πίσω στο throat (η παλιά πλαϊνή θέση πέφτει στον όγκο του
  αριστερού τροχού).
- **Offline verification (γεννημένο SDF, αριθμητικά)**: θέσεις τροχών
  ακριβώς nominal· carriage prismatic axis +y/−y, 0..8mm, k=1000·
  wheel joints effort 1.77 / vel 26.3· sensors→σωστά collisions· καμία
  αναφορά lift_wheel· clearances: ramp-τροχός 7.5mm, cheek-τροχός 9mm,
  basket-τροχός 30mm. **Phase-1 gates δουλεύουν**: generation με
  ENABLE_FUNNEL/RAMP=false δίνει SDF χωρίς cheeks/channel/deflector, με
  τροχούς+sensors παρόντες. Tests: 16/16 περνούν (console/db tests
  σπασμένα προϋπάρχοντα σε αυτό το branch — λείπει το module/deps).
- **Παράπλευρο fix**: start_sim.sh είχε σπασμένο shebang (`k#!`).
- **Εκκρεμεί (live sim)**: επιβεβαίωση counter-rotation σε /joint_states,
  οπτικός έλεγχος, και μετά bench adaptation (probe/analyzer/sweep axes
  είναι ακόμα roller-centric).
- **Status**: ✅ Φάση 0 offline πλήρης → επόμενο: bench adaptation (Task 4)
  και Phase 1 throat-only run.

### 25. Bench adaptation για dual-wheel (transport criteria)
- **sim_physics_probe.py**: dual-wheel geometry (nip/gap/radius/squeeze/bite
  dx από INTAKE_WHEEL_* envs), contact subs ΚΑΙ στα δύο /gz/roller_contact_0
  (left) + _1 (right) — generic `roller_contact_sample` type με νέο πεδίο
  `wheel: left|right` (συμβατό με summarizer), per-wheel sample counters +
  per-wheel joint velocities στο log/summary. Ball tracking πλέον προς το
  nip plane (dx_to_nip, lateral_y).
- **analyze_intake_release_criteria.py (ξαναγράφτηκε)**: transport criteria
  του decision doc με `--phase throat|funnel|ramp|full` gating:
  - Required (πάντα): both-roller contact, capture through throat (κέντρο
    μπάλας περνά το exit plane nip−bite_dx μετά την πρώτη επαφή), positive
    inward transport (ταχύτητα στο capture crossing), no stall/jam (μέγιστο
    συνεχές dwell <0.02 m/s μέσα στη ζώνη intake ≤ 2.0s).
  - Required (ramp/full): ramp climb started (z≥0.05) + crest crossing.
  - Preferred: transport peak ≥ target (0.40 default), contact duration,
    force_p95. Το launch-era "vertical velocity at release" ΔΕΝ υπάρχει
    πια (ρητό note στο output). Repeatability 4/5 = cross-run (sweep level).
- **analyze_intake_bench_poses.py**: nip-centric — απόσταση κέντρου μπάλας
  από τον πλησιέστερο κατακόρυφο άξονα τροχού (xy), radial_gap vs
  Rw+Rball, dx_to_nip.
- **run_native_intake_sweep.sh**: νέοι sweep axes INTAKE_SWEEP_WHEEL_GAPS /
  _WHEEL_RADII / _NIP_XS / _SPRING_KS / _WHEEL_SPEEDS / _DRIVE_SPEEDS /
  _BALL_LATERAL_OFFSETS (envelope), INTAKE_SWEEP_PHASE=throat|funnel|ramp|
  full → θέτει ENABLE_FUNNEL/RAMP. Wheel command `[-v,+v]`. Το
  wait_for_wheel_speed απαιτεί ΚΑΙ τους δύο τροχούς σε ταχύτητα ΚΑΙ
  αντίθετα πρόσημα (left<0<right) — δομικό live check του two-motor wiring
  σε κάθε run (exit 2 σε λάθος φορά).
- **summarize_contact_physics.py**: wheel_left/right_samples + νέα geometry
  πεδία.
- **Offline verification**: synthetic fixtures — pass case δίνει 4/4
  (transport peak 0.50 m/s, stall 0), stall case δίνει 0/4 (stall 5.7s,
  μόνο left contact). Bash syntax OK, όλα τα py compile, κανένα
  lift_wheel/roller-offset υπόλειμμα στο bench path.
- **Εκκρεμότητα (συνειδητή)**: `analyze_collect_bag.py` +
  `run_collect_test.sh` (collect_one flow) μένουν roller-era — αφορούν τη
  Phase 4/full integration, θα προσαρμοστούν τότε.
- **Status**: ✅ bench έτοιμο. Επόμενο: colcon build + Phase 1 throat-only
  live run (INTAKE_SWEEP_PHASE=throat).

### 26. Phase 1 πρώτο live run + διάγνωση contact instrumentation
- **Προεργασία**: σκοτώθηκαν 4 ΟΡΦΑΝΑ headless `gz sim` από παλιά bench
  runs (θα μόλυναν το gz transport — κοινό default partition). Μάθημα:
  πριν από bench run, `pgrep -f "gz sim"`.
- **Run 1 (nominal, gap 60/Rw 60/nip 0.590/k 1000/drive 0.12/ω 25)**:
  - ✅ **Counter-rotation live verified**: wheels_ready left=-25.000
    right=+25.000 — το two-motor wiring σωστό (Task 3 κλείνει).
  - ✅ **Η φυσική της σύλληψης ΔΟΥΛΕΨΕ με την πρώτη**: capture through
    throat = true, inward velocity στο exit plane **0.77 m/s** (6× το
    drive 0.12 — μόνο οι τροχοί μπορούν να την επιτάχυναν· funnel/ramp
    off, τίποτα άλλο μπροστά), κανένα stall. Required 3/4.
  - ❌ 0 contact samples ΚΑΙ στους δύο sensors — αδύνατο φυσικά (gap 60 <
    66) → instrumentation.
- **Διάγνωση (ζωντανό sim + teleport μπάλας στο nip)**:
  - Το gz εκπέμπει τα contact topics στο **world path** (όπου ακούν τα
    bridges) — το explicit `<topic>` του sensor ΔΕΝ είναι το ενεργό.
  - Με μπάλα στατικά στο nip: contact messages ρέουν ΚΑΙ στους δύο
    τροχούς, και το ROS άκρο (`ros2 topic echo /gz/roller_contact_0`)
    λαμβάνει — **bridge chain σωστό end-to-end**.
  - Άρα root cause του 0: η διέλευση είναι **impulsive** — το 3mm/side
    interference + kp 8000 εκτόξευσε τη μπάλα διαμέσου του throat σε
    χρόνο μικρότερο από την περίοδο του sensor (100Hz = 10ms) →
    μηδέν δείγματα παρότι υπήρξε επαφή.
- **Fixes**: sensor update_rate 100→**500Hz** (κοντά στο physics rate) και
  στους δύο τροχούς· `INTAKE_SWEEP_REPEATS` env στο bench (per-case
  επαναλήψεις με suffix _rN) για το required 4/5.
- **Παρατήρηση φυσικής (για τα sweeps)**: η μεταφορά είναι προς το παρόν
  kick, όχι ελεγχόμενο grip — αποδεκτό ως capture+transport, αλλά το
  contact duration/βαθμός ελέγχου είναι ζητούμενο των gap/spring_k sweeps.
- **Επόμενο**: Phase 1 ×5 nominal (τρέχει) → αν both-wheel contact
  επιβεβαιωθεί σε ≥4/5, η Phase 1 περνά.

### 27. Harness fixes + Phase 1 throat-only repeatability PASS
- **Review fixes πριν το τελικό run**:
  - `run_native_intake_sweep.sh`: το bench drive επέστρεψε στο πραγματικό
    live input του `diff_drive_controller`, `/diff_drive_controller/cmd_vel`
    (`TwistStamped`), αλλά πλέον με `header: auto`. Χωρίς live stamp, ο
    controller δεν κινούσε τη βάση. Το intermediate unstamped attempt
    αποδείχθηκε λάθος για το live runtime: το topic info έδειξε subscriber
    στο stamped `/diff_drive_controller/cmd_vel`.
  - Το probe ξεκινάει **πριν** από το drive publisher. Πριν ξεκινούσε μετά
    το `drive_response` και έχανε όλο το impulsive throat event, άρα έβλεπε
    capture από poses αλλά 0 contact samples.
  - `gazebo_extras_node.py`: `/sim/roller_contact` / RViz markers ακούν και
    τα δύο contact topics (`/gz/roller_contact_0`, `/gz/roller_contact_1`).
  - `diagnose_motion.sh`: updated στο νέο
    `/intake_wheel_velocity_controller/commands` με `[-10,+10]`.
  - `docs/dual-wheel-intake-design-el.md`: env typo
    `INTAKE_WHEEL_NIP_X_M` → `INTAKE_NIP_X_M`.
- **Sanity run μετά τα fixes**:
  - Path: `runtime/intake_sweeps/20260710_154459`.
  - Result: required **4/4** σε 1/1. Both-wheel contacts 38/38, contact
    duration 0.096s, capture inward velocity 0.80 m/s, no stall.
- **Phase 1 nominal ×5**:
  - Path: `runtime/intake_sweeps/20260710_154614`.
  - Config: throat-only, gap 60mm, wheel radius 60mm, nip x=0.590,
    spring k=1000 N/m, drive 0.12 m/s, wheel speed 25 rad/s.
  - Result: **PASS 5/5**. Κάθε repeat είχε required **4/4**:
    both-wheel contact, capture through throat, positive inward transport,
    no stall/jam.
  - Contact samples: r1-r4 = 38 left / 38 right, r5 = 35 / 35.
  - Contact duration: 0.080-0.094s. Force max 5.32N, p95 ~4.01-4.08N.
  - Capture inward velocity: 0.76-0.81 m/s, πάνω από το preferred transport
    target 0.40 m/s.
  - Base drive verified live: odom vx ~0.12 m/s και drive wheel velocity
    ~1.41 rad/s σε όλα τα repeats.
- **Συμπέρασμα**: Phase 1 throat-only έχει πλέον αποδειχθεί repeatable για
  centered ball. Το dual-wheel concept περνά το πρώτο gate. Επόμενο λογικό
  βήμα: Phase 2 funnel-only και lateral-offset envelope, όχι ramp/hopper ακόμα.

### 28. Phase 2 funnel-only envelope + boundary repeatability PASS
- **Motor option review**: εξετάστηκε JGB37-3530-1000 12V / 10:1 / 1000 RPM.
  Συμπέρασμα: όχι default για το intake. Με Rw=60mm δίνει ~5.0 m/s rated
  surface speed αλλά μόνο ~0.62N tangential force ανά τροχό στο rated torque
  και ~2.5N στο stall, ενώ το 251RPM μοτέρ έχει πολύ μεγαλύτερο torque
  reserve. Για `capture -> transport -> guide -> hopper` κρατάμε το
  GB37Y3530-12V-251R.
- **Bug στο Phase 2 sweep harness**:
  - Το πρώτο funnel envelope run (`runtime/intake_sweeps/20260710_155624`)
    αποκάλυψε ότι τα lateral offsets ήταν cumulative: κάθε case υπολόγιζε
    `BENCH_BALL_Y` από το ήδη-exported `INTAKE_BENCH_BALL_Y`.
  - Fix: προστέθηκε σταθερό `BENCH_BALL_Y_BASE` και κάθε case κάνει
    `BENCH_BALL_Y = base + lateral_offset`.
- **Καθαρό Phase 2 envelope probe**:
  - Path: `runtime/intake_sweeps/20260710_160008`.
  - Config: funnel-only, ramp off, nominal throat, offsets
    -80/-40/0/+40/+80mm, 1 repeat ανά offset.
  - Result: **PASS 5/5**, required **4/4** σε όλα.
  - Actual `bench_config.txt` επιβεβαίωσε σωστά `ball_y` = -0.08, -0.04,
    0.0, +0.04, +0.08.
  - Κάθε case είχε both-wheel contact 38/38, no stall, capture inward
    velocity 0.54-0.81 m/s.
- **Phase 2 boundary repeatability**:
  - Path: `runtime/intake_sweeps/20260710_160331`.
  - Config: ±80mm offsets, 5 repeats ανά όριο.
  - Result: **-80mm PASS 5/5**, **+80mm PASS 5/5**, required **4/4** σε όλα.
  - +80mm: contact samples 36-38/side, capture velocity 0.74-0.80 m/s,
    no stall.
  - -80mm: 4/5 repeats ισχυρά (38/38 samples, capture velocity 0.72-0.80
    m/s), 1/5 ασθενέστερο αλλά pass (11/10 samples, capture velocity
    0.21 m/s, no stall). Watch item για μελλοντικό wider-offset/approach
    variation, όχι blocker.
- **Συμπέρασμα**: Phase 2 funnel-only περνά για centered και ±80mm lateral
  offsets. Το επόμενο gate είναι Phase 3 ramp-only / feed-onto-ramp, όχι
  αλλαγή μοτέρ.

### 29. Phase 3 ramp-only: collision fixed, handoff still fails; not a motor-first problem
- **Initial ramp run**:
  - Path: `runtime/intake_sweeps/20260710_161533`.
  - Result: required **4/6**. The dual wheels captured and transported the
    ball (`capture_inward_velocity` ~0.69 m/s, both-wheel contact 38/38),
    but the ball stayed at ground height (`z` ~0.033m) and never contacted /
    climbed the ramp.
  - Diagnosis: the SDF `polyline` collision for `intake_channel_col` was
    present but did not behave as a reliable DART collision surface here.
- **Collision fix**:
  - `scripts/generate_robot_urdf.py` now replaces the ramp mesh collision with
    multiple thin, sloped box collision segments instead of one polyline.
  - A temporary axis-aligned box approximation made the ramp "exist" but
    created staircase edges and jams; the current sloped segments follow the
    ramp pitch.
- **Best Phase 3 result so far**:
  - Path: `runtime/intake_sweeps/20260710_162813`.
  - Config: ramp-only, `INTAKE_RAMP_ENTRY_X_M=0.580`, sloped box collision,
    wheel speed 25 rad/s.
  - Result: required **4/6**. The ball now climbs the ramp (`z` crossed
    0.05m at `base_x` ~0.5225), but it does not reach the ramp crest and then
    stalls near the throat / ramp handoff for ~9s.
- **Motor/speed check**:
  - Path: `runtime/intake_sweeps/20260710_162949`.
  - Config: same ramp setup, wheel speeds 35 and 45 rad/s.
  - Results: 35 rad/s stayed **4/6** with ~9.37s stall; 45 rad/s degraded to
    **3/6**. No crest crossing in either case.
  - Conclusion: this is **not** primarily solved by a larger/faster motor.
    The wheels already produce inward transport; the failure is the
    wheel-to-ramp handoff geometry / passive ramp interaction. Next phase-3
    work should examine a smoother/longer handoff, different ramp entry angle,
    or an active second-stage conveyor/roller before changing motor class.

### 30. Phase 3 follow-up: larger wheels and lower ramp target still do not pass
- **Wheel diameter check**:
  - Path: `runtime/intake_sweeps/20260710_164654`.
  - Config: `INTAKE_RAMP_ENTRY_X_M=0.580`, `Rw=70mm`, `nip_x=0.590/0.600`,
    ramp-only, wheel speed 25 rad/s.
  - Results:
    - `Rw=70mm`, `nip_x=0.590`: required **4/6**, climb reached
      `z=0.0543m`, no crest, longest stall ~7.86s.
    - `Rw=70mm`, `nip_x=0.600`: required **4/6**, climb reached
      `z=0.0510m`, no crest, longest stall ~9.32s.
  - Conclusion: larger wheels buy a little better handoff in the best case,
    but do not solve Phase 3. Diameter is not enough by itself.
- **Lower ramp / hopper target check**:
  - Code change: `scripts/generate_curved_scoop_mesh.py` and
    `scripts/generate_robot_urdf.py` now accept
    `INTAKE_RAMP_KNEE_Z_M` and `INTAKE_RAMP_END_Z_M`, so lower handoff
    heights can be tested without hardcoding a new design.
  - Path: `runtime/intake_sweeps/20260710_165010`.
  - Config: `INTAKE_RAMP_ENTRY_X_M=0.580`,
    `INTAKE_RAMP_KNEE_Z_M=0.020`, `INTAKE_RAMP_END_Z_M=0.075`,
    `INTAKE_BENCH_RAMP_CREST_Z_M=0.085`, `Rw=60mm`, ramp-only.
  - Result: required **4/6**. Ball climbed to `z=0.0513m` but still did not
    reach even the lowered crest criterion; longest stall ~8.24s.
  - Conclusion: simply lowering the ramp endpoint is also not enough. The
    failure is the passive handoff after the wheels, before sustained climb.
    Next design branch should add active support after the throat: either
    extend powered contact along the ramp, or add a small conveyor / roller /
    flywheel second stage.

### 31. Phase 3 active assist roller probe: point assist is not enough
- **Code change for fast iteration**:
  - Added optional `enable_assist` second-stage roller with its own
    `assist_wheel_velocity_controller`.
  - Added env-tunable geometry:
    `INTAKE_ASSIST_X_M`, `INTAKE_ASSIST_Z_M`,
    `INTAKE_ASSIST_RADIUS_M`, `INTAKE_ASSIST_LENGTH_M`.
  - The sweep harness records assist geometry in `bench_config.txt` and case
    names, and explicitly spawns/publishes the assist controller in bench
    runs.
- **First hardcoded placement check**:
  - Path: `runtime/intake_sweeps/20260710_170143`.
  - Config: assist true, speed 25 rad/s, old placement
    `x=0.545`, `z=0.050` in `base_link` frame.
  - Result: required **1/6**. The assist roller blocked the capture/ramp
    handoff instead of helping it; capture crossing and ramp climb were both
    false.
  - Diagnosis: too low/too far forward; it becomes an obstacle near the
    throat instead of a second-stage support.
- **Retuned placement sweep**:
  - `runtime/intake_sweeps/20260710_203638`: `x=0.510`, `z=0.075`,
    assist speed +25 rad/s. Result **4/6**: capture true, ramp climb true
    (`z=0.0526m`), no crest, longest stall ~9.21s.
  - `runtime/intake_sweeps/20260710_203812`: `x=0.505`, `z=0.065`,
    assist speed +25 rad/s. Result **4/6**: capture true, ramp climb true
    (`z=0.0508m`), no crest, longest stall ~9.32s.
  - `runtime/intake_sweeps/20260710_203925`: same geometry as above,
    assist speed **-25 rad/s**. Result **4/6**, essentially unchanged
    (`z=0.0508m`, stall ~9.36s). So the main issue is not assist direction.
  - `runtime/intake_sweeps/20260710_204104`: `x=0.505`, `z=0.055`,
    assist speed +25 rad/s. Result **3/6**: ramp climb false, stall ~8.99s.
- **Conclusion**: a single point assist roller has a narrow/no useful window
  in this geometry. Too high barely engages; mid height does not sustain the
  climb; too low becomes another jam/drag point. Phase 3 should pivot from
  "one extra roller" to **active support over length**: short belt/conveyor
  over the ramp, paired roller + compliant lower guide, or a mechanically
  smoother ramp throat that keeps the ball supported after the side wheels.

### 32. Phase 3B conveyor approximation: top rollers alone still do not pass
- **Code change**:
  - Added optional `INTAKE_ENABLE_CONVEYOR=true` prototype: three small
    powered rollers along the ramp (`intake_conveyor_front/mid/rear_joint`)
    controlled by `conveyor_velocity_controller`.
  - Added env knobs:
    `INTAKE_CONVEYOR_SPEED`, `INTAKE_CONVEYOR_X_BIAS_M`,
    `INTAKE_CONVEYOR_Z_BIAS_M`.
  - The model is a Gazebo-friendly approximation of a short belt: not a
    deformable belt, but a quick proof for "active support over length".
- **Offline verification**:
  - `INTAKE_ENABLE_CONVEYOR=true` render creates the 3 conveyor joints in
    URDF/SDF.
  - Flag-off render has no `intake_conveyor_*` links/joints, so Phase 1/2 and
    passive Phase 3 baselines remain comparable.
- **Live runs**:
  - `runtime/intake_sweeps/20260710_204701`: conveyor on, speed +25 rad/s,
    no x/z bias. Result **3/6**: capture true, ramp climb false, stall
    ~7.11s. Front roller was too low/early (bottom around 71mm ground).
  - `runtime/intake_sweeps/20260710_210524`: z-bias +15mm. Result **4/6**:
    capture true, ramp climb true (`z=0.0501m`), no crest, stall ~8.71s.
  - `runtime/intake_sweeps/20260710_210748`: same +15mm but speed **-25**
    rad/s. Result **4/6**, still no crest, stall ~9.12s. Direction is not
    the main blocker.
  - `runtime/intake_sweeps/20260710_210957`: x-bias +30mm and z-bias +15mm
    so the first powered roller starts closer to throat exit. Result **4/6**:
    ramp climb true (`z=0.0527m`), no crest, stall ~9.18s.
- **Conclusion**: top active rollers over the ramp improve/shape the handoff
  but still do not sustain climb. The ball reaches early climb and then falls
  back / stalls around `base_x≈0.566`, which suggests it is not being held in
  the active contact patch. Next mechanical branch should add a **compliant
  lower guide / paired pinch path** over the ramp, or replace the ramp with a
  true belt/conveyor surface that supports the ball from below while driving
  it rearward/upward.

### 33. Phase 3C tilted side wheels: lift appears, but not enough alone
- **User observation**: the side wheels were not tilted upward; they were
  vertical-axis wheels. That meant the dual-wheel stage produced mostly
  rearward transport, not a deliberate upward component toward the ramp.
- **Code change**:
  - Added `INTAKE_WHEEL_TILT_DEG` / `INTAKE_SWEEP_WHEEL_TILTS_DEG`.
  - The side-wheel joint frame is pitched around `y`, so the cylinder axis and
    spin axis tilt together toward `+x`.
  - With the existing command signs (`left=-v`, `right=+v`), positive tilt
    adds upward `z` velocity at the inner contact faces while keeping rearward
    transport.
- **Offline verification**:
  - `INTAKE_WHEEL_TILT_DEG=15` render produced SDF wheel joint poses with
    pitch `0.261799 rad`, confirming the wheels really tilt.
- **Ramp-only sweep**:
  - Path: `runtime/intake_sweeps/20260710_211656`.
  - Config: assist false, conveyor false, `Rw=60mm`, `nip_x=0.590`,
    `INTAKE_RAMP_ENTRY_X_M=0.580`, tilts `0/10/15` deg.
  - `0°`: required **4/6**, ramp climb true (`z=0.0530m`), no crest,
    stall ~9.07s.
  - `10°`: required **3/6**, ramp climb false, no crest, stall ~9.05s.
  - `15°`: required **4/6**, ramp climb true at capture (`z=0.0511m`),
    upward velocity `vz≈0.281m/s`, no crest, stall ~8.67s.
- **Upper bracket**:
  - Path: `runtime/intake_sweeps/20260710_211937`.
  - Config: `tilt=20°`, same ramp-only setup.
  - Result: required **4/6**, ramp climb true (`z=0.0549m`), upward velocity
    `vz≈0.287m/s`, no crest, stall ~8.06s.
- **Conclusion**: tilted side wheels are a better physical mechanism than the
  added top assist/conveyor rollers: they create real upward velocity and
  reduce stall somewhat. But even 20° does not reach the crest. The direction
  is promising, but it still needs either a gentler/lower/longer ramp or a
  short guided support section after the tilted wheels so the ball cannot fall
  out of the active path.

### 34. Tilt 20° + 430RPM motor hypothesis: speed alone still does not pass
- **Question**: maybe the right answer is tilted wheels plus a stronger /
  faster motor?
- **Important harness fix**:
  - The first `tilt=20`, `wheel_speed=45rad/s` run
    (`runtime/intake_sweeps/20260710_212321`) was **not a real 430RPM test**:
    the generated joint/controller velocity limit was still `26.3rad/s`
    (GB37Y3530-12V-251R no-load), and the summary confirmed
    `joint_vel_abs_max_rad_s=26.3`.
  - Added `INTAKE_WHEEL_MAX_VEL_RAD_S` so hypothetical faster motor variants
    can be tested explicitly without changing the default real-motor limit.
- **Actual 430RPM-style run**:
  - Path: `runtime/intake_sweeps/20260710_212523`.
  - Config: `tilt=20°`, `wheel_speed=45rad/s`, `INTAKE_WHEEL_MAX_VEL_RAD_S=45`,
    ramp-only, assist/conveyor off.
  - Verification: summary confirmed `joint_vel_abs_max_rad_s=45.0`.
  - Result: required **4/6**. Ramp climb true (`z=0.0500m`), upward velocity
    `vz≈0.309m/s`, but no crest; longest stall ~8.92s.
- **Conclusion**: 430RPM with 20° tilt increases the upward component, but it
  still does not solve Phase 3. The remaining blocker is not simply motor
  speed; the ball still falls out of the supported/active path before crest.
  A faster motor might be useful later, but the next design change should
  still target ramp/support geometry.

### 35. Tilt 20° + more squeeze at real motor speed
- **Question**: after the 430RPM check, try the real ~251RPM setup with more
  pressure on the ball.
- **Run**:
  - Path: `runtime/intake_sweeps/20260710_212704`.
  - Config: `tilt=20°`, wheel speed `25rad/s`, max velocity `26.3rad/s`,
    assist/conveyor off, gaps `58/56/54mm`, `Rw=60mm`, spring `k=1000`.
- **Results**:
  - `gap=58mm`: required **4/6**, ramp climb true (`z=0.0519m`), no crest,
    stall ~8.82s, capture inward velocity ~1.05m/s.
  - `gap=56mm`: required **4/6**, ramp climb true (`z=0.0618m`), no crest,
    stall ~7.50s, capture inward velocity ~0.91m/s. This is the best
    real-speed squeeze case so far: more lift and shorter stall, but still no
    crest.
  - `gap=54mm`: required **2/6**. Too tight: capture through throat false,
    positive inward transport false, ramp climb false.
- **Conclusion**: more squeeze helps up to a point. `56mm` looks like the
  useful pressure bracket for tilted wheels at real motor speed; `54mm` is too
  much and starts rejecting/jamming before transport. Pressure improves the
  handoff, but still does not replace the need for an easier ramp/support path.

### 36. Drive-motor specs as intake motor: high torque, too slow
- **Question**: try the specs of the existing drive motors as the intake wheel
  motor.
- **Specs used**:
  - DFRobot FIT0403 / drive motor: 12V, **122RPM**, **38kg·cm**.
  - Converted for sim: `INTAKE_WHEEL_MAX_VEL_RAD_S=12.78`,
    `INTAKE_WHEEL_EFFORT_NM=3.73`.
- **Code change**:
  - Added `INTAKE_WHEEL_EFFORT_NM` so motor torque variants can be represented
    explicitly instead of changing only velocity.
- **Run**:
  - Path: `runtime/intake_sweeps/20260710_213247`.
  - Config: `tilt=20°`, `gap=56mm`, `Rw=60mm`, wheel speed/max vel
    `12.78rad/s`, effort `3.73N·m`, ramp-only.
  - Verification: summary confirmed `joint_vel_abs_max_rad_s=12.78`;
    `bench_config.txt` confirmed `wheel_effort=3.73`.
  - Result: required **4/6**. Ramp climb true (`z=0.0530m`), no crest,
    stall ~7.81s, capture inward velocity ~0.68m/s, upward velocity
    `vz≈0.241m/s`.
- **Comparison**:
  - Best GB37 real-speed squeeze case (`25rad/s`, 56mm gap) reached
    `z=0.0618m`, inward ~0.91m/s, `vz≈0.379m/s`, stall ~7.50s.
  - The drive motor's torque is higher, but its 122RPM speed is much lower,
    and in this handoff the extra torque does not compensate for the lost
    surface speed.
- **Conclusion**: existing drive motor specs are not better for the intake
  lift/handoff. They are strong, but too slow for this tilted-wheel transport
  role. Keep the intake closer to the GB37 251RPM class, and solve the
  remaining failure with ramp/support geometry rather than swapping to the
  drive motors.

### 37. Tilt 20° + 251RPM + longer/gentler ramp
- **Question**: αν κρατήσουμε το καλύτερο έως τώρα intake set
  (`251RPM`, `tilt=20°`, `gap=56mm`, `Rw=60mm`) και αυξήσουμε την απόσταση
  της ράμπας ώστε να μικρύνει η κλίση, βοηθάει;
- **Implementation knob**:
  - `scripts/generate_robot_urdf.py` and `scripts/generate_curved_scoop_mesh.py`
    now read `INTAKE_RAMP_KNEE_X_M` and `INTAKE_RAMP_END_X_M`.
  - Defaults remain unchanged (`knee_x=0.520`, `end_x=0.400`), so old runs
    are unaffected unless those env vars are set.
- **Test**:
  - Path: `runtime/intake_sweeps/20260710_213846`.
  - Config: `INTAKE_RAMP_KNEE_X_M=0.470`,
    `INTAKE_RAMP_END_X_M=0.320`, `tilt=20°`, `gap=56mm`,
    wheel speed/max vel `25/26.3rad/s`, assist/conveyor off, ramp-only.
- **Result**: required **4/6**. Capture, both-wheel contact, inward transport,
  and ramp climb are true, but there is still no crest crossing and there is
  still a stall.
- **Useful comparison vs baseline gap=56mm run
  (`runtime/intake_sweeps/20260710_212704`)**:
  - inward transport: `0.91 -> 0.97m/s` (slightly better)
  - ramp climb z at crossing: `0.0618 -> 0.0639m` (about the same / slightly
    better)
  - upward velocity at crossing: `0.379 -> 0.270m/s` (worse)
  - longest stall: `7.50s -> 5.26s` (meaningfully better, but still fails the
    2s criterion)
- **Conclusion**: making the ramp longer/gentler helps, especially by reducing
  the stall, but it is not sufficient alone. It supports the current diagnosis:
  the remaining problem is the unsupported handoff after the tilted wheels. The
  next geometry step should combine the gentler ramp with a short lower/side
  guide or a longer active contact zone, not a motor-only change.

### 38. Short/steep ramp + higher wheel tilt concept
- **Question**: what if we do the opposite of the gentler ramp: shorten the
  ramp so it is closer to ~40°, increase wheel tilt, and accept that the ball
  may be thrown/lobbed instead of softly transported?
- **Test geometry**:
  - Short/steep ramp via `INTAKE_RAMP_KNEE_X_M=0.535`,
    `INTAKE_RAMP_END_X_M=0.450`, with `INTAKE_RAMP_ENTRY_X_M=0.580`.
  - Same motor class: wheel speed/max vel `25/26.3rad/s`, `Rw=60mm`.
- **Runs**:
  - `runtime/intake_sweeps/20260710_214321`: `tilt=40°`, `gap=56mm`.
  - `runtime/intake_sweeps/20260710_214455`: `tilt=40°`, `gap=60mm`.
  - `runtime/intake_sweeps/20260710_214607`: `tilt=25/30/35°`,
    `gap=60mm`.
- **Results**:
  - `40°/56mm`: required **4/6**, inward `1.02m/s`, `vz≈0.425m/s`,
    no crest, contact `8.94s`, stall `8.87s`.
  - `40°/60mm`: required **4/6**, inward `0.77m/s`, `vz≈0.333m/s`,
    no crest, contact `6.74s`, stall `6.64s`.
  - `25°/60mm`: required **4/6**, inward `0.83m/s`, `vz≈0.269m/s`,
    no crest, contact `9.21s`, stall `9.12s`.
  - `30°/60mm`: required **4/6**, inward `0.68m/s`, `vz≈0.260m/s`,
    no crest, contact `9.14s`, stall `9.12s`.
  - `35°/60mm`: required **4/6**, inward `0.73m/s`, `vz≈0.305m/s`,
    no crest, contact `8.70s`, stall `8.63s`.
- **Conclusion**: if throwing/lobbing is acceptable, this concept still needs
  a release mechanism/exit clearance. In sim, the steep-ramp/high-tilt setup
  does not cleanly throw the ball; it keeps it in prolonged wheel contact and
  stalls near the handoff. Opening the gap reduces stall at 40° but also
  reduces inward speed, and lower tilts do not find a better sweet spot. Next
  useful pivot is not more tilt alone, but shaping a short release throat:
  wheels should end before the steep ramp traps the ball, with a small
  backstop/guide or basket lip positioned to catch the lob.

### 39. Wheels closer to base + short handoff bar to basket entry
- **Question**: the wheels should be closer to the base; the old long ramp/bar
  is not the right abstraction. The entry bar should start just before handoff
  and run only to the basket entry.
- **Geometry tested**:
  - `INTAKE_NIP_X_M=0.540`: wheel centre moved closer to the chassis/base
    front edge. With `Rw=60mm`, the wheel rear edge is near `x=0.480`.
  - `INTAKE_RAMP_ENTRY_X_M=0.500`: short bar starts just before the rear
    wheel handoff.
  - `INTAKE_RAMP_END_X_M=0.455`: bar ends near the chassis/basket entry edge.
  - `INTAKE_RAMP_END_Z_M=0.052`: bar surface ends around the chassis top
    height, targeting ball-centre height around `0.085m`.
  - Analyzer target adjusted to `INTAKE_BENCH_RAMP_CREST_Z_M=0.085`.
- **Run**:
  - Path: `runtime/intake_sweeps/20260710_215854`.
  - Config: `tilt=30°`, `gap=60mm`, `Rw=60mm`, wheel speed/max vel
    `25/26.3rad/s`, assist/conveyor off.
- **Result**: required **4/6**.
  - Both-wheel contact true, but now wheel contact is short: `0.34s`
    instead of the multi-second trapped contact seen with the steep long case.
  - Ramp/guide contact is real (`1908` guide samples).
  - Ramp climb starts (`z=0.0548m`) but does not reach the lower basket-entry
    target (`0.085m`).
  - Release velocity points back/down (`vx≈+0.253m/s`, `vz≈-0.343m/s`), so
    the ball is being rejected/falling rather than caught by the bar.
  - Stall still fails (`8.24s`), but the failure mode changed from
    wheel-trap to guide/handoff rejection.
- **Conclusion**: this is a better abstraction than the old long ramp. The
  wheels should indeed sit closer to the base, and the "bar" should be a short
  basket-entry guide, not a long passive climb. However, the first short-bar
  geometry is too abrupt/low or has the wrong catch shape. Next refinement:
  keep this shorter handoff architecture, but shape the guide as a catching lip
  / curved pocket at ~8.5-10cm ball-centre target rather than a simple ramp
  segment.

### 40. Short handoff architecture: pressure, tilt, and wheel diameter sweeps
- **Question**: play with wheel tilt, pressure, and wheel diameter while
  keeping the newer architecture from #39: wheels closer to the base and a
  short handoff bar to basket entry.
- **Base geometry**:
  - `nip_x=0.540`, short bar `x=0.500 -> 0.455`.
  - bar end `z=0.052`, lower basket-entry target `0.085m`.
  - wheel speed/max vel `25/26.3rad/s`, assist/conveyor off.
- **Pressure + tilt sweep**:
  - Path: `runtime/intake_sweeps/20260710_220153`.
  - Grid: gaps `56/58/60mm`, tilts `25/30/35°`, `Rw=60mm`.
  - All cases remained **4/6**: capture + ramp climb true, no basket-entry
    crossing and stall still fails.
  - Notable patterns:
    - tighter gaps (`56/58mm`) produce more lift (`vz≈0.35-0.52m/s`,
      z up to `0.0666m`) but long contact/stall (`~6.5-8.8s`).
    - `gap=60mm` releases quickly at `30/35°` (`~0.34-0.38s` wheel contact)
      but loses lift (`vz≈0.16-0.20m/s`, z around `0.053-0.054m`) and then
      guide rejection/fall remains.
    - `gap=56mm, tilt=35°` had the strongest upward component
      (`vz≈0.523m/s`, z `0.0622m`) but still stalled and did not reach target.
- **Diameter sweep**:
  - Paths: `runtime/intake_sweeps/20260710_220856` and
    `runtime/intake_sweeps/20260710_221125`.
  - Tested `Rw=55/60mm` (diameter `110/120mm`) at `tilt=35°` for
    `gap=58/60mm`. `Rw=65mm` (diameter `130mm`) did not produce a valid
    physics result: the pre-drive wheel readiness check failed twice with
    joint speeds near zero and RTPS SHM port errors in `wheels_ready.log`.
  - `gap=58, Rw=55`: **4/6**, z `0.0516m`, `vz≈0.342m/s`, stall `8.78s`.
  - `gap=58, Rw=60`: **4/6**, z `0.0666m`, `vz≈0.375m/s`, stall `8.65s`.
  - `gap=60, Rw=55`: **4/6**, z `0.0537m`, `vz≈0.187m/s`, contact `0.31s`,
    release velocity down/back (`vz≈-0.237m/s`).
  - `gap=60, Rw=60`: **4/6**, z `0.0552m`, `vz≈0.111m/s`, contact `0.34s`,
    release velocity down/back (`vz≈-0.191m/s`).
- **Conclusion**: diameter is not the missing lever by itself. `120mm`
  diameter is still better than `110mm` for lift in the `58mm` gap case, but
  it does not solve basket-entry. `60mm` gap gives desirable short wheel
  contact, but then the guide must catch/support the ball; otherwise the ball
  is rejected downward. Next mechanical change should be guide shape/catch
  pocket, not more pure parameter sweep.

### 41. ΡΙΖΑ του Phase 3 plateau: το scoop-era basket floor κρεμόταν πάνω από το handoff
- **Ερώτημα**: ποιο είναι το πιο κοντινό σενάριο σε επιτυχία; → Ανάλυση του
  gz_poses του καλύτερου case (#40, gap=56/tilt=35): peak μπάλας
  **z=0.0749 στο dx=0.465** — ΑΚΡΙΒΩΣ το γεωμετρικό όριο κάτω από το
  basket floor: κάτω επιφάνεια slab 0.108 − ακτίνα 0.033 = **0.075**.
- **Διάγνωση**: το basket floor (top 0.128, μπροστινή ακμή x=0.50 — η
  scoop-era «επέκταση» του #15) καλύπτει από ΠΑΝΩ όλη τη ζώνη handoff του
  short bar (0.455-0.50). Η μπάλα χτυπούσε την ΚΑΤΩ πλευρά του πατώματος
  ενώ ανέβαινε (~0.25 m/s) και εκτοξευόταν πίσω-κάτω. Το "guide rejection"
  των #39/#40 ήταν αυτό — όχι κακό σχήμα του bar.
- **Αλλαγή (παραμετροποίηση, defaults ΑΘΙΚΤΑ)**: `basket.urdf.xacro` πήρε
  macro params `floor_front_x` / `floor_top_z_ground` (τοίχοι δένουν πλέον
  στο πάτωμα ώστε να μη ανοίγει πλαϊνό κενό)· xacro args
  `basket_floor_front_x`/`basket_floor_top_z` στο `tennis_robot.urdf.xacro`·
  envs `INTAKE_BASKET_FLOOR_FRONT_X_M` / `INTAKE_BASKET_FLOOR_TOP_Z_M` στον
  generator. Το bench_config.txt γράφει πλέον ΚΑΙ τα ramp/basket geometry
  (έλειπαν — τα #39/#40 δεν είχαν καταγράψει τα ramp env).
  Offline verified: default SDF πανομοιότυπο (front 0.500/top 0.128),
  lowered δίνει front 0.450/top 0.058.
- **Run A (lowered_basket_A)**: floor front 0.45 / top 0.058, bar
  0.500→(0.455, 0.058) flush, best config #40 (nip 0.540, gap 56, tilt 35,
  Rw 60, 25 rad/s), crest target 0.085.
  - Path: `runtime/intake_sweeps/lowered_basket_A`.
  - Αποτέλεσμα: required 3/6 — capture + both-wheel + inward transport ✓,
    ΟΧΙ crest, stall 22s (νέο failure mode: wedge στο (0.494, 0.035) μέσα
    στο πίσω άκρο των τροχών, συνεχής επαφή 21.4s).
  - **Μέτρηση-κλειδί**: ελεύθερο βαλλιστικό apex κέντρου **0.0755**
    (vz 0.51 m/s στο z 0.062)· η μπάλα άγγιξε τη γωνία του πατώματος
    (0.45, 0.058) ακριβώς στο apex (dist=0.033) και αναπήδησε μπροστά.
- **Ενεργειακό ισοζύγιο (από τα δεδομένα)**: ελάχιστο δυνατό hopper floor
  = 0.052 (chassis top) → απαιτεί κέντρο 0.085· το kick δίνει 0.0755 ⇒
  έλλειμμα ~10mm ΣΤΟ apex. Αλλά στο apex η μπάλα κρατά vx=0.93 m/s
  (οριζόντια KE ≈ 44mm ύψους). Η ενέργεια επαρκεί — είναι λάθος
  κατεύθυνσης. Κανένα flat πάτωμα δεν τη σώζει· χρειάζεται καμπύλη
  ανακατεύθυνση (το «catch pocket» του σχεδιασμού).
- **Run B (lowered_basket_B)**: ski-jump μέσω υπάρχοντος ramp profile
  (κανένας νέος κώδικας): entry 0.500, knee (0.455, 0.045), end
  (0.420, 0.085)· floor 0.052/front 0.42· crest 0.080 (πάνω από το
  ελεύθερο apex 0.0755 ⇒ περνά μόνο με πραγματική ανακατεύθυνση)·
  tilts 30/35/40.
  - **tilt 30: required 5/6 — ΠΡΩΤΟ crest crossing σε όλο το Phase 3**
    (peak 0.0823). tilt 35: 5/6, peak **0.0941**. tilt 40: 4/6 (0.0773 —
    πιο απότομο kick, χειρότερη πρόσβαση στο jump).
  - Το redirect δουλεύει: +19mm ύψος πάνω από το βαλλιστικό ταβάνι στο
    tilt 35. Μόνο failure: stall — η μπάλα καρφώνεται στο ΧΕΙΛΟΣ του jump
    (0.42, 0.085): πέρασμα πάνω από lip κοστίζει κέντρο 0.085+0.033=0.118,
    έφτασε 0.094 με ~0 ταχύτητα → γύρισε πίσω στη σφήνα (0.494, 0.035).
  - **Μετρημένο ενεργειακό ταβάνι**: capture E ≈ 0.140-0.146m ισοδύναμο
    ύψος, απώλειες redirect ≈ 0.05m ⇒ κέντρο-ceiling ≈ 0.093-0.094 ⇒
    το lip πρέπει ≤ 0.060 για να περνά το κέντρο (lip+0.033).
- **Run C (lowered_basket_C)**: ίδια αρχιτεκτονική, χαμηλότερο/ηπιότερο
  jump: knee (0.465, 0.020), exit lip (0.425, **0.055**) — μόλις 3mm πάνω
  από το πάτωμα 0.052 (πέρασμα κοστίζει 0.088 < ceiling 0.093, με 5mm
  περιθώριο)· tilts 30/35.
  - **Αποτέλεσμα: required 6/6 ΚΑΙ στα δύο tilts — ΠΡΩΤΟ ΠΛΗΡΕΣ PASS του
    Phase 3 σε όλη την ιστορία του project. Stall 0.00s.**
  - Τροχιά: peak 0.088 πάνω στο χείλος (dx≈0.411) → πέρασμα → προσγείωση
    στο hopper floor → κύλισμα ως το βάθος του καλαθιού, τελική ανάπαυση
    (dx≈-0.06, z=0.085 = κέντρο μπάλας πάνω στο πάτωμα 0.052). Καμία
    επιστροφή πάνω από το 3mm lip, καμία σφήνα στους τροχούς.
  - Inward velocity 1.10 (tilt 30) / 0.76 (tilt 35) m/s.
- **Run C ×5 (lowered_basket_C_x5): PASS 5/5 — το Phase 3 gate ΚΛΕΙΝΕΙ.**
  Κάθε repeat: required 6/6, crest crossing ✓, stall 0.00s, capture inward
  velocity 0.65-0.89 m/s, τελική ανάπαυση εξαιρετικά συνεπής
  (dx=-0.065±0.001, z=0.085 = κέντρο μπάλας πάνω στο hopper floor, βαθιά
  μέσα στο καλάθι). Πρώτο repeatable full-path αποτέλεσμα
  capture→transport→jump→hopper του project (2026-07-10/11).
- **Working γεωμετρία (bench, όλα σε ground frame)**: nip 0.540, gap 56mm,
  Rw 60mm, tilt 35°, GB37 251RPM (25/26.3 rad/s, 1.77 N·m), spring k=1000·
  bar/jump: entry 0.500 → knee (0.465, 0.020) → lip (0.425, 0.055)·
  basket floor top 0.052 (chassis-flush), front edge 0.42· crest
  criterion 0.080.

## 2026-07-11

### 42. Phase 4: full intake (funnel+ramp) + lateral envelope
- Commit checkpoint πριν τη φάση: **79b7020** (όλη η dual-wheel υλοποίηση
  Phase 0-3 + Phase 3 pass, εγγραφές #24-#41).
- **Setup**: winning γεωμετρία του #41 (nip 0.540, gap 56, Rw 60, tilt 35°,
  25 rad/s, jump 0.500→(0.465,0.020)→(0.425,0.055), floor 0.052/0.42,
  crest 0.080), `INTAKE_SWEEP_PHASE=full` (funnel+ramp ΜΑΖΙ πρώτη φορά),
  lateral offsets -80/-40/0/+40/+80mm.
- **Γνωστό ρίσκο προς μέτρηση**: τα funnel cheeks μπήκαν στο #24 για
  nip 0.590 (rear tip x=0.647)· με nip 0.540 υπάρχει ~35mm αφύλαχτο κενό
  cheek-tip→τροχούς. Αν off-centre μπάλες χάνονται εκεί, τα cheeks θα
  μετατοπιστούν πίσω αναλογικά (0.647-0.050).
- **Αποτελέσματα envelope (phase4_full_envelope)**: **PASS 5/5 — required
  6/6 σε ΟΛΑ τα offsets**, stall 0.00s παντού:
  | offset | inward m/s | τελική θέση |
  |---|---|---|
  | -80mm | 0.96 | (-0.065, 0.000, 0.085) |
  | -40mm | 0.72 | (-0.069, 0.000, 0.085) |
  | 0 | 0.80 | (-0.065, 0.000, 0.085) |
  | +40mm | 1.01 | (-0.065, 0.000, 0.085) |
  | +80mm | 1.15 | (-0.068, 0.000, 0.085) |
  Το ρίσκο του cheek-gap ΔΕΝ υλοποιήθηκε: το funnel κεντράρει (τελικό
  y=0.000 σε όλα), κάθε μπάλα καταλήγει στο ίδιο σημείο μέσα στο καλάθι.
  Πρώτο πλήρες funnel+wheels+jump+hopper πέρασμα του project.
- **Boundary repeatability (phase4_boundary_x5): PASS 10/10** — required
  6/6 σε κάθε repeat και στα δύο όρια (±80mm), stall 0.00s παντού,
  inward 0.60-1.11 m/s, τελική θέση (-0.063..-0.068, z 0.085) σε όλα.
  **Το Phase 4 bench gate κλείνει — το dual-wheel concept πέρασε και τις
  4 φάσεις του Concept Validation Plan στο bench.**
- **Εκκρεμότητες για live/full integration (επόμενη δουλειά)**:
  1. **Basket IR beams**: στο `tennis_robot.urdf.xacro` το ζεύγος είναι στο
     `basket_ir_z=0.113` base_link = **0.158 ground** — σχεδιασμένο για το
     scoop-era πάτωμα 0.128. Με το νέο hopper (floor 0.052, μπάλα που
     μπαίνει/κάθεται σε κέντρο 0.085-0.094) η μπάλα ΔΕΝ κόβει ποτέ τη
     δέσμη → το collection count δεν θα μετρήσει. Θέλει χαμήλωμα στο
     ~0.085-0.090 ground (base_link z≈0.040) μαζί με το πάτωμα.
  2. `analyze_collect_bag.py` + `run_collect_test.sh` είναι roller-era
     (συνειδητή εκκρεμότητα #25) — προσαρμογή για το collect_one live test.
  3. Τα νέα ramp/basket bench values να γίνουν defaults του
     sim/run_ubuntu.sh flow όταν επιβεβαιωθεί το live collect_one.

### 43. Live integration: defaults + beams + Jazzy motion chain· approach = νέο μπλόκο
- **Defaults flip (winning γεωμετρία #41-#42 παντού)**: generator + mesh
  script + xacro args + sweep harness + analyzer defaults έγιναν τα
  bench-proven (nip 0.540, gap 56, tilt 35, jump 0.500→(0.465,0.020)→
  (0.425,0.055), basket 0.42/0.052, crest 0.080). Offline verified στο
  env-free SDF. run_ubuntu.sh fallbacks διορθώθηκαν (ήταν 0.590/0.060) και
  το docker-compose πλέον περνά ΟΛΑ τα dual-wheel envs (πριν ΔΕΝ περνούσε
  κανένα — μόνο τα roller-era).
- **Basket IR beams**: 0.158 → **0.085 ground** (ισημερινός μπάλας στο
  hopper). Controller collection zone: BASKET_MIN_BALL_Z 0.12→0.075,
  zone x (0.0,0.42)→(-0.12,0.42).
- **sim.launch.py σέβεται πλέον τα ROBOT_*_FILE env overrides** (ήταν
  hardcoded στα root-owned runtime/ αρχεία — τα stale αρχεία διαγράφηκαν·
  το πρώτο live test διάβασε mode=collect από το παλιό αρχείο).
- **ΡΙΖΑ: η αλυσίδα κίνησης του live ήταν διπλά νεκρή στο native Jazzy**
  (το docker/Humble του run_ubuntu.sh δεν επηρεάζεται):
  1. Ο Ros2MotorAdapter δημοσίευε στο σκέτο `/cmd_vel` → gz bridge → το
     ΑΦΑΙΡΕΜΕΝΟ gz diff-drive plugin. Fix: `/cmd_vel_collection` (twist_mux
     input, priority 70).
  2. Jazzy twist_mux + diff_drive_controller είναι TwistStamped-only
     (κανένα use_stamped param)· όλοι οι producers μιλούν Twist. Fix:
     distro-aware wiring στο sim.launch.py — νέο node
     `cmd_vel_stamp_relay` (Twist→TwistStamped restamp) μπροστά από τα mux
     inputs (collection+teleop) και mux output κατευθείαν στο stamped
     `~/cmd_vel`. Στο Humble μένει η παλιά αλυσίδα αναλλοίωτη.
- **Live collect_one (full stack: Gazebo+YOLO perception+controller)**:
  ο robot πλέον ΚΙΝΕΙΤΑΙ και τρέχει τον πλήρη FSM κύκλο
  (scan→align→approach→capture→reverse_clear) με ζωντανό ball tracking.
  0 collected όμως — **ground truth (48s trace): ελάχιστη απόσταση από
  οποιαδήποτε μπάλα 0.73m**. Τα capture γίνονται στα τυφλά μακριά από
  πραγματικές μπάλες.
- **Νέο μπλόκο (εκτός intake): perception-guided approach.**
  - Η κάμερα (0.443m, 15.6° κάτω) χάνει τη μπάλα στο near field
    (~<0.9m από τη βάση) → το τελευταίο σκέλος είναι dead-reckoning σε
    remembered target με συσσωρευμένο σφάλμα → αστοχία >0.7m.
  - Τα status δείχνουν άλματα distance/target switching (πολλές μπάλες,
    ball_map memory), bearing που δεν συγκλίνει ποτέ <0.07 rad.
  - Το intake ΔΕΝ δοκιμάστηκε live — καμία μπάλα δεν έφτασε στο funnel.
    Το bench-proven capture μένει έγκυρο· το κενό είναι στο να ΦΤΑΣΕΙ
    η μπάλα στο στόμιο.
- **Επόμενα (νέα εκστρατεία, όχι intake)**: (α) near-field στρατηγική —
  χαμηλότερη/δεύτερη κάμερα ή IR-assisted τελική ευθυγράμμιση ή
  approach profile που κλειδώνει heading πριν το blind zone·
  (β) έλεγχος βαθμονόμησης perception bearing/distance→world
  (το πρώτο detection είχε αναντιστοιχία με το ground truth ball_02)·
  (γ) collect_one live rerun με τηλεμετρία gz-vs-perception ανά frame.

### 44. Live approach εκστρατεία: 4 στρώματα διορθώθηκαν, μένει το nip-entry με κυλιόμενη μπάλα
- **Επιβεβαίωση perception**: το πρώτο live detection ταίριαξε με την ball_09
  του κόσμου με σφάλμα **7.5cm στο 1.1m** — το YOLO+depth→world είναι
  βαθμονομημένο σωστά. Το AI pipeline (προσομοίωση OAK-D) λειτουργεί.
- **Στρώμα 1 — παγωμένο lock**: το collect_one κλείδωνε τη θέση-στόχο ΜΙΑ
  φορά στην πρώτη θέαση από το scan (3-5m, μέγιστο σφάλμα) και οδηγούσε
  τυφλά εκεί. Fix: **lock refresh** σε κάθε νεότερη θέαση (gate 0.6m κατά
  του target-stealing) — το σφάλμα πέφτει στο ~7cm της τελευταίας θέασης.
- **Στρώμα 2 — capture profile**: capture από 0.34m με budget 2.8s (0.39m
  τυφλής διαδρομής — δεν έφτανε καν τους τροχούς). Fix: capture_distance
  1.0m (πριν την επαφή με funnel, μέσα στην ορατότητα), timeout 10s,
  commit-ευθεία <0.45m, clamp στο capture steering. Compose capture
  speeds 0.07/0.05 → 0.14 (bench-proven).
- **Στρώμα 3 — τροχοί νεκροί στο capture**: το gate
  `intake_roller_latched |= intake_beam_broken` δεν άνοιγε ΠΟΤΕ live:
  μετρήθηκαν **0/2336 beam fires σε 60s** ενώ ο robot bulldoze-άρει μπάλες
  9.5m! Αιτία: το beam στο x=0.670 (#24 υπολόγισε επαφή στο 0.613) αλλά η
  πραγματική πρώτη επαφή με tilt 35° είναι ball-centre **0.645** → η
  σπρωγμένη μπάλα φτάνει max 0.678 και μόλις γλείφει το beam. Fix διπλό:
  CAPTURE πλέον ΑΓΕΤΑΙ χωρίς gate (committed ingest· το gate μένει στο
  APPROACH για το real-hw σκεπτικό) ΚΑΙ ir_x 0.670→**0.720**.
  Επιβεβαίωση: 2240 samples με τροχούς σε πλήρη περιστροφή στο capture.
- **Στρώμα 4 — odom yaw**: τα nodes κατανάλωναν το ΩΜΟ
  `/diff_drive_controller/odom` (yaw παραμορφωμένο από το wheel_separation
  fudge 1.0 vs 0.70) ενώ το EKF που φτιάχτηκε γι' αυτό τάιζε μόνο TF.
  Fix: `_odom_remap` → `/odometry/filtered`. Παράπλευρο: το EKF απέκλινε
  25m εκτός γηπέδου επειδή fuse-άρει accelerometer ax (double-integration
  runaway) → imu0 πλέον ΜΟΝΟ gyro yaw-rate. Αποτέλεσμα: **lateral aim
  0.00-0.08m** στα capture creeps (μέσα στο ±8cm funnel envelope).
- **Εναπομένον (νέο, καθαρά οριοθετημένο)**: η μπάλα πλέον ΜΠΑΙΝΕΙ στον
  διάδρομο με σωστή στόχευση, αλλά **wedge στη βάση του jump (0.478,
  z=0.037) επί 10s** — το γνωστό wedge pocket του Run A — με kick που
  φτάνει z=0.074 (οριακά κάτω από το χείλος 0.088). Κρίσιμη διαφορά από
  το bench: εκεί η μπάλα ήταν ΣΤΑΤΙΚΗ (σχετική ταχύτητα 0.12 στην επαφή)·
  live η μπάλα ΚΥΛΑΕΙ μπροστά από το robot από την επαφή με το funnel →
  σχεδόν μηδενική σχετική ταχύτητα στο nip → ασθενέστερη αρπαγή.
- **Επόμενη εκστρατεία (bench, deterministic — όχι live whack-a-mole)**:
  αναπαραγωγή του live σεναρίου στο bench (μπάλα που κυλά μπροστά από το
  robot / έλεγχος πραγματικής σχετικής ταχύτητας στην πρώτη επαφή), jam
  instrumentation στο wedge (κατάρρευση joint velocity υπό φορτίο vs
  effort 1.77), και sweep των υποψήφιων μοχλών: wheel speed στο capture,
  commit distance, micro-stop πριν το nip, γεωμετρία εισόδου jump.

## 2026-07-12

### 45. Καλάθι v2: βυθισμένο πλεγματένιο αφαιρούμενο μπιν — PASS envelope + retention
- **Design spec**: `docs/basket-bin-redesign-spec-el.md` (αποφάσεις χρήστη:
  ~50 μπάλες, αφαιρούμενο, πλέγμα· βυθισμένο πάτωμα αντί flap/ψηλού lip).
- **Υλοποίηση**: μπιν x 0.02-0.42 / ±0.14 / πάτωμα **0.030** (25mm
  συγκράτηση κάτω από το lip 0.055, 25mm διάκενο εδάφους) / τοίχοι 0.25·
  ΠΡΑΓΜΑΤΙΚΟ άνοιγμα πλάκας (2 πλαϊνές λωρίδες |y| 0.15-0.29 + πίσω μπλοκ
  x −0.46…0.01)· μπαταρία 6.26kg (19.8×16.6×17) στο πίσω deck
  x −0.226…−0.06 με πραγματική μάζα/αδράνεια (λαμπωμένη βάση 18.9kg)·
  lidar mast (−0.08 → −0.42, εκτός εσωτερικού)· basket IR ζεύγος στο
  **entry plane x=0.40** (στο 0.28 η κυλιόμενη μπάλα με κέντρο 0.063 θα
  περνούσε ΚΑΤΩ από τη δέσμη 0.085)· BASKET_ZONE (0.02, 0.42), half width
  0.14· electronics visual στο aft deck.
- **Παγίδες που πιάστηκαν στην πορεία**:
  1. Πρώτο envelope run απέτυχε 0/5 με τη μπάλα να ΡΟΛΑΡΕΙ (peak 0.061 =
     0.84²/2g·1.4 ακριβώς) αντί να εκτοξεύεται: το harness default για το
     tilt ήταν ακόμα 0.0 — είχα ενημερώσει gaps/nip/crest αλλά ΟΧΙ το tilt
     (fix: 35.0). Δίδαγμα: όταν γυρνάς winning τιμές σε defaults, έλεγξε
     ΚΑΘΕ fallback αλυσίδα (env → harness → generator → xacro).
  2. Το «άνοιγμα σασί = σημείωση κατασκευής» ήταν λάθος: χωρίς πραγματικό
     άνοιγμα στο collision, η μπάλα πατούσε στην ΠΛΑΚΑ (0.052) και το
     retention βάθος δεν υπήρχε στη φυσική. Τα fixed links δεν συγκρούονται
     ΜΕΤΑΞΥ ΤΟΥΣ, αλλά η ΜΠΑΛΑ βλέπει τα πάντα.
  3. Rebuild ΕΝΩ τρέχει sweep = σπασμένο case (controllers inactive).
- **Αποτελέσματα (πλήρες build)**:
  - Envelope (binv2_final_envelope): **PASS 5/5, 6/6 required, stall 0.00**
    σε όλα τα offsets ±80mm. Τελική ανάπαυση (0.073, **z=0.063**) = πάνω
    στο βυθισμένο πάτωμα ✓ (όχι στην πλάκα).
  - **Retention test (πρώτο του project): PASS 15/15** — 15 μπάλες στο
    μπιν, sprint 0.5 m/s + hard stop + spins ±2 rad/s + arcing + απότομη
    όπισθεν: **0 διαφυγές**.
- Εκκρεμή για το concept: rolling-ball nip entry (bench campaign #44) και
  camera blind zone — ανεξάρτητα από το καλάθι. OpenSCAD χτίζεται πάνω στο
  spec.

### 46. Rolling-ball campaign: η κόψη του 0.12 και το χαμηλό lip (v2.1)
- **Deterministic αναπαραγωγή του live #44**: drive-speed sweep
  (rolling_drive_sweep) — στο 0.14/0.25/0.40 η μπάλα φτάνει κυλώντας,
  παίρνει ασθενέστερο kick (peak 0.083/0.076/0.075 αντί 0.088) και
  σφηνώνει ΑΚΡΙΒΩΣ στο live wedge (0.494, 0.036). Αποκάλυψη-κλειδί: και
  το «winning» 0.12 είχε peak 0.088 = ακριβώς την απαίτηση — **το σύστημα
  περνούσε στην κόψη**· +0.02 m/s drive αρκούσε να το ρίξει.
- **Wheel speed 26.3 (max μοτέρ): ΚΑΜΙΑ αλλαγή** στα peaks — η μεταφορά
  ενέργειας είναι γεωμετρικά περιορισμένη, όχι από ταχύτητα (συνεπές
  με #34). Ο μοχλός με headroom είναι η ΑΠΑΙΤΗΣΗ, όχι η προσφορά.
- **Fix (v2.1)**: lip 0.055→**0.045** + πάτωμα μπιν 0.030→**0.025**
  (το βυθισμένο μπιν του #45 έδωσε τον χώρο: συγκράτηση μένει 20mm,
  διάκενο εδάφους 20mm). Απαίτηση εισόδου 0.078 → πραγματικό περιθώριο.
- **Κριτήριο-παγίδα στον analyzer**: με lip 0.045 η μπάλα μπαίνει με pivot
  ~0.077 = ακριβώς το crest plane → το "crossing" δεν καταγραφόταν παρότι
  η μπάλα ΗΤΑΝ στο μπιν (stall 0, rest 0.058). Προστέθηκε τίμιο κριτήριο:
  hopper_entry = τελική θέση ΜΕΣΑ στο μπιν (x 0.02-0.42, z≤0.075) OR
  crest crossing. Το μπιν είναι προσβάσιμο ΜΟΝΟ πάνω από το lip, άρα
  τελική θέση μέσα = απόδειξη εισόδου χωρίς false positive.
- **Αποτελέσματα (νέα defaults: RAMP_END_Z 0.045, FLOOR 0.025,
  crest 0.077)**:
  - Drive sweep 0.12/0.14/0.25: **PASS 3/3**, stall 0.00, in_hopper ✓.
  - Envelope ±80mm @ drive 0.14 (lowlip_final_envelope): **PASS 5/5**.
  - Retention 15 μπάλες + sprint/φρένα/spins/όπισθεν: **PASS 15/15, 0
    διαφυγές** και με τη 20mm συγκράτηση.
- **Συμπέρασμα**: το rolling-ball μπλόκο του live (#44) έκλεισε ΧΩΡΙΣ
  αλλαγή στο control — η γεωμετρία ανέχεται πλέον capture 0.12-0.25 m/s.
  Εκκρεμεί live collect_one rerun για το end-to-end (μαζί με τα camera
  blind zone / approach: η μπάλα πρέπει πρώτα να φτάνει στο funnel).

### 47. Live collect_one end-to-end: PASS controller count, με frame-fix στο /sim/balls
- **Harness fix**: το collect_one driver στο `run_native_intake_sweep.sh`
  είχε παλιό call σε ανύπαρκτο `run_probe_and_summarize`; αντικαταστάθηκε με
  `start_probe` → `wait_for_probe` → `summarize_probe` και ξεκινά πλέον
  `log_gz_poses.py` ώστε το live run να έχει release criteria.
- **Controller/sim fix**:
  - Το Gazebo basket beam μπορεί να ανάψει πριν η μπάλα μπει πραγματικά στο
    hopper, άρα στο sim το collection confirmation πρέπει να έρχεται από
    `/sim/balls` + basket volume και όχι από IR fallback.
  - Το ROS bridge για `/gz/pose_info` δεν είναι αξιόπιστη πηγή entity names.
    Το `gazebo_extras_node` διαβάζει πλέον direct `gz topic /world/.../pose/info`
    και δημοσιεύει `/sim/balls`.
  - Τα Gazebo world coords μετατρέπονται σε odom frame με βάση τη σχέση
    Gazebo robot pose ↔ `/odometry/filtered`; το `gazebo_extras_node` πήρε το
    ίδιο `/odom:=/odometry/filtered` remap με controller/perception.
  - Χαμηλό hopper v2.1: `BASKET_MIN_BALL_Z=0.055` και one-shot guard ανά sim
    ball def για να μη διπλομετράει μέχρι να γίνει remove.
- **Verification**: καθαρό run μετά από rebuild
  `runtime/intake_sweeps/20260712_195801`:
  - `release_criteria`: **required 6/6** (capture, both rollers, ramp climb,
    crest/hopper entry, no stall, positive inward transport).
  - `robot_status/summary`: **balls_collected=1**, `collector_state=survey`,
    mode `collect_one`.
  - `launch.log`: `ball collected -> removing ball_02 from world`.
- **Σημείωση για analyzer**: μετά το successful controller confirmation το sim
  αφαιρεί τη `ball_02` και το probe συνεχίζει να τρέχει όσο το robot επιστρέφει,
  οπότε το `final_in_hopper=false` στο τελικό pose snapshot δεν ακυρώνει το
  end-to-end pass. Για collect_one το authoritative κλείσιμο είναι
  `balls_collected=1` + remove event.

### 48. Physical retention gate: checkpoint PASS, smooth-entry repeatability FAIL

- Προστέθηκε `analyze_basket_evidence.py`: απαιτεί target dwell >=0.75s,
  settled speed <=0.08m/s για >=0.50s, target παρούσα στο τέλος, retention
  όλων των `stored_ball_*` και pitch/roll εντός 8deg. Η άμεση διαγραφή δεν
  μπορεί πλέον να περάσει ως φυσική επιτυχία.
- Το Gazebo κρατά πλέον τη collected μπάλα ως physics entity (legacy remove
  μόνο με `SIM_REMOVE_COLLECTED_BALL=true`). Το `collect_one` έχει stationary
  settle phase με 0.25s intake follow-through πριν από την επιστροφή.
- Το collect-one harness έγινε deterministic: prelaunch `idle`, σειριακοί
  controller spawners, explicit `set_pose` της `ball_02` και απομόνωση των
  υπόλοιπων court balls.
- **Unloaded centre baseline**, `runtime/intake_sweeps/20260712_220758`:
  transport **6/6**, basket evidence **8/8**, dwell 5.268s, settled 0.574s.
- **Controlled y=-0.08**, `runtime/intake_sweeps/20260712_221919`:
  transport **6/6**, target retained, dwell 3.088s, αλλά settled μόνο 0.050s
  → basket evidence **7/8 FAIL**. Η μπάλα κύλησε προς τον πίσω τοίχο και η
  έναρξη επιστροφής την ξανακίνησε πλευρικά.
- **Controlled y=-0.08 με 2s hold**,
  `runtime/intake_sweeps/20260712_222213`: στιγμιαίο collection event αλλά
  τελική target θέση `(x=0.498, y≈0, z=0.034)` έξω μπροστά από το bin,
  transport **5/6**, basket evidence **5/8 FAIL**.
- **Stop gate**: δεν ξεκινά loaded campaign 15/25/45. Το άδειο basket δεν
  έχει ακόμη repeatable smooth settling/retention. Επόμενο πείραμα πρέπει να
  αλλάξει μηχανική απόσβεση/συγκράτηση (rear/side lining, floor friction ή
  entry geometry), όχι να χαλαρώσει τα evidence thresholds.

## 49. Wire-mesh contact model: edge gates 4/5

- Το basket παραμένει κατασκευαστικά **συρμάτινο πλέγμα**, όχι συμπαγές
  κουτί. Τα xacro visuals αποδίδουν ανοιχτό grid 40mm, ενώ τα solid collision
  envelopes παραμένουν μόνο ως ισοδύναμη προσέγγιση για μπάλα 66mm.
- Το generated SDF μοντελοποιεί effective mesh rolling/contact losses:
  `floor mu=1.0`, `wall mu=0.8`, restitution `0.05`, floor `kp/kd=12000/80`
  και wall `kp/kd=4000/120`. Οι τιμές είναι environment-parameterized.
- Single smoke `y=-0.08`, `runtime/intake_sweeps/20260713_082238`: basket
  evidence **8/8**, dwell 7.332s, settled 1.282s, τελική θέση
  `(0.053, -0.107, 0.058)` μέσα στο bin.
- Repeatability `y=-0.08`, `runtime/intake_sweeps/20260713_082401`:
  **4/5 PASS**. Το ένα FAIL κατέληξε έξω μπροστά (`x=0.498`), άρα η
  συγκράτηση βελτιώθηκε αλλά παραμένει οριακή.
- Συμμετρικό edge `y=+0.08`, `runtime/intake_sweeps/20260713_082906`:
  **4/5 PASS**. Το μοναδικό FAIL δεν μπήκε ποτέ στο hopper (`dwell=0`,
  τελική `x=1.075`), επομένως είναι capture/transport failure. Στα τέσσερα
  collected runs η φυσική retention ήταν **4/4**.
- **Gate**: δεν ξεκινά load 15/25/45 πριν περάσουν με 4/5 και οι ενδιάμεσες
  unloaded συνθήκες `y=-0.04,0,+0.04`.

## 50. Passive-carriage instrumentation A/B: το validation μένει OFF

- Το `sim_physics_probe.py` καταγράφει wheel command/actual, robot twist και,
  όταν είναι διαθέσιμα, θέση/ταχύτητα των spring carriages. Για να εκτεθούν
  όμως τα passive carriage joints στο `/joint_states`, πρέπει να δηλωθούν ως
  state-only joints στο `gz_ros2_control`, κάτι που μπορεί να επηρεάσει τον
  τρόπο με τον οποίο το Gazebo χειρίζεται τα joints.
- Προστέθηκε το diagnostic flag `INTAKE_EXPOSE_CARRIAGE_STATE`, με default
  **`false`**. Με `false` τα carriage joints παραμένουν κανονικά στο μηχανικό
  URDF αλλά δεν αποτελούν μέρος του `ros2_control` system. Με `true` εκτίθενται
  μόνο `position`/`velocity`, χωρίς command interface.
- **A/B, unloaded centre, ίδιο collect_one setup, 5 runs ανά condition**:
  - `OFF`, `runtime/intake_sweeps/20260713_111939`: basket evidence **5/5
    PASS**, contact duration `0.336-0.476s`, χωρίς stall και χωρίς carriage
    samples, όπως αναμενόταν.
  - `ON`, `runtime/intake_sweeps/20260713_112506`: basket evidence **4/5
    PASS**. Το FAIL r3 είχε contact duration `17.574s`, wheel median actual
    speed `0rad/s` και τελική target θέση `x=0.497m` μπροστά από το bin. Τα
    carriages έφτασαν περίπου `4.85-4.99mm` στα πέντε runs.
- **Συμπέρασμα**: η καθαρή επιτυχία προηγούμενων instrumentation-ON runs δεν
  αποδεικνύει μηχανική βελτίωση. Το authoritative validation εκτελείται με
  carriage exposure **OFF**. Το ON χρησιμοποιείται μόνο για διάγνωση και τα
  αποτελέσματά του δεν αναμειγνύονται με release-gate datasets.
- **Gate**: το centre case είναι πλέον 5/5 με το authoritative OFF setup, αλλά
  δεν ξεκινά ακόμη load 15/25/45. Πρέπει πρώτα να επαναληφθούν OFF οι
  ενδιάμεσες θέσεις `y=-0.04,+0.04` και να περάσουν τουλάχιστον 4/5.

## 51. Controlled-launch profile: transport PASS, moving retention FAIL

- Προστέθηκε opt-in `INTAKE_RAMP_PROFILE=launch` στον generated SDF. Το
  rolling profile παραμένει default. Το launch kicker χρησιμοποιεί cubic
  Hermite καμπύλη με χαμηλή/οριζόντια είσοδο και παραμετρικά
  `INTAKE_LAUNCH_EXIT_X_M`, `INTAKE_LAUNCH_EXIT_Z_M` και
  `INTAKE_LAUNCH_EXIT_ANGLE_DEG`.
- Πρώτη γεωμετρία: local exit `x=0.465`, `z=0.032`, tangent `35deg`. Με το
  funnel-frame offset, το generated collision τελειώνει περίπου σε
  `x=0.450`, `z=0.030`, με μετρημένη τελευταία κλίση `34.8deg` και air gap
  περίπου 25mm μέχρι το basket lip.
- **Stationary centered bench**, `runtime/intake_sweeps/20260713_121229`:
  **3/3 transport 6/6 και basket evidence 8/8**. Roller contact duration
  `0.099-0.121s`, peak inward speed `1.06-1.08m/s` και μόνο 4 ramp-contact
  samples ανά run. Στο r1 η airborne τροχιά είχε περίπου
  `vx=-0.99m/s`, `vz=+0.52m/s` (`27.5deg`) πριν την είσοδο στο bin.
- **Centered collect_one**, `runtime/intake_sweeps/20260713_121607`:
  controller count **3/3**, αλλά physical basket evidence **0/3 (7/8)**.
  Και οι τρεις μπάλες μπήκαν, έμειναν `4.11-4.21s` και settled για
  `1.57-1.62s`, αλλά μετά την κίνηση του robot κατέληξαν στις μπροστινές
  γωνίες σε `x≈0.444`, έξω από το bin boundary `x<=0.420`.
- **Συμπέρασμα**: ο controlled launch λύνει το rolling-ramp stall και αξίζει
  να συνεχιστεί. Η αρχική αποτυχία δεν ήταν αναπήδηση πάνω από πλήρες lip:
  το 180mm channel άφηνε δύο ακάλυπτες μπροστινές γωνίες 50mm μέσα στο
  basket πλάτους 280mm. Η μπάλα έφτανε κοντά στον πίσω τοίχο, settled, και
  κατά την κίνηση του robot κύλαγε γύρω από το κεντρικό lip.
- Προστέθηκαν δύο mesh-equivalent corner retainers `10x50x20mm` στα
  `x=0.425`, `y=+/-0.115`, αφήνοντας το κεντρικό 180mm launch opening
  ανεπηρέαστο.
- Το πρώτο corrected run στο `runtime/intake_sweeps/20260713_122832` πέρασε
  transport **6/6** και retention **8/8**. Το επόμενο run συλλέχθηκε επίσης,
  αλλά το harness έχασε το probe start window επειδή το launch ολοκληρώθηκε
  πριν από το παλιό threshold 0.45m. Για launch profile το observation window
  ξεκινά πλέον αυτόματα στα 0.70m.
- **Clean corrected retest**, `runtime/intake_sweeps/20260713_123745`:
  **3/3 transport 6/6 και retention 8/8**, stall 0. Τελικές θέσεις
  `x=0.301-0.364`, αντί για `x≈0.444` έξω από τις ανοικτές γωνίες.
- **Official corrected gate**, `runtime/intake_sweeps/20260713_124148`:
  **5/5 transport 6/6 και retention 8/8**, stall 0, με carriage exposure OFF.
  Τελικές θέσεις `x=0.266-0.389`.

## 52. Loaded rollback και ballistic release contract

- Το sweep harness δημιουργεί πλέον πραγματικό preload με ονόματα
  `stored_ball_00...`, ίδιο mass/radius/friction με τις court balls και
  readiness gate που απαιτεί να εμφανιστεί ακριβώς ο ζητούμενος αριθμός στο
  ground-truth pose log πριν ξεκινήσει το `collect_one`.
- Στο πρώτο έγκυρο load-15 run χωρίς κεντρικό χείλος,
  `runtime/intake_sweeps/20260713_132448`, η `stored_ball_09` κύλησε από
  `x=0.35` έξω από το bin στο `x=0.421` ενώ η target ήταν ακόμη στο
  `x=0.859`. Έμεινε στο handoff (`x≈0.428`) και μπλόκαρε την target στα
  `x≈0.491`. Η αστοχία ήταν rollback του φορτίου, όχι αδυναμία motors.
- Προστέθηκε παραμετρικό fixed centre lip `10x180x20mm` στο `x=0.425`, μέσω
  `BASKET_CENTER_LIP_HEIGHT_M` (default `0.020`, `0` για A/B). Το unloaded
  gate `runtime/intake_sweeps/20260713_140430` πέρασε transport **6/6** και
  basket **8/8**. Στα loaded runs συγκράτησε σταθερά **15/15**, άρα λύνει το
  rollback, αλλά η εισερχόμενη μπάλα δεν καθαρίζει την πρώτη σειρά.
- Οι μονοπαραμετρικές αλλαγές δεν έλυσαν το loaded handoff:
  `40deg` (`20260713_141114`) basket **4/8**, `30rad/s`
  (`20260713_144237`) **4/8**, spring `1500N/m`
  (`20260713_144738`) **5/8**. Και στις τρεις περιπτώσεις το φορτίο έμεινε
  15/15, αλλά η target επέστρεψε περίπου στο `x=0.493-0.494`.
- Το νέο `analyze_launch_ballistics.py` κάνει fit με simulation time στα πρώτα
  καθαρά airborne samples. Ξεκινά από το στενό window `x=0.52..0.45` και, όταν
  η ταχύτητα/sampling δώσει λιγότερα από τρία samples, επεκτείνεται αυτόματα
  μόνο μέχρι την πρώτη προσγείωση, αποκλείοντας τη μετέπειτα κύλιση. Γράφει
  release vector, speed, angle, predicted apex/range/front-row
  clearance/landing και target errors στο `launch_ballistics.json` κάθε run.
- **Ballistic contract**: πραγματικό release περίπου `x=0.504,z=0.058`,
  στόχος δεύτερης σειράς `x=0.28`, apex `z=0.135` και first-row clearance
  `z>=0.124`. Η inverse-ballistic λύση από το μετρημένο release point είναι
  περίπου inward `vx=0.893m/s`, `vz=1.231m/s`, speed `1.521m/s`, angle
  `54.1deg`, range περίπου `0.224m`.
- **Τρέχουσα ρύθμιση 35deg/25rad/s/k=1000**, από
  `runtime/intake_sweeps/20260713_140430`: inward `vx=1.019m/s`,
  `vz=0.588m/s`, speed `1.177m/s`, πραγματική γωνία `30.0deg`, predicted
  apex `0.075m`, range `0.122m`, landing `x=0.382`. Υπάρχει ήδη περισσότερη
  από την απαιτούμενη οριζόντια συνιστώσα (`+0.127m/s`), αλλά λείπουν
  `0.644m/s` κατακόρυφης ταχύτητας. Επόμενες αλλαγές αξιολογούνται ως
  calibration map `RPM/angle/gap/stiffness -> actual vx/vz`, όχι μόνο με
  collection count ή ονομαστικές ρυθμίσεις.

## 53. Carriage telemetry και stiffness 1200 N/m

- Με launch geometry `exit x=0.500m`, `z=0.032m`, tangent `70deg`, gap `56mm`
  και `k=1000N/m`, τα carriages άνοιξαν συμμετρικά περίπου `4.93-4.96mm`.
  Δεν τερμάτισαν το διαθέσιμο travel των `8mm`, άρα το nominal squeeze των
  `5mm/side` εφαρμόζεται μέσω των passive springs και δεν είναι rigid jam.
- Το telemetry sweep `20/25/30rad/s` έδειξε ότι τα `20rad/s` αύξησαν την
  επαφή σε `0.685s`, αλλά με χαμηλότερο release speed `1.011m/s`. Τα
  `25rad/s` έδωσαν το καλύτερο `vz/vx=1.065`, ενώ τα `30rad/s` έστρεψαν την
  ενέργεια οριζόντια (`vz/vx=0.778`). Η διάρκεια επαφής μόνη της δεν αρκεί.
- Το πρώτο `k=1200` run στο
  `runtime/intake_sweeps/carriage_telemetry_wspeed_25_k1200_valid` είχε μόνο
  δύο samples στο παλιό στενό ballistic window. Η πραγματική τροχιά ήταν
  διαθέσιμη, αλλά περνούσε περίπου `35mm` ανά sample. Προστέθηκε adaptive
  fit-until-landing και regression test για sparse γρήγορη τροχιά.
- Με τον διορθωμένο analyzer, το πρώτο `k=1200` run έδωσε `5` fit samples,
  speed `1.329m/s`, inward `vx=1.090m/s`, `vz=0.761m/s`, angle `34.9deg`,
  landing `x=0.326` και basket **8/8**. Το repeat
  `carriage_telemetry_wspeed_25_k1200_rerun` έδωσε `6` samples, speed
  `1.234m/s`, `vx=1.073m/s`, `vz=0.610m/s`, angle `29.6deg`, landing
  `x=0.354` και basket **6/8**.
- **Συμπέρασμα**: το `k=1200` αυξάνει κυρίως την οριζόντια μετάδοση και δεν
  είναι repeatable αρκετά για default. Παραμένει experiment· το ballistic
  contract είναι **1/4** και στα δύο runs.

## 2026-07-14

### 54. Loaded 45-ball campaign: entry hood + rear clearance 120mm — δύο 8/8 configs, n=1

- **Provenance**: τα runs της 14/7 έγιναν χωρίς live log update· η ενότητα
  αυτή ανασυγκροτήθηκε από `runtime/intake_sweeps/*` (`bench_config.txt`,
  `basket_evidence.json`, `launch_ballistics.json`). Μελλοντικά: log στο
  ίδιο turn, όπως ορίζει ο κανόνας.
- **Load 15**: PASS 8/8 (`wheel_only_35_load_15_retry`). Το πρώτο attempt
  μέτρησε `retained 0/15` με 0 διαφυγές και target μέσα — pose-log glitch,
  όχι φυσική αστοχία.
- **Load 25**: baseline (`wheel_only_35_load_25`) **3/8 FAIL** — target
  εκτινάχθηκε μπροστά (`x=0.463`) και 1 stored διέφυγε. Receiver μόνο του
  **4/8** (target κόλλησε στο handoff `x=0.501`)· receiver −5mm **3/8** με
  διαφυγή. Το **low_transition** το έλυσε: **PASS 8/8**, target
  `x=0.400, z=0.067`, dwell 2.856s.
- **Load 45 (το πρόβλημα του γεμάτου καλαθιού) — δύο συνιστώσες**:
  1. *Διαρροή φορτίου στην είσοδο*: με low_transition μόνο, **3 stored
     διέφυγαν** και η target εκτινάχθηκε στο `x=0.666` → **3/8 FAIL**.
  2. *Απόρριψη target πάνω στον σωρό*: το **entry hood** (roof + cheeks,
     rear overhang 40mm, rear clearance 105mm) έλυσε τη διαρροή —
     **45/45 retained σε ΟΛΑ τα επόμενα runs** — αλλά η target συνέχισε
     να απορρίπτεται (`x=0.663`, 4/8).
- **Rear clearance 105→120mm**: η target πλέον μένει ΜΕΣΑ
  (`x≈0.40-0.42, z≈0.067` = πάνω στον σωρό). 7/8 με μόνο marginal
  dwell 0.682 < 0.75. Με drive 0.14: 6/8 (dwell 0.404).
- **Probe window ήταν μέρος των "FAIL"**: τα dwell/settled κόβονταν από το
  probe duration 25s, όχι από τη φυσική. Με **probe 35s** δύο 8/8 PASS:
  - `wheel_only_hood_rear120_drive014_load_45_longprobe` (rolling,
    drive 0.14, phase funnel, wheel_max_vel 35): dwell 1.808s,
    settled 1.524s, target `x=0.399`.
  - `launch_hood_rear120_load_45_probe35` (launch profile, drive 0.12,
    phase full, wheel_max_vel 26.3): dwell 1.98s, settled 1.566s,
    target `x=0.388`. Ballistics: release 43.1°, 1.185m/s,
    vz error **−0.47m/s** έναντι contract.
- **Angle 55° attempt** (`launch_angle55_hood_rear120_load_45_probe35`):
  η πραγματική γωνία ΕΠΕΣΕ στα 41.3° (από 43.1°) και το vz error έμεινε
  −0.46m/s → το ονομαστικό exit angle ΔΕΝ μεταφράζεται σε release angle·
  γεωμετρικό όριο μεταφοράς ενέργειας (συνεπές με #34/#46/#52). 7/8 FAIL
  μόνο στο `target_settled` (0.4 < 0.5s). Σημείωση: με hood+rear120 το
  ballistic contract (54.8°/1.57m/s) ΔΕΝ φαίνεται πλέον απαραίτητο — η
  μπάλα προσγειώνεται στο `x≈0.36-0.38` πάνω στον σωρό και μένει.
- **Display bug (μόνο εμφάνιση)**: τα echo fallbacks του
  `bench_config.txt` για `basket_floor_front_x/top_z` (0.50/0.128) δεν
  συμφωνούν με τα generator defaults (0.42/0.025). Τα runs έτρεξαν με τα
  σωστά 0.42/0.025· να διορθωθεί το echo στο sweep script.
- **Gate**: και τα δύο winning configs είναι **n=1**. Επόμενο βήμα:
  5x repeatability και για τα δύο (out dirs
  `wheel_only_hood_rear120_drive014_load_45_repeat5`,
  `launch_hood_rear120_load_45_repeat5`), κριτήριο ≥4/5, carriage
  exposure OFF. Μετά επιλογή default (rolling+drive014 vs launch).

### 55. 5x repeatability gate load 45: launch profile 5/5, wheel-only 2/5

- Σειριακό 5x run και για τα δύο configs του #54, ίδιο κοινό setup
  (load 45, probe 35s, hood rear overhang 40mm / rear clearance 120mm,
  carriage exposure OFF). Πριν την εκκίνηση σκοτώθηκε ορφανό headless
  `gz sim` από το τελευταίο run της 14/7 — μοιραζόταν gz transport topics
  με το ίδιο world name και θα μόλυνε το pose logging.
- **Config A — wheel_only rolling, drive 0.14, phase funnel, wmax 35**
  (`wheel_only_hood_rear120_drive014_load_45_repeat5`): **2/5 FAIL**.
  - r1/r3: η target ΔΕΝ μπήκε ποτέ στο μπιν (dwell 0, τελικές
    `x=0.496` / `x=0.423, y=-0.055` — η δεύτερη ακριβώς πάνω στο όριο
    0.42). Το γνωστό μοτίβο «περνάει στην κόψη» του #46.
  - r2: η target μπήκε και έκατσε (dwell 2.118s) αλλά **1 stored
    διέφυγε** → 7/8.
  - r4/r5: 8/8 PASS (dwell 2.112/1.560s).
- **Config B — launch profile, drive 0.12, phase full, wmax 26.3**
  (`launch_hood_rear120_load_45_repeat5`): **5/5 PASS 8/8**.
  Dwell 0.988-1.832s, settled 0.908-1.436s, τελικές θέσεις
  `x=0.384-0.412`, **45/45 retained και στα πέντε, 0 διαφυγές**.
- **Συμπέρασμα**: με γεμάτο καλάθι το rolling/wheel-only entry είναι
  δομικά οριακό (η μπάλα πρέπει να ανέβει πάνω στον σωρό με σχεδόν
  μηδενικό κατακόρυφο περιθώριο), ενώ ο controlled launch περνάει το
  gate καθαρά. **Προτεινόμενο default για loaded λειτουργία: launch
  profile + hood rear120.** Δεν έχει αλλάξει ακόμη κανένα default στο
  generator/harness — εκκρεμεί απόφαση χρήστη και unloaded regression
  (τα #51/#53 unloaded launch runs ήταν ήδη πράσινα σε bench, αλλά το
  τρέχον hood/rear120 combo δεν έχει τρέξει unloaded 5x).

### 56. Unloaded regression 5/5 → launch profile + hood rear120 γίνονται defaults

- **Unloaded 5x regression** του νικητή του #55
  (`launch_hood_rear120_unloaded_repeat5`): **5/5 PASS 8/8, transport 6/6
  σε όλα**. Τελική θέση `x≈0.08, z=0.058` (βαθιά στο μπιν), dwell 27-32s.
  Με αυτό το launch+hood combo είναι πράσινο και unloaded και loaded 45.
- **Default flip (πλήρης fallback αλυσίδα, δίδαγμα #45)**:
  - `scripts/generate_robot_urdf.py`: `INTAKE_RAMP_PROFILE`
    rolling→**launch**, `BASKET_HOOD_REAR_OVERHANG_M` 0.000→**0.040**,
    `BASKET_HOOD_REAR_CLEARANCE_Z_M` 0.105→**0.120**.
  - `scripts/generate_curved_scoop_mesh.py`: profile rolling→**launch**.
  - `urdf/tennis_robot.urdf.xacro`: hood args 0.000→**0.040**,
    0.105→**0.120**.
  - `run_native_intake_sweep.sh`: `RAMP_PROFILE` default **launch**
    (άρα και probe start 0.70 αυτόματα), `PROBE_DURATION` 25→**35**
    (τα dwell «FAIL» του #54 ήταν εν μέρει artifact του 25s), echo
    fallbacks συγχρονίστηκαν (hood 0.040/0.120, ramp launch) και
    διορθώθηκε το display bug floor 0.50/0.128→0.42/0.025.
  - `docker-compose.yml`: profile rolling→**launch**, ramp entry
    0.500→**0.540** (σε launch mode entry = nip· το 0.500 ήταν
    rolling-era) + passthrough για τα `INTAKE_LAUNCH_EXIT_*`.
  - `run_ubuntu.sh`: `BASKET_CENTER_LIP_HEIGHT_M` 0.020→**0.010** — τα
    validated runs έτρεξαν με το generator default 0.010· το 0.020 του
    run_ubuntu ήταν αναντιστοιχία live vs bench (το #52 έγραφε
    «default 0.020» αλλά ο generator είχε 0.010).
  - Όποιος θέλει rolling A/B πλέον το ζητά ρητά με
    `INTAKE_RAMP_PROFILE=rolling` (και δικό του entry_x).
- **Verification gate — PASS**: run με ΚΑΘΑΡΟ env (μόνο load 45 + out
  dir, κανένα intake/basket override) στο
  `runtime/intake_sweeps/defaults_flipped_load_45_verify`. Το
  bench_config έδειξε από τα defaults `ramp_profile=launch`,
  `probe_start 0.70`, hood `0.040/0.120`, launch exit `0.465/0.032/35`.
  Αποτέλεσμα: **transport 6/6, basket evidence 8/8**, dwell 2.18s,
  settled 2.066s, target `x=0.394, z=0.066`, **45/45 retained**. Η
  fallback αλυσίδα είναι πλήρης — το γεμάτο καλάθι (45) καλύπτεται πλέον
  από τα defaults του repo.

### 57. OpenSCAD basket-bin-v2 + spec sync (μηχανισμός κηρύχθηκε working)

- Απόφαση χρήστη: με τα #54-#56 ο μηχανισμός συλλογής θεωρείται
  **working**· τα υπόλοιπα validation κενά (live loaded collect_one,
  loaded lateral envelope, σταδιακό γέμισμα) θα απαντηθούν μαζί με τον
  αλγόριθμο μαζέματος μισού γηπέδου.
- **Spec sync**: το `basket-bin-redesign-spec-el.md` δεν είχε το hood —
  προστέθηκε §2 entry hood (roof 0.38→0.47, clearances 0.120/0.135,
  cheeks, sim params) και το «load 45 ανοικτό» του §8.4 έκλεισε με τα
  αποτελέσματα των #54-#56.
- **OpenSCAD** (`cad/basket-bin-v2/`): παραμετρικό μοντέλο από το spec —
  `params.scad` (μοναδική πηγή διαστάσεων, mm/ground frame), αφαιρούμενο
  mesh μπιν (πάτωμα/τοίχοι/tray/chute/guards/lip/flange/λαβές),
  chassis-mounted hood, chassis context (πλάκα με ΠΡΑΓΜΑΤΙΚΟ άνοιγμα,
  μπαταρία, IR ζεύγος, μπάλες κλίμακας), assembly με exploded view.
  Renders + STL export επαληθεύτηκαν με το `openscad` docker service.
- **Ευρήματα κατά τη μετάφραση sim→κατασκευή**:
  1. Τα hood cheeks του sim (x 0.42-0.47) ΔΙΑΠΕΡΝΟΥΝ τα corner guards
     του μπιν (x 0.42-0.43) — στο Gazebo αόρατο (fixed links δεν
     συγκρούονται), στην κατασκευή αδύνατο: δύο ξεχωριστά parts. Στο CAD
     τα cheeks κόπηκαν σε x 0.43-0.47 (το 10mm slot που μένει είναι
     πολύ κάτω από τη διάμετρο μπάλας).
  2. Το μπιν ΔΕΝ βγαίνει με σκέτο κατακόρυφο lift όσο το hood είναι
     πάνω: η υποδοχή (μέρος του μπιν) βρίσκει το roof μετά από ~85mm.
     Άρα το hood πρέπει να είναι βιδωτό/ανακλινόμενο, hood-off-first.
- Εκκρεμότητες CAD: λεπτομέρεια στήριξης hood (transverse bar vs funnel
  frame), IR beam vs mesh alignment, plywood-cut-list αναθεώρηση.

### 58. collect_route: 360° scan → route plan → Nav2 legs (κώδικας, unit-tested)

- **Στόχος** (issue #10): γρήγορο μάζεμα όλων των μπαλών του μισού γηπέδου
  πάνω στον working μηχανισμό. Σχεδίαση στο νέο
  `docs/collection-route-plan-el.md`.
- **Υπόθεση**: 360° scan (create gate 3→9 m στο BallMap) + greedy NN/2-opt
  ordering + Nav2 legs + ο υπάρχων P-controller για το τελικό capture
  αρκούν· μπάλες κοντά σε φράχτη/φιλέ θέλουν πλάγιο approach heading ώστε
  το funnel corridor (±0.17 m) να μένει καθαρό.
- **Υλοποίηση**: `collection_route_planner.py` (planner library: CourtModel
  από court_boundary.json v2, order_route, cheapest_insertion,
  approach_pose_for_ball direct/lateral), `collect_route_mission.py` (FSM
  scan→plan→nav→approach→settle, insertion, fail-loud στο Nav2),
  controller wiring (mode `collect_route`, multi-ball frame feed στο scan,
  route/order/metrics στο Collection Map payload), κουμπί «Collect Route».
  Ο Nav2LaneNavigator κατασκευάζεται πλέον όποτε τα deps υπάρχουν (το
  lawnmower path συνεχίζει να τιμά το COLLECTION_USE_NAV2).
- **Αποτέλεσμα**: 35 νέα unit tests πράσινα (planner 16, mission 14,
  ball_map export 5)· πλήρης σουίτα 84 passed (το test_console_app
  collection error προϋπάρχει στο main, περνάει standalone).
- **Status**: κώδικας ΟΚ offline· **εκκρεμεί sim επαλήθευση** κατά τα
  βήματα του §8 στο collection-route-plan-el.md (Nav2 stack up, lateral
  κοντά σε φράχτη/φιλέ, insertion mid-route, route overlay στο console).
- **➡️ Συνέχεια ΕΚΤΟΣ αυτού του log**: ο αλγόριθμος μαζέματος έχει πλέον
  δικό του τεκμηριωμένο log — `docs/collection-route-debug-log-el.md`
  (εγγραφή #1 = αυτή η υλοποίηση, με τα ευρήματα αναλυτικά). Το παρόν log
  μένει για ό,τι αφορά τον ΜΗΧΑΝΙΣΜΟ intake (γεωμετρία, roller, basket).

## Σημαντικά reference numbers (μη τα ξαναϋπολογίζεις)
- Roller/channel effective world position (τρέχοντα defaults
  `INTAKE_ROLLER_X_OFFSET_M=0.015`, `INTAKE_ROLLER_Z_OFFSET_M=-0.005`):
  `x=0.615, z=0.107` (ground frame, base_footprint).
- Debug camera mount: `base_link` frame, `xyz="0.45 0 -0.03"`,
  `rpy="0 -0.51 0"` → base_footprint pose `0.45 0 0.015 0 -0.51 0`.
- Target-direction unit vector (camera pos → roller target):
  `dx=0.165, dz=0.092 → normalized (0.8736, 0, 0.4870)`.
