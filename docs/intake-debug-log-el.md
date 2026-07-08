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
- **Status**: ⏳ υλοποίηση/test9.

## Σημαντικά reference numbers (μη τα ξαναϋπολογίζεις)
- Roller/channel effective world position (τρέχοντα defaults
  `INTAKE_ROLLER_X_OFFSET_M=0.015`, `INTAKE_ROLLER_Z_OFFSET_M=-0.005`):
  `x=0.615, z=0.107` (ground frame, base_footprint).
- Debug camera mount: `base_link` frame, `xyz="0.45 0 -0.03"`,
  `rpy="0 -0.51 0"` → base_footprint pose `0.45 0 0.015 0 -0.51 0`.
- Target-direction unit vector (camera pos → roller target):
  `dx=0.165, dz=0.092 → normalized (0.8736, 0, 0.4870)`.
