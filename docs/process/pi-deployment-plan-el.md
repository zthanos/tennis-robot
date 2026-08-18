# Pi Deployment Plan — Ubuntu 24.04 / ROS 2 Jazzy

## Context

**Στόχος:** το control stack + το exposed control-panel UI να τρέχουν σε Raspberry
Pi (`tennisserver`, Ubuntu 24.04 aarch64, native ROS 2 Jazzy), με τον Gazebo sim
στο PC, πάνω από distributed ROS 2 σε κοινό LAN. Ο πραγματικός μηχανισμός/hardware
έρχεται αργότερα.

**Το βασικό εύρημα:** και το PC και το Pi είναι **Ubuntu 24.04 + ROS 2 Jazzy**, αλλά
το codebase στοχεύει **ROS 2 Humble** — ο sim τρέχει σήμερα σε **Humble Docker
container** (`Dockerfile.gazebo` = `ros:humble-ros-base`, `run_ubuntu.sh` → docker
compose). Άρα το deployment είναι στην ουσία **migration Humble → Jazzy**, όχι copy.

**End-state:** όλα **native Jazzy, χωρίς Docker**, same-distro DDS.
- **PC:** Gazebo Harmonic + ros_gz bridge (publishes `/scan`,`/odom`,camera,`/clock`;
  subscribes `/cmd_vel*`).
- **Pi:** `controller_node` + `perception_node` + Nav2 + `CollectionFollowPath` C++
  plugin + control panel (exposed).
- Gazebo Harmonic + Jazzy είναι **επίσημα supported ζευγάρι** (`ros-jazzy-ros-gz`),
  άρα ο sim τρέχει native Jazzy και το Humble container πετιέται.

**Σειρά:** **PC-first.** Λύνουμε το migration στο x86_64 dev machine (γρήγορο, με GUI),
μετά το Pi (WS2) γίνεται «ίδια συνταγή σε ARM64».

---

## WS1 — Humble → Jazzy migration (PC, prerequisite)

Στόχος: το workspace χτίζει και τρέχει **native Jazzy** στο PC, ο sim χωρίς container.

1. **Deps (apt, jazzy):** `ros-jazzy-desktop` (ή `-ros-base`) + `ros-jazzy-ros-gz`
   (Harmonic), `ros-jazzy-navigation2` + `ros-jazzy-nav2-bringup`,
   `ros-jazzy-slam-toolbox`, `ros-jazzy-tf2-ros`, `ros-jazzy-cv-bridge`,
   `ros-jazzy-rmw-cyclonedds-cpp` (ή default Fast DDS). `onnxruntime` για τον ball
   detector (lazy import — module φορτώνει χωρίς αυτό, runtime το απαιτεί).
2. **C++ `CollectionFollowPath` plugin → Jazzy Nav2 API** (πιο πιθανό σημείο αλλαγής):
   `nav2_core::Controller` interface + lifecycle/pluginlib διαφορές Humble→Jazzy.
   Adapt + rebuild `tennis_robot_msgs`, `tennis_robot_collection_controller` με Jazzy
   `rosidl`/`ament`.
3. **rclpy nodes σε Python 3.12** (Jazzy): verify (το `typing.Self`→`TypeVar` fix
   ήδη έγινε· το 3.12 είναι forward-compatible). Pure pytest τρέχει ήδη πράσινο στο
   3.12 (321).
4. **Launch/config:** τα 10 launch files (`sim.launch.py`, `navigation.launch.py`,
   `tennis_robot.launch.py`, ...) verify σε Jazzy Nav2 (param names/plugin lists
   άλλαξαν σε μερικά Nav2 nodes). `nav2_params.yaml` review.
5. **Docker/docs cleanup:** ενημέρωση/απόσυρση `Dockerfile.gazebo` + `docker-compose*`,
   νέο native-Jazzy entry point αντί `run_ubuntu.sh`→docker. Ενημέρωση `CLAUDE.md`
   («ROS 2 Humble»→«Jazzy») + docs.

**Gate WS1:** `colcon build` καθαρό σε Jazzy (3 πακέτα) · C++ gtests πράσινα σε
Jazzy · pure pytest 321 · **sim launch native Jazzy** (Gazebo Harmonic + Nav2 +
nodes) · ένα collect_route run end-to-end στο PC χωρίς container.

### WS1 status (2026-07-23) — σχεδόν κλειστό, ΚΑΜΙΑ αλλαγή στο plugin

Validated native Jazzy στο PC:
- **Build καθαρό** (3 πακέτα)· το C++ `CollectionFollowPath` plugin compile-άρει στο
  Jazzy Nav2 API **χωρίς κώδικα**.
- **Tests:** tracking_core gtests 9/9 (incl. #30), path_canon/plugin/runtime 13/13,
  pure pytest 321.
- **Native sim launch** (headless): Gazebo Harmonic (gz 8.11) + ros_gz bridge + και
  τα 15 nodes σηκώνονται· gz_ros2_control φορτώνει (`controller_manager` up,
  `Received robot description`).

Διορθώσεις (branch `feat/pi-deployment`):
1. **Build recipe:** οι ROS builds πρέπει να χρησιμοποιούν το **system Python**
   (`/usr/bin`), όχι το uv-managed (`~/.local/bin/python3.12`) που shadow-άρει το PATH
   (μόνο το system έχει `empy`/`lark`+ROS modules). Θα μπει σε native build script.
2. **Jazzy Nav2 goal_checker:** το Jazzy απαιτεί non-empty `goal_checker_id` στο
   FollowPath goal (Humble το δεχόταν κενό). Runtime ήδη OK· διορθώθηκε το isolated
   launch test.
3. **gz_ros2_control plugin path:** το DEB είναι στο `/opt/ros/jazzy/lib`· το
   `sim.launch.py` έδειχνε container path — τώρα καλύπτει και τα δύο via
   `AMENT_PREFIX_PATH`.
4. **Python runtime deps (pip):** `duckdb` (control panel) + `onnxruntime` (perception)
   — ήταν baked στο Humble container· χρειάζονται explicit install (και για το Pi/WS2).

Ανοιχτό (minor): 1 survey-**RPP** integration test failure (γενικός Nav2 controller,
όχι το δικό μας plugin) — status 6 αντί 4· θέλει έναν έλεγχο. Εναπομείναν πλήρες gate:
collect_route run native Jazzy end-to-end.

---

## WS2 — Pi bring-up (aarch64)

1. Install ROS 2 Jazzy στο Pi (`-ros-base` + nav2 + slam_toolbox + control-panel deps).
2. Clone repo, `colcon build` το workspace στο **ARM64**.
3. **Ρίσκο #1 — onnxruntime ARM64:** επιβεβαίωσε wheel/build για aarch64
   (`pip install onnxruntime` ή `onnxruntime` apt/build). Ο ball detector fail-loud
   χωρίς αυτό.
4. **Ρίσκο #2 — Gazebo στο Pi:** ΔΕΝ τρέχει sim στο Pi (μένει στο PC). Το Pi χρειάζεται
   μόνο τα control/perception/Nav2 πακέτα — όχι `ros-gz-sim`.

**Gate WS2:** workspace χτίζει καθαρά στο Pi · τα Pi-side nodes ξεκινούν χωρίς sim
(π.χ. controller_node idle) · onnxruntime φορτώνει.

### WS2 — GATE PASSED (2026-07-24, μέσω SSH στο πραγματικό Pi)
Pi: Raspberry Pi (aarch64, host `tennisserver`), **Ubuntu 24.04.4**, 4 cores,
**15 GB RAM**, 256 GB NVMe (SPCC M.2 PCIe HAT, `/dev/nvme0n1`) + 48 GB / ,
passwordless sudo. Bring-up μέσω SSH (key `~/.ssh/id_pi`, `thanos@192.168.31.111`).
NVMe: unpartitioned/unmounted ακόμη — το workspace είναι στο `/` (48 GB, αρκετά)·
προαιρετική μελλοντική μετακίνηση workspace/build στον NVMe για ταχύτητα/χώρο.

**Gate WS2 verify:** (a) `py deps OK` — onnxruntime **1.27.0**, numpy 2.5.1,
duckdb 1.5.5, opencv-headless 5.0.0.93, όλα arm64 wheels (Ρίσκο #1 λύθηκε)·
(b) `tennis_robot_collection_controller` package present + 2 plugin xml·
(c) `controller_node` idle χωρίς sim (alive 8s, killed by timeout — όχι crash).
`colcon build` 3 πακέτα καθαρό (msgs 39s, C++ plugin 1m6s, tennis_robot 3s).

**Δύο Pi-image ιδιομορφίες που χρειάστηκαν fix (τώρα στο `setup_pi.sh`):**
1. Το image ήρθε **χωρίς `noble-updates` pocket** → οι patched runtime libs
   (liblz4-1/libzstd1/zlib1g 1build1.1) δεν είχαν matching `-dev` → ros-base
   "held broken packages". Fix: `_ensure_noble_updates`.
2. `unattended-upgrades` κρατούσε το dpkg lock στο boot → apt "Could not get
   lock". Fix: `_apt_prepare` (stop periodic apt units + wait for lock) +
   `DEBIAN_FRONTEND=noninteractive`/`NEEDRESTART_MODE=a`.

`scripts/setup_pi.sh` (idempotent, control-only — navigation2, slam-toolbox,
robot-localization, twist-mux, robot-state-publisher, tf2, xacro, rmw-fastrtps·
**όχι** Gazebo/ros_gz/ros2_control· rosdep `--skip-keys`· pip deps· colcon build)
+ `docs/process/pi-setup-el.md` (οδηγός + gate + troubleshooting).

**Επόμενο: WS3** — control-only launch (partition PC↔Pi) + distributed DDS.

---

## WS3 — Distributed ROS 2 (PC sim ↔ Pi control)

1. **Discovery:** κοινό `ROS_DOMAIN_ID`, ίδιο RMW, ίδιο LAN subnet → auto multicast
   discovery. Αν το multicast είναι flaky → Fast DDS discovery server.
2. **Partition:** PC = Gazebo + ros_gz bridge (sensors + `/clock` out, `/cmd_vel*` in).
   Pi = controller_node, perception_node, Nav2 lifecycle, CollectionFollowPath, panel.
3. **Sim time:** όλα τα Pi nodes `use_sim_time:=true`, κλειδωμένα στο `/clock` του PC.
4. **Verify:** topic list/echo cross-machine, latency `/scan`,`/odom`,camera PC→Pi και
   `/cmd_vel` Pi→PC αποδεκτό για tracking.

**Gate WS3:** Pi βλέπει sensors του PC, PC δέχεται cmd_vel του Pi, `/clock`
συγχρονισμένο, καμία type-hash σύγκρουση (same-distro).

---

## WS4 — Exposed UI από το Pi

1. Control panel bind σε `0.0.0.0` (όχι localhost) — έλεγχος `scripts/control_panel.py`
   host arg.
2. Firewall/port open· πρόσβαση από οποιαδήποτε συσκευή στο LAN: `http://<pi-ip>:<port>`.

**Gate WS4:** το panel ανοίγει από άλλο μηχάνημα, δείχνει live status + speed/elapsed,
τα κουμπιά (Collect Route/Stop) λειτουργούν.

### WS3 + WS4 — GATE PASSED (2026-07-24, live PC↔Pi)
Launch split: `sim.launch.py` env-gated `TENNIS_LAUNCH_SIM`/`TENNIS_LAUNCH_BRAIN`
(both default true = all-in-one). PC distributed = `TENNIS_LAUNCH_BRAIN=false
./run_native.sh` (sim only). Pi = `./run_pi.sh` (brain: controller + perception
+ SLAM mapping + Nav2 + panel, `TENNIS_LAUNCH_SIM=false`), shared
`ROS_DOMAIN_ID=42`. **Verified live** (PC sim headless ↔ Pi brain):
- Pi sees PC `/scan`,`/camera/image_raw`,`/camera/depth`,`/clock`,`/tf`,
  `/odometry/filtered`,`/sim/balls` over DDS· `/clock` sim-time flowing.
- Full Pi brain up, **no node deaths**: perception (YOLO ONNX on ARM, consuming
  PC camera), controller_node, slam_toolbox active (map->odom from PC `/scan`),
  Nav2 «Managed nodes are active».
- cmd_vel Pi→PC: Pi `/cmd_vel_nav` → PC robot moves (odom twist.x nonzero).
- WS4: Pi panel HTTP 200 cross-machine (`http://<pi>:8081`), binds `0.0.0.0`.

**Fix που χρειάστηκε (τώρα pinned):** fresh Pi pip έφερε opencv 5.0 + numpy 2.5·
perception + sensor_snapshot crash-άρανε στο `detect_court_line`
(`cv2.HoughLinesP` shape (N,1,4)→(N,4) στο 5.x). Pin `opencv<5`/`numpy<2`
(pyproject + setup_pi) = tested PC line (opencv 4.11/numpy 1.26). Επίσης: το
gitignored `models/yolov8n.onnx` πρέπει να αντιγραφεί στο Pi (scp).

**Επόμενο: WS5** — Collect Route από το Pi UI πάνω στο PC sim (χρειάζεται
`runtime/court_boundary.json` στο Pi: copy από PC ή Map Court από το Pi).

---

## WS5 — End-to-end verification

- Trigger `collect_route` από το **Pi UI**, sim στο **PC** → route completes
  cross-machine (οι 2 completions που είχαμε, τώρα distributed).
- Latency/reliability του distributed setup υπό load.

**Gate WS5:** ≥1 πλήρης route completion από Pi-driven control πάνω σε PC sim.

### WS5 — pipeline PASSED, ball-collecting run εκκρεμεί (2026-07-24)
Distributed collect_route από το Pi UI (`POST /api/command mode=collect_route`)
πάνω στο PC headless sim έτρεξε **end-to-end**: `Pi controller → navigate to
scan pose → 360° scan (perception στο PC) → plan → Pi Nav2 → PC robot →
executor terminal: completed`. Κανένα crash, controller_node ζωντανός.

**Fixes που χρειάστηκαν:** (1) `scan_timeout_s` 20→90 (distributed sweep πιο
αργός)· (2) **perception στο PC** (`TENNIS_PERCEPTION_ON_PC=true`, default για
run_native brain=false + run_pi): με perception στο Pi το streaming raw camera
PC→Pi + TF timing άφηνε το scan στο `insufficient_coverage 1/18` (~99 TF-cache
drops)· με perception δίπλα στην κάμερα (μόνο BallDetectionArray διασχίζει) το
scan καλύπτει κανονικά. **Απαραίτητο:** copy `runtime/court_boundary.json` στο Pi.

**Ball-collecting distributed run — PIPELINE OK, COLLECTION QUALITY ΑΝΕΠΑΡΚΗΣ
(2026-07-25, GUI sim + Pi brain).** Με GUI το perception (PC) επιβεβαίωσε έως 6
μπάλες σε ένα scan· αρχικό abort `insufficient_coverage 12/18` γιατί
`required_coverage_fraction` ήταν 1.0 (valid obs σε ΟΛΑ τα 18 headings — εύθραυστο
distributed). **Fix: `required_coverage_fraction` 1.0→0.6** (+ `scan_timeout_s`
20→90). Μετά το route έτρεξε & completed distributed — **ΑΛΛΑ η δοκιμή ΔΕΝ είναι
επιτυχής**: (Α) το scan δεν είδε όλες τις μπάλες (12/18 coverage + scattered balls
σε νέες θέσεις από προηγ. runs), (Β) **ο planner διάλεξε μόνο 2 από τις confirmed
μπάλες** — αυτό είναι το κύριο ανοιχτό (γιατί deferred/reject οι υπόλοιπες; θέλει
scan-diagnostic + planning-result του run).

**Ανοιχτά για επιτυχή distributed collection:** (Α) πλήρης scan coverage / clean
court· (Β) planner να επιλέγει όλες τις feasible confirmed μπάλες (debug γιατί 2)·
(Γ) collected-count telemetry δεν γράφεται στα runtime files (beam/plan
reconciliation, ήδη ανοιχτό).

**Pi DEPLOYMENT υποδομή (WS1-WS4) ΠΛΗΡΗΣ & PROVEN· WS5 pipeline completes
distributed αλλά collection quality θέλει δουλειά** (planner selection + coverage).
Επόμενο: debug planner-selects-2, coverage, mechanism/beam-reconciliation, S3-S8.

---

## Ρίσκα / αποφάσεις

- **Nav2 API Humble→Jazzy** για το C++ plugin — το πιο πιθανό build breakage (WS1.2).
- **onnxruntime aarch64** (WS2.3).
- **Cross-distro DDS** αποφεύγεται εντελώς αφού όλα πάνε Jazzy (γι' αυτό PC-first
  migration αντί να κρατήσουμε τον Humble container).
- **Rollback:** ο Humble container μένει διαθέσιμος μέχρι να περάσει το Gate WS1·
  δεν διαγράφουμε `Dockerfile.gazebo`/compose πριν επιβεβαιωθεί native Jazzy sim.

## Non-goals (αυτής της φάσης)

- Πραγματικό hardware/μηχανισμός συλλογής (επόμενη φάση).
- Phase 7 acceptance S3–S8 (μετά, από το Pi).
- ros2_control / real motor bring-up.
