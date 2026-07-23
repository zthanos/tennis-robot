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

---

## WS5 — End-to-end verification

- Trigger `collect_route` από το **Pi UI**, sim στο **PC** → route completes
  cross-machine (οι 2 completions που είχαμε, τώρα distributed).
- Latency/reliability του distributed setup υπό load.

**Gate WS5:** ≥1 πλήρης route completion από Pi-driven control πάνω σε PC sim.

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
