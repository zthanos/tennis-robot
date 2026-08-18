# WS2 — Pi bring-up (Ubuntu 24.04 aarch64 / ROS 2 Jazzy)

Στόχος (Gate WS2): το workspace **χτίζει καθαρά** στο Pi, ένα control node
**ξεκινά idle** χωρίς sim, και το **onnxruntime φορτώνει** σε ARM64. Δεν
εγκαθίσταται Gazebo — η προσομοίωση μένει στο PC και φτάνει στο Pi μέσω δικτύου
(WS3).

Αρχιτεκτονική: **PC = Gazebo + ros_gz bridge** (sensors, `/clock`, δέχεται
`/cmd_vel*`) · **Pi = controller_node + perception_node + Nav2 + slam_toolbox +
web console**. Δες [pi-deployment-plan-el.md](pi-deployment-plan-el.md).

## Προϋποθέσεις
- Raspberry Pi με **Ubuntu 24.04 aarch64** (`lsb_release -a`, `uname -m`).
- Δίκτυο + το repo cloned στο Pi (ίδιο path δεν χρειάζεται — το `TENNIS_ROBOT_ROOT`
  προκύπτει από το script).
- ≥ 8 GB κάρτα/δίσκος (η nav2 + slam apt εγκατάσταση θέλει χώρο).

## Βήματα

```bash
git clone <repo> tennis-robot && cd tennis-robot   # ή pull αν υπάρχει ήδη

# 1) Πρώτη φορά (εγκαθιστά και το ROS 2 Jazzy ros-base):
INSTALL_ROS=true ./scripts/setup_pi.sh

# ή, αν το ROS 2 Jazzy υπάρχει ήδη:
./scripts/setup_pi.sh
```

Το `setup_pi.sh` (idempotent):
1. (προαιρετικά) προσθέτει το ROS 2 apt repo + `ros-jazzy-ros-base`.
2. εγκαθιστά **μόνο τα Pi-side ROS πακέτα** — navigation2, nav2-bringup,
   slam-toolbox, robot-localization, twist-mux, robot-state-publisher, tf2, xacro,
   rmw-fastrtps-cpp + build tools (**όχι** Gazebo/ros_gz/ros2_control).
3. `rosdep install` με `--skip-keys` για τα sim-only deps (τρέχουν στο PC).
4. `pip install` τα runtime deps (numpy, opencv-headless, duckdb, **onnxruntime**)
   στο system Python (`--user --break-system-packages`, PEP 668).
5. `colcon build` τα 3 πακέτα → `ros2_ws/install_jazzy/`.

## Επαλήθευση (Gate WS2)

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install_jazzy/setup.bash

# (α) python runtime deps φορτώνουν σε ARM64
python3 -c 'import onnxruntime, duckdb, cv2, numpy; print("py deps OK")'

# (β) το C++ plugin package είναι εγκατεστημένο
ros2 pkg prefix tennis_robot_collection_controller

# (γ) ένα control node ξεκινά idle χωρίς sim (Ctrl-C να το κλείσεις)
export TENNIS_ROBOT_ROOT="$PWD"
ros2 run tennis_robot controller_node
```

Gate περνά όταν: (α) τυπώνει `py deps OK`, (β) δίνει path, (γ) το controller_node
μένει ζωντανό σε idle (δεν κρασάρει — δεν χρειάζεται sensors για να idle-άρει).

## Troubleshooting

- **onnxruntime ARM64 (Ρίσκο #1):** το wheel υπάρχει στο PyPI για `onnxruntime>=1.20`
  σε aarch64. Αν το `pip install` δεν το βρει: δοκίμασε piwheels
  (`pip install onnxruntime --extra-index-url https://www.piwheels.org/simple`) ή
  apt (`sudo apt install python3-onnxruntime` αν υπάρχει). Ο ball detector είναι
  **fail-loud** χωρίς αυτό — το `perception_node` δεν ξεκινά. (Στο distributed
  setup το perception μπορεί προσωρινά να τρέχει στο PC μέχρι να λυθεί.)
- **colcon build killed (OOM) σε Pi με λίγη RAM:** χτίζουμε μόνο 3 μικρά πακέτα
  (η nav2 έρχεται από apt, όχι από source), αλλά αν συμβεί, χτίσε σειριακά:
  `cd ros2_ws && MAKEFLAGS=-j1 colcon build --executor sequential --parallel-workers 1 \
  --build-base build_jazzy --install-base install_jazzy --packages-select \
  tennis_robot_msgs tennis_robot_collection_controller tennis_robot`. Πρόσθεσε swap
  αν χρειαστεί.
- **rosdep unresolved keys:** τα `ros_gz_*`, `gz_ros2_control`, οι ros2_control
  spawners **skip-άρονται επίτηδες** (τρέχουν στο PC). Το warning είναι αναμενόμενο.
- **`No module named 'em'/'lark'` στο build:** uv-managed python shadow-άρει το
  system — το script βάζει `/usr/bin` πρώτο, αλλά σε νέο shell κάνε
  `export PATH=/usr/bin:$PATH` πριν το build.

## Επόμενο — WS3 (distributed ROS 2)
PC sim ↔ Pi control μέσω κοινού `ROS_DOMAIN_ID` + ίδιο RMW + ίδιο LAN. Όλα τα Pi
nodes `use_sim_time:=true` κλειδωμένα στο `/clock` του PC. Χρειάζεται ένα
**control-only launch** για το Pi (partition των nodes PC↔Pi) — αυτό είναι το
πρώτο βήμα του WS3.
