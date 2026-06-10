# Milestone A — Court Mapping (slam_toolbox)

Πρώτο βήμα της by-the-book αυτόνομης πλοήγησης: χαρτογράφηση του γηπέδου με
`slam_toolbox`, πάνω στο sensor + ros2_control contract που ήδη φτιάξαμε.

## Τι κάνει

```text
/scan (LiDAR) + /odom + TF
        ↓
slam_toolbox (mapping)  →  /map  +  TF: map → odom
        ↓
RViz: βλέπεις τον χάρτη να χτίζεται
```

Η οδήγηση κατά τη χαρτογράφηση γίνεται με τυποποιημένο teleop μέσω `twist_mux`
(το κανονικό cmd_vel arbiter — αντικαθιστά το custom relay για τη διαδρομή
πλοήγησης).

```text
teleop → /cmd_vel_teleop ┐
nav    → /cmd_vel_nav     ├─ twist_mux → /diff_drive_controller/cmd_vel_unstamped
collect→ /cmd_vel_collection ┘
```

## Εκτέλεση (sim)

Τρία terminals:

```bash
# 1) Robot + Gazebo + ros2_control
ros2 launch tennis_robot sim.launch.py

# 2) SLAM mapping + twist_mux
ros2 launch tennis_robot slam_mapping.launch.py

# 3) Teleop — οδήγησε για να χαρτογραφήσεις το γήπεδο
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

Στο RViz: Fixed Frame = `map`, πρόσθεσε display `Map` (topic `/map`) και `TF`.

## Αποθήκευση χάρτη

```bash
# Serialized (για slam_toolbox localization αργότερα):
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: 'runtime/court_map'}"

# Ή κλασικό map_server format (.pgm + .yaml, για Nav2):
ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap "{name: {data: 'runtime/court_map'}}"
```

## Real robot

Ίδιες εντολές, αλλά:
- `real_sensors.launch.py` αντί για το sim (rplidar + OAK-D drivers),
- `use_sim_time:=false` στο `slam_mapping.launch.py`.

Τίποτα άλλο δεν αλλάζει — ίδια topics, ίδια frames.

## Γιατί δεν χρειάστηκε custom survey κώδικας

Όλη η λογική scan-matching, loop closure, map building και το TF `map→odom`
έρχεται από το `slam_toolbox`. Το παλιό `lidar_survey.py` / `court_boundary.json`
δεν χρειάζεται πια για τη χαρτογράφηση.

## Επόμενα

- **Β:** Αυτόνομη εξερεύνηση (Nav2 + explore_lite) ώστε να χαρτογραφεί μόνο του
  αντί για teleop.
- **Γ:** Συλλογή μπαλών — mission node στέλνει `NavigateToPose`, collection FSM
  για το τελευταίο ~0.5m.
