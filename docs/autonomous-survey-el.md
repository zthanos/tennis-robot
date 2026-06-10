# Milestone B — Autonomous Court Survey (Nav2 + explore_lite)

Το ρομπότ χαρτογραφεί **μόνο του** το γήπεδο: το `explore_lite` στέλνει στόχους
στα σύνορα γνωστού/άγνωστου χώρου, το Nav2 το πάει εκεί αποφεύγοντας εμπόδια, και
το `slam_toolbox` χτίζει τον χάρτη. Όταν τελειώσει, γυρίζει στην αρχή.

## Στοίβα

```text
explore_lite  →  NavigateToPose goals (σύνορα αγνώστου)
      ↓
Nav2 (controller RPP + planner + behaviors + BT) → /cmd_vel_nav
      ↓
twist_mux → /diff_drive_controller/cmd_vel_unstamped → ros2_control
      ↑
slam_toolbox: /scan + odom → /map + map→odom
```

Όλα κάθονται πάνω στο sensor + ros2_control contract που ήδη φτιάξαμε — τίποτα
από κάτω δεν αλλάζει.

## Πρώτα: rebuild (νέα packages)

Προστέθηκαν `navigation2`, `nav2_bringup` (apt) και `explore_lite` (source).

```bash
docker compose --profile gazebo build gazebo
```

## Εκτέλεση (sim)

```bash
# 1) Robot + Gazebo + ros2_control
docker compose --profile gazebo up gazebo

# 2) Αυτόνομο survey: SLAM + Nav2 + explore_lite μαζί
docker compose --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh && . /ros2_ws/install/setup.sh && \
   ros2 launch tennis_robot autonomous_survey.launch.py'

# 3) RViz για να το δεις
docker compose --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh && . /ros2_ws/install/setup.sh && rviz2'
```

RViz: Fixed Frame = `map`· Add → `Map` (/map), `TF`, `LaserScan` (/scan),
`Costmap` (global_costmap/costmap). Το ρομπότ θα αρχίσει να κινείται μόνο του.

## Δοκιμή μεμονωμένα (χρήσιμο για debug)

Αντί για το αυτόνομο, στείλε χειροκίνητο στόχο για να ελέγξεις μόνο το Nav2:

```bash
# SLAM + nav (χωρίς explore)
ros2 launch tennis_robot slam_mapping.launch.py &
ros2 launch tennis_robot navigation.launch.py
# Στο RViz: κουμπί "2D Goal Pose" → κλικ κάπου → το ρομπότ πάει εκεί.
```

## Αποθήκευση χάρτη

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /workspace/runtime/court_map}}"
```

## Real robot

Ίδιο, με `real_sensors.launch.py` αντί για sim και `use_sim_time:=false`.

## Πιθανά σημεία ρύθμισης στην πρώτη εκτέλεση

- **Ταχύτητες/footprint:** στο `config/nav2_params.yaml` (footprint 0.92×0.58,
  `desired_linear_vel: 0.4`). Αν είναι αργό/γρήγορο, εδώ.
- **explore_lite goals:** αν «κολλάει», `min_frontier_size` / `progress_timeout`
  στο `config/explore.yaml`.
- **Costmap inflation:** `inflation_radius: 0.55` — αν δεν περνάει από στενά,
  ρίξ' το.

Πες μου το log αν κάτι δεν ξεκινά — το Nav2 θέλει συχνά ένα tuning pass την πρώτη φορά.

## Επόμενο (Milestone Γ)

Συλλογή μπαλών: mission node παίρνει ball detections (OAK-D) → `NavigateToPose`
στη μπάλα → collection FSM για το τελευταίο ~0.5 m + intake. Το **ίδιο** Nav2.
