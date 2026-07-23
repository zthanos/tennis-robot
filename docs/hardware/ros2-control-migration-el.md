# ros2_control Migration

Μετάβαση του robot control από τα απευθείας Gazebo plugins στο πρότυπο
`ros2_control` (Controller Manager → Resource Manager → Hardware Interfaces).

## Γιατί

Το `ros2_control` δίνει ένα σταθερό hardware abstraction: το `diff_drive_controller`,
τα command topics και ολόκληρο το behavior stack από πάνω μένουν **ίδια** για sim
και για το πραγματικό ρομπότ. Για τη μετάβαση στο hardware αλλάζει **μόνο** το
`<hardware>` plugin στο URDF.

## Πριν → Μετά

| | Πριν (legacy) | Μετά (ros2_control) |
| --- | --- | --- |
| Drive | `gz-sim-diff-drive-system` plugin, raw `/cmd_vel` | `diff_drive_controller` |
| Intake roller | `gz-sim-joint-controller-system`, `/lift_wheel/cmd` | `lift_wheel_velocity_controller` |
| Joint states | `gz-sim-joint-state-publisher-system` | `joint_state_broadcaster` |
| Command msg | `geometry_msgs/Twist` σε `/cmd_vel` | `Twist` σε `/diff_drive_controller/cmd_vel_unstamped` |

## Νέα / αλλαγμένα αρχεία

- `urdf/components/ros2_control.urdf.xacro` — `<ros2_control>` block. `sim_mode` arg
  εναλλάσσει το hardware backend: `gz_ros2_control/GazeboSimSystem` (sim) ή
  `tennis_robot_hardware/TennisRobotSystem` (real, placeholder).
- `urdf/components/drivetrain.urdf.xacro` — αφαιρέθηκαν τα 3 legacy Gazebo control plugins.
- `urdf/tennis_robot.urdf.xacro` — καλεί το νέο macro· νέα xacro args `sim_mode`,
  `controllers_config`.
- `config/controllers.yaml` — controller_manager + diff_drive + joint_state_broadcaster
  + lift wheel velocity controller.
- `launch/ros2_control.launch.py` — robot_state_publisher + spawners (jsb → diff_drive → lift).
- `tennis_robot/drive_actuator_node.py` — actuation layer· το μοναδικό node που μιλάει στα
  controller command topics. Δέχεται neutral εντολές σε `/tennis_robot/cmd_drive` (Twist) και
  `/tennis_robot/cmd_collector` (Float64), με watchdog που σταματά τη βάση σε σιωπή upstream.
- `package.xml` / `setup.py` — deps + entry point + config install.
- `scripts/generate_robot_urdf.py` — περνά `sim_mode` / `controllers_config` στο xacro.

## Command contract

```text
behavior stack
  → /tennis_robot/cmd_drive      (geometry_msgs/Twist)
  → /tennis_robot/cmd_collector  (std_msgs/Float64)
drive_actuator_node
  → /diff_drive_controller/cmd_vel_unstamped     (geometry_msgs/Twist)
  → /lift_wheel_velocity_controller/commands     (std_msgs/Float64MultiArray)
```

`use_stamped_vel: false` → απλό `Twist` (όχι `TwistStamped`).

## Build dependency (ΣΗΜΑΝΤΙΚΟ)

Με **Gazebo Harmonic + ROS 2 Humble** το `gz_ros2_control` ΔΕΝ υπάρχει ως binary —
πρέπει να χτιστεί από source στο Docker image:

```bash
apt-get install gz-harmonic ros-humble-ros-gzharmonic
git clone https://github.com/ros-controls/gz_ros2_control -b humble
export GZ_VERSION=harmonic
# rosdep install + colcon build
```

Επίσης χρειάζονται: `ros-humble-ros2-control`, `ros-humble-ros2-controllers`.

## Εκκρεμή wiring (επόμενο βήμα)

1. Να γίνει include το `ros2_control.launch.py` στο `sim.launch.py` (μετά το spawn του robot)
   και να προστεθεί ο `drive_actuator_node`.
2. Ο high-level controller (`controller_node` / `motion_controller`) να δημοσιεύει στο
   `/tennis_robot/cmd_drive` αντί για raw `/cmd_vel`.
3. Docker image: προσθήκη των ros2_control packages + build του gz_ros2_control.
4. Smoke scenario: επιβεβαίωση ότι το ρομπότ κινείται μέσω των spawned controllers.

## Καθαρισμός corruption (έγινε ταυτόχρονα)

Κατά το build verification βρέθηκαν δύο **προϋπάρχουσες** φθορές που εμπόδιζαν εντελώς
το rendering του URDF, και διορθώθηκαν:

- block από NUL bytes στο τέλος του `drivetrain.urdf.xacro` (το git το έβλεπε ως binary),
- δεκάδες `[cite: N]` markers (artifact από paste) μέσα στα xacro αρχεία.
