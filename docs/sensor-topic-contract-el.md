# Sensor Topic Contract

Ενιαίο contract για τα sensors, κοινό σε sim και πραγματικό ρομπότ. Όλα τα layers
πάνω από τα drivers (perception, controller, planning) κάνουν subscribe **μόνο**
σε αυτά τα ονόματα — δεν ξέρουν αν από κάτω τρέχει Gazebo ή hardware.

## Contract

| ROS topic | Type | frame_id | Sim source | Real source |
| --- | --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | `lidar_link` | gz `/gz/lidar` → ros_gz_bridge | rplidar driver |
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8) | `camera_link` | gz `/gz/camera` → bridge | DepthAI/OAK-D driver |
| `/camera/depth` | `sensor_msgs/Image` (32FC1) | `camera_link` | gz `/gz/depth` → bridge | DepthAI/OAK-D driver |
| `/odom` | `nav_msgs/Odometry` | `odom`→`base_link` | `diff_drive_controller` | `diff_drive_controller` |
| `/ir/readings` | `tennis_robot_msgs/IrReadings` | `base_link` | `gazebo_extras_node` (από `/gz/ir_*`) | IR GPIO node |

Σημ.: το `/odom` πλέον δεν έρχεται από το Gazebo αλλά native από το
`diff_drive_controller`, ίδιο σε sim και real.

## Γιατί κρατάει την αρχιτεκτονική ενιαία

Η μόνη διαφορά sim↔real είναι **ποιος δημοσιεύει** σε αυτά τα topics:

```text
SIM:   Gazebo sensor plugin → ros_gz_bridge → /scan, /camera/*
REAL:  rplidar / DepthAI driver →             /scan, /camera/*   (ίδια ονόματα)
```

Για να παραμείνει σταθερό, ο πραγματικός driver πρέπει να ευθυγραμμιστεί σε:

1. **Topic name** — μέσω remapping στο `real_sensors.launch.py`.
2. **frame_id** — ο driver να δηλώνει `lidar_link` / `camera_link` (ίδια με το URDF),
   αλλιώς το TF δεν κλείνει.
3. **Message type/encoding** — `LaserScan`, `Image rgb8`, depth `Image 32FC1` σε **μέτρα**
   (το OAK-D βγάζει συχνά `16UC1` σε mm — χρειάζεται μετατροπή σε `32FC1` m).
4. **`use_sim_time`** — `true` στο sim, `false`/unset στο real.

## Frame tree (κοινό)

```text
odom → base_link → lidar_link
                 → camera_link
                 → left_wheel_link / right_wheel_link / lift_wheel_link
```

`base_link → *` από `robot_state_publisher` (URDF).
`odom → base_link` από `diff_drive_controller`.
