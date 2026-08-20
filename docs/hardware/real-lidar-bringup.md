# Real LiDAR bring-up on Raspberry Pi

Status: **hardware implementation and office interface validation complete**.
This document records evidence
for the LiDAR hardware interface milestone. It is not evidence for SLAM,
navigation, localisation, or outdoor/court performance.

The collection simulation planner, executor, and controller are frozen for this
milestone and must not be changed.

## L1 — Existing simulation contract

The simulated source is the generic Gazebo Harmonic `gpu_lidar` sensor named
`front_lidar`, declared by the `rplidar_c1_mount` URDF macro. Gazebo publishes a
`gz.msgs.LaserScan` on `/gz/lidar`; `ros_gz_bridge` maps it to the canonical ROS
topic `/scan` as `sensor_msgs/msg/LaserScan`.

The following values are the **declared simulation configuration**. Fields that
only exist after Gazebo-to-ROS conversion are deliberately not guessed and are
listed separately under runtime evidence.

| Property | Simulation |
| --- | --- |
| topic | `/scan` (from Gazebo `/gz/lidar`) |
| message type | `sensor_msgs/msg/LaserScan` |
| frame_id | `lidar_link` |
| configured rate | `31.25 Hz` (`32 ms` period) |
| angle range | `-pi .. +pi rad` (360 degrees) |
| angular resolution | configured 500 samples/revolution, nominally `2pi / 500 = 0.012566 rad` (`0.72 deg`) |
| samples/scan | `500` |
| min range | `0.15 m` |
| max range | `12.0 m` |
| range resolution | `0.01 m` |
| noise model | Gaussian, mean `0 m`, standard deviation `0.005 m` |
| QoS | no override in `gazebo/bridge_config.yaml`; effective publisher QoS must be captured with `ros2 topic info /scan --verbose` |

### Runtime-derived simulation fields

An existing Control Panel snapshot from the frozen simulation contains 500
ranges, `angle_increment=0.01259155385196209`, angle limits
`-3.141592741..+3.141592741`, and range limits `0.150000006..12.0 m`. The
configured 31.25 Hz remains a declaration, not a newly measured rate: the
simulation was not started for this hardware milestone.

The Jazzy `ros_gz_bridge` conversion source explicitly sets `scan_time=0` and
`time_increment=0`, because those fields do not exist in
`gz::msgs::LaserScan`. It copies the Gazebo ranges and intensities without
changing their numerical values. The repository bridge configuration does not
select a special QoS profile, so the bridge uses its default-constructed ROS 2
QoS (reliable, volatile). These source-derived facts and the archived snapshot
are sufficient for the interface comparison; they are not a new simulator
validation run.

For a future fresh simulation capture, use the following read-only commands;
do not modify the simulator to make it match the hardware:

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
ros2 topic echo /scan --once --field angle_min
ros2 topic echo /scan --once --field angle_max
ros2 topic echo /scan --once --field angle_increment
ros2 topic echo /scan --once --field time_increment
ros2 topic echo /scan --once --field scan_time
ros2 topic echo /scan --once --field range_min
ros2 topic echo /scan --once --field range_max
ros2 topic echo /scan --once --field ranges
```

The configured `31.25 Hz` must not be reported as an observed measurement.

### TF contract

The URDF publishes a fixed transform:

```text
base_link -> lidar_link
xyz = (-0.42, 0.0, 0.498) m
rpy = (0.0, 0.0, 0.0) rad
```

The Gazebo ray sensor itself has a local pose of `(0, 0, 0.035, 0, 0, 0)` inside
`lidar_link` so the simulated rays do not originate inside the visual housing,
while the message is labelled `lidar_link`. This 35 mm vertical simulation-only
offset does not change the planar scan orientation, but it is recorded here so
it is not mistaken for a measured physical mounting transform.

### Current consumers

Direct ROS consumers of canonical `/scan` found in the source tree are:

- `controller_node`: depth 1 using the rclpy default reliable/volatile QoS;
- `sensor_snapshot_node`: best-effort, volatile, keep-last depth 1; this is the
  adapter that feeds the Control Panel sensor JSON;
- `court_survey_v2_node`: best-effort, volatile, keep-last depth 1;
- `slam_toolbox`: `scan_topic: /scan`;
- Nav2 local and global costmap obstacle layers: `LaserScan` source `/scan`;
- RViz: best-effort, volatile, keep-last depth 5.

The two domain-bridge configurations also forward `/scan` during distributed
simulation. They are transport boundaries, not application consumers.

The existing Control Panel Diagnostics page does not subscribe to ROS directly.
It renders `front_lidar` snapshots written by `sensor_snapshot_node`; its current
plot already consumes `angle_min`, `angle_increment`, `range_min`, `range_max`,
and the ranges array without a Gazebo/hardware branch.

### Canonical-topic decision

`/scan` is already the sole downstream LiDAR contract. No new canonical topic is
justified. A physical driver should publish or remap into `/scan`, with any
hardware-specific normalization kept at the driver/adapter boundary.

## L2 — Physical device evidence

Repository names and old configuration are **not sufficient hardware
identification**. In particular, references to `RPLIDAR C1`, `sllidar_ros2`,
`/dev/ttyUSB0`, `460800`, and `Standard` mode in the current URDF, SLAM comments,
BOM, and `real_sensors.launch.py` started this milestone as unverified prior
assumptions.

### Enumeration captured 2026-08-19

Read-only enumeration was run over the repository's established SSH route to
the physical Pi. Gazebo and the robot runtime were not started.

| Property | Observed value |
| --- | --- |
| Pi hostname | `tennisserver` |
| OS | Ubuntu 24.04.4 LTS (`noble`) |
| kernel/architecture | `6.8.0-1060-raspi`, `aarch64` / `arm64` |
| USB device | `10c4:ea60 Silicon Labs CP210x UART Bridge` |
| kernel driver | `cp210x` |
| Linux device | `/dev/ttyUSB0` |
| stable device path | `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_3c21860b3b70f01184b98a301045c30f-if00-port0` |
| USB serial | `3c21860b3b70f01184b98a301045c30f` |
| permissions | `crw-rw---- root:dialout` |
| runtime user | `thanos`, member of `dialout`; read/write access verified after fresh login |
| installed LiDAR ROS package | none in the persistent Jazzy workspace; pinned upstream driver built successfully in `/tmp` |

This proves that a unique CP2102N serial interface is connected and gives us a
stable existing `by-id` path, so a custom udev naming rule is not currently
needed. It does **not**, by itself, prove which LiDAR model is behind the generic
USB-to-UART bridge.

The purchase record identifies the ordered unit as a Waveshare/Slamtec RPLIDAR
C1, and the C1 datasheet specifies 460800 baud. A non-scanning Slamtec
`GET_DEVICE_INFO` query at that baud returned model ID `0x41`, firmware `1.02`,
hardware revision `18`, and sensor serial
`6E4DE0F8C2E29AD2C1819FF59DFC4E1E`. Model `0x41` is the RPLIDAR C1. The exact
physical device is therefore confirmed as **Slamtec RPLIDAR C1**.

The runtime user was added to `dialout`. A fresh SSH login reports membership in
group 20 (`dialout`) and read/write access through the stable `by-id` path. No
root-run driver, `chmod 777`, or custom udev rule is required.

### Reproduction commands

The initial capture used:

```bash
uname -a
dpkg --print-architecture
lsb_release -a
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
id
```

The device-specific capture used:

```bash
udevadm info --query=all --name=/dev/ttyUSB0
udevadm info --attribute-walk --name=/dev/ttyUSB0
usb-devices
```

L2 is closed: model, USB identity, stable path, baud rate, permissions, and
non-root access are now evidenced.

## L3 — Persistent dependency and Pi-only ROS 2 bring-up

The official [`Slamtec/sllidar_ros2`](https://github.com/Slamtec/sllidar_ros2)
driver is source-pinned to commit
`34300099fadfc772965962dec837bf436706188f` in `ros2_ws/lidar.repos`.
`scripts/import_lidar_dependencies.sh` restores that exact checkout under
`ros2_ws/src/sllidar_ros2`, verifies `HEAD`, and applies the minimal
repository-managed `ros2_ws/patches/sllidar_ros2-clean-shutdown.patch`. The
patch releases the SDK driver and caller-owned channel on the successful exit
path. The importer verifies the resulting source SHA-256 and refuses staged or
unexpected local changes. The imported checkout plus `build_jazzy`,
`install_jazzy`, and `log_jazzy` are gitignored; arbitrary upstream or generated
artifacts are not vendored.

On native Ubuntu 24.04 ARM64 with ROS 2 Jazzy:

```bash
cd ~/tennis-robot
./scripts/setup_pi.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install_jazzy/setup.bash
ros2 pkg prefix sllidar_ros2
```

`setup_pi.sh` installs `python3-vcstool`, imports the pin before `rosdep`, and
builds `sllidar_ros2` with the robot packages. A targeted clean verification on
the physical Pi completed both `sllidar_ros2` and `tennis_robot` successfully in
24.5 seconds; only upstream SDK compiler warnings were emitted.

Only the repository-managed LiDAR launch was run in isolated
`ROS_DOMAIN_ID=77`. Gazebo, the simulation bridge, controller, SLAM, Nav2,
collection nodes, and drive motors were not started by this validation. The
driver reported:

```text
SLLidar SDK Version: 2.1.0
S/N: 6E4DE0F8C2E29AD2C1819FF59DFC4E1E
Firmware: 1.02
Hardware: 18
Health: OK
Mode: Standard
Sample rate: 5 kHz
Driver max distance: 16.0 m
Scan frequency: 10.0 Hz
```

The repository-managed live run produced:

| Property | Physical C1 observation |
| --- | --- |
| topic/type | `/scan`, `sensor_msgs/msg/LaserScan` |
| frame_id | `lidar_link` (launch override) |
| publish frequency | `10.009 Hz` final average; observed intervals `0.098..0.101 s` |
| angle_min / angle_max | `-3.141592741 / +3.141592741 rad` |
| angle_increment | `0.008738784 rad` (about 0.501 degrees) |
| samples/scan | `720` |
| range_min / range_max | `0.050000001 m / 16.0 m` |
| scan_time | approximately `0.0997 s` |
| time_increment | approximately `0.0001387 s` |
| intensities | `720` values |
| invalid returns | positive infinity; 279/720 in one isolated snapshot |
| NaNs | `0` in the captured scan |
| finite returns | `441/720` in that snapshot |
| publisher QoS | reliable, volatile; depth reported as unknown by ROS graph introspection |
| time source | system/wall time (`use_sim_time` not set; no `/clock`) |

The header timestamp was nonzero and successive arrivals proved advancing scan
data. `use_sim_time` was `False` and no `/clock` was present in the isolated
hardware graph. Publisher QoS was reliable/volatile; the Control Panel adapter
subscribed best-effort/volatile, which is compatible with both source modes.

### Launch command and stable contract

```bash
cd ~/tennis-robot
source /opt/ros/jazzy/setup.bash
source ros2_ws/install_jazzy/setup.bash
ros2 launch tennis_robot lidar_hardware.launch.py
```

The default serial device is:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_3c21860b3b70f01184b98a301045c30f-if00-port0
```

The canonical ROS interface is `/scan`, `sensor_msgs/msg/LaserScan`, frame
`lidar_link`, using system time. The config is
`config/hardware_lidar.yaml`; the launch explicitly remaps the driver's relative
`scan` output to `/scan` and starts the existing `sensor_snapshot_node`.

### Clean start/stop evidence

Repeated launches reached health `OK`. During the assisted office runs, the
unmodified pinned upstream exit path stopped the motor but exceeded launch's
default five-second SIGINT grace period, causing launch to escalate to SIGTERM.
No process or stale publisher remained, but the escalation did not satisfy the
strict clean-stop gate.

Source inspection showed that the successful upstream path did not release the
SDK driver or caller-owned channel. The repository patch adds that cleanup, and
the LiDAR process receives a node-specific ten-second `sigterm_timeout` because
native SDK cleanup consistently takes about 6--7 seconds. After rebuilding on
the Pi, two consecutive `Ctrl-C` runs stopped the motor and all three launch
processes (`sllidar_node`, temporary TF publisher, and sensor snapshot node)
without an error, SIGTERM escalation, or forced kill. After each run, resetting
ROS CLI discovery made `ros2 topic info /scan` return `Unknown topic`, and
process inspection found no `sllidar_node` or hardware launch process. The
Control Panel changed from `LIVE` to `STALE` and continued aging the last
message after shutdown.

## L4 — Hardware versus simulation contract classification

The allowed classifications are: **interface-compatible** (consumers accept the
difference as part of `LaserScan`), **normalized** (the repository launch/config
deliberately presents the same contract), and **future hardware-specific**
(requires measured installation/court evidence and is not invented here).

| Property | Existing simulation | Physical C1 | Classification |
| --- | --- | --- | --- |
| message type | `sensor_msgs/msg/LaserScan` | same | interface-compatible (identical) |
| source topic | Gazebo `/gz/lidar` | driver-relative `scan` | normalized to canonical `/scan` at each bridge/launch boundary |
| frame | `lidar_link` | driver configured `lidar_link` | normalized (identical downstream) |
| angle limits | `-pi .. +pi` | `-pi .. +pi` | interface-compatible (identical) |
| angle increment | `0.012591554 rad` archived runtime | `0.008738784 rad` | interface-compatible; dynamic arrays/angles |
| samples/scan | `500` | `720` | interface-compatible; no fixed-count consumer contract |
| range limits | `0.15 .. 12.0 m` | `0.05 .. 16.0 m` | interface-compatible sensor characteristics |
| rate | `31.25 Hz` configured, not remeasured | `10.009 Hz` observed | interface-compatible; no numerical equality required |
| `scan_time` | `0`, set by Jazzy bridge conversion | about `0.0997 s` | interface-compatible optional timing detail |
| `time_increment` | `0`, set by Jazzy bridge conversion | about `0.0001387 s` | interface-compatible optional timing detail |
| timestamps | Gazebo simulation clock | Pi system/wall time | normalized by runtime `use_sim_time` policy (`false` in hardware) |
| QoS | default reliable/volatile bridge publisher | reliable/volatile driver publisher | interface-compatible (identical publisher policies) |
| invalid samples | Gazebo values copied by bridge; UI snapshot records nonfinite values as `null` | `+Inf` observed; UI records nonfinite values as `null` | normalized only in diagnostics JSON; canonical ROS message remains untouched |
| intensities | optional array copied by bridge | 720 device-specific values | interface-compatible optional field |
| noise/accuracy | configured Gaussian simulation noise | physical optics/environment | future hardware-specific court/outdoor characterization |
| base-to-LiDAR extrinsic | fixed simulated URDF geometry | temporary identity bench TF only | future hardware-specific measured mounting transform |

No numerical adapter is justified. The canonical ROS message remains unmodified;
only topic, frame parameter, and clock mode are normalized.

## L5 — TF result

For bench validation, the launch defaults to an explicitly named
`temporary_lidar_bench_tf` publisher with identity transform:

```text
base_link -> lidar_link
translation = (0, 0, 0) m
rotation quaternion = (0, 0, 0, 1)
```

`tf2_echo base_link lidar_link` resolved the transform repeatedly. This proves
frame connectivity only. It is **not** final extrinsic calibration. Disable it
with `publish_temporary_bench_tf:=false` once the final physical mount has been
measured and the robot description publishes that transform.

## L6 — Control Panel diagnostics

`sensor_snapshot_node` now derives canonical metadata from any
`sensor_msgs/msg/LaserScan`: arrival-rate window, wall-clock heartbeat age,
frame, sample/valid/invalid counts, angles, limits, and timing fields. The
Diagnostics page uses one pure `LidarView.derive` transformation and the existing
numeric ranges array for its polar/top-down plot. It has no simulator, driver,
model, serial-port, or hardware branch.

Live physical inspection on port 8082 showed `LIVE`, age, `lidar_link`, 10.0 Hz,
720 samples, `-180..180 deg`, 0.50 degree increment, `0.05..16.0 m`, nearest
range, and a moving polar point cloud. After driver shutdown it changed to
`STALE`. The generic Control Panel copy no longer describes the console as
simulation-only.

## L7 — Office validation

Stationary baseline: three snapshots two seconds apart each contained 720
samples and 429--436 finite returns. Stable sectors stayed within roughly
1--12 mm in their robust near/median values; isolated changes appeared where a
person or small object could have moved. Continuous publication and a stable
static room outline are confirmed.

An assisted run used a chair with a 30 cm broad wooden target across its thin
legs. The clearest controlled observations were:

- Near the initial physical forward line, a target measured at approximately
  `1.37 m` produced `1.359..1.400 m`, median `1.373 m`, over `14.5 deg`.
- After moving the target around the sensor, the corresponding broad return
  moved to `+89.9..+103.4 deg`; at an estimated `1.30 m` it measured median
  `1.313 m` over `13.5 deg`.
- Moving that same side target closer to approximately `0.80 m` moved the return
  to median `0.784 m` and increased its angular width to `23.0 deg`. A 30 cm
  target at 0.80 m nominally spans about `21.2 deg`, so both range and apparent
  width changed coherently.

These observations close the communication-level stationary, bearing-change,
near/far, and approximate-distance checks. Later verbal viewpoints around the
housing arrow were ambiguous and are deliberately not used to claim a final
angle offset or extrinsic. The temporary identity TF proves connectivity only.
Final mount geometry, marked front/left/right calibration, controlled lighting
characterization, and slow robot-motion validation are deferred to GitHub issue
`#16` after the permanent mount exists.

The side observations occurred in a brighter balcony area, but position and
lighting were not varied independently. They are therefore not a controlled
sunlight test and do not change the explicit `NOT TESTED` classification below.

## Tests and results

- Native Pi Jazzy/ARM64 build: the patched `sllidar_ros2` and updated
  `tennis_robot` package passed; upstream SDK warnings only. The tested patched
  source matched repository SHA-256
  `133df72431aec2bec0f450a7fbf43780f47a1aa1539b60c1b694c4a461d82dd6`.
- Pi focused tests: 14 passed, 1 skipped (Node.js-only UI transform test skipped
  on the Pi when Node is unavailable).
- Development-host focused tests: 15 passed, covering launch/config parsing,
  dependency pin, probe protocol, metadata/liveness, and UI transformation.
- JavaScript syntax checks and `git diff --check` pass.
- After merging the Field Wi-Fi mainline change, the combined focused set is 41
  passing tests. Live lifecycle, TF, QoS/rate, system time, Control Panel
  LIVE/STALE, clean SIGINT shutdown, and stale-publisher checks pass.

## Files changed

- `.gitignore`
- `ros2_ws/lidar.repos`
- `ros2_ws/patches/sllidar_ros2-clean-shutdown.patch`
- `scripts/import_lidar_dependencies.sh`
- `scripts/setup_pi.sh`
- `scripts/probe_slamtec_lidar.py`
- `ros2_ws/src/tennis_robot/package.xml`
- `ros2_ws/src/tennis_robot/config/hardware_lidar.yaml`
- `ros2_ws/src/tennis_robot/launch/lidar_hardware.launch.py`
- `ros2_ws/src/tennis_robot/launch/real_sensors.launch.py` (LiDAR portion only;
  now reuses the same config and stable default path)
- `ros2_ws/src/tennis_robot/tennis_robot/lidar_preview.py`
- `ros2_ws/src/tennis_robot/tennis_robot/sensor_snapshot_node.py`
- `scripts/control_panel.html`
- `scripts/control_panel/app.js`
- `scripts/control_panel/lidar_view.js`
- `scripts/control_panel/views/sensors.html`
- `tests/test_probe_slamtec_lidar.py`
- `tests/test_hardware_lidar_config.py`
- `tests/test_lidar_preview.py`
- `tests/test_lidar_view_js.py`
- this document

No collection planner, executor, controller behaviour, SLAM, or Nav2 tuning file
was changed.

## Explicitly NOT TESTED

- outdoor sunlight;
- court/fence/net detection;
- court-scale SLAM/localization;
- navigation;
- obstacle avoidance while driving.

## Remaining mounting/court-only work

- measure and publish the final physical `base_link -> lidar_link` extrinsic;
- repeat range/visibility characterization outdoors and around court surfaces,
  fence, and net;
- only in a later milestone, validate/tune SLAM, localization, navigation, and
  driven obstacle avoidance.

## Current milestone classification

`LIDAR_HARDWARE_INTERFACE_READY`

Reason: repository reproducibility, native ARM64 build, canonical `/scan`,
system time, frame connectivity, simulation-contract comparison, source-agnostic
Control Panel diagnostics, stationary scans, object bearing/range movement,
approximate physical distances, clean shutdown, and stale-publisher gates pass.
This classification does not include final mounting/extrinsic calibration,
controlled lighting, robot-motion validation, outdoor/court sensing, SLAM,
localization, Nav2, or driven obstacle avoidance.
