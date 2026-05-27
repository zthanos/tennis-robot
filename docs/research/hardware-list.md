# Tennis Robot Hardware List

Last checked: 2026-05-16

Prices are approximate and exclude shipping, VAT/import duties, and local reseller markups. Treat this as a living bill of materials, not a final purchase order.

## Recommendation Summary

The Luxonis OAK-D S2 is a good first camera choice for this project. It gives us RGB vision, stereo depth, an onboard VPU for AI/CV workloads, USB connectivity, and an IMU in one compact unit.

For the first physical prototype, use the OAK-D S2 as the primary ball-perception sensor and add a 2D LiDAR for navigation/costmap sensing. The LiDAR does not replace the camera; it complements it by mapping obstacles, court boundaries, net/fence/wall clearance, and safer approach corridors.

## Phase 1: Vision And Bench Prototype

| Priority | Component | Est. cost | Link | Decision | Why |
|---|---:|---:|---|---|---|
| Buy first | Luxonis OAK-D S2 | US $329 | [Luxonis store](https://new-store.luxonis.com/products/oak-d-s2), [hardware docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2) | Recommended | Combines 12MP RGB, stereo depth, onboard AI/CV, USB2/USB3, and IMU. Good match for tennis ball detection plus distance estimation. |
| Buy with camera | USB3 cable, C-to-C or C-to-A | US $12.49-$19.99 | [Luxonis USB-C cable](https://shop.luxonis.com/collections/accessories), [locking C/A cable](https://shop.luxonis.com/products/usb-3-cable-type-c-to-type-a) | Recommended | OAK depth/RGB streaming needs a reliable USB3 link. A short, known-good cable removes a common failure point. |
| Buy with camera | Camera mount / 1/4-20 adapter / small bracket | US $10-$30 | [Luxonis tripod note](https://shop.luxonis.com/products/tripod) | Recommended | We need repeatable camera angle and height for calibration. A simple rigid mount is enough for bench tests. |
| Buy with navigation stack | Slamtec RPLIDAR C1 | EUR 80-90 | [Slamtec C1](https://www.slamtec.com/en/C1/), [ROS 2 docs](https://docs.ros.org/en/ros2_packages/humble/api/rplidar_ros/) | Recommended | 2D LiDAR for obstacle/court-boundary costmaps, safer route planning, and ROS 2 Nav2 compatibility. Complements OAK-D; does not detect tennis balls by itself. |
| Buy soon | Raspberry Pi 5 or equivalent SBC | US $80-$125 board-only estimate | [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) | Recommended, but can wait | Runs Python, DepthAI, OpenCV, telemetry, and high-level robot logic. For now, a laptop can be the host. |
| Buy with SBC | Active cooling + 5V/5A USB-C power | US $20-$35 | [Raspberry Pi 5 accessories](https://www.raspberrypi.com/products/raspberry-pi-5/) | Recommended if using Pi | Pi 5 needs solid power and active cooling under sustained vision workloads. |

Estimated Phase 1 total:

- Minimum, using laptop as host: about US $350-$380
- With Raspberry Pi host: about US $460-$540

## Phase 2: Mobile Base Prototype

These are intentionally not final yet. We should choose them after the simulation has a working `scan -> align -> approach -> stop_near_ball` loop.

| Priority | Component | Est. cost | Link | Decision | Why |
|---|---:|---:|---|---|---|
| Research next | 4WD or differential-drive outdoor-capable chassis | US $100-$400 | TBD | Deferred | Tennis courts need enough traction, wheel diameter, and ground clearance. Small indoor robot kits may not behave well on court surfaces. |
| Research next | DC gear motors with encoders | US $25-$80 each | TBD | Deferred | Encoders are important for odometry and closed-loop speed control. |
| Research next | Motor driver sized for chosen motors | US $30-$100 | TBD | Deferred | Driver depends on motor voltage/current. Pick after motors, not before. |
| Research next | Battery pack and regulator | US $60-$200 | TBD | Deferred | Needs separate sizing for drive motors, SBC, and camera. Safety and fuse planning matter here. |
| Research next | Emergency stop switch | US $10-$30 | TBD | Required before real movement tests | A moving robot with motors needs a physical cut-off. This is not optional once motors are installed. |

## Phase 3: Optional Sensors

| Priority | Component | Est. cost | Link | Decision | Why |
|---|---:|---:|---|---|---|
| Alternative budget LiDAR | LDROBOT LD19 / LDS-02 class | EUR 50-70 | [Waveshare LD19](https://www.waveshare.com/product/dtof-lidar-ld19.htm), [LDROBOT ROS 2 package](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) | Backup option | Lower cost, but expect more driver/package validation than Slamtec C1. |
| Optional later | OAK-D Pro / IR active stereo variant | US $329+ depending variant | [OAK camera family](https://docs.luxonis.com/hardware/) | Only if needed | IR/dot projector helps low-light or low-texture depth, but outdoor daylight tennis use may not benefit much from IR. |

## Why OAK-D S2 Fits This Project

- RGB camera detects tennis balls by color/shape and later with a neural detector.
- Stereo depth gives real distance instead of relying only on monocular ball size.
- On-device AI can reduce load on the Raspberry Pi or laptop.
- USB keeps integration simple for the first prototype.
- Integrated IMU is useful for future motion/state estimation.
- Luxonis has Python tooling and DepthAI support, which fits the current Python-first project.

## Why RPLIDAR C1 Fits This Project

- Gives a 2D scan for obstacles, net/fence/wall clearance, and court-boundary safety.
- Lets the route planner build a costmap instead of relying only on camera/depth heuristics.
- Improves defer/edge-pass decisions because the robot can see whether a ball has a safe approach corridor.
- Has a cleaner ROS 2 path than many low-cost rebranded vacuum LiDAR modules.
- Does not replace OAK-D S2 for ball detection; tennis balls remain a camera/RGB-D target.

## Caveats

- The standard OAK-D S2 does not include IR illumination/dot projection according to the hardware docs, so it is best for well-lit scenes.
- Outdoor sunlight, glare, green court surfaces, and shadows will need real-world calibration.
- The depth camera helps distance, but it does not replace proper navigation logic.
- For a moving outdoor robot, fixed mechanical mounting and vibration isolation matter.
- The LiDAR should be mounted high enough to see obstacles and boundaries, but low enough that close fence/net geometry remains useful for costmaps.

## Next Simulation Task

Before buying hardware, add a simulated depth/RGB camera contract to Webots:

1. Keep the current RGB detector.
2. Add a depth observation path that mimics OAK-D S2 output.
3. Add a simulated 2D LiDAR/costmap contract that mimics RPLIDAR C1 output.
4. Compare monocular distance estimate vs simulated depth.
5. Feed `distance_m`, `bearing_rad`, and costmap clearance into the controller state machine.
