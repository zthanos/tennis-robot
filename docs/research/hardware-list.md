# Tennis Robot Hardware List

Last checked: 2026-05-16

Prices are approximate and exclude shipping, VAT/import duties, and local reseller markups. Treat this as a living bill of materials, not a final purchase order.

## Recommendation Summary

The first sensor layout should use the Waveshare/Slamtec RPLIDAR C1 low on the robot for real-time court/obstacle mapping, with a top-mounted Luxonis OAK-D S2 as the primary ball detector.

For the first physical prototype, use the lower LiDAR to build the obstacle/court-edge costmap and identify shadow zones. Use the OAK-D S2 above the intake/body to detect tennis balls, estimate depth, and take targeted looks into areas that the LiDAR map marks as blocked or cluttered.

## Phase 1: Vision And Bench Prototype

| Priority | Component | Est. cost | Link | Decision | Why |
|---|---:|---:|---|---|---|
| Buy first | Waveshare/Slamtec RPLIDAR C1 | EUR 80-100 | [Amazon.de listing](https://www.amazon.de/-/en/Waveshare-C1-Omnidirectional-Anti-Interference-Anti-Adhesion/dp/B0CT31PH8S/), [Slamtec C1](https://www.slamtec.com/en/C1/), [Waveshare wiki](https://www.waveshare.net/wiki/RPLIDAR_C1), [ROS 2 docs](https://docs.ros.org/en/ros2_packages/humble/api/rplidar_ros/) | Recommended | Lower-body 360-degree sweep for obstacle/court-boundary costmaps, shadow zones, safer route planning, and ROS 2 Nav2 compatibility. |
| Buy first | Luxonis OAK-D S2 | US $329 | [Luxonis store](https://new-store.luxonis.com/products/oak-d-s2), [hardware docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2) | Recommended | Top RGB-D camera for tennis-ball detection, hidden-ball recovery, stereo depth, onboard AI/CV, USB2/USB3, and IMU. |
| Buy with camera | USB3 cable, C-to-C or C-to-A | US $12.49-$19.99 | [Luxonis USB-C cable](https://shop.luxonis.com/collections/accessories), [locking C/A cable](https://shop.luxonis.com/products/usb-3-cable-type-c-to-type-a) | Recommended | OAK depth/RGB streaming needs a reliable USB3 link. A short, known-good cable removes a common failure point. |
| Buy with camera | Camera mount / 1/4-20 adapter / small bracket | US $10-$30 | [Luxonis tripod note](https://shop.luxonis.com/products/tripod) | Recommended | We need repeatable camera angle and height for calibration. A simple rigid mount is enough for bench tests. |
| Buy with LiDAR | Lower-body guard / vibration-isolated LiDAR bracket | US $10-$35 | TBD | Recommended | Protects the low scan unit while keeping a clean 360-degree line of sight for obstacle and court-edge mapping. |
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

## Why The Waveshare/Slamtec RPLIDAR C1 Fits This Project

- A low 360-degree scan keeps the obstacle/court map fresh without a slow survey phase.
- It gives the planner continuous traversability data: net, fence, walls, people, bags, and blocked sectors.
- It improves route planning near the net, fence, walls, bags, and people.
- The C1 spec is a good fit for a tennis half-court: 12 m radius, 360-degree scan, 5000 Hz sampling, typical 10 Hz scan frequency, and about 0.72-degree angular resolution.
- It frees the OAK-D to spend its compute and field of view on ball detection instead of global obstacle surveying.
- It should not be trusted for tennis-ball detection: a 6.7 cm ball is a weak, inconsistent 2D LiDAR target.

## Why OAK-D S2 Fits This Project

- RGB camera detects tennis balls by color/shape and later with a neural detector.
- Stereo depth gives real distance for the selected ball and for targeted looks into LiDAR shadow zones.
- On-device AI can reduce load on the Raspberry Pi or laptop.
- USB keeps integration simple for the first prototype.
- Integrated IMU is useful for future motion/state estimation.
- Luxonis has Python tooling and DepthAI support, which fits the current Python-first project.

## Caveats

- The standard OAK-D S2 does not include IR illumination/dot projection according to the hardware docs, so it is best for well-lit scenes.
- Outdoor sunlight, glare, green court surfaces, and shadows will need real-world calibration.
- The depth camera helps distance, but it does not replace proper navigation logic.
- For a moving outdoor robot, fixed mechanical mounting and vibration isolation matter.
- The LiDAR should be mounted low on the body with a protective guard and clean 360-degree line of sight, but not treated as a ball-height detector.
- Very low LiDAR mounting will see more dust, collector structure, wheel occlusions, and court texture; bracket geometry matters.

## Next Simulation Task

Before buying hardware, add a simulated depth/RGB camera contract to Webots:

1. Keep the current RGB detector.
2. Add a depth observation path that mimics OAK-D S2 output.
3. Add a simulated low 360-degree LiDAR contract that emits costmap obstacles and shadow zones.
4. Compare monocular distance estimate vs simulated depth.
5. Feed `distance_m`, `bearing_rad`, and costmap clearance into the controller state machine.
