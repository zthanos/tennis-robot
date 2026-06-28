# Repository guidance for coding agents

## Execution environment

- The host workspace may be opened from native Windows, but ROS 2 Humble and
  Gazebo execution belong to the WSL 2 / Linux environment.
- Do not report ROS-dependent tests as failing merely because `python`, ROS 2,
  generated message packages, or Linux dependencies are unavailable in native
  PowerShell.
- Run ROS 2 builds, node tests, launch checks, and simulation validation from
  WSL 2, preferably through the repository's Docker Compose services.
- Native Windows test runs are appropriate only for explicitly ROS-independent
  modules whose dependencies are available there.
- When execution is unavailable from the current environment, report the test
  as "not run in native Windows; requires WSL 2/ROS 2" rather than as a product
  failure.

## Perception contract

- The neural detector is the required primary perception pipeline.
- Classical HSV/color-based ball recognition must not be used as an automatic
  runtime fallback. A missing or invalid neural model must produce an explicit
  unhealthy/not-ready state or fail startup.
- Gazebo supplies simulated RGB and depth images; downstream Collector, Nav2,
  and Behaviour Tree code must consume only the stable perception ROS 2
  interface shared with the physical OAK-D implementation.
- RGB detections must be fused only with timestamp-matched depth images.
- Perception publishers must emit empty results as a heartbeat, and downstream
  control must expire observations when that heartbeat stops.
- `/perception/ball_detections` is the sole downstream ball-perception
  contract. Its header frame is `camera_link_optical_frame`; XYZ is REP-103
  optical (right, down, forward), while `bearing_rad` is positive left/CCW.
