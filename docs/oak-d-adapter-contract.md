# Future OAK-D adapter contract

The physical OAK-D has not been purchased yet. This document fixes the adapter
boundary without guessing at a particular device, firmware, or DepthAI ROS
driver version.

The future hardware adapter has one responsibility: convert the OAK-D neural
spatial detections into `/perception/ball_detections` using
`tennis_robot_msgs/BallDetectionArray`.

Required contract:

- `header.stamp`: RGB acquisition timestamp.
- `header.frame_id`: `camera_link_optical_frame`.
- Bounding boxes: pixels in the corresponding RGB image.
- `confidence`: neural-network class confidence.
- `has_spatial`: false when aligned depth cannot produce a valid estimate.
- `position_x/y/z`: REP-103 optical coordinates in metres: right, down,
  forward.
- `distance_m`: spatial range in metres.
- `bearing_rad`: positive left/counter-clockwise, calculated as
  `atan2(-position_x, position_z)`.
- Publish an empty array for frames with no detected tennis balls.
- Continue publishing empty arrays as a heartbeat; do not retain detections.

The adapter must not publish simulation-only world coordinates. The controller
combines camera-relative detections with its authoritative SLAM/odometry pose.
Collector, Nav2, Behaviour Tree, and UI code must not depend on DepthAI-specific
message types or hardware state.

Implementation is intentionally deferred until the exact OAK-D model and its
supported DepthAI/ROS driver versions are known.
