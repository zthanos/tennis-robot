# All tennis-ball detections in a single camera frame.
#
# Published on /perception/ball_detections by the perception node. This is the
# canonical "detector output" contract — identical between the Gazebo-simulated
# OAK-D pipeline and the real OAK-D DepthAI spatial detection network, so the
# Collector / Nav2 / Behaviour Tree never depend on where detections came from.
#
# header.stamp     = acquisition time of the source RGB frame
# header.frame_id  = camera_link_optical_frame (REP-103 optical convention)
std_msgs/Header header

bool spatial_targets_healthy
string spatial_targets_health_reason

# Identity of the calibrated covariance producer for every spatial detection in
# this frame. calibration_id is the versioned calibration artifact ID and
# configuration_id its model version, both taken from the loaded
# SpatialCalibrationRuntime. Producers publish them verbatim; when the
# calibration runtime is unhealthy they are empty and consumers must reject
# spatial use with a typed rejection — there is no implicit default identity.
string calibration_id
string configuration_id

BallDetection[] detections
