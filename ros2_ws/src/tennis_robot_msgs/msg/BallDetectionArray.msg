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

BallDetection[] detections
