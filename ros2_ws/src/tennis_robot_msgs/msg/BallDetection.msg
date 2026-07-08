# A single tennis-ball detection from the perception pipeline.
#
# This message is produced identically by:
#   * the SIMULATED OAK-D pipeline (neural detector run on Gazebo RGB frames,
#     fused with the simulated depth image), and
#   * the REAL OAK-D on-device spatial detection network (DepthAI).
#
# It mirrors a single entry of depthai_ros_msgs/SpatialDetectionArray so that
# downstream nodes cannot tell whether a detection came from sim or hardware.

# --- Detection score ---------------------------------------------------------
float32 confidence            # neural-network class score, 0.0 .. 1.0

# --- 2D bounding box in the RGB image (pixels, original image resolution) ----
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height

# --- Spatial estimate from RGB + depth fusion (camera optical frame) ---------
# has_spatial is false when the depth image had no valid pixels in the bbox ROI;
# bearing/distance/position are then left at 0.0 and must be ignored.
bool has_spatial
float32 bearing_rad           # robot/navigation convention: + = left / CCW
float32 distance_m            # range to the ball surface (metres)
float32 position_x            # optical +X: right (metres)
float32 position_y            # optical +Y: down (metres)
float32 position_z            # optical +Z: forward (metres)
