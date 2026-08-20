"""Real-robot sensor bringup.

Counterpart of the Gazebo sensor plugins + ros_gz_bridge: brings up the physical
LiDAR and OAK-D drivers and REMAPS them onto the exact same topic names and
frame_ids the simulation uses, so perception/controller code is identical sim↔real.

Canonical contract (see docs/hardware/sensor-topic-contract-el.md):
    /scan              sensor_msgs/LaserScan   frame_id=lidar_link
    /camera/image_raw  sensor_msgs/Image rgb8  frame_id=camera_link
    /camera/depth      sensor_msgs/Image 32FC1 frame_id=camera_link  (metres)

NOTE: driver package/param names below match the common upstream drivers
(sllidar_ros2 / rplidar_ros, depthai_ros_driver). Adjust to the exact packages
flashed on the robot. Items marked TODO depend on the final hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


STABLE_LIDAR_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "3c21860b3b70f01184b98a301045c30f-if00-port0"
)


def generate_launch_description():
    serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_config = (
        get_package_share_directory("tennis_robot")
        + "/config/hardware_lidar.yaml"
    )

    args = [
        DeclareLaunchArgument(
            "lidar_serial_port", default_value=STABLE_LIDAR_DEVICE
        ),
    ]

    # ── RPLiDAR ───────────────────────────────────────────────────────────────
    # sllidar_ros2 publishes /scan already; we only pin the frame_id to lidar_link.
    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar",
        output="screen",
        parameters=[
            lidar_config,
            {"serial_port": serial_port, "use_sim_time": False},
        ],
        remappings=[("scan", "/scan")],
    )

    # ── OAK-D (DepthAI) ───────────────────────────────────────────────────────
    # depthai_ros_driver publishes under /oak/* ; remap onto the canonical names.
    # TODO: depth comes out as 16UC1 (mm). Add a depth_image_proc / converter to
    #       republish as 32FC1 in metres on /camera/depth to match the sim contract.
    oak_d = Node(
        package="depthai_ros_driver",
        executable="camera_node",
        name="oak_d",
        output="screen",
        parameters=[{
            "camera.i_pipeline_type": "RGBD",
            "rgb.i_frame_id": "camera_link",     # MUST match the URDF link
            "stereo.i_frame_id": "camera_link",
        }],
        remappings=[
            ("/oak/rgb/image_raw", "/camera/image_raw"),
            ("/oak/stereo/image_raw", "/camera/depth"),
        ],
    )

    return LaunchDescription(args + [rplidar, oak_d])
