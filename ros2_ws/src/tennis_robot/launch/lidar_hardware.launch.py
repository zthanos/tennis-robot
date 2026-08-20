"""Minimal Pi-only RPLIDAR C1 bring-up on the canonical /scan contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("tennis_robot")
    config_file = f"{package_share}/config/hardware_lidar.yaml"

    serial_port = LaunchConfiguration("serial_port")
    publish_bench_tf = LaunchConfiguration("publish_temporary_bench_tf")
    start_snapshot = LaunchConfiguration("start_sensor_snapshot")

    driver = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            config_file,
            {
                "serial_port": serial_port,
                "use_sim_time": False,
            },
        ],
        remappings=[("scan", "/scan")],
    )

    # Bench-only identity transform. It establishes frame connectivity without
    # claiming a final measured mounting calibration. Disable it when the final
    # robot URDF publishes the real base_link -> lidar_link extrinsic.
    bench_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="temporary_lidar_bench_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(publish_bench_tf),
    )

    snapshot = Node(
        package="tennis_robot",
        executable="sensor_snapshot_node",
        name="sensor_snapshot_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(start_snapshot),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "3c21860b3b70f01184b98a301045c30f-if00-port0"
                ),
                description="Enumerated stable RPLIDAR serial path",
            ),
            DeclareLaunchArgument(
                "publish_temporary_bench_tf",
                default_value="true",
                description="Publish identity base_link -> lidar_link for bench tests only",
            ),
            DeclareLaunchArgument(
                "start_sensor_snapshot",
                default_value="true",
                description="Feed canonical LaserScan data to the existing Control Panel",
            ),
            driver,
            bench_tf,
            snapshot,
        ]
    )
