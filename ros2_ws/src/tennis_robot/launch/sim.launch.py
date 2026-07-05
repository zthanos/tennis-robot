"""Launch Gazebo Harmonic + ros_gz bridge + all ROS 2 nodes."""

import glob as _glob
import os
import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# WORKSPACE env var is set in Dockerfile.gazebo and docker-compose.yml.
# Fallback: resolve from the source tree (works when run directly from gazebo/launch/).
WORKSPACE = os.environ.get(
    "WORKSPACE",
    str(Path(__file__).resolve().parents[2]),  # gazebo/launch/../.. = project root
)
GZ_MODELS = f"{WORKSPACE}/gazebo/models"
ROS_SOURCE_MODELS = f"{WORKSPACE}/ros2_ws/src"
GZ_WORLD = f"{WORKSPACE}/gazebo/worlds/tennis_court.sdf"
BRIDGE_CONFIG = f"{WORKSPACE}/gazebo/bridge_config.yaml"
ROBOT_URDF = f"{WORKSPACE}/runtime/tennis_robot.urdf"
ROBOT_SDF = f"{WORKSPACE}/runtime/tennis_robot.sdf"
# ros2_control controllers config baked into the gz_ros2_control plugin.
# /workspace is mounted into the Gazebo container, so this path resolves there.
CONTROLLERS_CONFIG = f"{WORKSPACE}/ros2_ws/src/tennis_robot/config/controllers.yaml"
EKF_CONFIG = f"{WORKSPACE}/ros2_ws/src/tennis_robot/config/ekf.yaml"
# Docker sets ROS2_INSTALL=/ros2_ws/install; native Linux uses {WORKSPACE}/ros2_ws/install
ROS2_INSTALL = os.environ.get("ROS2_INSTALL", f"{WORKSPACE}/ros2_ws/install")


def _site_packages(install_dir: str, pkg: str) -> list:
    return (
        _glob.glob(f"{install_dir}/{pkg}/lib/python*/site-packages")
        + _glob.glob(f"{install_dir}/{pkg}/local/lib/python*/dist-packages")
    )


_robot_paths = _site_packages(ROS2_INSTALL, "tennis_robot")
_msgs_paths = _site_packages(ROS2_INSTALL, "tennis_robot_msgs")
_source_robot_path = f"{WORKSPACE}/ros2_ws/src/tennis_robot"
ROS_PYTHONPATH = ":".join([_source_robot_path] + _robot_paths + _msgs_paths + [os.environ.get("PYTHONPATH", "")])
CONTROL_PANEL_PYTHONPATH = ":".join(
    [_source_robot_path] + _robot_paths + [f"{WORKSPACE}/scripts"] + [os.environ.get("PYTHONPATH", "")]
)


def generate_robot_urdf():
    script = Path(WORKSPACE) / "scripts" / "generate_robot_urdf.py"
    if not script.exists():
        raise RuntimeError(f"Robot URDF generator not found: {script}")
    subprocess.run(
        [
            sys.executable, str(script),
            "--output", ROBOT_URDF,
            "--sdf-output", ROBOT_SDF,
            "--sim-mode", "true",
            "--controllers-config", CONTROLLERS_CONFIG,
        ],
        check=True,
    )


def generate_launch_description():
    generate_robot_urdf()

    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo without GUI (headless mode)"
    )
    headless = LaunchConfiguration("headless")

    # ── Gazebo ──────────────────────────────────────────────────────────────
    # Tell Gazebo where the gz_ros2_control system plugin lives. Sourcing the
    # workspace does NOT reliably set this, so Gazebo fails to load the plugin
    # ("Could not find shared library") and exits before controllers can spawn.
    _gz_plugin_path = f"{ROS2_INSTALL}/gz_ros2_control/lib:" + os.environ.get(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", ""
    )
    _gz_resource_path = ":".join(
        filter(
            None,
            [GZ_MODELS, ROS_SOURCE_MODELS, os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
        )
    )
    _gz_env = {
        "GZ_SIM_RESOURCE_PATH": _gz_resource_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": _gz_plugin_path,
    }

    # Render engines: gpu_lidar is implemented ONLY in ogre2 — under ogre (v1)
    # the lidar depth buffer is all zeros and every ray reports range_min
    # (sensor effectively blind). So the SERVER (sensors) must run ogre2.
    # The GUI window stays on ogre v1, which is more stable under WSLg.
    gz_full = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r",
            "--render-engine-server", "ogre2",
            "--render-engine-gui", "ogre",
            GZ_WORLD,
        ],
        condition=UnlessCondition(headless),
        additional_env=_gz_env,
        output="screen",
    )

    # --headless-rendering forces EGL offscreen rendering. Without it, ogre2
    # sees DISPLAY=:0 and creates a GLX context via X — which crashes under
    # GALLIUM_DRIVER=d3d12 (glx drisw screen / GLXBadFBConfig, the server dies
    # and controller_manager never comes up). d3d12 GPU accel works ONLY on
    # the EGL path, so headless must never touch GLX. DISPLAY is cleared for
    # the same reason.
    gz_headless = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s", "--headless-rendering",
            "--render-engine", "ogre2", GZ_WORLD,
        ],
        condition=IfCondition(headless),
        additional_env={**_gz_env, "DISPLAY": ""},
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tennis_robot",
        arguments=[
            "-file", ROBOT_SDF,
            "-name", "tennis_robot",
            "-x", "-8",
            "-y", "0",
            "-z", "0.09",
        ],
        output="screen",
    )

    # ── ros2_control: robot_state_publisher + controller spawners ───────────
    # robot_state_publisher must be up before Gazebo loads the model: the
    # gz_ros2_control plugin reads robot_description from it to start the
    # controller_manager. It also publishes TF from /joint_states (now fed by
    # joint_state_broadcaster instead of the old gz JointStatePublisher plugin).
    with open(ROBOT_URDF, "r", encoding="utf-8") as _f:
        _robot_description = _f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": _robot_description, "use_sim_time": True}],
    )

    def _spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_{controller}",
            arguments=[
                controller,
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

    jsb_spawner = _spawner("joint_state_broadcaster")
    diff_drive_spawner = _spawner("diff_drive_controller")
    lift_wheel_spawner = _spawner("lift_wheel_velocity_controller")

    # Actuation layer: the only node that talks to the controller command topics.
    drive_actuator = Node(
        package="tennis_robot",
        executable="drive_actuator_node",
        name="drive_actuator_node",
        output="screen",
        additional_env={"PYTHONPATH": ROS_PYTHONPATH},
    )
    collector_logic = Node(
        package="tennis_robot",
        executable="collector_logic_node",
        name="collector_logic_node",
        output="screen",
        additional_env={"PYTHONPATH": ROS_PYTHONPATH, "COLLECTOR_BACKEND": "gazebo"},
    )

    # cmd_vel arbiter — part of the base robot bring-up so the web D-pad / teleop
    # can drive without needing the SLAM launch running. Output goes to the
    # diff_drive_controller; inputs are /cmd_vel_teleop, /cmd_vel_nav, etc.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            f"{WORKSPACE}/ros2_ws/src/tennis_robot/config/twist_mux.yaml",
            {"use_sim_time": True},
        ],
        remappings=[("cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    # ── ros_gz bridge ────────────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": BRIDGE_CONFIG}],
        output="screen",
    )

    # ── ROS 2 nodes (same as before — interface unchanged) ──────────────────
    # /odom remap: after the ros2_control migration odometry is published on
    # /diff_drive_controller/odom; the old gz DiffDrive plugin that fed /odom is
    # gone. Without this remap the nodes' pose/yaw silently freeze at 0 (e.g.
    # survey turn_180_timeout: the turn never completes because yaw never moves).
    _odom_remap = ("/odom", "/diff_drive_controller/odom")

    perception = Node(
        package="tennis_robot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        remappings=[_odom_remap],
        additional_env={
            "PYTHONPATH": ROS_PYTHONPATH,
            # Match the RGB camera horizontal_fov in oak_d.urdf.xacro (1.204 rad ≈ 69°).
            # The perception default of 60° skews bearings ~15% toward frame edges.
            "CAMERA_FOV_RAD": "1.204",
            # Simulated OAK-D AI: run the YOLO ONNX detector on Gazebo RGB frames.
            # The model is mandatory; node startup fails if it is absent or invalid.
            # Produce it with scripts/export_yolo_onnx.py.
            "BALL_DETECTOR_BACKEND": os.getenv("BALL_DETECTOR_BACKEND", "yolo_onnx"),
            "BALL_MODEL_PATH": os.getenv("BALL_MODEL_PATH", f"{WORKSPACE}/models/yolov8n.onnx"),
            "BALL_CONF_THRESHOLD": os.getenv("BALL_CONF_THRESHOLD", "0.35"),
            "BALL_CLASS_IDS": os.getenv("BALL_CLASS_IDS", "32"),
            "CAMERA_FRAME_ID": "camera_link_optical_frame",
        },
    )

    controller = Node(
        package="tennis_robot",
        executable="controller_node",
        name="controller_node",
        output="screen",
        # Advance collection/search timers with Gazebo physics. At a low
        # real-time factor, wall-time capture deadlines expire far too early.
        parameters=[{"use_sim_time": True}],
        remappings=[_odom_remap],
        additional_env={
            "PYTHONPATH": ROS_PYTHONPATH,
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
            # Without this, RobotStatusStore.from_env() resolves a default path
            # inside the colcon install tree (container-internal) and the status
            # file never lands in the mounted runtime/ dir.
            "ROBOT_STATUS_FILE": f"{WORKSPACE}/runtime/robot_status.json",
        },
    )

    navigation = Node(
        package="tennis_robot",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        # Route the legacy /cmd_vel (used by the web control-panel D-pad via
        # controller_node) through twist_mux instead of the removed gz plugin.
        remappings=[("/cmd_vel", "/cmd_vel_teleop"), _odom_remap],
    )

    command_bridge = Node(
        package="tennis_robot",
        executable="command_bridge_node",
        name="command_bridge_node",
        output="screen",
        additional_env={
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
        },
    )

    # Converts IR LaserScan + pose info → /ir/readings + /sim/balls
    gz_extras = Node(
        package="tennis_robot",
        executable="gazebo_extras_node",
        name="gazebo_extras_node",
        output="screen",
    )

    sensor_snapshots = Node(
        package="tennis_robot",
        executable="sensor_snapshot_node",
        name="sensor_snapshot_node",
        output="screen",
        additional_env={
            "PYTHONPATH": ROS_PYTHONPATH,
            "ROBOT_SENSOR_FILE": f"{WORKSPACE}/runtime/robot_sensors.json",
            "COLLECTOR_SERIAL_BRIDGE": os.getenv(
                "COLLECTOR_SERIAL_BRIDGE", "host.docker.internal:8091"
            ),
        },
    )

    # Web control panel — http://localhost:8081
    control_panel = ExecuteProcess(
        cmd=[
            "python3",
            f"{WORKSPACE}/scripts/control_panel.py",
            "--host",
            "0.0.0.0",
            "--db-file",
            f"{WORKSPACE}/runtime/tennis_robot_gazebo.db",
        ],
        additional_env={
            "PYTHONPATH": CONTROL_PANEL_PYTHONPATH,
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
            "ROBOT_STATUS_FILE": f"{WORKSPACE}/runtime/robot_status.json",
            "ROBOT_SENSOR_FILE": f"{WORKSPACE}/runtime/robot_sensors.json",
        },
        output="screen",
    )

    # Sensor fusion: wheel odom (/diff_drive_controller/odom) + IMU (/imu/data)
    # -> corrected odom->base_footprint TF. Required for skid-steer, whose wheel
    # odometry mis-estimates yaw; slam_toolbox still provides map->odom on top.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[EKF_CONFIG, {"use_sim_time": True}],
    )

    # Delay ROS nodes until Gazebo + bridge are up
    delayed_nodes = TimerAction(
        period=4.0,
        actions=[
            bridge, perception, controller, navigation,
            command_bridge, gz_extras, sensor_snapshots, drive_actuator, collector_logic, twist_mux,
            ekf,
        ],
    )

    delayed_spawn = TimerAction(
        period=1.0,
        actions=[spawn_robot],
    )

    # Spawn controllers once the gz_ros2_control plugin has brought up the
    # controller_manager inside Gazebo (spawners wait up to 60s regardless).
    delayed_controllers = TimerAction(
        period=6.0,
        actions=[jsb_spawner, diff_drive_spawner, lift_wheel_spawner],
    )

    return LaunchDescription([
        headless_arg,
        gz_full,
        gz_headless,
        robot_state_publisher,
        delayed_spawn,
        control_panel,
        delayed_nodes,
        delayed_controllers,
    ])
