"""Launch Gazebo Harmonic + ros_gz bridge + all ROS 2 nodes."""

import glob as _glob
import os
import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
GZ_WORLD = os.getenv(
    "GZ_WORLD_FILE", f"{WORKSPACE}/gazebo/worlds/tennis_court.sdf"
)
BRIDGE_CONFIG = f"{WORKSPACE}/gazebo/bridge_config.yaml"
ROBOT_URDF = f"{WORKSPACE}/runtime/tennis_robot.urdf"
ROBOT_SDF = f"{WORKSPACE}/runtime/tennis_robot.sdf"
GZ_GUI_CONFIG = f"{WORKSPACE}/runtime/gazebo_gui.config"
GZ_DEFAULT_GUI_CONFIG = "/usr/share/gz/gz-sim8/gui/gui.config"
GZ_INITIAL_CAMERA_POSE = os.environ.get(
    "GZ_INITIAL_CAMERA_POSE",
    "-6.45 -0.85 0.55 0 0.25 2.55",
)
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


_source_robot_path = f"{WORKSPACE}/ros2_ws/src/tennis_robot"
_robot_paths = _site_packages(ROS2_INSTALL, "tennis_robot")
# Keep the sourced ROS overlay authoritative.  Adding Python paths resolved
# from ROS2_INSTALL here previously put the image-baked message package before
# install_docker, so a newly generated BallDetectionArray schema was silently
# replaced by its stale image copy at runtime.
ROS_PYTHONPATH = ":".join([_source_robot_path, os.environ.get("PYTHONPATH", "")])
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
            "--packaging-variant", os.getenv("ROBOT_PACKAGING_VARIANT", "baseline"),
        ],
        check=True,
    )


def generate_gazebo_gui_config():
    default_config = Path(GZ_DEFAULT_GUI_CONFIG)
    if not default_config.exists():
        raise RuntimeError(f"Gazebo GUI config not found: {default_config}")

    config = default_config.read_text(encoding="utf-8")
    config = config.replace(
        "<camera_pose>-6 0 6 0 0.5 0</camera_pose>",
        f"<camera_pose>{GZ_INITIAL_CAMERA_POSE}</camera_pose>",
        1,
    )
    output = Path(GZ_GUI_CONFIG)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(config, encoding="utf-8")


def generate_launch_description():
    # Distributed split (WS3). TENNIS_LAUNCH_SIM brings up Gazebo + the robot
    # abstraction (PC side); TENNIS_LAUNCH_BRAIN brings up the control/perception
    # stack (Pi side). Both default true → single-machine all-in-one (run_native).
    _launch_sim = os.getenv("TENNIS_LAUNCH_SIM", "true").lower() in {"1", "true", "yes"}
    _launch_brain = os.getenv("TENNIS_LAUNCH_BRAIN", "true").lower() in {"1", "true", "yes"}
    # In the distributed sim, streaming raw RGB/depth PC->Pi plus TF timing
    # starves the scan (perception can't place detections before their TF ages
    # out of the buffer). TENNIS_PERCEPTION_ON_PC runs perception on the sim side
    # next to the Gazebo camera, so only the lightweight BallDetectionArray
    # crosses to the Pi. The real robot keeps perception on the Pi (camera is
    # local there), so this defaults off.
    _perception_on_pc = os.getenv("TENNIS_PERCEPTION_ON_PC", "false").lower() in {"1", "true", "yes"}
    if _launch_sim and not _launch_brain:
        _sensor_snapshot_mode = "publisher"
    elif _launch_brain and not _launch_sim and _perception_on_pc:
        _sensor_snapshot_mode = "receiver"
    else:
        _sensor_snapshot_mode = "local"

    # The URDF/SDF + GUI config are only needed by the sim side (spawn +
    # robot_state_publisher). On the Pi (sim off) they would fail — no xacro run.
    if _launch_sim:
        generate_robot_urdf()
        generate_gazebo_gui_config()
    bench_minimal = os.getenv("SIM_BENCH_MINIMAL", "false").lower() in {
        "1",
        "true",
        "yes",
    }
    enable_assist = os.getenv("INTAKE_ENABLE_ASSIST", "false").lower() in {
        "1",
        "true",
        "yes",
    }
    _packaging_variant = os.getenv("ROBOT_PACKAGING_VARIANT", "baseline").lower()
    # Mechanical operating configuration — MUST stay in step with the same
    # decision in scripts/generate_robot_urdf.py, which builds the model these
    # controllers are spawned against. Only the launch configuration (and the
    # provisional `compact` study) fits the launcher; every collection variant
    # leaves it off, because the launcher frame cuts through the intake mouth.
    _launch_configuration = _packaging_variant == "option-a-launch"
    enable_flywheel = os.getenv(
        "ROBOT_ENABLE_FLYWHEEL",
        "true" if _launch_configuration or _packaging_variant == "compact" else "false",
    ).lower() in {"1", "true", "yes"}
    enable_intake = os.getenv(
        "ROBOT_ENABLE_INTAKE", "false" if _launch_configuration else "true",
    ).lower() in {"1", "true", "yes"}
    skip_control_panel = bench_minimal or os.getenv(
        "SIM_SKIP_CONTROL_PANEL", "false"
    ).lower() in {"1", "true", "yes"}

    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo without GUI (headless mode)"
    )
    headless = LaunchConfiguration("headless")

    # ── Gazebo ──────────────────────────────────────────────────────────────
    # Tell Gazebo where the gz_ros2_control system plugin lives. Sourcing the
    # workspace does NOT reliably set this, so Gazebo fails to load the plugin
    # ("Could not find shared library") and exits before controllers can spawn.
    # Cover both layouts: the container/source build puts the plugin under
    # {ROS2_INSTALL}/gz_ros2_control/lib; a native Jazzy apt install ships
    # libgz_ros2_control-system.so in each ROS prefix's lib/ (AMENT_PREFIX_PATH).
    _ament_lib_dirs = [
        f"{prefix}/lib"
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":")
        if prefix
    ]
    _gz_plugin_path = ":".join(
        filter(
            None,
            [f"{ROS2_INSTALL}/gz_ros2_control/lib", *_ament_lib_dirs,
             os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")],
        )
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
            "--gui-config", GZ_GUI_CONFIG,
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
            "-x", os.getenv("SIM_ROBOT_SPAWN_X", "-8"),
            "-y", os.getenv("SIM_ROBOT_SPAWN_Y", "0"),
            "-z", os.getenv("SIM_ROBOT_SPAWN_Z", "0.09"),
            "-Y", os.getenv("SIM_ROBOT_SPAWN_YAW", "0"),
        ],
        output="screen",
    )

    # ── ros2_control: robot_state_publisher + controller spawners ───────────
    # robot_state_publisher must be up before Gazebo loads the model: the
    # gz_ros2_control plugin reads robot_description from it to start the
    # controller_manager. It also publishes TF from /joint_states (now fed by
    # joint_state_broadcaster instead of the old gz JointStatePublisher plugin).
    # robot_state_publisher lives on the sim side: it reads the generated URDF
    # and publishes robot_description + the base_footprint->base_link TF. On the
    # Pi (sim off) the URDF is never generated, so skip it there.
    robot_state_publisher = None
    if _launch_sim:
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
    intake_wheel_spawner = _spawner("intake_wheel_velocity_controller")
    assist_wheel_spawner = _spawner("assist_wheel_velocity_controller")
    flywheel_spawner = _spawner("flywheel_velocity_controller")
    basket_spawner = _spawner("basket_velocity_controller")

    # Actuation layer: the only node that talks to the controller command topics.
    drive_actuator = Node(
        package="tennis_robot",
        executable="drive_actuator_node",
        name="drive_actuator_node",
        output="screen",
        additional_env={"PYTHONPATH": ROS_PYTHONPATH},
    )
    # Distro fork (debug-log #43): on Humble, twist_mux and the
    # diff_drive_controller both speak plain Twist (`use_stamped_vel: false`,
    # ~/cmd_vel_unstamped). On Jazzy both are TwistStamped-only, so the
    # Twist-speaking producers (motor adapter on /cmd_vel_collection, survey/
    # teleop on /cmd_vel_teleop) must be restamped BEFORE the mux, and the mux
    # output goes straight to ~/cmd_vel. Producers stay distro-agnostic.
    _stamped_cmd_stack = os.environ.get("ROS_DISTRO", "") not in {"humble", "iron"}

    def _stamp_relay(name: str, in_topic: str, out_topic: str) -> Node:
        return Node(
            package="tennis_robot",
            executable="cmd_vel_stamp_relay",
            name=name,
            output="screen",
            parameters=[{"use_sim_time": True}],
            additional_env={
                "PYTHONPATH": ROS_PYTHONPATH,
                "CMD_VEL_RELAY_IN": in_topic,
                "CMD_VEL_RELAY_OUT": out_topic,
            },
        )

    cmd_vel_relays = (
        [
            _stamp_relay(
                "cmd_vel_stamp_relay_collection",
                "/cmd_vel_collection",
                "/cmd_vel_collection_stamped",
            ),
            _stamp_relay(
                "cmd_vel_stamp_relay_teleop",
                "/cmd_vel_teleop",
                "/cmd_vel_teleop_stamped",
            ),
        ]
        if _stamped_cmd_stack
        else []
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
            # Jazzy: mux inputs are TwistStamped, so point them at the
            # restamped variants; the raw Twist topics keep their names for
            # the producers. Nav2 already publishes TwistStamped on
            # /cmd_vel_nav, so that input needs no relay.
            *(
                [
                    {
                        "topics.collection.topic": "/cmd_vel_collection_stamped",
                        "topics.teleop.topic": "/cmd_vel_teleop_stamped",
                    }
                ]
                if _stamped_cmd_stack
                else []
            ),
        ],
        remappings=[
            (
                "cmd_vel_out",
                "/diff_drive_controller/cmd_vel"
                if _stamped_cmd_stack
                else "/diff_drive_controller/cmd_vel_unstamped",
            )
        ],
    )

    # ── ros_gz bridge ────────────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": BRIDGE_CONFIG}],
        output="screen",
    )
    sim_clock_relay = Node(
        package="tennis_robot",
        executable="sim_clock_relay_node",
        name="sim_clock_relay",
        output="screen",
        additional_env={
            "PYTHONPATH": ROS_PYTHONPATH,
            "SIM_CLOCK_PUBLISH_HZ": os.getenv("SIM_CLOCK_PUBLISH_HZ", "50"),
        },
    )

    # ── ROS 2 nodes (same as before — interface unchanged) ──────────────────
    # /odom remap: after the ros2_control migration odometry is published on
    # /diff_drive_controller/odom; the old gz DiffDrive plugin that fed /odom is
    # gone. Without this remap the nodes' pose/yaw silently freeze at 0 (e.g.
    # survey turn_180_timeout: the turn never completes because yaw never moves).
    # Consumers get the EKF-fused pose, NOT raw wheel odometry: skid-steer
    # wheel yaw is scaled by the wheel_separation SLAM fudge (1.0 vs the real
    # 0.70), so every in-place scan turn corrupted dead-reckoned target
    # bearings by ~40% and blind capture legs missed by 0.2-0.9 m
    # (debug-log #44). The EKF exists precisely to fix yaw with the IMU —
    # but only its TF was consumed before this remap.
    _odom_remap = ("/odom", "/odometry/filtered")

    perception = Node(
        package="tennis_robot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
            "BALL_CENTER_ZOOM_FACTOR": os.getenv("BALL_CENTER_ZOOM_FACTOR", "3.0"),
            "BALL_CENTER_ZOOM_TILES": os.getenv(
                "BALL_CENTER_ZOOM_TILES", "0.30:0.333,0.50:0.333,0.70:0.333"
            ),
            "BALL_CLASS_IDS": os.getenv("BALL_CLASS_IDS", "32"),
            # Bound both ONNX sessions. Without explicit pools, each session
            # expands to the workstation CPU topology and can starve Gazebo.
            "PERCEPTION_ONNX_INTRA_OP_THREADS": os.getenv(
                "PERCEPTION_ONNX_INTRA_OP_THREADS", "4"
            ),
            "PERCEPTION_ONNX_INTER_OP_THREADS": os.getenv(
                "PERCEPTION_ONNX_INTER_OP_THREADS", "1"
            ),
            "RGB_DEPTH_SYNC_QUEUE_SIZE": os.getenv(
                "RGB_DEPTH_SYNC_QUEUE_SIZE", "3"
            ),
            # Required neural court-scene semantics. Custom training uses
            # class 0=net and class 1=fence; no OpenCV runtime fallback.
            "COURT_SCENE_DETECTOR_BACKEND": os.getenv(
                "COURT_SCENE_DETECTOR_BACKEND", "yolo_onnx"
            ),
            "COURT_SCENE_MODEL_PATH": os.getenv(
                "COURT_SCENE_MODEL_PATH",
                f"{WORKSPACE}/models/court_scene_yolov8n.onnx",
            ),
            "COURT_SCENE_CLASS_MAP": os.getenv(
                "COURT_SCENE_CLASS_MAP", "0:net,1:fence"
            ),
            "COURT_SCENE_CONF_THRESHOLD": os.getenv(
                "COURT_SCENE_CONF_THRESHOLD", "0.45"
            ),
            "COURT_SCENE_CONFIRM_FRAMES": os.getenv(
                "COURT_SCENE_CONFIRM_FRAMES", "3"
            ),
            "CAMERA_FRAME_ID": "camera_link_optical_frame",
            # C2 v3 activation: reviewed evidence covers 1.02..6.77 m.
            "PERCEPTION_CALIBRATION_PLATFORM": "gazebo",
            "PERCEPTION_COVARIANCE_CALIBRATION_ARTIFACT": f"{WORKSPACE}/calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v3.json",
            "PERCEPTION_COVARIANCE_REQUIRED_ARTIFACT": f"{WORKSPACE}/calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v3.json",
            "PERCEPTION_COVARIANCE_CALIBRATION_ID": "gazebo-range-depth-quality-diagonal-v1-20260722-v3",
            "PERCEPTION_COVARIANCE_MODEL_VERSION": "gazebo-v3",
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
            # Respect caller overrides (bench/e2e harnesses point these at
            # per-run files); default into the mounted runtime/ dir, because
            # RobotStatusStore.from_env() would otherwise resolve a path
            # inside the colcon install tree.
            "ROBOT_COMMAND_FILE": os.getenv(
                "ROBOT_COMMAND_FILE", f"{WORKSPACE}/runtime/robot_command.json"
            ),
            "ROBOT_STATUS_FILE": os.getenv(
                "ROBOT_STATUS_FILE", f"{WORKSPACE}/runtime/robot_status.json"
            ),
            # Required, explicit 6D.4 executor input.  The factory validates
            # the artifact and has no model/config fallback.
            "COLLECTION_ROUTE_CALIBRATION_ARTIFACT": os.getenv(
                "COLLECTION_ROUTE_CALIBRATION_ARTIFACT",
                f"{WORKSPACE}/calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v3.json",
            ),
        },
    )

    navigation = Node(
        package="tennis_robot",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        # Route the legacy /cmd_vel (used by the web control-panel D-pad via
        # controller_node) through twist_mux instead of the removed gz plugin.
        remappings=[("/cmd_vel", "/cmd_vel_teleop"), _odom_remap],
    )

    command_bridge = Node(
        package="tennis_robot",
        executable="command_bridge_node",
        name="command_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env={
            "ROBOT_COMMAND_FILE": os.getenv(
                "ROBOT_COMMAND_FILE", f"{WORKSPACE}/runtime/robot_command.json"
            ),
        },
    )

    # Converts IR LaserScan + pose info → /ir/readings + /sim/balls
    gz_extras = Node(
        package="tennis_robot",
        executable="gazebo_extras_node",
        name="gazebo_extras_node",
        output="screen",
        remappings=[_odom_remap],
    )

    sensor_snapshots = Node(
        package="tennis_robot",
        executable="sensor_snapshot_node",
        name=(
            "sensor_snapshot_node"
            if _sensor_snapshot_mode == "local"
            else f"sensor_snapshot_{_sensor_snapshot_mode}"
        ),
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env={
            "PYTHONPATH": ROS_PYTHONPATH,
            "SENSOR_SNAPSHOT_MODE": _sensor_snapshot_mode,
            "ROBOT_SENSOR_FILE": os.getenv(
                "ROBOT_SENSOR_FILE", f"{WORKSPACE}/runtime/robot_sensors.json"
            ),
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
            # Gates the console's simulation-only actuator paths and its
            # permissive E-stop/motor-power defaults.  This launch file also
            # brings up a brain-only stack (run_pi.sh), which on a real robot
            # must NOT claim to be a simulation — so derive it from whether the
            # sim runs here, and let a distributed sim brain override it.
            "TENNIS_ROBOT_RUNTIME": os.getenv(
                "TENNIS_ROBOT_RUNTIME", "simulation" if _launch_sim else "hardware"
            ),
            "ROBOT_COMMAND_FILE": os.getenv(
                "ROBOT_COMMAND_FILE", f"{WORKSPACE}/runtime/robot_command.json"
            ),
            "ROBOT_STATUS_FILE": os.getenv(
                "ROBOT_STATUS_FILE", f"{WORKSPACE}/runtime/robot_status.json"
            ),
            "ROBOT_SENSOR_FILE": os.getenv(
                "ROBOT_SENSOR_FILE", f"{WORKSPACE}/runtime/robot_sensors.json"
            ),
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

    # ── Node groups by machine (WS3 distributed split) ─────────────────────
    # PC (sim) side: Gazebo bridge + IR/ball extractor + actuation + odometry
    # fusion. Pi (brain) side: perception + control. The file-IPC nodes
    # controller/command_bridge and the panel share the Pi filesystem. In the
    # distributed simulation, sensor snapshots are split: the PC encodes a
    # low-rate JPEG preview next to the raw Gazebo camera, while the Pi receives
    # only that compact preview. Raw RGB/depth therefore never cross the LAN.
    # gz /clock -> bridge -> /clock_raw -> sim_clock_relay -> /clock. These two
    # nodes are one transport, not two optional nodes: sim_clock_relay is the
    # ONLY publisher of /clock, so dropping it while keeping the bridge leaves
    # /clock_raw with zero subscribers and every use_sim_time node — including
    # the controller_manager that gz_ros2_control runs in-process — on an unset
    # clock. That failure is silent: physics keeps advancing and controllers
    # still report "active", but every ROS timestamp freezes at 0. Never split
    # this pair.
    _sim_clock_transport = [bridge, sim_clock_relay]
    _sim_node_actions = [*_sim_clock_transport, gz_extras, drive_actuator, *cmd_vel_relays,
                         collector_logic, twist_mux, ekf]
    _brain_node_actions = [perception, controller, navigation, command_bridge]
    if _sensor_snapshot_mode == "publisher":
        _sim_node_actions.append(sensor_snapshots)
    else:
        _brain_node_actions.append(sensor_snapshots)
    # Move perception to the sim side (next to the camera) when requested — see
    # TENNIS_PERCEPTION_ON_PC above.
    if _perception_on_pc:
        _brain_node_actions.remove(perception)
        _sim_node_actions.append(perception)

    # Delay ROS nodes until Gazebo + bridge are up (all-in-one), or until DDS
    # discovery of the PC sensors has settled (Pi brain).
    if bench_minimal:
        # Strips the brain and the actuation/fusion nodes, but keeps the full
        # clock transport — a bench still needs a valid simulation clock.
        delayed_node_actions = [*_sim_clock_transport, gz_extras]
    else:
        delayed_node_actions = []
        if _launch_sim:
            delayed_node_actions += _sim_node_actions
        if _launch_brain:
            delayed_node_actions += _brain_node_actions
    delayed_nodes = TimerAction(period=4.0, actions=delayed_node_actions)

    actions = [headless_arg]
    if _launch_sim:
        delayed_spawn = TimerAction(period=1.0, actions=[spawn_robot])
        # Spawn controllers sequentially. Jazzy's spawner serializes operations
        # with a shared lock; launching all spawners at once makes repeated fresh
        # simulations intermittently exhaust the lock timeout.
        # Spawn order is the sequence of controllers this mechanical
        # configuration actually has. A controller whose joints are not in the
        # model cannot configure, so the intake spawner is dropped along with
        # the intake assembly rather than being left to fail.
        spawn_sequence = [jsb_spawner, diff_drive_spawner]
        if enable_intake:
            spawn_sequence.append(intake_wheel_spawner)
        spawn_sequence.append(basket_spawner)
        if enable_assist:
            spawn_sequence.append(assist_wheel_spawner)
        if enable_flywheel:
            spawn_sequence.append(flywheel_spawner)
        delayed_controllers = TimerAction(period=6.0, actions=[spawn_sequence[0]])
        controller_chain = [
            RegisterEventHandler(OnProcessExit(target_action=previous, on_exit=[following]))
            for previous, following in zip(spawn_sequence, spawn_sequence[1:])
        ]
        actions += [gz_full, gz_headless, robot_state_publisher,
                    delayed_spawn, delayed_controllers, *controller_chain]

    actions.append(delayed_nodes)
    if _launch_brain and not skip_control_panel:
        actions.append(control_panel)
    return LaunchDescription(actions)
