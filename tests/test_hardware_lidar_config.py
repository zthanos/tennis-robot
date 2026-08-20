from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PIN = "34300099fadfc772965962dec837bf436706188f"
STABLE_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "3c21860b3b70f01184b98a301045c30f-if00-port0"
)


def test_lidar_dependency_is_source_pinned() -> None:
    manifest = yaml.safe_load((ROOT / "ros2_ws/lidar.repos").read_text())
    driver = manifest["repositories"]["sllidar_ros2"]

    assert driver == {
        "type": "git",
        "url": "https://github.com/Slamtec/sllidar_ros2.git",
        "version": PIN,
    }
    importer = (ROOT / "scripts/import_lidar_dependencies.sh").read_text()
    assert f'PIN="{PIN}"' in importer
    assert "refusing to overwrite" in importer


def test_hardware_lidar_config_is_canonical_and_uses_system_time() -> None:
    config = yaml.safe_load(
        (
            ROOT
            / "ros2_ws/src/tennis_robot/config/hardware_lidar.yaml"
        ).read_text()
    )["sllidar_node"]["ros__parameters"]

    assert config["use_sim_time"] is False
    assert config["channel_type"] == "serial"
    assert config["serial_port"] == STABLE_DEVICE
    assert config["serial_baudrate"] == 460800
    assert config["frame_id"] == "lidar_link"
    assert config["scan_mode"] == "Standard"


def test_minimal_hardware_launch_has_scan_snapshot_and_bench_tf_contract() -> None:
    launch_path = (
        ROOT
        / "ros2_ws/src/tennis_robot/launch/lidar_hardware.launch.py"
    )
    source = launch_path.read_text()
    ast.parse(source)

    assert 'remappings=[("scan", "/scan")]' in source
    assert '"use_sim_time": False' in source
    assert '"--frame-id", "base_link"' in source
    assert '"--child-frame-id", "lidar_link"' in source
    assert '"publish_temporary_bench_tf"' in source
    assert 'executable="sensor_snapshot_node"' in source


def test_combined_real_sensor_launch_reuses_stable_lidar_config() -> None:
    source = (
        ROOT / "ros2_ws/src/tennis_robot/launch/real_sensors.launch.py"
    ).read_text()
    ast.parse(source)

    assert STABLE_DEVICE in source.replace('"\n    "', "")
    assert '"/config/hardware_lidar.yaml"' in source
    assert '"use_sim_time": False' in source
    assert "/dev/ttyUSB0" not in source
