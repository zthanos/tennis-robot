from glob import glob

from setuptools import find_packages, setup

package_name = "tennis_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/urdf/components", glob("urdf/components/*.xacro")),
        (f"share/{package_name}/meshes", glob("meshes/*")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thano",
    maintainer_email="thanos.zikas21@gmail.com",
    description="Tennis ball collection robot ROS 2 nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = tennis_robot.perception_node:main",
            "controller_node = tennis_robot.controller_node:main",
            "navigation_node = tennis_robot.navigation_node:main",
            "command_bridge_node = tennis_robot.command_bridge_node:main",
            "gazebo_extras_node = tennis_robot.gazebo_extras_node:main",
            "sim_physics_probe = tennis_robot.sim_physics_probe:main",
            "sensor_snapshot_node = tennis_robot.sensor_snapshot_node:main",
            "drive_actuator_node = tennis_robot.drive_actuator_node:main",
            "collector_logic_node = tennis_robot.collector_logic_node:main",
            "court_landmarks_node = tennis_robot.court_landmarks_node:main",
            "court_survey_mission_node = tennis_robot.court_survey_mission_node:main",
        ],
    },
)
