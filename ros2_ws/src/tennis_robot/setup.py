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
        (f"share/{package_name}/launch", ["launch/tennis_robot.launch.py", "launch/sim.launch.py"]),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/urdf/components", glob("urdf/components/*.xacro")),
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
            "sensor_snapshot_node = tennis_robot.sensor_snapshot_node:main",
        ],
    },
)
