"""SLAM localization — collection workflow.

Runs slam_toolbox in LOCALIZATION mode against the map saved by the survey
(runtime/maps/court_*.posegraph + .data). This gives Nav2 a fully-known global
costmap from t=0, so the Regulated Pure Pursuit controller drives the lawnmower
lanes instead of refusing to move through unmapped/unknown space.

The newest court_*.posegraph is selected automatically. If none exists we fall
back to mapping mode with a loud warning (so a missing map is obvious rather
than silently producing an empty costmap).

Brings up the canonical twist_mux too? No — twist_mux lives in the base
bring-up (sim.launch.py), same as slam_mapping.launch.py.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _newest_map_basename() -> str | None:
    workspace = os.getenv("WORKSPACE", "/workspace")
    candidates = glob.glob(os.path.join(workspace, "runtime", "maps", "court_*.posegraph"))
    if not candidates:
        return None
    newest = max(candidates, key=os.path.getmtime)
    return newest[: -len(".posegraph")]  # slam_toolbox wants the basename, no extension


def _slam_lifecycle_manager() -> Node:
    """Configure+activate the slam_toolbox lifecycle node.

    On ROS 2 Jazzy slam_toolbox is a managed LifecycleNode and no longer
    auto-activates itself the way it did on Humble — left alone it stays
    ``unconfigured`` and never publishes map->odom, so Nav2's costmaps time out
    on the missing ``map`` frame.  A dedicated lifecycle manager (autostart)
    drives it to ``active``.  bond_timeout=0 disables bond monitoring because a
    large saved posegraph can take a while to deserialize on configure.
    """
    return Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 60.0,
        }],
    )


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tennis_robot")
    slam_config = os.path.join(pkg_share, "config", "slam_toolbox.yaml")

    map_basename = _newest_map_basename()

    if map_basename is None:
        print(
            "\n[slam_localization] WARNING: no runtime/maps/court_*.posegraph found — "
            "run 'Map Court' (survey) first to build a map. Falling back to MAPPING "
            "mode; Nav2 will start with an empty/unknown costmap.\n"
        )
        node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config, {"use_sim_time": True, "mode": "mapping",
                                      "use_lifecycle_manager": True}],
        )
        return [node, _slam_lifecycle_manager()]

    print(f"\n[slam_localization] Loading saved map: {map_basename}.posegraph\n")
    node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {
                "use_sim_time": True,
                "mode": "localization",
                "map_file_name": map_basename,
                # Start localized at the map origin (the survey start pose), which
                # is where the robot spawns in sim.
                "map_start_pose": [0.0, 0.0, 0.0],
                "use_lifecycle_manager": True,
            },
        ],
    )
    return [node, _slam_lifecycle_manager()]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
