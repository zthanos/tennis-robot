#!/usr/bin/env python3
"""P2-99c step (a) gate 4: Nav2 costmaps free of phantom obstacles + one goal.

Run inside the Jazzy container while the sim (SLAM + Nav2) is up:

    python3 scripts/sim_debug/p299c_nav_gate.py --out runtime/p299c/nav_gate

Phantom check: with no real obstacle within --clear-radius-m of the robot,
every lethal (>=253) or inscribed cell inside that radius in the local and
global costmaps is a phantom (e.g. a self-hit of the robot body). Checked
before and after the goal. Then one NavigateToPose goal must SUCCEED and end
within --xy-tol-m of the goal. Writes JSON and a costmap PNG.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


class NavGate(Node):
    def __init__(self) -> None:
        super().__init__("p299c_nav_gate")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.maps: dict[str, OccupancyGrid] = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ("local", "global"):
            self.create_subscription(OccupancyGrid, f"/{name}_costmap/costmap",
                                     lambda m, n=name: self.maps.__setitem__(n, m), qos)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, frame: str) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform(frame, "base_link", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw


def phantom_cells(grid: OccupancyGrid, rx: float, ry: float, radius: float) -> dict:
    info = grid.info
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    lethal = inscribed = 0
    worst = None
    for j in range(info.height):
        y = oy + (j + 0.5) * res
        if abs(y - ry) > radius:
            continue
        for i in range(info.width):
            x = ox + (i + 0.5) * res
            d = math.hypot(x - rx, y - ry)
            if d > radius:
                continue
            v = grid.data[j * info.width + i]
            if v >= 99:          # OccupancyGrid scaling: 100 lethal, 99 inscribed
                lethal += v == 100
                inscribed += v == 99
                if worst is None or d < worst[0]:
                    worst = (round(d, 3), round(x, 3), round(y, 3), v)
    return {"lethal": lethal, "inscribed": inscribed, "nearest": worst}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--goal", type=float, nargs=3, default=(2.5, 1.0, 0.0), help="map x y yaw")
    ap.add_argument("--clear-radius-m", type=float, default=1.5)
    ap.add_argument("--xy-tol-m", type=float, default=0.35)
    ap.add_argument("--timeout-s", type=float, default=120.0)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = NavGate()
    # Wait for both latched costmaps and the map pose (DDS discovery in a
    # fresh process can take several seconds).
    end = time.time() + 30.0
    while time.time() < end and (len(node.maps) < 2 or node.pose("map") is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    result: dict = {"pass": False}

    def audit(tag: str) -> bool:
        ok = True
        out = {}
        for name, frame in (("local", "odom"), ("global", "map")):
            grid, p = node.maps.get(name), node.pose(frame)
            if grid is None or p is None:
                out[name] = {"error": f"no costmap/pose ({frame})"}
                ok = False
                continue
            out[name] = phantom_cells(grid, p[0], p[1], args.clear_radius_m)
            out[name]["robot"] = [round(v, 3) for v in p]
            ok &= out[name]["lethal"] == 0 and out[name]["inscribed"] == 0
        result[f"costmaps_{tag}"] = out
        return ok

    ok_before = audit("before_goal")
    start = node.pose("map")
    result["start_map_pose"] = start

    goal_ok = False
    if not node.nav.wait_for_server(timeout_sec=20.0):
        result["goal"] = {"error": "navigate_to_pose not available"}
    else:
        gx, gy, gyaw = args.goal
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = gx, gy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        t0 = time.time()
        fut = node.nav.send_goal_async(msg)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        handle = fut.result()
        status = None
        if handle is not None and handle.accepted:
            res_fut = handle.get_result_async()
            while not res_fut.done() and time.time() - t0 < args.timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
            status = res_fut.result().status if res_fut.done() else "timeout"
        node.spin_for(1.0)
        end = node.pose("map")
        err = math.hypot(end[0] - gx, end[1] - gy) if end else None
        goal_ok = status == GoalStatus.STATUS_SUCCEEDED and err is not None and err <= args.xy_tol_m
        result["goal"] = {"target": list(args.goal), "accepted": bool(handle and handle.accepted),
                          "status": status, "succeeded": status == GoalStatus.STATUS_SUCCEEDED,
                          "duration_s": round(time.time() - t0, 1),
                          "end_map_pose": [round(v, 3) for v in end] if end else None,
                          "xy_error_m": round(err, 3) if err is not None else None,
                          "pass": goal_ok}
    ok_after = audit("after_goal")
    result["pass"] = bool(ok_before and goal_ok and ok_after)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        fig, axes = plt.subplots(1, 2, figsize=(13, 6))
        for ax, name, frame in ((axes[0], "local", "odom"), (axes[1], "global", "map")):
            g = node.maps.get(name)
            if g is None:
                continue
            a = np.array(g.data, dtype=float).reshape(g.info.height, g.info.width)
            a[a < 0] = np.nan
            ext = [g.info.origin.position.x, g.info.origin.position.x + g.info.width * g.info.resolution,
                   g.info.origin.position.y, g.info.origin.position.y + g.info.height * g.info.resolution]
            ax.imshow(a, origin="lower", extent=ext, cmap="magma_r", vmin=0, vmax=100)
            p = node.pose(frame)
            if p:
                ax.add_patch(plt.Circle((p[0], p[1]), args.clear_radius_m, fill=False, color="cyan"))
                ax.plot(p[0], p[1], "c^")
            if name == "global":
                ax.plot(args.goal[0], args.goal[1], "g*", ms=14)
            ax.set_title(f"{name} costmap ({frame}) after goal")
        fig.savefig(args.out / "costmaps.png", dpi=100)
    except Exception as exc:  # noqa: BLE001
        result["image_error"] = repr(exc)

    (args.out / "nav_gate.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, indent=2, default=str))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
