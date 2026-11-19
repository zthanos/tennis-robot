#!/usr/bin/env python3
"""P2-99c (d1b) gate: shared-flywheel tilt and flywheel spin in Gazebo.

Drives launcher_tilt_velocity_controller to the +20 deg seat, back to the
-3 deg seat, then to 0 deg (transport), and spins the flywheels with
flywheel_velocity_controller. Reads /joint_states. Optionally stops at each
seat for --hold-s so an external /scan gate can run (writes the current pose
name to <out>/pose.txt).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rate", type=float, default=0.09)
    ap.add_argument("--hold-s", type=float, default=0.0)
    ap.add_argument("--fly-rad-s", type=float, default=150.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node("p299c_tilt_gate")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    js: dict = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: js.update({k: (p, v) for k, p, v in zip(m.name, m.position, m.velocity or [0.0] * len(m.name))}), 20)
    tilt = n.create_publisher(Float64MultiArray, "/launcher_tilt_velocity_controller/commands", 10)
    fly = n.create_publisher(Float64MultiArray, "/flywheel_velocity_controller/commands", 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def angle():
        return js.get("launcher_tilt_joint", (None, None))[0]

    def drive_to(target, sign, timeout=20.0):
        t0 = time.time(); trace = []
        while time.time() - t0 < timeout:
            tilt.publish(Float64MultiArray(data=[sign * a.rate]))
            spin(0.05)
            ang = angle()
            if ang is not None:
                trace.append(ang)
                if (sign > 0 and ang >= target - 0.002) or (sign < 0 and ang <= target + 0.002):
                    break
        tilt.publish(Float64MultiArray(data=[0.0])); spin(1.0)
        return {"target_deg": round(math.degrees(target), 2), "reached_deg": round(math.degrees(angle()), 3),
                "wall_s": round(time.time() - t0, 2)}

    def drive_hard(sign, sec):
        """Command into the seat for `sec` s: the joint limit must hold it."""
        t0 = time.time()
        while time.time() - t0 < sec:
            tilt.publish(Float64MultiArray(data=[sign * a.rate])); spin(0.05)
        held = angle()
        tilt.publish(Float64MultiArray(data=[0.0])); spin(1.0)
        return round(math.degrees(held), 3)

    def hold(name):
        if a.hold_s > 0:
            (a.out / "pose.txt").write_text(name)
            t0 = time.time()
            while time.time() - t0 < a.hold_s:
                tilt.publish(Float64MultiArray(data=[0.0])); spin(0.1)

    spin(3.0)
    res = {"start_deg": round(math.degrees(angle()), 3) if angle() is not None else None}
    res["to_plus20"] = drive_to(math.radians(20.0), +1)
    res["push_into_plus20_seat_deg"] = drive_hard(+1, 3.0)
    hold("plus20")
    res["to_minus3"] = drive_to(math.radians(-3.0), -1, 30.0)
    hold("minus3")
    # NOT pushed into the modelled -3.5 deg stop: touching the lower limit
    # freezes the joint under velocity control (DART). The supervisor stops at
    # the -3 deg seat (overshoot ~0.16 deg < the 0.5 deg margin).
    res["to_zero"] = drive_to(0.0, +1)
    hold("zero")
    # flywheels: left +, right - (both inner faces drive the ball forward)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        fly.publish(Float64MultiArray(data=[a.fly_rad_s, -a.fly_rad_s])); spin(0.05)
    res["flywheel_cmd_rad_s"] = a.fly_rad_s
    res["flywheel_vel_rad_s"] = [round(js.get(f"flywheel_{s}_joint", (0, 0))[1], 2) for s in ("left", "right")]
    fly.publish(Float64MultiArray(data=[0.0, 0.0])); spin(1.0)
    rate_deg_s = math.degrees(a.rate)
    res["pass"] = bool(
        abs(res["push_into_plus20_seat_deg"] - 20.0) < 0.5 and abs(res["to_minus3"]["reached_deg"] + 3.0) < 0.5
        and abs(res["to_zero"]["reached_deg"]) < 0.5
        and abs(res["flywheel_vel_rad_s"][0] - a.fly_rad_s) < 0.1 * a.fly_rad_s
        and abs(res["flywheel_vel_rad_s"][1] + a.fly_rad_s) < 0.1 * a.fly_rad_s)
    res["note"] = f"rated tilt speed {rate_deg_s:.2f} deg/s -> ideal -3..+20 in {23 / rate_deg_s:.1f} s"
    (a.out / "tilt_gate.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    n.destroy_node(); rclpy.shutdown()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
