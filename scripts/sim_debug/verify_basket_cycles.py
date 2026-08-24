#!/usr/bin/env python3
"""Repeated-cycle reproducer for the basket lift actuator.

The basket lift failed intermittently in a way that a single raise/lower could
not catch, and that a supervisor with retry logic would have hidden: a prismatic
joint parked exactly on its lower position limit sits on a permanently active
DART limit constraint, and gz-sim then cannot drive it with a velocity command
at all — the joint stays frozen even with hold_joints=false and even under
gravity, while every other joint in the same model actuates normally.

This drives LOWERED -> RAISED -> LOWERED -> ... directly on the controller
command topic with a sustained rclpy publisher, with NO retries, and fails a
transition unless all three hold:

  * measured joint velocity becomes non-zero IN THE COMMANDED DIRECTION,
  * the endpoint is reached within a bounded timeout,
  * the joint settles inside the position and velocity tolerances.

Checking the velocity sign matters: a frozen joint and a joint that merely
started late look identical if you only sample the final position.

Run verify_sim_bench.py first — a bench with a dead simulation clock produces
motionless joints for reasons that have nothing to do with this actuator.

Usage:
    python3 scripts/sim_debug/verify_basket_cycles.py --cycles 5
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy

from tennis_robot.basket_lift_mover import JOINT_NAME, BasketLiftDriver

# The reproducer drives the SAME driver the console supervisor invokes, so a
# green run here is evidence about the runtime path, not about a test double.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--travel-m", type=float, default=0.100)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("basket_cycle_reproducer")
    driver = BasketLiftDriver(node)
    failures: list[str] = []
    try:
        if not driver.wait_for_feedback(20.0):
            print(f"[FAIL] no /joint_states feedback for {JOINT_NAME}", file=sys.stderr)
            return 1
        print(f"start position = {(driver.position or 0) * 1000:.3f} mm, "
              f"{args.cycles} cycles, no retries")
        for cycle in range(1, args.cycles + 1):
            for label, target in (("RAISE", args.travel_m), ("LOWER", 0.0)):
                ok, detail = driver.move_to(target, args.timeout_s)
                mark = "ok  " if ok else "FAIL"
                print(f"  cycle {cycle}/{args.cycles} {label:5s} [{mark}] {detail}")
                if not ok:
                    failures.append(f"cycle {cycle} {label}: {detail}")
    finally:
        driver.command(0.0)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        print(f"\nBASKET CYCLES FAILED ({len(failures)}/{args.cycles * 2} transitions)",
              file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(f"\nBASKET CYCLES PASSED ({args.cycles * 2}/{args.cycles * 2} transitions)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
