"""Offline analysis of a collect_one attempt: rosbag + gz pose JSONL.

Builds a timeline of ball-to-roller distance vs roller contact / joint
velocity / intake beam, finds the closest approach, and dumps the debug-camera
frames around it so the failure moment can be inspected visually.

Usage (inside the gazebo container):
  python3 analyze_collect_bag.py <bag_dir> <poses.jsonl> <frames_dir> <out_dir>

frames_dir holds f_<epoch>.png files from dump_intake_frames.py; they are
matched to the ground-truth timeline by wall clock.
"""
import glob
import json
import math
import os
import shutil
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROLLER_BASE_X = 0.600
ROLLER_BASE_Z = 0.112  # ground frame, before offsets
X_OFF = float(os.getenv("INTAKE_ROLLER_X_OFFSET_M", "0.015"))
LIP_X_OFF = float(os.getenv("INTAKE_LIP_X_OFFSET_M", "-0.006"))
Z_OFF = float(os.getenv("INTAKE_ROLLER_Z_OFFSET_M", "-0.003"))
ROLLER_X = ROLLER_BASE_X + X_OFF
LIP_X = ROLLER_X + LIP_X_OFF
ROLLER_Z = ROLLER_BASE_Z + Z_OFF
ROLLER_R = 0.045
BALL_R = 0.033
NOMINAL_BITE_DX = math.sqrt(
    max(0.0, (ROLLER_R + BALL_R) ** 2 - (ROLLER_Z - BALL_R) ** 2)
)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield topic, deserialize_message(data, types[topic]), t_ns / 1e9


def main():
    bag_dir, poses_path, frames_dir, out_dir = (
        sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    )
    os.makedirs(out_dir, exist_ok=True)

    # ── ground-truth timeline from gz poses ─────────────────────────────
    timeline = []  # (t_wall, ball_name, dist_3d, dx_fwd, dy_lat, ball_z)
    for line in open(poses_path):
        rec = json.loads(line)
        robot = next((p for p in rec["poses"] if p["n"] == "tennis_robot"), None)
        if robot is None:
            continue
        ryaw = yaw_of(robot["q"])
        cos_y, sin_y = math.cos(-ryaw), math.sin(-ryaw)
        # roller centre in world
        rx = robot["x"] + math.cos(ryaw) * ROLLER_X
        ry = robot["y"] + math.sin(ryaw) * ROLLER_X
        rz = ROLLER_Z  # robot z ~ 0
        best = None
        for p in rec["poses"]:
            if not p["n"].startswith("ball_"):
                continue
            d = math.sqrt((p["x"] - rx) ** 2 + (p["y"] - ry) ** 2 + (p["z"] - rz) ** 2)
            if best is None or d < best[2]:
                # ball position in base frame for fwd/lat decomposition
                bx, by = p["x"] - robot["x"], p["y"] - robot["y"]
                fwd = cos_y * bx - sin_y * by
                lat = sin_y * bx + cos_y * by
                best = (p["n"], d, d, fwd - ROLLER_X, lat, p["z"])
        if best:
            timeline.append((rec["t_wall"], best[0], best[1], best[3], best[4], best[5]))

    if not timeline:
        print("NO ground-truth timeline — poses.jsonl empty?")
        return

    closest = min(timeline, key=lambda r: r[2])
    t_close, ball, dmin, dxf, dyl, bz = closest
    surface_gap = dmin - ROLLER_R - BALL_R
    print(f"CLOSEST APPROACH: ball={ball} t_wall={t_close:.2f}")
    print(
        f"  geometry: roller_x={ROLLER_X*1000:.0f}mm lip_x={LIP_X*1000:.0f}mm "
        f"nominal bite dx={NOMINAL_BITE_DX*1000:.0f}mm"
    )
    print(f"  centre-to-centre={dmin*1000:.0f}mm  surface gap={surface_gap*1000:.0f}mm")
    print(f"  fwd offset from roller axis={dxf*1000:+.0f}mm lateral={dyl*1000:+.0f}mm ball_z={bz*1000:.0f}mm")

    # ── contact / beam / joint events from the bag ──────────────────────
    contacts = 0
    contact_ts = []
    beam_true = []
    joint_vel_at = []
    for topic, msg, t in read_bag(bag_dir):
        if topic == "/gz/roller_contact_0":
            names = " ".join(
                f"{c.collision1.name} {c.collision2.name}" for c in msg.contacts
            )
            if "ball_" in names:
                contacts += 1
                contact_ts.append(t)
        elif topic == "/collector/intake_beam_broken" and msg.data:
            beam_true.append(t)
        elif topic == "/joint_states":
            try:
                i = list(msg.name).index("lift_wheel_joint")
                joint_vel_at.append((t, msg.velocity[i] if i < len(msg.velocity) else 0.0))
            except ValueError:
                pass

    print(f"BALL-CONTACT messages on /gz/roller_contact_0: {contacts}")
    if contact_ts:
        print(f"  first={contact_ts[0]:.2f} last={contact_ts[-1]:.2f}")
    print(f"intake beam TRUE count: {len(beam_true)}")
    if joint_vel_at:
        vmax = max(abs(v) for _, v in joint_vel_at)
        print(f"lift_wheel_joint |vel| max: {vmax:.2f} rad/s over {len(joint_vel_at)} samples")

    # ── pick dumped camera frames around closest approach (wall clock) ──
    frame_files = []
    for path in glob.glob(os.path.join(frames_dir, "f_*.png")):
        try:
            frame_files.append((float(os.path.basename(path)[2:-4]), path))
        except ValueError:
            continue
    if frame_files:
        picked = 0
        for dt in (-3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0):
            w = t_close + dt
            ft, path = min(frame_files, key=lambda f: abs(f[0] - w))
            if abs(ft - w) > 1.0:
                continue
            shutil.copy(path, os.path.join(out_dir, f"closest_off{dt:+.1f}.png"))
            picked += 1
        print(f"picked {picked} frames around closest approach into {out_dir}")

    # full distance timeline for plotting/inspection
    with open(os.path.join(out_dir, "timeline.jsonl"), "w") as f:
        for t, b, d, fx, ly, bz_ in timeline:
            f.write(json.dumps({
                "t": t, "ball": b, "dist_mm": round(d * 1000),
                "fwd_mm": round(fx * 1000), "lat_mm": round(ly * 1000),
                "ball_z_mm": round(bz_ * 1000),
            }) + "\n")
    print("wrote timeline.jsonl")


if __name__ == "__main__":
    main()
