"""Ground-truth pose logger: gz pose/info -> JSONL (robot + balls).

The ros_gz Pose_V->TFMessage bridge drops entity names (child_frame_id=''),
so ball ground truth is unavailable on the ROS side. This taps the gz topic
directly via the gz CLI JSON output and writes a throttled JSONL timeline.
"""
import json
import subprocess
import sys
import time

OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gz_poses.jsonl"
PERIOD_S = 0.05  # ~20 Hz cap

out = open(OUT_PATH, "w")
proc = subprocess.Popen(
    ["gz", "topic", "-e", "-t", "/world/tennis_court/pose/info", "--json-output"],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    text=True,
)
last = 0.0
try:
    for line in proc.stdout:
        line = line.strip()
        if not line.startswith("{"):
            continue
        now = time.time()
        if now - last < PERIOD_S:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        last = now
        st = msg.get("header", {}).get("stamp", {})
        rec = {
            "t_wall": round(now, 3),
            "t_sim": float(st.get("sec", 0)) + float(st.get("nsec", 0)) / 1e9,
            "poses": [],
        }
        for p in msg.get("pose", []):
            n = p.get("name", "")
            if not (n.startswith("ball_") or n == "tennis_robot" or n.startswith("tennis_robot::")):
                continue
            pos = p.get("position", {})
            entry = {
                "n": n,
                "x": round(pos.get("x", 0.0), 4),
                "y": round(pos.get("y", 0.0), 4),
                "z": round(pos.get("z", 0.0), 4),
            }
            if n == "tennis_robot":
                q = p.get("orientation", {})
                entry["q"] = [q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)]
            rec["poses"].append(entry)
        out.write(json.dumps(rec) + "\n")
        out.flush()
finally:
    proc.terminate()
    out.close()
