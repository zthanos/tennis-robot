#!/usr/bin/env bash
# P2-99c Nav2-stall test, distributed: sim on PC (docker), brain (slam+Nav2) native on the Pi over Wi-Fi AP.
# Run from the repo root on the PC: scripts/sim_debug/p299c_pi_nav_test.sh <run_name>
# Needs the PC on the Pi AP (10.42.0.1), ~/.ssh/id_pi, and the code synced + rebuilt on the Pi.
set -u
cd /home/thanosz/projects/diy/tennis-robot
R=$1; D=runtime/p299c/pi_$R; rm -rf $D; mkdir -p $D
PI=thanos@10.42.0.1
S() { for k in 1 2 3 4 5; do ssh -o IdentityAgent=none -i $HOME/.ssh/id_pi -o ConnectTimeout=8 -o ServerAliveInterval=5 $PI "$@" && return 0; sleep 2; done; return 1; }
PIENV='source /opt/ros/jazzy/setup.bash; cd ~/tennis-robot; source ros2_ws/install_jazzy/setup.bash; export ROS_DOMAIN_ID=42'
ping -D -i 0.2 10.42.0.1 > $D/ping.log 2>&1 &
PINGPID=$!
export ROS_DOMAIN_ID=42 TENNIS_LAUNCH_BRAIN=false TENNIS_PERCEPTION_ON_PC=true
SLAM_MODE=mapping GAZEBO_HEADLESS=true docker/jazzy/run.sh sim > $D/pc_sim.log 2>&1 &
SIMPID=$!
for i in $(seq 1 120); do grep -q "Configured and activated.*diff_drive_controller\|activated diff_drive_controller\|Activating controllers: \[ diff_drive_controller" $D/pc_sim.log && break; sleep 2; done
echo "PC sim up after $((i*2))s"
S "mkdir -p ~/p299c_$R; $PIENV; nohup setsid ./run_pi.sh > ~/p299c_$R/pi_run.log 2>&1 < /dev/null & echo \$! > ~/p299c_$R/pi_run.pid; echo pi started"
for i in $(seq 1 90); do n=$(S "grep -c 'Managed nodes are active' ~/p299c_$R/pi_run.log" 2>/dev/null | tail -1); [ "${n:-0}" -ge 2 ] && break; sleep 4; done
echo "Pi brain up after $((i*4))s (active lines: $n)"
# --- what crosses the Wi-Fi -----------------------------------------------------------
S "$PIENV; IF=\$(ip -o -4 addr | awk '/10.42.0.1/{print \$2}'); a=\$(cat /sys/class/net/\$IF/statistics/rx_bytes); b=\$(cat /sys/class/net/\$IF/statistics/tx_bytes); sleep 10; c=\$(cat /sys/class/net/\$IF/statistics/rx_bytes); d=\$(cat /sys/class/net/\$IF/statistics/tx_bytes); echo \"wifi \$IF rx_kB_s=\$(( (c-a)/10240 )) tx_kB_s=\$(( (d-b)/10240 ))\";
  ros2 topic list --no-daemon > /tmp/p299c_topics.txt; echo \"topics: \$(wc -l < /tmp/p299c_topics.txt)\";
  for t in \$(grep -iE 'image|depth|camera|points' /tmp/p299c_topics.txt); do echo \"\$t subscribers: \$(ros2 topic info -v --no-daemon \$t | grep -A2 'Endpoint type: SUBSCRIPTION' ; ros2 topic info -v --no-daemon \$t | awk '/Node name/{n=\$3} /SUBSCRIPTION/{printf n\" \"}')\"; done;
  for t in /scan /tf /tf_static /clock /imu/data /joint_states /diff_drive_controller/odom /odometry/filtered /map /perception/ball_detections /survey/vision /robot_status; do grep -qx \$t /tmp/p299c_topics.txt || continue; timeout 7 ros2 topic bw \$t 2>/dev/null | grep -m1 -E 'B/s' | sed \"s#^#\$t #\"; done;
  fuser /dev/ttyACM0 2>&1 | sed 's/^/ttyACM0 users: /'; echo ttyACM0_check_done" > $D/wifi_bw.txt 2>&1
# --- the test --------------------------------------------------------------------------
S "$PIENV; timeout 240 python3 scripts/sim_debug/p299c_tf_age_probe.py --out ~/p299c_$R/probe --duration-s 200 --goal 20,2.5,1.0,0.0 --goal 50,0.0,0.0,0.0 --goal 80,2.5,1.0,0.0 --goal 120,0.0,0.0,0.0 --goal 150,2.5,1.0,0.0" > $D/probe.log 2>&1
# --- stop everything on the Pi, then the PC -------------------------------------------
S "pid=\$(cat ~/p299c_$R/pi_run.pid); pgid=\$(ps -o pgid= \$pid | tr -d ' '); [ -n \"\$pgid\" ] && kill -INT -- -\$pgid; sleep 8; pkill -INT -f 'ros2 launch tennis_robot' ; sleep 4; pkill -9 -f 'ros2 launch tennis_robot|/opt/ros/jazzy/lib|tennis-robot/ros2_ws/install_jazzy' ; sleep 2; rm -f /tmp/tennis_robot_pi_domain_42.lock; echo LEFT:; pkill -f ros2cli.daemon; sleep 1; pgrep -af 'ros2|nav2|slam|tennis_robot|python3 .*p299c' | grep -v pgrep || echo none"> $D/pi_stop.txt 2>&1
S "cat ~/p299c_$R/probe/tf_age.csv" > $D/tf_age.csv 2>/dev/null; S "cat ~/p299c_$R/pi_run.log" > $D/pi_run.log 2>/dev/null
kill $SIMPID; wait $SIMPID 2>/dev/null; kill -INT $PINGPID; sleep 1
for i in $(seq 1 10); do [ "$(docker ps -q | wc -l)" = 0 ] && break; sleep 3; done
echo "--- result $R"; tail -1 $D/probe.log; cat $D/wifi_bw.txt | head -20; cat $D/pi_stop.txt
echo "too_old=$(grep -c 'Transform data too old' $D/pi_run.log) first: $(grep -m1 'Transform time' $D/pi_run.log | sed 's/.*Data time/Data time/')"
grep -E "packet loss|rtt" -A0 $D/ping.log | tail -2
