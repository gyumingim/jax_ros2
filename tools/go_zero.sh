#!/usr/bin/env bash
# Moves the 4 AK45 joints to sim angle 0 (shoulder_roll, shoulder_yaw, upper_arm_roll, elbow) through
# arm_control_node, so approach_node can start (it assumes every sim angle is 0 at start, and stops when
# an AK45 is more than 15 deg off its command for 0.5 s).
#
#   tools/go_zero.sh          (ak45_node, stm_bridge_node and arm_control_node must be running)
#
# 1. `ak45_deg --list`: all 4 AK45 (ak45_1..ak45_4) must have feedback, else it stops.
# 2. shows the current angles and asks y/N.
# 3. publishes once to /arm_control/joint_command: all four 0.0 (sim angles; arm_control_node turns them
#    into motor angles with calibration.json and moves at its speed limit).
# 4. after 5 s shows `ak45_deg --list` again.
# THE ARM MOVES. Keep a hand near it.
set -u

list() { ros2 run ak45_ros2 ak45_deg --list 2>&1; }

echo "== current AK45 state (ak45_deg --list)"
now="$(list)"
echo "$now"
missing=""
for n in ak45_1 ak45_2 ak45_3 ak45_4; do
    echo "$now" | grep -qE "^${n}[[:space:]]" || missing="$missing $n"
done
if [ -n "$missing" ]; then
    echo "stop: no feedback from:$missing (is ak45_node running, is can0 up?)"
    exit 1
fi

read -r -p "Move shoulder_roll, shoulder_yaw, upper_arm_roll, elbow to sim 0? THE ARM MOVES. [y/N] " ans
case "$ans" in
    y|Y) ;;
    *) echo "cancelled, nothing sent"; exit 0 ;;
esac

ros2 topic pub --once /arm_control/joint_command sensor_msgs/msg/JointState \
    "{name: [shoulder_roll, shoulder_yaw, upper_arm_roll, elbow], position: [0.0, 0.0, 0.0, 0.0]}" || exit 1

echo "== sent; ak45_deg --list again in 5 s"
sleep 5
list
