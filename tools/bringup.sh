#!/usr/bin/env bash
# One-command bring-up: stops every old arm/camera process, resets can0, then runs
# `ros2 launch arm_control approach.launch.py`. Details: BRINGUP.md.
#
#   tools/bringup.sh camera [result_json:=/path/inference.json] [name:=value ...]
#   tools/bringup.sh fixed "[0.10, -0.30, 20.0]" [name:=value ...]      # no camera: fixed test pose
#
# name:=value arguments go to the launch as they are (force:=true, camera_to_robot:="[...]", ...).
# Steps: 1. y/N (old nodes stop -> motor torque is released, the arm can drop)  2. stop ak45_node,
# stm_bridge_node, arm_control_node, vision_node, approach_node, calibrate and every `ros2 launch` /
# `ros2 run`  3. wait until /tmp/ak45_ctrl.lock is free (stop if not)  4. can0 down/up, bitrate 1000000
# 5. the launch. It never calls /approach/start: moving the arm is a separate, manual step.
#
# Environment (for tests): BRINGUP_YES=1 skips the question, BRINGUP_CAN=skip leaves can0 alone,
# BRINGUP_SETUP=<setup.bash> sources another install.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/.." && pwd)"
ws="$(cd "$repo/../.." && pwd)"
DEFAULT_JSON="$HOME/Desktop/StereoDepthSeg/phone_stereo/runs/cube_test/inference.json"
LOCK=/tmp/ak45_ctrl.lock

usage() { sed -n '4,6p' "$0" | sed 's/^# //'; exit 2; }

mode="${1:-}"; shift || true
launch_args=()
case "$mode" in
    camera)
        printf '%s\n' "$@" | grep -q '^result_json:=' || launch_args+=("result_json:=$DEFAULT_JSON") ;;
    fixed)
        [ $# -ge 1 ] || usage
        launch_args+=("fixed_pose:=$1"); shift ;;
    *) usage ;;
esac
launch_args+=("$@")

# --- AK45 calibration as it is (printed only, never changed)
echo "== arm_control/calibration.json, AK45 rows"
python3 - "$repo/arm_control/calibration.json" <<'EOF'
import json, sys
for j in json.load(open(sys.argv[1]))["joints"]:
    if j["motor"] == "ak45":
        print(f"  {j['name']:7} {j['joint']:15} sign {j['sign']:+d}  zero_deg {j['zero_deg']:7.2f}  verified {j['verified']}")
EOF

# --- 1. confirm
if [ "${BRINGUP_YES:-}" != "1" ]; then
    read -r -p "Old nodes will be stopped: motor torque is released and the arm can drop. Hold the arm. Continue? [y/N] " ans
    case "$ans" in y|Y) ;; *) echo "cancelled"; exit 0 ;; esac
fi

# --- 2. stop old processes (SIGINT first, then SIGKILL after 5 s)
PATTERN='lib/(ak45_ros2/ak45_node|stm_bridge/stm_bridge_node|arm_control/(arm_control_node|approach_node|calibrate)|vision_node/vision_node)|bin/ros2 (launch|run) '
if pgrep -f "$PATTERN" >/dev/null; then
    echo "== stopping:"; pgrep -fa "$PATTERN" | sed 's/^/  /'
    pkill -INT -f "$PATTERN"
    for _ in $(seq 50); do pgrep -f "$PATTERN" >/dev/null || break; sleep 0.1; done
    if pgrep -f "$PATTERN" >/dev/null; then
        echo "  still running after 5 s: SIGKILL"; pkill -KILL -f "$PATTERN"; sleep 0.5
    fi
else
    echo "== no old process"
fi

# --- 3. the AK45 lock must be free
for _ in $(seq 100); do
    [ -e "$LOCK" ] && fuser -s "$LOCK" 2>/dev/null || break
    sleep 0.1
done
if [ -e "$LOCK" ] && fuser -s "$LOCK" 2>/dev/null; then
    echo "stop: $LOCK is still held by:"; fuser -v "$LOCK" 2>&1 | sed 's/^/  /'
    exit 1
fi
echo "== $LOCK is free"

# --- 4. can0 down/up
if [ "${BRINGUP_CAN:-}" = "skip" ]; then
    echo "== can0 left as it is (BRINGUP_CAN=skip)"
else
    echo "== can0 down/up, bitrate 1000000 (sudo)"
    sudo ip link set can0 down 2>/dev/null
    sudo ip link set can0 up type can bitrate 1000000 || { echo "stop: can0 did not come up"; exit 1; }
fi

# --- 5. launch
set +u      # the ROS setup files use unset variables
source /opt/ros/jazzy/setup.bash
source "${BRINGUP_SETUP:-$ws/install/setup.bash}"
echo "== ros2 launch arm_control approach.launch.py ${launch_args[*]}"
echo "   (moving the arm: tools/go_zero.sh, then ros2 service call /approach/start std_srvs/srv/Trigger)"
exec ros2 launch arm_control approach.launch.py "${launch_args[@]}"
