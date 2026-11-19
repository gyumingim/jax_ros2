#!/usr/bin/env bash
# Camera pose -> approach -> learned grasp (grasp_lift_25550) -> L return, on vcan, with a fake hand.
#
#   tests/grasp_vcan.sh          (after colcon build; no sudo: a private network namespace with vcan can0)
#
# ak45_vcan_sim.py plays the AK45s, fake_hand.py plays the STM pressure sensors (contact past a touch
# angle), approach_driver.py plays vision_node and calls /approach/start. Scenarios:
#   A fake hand: phases ... INSERT GRASP RETURN DONE, the last command is the L pose (README 7.7),
#     the fingers closed, no ERROR/Traceback
#   B no pressure topics at all: the approach ends in HOLD, as before the grasp existed
#   C pads never touch: GRASP_TIMEOUT after grasp_timeout_s (3 s here), the arm stays put
here="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${IN_NETNS:-}" ]; then
    exec env IN_NETNS=1 unshare -Urn bash "$0" "$@"
fi
set -m
ws="$(cd "$here/../../../.." && pwd)"
sim="$here/../../ak45_can/tools/ak45_vcan_sim.py"
drv="$here/approach_driver.py"
out="$(mktemp -d)"
ip link set lo up
ip link add can0 type vcan && ip link set can0 up || { echo "cannot create vcan can0 (is the vcan module loaded?)"; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=78

fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1"; fails=$((fails + 1)); fi; }
field() { python3 -c "import json,sys; d=json.load(open('$1')); print($2)"; }

up() {   # $1 = tag, $2 = fake hand args ("off" = no fake hand), $3.. = approach_node params
    tag="$1"; local hand="$2"; shift 2
    python3 "$sim" --iface can0 --ids 1,2,3,4 > "$out/sim_$tag.log" 2>&1 & simpid=$!
    ros2 launch arm_control arm.launch.py > "$out/launch_$tag.log" 2>&1 & launchpid=$!
    handpid=""
    if [ "$hand" != "off" ]; then python3 "$here/fake_hand.py" $hand > "$out/hand_$tag.log" 2>&1 & handpid=$!; fi
    ros2 run arm_control approach_node --ros-args -p hold_s:=1.0 -p force:=true "$@" > "$out/node_$tag.log" 2>&1 & nodepid=$!
    sleep 6
    # like tools/go_zero.sh: approach_node assumes every sim angle is 0 at start
    timeout 20 ros2 topic pub --once --wait-matching-subscriptions 1 /arm_control/joint_command sensor_msgs/msg/JointState \
        "{name: [shoulder_roll, shoulder_yaw, upper_arm_roll, elbow], position: [0.0, 0.0, 0.0, 0.0]}" > /dev/null 2>&1
    sleep 5
}
down() {
    kill -INT "$nodepid" 2>/dev/null; kill -INT "$launchpid" 2>/dev/null; [ -n "$handpid" ] && kill -INT "$handpid" 2>/dev/null
    sleep 3
    kill -KILL -- -"$launchpid" -"$nodepid" 2>/dev/null; [ -n "$handpid" ] && kill -KILL "$handpid" 2>/dev/null
    kill -- -"$simpid" 2>/dev/null; sleep 1
}
phases() { field "$1" "' '.join(p for _, p in d['phases'])"; }

echo "== A: fake hand -> GRASP RETURN DONE"
up a "--touch 6 6 -26"
python3 "$drv" --out "$out/a.json" --timeout 45
echo "   phases: $(phases "$out/a.json")"
check "start accepted" "$(field "$out/a.json" "int(d['start']['success'])")"
check "phases end INSERT GRASP RETURN DONE" "$(field "$out/a.json" "int([p for _,p in d['phases']][-4:] == ['INSERT','GRASP','RETURN','DONE'])")"
python3 - "$out/a.json" <<'EOF' > "$out/a_checks.txt"
import json, sys
from math import degrees
d = json.load(open(sys.argv[1]))
last = dict(zip(d["commands"][-1][1], d["commands"][-1][2]))
want = {"shoulder_roll": 0.0, "shoulder_yaw": 0.0, "upper_arm_roll": 7.2, "elbow": 90.0, "wrist_pitch": 0.0, "wrist_yaw": -2.6, "hand_roll": 0.0}
err = max(abs(degrees(last[n]) - v) for n, v in want.items())
print(f"PASS  last command = the L pose (README 7.7) within 0.05 deg (max error {err:.3f})" if err < 0.05 else f"FAIL  last command = the L pose: max error {err:.3f} deg")
f1, f3 = degrees(last["finger1_joint1"]), degrees(last["finger3_joint1"])
print(f"PASS  fingers closed past the touch angles (finger1 {f1:.1f} > 6, thumb {f3:.1f} > -26 deg)" if f1 > 6 and f3 > -26 else f"FAIL  fingers not closed: {f1:.1f}, {f3:.1f}")
g = [c for c in d["commands"] if c[0] >= d["start_t"]]
print(f"PASS  commands keep coming during the grasp ({len(g)} after start)" if len(g) > 600 else f"FAIL  only {len(g)} commands")
EOF
cat "$out/a_checks.txt"; fails=$((fails + $(grep -c '^FAIL' "$out/a_checks.txt")))
check "no ERROR/Traceback in approach_node log" "$(grep -qE 'ERROR|Traceback' "$out/node_a.log" && echo 0 || echo 1)"
down

echo "== B: no pressure topics -> HOLD as before"
up b off
python3 "$drv" --out "$out/b.json" --timeout 25
echo "   phases: $(phases "$out/b.json")"
check "phases end INSERT HOLD" "$(field "$out/b.json" "int([p for _,p in d['phases']][-2:] == ['INSERT','HOLD'])")"
check "log says why the grasp did not start" "$(grep -q 'grasp does not start: no /pressure' "$out/node_b.log" && echo 1 || echo 0)"
down

echo "== C: pads never touch -> GRASP_TIMEOUT"
up c "--none" -p grasp_timeout_s:=3.0
python3 "$drv" --out "$out/c.json" --timeout 30
echo "   phases: $(phases "$out/c.json")"
check "phases end GRASP GRASP_TIMEOUT" "$(field "$out/c.json" "int([p for _,p in d['phases']][-2:] == ['GRASP','GRASP_TIMEOUT'])")"
check "log names the timeout" "$(grep -q 'not lifted 4 cm within 3 s' "$out/node_c.log" && echo 1 || echo 0)"
down

echo "   logs: $out"
echo "== pgrep after"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node|approach_node|fake_hand' || echo "(none)"
echo
[ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"
[ "$fails" = 0 ]
