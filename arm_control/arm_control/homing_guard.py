"""homing_guard: sits between arm_control_node and ak45_node, and lets an AK45 move only after it was
homed in this session (homing.py says why and how).

    arm_control_node --/ak45/command_unguarded--> homing_guard --/ak45/command--> ak45_node
    arm_control_node <--/ak45/joint_states_homed-- homing_guard <--/joint_states-- ak45_node

arm_control_node is the same node as in arm.launch.py; arm_homed.launch.py remaps its /ak45/command and
/joint_states to the two topics above. Nothing in arm_control_node changes.

  - not homed: its commands are dropped (warning) and its feedback is not passed on, so arm_control_node
    waits for it as for a motor that does not answer.
  - homed: command + offset goes to ak45_node, feedback - offset comes back (offset = session zero -
    calibration.json zero_deg). Commands are ramped again here at max_speed_deg_s from the motor's
    feedback, so a joint homed again never jumps to a target arm_control_node kept from before.
  - power cycle seen (feedback gone for gap_s, or a jump of jump_deg): not homed again, its ramp and
    target here are dropped, and when its feedback comes back ak45_node is told to hold the motor
    where it is (ak45_node keeps resending the last target it got, which after a power cycle points
    somewhere else).

Interfaces (for a dashboard too):
    /arm_control/homing        std_msgs/String, JSON (homing.Homing.status), 2 Hz and on every change,
                               transient local (a late subscriber gets the last one)
    /arm_control/home_all      std_srvs/Trigger: home every AK45 that is not homed, at its home pose
    /arm_control/home_<joint>  std_srvs/Trigger: home one (home_shoulder_roll, home_elbow, ...)
    /arm_control/unhome_all    std_srvs/Trigger: forget every session zero (home again from scratch)
Home poses: homing.json `home_sim_deg` (sim degrees per joint, missing = 0).
"""
import json
import time
from math import degrees, isfinite, radians

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import calibration
from .homing import Homing, load_home_poses
from .motion import Mover
from .ros_io import ak45_topic_name

IN_COMMAND = "/ak45/command_unguarded"
OUT_COMMAND = "/ak45/command"
IN_STATES = "/joint_states"
OUT_STATES = "/ak45/joint_states_homed"
STATUS = "/arm_control/homing"
SERVICE_PREFIX = "/arm_control/"


class HomingGuard(Node):
    def __init__(self):
        super().__init__("homing_guard")
        p = lambda n, v: self.declare_parameter(n, v).value
        cal = calibration.load(p("calibration_file", "") or None)
        poses = load_home_poses(p("home_file", "") or None)
        hz = p("rate_hz", 50.0)
        self.homing = Homing({ak45_topic_name(j): j for j in cal.joints if j.motor == "ak45"}, poses,
                             gap_s=p("gap_s", 1.0), jump_deg=p("jump_deg", 60.0),
                             max_age_s=p("max_age_s", 0.5), warn_deg=p("warn_deg", 20.0))
        self.mover = Mover(p("max_speed_deg_s", 30.0), hz)
        self.pending_hold = set()       # power-cycled motors to hold where they are once feedback is back
        self.cmd_pub = self.create_publisher(JointState, OUT_COMMAND, 10)
        self.state_pub = self.create_publisher(JointState, OUT_STATES, 10)
        self.status_pub = self.create_publisher(
            String, STATUS, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, IN_STATES, self.on_state, 10)
        self.create_subscription(JointState, IN_COMMAND, self.on_command, 10)
        self.create_service(Trigger, SERVICE_PREFIX + "home_all", self.srv_home_all)
        self.create_service(Trigger, SERVICE_PREFIX + "unhome_all", self.srv_unhome_all)
        for n, h in self.homing.m.items():
            self.create_service(Trigger, f"{SERVICE_PREFIX}home_{h.joint.joint}",
                                lambda req, res, n=n: self.srv_home(res, [n]))
        self.create_timer(1.0 / hz, self.on_timer)
        self._tick, self._status_every = 0, max(1, int(hz / 2))
        names = ", ".join(f"{h.joint.joint}={n} home sim {h.home_sim_deg:g}" for n, h in self.homing.m.items())
        self.get_logger().info(f"every AK45 starts NOT homed ({names}); call {SERVICE_PREFIX}home_all "
                               "with the arm at its home pose")
        self.publish_status()

    # -- state -----------------------------------------------------------------------------------
    def _forget(self, name):
        self.mover.current.pop(name, None)
        self.mover.target.pop(name, None)

    def _hold(self, names):
        """Tell ak45_node to hold these motors at their present raw feedback."""
        hold = [(n, self.homing.m[n].raw_deg) for n in names]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [n for n, _ in hold]
        msg.position = [radians(d) for _, d in hold]
        self.cmd_pub.publish(msg)

    def on_state(self, msg):
        now = time.monotonic()
        out_n, out_p, changed, hold = [], [], False, []
        for n, p in zip(msg.name, msg.position):
            if n not in self.homing.m:
                out_n.append(n)          # not one of ours: passed on as it is
                out_p.append(p)
                continue
            if not isfinite(p):
                continue
            event, text = self.homing.feedback(n, degrees(p), now)
            if event:
                self.get_logger().warning(text)
                self._forget(n)
                self.pending_hold.add(n)
                changed = True
            if n in self.pending_hold:
                hold.append(n)
            d = self.homing.from_raw(n, degrees(p))
            if d is not None:
                out_n.append(n)
                out_p.append(radians(d))
        if hold:
            self._hold(hold)
            self.pending_hold.difference_update(hold)
            self.get_logger().warning(f"told ak45_node to hold {', '.join(hold)} where it is now")
        if out_n:
            o = JointState()
            o.header = msg.header
            o.name, o.position = out_n, out_p
            self.state_pub.publish(o)
        if changed:
            self.publish_status()

    def on_command(self, msg):
        log = self.get_logger()
        if len(msg.name) != len(msg.position) or any(n not in self.homing.m for n in msg.name) \
                or not all(isfinite(p) for p in msg.position):
            log.warning("bad /ak45/command_unguarded message (length, name or value); ignored",
                        throttle_duration_sec=5.0)
            return
        refused = []
        for n, p in zip(msg.name, msg.position):
            raw = self.homing.to_raw(n, degrees(p))
            if raw is None:
                refused.append(self.homing.m[n].joint.joint)
                self._forget(n)
                continue
            self.mover.set_target(n, raw)
        if refused:
            log.warning(f"not homed, command ignored: {', '.join(refused)} "
                        f"(home with {SERVICE_PREFIX}home_all or home_<joint>)", throttle_duration_sec=2.0)

    def on_timer(self):
        now = time.monotonic()
        lost = self.homing.check_lost(now)
        for text in lost:
            self.get_logger().warning(text)
        for h_name in [n for n, h in self.homing.m.items() if h.lost]:
            self._forget(h_name)
            self.pending_hold.add(h_name)
        for n in self.mover.needs_start():
            h = self.homing.m[n]
            if h.homed and h.seen is not None and now - h.seen <= self.homing.max_age_s:
                self.mover.start(n, h.raw_deg)
        moved = self.mover.step()
        if moved:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(moved)
            msg.position = [radians(d) for d in moved.values()]
            self.cmd_pub.publish(msg)
        self._tick += 1
        if lost or self._tick % self._status_every == 0:
            self.publish_status()

    def publish_status(self):
        s = self.homing.status(time.monotonic())
        s["stamp"] = time.time()
        self.status_pub.publish(String(data=json.dumps(s)))

    # -- services --------------------------------------------------------------------------------
    def srv_home(self, res, names):
        now = time.monotonic()
        texts, ok = [], True
        for n in names:
            good, text = self.homing.home(n, now)
            if good and "WARNING" not in text:     # rclpy: one call site, one severity
                self.get_logger().info(text)
            else:
                self.get_logger().warning(text)
            texts.append(text)
            ok = ok and good
            if good:
                self._forget(n)
                self.mover.start(n, self.homing.m[n].raw_deg)
                self._hold([n])
                self.pending_hold.discard(n)
        self.publish_status()
        res.success, res.message = ok, "; ".join(texts) or "nothing to home (all homed)"
        return res

    def srv_home_all(self, req, res):
        return self.srv_home(res, [n for n, h in self.homing.m.items() if not h.homed])

    def srv_unhome_all(self, req, res):
        for n, h in self.homing.m.items():
            self.homing._unhome(h, "unhomed by request")
            self._forget(n)
        self.get_logger().warning("every AK45 NOT homed (unhome_all)")
        self.publish_status()
        res.success, res.message = True, "every AK45 is not homed now"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = HomingGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
