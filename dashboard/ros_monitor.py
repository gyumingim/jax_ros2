"""ROS 구독: /diagnostics (ak45_node 의 ak45/<joint>, ak45/can_bus), /joint_states,
/arm_control/homing (homing_guard 의 원점 상태 JSON, transient local),
/servo/state (stm_bridge_node 가 마지막으로 보낸 서보 각도 — 측정값 아님), /object_pose (vision_node).

rclpy 가 없으면(Windows, 또는 ROS 를 source 하지 않고 서버를 켬) available=False 로 두고 아무것도 안 한다.
/ak45/joint_states_homed (homing_guard 가 원점 오프셋을 뺀 피드백 — arm_homed 일 때 sim 각 환산에 쓴다).

발행하는 명령 토픽은 하나: /arm_control/joint_command (sensor_msgs/JointState, sim 관절명, rad).
각도 제어 탭의 '보내기' 버튼을 누를 때만(publish_joint_command), 서버가 관절명·범위·유한값을 검사한 뒤.
arm_control_node 가 램프(max_speed_deg_s)·calibration 클립을, arm_homed 면 homing_guard 가 원점 검사를 한다.
부르는 서비스는 homing_guard 의 std_srvs/Trigger(/arm_control/home_all, home_<관절>, release_all, release_<관절>).
"""
import json
import math
import threading
import time


HOMING_TOPIC = "/arm_control/homing"
HOMING_SRV = "/arm_control/"   # + home_all, home_<관절>, release_all, release_<관절>
JOINT_COMMAND = "/arm_control/joint_command"
HOMED_STATES = "/ak45/joint_states_homed"


class RosMonitor:
    def __init__(self):
        self.available = False
        self.reason = "시작 전"
        self.lock = threading.Lock()
        self.diag = {}        # name -> {level, message, hardware_id, values{}, t}
        self.joints = {}      # name -> {deg, t}
        self.joint_t = None
        self.joints_homed = {}  # /ak45/joint_states_homed: name -> {deg, t}
        self._cmd_pub = None
        self._JointState = None
        self.diag_t = None
        self.homing = None    # homing_guard 의 마지막 JSON (homing.Homing.status + stamp)
        self.homing_t = None  # 받은 시각(이 PC time.time)
        self.servos = {}      # name -> {deg, t}  (/servo/state: 마지막으로 보낸 모터 각, rad -> deg)
        self.servo_t = None
        self.obj = None       # /object_pose 마지막 값 {x, y, z, yaw_deg, frame_id, stamp, t}
        self._node = None
        self._Trigger = None
        self._clients = {}
        self._cli_lock = threading.Lock()

    def start(self):
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
            from rclpy.qos import DurabilityPolicy, QoSProfile
        except Exception as e:
            self.reason = f"ROS 없음 (rclpy import 실패: {type(e).__name__})"
            return
        try:
            try:
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 는 uvicorn 이 받게
            except (ImportError, TypeError):
                rclpy.init()
            node = rclpy.create_node("jax_dashboard")
            node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
            node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
            node.create_subscription(JointState, "/servo/state", self._on_servo, 10)
            node.create_subscription(JointState, HOMED_STATES, self._on_homed, 10)
            node.create_subscription(PoseStamped, "/object_pose", self._on_pose, 10)
            # homing_guard 는 depth 1, transient local 로 발행 -> 늦게 켠 대시보드도 마지막 상태를 받는다
            node.create_subscription(String, HOMING_TOPIC, self._on_homing,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        except Exception as e:
            self.reason = f"ROS 초기화 실패: {e}"
            return
        self._node = node
        self._Trigger = Trigger
        self._JointState = JointState
        self.available = True
        self.reason = ""
        threading.Thread(target=self._spin, args=(rclpy, node), daemon=True).start()

    def _spin(self, rclpy, node):
        try:
            rclpy.spin(node)
        except Exception as e:
            self.available = False
            self.reason = f"ROS spin 멈춤: {e}"

    def stop(self):
        if self._node is None:
            return
        try:
            import rclpy
            self._node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass

    def _on_diag(self, msg):
        now = time.time()
        with self.lock:
            for s in msg.status:
                if not s.name.startswith("ak45/"):
                    continue
                self.diag[s.name] = {
                    "level": int.from_bytes(s.level, "little") if isinstance(s.level, bytes) else int(s.level),
                    "message": s.message, "hardware_id": s.hardware_id,
                    "values": {kv.key: kv.value for kv in s.values}, "t": now}
            self.diag_t = now

    def _on_joints(self, msg):
        now = time.time()
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joints[n] = {"deg": math.degrees(p), "t": now}
            self.joint_t = now

    def _on_homed(self, msg):
        now = time.time()
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joints_homed[n] = {"deg": math.degrees(p), "t": now}

    def publish_joint_command(self, names, positions):
        """/arm_control/joint_command 한 번. 검사는 서버가 했다(이름·범위·유한값). {ok, msg | error}."""
        if not self.available or self._node is None:
            return {"ok": False, "error": f"{self.reason or 'ROS 없음'} — 명령을 못 보냄"}
        with self._cli_lock:
            if self._cmd_pub is None:
                self._cmd_pub = self._node.create_publisher(self._JointState, JOINT_COMMAND, 10)
        subs = self._node.count_subscribers(JOINT_COMMAND)
        if subs < 1:
            return {"ok": False, "error": f"{JOINT_COMMAND} 구독자 없음 (arm_control_node 꺼짐)"}
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [str(n) for n in names]
        msg.position = [float(p) for p in positions]
        self._cmd_pub.publish(msg)
        return {"ok": True, "msg": f"{JOINT_COMMAND}: {len(names)}개 관절 보냄 (구독자 {subs})"}

    def _count_subs(self):
        try:
            return self._node.count_subscribers(JOINT_COMMAND) if self._node is not None else 0
        except Exception:
            return 0

    def _on_servo(self, msg):
        now = time.time()
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.servos[n] = {"deg": math.degrees(p), "t": now}
            self.servo_t = now

    def _on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        # vision_node 는 z 축 회전만 넣는다(yaw). 일반 쿼터니언에서도 z 축 yaw 를 뽑는다.
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        st = msg.header.stamp
        with self.lock:
            self.obj = {"x": p.x, "y": p.y, "z": p.z, "yaw_deg": yaw, "frame_id": msg.header.frame_id,
                        "stamp": st.sec + st.nanosec * 1e-9, "t": time.time()}

    def _on_homing(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.homing = d
            self.homing_t = time.time()

    def homing_joints(self):
        """guard 가 알려 준 관절 이름(없으면 빈 목록)."""
        with self.lock:
            return list(((self.homing or {}).get("joints") or {}).keys())

    def call_trigger(self, service, timeout=2.0):
        """std_srvs/Trigger 하나 부르기. {ok, msg | error}. spin 은 _spin 스레드가 한다."""
        if not self.available or self._node is None:
            return {"ok": False, "error": f"{self.reason or 'ROS 없음'} — 원점 서비스를 못 부름"}
        with self._cli_lock:
            cli = self._clients.get(service)
            if cli is None:
                cli = self._clients[service] = self._node.create_client(self._Trigger, service)
        t0 = time.time()
        while not cli.service_is_ready():
            if time.time() - t0 > min(1.0, timeout):
                return {"ok": False, "error": f"{service} 없음: homing_guard 가 안 켜짐 (arm_homed.launch.py 를 켜세요)"}
            time.sleep(0.05)
        done = threading.Event()
        fut = cli.call_async(self._Trigger.Request())
        fut.add_done_callback(lambda f: done.set())
        if not done.wait(max(0.1, timeout - (time.time() - t0))):
            fut.cancel()
            return {"ok": False, "error": f"{service}: {timeout:g}초 안에 답 없음"}
        try:
            r = fut.result()
        except Exception as e:
            return {"ok": False, "error": f"{service} 실패: {e}"}
        if r is None:
            return {"ok": False, "error": f"{service}: 답 없음"}
        return {"ok": bool(r.success), "msg": r.message} if r.success else {"ok": False, "error": r.message}

    def _homing_snapshot(self, now):
        if not self.available:
            return {"available": False, "reason": self.reason}
        if self.homing is None:
            return {"available": True, "seen": False, "joints": []}
        joints = []
        for name, j in (self.homing.get("joints") or {}).items():
            joints.append({
                "joint": name, "motor": j.get("motor"), "homed": bool(j.get("homed")), "reason": j.get("reason"),
                "home_sim_deg": j.get("home_sim_deg"), "offset_deg": j.get("offset_deg"),
                "session_zero_deg": j.get("session_zero_deg"), "calibration_zero_deg": j.get("calibration_zero_deg"),
                "raw_deg": j.get("raw_deg"), "feedback_age_s": j.get("feedback_age_s"),
                "feedback_lost": j.get("feedback_lost"), "command_active": j.get("command_active")})
        return {"available": True, "seen": True, "all_homed": bool(self.homing.get("all_homed")),
                "age": now - self.homing_t, "stamp": self.homing.get("stamp"), "joints": joints}

    def snapshot(self):
        now = time.time()
        with self.lock:
            motors = []
            bus = None
            for name, d in sorted(self.diag.items()):
                row = dict(d, name=name, age=now - d["t"])
                if name == "ak45/can_bus":
                    bus = row
                else:
                    j = self.joints.get(name.split("/", 1)[1])
                    row["joint_deg"] = j["deg"] if j else None
                    row["joint_age"] = (now - j["t"]) if j else None
                    motors.append(row)
            return {
                "available": self.available, "reason": self.reason,
                "motors": motors, "bus": bus,
                "diag_age": (now - self.diag_t) if self.diag_t else None,
                "joint_age": (now - self.joint_t) if self.joint_t else None,
                "homing": self._homing_snapshot(now),
                "joints": {n: {"deg": d["deg"], "age": now - d["t"]} for n, d in self.joints.items()},
                "servos": {n: {"deg": d["deg"], "age": now - d["t"]} for n, d in self.servos.items()},
                "joints_homed": {n: {"deg": d["deg"], "age": now - d["t"]} for n, d in self.joints_homed.items()},
                "cmd_subs": self._count_subs(),
                "servo_age": (now - self.servo_t) if self.servo_t else None,
                "object_pose": dict(self.obj, age=now - self.obj["t"]) if self.obj else None,
            }
