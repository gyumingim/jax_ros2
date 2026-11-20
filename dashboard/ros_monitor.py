"""ROS 구독: /diagnostics (ak45_node 의 ak45/<joint>, ak45/can_bus), /joint_states,
/arm_control/homing (homing_guard 의 원점 상태 JSON, transient local).

rclpy 가 없으면(Windows, 또는 ROS 를 source 하지 않고 서버를 켬) available=False 로 두고 아무것도 안 한다.
명령 토픽은 발행하지 않는다. 부르는 것은 homing_guard 의 std_srvs/Trigger 서비스
(/arm_control/home_all, home_<관절>, release_all, release_<관절>)뿐이다.
"""
import json
import math
import threading
import time


HOMING_TOPIC = "/arm_control/homing"
HOMING_SRV = "/arm_control/"   # + home_all, home_<관절>, release_all, release_<관절>


class RosMonitor:
    def __init__(self):
        self.available = False
        self.reason = "시작 전"
        self.lock = threading.Lock()
        self.diag = {}        # name -> {level, message, hardware_id, values{}, t}
        self.joints = {}      # name -> {deg, t}
        self.joint_t = None
        self.diag_t = None
        self.homing = None    # homing_guard 의 마지막 JSON (homing.Homing.status + stamp)
        self.homing_t = None  # 받은 시각(이 PC time.time)
        self._node = None
        self._Trigger = None
        self._clients = {}
        self._cli_lock = threading.Lock()

    def start(self):
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
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
            # homing_guard 는 depth 1, transient local 로 발행 -> 늦게 켠 대시보드도 마지막 상태를 받는다
            node.create_subscription(String, HOMING_TOPIC, self._on_homing,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        except Exception as e:
            self.reason = f"ROS 초기화 실패: {e}"
            return
        self._node = node
        self._Trigger = Trigger
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
            }
