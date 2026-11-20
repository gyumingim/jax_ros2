"""읽기 전용 ROS 구독: /diagnostics (ak45_node 의 ak45/<joint>, ak45/can_bus), /joint_states.

rclpy 가 없으면(Windows, 또는 ROS 를 source 하지 않고 서버를 켬) available=False 로 두고 아무것도 안 한다.
명령 토픽은 발행하지 않는다(이번 단계는 읽기만).
"""
import math
import threading
import time


class RosMonitor:
    def __init__(self):
        self.available = False
        self.reason = "시작 전"
        self.lock = threading.Lock()
        self.diag = {}        # name -> {level, message, hardware_id, values{}, t}
        self.joints = {}      # name -> {deg, t}
        self.joint_t = None
        self.diag_t = None
        self._node = None

    def start(self):
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from sensor_msgs.msg import JointState
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
        except Exception as e:
            self.reason = f"ROS 초기화 실패: {e}"
            return
        self._node = node
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
                # 나중 단계(homing 브랜치의 원점 guard 노드): homing 상태 토픽을 여기서 구독해 넣는다
                "homing": None,
            }
