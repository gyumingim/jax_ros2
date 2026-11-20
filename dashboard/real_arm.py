"""실물 팔을 ROS 없이: 이 프로세스가 can0 을 직접 열고(python-can socketcan) AK45 4개와 STM32 서보 노드를 다룬다.

대시보드의 '실물(직접 CAN)' 모드가 이것을 쓴다. 화면 쪽 인터페이스는 virtual_arm.VirtualArm 과 같다
(snapshot / send_joint_command / call_trigger / play_trajectory / stop_trajectory) — 같은 화면이 그대로 돈다.

    ak45_ros2 의 ak45_node + stm_bridge_node + arm_control_node + homing_guard 가 하던 일을 한 프로세스에서:

스레드 2개 (robot_arm v2 robot_v2.py class AK 와 같은 구조, 프레임만 CAN):
  rx       bus.recv → AK45 피드백 0x29(50 Hz 자동 업로드) 파싱, CAN 에러 프레임(bus-off 등) 집계,
           다른 프로그램이 같은 모터에 보내는 명령 프레임 감지.
  control  50 Hz. AK45 관절마다: 목표 → 사다리꼴 램프(속도 max_speed, 가속 max_acc, 한 스텝 max_step 이하)
           → carrot(명령을 측정 위치 ± carrot_deg 안으로) → 위치 프레임(모드 4). 움직이는 동안 50 Hz, 서 있으면 hold_hz(20).
           서보: arm_control.motion.Mover 로 램프 → servo_frame(StdId 5), 바뀔 때 + servo_resend_hz 로 재전송.

안전 규칙 (괄호 = 기본값, DEFAULTS):
  - 첫 명령 전엔 AK45 에 아무 프레임도 안 보낸다. 피드백 없는(또는 끊긴) 모터에는 램프를 시작하지 않는다.
  - 원점(arm_control.homing.Homing 그대로): 세션마다 원점을 다시 잡는다. 원점 없는 AK45 관절 명령은 거절.
    전원 재투입(긴 끊김)·위치 튐·힘 빼기·쉬게 하기·고장 → 원점 지움.
  - 피드백 짧은 끊김(≤ gap_keep_s 5 s): 마지막 명령 자리를 계속 보낸다(홀드 유지, 램프는 멈춤). 돌아왔을 때
    실제가 명령과 2° 넘게 다르면 지금 자리에서 다시 램프. 긴 끊김(> 5 s): 그 모터 송신 멈춤 + 원점 지움.
    다시 들어오면 지금 측정 자리에서 홀드(옛 목표로 가지 않음).
  - 위치 튐(피드백 두 개 사이 > jump_deg 60°): 지금 자리로 다시 홀드 + 원점 지움(원점 의심).
  - 막힘(명령 − 실제 > stall_deg 12° 가 stall_s 2 s): 그 관절 힘 뺌(프레임 중단 + 브레이크 0 A) + 원점 지움.
  - 온도: temp_warn 60 ℃ 경고, temp_stop 75 ℃ 그 관절 힘 뺌. 에러 코드 ≠ 0: 그 관절 힘 뺌.
    (이 모터는 무부하에서 53 ℃ 까지 오른 기록이 있다 — ak45_can/docs/FINDINGS.md)
  - |모터 명령| > max_command_deg(360) 거절.
  - STOP(hold_all): 모든 AK45 를 지금 측정 자리에 홀드, 서보 램프 멈춤, 궤적 재생 멈춤. 팔이 안 떨어진다.
  - 힘 빼기(limp): AK45 프레임 중단 + 브레이크(모드 2, 0 A) 3번. 팔이 떨어진다. 서보는 펌웨어에 '끄기'가 없어 그대로.
  - 같은 can0 을 쓰는 ak45_node 가 떠 있으면 시작 거부: ak45_can 과 같은 잠금 파일 /tmp/ak45_ctrl.lock 을 flock.
  - close(): 기본은 브레이크 0 A (프로세스가 끝나면 어차피 홀드 프레임이 끊겨 힘이 빠진다 — 숨기지 않고 명시).

프레임 (ak45_can/src/ak45_36_socketcan_control.cpp, CubeMars 매뉴얼 5.1/5.2, team-jax/CAN_STM):
  위치 = 확장 ID (4 << 8) | id, int32 BE deg×10000  (arm_control.protocol.ak45_pos_frame)
  브레이크 = 확장 ID (2 << 8) | id, int32 BE A×1000  (ak45_set_current_brake 와 같은 바이트)
  피드백 = 확장 ID (0x29 << 8) | id: pos int16×0.1°, spd int16×10 ERPM, cur int16×0.01 A, temp int8, err uint8
  서보 = 표준 ID 5, [motor_id][angle×10 H][L]  (stm_bridge.protocol.servo_frame). STM32 쪽은 피드백 없음.
"""
import math
import os
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO / "arm_control", REPO / "stm_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from arm_control import calibration as _cal   # noqa: E402  (rclpy 없음)
from arm_control.homing import Homing, load_home_poses   # noqa: E402
from arm_control.motion import Mover   # noqa: E402
from arm_control.protocol import AK45_ERRORS, AK45_FEEDBACK, ak45_pos_frame, parse_ak45_feedback   # noqa: E402
from arm_control.watch import TrackingWatch   # noqa: E402
from stm_bridge.protocol import SERVO_STDID, Frame, servo_frame   # noqa: E402

LOCK_PATH = "/tmp/ak45_ctrl.lock"     # ak45_can LOCK_FILE_PATH 와 같은 파일
AK45_MODE_BRAKE = 2

DEFAULTS = {
    "max_speed": 30.0,        # °/s  AK45 램프 속도 상한 (서보 램프도 같은 값)
    "max_acc": 90.0,          # °/s² 사다리꼴 가속 (robot_arm AK_ACC)
    "max_step": 3.0,          # °    한 제어 스텝 최대 이동 (robot_arm)
    "carrot": 30.0,           # °    명령을 측정 위치 ± 이 안으로 (robot_arm 45 → 보수적으로 30)
    "ctrl_hz": 50.0,          # Hz   제어 루프 / 움직이는 동안 송신
    "hold_hz": 20.0,          # Hz   서 있을 때 홀드 재송신 (robot_arm 20 Hz)
    "fresh_s": 0.25,          # s    피드백이 이보다 오래되면 '끊김' (50 Hz 업로드 기준 12 개)
    "gap_keep_s": 5.0,        # s    이 안의 끊김은 홀드 유지, 넘으면 멈춤 + 원점 지움 (robot_arm AK_GAP_KEEP_S)
    "resume_ramp_deg": 2.0,   # °    끊겼다 돌아올 때 이보다 벌어졌으면 지금 자리에서 다시 램프
    "jump_deg": 60.0,         # °    피드백 튐 = 원점 의심
    "stall_deg": 12.0,        # °    막힘: 명령 − 실제
    "stall_s": 2.0,           # s    막힘이 이만큼 이어지면 힘 뺌
    "temp_warn": 60.0,        # ℃
    "temp_stop": 75.0,        # ℃    그 관절 힘 뺌
    "max_command_deg": 360.0,  # °   ak45_node.yaml 과 같음
    "servo_resend_hz": 1.0,   # Hz   서보 마지막 값 재전송 (0 = 안 함). 펌웨어는 마지막 펄스를 유지한다
    "calib_step_max": 10.0,   # °    보정에서 한 번에 움직일 수 있는 양
}

# linux/can/error.h
CAN_ERR_CRTL, CAN_ERR_PROT, CAN_ERR_ACK, CAN_ERR_BUSOFF, CAN_ERR_RESTARTED = 0x04, 0x08, 0x20, 0x40, 0x100
CRTL_WARN, CRTL_PASSIVE, CRTL_ACTIVE = 0x04 | 0x08, 0x10 | 0x20, 0x40
PROT_FORM, PROT_STUFF = 0x02, 0x04


def ak45_brake_frame(controller_id, amps=0.0):
    """모드 2 (current brake). ak45_set_current_brake 와 같은 바이트: int32 BE, A × 1000."""
    amps = min(max(float(amps), 0.0), 5.0)
    return Frame((AK45_MODE_BRAKE << 8) | controller_id, True, struct.pack(">i", int(amps * 1000.0)))


def take_lock(path=LOCK_PATH):
    """ak45_can 과 같은 flock. 다른 프로세스(ak45_node 등)가 잡고 있으면 RuntimeError. 반환: fd (또는 None)."""
    if not path:
        return None
    try:
        import fcntl
    except ImportError:       # Windows: 잠금 없음 (가짜 버스 시험용)
        return None
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError(f"다른 프로그램이 이미 AK45 버스를 쓰는 중 ({path} 잠김: ak45_node / arm.launch.py / "
                           "arm_homed.launch.py 가 떠 있나?). 그것을 끈 뒤 다시")
    return fd


class AkMotor:
    def __init__(self, joint):
        self.joint = joint
        self.id = joint.id
        self.name = f"ak45_{joint.id}"
        # 피드백
        self.pos = self.spd = self.cur = None
        self.temp = None
        self.err = 0
        self.fb_t = None            # monotonic
        self.fb_n = 0
        # 명령
        self.target = None          # 최종 목표 (raw 모터 °). None = 프레임 안 보냄
        self.cmd = None             # 램프된 지금 명령
        self.vel = 0.0
        self.sent_t = 0.0
        self.sent_n = 0
        self.last_sent = None
        self.stall_t = None
        self.fault = None           # 힘 뺀 이유 (막힘·온도·에러). 원점을 다시 잡을 때 지운다
        self.note = None            # 원점 의심 등 알림
        self.temp_warn_t = 0.0
        self.stale_logged = False

    def fresh(self, now, fresh_s):
        return self.fb_t is not None and now - self.fb_t < fresh_s

    def active(self):
        return self.cmd is not None


class RealArm:
    """실물 AK45 4개 + 서보(STM32) 직접 CAN 제어. start() 로 버스를 열고 close() 로 닫는다."""

    def __init__(self, calibration_file=None, home_file=None, interface="socketcan", channel="can0",
                 lock_path=LOCK_PATH, params=None, log=None, log_file=None):
        self.p = dict(DEFAULTS)
        for k, v in (params or {}).items():
            if k in self.p and v is not None:
                self.p[k] = float(v)
        self.interface, self.channel, self.lock_path = interface, channel, lock_path
        self.log_cb = log or (lambda text: None)
        self.log_file = log_file
        self.lines = deque(maxlen=300)
        self.cal = _cal.load(calibration_file)
        self.joints = {j.joint: j for j in self.cal.joints}
        self.home_poses = load_home_poses(home_file)
        self.ak = {f"ak45_{j.id}": AkMotor(j) for j in self.cal.joints if j.motor == "ak45"}
        self.ak_by_id = {m.id: m for m in self.ak.values()}
        self.homing = Homing({n: m.joint for n, m in self.ak.items()}, self.home_poses,
                             gap_s=self.p["gap_keep_s"], jump_deg=self.p["jump_deg"], max_age_s=0.5)
        self.servo_mover = Mover(self.p["max_speed"], self.p["ctrl_hz"])
        self.servo_sent = {}        # 서보 이름 -> (모터 °, monotonic)
        self.servo_resend_t = 0.0
        self.lock = threading.RLock()
        self.bus = None
        self.lock_fd = None
        self.running = False
        self.started = None
        self.threads = []
        self.busst = {"state": "ERROR-ACTIVE", "err_frames": 0, "bit_stuff_errors": 0, "form_errors": 0,
                      "ack_errors": 0, "busoff_count": 0, "restart_count": 0, "send_errors": 0,
                      "last_send_error": "", "send_error_t": None, "rx_frames": 0}
        self.foreign = {}           # "AK45 id 2 모드 4" -> (count, monotonic)
        self.calib_dirty = False
        self.play = {"running": False}
        self._play_stop = threading.Event()
        self._play_bad = None

    # ------------------------------------------------------------------ 시작 / 끝
    def start(self):
        import can
        self._can = can
        self.lock_fd = take_lock(self.lock_path)
        try:
            if self.interface == "socketcan":
                self.bus = can.Bus(interface="socketcan", channel=self.channel)
            else:
                self.bus = can.Bus(interface=self.interface, channel=self.channel)
        except Exception:
            self._unlock()
            raise
        self.running = True
        self.started = time.time()
        self.threads = [threading.Thread(target=self._rx_loop, daemon=True, name="ak45-rx"),
                        threading.Thread(target=self._ctrl_loop, daemon=True, name="ak45-ctrl")]
        for t in self.threads:
            t.start()
        unver = [j.joint for j in self.cal.joints if not j.verified]
        self._log(f"직접 CAN 시작: {self.interface} {self.channel}, AK45 {', '.join(self.ak)} + 서보 "
                  f"{sum(1 for j in self.cal.joints if j.motor == 'servo')}개. 첫 명령 전엔 아무 프레임도 안 보냄. "
                  f"속도 {self.p['max_speed']:g} °/s, carrot {self.p['carrot']:g}°, 온도 {self.p['temp_warn']:g}/{self.p['temp_stop']:g} ℃. "
                  f"모든 AK45 원점 없음(세션마다 다시). calibration {self.cal.path}, 미확인 {', '.join(unver) or '없음'}")
        return self

    def close(self, brake=True):
        """brake=True: AK45 에 브레이크 0 A 를 보내고 닫는다(팔이 떨어진다). 닫으면 홀드 프레임이 끊겨 어차피 힘이 빠진다."""
        self._play_stop.set()
        if not self.running:
            return
        with self.lock:
            had = [m.name for m in self.ak.values() if m.target is not None]
            for m in self.ak.values():
                m.target = m.cmd = None
        if brake and self.bus is not None:
            for _ in range(3):
                for m in self.ak.values():
                    self._send(ak45_brake_frame(m.id))
                time.sleep(0.01)
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)
        try:
            self.bus.shutdown()
        except Exception:
            pass
        self.bus = None
        self._unlock()
        self._log("직접 CAN 닫음" + (f": 브레이크 0 A 보냄 (홀드하던 {', '.join(had)} 힘 빠짐)" if brake else ""))

    def _unlock(self):
        if self.lock_fd is not None:
            try:
                import fcntl
                fcntl.flock(self.lock_fd, fcntl.LOCK_UN)
            except Exception:
                pass
            os.close(self.lock_fd)
            self.lock_fd = None

    # ------------------------------------------------------------------ 기록
    def _log(self, text):
        line = time.strftime("%H:%M:%S ") + text
        self.lines.append(line)
        if self.log_file:
            try:
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass
        try:
            self.log_cb(text)
        except Exception:
            pass

    # ------------------------------------------------------------------ 송신
    def _send(self, fr):
        bus = self.bus
        if bus is None:
            return False
        try:
            bus.send(self._can.Message(arbitration_id=fr.arb_id, is_extended_id=fr.extended, data=fr.data), timeout=0.01)
            return True
        except Exception as e:
            b = self.busst
            b["send_errors"] += 1
            b["last_send_error"] = f"{type(e).__name__}: {e}"[:160]
            if b["send_error_t"] is None or time.monotonic() - b["send_error_t"] > 5.0:
                self._log(f"CAN 송신 실패: {b['last_send_error']} (can0 DOWN·bus-off·버퍼 가득?)")
            b["send_error_t"] = time.monotonic()
            return False

    # ------------------------------------------------------------------ rx
    def _rx_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(0.02)
            except Exception as e:
                if self.running:
                    self._log(f"CAN 수신 오류: {type(e).__name__}: {e}")
                    time.sleep(0.2)
                continue
            if msg is None:
                continue
            now = time.monotonic()
            self.busst["rx_frames"] += 1
            if msg.is_error_frame:
                self._on_error_frame(msg)
                continue
            st = parse_ak45_feedback(msg.arbitration_id, msg.is_extended_id, bytes(msg.data))
            if st is not None:
                self._on_feedback(st, now)
                continue
            self._on_foreign(msg, now)

    def _on_error_frame(self, msg):
        cid, d = msg.arbitration_id, bytes(msg.data) + bytes(8)
        b = self.busst
        with self.lock:
            b["err_frames"] += 1
            if cid & CAN_ERR_BUSOFF:
                b["state"] = "BUS-OFF"
                b["busoff_count"] += 1
                self._log("CAN bus-off: 송신 불가. 배선·종단 저항 확인, 복구는 "
                          "sudo ip link set can0 type can bitrate 1000000 restart-ms 100 (또는 down/up)")
            elif cid & CAN_ERR_RESTARTED:
                b["state"] = "ERROR-ACTIVE"
                b["restart_count"] += 1
            elif cid & CAN_ERR_CRTL:
                c = d[1]
                if c & CRTL_PASSIVE:
                    b["state"] = "ERROR-PASSIVE"
                elif c & CRTL_WARN:
                    b["state"] = "ERROR-WARNING"
                elif c & CRTL_ACTIVE:
                    b["state"] = "ERROR-ACTIVE"
            if cid & CAN_ERR_PROT:
                if d[2] & PROT_STUFF:
                    b["bit_stuff_errors"] += 1
                if d[2] & PROT_FORM:
                    b["form_errors"] += 1
            if cid & CAN_ERR_ACK:
                b["ack_errors"] += 1

    def _on_foreign(self, msg, now):
        """우리가 보내지 않은 명령 프레임(receive_own_messages 꺼짐) = 다른 프로그램이 같은 모터에 명령 중."""
        aid = msg.arbitration_id
        if msg.is_extended_id and (aid & 0xFF) in self.ak_by_id and (aid >> 8) <= 6:
            key = f"AK45 ID {aid & 0xFF} 모드 {aid >> 8}"
        elif not msg.is_extended_id and aid == SERVO_STDID:
            key = "서보 StdId 5"
        else:
            return
        with self.lock:
            n, t = self.foreign.get(key, (0, 0.0))
            if now - t > 10.0:
                self._log(f"⚠ 다른 프로그램이 버스에 {key} 명령을 보내는 중 — ak45_node·stm_bridge_node·cansend 등을 끄세요")
            self.foreign[key] = (n + 1, now)

    def _on_feedback(self, st, now):
        m = self.ak_by_id.get(st.controller_id)
        if m is None:
            return
        P = self.p
        with self.lock:
            was_fresh = m.fresh(now, P["fresh_s"])
            old, gap = m.pos, (None if m.fb_t is None else now - m.fb_t)
            m.pos, m.spd, m.cur, m.temp, m.err = st.position_deg, st.speed_erpm, st.current_a, st.temperature_c, st.error
            m.fb_t = now
            m.fb_n += 1
            event, text = self.homing.feedback(m.name, m.pos, now)   # 긴 끊김·튐 → 원점 지움
            if text:
                self._log(text)
            jump = old is not None and abs(m.pos - old) > P["jump_deg"]
            if m.target is not None:
                if jump:
                    # robot_arm: 응답은 이어지는데 위치만 튐 = 밀려 넘어감 또는 전원 재투입 → 지금 자리 다시 홀드, 원점 의심
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
                    m.note = f"위치 튐 {old:+.1f}° → {m.pos:+.1f}°: 지금 자리 홀드, 원점 다시 잡기"
                    self._log(f"{m.name} ⚠ {m.note}")
                elif not was_fresh and gap is not None:
                    if m.cmd is not None and gap <= P["gap_keep_s"]:
                        off = m.pos - m.cmd
                        if abs(off) > P["resume_ramp_deg"]:
                            m.cmd, m.vel = m.pos, 0.0
                        self._log(f"{m.name} 피드백 {gap:.2f}s 끊겼다 돌아옴 — 그동안 홀드 계속 보냄, 지금 {m.pos:+.1f}°"
                                  + (f" (명령과 {off:+.1f}° → 지금 자리에서 다시 램프)" if abs(off) > P["resume_ramp_deg"] else ""))
                    elif m.cmd is None:
                        # 긴 끊김(> gap_keep_s) 뒤: 옛 목표로 가지 않고 지금 자리 홀드. 원점은 지워져 있음
                        m.target = m.cmd = m.pos
                        m.vel = 0.0
                        m.note = f"피드백 {gap:.1f}s 끊김 뒤 돌아옴(전원 재투입?): 지금 자리 홀드, 원점 다시 잡기"
                        self._log(f"{m.name} ⚠ {m.note}")
                    m.stale_logged = False
            # 온도 / 에러 코드
            if m.temp is not None and m.temp >= P["temp_stop"]:
                if m.target is not None or m.fault is None:
                    self._fault(m, f"온도 {m.temp} ℃ ≥ {P['temp_stop']:g} ℃")
            elif m.temp is not None and m.temp >= P["temp_warn"] and now - m.temp_warn_t > 30.0:
                m.temp_warn_t = now
                self._log(f"{m.name} ⚠ 온도 {m.temp} ℃ ≥ {P['temp_warn']:g} ℃ — 오래 홀드·막힘 확인")
            if m.err and (m.target is not None or m.fault is None):
                self._fault(m, f"에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})")

    # ------------------------------------------------------------------ 관절 멈춤 / 원점 지움
    def _unhome(self, name, reason):
        h = self.homing.m[name]
        was = h.homed
        h.homed, h.offset_deg, h.reason = False, 0.0, reason
        return was

    def _limp_motor(self, m, reason, brake=True):
        had = m.target is not None
        m.target = m.cmd = None
        m.vel, m.stall_t = 0.0, None
        if brake and had:
            for _ in range(3):
                self._send(ak45_brake_frame(m.id))
        self._unhome(m.name, reason + " — 원점 다시 잡기")
        return had

    def _fault(self, m, reason):
        m.fault = reason
        had = self._limp_motor(m, f"힘 뺌: {reason}")
        self._log(f"{m.name} ✖ {reason} — {'힘 뺌(브레이크 0 A, 팔이 내려갈 수 있음)' if had else '명령 금지'}")
        if self.play.get("running"):
            self._play_bad = f"{m.name}: {reason}"

    # ------------------------------------------------------------------ control 50 Hz
    def _ctrl_loop(self):
        dt0 = 1.0 / self.p["ctrl_hz"]
        nxt = time.monotonic()
        last = nxt
        while self.running:
            nxt += dt0
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            try:
                with self.lock:
                    self._tick(now, dt)
            except Exception as e:
                self._log(f"제어 루프 오류: {type(e).__name__}: {e}")
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:
                nxt = time.monotonic()

    def _tick(self, now, dt):
        P = self.p
        for text in self.homing.check_lost(now):
            self._log(text)
        hold_dt = 1.0 / P["hold_hz"]
        for m in self.ak.values():
            send = None
            fresh = m.fresh(now, P["fresh_s"])
            if m.target is not None and fresh:
                if m.cmd is None:
                    m.cmd, m.vel = m.pos, 0.0
                prev = m.cmd
                d = m.target - m.cmd
                sgn = 1.0 if d >= 0 else -1.0
                acc, vmax = P["max_acc"], P["max_speed"]
                if abs(d) > m.vel * m.vel / (2.0 * acc):
                    m.vel += sgn * acc * dt
                else:
                    m.vel -= sgn * acc * dt
                    if sgn * m.vel < 0:
                        m.vel = 0.0
                m.vel = max(-vmax, min(vmax, m.vel))
                step = max(-P["max_step"], min(P["max_step"], m.vel * dt))
                if abs(step) >= abs(d) or abs(d) < 0.02:
                    m.cmd, m.vel = m.target, 0.0
                else:
                    m.cmd += step
                # carrot: 명령은 측정 ± carrot 안. 평소엔 안 걸리는 최후 방어선(런어웨이 차단)
                m.cmd = max(m.pos - P["carrot"], min(m.pos + P["carrot"], m.cmd))
                if abs(m.cmd - m.pos) > P["stall_deg"]:
                    m.stall_t = m.stall_t or now
                    if now - m.stall_t > P["stall_s"]:
                        self._fault(m, f"막힘: 명령 {m.cmd:+.1f}° · 실제 {m.pos:+.1f}° 가 {P['stall_s']:g}s 넘게 벌어짐 (물체·바닥을 누름?)")
                        continue
                else:
                    m.stall_t = None
                moving = m.cmd != prev or m.cmd != m.target
                if moving or now - m.sent_t >= hold_dt - 1e-3:
                    send = m.cmd
            elif m.target is not None and m.cmd is not None and m.fb_t is not None and now - m.fb_t <= P["gap_keep_s"]:
                # 짧은 끊김: 마지막 명령 자리를 계속 보냄(램프 멈춤). robot_arm 2026-10-21 실기: 안 보내면 팔이 떨어짐
                m.vel, m.stall_t = 0.0, None
                if not m.stale_logged:
                    m.stale_logged = True
                    self._log(f"{m.name} ⚠ 피드백 끊김 — 마지막 명령 {m.cmd:+.1f}° 홀드 계속 보냄 (최대 {P['gap_keep_s']:g}s)")
                if now - m.sent_t >= hold_dt - 1e-3:
                    send = m.cmd
            else:
                if m.cmd is not None:
                    age = None if m.fb_t is None else now - m.fb_t
                    self._log(f"{m.name} ✖ 피드백 {age if age is not None else 0:.1f}s 끊김 — 홀드 프레임 멈춤(전원 꺼짐?), 원점 지움. "
                              "다시 들어오면 지금 자리에서 홀드")
                    self._unhome(m.name, f"피드백 {P['gap_keep_s']:g}s 넘게 끊김 — 원점 다시 잡기")
                m.cmd, m.vel, m.stall_t = None, 0.0, None
            if send is not None:
                if abs(send) > P["max_command_deg"]:
                    self._fault(m, f"명령 {send:.1f}° 가 상한 ±{P['max_command_deg']:g}° 밖")
                    continue
                if self._send(ak45_pos_frame(m.id, send)):
                    m.sent_t, m.last_sent = now, send
                    m.sent_n += 1
        # 서보
        for n in self.servo_mover.needs_start():
            t = self.servo_mover.target[n]
            self.servo_mover.start(n, t)       # 위치를 몰라 첫 명령은 바로 그 각(arm_control_node 와 같음)
            self._servo_send(self.joints[n], t, now)
        for n, deg in self.servo_mover.step().items():
            self._servo_send(self.joints[n], deg, now)
        rh = P["servo_resend_hz"]
        if rh > 0 and self.servo_sent and now - self.servo_resend_t >= 1.0 / rh:
            self.servo_resend_t = now
            for j in self.cal.joints:
                if j.motor == "servo" and j.name in self.servo_sent:
                    self._send(servo_frame(j.id, self.servo_sent[j.name][0]))

    def _servo_send(self, j, deg, now):
        deg = min(max(deg, 0.0), 180.0)
        if self._send(servo_frame(j.id, deg)):
            self.servo_sent[j.name] = (deg, now)

    # ------------------------------------------------------------------ 명령 (대시보드가 부름)
    def _ready(self):
        return None if self.running else "팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결"

    def send_joint_command(self, names, positions):
        """sim 관절명, rad. arm_control_node + homing_guard 와 같은 변환: calibration(클립) → 원점 오프셋 → 램프."""
        return self.joint_command(list(names), [float(p) for p in positions])

    def joint_command(self, names, positions, _source="화면"):
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if len(names) != len(positions):
            return {"ok": False, "error": "name / position 길이가 다름"}
        for n, p in zip(names, positions):
            if n not in self.joints:
                return {"ok": False, "error": f"모르는 관절 {n}"}
            if not math.isfinite(p):
                return {"ok": False, "error": f"{n}: 유한값 아님"}
        now = time.monotonic()
        refused, done, clipped = [], [], []
        with self.lock:
            for n, p in zip(names, positions):
                j = self.joints[n]
                mdeg, cl = j.to_motor(math.degrees(p))
                if cl:
                    clipped.append(f"{n} → 모터 {mdeg:.1f}°")
                if j.motor == "servo":
                    self.servo_mover.set_target(n, mdeg)
                    done.append(n)
                    continue
                m = self.ak[f"ak45_{j.id}"]
                raw = self.homing.to_raw(m.name, mdeg)
                if m.fault:
                    refused.append(f"{n}: {m.fault} (원점을 다시 잡으면 풀림)")
                elif raw is None:
                    refused.append(f"{n}: 원점 없음")
                elif not m.fresh(now, self.p["fresh_s"]):
                    refused.append(f"{n}: 피드백 없음/끊김")
                elif abs(raw) > self.p["max_command_deg"]:
                    refused.append(f"{n}: 모터 {raw:.1f}° 가 ±{self.p['max_command_deg']:g}° 밖")
                else:
                    if m.target is None:
                        self._log(f"{m.name} 첫 명령({_source}): {raw:.2f}° (지금 {m.pos:+.2f}°에서 램프)")
                    m.target = raw
                    m.note = None
                    done.append(n)
        if clipped:
            text = "calibration 범위로 잘림: " + ", ".join(clipped)
            if text != getattr(self, "_clip_text", None) or now - getattr(self, "_clip_t", 0.0) > 5.0:
                self._clip_text, self._clip_t = text, now
                self._log(text)
        if refused and not done:
            return {"ok": False, "error": "안 보냄: " + "; ".join(refused)}
        msg = f"{len(done)}개 관절 목표 보냄"
        if refused:
            msg += " · 안 보냄: " + "; ".join(refused)
        return {"ok": True, "msg": msg, "refused": refused}

    def hold_all(self, why="STOP"):
        """STOP: 모든 AK45 지금 측정 자리 홀드(힘 있는 것만), 서보 램프 멈춤, 재생 멈춤. 팔이 안 떨어진다."""
        self._play_stop.set()
        now = time.monotonic()
        held, stale = [], []
        with self.lock:
            for m in self.ak.values():
                if m.target is None:
                    continue
                if m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
                    held.append(m.name)
                else:
                    m.target = m.cmd       # 끊긴 모터: 마지막 명령 자리 그대로
                    stale.append(m.name)
            mv = self.servo_mover
            for n in list(mv.target):
                if n in mv.current:
                    mv.target[n] = mv.current[n]
        text = f"{why}: 지금 자리 홀드 {', '.join(held) or '없음 (홀드 중인 AK45 없음)'}" + \
               (f" · 피드백 끊긴 {', '.join(stale)} 는 마지막 명령 자리" if stale else "") + " · 서보 램프 멈춤"
        self._log(text)
        return {"ok": True, "msg": text}

    def limp(self, names=None):
        """힘 빼기: AK45 프레임 중단 + 브레이크 0 A. 팔이 떨어진다. 원점 지움."""
        self._play_stop.set()
        with self.lock:
            ms = [self.ak[n] for n in (names or list(self.ak))]
            had = [m.name for m in ms if self._limp_motor(m, "힘 뺌(사용자)")]
            for m in ms:
                for _ in range(3):
                    self._send(ak45_brake_frame(m.id))
                m.note = None
        text = f"힘 뺌: {', '.join(had) or '홀드 중인 AK45 없었음'} — 브레이크 0 A, 원점 지움. 서보는 마지막 각 유지(펌웨어에 끄기 없음)"
        self._log(text)
        return {"ok": True, "msg": text}

    def stop_all(self):            # 가상 팔과 같은 이름: 서버가 모드를 바꿀 때
        return self.hold_all("모드 바꿈")

    # ------------------------------------------------------------------ 원점 (homing_guard 서비스와 같은 이름)
    def call_trigger(self, service, timeout=2.0):
        prefix = "/arm_control/"
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if not service.startswith(prefix):
            return {"ok": False, "error": f"{service}: 모르는 서비스"}
        act, _, joint = service[len(prefix):].partition("_")
        hm = self.homing
        try:
            if joint == "all":
                names = list(hm.m) if act == "release" else [n for n, h in hm.m.items() if not h.homed]
            else:
                names = [hm.by_joint(joint)]
        except KeyError:
            return {"ok": False, "error": f"{service} 없음"}
        if act == "home":
            return self.home(names)
        if act == "release":
            return self.release(names)
        return {"ok": False, "error": f"{service}: 모르는 서비스"}

    def home(self, names):
        now = time.monotonic()
        texts, ok = [], True
        with self.lock:
            for n in names:
                m = self.ak[n]
                if m.temp is not None and m.temp >= self.p["temp_stop"]:
                    texts.append(f"{m.joint.joint}: 거절, 온도 {m.temp} ℃ (식힌 뒤)")
                    ok = False
                    continue
                if m.err:
                    texts.append(f"{m.joint.joint}: 거절, 에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})")
                    ok = False
                    continue
                good, text = self.homing.home(n, now)
                texts.append(text)
                ok = ok and good
                if good:
                    m.fault = m.note = None
                    m.target = m.cmd = m.pos     # 원점 잡는 순간 지금 자리에서 홀드 시작(guard 와 같음)
                    m.vel = 0.0
        msg = "; ".join(texts) or "원점 없는 관절 없음 (전부 잡힘)"
        self._log("원점: " + msg)
        return {"ok": True, "msg": msg} if ok else {"ok": False, "error": msg}

    def release(self, names):
        """쉬게 하기: 원점을 지우고 명령을 막는다. 팔은 지금 자리에서 계속 홀드(힘을 빼려면 '힘 빼기')."""
        now = time.monotonic()
        with self.lock:
            rel = []
            for n in names:
                m = self.ak[n]
                if self._unhome(n, "쉬게 함: 다시 쓰기 전에 원점"):
                    rel.append(m.joint.joint)
                if m.target is not None and m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
        text = f"쉬게 함(원점 지움): {', '.join(rel) or '원점 잡힌 관절 없었음'}. 지금 자리 홀드는 계속 — 힘을 빼려면 '힘 빼기'"
        self._log(text)
        return {"ok": True, "msg": text}

    # ------------------------------------------------------------------ 보정 (calibrate.py Session 의 m / z / f / k / v / w)
    def calib(self, action, joint, value=None):
        why = self._ready()
        if why and action != "save":
            return {"ok": False, "error": why}
        if action == "save":
            with self.lock:
                self.cal.save()
                self.calib_dirty = False
            self._log(f"보정 저장: {self.cal.path}")
            return {"ok": True, "msg": f"저장함: {self.cal.path}"}
        j = self.joints.get(joint)
        if j is None:
            return {"ok": False, "error": f"모르는 관절 {joint!r}"}
        now = time.monotonic()
        with self.lock:
            m = self.ak.get(f"ak45_{j.id}") if j.motor == "ak45" else None
            if action == "move":       # 모터 각을 지금 자리에서 ±value° (원점 없이도: 상대 이동이라)
                try:
                    dv = float(value)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "움직일 각이 숫자가 아님"}
                if not math.isfinite(dv) or abs(dv) > self.p["calib_step_max"]:
                    return {"ok": False, "error": f"한 번에 ±{self.p['calib_step_max']:g}° 까지"}
                if m is not None:
                    if m.fault:
                        return {"ok": False, "error": f"{m.name}: {m.fault} — 원점을 다시 잡아 풀기"}
                    if not m.fresh(now, self.p["fresh_s"]):
                        return {"ok": False, "error": f"{m.name}: 피드백 없음/끊김 — 안 움직임"}
                    base = m.target if m.target is not None else m.pos
                    new = base + dv
                    if abs(new) > self.p["max_command_deg"]:
                        return {"ok": False, "error": f"모터 {new:.1f}° 가 ±{self.p['max_command_deg']:g}° 밖"}
                    m.target = new
                    text = f"보정 {j.joint}({m.name}): 모터 {base:+.2f}° → {new:+.2f}° (지금 {m.pos:+.2f}°)"
                else:
                    cur = self.servo_mover.current.get(j.joint)
                    base = cur if cur is not None else j.zero_deg
                    new = min(max(base + dv, 0.0), 180.0)
                    self.servo_mover.set_target(j.joint, new)
                    text = f"보정 {j.joint}({j.name}): 서보 {base:.1f}° → {new:.1f}°" + ("" if cur is not None else " (첫 명령: 바로 감)")
                self._log(text)
                return {"ok": True, "msg": text}
            if action == "zero":       # 지금 모터 각 = sim 0
                if m is not None:
                    if not m.fresh(now, self.p["fresh_s"]):
                        return {"ok": False, "error": f"{m.name}: 피드백 없음 — 영점 못 잡음"}
                    old, j.zero_deg = j.zero_deg, round(m.pos, 2)
                    h = self.homing.m[m.name]       # 지금 자리가 sim 0 이 됐으니 세션 원점도 여기(오프셋 0)
                    h.homed, h.offset_deg = True, 0.0
                    h.reason = f"보정 영점(z): 모터 {j.zero_deg:+.2f}° = sim 0"
                else:
                    cur = self.servo_sent.get(j.name)
                    if cur is None:
                        return {"ok": False, "error": f"{j.name}: 아직 보낸 값이 없음 — 먼저 움직여서 sim 0 자세로"}
                    old, j.zero_deg = j.zero_deg, round(cur[0], 1)
                text = f"보정 {j.joint}: zero_deg {old:g} → {j.zero_deg:g} (저장 전)"
            elif action == "flip":
                j.sign = -j.sign
                text = f"보정 {j.joint}: sign → {j.sign:+d} (저장 전)"
                if m is not None and m.target is not None and m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
            elif action == "scale":
                try:
                    k = float(value)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "scale 이 숫자가 아님"}
                if not (math.isfinite(k) and 0.01 <= k <= 100):
                    return {"ok": False, "error": "scale 0.01~100"}
                j.scale = k
                text = f"보정 {j.joint}: scale → {k:g} (저장 전)"
            elif action == "verify":
                j.verified = bool(value)
                text = f"보정 {j.joint}: verified → {j.verified} (저장 전)"
            else:
                return {"ok": False, "error": f"모르는 보정 동작 {action}"}
            self.calib_dirty = True
        self._log(text)
        return {"ok": True, "msg": text}

    # ------------------------------------------------------------------ 궤적 재생 (실물)
    def play_trajectory(self, traj, max_speed, hold=5.0, max_error=15.0):
        """play.play 로 프레임을 joint_command 에 넣고, TrackingWatch 로 AK45 추종을 본다. 벗어나면 그 자리 홀드.
        check_trajectory 는 서버가 먼저 한다(거절·--force 확인창)."""
        from arm_control import play as P
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if self.play.get("running"):
            return {"ok": False, "error": "이미 재생 중"}
        names = traj["joint_names"]
        not_homed = [n for n in names if self.joints[n].motor == "ak45"
                     and not self.homing.m[f"ak45_{self.joints[n].id}"].homed]
        if not_homed:
            return {"ok": False, "error": "원점 없는 AK45: " + ", ".join(not_homed) + " — 각도 제어 탭에서 원점 먼저"}
        speed = min(float(max_speed), self.p["max_speed"])
        period, slow, peak = P.schedule(traj, speed)
        n = len(traj["positions"])
        motors = [(k, self.joints[nm]) for k, nm in enumerate(names) if self.joints[nm].motor == "ak45"]
        tracker = TrackingWatch(max_error)
        self._play_stop.clear()
        self._play_bad = None
        self.play = {"running": True, "frame": 0, "frames": n, "period": period, "slow": slow,
                     "seconds": hold + (n - 1) * period, "started": time.time(), "reason": None, "real": True}
        state = {"k": 0}

        def publish(nm, positions):
            r = self.joint_command(list(nm), [float(p) for p in positions], _source="궤적")
            if not r.get("ok") or r.get("refused"):
                self._play_bad = self._play_bad or ("안 보낸 관절: " + "; ".join(r.get("refused") or [r.get("error", "?")]))
            state["k"] += 1
            self.play["frame"] = max(0, state["k"] - max(1, int(hold * 10)))

        def watch(frame):
            if self._play_bad:
                return self._play_bad
            now = time.monotonic()
            exp, fb = {}, {}
            with self.lock:
                for k, j in motors:
                    nm = f"ak45_{j.id}"
                    m = self.ak[nm]
                    exp[nm] = j.to_motor(math.degrees(frame[k]))[0]
                    fb[nm] = self.homing.from_raw(nm, m.pos) if m.fresh(now, self.p["fresh_s"]) and m.pos is not None else None
            return tracker.check(exp, fb, now)

        def run():
            try:
                reason = P.play(publish, traj, speed, hold, stop=self._play_stop, watch=watch)
            except Exception as e:
                reason = f"{type(e).__name__}: {e}"
            if reason and reason != "stopped":
                self.hold_all(f"재생 멈춤({reason})")
            self.play.update(running=False, reason=reason or "끝까지 재생", ended=time.time())
            self._log(f"실물 궤적 재생 끝: {reason or '끝까지 (마지막 프레임 홀드)'}")

        threading.Thread(target=run, daemon=True, name="play").start()
        self._log(f"실물 궤적 재생 시작: {n} 프레임, {slow:.1f}배 느리게({speed:g} °/s), 약 {self.play['seconds']:.0f}초")
        return {"ok": True, "msg": f"실물 재생: {n} 프레임, {slow:.1f}배 느리게, 약 {self.play['seconds']:.0f}초. 멈춤 = STOP"}

    def stop_trajectory(self):
        self._play_stop.set()
        return {"ok": True, "msg": "재생 멈춤 요청 (팔은 그 자리 홀드)"}

    # ------------------------------------------------------------------ 상태 (RosMonitor.snapshot 과 같은 모양)
    def snapshot(self):
        now_m, now = time.monotonic(), time.time()
        P = self.p
        with self.lock:
            motors, joints, homed_fb = [], {}, {}
            for n, m in self.ak.items():
                age = None if m.fb_t is None else now_m - m.fb_t
                fresh = m.fresh(now_m, P["fresh_s"])
                if m.fault:
                    lv, msg = 2, f"힘 뺌: {m.fault}"
                elif m.fb_t is None:
                    lv, msg = 1, "피드백 수신 없음 (전원·배선·CubeMarsTool 'Send status over CAN' Rate 확인)"
                elif not fresh:
                    lv, msg = 2, f"피드백 끊김 {age:.1f}s" + (" — 마지막 자리 홀드 계속" if m.cmd is not None else "")
                elif m.err:
                    lv, msg = 2, f"에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})"
                elif m.note:
                    lv, msg = 1, m.note
                elif m.temp is not None and m.temp >= P["temp_warn"]:
                    lv, msg = 1, f"온도 높음 {m.temp} ℃"
                else:
                    lv, msg = 0, "정상"
                vals = {"position_deg": "—" if m.pos is None else f"{m.pos:.2f}",
                        "speed_erpm": "—" if m.spd is None else f"{m.spd:.0f}",
                        "current_a": "—" if m.cur is None else f"{m.cur:.2f}",
                        "temperature_c": "—" if m.temp is None else str(m.temp),
                        "error_code": str(m.err), "watchdog_ok": "1" if fresh else "0",
                        "target_deg": "none" if m.target is None else f"{m.target:.2f}",
                        "command_deg": "none" if m.cmd is None else f"{m.cmd:.2f}",
                        "command_active": "1" if m.cmd is not None else "0",
                        "position_error_deg": "none" if m.cmd is None or m.pos is None else f"{m.cmd - m.pos:.2f}",
                        "frames_sent": str(m.sent_n), "feedback_count": str(m.fb_n)}
                motors.append({"name": f"ak45/{n}", "level": lv, "message": msg, "hardware_id": f"ak45_id_0x{m.id:02X}",
                               "values": vals, "t": now, "age": 0.0, "joint_deg": m.pos if fresh else None,
                               "joint_age": age, "joint": m.joint.joint, "fault": m.fault})
                if m.fb_t is not None:
                    joints[n] = {"deg": m.pos, "age": age}
                    d = self.homing.from_raw(n, m.pos)
                    if d is not None:
                        homed_fb[n] = {"deg": d, "age": age}
            b = dict(self.busst)
            blv = 0 if b["state"] == "ERROR-ACTIVE" else (2 if b["state"] == "BUS-OFF" else 1)
            recent_send_err = b["send_error_t"] is not None and now_m - b["send_error_t"] < 3.0
            if recent_send_err:
                blv = 2
            bus = {"name": "ak45/can_bus", "level": blv, "hardware_id": f"{self.channel} ({self.interface})",
                   "message": (f"송신 실패: {b['last_send_error']}" if recent_send_err else
                               "정상" if blv == 0 else f"{b['state']}"),
                   "values": {k: str(v) for k, v in b.items() if k not in ("send_error_t",)}, "t": now, "age": 0.0}
            hs = self.homing.status(now_m)
            hj = [dict(j, joint=jn) for jn, j in hs["joints"].items()]
            for row in hj:
                m = self.ak[row["motor"]]
                row["command_active"] = m.cmd is not None
                row["fault"] = m.fault
            servos = {n: {"deg": d, "age": now_m - t} for n, (d, t) in self.servo_sent.items()}
            foreign = {k: {"count": c, "age": now_m - t} for k, (c, t) in self.foreign.items() if now_m - t < 30}
            cal_rows = [{"joint": j.joint, "motor": j.motor, "id": j.id, "name": j.name, "sign": j.sign, "scale": j.scale,
                         "zero_deg": j.zero_deg, "verified": j.verified} for j in self.cal.joints]
            return {
                "available": self.running, "reason": "" if self.running else "팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결",
                "virtual": False, "direct": True,
                "motors": motors, "bus": bus,
                "diag_age": 0.0 if self.running else None,
                "joint_age": min((now_m - m.fb_t for m in self.ak.values() if m.fb_t), default=None),
                "homing": {"available": True, "seen": self.running, "all_homed": hs["all_homed"], "age": 0.001,   # 0.0 이면 server.cmd_gate 의 (age or 99) 가 99 가 됨
                           "stamp": now, "joints": hj},
                "joints": joints, "joints_homed": homed_fb,
                "servos": servos,
                "servo_age": (now_m - max(t for _, t in self.servo_sent.values())) if self.servo_sent else None,
                "object_pose": None,
                "cmd_subs": 1 if self.running else 0,
                "conn": {"state": "direct" if self.running else "off", "reason": "직접 CAN", "pid": os.getpid()},
                "direct_state": {
                    "connected": self.running, "interface": self.interface, "channel": self.channel,
                    "lock": self.lock_path if self.lock_fd is not None else None, "started": self.started,
                    "params": dict(P), "playback": dict(self.play), "foreign": foreign,
                    "calib_dirty": self.calib_dirty, "calibration": cal_rows, "calibration_file": str(self.cal.path),
                    "servo_targets": dict(self.servo_mover.target),
                    "log": list(self.lines)[-40:],
                },
            }


class Disconnected:
    """팔 연결 전(직접 CAN 모드) 서버가 쓰는 빈 백엔드."""

    def __init__(self, reason="팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결", lines=()):
        self.reason, self.lines = reason, list(lines)
        self.play = {"running": False}

    def _no(self, *a, **k):
        return {"ok": False, "error": self.reason}

    send_joint_command = call_trigger = play_trajectory = calib = limp = _no

    def hold_all(self, *a, **k):
        return {"ok": True, "msg": "팔 연결 안 됨 — 홀드할 것 없음"}

    stop_trajectory = stop_all = hold_all

    def snapshot(self):
        return {"available": False, "reason": self.reason, "virtual": False, "direct": True, "motors": [], "bus": None,
                "diag_age": None, "joint_age": None, "homing": {"available": True, "seen": False, "joints": []},
                "joints": {}, "joints_homed": {}, "servos": {}, "servo_age": None, "object_pose": None, "cmd_subs": 0,
                "conn": {"state": "off", "reason": self.reason, "pid": None},
                "direct_state": {"connected": False, "log": self.lines[-40:], "playback": {"running": False},
                                 "params": dict(DEFAULTS), "foreign": {}}}
