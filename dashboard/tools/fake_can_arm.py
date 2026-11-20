"""가짜 AK45 4개 + 가짜 STM32 서보 노드 (python-can 버스 위, 실물 없이 real_arm 시험용).

    python-can 'virtual' 인터페이스(같은 프로세스 안, Windows 가능) 또는 Linux vcan 에 붙는다.

가짜 AK45 (CubeMars 서보 모드 CAN 흉내, arm_control/tests/fake_ak45.py 의 확장):
  - 0x29 피드백을 50 Hz 로 보낸다 (pos int16×0.1°, spd int16×10 ERPM, cur int16×0.01 A, temp int8, err uint8).
  - 위치 명령(모드 4)을 받으면 그 각으로 속도 한계 120 °/s 로 따라간다. 명령이 0.25 s 넘게 안 오면 힘이 빠진다
    (ak45_node 주석: 재송신을 멈추면 모터 쪽 명령 타임아웃으로 홀딩 토크가 풀린다 — 가정) → 0° 쪽으로 40 °/s 로 떨어진다.
  - 브레이크(모드 2): 바로 힘 빠짐.
  - 시험용 손잡이: silent(피드백 끊기), grab(붙잡기 = 안 움직임, 막힘), temp, err, enc_jump(엔코더 영점 튐),
    press_k(누르기 전류). reset() = 손잡이를 기본값으로(시험 사이 상태가 섞이지 않게; 축 자리·엔코더 영점은 그대로).
가짜 STM32: StdId 5 [id][angle×10] 를 받아 서보 각을 기록 (피드백은 보내지 않음 — 실물 펌웨어와 같음).
모든 수신 프레임은 frames 에 (시각, id, ext, data) 로 쌓인다(첫 명령 전 무송신 검사용).
"""
import struct
import threading
import time

import can

FB = 0x29


class FakeMotor:
    def __init__(self, cid):
        self.id = cid
        self.phys = 0.0
        self.enc_off = 0.0
        self.target = None
        self.cmd_t = None
        self.powered = True
        self.silent_until = 0.0
        self.grab = False
        self.temp = 30
        self.err = 0
        self.vel = 0.0
        self.cur = 0.0
        self.brakes = 0
        self.pos_cmds = 0
        self.press_k = 0.0          # A/° — 누르기 흉내: 홀드 중 전류 += press_k × |목표 − 실제| (0 = 끔, 기본)


class FakeArm:
    def __init__(self, interface="virtual", channel="fake_can0", ids=(1, 2, 3, 4), speed=120.0, cmd_timeout=0.25):
        self.bus = can.Bus(interface=interface, channel=channel)
        self.m = {i: FakeMotor(i) for i in ids}
        self.servo = {}
        self.frames = []
        self.speed, self.cmd_timeout = speed, cmd_timeout
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._sim, daemon=True).start()

    def close(self):
        self.running = False
        time.sleep(0.05)
        self.bus.shutdown()

    def _rx(self):
        while self.running:
            try:
                msg = self.bus.recv(0.02)
            except Exception:
                continue
            if msg is None:
                continue
            now = time.monotonic()
            d = bytes(msg.data)
            with self.lock:
                self.frames.append((now, msg.arbitration_id, msg.is_extended_id, d))
                if msg.is_extended_id:
                    mode, cid = msg.arbitration_id >> 8, msg.arbitration_id & 0xFF
                    m = self.m.get(cid)
                    if m is None or not m.powered or mode == FB:
                        continue
                    if mode == 4 and len(d) >= 4:
                        m.target = struct.unpack(">i", d[:4])[0] / 10000.0 + m.enc_off   # 명령 = 엔코더 좌표
                        m.cmd_t = now
                        m.pos_cmds += 1
                    elif mode == 2:
                        m.target, m.cmd_t = None, None
                        m.brakes += 1
                elif msg.arbitration_id == 5 and len(d) >= 3:
                    self.servo[d[0]] = ((d[1] << 8) | d[2]) / 10.0

    def _sim(self):
        dt, k = 0.005, 0
        nxt = time.monotonic()
        while self.running:
            nxt += dt
            now = time.monotonic()
            k += 1
            with self.lock:
                for m in self.m.values():
                    holding = m.powered and m.target is not None and m.cmd_t is not None and now - m.cmd_t <= self.cmd_timeout
                    if m.grab:
                        v = 0.0
                    elif holding:
                        v = max(-self.speed, min(self.speed, (m.target - m.phys) / 0.03))
                    else:
                        v = max(-40.0, min(40.0, -m.phys / 0.15))
                    m.phys += v * dt
                    m.vel = v
                    m.cur = min(8.0, 0.3 + 0.01 * abs(v) + m.press_k * abs(m.target - m.phys)) if holding else 0.0
                    if k % 4 == 0 and m.powered and now >= m.silent_until:      # 50 Hz
                        pos = m.phys - m.enc_off
                        data = struct.pack(">hhhbB", int(round(pos * 10)), int(v / 6 * 14 / 10), int(m.cur * 100),
                                           int(m.temp), int(m.err))
                        try:
                            self.bus.send(can.Message(arbitration_id=(FB << 8) | m.id, is_extended_id=True, data=data))
                        except Exception:
                            pass
            time.sleep(max(0.0, nxt - time.monotonic()))

    # ---- 시험 손잡이
    def silent(self, cid, sec):
        self.m[cid].silent_until = time.monotonic() + sec

    def power(self, cid, on, enc_jump=0.0):
        with self.lock:
            m = self.m[cid]
            m.powered = on
            if not on:
                m.target = m.cmd_t = None
            else:
                m.enc_off += enc_jump

    def reset(self):
        with self.lock:
            for m in self.m.values():
                m.grab, m.temp, m.err, m.silent_until, m.press_k = False, 30, 0, 0.0, 0.0
                m.powered = True

    def pos(self, cid):
        m = self.m[cid]
        return m.phys - m.enc_off

    def count(self, pred=lambda f: True):
        with self.lock:
            return sum(1 for f in self.frames if pred(f))
