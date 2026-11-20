"""서버 쪽 ROS 연결: ROS 구독(ros_monitor.RosMonitor)을 서버 안에서 import 하지 않고 자식 프로세스로 띄운다.

서버를 ROS 를 source 하지 않은 터미널에서 켰어도 화면의 'ROS 연결(source)' 버튼으로 붙을 수 있게 하려는 것.
자식 = bash -c "source /opt/ros/<JAX_ROS_DISTRO>/setup.bash; source <JAX_WS>/install/setup.bash(있으면);
                exec python3 -u dashboard/ros_monitor_proc.py"
자식 stdout 의 JSON 줄로 상태를 받고, 원점 서비스 호출은 stdin 명령으로 보낸다(ros_monitor_proc.py 맨 위).
자식의 stderr 는 logs/ros_monitor.log.
"""
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from procman import IS_LINUX, LOG_DIR, tail_file
from services import ROS_DISTRO, WS

HERE = Path(__file__).resolve().parent
LOG = LOG_DIR / "ros_monitor.log"
STALE_S = 3.0     # 자식이 이만큼 아무 줄도 안 보내면 '응답 없음'


class RosBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.state = "off"        # off | starting | connected | failed | dead
        self.reason = "시작 전"
        self.snap = None
        self.snap_t = None
        self.started = None
        self._replies = {}
        self._next_id = 0

    # ---------- 켜기 / 다시 연결 ----------
    def setup_paths(self):
        return f"/opt/ros/{ROS_DISTRO}/setup.bash", f"{WS}/install/setup.bash"

    def start(self):
        """자식을 (다시) 띄운다. {ok, msg | error}."""
        self.stop()
        distro_setup, ws_setup = self.setup_paths()
        if not Path(distro_setup).is_file():
            return self._fail(f"이 PC엔 ROS 없음 ({distro_setup} 없음)")
        bash = shutil.which("bash")
        if not bash:
            return self._fail("이 PC엔 bash 없음 — ROS 연결 불가")
        script = (f"source {shlex.quote(distro_setup)} || exit 3\n"
                  f"[ -f {shlex.quote(ws_setup)} ] && source {shlex.quote(ws_setup)}\n"
                  f"exec python3 -u {shlex.quote(str(HERE / 'ros_monitor_proc.py'))}\n")
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        err = open(LOG, "ab", buffering=0)
        err.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} ROS 자식 켜기 ({distro_setup}, ws {ws_setup})\n".encode())
        try:
            p = subprocess.Popen([bash, "-c", script], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                                 env=dict(os.environ, PYTHONUNBUFFERED="1"), start_new_session=IS_LINUX)
        except OSError as e:
            err.close()
            return self._fail(f"ROS 자식 실행 실패: {e}")
        err.close()
        with self.lock:
            self.proc, self.state, self.reason = p, "starting", "ROS 연결 중…"
            self.snap = self.snap_t = None
            self.started = time.time()
        threading.Thread(target=self._read, args=(p,), daemon=True).start()
        # 붙었는지 잠깐 기다려 답에 넣는다(최대 5초, rclpy import 가 느린 PC)
        end = time.time() + 5
        while time.time() < end and self.state == "starting":
            time.sleep(0.05)
        if self.state == "connected":
            return {"ok": True, "msg": f"ROS 연결됨 ({ROS_DISTRO}, pid {p.pid})"}
        if self.state == "starting":
            return {"ok": True, "msg": "ROS 연결 중… (아직 답 없음)"}
        return {"ok": False, "error": self.reason}

    def _fail(self, why):
        with self.lock:
            self.state, self.reason = "failed", why
        return {"ok": False, "error": why}

    def stop(self):
        p = self.proc
        if p is None:
            return
        self.proc = None
        try:
            p.stdin.close()   # 자식은 stdin EOF 로 끝난다
        except Exception:
            pass
        try:
            p.wait(2)
        except subprocess.TimeoutExpired:
            try:
                if IS_LINUX:
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(2)
                else:
                    p.kill()
            except Exception:
                pass
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGKILL) if IS_LINUX else p.kill()
                except Exception:
                    pass
        with self.lock:
            self.state, self.reason = "off", "꺼짐"

    # ---------- 읽기 ----------
    def _read(self, p):
        for raw in p.stdout:
            try:
                d = json.loads(raw.decode("utf-8", "replace"))
            except ValueError:
                continue
            t = d.get("type")
            with self.lock:
                if p is not self.proc:
                    return
                if t == "snap":
                    self.snap, self.snap_t = d.get("snap"), time.time()
                    self.state, self.reason = "connected", ""
                elif t == "ready":
                    self.state, self.reason = "connected", ""
                elif t == "error":
                    self.state, self.reason = "failed", d.get("reason") or "ROS 초기화 실패"
                elif t == "reply":
                    ev = self._replies.get(d.get("id"))
                    if ev is not None:
                        ev[1] = d.get("result")
                        ev[0].set()
        code = p.wait()
        with self.lock:
            if p is not self.proc:
                return
            if self.state != "failed":
                last = [x for x in tail_file(LOG, 5) if x.strip() and not x.startswith("=====")]
                self.state = "dead"
                self.reason = f"ROS 자식 프로세스 끝남 (코드 {code})" + (f": {last[-1][:200]}" if last else "")
            elif code == 3:
                self.reason = f"ROS source 실패: {self.setup_paths()[0]}"

    # ---------- 서버가 쓰는 것 (RosMonitor 와 같은 모양) ----------
    @property
    def available(self):
        return self.state == "connected"

    def homing_joints(self):
        with self.lock:
            h = (self.snap or {}).get("homing") or {}
        return [j.get("joint") for j in h.get("joints") or []]

    def call_trigger(self, service, timeout=2.0):
        with self.lock:
            p, ok = self.proc, self.state == "connected"
            self._next_id += 1
            rid = self._next_id
        if not ok or p is None:
            return {"ok": False, "error": f"{self.reason or 'ROS 없음'} — 원점 서비스를 못 부름"}
        ev = [threading.Event(), None]
        self._replies[rid] = ev
        try:
            p.stdin.write((json.dumps({"id": rid, "op": "trigger", "service": service, "timeout": timeout}) + "\n").encode())
            p.stdin.flush()
            if not ev[0].wait(timeout + 1.5):
                return {"ok": False, "error": f"{service}: ROS 자식이 {timeout + 1.5:g}초 안에 답 없음"}
            return ev[1] or {"ok": False, "error": "빈 답"}
        except (BrokenPipeError, OSError, ValueError) as e:
            return {"ok": False, "error": f"ROS 자식에 못 보냄: {e}"}
        finally:
            self._replies.pop(rid, None)

    def snapshot(self):
        now = time.time()
        with self.lock:
            state, reason, snap, snap_t, p = self.state, self.reason, self.snap, self.snap_t, self.proc
        if state == "connected" and snap_t is not None and now - snap_t > STALE_S:
            state, reason = "stale", f"ROS 자식이 {now - snap_t:.0f}초째 응답 없음"
        conn = {"state": state, "reason": reason, "pid": p.pid if p and p.poll() is None else None,
                "distro": ROS_DISTRO, "setup": self.setup_paths()[0], "ws_setup": self.setup_paths()[1],
                "log": str(LOG)}
        if state == "connected" and snap:
            return dict(snap, conn=conn)
        why = reason or ("ROS 연결됨 — 첫 상태를 기다리는 중" if state == "connected" else "ROS 없음")
        return {"available": False, "reason": why, "conn": conn, "motors": [], "bus": None, "diag_age": None,
                "joint_age": None, "homing": {"available": False, "reason": why}, "joints": {}, "servos": {},
                "servo_age": None, "object_pose": None}
