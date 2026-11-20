"""ROS 구독 자식 프로세스: 대시보드 서버(ros_bridge.py)가 ROS 를 source 한 bash 안에서 띄운다.

    bash -c "source /opt/ros/<distro>/setup.bash; source <ws>/install/setup.bash; exec python3 -u ros_monitor_proc.py"

stdout 한 줄 = JSON 하나:
  {"type": "ready"}                                  rclpy 구독 시작됨
  {"type": "error", "reason": "..."}                 rclpy 를 못 씀 (그 뒤 종료코드 2)
  {"type": "snap", "snap": RosMonitor.snapshot()}    0.25초마다
  {"type": "reply", "id": n, "result": {...}}        stdin 명령의 답
stdin 한 줄 = 명령:
  {"id": n, "op": "trigger", "service": "/arm_control/home_elbow", "timeout": 2.0}
stdin 이 닫히면(서버가 죽으면) 끝난다. 명령 토픽은 발행하지 않는다(ros_monitor.py 와 같음).
"""
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_monitor import RosMonitor   # noqa: E402

_out_lock = threading.Lock()


def emit(d):
    line = json.dumps(d, ensure_ascii=False)
    with _out_lock:
        sys.stdout.write(line + "\n")
        sys.stdout.flush()


def _reader(mon, stop):
    for line in sys.stdin:
        try:
            cmd = json.loads(line)
        except ValueError:
            continue
        if cmd.get("op") == "trigger":
            def run(c=cmd):
                try:
                    r = mon.call_trigger(str(c.get("service")), float(c.get("timeout") or 2.0))
                except Exception as e:
                    r = {"ok": False, "error": f"서비스 호출 실패: {e}"}
                emit({"type": "reply", "id": c.get("id"), "result": r})
            threading.Thread(target=run, daemon=True).start()
    stop.set()   # stdin EOF = 서버가 끝남


def main():
    mon = RosMonitor()
    mon.start()
    if not mon.available:
        emit({"type": "error", "reason": mon.reason})
        raise SystemExit(2)
    emit({"type": "ready"})
    stop = threading.Event()
    threading.Thread(target=_reader, args=(mon, stop), daemon=True).start()
    try:
        while not stop.is_set():
            emit({"type": "snap", "snap": mon.snapshot()})
            stop.wait(0.25)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        mon.stop()


if __name__ == "__main__":
    main()
