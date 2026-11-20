"""JAX 팔 대시보드: 노드 켜기/끄기/로그 + AK45 상태(읽기 전용).

    python3 dashboard/server.py                 # http://localhost:8080 (이 PC에서만)
    python3 dashboard/server.py --host 0.0.0.0  # 같은 네트워크의 폰·노트북에서도 (보호 없음, 주의)

AK45 상태 칸을 보려면 서버를 켜기 전에 ROS 를 source 한다(rclpy 필요). 없으면 그 칸만 'ROS 없음'.
"""
import argparse
import asyncio
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect   # noqa: E402
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse   # noqa: E402
from fastapi.staticfiles import StaticFiles   # noqa: E402

from procman import IS_LINUX, LOG_DIR, ProcessManager, tail_file   # noqa: E402
from ros_monitor import RosMonitor   # noqa: E402
from services import CALIBRATE_GUIDE, CAN_IFACE, CAN_UP_CMD, REPO, ROS_DISTRO, WS   # noqa: E402

pm = ProcessManager()
ros = RosMonitor()
can_state = {"available": IS_LINUX, "text": "Linux 아님" if not IS_LINUX else "확인 중"}


def _can_watch():
    while True:
        try:
            r = subprocess.run(["ip", "-details", "-json", "link", "show", CAN_IFACE],
                               capture_output=True, text=True, timeout=3)
            if r.returncode != 0:
                can_state.update(up=False, exists=False, text=f"{CAN_IFACE} 없음", state=None, bitrate=None)
            else:
                d = json.loads(r.stdout)[0]
                info = (d.get("linkinfo") or {}).get("info_data") or {}
                up = "UP" in (d.get("flags") or [])
                can_state.update(exists=True, up=up, state=info.get("state"),
                                 bitrate=(info.get("bittiming") or {}).get("bitrate"),
                                 text=f"{CAN_IFACE} {'UP' if up else 'DOWN'}"
                                      + (f" · {info.get('state')}" if info.get("state") else ""))
        except Exception as e:
            can_state.update(up=None, text=f"{CAN_IFACE} 확인 실패: {type(e).__name__}")
        time.sleep(2)


def state():
    return {
        "t": time.time(),
        "services": pm.snapshot(),
        "strays": pm.strays,
        "ros": ros.snapshot(),
        "can": dict(can_state, up_cmd=CAN_UP_CMD),
        "calibrate": CALIBRATE_GUIDE,
        "info": {"ws": WS, "distro": ROS_DISTRO, "repo": str(REPO), "logs": str(LOG_DIR),
                 "platform": sys.platform},
    }


app = FastAPI(title="JAX dashboard")
app.mount("/static", StaticFiles(directory=str(HERE / "static")), name="static")


@app.get("/")
def index():
    return FileResponse(str(HERE / "static" / "index.html"))


@app.get("/api/state")
def api_state():
    return state()


async def _body(req):
    try:
        return await req.json()
    except Exception:
        return {}


@app.post("/api/svc/{sid}/{action}")
async def api_svc(sid: str, action: str, req: Request):
    b = await _body(req)
    params = b.get("params") or {}
    if action == "start":
        r = pm.start(sid, params)
    elif action == "stop":
        r = pm.stop(sid)
    elif action == "restart":
        r = await asyncio.to_thread(pm.restart, sid, params)
    else:
        r = {"ok": False, "error": f"없는 동작 {action}"}
    return JSONResponse(r)


@app.post("/api/stop")
def api_stop():
    return pm.stop_all()


@app.get("/api/log/{sid}")
def api_log(sid: str, n: int = 300):
    it = pm.items.get(sid)
    if it is None:
        return PlainTextResponse("없는 항목", status_code=404)
    return PlainTextResponse("\n".join(tail_file(it.log_path, min(max(n, 1), 5000), 1 << 20)))


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            await sock.send_text(json.dumps(state()))
            await asyncio.sleep(0.5)
    except (WebSocketDisconnect, RuntimeError):
        pass
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="JAX 팔 대시보드")
    ap.add_argument("--host", default=os.environ.get("JAX_DASH_HOST", "127.0.0.1"),
                    help="기본 127.0.0.1(이 PC만). 0.0.0.0 이면 같은 네트워크 누구나 노드를 켜고 끌 수 있음")
    ap.add_argument("--port", type=int, default=int(os.environ.get("JAX_DASH_PORT", "8080")))
    a = ap.parse_args()
    import uvicorn
    ros.start()
    if IS_LINUX:
        threading.Thread(target=_can_watch, daemon=True).start()
    shown = "localhost" if a.host in ("127.0.0.1", "localhost") else a.host
    print(f"대시보드: http://{shown}:{a.port}/   ws={WS} distro={ROS_DISTRO} ROS={'있음' if ros.available else ros.reason}",
          flush=True)
    if a.host not in ("127.0.0.1", "localhost", "::1"):
        print("주의: 외부 접속 열림 — 같은 네트워크 누구나 노드를 켜고 끌 수 있다.", flush=True)
    try:
        uvicorn.run(app, host=a.host, port=a.port, log_level="warning")
    finally:
        # 서버가 꺼져도 노드는 그대로 둔다(끄면 팔이 떨어질 수 있음). 다시 켜면 pids.json 으로 다시 붙잡는다(Linux).
        ros.stop()
        alive = [it.svc.label for it in pm.items.values() if it.alive()]
        if alive:
            print("대시보드 종료. 아직 켜져 있는 항목(그대로 둠): " + ", ".join(alive), flush=True)


if __name__ == "__main__":
    main()
