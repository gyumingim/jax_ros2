"""대시보드 '휴대폰 카메라' 탭의 서버 쪽: 휴대폰 사진 → 큐브 자리 (phone_detect.py, ArUco). 팔은 여기서 안 움직인다.

server.py 는 install(app, log_dir) 한 줄만 가진다. 계획·재생은 화면(phone.js)이 원래 있던 경로
(/api/svc/pick_dry/start → 역기구학 탭의 재생 버튼과 같은 확인창)를 그대로 쓴다.

    POST /api/phone/detect   본문 = 사진 바이트 (JPEG/PNG, multipart 아님)  → {ok, cube{x,y,yaw_deg}, refs, warn, image(data URL), …}
    GET  /api/phone/config   기준 종이가 놓인 자리 {x, y, rot_deg, cube_h}
    POST /api/phone/config   {x, y, rot_deg, cube_h} 저장 (logs/phone_cam.json)
    GET  /api/phone/sheet    인쇄물 PDF (dashboard/phone_markers/phone_markers.pdf)

'세그먼트(폰 뎁스)' 모드 (phone_depth.py: 폰 스테레오 프로그램 SteroDepthSeg 의 inference.json 을 읽는다):
    GET  /api/phone/depth?d=<책상 높이>   지금 결과 {ok, cube{x,y,yaw_deg}, warn, age_s, pipeline{alive, …}, image_mtime, …}
    GET  /api/phone/depth/image           그 프로그램이 저장한 inference.png
    GET  /api/phone/depth/config, POST    {repo_dir, run_dir, camera_to_robot[6], yaw_manual_deg, …} (logs/phone_depth.json)
    POST /api/phone/depth/start, /stop    폰 스테레오 프로그램 켜기·끄기 (venv_ffs, --headless; 끌 때 Ctrl+C 와 같은 SIGINT)

'책상 높이 재기' (phone_table.py: 거치대의 폰이 CAD 자리에 있다고 보고 책상 높이 d 를 잰다 — 보정 없음):
    POST /api/phone/table_height          본문 = 사진 바이트 (기준 종이가 보이게; 비우면 뎁스만) → {ok, d, source, text, in_range, tilt_deg,
                                          aruco{…}, depth{…}, diff_mm, warn, image}. 뎁스 = <결과 폴더>/rgbd/rgbd.npz 가 새것일 때만
"""
import asyncio
import base64
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from fastapi import Request
from fastapi.responses import FileResponse, JSONResponse

import phone_depth as DP
import phone_detect as PD
import phone_table as PT

HERE = Path(__file__).resolve().parent
SHEET = HERE / "phone_markers" / "phone_markers.pdf"
MAX_BYTES = 25 << 20
LIMITS = {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "rot_deg": (-180.0, 180.0), "cube_h": (0.0, 0.3)}


def _load(path):
    cfg = dict(PD.DEFAULT_CFG)
    try:
        d = json.loads(path.read_text())
        cfg.update({k: float(d[k]) for k in LIMITS if k in d})
    except (OSError, ValueError, TypeError):
        pass
    return cfg


def _jpeg(r):
    """결과 dict 의 annotated (BGR 그림) 를 image (data URL) 로 바꾼다."""
    import cv2
    vis = r.pop("annotated", None)
    if vis is not None:
        k = 1280 / max(vis.shape[:2])
        if k < 1:
            vis = cv2.resize(vis, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            r["image"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
    return r


def _run(data, cfg):
    return _jpeg(PD.detect(data, cfg))


def _table_height(data, depth_cfg):
    """사진이 있으면 ArUco 로, 폰 뎁스 맵이 새것이면 그것으로도 재서 한 줄로 (phone_table.combine)."""
    try:
        import hand_ik
        PT.D_RANGE = tuple(hand_ik.D_RANGE)     # 계획이 받는 범위 한 곳
    except Exception:
        pass
    aruco = _jpeg(PT.measure_aruco(data)) if data else None
    depth = PT.measure_depth(DP.run_dir(depth_cfg), time.time())
    if aruco is not None and not depth["ok"] and (depth.get("missing") or depth.get("stale")):
        depth = None                            # 사진으로 재는 중이고 폰 뎁스는 안 켜져 있음 — 말할 것 없음
    r = PT.combine(aruco, depth)
    if aruco is not None and "image" in aruco:
        r["image"] = aruco.pop("image")
    return r


class Pipeline:
    """폰 스테레오 프로그램 (SteroDepthSeg z_phone_stereo.py) 한 개를 켜고 끈다. 새 프로세스 그룹으로 띄우고 끌 때 그룹에 SIGINT
    (터미널 Ctrl+C 와 같음 — 그 프로그램이 폰 카메라·adb 포워딩을 풀고 끝난다) → 8초 뒤에도 살아 있으면 SIGTERM → 3초 뒤 SIGKILL.
    대시보드를 껐다 켜도 logs/phone_depth_pid.json (pid + 시작 시각) 으로 다시 알아본다."""

    def __init__(self, log_dir):
        self.log = Path(log_dir) / "phone_depth.log"
        self.pid_file = Path(log_dir) / "phone_depth_pid.json"
        self.proc = None
        self.lock = threading.Lock()

    @staticmethod
    def _stime(pid):
        try:
            f = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            return None if f[0] == "Z" else f[19]      # 좀비 = 이미 끝남
        except Exception:
            return None

    def pid(self):
        """살아 있는 우리 프로그램의 pid, 없으면 None."""
        if self.proc is not None:
            if self.proc.poll() is None:
                return self.proc.pid
            self.proc = None
        try:
            d = json.loads(self.pid_file.read_text())
            if d.get("stime") and self._stime(d["pid"]) == d["stime"]:
                return int(d["pid"])
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return None

    def command(self, cfg):
        repo = Path(cfg["repo_dir"])
        return [str(repo / "venv_ffs" / "bin" / "python"), "z_phone_stereo.py", "--physical", "2", "5", "--calib-stereo", "calib_phone_pair.json",
                "--yolo", "seg", "--headless", "--out", DP.run_dir(cfg)] + (["--rgbd-viewer"] if cfg.get("save_rgbd") else [])

    def missing(self, cfg):
        repo = Path(cfg["repo_dir"])
        need = [(repo / "z_phone_stereo.py", "폰 스테레오 저장소 (git clone https://github.com/team-jax/SteroDepthSeg)"),
                (repo / "venv_ffs" / "bin" / "python", "venv_ffs (그 저장소에서 bash tools/setup_ffs.sh)"),
                (repo / "calib_phone_pair.json", "calib_phone_pair.json (S25 두 렌즈 보정 파일)")]
        return [f"{what} — {p} 없음" for p, what in need if not p.exists()]

    def start(self, cfg):
        with self.lock:
            if self.pid() is not None:
                return {"ok": False, "error": "이미 켜져 있음"}
            miss = self.missing(cfg)
            if miss:
                return {"ok": False, "error": "켤 수 없음: " + " / ".join(miss)}
            cmd = self.command(cfg)
            Path(DP.run_dir(cfg)).mkdir(parents=True, exist_ok=True)
            self.log.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log, "wb") as lf:
                lf.write(("$ " + " ".join(cmd) + "\n").encode())
                lf.flush()
                self.proc = subprocess.Popen(cmd, cwd=cfg["repo_dir"], stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                             start_new_session=True)
            self.pid_file.write_text(json.dumps({"pid": self.proc.pid, "stime": self._stime(self.proc.pid)}))
            return {"ok": True, "msg": "폰 스테레오 프로그램 켬 — 폰 잠금을 풀고, 처음 몇 초는 가만히"}

    def stop(self):
        with self.lock:
            pid = self.pid()
            if pid is None:
                return {"ok": True, "msg": "이미 꺼져 있음"}
            for sig, wait in ((signal.SIGINT, 8.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(pid, sig)
                except (ProcessLookupError, PermissionError):
                    break
                t0 = time.monotonic()
                while time.monotonic() - t0 < wait and self.pid() is not None:
                    if self.proc is not None:
                        try:
                            self.proc.wait(0.2)
                        except subprocess.TimeoutExpired:
                            pass
                    else:
                        time.sleep(0.2)
                if self.pid() is None:
                    break
            alive = self.pid() is not None
            if not alive:
                self.pid_file.unlink(missing_ok=True)
            return {"ok": not alive, "msg": "폰 스테레오 프로그램 끔"} if not alive else {"ok": False, "error": "안 꺼짐"}

    def tail(self, n=6):
        try:
            return self.log.read_bytes()[-4000:].decode("utf-8", "replace").splitlines()[-n:]
        except OSError:
            return []


def install(app, log_dir):
    _install_depth(app, log_dir)
    cfg_file = Path(log_dir) / "phone_cam.json"

    @app.post("/api/phone/detect")
    async def _detect(req: Request):
        data = await req.body()
        if not data or len(data) > MAX_BYTES:
            return JSONResponse({"ok": False, "reason": "사진이 없거나 너무 큼 (25 MB 까지)"}, status_code=400)
        try:
            return JSONResponse(await asyncio.to_thread(_run, data, _load(cfg_file)))
        except Exception as e:
            return JSONResponse({"ok": False, "reason": f"찾는 중 오류: {type(e).__name__}: {e}"})

    @app.get("/api/phone/config")
    def _get():
        return JSONResponse({"ok": True, "cfg": _load(cfg_file), "default": PD.DEFAULT_CFG, "sheet": SHEET.is_file(),
                             "cube_id": PD.CUBE_ID, "ref_ids": sorted(PD.REF_MM)})

    @app.post("/api/phone/config")
    async def _set(req: Request):
        try:
            b = await req.json()
            cfg = {k: float(b[k]) for k in LIMITS}
            for k, (lo, hi) in LIMITS.items():
                if not (math.isfinite(cfg[k]) and lo <= cfg[k] <= hi):
                    raise ValueError(k)
        except Exception:
            return JSONResponse({"ok": False, "error": "x, y (m, ±1), rot_deg (±180), cube_h (0~0.3 m) 숫자"}, status_code=400)
        cfg_file.parent.mkdir(parents=True, exist_ok=True)
        cfg_file.write_text(json.dumps(cfg))
        return JSONResponse({"ok": True, "cfg": cfg, "msg": "기준 종이 자리 저장"})

    @app.post("/api/phone/table_height")
    async def _table(req: Request):
        data = await req.body()
        if len(data) > MAX_BYTES:
            return JSONResponse({"ok": False, "reason": "사진이 너무 큼 (25 MB 까지)"}, status_code=400)
        try:
            return JSONResponse(await asyncio.to_thread(_table_height, data, DP.load_cfg(Path(log_dir) / "phone_depth.json")))
        except Exception as e:
            return JSONResponse({"ok": False, "reason": f"재는 중 오류: {type(e).__name__}: {e}"})

    @app.get("/api/phone/sheet")
    def _sheet():
        if not SHEET.is_file():
            return JSONResponse({"ok": False, "error": "인쇄물 없음 — dashboard/tools/make_phone_markers.py"}, status_code=404)
        return FileResponse(str(SHEET), media_type="application/pdf", filename="phone_markers.pdf")


def _install_depth(app, log_dir):
    cfg_file = Path(log_dir) / "phone_depth.json"
    pipe = Pipeline(log_dir)
    prev = {"pos": None, "t": 0.0}     # 직전에 고른 자리 (3초 지나면 잊음) — 후보가 여럿일 때 프레임마다 바뀌지 않게

    @app.get("/api/phone/depth")
    def _depth(d: str = ""):
        try:
            td = float(d) if d.strip() else 0.4775
            if not (math.isfinite(td) and 0.2 <= td <= 0.8):
                raise ValueError
        except ValueError:
            return JSONResponse({"ok": False, "reason": "책상 높이 d 숫자 (0.2~0.8 m)"}, status_code=400)
        cfg = DP.load_cfg(cfg_file)
        now = time.monotonic()
        r = DP.evaluate(cfg, td, time.time(), prev["pos"] if now - prev["t"] <= 3.0 else None)
        if r["ok"]:
            prev.update(pos=r.pop("pos"), t=now)
        try:
            r["image_mtime"] = os.stat(os.path.join(DP.run_dir(cfg), "inference.png")).st_mtime
        except OSError:
            r["image_mtime"] = None
        pid = pipe.pid()
        r["pipeline"] = {"alive": pid is not None, "pid": pid, "log": pipe.tail() if pid is None else pipe.tail(3)}
        return JSONResponse(r)

    @app.get("/api/phone/depth/image")
    def _image():
        p = Path(DP.run_dir(DP.load_cfg(cfg_file))) / "inference.png"
        if not p.is_file():
            return JSONResponse({"ok": False, "error": "그림 없음"}, status_code=404)
        return FileResponse(str(p), media_type="image/png", headers={"cache-control": "no-store"})

    @app.get("/api/phone/depth/config")
    def _get():
        cfg = DP.load_cfg(cfg_file)
        return JSONResponse({"ok": True, "cfg": cfg, "default_c2r": DP.DEFAULT_C2R, "run_dir": DP.run_dir(cfg),
                             "missing": pipe.missing(cfg), "command": " ".join(pipe.command(cfg))})

    @app.post("/api/phone/depth/config")
    async def _set(req: Request):
        try:
            new = DP.clean_cfg(await req.json())
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e) or "설정 값이 틀림"}, status_code=400)
        saved = {}
        try:
            saved = DP.clean_cfg(json.loads(cfg_file.read_text()))
        except (OSError, ValueError, TypeError):
            pass
        saved.update(new)
        cfg_file.parent.mkdir(parents=True, exist_ok=True)
        cfg_file.write_text(json.dumps(saved, ensure_ascii=False, indent=1))
        prev["pos"] = None
        return JSONResponse({"ok": True, "cfg": DP.load_cfg(cfg_file), "msg": "폰 뎁스 설정 저장"})

    @app.post("/api/phone/depth/start")
    async def _start():
        return JSONResponse(await asyncio.to_thread(pipe.start, DP.load_cfg(cfg_file)))

    @app.post("/api/phone/depth/stop")
    async def _stop():
        return JSONResponse(await asyncio.to_thread(pipe.stop))
