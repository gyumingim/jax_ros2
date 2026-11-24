"""대시보드 '휴대폰 카메라' 탭의 서버 쪽: 큐브 자리 = 거치한 폰의 프레임에서 ArUco (phone_detect.py) + 폰 뎁스 (phone_depth.py) 를
있는 대로 합친 것 (phone_fuse.py). 모드도 사진 올리기도 없다 (2026-11-24 뺌 — 인식은 폰 스테레오 프로그램의 프레임으로만). 팔은 여기서 안 움직인다.

server.py 는 install(app, log_dir) 한 줄만 가진다. 계획·재생은 화면(phone.js)이 원래 있던 경로
(/api/svc/pick_dry/start → 역기구학 탭의 재생 버튼과 같은 확인창)를 그대로 쓴다.

    GET  /api/phone/live?d=&pick=   live_api.py 로 옮김 (2026-11-24): 서버 스레드가 프레임마다 미리 계산한 것을 준다 (GET /api/live/cubes 의 pick)
    GET  /api/phone/config   기준 종이가 놓인 자리 {x, y, rot_deg, cube_h} (종이는 없어도 된다 — 그때는 CAD 거치 자세 + 책상 높이로 큐브 자리)
    POST /api/phone/config   {x, y, rot_deg, cube_h} 저장 (logs/phone_cam.json)
    GET  /api/phone/sheet    인쇄물 PDF (dashboard/phone_markers/phone_markers.pdf)

폰 뎁스 (phone_depth.py: 폰 스테레오 프로그램 SteroDepthSeg 의 inference.json·inference.png 를 읽는다):
    GET  /api/phone/depth?d=<책상 높이>   뎁스만의 지금 결과 {ok, cube, warn, age_s, pipeline{alive, …}, image_mtime, …} (화면은 안 씀)
    GET  /api/phone/depth/image           그 프로그램이 저장한 inference.png
    GET  /api/phone/depth/config, POST    {repo_dir, run_dir, camera_to_robot[6], yaw_manual_deg, …} (logs/phone_depth.json)
    POST /api/phone/depth/start, /stop    폰 스테레오 프로그램 켜기·끄기 (venv_ffs, --headless; 끌 때 Ctrl+C 와 같은 SIGINT)
                                          보정 파일은 붙은 폰 모델로 고른다 (CALIB_BY_MODEL: SM-S921N → calib_phone_pair_s24.json, 그 밖 기본)
                                          → config 의 calib {file, model, note}

'책상 높이 재기' (phone_table.py: 거치대의 폰이 CAD 자리에 있다고 보고 책상 높이 d 를 잰다 — 보정 없음):
    POST /api/phone/table_height          본문 {d: 지금 d 칸} (없어도 됨). 지금 프레임의 큐브 윗면 마커 (폰 보정값 + CAD 자세), 기준 종이 (있으면 — 이것을 먼저 쓴다),
                                          뎁스 맵 (<결과 폴더>/rgbd/rgbd.npz 가 새것일 때) 으로
                                          → {ok, d, source 'aruco'|'cubes'|'depth', text, in_range, fill_ok ('넣기' 버튼), tilt_deg, aruco{…}, cubes{…}, depth{…}, diffs, warn, image}
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
PIPE = None                 # 폰 스테레오 프로그램 (Pipeline) — live_api 가 상태를 읽는다
LAST_TABLE = {"r": None}    # 마지막 '책상 높이 재기' {d, source, t} — live_api 의 table 칸
SHEET = HERE / "phone_markers" / "phone_markers.pdf"
# 폰 모델별 두 렌즈 보정 파일 (저장소 안). 없는 모델이면 calib_phone_pair.json (지금 S25 SM-S931N 것).
# 다른 폰의 보정으로 켜면 그 프로그램이 'Phone rectification failed' 로 멈춘다 (dy 20 px 넘음).
CALIB_DEFAULT = "calib_phone_pair.json"
CALIB_BY_MODEL = {"SM-S921N": "calib_phone_pair_s24.json"}     # Galaxy S24 — 2026-09-22 보드 캘리 (SteroDepthSeg 96fad02 에서 꺼냄)
CALIB_MODELS = {"SM-S931N": CALIB_DEFAULT, **CALIB_BY_MODEL}   # 큐브 마커 재기 (초점거리) 에 보정값을 믿는 모델 — 그 밖이면 보정 없음 (intrinsics None)
MODEL_KEEP_S = 60.0         # adb 가 잠깐 안 되면 이만큼 마지막 모델을 그대로
_MODEL = {"t": 0.0, "v": None, "ok_t": 0.0, "last": None}
LIMITS = {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "rot_deg": (-180.0, 180.0), "cube_h": (0.0, 0.3)}


def phone_model():
    """USB 로 붙은 폰의 모델 (adb getprop ro.product.model), 없거나 adb 가 안 되면 None. 5초 캐시."""
    if time.monotonic() - _MODEL["t"] < 5.0:
        return _MODEL["v"]
    v = None
    try:
        r = subprocess.run(["adb", "shell", "getprop", "ro.product.model"], capture_output=True, text=True, timeout=3)
        if r.returncode == 0 and r.stdout.strip():
            v = r.stdout.strip().splitlines()[-1].strip()
    except (OSError, subprocess.SubprocessError):
        pass
    now = time.monotonic()
    if v is not None:
        _MODEL.update(ok_t=now, last=v)
    elif _MODEL["last"] is not None and now - _MODEL["ok_t"] < MODEL_KEEP_S:
        v = _MODEL["last"]
    _MODEL.update(t=now, v=v)
    return v


def calib_choice(cfg):
    """{file, model, note}: 붙은 폰 모델로 고른 보정 파일 (저장소 기준 상대 경로)."""
    model = phone_model()
    name = CALIB_BY_MODEL.get(model or "")
    if name and not (Path(cfg["repo_dir"]) / name).exists():
        return {"file": CALIB_DEFAULT, "model": model, "note": f"{model} 용 {name} 이 저장소에 없어 기본 파일 — 다른 폰 보정이면 정류 검사에서 멈춘다"}
    if name:
        return {"file": name, "model": model, "note": f"{model} → {name}"}
    if model is None:
        return {"file": CALIB_DEFAULT, "model": None, "note": f"폰이 USB 로 안 보임 (adb) — 기본 {CALIB_DEFAULT} (S25 SM-S931N)"}
    return {"file": CALIB_DEFAULT, "model": model, "note": f"{model} → 기본 {CALIB_DEFAULT} (S25 SM-S931N" + (" — 이 폰 것" if model == "SM-S931N" else " 것 — 이 폰 보정이 아니면 정류 검사에서 멈춘다") + ")"}


_INTR = {"t": 0.0, "key": None, "v": None}


def intrinsics(dcfg):
    """붙은 폰의 왼쪽 렌즈 보정값 {K, dist, size, file} (calib_choice 로 고른 파일) 또는 None. 30초 캐시 (실시간 인식이 프레임마다 부른다).
    2026-11-24 검토: 모르는 폰 (CALIB_MODELS 에 없음 · adb 로 안 보임) 이나 그 모델 파일이 없어 기본으로 떨어진 경우는 None
    — 다른 폰의 초점거리로 큐브 마커 거리를 조용히 틀리게 재지 않게 (큐브 마커 재기·종이 없는 큐브 자리 = '보정 없음'). 프레임 비율 검사는 phone_table.scaled_K."""
    key = dcfg.get("repo_dir")
    if _INTR["key"] == key and time.monotonic() - _INTR["t"] < 30.0:
        return _INTR["v"]
    v = None
    try:
        ch = calib_choice(dcfg)
        if ch["model"] in CALIB_MODELS and ch["file"] == CALIB_MODELS[ch["model"]]:
            v = PT.load_intrinsics(str(Path(dcfg["repo_dir"]) / ch["file"]))
    except Exception:
        pass
    _INTR.update(t=time.monotonic(), key=key, v=v)
    return v


def image_rot_q(dcfg):
    """큐브 마커의 사진 회전 설정 (phone_table.choose_rot) = 폰 뎁스 설정 camera_to_robot 회전에 가까운 q (거치 고정, 기본 뒤집힘 180°) 또는 None."""
    return PT.rot_from_c2r(dcfg.get("camera_to_robot") or [])


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


def _table_height(image, depth_cfg, cam_cfg=None, d_now=None):
    """프레임 (BGR 배열 | None) 의 큐브 윗면 마커·기준 종이로, 폰 뎁스 맵이 새것이면 그것으로도 재서 한 줄로 (phone_table.combine).
    기준 종이는 없어도 된다 (2026-11-24 사용자: 큐브 윗면 마커로 책상 높이도)."""
    try:
        import hand_ik
        PT.D_RANGE = tuple(hand_ik.D_RANGE)     # 계획이 받는 범위 한 곳
    except Exception:
        pass
    cam_cfg = cam_cfg or dict(PD.DEFAULT_CFG)
    intr = intrinsics(depth_cfg)
    aruco = _jpeg(PT.measure_aruco(image, intr)) if image is not None else None
    cubes = _jpeg(PT.measure_cubes(image, intr, cam_cfg.get("cube_h"), image_rot_q(depth_cfg))) if image is not None else None
    depth = PT.measure_depth(DP.run_dir(depth_cfg), time.time())
    if image is None and depth.get("missing"):
        return {"ok": False, "warn": [], "reason": "폰 프로그램의 새 프레임이 없음 — 폰 프로그램을 켜고 (거치한 폰이 책상 위 큐브를 보게) 다시"}
    marker_ok = any(r is not None and r["ok"] for r in (aruco, cubes))
    if marker_ok and not depth["ok"] and (depth.get("missing") or depth.get("stale")):
        depth = None                            # 프레임으로 쟀고 뎁스 맵은 안 켜져 있음 — 말할 것 없음
    if aruco is not None and not aruco["ok"] and not aruco.get("refs"):
        aruco = None                            # 기준 종이가 아예 없음 — 없어도 된다, 말하지 않는다
    r = PT.combine(aruco, depth, cubes, d_now)
    main = {"aruco": aruco, "cubes": cubes}.get(r.get("source"))
    pic = next((x["image"] for x in (main, cubes, aruco) if x and "image" in x), None)     # 쓴 것의 그림 먼저
    for x in (aruco, cubes):
        if x is not None:
            x.pop("image", None)
    if pic:
        r["image"] = pic
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

    @staticmethod
    def live_jpg_supported(cfg):
        """그 프로그램에 --live-jpg (손대지 않은 프레임을 <결과 폴더>/live.jpg 로 자주 저장, README 의 작은 고침) 가 있나."""
        try:
            return "--live-jpg" in (Path(cfg["repo_dir"]) / "z_phone_stereo.py").read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False

    def command(self, cfg, calib=None):
        repo = Path(cfg["repo_dir"])
        calib = calib or calib_choice(cfg)["file"]
        return ([str(repo / "venv_ffs" / "bin" / "python"), "z_phone_stereo.py", "--physical", "2", "5", "--calib-stereo", calib,
                 "--yolo", "seg", "--headless", "--out", DP.run_dir(cfg)] + (["--rgbd-viewer"] if cfg.get("save_rgbd") else [])
                + (["--live-jpg", "10"] if self.live_jpg_supported(cfg) else []))

    def missing(self, cfg, calib=None):
        repo = Path(cfg["repo_dir"])
        calib = calib or calib_choice(cfg)["file"]
        need = [(repo / "z_phone_stereo.py", "폰 스테레오 저장소 (git clone https://github.com/team-jax/SteroDepthSeg)"),
                (repo / "venv_ffs" / "bin" / "python", "venv_ffs (그 저장소에서 bash tools/setup_ffs.sh)"),
                (repo / calib, f"{calib} (이 폰의 두 렌즈 보정 파일)")]
        return [f"{what} — {p} 없음" for p, what in need if not p.exists()]

    def start(self, cfg):
        with self.lock:
            if self.pid() is not None:
                return {"ok": False, "error": "이미 켜져 있음"}
            calib = calib_choice(cfg)
            miss = self.missing(cfg, calib["file"])
            if miss:
                return {"ok": False, "error": "켤 수 없음: " + " / ".join(miss)}
            cmd = self.command(cfg, calib["file"])
            Path(DP.run_dir(cfg)).mkdir(parents=True, exist_ok=True)
            self.log.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log, "wb") as lf:
                lf.write(("$ " + " ".join(cmd) + "\n").encode())
                lf.flush()
                self.proc = subprocess.Popen(cmd, cwd=cfg["repo_dir"], stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                             start_new_session=True)
            self.pid_file.write_text(json.dumps({"pid": self.proc.pid, "stime": self._stime(self.proc.pid)}))
            return {"ok": True, "calib": calib, "msg": f"폰 스테레오 프로그램 켬 ({calib['file']}) — 폰 잠금을 풀고, 처음 몇 초는 가만히"}

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


def install(app, log_dir, mode=lambda: "direct"):
    """mode() → 'direct'|'virtual'|'ros'. 가상 모드는 카메라를 안 쓴다 (책상 높이 재기 거절)."""
    _install_depth(app, log_dir)
    cfg_file = Path(log_dir) / "phone_cam.json"

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
        try:     # 본문 {d: 화면의 지금 d 칸} (없어도 됨 — 큐브 마커 값 확인에 쓴다)
            b = await req.json()
            d_now = float(b.get("d")) if isinstance(b, dict) and b.get("d") is not None else None
        except Exception:
            d_now = None
        if mode() == "virtual":
            return JSONResponse({"ok": False, "reason": "가상 모드에서는 카메라를 안 쓴다 — 책상 높이는 역기구학 탭 칸 그대로"})
        try:
            dcfg = DP.load_cfg(Path(log_dir) / "phone_depth.json")
            frame = DP.read_frame(dcfg, time.time())[0]
            r = await asyncio.to_thread(_table_height, frame, dcfg, _load(cfg_file), d_now)
            if r.get("ok"):
                LAST_TABLE["r"] = {"d": r.get("d"), "source": r.get("source"), "in_range": r.get("in_range"), "fill_ok": r.get("fill_ok"), "t": time.time()}
            return JSONResponse(r)
        except Exception as e:
            return JSONResponse({"ok": False, "reason": f"재는 중 오류: {type(e).__name__}: {e}"})

    @app.get("/api/phone/sheet")
    def _sheet():
        if not SHEET.is_file():
            return JSONResponse({"ok": False, "error": "인쇄물 없음 — dashboard/tools/make_phone_markers.py"}, status_code=404)
        return FileResponse(str(SHEET), media_type="application/pdf", filename="phone_markers.pdf")


def _install_depth(app, log_dir):
    global PIPE
    cfg_file = Path(log_dir) / "phone_depth.json"
    pipe = PIPE = Pipeline(log_dir)
    prev = {"pos": None, "t": 0.0}     # 직전에 고른 자리 (3초 지나면 잊음) — 후보가 여럿일 때 프레임마다 바뀌지 않게

    def _td(d):
        td = float(d) if d.strip() else 0.4775
        if not (math.isfinite(td) and 0.2 <= td <= 0.8):
            raise ValueError
        return td

    def _eval(td):
        """지금 뎁스 결과 + 그림 시각 + 폰 프로그램 상태."""
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
        return r

    @app.get("/api/phone/depth")
    def _depth(d: str = ""):
        try:
            td = _td(d)
        except ValueError:
            return JSONResponse({"ok": False, "reason": "책상 높이 d 숫자 (0.2~0.8 m)"}, status_code=400)
        return JSONResponse(_eval(td))

    @app.get("/api/phone/depth/image")
    def _image():
        p = Path(DP.run_dir(DP.load_cfg(cfg_file))) / "inference.png"
        if not p.is_file():
            return JSONResponse({"ok": False, "error": "그림 없음"}, status_code=404)
        return FileResponse(str(p), media_type="image/png", headers={"cache-control": "no-store"})

    @app.get("/api/phone/depth/config")
    def _get():     # def (async 아님) — adb getprop 이 이벤트 루프를 막지 않게 스레드에서
        cfg = DP.load_cfg(cfg_file)
        calib = calib_choice(cfg)
        return JSONResponse({"ok": True, "cfg": cfg, "default_c2r": DP.DEFAULT_C2R, "run_dir": DP.run_dir(cfg), "calib": calib,
                             "missing": pipe.missing(cfg, calib["file"]), "command": " ".join(pipe.command(cfg, calib["file"]))})

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
