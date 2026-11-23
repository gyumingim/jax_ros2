"""'강화학습' 탭의 서버 쪽: 팀원이 학습한 집기 정책(team-jax/IK sim2sim, MuJoCo)을 자식 프로세스로 돌리고 결과를 준다.

시뮬만 돈다. 실물 팔·CAN·가상 팔에는 아무것도 보내지 않는다. server.py 는 install(app, ik_dir) 한 줄만 가진다.

    GET  /api/rl/info                       놓는 범위 표, 27곳, 지금 상태
    POST /api/rl/run   {x_cm, y_cm, yaw_deg}   한 곳 돌리기 (한 번에 하나, 몇 초)
    POST /api/rl/eval                       27곳 평가 (run_eval.py --cond base, 1분쯤). 결과는 logs/rl/eval — IK 레포는 안 건드림
    GET  /api/rl/status                     {busy, kind, error, run_id, eval}
    GET  /api/rl/result                     마지막 실행 결과 (rl_runner.py run 의 JSON)
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from fastapi import Request
from fastapi.responses import FileResponse, JSONResponse

from services import LOG_DIR

HERE = Path(__file__).resolve().parent
OUT = LOG_DIR / "rl"
RESULT = OUT / "last_run.json"
EVAL_DIR = OUT / "eval"
RUN_TIMEOUT, EVAL_TIMEOUT = 120, 900

ST = {"busy": False, "kind": None, "since": 0.0, "error": "", "run_id": 0, "eval": None, "info": None}
LOCK = threading.Lock()


def _python():
    return os.environ.get("JAX_RL_PYTHON") or sys.executable


def _sim2sim(ik_dir):
    d = ik_dir()
    p = d / "sim2sim" if d else None
    return p if p and (p / "chain.py").is_file() else None


def _tail(text, n=3):
    lines = [x for x in (text or "").strip().splitlines() if "Warning" not in x and "warn" not in x]
    return " / ".join(lines[-n:])[-400:]


def _call(cmd, timeout):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, f"{timeout}초 안에 안 끝남"
    except Exception as e:
        return None, f"{type(e).__name__}: {e}"
    if r.returncode != 0:
        return None, _tail(r.stderr) or f"종료 코드 {r.returncode}"
    return r.stdout, ""


def _info(ik_dir):
    if ST["info"] is None:
        d = ik_dir()
        if _sim2sim(ik_dir) is None:
            return {"ok": False, "error": "team-jax/IK 의 sim2sim/ 폴더를 못 찾음 (JAX_IK_DIR 가 handroll-limit 브랜치를 가리켜야 함)"}
        out, err = _call([_python(), str(HERE / "rl_runner.py"), "info", "--ik", str(d)], 60)
        if out is None:
            return {"ok": False, "error": "sim2sim 을 못 읽음: " + err + " (mujoco·numpy 필요)"}
        try:
            ST["info"] = json.loads(out.strip().splitlines()[-1])
        except Exception as e:
            return {"ok": False, "error": f"sim2sim 정보 해석 실패: {e}"}
    return ST["info"]


def _eval_summary():
    p = EVAL_DIR / "base" / "summary.json"
    if not p.is_file():
        return None
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return None
    rows = [{"x_cm": r["spot_cm"][0], "y_cm": r["spot_cm"][1], "yaw_deg": r["cube_yaw_deg"], "done": bool(r["done"]),
             "fail": r.get("fail_reason")} for r in d.get("runs", [])]
    return {"done": d["aggregate"]["done"], "runs": d["aggregate"]["runs"], "rows": rows, "mtime": p.stat().st_mtime}


def _job(kind, cmd, timeout):
    out, err = _call(cmd, timeout)
    with LOCK:
        ST.update(busy=False, error="" if out is not None else err)
        if out is not None and kind == "run":
            ST["run_id"] += 1
        if kind == "eval":
            ST["eval"] = _eval_summary()


def _start(kind, cmd, timeout):
    with LOCK:
        if ST["busy"]:
            return {"ok": False, "error": "이미 돌리는 중 (" + ("27곳 평가" if ST["kind"] == "eval" else "한 곳 실행") + ") — 끝난 뒤 다시"}
        ST.update(busy=True, kind=kind, since=time.time(), error="")
    OUT.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=_job, args=(kind, cmd, timeout), daemon=True).start()
    return {"ok": True}


def _status():
    return {"ok": True, "busy": ST["busy"], "kind": ST["kind"], "elapsed": round(time.time() - ST["since"], 1) if ST["busy"] else None,
            "error": ST["error"], "run_id": ST["run_id"], "has_result": RESULT.is_file(), "eval": ST["eval"]}


def install(app, ik_dir):
    """ik_dir: () → team-jax/IK 폴더(Path) 또는 None (server._ik_dir)."""
    ST["eval"] = _eval_summary()

    @app.get("/api/rl/info")
    def rl_info():
        i = _info(ik_dir)
        return dict(i, status=_status()) if i.get("ok") else i

    @app.get("/api/rl/status")
    def rl_status():
        return _status()

    @app.post("/api/rl/run")
    async def rl_run(req: Request):
        try:
            b = await req.json()
            x, y, yaw = float(b.get("x_cm", 0)), float(b.get("y_cm", 0)), float(b.get("yaw_deg", 0))
        except Exception:
            return JSONResponse({"ok": False, "error": "x, y, 각도는 숫자"})
        if not all(math.isfinite(v) for v in (x, y, yaw)) or abs(x) > 40 or abs(y) > 40 or abs(yaw) > 180:
            return JSONResponse({"ok": False, "error": "범위 밖: x·y ±40 cm, 각도 ±180° 안"})
        d = ik_dir()
        if _sim2sim(ik_dir) is None:
            return JSONResponse({"ok": False, "error": "sim2sim/ 폴더 없음 (JAX_IK_DIR)"})
        return JSONResponse(_start("run", [_python(), str(HERE / "rl_runner.py"), "run", "--ik", str(d),
                                           "--x", str(x), "--y", str(y), "--yaw", str(yaw), "--out", str(RESULT)], RUN_TIMEOUT))

    @app.post("/api/rl/eval")
    def rl_eval():
        s = _sim2sim(ik_dir)
        if s is None:
            return JSONResponse({"ok": False, "error": "sim2sim/ 폴더 없음 (JAX_IK_DIR)"})
        return JSONResponse(_start("eval", [_python(), str(s / "run_eval.py"), "--cond", "base", "--out", str(EVAL_DIR)], EVAL_TIMEOUT))

    @app.get("/api/rl/result")
    def rl_result():
        if not RESULT.is_file():
            return JSONResponse({"ok": False, "error": "아직 돌린 결과 없음"})
        return FileResponse(str(RESULT), media_type="application/json", headers={"Cache-Control": "no-store"})
