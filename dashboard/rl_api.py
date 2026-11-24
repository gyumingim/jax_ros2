"""'강화학습' 탭의 서버 쪽: 팀원이 학습한 집기 정책(team-jax/IK sim2sim, MuJoCo)을 자식 프로세스로 돌리고 결과를 준다.
들기 = 그 정책, 내리기 = 우리 IK 계획(pick_plan.plan_place_from) — 둘 다 자식 프로세스(rl_runner.py)에서.

계산은 시뮬만. 팔에는 '실물로 재생' / '가상 팔로 재생'(아래 play_*)에서만 보낸다. server.py 는 install(app, ik_dir, arm) 한 줄만 가진다.

    GET  /api/rl/info                       놓는 범위 표, 27곳, 지금 상태
    POST /api/rl/run   {x_cm, y_cm, yaw_deg}   한 곳 돌리기 (한 번에 하나, 몇 초)
    POST /api/rl/eval                       27곳 평가 (run_eval.py --cond base, 1분쯤). 결과는 logs/rl/eval — IK 레포는 안 건드림
    GET  /api/rl/status                     {busy, kind, error, run_id, eval}
    POST /api/rl/place {x, y, yaw_deg}       '역기구학으로 내리기': 마지막 들기(성공) 끝 자세에서 이어 우리 IK 계획으로 놓고 HOME.
                                            x, y = 놓을 자리 실좌표 m (역기구학 탭·3D 틀), 화면 재생만 — 팔에는 안 보냄
    GET  /api/rl/result                     마지막 결과: 내리기를 했으면 들기 + 내리기 (rl_runner.py place), 아니면 들기만 (run)
    GET  /api/rl/traj                       팔 재생용 궤적 요약 {ok, error, mtime, frames, sim_s, play_s, stretch} (궤적은 안 보냄)
    POST /api/rl/play_real    {force, mtime}  들기 + 내리기를 실물 팔로 (직접 CAN 모드). direct_api.play_real 그대로 = IK 탭 실물 재생과 같은 검사
    POST /api/rl/play_virtual {mtime}         같은 궤적을 가상 팔로 (가상 모드). server._play_virtual 그대로

팔 재생 (2026-11-24): 시뮬 결과를 그대로 되풀이(열린 루프)한다. 실물 손엔 정책 관측 37–42 의 손가락 압력 센서가 없고 큐브를 매 순간
따라가지도 않는다. 궤적 = 마지막 last_place.json(들기 + 내리기)의 프레임을 IK 계획과 같은 꼴(관절 13개 이름·순서, rad, dt 0.02)로
바꾼 것 → logs/rl/rl_traj.json (자기 mtime). 엄지 받침(finger_rotation)은 궤적에 안 넣는다 — 아래 build_traj.
"""
import asyncio
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
PLACE = OUT / "last_place.json"      # 들기 결과 + 내리기 프레임 (들기를 새로 돌리면 지운다)
EVAL_DIR = OUT / "eval"
TRAJ = OUT / "rl_traj.json"         # 팔 재생용 궤적 (last_place.json 에서 만든다. 그보다 옛것이면 다시 만든다)
RUN_TIMEOUT, EVAL_TIMEOUT, PLACE_TIMEOUT = 120, 900, 300

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
    return {"done": d["aggregate"]["done"], "runs": d["aggregate"]["runs"], "rows": rows, "mtime": p.stat().st_mtime,
            "policy": d["aggregate"].get("policy") or "grasp_lift_23025"}   # 옛 결과 파일에는 정책 이름이 없다 (23025 때)


def _job(kind, cmd, timeout):
    out, err = _call(cmd, timeout)
    with LOCK:
        ST.update(busy=False, error="" if out is not None else err)
        if out is not None and kind in ("run", "place"):
            ST["run_id"] += 1
        if kind == "eval":
            ST["eval"] = _eval_summary()


def _start(kind, cmd, timeout):
    with LOCK:
        if ST["busy"]:
            return {"ok": False, "error": "이미 돌리는 중 (" + {"eval": "27곳 평가", "place": "내리기 계획"}.get(ST["kind"], "들기") + ") — 끝난 뒤 다시"}
        ST.update(busy=True, kind=kind, since=time.time(), error="")
    OUT.mkdir(parents=True, exist_ok=True)
    if kind == "run":
        PLACE.unlink(missing_ok=True)
        TRAJ.unlink(missing_ok=True)
    elif kind == "place":
        TRAJ.unlink(missing_ok=True)
    threading.Thread(target=_job, args=(kind, cmd, timeout), daemon=True).start()
    return {"ok": True}


def _status():
    return {"ok": True, "busy": ST["busy"], "kind": ST["kind"], "elapsed": round(time.time() - ST["since"], 1) if ST["busy"] else None,
            "error": ST["error"], "run_id": ST["run_id"], "has_result": RESULT.is_file(), "eval": ST["eval"]}


# ---------- 팔 재생 (실물 / 가상): 들기 + 내리기 궤적 ----------
DT = 0.02          # IK 계획(pick_plan.DT)과 같은 50 Hz
RL_SPEED = 30.0    # °/s. 다시 맞출 때 관절 하나가 한 프레임에 이것보다 빨리 안 가게 (IK 탭 기본 max_speed 와 같음)
NAMES = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow", "wrist_pitch", "wrist_yaw", "hand_roll",
         "finger1_joint1", "finger1_joint2", "finger2_joint1", "finger2_joint2", "finger3_joint1", "finger3_joint2"]   # pick_plan 궤적과 같은 순서
FINGER_HOME = [-0.26, 0.0, -0.26, 0.0, -0.26, 0.0]   # HOME 의 손가락 (IK env FINGER_OPEN = pick_plan open_exit), 내리기 끝 프레임이 없을 때만
OLDEST_OK = 23025
BUILD = 2          # build_traj 규칙 판 — 바꾸면 올린다 (옛 rl_traj.json 을 다시 만들게)  # 정책 grasp_lift_<번호>: 이 번호 이하(옛 사양 11-08)는 실물 재생 거절


def _policy_no(name):
    try:
        return int(str(name).rsplit("_", 1)[-1])
    except ValueError:
        return None


def build_traj(d, speed=RL_SPEED):
    """last_place.json → 재생 궤적 (IK 계획과 같은 꼴). 거절할 결과면 (None, 이유).

    - 관절: 대시보드 이름 13개를 pick_plan 순서로, rad. 손가락 = 체인(정책)의 손가락 관절값 그대로 — sim2sim 장면과 arm.xml 은 부호·0 이 같고
      (rl_runner.JOINT_MAP, 몸체 원점 0.00 mm), calibration.json 은 arm.xml 각 → 서보이므로 그대로 보낸다. 25550 의 쥐는 힘은 정책이 낸
      손가락 목표각일 뿐 — 실물에선 압력 센서 없이 그 각으로만 간다.
    - 엄지 받침(finger_rotation, F_ROT): 넣지 않는다 = 지금(보정 0 = 모델의 −180°) 그대로. 체인의 rot 은 늘어진 자세 0 에서 −170° 까지
      돈다 (−180° 기준으로 바꾸면 F_ROT +180 → +10°). F_ROT 의 방향·0 은 아직 확인 안 됐고(verified false) 180° 를 휘두르게 되므로,
      확인 전엔 IK 탭 궤적처럼 빼 둔다. 쥘 때 차이는 10° (체인 −170°, 실물 −180°).
    - 시작: 'HOME 으로 가기' 끝 자세(손가락 벌림)에서 손가락만 0 으로 오므리는 1 초를 붙인 뒤 체인 첫 프레임(모든 관절 0)부터.
    - 시간: 30 Hz 프레임 사이마다 가장 빠른 관절이 speed °/s 를 넘으면 그 사이만 늘이고(정책 구간은 60 Hz 로 빠르다), DT 로 다시 뽑는다.
      재생(play.schedule)은 이 위에 max_speed 로 한 번 더 늦춘다 (보통 1배)."""
    s = d.get("summary") or {}
    pl = d.get("place")
    no = _policy_no(d.get("policy"))
    if no is None or no <= OLDEST_OK:
        return None, f"옛 정책 결과 ({d.get('policy') or '정책 이름 없음 = 23025 때'}) — '강화학습으로 들기'부터 다시"
    if not (s.get("done") and s.get("still_up_at_end", True)):
        return None, "들기가 성공하지 않은 결과 — 실물·가상 팔로 재생 안 함"
    if not pl:
        return None, "내리기 계획이 없음 — '역기구학으로 내리기' 먼저"
    if not pl.get("ok") or pl.get("status") != "playable":
        return None, "내리기 계획이 실패한 결과: " + str(pl.get("reason") or pl.get("status"))
    names = d["joint_names"]
    if sorted(names) != sorted(NAMES):
        return None, "관절 이름이 다름: " + ", ".join(names)
    col = [names.index(n) for n in NAMES]
    q = [[float(f[c]) for c in col] for f in d["q"]]
    t_sim = [float(v) for v in d["t"]]
    if len(q) < 2 or any(abs(v) > 1e-6 for v in q[0]):
        return None, "첫 프레임이 늘어진 자세(모든 관절 0 = HOME)가 아님"
    # 시작 = 'HOME 으로 가기'(pick_plan.plan_home)가 끝나는 자세: 팔 0 + 손가락 첫 마디 벌림(open_exit, −14.9°). 체인은 손가락 0 에서
    # 시작하므로 그 자리(책상 옆에 늘어진 손, 빈 곳)에서 손가락만 0 으로 오므리는 1 초를 앞에 붙인다 — 안 붙이면 HOME 뒤에도 시작 자세
    # 검사(5°)에 걸린다. 벌림 값은 내리기 끝 프레임(같은 계획기의 HOME 도착 = open_exit)에서 읽는다
    fin = q[-1][7:] if all(abs(v) < 1e-6 for v in q[-1][:7]) else list(FINGER_HOME)
    if any(abs(v) > 1e-6 for v in fin):
        q.insert(0, [0.0] * 7 + [float(v) for v in fin])
        t_sim.insert(0, t_sim[0] - 1.0)
    # 시간 다시 맞추기: 사이 i 의 길이 = max(원래, 가장 큰 움직임 / speed)
    w = math.radians(speed)
    T = [0.0]
    stretch = 1.0
    for i in range(len(q) - 1):
        h = max(t_sim[i + 1] - t_sim[i], 1e-6)
        need = max(abs(b - a) for a, b in zip(q[i], q[i + 1])) / w
        T.append(T[-1] + max(h, need))
        stretch = max(stretch, need / h)
    n = int(math.ceil(T[-1] / DT - 1e-9))
    pos, i = [], 0
    for k in range(n + 1):
        tk = min(k * DT, T[-1])
        while i < len(T) - 2 and T[i + 1] < tk:
            i += 1
        u = (tk - T[i]) / ((T[i + 1] - T[i]) or 1.0)
        u = min(max(u, 0.0), 1.0)
        pos.append([round(a + (b - a) * u, 6) for a, b in zip(q[i], q[i + 1])])
    pos[-1] = [round(v, 6) for v in q[-1]]
    peak = max((abs(math.degrees(b - a)) / DT for p0, p1 in zip(pos, pos[1:]) for a, b in zip(p0, p1)), default=0.0)
    sim_s, play_s = t_sim[-1] - t_sim[0], n * DT
    return {"planner": "analytic", "source": "rl", "kind": "rl_lift_place",   # planner analytic: direct_api.analytic_fatal 이 status·checks 를 본다
            "status": "playable", "playable": True, "delivered_in_sim": True,
            "reason": f"강화학습 들기({d.get('policy')}) + 역기구학 내리기, 시뮬 결과 그대로 (열린 루프)",
            "joint_names": NAMES, "dt": DT, "unit": "rad", "positions": pos, "peak_speed_deg_s": round(peak, 1),
            "checks": pl.get("checks") or {},     # 내리기 계획의 joint_space_fallback 을 analytic_fatal 이 본다
            "rl": {"policy": d.get("policy"), "spot": d.get("spot"), "target": pl.get("target"), "speed_deg_s": speed,
                   "sim_s": round(sim_s, 2), "play_s": round(play_s, 2), "factor": round(play_s / sim_s, 2) if sim_s > 0 else None,
                   "stretch_max": round(stretch, 2), "lift_s": round(d["t"][pl["start_frame"]] - d["t"][0], 2) if "start_frame" in pl else None}}, ""


def _traj():
    """direct_api.play_real / server._play_virtual 이 부르는 꼴: {ok, traj, mtime, params} 또는 {ok False, error}.
    rl_traj.json 이 last_place.json 보다 옛것이면 다시 만든다 (mtime 이 바뀐다 → 화면이 보고 확인한 것과 다르면 play_real 이 거절)."""
    with LOCK:
        if ST["busy"] and ST["kind"] in ("run", "place"):
            return {"ok": False, "error": "강화학습 계산 중 — 끝난 뒤"}
        if not PLACE.is_file():
            TRAJ.unlink(missing_ok=True)
            return {"ok": False, "error": "들기 + 내리기 결과가 없음 — '강화학습으로 들기' → '역기구학으로 내리기' 먼저"}
        try:
            d = json.loads(TRAJ.read_text(encoding="utf-8")) if TRAJ.is_file() and TRAJ.stat().st_mtime >= PLACE.stat().st_mtime else {}
            if d.get("build") != BUILD:      # 없음 · last_place.json 보다 옛것 · 만드는 규칙이 바뀜 → 다시 만든다
                tr, why = build_traj(json.loads(PLACE.read_text(encoding="utf-8")))
                d = dict(tr or {"error": why}, build=BUILD)
                tmp = TRAJ.with_suffix(".tmp")
                tmp.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
                os.replace(tmp, TRAJ)
        except Exception as e:
            return {"ok": False, "error": f"재생 궤적을 못 만듦: {type(e).__name__}: {e}"}
        if d.get("error") or not d.get("positions"):
            return {"ok": False, "error": d.get("error") or "궤적 없음"}
        return {"ok": True, "traj": d, "mtime": TRAJ.stat().st_mtime, "params": {"max_speed": RL_SPEED}}


def _same_mtime(b, t):
    m = b.get("mtime")
    try:
        return m is not None and abs(float(m) - float(t["mtime"])) <= 1e-6
    except (TypeError, ValueError):
        return False


def install(app, ik_dir, arm=None):
    """ik_dir: () → team-jax/IK 폴더(Path) 또는 None (server._ik_dir).
    arm: {"mode": () → "direct"|"virtual"|"ros", "play_virtual": server._play_virtual} — 없으면 팔 재생 경로 없음."""
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

    @app.post("/api/rl/place")
    async def rl_place(req: Request):
        try:
            b = await req.json()
            x, y, yaw = float(b.get("x", 0)), float(b.get("y", 0)), float(b.get("yaw_deg", 0))
        except Exception:
            return JSONResponse({"ok": False, "error": "x, y, 각도는 숫자"})
        if not all(math.isfinite(v) for v in (x, y, yaw)) or abs(x) > 1.5 or abs(y) > 1.5 or abs(yaw) > 180:
            return JSONResponse({"ok": False, "error": "범위 밖: x·y ±1.5 m, 각도 ±180° 안"})
        d = ik_dir()
        if _sim2sim(ik_dir) is None:
            return JSONResponse({"ok": False, "error": "sim2sim/ 폴더 없음 (JAX_IK_DIR)"})
        if not RESULT.is_file():
            return JSONResponse({"ok": False, "error": "먼저 '강화학습으로 들기'"})
        return JSONResponse(_start("place", [_python(), str(HERE / "rl_runner.py"), "place", "--ik", str(d), "--lift", str(RESULT),
                                             "--tx", str(x), "--ty", str(y), "--tyaw", str(yaw), "--out", str(PLACE)], PLACE_TIMEOUT))

    @app.post("/api/rl/eval")
    def rl_eval():
        s = _sim2sim(ik_dir)
        if s is None:
            return JSONResponse({"ok": False, "error": "sim2sim/ 폴더 없음 (JAX_IK_DIR)"})
        return JSONResponse(_start("eval", [_python(), str(s / "run_eval.py"), "--cond", "base", "--out", str(EVAL_DIR)], EVAL_TIMEOUT))

    @app.get("/api/rl/traj")
    def rl_traj():
        t = _traj()
        if not t["ok"]:
            return JSONResponse({"ok": False, "error": t["error"]})
        r = t["traj"]["rl"]
        return JSONResponse(dict(r, ok=True, mtime=t["mtime"], frames=len(t["traj"]["positions"]), peak_speed_deg_s=t["traj"]["peak_speed_deg_s"]))

    async def body(req):
        try:
            b = await req.json()
            return b if isinstance(b, dict) else {}
        except Exception:
            return {}

    @app.post("/api/rl/play_real")
    async def rl_play_real(req: Request):
        if not arm or arm["mode"]() != "direct":
            return JSONResponse({"ok": False, "error": "실물(직접 CAN) 모드에서만"})
        b = await body(req)
        if b.get("mtime") is None:   # IK 탭과 달리 mtime 필수: 화면이 보여 준 결과만
            return JSONResponse({"ok": False, "error": "mtime 없음 — 화면에서 다시"})
        import direct_api
        return JSONResponse(await asyncio.to_thread(direct_api.play_real, _traj, b))

    @app.post("/api/rl/play_virtual")
    async def rl_play_virtual(req: Request):
        if not arm or arm["mode"]() != "virtual":
            return JSONResponse({"ok": False, "error": "가상 모드에서만"})
        b = await body(req)
        t = _traj()
        if not t["ok"]:
            return JSONResponse({"ok": False, "error": t["error"]})
        if not _same_mtime(b, t):
            return JSONResponse({"ok": False, "error": "결과가 바뀜 — 화면의 결과를 확인하고 다시 재생"})
        import direct_api
        fatal = direct_api.analytic_fatal(t["traj"])
        if fatal:
            return JSONResponse({"ok": False, "error": f"재생 거절: {fatal}"})
        return JSONResponse(await asyncio.to_thread(arm["play_virtual"], lambda: t))

    @app.get("/api/rl/result")
    def rl_result():
        if not RESULT.is_file():
            return JSONResponse({"ok": False, "error": "아직 돌린 결과 없음"})
        f = PLACE if PLACE.is_file() and PLACE.stat().st_mtime >= RESULT.stat().st_mtime else RESULT
        return FileResponse(str(f), media_type="application/json", headers={"Cache-Control": "no-store"})
