"""대시보드 '탑 쌓기' 탭의 서버 쪽: 실시간 인식의 큐브 여러 개 + 순서·탑 자리 → 걸음 목록 ('N층: N번 옮기기', 걸음마다 버튼).
사진 올리기 (POST /api/stack/detect) 는 없앴다 (2026-11-24) — 인식은 detect_frame(프레임 배열, 종이 자리) 을 부르는 쪽 (실시간 인식, 준비 중) 이 한다.
팔은 여기서 안 움직인다 — 계획·재생은 화면(static/stack.js)이 원래 있던 경로를 그대로 쓴다:
계획 = POST /api/svc/pick_dry/start (역기구학 탭의 계획, stack 칸으로 놓는 층·다른 큐브를 넘김), 재생 = 그 탭의 재생 버튼(확인창·검사 그대로).
server.py 는 install(app, log_dir) 한 줄만 가진다.

    GET  /api/stack/last     마지막 detect_frame 결과 {ok, cubes: [{id, x, y, yaw_deg, level, top_cm}], refs, warn, image, t}. 없으면 {ok: false, none: true}
    POST /api/stack/steps    {cubes, base: {x, y, yaw_deg}, order: [id…], level_unsure} → 걸음 목록 + 다음 걸음 (아래 steps, 상태 없음)
    GET/POST /api/stack/plan {order, base} — 화면의 순서·탑 자리 (logs/stack_plan.json, 페이지·서버를 다시 켜도 남음)
    POST /api/stack/placed   {level, id} — 이 화면이 놓은 기록 (힌트만, steps 의 placed)
    POST /api/stack/spot     {x, y, yaw_deg, levels, d} → 그 자리에 levels 층까지 놓을 수 있나 (계획기 point_check, 손 수직) {ok, text, levels: [{level, ok, why}]}
    GET  /api/stack/map?d=   놓을 수 있는 곳 지도 (2 cm 칸마다 바닥부터 몇 층까지 놓을 수 있나, 처음 한 번 계산 20~40초 → 책상 높이마다 기억)
    GET  /api/stack/sheet    큐브 마커 인쇄물 (phone_markers/stack_markers.pdf, 4x4 ID 0~4, 40 mm)

기준 종이 자리·큐브 높이는 휴대폰 카메라 탭의 설정(logs/phone_cam.json)을 같이 쓴다.
"""
import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path

from fastapi import Request
from fastapi.responses import FileResponse, JSONResponse

import phone_detect as PD

HERE = Path(__file__).resolve().parent
SHEET = HERE / "phone_markers" / "stack_markers.pdf"
TOWER_R = 0.025       # 탑 자리에서 이 안(m)에 보이는 큐브 = 탑의 맨 위 큐브 (놓인 큐브가 사진마다 몇 mm 씩 달리 보이고, 층마다 조금씩 어긋나도)
OVERLAP_R = 0.08      # 탑 자리에서 이 안인데 탑은 아닌 큐브 = 탑 자리와 겹침 (큐브 한 변 5 cm + 손가락 자리)
NEAR_M = 0.15         # 집을 큐브에서 이 안에 다른 큐브가 있으면 알림 (벌린 손가락이 닫히며 큐브 중심에서 12 cm 쯤까지 나온다)
OFF_TOL = 0.02        # 놓은 큐브가 계획 자리에서 이보다 멀면 멈춤 (m)
TILT_TOL = 15.0       # 놓은 큐브 윗면이 이보다 기울면 멈춤 (°, ArUco 마커 한 장으로 잰 값이라 몇 도 흔들린다)
MAX_LEVELS = 3        # 2026-11-24 사용자: 탑은 3층까지만 (순서는 바닥 큐브 포함 3개까지, 4층째는 계획하지 않음)
REACH_LEVELS = 3      # 기본 책상 높이(어깨 아래 0.4775 m)에서 팔이 놓을 수 있는 층 수 (4층째 = 책상 위 15 cm 에 놓기는 어디서도 안 됨, README)
LAST = {"r": None}
MAP_X = (-0.26, 0.22)   # 지도 범위 (실좌표 m), 칸 2 cm
MAP_Y = (-0.44, -0.16)
MAP_STEP = 0.02
_PL = {}                # 책상 높이 → 계획기 (MuJoCo 모델 하나씩; plan_home 처럼 서버 프로세스 안)
_PL_LOCK = threading.Lock()   # 계획기 하나가 MjData 를 같이 쓴다 → 한 번에 하나
_MAPS = {}


def _planner(d):
    import pick_plan as PP
    key = None if d is None else round(float(d), 4)
    if key not in _PL:
        if len(_PL) >= 2:
            _PL.clear()
        _PL[key] = PP.Planner(PP._find_ik(os.environ.get("JAX_IK_DIR") or None), PP._cmd_ranges(PP.REPO / "arm_control" / "calibration.json"),
                              d=d, grasp="vertical")
    return _PL[key], PP


def _d(v):
    v = "" if v is None else str(v).strip()
    return float(v) if v else None


def _place_ok(P, PP, x, y, yaw, level):
    """(x, y) 에 level 층째(0 = 책상)를 수직 손으로 놓을 수 있나 — 계획(pick_plan._plan_style)의 놓을 자리 판정과 같은 point_check."""
    s = P.s
    up = level * 2.0 * float(s.E.OBJECT_HALF_SIZE) + (PP.STACK_GAP if level else 0.0)
    ok, why, _ = P.point_check(P.np.asarray(s.to_sim([x, y, float(s.E.OBJECT_HALF_SIZE) - P.table_d + up])),
                               yaws=[yaw + s.R.SIM_FROM_REAL_YAW], style="vertical")
    return bool(ok), why


def spot_check(x, y, yaw, levels, d=None):
    with _PL_LOCK:
        try:
            P, PP = _planner(d)
        except Exception as e:
            return {"ok": False, "text": f"계획기를 못 만듦: {e}", "levels": []}
        rows = []
        for lv in range(levels):
            ok, why = _place_ok(P, PP, x, y, yaw, lv)
            rows.append({"level": lv, "ok": ok, "why": None if ok else why})
            if not ok:
                break
    bad = next((r for r in rows if not r["ok"]), None)
    if bad:
        return {"ok": False, "levels": rows, "can": bad["level"],
                "text": f"이 자리는 {bad['level'] + 1}층째를 못 놓음: {bad['why']}" + (f" ({bad['level']}층까지는 됨)" if bad["level"] else "")}
    return {"ok": True, "levels": rows, "can": levels, "text": f"이 자리에 {levels}층까지 놓을 수 있음 (손 수직)"}


def place_map(d=None):
    key = None if d is None else round(float(d), 4)
    if key in _MAPS:
        return _MAPS[key]
    with _PL_LOCK:
        if key in _MAPS:
            return _MAPS[key]
        t0 = time.time()
        P, PP = _planner(d)
        nx = int(round((MAP_X[1] - MAP_X[0]) / MAP_STEP)) + 1
        ny = int(round((MAP_Y[1] - MAP_Y[0]) / MAP_STEP)) + 1
        xs = [round(MAP_X[0] + i * MAP_STEP, 3) for i in range(nx)]
        ys = [round(MAP_Y[0] + j * MAP_STEP, 3) for j in range(ny)]
        rows = []
        for y in ys:
            row = ""
            for x in xs:
                n = 0
                while n < REACH_LEVELS + 1 and _place_ok(P, PP, x, y, 0.0, n)[0]:
                    n += 1
                row += str(n)
            rows.append(row)
        r = {"ok": True, "xs": xs, "ys": ys, "step": MAP_STEP, "rows": rows, "d": P.table_d, "seconds": round(time.time() - t0, 1)}
        if len(_MAPS) >= 3:
            _MAPS.clear()
        _MAPS[key] = r
        return r


def _f3(v):
    return f"{float(v):.3f}"


def _check_cube(c, ref, tol, what):
    """탑 속 큐브 c 가 ref (그 아래 큐브 또는 탑 자리) 에서 tol 안이고 반듯한가 → None 또는 이유 (예전 check_placed 의 어긋남·기울기 판정)."""
    off = math.hypot(c["x"] - ref[0], c["y"] - ref[1])
    if off > tol:
        return f"큐브 {c['id']} ({c['level'] + 1}층) 가 {what}에서 {off * 100:.1f} cm 어긋남 (허용 {tol * 100:.0f} cm)"
    if (c.get("tilt_deg") or 0.0) > TILT_TOL:
        return f"큐브 {c['id']} ({c['level'] + 1}층) 가 {c['tilt_deg']:.0f}° 기울어짐 (허용 {TILT_TOL:.0f}°)"
    return None


def steps(cubes, base, order, placed=None, level_unsure=False):
    """지금 인식 (cubes) + 탑 자리 (base) + 순서 (order, 비어도 됨) → 층마다 한 걸음. 상태를 안 가진다 (2026-11-24):
    누를 때마다 지금 보이는 것만으로 판단하므로 한 걸음 하고 다른 일을 하다 와도, 페이지·서버를 다시 켜도 된다.
    책상 위에 큐브가 하나씩만 있어도 된다 (사용자: 여러 개면 손이 부딪힌다) — 순서의 다른 큐브가 안 보여도 막지 않는다.

    탑 = 탑 자리 TOWER_R 안의 바닥 큐브 + OVERLAP_R 안의 쌓인 큐브. 탑 높이 = 보이는 맨 위 큐브의 층 + 1 (이것을 믿는다: 아래는 가려져도 됨).
    층 L 의 걸음: L < 높이 → 완료. L = 높이 → 다음 걸음: 순서에서 아직 안 쓴 첫 번호가 책상 위에 보이면 그것, 없으면 책상 위에 보이는
    다른 아무 큐브 ('보이는 큐브 N번', 순서를 비우면 늘 이것). placed = {층: 큐브 ID} 는 이 화면이 놓았던 기록 (힌트) — 가려진 층의 번호 표시와
    '그 큐브가 다른 층에 보임' 판정에만 쓴다. 탑 속 보이는 큐브: 아래 큐브 (안 보이면 탑 자리, 허용은 층마다 OFF_TOL 씩) 에서 2 cm 넘게 어긋남 ·
    15° 넘게 기울어짐 · 기록과 다른 층 → 문제 → 다음 걸음을 막는다 (이유).
    층·자리가 확실하지 않은 인식 (2026-11-24 검토: 2층 맨 위가 1층으로 읽히면 든 큐브가 기존 큐브를 누른다): level_unsure (프레임 전체) 이거나
    집을 큐브·탑 속 큐브가 unsure 면 다음 걸음을 막는다 (state blocked).
    돌려주는 것: {ok, text, bad?, done?, height, top, warn, steps: [{n, id, level, state done|bad|next|blocked|wait, text, why}], next: {n, cube, level, target, obstacles, params}}"""
    out = {"ok": False, "steps": [], "warn": [], "next": None, "height": 0, "top": None}
    try:
        bx, by, byaw = float(base["x"]), float(base["y"]), float(base.get("yaw_deg") or 0.0)
        order = [int(i) for i in (order or [])]
        hint = {int(k): int(v) for k, v in (placed or {}).items()}
        cs = [{"id": int(c["id"]), "x": float(c["x"]), "y": float(c["y"]), "yaw_deg": float(c["yaw_deg"]), "level": int(c["level"]),
               "tilt_deg": c.get("tilt_deg"), "unsure": bool(c.get("unsure"))} for c in cubes if c.get("id") is not None]
        if not all(math.isfinite(v) for v in (bx, by, byaw)) or len(order) != len(set(order)) or len(order) > 12:
            raise ValueError
    except (KeyError, TypeError, ValueError, AttributeError):
        return {**out, "text": "탑 자리·순서·큐브 값이 틀림"}
    if len(order) > MAX_LEVELS:
        return {**out, "text": f"순서는 {MAX_LEVELS}개까지 — 탑은 {MAX_LEVELS}층까지만 쌓는다 (지금 {len(order)}개)"}
    near = lambda c, r: math.hypot(c["x"] - bx, c["y"] - by) <= r   # noqa: E731
    tower = sorted([c for c in cs if (c["level"] == 0 and near(c, TOWER_R)) or (c["level"] > 0 and near(c, OVERLAP_R))], key=lambda c: c["level"])
    loose = [c for c in cs if c["level"] == 0 and c not in tower]
    odd = [c for c in cs if c["level"] > 0 and c not in tower]
    top = tower[-1] if tower else None
    height = top["level"] + 1 if top else 0
    at = {c["level"]: c for c in tower}
    probs = {}   # 층 → 이유
    for c in tower:
        lv = c["level"]
        hl = next((k for k, v in hint.items() if v == c["id"] and k < height), None)
        if hl is not None and hl != lv:
            probs.setdefault(lv, f"큐브 {c['id']} 가 탑 {lv + 1}층으로 보임 (놓은 곳은 {hl + 1}층) — 떨어졌거나 기울어짐")
            continue
        below = at.get(lv - 1)
        why = _check_cube(c, (below["x"], below["y"]) if below else (bx, by), OFF_TOL if below or lv == 0 else OFF_TOL * (lv + 1),
                          f"아래 큐브 {below['id']} " if below else "탑 자리")
        if why:
            probs.setdefault(lv, why)
    over = [c for c in loose if near(c, OVERLAP_R)]
    out.update(height=height, top=top)
    if odd:
        out["warn"].append("탑 밖에 겹쳐 있는 큐브: " + ", ".join(str(c["id"]) for c in odd) + " (그 자리는 피해 간다)")
    used = {c["id"] for c in tower} | {v for k, v in hint.items() if k < height and k not in at}
    rest = [i for i in order if i not in used]
    total = min(MAX_LEVELS, height + len(rest)) if order else MAX_LEVELS
    total = max(total, min(height, MAX_LEVELS))
    rows = []
    for L in range(height):
        c = at.get(L)
        i = c["id"] if c else hint.get(L)
        rows.append({"n": L + 1, "id": i, "level": L, "state": "bad" if L in probs else "done", "text": f"{L + 1}층: " + (f"{i}번" if i is not None else "큐브"),
                     "why": probs.get(L) or ("탑에 보임" if c else "탑 속 (위 큐브에 가려 안 보임)")})
    for k, L in enumerate(range(height, total)):
        i = rest[k] if k < len(rest) else None
        rows.append({"n": L + 1, "id": i, "level": L, "state": "wait", "text": f"{L + 1}층: " + (f"{i}번 옮기기" if i is not None else "보이는 큐브 옮기기"), "why": "앞 걸음 먼저"})
    stop = probs[min(probs)] if probs else None
    if height >= MAX_LEVELS or height >= total:
        if stop:
            return {**out, "ok": True, "steps": rows, "text": stop, "bad": True}
        return {**out, "ok": True, "done": True, "steps": rows, "text": f"탑 완성: {height}층" + (f" (맨 위 큐브 {top['id']})" if top else "")}
    r = rows[height]
    L = height
    c = next((q for q in loose if q["id"] == r["id"]), None) if r["id"] is not None else None
    sub = False
    if c is None:   # 순서의 다음 번호가 책상 위에 없음 → 보이는 다른 큐브 (순서의 남은 번호 먼저, 탑 자리와 겹친 것은 빼고)
        cand = [q for q in loose if q not in over]
        cand.sort(key=lambda q: (q["id"] not in rest, rest.index(q["id"]) if q["id"] in rest else 0, q["id"]))
        if cand:
            c, sub = cand[0], r["id"] is not None
            r.update(id=c["id"], text=f"{L + 1}층: " + ("보이는 큐브 " if sub or not order else "") + f"{c['id']}번 옮기기")
    why = stop
    if not why and c is None:
        why = (f"큐브 {r['id']} 가 책상 위에 안 보임" if r["id"] is not None else "책상 위에 보이는 큐브가 없음") + " — 탑 옆 (8 cm 밖) 에 하나 놓기"
        o = next((q for q in odd if q["id"] == r["id"]), None)
        if o:
            why = f"큐브 {o['id']} 가 다른 큐브 위에 있음 ({o['level'] + 1}층) — 책상에 내려놓기"
    if not why and level_unsure:
        why = "큐브 층이 확실하지 않은 프레임 — 기준 종이를 같이 보이게 하거나 큐브를 폰 가까이 (층이 틀리면 든 큐브가 탑을 누른다)"
    if not why:
        us = [q for q in tower + ([c] if c is not None else []) if q["unsure"]]
        if us:
            why = "큐브 " + ", ".join(str(q["id"]) for q in us) + " 의 층·자리가 확실하지 않음 (마커 한 변이 안 맞음) — 큐브를 반듯이·폰 가까이 두고 다시"
    if not why and over:
        why = f"탑 자리가 큐브 {', '.join(str(o['id']) for o in over)} 와 겹침 — 8 cm 넘게 띄우거나, 그 큐브 위에 쌓으려면 지도에서 그 큐브를 누름"
    if why:
        r.update(state="blocked", why=why)
        return {**out, "ok": True, "steps": rows, "text": why, "bad": True}
    r.update(state="next", why="")
    if sub:
        out["warn"].append(f"순서의 큐브 {rest[0]} 대신 보이는 큐브 {c['id']}")
    tx, ty, tyaw = (top["x"], top["y"], top["yaw_deg"]) if top else (bx, by, byaw)   # 실제로 보이는 맨 위 큐브 위에 (없으면 탑 자리)
    tyaw = (tyaw + 45.0) % 90.0 - 45.0   # 큐브는 90° 마다 같다
    obst = [[round(tx, 4), round(ty, 4), round(tyaw, 1), k] for k in range(height)]
    obst += [[round(o["x"], 4), round(o["y"], 4), round(o["yaw_deg"], 1), 0] for o in loose if o is not c]
    obst += [[round(o["x"], 4), round(o["y"], 4), round(o["yaw_deg"], 1), k] for o in odd for k in range(o["level"] + 1)]
    if [o for o in obst if math.hypot(o[0] - c["x"], o[1] - c["y"]) < NEAR_M]:
        out["warn"].append(f"큐브 {c['id']} 의 {NEAR_M * 100:.0f} cm 안에 다른 큐브 — 벌린 손가락이 닿을 수 있다 (계획이 검사한다, 걸리면 더 띄우기)")
    if L >= REACH_LEVELS:
        out["warn"].append(f"{L + 1}층째는 기본 책상 높이에서 팔이 못 닿는다 ({REACH_LEVELS}층까지) — 계획이 거절할 것")
    p = {"cube_x": _f3(c["x"]), "cube_y": _f3(c["y"]), "cube_yaw": f"{c['yaw_deg']:.1f}", "target_x": _f3(tx), "target_y": _f3(ty), "grasp": "vertical"}
    p["stack"] = json.dumps({"for": [float(p["cube_x"]), float(p["cube_y"]), float(p["target_x"]), float(p["target_y"])],
                             "level": L, "target_yaw": round(tyaw, 1), "obstacles": obst})
    out["next"] = {"n": r["n"], "cube": c, "level": L, "target": {"x": tx, "y": ty, "yaw_deg": tyaw}, "obstacles": obst, "params": p, "top": top, "height": height}
    return {**out, "ok": True, "steps": rows,
            "text": f"다음: {r['n']}층 — 큐브 {c['id']}" + (f" (큐브 {top['id']} 위)" if top else " (탑 자리 바닥)")}


def detect_frame(image, cfg=None, keep=True):
    """프레임 한 장 (BGR numpy 배열; JPEG·PNG 바이트도 됨) → 화면에 줄 dict {ok, cubes, refs, warn, image (data URL), t}.
    keep 이면 /api/stack/last 가 돌려줄 마지막 결과로 둔다. cfg = 기준 종이 자리 (logs/phone_cam.json, phone_api._load)."""
    import base64

    import cv2
    r = PD.detect_cubes(image, cfg)
    vis = r.pop("annotated", None)
    if vis is not None:
        k = 1280 / max(vis.shape[:2])
        if k < 1:
            vis = cv2.resize(vis, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            r["image"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
    r["t"] = time.time()
    if keep:
        LAST["r"] = r
    return r


def install(app, log_dir):
    @app.get("/api/stack/last")
    def _last():
        return JSONResponse(LAST["r"] or {"ok": False, "none": True})

    plan_file = Path(log_dir) / "stack_plan.json"

    def _saved():
        try:
            d = json.loads(plan_file.read_text())
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write(d):
        d["t"] = time.time()
        plan_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = plan_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(d))
        os.replace(tmp, plan_file)
        return d

    @app.post("/api/stack/steps")
    async def _steps(req: Request):
        try:
            b = await req.json()
            return JSONResponse(steps(b.get("cubes") or [], b.get("base") or {}, b.get("order") or [], _saved().get("placed"), bool(b.get("level_unsure"))))
        except Exception:
            return JSONResponse({"ok": False, "text": "요청 값이 틀림"}, status_code=400)

    @app.get("/api/stack/plan")
    def _plan_get():
        d = _saved()
        return JSONResponse({"ok": True, "order": d.get("order"), "base": d.get("base"), "placed": d.get("placed") or {}})

    @app.post("/api/stack/plan")
    async def _plan_set(req: Request):
        """순서·탑 자리 저장. 탑 자리가 바뀌면 놓은 기록 (placed) 은 지운다."""
        try:
            b = await req.json()
            order = [int(i) for i in b.get("order") or []][:12]
            base = b.get("base") or {}
            base = {"x": float(base["x"]), "y": float(base["y"]), "yaw_deg": float(base.get("yaw_deg") or 0.0)}
            if not all(math.isfinite(v) and abs(v) <= 360 for v in base.values()):
                raise ValueError
        except Exception:
            return JSONResponse({"ok": False, "text": "순서·탑 자리 값이 틀림"}, status_code=400)
        old = _saved()
        placed = old.get("placed") or {}
        if old.get("base") != base:
            placed = {}
        return JSONResponse({"ok": True, **_write({"order": order, "base": base, "placed": placed})})

    @app.post("/api/stack/placed")
    async def _placed(req: Request):
        """화면이 걸음을 끝까지 재생했다: {level, id} — 힌트 기록 (가려진 층의 번호 표시, 다른 층에 보이면 막기)."""
        try:
            b = await req.json()
            lv, i = int(b["level"]), int(b["id"])
            if not 0 <= lv < 12:
                raise ValueError
        except Exception:
            return JSONResponse({"ok": False}, status_code=400)
        d = _saved()
        pl = {k: v for k, v in (d.get("placed") or {}).items() if int(k) < lv and v != i}
        pl[str(lv)] = i
        d["placed"] = pl
        return JSONResponse({"ok": True, **_write(d)})

    @app.post("/api/stack/spot")
    async def _spot(req: Request):
        try:
            b = await req.json()
            x, y, yaw, n, d = float(b["x"]), float(b["y"]), float(b.get("yaw_deg") or 0.0), int(b.get("levels") or 1), _d(b.get("d"))
            if not all(math.isfinite(v) for v in (x, y, yaw)) or max(abs(x), abs(y)) > 1.0 or not 1 <= n <= 8:
                raise ValueError
        except Exception:
            return JSONResponse({"ok": False, "text": "탑 자리 x, y (m) 숫자"}, status_code=400)
        try:
            return JSONResponse(await asyncio.to_thread(spot_check, x, y, yaw, n, d))
        except Exception as e:
            return JSONResponse({"ok": False, "text": f"확인 못 함: {type(e).__name__}: {e}"})

    @app.get("/api/stack/map")
    async def _map(d: str = ""):
        try:
            return JSONResponse(await asyncio.to_thread(place_map, _d(d)))
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"})

    @app.get("/api/stack/sheet")
    def _sheet():
        if not SHEET.is_file():
            return JSONResponse({"ok": False, "error": "인쇄물 없음 — dashboard/tools/make_phone_markers.py"}, status_code=404)
        return FileResponse(str(SHEET), media_type="application/pdf", filename="stack_markers.pdf")
