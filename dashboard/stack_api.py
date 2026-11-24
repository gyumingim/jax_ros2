"""대시보드 '탑 쌓기' 탭의 서버 쪽: 휴대폰 사진 → 큐브 여러 개 (phone_detect.detect_cubes, 큐브마다 다른 ArUco) → 다음에 옮길 큐브 한 걸음.
팔은 여기서 안 움직인다 — 계획·재생은 화면(static/stack.js)이 원래 있던 경로를 그대로 쓴다:
계획 = POST /api/svc/pick_dry/start (역기구학 탭의 계획, stack 칸으로 놓는 층·다른 큐브를 넘김), 재생 = 그 탭의 재생 버튼(확인창·검사 그대로).
server.py 는 install(app, log_dir) 한 줄만 가진다.

    POST /api/stack/detect   본문 = 사진 바이트 (JPEG/PNG) → {ok, cubes: [{id, x, y, yaw_deg, level, top_cm}], refs, warn, image, t}
    GET  /api/stack/last     마지막 detect 결과 (폰으로 찍고 PC 화면에서도 보게). 없으면 {ok: false}
    POST /api/stack/next     {cubes, base: {x, y, yaw_deg, id?}, order: [id…]} → 한 걸음 (아래 next_step)
    POST /api/stack/spot     {x, y, yaw_deg, levels, d} → 그 자리에 levels 층까지 놓을 수 있나 (계획기 point_check, 손 수직) {ok, text, levels: [{level, ok, why}]}
    GET  /api/stack/map?d=   놓을 수 있는 곳 지도 (2 cm 칸마다 바닥부터 몇 층까지 놓을 수 있나, 처음 한 번 계산 20~40초 → 책상 높이마다 기억)
    GET  /api/stack/sheet    큐브 마커 인쇄물 (phone_markers/stack_markers.pdf, ID 10~15)

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

import phone_api
import phone_detect as PD

HERE = Path(__file__).resolve().parent
SHEET = HERE / "phone_markers" / "stack_markers.pdf"
TOWER_R = 0.025       # 탑 자리에서 이 안(m)에 보이는 큐브 = 탑의 맨 위 큐브 (놓인 큐브가 사진마다 몇 mm 씩 달리 보이고, 층마다 조금씩 어긋나도)
OVERLAP_R = 0.08      # 탑 자리에서 이 안인데 탑은 아닌 큐브 = 탑 자리와 겹침 (큐브 한 변 5 cm + 손가락 자리)
NEAR_M = 0.15         # 집을 큐브에서 이 안에 다른 큐브가 있으면 알림 (벌린 손가락이 닫히며 큐브 중심에서 12 cm 쯤까지 나온다)
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


def next_step(cubes, base, order):
    """사진에서 찾은 큐브들 + 탑 자리(base) + 쌓는 순서(order = 이 탑에 쓰는 큐브 ID 전부, 바닥 큐브 포함) → 다음 한 걸음.

    탑의 높이는 사진이 정한다: base 자리(3 cm 안)에 보이는 큐브 중 가장 높은 것이 맨 위 (그 아래 큐브는 가려서 안 보인다).
    안 보이는 order 의 큐브 = 탑 속에 깔린 것으로 본다 → 그 수와 사진의 탑 높이가 안 맞으면 (마커를 못 찾음·큐브가 떨어짐) 멈춘다.
    돌려주는 것: {ok, done, text, height(지금 탑 층 수), cube(다음에 집을 것), level(놓을 층, 0 = 책상), target, params(pick_dry 칸), warn}."""
    out = {"ok": False, "done": False, "warn": []}
    try:
        bx, by, byaw = float(base["x"]), float(base["y"]), float(base.get("yaw_deg") or 0.0)
        order = [int(i) for i in order]
        cs = [{"id": int(c["id"]), "x": float(c["x"]), "y": float(c["y"]), "yaw_deg": float(c["yaw_deg"]), "level": int(c["level"])} for c in cubes]
        if not all(math.isfinite(v) for v in (bx, by, byaw)) or len(order) != len(set(order)) or not 1 <= len(order) <= 12:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {**out, "text": "탑 자리·순서·큐브 값이 틀림"}
    tower = [c for c in cs if math.hypot(c["x"] - bx, c["y"] - by) <= TOWER_R]
    top = max(tower, key=lambda c: c["level"]) if tower else None
    height = top["level"] + 1 if top else 0
    seen = {c["id"] for c in cs}
    hidden = [i for i in order if i not in seen]
    loose = [c for c in cs if c not in tower and c["level"] == 0]
    odd = [c for c in cs if c not in tower and c["level"] > 0]
    out.update(height=height, top=top, hidden=hidden)
    inside = len(hidden) + (1 if top else 0)     # order 로 설명되는 탑 큐브 수
    if top and top["id"] not in order:
        out["warn"].append(f"탑 맨 위 큐브 {top['id']} 는 순서에 없는 큐브")
    if inside > height:
        miss = ", ".join(map(str, hidden))
        return {**out, "text": f"큐브 {miss} 가 사진에 안 보임 — 탑은 {height}층으로 보이는데 순서의 큐브 {len(order)}개 중 {len(hidden)}개가 없다. "
                               "가려지지 않게 다시 찍기 (큐브가 떨어졌으면 '처음부터')"}
    if inside < height:
        out["warn"].append(f"탑이 {height}층으로 보이는데 순서의 큐브로는 {inside}층 — 순서에 없는 큐브가 깔려 있거나 높이를 잘못 봄")
    if odd:
        out["warn"].append("탑 밖에 겹쳐 있는 큐브: " + ", ".join(str(c["id"]) for c in odd) + " (그 자리는 피해 간다)")
    over = [c for c in loose + odd if math.hypot(c["x"] - bx, c["y"] - by) <= OVERLAP_R]
    if over:
        return {**out, "text": f"탑 자리가 큐브 {', '.join(str(c['id']) for c in over)} 와 겹침 — 그 큐브 위에 쌓으려면 지도에서 그 큐브를 누르고, 아니면 8 cm 넘게 띄우기"}
    if len(order) > MAX_LEVELS:
        return {**out, "text": f"순서는 {MAX_LEVELS}개까지 — 탑은 {MAX_LEVELS}층까지만 쌓는다 (지금 {len(order)}개)"}
    if height >= MAX_LEVELS:
        return {**out, "ok": True, "done": True, "text": f"탑 완성: {height}층 (최대 {MAX_LEVELS}층)"}
    nxt = next((c for i in order for c in loose if c["id"] == i), None)
    if nxt is None:
        if height == 0:
            return {**out, "text": "쌓을 큐브가 사진에 없음 — 순서의 큐브가 책상 위에 보이게 다시 찍기"}
        return {**out, "ok": True, "done": True, "text": f"탑 완성: {height}층 (맨 위 큐브 {top['id']})"}
    level = height
    tx, ty, tyaw = (top["x"], top["y"], top["yaw_deg"]) if top else (bx, by, byaw)   # 실제로 놓인 맨 위 큐브 위에 (없으면 탑 자리)
    obst = [[round(tx, 4), round(ty, 4), round(tyaw, 1), k] for k in range(height)]
    obst += [[round(c["x"], 4), round(c["y"], 4), round(c["yaw_deg"], 1), 0] for c in loose if c is not nxt]
    obst += [[round(c["x"], 4), round(c["y"], 4), round(c["yaw_deg"], 1), k] for c in odd for k in range(c["level"] + 1)]
    near = [o for o in obst if math.hypot(o[0] - nxt["x"], o[1] - nxt["y"]) < NEAR_M]
    if near:
        out["warn"].append(f"큐브 {nxt['id']} 의 {NEAR_M * 100:.0f} cm 안에 다른 큐브 — 벌린 손가락이 닿을 수 있다 (계획이 검사한다, 걸리면 더 띄우기)")
    if level >= REACH_LEVELS:
        out["warn"].append(f"{level + 1}층째는 기본 책상 높이에서 팔이 못 닿는다 ({REACH_LEVELS}층까지) — 계획이 거절할 것")
    p = {"cube_x": _f3(nxt["x"]), "cube_y": _f3(nxt["y"]), "cube_yaw": f"{nxt['yaw_deg']:.1f}", "target_x": _f3(tx), "target_y": _f3(ty), "grasp": "vertical"}
    p["stack"] = json.dumps({"for": [float(p["cube_x"]), float(p["cube_y"]), float(p["target_x"]), float(p["target_y"])],
                             "level": level, "target_yaw": round(tyaw, 1), "obstacles": obst})
    return {**out, "ok": True, "cube": nxt, "level": level, "target": {"x": tx, "y": ty, "yaw_deg": tyaw}, "obstacles": obst, "params": p,
            "text": f"다음: 큐브 {nxt['id']} → {level + 1}층" + (f" (큐브 {top['id']} 위)" if top else " (탑 자리 바닥)")}


def install(app, log_dir):
    cfg_file = Path(log_dir) / "phone_cam.json"

    def run(data):
        import base64

        import cv2
        r = PD.detect_cubes(data, phone_api._load(cfg_file))
        vis = r.pop("annotated", None)
        if vis is not None:
            k = 1280 / max(vis.shape[:2])
            if k < 1:
                vis = cv2.resize(vis, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                r["image"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
        r["t"] = time.time()
        return r

    @app.post("/api/stack/detect")
    async def _detect(req: Request):
        data = await req.body()
        if not data or len(data) > phone_api.MAX_BYTES:
            return JSONResponse({"ok": False, "reason": "사진이 없거나 너무 큼 (25 MB 까지)"}, status_code=400)
        try:
            r = await asyncio.to_thread(run, data)
        except Exception as e:
            return JSONResponse({"ok": False, "reason": f"찾는 중 오류: {type(e).__name__}: {e}"})
        LAST["r"] = r
        return JSONResponse(r)

    @app.get("/api/stack/last")
    def _last():
        return JSONResponse(LAST["r"] or {"ok": False, "none": True})

    @app.post("/api/stack/next")
    async def _next(req: Request):
        try:
            b = await req.json()
            return JSONResponse(next_step(b.get("cubes") or [], b.get("base") or {}, b.get("order") or []))
        except Exception:
            return JSONResponse({"ok": False, "text": "요청 값이 틀림"}, status_code=400)

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
