"""대시보드 '휴대폰 카메라' 탭의 서버 쪽: 휴대폰 사진 → 큐브 자리 (phone_detect.py, ArUco). 팔은 여기서 안 움직인다.

server.py 는 install(app, log_dir) 한 줄만 가진다. 계획·재생은 화면(phone.js)이 원래 있던 경로
(/api/svc/pick_dry/start → 역기구학 탭의 재생 버튼과 같은 확인창)를 그대로 쓴다.

    POST /api/phone/detect   본문 = 사진 바이트 (JPEG/PNG, multipart 아님)  → {ok, cube{x,y,yaw_deg}, refs, warn, image(data URL), …}
    GET  /api/phone/config   기준 종이가 놓인 자리 {x, y, rot_deg, cube_h}
    POST /api/phone/config   {x, y, rot_deg, cube_h} 저장 (logs/phone_cam.json)
    GET  /api/phone/sheet    인쇄물 PDF (dashboard/phone_markers/phone_markers.pdf)
"""
import asyncio
import base64
import json
import math
from pathlib import Path

from fastapi import Request
from fastapi.responses import FileResponse, JSONResponse

import phone_detect as PD

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


def _run(data, cfg):
    import cv2
    r = PD.detect(data, cfg)
    vis = r.pop("annotated", None)
    if vis is not None:
        k = 1280 / max(vis.shape[:2])
        if k < 1:
            vis = cv2.resize(vis, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            r["image"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
    return r


def install(app, log_dir):
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

    @app.get("/api/phone/sheet")
    def _sheet():
        if not SHEET.is_file():
            return JSONResponse({"ok": False, "error": "인쇄물 없음 — dashboard/tools/make_phone_markers.py"}, status_code=404)
        return FileResponse(str(SHEET), media_type="application/pdf", filename="phone_markers.pdf")
