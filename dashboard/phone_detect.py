"""휴대폰 사진 한 장 → 큐브 자리 (x, y, yaw) — ArUco 만 쓴다 (OpenCV 4.5 옛 API: Dictionary_get / detectMarkers).

    기준 종이 (A4 가로, dashboard/phone_markers/phone_markers.pdf 1쪽): ArUco 5x5 ID 0~5, 한 변 32 mm. 책상에 붙인다.
    큐브 윗면: ArUco 5x5 ID 10, 한 변 40 mm (같은 PDF 2쪽).

좌표 = 로봇 실좌표 (pick_plan 과 같음): 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (책상은 −y), 위에서 본 yaw 반시계.
종이 좌표 (mm): 원점 = 종이 한가운데 십자, +x = 종이 오른쪽, +y = 종이 위쪽 ('로봇 쪽' 화살표). 종이를 로봇과 나란히 놓으면
(rot 0) 종이 +x = 로봇 +x, 종이 +y = 로봇 +y. 종이 가운데가 놓인 자리 (x, y, rot) 는 cfg 로 준다.

큐브 윗면은 책상보다 cube_h (5 cm) 높다 → 책상 평면 호모그래피만 쓰면 카메라 반대쪽으로 밀려 보인다 (비스듬히 찍으면 몇 cm).
그래서 기준 마커로 카메라 자세를 구해 (solvePnP; 초점거리는 훑어서 재투영 오차가 작고 큐브 마커 한 변이 40 mm 에 가장
가깝게 나오는 값) 큐브 마커 꼭짓점을 높이 cube_h 평면에 쏜다. 확인값: 그렇게 잰 큐브 마커 한 변.
"""
import math

DICT_NAME = "DICT_5X5_50"
REF_SIZE_MM = 32.0
REF_MM = {0: (-122.0, 78.0), 1: (122.0, 78.0), 2: (122.0, 0.0), 3: (122.0, -78.0), 4: (-122.0, -78.0), 5: (-122.0, 0.0)}   # 마커 중심 (종이 mm)
CUBE_ID = 10
CUBE_MARKER_MM = 40.0
CUBE_IDS = tuple(range(10, 16))   # 탑 쌓기: 큐브마다 다른 마커 (ID 10~15, 같은 40 mm) — detect_cubes
MAX_LEVELS = 5                    # detect_cubes 가 보는 층 수 (윗면 높이 = 층 × cube_h)
DEFAULT_CFG = {"x": -0.05, "y": -0.32, "rot_deg": 0.0, "cube_h": 0.05}   # 종이 가운데 (실좌표 m), 종이 돌아간 각, 큐브 높이
MAX_SIDE = 2000   # 이보다 큰 사진은 줄여서 찾는다


def _dictionary(cv2):
    return cv2.aruco.Dictionary_get(getattr(cv2.aruco, DICT_NAME))


def ref_corners_real(cfg):
    """{id: 4x2 (TL, TR, BR, BL) 실좌표 m} — detectMarkers 꼭짓점 순서와 같다."""
    import numpy as np
    c, s = math.cos(math.radians(cfg["rot_deg"])), math.sin(math.radians(cfg["rot_deg"]))
    h = REF_SIZE_MM / 2
    out = {}
    for i, (mx, my) in REF_MM.items():
        pts = [(mx - h, my + h), (mx + h, my + h), (mx + h, my - h), (mx - h, my - h)]
        out[i] = np.array([(cfg["x"] + (c * px - s * py) / 1000, cfg["y"] + (s * px + c * py) / 1000) for px, py in pts])
    return out


def _cameras(cv2, np, obj, img, w, h):
    """기준 꼭짓점(책상 z=0) → 초점거리 후보마다 (K, R, 카메라 자리, 재투영 rms px). 초점거리를 모르니 훑는다."""
    obj3 = np.hstack([obj, np.zeros((len(obj), 1))]).astype(np.float64)
    out = []
    for f in np.geomspace(0.45, 2.6, 36) * max(w, h):
        K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
        try:
            ok, rv, tv = cv2.solvePnP(obj3, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            continue
        if not ok:
            continue
        pr, _ = cv2.projectPoints(obj3, rv, tv, K, None)
        rms = float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - img) ** 2, axis=1))))
        R, _ = cv2.Rodrigues(rv)
        C = (-R.T @ tv).ravel()
        if C[2] > 0.08:   # 카메라가 책상 아래·바로 위일 리 없다 (거울 해)
            out.append((K, R, C, rms))
    return out


def _top(np, cam, cpix, cube_h):
    """큐브 마커 꼭짓점(사진) → 높이 cube_h 평면 위 실좌표 4x2, 그렇게 잰 한 변 (mm)."""
    K, R, C, _ = cam
    rays = (R.T @ np.linalg.inv(K) @ np.hstack([cpix, np.ones((4, 1))]).T).T
    top = np.array([(C + d * (cube_h - C[2]) / d[2])[:2] for d in rays])
    return top, float(np.mean([np.linalg.norm(top[i] - top[(i + 1) % 4]) for i in range(4)])) * 1000


def detect(image_bytes, cfg=None):
    """사진(JPEG/PNG 바이트) → dict. ok=False 여도 annotated(BGR 그림)·refs 는 될 수 있는 만큼 채운다."""
    import cv2
    import numpy as np
    cfg = {**DEFAULT_CFG, **(cfg or {})}
    out = {"ok": False, "refs": [], "cube_seen": False, "warn": [], "cfg": cfg}
    img = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), _dictionary(cv2), parameters=prm)
    ids = [] if ids is None else [int(i) for i in ids.ravel()]
    found = {i: c.reshape(4, 2).astype(np.float64) for i, c in zip(ids, corners)}
    vis = img.copy()
    lw = max(2, w // 400)
    real = ref_corners_real(cfg)
    refs = sorted(i for i in found if i in real)
    for i in refs:
        cv2.polylines(vis, [found[i].astype(np.int32)], True, (60, 200, 60), lw)
        cv2.putText(vis, str(i), tuple(int(v) for v in found[i].mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    out.update(refs=refs, cube_seen=CUBE_ID in found, annotated=vis, size=[w, h])
    if CUBE_ID in found:
        cv2.polylines(vis, [found[CUBE_ID].astype(np.int32)], True, (0, 140, 255), lw)
    if not refs:
        return {**out, "reason": "기준 종이의 마커가 하나도 안 보임 — 종이가 다 나오게, 밝은 곳에서 다시"}
    if CUBE_ID not in found:
        return {**out, "reason": f"큐브 마커(ID {CUBE_ID})가 안 보임 — 큐브 윗면이 보이게 위쪽에서 다시 (기준 마커는 {len(refs)}개 보임)"}
    if len(refs) < 2:
        out["warn"].append("기준 마커가 1개만 보임 — 정확도가 떨어진다 (2개 이상 나오게)")

    obj = np.vstack([real[i] for i in refs])
    pix = np.vstack([found[i] for i in refs])
    H, _ = cv2.findHomography(pix, obj)   # 사진 → 책상 평면 (실좌표 m)
    if H is None:
        return {**out, "reason": "기준 마커로 책상 평면을 못 구함"}
    back = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)
    out["ref_rms_mm"] = round(float(np.sqrt(np.mean(np.sum((back - obj) ** 2, axis=1)))) * 1000, 2)
    cpix = found[CUBE_ID]
    flat = cv2.perspectiveTransform(cpix.reshape(-1, 1, 2), H).reshape(-1, 2)   # 높이 보정 없는 값 (책상 평면에 쏜 것)
    cams = _cameras(cv2, np, obj, pix, w, h)
    if cams:
        # 재투영 오차가 거의 같은 후보(바로 위에서 찍으면 초점거리가 안 정해진다) 중 큐브 마커 한 변이 원래 크기에 가장 가까운 것
        lo = min(c[3] for c in cams)
        cam = min((c for c in cams if c[3] <= 1.25 * lo + 0.3), key=lambda c: abs(_top(np, c, cpix, cfg["cube_h"])[1] - CUBE_MARKER_MM))
        top, side = _top(np, cam, cpix, cfg["cube_h"])
        out.update(camera={"pos": [round(float(v), 3) for v in cam[2]], "f_px": round(float(cam[0][0, 0]), 1), "reproj_px": round(cam[3], 2)},
                   height_shift_mm=round(float(np.linalg.norm(flat.mean(axis=0) - top.mean(axis=0))) * 1000, 1))
    else:
        top = flat
        side = float(np.mean([np.linalg.norm(top[i] - top[(i + 1) % 4]) for i in range(4)])) * 1000
        out["warn"].append("카메라 자세를 못 구해 큐브 높이 보정 없음 — 비스듬한 사진이면 몇 cm 틀릴 수 있다 (위에서 찍기)")
    out["cube_marker_mm"] = round(side, 1)
    if abs(side - CUBE_MARKER_MM) > 0.1 * CUBE_MARKER_MM:
        out["warn"].append(f"큐브 마커 한 변이 {side:.1f} mm 로 재짐 (원래 {CUBE_MARKER_MM:.0f} mm) — 인쇄 크기·큐브 높이·사진 각도 확인")
    cx, cy = top.mean(axis=0)
    ex = (top[1] - top[0]) + (top[2] - top[3])   # 마커 +x (TL→TR, BL→BR)
    yaw = math.degrees(math.atan2(ex[1], ex[0]))
    yaw = (yaw + 45.0) % 90.0 - 45.0             # 큐브는 90° 마다 같다
    out.update(ok=True, cube={"x": round(float(cx), 4), "y": round(float(cy), 4), "yaw_deg": round(yaw, 1)})
    c = tuple(int(v) for v in cpix.mean(axis=0))
    cv2.circle(vis, c, lw * 3, (0, 140, 255), -1)
    cv2.arrowedLine(vis, c, tuple(int(v) for v in (cpix[1] + cpix[2]) / 2), (0, 140, 255), lw)
    return out


def detect_cubes(image_bytes, cfg=None, ids=CUBE_IDS):
    """탑 쌓기: 사진 한 장 → 큐브 여러 개 (마커 ID 가 ids 안인 것 모두). detect() 는 그대로 (ID 10 하나).

    돌려주는 것: {ok, cubes: [{id, x, y, yaw_deg, level, top_cm, marker_mm}], refs, warn, annotated, …}.
    level = 그 큐브 밑에 깔린 큐브 수 (0 = 책상 위, 1 = 한 층 위 …), top_cm = 윗면 높이 (= (level + 1) × cube_h).
    높이: 기준 마커로 구한 카메라 자세에서 마커 꼭짓점을 높이 h 평면에 쏘면 마커 한 변이 h 에 따라 달라진다 (카메라가 50 cm 위면
    한 층에 약 12 %) → 한 변이 40 mm 에 가장 가까운 층. 초점거리를 몰라서, 재투영 오차가 비슷한 초점거리 후보 중 모든 큐브의
    한 변이 가장 잘 맞는 것을 고른다 (같으면 낮은 층). 바로 위에서 찍으면 초점거리와 높이가 서로 바뀌어도 사진이 같아
    층을 못 가린다 — 비스듬히(30° 이상) 찍어야 한다. 그럴 때는 warn 에 적는다."""
    import cv2
    import numpy as np
    cfg = {**DEFAULT_CFG, **(cfg or {})}
    out = {"ok": False, "refs": [], "cubes": [], "warn": [], "cfg": cfg}
    img = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, mids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), _dictionary(cv2), parameters=prm)
    mids = [] if mids is None else [int(i) for i in mids.ravel()]
    found = {i: c.reshape(4, 2).astype(np.float64) for i, c in zip(mids, corners)}
    vis = img.copy()
    lw = max(2, w // 400)
    real = ref_corners_real(cfg)
    refs = sorted(i for i in found if i in real)
    seen = sorted(i for i in found if i in ids)
    for i in refs:
        cv2.polylines(vis, [found[i].astype(np.int32)], True, (60, 200, 60), lw)
    out.update(refs=refs, annotated=vis, size=[w, h], seen=seen)
    if not refs:
        return {**out, "reason": "기준 종이의 마커가 하나도 안 보임 — 종이가 다 나오게, 밝은 곳에서 다시"}
    if not seen:
        return {**out, "reason": f"큐브 마커(ID {min(ids)}~{max(ids)})가 하나도 안 보임 — 큐브 윗면이 보이게 다시 (기준 마커는 {len(refs)}개 보임)"}
    if len(refs) < 2:
        out["warn"].append("기준 마커가 1개만 보임 — 정확도가 떨어진다 (2개 이상 나오게)")
    obj = np.vstack([real[i] for i in refs])
    pix = np.vstack([found[i] for i in refs])
    cams = _cameras(cv2, np, obj, pix, w, h)
    if not cams:
        return {**out, "reason": "기준 마커로 카메라 자세를 못 구함 — 큐브 높이를 알 수 없다. 기준 종이가 더 크게·또렷하게 나오게 다시"}
    ch = float(cfg["cube_h"])

    def fit(cam):   # 큐브마다 한 변이 가장 잘 맞는 층 → (점수, [(id, 층, 윗면 4x2, 한 변 mm)])
        rows, cost = [], 0.0
        for i in seen:
            best = None
            for lv in range(MAX_LEVELS):
                top, side = _top(np, cam, found[i], (lv + 1) * ch)
                e = abs(side - CUBE_MARKER_MM) / CUBE_MARKER_MM
                if best is None or e < best[0] - 1e-9:
                    best = (e, lv, top, side)
            rows.append((i, best[1], best[2], best[3]))
            cost += best[0] + 0.004 * best[1]   # 거의 같으면 낮은 층
        return cost, rows
    lo = min(c[3] for c in cams)
    near = [c for c in cams if c[3] <= 1.25 * lo + 0.3]
    fits = [(fit(c), c) for c in near]
    (cost, rows), cam = min(fits, key=lambda x: x[0][0])
    # 다른 초점거리 후보가 다른 층으로도 비슷하게 맞으면 (바로 위에서 찍음) 층을 믿을 수 없다
    alt = [f for f, _ in fits if [r[1] for r in f[1]] != [r[1] for r in rows] and f[0] - sum(0.004 * r[1] for r in f[1]) < 0.03 * len(seen)]
    if alt:
        out["warn"].append("사진이 거의 바로 위에서 찍혀 큐브 높이(층)가 확실하지 않음 — 비스듬히(30° 이상 기울여) 다시 찍기")
        out["level_unsure"] = True
    out["camera"] = {"pos": [round(float(v), 3) for v in cam[2]], "f_px": round(float(cam[0][0, 0]), 1), "reproj_px": round(cam[3], 2)}
    for i, lv, top, side in rows:
        cx, cy = top.mean(axis=0)
        ex = (top[1] - top[0]) + (top[2] - top[3])
        yaw = (math.degrees(math.atan2(ex[1], ex[0])) + 45.0) % 90.0 - 45.0
        c = {"id": i, "x": round(float(cx), 4), "y": round(float(cy), 4), "yaw_deg": round(yaw, 1), "level": lv,
             "top_cm": round((lv + 1) * ch * 100, 1), "marker_mm": round(side, 1)}
        if abs(side - CUBE_MARKER_MM) > 0.06 * CUBE_MARKER_MM:
            c["unsure"] = True
            out["warn"].append(f"큐브 {i}: 마커 한 변이 {side:.1f} mm 로 재짐 (원래 {CUBE_MARKER_MM:.0f}) — 높이·자리가 틀릴 수 있다")
        out["cubes"].append(c)
        col = (0, 140, 255) if lv == 0 else (255, 120, 0)
        cpix = found[i]
        cv2.polylines(vis, [cpix.astype(np.int32)], True, col, lw)
        p0 = tuple(int(v) for v in cpix.mean(axis=0))
        cv2.arrowedLine(vis, p0, tuple(int(v) for v in (cpix[1] + cpix[2]) / 2), col, lw)
        cv2.putText(vis, f"{i}" + (f" +{lv}" if lv else ""), tuple(int(v) for v in cpix[0]), cv2.FONT_HERSHEY_SIMPLEX, w / 900, col, lw)
    out["ok"] = True
    return out
