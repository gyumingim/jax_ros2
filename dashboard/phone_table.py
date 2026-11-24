"""휴대폰 카메라 탭의 '책상 높이 재기': 거치대에 끼운 폰의 사진 한 장 (또는 폰 뎁스) → 책상 높이 d (어깨 roll 축 ~ 책상 윗면, m).

보정 단계 없이 폰이 CAD 자리에 있다고 믿고 잰다. 거치 사양 = team-jax/JaxIsaacLab artifacts/deploy_2026-11-05/CAMERA.md (2026-11-12 CAD):
    카메라 = 바닥에서 1.2005 m (어깨 피치축 − 7 mm), 프로파일 중심선 앞 117.7 mm, 좌우 가운데, 앞을 보고 45° 아래.
    카메라 좌표 (정우 '베이스' 축: x = 뒤, y = 로봇 오른쪽, z = 위) 에서 어깨 피치축 = (−0.032, 0, +0.007).
    광학 좌표 (OpenCV: x 영상 오른쪽, y 영상 아래, z 시선) → 카메라 좌표 = R_OPT.

우리 로봇 좌표 (pick_plan 과 같음): 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (앞 = −y), +z = 위, 책상 면 z = −d.
축 대응:  우리 (x, y, z) = (−y_c, x_c, z_c) + CAM_POS      (정우 y = 로봇 오른쪽이라 부호가 바뀌고, 정우 x = 뒤 = 우리 y)
    CAM_POS = (0.174, −0.1177, −0.012): 팀 camera.yaml 의 CAD 점 (거치대 꼭짓점 P − 어깨 roll 축 R0) — phone_depth.DEFAULT_C2R 의 앞 3개와 같다.
    CAMERA.md 와 맞춰 보면: 어깨 피치축 = 우리 좌표 (0.174, −0.1497, −0.005) → roll 축이 피치축보다 5 mm 위. 그래서
    CAMERA.md 의 '책상 윗면 0.735 m, 카메라 0.4655 m 위' = 우리 d 0.4655 + 0.012 = 0.4775 (역기구학 탭 기본값) 로 딱 맞는다.
    회전: DEFAULT_C2R 의 뒤 3개 (IMU 로 잰 값, 폰이 뒤집혀 끼워짐) = 아래 R_CAD × 광축 둘레 180° 에서 1.4° 다르다.

재는 법: 책상 평면을 광학 좌표에서 구한다 (법선 n, 카메라에서 평면까지 수직 거리 h).
    ArUco  = 기준 종이 마커 (ID 0~5, 32 mm) 꼭짓점으로 solvePnP. 초점거리는 모르므로 재투영 오차가 가장 작은 값으로 맞춘다.
    뎁스   = 폰 스테레오 프로그램이 저장한 rgbd/rgbd.npz (--rgbd-viewer) 의 점들에 평면 맞추기 (RANSAC).
    d = h − CAM_POS z = h + 0.012.   책상이 수평이면 수직 거리 h 는 폰이 얼마나 기울었든 같다 → 45° 가 1° 틀려도 d 는 안 변한다
    (CAD 회전으로 점을 옮겨 z 를 읽으면 1° 에 약 8 mm 틀린다 — 그 값은 d_cad_rot 으로 같이 준다).
    기울기 검사: 잰 법선과 CAD 가 말하는 '위' 의 각 (사진이 90° 단위로 돌아가 있어도 되게 4 방향 중 가장 가까운 것). 2° 넘으면 경고.

오차 (솔직하게): ① 렌즈 자리 — CAD 점은 거치대 꼭짓점이고 렌즈는 거기서 경사를 따라 0~40 mm 위 → d 가 0~28 mm 크게 나올 수 있다 (미반영).
② 초점거리 — 사진에서 추정한다. 거리가 그 오차만큼 비례해 틀린다 (1 % = 약 5 mm). 렌즈 왜곡은 안 본다. ③ 인쇄 크기 (100 mm 선 확인).
그래서 처음 한 번은 줄자로 잰 값과 맞춰 볼 것.
"""
import json
import math
import os

import phone_detect as PD

CAM_POS = (0.174, -0.1177, -0.012)      # 카메라 자리 (우리 로봇 좌표 m) — CAMERA.md / camera.yaml CAD
CAM_PITCH_DEG = 45.0                    # 아래로 기운 각 (CAD)
_S = math.sqrt(0.5)
R_OPT = ((0.0, _S, -_S), (1.0, 0.0, 0.0), (0.0, -_S, -_S))            # CAMERA.md 2.1: 광학 → 카메라 좌표 (정우 축)
R_CAD = ((-1.0, 0.0, 0.0), (0.0, _S, -_S), (0.0, -_S, -_S))           # 광학 → 우리 로봇 좌표 = [[0,−1,0],[1,0,0],[0,0,1]] · R_OPT
D_RANGE = (0.44, 0.53)                  # 계획이 받는 범위 (hand_ik.D_RANGE 와 같게 — phone_api 가 그쪽 값으로 덮어쓴다)
TILT_WARN_DEG = 2.0
MIN_REFS = 3                            # 초점거리까지 맞추려면 마커 3개 이상
DEPTH_MAX_AGE_S = 20.0                  # rgbd.npz 는 5초마다 저장된다
DEPTH_Z_RANGE = (0.2, 1.5)              # 평면 맞추기에 쓰는 점의 거리 (m)


def cam_to_robot(p_opt):
    """광학 좌표의 점 → 우리 로봇 좌표 (CAD 자세)."""
    return tuple(sum(R_CAD[i][j] * p_opt[j] for j in range(3)) + CAM_POS[i] for i in range(3))


def plane_to_d(n, p0):
    """광학 좌표의 책상 평면 (법선 n, 평면 위 한 점 p0) → dict(d, h, tilt_deg, image_rot_deg, cam_pitch_deg, d_cad_rot).
    d = 수직 거리 h − CAM_POS z. tilt = 법선과 CAD '위' (R_CAD 3행) 의 각 — 사진이 광축 둘레로 90° 단위 돌아간 4 경우 중 가장 작은 것."""
    n, p0 = [float(v) for v in n], [float(v) for v in p0]
    k = math.sqrt(sum(v * v for v in n))
    n = [v / k for v in n]
    if sum(a * b for a, b in zip(n, p0)) > 0:      # 법선이 카메라 쪽 (책상에서 위) 을 보게
        n = [-v for v in n]
    h = -sum(a * b for a, b in zip(n, p0))
    best = None
    for q in range(4):                             # 광학 좌표를 광축 둘레로 q × 90° 돌린 뒤 R_CAD
        c, s = round(math.cos(q * math.pi / 2)), round(math.sin(q * math.pi / 2))
        rot = lambda v: (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])   # noqa: E731
        up = sum(R_CAD[2][j] * rot(n)[j] for j in range(3))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        if best is None or ang < best[0]:
            best = (ang, q * 90, -(sum(R_CAD[2][j] * rot(p0)[j] for j in range(3)) + CAM_POS[2]))
    return {"d": h - CAM_POS[2], "h": h, "tilt_deg": best[0], "image_rot_deg": best[1],
            "cam_pitch_deg": math.degrees(math.asin(max(-1.0, min(1.0, abs(n[2]))))), "d_cad_rot": best[2]}


def _finish(out, pl):
    """plane_to_d 결과를 화면용으로 (반올림 + 기울기 경고)."""
    out.update(ok=True, d=round(pl["d"], 4), cam_above_table=round(pl["h"], 4), tilt_deg=round(pl["tilt_deg"], 2),
               image_rot_deg=pl["image_rot_deg"], cam_pitch_deg=round(pl["cam_pitch_deg"], 2), d_cad_rot=round(pl["d_cad_rot"], 4))
    if pl["tilt_deg"] > TILT_WARN_DEG:
        out["warn"].append(f"책상 평면이 CAD 자세에서 {pl['tilt_deg']:.1f}° 기울어 보임 (카메라 기울기 {pl['cam_pitch_deg']:.1f}°, CAD {CAM_PITCH_DEG:.0f}°) "
                           "— 폰이 거치대에 제대로 안 끼워졌거나 거치 각이 다르다")
    return out


# ---- ArUco: 기준 종이 한 장 --------------------------------------------------------------------------------------------

def _pose(cv2, np, obj3, pix, f, w, h):
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    try:
        ok, rv, tv = cv2.solvePnP(obj3, pix, K, None, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not ok or float(tv[2]) <= 0:
        return None
    rv, tv = cv2.solvePnPRefineLM(obj3, pix, K, None, rv, tv)      # IPPE 는 호모그래피 해라 재투영 최소가 아니다 → 이 f 에서의 최소로
    pr, _ = cv2.projectPoints(obj3, rv, tv, K, None)
    rms = float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - pix) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rv)
    return rms, R, tv.ravel()


def _fit_focal(cv2, np, obj3, pix, w, h):
    """초점거리(px)를 훑어 재투영 오차가 가장 작은 것 → (f, (rms, R, t), [f 낮은 쪽, 높은 쪽] 2σ) 또는 None.
    평면 한 장이라도 비스듬히 (거치대 45°) 보면 가까운 쪽·먼 쪽 마커의 크기 차이로 초점거리가 정해진다. 바로 위에서 보면 안 정해진다."""
    m = max(w, h)
    rows = [(f, _pose(cv2, np, obj3, pix, f, w, h)) for f in np.geomspace(0.4, 3.0, 60) * m]
    rows = [(f, p) for f, p in rows if p is not None]
    if not rows:
        return None
    f0 = min(rows, key=lambda r: r[1][0])[0]
    fine = [(f, _pose(cv2, np, obj3, pix, f, w, h)) for f in np.geomspace(f0 / 1.15, f0 * 1.15, 141)]
    fine = [(f, p) for f, p in fine if p is not None]
    f, best = min(fine, key=lambda r: r[1][0])
    # 2σ: 꼭짓점 잡음이 재투영 오차만 하다고 보고 Δχ² = 4 안쪽 (자유도 = 좌표 2N − 미지수 7). 잡음 바닥 0.15 px
    dof = max(1, 2 * len(pix) - 7)
    lim = max(best[0], 0.15) ** 2 * (1 + 4.0 / dof)
    near = [ff for ff, p in fine if p[0] ** 2 <= lim]
    return float(f), best, (float(min(near)), float(max(near)))


def measure_aruco(image_bytes):
    """사진 (JPEG/PNG 바이트, 거치대의 폰으로 찍은 것) → dict(ok, d, tilt_deg, f_px, d_pm_mm, refs, warn, annotated …). 종이가 놓인 자리는 몰라도 된다."""
    import cv2
    import numpy as np
    out = {"ok": False, "refs": [], "warn": []}
    img = PD.as_image(image_bytes)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = PD.MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), PD._dictionary(cv2), parameters=prm)
    ids = [] if ids is None else [int(i) for i in ids.ravel()]
    found = {i: c.reshape(4, 2).astype(np.float64) for i, c in zip(ids, corners)}
    sheet = PD.ref_corners_real({"x": 0.0, "y": 0.0, "rot_deg": 0.0})     # 종이 좌표 (m) — 평면만 필요해 놓인 자리는 안 쓴다
    refs = sorted(i for i in found if i in sheet)
    vis = img.copy()
    lw = max(2, w // 400)
    for i in refs:
        cv2.polylines(vis, [found[i].astype(np.int32)], True, (60, 200, 60), lw)
        cv2.putText(vis, str(i), tuple(int(v) for v in found[i].mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    out.update(refs=refs, annotated=vis, size=[w, h])
    if len(refs) < MIN_REFS:
        return {**out, "reason": f"기준 종이의 마커가 {len(refs)}개만 보임 ({MIN_REFS}개 이상 필요) — 종이를 화면 가운데쯤 책상에 평평하게 두고 다시"}
    obj = np.vstack([sheet[i] for i in refs])
    obj3 = np.hstack([obj, np.zeros((len(obj), 1))]).astype(np.float64)
    pix = np.vstack([found[i] for i in refs])
    fit = _fit_focal(cv2, np, obj3, pix, w, h)
    if fit is None:
        return {**out, "reason": "기준 마커로 카메라 자세를 못 구함 — 종이가 더 크게·또렷하게 나오게 다시"}
    f, (rms, R, t), (f_lo, f_hi) = fit
    pl = plane_to_d(R[:, 2], t)
    ds = []
    for ff in (f_lo, f_hi):                        # 초점거리 흔들림이 d 에 주는 폭
        p = _pose(cv2, np, obj3, pix, ff, w, h)
        if p is not None:
            ds.append(plane_to_d(p[1][:, 2], p[2])["d"])
    pm = max([abs(v - pl["d"]) for v in ds] or [0.0]) * 1000
    out.update(f_px=round(f, 1), reproj_px=round(rms, 2), d_pm_mm=round(pm, 1))
    if len(refs) < 5:
        out["warn"].append(f"기준 마커 {len(refs)}개만 보임 — 초점거리 추정이 덜 정확하다 (6개 다 나오게)")
    if rms > 1.5:
        out["warn"].append(f"재투영 오차 {rms:.1f} px — 종이가 울었거나 사진이 흐리다")
    if pm > 8:
        out["warn"].append(f"초점거리가 잘 안 정해짐 (d ± {pm:.0f} mm) — 종이가 더 크게 나오게")
    cv2.putText(vis, f"d {pl['d']:.3f} m  tilt {pl['tilt_deg']:.1f} deg", (lw * 6, int(h - lw * 8)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    return _finish(out, pl)


# ---- 뎁스: 폰 스테레오 프로그램의 rgbd.npz ------------------------------------------------------------------------------

def fit_plane(points, tol=0.004, iters=300, seed=0):
    """(N, 3) 점 (광학 좌표 m) 에서 가장 큰 평면 (RANSAC 뒤 인라이어 최소제곱) → (법선, 평면 위 점, 인라이어 비율, 잔차 rms m) 또는 None."""
    import numpy as np
    p = np.asarray(points, dtype=np.float64)
    if len(p) < 200:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        k = np.linalg.norm(n)
        if k < 1e-9:
            continue
        n /= k
        cnt = int((np.abs((p - a) @ n) < tol).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None or best_n < 0.3 * len(p):
        return None
    n, a = best
    for _ in range(2):
        inl = p[np.abs((p - a) @ n) < tol]
        a = inl.mean(axis=0)
        n = np.linalg.svd(inl - a, full_matrices=False)[2][2]
    res = (p - a) @ n
    inl = np.abs(res) < tol
    return n, a, float(inl.mean()), float(np.sqrt(np.mean(res[inl] ** 2)))


def measure_depth(run_dir, now, max_age_s=DEPTH_MAX_AGE_S):
    """<run_dir>/rgbd/rgbd.npz (points_left_m = 왼쪽 렌즈 광학 좌표 점, valid) → dict(ok, d, tilt_deg, inlier, rms_mm, age_s, warn) 또는 ok False + reason."""
    import numpy as np
    path = os.path.join(run_dir, "rgbd", "rgbd.npz")
    out = {"ok": False, "warn": [], "file": path, "age_s": None}
    try:
        age = now - os.stat(path).st_mtime
    except OSError:
        return {**out, "missing": True, "reason": "뎁스 맵 없음 — 준비의 '뎁스 맵 저장' 을 켜고 폰 프로그램을 다시 켜기 (터미널이면 --rgbd-viewer)"}
    out["age_s"] = round(age, 1)
    if age > max_age_s:
        return {**out, "stale": True, "reason": f"뎁스 맵이 오래됨 ({age:.0f}초 전) — 폰 프로그램이 꺼졌거나 멈춤"}
    try:
        with np.load(path) as z:
            pts, valid = z["points_left_m"], z["valid"]
    except Exception as e:      # 쓰는 중이면 깨진 zip
        return {**out, "reason": f"뎁스 맵을 못 읽음 (쓰는 중일 수 있음 — 다시 누르기): {type(e).__name__}"}
    ok = valid.astype(bool) & np.isfinite(pts).all(axis=2) & (pts[..., 2] >= DEPTH_Z_RANGE[0]) & (pts[..., 2] <= DEPTH_Z_RANGE[1])
    p = pts[ok]
    if len(p) > 20000:
        p = p[:: len(p) // 20000 + 1]
    fit = fit_plane(p)
    if fit is None:
        return {**out, "reason": f"책상 평면을 못 찾음 (쓸 수 있는 점 {len(p)}개) — 책상이 화면의 큰 부분을 차지해야 한다"}
    n, a, frac, rms = fit
    out.update(inlier=round(frac, 2), rms_mm=round(rms * 1000, 1))
    try:
        st = json.loads(open(os.path.join(run_dir, "rgbd", "frame.json")).read()).get("scale_status")
        if st != "metric":
            out["warn"].append(f"폰 스테레오의 거리 눈금이 확인 안 됨 (scale_status {st})")
    except (OSError, ValueError, AttributeError):
        pass
    return _finish(out, plane_to_d(n, a))


# ---- 둘을 한 줄로 -------------------------------------------------------------------------------------------------------

def combine(aruco, depth):
    """두 결과 (없으면 None) → 화면에 줄 dict: ok, d (ArUco 먼저), source, in_range, diff_mm, text, warn, reason."""
    a = aruco if aruco and aruco.get("ok") else None
    p = depth if depth and depth.get("ok") else None
    out = {"ok": bool(a or p), "aruco": aruco, "depth": depth, "range": list(D_RANGE), "warn": []}
    if not out["ok"]:
        out["reason"] = (aruco or depth or {}).get("reason") or "잰 값 없음"
        return out
    main, src = (a, "aruco") if a else (p, "depth")
    d = main["d"]
    text = f"책상 높이 d = {d:.3f} m ({'ArUco' if a else '뎁스'})"
    if a and p:
        out["diff_mm"] = round(abs(a["d"] - p["d"]) * 1000, 1)
        text += f" · 뎁스 {p['d']:.3f} m · 차이 {out['diff_mm']:.0f} mm"
        if out["diff_mm"] > 10:
            out["warn"].append("ArUco 와 뎁스가 1 cm 넘게 다름 — 줄자로 확인")
    out.update(d=d, source=src, text=text, tilt_deg=main["tilt_deg"], in_range=D_RANGE[0] - 1e-9 <= d <= D_RANGE[1] + 1e-9)
    if not out["in_range"]:
        out["reason"] = f"계획이 받는 범위 {D_RANGE[0]:g} ~ {D_RANGE[1]:g} m 밖이라 역기구학 탭에 넣지 않음 — 폰이 거치대에 있는지, 종이가 책상 위인지 확인"
    for r in (a, p):
        out["warn"] += (r or {}).get("warn", [])
    return out
