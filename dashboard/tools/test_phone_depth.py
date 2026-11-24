#!/usr/bin/env python3
"""phone_depth.py 시험 (폰·GPU 없이): python3 dashboard/tools/test_phone_depth.py

  1. 좌표 변환이 팀 vision_node 의 확인값 (config/camera.yaml '검증', tests/check_camera_result.py 손 계산) 과 같은지
  2. 실제 파일에서 따온 큐브 + 잡음 덩어리 (check_camera_result.py 의 값) 에서 큐브만 고르는지, 헷갈리면 안 고르는지
  3. 기록한 것 같은 inference.json 한 벌: 로봇 좌표에 둔 큐브를 카메라 좌표로 거꾸로 옮겨 쓴 파일 → evaluate 가 그 자리·각을 돌려주는지
  4. 큐브 각 (옆면·윗면), 높이 경고, 오래된 파일, 거치 자세 풀기 (solve_mount)
"""
import json
import os
import sys
import tempfile
import time
from math import cos, radians, sin
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import phone_depth as DP   # noqa: E402

bad = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f": {detail}" if detail != "" else ""))
    if not ok:
        bad.append(name)


def near(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


C2R = DP.DEFAULT_C2R
D = 0.4775

# 1. camera.yaml 의 검증값 (1e-3 안)
for cam, want in (((0, 0, 0.45), (0.174, -0.4352, -0.3308)), ((0.1, 0, 0.45), (0.274, -0.4335, -0.3326)), ((0, 0.1, 0.45), (0.1764, -0.5061, -0.2603))):
    got = DP.camera_to_robot(cam, C2R)
    check(f"camera.yaml 검증 cam {cam}", near(got, want, 1e-3), tuple(round(v, 4) for v in got))
check("손 계산: 세 각 모두 90°, p (1,2,3), t (10,20,30) → (13,22,29)", near(DP.camera_to_robot((1, 2, 3), (10, 20, 30, 90, 90, 90)), (13, 22, 29), 1e-9))
check("점수 50x40 (범위 40~80) = 0.525", abs(DP.score_candidate((50, 40, 3), (40, 80)) - 0.525) < 1e-12)

# 2. 실제 파일에서 따온 값 (vision_node/tests/check_camera_result.py)
REAL = {"instance": 1, "label": "blob", "status": "ok", "bbox_xyxy": [467, 262, 546, 330], "depth_rect_m": 0.442, "dims_mm": [60.3, 55.5, 2.8],
        "center_cam1_m": [0.0715, 0.0906, 0.4457], "rotation": {"yaw_deg": 6.05, "pitch_deg": 37.3, "roll_deg": -19.1, "tilt_deg": 37.6}, "warnings": []}
NOISE = {"instance": 2, "label": "blob", "status": "ok", "bbox_xyxy": [607, 14, 640, 26], "depth_rect_m": 0.789, "dims_mm": [93.2, 38.1, 7.1],
         "center_cam1_m": [0.5, -0.2, 0.789], "rotation": {"yaw_deg": 80.0}, "warnings": []}


def doc(*objs):
    return json.dumps({"timestamp_ns": 1, "objects": {"seg": list(objs)}})


pick, why = DP.select_cube(doc(REAL, NOISE), (40, 80), (0.3, 1.0))
check("큐브 + 잡음: 큐브만", pick is not None and pick["instance"] == 1 and pick["xyz"] == (0.0715, 0.0906, 0.4457), why)
pick, why = DP.select_cube(doc(NOISE), (40, 80), (0.3, 1.0))
check("잡음만: 후보 없음 + 이유", pick is None and "후보 없음" in why and "93.2x38.1" in why, why)
pick, why = DP.select_cube(doc(REAL, {**REAL, "instance": 3, "center_cam1_m": [0.2, 0.1, 0.5]}), (40, 80), (0.3, 1.0))
check("같은 큐브 2개: 헷갈림 → 안 고름", pick is None and "헷갈림" in why, why)
pick, why = DP.select_cube(doc(REAL, {**REAL, "instance": 3, "center_cam1_m": [0.2, 0.1, 0.5]}), (40, 80), (0.3, 1.0), prev=(0.2, 0.1, 0.51))
check("직전 자리 3 cm 안의 후보를 이어 고름", pick is not None and pick["instance"] == 3 and pick["tracked"], why)
for name, raw in (("잘린 JSON", doc(REAL)[:40]), ("빈 파일", ""), ("objects 없음", "{}")):
    pick, why = DP.select_cube(raw, (40, 80), (0.3, 1.0))
    check(f"{name}: 예외 없이 건너뜀", pick is None and isinstance(why, str))
p = DP.camera_to_robot(REAL["center_cam1_m"], C2R)
print(f"      (참고) 실제 파일의 큐브를 CAD 임시 거치 자세로 옮기면 로봇 ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) = 책상 위 {(p[2] + D) * 100:.1f} cm"
      " — 그 파일은 거치대에서 찍은 것이 아니다 (camera.yaml 도 '책상 높이와 맞는지 확인 안 함')")


# 3. 기록한 것 같은 파일 한 벌: 로봇 좌표 → 카메라 좌표 (R 의 역 = 전치)
def rot(v, rpy):
    return DP.camera_to_robot(v, (0, 0, 0) + tuple(rpy))


def R_cols(rpy):
    return [rot(e, rpy) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]      # R 의 열 3개


def robot_to_camera(p, c2r):
    d = [p[i] - c2r[i] for i in range(3)]
    return tuple(sum(c[i] * d[i] for i in range(3)) for c in R_cols(c2r[3:]))      # Rᵀ · (p − t)


def dir_to_camera(v, c2r):
    return tuple(sum(c[i] * v[i] for i in range(3)) for c in R_cols(c2r[3:]))


def recorded(x, y, yaw_deg, face="side"):
    """로봇 (x, y) 에 yaw 로 놓인 5 cm 큐브를 거치 카메라가 본 객체 한 개 (보이는 면 = 카메라 쪽 옆면 또는 윗면)."""
    c, s = cos(radians(yaw_deg)), sin(radians(yaw_deg))
    if face == "side":
        n_r, ctr = (-s, c, 0.0), (x - s * 0.025, y + c * 0.025, -D + 0.025)     # 로봇(+y) 쪽 옆면: 법선 = 큐브 +y 축
        roll = 0.0
    else:
        n_r, ctr = (0.0, 0.0, 1.0), (x, y, -D + 0.05)
        roll = None
    n = dir_to_camera(n_r, C2R)
    cam = robot_to_camera(ctr, C2R)
    rotn = {"yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": roll, "normal_cam1": [round(v, 4) for v in n]}
    if face == "top":      # 긴 변 = 큐브 x 축. roll = 면 안 기저 (hx = n × ŷ, vx = hx × n) 에서 본 그 방향의 각 (lib_stereo.plane_orientation)
        import numpy as np
        e = np.array(dir_to_camera((c, s, 0.0), C2R)); nn = np.array(n)
        hx = np.cross(nn, [0, 1.0, 0]); hx /= np.linalg.norm(hx); vx = np.cross(hx, nn)
        rotn["roll_deg"] = float(np.degrees(np.arctan2(e @ vx, e @ hx)))
    return {"instance": 1, "label": "blob", "status": "ok", "depth_rect_m": round(cam[2], 3), "dims_mm": [52.0, 50.5, 2.0],
            "center_cam1_m": [round(v, 4) for v in cam], "rotation": rotn, "warnings": []}


with tempfile.TemporaryDirectory() as tmp:
    cfg = {**DP.load_cfg(os.path.join(tmp, "none.json")), "run_dir": tmp}
    f = os.path.join(tmp, "inference.json")
    r = DP.evaluate(cfg, D, time.time())
    check("파일 없음: 이유", not r["ok"] and "못 읽음" in r["reason"], r["reason"])
    for face, x, y, yaw in (("side", -0.05, -0.33, 20.0), ("side", 0.08, -0.30, -30.0), ("top", -0.05, -0.33, 20.0), ("top", -0.10, -0.36, -40.0)):
        Path(f).write_text(doc(recorded(x, y, yaw, face), NOISE))
        r = DP.evaluate(cfg, D, time.time())
        ex = (x - sin(radians(yaw)) * 0.025, y + cos(radians(yaw)) * 0.025) if face == "side" else (x, y)   # 보이는 면의 가운데
        ok = r["ok"] and near((r["cube"]["x"], r["cube"]["y"]), ex, 0.001) and abs(r["cube"]["yaw_deg"] - yaw) < 0.5 and r["yaw_from"] == face and r["usable"]
        check(f"기록 파일 ({face}) 큐브 ({x}, {y}, {yaw}°)", ok, {k: r.get(k) for k in ("cube", "height_cm", "yaw_from", "cam", "reason")})
    check("경고: 거치 자세가 CAD 임시값", any("임시값" in w for w in r["warn"]), r["warn"])
    o = recorded(-0.05, -0.33, 0.0)
    o["center_cam1_m"][2] -= 0.15          # 15 cm 가까이 → 책상 위로 뜬 자리
    Path(f).write_text(doc(o))
    r = DP.evaluate(cfg, D, time.time())
    check("높이가 안 맞으면 usable=False + 경고", r["ok"] and r["usable"] is False and any("높이" in w for w in r["warn"]), (r.get("height_cm"), r["warn"]))
    Path(f).write_text(doc(recorded(-0.05, -0.33, 20.0)))
    r = DP.evaluate({**cfg, "yaw_manual_deg": 100.0}, D, time.time())
    check("각 직접 넣기 100° → 10° (90° 마다 같음)", r["ok"] and r["cube"]["yaw_deg"] == 10.0 and r["yaw_from"] == "manual", r.get("cube"))
    os.utime(f, (time.time() - 10, time.time() - 10))
    r = DP.evaluate(cfg, D, time.time())
    check("10초 지난 파일: 안 씀", not r["ok"] and r.get("stale") and "오래됨" in r["reason"], r["reason"])

# 4. 실제 파일의 회전값 (법선 없음 → yaw·pitch 로 되만듦): 카메라 쪽 옆면, 거의 정면
yaw, src = DP.table_yaw(REAL["rotation"], C2R)
check("실제 파일의 회전 → 옆면, 각 −45~45°", src == "side" and -45 <= yaw < 45, (round(yaw, 1), src))
check("회전 없음 → 각 없음", DP.table_yaw(None, C2R) == (None, None))

# 거치 자세 풀기: 아는 자세로 만든 점 4개 → 같은 숫자 6개
TRUE = (0.15, -0.10, 0.02, 140.0, 3.0, -5.0)
pts = [(robot_to_camera((x, y, -D + 0.025), TRUE), (x, y)) for x, y in ((-0.10, -0.30), (0.05, -0.30), (-0.05, -0.40), (0.10, -0.38))]
params, res = DP.solve_mount(pts, -D + 0.025)
check("solve_mount: 아는 자세를 되찾음", near(params, TRUE, 1e-6) and max(res) < 1e-6, [round(v, 4) for v in params])
try:
    DP.solve_mount([((0, 0, 0.4), (0, -0.3)), ((0.1, 0, 0.4), (0.1, -0.3)), ((0.2, 0, 0.4), (0.2, -0.3))], -0.45)
    check("일렬인 점: 거절", False)
except ValueError:
    check("일렬인 점: 거절", True)

print(f"\n{'전부 통과' if not bad else '실패 ' + str(len(bad)) + '개: ' + ', '.join(bad)}")
sys.exit(1 if bad else 0)
