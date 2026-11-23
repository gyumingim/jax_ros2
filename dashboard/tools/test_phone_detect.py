#!/usr/bin/env python3
"""phone_detect 시험: 기준 종이 + 큐브 윗면 마커(책상 위 5 cm)를 아는 카메라로 그린 가짜 사진 → 찾은 자리가 맞나.

    /usr/bin/python3 dashboard/tools/test_phone_detect.py [그림 저장 폴더]

통과 기준: 위치 1 cm, yaw 3° 안. 실패하면 종료코드 1.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import phone_detect as PD            # noqa: E402
import make_phone_markers as MK      # noqa: E402

W, H = 1600, 1200


def look_at(cam, target):
    z = np.array(target, float) - cam; z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0]); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    return R, -R @ cam


def render(cfg, cube, cam, f):
    """cube = (x, y, yaw°) 실좌표. cam = 카메라 자리 (x, y, 책상 위 높이 m)."""
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    R, t = look_at(np.array(cam, float), [cfg["x"], cfg["y"], 0])
    proj = lambda p: (lambda q: q[:2] / q[2])(K @ (R @ np.array(p, float) + t))   # noqa: E731
    img = np.full((H, W, 3), (150, 170, 185), np.uint8)   # 책상 색
    sheet = cv2.cvtColor(np.array(MK.sheet_image(150)), cv2.COLOR_GRAY2BGR)
    sh, sw = sheet.shape[:2]
    c, s = math.cos(math.radians(cfg["rot_deg"])), math.sin(math.radians(cfg["rot_deg"]))
    real = lambda px, py, z=0.0: (cfg["x"] + (c * px - s * py) / 1000, cfg["y"] + (s * px + c * py) / 1000, z)   # noqa: E731
    a, b = MK.A4[0] / 2, MK.A4[1] / 2
    dst = np.float32([proj(real(-a, b)), proj(real(a, b)), proj(real(a, -b)), proj(real(-a, -b))])
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [sw, 0], [sw, sh], [0, sh]]), dst)
    cv2.warpPerspective(sheet, M, (W, H), dst=img, borderMode=cv2.BORDER_TRANSPARENT, flags=cv2.INTER_AREA)
    # 큐브 윗면 (50 mm 흰 면 + 40 mm 마커), 높이 cube_h
    cx, cy, yaw = cube
    cc, cs = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    top = np.full((500, 500), 255, np.uint8)
    top[50:450, 50:450] = cv2.aruco.drawMarker(PD._dictionary(cv2), PD.CUBE_ID, 400)
    top = cv2.cvtColor(top, cv2.COLOR_GRAY2BGR)
    pts = [(-0.025, 0.025), (0.025, 0.025), (0.025, -0.025), (-0.025, -0.025)]
    dst = np.float32([proj((cx + cc * u - cs * v, cy + cs * u + cc * v, cfg["cube_h"])) for u, v in pts])
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [500, 0], [500, 500], [0, 500]]), dst)
    cv2.warpPerspective(top, M, (W, H), dst=img, borderMode=cv2.BORDER_TRANSPARENT, flags=cv2.INTER_AREA)
    return cv2.GaussianBlur(img, (3, 3), 0)


CASES = [   # (이름, 종이 cfg 덮어쓰기, 큐브 (x, y, yaw), 카메라 (x, y, 높이), 초점 px)
    ("위에서", {}, (-0.06, -0.33, 20.0), (-0.05, -0.34, 0.60), 1150),
    ("앞에서 비스듬히", {}, (0.03, -0.31, -30.0), (-0.10, -0.75, 0.45), 1250),
    ("옆에서 비스듬히 + 종이 다른 자리", {"x": 0.02, "y": -0.30, "rot_deg": 8.0}, (-0.15, -0.34, 40.0), (0.45, -0.55, 0.50), 1400),
]


def main():
    save = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    bad = 0
    for k, (name, over, cube, cam, f) in enumerate(CASES):
        cfg = {**PD.DEFAULT_CFG, **over}
        ok, buf = cv2.imencode(".jpg", render(cfg, cube, cam, f), [cv2.IMWRITE_JPEG_QUALITY, 90])
        r = PD.detect(buf.tobytes(), cfg)
        if save:
            save.mkdir(parents=True, exist_ok=True)
            (save / f"phone_case{k}.jpg").write_bytes(buf.tobytes())
            cv2.imwrite(str(save / f"phone_case{k}_found.jpg"), r["annotated"])
        if not r["ok"]:
            print(f"✗ {name}: {r.get('reason')}"); bad += 1; continue
        c = r["cube"]
        dpos = math.hypot(c["x"] - cube[0], c["y"] - cube[1]) * 1000
        dyaw = abs((c["yaw_deg"] - cube[2] + 45) % 90 - 45)
        good = dpos <= 10 and dyaw <= 3
        bad += not good
        print(f"{'✓' if good else '✗'} {name}: 위치 오차 {dpos:.1f} mm, yaw 오차 {dyaw:.1f}° · 기준 {len(r['refs'])}개, "
              f"높이 보정 {r.get('height_shift_mm')} mm, 큐브 마커 {r.get('cube_marker_mm')} mm, 초점 {r.get('camera', {}).get('f_px')} (참 {f}) {r['warn'] or ''}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
