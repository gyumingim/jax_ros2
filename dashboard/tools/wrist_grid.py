"""손목 중앙 기준 IK 격자 시험 (서버 /api/ik/hand, /api/ik/fk 사용).

    python dashboard/tools/wrist_grid.py --port 8098 [--mode vertical] [--yaw 0]

격자 24점(x −0.10…0.15 × y −0.35…−0.20) + 사용자 점(−0.03, −0.36). 높이 = 집는 점 책상 위 2.5 cm 에 해당하는 손목 높이
(수직 손이면 + wrist_above_grip_vertical_m). 어깨 옆 고정 켬/끔. 풀린 해는 FK 로 손목 중앙 위치를 다시 재서 목표와 mm 차이.
"""
import argparse
import json
import sys
import urllib.request

sys.stdout.reconfigure(encoding="utf-8")
ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8098)
ap.add_argument("--mode", default="vertical")
ap.add_argument("--yaw", type=float, default=0.0)
ap.add_argument("--zc", type=float, default=None, help="손목 높이 cm (없으면 집는 점 2.5 cm 에 해당하는 수직 손 값)")
ap.add_argument("--quiet", action="store_true")
a = ap.parse_args()
U = f"http://127.0.0.1:{a.port}"


def post(path, body):
    rq = urllib.request.Request(U + path, data=json.dumps(body).encode(), headers={"content-type": "application/json"})
    try:
        with urllib.request.urlopen(rq, timeout=120) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


info = post("/api/ik/fk", {"mode": a.mode})["ref_info"]
above = info["wrist_above_grip_vertical_m"]
zc = a.zc if a.zc is not None else 2.5 + above * 100
print(f"ref_info {info} → 손목 높이 {zc:.2f} cm (집는 점 2.5 cm, 수직 손)")
pts = [(x, y) for y in (-0.35, -0.30, -0.25, -0.20) for x in (-0.10, -0.05, 0.0, 0.05, 0.10, 0.15)] + [(-0.03, -0.36)]
for fix in (False, True):
    n_ok, errs, fails = 0, [], []
    for x, y in pts:
        b = {"x": x, "y": y, "z_table_cm": zc, "yaw_deg": a.yaw, "mode": a.mode, "ref": "wrist", "fix_deg": {"shoulder_yaw": 0} if fix else {}}
        r = post("/api/ik/hand", b)
        if r.get("ok"):
            n_ok += 1
            fk = post("/api/ik/fk", {"mode": a.mode, "joints": r["joints"]})
            w = fk["wrist_real"]
            e = ((w[0] - x) ** 2 + (w[1] - y) ** 2 + (w[2] - (zc / 100 - 0.4775)) ** 2) ** 0.5 * 1000
            errs.append(e)
            g = fk["grip_real"]
            tag = f"ok 손목 FK 오차 {e:.1f} mm · 집는 점 책상 위 {(g[2] + 0.4775) * 100:.1f} cm · {r['ms']} ms"
        else:
            tag = f"안 됨: {r.get('reason')} (가장 가까운 해 {r.get('pos_err_mm')} mm · {r.get('ang_err_deg')}°)"
            fails.append((x, y))
        if (x, y) == (-0.03, -0.36) or (not r.get("ok") and not a.quiet):
            print(f"  fix={fix} ({x:+.2f},{y:+.2f}) {tag}")
    print(f"고정 {'켬' if fix else '끔'}: 24점+사용자 점 중 {n_ok}/{len(pts)} 풀림 · 손목 FK 오차 최대 {max(errs) if errs else 0:.1f} mm")
