"""해석 IK 검증 (서버 없이, 같은 프로세스에서 hand_ik._Solver):

    JAX_DASH_PYTHONPATH=C:/Users/a3162/Documents/GitHub/IK/.pydeps python dashboard/tools/analytic_check.py [--n 1000]

(a) 무작위 관절각(shoulder_yaw 고정, 손목 3 관절도 무작위) → FK 손목 중앙 → 해석 IK 해 목록에 원래 각이 있나 (복원율, 오차)
(b) 24점 + 사용자 점 2개, 손목 높이 27 cm: 해석 vs 예전 수치(JAX_IK_NUMERIC=1, 같은 조건) 풀림 수 · 수치가 찾은 해가 해석 목록에 있나
(d) 풀이 시간
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=1000)
ap.add_argument("--ik", default=os.environ.get("JAX_IK_DIR", r"C:\Users\a3162\Documents\GitHub\IK"))
ap.add_argument("--fix", type=float, default=0.0, help="shoulder_yaw 고정값 (°)")
ap.add_argument("--zc", type=float, default=27.0, help="손목 중앙 높이 (책상 위 cm)")
a = ap.parse_args()

import warnings
warnings.filterwarnings("ignore")
import numpy as np
from hand_ik import A3, _Solver

s = _Solver(a.ik)
lim = s.set_ranges({"shoulder_yaw": [a.fix, a.fix]})
rng = np.random.default_rng(0)
idx = [s.arm.index(n) for n in A3]
print(f"관절 {s.arm}\n범위(°) {np.degrees(lim).round(1).tolist()}\n손목 중앙 축 간격 {s.wrist_axes_gap_mm:.4f} mm")

# ---- (a) 복원
hit, errs, nsol, ts, worst = 0, [], [], [], 0.0
for k in range(a.n):
    q = rng.uniform(lim[:, 0], lim[:, 1])
    s.fk(q)
    p = s.wrist_pos().copy()
    t0 = time.perf_counter()
    qs, n_raw = s.analytic_position(p, q, lim)
    ts.append((time.perf_counter() - t0) * 1000)
    nsol.append(len(qs))
    dd = [float(np.abs(x[idx] - q[idx]).max()) for x in qs]
    if dd and min(dd) < 1e-5:
        hit += 1
        worst = max(worst, min(dd))
    else:
        print(f"  복원 실패 #{k}: q(°)={np.degrees(q).round(2).tolist()} 해 {len(qs)} (범위 전 {n_raw}) 가장 가까운 {math.degrees(min(dd)) if dd else None}")
    for x in qs:
        s.fk(x)
        errs.append(float(np.linalg.norm(s.wrist_pos() - p)))
print(f"(a) 복원 {hit}/{a.n} · 원래 각과 최대 차이 {math.degrees(worst):.2e}° · 해 FK 오차 최대 {max(errs) * 1000:.2e} mm · "
      f"해 수 분포 {dict(zip(*np.unique(nsol, return_counts=True)))} · 시간 중앙 {np.median(ts):.2f} ms, 최대 {max(ts):.2f} ms")

# ---- (b) 격자: 해석 vs 수치 (같은 조건: 기준 자세 HOME, 손목 3 관절 HOME 값)
pts = [(x, y) for y in (-0.35, -0.30, -0.25, -0.20) for x in (-0.10, -0.05, 0.0, 0.05, 0.10, 0.15)] + [(-0.03, -0.36), (0.04, -0.25)]
z = a.zc / 100 - 0.4775
res = {}
for meth in ("analytic", "numeric"):
    os.environ["JAX_IK_NUMERIC"] = "1" if meth == "numeric" else "0"
    rows = []
    for x, y in pts:
        b = {"x": x, "y": y, "z": z, "mode": "position", "ref": "wrist", "fix_deg": {"shoulder_yaw": a.fix}}
        t0 = time.perf_counter()
        r = s.solve(b)
        rows.append((r, (time.perf_counter() - t0) * 1000))
    res[meth] = rows
os.environ["JAX_IK_NUMERIC"] = "0"
na = sum(r["ok"] for r, _ in res["analytic"])
nn = sum(r["ok"] for r, _ in res["numeric"])
print(f"(b) 손목 높이 {a.zc} cm, 고정 {a.fix}°: 풀림 해석 {na}/{len(pts)} · 수치 {nn}/{len(pts)}")
bad = 0
for (x, y), (ra, ta), (rn, tn) in zip(pts, res["analytic"], res["numeric"]):
    note = ""
    if rn["ok"] and not ra["ok"]:
        bad += 1; note = "  ← 수치만 풀림 (버그)"
    if rn["ok"] and ra["ok"]:
        qn = np.array([rn["joints"][n] for n in A3])
        inlist = min((max(abs(qn[i] - sol["joints"][n]) for i, n in enumerate(A3)) for sol in ra.get("solutions") or []), default=999.0)
        same = max(abs(rn["joints"][n] - ra["joints"][n]) for n in A3)
        note = f"  수치 해가 해석 목록에 {'있음' if inlist < 0.5 else f'없음({inlist:.1f}°)'} · 고른 해 차이 {same:.1f}°"
        if ra.get("n_solutions") == 0:
            note = f"  해석 정확해 없음(범위 밖) → 수치 한계 해 {rn['pos_err_mm']} mm 로 풀림 (같음)"
            if ra.get("analytic_miss"):
                bad += 1; note += " ← analytic_miss (버그)"
        elif inlist >= 0.5:
            bad += 1
    print(f"  ({x:+.2f},{y:+.2f}) 해석 {'ok' if ra['ok'] else 'X'} 해 {ra.get('n_solutions')} (범위 전 {ra.get('n_raw')}) {ta:.1f} ms"
          f" · 수치 {'ok' if rn['ok'] else 'X'} {tn:.0f} ms{note}" + ("" if ra["ok"] else f" · {ra.get('reason')}"))
print(f"    해석이 놓친 것 {bad} · 시간 중앙 해석 {np.median([t for _, t in res['analytic']]):.1f} ms, 수치 {np.median([t for _, t in res['numeric']]):.0f} ms")
