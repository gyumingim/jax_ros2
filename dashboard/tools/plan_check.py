"""계획 회귀 검사: 지도(graspmap)의 초록 칸 둘을 무작위로 골라 pick_plan.plan 을 N 번 돌려 'playable' 비율을 본다.

    python dashboard/tools/plan_check.py gm.json 30 1          # gm.json = GET /api/ik/graspmap 저장본, N, seed
    python dashboard/tools/plan_check.py --make-map gm.json    # 지도를 이 프로세스에서 계산해 저장 (서버 없이)

한 줄씩 JSON (큐브, yaw, 놓을 자리, 상태, 이유, 시간, 주요 검사값), 끝에 상태별 개수. 결과는 gm.json 옆 plan_check_<seed>.json.
yaw 는 (0, 20, 45, -30, 70) 중 무작위 (큐브는 90° 마다 같아 이것으로 0~90 을 고루 본다).
"""
import argparse
import collections
import json
import os
import random
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pick_plan   # noqa: E402


def make_map(path):
    import hand_ik
    s = hand_ik._Solver(str(pick_plan._find_ik(None)))
    ranges = pick_plan._cmd_ranges(pick_plan.REPO / "arm_control" / "calibration.json")
    x0, x1, y0, y1 = hand_ik.MAP_BOX
    r = s.graspmap({"ranges_deg": ranges, "shoulder_yaw": 0.0, "step": 0.01, "x0": x0, "x1": x1, "y0": y0, "y1": y1})
    Path(path).write_text(json.dumps(r), encoding="utf-8")
    print(f"map: {r['counts']} {r['ms']} ms -> {path}")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("gm")
    ap.add_argument("n", nargs="?", type=int, default=30)
    ap.add_argument("seed", nargs="?", type=int, default=1)
    ap.add_argument("--make-map", action="store_true")
    a = ap.parse_args()
    if a.make_map:
        return make_map(a.gm)
    gm = json.loads(Path(a.gm).read_text(encoding="utf-8"))
    green = [(x, y) for y, r in zip(gm["ys"], gm["rows"]) for x, c in zip(gm["xs"], r) if c == "2"]
    random.seed(a.seed)
    out = []
    for _ in range(a.n):
        c = random.choice(green); t = random.choice(green); yaw = random.choice([0, 20, 45, -30, 70])
        ns = argparse.Namespace(cube=[c[0], c[1], yaw], target=list(t), target_yaw=None, d=None, ik=None, calibration=None, max_speed=30.0)
        t0 = time.time()
        try:
            r = pick_plan.plan(ns)
            row = {"cube": c, "yaw": yaw, "target": t, "status": r["status"], "reason": r.get("reason"), "s": round(time.time() - t0, 1),
                   "checks": {k: r.get("checks", {}).get(k) for k in ("lowest_finger_above_table_mm", "joint_space_fallback",
                                                                       "carry_cube_bottom_min_above_table_mm", "approach")},
                   "n": len(r.get("positions") or []), "tilt": (r.get("sim") or {}).get("hand_tilt_deg")}
        except Exception:
            row = {"cube": c, "yaw": yaw, "target": t, "status": "EXC", "reason": traceback.format_exc()[-600:], "s": round(time.time() - t0, 1)}
        out.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    cnt = collections.Counter(r["status"] for r in out)
    print(f"summary: playable {cnt['playable']}/{len(out)} ({100 * cnt['playable'] / max(1, len(out)):.0f}%) · " +
          ", ".join(f"{k} {v}" for k, v in cnt.most_common()), flush=True)
    Path(a.gm).with_name(f"plan_check_{a.seed}.json").write_text(json.dumps(out, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
