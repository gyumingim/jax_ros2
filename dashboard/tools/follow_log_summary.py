"""dashboard/logs/follow.log 요약: 구간(mark)마다 보낸 수·안 보낸 이유 개수, 20° 초과 순간 직전·직후 해 비교.

    python dashboard/tools/follow_log_summary.py [log] [--detail N]
"""
import collections
import json
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
args = [x for x in sys.argv[1:] if not x.startswith("--")]
detail = int(sys.argv[sys.argv.index("--detail") + 1]) if "--detail" in sys.argv else 3
p = Path(args[0]) if args else Path(__file__).resolve().parents[1] / "logs" / "follow.log"
rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
ARM7 = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow", "wrist_pitch", "wrist_yaw", "hand_roll"]


def qs(q):
    return " ".join(f"{(q or {}).get(n, float('nan')):7.1f}" for n in ARM7) if q else "-"


seg, cnt, shown = "(처음)", collections.Counter(), 0
last_ok_solve = None
prev_solve = None
for r in rows:
    ev = r.get("ev")
    if ev == "mark":
        if cnt:
            print(f"  {seg}: " + ", ".join(f"{k} {v}" for k, v in cnt.most_common()))
        seg, cnt = r.get("why"), collections.Counter()
        print("== " + seg)
        continue
    if ev == "solve":
        cnt[f"풀이 {r.get('kind')} {'ok' if r.get('ok') else '실패'}"] += 1
        if prev_solve and r.get("ok") and prev_solve.get("ok"):
            jump = max(abs(r["q"][n] - prev_solve["q"][n]) for n in ARM7)
            if jump > 20 and shown < detail * 4:
                shown += 1
                print(f"  [가지 점프 {jump:.0f}°] {prev_solve['kind']}→{r['kind']} flip {prev_solve.get('flip')}→{r.get('flip')} "
                      f"turn {prev_solve.get('turn')}→{r.get('turn')} tilt {prev_solve.get('tilt')}→{r.get('tilt')} "
                      f"tgt {r.get('tgt')}")
                print(f"     전 {qs(prev_solve['q'])}\n     후 {qs(r['q'])}")
        if r.get("ok"):
            prev_solve = r
        continue
    if ev == "tick":
        w = r.get("why", "")
        cnt[("보냄" if r.get("sent") else w.split(":")[0])] += 1
        continue
    cnt[ev] += 1
if cnt:
    print(f"  {seg}: " + ", ".join(f"{k} {v}" for k, v in cnt.most_common()))

# 전체: 보낸 틱의 관절 최대 변화(직전 보낸 해 대비), 연속한 좋은 풀이 사이 최대 변화
sent_d = sorted(r["d"] for r in rows if r.get("ev") == "tick" and r.get("sent") and r.get("d") is not None)
over = [r["d"] for r in rows if r.get("ev") == "tick" and str(r.get("why", "")).startswith("20°")]
jumps, prev = [], None
for r in rows:
    if r.get("ev") == "solve" and r.get("ok") and r.get("q"):
        if prev:
            jumps.append(max(abs(r["q"][n] - prev[n]) for n in ARM7))
        prev = r["q"]
jumps.sort()
med = lambda a: a[len(a) // 2] if a else None
print(f"\n보냄 {len(sent_d)} 틱 · 틱당 최대 관절 변화 중앙 {med(sent_d)}° 최대 {sent_d[-1] if sent_d else None}° · "
      f"20° 초과로 안 보낸 틱 {len(over)} · 풀이 사이 변화 중앙 {med(jumps):.1f}° 최대 {jumps[-1] if jumps else 0:.1f}° · 20° 넘는 풀이 점프 {sum(j > 20 for j in jumps)}")
why = collections.Counter(("보냄" if r.get("sent") else str(r.get("why", "")).split(":")[0].split("(")[0].strip()) for r in rows if r.get("ev") == "tick")
print("틱 이유 전체: " + ", ".join(f"{k} {v}" for k, v in why.most_common()))
