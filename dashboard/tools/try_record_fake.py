"""run_record 훅이 50 Hz 제어 루프를 늦추지 않는지 + joints.csv 줄 수를 가짜 AK45·STM32 (fake_can_arm.py) 로 잰다. 실물 없음.

    python3 dashboard/tools/try_record_fake.py [--seconds 8]

같은 동작(원점 → 팔꿈치 0↔30° 왕복)을 기록 끔 / 켬으로 한 번씩: 틱 간격(평균·p99·최대), 잠금 안 시간(_tick + 훅) 평균·p99,
기록 켬이면 joints.csv 줄 수 / 초. 임시 폴더만 쓴다 (calibration.json 은 복사본).
"""
import argparse
import csv
import math
import shutil
import statistics as st
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from fake_can_arm import FakeArm   # noqa: E402
import real_arm as R   # noqa: E402
import run_record   # noqa: E402


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    a = ap.parse_args()
    tmp = Path(tempfile.mkdtemp())
    cal = tmp / "calibration.json"
    shutil.copy(R.REPO / "arm_control" / "calibration.json", cal)
    fake = FakeArm("virtual", "rec_can0")
    arm = R.RealArm(calibration_file=cal, interface="virtual", channel="rec_can0", lock_path=None, log_file=str(tmp / "direct.log"))
    starts, durs = [], []
    tick0 = arm._tick

    def timed(now, dt):          # _tick 시작 시각 + (_tick + 훅) 시간: 훅은 _tick 바로 뒤라 다음 틱 시작 전까지 = 잠금 안 시간
        starts.append(time.perf_counter())
        tick0(now, dt)
    arm._tick = timed

    arm.start()
    time.sleep(1.0)
    arm.home(list(arm.ak))
    servos = [j.joint for j in arm.cal.joints if j.motor == "servo"]
    arm.joint_command(servos, [0.0] * len(servos))

    def phase(rec_on):
        starts.clear()
        rec = None
        if rec_on:
            rec = run_record.Recorder()
            rec.setup(tmp / "logs", lambda: (arm, False), lambda: "direct", lambda: "fake")
            rec.start_manual("manual")
        # 잠금 안 시간: 훅 포함 (_ctrl_loop 의 with self.lock 블록) — rec 를 감싸 끝 시각을 잰다
        if rec_on:
            inner = arm.rec

            def wrap(s, now):
                inner(s, now)
                durs.append(time.perf_counter() - starts[-1])
            arm.rec = wrap
        else:
            def wrap(s, now):
                durs.append(time.perf_counter() - starts[-1])
            arm.rec = wrap
        time.sleep(0.5)
        starts.clear()
        durs.clear()
        t_end = time.monotonic() + a.seconds
        k = 0
        while time.monotonic() < t_end:
            arm.joint_command(["elbow"], [math.radians(30.0 if k % 2 == 0 else 0.0)])
            k += 1
            time.sleep(1.0)
        n = len(starts)
        gaps = [(b - a_) * 1000 for a_, b in zip(starts, starts[1:])]
        d = [x * 1000 for x in durs]
        out = {"ticks": n, "hz": round((n - 1) / (starts[-1] - starts[0]), 2), "gap_mean_ms": round(st.mean(gaps), 3),
               "gap_p99_ms": round(pct(gaps, 0.99), 3), "gap_max_ms": round(max(gaps), 3),
               "lock_mean_us": round(st.mean(d) * 1000, 1), "lock_p99_us": round(pct(d, 0.99) * 1000, 1)}
        if rec_on:
            run = rec.cur
            arm.rec = inner
            rec.stop("시험 끝")
            with open(run.dir / "joints.csv") as f:
                rows = list(csv.reader(f))
            ts = [float(r[0]) for r in rows[1:]]
            out.update(csv_rows=len(rows) - 1, csv_hz=round((len(ts) - 1) / (ts[-1] - ts[0]), 2), cols=len(rows[0]), dir=str(run.dir))
        arm.rec = None
        return out

    off = phase(False)
    on = phase(True)
    arm.close()
    print("기록 끔:", off)
    print("기록 켬:", on)
    ok = on["hz"] > 49.0 and abs(on["gap_mean_ms"] - off["gap_mean_ms"]) < 0.5 and 48.0 <= on["csv_hz"] <= 51.0
    print("OK" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
