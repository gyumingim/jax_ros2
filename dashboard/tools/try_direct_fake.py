"""real_arm.RealArm 을 가짜 AK45 4개 + 가짜 STM32(fake_can_arm.py) 에 붙여 안전 규칙을 하나씩 확인한다. 실물 없음.

    python dashboard/tools/try_direct_fake.py                       # python-can virtual (Windows 가능)
    python3 dashboard/tools/try_direct_fake.py --interface socketcan --channel vcan0   # Linux vcan

calibration.json 은 임시 복사본을 쓴다(보정 저장 시험이 레포 파일을 건드리지 않게). 결과는 표로 출력.
"""
import argparse
import json
import math
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from fake_can_arm import FakeArm   # noqa: E402
import real_arm as R   # noqa: E402

rows = []


def check(name, ok, detail):
    rows.append((name, "OK" if ok else "FAIL", detail))
    print(f"[{'OK' if ok else 'FAIL'}] {name}: {detail}", flush=True)


def wait(cond, timeout, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="virtual")
    ap.add_argument("--channel", default="fake_can0")
    a = ap.parse_args()
    tmp = Path(tempfile.mkdtemp())
    cal = tmp / "calibration.json"
    shutil.copy(R.REPO / "arm_control" / "calibration.json", cal)

    fake = FakeArm(a.interface, a.channel)
    arm = R.RealArm(calibration_file=cal, interface=a.interface, channel=a.channel, lock_path=None,
                    log=lambda t: None, log_file=str(tmp / "direct.log"))
    t0 = time.monotonic()
    arm.start()
    ours = lambda f: f[2] and (f[1] >> 8) in (2, 4)         # 우리가 보낸 AK45 위치/브레이크
    rad = math.radians
    fb = lambda j: arm.ak[f"ak45_{j}"].pos
    homed = lambda j: arm.homing.m[f"ak45_{j}"].homed

    # 1. 첫 명령 전 무송신
    time.sleep(1.0)
    n0 = fake.count(ours)
    seen = all(m.fb_n > 10 for m in arm.ak.values())
    check("첫 명령 전 무송신", n0 == 0 and seen, f"1 s 동안 AK45 프레임 {n0}개, 피드백 수신 {[m.fb_n for m in arm.ak.values()]}")

    # 2. 원점 없을 때 차단
    r = arm.joint_command(["elbow"], [rad(10)])
    check("원점 없으면 AK45 명령 거절", not r["ok"] and fake.count(ours) == 0, r.get("error", r.get("msg")))

    # 3. 원점 → 램프 이동
    r = arm.call_trigger("/arm_control/home_all")
    check("원점 잡기(4개)", r["ok"] and all(homed(i) for i in (1, 2, 3, 4)), r.get("msg", r.get("error"))[:90] + "…")
    time.sleep(0.3)
    p_start = fake.pos(4)
    samples = []
    t1 = time.monotonic()
    arm.joint_command(["elbow"], [rad(20)])
    while time.monotonic() - t1 < 2.5:
        samples.append((time.monotonic(), fake.pos(4)))
        time.sleep(0.02)
    v = [abs(b[1] - a_[1]) / (b[0] - a_[0]) for a_, b in zip(samples, samples[5:])]
    arrive = next((t - t1 for t, p in samples if abs(p - 20) < 0.5), None)
    check("원점 → 보내기 → 램프 이동", abs(fake.pos(4) - 20) < 0.5 and max(v) < 33,
          f"elbow 0→20°: {arrive:.2f}s 에 도착, 최대 속도 {max(v):.1f} °/s (상한 30), 지금 {fake.pos(4):.2f}° (시작 {p_start:.2f})")

    # 4. 피드백 1.5 s 끊김 → 홀드 유지
    fake.silent(4, 1.5)
    k0 = fake.count(lambda f: ours(f) and (f[1] & 0xFF) == 4)
    time.sleep(1.0)
    mid = fake.pos(4)
    k1 = fake.count(lambda f: ours(f) and (f[1] & 0xFF) == 4)
    time.sleep(1.0)
    check("피드백 1.5 s 끊김 → 홀드 유지·복귀", abs(mid - 20) < 1 and abs(fake.pos(4) - 20) < 1 and k1 - k0 >= 15 and homed(4),
          f"끊긴 동안 1 s 에 홀드 프레임 {k1 - k0}개, 팔 {mid:.1f}° (안 떨어짐), 돌아온 뒤 원점 {'유지' if homed(4) else '지워짐'}")

    # 5. 7 s 끊김 → 멈춤·원점 지움, 돌아오면 지금 자리 홀드
    fake.silent(4, 7.0)
    time.sleep(4.5)
    during = fake.pos(4)
    time.sleep(1.0)
    stopped = arm.ak["ak45_4"].cmd is None
    k2 = fake.count(lambda f: ours(f) and (f[1] & 0xFF) == 4)
    time.sleep(0.5)
    k3 = fake.count(lambda f: ours(f) and (f[1] & 0xFF) == 4)
    fell = fake.pos(4)
    wait(lambda: arm.ak["ak45_4"].fresh(time.monotonic(), 0.25), 2.0)
    time.sleep(0.5)
    m4 = arm.ak["ak45_4"]
    check("피드백 7 s 끊김 → 멈춤·원점 지움", abs(during - 20) < 1 and stopped and k3 == k2 and not homed(4),
          f"4.5 s 까진 홀드({during:.1f}°), 5 s 넘자 송신 멈춤({k3 - k2}개/0.5 s), 원점 {'지워짐' if not homed(4) else '남음'}, 팔 {fell:.1f}°로 내려감")
    check("긴 끊김 뒤 복귀 → 지금 자리 홀드", m4.target is not None and abs(m4.target - fake.pos(4)) < 2 and abs(fake.pos(4) - 20) > 5,
          f"목표 {m4.target if m4.target is None else round(m4.target, 1)}° = 지금 자리 {fake.pos(4):.1f}° (옛 목표 20° 로 안 감)")

    # 6. 60° 튐
    arm.call_trigger("/arm_control/home_elbow")
    time.sleep(0.3)
    before = fake.m[4].phys
    with fake.lock:
        fake.m[4].enc_off += 70.0
    time.sleep(0.6)
    check("위치 60° 튐 → 지금 자리 재홀드 + 원점 지움", not homed(4) and abs(fake.m[4].phys - before) < 1.5 and arm.ak["ak45_4"].target is not None,
          f"엔코더 +70°: 실제 축 {before:.1f}→{fake.m[4].phys:.1f}° (안 움직임), 원점 {'지워짐' if not homed(4) else '남음'}, 메모: {arm.ak['ak45_4'].note}")

    # 7. 막힘 → 힘 뺌
    arm.call_trigger("/arm_control/home_elbow")
    time.sleep(0.2)
    fake.m[4].grab = True
    b0 = fake.m[4].brakes
    tg = time.monotonic()
    arm.joint_command(["elbow"], [rad(40)])
    ok = wait(lambda: arm.ak["ak45_4"].fault is not None, 5.0)
    t_stall = time.monotonic() - tg
    fake.m[4].grab = False
    check("막힘(붙잡음) → 힘 뺌", ok and fake.m[4].brakes > b0 and not homed(4),
          f"{t_stall:.1f}s 뒤: {arm.ak['ak45_4'].fault} · 브레이크 {fake.m[4].brakes - b0}개")
    r = arm.joint_command(["elbow"], [rad(0)])
    check("고장 관절 명령 거절", not r["ok"], r.get("error"))

    # 8. 온도
    arm.call_trigger("/arm_control/home_upper_arm_roll")
    time.sleep(0.2)
    fake.m[3].temp = 62
    time.sleep(0.3)
    warned = any("온도 62" in x for x in arm.lines)
    held = arm.ak["ak45_3"].cmd is not None
    fake.m[3].temp = 76
    ok = wait(lambda: arm.ak["ak45_3"].fault is not None, 1.0)
    check("온도 60 경고 / 75 힘 뺌", warned and held and ok and not homed(3),
          f"62 ℃: 경고{'함' if warned else ' 없음'}·홀드 {'유지' if held else '끊김'} / 76 ℃: {arm.ak['ak45_3'].fault}")
    r = arm.call_trigger("/arm_control/home_upper_arm_roll")
    check("뜨거운 동안 원점 거절", not r["ok"], r.get("error"))
    fake.m[3].temp = 40
    time.sleep(0.2)

    # 9. 에러 코드
    fake.m[2].err = 2
    ok = wait(lambda: arm.ak["ak45_2"].fault is not None, 1.0)
    check("에러 코드 → 그 관절 힘 뺌", ok, f"{arm.ak['ak45_2'].fault}")
    fake.m[2].err = 0
    time.sleep(0.2)

    # 10. STOP = 홀드
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.3)
    arm.joint_command(["shoulder_roll"], [rad(40)])
    time.sleep(0.8)
    r = arm.hold_all()
    at = fake.pos(1)
    time.sleep(1.5)
    check("STOP = 지금 자리 홀드(안 떨어짐)", abs(fake.pos(1) - at) < 2 and 3 < at < 38 and arm.ak["ak45_1"].cmd is not None,
          f"40° 로 가던 중 STOP: {at:.1f}° 에서 섬, 1.5 s 뒤 {fake.pos(1):.1f}°, 홀드 프레임 계속")

    # 11. 서보
    s0 = dict(fake.servo)
    arm.joint_command(["wrist_pitch", "hand_roll"], [rad(10), rad(-20)])
    time.sleep(0.2)
    first = dict(fake.servo)
    arm.joint_command(["wrist_pitch"], [rad(40)])
    time.sleep(0.4)
    mid = fake.servo.get(8)
    time.sleep(1.2)
    check("서보: 첫 명령 바로, 다음은 램프", first.get(8) == 100.0 and first.get(7) == 70.0 and mid is not None and 100 < mid < 130 and fake.servo.get(8) == 130.0,
          f"W_YAW(id 8) 첫 명령 {s0.get(8)}→{first.get(8)}°(바로), 130° 로 램프 중 0.4 s 에 {mid}°, 끝 {fake.servo.get(8)}° · W_X {first.get(7)}°")

    # 12. 궤적 재생 + 추종 감시
    names = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow"]
    here = [math.radians(arm.joints[n].to_sim(arm.homing.from_raw(f"ak45_{arm.joints[n].id}", arm.ak[f"ak45_{arm.joints[n].id}"].pos))) for n in names]
    pos = [[here[0] + rad(10) * k / 40, here[1], here[2], here[3] + rad(15) * k / 40] for k in range(41)]
    traj = {"joint_names": names, "dt": 0.05, "positions": pos, "status": "playable"}
    r = arm.play_trajectory(traj, 30.0, hold=0.5)
    wait(lambda: not arm.play.get("running"), 10)
    check("궤적 재생(끝까지)", r["ok"] and arm.play.get("reason") == "끝까지 재생",
          f"{r.get('msg', r.get('error'))} → {arm.play.get('reason')}, elbow {fake.pos(4):.1f}°")
    pos2 = [[p[0], p[1], p[2], p[3] + rad(60) * k / 40] for k, p in enumerate(reversed(pos))]
    traj2 = dict(traj, positions=pos2)
    fake.m[4].grab = True
    arm.play_trajectory(traj2, 30.0, hold=0.3)
    wait(lambda: not arm.play.get("running"), 15)
    fake.m[4].grab = False
    check("재생 중 추종 벗어남 → 멈춤·그 자리 홀드", "ak45_4" in str(arm.play.get("reason")) and arm.ak["ak45_1"].cmd is not None,
          f"elbow 붙잡음 → {arm.play.get('reason')}")

    # 13. 보정 (calibrate Session 의 m/z/f/v/w)
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.2)
    r1 = arm.calib("move", "elbow", 5)
    time.sleep(1.0)
    r2 = arm.calib("zero", "elbow")
    z = arm.joints["elbow"].zero_deg
    r3 = arm.calib("flip", "elbow")
    r4 = arm.calib("verify", "elbow", True)
    r5 = arm.calib("save", None)
    saved = json.loads(cal.read_text())
    je = next(j for j in saved["joints"] if j["joint"] == "elbow")
    rbig = arm.calib("move", "elbow", 30)
    check("보정: ±N° · 영점 · 방향 · 확인 · 저장", all(x["ok"] for x in (r1, r2, r3, r4, r5)) and je["zero_deg"] == z and je["sign"] == -1
          and je["verified"] is True and not rbig["ok"],
          f"elbow +5° → zero_deg {z:g}, sign -1, verified 저장됨(임시 복사본) · +30° 는 거절({rbig.get('error')})")

    # 14. 힘 빼기
    p0 = fake.count(lambda f: f[2] and (f[1] >> 8) == 4)
    b0 = sum(m.brakes for m in fake.m.values())
    r = arm.limp()
    time.sleep(0.5)
    p1 = fake.count(lambda f: f[2] and (f[1] >> 8) == 4)
    time.sleep(0.5)
    p2 = fake.count(lambda f: f[2] and (f[1] >> 8) == 4)
    nb = sum(m.brakes for m in fake.m.values()) - b0
    check("힘 빼기 = 프레임 중단 + 브레이크 0 A", nb >= 12 and p2 == p1 and not any(homed(i) for i in (1, 2, 3, 4)),
          f"브레이크 {nb}개, 그 뒤 위치 프레임 {p2 - p1}개, 원점 전부 지움, 팔 elbow {fake.pos(4):.1f}° 로 내려감")
    r = arm.calib("move", "elbow", 3)

    # 15. 프레임 바이트
    br = R.ak45_brake_frame(3)
    check("브레이크 프레임 = ak45_set_current_brake(0 A)", str(br) == "00000203#00000000", str(br))

    arm.close()
    fake.close()
    print()
    print(f"| 시험 | 결과 | 내용 |\n|---|---|---|")
    for n, s, d in rows:
        print(f"| {n} | {s} | {d} |")
    print(f"\n{sum(1 for r in rows if r[1] == 'OK')}/{len(rows)} OK · {time.monotonic() - t0:.0f} s · 로그 {tmp / 'direct.log'}")
    return 0 if all(r[1] == "OK" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
