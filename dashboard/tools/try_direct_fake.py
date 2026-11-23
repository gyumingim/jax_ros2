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
import threading
import time
import types
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
    fake.reset()
    time.sleep(0.2)

    # 9. 에러 코드
    fake.m[2].err = 2
    ok = wait(lambda: arm.ak["ak45_2"].fault is not None, 1.0)
    check("에러 코드 → 그 관절 힘 뺌", ok, f"{arm.ak['ak45_2'].fault}")
    fake.reset()
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
    jw, jh = arm.joints["wrist_pitch"], arm.joints["hand_roll"]      # 기대값은 보정(영점·핀)에서 — 보정이 바뀌어도 시험이 맞게
    iw, ih = jw.id, jh.id
    a10, a40, h20 = round(jw.to_motor(10)[0], 1), round(jw.to_motor(40)[0], 1), round(jh.to_motor(-20)[0], 1)
    mid = fake.servo.get(iw)
    time.sleep(1.2)
    check("서보: 첫 명령 바로, 다음은 램프", first.get(iw) == a10 and first.get(ih) == h20 and mid is not None
          and min(a10, a40) < mid < max(a10, a40) and fake.servo.get(iw) == a40,
          f"{jw.name}(id {iw}) 첫 명령 {s0.get(iw)}→{first.get(iw)}°(바로), {a40}° 로 램프 중 0.4 s 에 {mid}°, 끝 {fake.servo.get(iw)}° · {jh.name} {first.get(ih)}°")

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
    rs = arm.play_trajectory({"joint_names": names + ["finger1_joint1"], "dt": 0.05,
                              "positions": [p + [0.0] for p in pos], "status": "playable"}, 30.0, hold=0.2)
    check("안 보낸 서보가 든 궤적 재생 거절", not rs["ok"] and "finger1_joint1" in rs.get("error", "") and not arm.play.get("running"),
          rs.get("error", rs.get("msg")))
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

    review_tests(a, cal, tmp, check, wait)
    print()
    print(f"| 시험 | 결과 | 내용 |\n|---|---|---|")
    for n, s, d in rows:
        print(f"| {n} | {s} | {d} |")
    print(f"\n{sum(1 for r in rows if r[1] == 'OK')}/{len(rows)} OK · {time.monotonic() - t0:.0f} s · 로그 {tmp / 'direct.log'}")
    return 0 if all(r[1] == "OK" for r in rows) else 1


class _Die(BaseException):
    """rx 스레드를 진짜로 죽이는 시험용 (Exception 이 아니라 rx 보호에 안 걸림)."""


def review_tests(a, cal, tmp, check, wait):
    """2026-11-20 독립 검사 지적(1·2·3·4·5·7·9·10) 시험. 시험마다 새 가짜 팔 + 새 RealArm(상태가 안 섞이게)."""
    rad = math.radians
    k = [0]

    def rig(home=True):
        k[0] += 1
        ch = f"{a.channel}_r{k[0]}" if a.interface == "virtual" else a.channel
        fake = FakeArm(a.interface, ch)
        arm = R.RealArm(calibration_file=cal, interface=a.interface, channel=ch, lock_path=None,
                        log=lambda t: None, log_file=str(tmp / "direct.log"))
        arm.start()
        wait(lambda: all(m.fresh(time.monotonic(), 0.25) for m in arm.ak.values()), 2.0)
        if home:
            arm.call_trigger("/arm_control/home_all")
            time.sleep(0.2)
        return fake, arm

    def done(fake, arm):
        arm.close()
        fake.close()

    names = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow"]

    # R1. 재생 첫 프레임 홀드(5 s) 중 STOP → 그 자리 홀드, 첫 프레임으로 다시 안 감
    fake, arm = rig()
    traj = {"joint_names": names, "dt": 0.05, "positions": [[0, 0, 0, rad(90)]] * 6, "status": "playable"}

    # R0. 재생 시작 규칙: 첫 프레임이 멀면 거절 / 계획 중 STOP 이면 거절 / 저장 안 한 보정이면 거절 / 동시에 둘이면 하나만
    r = arm.play_trajectory(traj, 30.0, hold=0.3)
    check("첫 프레임 90° 떨어짐 → 재생 거절", not r["ok"] and "첫 프레임" in r.get("error", "") and not arm.play.get("running")
          and abs(fake.pos(4)) < 1, r.get("error", r.get("msg"))[:110])
    near = dict(traj, positions=[[0, 0, 0, rad(2)]] * 6)
    g = arm.stop_gen
    arm.hold_all("STOP")
    r = arm.play_trajectory(near, 30.0, hold=0.3, stop_gen=g)
    check("계획 중 STOP → 재생 거절", not r["ok"] and "STOP" in r.get("error", "") and not arm.play.get("running"), r.get("error", r.get("msg")))
    arm.calib_dirty = True
    r = arm.play_trajectory(near, 30.0, hold=0.3)
    arm.calib_dirty = False
    check("보정 저장 안 함 → 재생 거절", not r["ok"] and "저장" in r.get("error", ""), r.get("error", r.get("msg"))[:80])
    res = []
    ts = [threading.Thread(target=lambda: res.append(arm.play_trajectory(near, 30.0, hold=0.3))) for _ in range(4)]
    [t.start() for t in ts]; [t.join() for t in ts]
    nok = sum(1 for x in res if x["ok"])
    wait(lambda: not arm.play.get("running"), 5)
    check("동시에 재생 4개 → 하나만 시작", nok == 1 and all("이미 재생 중" in x.get("error", "") for x in res if not x["ok"]),
          f"시작 {nok}개, 나머지: {[x.get('error') for x in res if not x['ok']][:1]}")
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.2)

    arm.play_trajectory(traj, 30.0, hold=5.0, start_tol=999)    # 아래 시험은 첫 프레임 홀드 동작을 본다(시작 자세 검사 끔)
    time.sleep(0.5)
    p_stop = fake.pos(4)
    arm.hold_all("STOP")
    ended = wait(lambda: not arm.play.get("running"), 1.0)
    time.sleep(2.0)
    check("재생 첫 프레임 홀드 중 STOP → 그 자리", ended and abs(fake.pos(4) - p_stop) < 2 and arm.play.get("reason") == "stopped",
          f"STOP 때 elbow {p_stop:.1f}°, 2 s 뒤 {fake.pos(4):.1f}° (첫 프레임 90° 로 안 감), 재생 끝: {arm.play.get('reason')}")

    # R2. 첫 프레임 홀드 중 '재생 멈춤'(stop_trajectory) → 끝에 hold_all
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.2)
    arm.play_trajectory(traj, 30.0, hold=5.0, start_tol=999)
    time.sleep(0.5)
    arm.stop_trajectory()
    wait(lambda: not arm.play.get("running"), 1.0)
    p_stop = fake.pos(4)
    time.sleep(1.5)
    check("재생 멈춤 버튼 → 지금 자리 홀드", abs(fake.pos(4) - p_stop) < 2 and arm.ak["ak45_4"].cmd is not None,
          f"멈춤 뒤 elbow {p_stop:.1f}° → 1.5 s 뒤 {fake.pos(4):.1f}°, 홀드 {'계속' if arm.ak['ak45_4'].cmd is not None else '끊김'}")

    # R3. 프레임 재생 중 STOP → 목표가 STOP 자리에서 안 바뀜(한 프레임 레이스 없음)
    pos = [[0, 0, 0, rad(-60) * i / 80] for i in range(81)]
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.2)
    arm.play_trajectory(dict(traj, positions=pos), 30.0, hold=0.2)
    time.sleep(1.2)
    arm.hold_all("STOP")
    t_stop = arm.ak["ak45_4"].target
    time.sleep(1.0)
    t_after = arm.ak["ak45_4"].target
    check("프레임 재생 중 STOP → 목표 안 덮어씀", not arm.play.get("running") and t_stop is not None and abs(t_after - t_stop) < 1.0,
          f"STOP 목표 {t_stop:+.1f}° → 1 s 뒤 {t_after:+.1f}°, 팔 {fake.pos(4):+.1f}°")
    done(fake, arm)

    # R4. 5 s 이내 짧은 전원 재투입 + 엔코더 영점 30°(60° 미만) → 원점 지움, 지금 자리 홀드(옛 목표로 안 감)
    fake, arm = rig(home=False)
    arm.call_trigger("/arm_control/home_elbow")
    time.sleep(0.2)
    arm.joint_command(["elbow"], [rad(20)])
    time.sleep(1.5)
    fake.power(4, False)
    time.sleep(1.0)
    fake.power(4, True, enc_jump=30.0)
    ph_on = fake.m[4].phys
    time.sleep(2.0)
    m4 = arm.ak["ak45_4"]
    r = arm.joint_command(["elbow"], [rad(20)])
    check("짧은 전원 재투입(영점 30° 바뀜) → 원점 지움·지금 자리", not arm.homing.m["ak45_4"].homed and abs(fake.m[4].phys - ph_on) < 5
          and m4.target is not None and abs(m4.target - m4.pos) < 2 and not r["ok"],
          f"켜질 때 축 {ph_on:.1f}° → 2 s 뒤 {fake.m[4].phys:.1f}° (옛 목표면 축 50°), 원점 지움, 명령 거절: {r.get('error')}")

    # R5. 긴 끊김 경계(제어 루프가 cmd 를 지우기 전 5.05 s 째 피드백) → 옛 목표 말고 지금 자리
    arm.call_trigger("/arm_control/home_elbow")
    time.sleep(0.2)
    with arm.lock:
        now = time.monotonic()
        m4.fb_t = arm.homing.m["ak45_4"].seen = now - 5.05
        m4.target, m4.cmd = m4.pos + 20.0, m4.pos + 20.0
        st = types.SimpleNamespace(controller_id=4, position_deg=m4.pos - 10.0, speed_erpm=0, current_a=0.0,
                                   temperature_c=30, error=0)
        arm._on_feedback(st, now)
        tg, ps = m4.target, m4.pos
    check("긴 끊김 경계 순간 → 지금 자리 홀드", abs(tg - ps) < 0.01 and not arm.homing.m["ak45_4"].homed,
          f"5.05 s 끊김 뒤 피드백 {ps:+.1f}° → 목표 {tg:+.1f}° (옛 목표 +20 쪽 아님), 원점 지움")
    done(fake, arm)

    # R6. rx 처리 예외 → 스레드 살아 있음 + 화면 빨강, 고치면 피드백 계속
    fake, arm = rig()
    orig = arm._on_feedback

    def bad(st, now):
        raise ValueError("시험 예외")
    arm._on_feedback = bad
    time.sleep(0.3)
    snap = arm.snapshot()
    arm._on_feedback = orig
    ok = wait(lambda: arm.ak["ak45_4"].fresh(time.monotonic(), 0.25), 1.0)
    check("rx 처리 예외 → 스레드 삶·화면 빨강", arm.threads[0].is_alive() and ok and snap["bus"]["level"] == 2
          and "수신 처리 오류" in snap["bus"]["message"] and arm.ak["ak45_4"].cmd is not None,
          f"예외 {arm.busst['rx_errors']}개 동안 rx 살아 있음, 화면: {snap['bus']['message'][:60]}…")

    # R7. rx 스레드가 진짜 죽으면 → 제어 루프가 다시 띄움
    hook = threading.excepthook
    threading.excepthook = lambda args: None
    once = [True]

    def die(st, now):
        if once[0]:
            once[0] = False
            raise _Die()
        return orig(st, now)
    arm._on_feedback = die
    ok = wait(lambda: arm.busst["rx_restarts"] >= 1 and arm.threads[0].is_alive(), 2.0)
    arm._on_feedback = orig
    threading.excepthook = hook
    fresh = wait(lambda: arm.ak["ak45_4"].fresh(time.monotonic(), 0.25), 1.0)
    check("rx 스레드 죽음 → 다시 띄움", ok and fresh and abs(fake.pos(4)) < 1.0,
          f"다시 띄움 {arm.busst['rx_restarts']}번, 피드백 {'돌아옴' if fresh else '없음'}, 팔 {fake.pos(4):.1f}° (안 떨어짐)")

    # R8. recv 가 계속 실패(USB-CAN 빠짐 흉내) → 버스 다시 엶, 로그 폭주 없음
    n_log = len(arm.lines)
    bus0 = arm.bus

    def broken(timeout=None):
        time.sleep(0.01)
        raise OSError(100, "Network is down (시험)")
    bus0.recv = broken
    ok = wait(lambda: arm.busst["bus_reopens"] >= 1 and arm.bus is not bus0, 4.0)
    fresh = wait(lambda: arm.ak["ak45_4"].fresh(time.monotonic(), 0.25), 1.0)
    new_lines = sum(1 for x in list(arm.lines)[n_log - len(arm.lines):] if "수신 오류" in x) if len(arm.lines) > n_log else 0
    check("수신 실패 → 버스 다시 엶·로그 몇 줄", ok and fresh and new_lines <= 2 and abs(fake.pos(4)) < 1.0,
          f"다시 엶 {arm.busst['bus_reopens']}번, 수신 오류 {arm.busst['rx_errors']}개에 '수신 오류' 로그 {new_lines}줄(5 s 에 한 줄), 피드백 돌아옴, 팔 {fake.pos(4):.1f}°")
    done(fake, arm)

    # R9. 보정 move 를 막힌 관절에 여러 번 → 목표는 측정 ±10° 안
    fake, arm = rig(home=False)
    with fake.lock:
        fake.m[4].grab = True
    for _ in range(10):
        arm.calib("move", "elbow", 10)
    m4 = arm.ak["ak45_4"]
    check("보정 move 누적 안 됨", abs(m4.target - m4.pos) <= 10.0 + 1e-6,
          f"막힌 elbow 에 +10° 10번 → 목표 {m4.target:+.1f}° · 실제 {m4.pos:+.1f}° (전엔 +100°)")
    fake.reset()
    done(fake, arm)

    # R10. 브레이크 송신 실패(ENOBUFS 흉내) → 재시도로 다 나감 / 계속 실패면 화면 경고
    fake, arm = rig()
    bus0 = arm.bus
    real_send = bus0.send
    calls = [0]

    def flaky(msg, timeout=None):
        calls[0] += 1
        if calls[0] % 3:
            raise OSError(105, "No buffer space available (시험)")
        return real_send(msg, timeout)
    b0 = {i: fake.m[i].brakes for i in fake.m}
    bus0.send = flaky
    arm.limp()
    time.sleep(0.2)
    got = {i: fake.m[i].brakes - b0[i] for i in fake.m}

    def dead(msg, timeout=None):
        raise OSError(105, "No buffer space available (시험)")
    bus0.send = real_send
    arm.call_trigger("/arm_control/home_all")
    time.sleep(0.1)
    bus0.send = dead
    arm.limp(["ak45_4"])
    snap = arm.snapshot()
    bus0.send = real_send
    note = arm.ak["ak45_4"].note or ""
    check("브레이크 송신 실패 → 재시도·경고", all(v >= 3 for v in got.values()) and snap["bus"]["level"] == 2 and "못 보냄" in note,
          f"3번 중 2번 실패해도 모터마다 브레이크 {list(got.values())}개 · 계속 실패면: {note[:40]}… / 화면 {snap['bus']['message'][:40]}…")
    done(fake, arm)

    # R11. 작은 오차로 계속 누름(막힘 12° 아래, 큰 전류) → 목표를 지금 자리로(힘 유지), 끄면 안 함
    fake, arm = rig()
    with fake.lock:
        fake.m[4].grab = True
        fake.m[4].press_k = 0.5
    arm.joint_command(["elbow"], [rad(8)])
    ok = wait(lambda: "누르기 멈춤" in (arm.ak["ak45_4"].note or ""), 3.0)
    time.sleep(0.3)
    m4 = arm.ak["ak45_4"]
    cur = m4.cur
    check("누르기(전류 큼, 오차 8°) → 목표 풀어 줌·힘 유지", ok and m4.fault is None and m4.cmd is not None
          and abs(m4.target - m4.pos) < 0.5 and cur < 1.0 and arm.homing.m["ak45_4"].homed,
          f"{(m4.note or '')[:50]}… → 전류 {cur:.2f} A, 홀드 유지, 원점 유지")
    arm.p["press_amps"] = 0.0
    arm.joint_command(["elbow"], [rad(-8)])
    time.sleep(2.0)
    check("누르기 멈춤 끄기(press_amps 0)", "누르기" not in (m4.note or "") and abs(m4.target - m4.pos) > 6,
          f"끄면 2 s 뒤에도 목표 {m4.target:+.1f}° · 실제 {m4.pos:+.1f}° 그대로 (막힘 12° 아래)")
    fake.reset()
    done(fake, arm)

    # R12. 온도 히스테리시스 + 온도로 힘 뺀 관절은 경고 아래에서만 원점
    fake, arm = rig()
    m3 = arm.ak["ak45_3"]
    seq = []
    for t in (62, 57, 54):
        fake.m[3].temp = t
        time.sleep(0.15)
        seq.append(m3.hot)
    fake.m[3].temp = 76
    wait(lambda: m3.fault is not None, 1.0)
    fake.m[3].temp = 65
    time.sleep(0.15)
    r65 = arm.call_trigger("/arm_control/home_upper_arm_roll")
    fake.m[3].temp = 58
    time.sleep(0.15)
    r58 = arm.call_trigger("/arm_control/home_upper_arm_roll")
    check("온도 경고 풀림 −5 ℃ · 원점은 경고 아래", seq == [True, True, False] and not r65["ok"] and r58["ok"],
          f"62/57/54 ℃ 경고 {seq}, 76 ℃ 힘 뺀 뒤 65 ℃ 원점 {'거절' if not r65['ok'] else '허용'}, 58 ℃ {'허용' if r58['ok'] else '거절'}")
    fake.reset()
    done(fake, arm)

    # R13. 재생 중 close() → 브레이크 뒤 위치 프레임 없음
    fake, arm = rig()
    arm.play_trajectory(dict(traj, positions=pos), 30.0, hold=0.2)
    time.sleep(0.8)
    arm.close()
    time.sleep(0.3)
    with fake.lock:
        t_brake = min((f[0] for f in fake.frames if f[2] and (f[1] >> 8) == 2), default=None)
        after = sum(1 for f in fake.frames if t_brake is not None and f[0] > t_brake and f[2] and (f[1] >> 8) == 4)
    check("재생 중 닫기 → 브레이크 뒤 위치 프레임 0", t_brake is not None and after == 0, f"브레이크 뒤 위치 프레임 {after}개")
    fake.close()


if __name__ == "__main__":
    raise SystemExit(main())
