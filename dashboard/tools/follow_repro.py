"""손 위치 카드 '팔도 바로 따라가기' 끊김 재현 (headless Playwright, 창 안 뜸).

    python dashboard/tools/follow_repro.py --port 8098 [--fix 1|0] [--yaw 20] [--tag a] [--no-send]

가상 모드 서버에서: 역기구학 탭 → yaw/고정 설정 → 시작점 전체 풀이 → [보내기](--no-send 면 생략) → 따라가기 켬 →
사람 드래그처럼 슬라이더 input 이벤트(값이 한 칸 바뀔 때만, 약 0.15 초 간격)와 지도 드래그(hkSetXY live, 30 Hz) 를 흉내 낸다.
로그는 서버가 dashboard/logs/follow.log 에 쓴다. 각 단계 끝에 'mark' 줄을 남겨 구간을 나눈다.
"""
import argparse
import sys
import time

from playwright.sync_api import sync_playwright

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8098)
ap.add_argument("--fix", type=int, default=1)
ap.add_argument("--yaw", type=float, default=20.0)
ap.add_argument("--tag", default="run")
ap.add_argument("--no-send", action="store_true")
ap.add_argument("--mode", default=None, help="손 자세 (없으면 화면 기본)")
ap.add_argument("--resync", type=int, default=1, help="구간마다 막혀 있으면 [보내기] 로 다시 맞춤")
ap.add_argument("--dt", type=float, default=0.15, help="슬라이더 한 칸 사이 시간 (s)")
a = ap.parse_args()
sys.stdout.reconfigure(encoding="utf-8")


USER_STATE = ("() => ({ ok: HK.last && HK.last.ok, reason: HK.last && HK.last.reason, sol_vs_sent: hkDiff(HK.last && HK.last.joints, HK.followSent), "
              "arm_vs_sent: hkDiff(hkArm(), HK.followSent), note: HK.followNote, res: HK.res.textContent.split(String.fromCharCode(10))[0] })")


def resync(page):
    """구간 시작 전: 따라가기가 막혀 있으면(직전 구간에서 못 닿는 곳을 지나 20° 넘게 벌어짐) [보내기] 로 다시 맞춘다 — 구간마다 따로 평가."""
    settle(page)
    if page.evaluate("() => !!(HK.last && HK.last.ok && HK.followSent && hkDiff(HK.last.joints, HK.followSent).d > 20)"):
        page.evaluate("() => hkSend()")
        mark(page, "resync [보내기]")
        time.sleep(4)


def mark(page, text):
    page.evaluate("(t) => { HK.flogQ.push({ t: Math.round(performance.now()), ev: 'mark', why: t }); return hkFlogFlush(); }", text)


def settle(page, timeout=20.0):
    """전체 풀이가 끝나고(busy·inflight 없음) 입력과 풀이가 같을 때까지."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if page.evaluate("() => !HK.busy && !HK.inflight && !!HK.last && hkSame(hkVals(), HK.last.req)"):
            return True
        time.sleep(0.1)
    return False


def slide(page, key, a0, a1, step, digits):
    """사람 드래그: range 값을 한 칸씩 바꾸며 input, 끝에 change."""
    n = max(1, round(abs(a1 - a0) / step))
    for i in range(n + 1):
        v = a0 + (a1 - a0) * i / n
        page.evaluate("([k, v, d]) => { const s = HK.f[k]; s.range.value = v; s.range.dispatchEvent(new Event('input')); }", [key, v, digits])
        time.sleep(a.dt)
    page.evaluate("(k) => HK.f[k].range.dispatchEvent(new Event('change'))", key)


def mapdrag(page, p0, p1, secs):
    n = max(2, int(secs * 30))
    for i in range(n + 1):
        x = p0[0] + (p1[0] - p0[0]) * i / n
        y = p0[1] + (p1[1] - p0[1]) * i / n
        page.evaluate("([x, y]) => hkSetXY(x, y, true)", [x, y])
        time.sleep(1 / 30)
    page.evaluate("() => hkEdit(false)")


with sync_playwright() as pw:
    br = pw.chromium.launch(headless=True, channel="msedge")   # 설치된 Edge, headless (창 안 뜸)
    page = br.new_page(viewport={"width": 1400, "height": 1000})
    page.on("dialog", lambda d: d.accept())
    page.goto(f"http://127.0.0.1:{a.port}/#ik")
    page.wait_for_function("() => typeof HK !== 'undefined' && !!HK.f", timeout=30000)
    page.evaluate("([fx, yaw, md]) => { HK.f.fix.checked = !!fx; HK.f.yaw.set(yaw); if (md) HK.f.mode.value = md; HK.f.x.set(-0.10); HK.f.y.set(-0.30); HK.f.z.set(Z0); hkEdit(false); }".replace("Z0", str(0)), [a.fix, a.yaw, a.mode])
    z0 = page.evaluate("() => HK.f.ref && HK.f.ref.value === 'wrist' ? HK_WRIST_Z0 : 2.5")   # 기준점 높이 (손목 27 / 집는 점 2.5)
    page.evaluate("(z) => { HK.f.z.set(z); hkEdit(false); }", z0)
    time.sleep(0.5)
    print("start settle", settle(page), page.evaluate("() => HK.last && [HK.last.ok, HK.last.reason]"))
    page.wait_for_function("() => S && S.gate && Object.values(S.gate.joints || {}).length && Object.values(S.gate.joints).every((j) => j.ok)", timeout=30000)
    if not a.no_send:
        for _ in range(5):
            page.evaluate("() => hkSend()")
            time.sleep(0.8)
            if page.evaluate("() => !!HK.followSent"):
                break
        print("manual send", page.evaluate("() => HK.followSent"), flush=True)
        time.sleep(6)   # 가상 팔 램프(30°/s)로 도착
    page.evaluate("() => { HK.f.followBox.checked = true; HK.f.followBox.dispatchEvent(new Event('change')); }")
    mark(page, f"start {a.tag} fix={a.fix} yaw={a.yaw} send={not a.no_send}")
    time.sleep(3)
    for desc, fn in [
        ("x -0.10→0.15 (y -0.30, z 2.5)", lambda: slide(page, "x", -0.10, 0.15, 0.005, 3)),
        ("y -0.30→-0.20", lambda: slide(page, "y", -0.30, -0.20, 0.005, 3)),
        ("y -0.20→-0.35", lambda: slide(page, "y", -0.20, -0.35, 0.005, 3)),
        ("x 0.15→-0.10", lambda: slide(page, "x", 0.15, -0.10, 0.005, 3)),
        ("z +0→+12.5 cm", lambda: slide(page, "z", z0, z0 + 12.5, 0.5, 1)),
        ("z +12.5→+0 cm", lambda: slide(page, "z", z0 + 12.5, z0, 0.5, 1)),
        ("map (-0.10,-0.35)→(0.04,-0.25) 사용자 점", lambda: mapdrag(page, (-0.10, -0.35), (0.04, -0.25), 6)),
        ("map (0.04,-0.25)→(0.15,-0.20)", lambda: mapdrag(page, (0.04, -0.25), (0.15, -0.20), 5)),
        ("map (0.15,-0.20)→(-0.10,-0.30)", lambda: mapdrag(page, (0.15, -0.20), (-0.10, -0.30), 8)),
        ("map (-0.10,-0.30)→(0.04,-0.25)", lambda: mapdrag(page, (-0.10, -0.30), (0.04, -0.25), 5)),
    ]:
        if a.resync:
            resync(page)
        mark(page, "seg " + desc)
        fn()
        time.sleep(2.5)
        st = page.evaluate("() => ({ n: HK.followN, note: HK.followNote })")
        print(desc, st, flush=True)
    time.sleep(4)
    print("사용자 점(0.04,-0.25) 끝 상태:", page.evaluate(USER_STATE), flush=True)
    mark(page, f"end {a.tag}")
    page.evaluate("() => { HK.f.followBox.checked = false; HK.f.followBox.dispatchEvent(new Event('change')); }")
    time.sleep(1.2)
    br.close()
