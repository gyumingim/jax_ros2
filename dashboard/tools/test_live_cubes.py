#!/usr/bin/env python3
"""live_api 시험 (폰·GPU 없이): 실시간 인식 스레드 + 가상 장면.

    /usr/bin/python3 dashboard/tools/test_live_cubes.py

1. 가짜 결과 폴더에 그린 프레임을 inference.png (뎁스 색 칸까지 붙인 모양) 로 → 스레드가 큐브 여럿 (ID·층) 을 찾는다, 휴대폰 탭 pick 도.
2. live.jpg 를 쓰기 시작하면 그것으로 바뀐다 (5 Hz).
3. 프레임이 끊기면 1.5초 뒤 '영상 없음' (stale, pick 막음), 다시 오면 풀림.
4. 가상 장면: 놓기·옮기기(위에 쌓기)·지우기 규칙, 재생이 끝까지 가면 옮김, 어긋남 ±1 cm·±5°, 파일에 남음.
5. stack_api.next_step 의 놓인 큐브 확인 (어긋남·층).
실패하면 종료코드 1.
"""
import math
import os
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import live_api as L             # noqa: E402
import phone_detect as PD        # noqa: E402
import stack_api as SA           # noqa: E402
import test_phone_detect as T    # noqa: E402

BAD = []


def check(name, ok, detail=""):
    print(("✓ " if ok else "✗ ") + name + (f" — {detail}" if detail else ""))
    if not ok:
        BAD.append(name)


def wait(cond, s):
    t0 = time.time()
    while time.time() - t0 < s:
        if cond():
            return True
        time.sleep(0.05)
    return False


def camera(tmp):
    run = tmp / "run"
    run.mkdir()
    (tmp / "phone_depth.json").write_text('{"run_dir": "%s"}' % run)
    L.LIVE = L.Live(tmp, lambda: "direct")
    cubes = [(10, 0.02, -0.27, 10.0, 0), (11, -0.14, -0.30, 5.0, 0), (12, -0.14, -0.30, 5.0, 1)]
    img = T.render(dict(PD.DEFAULT_CFG), None, (-0.10, -0.80, 0.50), 1250, cubes)
    panel = np.hstack([img, np.zeros((img.shape[0], img.shape[1] + 70, 3), np.uint8)])   # inference.png: [객체 그림 | 뎁스 색 + 눈금 70 px]
    L.LIVE.start()

    def put(name, im):
        tmpf = run / ("tmp_" + name)
        cv2.imwrite(str(tmpf), im)
        os.replace(tmpf, run / name)

    put("inference.png", panel)
    ok = wait(lambda: L.LIVE.res is not None and L.LIVE.res.get("src") == "inference.png", 8)
    r = L.cubes(0.4775, None)
    got = {c["id"]: c for c in r["cubes"]}
    good = ok and not r["stale"] and set(got) == {10, 12} and got[12]["level"] == 1 and got[10]["level"] == 0
    check("inference.png (오른쪽 뎁스 칸 잘라 냄) → 큐브 10 (1층) · 12 (2층, 11 은 가려짐)", good,
          ", ".join(f"{i}: ({c['x']:.3f}, {c['y']:.3f}) {c['level'] + 1}층 기울기 {c.get('tilt_deg')}°" for i, c in got.items()) or r.get("reason"))
    if good:
        check("자리 1 cm 안", math.hypot(got[10]["x"] - 0.02, got[10]["y"] + 0.27) < 0.01 and math.hypot(got[12]["x"] + 0.14, got[12]["y"] + 0.30) < 0.01)
    p = r["pick"]
    check("휴대폰 탭 pick = ArUco 10", bool(p.get("ok")) and p.get("used") == "aruco" and abs(p["cube"]["x"] - 0.02) < 0.01, p.get("text") or p.get("reason"))
    p12 = L.cubes(None, 12)["pick"]
    check("pick=12 (2층) → 막음 '쌓인 큐브'", bool(p12.get("ok")) and p12.get("blocked"), p12.get("block"))

    t0, n0 = time.time(), L.LIVE.n
    while time.time() - t0 < 2.0:
        put("live.jpg", img)
        time.sleep(0.2)
    time.sleep(0.3)
    r = L.cubes(None, None)
    check("live.jpg 가 있으면 그것 (5 Hz 로 씀 → 프레임마다 인식)", r["src"] == "live.jpg" and L.LIVE.n - n0 >= 7,
          f"src {r['src']}, 프레임 {L.LIVE.n - n0}개, {r['rate_hz']} Hz, 한계 {r['stale_s']} s")
    time.sleep(L.STALE_S + 0.3)
    r = L.cubes(None, None)
    check("프레임이 1.5초 넘게 없으면 영상 없음 (stale, 목록 비움, pick 막음)", r["stale"] and not r["cubes"] and not r["pick"]["ok"], r["reason"])
    put("live.jpg", img)
    check("새 프레임이 오면 풀림", wait(lambda: not L.cubes(None, None)["stale"], 3))


def scene(tmp):
    S = L.SCENE = L.Scene(tmp / "virtual_scene.json")
    a = S.op({"op": "add", "x": 0.02, "y": -0.27})
    b = S.op({"op": "add", "x": 0.10, "y": -0.21, "yaw_deg": 50})
    c = S.op({"op": "add", "x": 0.02, "y": -0.27})
    cs = S.snapshot()["cubes"]
    check("놓기: 다음 빈 번호 10, 11 · 같은 자리는 거절 · 각은 −45~45", a["ok"] and b["ok"] and not c["ok"] and [k["id"] for k in cs] == [10, 11] and cs[1]["yaw_deg"] == -40.0)
    m = S.op({"op": "move", "id": 11, "x": 0.025, "y": -0.268})
    k = {q["id"]: q for q in S.snapshot()["cubes"]}
    check("다른 큐브 위로 끌면 그 위 (자리·각 맞춤, 2층)", m["ok"] and k[11]["level"] == 1 and k[11]["x"] == 0.02 and k[11]["yaw_deg"] == k[10]["yaw_deg"])
    check("밑의 큐브는 못 옮기고 못 지움", not S.op({"op": "move", "id": 10, "x": 0, "y": -0.3})["ok"] and not S.op({"op": "delete", "id": 10})["ok"])
    S.op({"op": "move", "id": 11, "x": 0.10, "y": -0.21})
    end = S.on_play({"real": {"cube": [0.10, -0.21, 0.0], "target": [-0.16, -0.28], "target_yaw": 30.0}})
    end("stopped")
    check("재생이 중간에 멈추면 안 옮김", next(q for q in S.snapshot()["cubes"] if q["id"] == 11)["x"] == 0.10)
    end("끝까지 재생")
    q = next(q for q in S.snapshot()["cubes"] if q["id"] == 11)
    check("끝까지 재생 → 놓을 자리로 (각 = target_yaw)", (q["x"], q["y"], q["yaw_deg"], q["level"]) == (-0.16, -0.28, 30.0, 0), S.snapshot()["last_move"]["text"])
    S.op({"op": "jitter", "on": True})
    S.on_play({"real": {"cube": [0.02, -0.27, 10.0], "target": [-0.16, -0.28], "target_yaw": 30.0}})("끝까지 재생")
    q = next(q for q in S.snapshot()["cubes"] if q["id"] == 10)
    dx, dy, dyaw = q["x"] + 0.16, q["y"] + 0.28, q["yaw_deg"] - 30.0
    check("어긋남 켬 → 큐브 11 위 (2층), ±1 cm · ±5°", q["level"] == 1 and abs(dx) <= 0.0101 and abs(dy) <= 0.0101 and abs(dyaw) <= 5.01,
          S.snapshot()["last_move"]["text"])
    S2 = L.Scene(tmp / "virtual_scene.json")
    check("파일에 남음 (서버를 다시 켜도)", S2.snapshot()["cubes"] == S.snapshot()["cubes"] and S2.jitter)
    # 탑 쌓기: 놓인 큐브 확인
    cubes = [{"id": 11, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 0}, {"id": 10, "x": -0.152, "y": -0.284, "yaw_deg": 33.0, "level": 1},
             {"id": 12, "x": 0.10, "y": -0.21, "yaw_deg": 0.0, "level": 0}]
    base, order = {"x": -0.16, "y": -0.28, "yaw_deg": 30.0}, [11, 10, 12]
    r = SA.next_step(cubes, base, order, {"id": 10, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 1})
    check("next_step: 9 mm 어긋난 큐브 10 → 통과, 다음은 실제 큐브 10 위 (각 33°)", r["ok"] and r["level"] == 2 and r["target"]["x"] == -0.152 and r["target"]["yaw_deg"] == 33.0, r["text"])
    bad = dict(cubes[1], x=-0.13)
    r = SA.next_step([cubes[0], bad, cubes[2]], base, order, {"id": 10, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 1})
    check("next_step: 3 cm 어긋남 → 멈춤", not r["ok"] and r.get("stop"), r["text"])
    r = SA.next_step([cubes[0], dict(cubes[1], tilt_deg=25.0), cubes[2]], base, order, {"id": 10, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 1})
    check("next_step: 25° 기울어짐 → 멈춤", not r["ok"] and r.get("stop"), r["text"])
    r = SA.next_step([cubes[0], dict(cubes[1], level=0, x=0.0, y=-0.40), cubes[2]], base, order, {"id": 10, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 1})
    check("next_step: 1층으로 떨어짐 → 멈춤", not r["ok"] and r.get("stop"), r["text"])


def main():
    tmp = Path(tempfile.mkdtemp(prefix="live_test"))
    camera(tmp)
    scene(tmp)
    print("통과" if not BAD else f"실패 {len(BAD)}: {BAD}")
    sys.exit(1 if BAD else 0)


if __name__ == "__main__":
    main()
