#!/usr/bin/env python3
"""폰 거치 자세 (camera_to_robot 숫자 6개) 를 큐브를 아는 자리에 놓고 본 점들로 푼다 — 휴대폰 카메라 탭 '세그먼트(폰 뎁스)' 모드용.

    python3 dashboard/tools/solve_phone_mount.py --d 0.4775 \\
        --point "0.0715,0.0906,0.4457:-0.05,-0.33" --point "…:…" --point "…:…" [--save]

--point "cx,cy,cz:rx,ry": cx,cy,cz = 화면 결과 줄의 '카메라 좌표' (inference.json 의 center_cam1_m, m),
rx,ry = 그때 큐브 중심의 로봇 좌표 (m; 원점 = 어깨 roll 축 바로 아래, +x = 로봇 왼쪽, 앞 = −y). 3곳 이상, 일렬이 아니게 (넓게 벌릴수록 좋다).
--d = 책상 높이 (어깨 축 아래 m, 역기구학 탭의 값). 점들의 로봇 z = −d + 0.025 (큐브 중심; --z 로 직접 줄 수도 있다).
--save 를 주면 dashboard/logs/phone_depth.json 의 camera_to_robot 에 쓴다 (서버 재시작 필요 없음, 다른 폴더면 --cfg).

팀 vision_node/tools/solve_camera_to_robot.py 와 같은 계산 (phone_depth.solve_mount). 주의: 점이 모두 한 높이라 잔차가 0 이어도
부호를 잘못 넣은 것(거울)은 못 잡는다 — 풀고 나서 큐브를 새 자리에 놓고 화면의 x, y 가 맞는지 꼭 확인.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import phone_depth as DP   # noqa: E402


def main():
    argv, i = [], 1
    while i < len(sys.argv):          # '--point -0.05,…' 처럼 값이 '-' 로 시작해도 되게
        if sys.argv[i] == "--point" and i + 1 < len(sys.argv):
            argv.append("--point=" + sys.argv[i + 1]); i += 2
        else:
            argv.append(sys.argv[i]); i += 1
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--point", action="append", default=[], metavar="cx,cy,cz:rx,ry")
    ap.add_argument("--d", type=float, default=0.4775, help="책상 높이 (어깨 축 아래 m)")
    ap.add_argument("--z", type=float, help="점들의 로봇 z (기본 −d + 0.025)")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--cfg", default=str(HERE.parent / "logs" / "phone_depth.json"))
    a = ap.parse_args(argv)
    try:
        pts = []
        for t in a.point:
            cam, rob = t.split(":")
            c, r = [float(v) for v in cam.split(",")], [float(v) for v in rob.split(",")]
            if len(c) != 3 or len(r) != 2:
                raise ValueError(f"--point {t!r}: 'cx,cy,cz:rx,ry'")
            pts.append((tuple(c), tuple(r)))
        z = a.z if a.z is not None else -a.d + 0.025
        params, res = DP.solve_mount(pts, z)
    except ValueError as e:
        print(f"오류: {e}", file=sys.stderr)
        return 2
    print("camera_to_robot: [" + ", ".join(f"{v:.6f}" for v in params) + "]")
    for k, m in enumerate(res, 1):
        print(f"  점 {k}: 잔차 {m:.1f} mm")
    mean = sum(res) / len(res)
    print(f"평균 잔차 {mean:.1f} mm, 최대 {max(res):.1f} mm ({len(res)}점, 로봇 z {z:g} m)")
    if mean > 20:
        print("경고: 평균 잔차가 20 mm 를 넘는다 — 적은 숫자·부호를 확인")
    if a.save:
        p = Path(a.cfg)
        try:
            d = json.loads(p.read_text())
        except (OSError, ValueError):
            d = {}
        d["camera_to_robot"] = params
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(d, ensure_ascii=False, indent=1))
        print(f"저장: {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
