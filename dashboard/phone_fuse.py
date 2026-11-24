"""휴대폰 카메라 탭: 큐브를 찾는 두 방법 (프레임의 ArUco = phone_detect.detect, 폰 뎁스 = phone_depth.evaluate) 의 결과를 하나로.

화면에는 모드도 사진 올리기도 없다 — 거치한 폰의 프레임마다 있는 것을 다 쓴다 (phone_api 의 GET /api/phone/live 가 find 를 부른다).

    둘 다 큐브를 찾음  → 자리·각은 ArUco (더 정확하고 각이 정확). 뎁스로 맞춰 본다: 자리 2 cm 또는 각 15° 넘게 다르면
                         blocked (화면이 '집으러 가기' 를 막는다 — 다음 프레임에서 맞으면 풀린다).
    하나만 찾음        → 그것을 쓰고 어느 것인지 말한다. 못 찾은 쪽의 이유는 note 에.
    둘 다 없음         → 무엇을 해야 하는지 (reason).
    뎁스 높이 검사     → 뎁스가 큐브를 책상 위 0~5 cm 밖으로 재면 (phone_depth 의 usable=False) ArUco 가 있어도 막는다
                         (거치 자세나 책상 높이 d 가 틀렸다는 뜻이고, d 가 틀리면 ArUco 자리로 가도 손이 책상을 누른다).

들어가는 것은 '프레임 하나 + 그때의 뎁스 결과' 다: find(image, cam_cfg, depth) 의 image 는 영상 프레임 (BGR numpy 배열; JPEG·PNG 바이트도 됨,
없으면 None). 프레임을 계속 읽는 루프가 프레임마다 부른다. 파일·HTTP 는 여기서 안 건드린다 (프레임 읽기 = phone_depth.read_frame).

각 비교는 90° 마다 같은 것으로 본다 (뎁스 각은 −45~45° 로만 나온다). 뎁스 각이 믿을 만할 때만 (옆면·윗면으로 추정) 비교한다 —
못 구했거나, 두 면이 섞였거나, 직접 넣은 값이면 각은 안 본다.
"""
from math import hypot

POS_TOL_M = 0.02
YAW_TOL_DEG = 15.0
NAME = {"aruco": "ArUco", "depth": "뎁스"}
NOTHING = "폰 프로그램이 꺼져 있음 (새 결과 없음) — 폰 프로그램을 켜고, 거치한 폰이 큐브를 보게 하기"


def find(image, cam_cfg=None, depth=None):
    """image (BGR 배열 | JPEG·PNG 바이트 | None) 에서 ArUco 를 찾고 depth (phone_depth.evaluate 결과 | None) 와 합친다.
    → fuse 의 dict + aruco (phone_detect.detect 결과 그대로, 'annotated' = 마커를 그린 BGR 그림 포함; 그림이 없었으면 None) + depth."""
    aruco = None
    if image is not None and len(image):
        import phone_detect as PD
        aruco = PD.detect(image, cam_cfg)
    out = fuse(aruco, depth)
    out.update(aruco=aruco, depth=depth)
    return out


def yaw_diff(a, b):
    """큐브 각 차이 (°, 0~45): 90° 마다 같다."""
    d = abs(a - b) % 90.0
    return min(d, 90.0 - d)


def fuse(aruco, depth):
    """aruco = phone_detect.detect 결과 (프레임이 없었으면 None), depth = phone_depth.evaluate 결과 (None 가능).
    → {ok, cube{x, y, yaw_deg}, used 'aruco'|'depth', sources [...], text '출처: …', blocked, block (버튼에 쓸 짧은 말),
       check{dpos_mm, dyaw_deg}, warn [...], note [...], reason}"""
    a = aruco if aruco and aruco.get("ok") else None
    p = depth if depth and depth.get("ok") else None
    out = {"ok": bool(a or p), "sources": [k for k, v in (("aruco", a), ("depth", p)) if v], "used": None, "blocked": False, "block": "",
           "warn": [], "note": [], "check": None}
    # 못 쓴 쪽이 왜 못 썼는지 (뎁스가 꺼져 있거나 오래된 것은 말할 것 없음 — 둘 다 없을 때만)
    if aruco is not None and not a:
        out["note"].append("ArUco: " + (aruco.get("reason") or "못 찾음").split(" — ")[0])
    depth_off = depth is None or depth.get("stale") or depth.get("age_s") is None
    if depth is not None and not p and not depth_off:
        out["note"].append("뎁스: " + (depth.get("reason") or "못 찾음"))
    if not out["ok"]:
        if aruco is None and depth_off:
            out["reason"] = NOTHING
        elif aruco is None:
            out["reason"] = "큐브를 못 찾음 — 큐브 윗면 마커가 화면에 나오게 하기"
        else:
            out["reason"] = aruco.get("reason") or "못 찾음"
            if depth_off:
                out["note"] = ["뎁스 결과는 없음"]
            else:
                out["note"] = [n for n in out["note"] if not n.startswith("ArUco")]
        return out
    used = "aruco" if a else "depth"
    main = a or p
    out.update(used=used, cube=dict(main["cube"]))
    for src, r in (("aruco", a), ("depth", p)):
        out["warn"] += [w if src == used or not a else "뎁스: " + w for w in (r or {}).get("warn", [])]
    if a and p:
        dpos = hypot(a["cube"]["x"] - p["cube"]["x"], a["cube"]["y"] - p["cube"]["y"])
        cmp_yaw = p.get("yaw_from") in ("side", "top")
        dyaw = yaw_diff(a["cube"]["yaw_deg"], p["cube"]["yaw_deg"]) if cmp_yaw else None
        out["check"] = {"dpos_mm": round(dpos * 1000, 1), "dyaw_deg": None if dyaw is None else round(dyaw, 1)}
        bad = []
        if dpos > POS_TOL_M:
            bad.append(f"자리 {dpos * 100:.1f} cm")
        if dyaw is not None and dyaw > YAW_TOL_DEG:
            bad.append(f"각 {dyaw:.0f}°")
        if bad:
            out.update(blocked=True, block="ArUco·뎁스가 안 맞음")
            out["warn"].insert(0, f"ArUco 와 뎁스가 {' · '.join(bad)} 다름 (허용 {POS_TOL_M * 100:.0f} cm · {YAW_TOL_DEG:.0f}°) — 큐브가 움직였거나 "
                                  f"기준 종이 자리·책상 높이·폰 거치 자세가 틀림 (뎁스 x {p['cube']['x']:.3f} y {p['cube']['y']:.3f} yaw {p['cube']['yaw_deg']:.0f}°)")
            out["text"] = "출처: ArUco + 뎁스 — 서로 안 맞음"
        else:
            out["text"] = (f"출처: ArUco + 뎁스 (자리·각은 ArUco, 뎁스와 {dpos * 1000:.0f} mm" +
                           (f" · {dyaw:.0f}°" if dyaw is not None else " · 각은 비교 안 함") + " 차이로 맞음)")
    else:
        out["text"] = f"출처: {NAME[used]} 만"
    if p and p.get("usable") is False and not out["blocked"]:
        out.update(blocked=True, block="뎁스 높이가 안 맞아 못 감")
    return out
