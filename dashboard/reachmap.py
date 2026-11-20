"""역기구학 탭 2D 지도: team-jax/IK 의 도달 맵(reachmap_real*.npz)과 판정(realcoords.plan)을 실좌표로 준다.

realcoords.py 는 numpy 만 쓴다(패키지 __init__ 은 MuJoCo 를 부르므로 파일을 직접 읽는다).
실좌표: 원점 = 어깨 roll 축, x = 로봇 왼쪽 +, y = 뒤 + (앞 = −y), m. 칸 중심 = realcoords 의 sim 그리드를 되돌린 것
(x_real = 0.052 − y_sim, y_real = x_sim − 0.34 — 상수는 realcoords.ORIGIN_XY 에서 읽는다).
"""
import importlib.util
import math
from pathlib import Path

SUB = Path("motion1") / "src" / "humanoid_grasp_rl"
_cache = {"key": None, "rc": None, "base": None, "yaw": None}


def _load(ik_dir):
    """(realcoords 모듈, 기본 맵 dict, yaw 맵 dict). 파일이 바뀌면 다시 읽는다. 없으면 FileNotFoundError."""
    import numpy as np
    if ik_dir is None:
        raise FileNotFoundError("team-jax/IK 를 못 찾음 (JAX_IK_DIR)")
    d = Path(ik_dir) / SUB
    files = [d / "realcoords.py", d / "reachmap_real.npz", d / "reachmap_real_yaw.npz"]
    for f in files:
        if not f.is_file():
            raise FileNotFoundError(f"없음: {f}")
    key = tuple((str(f), f.stat().st_mtime) for f in files)
    if _cache["key"] != key:
        spec = importlib.util.spec_from_file_location("_jax_realcoords", files[0])
        rc = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rc)
        with np.load(files[1]) as z:
            base = {k: z[k] for k in ("xs", "ys", "grasp", "place", "feasible")}
        with np.load(files[2]) as z:
            yaw = {k: z[k] for k in ("yaws", "offsets", "xs", "ys", "grasp", "place", "feasible")}
        _cache.update(key=key, rc=rc, base=base, yaw=yaw)
    return _cache["rc"], _cache["base"], _cache["yaw"]


def _real_xy(rc, xs, ys):
    """sim 그리드 (xs[i], ys[j]) -> 실좌표 칸 중심: to_sim 의 역(−90° 회전 + 원점)."""
    ox, oy = rc.ORIGIN_XY
    c, s = math.cos(math.radians(rc.SIM_FROM_REAL_YAW)), math.sin(math.radians(rc.SIM_FROM_REAL_YAW))
    out = []
    for i, xv in enumerate(xs):
        for j, yv in enumerate(ys):
            dx, dy = float(xv) - ox, float(yv) - oy
            out.append((i, j, c * dx + s * dy, -s * dx + c * dy))   # R(-yaw) · (sim - origin)
    return out


def _yaw_masks(rc, ym, yaw_real):
    """실 yaw 하나에 대해 칸마다 (grasp, place, feasible, 되는 손 돌림 목록) — reachable_turned 와 같은 yaw·손 돌림 고르기."""
    yaw_sim = rc.fold_yaw(yaw_real + rc.SIM_FROM_REAL_YAW)
    yaws, offsets = ym["yaws"], ym["offsets"]
    k = min(range(len(yaws)), key=lambda q: abs(rc.fold_yaw(float(yaws[q]) - yaw_sim)))
    rep = float(yaws[k]) + rc.fold_yaw(yaw_sim - float(yaws[k]))
    picks = []
    for turn in rc.hand_turns(yaw_sim):
        o = min(range(len(offsets)), key=lambda q: abs(float(offsets[q]) - (turn - rep)))
        if abs(float(offsets[o]) - (turn - rep)) <= 1.0:
            picks.append((turn, o))
    return yaw_sim, float(yaws[k]), k, picks


def reachmap(ik_dir, yaw_real=0.0):
    """GET /api/reachmap?yaw= : 칸 목록(실좌표) + 그 큐브 yaw 에서의 grasp/place/feasible."""
    try:
        rc, base, ym = _load(ik_dir)
    except FileNotFoundError as e:
        return {"ok": False, "error": f"도달 맵 없음 — {e}"}
    except Exception as e:
        return {"ok": False, "error": f"도달 맵을 못 읽음: {type(e).__name__}: {e}"}
    yaw_sim, map_yaw, k, picks = _yaw_masks(rc, ym, float(yaw_real))
    cells = []
    for i, j, x, y in _real_xy(rc, ym["xs"], ym["ys"]):
        g = any(bool(ym["grasp"][k, o, i, j]) for _, o in picks)
        p = any(bool(ym["place"][k, o, i, j]) for _, o in picks)
        turns = [t for t, o in picks if ym["feasible"][k, o, i, j]]
        cells.append({"x": round(x, 4), "y": round(y, 4), "grasp": g, "place": p, "feasible": bool(turns),
                      "turns": turns, "base": bool(base["feasible"][i, j])})
    xs, ys = ym["xs"], ym["ys"]
    return {"ok": True, "yaw_real": float(yaw_real), "yaw_sim": yaw_sim, "map_yaw_sim": map_yaw,
            "hand_turns": [t for t, _ in picks],
            "cell": {"dx": abs(float(ys[1] - ys[0])), "dy": abs(float(xs[1] - xs[0]))},   # 실 x 폭 = sim y 간격
            "d": rc.MAP_D, "cube_size": rc.CUBE_SIZE, "cells": cells,
            "files": [k_[0] for k_ in _cache["key"]],
            "note": "feasible = 집기·놓기 다 되고 상하좌우 칸도 됨(realcoords.plan 이 보는 값). 칸 판정은 가장 가까운 칸, 사이 보간 없음."}


def check(ik_dir, body):
    """POST /api/reachmap/check {cube:[x,y,yaw], target:[x,y], d?} -> realcoords.plan() 결과 + 큐브·놓을 자리 따로 판정."""
    try:
        rc, _, _ = _load(ik_dir)
    except FileNotFoundError as e:
        return {"ok": False, "error": f"도달 맵 없음 — {e}"}
    except Exception as e:
        return {"ok": False, "error": f"도달 맵을 못 읽음: {type(e).__name__}: {e}"}
    try:
        cube = [float(v) for v in body["cube"]][:3]
        target = [float(v) for v in body["target"]][:2]
        if len(cube) == 2:
            cube.append(0.0)
        d = body.get("d")
        d = rc.MAP_D if d in (None, "") else float(d)
        if len(cube) != 3 or len(target) != 2 or not all(math.isfinite(v) for v in cube + target + [d]):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "error": "cube: [x, y, yaw], target: [x, y] 숫자"}
    r = rc.plan(tuple(cube), tuple(target), d)
    out = {k: ([round(float(x), 4) for x in v] if hasattr(v, "tolist") else v) for k, v in r.items()}
    yaw_sim = out.get("yaw", rc.fold_yaw(cube[2] + rc.SIM_FROM_REAL_YAW))
    one = {}
    for name, xy in (("cube", out["cube"][:2]), ("target", out["target"][:2])):
        ok, turn, why = rc.reachable_turned([xy], yaw_sim)
        one[name] = {"reachable": bool(ok), "hand_yaw": turn, "reason": why}
    out.update(ok=True, alone=one, d=d, map_d=rc.MAP_D, input={"cube": cube, "target": target})
    return out
