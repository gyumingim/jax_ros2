"""강화학습 탭의 자식 프로세스: team-jax/IK `sim2sim/` 체인(정우 님 grasp_lift 정책, MuJoCo)을 돌려 JSON 으로 낸다.

    python3 dashboard/rl_runner.py info --ik <IK 폴더>                         # 놓는 범위 표 + 27곳 (stdout JSON)
    python3 dashboard/rl_runner.py run  --ik <IK 폴더> --x 0 --y 0 --yaw 0 --out result.json
    python3 dashboard/rl_runner.py place --ik <IK 폴더> --lift result.json --tx 0 --ty -0.33 --tyaw 0 --out place.json
        # '역기구학으로 내리기': 들기(run) 결과의 끝 자세에서 이어 우리 IK 계획(pick_plan.plan_place_from)으로 놓고 HOME.
        # 결과 = 들기 결과 + 뒤에 붙인 내리기 프레임 (같은 꼴) + place {ok, reason, …}. mujoco 는 IK 레포 motion1 장면(계획기)만 쓴다

서버(rl_api.py)가 부른다. 서버 프로세스에는 mujoco 를 올리지 않는다. 시뮬만 돈다 — 실물 팔과 상관없다.

좌표: 입력 x, y(cm)·각도(°)는 정우 님 기준(기준 큐브 (−0.285, 0.26) m 에서 벗어난 양, 월드 x·y). 결과의 큐브·책상은 대시보드
3D 틀(viewer.js: 원점 = 어깨 관절, x 로봇 왼쪽, y 뒤, z 위)로 바꿔 낸다: 장면의 Shoulder_Pitch 몸체 틀 + x 0.052 m.

관절: sim2sim 장면(deploy/robot_v3 URDF, V2 부호)과 대시보드 3D 모델(motion2/arm/arm.xml)은 이름만 다르고 각도·부호가 같다
(JOINT_MAP). 무작위 자세 20개에서 두 모델의 몸체 14개 원점이 0.00 mm 로 같음을 확인했다(2026-11-23). 하나 다른 곳:
엄지 받침(hand_base_to_rotation_base)은 arm.xml 에 관절이 없고 −180° 에 굳어 있다 → 따로 `rot` 로 내고 화면(rl.js)이 돌린다.
"""
import argparse
import json
import math
import os
import sys

JOINT_MAP = {   # sim2sim (Isaac 이름) → 대시보드 (motion2 이름). 부호 그대로
    "shoulder_pitch_to_shoulder_roll": "shoulder_roll",
    "shoulder_roll_to_shoulder_yaw": "shoulder_yaw",
    "shoulder_yaw_to_upper_arm": "upper_arm_roll",
    "upper_arm_to_under_arm": "elbow",
    "under_arm_to_wrist_pitch": "wrist_pitch",
    "wrist_pitch_to_wrist_yaw": "wrist_yaw",
    "wrist_yaw_to_hand_base": "hand_roll",
    "hand_base_to_finger_1_1": "finger1_joint1",
    "finger_1_1_to_finger_1_2": "finger1_joint2",
    "hand_base_to_finger_2_1": "finger2_joint1",
    "finger_1_2_to_finger_2_2": "finger2_joint2",   # 정우 님 URDF 의 이름이 이렇다 (손가락 2 둘째 관절)
    "rotation_base_to_finger_3_1": "finger3_joint1",
    "finger_3_1_to_finger_3_2": "finger3_joint2",
}
ROT_JOINT = "hand_base_to_rotation_base"
EVERY = 2   # 60 Hz 기록 → 30 Hz 로 낸다
HZ = 30.0
PLACE_PHASES = ("CARRY", "LOWER", "OPEN", "RETREAT", "HOME")   # 내리기(우리 IK) 단계 — 들기 단계(chain.PHASES) 뒤에 붙인다
CUBE_HALF, CUBE_EDGE_IN = 0.025, 0.01


def load(ik):
    d = os.path.join(os.path.abspath(os.path.expanduser(ik)), "sim2sim")
    if not os.path.isfile(os.path.join(d, "chain.py")):
        raise SystemExit(f"sim2sim/chain.py 없음: {d}")
    sys.path.insert(0, d)
    import chain
    return d, chain


def info(args):
    d, C = load(args.ik)
    import run_eval
    t = C.Tables()
    meta = {}
    try:
        meta = json.load(open(os.path.join(C.DEPLOY, "grasp_lift_23025_meta.json"), encoding="utf-8"))
    except Exception:
        pass
    return {"ok": True, "sim2sim": d, "policy": "grasp_lift_23025", "checkpoint": meta.get("checkpoint"),
            "gx_cm": [round(float(v) * 100, 3) for v in t.gx], "gy_cm": [round(float(v) * 100, 3) for v in t.gy],
            "valid": [[int(bool(v)) for v in row] for row in t.mask["valid"]],
            "spots": [[round(x * 100, 2), round(y * 100, 2), yaw] for x, y, yaw in run_eval.spots()]}


def run(args):
    import mujoco
    import numpy as np
    d, C = load(args.ik)
    s, tr = C.run(args.x / 100.0, args.y / 100.0, args.yaw)
    m = mujoco.MjModel.from_xml_path(C.SCENE)
    dd = mujoco.MjData(m)
    mujoco.mj_kinematics(m, dd)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "Shoulder_Pitch")
    R, p0 = dd.xmat[b].reshape(3, 3).copy(), dd.xpos[b].copy()
    off = np.array([0.052, 0.0, 0.0])
    to_dash = lambda p: (np.asarray(p) - p0) @ R + off          # R^T (p − p0) + off
    qT = np.zeros(4)
    mujoco.mju_mat2Quat(qT, R.T.reshape(-1).copy())
    idx = list(range(0, len(tr["t"]), EVERY))
    if idx[-1] != len(tr["t"]) - 1:
        idx.append(len(tr["t"]) - 1)
    names = list(JOINT_MAP.values())
    cols = [C.ISAAC.index(k) for k in JOINT_MAP]
    cube = []
    for k in idx:
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, qT, tr["cube_quat"][k])
        p = to_dash(tr["cube_pos"][k])
        cube.append([round(float(v), 4) for v in p] + [round(float(q[i]), 4) for i in (1, 2, 3, 0)])   # x y z, quat x y z w
    close_t = s["phase_t_s"]["GRASP_LIFT"]
    ci = int(np.searchsorted(tr["t"], close_t - 1e-9))
    z0 = float(tr["cube_pos"][min(ci, len(tr["t"]) - 1), 2])
    tb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    table = None
    if tb >= 0:
        sz = m.geom_size[tb] * 2
        ax = np.abs(R.T @ np.diag(sz)).sum(axis=1)             # 축에 나란한 상자 (틀 바꿈이 90° 회전)
        table = {"pos": [round(float(v), 4) for v in to_dash(dd.geom_xpos[tb])], "size": [round(float(v), 4) for v in ax]}
    return {"ok": True, "spot": {"x_cm": args.x, "y_cm": args.y, "yaw_deg": args.yaw}, "summary": s,
            "phases": list(C.PHASES), "joint_names": names,
            "t": [round(float(tr["t"][k]), 3) for k in idx], "phase": [int(tr["phase"][k]) for k in idx],
            "q": [[round(float(tr["q"][k][c]), 4) for c in cols] for k in idx],
            "rot": [round(float(tr["q"][k][C.ISAAC.index(ROT_JOINT)]), 4) for k in idx],
            "cube": cube, "rise_cm": [round((float(tr["cube_pos"][k, 2]) - z0) * 100, 2) for k in idx],
            "table": table, "table_d": round(float(p0[2] - C.TABLE_TOP), 4)}


def place(args):
    """들기 결과(--lift) 끝 자세·손가락·큐브에서 이어서 놓기 계획. 놓을 자리 tx, ty 는 실좌표 m (= 3D 뷰·역기구학 탭 틀), tyaw °."""
    lift = json.load(open(args.lift, encoding="utf-8"))
    for k in ("place",):
        lift.pop(k, None)
    tgt = {"x": args.tx, "y": args.ty, "yaw_deg": args.tyaw}

    def fail(why):
        return dict(lift, place={"ok": False, "target": tgt, "reason": why})
    s = lift.get("summary") or {}
    if not s.get("done") or not s.get("still_up_at_end", True):
        return fail("들기가 성공으로 끝나야 이어서 내린다 (지금 결과는 실패)")
    tb = lift.get("table")
    if tb:   # 정우 님 장면의 책상(3D 에 보이는 책상) 위인가 — 우리 계획의 책상은 앞 끝만 있고 뒤·옆은 끝없다
        e = CUBE_HALF + CUBE_EDGE_IN
        x0, x1 = tb["pos"][0] - tb["size"][0] / 2 + e, tb["pos"][0] + tb["size"][0] / 2 - e
        y0, y1 = tb["pos"][1] - tb["size"][1] / 2 + e, tb["pos"][1] + tb["size"][1] / 2 - e
        if not (x0 <= args.tx <= x1 and y0 <= args.ty <= y1):
            return fail(f"놓을 자리 ({args.tx:+.3f}, {args.ty:+.3f}) 가 이 장면 책상 밖 — x {x0:+.2f}~{x1:+.2f}, y {y0:+.2f}~{y1:+.2f} m 안이어야")
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    import pick_plan
    names = lift["joint_names"]
    deg = {n: math.degrees(v) for n, v in zip(names, lift["q"][-1])}
    try:
        r = pick_plan.plan_place_from(deg, deg, lift["cube"][-1], (args.tx, args.ty), args.tyaw, d=lift.get("table_d"), ik=args.ik)
    except Exception as e:
        return fail(f"계획 오류: {type(e).__name__}: {e}")
    info = {"target": tgt, "reason": r.get("reason", ""), "status": r.get("status"), "checks": r.get("checks"),
            "start_over_limit": r.get("start_over_limit"), "plan_s": r.get("plan_s")}
    if r.get("status") != "playable":
        return dict(lift, place=dict(info, ok=False))
    # 계획 50 Hz → 화면 30 Hz (들기와 같은 간격), 관절 순서는 들기 결과 그대로
    P, dt, mk = r["positions"], r["dt"], r["marks"]
    col = [r["joint_names"].index(n) for n in names]
    n_pl = int(math.ceil((len(P) - 1) * dt * HZ - 1e-9))
    t_end, base = lift["t"][-1], lift["cube"][-1][2] - lift["rise_cm"][-1] / 100.0
    ends = [mk["above_place"], mk["place"], mk["opened"], mk["lift2"], len(P) - 1]   # PLACE_PHASES 가 끝나는 계획 프레임
    n0 = len(lift["phases"])
    out = dict(lift, phases=list(lift["phases"]) + list(PLACE_PHASES))
    for key in ("t", "phase", "q", "rot", "cube", "rise_cm"):
        out[key] = list(lift[key])
    for k in range(1, n_pl + 1):
        x = min(k / HZ / dt, len(P) - 1.0)
        i = min(int(x), len(P) - 2)
        u = x - i
        mix = lambda a, b: [a[j] + (b[j] - a[j]) * u for j in range(len(a))]
        q, c = mix(P[i], P[i + 1]), mix(r["cube"][i], r["cube"][i + 1])
        nq = math.sqrt(sum(v * v for v in c[3:])) or 1.0
        c = c[:3] + [v / nq for v in c[3:]]
        out["t"].append(round(t_end + k / HZ, 3))
        out["phase"].append(n0 + next(j for j, e in enumerate(ends) if x <= e + 1e-9))
        out["q"].append([round(q[j], 4) for j in col])
        out["rot"].append(lift["rot"][-1])   # 엄지 받침은 정책 끝 값 그대로 (우리 계획 모델엔 이 관절이 없다)
        out["cube"].append([round(v, 4) for v in c])
        out["rise_cm"].append(round((c[2] - base) * 100, 2))
    out["place"] = dict(info, ok=True, start_frame=len(lift["t"]) - 1, seconds=round(n_pl / HZ, 1), total_s=out["t"][-1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("info", "run", "place"))
    ap.add_argument("--lift")
    ap.add_argument("--tx", type=float, default=0.0)
    ap.add_argument("--ty", type=float, default=0.0)
    ap.add_argument("--tyaw", type=float, default=0.0)
    ap.add_argument("--ik", required=True)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    for v in (a.x, a.y, a.yaw, a.tx, a.ty, a.tyaw):
        if not math.isfinite(v):
            raise SystemExit("x, y, 각도는 숫자")
    out = info(a) if a.cmd == "info" else run(a) if a.cmd == "run" else place(a)
    text = json.dumps(out, ensure_ascii=False)
    if a.out:
        tmp = a.out + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(tmp, a.out)
        print("ok")
    else:
        print(text)


if __name__ == "__main__":
    main()
