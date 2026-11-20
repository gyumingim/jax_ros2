"""역기구학 탭 '손 위치로 보내기': 손이 쥐는 점(x, y, z 실좌표) + 손 방향 -> 팔 관절각 (team-jax/IK 의 env·ik 재사용).

MuJoCo 모델은 서버 프로세스가 아니라 자식 프로세스(이 파일 --worker) 하나에서 한 번 읽고 계속 쓴다
(JAX_SETUP=real 은 humanoid_grasp_rl.env 를 처음 import 할 때 읽혀서, 서버 안에서 섞이지 않게). JSON 한 줄씩 주고받는다.

손 방향 (env.py 와 같은 정의):
  grasp    = 실물 셋업의 파지 자세: 손가락 아래 + 손가락 사이 축으로 HAND_TILT(-40°) 기울임 (PALM_DOWN),
             손 축 따라 GRASP_DZ(21.5 mm) 위가 파지 중심(end_effector) — env.pick_pose 와 같다. 큐브 잡는 자세 그대로.
  vertical = 손가락 수직 아래(기울기 0, 시뮬 셋업의 손), GRASP_DZ 0. 실물 책상(어깨 아래 477.5 mm)에선 대부분 안 닿는다.
  auto     = (기본) 손가락 아래, 기울기 자동: 손가락 사이 축(손 x = 손가락 1·2 ↔ 3 이 닫히는 방향) 둘레 기울기 t 는
             ±tilt_max(기본 60°) 안에서 IK 가 고른다. 그 축 둘레로 돌려도 패드는 같은 두 면에 평행하게 닿아 파지가 안 바뀐다.
             풀이: 쥐는 점 3 + 손 x 축 방향 2 를 주 과제로(그 축 둘레 회전은 자유), |t| 가 한계를 넘을 때만 한계에서 묶고,
             남는 자유도로 t -> 0(수직 쪽)과 HOME 쪽으로 당긴다. 후보 고르기: 손가락이 책상에 안 닿음 -> 수직에 가까움(10° 단위)
             -> 지금 자세에서 가까움. GRASP_DZ 는 grasp 와 같은 21.5 mm (env 와 같은 정의).
  yaw      = 이 손이 쥘 큐브의 yaw (실좌표, 지도와 같은 기준). 손 돌림은 그 yaw 의 세 면(+0, ±90) × 손 그대로/반 바퀴(HAND_FLIP)
             중 관절 한계 안에서 풀리는 것, 그중 지금 자세에서 가장 가까운 것.
점(x, y, z) = 손이 쥐는 큐브의 중심(파지 점). z 는 실좌표(어깨 축 기준, 위 +). 화면은 '책상 위 cm' 로 받아 바꾼다.
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

SUB = Path("motion1") / "src"
TOL_POS = 0.005               # 풀림 판정: 위치 5 mm (env.reachable 의 position_tolerance)
TOL_ROT = math.radians(1.5)   # 손 방향 1.5°
FAST_ITERS = 80               # 드래그 중 빠른 풀이의 반복 수 (직전 해에서 시작)
MODES = ("auto", "grasp", "vertical")
TILT_MAX_DEFAULT = 60.0       # auto 의 기울기 상한 (°)
TILT_PULL = 0.05              # auto: 남는 자유도로 기울기를 0 쪽으로 당기는 세기 (한 반복에 t 의 몇 배)


# ============================================================ worker (자식 프로세스)
class _Solver:
    def __init__(self, ik_dir):
        os.environ["JAX_SETUP"] = "real"
        sys.path.insert(0, str(Path(ik_dir) / SUB))
        sys.path.insert(0, str(Path(ik_dir) / SUB / "humanoid_grasp_rl"))
        sys.path += [x for x in os.environ.get("JAX_DASH_PYTHONPATH", "").split(os.pathsep) if x]
        import numpy as np
        import mujoco
        import humanoid_grasp_rl.env as E
        from humanoid_grasp_rl.ik import rotation_error, solve_site_pose
        from humanoid_grasp_rl.single_env import JaxDeliveryEnv
        import realcoords as R
        self.np, self.mj, self.E, self.R = np, mujoco, E, R
        self.solve_site_pose, self.rotation_error = solve_site_pose, rotation_error
        env = self.env = JaxDeliveryEnv()
        env.reset(seed=0)
        m = self.m = env.model
        self.d = mujoco.MjData(m)
        self.d.qpos[:] = env.data.qpos
        self.arm = list(env._robot.arm_joints)
        self.fingers = list(env._robot.finger_joints)
        self.jid = env.arm_joint_ids
        self.qadr = m.jnt_qposadr[self.jid]
        self.fadr = m.jnt_qposadr[env.finger_joint_ids]
        self.site = env.site_id
        self.base_range = m.jnt_range[self.jid].copy()   # env 가 hand_roll 을 서보 범위(±88)로 줄인 뒤
        self.hand_roll = self.arm.index("hand_roll")
        self.finger_geoms = [g for g in range(m.ngeom) if env._is_hand_geom[g] and m.geom_contype[g]]
        self.table_z = float(E.TABLE_SURFACE_Z)
        self.open_fingers = env.finger_targets(0.0)
        self.d.qpos[self.fadr] = self.open_fingers

    # ---- 좌표
    def to_sim(self, p):
        return self.R.to_sim(p)

    def to_real(self, s):
        np, R = self.np, self.R
        c, sn = math.cos(math.radians(R.SIM_FROM_REAL_YAW)), math.sin(math.radians(R.SIM_FROM_REAL_YAW))
        o = np.array([R.ORIGIN_XY[0], R.ORIGIN_XY[1], R.TABLE_Z + R.MAP_D])
        dx, dy, dz = np.asarray(s) - o
        return [c * dx + sn * dy, -sn * dx + c * dy, dz]

    def hand_rot(self, mode, turn_deg, flip, tilt_deg=None):
        """손 방향 = z 둘레 turn · 손가락 사이 축(x) 둘레 기울기 · (반 바퀴). tilt_deg 가 있으면 그 값(auto),
        없으면 grasp = HAND_TILT, vertical/auto = 0."""
        np, E = self.np, self.E
        if tilt_deg is None:
            tilt_deg = E.HAND_TILT if mode == "grasp" else 0.0
        t = math.radians(tilt_deg)
        palm = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
        if flip:
            palm = palm @ E.HALF_TURN
        a = math.radians(turn_deg)
        rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        return rz @ palm

    def dz(self, mode):
        return float(self.E.GRASP_DZ) if mode in ("grasp", "auto") else 0.0

    def tilt_of(self, rot, turn_deg, flip):
        """손 회전 rot 을 rz(turn)·Rx(t)·(HALF_TURN) 꼴로 볼 때 가장 가까운 t (rad)."""
        np = self.np
        a = math.radians(turn_deg)
        rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        M = rz.T @ rot @ (self.E.HALF_TURN if flip else np.eye(3))
        return math.atan2(M[2, 1] - M[1, 2], M[1, 1] + M[2, 2])

    def solve_free_tilt(self, seed, point, turn_deg, flip, tilt_max_deg, its=400, tilt_init_deg=None, warm=120, pull=TILT_PULL, home_pull=0.02, polish=40):
        """auto: 손이 쥐는 점(point, sim) + 손 x 축(손가락이 닫히는 방향) = rz(turn)·(±x) 만 맞추고, x 축 둘레 기울기 t 는
        |t| <= tilt_max 안에서 자유(넘으면 그 한계에 묶음). 남는 자유도로 t -> 0, 관절 -> HOME.
        돌려줌 (q, 쥐는 점 오차 m, 손 방향 오차 rad(x 축 + 한계 넘은 만큼), t rad)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        dz = float(self.E.GRASP_DZ)
        tmax = math.radians(tilt_max_deg)
        a = math.radians(turn_deg)
        axis = np.array([math.cos(a), math.sin(a), 0.0])      # 기울기 축(세계) = rz·e_x
        dof = m.jnt_dofadr[self.jid]
        lim = m.jnt_range[self.jid]
        home = np.asarray(self.E.HOME_ARM_QPOS, dtype=float)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        damp = 1e-3 * np.eye(6)
        d.qpos[self.qadr] = seed
        d.qvel[:] = 0

        def state(k=its):
            mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
            R = d.site_xmat[self.site].reshape(3, 3).copy()
            t = self.tilt_of(R, turn_deg, flip)
            tc = max(-tmax, min(tmax, t)) if tilt_init_deg is None or k >= warm else math.radians(tilt_init_deg)
            ep = point - (d.site_xpos[self.site] - dz * R[:, 2])
            er = self.rotation_error(R, self.hand_rot("auto", turn_deg, flip, math.degrees(tc)))
            return R, t, ep, er

        t_prev, settle = None, 0
        for k in range(its):
            R, t, ep, er = state(k)
            held = tilt_init_deg is not None and k < warm          # 처음 warm 번은 기울기를 tilt_init 에 묶고 시작(가지 고르기)
            pe, re_ = np.linalg.norm(ep), np.linalg.norm(er)
            settle = settle + 1 if (not held and pe <= 0.0005 and re_ <= 0.004 and t_prev is not None
                                    and abs(t - t_prev) < 2e-4) else 0
            if settle >= 15:                                   # 맞았고 기울기도 더 안 움직임
                break
            t_prev = t
            mj.mj_jacSite(m, d, jp, jr, self.site)
            Jp, Jr = jp[:, dof], jr[:, dof]
            z = R[:, 2]
            sk = np.array([[0.0, -z[2], z[1]], [z[2], 0.0, -z[0]], [-z[1], z[0], 0.0]])
            Jg = Jp + dz * sk @ Jr                             # 쥐는 점(site - dz·z)의 자코비안
            P = np.eye(3) if held or abs(t) > tmax else np.eye(3) - np.outer(axis, axis)   # 한계 안이면 축 둘레 회전은 자유
            J0 = np.vstack((Jg, 0.25 * P @ Jr))
            e = np.concatenate((ep, 0.25 * P @ er))
            q = d.qpos[self.qadr].copy()
            # 남는 자유도 당기기(기울기 -> 0, HOME 쪽)는 주 과제가 거의 맞았을 때만, 마지막 polish 번은 안 함
            sec = np.zeros(len(q))
            if not held and k < its - polish and pe < 0.002 and re_ < 0.02:
                jt = axis @ Jr                                 # dt/dq
                sec = home_pull * (home - q) + jt * (-pull * t) / (jt @ jt + 1e-6)
            free = np.ones(len(q), dtype=bool)
            for _ in range(2):                                 # 한계에 붙은 관절이 밖으로 밀리면 그 관절을 빼고 다시(clip 이 위치를 깨지 않게)
                J = J0 * free
                JJ = J @ J.T + damp
                step = J.T @ np.linalg.solve(JJ, e) + (sec * free - J.T @ np.linalg.solve(JJ, J @ (sec * free)))
                out = ((q <= lim[:, 0] + 1e-9) & (step < 0)) | ((q >= lim[:, 1] - 1e-9) & (step > 0))
                if not (out & free).any():
                    break
                free &= ~out
            d.qpos[self.qadr] = np.clip(q + 0.25 * step, lim[:, 0], lim[:, 1])
        R, t, ep, er = state()
        return d.qpos[self.qadr].copy(), float(np.linalg.norm(ep)), float(np.linalg.norm(er)), t

    # ---- 한 자세의 FK
    def fk(self, q):
        mj, np = self.mj, self.np
        self.d.qpos[self.qadr] = q
        mj.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def lowest_finger_z(self):
        """지금 self.d 자세에서 손가락 충돌 상자 8 꼭짓점 중 가장 낮은 z (sim)."""
        np = self.np
        low = 9.0
        for g in self.finger_geoms:
            c, R, h = self.d.geom_xpos[g], self.d.geom_xmat[g].reshape(3, 3), self.m.geom_size[g]
            low = min(low, float(c[2] - np.abs(R[2]) @ h))
        return low

    def set_ranges(self, ranges_deg, servo_unlimited=False):
        """관절 한계 = 모델(hand_roll ±88) ∩ 보낼 수 있는 범위(calibration cmd_range). servo_unlimited: 진단용으로 hand_roll 만 URDF ±120."""
        np = self.np
        r = self.base_range.copy()
        for i, n in enumerate(self.arm):
            if ranges_deg and n in ranges_deg and ranges_deg[n]:
                lo, hi = (math.radians(float(v)) for v in ranges_deg[n])
                r[i] = [max(r[i][0], lo), min(r[i][1], hi)]
        if servo_unlimited:
            r[self.hand_roll] = np.radians([-120.0, 120.0])
        self.m.jnt_range[self.jid] = r
        return r

    def solve_from(self, seed, target, rot, its=400):
        self.d.qpos[self.qadr] = seed
        self.d.qvel[:] = 0
        _, pe, re_ = self.solve_site_pose(self.m, self.d, self.site, self.jid, target, rot, self.E.HOME_ARM_QPOS,
                                          position_tolerance=0.0005, rotation_tolerance=0.004, max_iterations=its)
        return self.d.qpos[self.qadr].copy(), pe, re_

    def position_only(self, seeds, target, its=300):
        """손 방향을 빼고 위치만: 팔이 그 점에 닿기는 하나 (진단). 가장 가까운 거리(m)."""
        np, mj = self.np, self.mj
        lim = self.m.jnt_range[self.jid]
        dof = self.m.jnt_dofadr[self.jid]
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        best = 9.0
        for s in seeds:
            self.d.qpos[self.qadr] = s
            for _ in range(its):
                mj.mj_kinematics(self.m, self.d); mj.mj_comPos(self.m, self.d)
                e = target - self.d.site_xpos[self.site]
                if np.linalg.norm(e) < 5e-4:
                    break
                mj.mj_jacSite(self.m, self.d, jp, jr, self.site)
                J = jp[:, dof]
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), e)
                self.d.qpos[self.qadr] = np.clip(self.d.qpos[self.qadr] + 0.5 * dq, lim[:, 0], lim[:, 1])
            mj.mj_kinematics(self.m, self.d)
            best = min(best, float(np.linalg.norm(target - self.d.site_xpos[self.site])))
        return best

    def seeds(self, current):
        np, E = self.np, self.E
        home = np.asarray(E.HOME_ARM_QPOS, dtype=float)
        out = ([current] if current is not None else []) + [home]
        for hr in (-60.0, 0.0, 60.0):
            s = home.copy(); s[self.hand_roll] = math.radians(hr); out.append(s)
        out.append(np.zeros(len(self.arm)))
        return out

    def candidates(self, mode, yaw_real, point_sim, current, ranges_deg, servo_unlimited=False, tilt_max=TILT_MAX_DEFAULT):
        np, R = self.np, self.R
        lim = self.set_ranges(ranges_deg, servo_unlimited)
        y = R.fold_yaw(yaw_real + R.SIM_FROM_REAL_YAW)
        turns = sorted({round(y, 6), round(y + 90.0, 6), round(y - 90.0, 6)}, key=abs)
        out = []
        for turn in turns:
            for flip in (False, True):
                for s in self.seeds(current):
                    s = np.clip(s, lim[:, 0], lim[:, 1])
                    if mode == "auto":
                        q, pe, re_, t = self.solve_free_tilt(s, point_sim, turn, flip, tilt_max)
                        tilt = math.degrees(t)
                        rot = self.hand_rot(mode, turn, flip, max(-tilt_max, min(tilt_max, tilt)))
                    else:
                        rot = self.hand_rot(mode, turn, flip)
                        q, pe, re_ = self.solve_from(s, point_sim + self.dz(mode) * rot[:, 2], rot)
                        tilt = float(self.E.HAND_TILT if mode == "grasp" else 0.0)
                    out.append({"q": q, "pe": pe, "re": re_, "turn": turn, "flip": flip, "rot": rot, "tilt": tilt})
        return out, lim

    def solve(self, b):
        """fix_deg {관절: 각} 이 있으면 그 관절을 그 각에 묶고(범위를 [v, v] 로) 나머지로 푼다. 7 관절에 손 자세 6 값이라
        팔 모양이 하나 남는데(팔꿈치 위치), 하나를 묶으면 거의 하나로 정해진다. 묶어서 안 풀리면 풀고 다시 풀어 비교한다."""
        fix = {k: float(v) for k, v in (b.get("fix_deg") or {}).items() if v is not None}
        ranges = dict(b.get("ranges_deg") or {})
        for j, v in fix.items():
            if j not in self.arm or not math.isfinite(v):
                return {"ok": False, "kind": "input", "reason": f"고정할 관절 {j!r} 이 이상함"}
            i = self.arm.index(j)
            lo, hi = (math.degrees(float(x)) for x in self.base_range[i])
            if ranges.get(j):
                lo, hi = max(lo, float(ranges[j][0])), min(hi, float(ranges[j][1]))
            if not lo - 1e-6 <= v <= hi + 1e-6:
                return {"ok": False, "kind": "input", "reason": f"{j} 고정값 {v:g}° 가 범위 {lo:g}~{hi:g}° 밖"}
        if not fix:
            return {**self._solve_once(b, ranges, ()), "fixed": {}}
        r = self._solve_once(b, {**ranges, **{j: [v, v] for j, v in fix.items()}}, tuple(fix))
        r["fixed"] = fix
        if not r.get("ok") and r.get("kind") != "below_table":
            u = self._solve_once(b, ranges, ())
            r["unfixed"] = {k: u.get(k) for k in ("ok", "pos_err_mm", "ang_err_deg", "joints", "flip", "hand_turn", "reason")}
            if u.get("ok"):
                names = ", ".join(f"{j} {u['joints'][j]:+.1f}°" for j in fix)
                r["kind"] = "fixed"
                r["reason"] = (f"어깨 옆 고정({', '.join(f'{v:g}°' for v in fix.values())}) 때문 — 고정을 풀면 풀림 ({names}, 오차 {u['pos_err_mm']} mm). "
                               f"고정한 채 가장 가까운 해는 {r.get('pos_err_mm')} mm · {r.get('ang_err_deg')}° 차이")
        return r

    def _solve_once(self, b, ranges, fixed):
        np, R, E = self.np, self.R, self.E
        mode = b.get("mode") or "auto"
        if mode not in MODES:
            return {"ok": False, "reason": f"mode 는 {' | '.join(MODES)} ({mode!r})"}
        tilt_max = min(90.0, max(0.0, float(TILT_MAX_DEFAULT if b.get("tilt_max_deg") is None else b["tilt_max_deg"])))
        p = [float(b[k]) for k in ("x", "y", "z")]
        yaw = float(b.get("yaw_deg") or 0.0)
        cur = b.get("current_deg") or {}
        current = None
        if all(n in cur for n in self.arm):
            current = np.radians([float(cur[n]) for n in self.arm])
        z_table = p[2] + R.MAP_D   # 책상 위 높이 (m)
        base = {"mode": mode, "input": {"x": p[0], "y": p[1], "z": p[2], "yaw_deg": yaw, "z_table_m": z_table},
                "tilt_deg": float(E.HAND_TILT if mode == "grasp" else 0.0), "tilt_max_deg": tilt_max if mode == "auto" else None,
                "grasp_dz_m": self.dz(mode)}
        if z_table < 0.0:
            return {**base, "ok": False, "kind": "below_table",
                    "reason": f"책상 아래 — 점이 책상 면보다 {-z_table * 100:.1f} cm 아래 (책상 = 어깨 아래 {R.MAP_D * 100:.2f} cm)"}
        point = np.asarray(self.to_sim(p))
        t0 = time.time()
        cands, lim = self.candidates(mode, yaw, point, current, ranges, tilt_max=tilt_max)
        ref = current if current is not None else np.asarray(E.HOME_ARM_QPOS)
        tol_p, tol_r = TOL_POS, TOL_ROT   # env.reachable 과 같은 5 mm, 손 방향 1.5°
        good = [c for c in cands if c["pe"] <= tol_p and c["re"] <= tol_r]
        for c in cands:
            c["dist"] = float(np.abs(c["q"] - ref).max())
        out = dict(base)
        if good:
            # 손가락이 책상에 안 닿는 해 먼저, 그다음 지금 자세에서 가장 가까운(관절 최대 이동각) 해
            for c in good:
                self.fk(c["q"]); self.d.qpos[self.fadr] = self.open_fingers
                self.mj.mj_kinematics(self.m, self.d)
                c["low"] = self.lowest_finger_z() - self.table_z
            # auto: 그다음 수직에 가까운(10° 단위) 해 먼저
            best = min(good, key=lambda c: (c["low"] < -0.001, round(abs(c["tilt"]) / 10.0) if mode == "auto" else 0, c["dist"]))
            out["ok"] = True
        else:
            best = min(cands, key=lambda c: (c["pe"] + 0.05 * c["re"], c["dist"]))
            self.fk(best["q"]); self.d.qpos[self.fadr] = self.open_fingers
            self.mj.mj_kinematics(self.m, self.d)
            best["low"] = self.lowest_finger_z() - self.table_z
            out["ok"] = False
        q = best["q"]
        site, rot = self.fk(q)
        grip = site - self.dz(mode) * rot[:, 2]
        at_lim = [self.arm[i] for i in range(len(self.arm)) if self.arm[i] not in fixed and (q[i] <= lim[i][0] + 1e-3 or q[i] >= lim[i][1] - 1e-3)]
        out.update(joints={n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, q)},
                   pos_err_mm=round(float(np.linalg.norm(grip - point)) * 1000, 1),
                   ang_err_deg=round(math.degrees(best["re"]), 2),
                   flip=bool(best["flip"]), hand_turn=round(float(best["turn"]), 2), tilt_deg=round(float(best["tilt"]), 1),
                   finger_low_mm=round(best["low"] * 1000, 1), at_limit=at_lim,
                   max_move_deg=round(math.degrees(best["dist"]), 1) if current is not None else None,
                   grip_real=[round(float(v), 4) for v in self.to_real(grip)],
                   site_real=[round(float(v), 4) for v in self.to_real(site)],
                   n_good=len(good), n_tried=len(cands), from_current=current is not None,
                   open_fingers_deg={n: round(math.degrees(float(v)), 2) for n, v in zip(self.fingers, self.open_fingers)})
        warn = []
        if out["ok"] and best["low"] < -0.001:
            warn.append(f"손가락이 책상에 {-best['low'] * 1000:.0f} mm 파묻힘 (벌린 손가락 기준) — 더 높게")
        if out["ok"]:
            out["reason"] = "풀림" + (" · " + " · ".join(warn) if warn else "")
            out["warn"] = warn
        else:
            out["kind"], out["reason"] = self.why(mode, yaw, point, current, ranges, best, tilt_max)
        out["ms"] = round((time.time() - t0) * 1000)
        return out

    def fast(self, b):
        """드래그 중 빠른 풀이: 직전 해(seed_deg)에서 한 번만, 같은 손 모양(flip)·가장 가까운 손 돌림으로. 반복 FAST_ITERS."""
        np, R, E = self.np, self.R, self.E
        t0 = time.time()
        mode = b.get("mode") or "auto"
        p = [float(b[k]) for k in ("x", "y", "z")]
        yaw = float(b.get("yaw_deg") or 0.0)
        ranges = dict(b.get("ranges_deg") or {})
        fix = {k: float(v) for k, v in (b.get("fix_deg") or {}).items()}
        ranges.update({j: [v, v] for j, v in fix.items()})
        seed_d = b.get("seed_deg") or {}
        if not all(n in seed_d for n in self.arm):
            return {"ok": False, "fast": True, "kind": "input", "reason": "빠른 풀이에 시작 자세가 없음"}
        if p[2] + R.MAP_D < 0.0:
            return {"ok": False, "fast": True, "kind": "below_table", "reason": "책상 아래"}
        lim = self.set_ranges(ranges)
        flip = bool(b.get("flip"))
        prev = float(b.get("hand_turn") or 0.0)
        y = R.fold_yaw(yaw + R.SIM_FROM_REAL_YAW)
        turn = min((y, y + 90.0, y - 90.0), key=lambda t: abs(t - prev))
        point = np.asarray(self.to_sim(p))
        seed = np.clip(np.radians([float(seed_d[n]) for n in self.arm]), lim[:, 0], lim[:, 1])
        if mode == "auto":
            tilt_max = min(90.0, max(0.0, float(TILT_MAX_DEFAULT if b.get("tilt_max_deg") is None else b["tilt_max_deg"])))
            q, pe, re_, t = self.solve_free_tilt(seed, point, turn, flip, tilt_max, its=FAST_ITERS)
            tilt = math.degrees(t)
        else:
            rot = self.hand_rot(mode, turn, flip)
            q, pe, re_ = self.solve_from(seed, point + self.dz(mode) * rot[:, 2], rot, its=FAST_ITERS)
            tilt = float(E.HAND_TILT if mode == "grasp" else 0.0)
        site, srot = self.fk(q)
        self.d.qpos[self.fadr] = self.open_fingers
        self.mj.mj_kinematics(self.m, self.d)
        low = self.lowest_finger_z() - self.table_z
        grip = site - self.dz(mode) * srot[:, 2]
        ok = pe <= TOL_POS and re_ <= TOL_ROT
        return {"ok": bool(ok), "fast": True, "mode": mode, "flip": flip, "hand_turn": round(float(turn), 2), "tilt_deg": round(tilt, 1), "fixed": fix,
                "joints": {n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, q)},
                "pos_err_mm": round(float(np.linalg.norm(grip - point)) * 1000, 1), "ang_err_deg": round(math.degrees(re_), 2),
                "finger_low_mm": round(low * 1000, 1), "warn": [] if low >= -0.001 else [f"손가락이 책상에 {-low * 1000:.0f} mm 파묻힘"],
                "open_fingers_deg": {n: round(math.degrees(float(v)), 2) for n, v in zip(self.fingers, self.open_fingers)},
                "reason": "풀림 (빠른 풀이)" if ok else "빠른 풀이가 못 따라감 — 손을 떼면 전체 풀이", "ms": round((time.time() - t0) * 1000, 1)}

    def why(self, mode, yaw, point, current, ranges, best, tilt_max=TILT_MAX_DEFAULT):
        """실패 이유를 사람 말로: 위치부터 안 닿나 → 손 비틀기 서보(±88)만 풀면 되나 → 그 손 방향이 안 되나."""
        np = self.np
        self.set_ranges(ranges)
        reach = self.position_only(self.seeds(current), point + self.dz(mode) * best["rot"][:, 2])
        miss = best["pe"] * 1000
        if reach > TOL_POS:
            return "no_reach", f"팔이 안 닿음 (손 방향을 빼도 {reach * 1000:.0f} mm 모자람, 가장 가까운 해 {miss:.0f} mm 차이)"
        wide, _ = self.candidates(mode, yaw, point, current, ranges, servo_unlimited=True, tilt_max=tilt_max)
        ok = [c for c in wide if c["pe"] <= TOL_POS and c["re"] <= TOL_ROT]
        self.set_ranges(ranges)
        if ok:
            hr = min(ok, key=lambda c: abs(c["q"][self.hand_roll]))["q"][self.hand_roll]
            return "hand_roll", f"손 비틀기 서보 범위 밖 — 이 손 방향이면 손 비틀기 {math.degrees(hr):+.0f}° 필요 (서보 ±88°). 손 방향(yaw)을 바꿔 보기"
        tilt = ("수직(0°)" if mode == "vertical" else f"{self.E.HAND_TILT:+.0f}° 기울인" if mode == "grasp"
                else f"기울기 ±{tilt_max:.0f}° 안의")
        return "orientation", (f"그 손 방향으로는 안 됨 — 위치만은 닿지만 {tilt} 손으로는 가장 가까운 해가 "
                               f"{miss:.0f} mm · {math.degrees(best['re']):.0f}° 차이")

    def fk_req(self, b):
        np = self.np
        js = b.get("joints_deg") or {}
        q = np.radians([float(js.get(n, 0.0)) for n in self.arm])
        mode = b.get("mode") or "grasp"
        site, rot = self.fk(q)
        grip = site - self.dz(mode) * rot[:, 2]
        # 손 축(site z) 이 수직에서 얼마나 기울었나: PALM_DOWN(기울기 0) 이면 site z = 세계 +z
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
        return {"ok": True, "grip_real": [round(float(v), 4) for v in self.to_real(grip)],
                "site_real": [round(float(v), 4) for v in self.to_real(site)], "axis_tilt_deg": round(tilt, 1)}


def _worker(ik_dir):
    try:
        s = _Solver(ik_dir)
        print(json.dumps({"ready": True, "arm": s.arm, "fingers": s.fingers,
                          "open_fingers_deg": [round(math.degrees(float(v)), 2) for v in s.open_fingers]}), flush=True)
    except Exception as e:
        print(json.dumps({"ready": False, "error": f"{type(e).__name__}: {e}"}), flush=True)
        return
    for line in sys.stdin:
        try:
            b = json.loads(line)
            op = b.get("op")
            r = s.solve(b) if op == "solve" else s.fast(b) if op == "fast" else s.fk_req(b) if op == "fk" else {"ok": False, "reason": f"모르는 op {op!r}"}
        except Exception as e:
            r = {"ok": False, "reason": f"풀이 중 오류: {type(e).__name__}: {e}", "kind": "error"}
        print(json.dumps(r), flush=True)


# ============================================================ client (서버 안)
class HandIK:
    """자식 프로세스 하나. 첫 호출에서 띄우고(모델 읽기 약 1~3초) 이후 재사용. 호출은 하나씩(lock)."""

    def __init__(self):
        self.p = None
        self.info = None
        self.lock = threading.Lock()
        self.ik_dir = None
        self.started = None

    def _start(self, ik_dir):
        self.close()
        env = dict(os.environ, JAX_SETUP="real", PYTHONIOENCODING="utf-8")
        self.p = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve()), "--worker", str(ik_dir)],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, encoding="utf-8", env=env,
                                  creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self.ik_dir, self.started = ik_dir, time.time()
        self.info = self._read(60.0)
        if not self.info or not self.info.get("ready"):
            err = (self.info or {}).get("error") or "자식 프로세스가 응답 없음"
            self.close()
            raise RuntimeError(f"IK 모델을 못 읽음: {err}")

    def _read(self, timeout):
        box = {}
        t = threading.Thread(target=lambda: box.update(line=self.p.stdout.readline()), daemon=True)
        t.start(); t.join(timeout)
        line = box.get("line")
        if not line:
            return None
        return json.loads(line)

    def call(self, ik_dir, req, timeout=60.0):
        with self.lock:
            first = False
            if self.p is None or self.p.poll() is not None or self.ik_dir != ik_dir:
                t0 = time.time()
                self._start(ik_dir)
                first = time.time() - t0
            self.p.stdin.write(json.dumps(req) + "\n"); self.p.stdin.flush()
            r = self._read(timeout)
            if r is None:
                self.close()
                raise RuntimeError("IK 풀이가 응답 없음 (시간 초과)")
            if first:
                r["load_s"] = round(first, 1)
            return r

    def close(self):
        if self.p is not None:
            try:
                self.p.kill()
            except Exception:
                pass
        self.p = None


SOLVER = HandIK()


def solve(ik_dir, body, ranges_deg, current_deg):
    """POST /api/ik/hand. body: {x, y, z 실좌표 m | z_table_cm, yaw_deg, mode, fix_deg?: {관절: 각}}."""
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음 (JAX_IK_DIR)"}
    b = body if isinstance(body, dict) else {}
    try:
        x, y = float(b["x"]), float(b["y"])
        if b.get("z") not in (None, ""):
            z = float(b["z"])
        else:
            z = float(b["z_table_cm"]) / 100.0 - 0.4775
        yaw = float(b.get("yaw_deg") or 0.0)
        fix = b.get("fix_deg") or {}
        if not isinstance(fix, dict) or len(fix) > 7:
            raise ValueError
        fix = {str(k): float(v) for k, v in fix.items()}
        if not all(math.isfinite(v) for v in (x, y, z, yaw)) or max(abs(x), abs(y), abs(z)) > 2.0:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "reason": "x, y (m), z (m) 또는 z_table_cm, yaw_deg 숫자 (|x|,|y|,|z| ≤ 2 m)"}
    mode = b.get("mode") or "auto"
    try:
        tilt_max = float(TILT_MAX_DEFAULT if b.get("tilt_max_deg") is None else b["tilt_max_deg"])
        if not math.isfinite(tilt_max):
            raise ValueError
    except (TypeError, ValueError):
        return {"ok": False, "reason": "tilt_max_deg 숫자 (0~90°)"}
    if b.get("fast"):
        seed = b.get("seed_deg")
        if not isinstance(seed, dict):
            return {"ok": False, "fast": True, "reason": "seed_deg 없음"}
        try:
            return SOLVER.call(ik_dir, {"op": "fast", "x": x, "y": y, "z": z, "yaw_deg": yaw, "mode": mode, "fix_deg": fix,
                                        "ranges_deg": ranges_deg, "seed_deg": {str(k): float(v) for k, v in seed.items()},
                                        "flip": bool(b.get("flip")), "hand_turn": float(b.get("hand_turn") or 0.0),
                                        "tilt_max_deg": tilt_max}, timeout=10.0)
        except Exception as e:
            return {"ok": False, "fast": True, "reason": str(e), "kind": "error"}
    try:
        return SOLVER.call(ik_dir, {"op": "solve", "x": x, "y": y, "z": z, "yaw_deg": yaw, "mode": mode, "tilt_max_deg": tilt_max,
                                    "ranges_deg": ranges_deg, "current_deg": current_deg, "fix_deg": fix})
    except Exception as e:
        return {"ok": False, "reason": str(e), "kind": "error"}


def fk(ik_dir, joints_deg, mode="grasp"):
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음"}
    try:
        return SOLVER.call(ik_dir, {"op": "fk", "joints_deg": joints_deg, "mode": mode})
    except Exception as e:
        return {"ok": False, "reason": str(e)}


if __name__ == "__main__" and len(sys.argv) >= 3 and sys.argv[1] == "--worker":
    _worker(sys.argv[2])
