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
             남는 자유도로 t -> 0(수직 쪽)과 HOME 쪽으로 당긴다. 후보 고르기(2026-11-20 사용자): 손가락이 책상에 닿는 해는 탈락
             -> 지금 자세에서 최소 회전(관절 최대 이동각) -> 그 최소에서 2° 안이면 수직에 가까운 것. GRASP_DZ 는 grasp 와 같은 21.5 mm (env 와 같은 정의).
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
MODES = ("auto", "grasp", "vertical", "position")   # position = 기준점 위치만 풀고, 손목 기준이면 손목 2 관절로 손이 최대한 아래 (hand_roll 은 지금 값)
TILT_MAX_DEFAULT = 60.0       # auto 의 기울기 상한 (°)
AUTO_TILT_INITS = (0.0, -40.0, 40.0)   # auto: 시작 기울기(처음 120 번 묶음) — 가지 고르기
TIE_DEG = 2.0                 # 해 고르기: 최소 회전에서 이 각 안이면 동률 → 손 기울기 수직에 가까운 것
TILT_PULL = 0.05              # auto: 남는 자유도로 기울기를 0 쪽으로 당기는 세기 (한 반복에 t 의 몇 배)
TOL_ANALYTIC = 0.0005         # 해석 IK 해의 MuJoCo FK 검증 (손목 중앙 0.5 mm)
A3 = ("shoulder_roll", "upper_arm_roll", "elbow")   # 해석 IK 가 푸는 관절 (shoulder_yaw 고정, 손목 3 관절 지금 값)
GRASP_TILT_TOL = 3.0          # 집을 수 있음: 손 축을 수직 아래 이 각(°) 안으로 돌릴 수 있어야 (3° = 집는 점이 옆으로 약 1.3 cm 안)


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
        self._init_wrist()
        self.analytic = None

    def _init_wrist(self):
        """손목 중앙 = 손목 굽힘(wrist_yaw) 축과 손목 비틀기(wrist_pitch) 축의 교점. arm.xml 에서 두 축은 직교하고 만난다
        (wrist_yaw 관절 기준점에서 그 축 따라 20 mm). 그 점은 Wrist_yaw 몸통에 고정 → 몸통 좌표 오프셋으로 둔다
        (앞 4 관절만으로 정해지고 wrist_pitch·wrist_yaw·hand_roll 에는 안 움직인다)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        wp, wy = m.joint("wrist_pitch").id, m.joint("wrist_yaw").id
        self.wbody = int(m.jnt_bodyid[wy])
        d.qpos[self.qadr] = np.asarray(self.E.HOME_ARM_QPOS, dtype=float)
        mj.mj_kinematics(m, d)
        p1, a1, p2, a2 = d.xanchor[wp], d.xaxis[wp], d.xanchor[wy], d.xaxis[wy]
        # 두 직선의 가장 가까운 점(wrist_yaw 축 위): w = p1 - p2 (2026-11-20 고침: 예전엔 p2 - p1 이라 부호가 뒤집혀
        # 교점의 반대쪽 40 mm — wrist_pitch 축에서 40 mm 떨어진 점 — 을 썼다. 높이(손 축 방향)는 같고 옆으로만 달랐다)
        w, bb = p1 - p2, float(a1 @ a2)
        t2 = (float(a1 @ a1) * float(a2 @ w) - bb * float(a1 @ w)) / (float(a1 @ a1) * float(a2 @ a2) - bb * bb)
        x = p2 + t2 * a2
        self.wrist_off = d.xmat[self.wbody].reshape(3, 3).T @ (x - d.xpos[self.wbody])
        n = np.cross(a1, a2)
        self.wrist_axes_gap_mm = float(abs(w @ n) / np.linalg.norm(n) * 1000)
        # 손 좌표(site)에서 손목 중앙·벌린 손가락 가장 낮은 점의 손 축(site z) 성분 — 손이 수직이면 높이 차가 이것
        rs = d.site_xmat[self.site].reshape(3, 3)
        self.wrist_above_site = float((rs.T @ (x - d.site_xpos[self.site]))[2])
        d.qpos[self.fadr] = self.open_fingers
        mj.mj_kinematics(m, d)
        low = 9.0
        for g in self.finger_geoms:
            c, R, h = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        low = min(low, float((rs.T @ (c + R @ (h * np.array([sx, sy, sz])) - d.site_xpos[self.site]))[2]))
        self.finger_below_site = low   # 음수 (손 축 따라 site 아래)

    def wrist_pos(self):
        """지금 self.d 의 손목 중앙(세계, sim)."""
        return self.d.xpos[self.wbody] + self.d.xmat[self.wbody].reshape(3, 3) @ self.wrist_off

    def ref_info(self):
        """화면용: 손이 수직일 때 손목 중앙이 집는 점(site)보다 몇 m 위, 벌린 손가락 끝이 책상에 닿는 손목 높이."""
        return {"wrist_above_grip_vertical_m": round(self.wrist_above_site, 4),
                "wrist_min_vertical_m": round(self.wrist_above_site - self.finger_below_site, 4),
                "wrist_axes_gap_mm": round(self.wrist_axes_gap_mm, 2)}

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

    def solve_free_tilt(self, seed, point, turn_deg, flip, tilt_max_deg, its=400, tilt_init_deg=None, warm=120, pull=TILT_PULL, home_pull=0.02, polish=40,
                        ref="grip", tilt_fix_deg=None):
        """auto: 손이 쥐는 점(point, sim) + 손 x 축(손가락이 닫히는 방향) = rz(turn)·(±x) 만 맞추고, x 축 둘레 기울기 t 는
        |t| <= tilt_max 안에서 자유(넘으면 그 한계에 묶음). 남는 자유도로 t -> 0, 관절 -> HOME.
        ref = "wrist" 면 위치 과제가 쥐는 점 대신 손목 중앙(point = 손목 중앙 목표). tilt_fix_deg 가 있으면 기울기를 그 값에 묶은
        6D 풀이(수직 = 0, grasp = HAND_TILT).
        돌려줌 (q, 기준점 오차 m, 손 방향 오차 rad(x 축 + 한계 넘은 만큼), t rad)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        dz = float(self.E.GRASP_DZ) if tilt_fix_deg is None else 0.0
        wrist = ref == "wrist"
        fixed_t = tilt_fix_deg is not None
        tmax = math.radians(tilt_max_deg)
        a = math.radians(turn_deg)
        axis = np.array([math.cos(a), math.sin(a), 0.0])      # 기울기 축(세계) = rz·e_x
        dof = m.jnt_dofadr[self.jid]
        lim = m.jnt_range[self.jid]
        home = np.asarray(self.E.HOME_ARM_QPOS, dtype=float)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        jpw, jrw = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        damp = 1e-3 * np.eye(6)
        d.qpos[self.qadr] = seed
        d.qvel[:] = 0

        def state(k=its):
            mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
            R = d.site_xmat[self.site].reshape(3, 3).copy()
            t = self.tilt_of(R, turn_deg, flip)
            tc = (math.radians(tilt_fix_deg) if fixed_t else max(-tmax, min(tmax, t)) if tilt_init_deg is None or k >= warm
                  else math.radians(tilt_init_deg))
            ep = point - (self.wrist_pos() if wrist else d.site_xpos[self.site] - dz * R[:, 2])
            er = self.rotation_error(R, self.hand_rot("auto", turn_deg, flip, math.degrees(tc)))
            return R, t, ep, er

        t_prev, settle = None, 0
        for k in range(its):
            R, t, ep, er = state(k)
            held = fixed_t or (tilt_init_deg is not None and k < warm)   # 처음 warm 번은 기울기를 tilt_init 에 묶고 시작(가지 고르기)
            pe, re_ = np.linalg.norm(ep), np.linalg.norm(er)
            settle = settle + 1 if ((not held or fixed_t) and pe <= 0.0005 and re_ <= 0.004 and t_prev is not None
                                    and abs(t - t_prev) < 2e-4) else 0
            if settle >= 15:                                   # 맞았고 기울기도 더 안 움직임
                break
            t_prev = t
            mj.mj_jacSite(m, d, jp, jr, self.site)
            Jp, Jr = jp[:, dof], jr[:, dof]
            z = R[:, 2]
            sk = np.array([[0.0, -z[2], z[1]], [z[2], 0.0, -z[0]], [-z[1], z[0], 0.0]])
            Jg = Jp + dz * sk @ Jr                             # 쥐는 점(site - dz·z)의 자코비안
            if wrist:
                mj.mj_jac(m, d, jpw, jrw, self.wrist_pos(), self.wbody)
                Jg = jpw[:, dof]
            P = np.eye(3) if held or abs(t) > tmax else np.eye(3) - np.outer(axis, axis)   # 한계 안이면 축 둘레 회전은 자유
            J0 = np.vstack((Jg, 0.25 * P @ Jr))
            e = np.concatenate((ep, 0.25 * P @ er))
            q = d.qpos[self.qadr].copy()
            # 남는 자유도 당기기(기울기 -> 0, HOME 쪽)는 주 과제가 거의 맞았을 때만, 마지막 polish 번은 안 함
            sec = np.zeros(len(q))
            if k < its - polish and pe < 0.002 and re_ < 0.02 and (not held or (fixed_t and home_pull > 0)):
                jt = axis @ Jr                                 # dt/dq
                sec = home_pull * (home - q) + (0.0 if held else 1.0) * jt * (-pull * t) / (jt @ jt + 1e-6)
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

    def position_only(self, seeds, target, its=300, ref="grip"):
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
                e = target - (self.wrist_pos() if ref == "wrist" else self.d.site_xpos[self.site])
                if np.linalg.norm(e) < 5e-4:
                    break
                if ref == "wrist":
                    mj.mj_jac(self.m, self.d, jp, jr, self.wrist_pos(), self.wbody)
                else:
                    mj.mj_jacSite(self.m, self.d, jp, jr, self.site)
                J = jp[:, dof]
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), e)
                self.d.qpos[self.qadr] = np.clip(self.d.qpos[self.qadr] + 0.5 * dq, lim[:, 0], lim[:, 1])
            mj.mj_kinematics(self.m, self.d)
            best = min(best, float(np.linalg.norm(target - (self.wrist_pos() if ref == "wrist" else self.d.site_xpos[self.site]))))
        return best

    def seeds(self, current, few=False):
        np, E = self.np, self.E
        home = np.asarray(E.HOME_ARM_QPOS, dtype=float)
        out = ([current] if current is not None else []) + [home]
        if few:
            return out + [np.zeros(len(self.arm))]
        for hr in (-60.0, 0.0, 60.0):
            s = home.copy(); s[self.hand_roll] = math.radians(hr); out.append(s)
        out.append(np.zeros(len(self.arm)))
        return out

    WRIST3 = ("wrist_pitch", "wrist_yaw", "hand_roll")

    def use_analytic(self, mode, ref, lim):
        """손 각도 무시 + 손목 중앙 기준 + shoulder_yaw 고정(범위 [v, v]) + 나머지 3 관절은 움직임 → 해석 IK.
        JAX_IK_NUMERIC=1 이면 예전 수치 방식(비교용)."""
        if mode != "position" or os.environ.get("JAX_IK_NUMERIC") == "1":
            return False
        w = lim[:, 1] - lim[:, 0]
        return w[self.arm.index("shoulder_yaw")] < 1e-9 and all(w[self.arm.index(n)] > 1e-9 for n in A3)

    def analytic_position(self, point, hold, lim):
        """analytic_ik.solve3 로 손목 중앙 = point 인 (shoulder_roll, upper_arm_roll, elbow) 해를 모두. 기하는 지금 모델에서
        (shoulder_yaw = 고정값, 손목 3 관절 = hold). 관절 범위 안(±360° 도 범위 안이면 별개 해) + MuJoCo FK 오차 < 0.5 mm 만.
        돌려줌 (q 목록, 범위 거르기 전 해 수)."""
        import itertools
        from analytic_ik import solve3
        np, mj, m, d = self.np, self.mj, self.m, self.d
        idx = [self.arm.index(n) for n in A3]
        q0 = np.array(hold, dtype=float)
        q0[idx] = 0.0
        q0[self.arm.index("shoulder_yaw")] = lim[self.arm.index("shoulder_yaw"), 0]
        d.qpos[self.qadr] = q0
        mj.mj_kinematics(m, d)
        g = []
        for n in A3:
            j = m.joint(n).id
            g += [d.xanchor[j].copy(), d.xaxis[j].copy() / np.linalg.norm(d.xaxis[j])]
        g.append(self.wrist_pos().copy())
        raw = solve3(tuple(g), np.asarray(point, dtype=float))
        out = []
        for q3, _ in raw:
            opts = []
            for k, i in enumerate(idx):
                opts.append([v for v in (q3[k] - 2 * math.pi, q3[k], q3[k] + 2 * math.pi) if lim[i, 0] - 1e-9 <= v <= lim[i, 1] + 1e-9])
            for combo in itertools.product(*opts):
                q = q0.copy()
                q[idx] = np.clip(combo, lim[idx, 0], lim[idx, 1])
                d.qpos[self.qadr] = q
                mj.mj_kinematics(m, d)
                if float(np.linalg.norm(self.wrist_pos() - point)) < TOL_ANALYTIC:
                    out.append(self.point_down(q, lim))
        return out, len(raw)

    def analytic_tip(self, point, hold, lim, mode, iters=10):
        """기준점 = 집는 점(손끝 사이)인데 역기구학은 손목 중앙으로 (사용자 2026-11-20): 손목 목표 w 를 집는 점 + 수직 위
        (손목↔집는 점 거리) 에서 시작 → 해석 IK(앞 4 관절) + 손목 2 관절로 손 최대한 아래 → 실제 집는 점 오차만큼 w 를 옮겨
        다시 (손이 살짝 기울어도 집는 점이 목표에 오게). 가지마다 따로, 매번 직전 해에 가장 가까운 해를 잇는다.
        돌려줌 (q 목록, 첫 손목 목표의 해 수)."""
        np = self.np
        dz = self.dz(mode)

        def tip(q):
            site, rot = self.fk(q)
            return site - dz * rot[:, 2]
        w0 = np.asarray(point, dtype=float) + np.array([0.0, 0.0, self.wrist_above_site + dz])
        qs, n_raw = self.analytic_position(w0, hold, lim)
        out = []
        for q in qs:
            w = w0.copy()
            for _ in range(iters):
                e = point - tip(q)
                if float(np.linalg.norm(e)) < 2e-4:
                    break
                nxt = []
                for k in (1.0, 0.5, 0.25):   # 그 손목 자리가 안 닿으면 반만 옮겨 다시
                    nxt, _ = self.analytic_position(w + k * e, hold, lim)
                    if nxt:
                        w = w + k * e
                        break
                if not nxt:
                    break
                q = min(nxt, key=lambda c: float(np.abs(c - q).max()))
            if not any(float(np.abs(q - o).max()) < 1e-4 for o in out):
                out.append(q)
        return out, n_raw

    def solve_position(self, seed, point, ref, its=300):
        """손 각도 무시: 기준점(손목 중앙 / site) 위치 3 만 맞춘다. 손목 3 관절은 seed 값 그대로(움직이지 않음),
        앞 4 관절(범위 [v, v] 로 묶인 고정 관절은 그대로)로만. 돌려줌 (q, 위치 오차 m)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        dof = m.jnt_dofadr[self.jid]
        lim = m.jnt_range[self.jid]
        move = np.array([n not in self.WRIST3 for n in self.arm]) & (lim[:, 1] - lim[:, 0] > 1e-9)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        d.qpos[self.qadr] = seed
        wrist = ref == "wrist"
        for _ in range(its):
            mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
            pos = self.wrist_pos() if wrist else d.site_xpos[self.site]
            e = point - pos
            if np.linalg.norm(e) < 2e-4:
                break
            if wrist:
                mj.mj_jac(m, d, jp, jr, pos, self.wbody)
            else:
                mj.mj_jacSite(m, d, jp, jr, self.site)
            J = jp[:, dof] * move
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), e)
            d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + 0.5 * dq, lim[:, 0], lim[:, 1])
        mj.mj_kinematics(m, d)
        pos = self.wrist_pos() if wrist else d.site_xpos[self.site]
        return d.qpos[self.qadr].copy(), float(np.linalg.norm(point - pos))

    def axis_tilt(self, rot):
        return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))

    def tilt_fix(self, mode):
        """손목 기준 풀이에서 묶을 기울기(°): vertical 0, grasp HAND_TILT, auto 는 None(자유)."""
        return None if mode == "auto" else 0.0 if mode == "vertical" else float(self.E.HAND_TILT)

    def candidates(self, mode, yaw_real, point_sim, current, ranges_deg, servo_unlimited=False, tilt_max=TILT_MAX_DEFAULT, near=None, ref="grip"):
        np, R = self.np, self.R
        lim = self.set_ranges(ranges_deg, servo_unlimited)
        y = R.fold_yaw(yaw_real + R.SIM_FROM_REAL_YAW)
        turns = sorted({round(y, 6), round(y + 90.0, 6), round(y - 90.0, 6)}, key=abs)
        out = []
        if mode == "position":
            # 손목 3 관절 = 지금 값(current, 없으면 HOME) 그대로. 시작점: 직전 보낸 해·지금·HOME·0 의 앞 4 관절
            hold = current if current is not None else np.asarray(self.E.HOME_ARM_QPOS, dtype=float)
            if self.use_analytic(mode, ref, lim):
                hold_c = np.clip(hold, lim[:, 0], lim[:, 1])
                if ref == "wrist":
                    qs, n_raw = self.analytic_position(point_sim, hold_c, lim)
                else:
                    qs, n_raw = self.analytic_tip(point_sim, hold_c, lim, mode)
                ok_q = []
                for q in qs:
                    site, rot = self.fk(q)
                    pe = float(np.linalg.norm((self.wrist_pos() if ref == "wrist" else site - self.dz(mode) * rot[:, 2]) - point_sim))
                    if pe <= TOL_POS:
                        ok_q.append(q)
                    out.append({"q": q, "pe": pe, "re": 0.0, "turn": 0.0, "flip": False,
                                "rot": rot, "tilt": self.axis_tilt(rot), "analytic": True})
                qs = ok_q
                self.analytic = {"n_raw": n_raw, "n": len(qs)}
                if qs:
                    return out, lim
                # 해가 없으면(안 닿음) 가장 가까운 자세·이유를 위해 예전 수치 풀이도 (수치가 풀면 analytic_miss = 해석 쪽 버그)
            wi = [self.arm.index(n) for n in self.WRIST3]
            for s0 in ([near] if near is not None else []) + self.seeds(current):
                s0 = np.array(s0, dtype=float); s0[wi] = hold[wi]
                q, pe = self.solve_position(np.clip(s0, lim[:, 0], lim[:, 1]), point_sim, ref)
                if ref == "wrist":
                    q = self.point_down(q, lim)
                    self.fk(q); pe = float(np.linalg.norm(self.wrist_pos() - point_sim))
                # (집는 점 기준 수치 풀이는 손목 3 관절을 지금 값 그대로 — 해석 쪽이 안 닿을 때 가장 가까운 자세 보기용)
                _, rot = self.fk(q)
                out.append({"q": q, "pe": pe, "re": 0.0, "turn": 0.0, "flip": False, "rot": rot, "tilt": self.axis_tilt(rot)})
            return out, lim
        for turn in turns:
            for flip in (False, True):
                if ref == "wrist" and mode != "auto":
                    # 손목 중앙 기준 + 기울기 고정(수직·−40): 6D 풀이를 직전 보낸 해·지금·HOME 등에서 (이어 풀기는 당김 없이)
                    tf = self.tilt_fix(mode)
                    rot = self.hand_rot(mode, turn, flip, tf)
                    starts = ([(near, 0.0)] if near is not None else []) + [(x, 0.02) for x in self.seeds(current)]
                    for s0, hp in starts:
                        s0 = np.clip(s0, lim[:, 0], lim[:, 1])
                        q, pe, re_, t = self.solve_free_tilt(s0, point_sim, turn, flip, tilt_max, ref="wrist", tilt_fix_deg=tf, home_pull=hp)
                        out.append({"q": q, "pe": pe, "re": re_, "turn": turn, "flip": flip, "rot": rot, "tilt": tf})
                    continue
                if mode == "auto":
                    if near is not None:
                        # 이어 풀기: 직전 보낸 해(없으면 지금 자세)에서 기울기를 묶지 않고·당기지 않고 그대로 이어 푼 후보(같은 가지).
                        # 아래 0/±40 묶음 시작은 처음 120 번 기울기를 억지로 바꿔 직전 가지에서 떨어뜨린다.
                        s = np.clip(near, lim[:, 0], lim[:, 1])
                        q, pe, re_, t = self.solve_free_tilt(s, point_sim, turn, flip, tilt_max, pull=0.0, home_pull=0.0, ref=ref)
                        tilt = math.degrees(t)
                        rot = self.hand_rot(mode, turn, flip, max(-tilt_max, min(tilt_max, tilt)))
                        out.append({"q": q, "pe": pe, "re": re_, "turn": turn, "flip": flip, "rot": rot, "tilt": tilt})
                    # 기울기 자유 풀이는 시작 가지에 따라 한계(±60) 쪽에 머물 때가 있어(어깨 옆 고정 시 24점 중 절반),
                    # 처음 몇 번을 기울기 0 / ±40 에 묶고 시작해 가지를 고른 뒤 풀어 준다. 시작점은 지금·HOME·0 세 개.
                    for s in self.seeds(current, few=True):
                        s = np.clip(s, lim[:, 0], lim[:, 1])
                        for t0 in AUTO_TILT_INITS:
                            q, pe, re_, t = self.solve_free_tilt(s, point_sim, turn, flip, tilt_max,
                                                                 tilt_init_deg=max(-tilt_max, min(tilt_max, t0)), ref=ref)
                            tilt = math.degrees(t)
                            rot = self.hand_rot(mode, turn, flip, max(-tilt_max, min(tilt_max, tilt)))
                            out.append({"q": q, "pe": pe, "re": re_, "turn": turn, "flip": flip, "rot": rot, "tilt": tilt})
                    continue
                for s in self.seeds(current):
                    s = np.clip(s, lim[:, 0], lim[:, 1])
                    if False:
                        pass
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
        # 기준 자세 = 지금 팔(피드백·서보 보낸 값), 빠진 관절은 직전 보낸 해(near_deg), 그것도 없으면 HOME
        cur = b.get("current_deg") or {}
        nd = b.get("near_deg") or {}
        home = np.asarray(E.HOME_ARM_QPOS, dtype=float)
        current = None
        if any(n in cur or n in nd for n in self.arm):
            current = np.array([math.radians(float(cur[n])) if n in cur else math.radians(float(nd[n])) if n in nd else home[i]
                                for i, n in enumerate(self.arm)])
        near = np.radians([float(nd[n]) for n in self.arm]) if all(n in nd for n in self.arm) else current
        z_table = p[2] + R.MAP_D   # 책상 위 높이 (m)
        base = {"mode": mode, "input": {"x": p[0], "y": p[1], "z": p[2], "yaw_deg": yaw, "z_table_m": z_table},
                "tilt_deg": float(E.HAND_TILT if mode == "grasp" else 0.0), "tilt_max_deg": tilt_max if mode == "auto" else None,
                "grasp_dz_m": self.dz(mode)}
        if z_table < 0.0:
            return {**base, "ok": False, "kind": "below_table",
                    "reason": f"책상 아래 — 점이 책상 면보다 {-z_table * 100:.1f} cm 아래 (책상 = 어깨 아래 {R.MAP_D * 100:.2f} cm)"}
        point = np.asarray(self.to_sim(p))
        t0 = time.time()
        pref = "wrist" if b.get("ref") == "wrist" else "grip"   # 기준점: 손목 중앙 / 손가락 사이(집는 점)
        self.analytic = None
        cands, lim = self.candidates(mode, yaw, point, current, ranges, tilt_max=tilt_max, near=near, ref=pref)
        an = self.analytic
        ref = current if current is not None else np.asarray(E.HOME_ARM_QPOS)
        tol_p, tol_r = TOL_POS, TOL_ROT   # env.reachable 과 같은 5 mm, 손 방향 1.5°
        good = [c for c in cands if c["pe"] <= tol_p and c["re"] <= tol_r]
        for c in cands:
            c["dist"] = float(np.abs(c["q"] - ref).max())
        out = dict(base)
        for c in good:
            self.fk(c["q"]); self.d.qpos[self.fadr] = self.open_fingers
            self.mj.mj_kinematics(self.m, self.d)
            c["low"] = self.lowest_finger_z() - self.table_z
        clean = [c for c in good if c["low"] >= -0.001]   # 손가락(벌린)이 책상에 닿는 해는 탈락 (안전)
        if clean:
            # 1순위: 지금 자세에서 최소 회전(관절 최대 이동각). 관절각은 실제로 도는 각이라 ±180 감김은 안 한다
            #   (upper_arm_roll 은 범위가 ±180 을 넘어 q 와 q±360 이 다른 자세 = 실제로 그만큼 돈다).
            # 2순위: 최소에서 TIE_DEG 안이면 손 기울기 수직에 가까운 것, 그다음 이동각 합.
            for c in clean:
                c["sum"] = float(np.abs(c["q"] - ref).sum())
            d0 = min(c["dist"] for c in clean)
            tie = [c for c in clean if c["dist"] <= d0 + math.radians(TIE_DEG)]
            best = min(tie, key=lambda c: (abs(c["tilt"]) if mode == "auto" else 0.0, c["sum"]))
            out["ok"] = True
        elif good:
            best = min(good, key=lambda c: (-c["low"], c["dist"]))
            out["ok"] = False
            out["kind"], out["reason"] = "finger_table", f"손가락이 책상에 닿음 — 풀린 해 {len(good)}개 모두 벌린 손가락이 책상에 {-best['low'] * 1000:.0f} mm 이상 파묻힘. 더 높게"
        else:
            best = min(cands, key=lambda c: (c["pe"] + 0.05 * c["re"], c["dist"]))
            self.fk(best["q"]); self.d.qpos[self.fadr] = self.open_fingers
            self.mj.mj_kinematics(self.m, self.d)
            best["low"] = self.lowest_finger_z() - self.table_z
            out["ok"] = False
        q = best["q"]
        site, rot = self.fk(q)
        grip = site - self.dz(mode) * rot[:, 2]
        wpos = self.wrist_pos().copy()
        at_lim = [self.arm[i] for i in range(len(self.arm)) if self.arm[i] not in fixed and (q[i] <= lim[i][0] + 1e-3 or q[i] >= lim[i][1] - 1e-3)]
        out.update(joints={n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, q)},
                   pos_err_mm=round(float(np.linalg.norm((wpos if pref == "wrist" else grip) - point)) * 1000, 1),
                   ang_err_deg=round(math.degrees(best["re"]), 2), ref=pref, wrist_real=[round(float(v), 4) for v in self.to_real(wpos)],
                   ref_info=self.ref_info(),
                   flip=bool(best["flip"]), hand_turn=round(float(best["turn"]), 2), tilt_deg=round(float(best["tilt"]), 1),
                   finger_low_mm=round(best["low"] * 1000, 1), at_limit=at_lim,
                   max_move_deg=round(math.degrees(best["dist"]), 1) if current is not None else None,
                   grip_real=[round(float(v), 4) for v in self.to_real(grip)],
                   site_real=[round(float(v), 4) for v in self.to_real(site)],
                   n_good=len(good), n_tried=len(cands), from_current=current is not None,
                   open_fingers_deg={n: round(math.degrees(float(v)), 2) for n, v in zip(self.fingers, self.open_fingers)})
        if an is not None:
            out.update(self.analytic_info(an, cands, ref, best))
        warn = []
        if out["ok"] and best["low"] < -0.001:
            warn.append(f"손가락이 책상에 {-best['low'] * 1000:.0f} mm 파묻힘 (벌린 손가락 기준) — 더 높게")
        if out["ok"]:
            out["reason"] = "풀림" + (" · " + " · ".join(warn) if warn else "")
            out["warn"] = warn
        elif not good:
            out["kind"], out["reason"] = self.why(mode, yaw, point, current, ranges, best, tilt_max, ref=pref)
        out["ms"] = round((time.time() - t0) * 1000)
        return out

    def analytic_info(self, an, cands, ref, best):
        """결과에 붙일 해석 IK 정보: method, n_solutions(범위 안 + FK 검증된 해), 해 목록(고른 순서: 손가락 OK → 최소 회전)."""
        np = self.np
        sol = [c for c in cands if c.get("analytic")]
        for c in sol:
            if "low" not in c:
                self.fk(c["q"]); self.d.qpos[self.fadr] = self.open_fingers
                self.mj.mj_kinematics(self.m, self.d)
                c["low"] = self.lowest_finger_z() - self.table_z
        sol.sort(key=lambda c: (not c is best, c["low"] < -0.001, float(np.abs(c["q"] - ref).max()), float(np.abs(c["q"] - ref).sum())))
        # 해석 해가 없는데 수치가 0.5 mm 안으로 맞추면 해석 쪽 버그(놓친 해). 한계에 붙어 0.5~5 mm 로 '풀림' 인 수치 해는 정상
        # (정확한 해는 범위 밖, 한계에서 가장 가까운 자세 — 그대로 수치 해를 쓴다)
        miss = an["n"] == 0 and any(c["pe"] <= TOL_ANALYTIC for c in cands)
        return {"method": "analytic", "n_solutions": an["n"], "n_raw": an["n_raw"], "analytic_miss": bool(miss),
                "solutions": [{"joints": {n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, c["q"])},
                               "max_move_deg": round(math.degrees(float(np.abs(c["q"] - ref).max())), 1),
                               "finger_low_mm": round(c["low"] * 1000, 1), "finger_ok": bool(c["low"] >= -0.001),
                               "pos_err_mm": round(c["pe"] * 1000, 3), "chosen": c is best} for c in sol]}

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
        ref = "wrist" if b.get("ref") == "wrist" else "grip"
        an = None
        if self.use_analytic(mode, ref, lim):   # 전체 풀이와 같은 해석 IK, 기준 자세 = seed(직전 해)
            if ref == "wrist":
                qs, n_raw = self.analytic_position(point, seed, lim)
            else:
                qs, n_raw = self.analytic_tip(point, seed, lim, mode)
            cands = []
            for q in qs:
                site, rot = self.fk(q)
                pe = float(np.linalg.norm((self.wrist_pos() if ref == "wrist" else site - self.dz(mode) * rot[:, 2]) - point))
                if pe > TOL_POS:
                    continue
                self.d.qpos[self.fadr] = self.open_fingers
                self.mj.mj_kinematics(self.m, self.d)
                cands.append({"q": q, "pe": pe, "low": self.lowest_finger_z() - self.table_z,
                              "dist": float(np.abs(q - seed).max()), "sum": float(np.abs(q - seed).sum())})
            clean = [c for c in cands if c["low"] >= -0.001]
            bc = None
            if clean:
                d0 = min(c["dist"] for c in clean)
                bc = min((c for c in clean if c["dist"] <= d0 + math.radians(TIE_DEG)), key=lambda c: c["sum"])
            elif cands:
                bc = min(cands, key=lambda c: (-c["low"], c["dist"]))
            if bc is not None:
                q, pe = bc["q"], bc["pe"]
            else:   # 안 닿음: 가장 가까운 자세(수치)만 보여 줌
                q, pe = self.solve_position(seed, point, ref, its=FAST_ITERS)
                if ref == "wrist":
                    q = self.point_down(q, lim)
            re_ = 0.0
            tilt = self.axis_tilt(self.fk(q)[1])
            an = {"method": "analytic", "n_solutions": len(cands), "n_raw": n_raw}
        elif mode == "position":   # 손목 기준이면 손목 2 관절 = 손이 최대한 아래 (아니면 seed 값 그대로)
            q, pe = self.solve_position(seed, point, ref, its=FAST_ITERS)
            if ref == "wrist":
                q = self.point_down(q, lim)
            re_ = 0.0
            tilt = self.axis_tilt(self.fk(q)[1])
        elif ref == "wrist" and mode != "auto":
            tilt = self.tilt_fix(mode)
            q, pe, re_, _ = self.solve_free_tilt(seed, point, turn, flip, TILT_MAX_DEFAULT, its=FAST_ITERS, home_pull=0.0, ref="wrist", tilt_fix_deg=tilt)
        elif mode == "auto":
            tilt_max = min(90.0, max(0.0, float(TILT_MAX_DEFAULT if b.get("tilt_max_deg") is None else b["tilt_max_deg"])))
            # 당김 없음(기울기 0·HOME 쪽으로 안 밈): 남는 자유도는 직전 해에 머물러 가지가 안 바뀐다
            q, pe, re_, t = self.solve_free_tilt(seed, point, turn, flip, tilt_max, its=FAST_ITERS, pull=0.0, home_pull=0.0, ref=ref)
            tilt = math.degrees(t)
        else:
            rot = self.hand_rot(mode, turn, flip)
            q, pe, re_ = self.solve_from(seed, point + self.dz(mode) * rot[:, 2], rot, its=FAST_ITERS)
            tilt = float(E.HAND_TILT if mode == "grasp" else 0.0)
        site, srot = self.fk(q)
        wpos = self.wrist_pos().copy()
        self.d.qpos[self.fadr] = self.open_fingers
        self.mj.mj_kinematics(self.m, self.d)
        low = self.lowest_finger_z() - self.table_z
        grip = site - self.dz(mode) * srot[:, 2]
        ok = pe <= TOL_POS and re_ <= TOL_ROT and low >= -0.001   # 손가락이 책상에 닿으면 탈락 (전체 풀이와 같음)
        return {**(an or {}), "ok": bool(ok), "fast": True, "mode": mode, "flip": flip, "hand_turn": round(float(turn), 2), "tilt_deg": round(tilt, 1), "fixed": fix,
                "joints": {n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, q)},
                "pos_err_mm": round(float(np.linalg.norm((wpos if ref == "wrist" else grip) - point)) * 1000, 1), "ang_err_deg": round(math.degrees(re_), 2),
                "ref": ref, "wrist_real": [round(float(v), 4) for v in self.to_real(wpos)], "grip_real": [round(float(v), 4) for v in self.to_real(grip)],
                "finger_low_mm": round(low * 1000, 1), "warn": [] if low >= -0.001 else [f"손가락이 책상에 {-low * 1000:.0f} mm 파묻힘"],
                "open_fingers_deg": {n: round(math.degrees(float(v)), 2) for n, v in zip(self.fingers, self.open_fingers)},
                "reason": "풀림 (빠른 풀이)" if ok else ("빠른 풀이: 손가락이 책상에 닿음" if pe <= TOL_POS and re_ <= TOL_ROT else "빠른 풀이가 못 따라감 — 손을 떼면 전체 풀이"), "ms": round((time.time() - t0) * 1000, 1)}

    def why(self, mode, yaw, point, current, ranges, best, tilt_max=TILT_MAX_DEFAULT, ref="grip"):
        """실패 이유를 사람 말로: 위치부터 안 닿나 → 손 비틀기 서보(±88)만 풀면 되나 → 그 손 방향이 안 되나."""
        np = self.np
        if mode == "position":
            return "no_reach", f"팔이 안 닿음 (손목 관절은 지금 값 그대로, 앞 4 관절로 가장 가까운 해 {best['pe'] * 1000:.0f} mm 차이)"
        self.set_ranges(ranges)
        reach = (self.position_only(self.seeds(current), point, ref="wrist") if ref == "wrist"
                 else self.position_only(self.seeds(current), point + self.dz(mode) * best["rot"][:, 2]))
        miss = best["pe"] * 1000
        if reach > TOL_POS:
            return "no_reach", f"팔이 안 닿음 (손 방향을 빼도 {reach * 1000:.0f} mm 모자람, 가장 가까운 해 {miss:.0f} mm 차이)"
        wide, _ = self.candidates(mode, yaw, point, current, ranges, servo_unlimited=True, tilt_max=tilt_max, ref=ref)
        ok = [c for c in wide if c["pe"] <= TOL_POS and c["re"] <= TOL_ROT]
        self.set_ranges(ranges)
        if ok:
            hr = min(ok, key=lambda c: abs(c["q"][self.hand_roll]))["q"][self.hand_roll]
            return "hand_roll", f"손 비틀기 서보 범위 밖 — 이 손 방향이면 손 비틀기 {math.degrees(hr):+.0f}° 필요 (서보 ±88°). 손 방향(yaw)을 바꿔 보기"
        tilt = ("수직(0°)" if mode == "vertical" else f"{self.E.HAND_TILT:+.0f}° 기울인" if mode == "grasp"
                else f"기울기 ±{tilt_max:.0f}° 안의")
        return "orientation", (f"그 손 방향으로는 안 됨 — 위치만은 닿지만 {tilt} 손으로는 가장 가까운 해가 "
                               f"{miss:.0f} mm · {math.degrees(best['re']):.0f}° 차이")

    def pair(self, b):
        """지도 판정(도달 맵 대신 실제 IK): 큐브(x, y, yaw)를 집는 자세와 놓을 자리(x, y)에 놓는 자세가 **같은 파지**
        (손 돌림·반 바퀴·기울기)로 둘 다 풀리나. 파지마다: 집기 = 기울기 자유 풀이(HOME 에서, 시작 기울기 0 -> -40 -> 40 중
        처음 풀리는 것), 놓기 = 집기 해에서 시작해 그 기울기 그대로 6D. 큐브 중심 높이 = 책상 위 반 변(놓을 때도 같게 봄).
        fix_deg·관절 한계는 손 위치 카드와 같다. 빠르게: 파지 6개 × 많아야 3 + 1 풀이."""
        np, R, E = self.np, self.R, self.E
        t0 = time.time()
        cube, target = b["cube"], b["target"]
        yaw = float(cube[2]) if len(cube) > 2 else 0.0
        tilt_max = min(90.0, max(0.0, float(TILT_MAX_DEFAULT if b.get("tilt_max_deg") is None else b["tilt_max_deg"])))
        ranges = dict(b.get("ranges_deg") or {})
        fix = {k: float(v) for k, v in (b.get("fix_deg") or {}).items()}
        ranges.update({j: [v, v] for j, v in fix.items()})
        lim = self.set_ranges(ranges)
        zc = -R.MAP_D + float(E.OBJECT_HALF_SIZE)
        pc = np.asarray(self.to_sim([float(cube[0]), float(cube[1]), zc]))
        pt = np.asarray(self.to_sim([float(target[0]), float(target[1]), zc]))
        y = R.fold_yaw(yaw + R.SIM_FROM_REAL_YAW)
        home = np.clip(np.asarray(E.HOME_ARM_QPOS, dtype=float), lim[:, 0], lim[:, 1])
        tries, pick_ok = [], []
        for turn in sorted({round(y, 6), round(y + 90.0, 6), round(y - 90.0, 6)}, key=abs):
            for flip in (False, True):
                pk = None
                for t_init in AUTO_TILT_INITS:
                    q, pe, re_, t = self.solve_free_tilt(home, pc, turn, flip, tilt_max, tilt_init_deg=t_init)
                    if pe <= TOL_POS and re_ <= TOL_ROT:
                        pk = (q, math.degrees(t)); break
                row = {"hand_turn": turn, "flip": flip, "pick": pk is not None}
                if pk:
                    pick_ok.append(row)
                    tilt = max(-tilt_max, min(tilt_max, pk[1]))
                    q2, pe2, re2, _ = self.solve_free_tilt(pk[0], pt, turn, flip, tilt_max, its=300, tilt_init_deg=tilt, warm=300)
                    row.update(tilt_deg=round(tilt, 1), place=bool(pe2 <= TOL_POS and re2 <= TOL_ROT),
                               place_err_mm=round(pe2 * 1000, 1), place_ang_deg=round(math.degrees(re2), 2))
                tries.append(row)
                if row.get("place"):
                    return {"ok": True, "both": True, "grasp": row, "tries": tries, "fixed": fix, "ms": round((time.time() - t0) * 1000)}
        place_alone = None
        if not pick_ok:   # 집기가 하나도 안 되면 놓을 자리만이라도 되나 (같은 풀이로)
            place_alone = False
            for turn in sorted({round(y, 6), round(y + 90.0, 6), round(y - 90.0, 6)}, key=abs):
                for flip in (False, True):
                    q, pe, re_, t = self.solve_free_tilt(home, pt, turn, flip, tilt_max, tilt_init_deg=0.0)
                    if pe <= TOL_POS and re_ <= TOL_ROT:
                        place_alone = True; break
                if place_alone:
                    break
        return {"ok": True, "both": False, "pick_any": bool(pick_ok), "place_alone": place_alone, "tries": tries,
                "fixed": fix, "ms": round((time.time() - t0) * 1000)}

    # ---- 집을 수 있는 곳 (graspable): 해석 IK 로 책상 위 (x, y) 마다
    # 정의: 큐브(5 cm)가 책상 위 (x, y) 에 있을 때, 손을 수직으로 내려 집는 점이 큐브 중심(책상 위 2.5 cm)에 오는 자세가 있나 =
    #   (1) 손목 중앙 = (x, y, 책상 위 GRASP_WRIST_Z) 를 해석 IK(shoulder_yaw 고정, 관절 한계 = calibration 보낼 수 있는 범위)로 풀 수 있고
    #   (2) 그 해 중 하나에서 손목 두 관절(wrist_pitch·wrist_yaw, 한계 안)로 손 축을 수직 아래 GRASP_TILT_TOL 안으로 돌릴 수 있다.
    #   손 비틀기(hand_roll ±88°)는 손 축 둘레라 수직에 영향 없고, 큐브는 90° 마다 같아 176° 범위면 어떤 yaw 든 맞춘다 → 검사 안 함.
    #   GRASP_WRIST_Z = 2.5 cm + 수직 손에서 손목↔집는 점 거리(ref_info, 24.35 cm) 를 0.5 cm 로 반올림 = 27 cm (IK 탭 기본·'큐브 잡는 높이' 와 같음).
    def grasp_z(self):
        return round((0.025 + self.wrist_above_site) * 200) / 200

    def grasp_setup(self, ranges, sy_deg):
        """한계(shoulder_yaw = sy 고정) + 해석 IK 기하 + 도달 상한(어깨 roll 축 기준 손목 중앙 최대 거리)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        lim = self.set_ranges({**(ranges or {}), "shoulder_yaw": [sy_deg, sy_deg]})
        q0 = np.clip(np.asarray(self.E.HOME_ARM_QPOS, dtype=float), lim[:, 0], lim[:, 1])
        for n in A3:
            q0[self.arm.index(n)] = 0.0
        q0[self.arm.index("shoulder_yaw")] = math.radians(sy_deg)
        d.qpos[self.qadr] = q0
        mj.mj_kinematics(m, d)
        g = []
        for n in A3:
            j = m.joint(n).id
            g += [d.xanchor[j].copy(), d.xaxis[j].copy() / np.linalg.norm(d.xaxis[j])]
        g.append(self.wrist_pos().copy())
        o1, _, o3, _, o4, a4, w0 = g
        from analytic_ik import rot
        rmax = max(float(np.linalg.norm(o4 + rot(a4, t, w0 - o4) - o3)) for t in np.linspace(-math.pi, math.pi, 361))
        return {"lim": lim, "q0": q0, "g": tuple(g), "reach": float(np.linalg.norm(o3 - o1)) + rmax + 0.002}

    def hand_down_tilt(self, q, lim):
        """팔 앞 4 관절 = q 일 때 wrist_pitch·wrist_yaw(한계 안)로 만들 수 있는 손 축의 수직(아래) 에서 최소 기울기 (°).
        손 축 = site z (PALM_DOWN 이면 세계 +z, fk_req 와 같은 정의). q6 은 1° 격자, q5 는 닫힌 식(구간 끝·극값)."""
        np, mj, m, d = self.np, self.mj, self.m, self.d
        i5, i6 = self.arm.index("wrist_pitch"), self.arm.index("wrist_yaw")
        qq = q.copy(); qq[i5] = 0.0; qq[i6] = 0.0
        d.qpos[self.qadr] = qq
        mj.mj_kinematics(m, d)
        a5 = d.xaxis[m.joint("wrist_pitch").id].copy(); a5 /= np.linalg.norm(a5)
        a6 = d.xaxis[m.joint("wrist_yaw").id].copy(); a6 /= np.linalg.norm(a6)
        h = d.site_xmat[self.site].reshape(3, 3)[:, 2].copy()
        lo6, hi6 = float(lim[i6, 0]), float(lim[i6, 1])
        lo5, hi5 = float(lim[i5, 0]), float(lim[i5, 1])
        z = np.array([0.0, 0.0, 1.0])

        def best_of(t6):
            v = a6 * (h @ a6) * (1 - np.cos(t6)) + np.cos(t6) * h + np.sin(t6) * np.cross(a6, h)   # (n, 3)
            A = (v @ a5) * a5[2]
            B = v[:, 2] - A
            C = np.cross(a5, v)[:, 2]
            cands = [np.full(len(v), lo5), np.full(len(v), hi5)]
            ph = np.arctan2(C, B)
            for k in (-2, 0, 2):
                c = ph + k * math.pi
                cands.append(np.where((c >= lo5) & (c <= hi5), c, lo5))
            best, b5, b6 = -2.0, 0.0, 0.0
            for c in cands:
                val = A + B * np.cos(c) + C * np.sin(c)
                j = int(np.argmax(val))
                if float(val[j]) > best:
                    best, b5, b6 = float(val[j]), float(c[j]), float(t6[j, 0])
            return best, b5, b6
        # q6: 1° 격자 → 가장 좋은 칸 ±1° 를 0.01° 로 다시 (1° 격자만이면 집는 점이 수 mm 튄다, 2026-11-20 밤)
        best, b5, b6 = best_of(np.linspace(lo6, hi6, max(2, int(math.degrees(hi6 - lo6)) + 1))[:, None])
        d1 = math.radians(1.0)
        best, b5, b6 = best_of(np.linspace(max(lo6, b6 - d1), min(hi6, b6 + d1), 201)[:, None])
        self._down_q56 = (b5, b6)
        return math.degrees(math.acos(max(-1.0, min(1.0, best))))

    def point_down(self, q, lim):
        """손목 중앙 기준 위치 풀이 뒤: wrist_pitch·wrist_yaw(한계 안)를 손이 최대한 아래를 보게 (사용자 2026-11-20).
        손목 중앙은 이 두 관절에 안 움직이므로 위치는 그대로. hand_roll 은 손 축 둘레라 그대로 둔다."""
        q = q.copy()
        self.hand_down_tilt(q, lim)
        q[self.arm.index("wrist_pitch")], q[self.arm.index("wrist_yaw")] = self._down_q56
        return q

    def grasp_eval(self, S, x, y, verify):
        """(x, y) 실좌표 -> (코드, 최소 기울기 °, 해 수). 코드 2 = 집을 수 있음, 1 = 손목은 닿지만 손을 수직으로 못 내림, 0 = 안 닿음."""
        import itertools
        from analytic_ik import solve3
        np = self.np
        lim, q0, g = S["lim"], S["q0"], S["g"]
        p = np.asarray(self.to_sim([x, y, self.grasp_z() - self.R.MAP_D]))
        if float(np.linalg.norm(p - g[0])) > S["reach"]:
            return 0, None, 0
        idx = [self.arm.index(n) for n in A3]
        qs = []
        for q3, _ in solve3(g, p):
            opts = [[v for v in (q3[k] - 2 * math.pi, q3[k], q3[k] + 2 * math.pi) if lim[i, 0] - 1e-9 <= v <= lim[i, 1] + 1e-9]
                    for k, i in enumerate(idx)]
            for combo in itertools.product(*opts):
                q = q0.copy(); q[idx] = np.clip(combo, lim[idx, 0], lim[idx, 1])
                if verify:   # MuJoCo FK 로 다시 확인 (analytic_position 과 같은 0.5 mm)
                    self.d.qpos[self.qadr] = q
                    self.mj.mj_kinematics(self.m, self.d)
                    if float(np.linalg.norm(self.wrist_pos() - p)) >= TOL_ANALYTIC:
                        continue
                qs.append(q)
        if not qs:
            return 0, None, 0
        # 사용자 2026-11-20 밤: 기준점 = 집는 점(손끝 사이). 집는 점을 큐브 중심(책상 위 2.5 cm)에 두고, 손목 중앙으로 푸는
        # analytic_tip(손 최대한 아래 + 기울어진 만큼 손목을 옮겨 다시)으로 5 mm 안에 맞으면 집을 수 있음.
        tipp = np.asarray(self.to_sim([x, y, 0.025 - self.R.MAP_D]))
        best = None
        for q in self.analytic_tip(tipp, q0, lim, "position")[0]:
            site, rot = self.fk(q)
            pe = float(np.linalg.norm(site - tipp))
            if best is None or pe < best[0]:
                best = (pe, self.axis_tilt(rot))
        if best is not None and best[0] <= TOL_POS:
            return 2, best[1], len(qs)
        return 1, (best[1] if best else None), len(qs)

    def graspmap(self, b):
        """op graspmap: 책상 위 격자 (x0..x1, y0..y1, step) 마다 grasp_eval. rows[j] = y 한 줄의 코드 문자열 ('2'/'1'/'0')."""
        np = self.np
        t0 = time.time()
        S = self.grasp_setup(b.get("ranges_deg"), float(b.get("shoulder_yaw") or 0.0))
        st = float(b.get("step") or 0.01)
        xs = [round(float(v), 4) for v in np.arange(float(b["x0"]), float(b["x1"]) + st / 2, st)]
        ys = [round(float(v), 4) for v in np.arange(float(b["y0"]), float(b["y1"]) + st / 2, st)]
        rows, n = [], [0, 0, 0]
        for y in ys:
            r = ""
            for x in xs:
                c = self.grasp_eval(S, x, y, verify=False)[0]
                n[c] += 1
                r += str(c)
            rows.append(r)
        return {"ok": True, "xs": xs, "ys": ys, "step": st, "rows": rows, "z_table_m": self.grasp_z(),
                "counts": {"graspable": n[2], "wrist_only": n[1], "no": n[0]}, "tilt_tol_deg": GRASP_TILT_TOL,
                "shoulder_yaw": float(b.get("shoulder_yaw") or 0.0), "ms": round((time.time() - t0) * 1000)}

    def graspcheck(self, b):
        """op graspcheck: 한 점 (x, y) — 격자와 같은 판정 + MuJoCo FK 검증."""
        t0 = time.time()
        S = self.grasp_setup(b.get("ranges_deg"), float(b.get("shoulder_yaw") or 0.0))
        c, tilt, ns = self.grasp_eval(S, float(b["x"]), float(b["y"]), verify=True)
        why = {2: "집을 수 있음", 1: "손목은 닿지만 손끝(집는 점)을 큐브에 못 맞춤", 0: "팔이 안 닿음"}[c]
        return {"ok": True, "graspable": c == 2, "code": c, "reason": why, "tilt_min_deg": None if tilt is None else round(tilt, 1),
                "n_solutions": ns, "z_table_m": self.grasp_z(), "x": float(b["x"]), "y": float(b["y"]),
                "shoulder_yaw": float(b.get("shoulder_yaw") or 0.0), "ms": round((time.time() - t0) * 1000, 1)}

    def fk_req(self, b):
        np = self.np
        js = b.get("joints_deg") or {}
        q = np.radians([float(js.get(n, 0.0)) for n in self.arm])
        mode = b.get("mode") or "grasp"
        site, rot = self.fk(q)
        grip = site - self.dz(mode) * rot[:, 2]
        # 손 축(site z) 이 수직에서 얼마나 기울었나: PALM_DOWN(기울기 0) 이면 site z = 세계 +z
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
        return {"ok": True, "grip_real": [round(float(v), 4) for v in self.to_real(grip)], "ref_info": self.ref_info(),
                "wrist_real": [round(float(v), 4) for v in self.to_real(self.wrist_pos())],
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
            r = s.solve(b) if op == "solve" else s.fast(b) if op == "fast" else s.fk_req(b) if op == "fk" else s.pair(b) if op == "pair" else s.graspmap(b) if op == "graspmap" else s.graspcheck(b) if op == "graspcheck" else {"ok": False, "reason": f"모르는 op {op!r}"}
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
                                        "tilt_max_deg": tilt_max, "ref": b.get("ref")}, timeout=10.0)
        except Exception as e:
            return {"ok": False, "fast": True, "reason": str(e), "kind": "error"}
    near = b.get("near_deg")   # 직전 보낸 해: 지금 팔에서 빠진 관절(서보 안 보냄)을 채우고, 이어 풀기 시작점
    try:
        near = {str(k): float(v) for k, v in near.items() if isinstance(v, (int, float)) and math.isfinite(v)} if isinstance(near, dict) else {}
    except (TypeError, ValueError):
        near = {}
    try:
        return SOLVER.call(ik_dir, {"op": "solve", "x": x, "y": y, "z": z, "yaw_deg": yaw, "mode": mode, "tilt_max_deg": tilt_max,
                                    "ranges_deg": ranges_deg, "current_deg": current_deg, "fix_deg": fix, "near_deg": near,
                                    "ref": b.get("ref")})
    except Exception as e:
        return {"ok": False, "reason": str(e), "kind": "error"}


def pair(ik_dir, body, ranges_deg):
    """POST /api/ik/pair {cube: [x, y, yaw], target: [x, y], fix_deg?, tilt_max_deg?} -> 같은 파지로 집기·놓기 둘 다 되나 (실제 IK)."""
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음"}
    b = body if isinstance(body, dict) else {}
    try:
        cube = [float(v) for v in b["cube"]][:3]
        target = [float(v) for v in b["target"]][:2]
        fix = {str(k): float(v) for k, v in (b.get("fix_deg") or {}).items()}
        tm = float(b.get("tilt_max_deg") if b.get("tilt_max_deg") is not None else TILT_MAX_DEFAULT)
        if len(cube) < 2 or len(target) < 2 or not all(math.isfinite(v) and abs(v) <= 2.0 for v in cube[:2] + target):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "reason": "cube [x, y, yaw], target [x, y] (m, °)"}
    try:
        return SOLVER.call(ik_dir, {"op": "pair", "cube": cube, "target": target, "fix_deg": fix, "tilt_max_deg": tm,
                                    "ranges_deg": ranges_deg}, timeout=30.0)
    except Exception as e:
        return {"ok": False, "reason": str(e)}


def graspmap(ik_dir, ranges_deg, shoulder_yaw=0.0, box=(-0.28, 0.38, -0.45, 0.075), step=0.01):
    """책상 위 집을 수 있는 곳 지도 (해석 IK, 서버가 메모리에 캐시)."""
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음"}
    try:
        return SOLVER.call(ik_dir, {"op": "graspmap", "ranges_deg": ranges_deg, "shoulder_yaw": shoulder_yaw, "step": step,
                                    "x0": box[0], "x1": box[1], "y0": box[2], "y1": box[3]}, timeout=120.0)
    except Exception as e:
        return {"ok": False, "reason": str(e)}


def graspcheck(ik_dir, ranges_deg, x, y, shoulder_yaw=0.0):
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음"}
    try:
        return SOLVER.call(ik_dir, {"op": "graspcheck", "ranges_deg": ranges_deg, "shoulder_yaw": shoulder_yaw, "x": x, "y": y}, timeout=20.0)
    except Exception as e:
        return {"ok": False, "reason": str(e)}


def fk(ik_dir, joints_deg, mode="grasp"):
    if ik_dir is None:
        return {"ok": False, "reason": "team-jax/IK 를 못 찾음"}
    try:
        return SOLVER.call(ik_dir, {"op": "fk", "joints_deg": joints_deg, "mode": mode})
    except Exception as e:
        return {"ok": False, "reason": str(e)}


if __name__ == "__main__" and len(sys.argv) >= 3 and sys.argv[1] == "--worker":
    _worker(sys.argv[2])
