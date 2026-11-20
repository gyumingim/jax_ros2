"""역기구학 탭 '손 위치로 보내기': 손이 쥐는 점(x, y, z 실좌표) + 손 방향 -> 팔 관절각 (team-jax/IK 의 env·ik 재사용).

MuJoCo 모델은 서버 프로세스가 아니라 자식 프로세스(이 파일 --worker) 하나에서 한 번 읽고 계속 쓴다
(JAX_SETUP=real 은 humanoid_grasp_rl.env 를 처음 import 할 때 읽혀서, 서버 안에서 섞이지 않게). JSON 한 줄씩 주고받는다.

손 방향 (env.py 와 같은 정의):
  grasp    = 실물 셋업의 파지 자세: 손가락 아래 + 손가락 사이 축으로 HAND_TILT(-40°) 기울임 (PALM_DOWN),
             손 축 따라 GRASP_DZ(21.5 mm) 위가 파지 중심(end_effector) — env.pick_pose 와 같다. 큐브 잡는 자세 그대로.
  vertical = 손가락 수직 아래(기울기 0, 시뮬 셋업의 손), GRASP_DZ 0. 실물 책상(어깨 아래 477.5 mm)에선 대부분 안 닿는다.
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

    def hand_rot(self, mode, turn_deg, flip):
        np, E = self.np, self.E
        t = math.radians(E.HAND_TILT if mode == "grasp" else 0.0)
        palm = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
        if flip:
            palm = palm @ E.HALF_TURN
        a = math.radians(turn_deg)
        rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        return rz @ palm

    def dz(self, mode):
        return float(self.E.GRASP_DZ) if mode == "grasp" else 0.0

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

    def candidates(self, mode, yaw_real, point_sim, current, ranges_deg, servo_unlimited=False):
        np, R = self.np, self.R
        lim = self.set_ranges(ranges_deg, servo_unlimited)
        y = R.fold_yaw(yaw_real + R.SIM_FROM_REAL_YAW)
        turns = sorted({round(y, 6), round(y + 90.0, 6), round(y - 90.0, 6)}, key=abs)
        out = []
        for turn in turns:
            for flip in (False, True):
                rot = self.hand_rot(mode, turn, flip)
                target = point_sim + self.dz(mode) * rot[:, 2]
                for s in self.seeds(current):
                    s = np.clip(s, lim[:, 0], lim[:, 1])
                    q, pe, re_ = self.solve_from(s, target, rot)
                    out.append({"q": q, "pe": pe, "re": re_, "turn": turn, "flip": flip, "target": target, "rot": rot})
        return out, lim

    def solve(self, b):
        np, R, E = self.np, self.R, self.E
        mode = b.get("mode") or "grasp"
        if mode not in ("grasp", "vertical"):
            return {"ok": False, "reason": f"mode 는 grasp | vertical ({mode!r})"}
        p = [float(b[k]) for k in ("x", "y", "z")]
        yaw = float(b.get("yaw_deg") or 0.0)
        ranges = b.get("ranges_deg") or {}
        cur = b.get("current_deg") or {}
        current = None
        if all(n in cur for n in self.arm):
            current = np.radians([float(cur[n]) for n in self.arm])
        z_table = p[2] + R.MAP_D   # 책상 위 높이 (m)
        base = {"mode": mode, "input": {"x": p[0], "y": p[1], "z": p[2], "yaw_deg": yaw, "z_table_m": z_table},
                "tilt_deg": float(E.HAND_TILT if mode == "grasp" else 0.0), "grasp_dz_m": self.dz(mode)}
        if z_table < 0.0:
            return {**base, "ok": False, "kind": "below_table",
                    "reason": f"책상 아래 — 점이 책상 면보다 {-z_table * 100:.1f} cm 아래 (책상 = 어깨 아래 {R.MAP_D * 100:.2f} cm)"}
        point = np.asarray(self.to_sim(p))
        t0 = time.time()
        cands, lim = self.candidates(mode, yaw, point, current, ranges)
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
            best = min(good, key=lambda c: (c["low"] < -0.001, c["dist"]))
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
        at_lim = [self.arm[i] for i in range(len(self.arm)) if q[i] <= lim[i][0] + 1e-3 or q[i] >= lim[i][1] - 1e-3]
        out.update(joints={n: round(math.degrees(float(v)), 2) for n, v in zip(self.arm, q)},
                   pos_err_mm=round(float(np.linalg.norm(grip - point)) * 1000, 1),
                   ang_err_deg=round(math.degrees(best["re"]), 2),
                   flip=bool(best["flip"]), hand_turn=round(float(best["turn"]), 2),
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
            out["kind"], out["reason"] = self.why(mode, yaw, point, current, ranges, best)
        out["ms"] = round((time.time() - t0) * 1000)
        return out

    def why(self, mode, yaw, point, current, ranges, best):
        """실패 이유를 사람 말로: 위치부터 안 닿나 → 손 비틀기 서보(±88)만 풀면 되나 → 그 손 방향이 안 되나."""
        np = self.np
        self.set_ranges(ranges)
        reach = self.position_only(self.seeds(current), point + self.dz(mode) * best["rot"][:, 2])
        miss = best["pe"] * 1000
        if reach > TOL_POS:
            return "no_reach", f"팔이 안 닿음 (손 방향을 빼도 {reach * 1000:.0f} mm 모자람, 가장 가까운 해 {miss:.0f} mm 차이)"
        wide, _ = self.candidates(mode, yaw, point, current, ranges, servo_unlimited=True)
        ok = [c for c in wide if c["pe"] <= TOL_POS and c["re"] <= TOL_ROT]
        self.set_ranges(ranges)
        if ok:
            hr = min(ok, key=lambda c: abs(c["q"][self.hand_roll]))["q"][self.hand_roll]
            return "hand_roll", f"손 비틀기 서보 범위 밖 — 이 손 방향이면 손 비틀기 {math.degrees(hr):+.0f}° 필요 (서보 ±88°). 손 방향(yaw)을 바꿔 보기"
        tilt = "수직(0°)" if mode == "vertical" else f"{self.E.HAND_TILT:+.0f}° 기울인"
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
            r = s.solve(b) if op == "solve" else s.fk_req(b) if op == "fk" else {"ok": False, "reason": f"모르는 op {op!r}"}
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
    """POST /api/ik/hand. body: {x, y, z 실좌표 m | z_table_cm, yaw_deg, mode}."""
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
        if not all(math.isfinite(v) for v in (x, y, z, yaw)) or max(abs(x), abs(y), abs(z)) > 2.0:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "reason": "x, y (m), z (m) 또는 z_table_cm, yaw_deg 숫자 (|x|,|y|,|z| ≤ 2 m)"}
    mode = b.get("mode") or "grasp"
    try:
        return SOLVER.call(ik_dir, {"op": "solve", "x": x, "y": y, "z": z, "yaw_deg": yaw, "mode": mode,
                                    "ranges_deg": ranges_deg, "current_deg": current_deg})
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
