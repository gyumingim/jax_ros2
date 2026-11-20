"""대시보드 집어 옮기기 계획 (해석 IK) — '집을 수 있는 곳' 지도와 같은 IK 로 (2026-11-20).

    python dashboard/pick_plan.py --cube X Y YAW --target X Y --max-speed 30 --save logs/pick_dry_traj.json --dry-run

[계획 돌려 보기] 의 기본. 팀 계획(arm_control/pick_place.py: team-jax/IK motion1 MuJoCo 물리 시뮬, 자기 수치 IK)은
지도(hand_ik.grasp_eval, 해석 IK)와 판정이 달라 초록 칸에서도 실패했다. 여기는 지도와 같은 hand_ik._Solver 로 푼다:
  - 집는 점(end_effector site, 손끝 사이) = 큐브 중심(책상 위 2.5 cm), shoulder_yaw 0 고정, 손목 중앙으로 해석 IK
    + 손목 2 관절로 손 최대한 아래 + 집는 점 오차만큼 손목을 옮겨 다시 (analytic_tip) — 지도와 같은 풀이.
  - 손 돌림: 손가락이 닫히는 축(site x)을 위에서 본 각(손 yaw)을 FK 로 재고, hand_roll(±88°)로 큐브 yaw 에 맞춘다
    (큐브는 90° 마다 같으니 yaw mod 90). 손목 관절이 이미 손을 돌린 만큼은 FK 로 재서 빼므로 자동으로 들어간다.
  - 경유점: HOME(열림) → 큐브 위(집는 점 9 cm 위, 안 되면 1 cm 씩 낮춤) → 내려감(큐브 중심) → 손가락 닫기 → 들기(큐브 위)
    → 놓을 자리 위 → 내려감(큐브 바닥 = 책상) → 손가락 열기 → 들기 → HOME.
  - 가지 고르기: 경유점마다 해석 IK 해(가지) × hand_roll 후보 중, 이웃 경유점 사이 관절 최대 이동각의 합이 가장 작은 줄
    (손 IK 의 '최소 회전' 규칙을 경로 전체로).
  - 경유점 사이는 관절 공간 보간(부드러운 시작·끝), 프레임 간격 dt 0.02 s, 가장 빠른 관절이 max_speed(°/s) 를 안 넘게.
  - 프레임마다 손가락 충돌 상자 가장 낮은 점이 책상 위인지 (lowest_finger_z) 검사, 아니면 상태 'finger below table'.
물리·쥐기 시뮬은 없다: 큐브가 실제로 잡히고 미끄러지지 않는지는 이 계획이 모른다 (팀 시뮬 계획은 '팀 시뮬 계획 (물리)').

출력 JSON 은 pick_place --save 와 같은 꼴 (joint_names 팔 7 + 손가락 6, dt, positions rad, status ...) + planner/waypoints.
종료코드: pick_place 와 같다 — 0 통과, 1 거절(검사), 3 팔이 못 닿음.
"""
import argparse
import json
import math
import os
import sys
import time
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", message=".*reachmap.*")   # team env 가 import 때 내는 도달 맵 경고 (이 계획은 도달 맵을 안 씀)

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path[:0] = [str(HERE), str(REPO / "arm_control"), str(REPO / "stm_bridge")]   # pick_place 로컬 실행과 같게

ABOVE_CM = (9.0, 7.0, 5.0, 3.0)   # 큐브·놓을 자리 위 경유점: 집는 점을 이만큼 (4.25 cm 미만이면 벌린 손가락 끝이 큐브 윗면보다 낮다:
                                  # 그때는 손가락이 큐브 양옆으로 곧게 내려가야 한다 — 손가락↔큐브 검사가 본다)
APPROACH_TILTS = (0, 15, 30, 45, 60)   # 그 경유점 방향: 수직에서 기울기(°) × 방위 45° 마다
APPROACH_AZ = 45
STEP_TRANSIT = 0.01         # 옮겨 가는 길: 손목 중앙 1 cm 마다 IK
STEP_DESCENT = 0.01         # 내려가기·들기: 집는 점 1 cm 마다 IK (그 사이는 관절 보간 — 옆으로 1 mm 안팎)
FALLBACK_COST = 10.0        # 길을 못 찾아 관절 직선으로 가는 구간의 벌점 (rad, 고르기에서 뒤로)
TRANSIT_LIFTS = (0.0, 0.03, 0.06, None)   # 옮겨 가는 길의 높이: max(두 끝, 하한) + 이만큼 (None = 곧은 선)
CLEAR_CM = 3.0               # 옮겨 갈 때 손가락 끝이 큐브 윗면보다 이만큼 위 (손목 높이 하한)
JUMP_DEG = 15.0             # 그 사이 팔 관절이 이보다 크게 튀면 가지가 끊긴 것 (그 길 버림)
SAME_BRANCH_DEG = 2.0       # 길 끝 해가 목표 자세와 같은 가지인가 (팔 3 관절 차이)
MAX_TRIES = 15               # 손가락이 큐브에 닿지 않는 접근을 찾을 때 볼 후보 수 (집기·놓기 각각)
DT = 0.02                    # 프레임 간격 (팀 계획과 같은 50 Hz)
GRIP_S = 0.8                 # 손가락 닫기·열기 시간 (팀: 한 스텝 0.025 씩 → 40 스텝 = 0.8 s)
GRIP_HOLD_S = 0.2            # 닫은/연 뒤 기다림
YAW_TOL = 2.0                # 손 yaw 가 큐브 면에서 이 각(°) 안
TIP_TOL = 0.005              # 집는 점 오차 5 mm (지도와 같은 TOL_POS)
SMOOTH_PEAK = 1.5            # 3차 smoothstep 의 최대 속도 / 평균 속도
TABLE_MARGIN = 0.008         # 손가락 충돌 상자·들고 가는 큐브 바닥이 책상 면보다 이만큼(m) 위여야 (실물 책상 높이 MAP_D 0.4775 는 가정 —
                             # 실측 전엔 오차가 이 여유를 먹는다. README). 손이 수직이면 벌린 손가락 9.7 mm·쥔 손가락 8.5 mm 위라 통과,
                             # 12° 기울면 4 mm 아래쪽 → 그런 곳은 '집을 수 없음' (지도도 같은 판정: grasp_clear)
REASON = {1: "손목은 닿지만 손끝(집는 점)을 큐브에 못 맞춤", 0: "팔이 안 닿음"}   # hand_ik.graspcheck 와 같은 말


def _cmd_ranges(cal_file):
    """calibration.json -> {관절: [lo, hi] sim °} 보낼 수 있는 범위 (server._cmd_range 와 같은 계산)."""
    d = json.loads(Path(cal_file).read_text(encoding="utf-8"))
    out = {}
    for j in d.get("joints") or []:
        try:
            lo, hi = (float(v) for v in j["sim_range_deg"])
            if j["motor"] == "servo":
                mlo, mhi = 0.0, 180.0
            else:
                mlo, mhi = j["zero_deg"] - j["limit_deg"], j["zero_deg"] + j["limit_deg"]
            a, b = sorted(((mlo - j["zero_deg"]) / (j["sign"] * j["scale"]), (mhi - j["zero_deg"]) / (j["sign"] * j["scale"])))
            r = [max(lo, a), min(hi, b)]
            if r[0] <= r[1]:
                out[j["joint"]] = r
        except Exception:
            pass
    return out


def _find_ik(path):
    for c in (path, os.environ.get("JAX_IK_DIR"), REPO.parent / "IK", Path.home() / "IK"):
        if c and (Path(c) / "motion1" / "src" / "humanoid_grasp_rl").is_dir():
            return Path(c).resolve()
    raise FileNotFoundError("team-jax/IK 를 못 찾음 (--ik 또는 JAX_IK_DIR)")


class Planner:
    def __init__(self, ik_dir, ranges):
        import hand_ik
        self.s = s = hand_ik._Solver(str(ik_dir))
        self.np = np = s.np
        self.S = s.grasp_setup(ranges, 0.0)          # 지도와 같은 한계 (shoulder_yaw 0 고정)
        self.lim = self.S["lim"]
        self.hr = s.arm.index("hand_roll")
        self.open = np.asarray(s.env.finger_targets(0.0), dtype=float)
        self.closed = np.asarray(s.env.finger_targets(1.0), dtype=float)   # 팀 계획이 들고 갈 때 쓰는 값 (grip 1.0)
        self.home = np.asarray(s.E.HOME_ARM_QPOS, dtype=float)
        self.cube_grip = float(getattr(s.E, "CUBE_GRIP", 1.0))
        # 옮겨 갈 때 손목 높이 하한: 손을 수직으로 내려도 벌린 손가락 끝이 큐브 윗면(5 cm) + CLEAR_CM 위
        self.z_clear = s.table_z + 0.05 + CLEAR_CM / 100 + (s.wrist_above_site - s.finger_below_site)
        self.obj_body = int(s.env.object_body_ids[0])
        self.cube_geom = [g for g in range(s.m.ngeom) if s.m.geom_bodyid[g] == self.obj_body][0]
        self.fgeoms = set(s.finger_geoms)   # 충돌하는 손 상자는 손가락 6 개뿐 (손바닥·서보 메시는 충돌 없음)

    # ---- 손 yaw (위에서 본 손가락 닫히는 축 = site x 의 각, sim °)
    def hand_yaw(self, q):
        _, rot = self.s.fk(q)
        return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))

    def grip(self, q):
        return self.s.fk(q)[0]

    def candidates(self, p, yaw_sim):
        """집는 점 = p (sim) 이고 손 yaw ≡ yaw_sim (mod 90) 인 해 모두: 해석 IK 가지 × hand_roll 후보."""
        np, s, lim, hr = self.np, self.s, self.lim, self.hr
        hold = self.S["q0"].copy()
        hold[hr] = 0.0
        base, _ = s.analytic_tip(p, hold, lim, "position")
        out = []
        for q in base:
            phi = self.hand_yaw(q)
            for k in range(-3, 4):
                r = q[hr] + math.radians(yaw_sim - phi + 90 * k)
                if not (lim[hr, 0] - 1e-9 <= r <= lim[hr, 1] + 1e-9):
                    continue
                qq = q.copy()
                for _ in range(3):    # hand_roll 을 바꿔 다시 풀고(같은 가지), 손 yaw 를 다시 재서 맞춘다
                    h = hold.copy(); h[hr] = float(np.clip(r, lim[hr, 0], lim[hr, 1]))
                    sols, _ = s.analytic_tip(p, h, lim, "position")
                    if not sols:
                        qq = None
                        break
                    qq = min(sols, key=lambda c: float(np.abs(c - qq).max()))
                    e = ((yaw_sim - self.hand_yaw(qq) + 45) % 90) - 45
                    if abs(e) < 0.05:
                        break
                    r = qq[hr] + math.radians(e)
                if qq is None:
                    continue
                e = ((yaw_sim - self.hand_yaw(qq) + 45) % 90) - 45
                tip = float(np.linalg.norm(self.grip(qq) - p))
                inside = bool(np.all(qq >= lim[:, 0] - 1e-6) and np.all(qq <= lim[:, 1] + 1e-6))
                if abs(e) <= YAW_TOL and tip <= TIP_TOL and inside and not any(float(np.abs(qq - o).max()) < 1e-3 for o in out):
                    out.append(qq)
        return out

    def approaches(self, p, yaw_sim, axis, fam=True):
        """큐브 위 경유점 후보 (h cm, 방향 v, 해 목록) 를 좋은 순서로 (게으르게). 집는 점 = p + h·v.
        순서 점수 = v 와 잡는 자세 손 축(axis) 사이 각(°) + 5 × (9 − h cm): 손가락 방향 그대로 위로 빠지는 것, 높은 것 먼저.
        지도 가장자리에선 바로 위로 못 올라가 (손을 아래로 못 돌림) 비스듬히 빠지는 길만 풀린다."""
        np = self.np
        opts = []
        for al in APPROACH_TILTS:
            for be in (range(0, 360, APPROACH_AZ) if al else (0,)):
                a_, b_ = math.radians(al), math.radians(be)
                v = np.array([math.sin(a_) * math.cos(b_), math.sin(a_) * math.sin(b_), math.cos(a_)])
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(v @ axis)))))
                for cm in ABOVE_CM:
                    opts.append((ang + 5.0 * (ABOVE_CM[0] - cm), cm, al, be, v))
        opts.sort(key=lambda o: o[0])
        hold = self.S["q0"].copy()
        for _, cm, al, be, v in opts:
            pp = p + v * (cm / 100)
            base, _ = self.s.analytic_tip(pp, hold, self.lim, "position", first=True)   # 빠른 거르기: 손 돌림 전 집는 점이 맞나
            if not any(float(np.linalg.norm(self.grip(q) - pp)) <= TIP_TOL for q in base):
                continue
            c = self.candidates(pp, yaw_sim)
            if fam:
                e = self.s.arm.index("elbow")
                c = [q for q in c if (q[e] < 0) == (self.home[e] < 0)]
            if c:
                yield {"cm": cm, "tilt_deg": al, "azimuth_deg": be, "v": v, "cands": c}

    def hand_axis(self, q):
        return self.s.fk(q)[1][:, 2].copy()

    def cube_hits(self, frames, idx, pos, yaw_deg):
        """frames[idx] 중 손가락 충돌 상자가 큐브(5 cm, pos sim, yaw)에 0.5 mm 넘게 파고드는 프레임 번호."""
        mj, s = self.s.mj, self.s
        m, d = s.m, s.d
        a = m.jnt_qposadr[m.body_jntadr[self.obj_body]]
        h = math.radians(yaw_deg) / 2
        d.qpos[a:a + 7] = [pos[0], pos[1], pos[2], math.cos(h), 0.0, 0.0, math.sin(h)]
        out = []
        for i in idx:
            q, f = frames[i]
            d.qpos[s.qadr] = q
            d.qpos[s.fadr] = f
            mj.mj_kinematics(m, d)
            mj.mj_collision(m, d)
            for c in d.contact[:d.ncon]:
                if self.cube_geom in (c.geom1, c.geom2) and (c.geom1 in self.fgeoms or c.geom2 in self.fgeoms) and c.dist < -5e-4:
                    out.append(i)
                    break
        return out

    # ---- 경로 (관절 목록)
    A3I = (0, 2, 3)   # shoulder_roll, upper_arm_roll, elbow: 해석 IK 가 푸는 팔 관절 (가지 이어 붙이기는 이것으로)

    def wrist(self, q):
        self.s.fk(q)
        return self.s.wrist_pos().copy()

    def _near(self, sols, prev):
        i = list(self.A3I)
        return min(sols, key=lambda c: float(self.np.abs(c[i] - prev[i]).max()))

    def _jump(self, q, prev):
        """이웃 IK 점 사이 팔 관절이 JUMP_DEG 넘게 튀면 (가지가 끊겨 다른 가지로 넘어감) 그 길은 버린다."""
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(q[i] - prev[i])).max()) > JUMP_DEG

    def _same(self, q, qt):
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(q[i] - qt[i])).max()) <= SAME_BRANCH_DEG

    def transit(self, qf, qt, f_home=False, t_home=False):
        """손목 중앙 길: 위로 → 높이 유지하며 옆으로 → 아래로 (높이 = 두 끝과 z_clear 중 높은 쪽, 안 되면 +3, +6 cm, 그래도 안 되면 곧은 선),
        1 cm 마다 해석 IK(앞 가지 이어서, 튀면 버림). 손목 2 관절은 손이 최대한 아래 (point_down). 끝이 HOME(손이 옆으로 누운 자세)이면
        그 손목 값에서/으로 옆 이동 동안 섞는다. hand_roll 은 옆 이동 동안 qf → qt. 끝 해가 qt 와 다른 가지면 None."""
        np = self.np
        w0, w1 = self.wrist(qf), self.wrist(qt)
        for dz in TRANSIT_LIFTS:
            if dz is None:
                legs = ((w0, w1, 1),)
            else:
                zt = max(w0[2], w1[2], self.z_clear) + dz
                u0, u1 = np.array([w0[0], w0[1], zt]), np.array([w1[0], w1[1], zt])
                legs = ((w0, u0, 0), (u0, u1, 1), (u1, w1, 2))
            pts = []
            for a, b, leg in legs:
                n = int(math.ceil(float(np.linalg.norm(b - a)) / STEP_TRANSIT))
                for k in range(1, n + 1):
                    pts.append((a + (b - a) * (k / n), leg, k / n))
            r = self._track(pts, qf, qt, f_home, t_home)
            if r is not None:
                return r
        return None

    def _track(self, pts, qf, qt, f_home, t_home):
        s = self.s
        prev, out = qf.copy(), [qf.copy()]
        hr, i56 = self.hr, [4, 5]
        for w, leg, u in pts:
            b = 0.0 if leg == 0 else 1.0 if leg == 2 else u * u * (3 - 2 * u)
            hold = prev.copy()
            hold[hr] = qf[hr] + (qt[hr] - qf[hr]) * b
            sols, _ = s.analytic_position(w, hold, self.lim)
            if not sols:
                return None
            q = self._near(sols, prev).copy()
            if self._jump(q, prev):
                return None
            a56 = qf[i56] if f_home else q[i56]
            b56 = qt[i56] if t_home else q[i56]
            q[i56] = a56 + (b56 - a56) * b
            out.append(q)
            prev = q
        if not out or not self._same(out[-1], qt):
            return None
        out[-1] = qt.copy()
        return out

    def family(self, cands):
        """HOME 과 같은 팔꿈치 방향(부호) 가지만. 반대 가지로 넘어가려면 팔을 쭉 펴야 해서(특이점) 공중에서 크게 휘두르게 된다.
        같은 가지 해가 없으면 그대로 (그때는 길을 못 찾아 관절 직선 → 검사에 걸림)."""
        e = self.s.arm.index("elbow")
        same = [q for q in cands if (q[e] < 0) == (self.home[e] < 0)]
        return same or cands

    def near_branch(self, qa, qg):
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(qa[i] - qg[i])).max()) <= 45.0

    def descent(self, qa, qg):
        """집는 점을 qa 의 집는 점에서 qg 의 집는 점까지 직선으로 (STEP_DESCENT 마다, 앞 가지 이어서). hand_roll 은 qa → qg.
        각 점: 손목 목표 = 앞 손목 + (집는 점 목표 − 앞 집는 점) 에서 시작해 해석 IK(손 최대한 아래) → 집는 점 오차만큼 손목을 옮겨
        다시 (analytic_tip 과 같은 방식, 앞 점에서 이어 시작해 빠르다)."""
        np, s = self.np, self.s
        pa, pg = self.grip(qa), self.grip(qg)
        n = max(1, int(math.ceil(float(np.linalg.norm(pg - pa)) / STEP_DESCENT)))
        prev, out = qa.copy(), [qa.copy()]
        for k in range(1, n + 1):
            p = pa + (pg - pa) * (k / n)
            hold = prev.copy()
            hold[self.hr] = qa[self.hr] + (qg[self.hr] - qa[self.hr]) * (k / n)
            w = self.wrist(prev) + (p - self.grip(prev))
            q = None
            for _ in range(8):
                sols, _ = s.analytic_position(w, hold, self.lim)
                if not sols:
                    return None
                q = self._near(sols, prev)
                e = p - self.grip(q)
                if float(np.linalg.norm(e)) < 3e-4:
                    break
                w = w + e
            if self._jump(q, prev):
                return None
            out.append(q)
            prev = q
        if not self._same(out[-1], qg) or float(np.linalg.norm(self.grip(out[-1]) - pg)) > TIP_TOL:
            return None
        out[-1] = qg.copy()
        return out

    def arc(self, qs):
        np = self.np
        return float(sum(np.abs(b - a).max() for a, b in zip(qs, qs[1:])))

    def frames(self, qs, fa, fb, speed_deg_s, min_s=0.0):
        """관절 목록 qs (꺾은선) 를 따라, 손가락 fa → fb. 길이 = 이웃 사이 관절 최대 이동각의 합(손가락 포함),
        smoothstep 시간으로 — 가장 빠른 관절도 speed 를 안 넘는다. 첫 프레임 뺀 (q, f) 목록."""
        np = self.np
        fmove = float(np.abs(fb - fa).max())
        segs = [float(np.abs(b - a).max()) for a, b in zip(qs, qs[1:])]
        L = sum(segs)
        n = max(1, int(math.ceil(max(math.degrees(max(L, fmove)) * SMOOTH_PEAK / (speed_deg_s * DT), min_s / DT))))
        cum = np.concatenate([[0.0], np.cumsum(segs)])
        out = []
        for k in range(1, n + 1):
            t_ = k / n
            u = t_ * t_ * (3 - 2 * t_)
            if L <= 0 or len(qs) == 1:
                q = qs[-1].copy()
            else:
                x = u * L
                j = min(int(np.searchsorted(cum, x, side="right")) - 1, len(segs) - 1)
                r = 0.0 if segs[j] <= 0 else (x - cum[j]) / segs[j]
                q = qs[j] + (qs[j + 1] - qs[j]) * min(1.0, max(0.0, r))
            out.append((q, fa + (fb - fa) * u))
        return out

    def table_penalty(self, q):
        """잡는·놓는 자세에서 손가락(벌림·쥠)이 책상 아래면 벌점. 손이 기울면 아래쪽 손가락 끝이 내려간다 —
        hand_roll 90° 다른 해(큐브는 90° 마다 같음)는 기울기 축이 달라 손가락 높이가 다르다: 닿지 않는 쪽을 고르게."""
        return 0.0 if self.grasp_clearance(q) >= TABLE_MARGIN else FALLBACK_COST

    def grasp_clearance(self, q):
        """잡는·놓는 자세 q 에서 손가락(벌림·쥠 중 낮은 쪽) 가장 낮은 점의 책상 위 높이 (m)."""
        return min(self.finger_low(q, self.open), self.finger_low(q, self.closed)) - self.s.table_z

    def below_table(self, fr):
        return any(self.finger_low(q, f) < self.s.table_z + TABLE_MARGIN for q, f in fr)

    def carried_cube_low(self, qg, pc, yaw_deg, qs):
        """잡는 자세 qg 에서 큐브(중심 pc, yaw, 수평)를 손에 붙여 qs 자세들로 옮길 때 큐브 8 꼭짓점 중 가장 낮은 z (sim).
        손이 기울면 큐브도 같이 기운다 (바닥 중심이 아니라 가장 낮은 모서리)."""
        np = self.np
        ps, rs = self.s.fk(qg)
        h = math.radians(yaw_deg)
        rc = np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]])
        rel_r, rel_p = rs.T @ rc, rs.T @ (np.asarray(pc) - ps)
        half = float(self.s.E.OBJECT_HALF_SIZE)
        low = 9.0
        for q in qs:
            p, r = self.s.fk(q)
            c, rr = p + r @ rel_p, r @ rel_r
            low = min(low, float(c[2] - half * np.abs(rr[2]).sum()))
        return low

    def finger_low(self, q, f):
        """손가락 충돌 상자 가장 낮은 z. 닫는 값이 CUBE_GRIP(패드 사이 = 큐브 50 mm, env) 을 넘는 프레임은 큐브가 손가락을
        거기서 막으므로 CUBE_GRIP 자세로 잰다 (명령은 팀처럼 1.0 — 남는 만큼이 쥐는 힘)."""
        s, np = self.s, self.np
        g = float(np.clip((f[0] - self.open[0]) / (self.closed[0] - self.open[0]), 0.0, 1.0)) if self.closed[0] != self.open[0] else 0.0
        if g > self.cube_grip:
            f = np.asarray(s.env.finger_targets(self.cube_grip), dtype=float)
        s.d.qpos[s.fadr] = f
        s.fk(q)
        return s.lowest_finger_z()


def plan(a):
    t0 = time.time()
    cal_file = a.calibration or (REPO / "arm_control" / "calibration.json")
    P = Planner(_find_ik(a.ik), _cmd_ranges(cal_file))
    s, np = P.s, P.np
    R = s.R
    out = {"planner": "analytic",
           "real": {"cube": [float(v) for v in a.cube], "target": [float(v) for v in a.target], "d": float(R.MAP_D if a.d is None else a.d)}}
    if a.d is not None and abs(a.d - R.MAP_D) > R.D_TOLERANCE:
        return {**out, "status": "unreachable", "reason": f"책상 높이 d = {a.d} m 는 계획 못 함 (IK 모델은 {R.MAP_D} m)"}
    cx, cy, cyaw = a.cube
    tx, ty = a.target
    tyaw = cyaw if a.target_yaw is None else a.target_yaw
    # 1) 지도와 같은 판정 (그 점 그대로, MuJoCo FK 검증 포함)
    for what, x, y in (("큐브", cx, cy), ("놓을 자리", tx, ty)):
        code, _, _ = s.grasp_eval(P.S, x, y, verify=True)
        if code != 2:
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {REASON[code]}"}
    zc = 0.025 - R.MAP_D                     # 큐브 중심 높이 (실좌표, 어깨 축 기준)
    pc = np.asarray(s.to_sim([cx, cy, zc]))
    pt = np.asarray(s.to_sim([tx, ty, zc]))
    ycube = cyaw + R.SIM_FROM_REAL_YAW        # sim yaw (mod 90 만 쓴다)
    yplace = tyaw + R.SIM_FROM_REAL_YAW
    # 2) 경유점 후보
    G = P.family(P.candidates(pc, ycube))
    Pl = P.family(P.candidates(pt, yplace))
    for name, c in (("큐브를 잡는 자세", G), ("놓는 자세", Pl)):
        if not c:
            return {**out, "status": "unreachable", "reason": f"{name}: 손 방향(yaw)까지 맞는 해 없음"}
    # 집는 점은 그대로(큐브 중심), 손가락이 책상 위 TABLE_MARGIN 을 지키는 해만. 없으면 그 자리는 집을 수 없음
    for name, c in (("큐브를 잡는 자세", G), ("놓는 자세", Pl)):
        if not any(P.grasp_clearance(q) >= TABLE_MARGIN for q in c):
            best = max(c, key=P.grasp_clearance)
            return {**out, "status": "unreachable",
                    "reason": f"{name}: 손가락이 책상 위 {TABLE_MARGIN * 1000:.0f} mm 를 못 지킴 (가장 좋은 해 {P.grasp_clearance(best) * 1000:+.1f} mm, "
                              f"손 기울기 {s.axis_tilt(s.fk(best)[1]):.0f}°)"}
    G = [q for q in G if P.grasp_clearance(q) >= TABLE_MARGIN]
    Pl = [q for q in Pl if P.grasp_clearance(q) >= TABLE_MARGIN]
    home = P.home    # 팀 계획과 같은 시작·끝 (env HOME_ARM_QPOS, shoulder_yaw 6.3°)
    home0 = home.copy()
    home0[s.arm.index("shoulder_yaw")] = 0.0   # 해석 IK 는 shoulder_yaw 0 고정 → HOME 에서 먼저 그것만 0 으로
    sp = float(a.max_speed)
    O, C = P.open, P.closed

    def leg(path, qf, qt, what, fb):
        if path is None:   # 손목 높이 유지 길을 못 찾음: 관절 공간 직선 (검사가 책상·큐브 닿음을 잡는다)
            fb.append(what)
            return [qf.copy(), qt.copy()]
        return path

    # 2) 큐브 위·놓을 자리 위 경유점: 손가락 방향 그대로 위로 빠지는 길부터. 고르는 순서 (욕심쟁이, 집기 먼저):
    #    집기 = HOME→큐브 위 길 + 내려가기 의 관절 이동 합이 가장 작은 (큐브 위, 잡는 자세) 가지, 벌린 손가락이 큐브에 안 닿으면 확정
    #    놓기 = 그 큐브 위 자세에서 놓을 자리 위로 + 내려가기 + HOME 으로 의 합이 가장 작은 것, 빠질 때 놓은 큐브에 안 닿으면 확정
    g_ref = min(G, key=lambda q: float(np.abs(q - home0).max()))
    p_ref = min(Pl, key=lambda q: float(np.abs(q - home0).max()))
    target_pos = np.array([pt[0], pt[1], pc[2]])

    def pick_part(ap, clean_only):
        """clean_only: 이미 대체(안 깨끗한) 계획이 있으면 길을 못 찾은 후보는 아예 안 푼다 (시간)."""
        best = None
        t1s = [(qa, P.transit(home0, qa, f_home=True)) for qa in ap["cands"]]
        if any(t1 for _, t1 in t1s):
            t1s = [(qa, t1) for qa, t1 in t1s if t1]
        elif clean_only:
            return None
        for qa, t1 in t1s:
            for qg in sorted((g for g in G if P.near_branch(qa, g)), key=lambda g: (P.table_penalty(g), float(np.abs(g - qa).max())))[:1]:
                d1 = P.descent(qa, qg)
                if d1 is None:
                    continue
                c = P.arc(t1 or [home0, qa]) + P.arc(d1) + (0.0 if t1 else FALLBACK_COST) + P.table_penalty(qg)
                if best is None or c < best[0]:
                    best = (c, qa, qg, t1, d1)
        if best is None:
            return None
        c, qa, qg, t1, d1 = best
        fr, fb = [(home, O)], []
        fr += P.frames([home, home0], O, O, sp)
        fr += P.frames(leg(t1, home0, qa, "HOME→큐브 위", fb), O, O, sp)
        i_above = len(fr) - 1
        fr += P.frames(d1, O, O, sp)
        hit = P.cube_hits(fr, range(len(fr)), pc, ycube)
        bad = bool(hit or fb or P.below_table(fr))
        return {"bad": bad, "c": c, "qa": qa, "qg": qg, "d1": d1, "frames": fr, "above": i_above, "hit": hit, "ap": ap, "fb": fb}

    def place_part(pk, bp, clean_only):
        best = None
        qa = pk["qa"]
        ts = [(qb, P.transit(qa, qb), P.transit(qb, home0, t_home=True)) for qb in bp["cands"]]
        if any(t2 and t3 for _, t2, t3 in ts):
            ts = [x for x in ts if x[1] and x[2]]
        elif clean_only:
            return None
        for qb, t2, t3 in ts:
            for qp in sorted((g for g in Pl if P.near_branch(qb, g)), key=lambda g: (P.table_penalty(g), float(np.abs(g - qb).max())))[:1]:
                d2 = P.descent(qb, qp)
                if d2 is None:
                    continue
                c = (P.arc(t2 or [qa, qb]) + P.arc(d2) + P.arc(t3 or [qb, home0])
                     + (0.0 if t2 else FALLBACK_COST) + (0.0 if t3 else FALLBACK_COST) + P.table_penalty(qp))
                if best is None or c < best[0]:
                    best = (c, qb, qp, t2, d2, t3)
        if best is None:
            return None
        c, qb, qp, t2, d2, t3 = best
        qg, d1 = pk["qg"], pk["d1"]
        fr, mk = list(pk["frames"]), {"start": 0, "above_pick": pk["above"], "grasp": len(pk["frames"]) - 1}
        fb = list(pk["fb"])

        def add(qs, fa, fz, key, min_s=0.0):
            fr.extend(P.frames(qs, fa, fz, sp, min_s))
            mk[key] = len(fr) - 1
        add([qg, qg], O, C, "close", GRIP_S)
        add([qg, qg], C, C, "closed", GRIP_HOLD_S)
        add(d1[::-1], C, C, "lift")
        add(leg(t2, qa, qb, "큐브 위→놓을 자리 위", fb), C, C, "above_place")
        add(d2, C, C, "place")
        add([qp, qp], C, O, "open", GRIP_S)
        add([qp, qp], O, O, "opened", GRIP_HOLD_S)
        add(d2[::-1], O, O, "lift2")
        add(leg(t3, qb, home0, "놓을 자리 위→HOME", fb), O, O, "home0")
        add([home0, home], O, O, "home")
        hit = P.cube_hits(fr, range(mk["opened"], len(fr)), target_pos, yplace)
        bad = bool(hit or len(fb) > len(pk["fb"]) or P.below_table(fr[mk["grasp"]:]))
        return {"bad": bad, "c": c + pk["c"], "q": (qa, qg, qb, qp), "frames": fr, "marks": mk, "hit": hit, "bp": bp, "fb": fb}

    ie = s.arm.index("elbow")
    fam = lambda L: any((q[ie] < 0) == (home[ie] < 0) for q in L)
    pick_it = P.approaches(pc, ycube, P.hand_axis(g_ref), fam(G))
    place_it = P.approaches(pt, yplace, P.hand_axis(p_ref), fam(Pl))
    pk = first_pk = None
    tries_a = 0
    for ap in pick_it:
        tries_a += 1
        r = pick_part(ap, first_pk is not None)
        if r is not None:
            first_pk = first_pk or r
            if not r["bad"]:
                pk = r
                break
        if tries_a >= MAX_TRIES:
            break
    pk = pk or first_pk
    if pk is None:
        return {**out, "status": "unreachable", "reason": "큐브 위 5~9 cm 에서 손을 아래로 둔 채 큐브까지 곧게 내려가는 길이 없음"}
    plan_ = first_pl = None
    tries_b = 0
    for bp in place_it:
        tries_b += 1
        r = place_part(pk, bp, first_pl is not None)
        if r is not None:
            first_pl = first_pl or r
            if not r["bad"]:
                plan_ = r
                break
        if tries_b >= MAX_TRIES:
            break
    plan_ = plan_ or first_pl
    if plan_ is None:
        return {**out, "status": "unreachable", "reason": "놓을 자리 위 5~9 cm 에서 손을 아래로 둔 채 곧게 내려가는 길이 없음"}
    fallback = plan_["fb"]
    ap, bp = pk["ap"], plan_["bp"]
    frames, mk = plan_["frames"], plan_["marks"]
    qa, qg, qb, qp = plan_["q"]
    ha, hb = ap["cm"], bp["cm"]
    labels = {"start": "start (HOME)", "above_pick": f"큐브 위 (+{ha:g} cm, 수직에서 {ap['tilt_deg']}°)", "grasp": "내려감: 큐브 잡는 자세",
              "close": "손가락 닫기", "closed": "기다림", "lift": "들기", "above_place": f"놓을 자리 위 (+{hb:g} cm, 수직에서 {bp['tilt_deg']}°)",
              "place": "내려감: 놓는 자세", "open": "손가락 열기", "opened": "기다림", "lift2": "들기",
              "home0": "HOME (shoulder_yaw 만 0)", "home": "HOME"}
    # 3) 검사: 손가락↔큐브, 손가락↔책상, 관절 한계, 잡는 점 오차, 패드↔큐브 면 각, 들고 갈 때 큐브 바닥 높이
    hit_pick = P.cube_hits(frames, range(0, mk["grasp"] + 1), pc, ycube)
    hit_place = P.cube_hits(frames, range(mk["opened"], len(frames)), target_pos, yplace)
    lim = P.lim
    low = [P.finger_low(q, f) for q, f in frames]
    below = [i for i, z in enumerate(low) if z < s.table_z + TABLE_MARGIN]
    names = list(s.arm) + list(s.fingers)
    traj_q = [list(map(float, q)) + list(map(float, f)) for q, f in frames]
    sy = s.arm.index("shoulder_yaw")   # 0 고정 (HOME 만 6.3°: 그 사이 보간은 범위 안)
    over = [n for k, n in enumerate(s.arm) if k != sy and any(not (lim[k, 0] - 1e-4 <= q[k] <= lim[k, 1] + 1e-4) for q, _ in frames)]
    grip_err = float(np.linalg.norm(P.grip(qg) - pc))
    place_err = float(np.linalg.norm(P.grip(qp) - pt))

    def pad_face(q, yaw):
        """손가락이 닫히는 축(site x)과 가장 가까운 큐브 옆면 법선 사이 3D 각 (°). 손이 기울면 0 이 아닐 수 있다."""
        x = s.fk(q)[1][:, 0]
        return min(math.degrees(math.acos(min(1.0, abs(float(x @ [math.cos(math.radians(yaw + k)), math.sin(math.radians(yaw + k)), 0.0])))))
                   for k in (0, 90))

    def side(i0, i1, p):
        return max(float(np.linalg.norm((P.grip(frames[i][0]) - p)[:2])) for i in range(i0, i1 + 1))
    carry_low = P.carried_cube_low(qg, pc, ycube, [frames[i][0] for i in range(mk["lift"], mk["above_place"] + 1)]) - s.table_z
    spd = np.degrees(np.abs(np.diff(np.array(traj_q), axis=0)).max(axis=0)) / DT
    problems = []
    if hit_pick:
        problems.append(f"벌린 손가락이 큐브에 닿음 (집으러 갈 때): 프레임 {hit_pick[0]}~{hit_pick[-1]} ({len(hit_pick)}개)")
    if hit_place:
        problems.append(f"손가락이 놓은 큐브에 닿음 (빠질 때): 프레임 {hit_place[0]}~{hit_place[-1]} ({len(hit_place)}개)")
    if below:
        problems.append(f"손가락이 책상 위 여유 {TABLE_MARGIN * 1000:.0f} mm 보다 낮게 감: 프레임 {below[0]}~{below[-1]} ({len(below)}개), "
                        f"가장 낮게 책상 위 {(min(low) - s.table_z) * 1000:+.1f} mm")
    if over:
        problems.append("관절 한계 밖: " + ", ".join(over))
    if carry_low < TABLE_MARGIN:
        problems.append(f"들고 가는 큐브의 가장 낮은 모서리가 책상 위 {carry_low * 1000:+.1f} mm (여유 {TABLE_MARGIN * 1000:.0f} mm 아래)")
    if fallback:
        problems.append("손목 높이 유지 길을 못 찾아 관절 공간 직선으로 감: " + ", ".join(fallback))
    if grip_err > TIP_TOL or place_err > TIP_TOL:
        problems.append(f"집는 점이 큐브 중심에서 잡을 때 {grip_err * 1000:.1f} mm, 놓을 때 {place_err * 1000:.1f} mm")
    status = "playable" if not problems else ("finger below table" if below else "finger hits cube" if (hit_pick or hit_place) else "check failed")
    wp = [{"label": labels[k], "frame": i, "t_s": round(i * DT, 2),
           "joints_deg": {n: round(math.degrees(float(v)), 2) for n, v in zip(s.arm, frames[i][0])},
           "grip_real": [round(float(v), 4) for v in s.to_real(P.grip(frames[i][0]))],
           "hand_yaw_real_deg": round(P.hand_yaw(frames[i][0]) - R.SIM_FROM_REAL_YAW, 1),
           "hand_tilt_deg": round(s.axis_tilt(s.fk(frames[i][0])[1]), 1),
           "fingers": "closed" if np.allclose(frames[i][1], C) else "open"} for k, i in mk.items()]
    out["sim"] = {"cube": pc.tolist(), "target": pt[:2].tolist(), "cube_yaw_deg": ycube,
                  "hand_yaw_deg": round(P.hand_yaw(qg), 2), "hand_tilt_deg": round(s.axis_tilt(s.fk(qg)[1]), 2)}
    return {**out, "status": status,
            "reason": "해석 IK 계획 (지도와 같은 IK, 물리·쥐기 시뮬 없음)" if not problems else "; ".join(problems),
            "playable": status == "playable", "delivered_in_sim": None, "retries": [], "final_error_m": None,
            "joint_names": names, "dt": DT, "positions": traj_q, "unit": "rad",
            "peak_speed_deg_s": {"joint": names[int(spd.argmax())], "value": float(spd.max())},
            "waypoints": wp, "problems": problems,
            "checks": {"grasp_grip_error_mm": round(grip_err * 1000, 2), "place_grip_error_mm": round(place_err * 1000, 2),
                       "pad_face_deg": {"pick": round(pad_face(qg, ycube), 1), "place": round(pad_face(qp, yplace), 1)},
                       "lowest_finger_above_table_mm": round((min(low) - s.table_z) * 1000, 1),
                       "finger_cube_hits": {"pick": len(hit_pick), "place": len(hit_place)},
                       "descent_side_max_mm": {"pick": round(side(mk["above_pick"], mk["grasp"], pc) * 1000, 1),
                                               "place": round(side(mk["above_place"], mk["place"], pt) * 1000, 1)},
                       "carry_cube_bottom_min_above_table_mm": round(carry_low * 1000, 1),
                       "approach": {"pick": {"cm": ha, "tilt_deg": ap["tilt_deg"], "azimuth_deg": ap["azimuth_deg"], "tried": tries_a},
                                    "place": {"cm": hb, "tilt_deg": bp["tilt_deg"], "azimuth_deg": bp["azimuth_deg"], "tried": tries_b}},
                       "path_joint_move_deg": round(math.degrees(plan_["c"]), 1),
                       "joint_space_fallback": fallback,
                       "n_candidates": {"above_pick": len(ap["cands"]), "grasp": len(G), "above_place": len(bp["cands"]), "place": len(Pl)}},
            "plan_s": round(time.time() - t0, 2)}


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8")   # 로그(procman)는 utf-8 로 읽는다
    except Exception:
        pass
    from arm_control.play import add_run_args, checked_run
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cube", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    ap.add_argument("--target", type=float, nargs=2, required=True, metavar=("X", "Y"))
    ap.add_argument("--target-yaw", type=float, default=None, help="놓을 때 큐브 yaw (°, 실좌표). 비우면 집을 때와 같게")
    ap.add_argument("--d", type=float, default=None)
    ap.add_argument("--ik", default=None)
    ap.add_argument("--save", default=None)
    add_run_args(ap)
    a, _ = ap.parse_known_args(argv)
    print("planning (analytic IK, same as the graspable map; no physics) ...", flush=True)
    r = plan(a)
    if r["status"] == "unreachable":
        print(f"refused: {r['reason']}")
        raise SystemExit(3)
    c = r["checks"]
    print(f"plan: {r['status']} — grip {c['grasp_grip_error_mm']} mm from the cube centre at the grasp, place "
          f"{c['place_grip_error_mm']} mm, hand turned {r['sim']['hand_yaw_deg']:+.1f} deg (sim), tilt {r['sim']['hand_tilt_deg']:.1f} deg, "
          f"lowest finger {c['lowest_finger_above_table_mm']:+.1f} mm above the table, {r['plan_s']} s")
    for w in r["waypoints"]:
        print(f"  {w['t_s']:6.2f}s  {w['label']}: grip {w['grip_real']}, hand yaw {w['hand_yaw_real_deg']:+.1f}, fingers {w['fingers']}")
    for p in r["problems"]:
        print(f"check: {p}")
    if a.save:
        with open(a.save, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, ensure_ascii=False)
        print(f"wrote {a.save}")
    raise SystemExit(checked_run(r, a))


if __name__ == "__main__":
    main()
