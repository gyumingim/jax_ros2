"""대시보드 집어 옮기기 계획 (해석 IK) — '집을 수 있는 곳' 지도와 같은 IK 로 (2026-11-20).

    python dashboard/pick_plan.py --cube X Y YAW --target X Y --max-speed 30 --save logs/pick_dry_traj.json --dry-run

[계획 돌려 보기] 의 기본. 팀 계획(arm_control/pick_place.py: team-jax/IK motion1 MuJoCo 물리 시뮬, 자기 수치 IK)은
지도(hand_ik.grasp_eval, 해석 IK)와 판정이 달라 초록 칸에서도 실패했다. 여기는 지도와 같은 hand_ik._Solver 로 푼다:
  - 집는 점(end_effector site, 손끝 사이) = 큐브 중심 + GRIP_ABOVE_CENTER 1 cm (책상 위 3.5 cm, hand_ik 한 곳), shoulder_yaw 0 고정, 손목 중앙으로 해석 IK
    + 손목 2 관절로 손 최대한 아래 + 집는 점 오차만큼 손목을 옮겨 다시 (analytic_tip) — 지도와 같은 풀이.
  - 손 돌림: 손가락이 닫히는 축(site x)을 위에서 본 각(손 yaw)을 FK 로 재고, hand_roll(±88°)로 큐브 yaw 에 맞춘다
    (큐브는 90° 마다 같으니 yaw mod 90). 손목 관절이 이미 손을 돌린 만큼은 FK 로 재서 빼므로 자동으로 들어간다.
  - 경유점: HOME(열림) → 큐브 위(집는 점 9 cm 위, 안 되면 1 cm 씩 낮춤) → 내려감(집는 점 = 큐브 중심 + 1 cm) → 손가락 닫기 → 들기(큐브 위)
    → 놓을 자리 위 → 내려감(큐브 바닥 = 책상) → 손가락 열기 → 들기 → HOME.
  - 가지 고르기: 경유점마다 해석 IK 해(가지) × hand_roll 후보 중, 이웃 경유점 사이 관절 최대 이동각의 합이 가장 작은 줄
    (손 IK 의 '최소 회전' 규칙을 경로 전체로).
  - 경유점 사이는 관절 공간 보간, 사다리꼴 시간(가장 빠른 관절이 max_speed °/s, 실물 램프 가속 max_acc °/s² 를 안 넘게), 프레임 간격 dt 0.02 s.
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
from hand_ik import GRIP_ABOVE_CENTER   # noqa: E402  집는 점 = 큐브 중심 + 이것 (지도·계획·IK 탭 한 값)

ABOVE_CM = (9.0, 7.0, 5.0, 3.0)   # 큐브·놓을 자리 위 경유점: 집는 점을 이만큼 (3.25 cm 미만이면 벌린 손가락 끝이 큐브 윗면보다 낮다:
                                  # 그때는 손가락이 큐브 양옆으로 곧게 내려가야 한다 — 손가락↔큐브 검사가 본다)
APPROACH_TILTS = (0, 15, 30, 45, 60)   # 그 경유점 방향: 수직에서 기울기(°) × 방위 45° 마다
APPROACH_AZ = 45
STEP_TRANSIT = 0.01         # 옮겨 가는 길: 손목 중앙 1 cm 마다 IK
STEP_DESCENT = 0.005        # 내려가기·들기: 집는 점 0.5 cm 마다 IK (그 사이는 관절 보간). 1 cm 이면 손목 2 관절이 한계에 닿는 곳에서
                            # 한 걸음에 팔이 15° 넘게 움직여 '가지 튐' 으로 버려졌다 (2026-11-21 지도 판정과 같이 0.5)
FALLBACK_COST = 10.0        # 길을 못 찾아 관절 직선으로 가는 구간의 벌점 (rad, 고르기에서 뒤로)
TRANSIT_LIFTS = (0.0, 0.03, 0.06, None)   # 옮겨 가는 길의 높이: max(두 끝, 하한) + 이만큼 (None = 곧은 선)
CLEAR_CM = 3.0               # 옮겨 갈 때 손가락 끝이 큐브 윗면보다 이만큼 위 (손목 높이 하한)
JUMP_DEG = 15.0             # 그 사이 팔 관절이 이보다 크게 튀면 가지가 끊긴 것 (그 길 버림)
SAME_BRANCH_DEG = 2.0       # 길 끝 해가 목표 자세와 같은 가지인가 (팔 3 관절 차이)
MAX_TRIES = 100               # 손가락이 큐브에 닿지 않는 접근을 찾을 때 볼 후보 수 (집기·놓기 각각)
DT = 0.02                    # 프레임 간격 (팀 계획과 같은 50 Hz)
GRIP_S = 0.8                 # 손가락 닫기·열기 시간 (팀: 한 스텝 0.025 씩 → 40 스텝 = 0.8 s)
GRIP_HOLD_S = 0.2            # 닫은/연 뒤 기다림
YAW_TOL = 2.0                # 손 yaw 가 큐브 면에서 이 각(°) 안
TIP_TOL = 0.005              # 집는 점 오차 5 mm (지도와 같은 TOL_POS)
TRACK_LAG_DEG = 2.0          # 실물 램프(real_arm)가 순항 중 늦게 따라가는 각 상한 → 순항 속도 √(2·max_acc·이것) = 19 °/s (max_acc 90)
try:   # 실물 램프 가속(°/s²) — real_arm.DEFAULTS 한 곳에서 읽는다 (계획 프레임을 실물이 따라갈 수 있게)
    from real_arm import DEFAULTS as _REAL
    MAX_ACC = float(_REAL["max_acc"])
except Exception:   # real_arm 을 못 읽는 곳(팀 PC 등)에선 보수적으로 같은 기본값 — 경고를 남긴다
    MAX_ACC = 90.0
    print("pick_plan: real_arm.DEFAULTS 를 못 읽어 MAX_ACC 90 °/s² 가정", file=sys.stderr)
TABLE_MARGIN = 0.008         # 손가락 충돌 상자·들고 가는 큐브 바닥이 책상 면보다 이만큼(m) 위여야 (실물 책상 높이 MAP_D 0.4775 는 가정 —
                             # 실측 전엔 오차가 이 여유를 먹는다. README). 집는 점 = 큐브 중심이던 때는 수직 손에서 쥔 손가락 8.5 mm 위라
                             # 손이 조금만 기울어도 못 지켰다 → 2026-11-21 집는 점 +1 cm (GRIP_ABOVE_CENTER, 수직이면 18.5 mm 위)
YAW_SAMPLES = tuple(range(0, 90, 15))   # 지도 판정에서 볼 큐브 yaw (sim °, 90° 마다 같음)
VERT_TILT = 90.0            # ° 지도: 잡는 자세 손이 이보다 기울면 '못 집음' — 2026-11-21 집는 점 +1 cm 부터 사실상 끔 (손가락 책상 여유
                            # _roll_best 가 판정). 예전(집는 점 = 큐브 중심) 0.5: 수직이어도 쥔 손가락 8.5 mm 라 1° 만 기울어도 8 mm 를 못 지켰다.
                            # +1 cm 면 수직 18.5 mm 위라 8~15° 기운 칸도 통과 (plan_check 로 확인, README)
ELBOW_BRANCH = -1           # 계획·지도가 쓰는 팔꿈치 가지 (elbow 부호) = 예전 HOME(elbow −0.92 rad)의 가지. HOME 이 0 이 된 뒤에도 (2026-11-21)
# HOME = 모든 관절 0 (팔꿈치가 이 각 안 = 팔이 곧게 늘어짐) 이면 HOME 에서 나가고 들어오는 관절 경유점 (Planner._home_exit, 2026-11-21).
# 값은 경유점 격자 검색으로 고름 (shoulder_roll −45..0, elbow −90..−135, wrist ±45·90 중 관절 이동 합이 작고 손가락이 책상 발자국 위에서
# 책상 면 + 41.6 mm 이상): elbow −120 이면 이동이 10° 짧지만 여유 21~25 mm 라 이것. elbow −130 은 한계 ±135 에 5° 남음 (실물 끝 간섭 미확인, calibration note).
CUBE_EDGE_IN = 0.01          # 큐브(놓을 자리 포함)는 책상 발자국 안쪽으로 반 변 + 이만큼 (env CUBE_X_MAX 와 같은 1 cm, 2026-11-21)
HOME_STRAIGHT_DEG = 5.0
HOME_EXIT_MID_DEG = {"shoulder_roll": -40.0, "upper_arm_roll": 180.0, "elbow": -130.0, "wrist_pitch": -45.0, "wrist_yaw": 90.0, "hand_roll": 0.0}
HOME_EXIT_WRIST_X = 0.18     # 그다음 경유점: 손목 중앙 sim x (책상 앞 끝 0.20 에서 2 cm 안, 책상 위), y = HOME 손목 y, 높이 z_clear
HOME_EXIT_LIFT = 0.04        # 그 경유점 손목 높이 = z_clear + 이만큼. 0 이면 책상 앞 끝 가까이(실좌표 y −0.18) 놓은 큐브를 빠질 때 ③→② 에서
                             # 손가락이 건드렸다 (plan_check 3/30). 0.07 은 손목이 안 닿음. 0.04 면 그 해 elbow −133° (한계 135 에 2° 남음)
MAP_SLACK = 0.0003       # m 지도 판정의 책상 여유 덤: hand_roll 만 돌린 근사·중간 프레임 차이 (계획은 덤 없이 TABLE_MARGIN)
MAP_TILTS = (15, 30)        # 지도가 보는 비스듬한 내려가기 (수직에서 °, APPROACH_TILTS 의 앞쪽). 계획은 45·60 도 보지만 지도는 시간 때문에 여기까지
REASON = {1: "손목은 닿지만 손끝(집는 점)을 큐브에 못 맞춤", 0: "팔이 안 닿음"}   # hand_ik.graspcheck 와 같은 말


def _trap(t, T, L, v, acc):
    """사다리꼴 속도(최대 v, 가속 acc, 총 시간 T, 거리 L)로 t 초에 간 거리. 짧으면 삼각형(최대 속도 √(L·acc))."""
    vp = min(v, math.sqrt(L * acc))
    ta = vp / acc
    if t <= 0:
        return 0.0
    if t >= T:
        return L
    if t < ta:
        return 0.5 * acc * t * t
    if t > T - ta:
        r = T - t
        return L - 0.5 * acc * r * r
    return 0.5 * acc * ta * ta + vp * (t - ta)


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
    def __init__(self, ik_dir, ranges, solver=None, setup=None):
        """solver/setup: 지도(hand_ik worker)가 자기 _Solver 와 grasp_setup 결과로 이 계획기의 집기 조건(point_check)을 쓴다."""
        if solver is None:
            import hand_ik
            solver = hand_ik._Solver(str(ik_dir))
        self.s = s = solver
        self.np = np = s.np
        self.S = setup if setup is not None else s.grasp_setup(ranges, 0.0)   # 지도와 같은 한계 (shoulder_yaw 0 고정)
        self.lim = self.S["lim"]
        self.hr = s.arm.index("hand_roll")
        self.open = np.asarray(s.env.finger_targets(0.0), dtype=float)
        self.closed = np.asarray(s.env.finger_targets(1.0), dtype=float)   # 팀 계획이 들고 갈 때 쓰는 값 (grip 1.0)
        self.home = np.asarray(s.E.HOME_ARM_QPOS, dtype=float)
        self.home0 = self.home.copy()
        self.home0[s.arm.index("shoulder_yaw")] = 0.0   # 해석 IK 는 shoulder_yaw 0 고정 → 계획은 HOME 에서 먼저 그것만 0 으로
        self.cube_grip = float(getattr(s.E, "CUBE_GRIP", 1.0))
        # 옮겨 갈 때 손목 높이 하한: 손을 수직으로 내려도 벌린 손가락 끝이 큐브 윗면(5 cm) + CLEAR_CM 위
        self.z_clear = s.table_z + 0.05 + CLEAR_CM / 100 + (s.wrist_above_site - s.finger_below_site)
        self.obj_body = int(s.env.object_body_ids[0])
        self.cube_geom = [g for g in range(s.m.ngeom) if s.m.geom_bodyid[g] == self.obj_body][0]
        self.fgeoms = set(s.finger_geoms)   # 충돌하는 손 상자는 손가락 6 개뿐 (손바닥·서보 메시는 충돌 없음)
        self._home_exit()

    def _home_exit(self):
        """HOME 에서 옮겨 가는 길의 시작 (self.exit = HOME0 부터의 관절 경유점, 끝 = 손목 길(transit)의 시작 자세).
        예전 HOME(손이 책상 앞 끝 위)은 HOME0 에서 바로 transit (손목 2 관절을 HOME 값에서 섞음, exit_fh True).
        HOME = 모든 관절 0 (2026-11-21, 팔이 책상 옆에 곧게 늘어짐 = 팔꿈치 0 특이점) 이면 손목 길로는 못 나간다:
        그 자리에서 손목을 곧게 올리면 shoulder_roll 한계(45°)로 약 20 cm 까지만 오르고(손가락은 아직 책상 면 아래),
        팔꿈치를 곧장 구부려 계획 가지(upper_arm_roll ≈ 180, elbow < 0)로 가면 앞팔이 책상 쪽으로 돌며 손가락이 책상 모서리를
        23 cm 파고든다 (관절 직선 검사). 그래서 관절 경유점 셋 (HOME_EXIT):
          1) upper_arm_roll 0 → 180 만 (팔이 곧아 손이 제자리에서 돈다, 책상 옆)
          2) HOME_EXIT_MID: 어깨를 뒤로(shoulder_roll −40) 젖히며 팔꿈치 −130 + 손목으로 손을 아래로 → 손이 책상 옆에서 위로
          3) 손목 중앙 (HOME_EXIT_WRIST_X, HOME 손목 y, z_clear) 의 해석 IK 해 (계획 가지, 손 최대한 아래) → 거기서 손목 길
        각 구간은 따로 사다리꼴 (모서리에서 멈춤). 이 경유점은 계획기를 만들 때 프레임처럼 촘촘히 검사 (exit_clear_mm)."""
        np, s = self.np, self.s
        ie = s.arm.index("elbow")
        self.exit, self.exit_fh, self.exit_ok, self.exit_clear_mm, self.exit_why = [self.home0.copy()], True, True, None, None
        if abs(float(self.home0[ie])) > math.radians(HOME_STRAIGHT_DEG):
            return
        h1 = self.home0.copy()
        h1[s.arm.index("upper_arm_roll")] = math.radians(HOME_EXIT_MID_DEG["upper_arm_roll"])
        mid = self.home0.copy()
        for n, v in HOME_EXIT_MID_DEG.items():
            mid[s.arm.index(n)] = math.radians(v)
        w = self.wrist(self.home0)
        sols, _ = s.analytic_position(np.array([HOME_EXIT_WRIST_X, w[1], self.z_clear + HOME_EXIT_LIFT]), mid, self.lim)
        sols = [c for c in sols if (c[ie] < 0) == (ELBOW_BRANCH < 0)]
        if not sols:
            self.exit_ok = False
            self.exit_why = "HOME 에서 나가는 경유점(책상 위 손목 자리)을 못 풂"
            return
        e = min(sols, key=lambda c: float(np.abs(c - mid).max()))
        self.exit, self.exit_fh = [self.home0.copy(), h1, mid, e], False
        low = 9.0
        for a, b in zip(self.exit, self.exit[1:]):
            for k in range(81):
                q = a + (b - a) * (k / 80)
                low = min(low, self.finger_low(q, self.open) - s.table_z, self.arm_low(q) - s.table_z + TABLE_MARGIN)
        self.exit_clear_mm = round(low * 1000, 1)
        self.exit_ok = low >= TABLE_MARGIN
        self.exit_why = None if self.exit_ok else f"HOME 에서 나가는 경유점 길에서 손가락이 책상 위 {low * 1000:+.1f} mm (여유 {TABLE_MARGIN * 1000:.0f} mm)"

    def exit_frames(self, fa, sp, back=False):
        """HOME0 ↔ 손목 길 시작 경유점 프레임 (구간마다 따로 사다리꼴). back: 거꾸로 (손목 길 끝 → HOME0)."""
        qs = self.exit[::-1] if back else self.exit
        out = []
        for a, b in zip(qs, qs[1:]):
            out += self.frames([a, b], fa, fa, sp)
        return out

    # ---- 지도(집을 수 있는 곳)의 판정 = 이 계획기가 그 자리에서 집을 수 있나 (2026-11-21, 두 판정이 어긋나지 않게 여기 한 곳)
    def point_check(self, pc):
        """큐브 중심 pc (sim) 에서 이 계획기의 집기(=놓기·빠지기) 조건을 싸게: (됨?, 이유, 잡는 자세 손 기울기 °).
        1) 집는 점 = 큐브 중심 + GRIP_ABOVE_CENTER 인 해 (analytic_tip, 모든 가지 → HOME 팔꿈치 가지 우선 = family).
        2) 그 해에서 집는 점을 곧게 위로 min(ABOVE_CM) 까지 (descent 와 같은 한 걸음 _tip_step, 손 최대한 아래, 같은 가지),
           그 위는 손목을 옮겨 갈 높이 z_clear 까지 (transit 의 마지막 다리, 손목 2 cm 마다).
        3) 큐브 yaw 를 YAW_SAMPLES 로 돌려 가며 모두: hand_roll(±88°, 큐브는 90° 마다 같으니 roll 후보 1~2 개)을 맞춘 그 길에서
           잡는 자세 손가락(벌림·쥠) 책상 위 TABLE_MARGIN 이상, 벌린 손가락이 길 내내 책상 여유 위·큐브에 안 닿음.
           hand_roll 만 바꾸고 다시 풀지 않는다 (손 축 둘레라 집는 점이 3 mm 안으로만 움직임 — 계획은 다시 풀어 정확히 검사).
        큐브는 손이 수직이면 손 축 둘레 대칭이라 yaw 와 상관없고, 기울면 yaw 에 따라 아래쪽 손가락이 달라 모든 yaw 를 본다."""
        np, s, lim = self.np, self.s, self.lim
        if not self.on_table(pc):
            return False, self.off_table_why(), None
        pg = self.grip_target(pc)
        qs, _ = s.analytic_tip(pg, self.S["q0"], lim, "position", tol=0.002)
        qs = [q for q in qs if float(np.linalg.norm(self.grip(q) - pg)) <= TIP_TOL]
        if not qs:
            return False, f"손끝(집는 점)을 큐브 중심 +{GRIP_ABOVE_CENTER * 100:g} cm 에 못 맞춤", None
        qs = self.family(qs)
        tilts = [s.axis_tilt(s.fk(q)[1]) for q in qs]
        why = None
        for q, t in sorted(zip(qs, tilts), key=lambda c: c[1]):
            if t > VERT_TILT or self._roll_best(q) < TABLE_MARGIN + MAP_SLACK:   # 손이 기울면 손가락이 여유 아래 (아래 참고)
                why = why or f"손이 {t:.0f}° 기울어 손가락이 책상 위 {TABLE_MARGIN * 1000:.0f} mm 를 못 지킴"
                continue
            need = set(YAW_SAMPLES)
            for v in self._dirs():
                path, w = self._rise(pg, q, v)
                if path is None:
                    why = why or w
                    continue
                got = self._yaws_pass(pc, path, need)
                if not got:
                    continue
                if not self.exit_ok:
                    return False, self.exit_why, min(tilts)
                t1 = self.transit(self.exit[-1], path[min(len(path) - 1, int(round(min(ABOVE_CM) / 100 / STEP_DESCENT)))], f_home=self.exit_fh)
                if t1 is None:
                    why = why or "HOME 에서 이 자리 위로 옮겨 가는 길이 없음 (팔 가지·도달 범위)"
                    continue
                if any(self.finger_low(q, self.open) < self.s.table_z + TABLE_MARGIN + MAP_SLACK or self.arm_low(q) < self.s.table_z for q in t1):
                    why = why or "HOME 에서 옮겨 오는 길에 손가락이 책상 여유 아래 (어깨 가까운 곳: 손을 아래로 못 돌림)"
                    continue
                need -= got
                if not need:
                    return True, "집을 수 있음", min(tilts)
            if need != set(YAW_SAMPLES):
                why = ("큐브 yaw " + ", ".join(f"{y - s.R.SIM_FROM_REAL_YAW:+.0f}°" for y in sorted(need))
                       + " (실좌표) 에선 곧게 내려가는 길에서 손가락이 큐브·책상 여유에 걸림")
        return False, why or "곧게 내려가는 길 없음", min(tilts)

    def on_table(self, pc):
        """큐브 중심 pc (sim) 가 책상 위에 설 수 있나: 책상 발자국 안쪽으로 반 변 + CUBE_EDGE_IN (env CUBE_X_MAX 와 같은 1 cm).
        2026-11-21 책상을 발자국으로 본 뒤: 예전 지도는 책상 앞 끝(어깨 축 앞 14 cm) 너머 칸도 초록이었다."""
        x0, x1, y0, y1 = self.s.table_xy
        e = float(self.s.E.OBJECT_HALF_SIZE) + CUBE_EDGE_IN
        return bool(x0 + e <= pc[0] <= x1 - e and y0 + e <= pc[1] <= y1 - e)

    def off_table_why(self):
        x1 = self.s.table_xy[1]
        e = float(self.s.E.OBJECT_HALF_SIZE) + CUBE_EDGE_IN
        return (f"큐브가 책상 밖: 책상 앞 끝(어깨 축 앞 {self.s.R.ORIGIN_XY[0] - x1:.2f} m, 가정)에서 {e * 100:.1f} cm 안쪽 = "
                f"실좌표 y ≤ {x1 - e - self.s.R.ORIGIN_XY[0]:+.3f} 이어야")

    def grip_target(self, pc):
        """큐브 중심 pc (sim) → 잡을(놓을) 때 집는 점 목표 = 위로 GRIP_ABOVE_CENTER (sim z = 위)."""
        pg = self.np.array(pc, dtype=float)
        pg[2] += GRIP_ABOVE_CENTER
        return pg

    def _dirs(self):
        """지도가 보는 내려가기 방향: 수직, 그다음 MAP_TILTS 기울기 × 방위 APPROACH_AZ 마다 (계획 approaches 의 앞쪽과 같은 순서)."""
        np = self.np
        out = []
        for al in (0,) + MAP_TILTS:
            for be in (range(0, 360, APPROACH_AZ) if al else (0,)):
                a_, b_ = math.radians(al), math.radians(be)
                out.append(np.array([math.sin(a_) * math.cos(b_), math.sin(a_) * math.sin(b_), math.cos(a_)]))
        return out

    def _roll_best(self, q):
        hr, best = self.hr, -1.0
        for r in self.np.linspace(self.lim[hr, 0], self.lim[hr, 1], 24):
            qq = q.copy(); qq[hr] = r
            best = max(best, self.grasp_clearance(qq))
        return best

    def _rise(self, pg, qg, v):
        """qg 에서 집는 점을 v 방향 곧게 min(ABOVE_CM) 까지 STEP_DESCENT 마다 (descent 의 거꾸로), 그 위 손목을 z_clear 까지
        2 cm 마다 (transit 의 마지막 다리). (자세 목록, None) 또는 (None, 이유)."""
        np, s = self.np, self.s
        path, why = self._tip_path(pg, qg, v, min(ABOVE_CM) / 100)
        if path is None:
            return None, why
        prev = path[-1]
        w = self.wrist(prev)
        n = int(math.ceil((self.z_clear - w[2]) / 0.02))
        for i in range(1, n + 1):
            sols, _ = s.analytic_position(np.array([w[0], w[1], min(self.z_clear, w[2] + i * 0.02)]), prev, self.lim)
            if not sols:
                return None, "큐브 위로 손목을 옮겨 갈 높이까지 못 올림"
            q = self._near(sols, prev)
            if self._jump(q, prev):
                return None, "큐브 위로 손목을 올리면 팔 가지가 바뀜"
            path.append(q)
            prev = q
        return path, None

    def _yaws_pass(self, pc, path, yaws):
        """yaws 중 이 길(path)로 되는 것 (집합)."""
        np, hr, lim = self.np, self.hr, self.lim
        qg = path[0]
        # 이웃 자세 사이 중간점도 (계획의 프레임은 그 사이 관절 보간)
        poses = [qg] + [a + (b - a) * u for a, b in zip(path, path[1:]) for u in (0.5, 1.0)]
        phi = self.hand_yaw(qg)
        good = set()
        for yaw in yaws:
            d = ((yaw - phi + 45) % 90) - 45
            ok = False
            for k in (-2, -1, 0, 1, 2):
                r = qg[hr] + math.radians(d + 90 * k)
                if not (lim[hr, 0] - 1e-9 <= r <= lim[hr, 1] + 1e-9):
                    continue
                qs = []
                for q in poses:
                    qq = q.copy(); qq[hr] = r + (q[hr] - qg[hr])
                    qs.append(qq)
                if self.grasp_clearance(qs[0]) < TABLE_MARGIN + MAP_SLACK:
                    continue
                if any(self.finger_low(q, self.open) < self.s.table_z + TABLE_MARGIN + MAP_SLACK for q in qs):
                    continue
                if self.cube_hits([(q, self.open) for q in qs], range(len(qs)), pc, yaw):
                    continue
                ok = True
                break
            if ok:
                good.add(yaw)
        return good

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

    def approaches(self, p, G, axis):
        """큐브 위 경유점 후보를 좋은 순서로 (게으르게): 잡는 자세 qg(G) 마다 집는 점을 v 방향 곧게 h cm 올린 길 (_tip_path,
        hand_roll 은 qg 그대로 — 지도 point_check 와 같은 걸음). yield {cm, 방향, pairs: [(위 자세 qa, qg, 내려가기 qa→qg)]}.
        순서 점수 = 3 × (v 와 잡는 자세 손 축(axis) 사이 각 °) + 5 × (9 − h cm): 손가락 방향 그대로 위로 빠지는 것을 모든 높이에서
        먼저, 그다음 높은 것. 2026-11-21: 예전엔 위 경유점을 따로(처음부터) 풀어 잡는 자세로 내려갔는데, 손목 관절이 한계에 닿는
        곳에선 따로 푼 해가 없거나 다른 가지라 지도(이어서 올림)와 판정이 달랐다."""
        np = self.np
        opts = []
        for al in APPROACH_TILTS:
            for be in (range(0, 360, APPROACH_AZ) if al else (0,)):
                a_, b_ = math.radians(al), math.radians(be)
                v = np.array([math.sin(a_) * math.cos(b_), math.sin(a_) * math.sin(b_), math.cos(a_)])
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(v @ axis)))))
                for cm in ABOVE_CM:
                    opts.append((3.0 * ang + 5.0 * (ABOVE_CM[0] - cm), cm, al, be, v))   # 손 축 방향(수직) 먼저, 그다음 높은 것
        opts.sort(key=lambda o: o[0])
        for _, cm, al, be, v in opts:
            pairs = []
            for qg in G:
                path, _ = self._tip_path(p, qg, v, cm / 100)
                if path is not None:
                    pairs.append((path[-1], qg, path[::-1]))
            if pairs:
                yield {"cm": cm, "tilt_deg": al, "azimuth_deg": be, "v": v, "pairs": pairs}

    def _tip_path(self, p, qg, v, top):
        """qg(집는 점 = p)에서 집는 점을 v 방향 곧게 top(m) 까지 STEP_DESCENT 마다 (_tip_step, hand_roll 고정). (자세 목록, None) 또는 (None, 이유)."""
        path, prev = [qg], qg
        n = max(1, int(round(top / STEP_DESCENT)))
        for k in range(1, n + 1):
            q = self._tip_step(prev, p + v * (top * k / n), qg[self.hr])
            if q is None:
                return None, f"큐브 위 {top * k / n * 100:.1f} cm 까지 곧게 못 올라감 (같은 팔 가지)"
            path.append(q)
            prev = q
        return path, None

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
        ie = s.arm.index("elbow")
        for w, leg, u in pts:
            b = 0.0 if leg == 0 else 1.0 if leg == 2 else u * u * (3 - 2 * u)
            hold = prev.copy()
            hold[hr] = qf[hr] + (qt[hr] - qf[hr]) * b
            sols, _ = s.analytic_position(w, hold, self.lim)
            # 끝(qt)과 같은 팔꿈치 가지만: HOME(팔꿈치 0, 팔을 쭉 편 특이점)에서 나갈 때 두 가지가 똑같이 가까워 엉뚱한 쪽을 탈 수 있다
            sols = [c for c in sols if (c[ie] < 0) == (qt[ie] < 0)] or sols
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

    def via_home(self, qa, qb, home0=None):
        """곧장 옮겨 갈 길이 없으면 (두 자리가 팔 가지·도달 범위로 갈라져 있음) HOME 쪽을 거쳐서: qa → (손목 길 시작 경유점 self.exit[-1],
        예전 HOME 이면 HOME0) → qb. 없으면 None."""
        hx, fh = self.exit[-1], self.exit_fh
        t1 = self.transit(qa, hx, t_home=fh)
        t2 = self.transit(hx, qb, f_home=fh) if t1 else None
        return t1 + t2[1:] if t1 and t2 else None

    def family(self, cands):
        """ELBOW_BRANCH 쪽 팔꿈치 가지만 (예전 HOME 의 가지 = elbow < 0). 반대 가지로 넘어가려면 팔을 쭉 펴야 해서(특이점)
        공중에서 크게 휘두르게 된다. HOME 이 모든 관절 0 (팔꿈치 0 = 두 가지의 경계) 이 된 2026-11-21 부터 HOME 이 아니라 상수로.
        같은 가지 해가 없으면 그대로 (그때는 길을 못 찾아 관절 직선 → 검사에 걸림)."""
        e = self.s.arm.index("elbow")
        same = [q for q in cands if (q[e] < 0) == (ELBOW_BRANCH < 0)]
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
            q = self._tip_step(prev, p, qa[self.hr] + (qg[self.hr] - qa[self.hr]) * (k / n))
            if q is None:
                return None
            out.append(q)
            prev = q
        if not self._same(out[-1], qg) or float(np.linalg.norm(self.grip(out[-1]) - pg)) > TIP_TOL:
            return None
        out[-1] = qg.copy()
        return out

    def _tip_step(self, prev, p, roll):
        """한 걸음: 집는 점을 p 로 (hand_roll = roll). 손목 목표 = 앞 손목 + (p − 앞 집는 점) 에서 해석 IK(손 최대한 아래) →
        집는 점 오차만큼 손목을 옮겨 다시. 앞 가지에 가장 가까운 해, 튀면 None. descent·지도(_rise) 가 같이 쓴다."""
        np, s = self.np, self.s
        hold = prev.copy()
        hold[self.hr] = roll
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
        # 2026-11-21: 8 번 안에 집는 점이 p 에 안 오면(손목 한계) 버린다. 예전엔 오차를 안 봐서 '큐브 위 9 cm' 가 19 cm 옆인 길을 받아들였다
        if float(np.linalg.norm(p - self.grip(q))) > TIP_TOL:
            return None
        return None if self._jump(q, prev) else q

    def arc(self, qs):
        np = self.np
        return float(sum(np.abs(b - a).max() for a, b in zip(qs, qs[1:])))

    def frames(self, qs, fa, fb, speed_deg_s, min_s=0.0):
        """관절 목록 qs (꺾은선) 를 따라, 손가락 fa → fb. 길이 L = 이웃 사이 관절 최대 이동각의 합(손가락 포함).
        시간은 사다리꼴 (2026-11-21): 속도 ≤ min(speed, √(2·MAX_ACC·TRACK_LAG_DEG)), 가속 ≤ MAX_ACC (실물 real_arm 램프의 max_acc) — 가장 빠른 관절도
        둘 다 안 넘는다 (꺾은선 모서리에선 관절별 방향이 바뀌어 순간 가속이 더 클 수 있다: 1 cm 마다라 작다). min_s 보다 짧으면 늘인다.
        첫 프레임 뺀 (q, f) 목록."""
        np = self.np
        fmove = float(np.abs(fb - fa).max())
        segs = [float(np.abs(b - a).max()) for a, b in zip(qs, qs[1:])]
        L = sum(segs)
        Lt = math.degrees(max(L, fmove))
        acc = MAX_ACC
        v = min(float(speed_deg_s), math.sqrt(2.0 * acc * TRACK_LAG_DEG))   # 실물 램프는 목표에서 멈추게 감속하므로 움직이는 목표를
        #   v²/(2·acc) 만큼 늦게 따라간다 (30 °/s 면 5°) — 늦음을 TRACK_LAG_DEG 안으로: 순항 속도 √(2·acc·lag)
        if Lt >= v * v / acc:
            T = Lt / v + v / acc
        else:
            T = 2.0 * math.sqrt(Lt / acc) if Lt > 0 else 0.0
        k_ = max(1.0, (min_s / T) if T > 0 else 1.0)      # min_s 로 늘이기: 시간 축만 늘림 (속도 1/k, 가속 1/k²)
        Ts = max(T * k_, min_s)
        n = max(1, int(math.ceil(Ts / DT - 1e-9)))
        cum = np.concatenate([[0.0], np.cumsum(segs)])
        out = []
        for k in range(1, n + 1):
            u = _trap(k / n * T, T, Lt, v, acc) / Lt if Lt > 0 and T > 0 else 1.0
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
        return any(self.finger_low(q, f) < self.s.table_z + TABLE_MARGIN or self.arm_low(q) < self.s.table_z for q, f in fr)

    def arm_low(self, q):
        """팔 기준점(팔꿈치·손목, 링크 반지름 hand_ik.ARM_RADIUS 뺌) 중 책상 발자국 위에 있는 것의 가장 낮은 z (sim). 없으면 9.0."""
        self.s.fk(q)
        return self.s.arm_low_z()

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
        if not P.on_table(s.to_sim([x, y, 0.0])):
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {P.off_table_why()}"}
        code, _, _ = s.grasp_eval(P.S, x, y, verify=True)
        if code != 2:
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {REASON[code]}"}
    zc = float(s.E.OBJECT_HALF_SIZE) - R.MAP_D   # 큐브 중심 높이 (실좌표, 어깨 축 기준). 놓을 때도 큐브 바닥 = 책상
    pc = np.asarray(s.to_sim([cx, cy, zc]))      # 큐브 중심 (sim) — 손가락↔큐브 검사·들고 가는 큐브
    pt = np.asarray(s.to_sim([tx, ty, zc]))
    gc, gt = P.grip_target(pc), P.grip_target(pt)   # 집는 점 목표 = 큐브 중심 + GRIP_ABOVE_CENTER (집기·놓기)
    ycube = cyaw + R.SIM_FROM_REAL_YAW        # sim yaw (mod 90 만 쓴다)
    yplace = tyaw + R.SIM_FROM_REAL_YAW
    # 2) 경유점 후보
    G = P.family(P.candidates(gc, ycube))
    Pl = P.family(P.candidates(gt, yplace))
    for name, c in (("큐브를 잡는 자세", G), ("놓는 자세", Pl)):
        if not c:
            return {**out, "status": "unreachable", "reason": f"{name}: 손 방향(yaw)까지 맞는 해 없음"}
    # 집는 점은 그대로(큐브 중심 + GRIP_ABOVE_CENTER), 손가락이 책상 위 TABLE_MARGIN 을 지키는 해만. 없으면 그 자리는 집을 수 없음
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
    hx = P.exit[-1]   # 손목 길(transit)이 시작·끝나는 HOME 쪽 자세 (HOME = 0 이면 책상 위 경유점, 예전 HOME 이면 HOME0)
    if not P.exit_ok:
        return {**out, "status": "unreachable", "reason": P.exit_why}
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
    g_ref = min(G, key=lambda q: float(np.abs(q - hx).max()))   # 손목 길 시작(HOME 쪽) 자세에 가장 가까운 잡는 자세
    p_ref = min(Pl, key=lambda q: float(np.abs(q - hx).max()))
    target_pos = np.array([pt[0], pt[1], pc[2]])

    def pick_part(ap, clean_only):
        """clean_only: 이미 대체(안 깨끗한) 계획이 있으면 길을 못 찾은 후보는 아예 안 푼다 (시간)."""
        best = None
        t1s = [(qa, qg, d1, P.transit(hx, qa, f_home=P.exit_fh)) for qa, qg, d1 in ap["pairs"]]
        if any(x[3] for x in t1s):
            t1s = [x for x in t1s if x[3]]
        elif clean_only:
            return None
        for qa, qg, d1, t1 in t1s:
            c = P.arc(t1 or [hx, qa]) + P.arc(d1) + (0.0 if t1 else FALLBACK_COST) + P.table_penalty(qg)
            if best is None or c < best[0]:
                best = (c, qa, qg, t1, d1)
        if best is None:
            return None
        c, qa, qg, t1, d1 = best
        fr, fb = [(home, O)], []
        if float(np.abs(home - home0).max()) > 0:   # 예전 HOME(shoulder_yaw 6.3°) 만. HOME = 모든 관절 0 이면 같은 자세
            fr += P.frames([home, home0], O, O, sp)
        fr += P.exit_frames(O, sp)   # HOME = 0 이면 관절 경유점 3 개 (Planner._home_exit), 예전 HOME 이면 없음
        i_exit = len(fr) - 1
        fr += P.frames(leg(t1, hx, qa, "HOME→큐브 위", fb), O, O, sp)
        i_above = len(fr) - 1
        fr += P.frames(d1, O, O, sp)
        hit = P.cube_hits(fr, range(len(fr)), pc, ycube)
        bad = bool(hit or fb or P.below_table(fr))
        return {"bad": bad, "c": c, "qa": qa, "qg": qg, "d1": d1, "frames": fr, "above": i_above, "exit": i_exit, "hit": hit, "ap": ap, "fb": fb}

    def place_part(pk, bp, clean_only):
        best = None
        qa = pk["qa"]
        ts = [(qb, qp, d2, P.transit(qa, qb) or P.via_home(qa, qb), P.transit(qb, hx, t_home=P.exit_fh)) for qb, qp, d2 in bp["pairs"]]
        if any(x[3] and x[4] for x in ts):
            ts = [x for x in ts if x[3] and x[4]]
        elif clean_only:
            return None
        for qb, qp, d2, t2, t3 in ts:
            c = (P.arc(t2 or [qa, qb]) + P.arc(d2) + P.arc(t3 or [qb, hx])
                 + (0.0 if t2 else FALLBACK_COST) + (0.0 if t3 else FALLBACK_COST) + P.table_penalty(qp))
            if best is None or c < best[0]:
                best = (c, qb, qp, t2, d2, t3)
        if best is None:
            return None
        c, qb, qp, t2, d2, t3 = best
        qg, d1 = pk["qg"], pk["d1"]
        fr, mk = list(pk["frames"]), {"start": 0, "above_pick": pk["above"], "grasp": len(pk["frames"]) - 1}
        if len(P.exit) > 1:
            mk = {"start": 0, "exit_out": pk["exit"], **{k: v for k, v in mk.items() if k != "start"}}
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
        if len(P.exit) > 1:   # HOME = 0: 손목 길로 나감 경유점까지, 그다음 관절 경유점을 거꾸로 HOME 까지
            add(leg(t3, qb, hx, "놓을 자리 위→HOME", fb), O, O, "exit_back")
            fr.extend(P.exit_frames(O, sp, back=True))
            mk["home0"] = len(fr) - 1
        else:
            add(leg(t3, qb, home0, "놓을 자리 위→HOME", fb), O, O, "home0")
        if float(np.abs(home - home0).max()) > 0:
            add([home0, home], O, O, "home")
        hit = P.cube_hits(fr, range(mk["opened"], len(fr)), target_pos, yplace)
        bad = bool(hit or len(fb) > len(pk["fb"]) or P.below_table(fr[mk["grasp"]:]))
        return {"bad": bad, "c": c + pk["c"], "q": (qa, qg, qb, qp), "frames": fr, "marks": mk, "hit": hit, "bp": bp, "fb": fb}

    ie = s.arm.index("elbow")
    pick_it = P.approaches(gc, G, P.hand_axis(g_ref))
    place_it = P.approaches(gt, Pl, P.hand_axis(p_ref))
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
        return {**out, "status": "unreachable", "reason": f"큐브 위 {min(ABOVE_CM):g}~{max(ABOVE_CM):g} cm 에서 손을 아래로 둔 채 큐브까지 곧게 내려가는 길이 없음"}
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
        return {**out, "status": "unreachable", "reason": f"놓을 자리 위 {min(ABOVE_CM):g}~{max(ABOVE_CM):g} cm 에서 손을 아래로 둔 채 곧게 내려가는 길이 없음"}
    fallback = plan_["fb"]
    ap, bp = pk["ap"], plan_["bp"]
    frames, mk = plan_["frames"], plan_["marks"]
    qa, qg, qb, qp = plan_["q"]
    ha, hb = ap["cm"], bp["cm"]
    labels = {"start": "start (HOME)", "above_pick": f"큐브 위 (+{ha:g} cm, 수직에서 {ap['tilt_deg']}°)", "grasp": "내려감: 큐브 잡는 자세",
              "close": "손가락 닫기", "closed": "기다림", "lift": "들기", "above_place": f"놓을 자리 위 (+{hb:g} cm, 수직에서 {bp['tilt_deg']}°)",
              "place": "내려감: 놓는 자세", "open": "손가락 열기", "opened": "기다림", "lift2": "들기",
              "home0": "HOME (shoulder_yaw 만 0)" if "home" in mk else "HOME", "home": "HOME",
              "exit_out": "HOME 나감 경유점 (팔 돌림 → 어깨 젖히고 팔꿈치 접기 → 책상 위)", "exit_back": "HOME 들어가기 전 경유점 (책상 위)"}
    # 3) 검사: 손가락↔큐브, 손가락↔책상, 관절 한계, 잡는 점 오차, 패드↔큐브 면 각, 들고 갈 때 큐브 바닥 높이
    hit_pick = P.cube_hits(frames, range(0, mk["grasp"] + 1), pc, ycube)
    hit_place = P.cube_hits(frames, range(mk["opened"], len(frames)), target_pos, yplace)
    lim = P.lim
    low = [P.finger_low(q, f) for q, f in frames]
    arm_lo = [P.arm_low(q) for q, _ in frames]   # 팔 기준점(팔꿈치·손목)도 책상 발자국 위에선 책상 면 위 (2026-11-21)
    below = [i for i, z in enumerate(low) if z < s.table_z + TABLE_MARGIN or arm_lo[i] < s.table_z]
    names = list(s.arm) + list(s.fingers)
    traj_q = [list(map(float, q)) + list(map(float, f)) for q, f in frames]
    sy = s.arm.index("shoulder_yaw")   # 0 고정 (HOME 만 6.3°: 그 사이 보간은 범위 안)
    over = [n for k, n in enumerate(s.arm) if k != sy and any(not (lim[k, 0] - 1e-4 <= q[k] <= lim[k, 1] + 1e-4) for q, _ in frames)]
    grip_err = float(np.linalg.norm(P.grip(qg) - gc))
    place_err = float(np.linalg.norm(P.grip(qp) - gt))

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
        problems.append(f"집는 점이 목표(큐브 중심 +{GRIP_ABOVE_CENTER * 100:g} cm)에서 잡을 때 {grip_err * 1000:.1f} mm, 놓을 때 {place_err * 1000:.1f} mm")
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
                       "home_exit": {"waypoints_deg": [[round(math.degrees(float(v)), 1) for v in q] for q in P.exit[1:]],
                                     "lowest_finger_above_table_mm": P.exit_clear_mm},
                       "joint_space_fallback": fallback,
                       "n_candidates": {"above_pick": len(ap["pairs"]), "grasp": len(G), "above_place": len(bp["pairs"]), "place": len(Pl)}},
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
