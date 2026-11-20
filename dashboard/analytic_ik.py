"""손목 중앙 위치 해석(공식) IK — 관절 3개(shoulder_roll q1, upper_arm_roll q3, elbow q4), shoulder_yaw 는 고정값.

유도 (축이 서로 안 만나는 오프셋까지 그대로 — 기하는 MuJoCo 모델에서 그때그때 뽑는다, hand_ik._Solver.analytic_position):
  q1 = q3 = q4 = 0 (shoulder_yaw = 고정값, 손목 3 관절 = 지금 값) 자세에서
    o1, a1 = shoulder_roll 축의 점·방향,  o3, a3 = upper_arm_roll,  o4, a4 = elbow,  w0 = 손목 중앙 점.
  회전 Rot(a, t) v = (a.v) a + cos t (v - (a.v) a) + sin t (a x v)  (축이 점 o 를 지나면 o 기준으로).
    h(q4)     = o4 + Rot(a4, q4) (w0 - o4)            팔꿈치만 돌린 손목 중앙
    f(q3, q4) = o3 + Rot(a3, q3) (h - o3)              + 위팔 비틀기
    p         = o1 + Rot(a1, q1) (f - o1)              + 어깨 앞뒤
  q1 둘레 회전으로 안 바뀌는 두 값:  a1.(f - o1) = a1.(p - o1),   |f - o1|^2 = |p - o1|^2.
  u = h - o3, r = o3 - o1 로 두면 k.Rot(a3, q3) u = K0 + Kc cos q3 + Ks sin q3  (K0 = (a3.u)(k.a3), Kc = k.u - K0, Ks = k.(a3 x u)) 라서
    식1: A1 c3 + B1 s3 = G1   (k = a1),       식2: A2 c3 + B2 s3 = G2   (k = r, |u|^2 포함)
  각 계수는 q4 의 1차 삼각식(cos q4, sin q4 의 1차). (c3, s3) 를 소거(c3^2 + s3^2 = 1):
    F(q4) = (G1 B2 - G2 B1)^2 + (A1 G2 - A2 G1)^2 - (A1 B2 - A2 B1)^2 = 0     ← q4 의 4차 삼각 다항식
  → z = e^{i q4} 로 8차 다항식(계수는 16 점 FFT 로 정확히), 근 중 단위원 위의 것이 실제 q4 (일반 3R 위치 문제는 실해 많아야 4 개).
  각 q4 에서 q3 = atan2(Cramer), q1 = 수선 평면 안 각 (하나씩). 즉 해는 많아야 4 개 (+ 범위가 360° 를 넘는 관절은 ±360° 도 별개 해).
  2 x 2 가 특이(A1 B2 - A2 B1 ≈ 0, 손목이 shoulder_roll 축 위 등)면 두 식을 따로 풀어 둘 다 맞는 q3 를 고른다.
  모든 해는 이 사슬로 Newton 몇 번 다듬고(근 오차 제거) 사슬 오차 10 um 안만 남긴다(단위원 근처 복소근 = 가짜 제거). 호출자가 MuJoCo FK 로 다시 검증한다.
"""
import math

import numpy as np


def rot(a, t, v):
    """축 a(단위) 둘레 t 만큼 v 회전 (v 는 (3,) 또는 (n, 3))."""
    c, s = math.cos(t), math.sin(t)
    av = v @ a
    return np.outer(av, a).reshape(v.shape) * (1 - c) + c * v + s * np.cross(a, v)


def fk3(g, q1, q3, q4):
    o1, a1, o3, a3, o4, a4, w0 = g
    h = o4 + rot(a4, q4, w0 - o4)
    f = o3 + rot(a3, q3, h - o3)
    return o1 + rot(a1, q1, f - o1)


def _coef(g, q4s):
    """q4 배열 -> 식1·식2 계수 (A1, B1, G1, A2, B2, G2) 와 함께 쓸 값."""
    o1, a1, o3, a3, o4, a4, w0 = g
    v = w0 - o4
    c, s = np.cos(q4s)[:, None], np.sin(q4s)[:, None]
    av = v @ a4
    h = o4 + av * a4 * (1 - c) + c * v + s * np.cross(a4, v)        # (n, 3)
    u = h - o3
    r = o3 - o1
    au = u @ a3
    x = np.cross(a3, u)
    out = []
    for k in (a1, r):
        k0 = au * (k @ a3)
        out.append((u @ k - k0, x @ k, k0))
    (A1, B1, K01), (A2, B2, K02) = out
    return A1, B1, K01, A2 * 2, B2 * 2, K02 * 2, u


def solve3(g, p, tol_unit=2e-3, newton=8, tol_pos=1e-5):
    """손목 중앙 목표 p -> 모든 (q1, q3, q4) (rad, (-pi, pi]) 와 사슬 오차. 범위 거르기는 호출자가.
    근 찾기는 |z| 가 1 에서 tol_unit 안인 근을 넓게 받고, Newton 으로 다듬은 뒤 사슬 오차 tol_pos(10 um) 안인 것만 해로."""
    o1, a1, o3, a3, o4, a4, w0 = g
    pr = p - o1
    zeta = pr @ a1
    rho2 = pr @ pr
    r = o3 - o1

    def F(q4s):
        A1, B1, K1, A2, B2, K2, u = _coef(g, q4s)
        G1 = zeta - r @ a1 - K1
        G2 = rho2 - r @ r - np.einsum("ij,ij->i", u, u) - K2
        D = A1 * B2 - A2 * B1
        return (G1 * B2 - G2 * B1) ** 2 + (A1 * G2 - A2 * G1) ** 2 - D ** 2, (A1, B1, G1, A2, B2, G2)

    # F 는 q4 의 4차 삼각 다항식 → 16 점 FFT 로 계수(정확), z^4 F = 8차 다항식
    n = 16
    th = 2 * np.pi * np.arange(n) / n
    Fv, _ = F(th)
    C = np.fft.fft(Fv) / n                       # F(t) = sum_k C_k e^{ikt}
    poly = np.array([C[4], C[3], C[2], C[1], C[0], C[n - 1], C[n - 2], C[n - 3], C[n - 4]])
    scale = np.abs(poly).max()
    if scale < 1e-30:
        return []
    roots = np.roots(poly / scale)
    q4c = sorted({round(float(np.angle(z)), 12) for z in roots if abs(abs(z) - 1.0) < tol_unit})
    sols = []
    for q4 in q4c:
        _, (A1, B1, G1, A2, B2, G2) = F(np.array([q4]))
        A1, B1, G1, A2, B2, G2 = (float(x[0]) for x in (A1, B1, G1, A2, B2, G2))
        D = A1 * B2 - A2 * B1
        nrm = max(math.hypot(A1, B1) * math.hypot(A2, B2), 1e-18)
        q3s = []
        if abs(D) > 1e-6 * nrm:
            q3s.append(math.atan2(A1 * G2 - A2 * G1, G1 * B2 - G2 * B1) if D > 0 else math.atan2(-(A1 * G2 - A2 * G1), -(G1 * B2 - G2 * B1)))
        else:   # 두 식이 거의 같은 방향: 각각 풀어 둘 다 맞는 것
            for A, B, G in ((A1, B1, G1), (A2, B2, G2)):
                m = math.hypot(A, B)
                if m < 1e-12:
                    continue
                ph, cc = math.atan2(B, A), max(-1.0, min(1.0, G / m))
                q3s += [ph + math.acos(cc), ph - math.acos(cc)]
        for q3 in q3s:
            f = fk3(g, 0.0, q3, q4)
            v, t = f - o1, pr
            vp, tp = v - (v @ a1) * a1, t - (t @ a1) * a1
            q1 = math.atan2(float(a1 @ np.cross(vp, tp)), float(vp @ tp))
            q = np.array([q1, q3, q4])
            x0 = fk3(g, *q)
            if np.linalg.norm(p - x0) > 0.005:   # 근이 아님(단위원 근처 복소근) — 다듬지 않고 버림
                continue
            for _ in range(newton):      # 근 오차 다듬기 (3 x 3, 수치 자코비안)
                e = p - x0
                if np.linalg.norm(e) < 1e-12:
                    break
                J = np.empty((3, 3))
                for i in range(3):
                    dq = np.zeros(3); dq[i] = 1e-7
                    J[:, i] = (fk3(g, *(q + dq)) - x0) / 1e-7
                q = q + np.linalg.solve(J.T @ J + 1e-12 * np.eye(3), J.T @ e)
                x0 = fk3(g, *q)
            q = (q + np.pi) % (2 * np.pi) - np.pi
            err = float(np.linalg.norm(p - fk3(g, *q)))
            if err > tol_pos:            # 단위원 근처지만 실제 근이 아닌 것(복소 쌍) — 다듬어도 안 맞음
                continue
            if all(np.abs(((q - s[0]) + np.pi) % (2 * np.pi) - np.pi).max() > 1e-5 for s in sols):
                sols.append((q, err))
    return sols
