"""Forward kinematics of the deploy robot (model/robot_v3.urdf = JaxIsaacLab deploy_2026-11-05 robot_v3,
document sign convention), NumPy only. Used by the grasp (deploy README 7.6): the fingertip midpoint and
the hand pose come from the joint TARGETS sent (the real robot has no encoders on the servos).

World placement (deploy README 4): base_link at (0, 0.01, -0.005), turned -90 deg about z
(base x = world -y, base y = world +x), so the shoulder pitch axis is at (0, 0.16, 1.2075).
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

BASE_POS = np.array([0.0, 0.01, -0.005])
BASE_ROT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # Rz(-90 deg)
TIP_LINKS = ("Finger_1_2", "Finger_2_2", "Finger_3_2")
HAND_LINK = "MG996R"


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _axis_rot(axis, t):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(t) * k + (1 - np.cos(t)) * k @ k


class RobotV3:
    def __init__(self, urdf, base_pos=BASE_POS, base_rot=BASE_ROT):
        root = ET.parse(Path(urdf)).getroot()
        self.base_pos, self.base_rot = np.asarray(base_pos, float), np.asarray(base_rot, float)
        self.joints = {}        # child link -> (name, parent, xyz, R0, axis or None)
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in o.get("rpy").split()] if o is not None else [0, 0, 0]
            axis = None
            if j.get("type") != "fixed":
                a = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                axis = a / np.linalg.norm(a)
            self.joints[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"), xyz, _rpy(*rpy), axis)

    def link(self, name, q):
        """(R, p) of a link in the world for joint angles q = {joint name: rad} (missing = 0)."""
        if name not in self.joints:
            return self.base_rot.copy(), self.base_pos.copy()
        jname, parent, xyz, r0, axis = self.joints[name]
        rot, pos = self.link(parent, q)
        pos = pos + rot @ xyz
        rot = rot @ r0
        if axis is not None:
            rot = rot @ _axis_rot(axis, q.get(jname, 0.0))
        return rot, pos

    def fingertip_mid(self, q):
        return np.mean([self.link(n, q)[1] for n in TIP_LINKS], axis=0)

    def hand(self, q):
        return self.link(HAND_LINK, q)

    def point_jacobian(self, q, joints, point, eps=1e-6):
        """6 x len(joints): linear velocity of `point` (world) fixed to the hand, and the hand's angular
        velocity, per joint rate (README 7.6: J_v' = J_v - [r]x J_w at the fingertip midpoint)."""
        rot0, pos0 = self.hand(q)
        local = rot0.T @ (np.asarray(point) - pos0)
        jac = np.zeros((6, len(joints)))
        for i, name in enumerate(joints):
            dq = dict(q)
            dq[name] = q.get(name, 0.0) + eps
            rot1, pos1 = self.hand(dq)
            jac[:3, i] = ((pos1 + rot1 @ local) - (pos0 + rot0 @ local)) / eps
            w = rot1 @ rot0.T
            jac[3:, i] = np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]]) / (2 * eps)
        return jac


def rot_to_quat(r):
    """wxyz of a rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        return np.array([s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
    q = np.zeros(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = s / 4
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def rotation_error(r_want, r_now):
    """README 7.6 w = 2 axis_angle(q0 (x) q_hand^-1): the world rotation vector taking r_now to r_want."""
    e = r_want @ r_now.T
    angle = np.arccos(np.clip((np.trace(e) - 1) / 2, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([e[2, 1] - e[1, 2], e[0, 2] - e[2, 0], e[1, 0] - e[0, 1]]) / (2 * np.sin(angle))
    return 2 * angle * v
