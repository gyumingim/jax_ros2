"""Test stand-in for the STM pressure sensors: a crude hand that touches the cube when a finger's first
joint passes a touch angle and pushes back harder the further it goes. Publishes /pressure (N) and
/pressure_adc (README 6.3 model) at 60 Hz from the finger targets on /arm_control/joint_command.

    python3 fake_hand.py [--touch 6 6 -26] [--stiffness 40] [--none]

--none: no contact ever (pads stay 0). Not a physics model: it only exercises the grasp's rules.
"""
import argparse
from math import log, exp

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

LOADS = [5, 55, 100, 165, 260, 350, 470, 590, 750, 975, 1290, 1585, 1920, 2465, 3240, 4000]
RES = [60, 19.5, 11.5, 8.8, 7.5, 6.8, 5.4, 4.4, 4.0, 3.5, 3.1, 2.9, 2.7, 2.6, 2.55, 2.5]


def adc(f_n):
    gf = f_n * 1000 / 9.80665
    if gf < 5:
        return 0.0
    gf = min(gf, 4000.0)
    for i in range(len(LOADS) - 1):
        if gf <= LOADS[i + 1]:
            t = (log(gf) - log(LOADS[i])) / (log(LOADS[i + 1]) - log(LOADS[i]))
            r = exp(log(RES[i]) + t * (log(RES[i + 1]) - log(RES[i])))
            return round(1023 * 10 / (r + 10)) / 1023
    return round(1023 * 10 / (RES[-1] + 10)) / 1023


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--touch", nargs=3, type=float, default=[6, 6, -26])
    ap.add_argument("--stiffness", type=float, default=40.0, help="N per rad past the touch angle")
    ap.add_argument("--none", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("fake_hand")
    first = {"finger1_joint1": 0.0, "finger2_joint1": 0.0, "finger3_joint1": -0.5236}
    touch = {n: t * 3.141592653589793 / 180 for n, t in zip(first, a.touch)}

    def on_cmd(m):
        for n, p in zip(m.name, m.position):
            if n in first:
                first[n] = p

    node.create_subscription(JointState, "/arm_control/joint_command", on_cmd, 50)
    p_n = node.create_publisher(Float32MultiArray, "/pressure", 10)
    p_adc = node.create_publisher(Float32MultiArray, "/pressure_adc", 10)

    def tick():
        distal = [0.0 if a.none else max(0.0, (first[n] - touch[n]) * a.stiffness) for n in first]
        forces = [0.0, 0.0, 0.0] + distal
        p_n.publish(Float32MultiArray(data=forces))
        p_adc.publish(Float32MultiArray(data=[adc(f) for f in forces]))

    node.create_timer(1 / 60, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
