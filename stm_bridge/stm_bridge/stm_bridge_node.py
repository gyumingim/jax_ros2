"""stm_bridge_node: servo motor angles in, CAN_STM frames out.

    /servo/command  sensor_msgs/JointState, names W_PITCH, F1_1, ... (SERVO_IDS), position = servo
                    MOTOR angle in rad, 0..pi (arm_control converts sim angle -> motor angle)
      -> one CAN frame per joint on servo_channel (standard id 5, see protocol.py)
    /servo/state    same form; position = the angle LAST SENT, not a measurement (servos have no
                    feedback). Not part of /joint_states.

Rules, like ak45_node: nothing is sent before the first command; a message with a length mismatch,
an unknown name, a non-finite value or an angle outside 0..180 deg is dropped whole (arm_control has
already clipped to the servo range, so a value outside it is a bug of the sender).

    /pressure_adc   std_msgs/Float32MultiArray, 6 values: the STM32 pressure frame (standard id 6, 6 bytes,
                    every 100 ms) as byte * 4 / 1023 (see protocol.py). One message per frame received.
                    No /pressure in newtons yet: the ADC -> N conversion is not known.
"""
from math import degrees, isfinite, radians

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from .can_io import ServoCan
from .protocol import pressure_adc, servo_frame
from .servo_names import SERVO_IDS

COMMAND_TOPIC = "/servo/command"
STATE_TOPIC = "/servo/state"
PRESSURE_ADC_TOPIC = "/pressure_adc"
RX_PERIOD_S = 0.01      # how often received frames are read (pressure frames come every 100 ms)
RANGE_SLACK_DEG = 0.05    # float noise of rad -> deg at the ends (0 and 180 deg)


class StmBridgeNode(Node):
    def __init__(self):
        super().__init__("stm_bridge")
        interface = self.declare_parameter("servo_interface", "socketcan").value
        channel = self.declare_parameter("servo_channel", "can0").value
        self.servo = ServoCan(interface, channel)
        self.sent_deg = {}    # name -> angle of the last frame sent
        self.state_pub = self.create_publisher(JointState, STATE_TOPIC, 10)
        self.create_subscription(JointState, COMMAND_TOPIC, self.on_command, 10)
        self.pressure_pub = self.create_publisher(Float32MultiArray, PRESSURE_ADC_TOPIC, 10)
        self.create_timer(RX_PERIOD_S, self.on_rx)
        self.get_logger().info(f"{COMMAND_TOPIC} -> servo frames on {interface} {channel}")

    def on_command(self, msg):
        log = self.get_logger()
        if len(msg.name) != len(msg.position):
            log.warning(f"name and position lengths differ ({len(msg.name)}, {len(msg.position)}); "
                        "message ignored", throttle_duration_sec=5.0)
            return
        for n, p in zip(msg.name, msg.position):
            if n not in SERVO_IDS:
                log.warning(f"unknown servo {n}; message ignored", throttle_duration_sec=5.0)
                return
            if not isfinite(p):
                log.warning(f"non-finite target for {n}; message ignored", throttle_duration_sec=5.0)
                return
            if not -RANGE_SLACK_DEG <= degrees(p) <= 180.0 + RANGE_SLACK_DEG:
                log.warning(f"{n}: {degrees(p):.1f} deg is outside 0..180; message ignored",
                            throttle_duration_sec=5.0)
                return
        for n, p in zip(msg.name, msg.position):
            deg = degrees(p)
            frame = servo_frame(SERVO_IDS[n], deg)
            self.servo.send(frame)
            self.sent_deg[n] = ((frame.data[1] << 8) | frame.data[2]) / 10.0
        self.publish_state()

    def on_rx(self):
        """Reads every waiting frame; pressure frames go to /pressure_adc, others (AK45 replies on the
        same bus) are dropped."""
        while (f := self.servo.recv()) is not None:
            values = pressure_adc(*f)
            if values is not None:
                self.pressure_pub.publish(Float32MultiArray(data=values))

    def publish_state(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        names = sorted(self.sent_deg, key=SERVO_IDS.get)
        state.name = names
        state.position = [radians(self.sent_deg[n]) for n in names]
        self.state_pub.publish(state)

    def destroy_node(self):
        self.servo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
