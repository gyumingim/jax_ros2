"""arm.launch.py with homing_guard between arm_control_node and ak45_node: no AK45 moves until it was
homed in this session, and again after each power cycle of that motor (arm_control/homing.py).

    ros2 launch arm_control arm_homed.launch.py [servo_channel:=can0] [max_speed_deg_s:=30.0] [calibration_file:=...]
    # arm at its home pose (homing.json, default sim 0), then:
    ros2 service call /arm_control/home_all std_srvs/srv/Trigger
    ros2 topic echo /arm_control/homing

arm_control_node is unchanged; only its /ak45/command and /joint_states are remapped to the guard.
play_trajectory / pick_place, run alone, read /joint_states directly: give them the guard's feedback
with  --ros-args -r /joint_states:=/ak45/joint_states_homed  (else their tracking watch compares
against the raw motor angle).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ak45_launch = os.path.join(get_package_share_directory("ak45_ros2"), "launch", "ak45_node.launch.py")
    speed = LaunchConfiguration("max_speed_deg_s")
    cal = LaunchConfiguration("calibration_file")
    return LaunchDescription([
        DeclareLaunchArgument("servo_channel", default_value="can0",
                              description="SocketCAN interface of stm_bridge_node"),
        DeclareLaunchArgument("max_speed_deg_s", default_value="30.0",
                              description="speed limit of every motor command, deg/s (arm_control and the guard)"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="empty: the package's calibration.json"),
        DeclareLaunchArgument("home_file", default_value="",
                              description="empty: the package's homing.json"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ak45_launch)),
        Node(
            package="stm_bridge",
            executable="stm_bridge_node",
            name="stm_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[{"servo_channel": LaunchConfiguration("servo_channel")}],
        ),
        Node(
            package="arm_control",
            executable="homing_guard",
            name="homing_guard",
            output="screen",
            emulate_tty=True,
            parameters=[{"max_speed_deg_s": speed, "calibration_file": cal,
                         "home_file": LaunchConfiguration("home_file")}],
        ),
        Node(
            package="arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[{"max_speed_deg_s": speed, "calibration_file": cal}],
            remappings=[("/ak45/command", "/ak45/command_unguarded"),
                        ("/joint_states", "/ak45/joint_states_homed")],
        ),
    ])
