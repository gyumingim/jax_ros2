"""Camera (or a fixed test pose) -> model approach -> motors: arm.launch.py + vision_node + approach_node.

    ros2 launch arm_control approach.launch.py result_json:=<StereoDepthSeg --out>/inference.json
    ros2 launch arm_control approach.launch.py fixed_pose:="[0.10, -0.30, 20.0]"      # no camera
    ros2 service call /approach/start std_srvs/srv/Trigger       # moves the arm
    ros2 service call /approach/stop  std_srvs/srv/Trigger

Exactly one of result_json (camera mode) and fixed_pose ([x m, y m, yaw deg] in shoulder_roll_axis) must
be given. camera_to_robot: empty = vision_node/config/camera.yaml (the value kept in the repo), or
"[tx, ty, tz, roll_deg, pitch_deg, yaw_deg]" to override it. A fixed pose has z 0, so /approach/start
refuses it (table height check, max_z_error_m): fixed_pose is for checking the bring-up, not for moving.
tools/bringup.sh stops old processes and resets can0 before this launch (BRINGUP.md).
Do not run calibrate --ros alongside.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def val(name):
        return LaunchConfiguration(name).perform(context).strip()

    def parsed(name):
        return yaml.safe_load(val(name))

    result_json, fixed_pose, c2r = val("result_json"), val("fixed_pose"), val("camera_to_robot")
    if bool(result_json) == bool(fixed_pose):
        raise RuntimeError("give exactly one of result_json:=<inference.json> and fixed_pose:=\"[x, y, yaw_deg]\"")
    vision_params = []
    if result_json:
        vision_params.append(os.path.join(get_package_share_directory("vision_node"), "config", "camera.yaml"))
        vision = {"result_json": result_json}
        if c2r:
            vision["camera_to_robot"] = [float(v) for v in yaml.safe_load(c2r)]
    else:
        pose = yaml.safe_load(fixed_pose)
        if not (isinstance(pose, list) and len(pose) == 3):
            raise RuntimeError(f"fixed_pose must be [x, y, yaw_deg], got {fixed_pose!r}")
        vision = {"fixed_pose": [float(v) for v in pose]}
    vision_params.append(vision)
    return [
        Node(package="vision_node", executable="vision_node", name="vision_node", output="screen",
             emulate_tty=True, parameters=vision_params),
        Node(package="arm_control", executable="approach_node", name="approach_node", output="screen",
             emulate_tty=True,
             parameters=[{"force": bool(parsed("force")), "send_finger_rotation": bool(parsed("send_finger_rotation")),
                          "max_z_error_m": float(parsed("max_z_error_m")), "table_e": float(parsed("table_e")),
                          "rate_hz": float(parsed("rate_hz")), "calibration_file": val("calibration_file")}]),
    ]


def generate_launch_description():
    arm_launch = os.path.join(get_package_share_directory("arm_control"), "launch", "arm.launch.py")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("result_json", default_value="", description="the camera program's inference.json"),
        DeclareLaunchArgument("fixed_pose", default_value="", description="[x, y, yaw_deg] test pose instead of the camera"),
        DeclareLaunchArgument("camera_to_robot", default_value="",
                              description="empty: vision_node/config/camera.yaml; or [tx, ty, tz, roll, pitch, yaw]"),
        DeclareLaunchArgument("force", default_value="false",
                              description="start despite unverified calibration (never a pose problem)"),
        DeclareLaunchArgument("send_finger_rotation", default_value="false",
                              description="command finger_rotation (needs its calibration first)"),
        DeclareLaunchArgument("max_z_error_m", default_value="0.05", description="approach_node: allowed cube z error"),
        DeclareLaunchArgument("table_e", default_value="0.0", description="approach_node: table offset"),
        DeclareLaunchArgument("rate_hz", default_value="60.0", description="approach_node: command rate"),
        DeclareLaunchArgument("servo_channel", default_value="can0"),
        DeclareLaunchArgument("max_speed_deg_s", default_value="30.0"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(arm_launch), launch_arguments={
            "servo_channel": arg("servo_channel"), "max_speed_deg_s": arg("max_speed_deg_s"),
            "calibration_file": arg("calibration_file")}.items()),
        OpaqueFunction(function=nodes),
    ])
