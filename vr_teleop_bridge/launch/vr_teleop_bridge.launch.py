"""Start only the VR-to-MoveIt-Servo bridge node.

This launch converts Unity/PICO controller poses into MoveIt Servo twist
commands. It expects MoveIt Servo to already be running, normally through:
  ros2 launch dual_arm_bringup sim.launch.py mode:=servo enable_vr_teleop:=true

It does not start MoveIt Servo, MoveGroup, ros2_control, RViz, or the Unity TCP
endpoint.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    vr_teleop_bridge_pkg = get_package_share_directory("vr_teleop_bridge")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            vr_teleop_bridge_pkg,
            "config",
            "vr_teleop_bridge.yaml",
        ]),
        description="VR teleop bridge parameter file",
    )

    node = Node(
        package="vr_teleop_bridge",
        executable="vr_pose_to_servo_node.py",
        name="vr_pose_to_servo_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_arg,
        node,
    ])
