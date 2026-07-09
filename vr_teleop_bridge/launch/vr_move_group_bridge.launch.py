"""Start only the VR-to-MoveGroup bridge node.

This launch converts relative Unity/PICO controller motion into sparse
MoveGroup pose goals. It expects move_group to already be running, normally via
dual_arm_bringup sim.launch.py/real.launch.py with mode:=moveit.

It does not start MoveGroup, MoveIt Servo, ros2_control, RViz, or the Unity TCP
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
            "vr_move_group_bridge.yaml",
        ]),
        description="VR relative-pose to MoveGroup parameter file",
    )

    node = Node(
        package="vr_teleop_bridge",
        executable="vr_pose_to_move_group_node.py",
        name="vr_pose_to_move_group_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_arg,
        node,
    ])
