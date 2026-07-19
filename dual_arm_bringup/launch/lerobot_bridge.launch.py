"""Start the standalone dual_arm <-> LeRobot ZMQ data bridge.

This launch file never starts ros2_control, MoveIt, Servo, SOEM, or a command
gate.  Start the already-tested sim/real stack separately, then add this bridge
for recording.  The default YAML still publishes observations, JTC-reference
demonstration labels, and cameras; it disables only incoming trained-policy
commands.  Its ~/enable_actions service is not the SOEM ~/enable motor gate.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("dual_arm_bringup")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [package_share, "config", "lerobot_bridge.yaml"]
        ),
        description="LeRobot ZMQ bridge parameter file",
    )
    bridge = Node(
        package="dual_arm_bringup",
        executable="zmq_bridge_node.py",
        name="zmq_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )
    return LaunchDescription([params_file_arg, bridge])
