"""Compatibility wrapper for servo.launch.py.

This file exists for older commands that used servo_control.launch.py. It only
includes dual_arm_servo/launch/servo.launch.py and therefore starts the two
Servo nodes, not the full robot stack.

For normal simulation or hardware use, prefer:
  ros2 launch dual_arm_bringup sim.launch.py mode:=servo
  ros2 launch dual_arm_bringup real.launch.py mode:=servo
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    dual_arm_servo_pkg = get_package_share_directory("dual_arm_servo")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟",
    )
    hw_plugin_arg = DeclareLaunchArgument(
        "hw_plugin",
        default_value="mock_components/GenericSystem",
        description="硬件插件类型",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    hw_plugin = LaunchConfiguration("hw_plugin")

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_servo_pkg, "launch", "servo.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        servo_launch,
    ])
