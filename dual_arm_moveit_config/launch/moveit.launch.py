"""Compatibility wrapper for MoveGroup plus RViz.

This file keeps older MoveIt-only commands working by including
move_group.launch.py and rviz.launch.py.

It does not start ros2_control or arm controllers. For a complete mock or
hardware stack, use:
  ros2 launch dual_arm_bringup sim.launch.py mode:=moveit
  ros2 launch dual_arm_bringup real.launch.py mode:=moveit
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory("dual_arm_moveit_config")

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

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_moveit_config_pkg, "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_moveit_config_pkg, "launch", "rviz.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        move_group_launch,
        rviz_launch,
    ])
