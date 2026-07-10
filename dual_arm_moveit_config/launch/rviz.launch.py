"""Start only RViz with the shared MoveIt RViz config.

This launch loads robot_description, SRDF, and kinematics for RViz. The
rviz_config argument selects a config installed under dual_arm_moveit_config/config.

It does not start MoveGroup, MoveIt Servo, ros2_control, or arm controllers.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dual_arm_description_pkg = get_package_share_directory("dual_arm_description")
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
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="dual_arm_moveit_config/config 下的 RViz 配置文件",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    hw_plugin = LaunchConfiguration("hw_plugin")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([dual_arm_description_pkg, "urdf", "dual_arm_1kg.urdf.xacro"]),
        " hw_plugin:=",
        hw_plugin,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    srdf_file = os.path.join(dual_arm_moveit_config_pkg, "config", "dual_arm_1kg.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = f.read()

    kinematics_yaml_file = os.path.join(dual_arm_moveit_config_pkg, "config", "kinematics.yaml")
    with open(kinematics_yaml_file, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    rviz_config_file = PathJoinSubstitution([
        dual_arm_moveit_config_pkg,
        "config",
        rviz_config,
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{
            "robot_description": robot_description,
            "robot_description_semantic": robot_description_semantic,
            "robot_description_kinematics": kinematics_yaml,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        rviz_config_arg,
        rviz_node,
    ])
