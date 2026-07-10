"""Start only the two MoveIt Servo nodes.

This launch loads the robot description, MoveIt kinematics, and
dual_arm_servo/config/servo_left.yaml + servo_right.yaml, then starts
/servo_left and /servo_right.

It does not start ros2_control, arm controllers, RViz, MoveGroup, VR bridge, or
the Unity TCP endpoint. For normal simulation or hardware use, prefer:
  ros2 launch dual_arm_bringup sim.launch.py mode:=servo
  ros2 launch dual_arm_bringup real.launch.py mode:=servo
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dual_arm_description_pkg = get_package_share_directory("dual_arm_description")
    dual_arm_moveit_config_pkg = get_package_share_directory("dual_arm_moveit_config")
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
    enable_rviz_marker_arg = DeclareLaunchArgument(
        "enable_rviz_marker",
        default_value="true",
        description="启动 RViz Interactive Marker 到 Servo 的输入适配器",
    )
    startup_delay_arg = DeclareLaunchArgument(
        "startup_delay",
        default_value="3.0",
        description="等待 ros2_control 控制器和 joint_states 稳定的秒数",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    hw_plugin = LaunchConfiguration("hw_plugin")
    enable_rviz_marker = LaunchConfiguration("enable_rviz_marker")
    startup_delay = LaunchConfiguration("startup_delay")

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

    # Servo needs an IK solver whose tolerance is smaller than one control-cycle
    # Cartesian step. MoveGroup keeps using its separate PickIK configuration.
    kinematics_yaml_file = os.path.join(dual_arm_servo_pkg, "config", "kinematics.yaml")
    with open(kinematics_yaml_file, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    servo_left_yaml_file = os.path.join(dual_arm_servo_pkg, "config", "servo_left.yaml")
    with open(servo_left_yaml_file, "r") as f:
        servo_left_params = {"moveit_servo": yaml.safe_load(f)}

    servo_right_yaml_file = os.path.join(dual_arm_servo_pkg, "config", "servo_right.yaml")
    with open(servo_right_yaml_file, "r") as f:
        servo_right_params = {"moveit_servo": yaml.safe_load(f)}

    common_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics_yaml,
        "use_sim_time": use_sim_time,
    }
    servo_left_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_left",
        parameters=[servo_left_params, common_params],
        remappings=[
            ("/get_planning_scene", "/servo_left/get_planning_scene"),
        ],
        output="screen",
    )
    servo_right_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_right",
        parameters=[servo_right_params, common_params],
        remappings=[
            ("/get_planning_scene", "/servo_right/get_planning_scene"),
        ],
        output="screen",
    )

    marker_params_file = os.path.join(
        dual_arm_servo_pkg, "config", "rviz_servo_marker.yaml"
    )
    rviz_servo_marker_node = Node(
        package="dual_arm_servo",
        executable="rviz_servo_marker",
        name="rviz_servo_marker",
        output="screen",
        parameters=[marker_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_rviz_marker),
    )
    delayed_servo_stack = TimerAction(
        period=startup_delay,
        actions=[servo_left_node, servo_right_node, rviz_servo_marker_node],
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        enable_rviz_marker_arg,
        startup_delay_arg,
        delayed_servo_stack,
    ])
