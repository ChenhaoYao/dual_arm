"""Start only MoveGroup for dual-arm planning mode.

This launch loads the MoveIt planning configuration and starts move_group.

It does not start ros2_control, arm controllers, RViz, MoveIt Servo, VR bridge,
or the Unity TCP endpoint. For a complete mock or hardware stack, use
dual_arm_bringup sim.launch.py/real.launch.py with mode:=moveit.
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

    use_sim_time = LaunchConfiguration("use_sim_time")
    hw_plugin = LaunchConfiguration("hw_plugin")

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

    joint_limits_yaml_file = os.path.join(dual_arm_moveit_config_pkg, "config", "joint_limits.yaml")
    with open(joint_limits_yaml_file, "r") as f:
        joint_limits_yaml = yaml.safe_load(f)

    ompl_planning_yaml_file = os.path.join(dual_arm_moveit_config_pkg, "config", "ompl_planning.yaml")
    with open(ompl_planning_yaml_file, "r") as f:
        ompl_planning_yaml = yaml.safe_load(f)

    moveit_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, "config", "moveit_controllers.yaml")
    with open(moveit_controllers_yaml, "r") as f:
        moveit_controllers = yaml.safe_load(f)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_yaml,
                "robot_description_planning": joint_limits_yaml,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_manage_controllers": True,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "use_sim_time": use_sim_time,
                "trajectory_execution": {
                    "allowed_start_tolerance": 0.05,
                    "allowed_goal_duration_margin": 5.0,
                    "execution_duration_monitoring": True,
                },
                "octomap_frame": "",
                "octomap_resolution": 0.0,
                "octomap_sensor_name": "",
                "octomap_update_interval": 0,
            },
            {"ompl": ompl_planning_yaml},
            moveit_controllers,
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        move_group_node,
    ])
