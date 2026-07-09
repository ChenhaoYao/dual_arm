"""Shared robot base stack used by sim.launch.py and real.launch.py.

Starts robot_state_publisher, ros2_control_node, optional
joint_state_broadcaster, and the left/right arm JointTrajectoryControllers.

It does not start MoveGroup, MoveIt Servo, RViz, VR bridge, or SOEM bridge.
Use this directly only when composing a custom launch; normal users should use
sim.launch.py or real.launch.py.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    use_broadcaster_arg = DeclareLaunchArgument(
        "use_broadcaster",
        default_value="true",
        description="是否启动 joint_state_broadcaster",
    )
    controllers_config_arg = DeclareLaunchArgument(
        "controllers_config",
        default_value="ros2_controllers.yaml",
        description="控制器配置文件名",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    hw_plugin = LaunchConfiguration("hw_plugin")
    use_broadcaster = LaunchConfiguration("use_broadcaster")
    controllers_config = LaunchConfiguration("controllers_config")

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([dual_arm_description_pkg, "urdf", "dual_arm_1kg.urdf.xacro"]),
        " hw_plugin:=",
        hw_plugin,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    ros2_controllers_yaml = PathJoinSubstitution([
        dual_arm_moveit_config_pkg,
        "config",
        controllers_config,
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ros2_controllers_yaml,
        ],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_broadcaster),
    )

    left_arm_controller_spawner_after_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    left_arm_controller_spawner_without_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(use_broadcaster),
    )
    right_arm_controller_spawner_after_left_with_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    right_arm_controller_spawner_after_left_without_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_left_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner_after_broadcaster],
        )
    )
    spawn_right_after_left_with_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller_spawner_after_broadcaster,
            on_exit=[right_arm_controller_spawner_after_left_with_broadcaster],
        )
    )
    spawn_right_after_left_without_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller_spawner_without_broadcaster,
            on_exit=[right_arm_controller_spawner_after_left_without_broadcaster],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        use_broadcaster_arg,
        controllers_config_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner_without_broadcaster,
        spawn_left_after_broadcaster,
        spawn_right_after_left_with_broadcaster,
        spawn_right_after_left_without_broadcaster,
    ])
