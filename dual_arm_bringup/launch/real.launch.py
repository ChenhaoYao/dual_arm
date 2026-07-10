"""Top-level real-hardware entry point.

This is the recommended hardware launch. It always starts the shared robot base
stack and RViz with dual_arm_control/DualArmHardware, then starts exactly one
control mode:
  mode:=moveit -> MoveGroup
  mode:=servo  -> two MoveIt Servo nodes

Start dual_arm_soem_bridge separately before enabling motor commands.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def generate_launch_description():
    dual_arm_bringup_pkg = get_package_share_directory("dual_arm_bringup")
    dual_arm_moveit_config_pkg = get_package_share_directory("dual_arm_moveit_config")
    dual_arm_servo_pkg = get_package_share_directory("dual_arm_servo")
    vr_teleop_bridge_pkg = get_package_share_directory("vr_teleop_bridge")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟",
    )
    use_broadcaster_arg = DeclareLaunchArgument(
        "use_broadcaster",
        default_value="false",
        description="是否启动 joint_state_broadcaster",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="moveit",
        choices=["moveit", "servo"],
        description="控制模式: moveit（运动规划）或 servo（实时伺服）",
    )
    enable_vr_teleop_arg = DeclareLaunchArgument(
        "enable_vr_teleop",
        default_value="false",
        description="是否启动 VR 遥操作桥接节点",
    )
    enable_rviz_servo_marker_arg = DeclareLaunchArgument(
        "enable_rviz_servo_marker",
        default_value="false",
        description="真机 Servo 模式下是否启用 RViz 双臂交互 marker",
    )
    vr_teleop_params_file_arg = DeclareLaunchArgument(
        "vr_teleop_params_file",
        default_value=PathJoinSubstitution([
            vr_teleop_bridge_pkg,
            "config",
            "vr_teleop_bridge.yaml",
        ]),
        description="VR teleop bridge 参数文件",
    )
    enable_ros_tcp_endpoint_arg = DeclareLaunchArgument(
        "enable_ros_tcp_endpoint",
        default_value="false",
        description="是否启动 Unity ROS TCP Endpoint",
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port",
        default_value="10000",
        description="Unity ROS TCP Connector 端口",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_broadcaster = LaunchConfiguration("use_broadcaster")
    mode = LaunchConfiguration("mode")
    enable_vr_teleop = LaunchConfiguration("enable_vr_teleop")
    enable_rviz_servo_marker = LaunchConfiguration("enable_rviz_servo_marker")
    vr_teleop_params_file = LaunchConfiguration("vr_teleop_params_file")
    enable_ros_tcp_endpoint = LaunchConfiguration("enable_ros_tcp_endpoint")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")
    hw_plugin = "dual_arm_control/DualArmHardware"

    control_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_bringup_pkg, "launch", "control_base.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
            "use_broadcaster": use_broadcaster,
            "controllers_config": "ros2_controllers_real.yaml",
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_moveit_config_pkg, "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
        }.items(),
        condition=mode_is(mode, "moveit"),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_servo_pkg, "launch", "servo.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
            "enable_rviz_marker": PythonExpression([
                "'", enable_rviz_servo_marker, "' == 'true' and '",
                enable_vr_teleop, "' != 'true'",
            ]),
        }.items(),
        condition=mode_is(mode, "servo"),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_moveit_config_pkg, "launch", "rviz.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
        }.items(),
        condition=mode_is(mode, "moveit"),
    )

    servo_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([dual_arm_moveit_config_pkg, "launch", "rviz.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hw_plugin": hw_plugin,
            "rviz_config": "servo.rviz",
        }.items(),
        condition=mode_is(mode, "servo"),
    )

    vr_teleop_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vr_teleop_bridge_pkg, "launch", "vr_teleop_bridge.launch.py"])
        ),
        launch_arguments={
            "params_file": vr_teleop_params_file,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", enable_vr_teleop, "' == 'true' and '", mode, "' == 'servo'"
        ])),
    )

    vr_move_group_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vr_teleop_bridge_pkg, "launch", "vr_move_group_bridge.launch.py"])
        ),
        condition=IfCondition(PythonExpression([
            "'", enable_vr_teleop, "' == 'true' and '", mode, "' == 'moveit'"
        ])),
    )

    ros_tcp_endpoint_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="UnityEndpoint",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"ROS_IP": "0.0.0.0"},
            {"ROS_TCP_PORT": ros_tcp_port},
        ],
        condition=IfCondition(enable_ros_tcp_endpoint),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_broadcaster_arg,
        mode_arg,
        enable_vr_teleop_arg,
        enable_rviz_servo_marker_arg,
        vr_teleop_params_file_arg,
        enable_ros_tcp_endpoint_arg,
        ros_tcp_port_arg,
        control_base_launch,
        move_group_launch,
        servo_launch,
        moveit_rviz_launch,
        servo_rviz_launch,
        ros_tcp_endpoint_node,
        vr_move_group_bridge_launch,
        vr_teleop_bridge_launch,
    ])
