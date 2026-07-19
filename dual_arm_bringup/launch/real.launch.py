"""Top-level real-hardware entry point.

This is the recommended hardware launch. It starts the shared robot base stack
with dual_arm_control/DualArmHardware, then starts exactly one control mode and
its RViz configuration:
  mode:=moveit -> MoveGroup
  mode:=servo  -> two MoveIt Servo nodes

Start dual_arm_soem_bridge separately before enabling motor commands.
When move_to_ready_on_start:=true, the selected mode is held back until the
guarded, one-shot ready motion succeeds.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def mode_is_without_ready(mode, expected, move_to_ready):
    return IfCondition(PythonExpression([
        "'", mode, "' == '", expected, "' and '",
        move_to_ready, "' != 'true'",
    ]))


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
    move_to_ready_on_start_arg = DeclareLaunchArgument(
        "move_to_ready_on_start",
        default_value="false",
        description=(
            "实物启动时是否自动执行镜像预备位；会自动打开 SOEM 命令门并产生运动"
        ),
    )
    ready_pose_params_file_arg = DeclareLaunchArgument(
        "ready_pose_params_file",
        default_value=PathJoinSubstitution([
            dual_arm_bringup_pkg,
            "config",
            "ready_pose.yaml",
        ]),
        description="实物镜像预备位和安全检查参数文件",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_broadcaster = LaunchConfiguration("use_broadcaster")
    mode = LaunchConfiguration("mode")
    enable_vr_teleop = LaunchConfiguration("enable_vr_teleop")
    enable_rviz_servo_marker = LaunchConfiguration("enable_rviz_servo_marker")
    vr_teleop_params_file = LaunchConfiguration("vr_teleop_params_file")
    enable_ros_tcp_endpoint = LaunchConfiguration("enable_ros_tcp_endpoint")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")
    move_to_ready_on_start = LaunchConfiguration("move_to_ready_on_start")
    ready_pose_params_file = LaunchConfiguration("ready_pose_params_file")
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

    def create_mode_actions(wait_for_ready):
        """
        Build an independent set of mode-specific actions:
        - The normal set starts immediately only when ready motion is disabled.
        - The deferred set is returned only after the ready process exits cleanly.
        - Command-producing Servo/VR nodes therefore cannot race the JTC ready goal.
        """
        if wait_for_ready:
            moveit_condition = mode_is(mode, "moveit")
            servo_condition = mode_is(mode, "servo")
            endpoint_condition = IfCondition(enable_ros_tcp_endpoint)
            vr_servo_condition = IfCondition(PythonExpression([
                "'", enable_vr_teleop, "' == 'true' and '", mode, "' == 'servo'"
            ]))
            vr_moveit_condition = IfCondition(PythonExpression([
                "'", enable_vr_teleop, "' == 'true' and '", mode, "' == 'moveit'"
            ]))
        else:
            moveit_condition = mode_is_without_ready(
                mode, "moveit", move_to_ready_on_start
            )
            servo_condition = mode_is_without_ready(
                mode, "servo", move_to_ready_on_start
            )
            endpoint_condition = IfCondition(PythonExpression([
                "'", enable_ros_tcp_endpoint, "' == 'true' and '",
                move_to_ready_on_start, "' != 'true'",
            ]))
            vr_servo_condition = IfCondition(PythonExpression([
                "'", enable_vr_teleop, "' == 'true' and '", mode,
                "' == 'servo' and '", move_to_ready_on_start, "' != 'true'",
            ]))
            vr_moveit_condition = IfCondition(PythonExpression([
                "'", enable_vr_teleop, "' == 'true' and '", mode,
                "' == 'moveit' and '", move_to_ready_on_start, "' != 'true'",
            ]))

        move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                dual_arm_moveit_config_pkg, "launch", "move_group.launch.py"
            ])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hw_plugin": hw_plugin,
            }.items(),
            condition=moveit_condition,
        )
        servo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                dual_arm_servo_pkg, "launch", "servo.launch.py"
            ])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hw_plugin": hw_plugin,
                "enable_rviz_marker": PythonExpression([
                    "'", enable_rviz_servo_marker, "' == 'true' and '",
                    enable_vr_teleop, "' != 'true'",
                ]),
            }.items(),
            condition=servo_condition,
        )
        moveit_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                dual_arm_moveit_config_pkg, "launch", "rviz.launch.py"
            ])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hw_plugin": hw_plugin,
            }.items(),
            condition=moveit_condition,
        )
        servo_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                dual_arm_moveit_config_pkg, "launch", "rviz.launch.py"
            ])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hw_plugin": hw_plugin,
                "rviz_config": "servo.rviz",
            }.items(),
            condition=servo_condition,
        )
        vr_servo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                vr_teleop_bridge_pkg, "launch", "vr_teleop_bridge.launch.py"
            ])),
            launch_arguments={"params_file": vr_teleop_params_file}.items(),
            condition=vr_servo_condition,
        )
        vr_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                vr_teleop_bridge_pkg, "launch", "vr_move_group_bridge.launch.py"
            ])),
            condition=vr_moveit_condition,
        )
        endpoint = Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="UnityEndpoint",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"ROS_IP": "0.0.0.0"},
                {"ROS_TCP_PORT": ros_tcp_port},
            ],
            condition=endpoint_condition,
        )
        return [
            move_group,
            servo,
            moveit_rviz,
            servo_rviz,
            endpoint,
            vr_moveit,
            vr_servo,
        ]

    normal_mode_actions = create_mode_actions(wait_for_ready=False)
    deferred_mode_actions = create_mode_actions(wait_for_ready=True)

    ready_node = Node(
        package="dual_arm_bringup",
        executable="move_to_ready.py",
        name="move_to_ready_node",
        output="screen",
        parameters=[ready_pose_params_file],
        condition=IfCondition(move_to_ready_on_start),
    )

    def after_ready_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    "[ERROR] Ready motion failed; Servo/MoveGroup/VR will not "
                    "start. The real-hardware stack is shutting down."
                )),
                EmitEvent(event=Shutdown(reason="ready motion failed")),
            ]
        return [
            LogInfo(msg="Ready motion succeeded; starting the selected control mode"),
            *deferred_mode_actions,
        ]

    start_mode_after_ready = RegisterEventHandler(
        OnProcessExit(target_action=ready_node, on_exit=after_ready_exit),
        condition=IfCondition(move_to_ready_on_start),
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
        move_to_ready_on_start_arg,
        ready_pose_params_file_arg,
        control_base_launch,
        start_mode_after_ready,
        ready_node,
        *normal_mode_actions,
    ])
