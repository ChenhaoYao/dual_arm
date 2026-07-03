from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')
    vr_teleop_bridge_pkg = get_package_share_directory('vr_teleop_bridge')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟（有 Gazebo 时设为 true）'
    )

    # 是否启动 joint_state_broadcaster（发布 /joint_states）
    # 使用 soem_bridge 时设为 false，避免 mock 数据覆盖真实编码器数据
    use_broadcaster_arg = DeclareLaunchArgument(
        'use_broadcaster',
        default_value='true',
        description='是否启动 joint_state_broadcaster'
    )

    # 控制模式：moveit（规划）或 servo（实时）
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='moveit',
        description='控制模式: moveit（运动规划）或 servo（实时伺服）'
    )

    enable_vr_teleop_arg = DeclareLaunchArgument(
        'enable_vr_teleop',
        default_value='false',
        description='servo 模式下是否启动 VR 到 MoveIt Servo 的桥接节点'
    )

    vr_teleop_params_file_arg = DeclareLaunchArgument(
        'vr_teleop_params_file',
        default_value=PathJoinSubstitution([
            vr_teleop_bridge_pkg,
            'config',
            'vr_teleop_bridge.yaml',
        ]),
        description='VR teleop bridge 参数文件'
    )

    enable_ros_tcp_endpoint_arg = DeclareLaunchArgument(
        'enable_ros_tcp_endpoint',
        default_value='false',
        description='是否启动 Unity ROS TCP Endpoint'
    )

    ros_tcp_port_arg = DeclareLaunchArgument(
        'ros_tcp_port',
        default_value='10000',
        description='Unity ROS TCP Connector 端口'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_broadcaster = LaunchConfiguration('use_broadcaster')
    mode = LaunchConfiguration('mode')
    enable_vr_teleop = LaunchConfiguration('enable_vr_teleop')
    vr_teleop_params_file = LaunchConfiguration('vr_teleop_params_file')
    enable_ros_tcp_endpoint = LaunchConfiguration('enable_ros_tcp_endpoint')
    ros_tcp_port = LaunchConfiguration('ros_tcp_port')

    moveit_demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([dual_arm_moveit_config_pkg, 'launch', 'moveit.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hw_plugin': 'mock_components/GenericSystem',
            'use_broadcaster': use_broadcaster,
            'controllers_config': 'ros2_controllers.yaml',
            'mode': mode,
        }.items()
    )

    vr_teleop_bridge_launch = IncludeLaunchDescription(
        PathJoinSubstitution([vr_teleop_bridge_pkg, 'launch', 'vr_teleop_bridge.launch.py']),
        launch_arguments={
            'params_file': vr_teleop_params_file,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", enable_vr_teleop, "' == 'true' and '", mode, "' == 'servo'"
        ])),
    )

    ros_tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='UnityEndpoint',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'ROS_IP': '0.0.0.0'},
            {'ROS_TCP_PORT': ros_tcp_port},
        ],
        condition=IfCondition(enable_ros_tcp_endpoint),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_broadcaster_arg,
        mode_arg,
        enable_vr_teleop_arg,
        vr_teleop_params_file_arg,
        enable_ros_tcp_endpoint_arg,
        ros_tcp_port_arg,
        moveit_demo_launch,
        ros_tcp_endpoint_node,
        vr_teleop_bridge_launch,
    ])
