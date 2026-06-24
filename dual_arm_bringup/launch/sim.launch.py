from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟（有Gazebo时设为true）'
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_broadcaster = LaunchConfiguration('use_broadcaster')
    mode = LaunchConfiguration('mode')

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

    return LaunchDescription([
        use_sim_time_arg,
        use_broadcaster_arg,
        mode_arg,
        moveit_demo_launch,
    ])
