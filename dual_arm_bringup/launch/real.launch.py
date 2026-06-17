from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟'
    )

    # 是否启动 joint_state_broadcaster（发布 /joint_states）
    # 使用 soem_bridge 时设为 false，避免与真实编码器数据冲突
    use_broadcaster_arg = DeclareLaunchArgument(
        'use_broadcaster',
        default_value='false',
        description='是否启动 joint_state_broadcaster'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_broadcaster = LaunchConfiguration('use_broadcaster')

    moveit_demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([dual_arm_moveit_config_pkg, 'launch', 'moveit.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hw_plugin': 'dual_arm_control/DualArmHardware',
            'use_broadcaster': use_broadcaster,
            'controllers_config': 'ros2_controllers_real.yaml',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_broadcaster_arg,
        moveit_demo_launch,
    ])
