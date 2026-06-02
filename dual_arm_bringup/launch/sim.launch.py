from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([dual_arm_moveit_config_pkg, 'launch', 'demo.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hw_plugin': 'mock_components/GenericSystem',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        moveit_demo_launch,
    ])
