"""servo_control.launch.py — 独立 Servo 启动文件

启动 moveit servo 模式。
用法:
    ros2 launch dual_arm_servo servo_control.launch.py
    ros2 launch dual_arm_servo servo_control.launch.py use_broadcaster:=false  # 实物
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    use_broadcaster_arg = DeclareLaunchArgument(
        'use_broadcaster', default_value='true',
        description='是否启动 joint_state_broadcaster'
    )
    use_broadcaster = LaunchConfiguration('use_broadcaster')

    moveit_servo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            dual_arm_moveit_config_pkg, 'launch', 'moveit.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'hw_plugin': 'mock_components/GenericSystem',
            'use_broadcaster': use_broadcaster,
            'controllers_config': 'ros2_controllers.yaml',
            'mode': 'servo',
        }.items()
    )

    return LaunchDescription([
        use_broadcaster_arg,
        moveit_servo_launch,
    ])
