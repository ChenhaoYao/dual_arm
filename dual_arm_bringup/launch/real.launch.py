import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    dual_arm_description_pkg = get_package_share_directory('dual_arm_description')
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description with xacro (using real hardware plugin)
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([dual_arm_description_pkg, 'urdf', 'dual_arm_1kg.urdf.xacro']),
        ' hw_plugin:=dual_arm_control/DualArmHardware'
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Include MoveIt2 demo launch
    moveit_demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([dual_arm_moveit_config_pkg, 'launch', 'demo.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hw_plugin': 'dual_arm_control/DualArmHardware',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        moveit_demo_launch,
    ])
