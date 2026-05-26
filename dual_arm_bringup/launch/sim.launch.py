import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包的共享目录路径
    dual_arm_description_pkg = get_package_share_directory('dual_arm_description')
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    # 声明启动参数
    # use_sim_time: 是否使用仿真时间（仿真模式下为 true）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟（true 表示使用仿真时间，false 表示使用系统时间）'
    )

    # 获取启动参数配置
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 机器人描述文件（使用仿真硬件插件）
    # 通过 xacro 处理 URDF 文件，并指定硬件插件为 GenericSystem（仿真模式）
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([dual_arm_description_pkg, 'urdf', 'dual_arm_1kg.urdf.xacro']),
        ' hw_plugin:=mock_components/GenericSystem'
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # 包含 MoveIt2 演示启动文件
    # 传入仿真时间和硬件插件参数
    moveit_demo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([dual_arm_moveit_config_pkg, 'launch', 'demo.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hw_plugin': 'mock_components/GenericSystem',
        }.items()
    )

    # 返回启动描述，包含以下节点：
    # 1. use_sim_time 参数声明
    # 2. MoveIt2 演示启动文件（包含 robot_state_publisher、ros2_control、控制器等）
    return LaunchDescription([
        use_sim_time_arg,
        moveit_demo_launch,
    ])
