import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包的共享目录路径
    dual_arm_description_pkg = get_package_share_directory('dual_arm_description')
    dual_arm_moveit_config_pkg = get_package_share_directory('dual_arm_moveit_config')

    # ========== 启动参数声明 ==========
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )

    hw_plugin_arg = DeclareLaunchArgument(
        'hw_plugin',
        default_value='mock_components/GenericSystem',
        description='硬件插件类型'
    )

    # 是否启动 joint_state_broadcaster（发布 /joint_states）
    # 使用 soem_bridge 时设为 false，避免 mock 数据覆盖真实编码器数据
    use_broadcaster_arg = DeclareLaunchArgument(
        'use_broadcaster',
        default_value='true',
        description='是否启动 joint_state_broadcaster'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    hw_plugin = LaunchConfiguration('hw_plugin')
    use_broadcaster = LaunchConfiguration('use_broadcaster')

    # ========== 机器人描述 ==========
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([dual_arm_description_pkg, 'urdf', 'dual_arm_1kg.urdf.xacro']),
        ' hw_plugin:=', hw_plugin
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ========== SRDF 语义描述 ==========
    srdf_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'dual_arm_1kg.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # ========== 运动学配置 ==========
    kinematics_yaml_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    # ========== 关节限制配置 ==========
    joint_limits_yaml_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_yaml_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)

    ompl_planning_yaml_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_file, 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)

    # ========== 控制器配置文件 ==========
    ros2_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_yaml, 'r') as f:
        moveit_controllers = yaml.safe_load(f)

    # ========== 节点定义 ==========

    # 节点 1: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 节点 2: ros2_control_node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_controllers_yaml
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # 节点 3: Joint State Broadcaster（可选）
    joint_state_broadcaster_spawner = GroupAction(
        condition=IfCondition(use_broadcaster),
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # 节点 4: Left Arm Controller
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 节点 5: Right Arm Controller
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 节点 6: Move Group (MoveIt2 核心规划节点)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_manage_controllers': True,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'use_sim_time': use_sim_time,
                'trajectory_execution': {
                    'allowed_start_tolerance': 0.05,
                },
                'octomap_frame': '',
                'octomap_resolution': 0.0,
                'octomap_sensor_name': '',
                'octomap_update_interval': 0,
            },
            # OMPL 规划器配置（以嵌套字典形式传递）
            {
                'ompl': ompl_planning_yaml
            },
            moveit_controllers,
        ]
    )

    # 节点 7: RViz2
    rviz_config_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        hw_plugin_arg,
        use_broadcaster_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
