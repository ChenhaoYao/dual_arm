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
        default_value='true',
        description='Use simulation clock if true'
    )

    hw_plugin_arg = DeclareLaunchArgument(
        'hw_plugin',
        default_value='mock_components/GenericSystem',
        description='Hardware plugin to use'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    hw_plugin = LaunchConfiguration('hw_plugin')

    # Robot description with xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([dual_arm_description_pkg, 'urdf', 'dual_arm_1kg.urdf.xacro']),
        ' hw_plugin:=', hw_plugin
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # SRDF content
    srdf_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'dual_arm_1kg.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Kinematics config
    kinematics_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'kinematics.yaml')

    # Joint limits config
    joint_limits_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'joint_limits.yaml')

    # Controllers config
    ros2_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'moveit_controllers.yaml')

    # Robot state publisher
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

    # ros2_control node
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

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # MoveGroup node
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
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': moveit_controllers_yaml,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # RViz2 with MoveIt plugin
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
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
