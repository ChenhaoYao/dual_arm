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

    # ========== 启动参数声明 ==========
    # use_sim_time: 是否使用仿真时间（true 使用 Gazebo 仿真时钟，false 使用系统时间）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟（true 表示使用仿真时间，false 表示使用系统时间）'
    )

    # hw_plugin: 硬件接口插件名称
    # - mock_components/GenericSystem: 仿真模式（默认）
    # - dual_arm_control/DualArmHardware: 真实硬件模式
    hw_plugin_arg = DeclareLaunchArgument(
        'hw_plugin',
        default_value='mock_components/GenericSystem',
        description='硬件插件类型（mock_components/GenericSystem 或 dual_arm_control/DualArmHardware）'
    )

    # 获取启动参数配置
    use_sim_time = LaunchConfiguration('use_sim_time')
    hw_plugin = LaunchConfiguration('hw_plugin')

    # ========== 机器人描述 ==========
    # 通过 xacro 处理 URDF 文件，生成完整的机器人描述
    # 包含：链接、关节、碰撞体、视觉模型、ros2_control 硬件接口配置
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([dual_arm_description_pkg, 'urdf', 'dual_arm_1kg.urdf.xacro']),
        ' hw_plugin:=', hw_plugin
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ========== SRDF 语义描述 ==========
    # SRDF (Semantic Robot Description Format) 定义：
    # - 规划组（左臂、右臂、双臂组合）
    # - 末端执行器
    # - 预定义姿态（home 等）
    # - 碰撞检测禁用对（相邻链接、左右臂之间）
    srdf_file = os.path.join(dual_arm_moveit_config_pkg, 'config', 'dual_arm_1kg.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # ========== 运动学配置 ==========
    # KDL 运动学求解器配置：
    # - 求解器类型：KDL（运动学动力学库）
    # - 搜索分辨率：0.005 rad
    # - 超时时间：0.05 秒
    # - 最大尝试次数：3 次
    kinematics_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'kinematics.yaml')

    # ========== 关节限制配置 ==========
    # 关节物理限制：
    # - 速度限制：2.0 rad/s
    # - 力矩限制：20.0 Nm
    # - 位置限制：±3.14 rad（±180°）
    joint_limits_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'joint_limits.yaml')

    # ========== 控制器配置 ==========
    # ros2_controllers.yaml: ros2_control 控制器配置
    # - controller_manager: 控制器管理器（100 Hz 更新频率）
    # - joint_state_broadcaster: 关节状态广播器
    # - left_arm_controller: 左臂 JointTrajectoryController（位置控制）
    # - right_arm_controller: 右臂 JointTrajectoryController（位置控制）
    ros2_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'ros2_controllers.yaml')

    # moveit_controllers.yaml: MoveIt 控制器映射
    # - 将 MoveIt 的运动规划请求映射到 ros2_control 控制器
    # - 定义 follow_joint_trajectory 动作接口
    moveit_controllers_yaml = os.path.join(dual_arm_moveit_config_pkg, 'config', 'moveit_controllers.yaml')

    # ========== 节点定义 ==========

    # 节点 1: Robot State Publisher（机器人状态发布器）
    # - 订阅: /robot_description (URDF)
    # - 发布: /tf (关节变换), /robot_description
    # - 功能: 根据关节状态计算并发布每个链接的位置
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

    # 节点 2: ros2_control_node（控制器管理器）
    # - 包: controller_manager
    # - 功能: 加载和管理所有硬件接口和控制器
    # - 硬件接口: 根据 hw_plugin 参数加载仿真或真实硬件插件
    # - 控制器: 根据 ros2_controllers.yaml 配置加载控制器
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

    # 节点 3: Joint State Broadcaster（关节状态广播器）
    # - 功能: 从硬件接口读取关节状态并发布到 /joint_states
    # - 发布: /joint_states (sensor_msgs/JointState)
    # - 用途: rviz2 显示、MoveIt 运动规划
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 节点 4: Left Arm Controller（左臂控制器）
    # - 类型: JointTrajectoryController
    # - 功能: 接收轨迹点并控制左臂 7 个关节运动
    # - 关节: laxis1_joint ~ laxis7_joint
    # - 接口: 位置控制（position）
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 节点 5: Right Arm Controller（右臂控制器）
    # - 类型: JointTrajectoryController
    # - 功能: 接收轨迹点并控制右臂 7 个关节运动
    # - 关节: raxis1_joint ~ raxis7_joint
    # - 接口: 位置控制（position）
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 节点 6: Move Group（MoveIt2 核心规划节点）
    # - 包: moveit_ros_move_group
    # - 功能: 运动规划、碰撞检测、轨迹生成
    # - 参数说明:
    #   - robot_description: URDF 模型
    #   - robot_description_semantic: SRDF 语义描述
    #   - robot_description_kinematics: 运动学配置
    #   - robot_description_planning: 关节限制配置
    #   - moveit_controller_manager: 控制器管理器类型
    #   - moveit_simple_controller_manager: 控制器映射配置
    #   - publish_robot_description_semantic: 发布 SRDF 到参数服务器
    #   - allow_trajectory_execution: 允许轨迹执行
    #   - publish_planning_scene: 发布规划场景
    #   - publish_geometry_updates: 发布几何更新
    #   - publish_state_updates: 发布状态更新
    #   - publish_transforms_updates: 发布变换更新
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

    # 节点 7: RViz2（可视化界面）
    # - 加载 MoveIt 插件配置
    # - 显示内容:
    #   - 机器人模型 (RobotModel)
    #   - 规划场景 (PlanningScene)
    #   - 运动规划 (MotionPlanning)
    #   - 坐标系 (TF)
    #   - 网格 (Grid)
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

    # ========== 返回启动描述 ==========
    # 启动顺序（按依赖关系）：
    # 1. use_sim_time 参数声明
    # 2. hw_plugin 参数声明
    # 3. robot_state_publisher - 发布机器人描述
    # 4. ros2_control_node - 加载硬件接口
    # 5. joint_state_broadcaster_spawner - 广播关节状态
    # 6. left_arm_controller_spawner - 激活左臂控制器
    # 7. right_arm_controller_spawner - 激活右臂控制器
    # 8. move_group_node - 启动运动规划
    # 9. rviz_node - 启动可视化
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
