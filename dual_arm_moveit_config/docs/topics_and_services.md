# Dual Arm MoveIt2 仿真话题与服务参考手册

> 基于 `moveit.launch.py` 启动的完整仿真环境（mock_components/GenericSystem）

---

## 目录

- [系统节点总览](#系统节点总览)
- [话题（Topics）](#话题topics)
  - [MoveIt 规划相关](#moveit-规划相关)
  - [MoveIt 执行相关](#moveit-执行相关)
  - [碰撞物体管理](#碰撞物体管理)
  - [关节状态](#关节状态)
  - [控制器状态](#控制器状态)
  - [控制器管理器](#控制器管理器)
  - [TF 坐标变换](#tf-坐标变换)
  - [机器人描述](#机器人描述)
  - [RViz 交互](#rviz-交互)
  - [系统/诊断](#系统诊断)
- [服务（Services）](#服务services)
  - [MoveIt 规划服务](#moveit-规划服务)
  - [MoveIt 场景服务](#moveit-场景服务)
  - [MoveIt 参数服务](#moveit-参数服务)
  - [控制器管理器服务](#控制器管理器服务)
  - [控制器服务](#控制器服务)
  - [其他节点服务](#其他节点服务)
- [Plan 与 Execute 流程详解](#plan-与-execute-流程详解)

---

## 系统节点总览

| 节点 | 包名 | 作用 |
|------|------|------|
| `robot_state_publisher` | robot_state_publisher | 发布 URDF 和 TF 变换 |
| `ros2_control_node` | controller_manager | 硬件抽象层，管理控制器生命周期 |
| `joint_state_broadcaster` | controller_manager | 广播关节状态到 /joint_states |
| `left_arm_controller` | controller_manager | 左臂关节轨迹控制器（7轴） |
| `right_arm_controller` | controller_manager | 右臂关节轨迹控制器（7轴） |
| `move_group` | moveit_ros_move_group | MoveIt2 核心规划与执行节点 |
| `rviz2` | rviz2 | 可视化与交互界面 |

---

## 话题（Topics）

### MoveIt 规划相关

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | move_group | **Plan 按钮核心输出**。发布规划好的轨迹，包含起始状态和路径点序列。RViz 接收后显示橙色轨迹动画 |
| `/pipeline_state` | `moveit_msgs/msg/PipelineState` | move_group | 当前规划管线状态（是否在规划、使用的规划器/采样器类型、耗时等） |
| `/planning_scene` | `moveit_msgs/msg/PlanningScene` | move_group | **完整规划场景**。包含机器人当前状态、所有碰撞物体、允许碰撞矩阵、关节限制等 |
| `/monitored_planning_scene` | `moveit_msgs/msg/PlanningScene` | move_group | 规划场景增量更新。只发布变化部分，用于 WorldGeometryMonitor 实时更新障碍物 |
| `/planning_scene_world` | `moveit_msgs/msg/PlanningSceneWorld` | move_group | 仅世界部分（碰撞物体 + 八叉树地图），不含机器人状态 |
| `/display_contacts` | `visualization_msgs/msg/MarkerArray` | move_group | 碰撞检测接触点可视化标记。路径碰撞检查时在 RViz 中显示接触点 |

### MoveIt 执行相关

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/trajectory_execution_event` | `moveit_msgs/msg/TrajectoryExecutionEvent` | move_group | **Execute 按钮核心反馈**。发布执行生命周期事件：VALIDATE → EXECUTE → SUCCESS/ABORT/PREEMPT |

### 碰撞物体管理

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/collision_object` | `moveit_msgs/msg/CollisionObject` | move_group / 用户节点 | 向规划场景添加/移除/修改碰撞物体（桌子、障碍物等）。MoveIt 规划时会避开这些物体 |
| `/attached_collision_object` | `moveit_msgs/msg/AttachedCollisionObject` | move_group | 通知某碰撞物体已附着到机器人连杆（如夹爪抓取物体后），物体跟随机器人运动 |
| `/recognized_object_array` | `object_recognition_msgs/msg/RecognizedObjectArray` | 物体识别节点 | 物体识别结果接口（当前配置无视觉传感器，此话题通常为空） |

### 关节状态

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster | **核心状态话题**。所有 14 个关节的名称、位置、速度、力矩。RViz 用它更新模型，MoveIt 用它获取当前状态。频率 100Hz |
| `/dynamic_joint_states` | `sensor_msgs/msg/DynamicJointStates` | joint_state_broadcaster | 扩展版关节状态，支持任意命名的状态接口值（温度、电流等），比 /joint_states 更灵活 |

### 控制器状态

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/left_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | left_arm_controller | 左臂控制器详细状态：当前关节位置/速度/加速度、参考轨迹点、跟踪误差 |
| `/right_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | right_arm_controller | 右臂控制器详细状态，与左臂对称 |
| `/left_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | — | 左臂关节轨迹话题接口（实际执行主要通过 action） |
| `/right_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | — | 右臂关节轨迹话题接口（实际执行主要通过 action） |
| `/left_arm_controller/speed_scaling_input` | `std_msgs/msg/Float64` | 外部节点 | 左臂速度缩放输入（0.0~1.0），允许安全传感器动态降速。当前未连接 |
| `/right_arm_controller/speed_scaling_input` | `std_msgs/msg/Float64` | 外部节点 | 右臂速度缩放输入，与左臂对称 |
| `/left_arm_controller/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | left_arm_controller | 左臂控制器生命周期事件（unconfigured → inactive → active） |
| `/right_arm_controller/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | right_arm_controller | 右臂控制器生命周期事件 |
| `/joint_state_broadcaster/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | joint_state_broadcaster | 广播器生命周期事件 |

### 控制器管理器

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/controller_manager/activity` | 日志类型 | ros2_control_node | 控制器管理器活动日志（加载/激活/停用事件流） |
| `/controller_manager/introspection_data/full` | 二进制 | ros2_control_node | 所有控制器的完整内省数据（参数、接口、链等） |
| `/controller_manager/introspection_data/names` | 字符串数组 | ros2_control_node | 已加载控制器的名称索引列表 |
| `/controller_manager/introspection_data/values` | 二进制 | ros2_control_node | 对应控制器的详细状态值 |
| `/controller_manager/statistics/full` | 二进制 | ros2_control_node | 完整性能统计（CPU、内存、更新频率） |
| `/controller_manager/statistics/names` | 字符串数组 | ros2_control_node | 统计条目名称索引 |
| `/controller_manager/statistics/values` | 二进制 | ros2_control_node | 对应统计数值 |

### TF 坐标变换

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/tf` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | 实时坐标变换。所有关节的 TF（base_link → 各连杆）。MoveIt 用它计算正运动学，RViz 用它定位模型 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | 静态坐标变换（传感器安装位置、固定偏移等）。只发布一次，带 latch 特性 |

### 机器人描述

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | URDF 机器人描述（XML 格式）。所有需要知道机器人结构的节点都订阅此话题 |
| `/robot_description_semantic` | `std_msgs/msg/String` | move_group | SRDF 语义描述。包含规划组定义、自碰撞排除表、末端执行器定义、虚拟关节等 |

### RViz 交互

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `.../interactive_marker_topic/feedback` | `visualization_msgs/msg/InteractiveMarkerFeedback` | RViz 插件 | 用户在 RViz 中拖拽交互标记（末端目标位姿）的反馈。move_group 用此更新目标 |
| `.../interactive_marker_topic/update` | `visualization_msgs/msg/InteractiveMarkerUpdate` | move_group | move_group 向 RViz 推送交互标记更新（规划组变化时更新标记位置） |

### 系统/诊断

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/rosout` | `rcl_interfaces/msg/Log` | 所有节点 | ROS 2 日志聚合（INFO/WARN/ERROR/FATAL）。`rqt_console` 可查看 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 各节点 | 诊断信息（健康状态、温度、频率等）。`rqt_robot_monitor` 可视化 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 所有节点 | 参数变更事件总线。任何节点参数被修改时广播 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真环境 | 仿真时钟（use_sim_time=true 时使用） |

---

## 服务（Services）

### MoveIt 规划服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` | **Plan 按钮调用**。请求运动规划，返回关节空间轨迹 |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | 正运动学求解。给定关节角，计算末端位姿 |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | 逆运动学求解。给定末端位姿，求解关节角 |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | 笛卡尔路径规划。直线运动等笛卡尔约束下的路径 |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | 检查某个机器人状态是否无碰撞/满足约束 |
| `/query_planner_interface` | `moveit_msgs/srv/QueryPlannerInterfaces` | 查询可用的规划器接口和参数 |
| `/get_planner_params` | `moveit_msgs/srv/GetPlannerParams` | 获取指定规划器的参数 |
| `/set_planner_params` | `moveit_msgs/srv/SetPlannerParams` | 设置规划器参数 |

### MoveIt 场景服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | 获取当前规划场景（可指定获取哪些组件） |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | 应用规划场景更新（添加/移除物体、修改碰撞矩阵等） |
| `/clear_octomap` | `std_srvs/srv/Empty` | 清除八叉树地图（当前未使用 Octomap） |

### MoveIt 参数服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/load_geometry_from_file` | 自定义 | 从文件加载几何体到规划场景 |
| `/save_geometry_to_file` | 自定义 | 将规划场景中的几何体保存到文件 |
| `/load_map` | 自定义 | 加载地图 |
| `/save_map` | 自定义 | 保存地图 |

### 控制器管理器服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | 列出所有已加载控制器及其状态 |
| `/controller_manager/list_controller_types` | `controller_manager_msgs/srv/ListControllerTypes` | 列出可用的控制器类型插件 |
| `/controller_manager/load_controller` | `controller_manager_msgs/srv/LoadController` | 加载控制器到管理器 |
| `/controller_manager/unload_controller` | `controller_manager_msgs/srv/UnloadController` | 卸载控制器 |
| `/controller_manager/configure_controller` | `controller_manager_msgs/srv/ConfigureController` | 配置控制器（inactive 状态） |
| `/controller_manager/cleanup_controller` | `controller_manager_msgs/srv/CleanupController` | 清理控制器（回到 unconfigured） |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | **切换控制器**（激活/停用） |
| `/controller_manager/reload_controller_libraries` | `controller_manager_msgs/srv/ReloadControllerLibraries` | 重新加载控制器插件库 |
| `/controller_manager/list_hardware_components` | `controller_manager_msgs/srv/ListHardwareComponents` | 列出硬件组件 |
| `/controller_manager/list_hardware_interfaces` | `controller_manager_msgs/srv/ListHardwareInterfaces` | 列出硬件接口 |
| `/controller_manager/set_hardware_component_state` | `controller_manager_msgs/srv/SetHardwareComponentState` | 设置硬件组件状态 |

### 控制器服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/left_arm_controller/query_state` | `control_msgs/srv/QueryTrajectoryState` | 查询左臂控制器在未来某时刻的状态 |
| `/right_arm_controller/query_state` | `control_msgs/srv/QueryTrajectoryState` | 查询右臂控制器在未来某时刻的状态 |

### 其他节点服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/get_urdf` | 自定义 | 获取 URDF 描述 |
| `/rviz2/reset_time` | 自定义 | 重置 RViz 时间 |
| `.../interactive_marker_topic/get_interactive_markers` | `visualization_msgs/srv/GetInteractiveMarkers` | 获取交互标记定义 |

---

## Plan 与 Execute 流程详解

### 点击 Plan 按钮

```
用户点击 "Plan" (RViz)
    │
    ▼
RViz MotionPlanning 插件
    │
    ├─ 1. 读取当前状态: /joint_states, /tf
    │
    ├─ 2. 调用服务: /plan_kinematic_path
    │     └─ move_group 接收请求
    │         ├─ 读取 /robot_description (URDF)
    │         ├─ 读取 /robot_description_semantic (SRDF)
    │         ├─ 读取 /planning_scene (当前场景)
    │         ├─ 调用 /check_state_validity (检查起始状态)
    │         └─ OMPL 规划器求解 → 生成轨迹
    │
    ├─ 3. 发布: /display_planned_path (显示橙色轨迹)
    │
    └─ 4. 发布: /pipeline_state (规划状态更新)
```

**涉及话题**: `/joint_states`, `/tf`, `/display_planned_path`, `/pipeline_state`, `/planning_scene`
**涉及服务**: `/plan_kinematic_path`, `/check_state_validity`

---

### 点击 Execute 按钮

```
用户点击 "Execute" (RViz)
    │
    ▼
RViz MotionPlanning 插件
    │
    ├─ 1. 调用服务: /move_group/execute_trajectory (或直接通过 action)
    │     └─ move_group 接收执行请求
    │
    ├─ 2. 发布: /trajectory_execution_event (VALIDATE)
    │
    ├─ 3. move_group 通过 Action 发送轨迹:
    │     ├─ /left_arm_controller/follow_joint_trajectory  (Action Goal)
    │     └─ /right_arm_controller/follow_joint_trajectory (Action Goal)
    │
    ├─ 4. 控制器执行:
    │     ├─ left_arm_controller → 硬件接口 → 关节运动
    │     └─ right_arm_controller → 硬件接口 → 关节运动
    │
    ├─ 5. joint_state_broadcaster 发布:
    │     ├─ /joint_states (实时关节状态更新)
    │     └─ /dynamic_joint_states
    │
    ├─ 6. 控制器发布:
    │     ├─ /left_arm_controller/controller_state
    │     └─ /right_arm_controller/controller_state
    │
    ├─ 7. robot_state_publisher 发布:
    │     ├─ /tf (各连杆位姿更新)
    │     └─ /tf_static
    │
    └─ 8. 发布: /trajectory_execution_event (SUCCESS / ABORT)
```

**涉及话题**: `/joint_states`, `/tf`, `/trajectory_execution_event`, `*/controller_state`
**涉及服务**: `/move_group/execute_trajectory`
**涉及 Action**: `*/follow_joint_trajectory`

---

## 话题统计

| 类别 | 话题数量 |
|------|----------|
| MoveIt 规划相关 | 6 |
| MoveIt 执行相关 | 1 |
| 碰撞物体管理 | 3 |
| 关节状态 | 2 |
| 控制器状态 | 8 |
| 控制器管理器 | 6 |
| TF 坐标变换 | 2 |
| 机器人描述 | 2 |
| RViz 交互 | 2 |
| 系统/诊断 | 4 |
| **合计** | **36** |

## 服务统计（排除通用参数服务）

| 类别 | 服务数量 |
|------|----------|
| MoveIt 规划服务 | 8 |
| MoveIt 场景服务 | 3 |
| MoveIt 参数/文件服务 | 4 |
| 控制器管理器服务 | 11 |
| 控制器服务 | 2 |
| 其他 | 3 |
| **合计（核心服务）** | **31** |
