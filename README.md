# dual_arm

ROS 2 工作空间：双臂移动机器人（两个 7 自由度机械臂 + 轮式底盘）

## 快速开始

```bash
# 构建
colcon build
source install/setup.bash

# 仿真模式（mock hardware，默认 moveit 模式）
ros2 launch dual_arm_bringup sim.launch.py

# Servo 仿真模式（MoveGroup 不启动，RViz 仍启动）
ros2 launch dual_arm_bringup sim.launch.py mode:=servo

# 真实硬件模式（默认 moveit 模式）
ros2 launch dual_arm_bringup real.launch.py
```

Servo 模式当前复用 `moveit.rviz`，因此 RViz 的 MotionPlanning 面板可能提示 MoveGroup 不可用；这是预期现象，不表示 Servo 节点失败。

单独构建某个包：
```bash
colcon build --packages-select dual_arm_control
```

手动关节控制 GUI：
```bash
python3 dual_arm_description/joint_control_panel.py
```

## 包结构

```
dual_arm_description/    # URDF/xacro、网格、rviz 配置
dual_arm_control/        # C++ ros2_control 硬件接口插件
dual_arm_moveit_config/  # MoveIt2 配置：SRDF、运动学、控制器、MoveGroup/RViz launch
dual_arm_servo/          # MoveIt Servo 配置和 launch
dual_arm_bringup/        # 顶层启动文件
```

依赖关系：`bringup → moveit_config/servo/control → description`

## 硬件插件切换

xacro 文件通过 `hw_plugin` 参数选择硬件后端：

- `mock_components/GenericSystem` — 仿真（默认）
- `dual_arm_control/DualArmHardware` — 真实硬件插件

## 机器人结构

- **base_link** — 底盘
- **左臂**: laxis1_joint – laxis7_joint（7 自由度旋转关节）
- **右臂**: raxis1_joint – raxis7_joint（7 自由度旋转关节）
- **轮子**: 6 个轮子（仅 URDF 定义，无功能关节）

关节限制：±3.14 rad（±180°），力矩 20 Nm，速度 2.0 rad/s

## 依赖

- ROS 2（Humble 或更新版本）
- MoveIt 2
- ros2_control
- xacro
- robot_state_publisher
- joint_state_publisher_gui
- rviz2
- gazebo_ros（可选）

## 许可证

待定
