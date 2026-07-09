# dual_arm_servo

MoveIt Servo 双臂实时伺服控制包。提供两个独立的 Servo 节点（左臂 / 右臂），
接受 Twist 笛卡尔速度命令，实时转换为关节轨迹并发送给 ros2_control 执行。

---

## 快速开始

### 终端 1 — 启动完整 Servo 仿真栈

```bash
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=servo
```

等待所有节点就绪（servo_left、servo_right、rviz2 窗口弹出）。

### 终端 2 — 启动键盘遥操作

```bash
source install/setup.bash
ros2 run dual_arm_servo keyboard_teleop
```

看到 `双臂键盘遥操作就绪` 即可操作。

### 终端 3 — 监控关节状态（可选）

```bash
source install/setup.bash
ros2 topic echo /joint_states --field position
```

### 键盘操作

| 按键 | 效果 |
|---|---|
| `1` / `2` | 切换左臂 / 右臂 |
| `w` / `s` | 前进 / 后退 (X轴平移) |
| `a` / `d` | 左移 / 右移 (Y轴平移) |
| `q` / `e` | 上升 / 下降 (Z轴平移) |
| `i` / `k` | 绕 X 轴旋转 |
| `j` / `l` | 绕 Y 轴旋转 |
| `u` / `o` | 绕 Z 轴旋转 |
| `Ctrl+C` | 退出 |

---

## 数据流

完整的控制链路如下（以左臂为例）：

```
键盘按键
   │
   ▼
┌─────────────────────┐
│  keyboard_teleop    │  发布 TwistStamped
│  (Python 节点)      │──────────────────────────▶  /servo_left/delta_twist_cmds
└─────────────────────┘                             消息类型: geometry_msgs/TwistStamped
                                                    坐标系: base_link
   ▲                                                    │
   │                                                    │ 订阅
   │                                                    ▼
   │                     读取关节状态              ┌──────────────────────┐
   │◀──────────────────────────────────────────────│  servo_left          │
   │                     /joint_states             │  (servo_node_main)   │
   │                                              │                      │
   │                                              │  1. 接收 Twist        │
   │                                              │  2. 读取当前关节状态   │
   │                                              │  3. 逆运动学求解      │
   │                                              │  4. 奇异点/关节限制检测│
   │                                              │  5. 输出关节轨迹      │
   │                                              └──────────┬───────────┘
   │                                                         │
   │                              发布 JointTrajectory        │
   │◀────────────────────────────────────────────────────────┘
   │                              /left_arm_controller/joint_trajectory
   │                              消息类型: trajectory_msgs/JointTrajectory
   │                                                         │
   │                                                         ▼
   │                                              ┌──────────────────────┐
   │                                              │  left_arm_controller │
   │                                              │  (JointTrajectory-   │
   │                                              │   Controller)        │
   │                                              │                      │
   │                                              │  跟踪轨迹并生成      │
   │                                              │  关节位置指令        │
   │                                              └──────────┬───────────┘
   │                                                         │
   │                                                         ▼
   │                                              ┌──────────────────────┐
   │                                              │  ros2_control_node   │
   │                                              │  (controller_manager)│
   │                                              │                      │
   │                                              │  mock_components/    │
   │                                              │  GenericSystem       │
   │                                              │  模拟硬件响应        │
   │                                              └──────────┬───────────┘
   │                                                         │
   │                                                         ▼
   │                                              ┌──────────────────────┐
   │                                              │ joint_state_         │
   │                                              │ broadcaster          │
   │                                              │                      │
   │                                              │ 读取硬件状态，发布   │
   └──────────────────────────────────────────────│ 到 /joint_states     │
                                                  └──────────────────────┘
```

### 话题通信一览

| 话题 | 消息类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/servo_left/delta_twist_cmds` | `TwistStamped` | keyboard_teleop | servo_left | 左臂笛卡尔速度命令 |
| `/servo_right/delta_twist_cmds` | `TwistStamped` | keyboard_teleop | servo_right | 右臂笛卡尔速度命令 |
| `/left_arm_controller/joint_trajectory` | `JointTrajectory` | servo_left | left_arm_controller | 左臂关节轨迹 |
| `/right_arm_controller/joint_trajectory` | `JointTrajectory` | servo_right | right_arm_controller | 右臂关节轨迹 |
| `/joint_states` | `JointState` | joint_state_broadcaster | servo_left, servo_right | 关节反馈（闭环） |
| `/servo_left/start_servo` | `std_srvs/Trigger` | keyboard_teleop | servo_left | 启动 Servo 服务 |
| `/servo_right/start_servo` | `std_srvs/Trigger` | keyboard_teleop | servo_right | 启动 Servo 服务 |

### 节点一览

| 节点 | 包 | 可执行文件 | 功能 |
|---|---|---|---|
| `servo_left` | moveit_servo | servo_node_main | 左臂 Servo 实时控制 |
| `servo_right` | moveit_servo | servo_node_main | 右臂 Servo 实时控制 |
| `keyboard_teleop` | dual_arm_servo | keyboard_teleop | 键盘输入 → Twist |
| `robot_state_publisher` | robot_state_publisher | robot_state_publisher | 发布 TF 变换 |
| `ros2_control_node` | controller_manager | ros2_control_node | 硬件抽象层 |
| `joint_state_broadcaster` | joint_state_broadcaster | spawner | 发布 /joint_states |
| `left_arm_controller` | joint_trajectory_controller | spawner | 执行左臂轨迹 |
| `right_arm_controller` | joint_trajectory_controller | spawner | 执行右臂轨迹 |

---

## 包结构

```
dual_arm_servo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── servo_left.yaml          # 左臂 Servo 参数
│   └── servo_right.yaml         # 右臂 Servo 参数
├── launch/
│   ├── servo.launch.py          # 启动两个 Servo 节点
│   └── servo_control.launch.py  # 兼容入口（委托给 servo.launch.py）
└── dual_arm_servo/
    ├── __init__.py
    └── keyboard_teleop.py       # 键盘遥操作节点
```

---

## 实现原理

### MoveIt Servo 工作方式

MoveIt Servo (`servo_node_main`) 是一个实时控制节点，核心流程：

1. **接收命令**：订阅 `/servo_left/delta_twist_cmds`（TwistStamped），获取笛卡尔速度增量
2. **读取状态**：订阅 `/joint_states` 获取当前关节角度
3. **逆运动学**：根据 kinematics.yaml 配置求解关节速度
4. **安全检测**：检查奇异点（条件数）、关节限位、碰撞（如启用）
5. **输出命令**：将结果转换为 JointTrajectory 发布给 JointTrajectoryController

与 MoveGroup 不同，Servo **不进行路径规划**，而是增量式地将速度命令转换为关节运动，
延迟极低（~30ms），适合遥操作和实时交互场景。

### Servo 与 MoveGroup 的区别

| | MoveGroup (mode:=moveit) | Servo (mode:=servo) |
|---|---|---|
| 启动的节点 | move_group | servo_left + servo_right |
| 控制方式 | 离线规划 → 一次性执行 | 实时增量式控制 |
| 延迟 | 高（需要规划时间） | 低（~30ms） |
| 交互方式 | RViz 拖拽 / 代码 API | Twist 命令（键盘/VR/摇杆） |
| 是否需要 MoveGroup | 是 | 否 |
| Planning Scene | 由 MoveGroup 管理 | 由 Servo 自己管理 |

### 关键配置参数

`config/servo_left.yaml` / `servo_right.yaml` 中的核心参数：

```yaml
command_in_type: "speed_units"     # 输入为 m/s 和 rad/s
scale:
  linear: 0.2                      # 笛卡尔最大线速度 m/s
  rotational: 0.8                  # 笛卡尔最大角速度 rad/s
  joint: 0.5                       # 关节最大速度 rad/s

command_out_type: trajectory_msgs/JointTrajectory  # 输出给 JTC
publish_period: 0.034              # 发布周期 ~30Hz
incoming_command_timeout: 0.1      # 100ms 无命令则停止

lower_singularity_threshold: 17.0  # 开始减速的条件数阈值
hard_stop_singularity_threshold: 30.0  # 停止的条件数阈值
```

---

## 独立启动（不使用 bringup）

如果只需要 Servo 节点（已手动启动 ros2_control 和控制器）：

```bash
# 仅启动 Servo 节点
ros2 launch dual_arm_servo servo.launch.py

# 或使用兼容入口
ros2 launch dual_arm_servo servo_control.launch.py
```

注意：此方式不会启动 ros2_control、控制器、RViz。需要确保 `robot_state_publisher`
和 JointTrajectoryController 已在运行。

---

## 常见问题

**Q: MotionPlanning 面板提示 "MoveGroup action server not available"**
A: 正常现象。Servo 模式不启动 MoveGroup，该面板不可用。机器人的运动通过 Twist 命令控制。

**Q: 按 WASD 机器人不动**
A: 检查：
1. servo_left / servo_right 节点是否正常启动（终端 1 无 ERROR）
2. `/joint_states` 是否有数据：`ros2 topic hz /joint_states`
3. 控制器是否激活：`ros2 control list_controllers`，确认 left/right_arm_controller 状态为 active

**Q: 如何同时控制双臂**
A: 在 keyboard_teleop 中按 `1` 切到左臂操作，再按 `2` 切到右臂操作。当前设计是分时控制（同一时刻只控制一个臂）。
