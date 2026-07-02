# vrtest-full-lite 与 dual_arm 合并计划

## 目标

把当前 Unity/PICO VR 项目与 `~/dual_arm` ROS2/MoveIt 项目合并成一个可维护的工程，使 VR 手柄位姿可以进入 `dual_arm` 控制链路，并最终驱动仿真或真实双臂。

合并后的主数据流：

```text
PICO / Unity
  -> ROS TCP Connector
  -> ros_tcp_endpoint
  -> /vr/left_hand/pose, /vr/right_hand/pose
  -> VR 到 Servo 适配节点
  -> /servo_left/delta_twist_cmds, /servo_right/delta_twist_cmds
  -> MoveIt Servo
  -> left/right_arm_controller
  -> ros2_control / soem_bridge
  -> 电机目标位置或速度
```

## 当前项目分工

### vrtest-full-lite

- 类型：Unity 2022.3 PICO VR 项目 + ROS TCP Endpoint 辅助文件。
- 主要功能：读取 PICO 左右手柄位姿，并发布到 ROS2。
- 关键文件：
  - `vrtest/Assets/VRHandPublisher.cs`
  - `vrtest/Assets/Resources/ROSConnectionPrefab.prefab`
  - `vrtest-package/ros2-tools/ros_tcp_endpoint`
  - `vrtest-package/ros2-tools/vr_hand_subscriber.py`
- 当前 ROS topic：
  - `/vr/left_hand/pose`: `geometry_msgs/msg/PoseStamped`
  - `/vr/right_hand/pose`: `geometry_msgs/msg/PoseStamped`
  - `/vr/status`: `std_msgs/msg/String`

### dual_arm

- 类型：ROS2 Humble 工作区。
- 主要功能：双臂 URDF/SRDF、MoveIt2、MoveIt Servo、ros2_control、SOEM 实机桥接。
- 关键包：
  - `dual_arm_description`
  - `dual_arm_moveit_config`
  - `dual_arm_servo`
  - `dual_arm_bringup`
  - `dual_arm_control`
  - `dual_arm_soem_bridge`
- 关键控制入口：
  - MoveIt 规划模式：`/compute_ik` + `/move_action`
  - Servo 实时模式：`/servo_left/delta_twist_cmds`、`/servo_right/delta_twist_cmds`
  - 控制器输出：`/left_arm_controller/joint_trajectory`、`/right_arm_controller/joint_trajectory`

## 合并原则

1. 以 `dual_arm` 作为 ROS2 主工作区。
2. 保留 Unity 项目为独立前端，不把 Unity 资源混入 ROS 包内部。
3. ROS TCP Endpoint、VR 接收、VR 到 MoveIt 的适配逻辑迁入 `dual_arm`。
4. Unity 只发布标准 ROS 消息，不直接理解 MoveIt、IK、关节名或电机协议。
5. 先打通仿真，再接实机。
6. 先用 MoveIt Servo 做连续遥操作，点到点位姿规划作为后续功能。

## 推荐合并后的目录结构

建议以 `dual_arm` 为根目录，新增 `unity/` 或 `vr_unity/` 保存 Unity 项目：

```text
dual_arm/
  dual_arm_bringup/
  dual_arm_description/
  dual_arm_moveit_config/
  dual_arm_servo/
  dual_arm_control/
  dual_arm_soem_bridge/
  vr_teleop_bridge/                 # 新增 ROS2 包
  third_party/
    ros_tcp_endpoint/               # 从 vrtest-package 迁入或作为 git submodule
  unity/
    vrtest/                         # 当前 Unity 项目
  docs/
    vr_merge_plan.md
    vr_usage.md
```

如果短期不想移动 `dual_arm`，也可以反过来在当前仓库下加入 `dual_arm/`，但不推荐。原因是 `dual_arm` 已经是完整 ROS2 工作区，后续 colcon、launch、实机权限和 MoveIt 配置都以它为中心更自然。

## 新增 ROS2 包设计

新增包名建议：`vr_teleop_bridge`

职责：

- 启动或依赖 `ros_tcp_endpoint`。
- 订阅 Unity 发布的 VR 位姿。
- 完成 VR 坐标系到机器人 `base_link` 坐标系的映射。
- 实现手柄使能、死区、限速、滤波、超时急停。
- 将 VR 相对运动转换为 MoveIt Servo 的 `TwistStamped`。
- 发布到：
  - `/servo_left/delta_twist_cmds`
  - `/servo_right/delta_twist_cmds`

核心节点建议：

```text
vr_pose_to_servo_node
```

输入：

```text
/vr/left_hand/pose   geometry_msgs/msg/PoseStamped
/vr/right_hand/pose  geometry_msgs/msg/PoseStamped
/vr/status           std_msgs/msg/String
```

输出：

```text
/servo_left/delta_twist_cmds   geometry_msgs/msg/TwistStamped
/servo_right/delta_twist_cmds  geometry_msgs/msg/TwistStamped
```

参数：

```yaml
control_frame: base_link
left_enabled: true
right_enabled: true
linear_scale: 0.5
angular_scale: 1.0
max_linear_speed: 0.15
max_angular_speed: 0.5
deadband_position: 0.005
deadband_rotation: 0.02
command_timeout: 0.2
publish_rate: 50.0
```

## 控制模式选择

### 第一阶段：MoveIt Servo

用于连续遥操作，是优先方案。

启动方式：

```bash
source ~/dual_arm/install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=servo
```

优点：

- 匹配 VR 50Hz 连续输入。
- 已有 `dual_arm_servo` 配置。
- 可以输出关节轨迹给现有 controller。
- 实机 velocity 控制链路更容易接入。

### 第二阶段：MoveIt 规划

用于“采样一次 VR 位姿，机械臂自动规划过去”。

控制入口：

```text
/compute_ik
/move_action
```

该模式不适合持续 50Hz 重规划，只适合按钮触发的离散目标。

## VR 坐标系与机器人坐标系

当前 Unity 发布的是手柄在 Unity/XR 空间中的位姿，不是机器人 `base_link` 下的目标位姿。

必须新增标定关系：

```text
T_base_vr
```

建议采用相对控制，不直接采用绝对位置：

1. 用户按住使能按钮时，记录当前手柄位姿 `H0`。
2. 同时记录当前机器人末端位姿 `E0`。
3. 手柄后续变化 `H - H0` 转换成末端速度指令。
4. 松开按钮或丢失 VR 数据时，立即发布零速度。

这样可以避免手柄初始位置与机械臂工作空间不一致导致的突跳。

## Unity 侧改造

短期保持 `VRHandPublisher.cs` 不大改，只做必要增强：

1. 把 ROS IP 和端口从 Prefab 固定值改为可配置项。
2. 增加手柄按钮状态 topic，例如：

```text
/vr/left_hand/buttons
/vr/right_hand/buttons
```

或合并成：

```text
/vr/controller_state
```

3. 增加 sequence 或真实时间戳，便于 ROS 侧检测丢包和超时。
4. 在 Unity 日志中明确输出 ROS 连接状态。

中期可以把发布消息从多个 topic 改为一个自定义消息，但第一阶段不建议引入自定义 Unity ROS 消息生成流程。

## ROS TCP Endpoint 迁移

当前 `ros_tcp_endpoint` 位于：

```text
vrtest-package/ros2-tools/ros_tcp_endpoint
```

迁移方案二选一：

### 方案 A：作为 third_party 目录

```text
dual_arm/third_party/ros_tcp_endpoint
```

优点：最直接，离线可用。

缺点：后续同步上游不方便。

### 方案 B：作为 git submodule

```text
dual_arm/third_party/ros_tcp_endpoint
```

优点：来源清晰，后续可升级。

缺点：需要管理 submodule。

第一阶段推荐方案 A，先确保系统能跑通。

## Launch 设计

新增一个统一 VR 仿真启动：

```text
dual_arm_bringup/launch/vr_servo_sim.launch.py
```

包含：

- `sim.launch.py mode:=servo`
- `ros_tcp_endpoint`
- `vr_pose_to_servo_node`

新增一个统一 VR 实机启动：

```text
dual_arm_bringup/launch/vr_servo_real.launch.py
```

包含：

- `real.launch.py mode:=servo`
- `soem_bridge.launch.py`
- `ros_tcp_endpoint`
- `vr_pose_to_servo_node`

实机 launch 默认不自动使能电机，必须单独调用 enable 服务。

## 实施阶段

### 阶段 0：冻结当前可运行状态

- 记录当前 `vrtest-full-lite` 能发布 VR topic 的启动命令。
- 记录当前 `dual_arm` 仿真 Servo 能用键盘遥操作的启动命令。
- 确认以下 topic 存在：

```bash
ros2 topic echo /vr/left_hand/pose
ros2 topic echo /vr/right_hand/pose
ros2 topic echo /servo_left/delta_twist_cmds
ros2 topic echo /servo_right/delta_twist_cmds
```

### 阶段 1：迁移 ROS TCP Endpoint

- 把 `vrtest-package/ros2-tools/ros_tcp_endpoint` 迁入 `dual_arm/third_party/ros_tcp_endpoint`。
- 确认能在 `dual_arm` 环境中启动：

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000
```

### 阶段 2：新增 vr_teleop_bridge 包

- 创建 ROS2 Python 包 `vr_teleop_bridge`。
- 实现 `vr_pose_to_servo_node`。
- 第一版只处理平移速度，不处理姿态。
- 加入超时急停：超过 `command_timeout` 未收到 VR pose，发布零速度。

### 阶段 3：仿真验证

- 启动 `sim.launch.py mode:=servo`。
- 启动 `ros_tcp_endpoint`。
- 启动 Unity/PICO。
- 启动 `vr_pose_to_servo_node`。
- 验证 RViz 中双臂可以跟随手柄相对运动。

通过标准：

- `/vr/*/pose` 稳定 50Hz 左右。
- `/servo_* /delta_twist_cmds` 有数据。
- `/left_arm_controller/joint_trajectory`、`/right_arm_controller/joint_trajectory` 有输出。
- 松开使能或停止 Unity 后机械臂停止。

### 阶段 4：加入按钮使能

- Unity 发布 grip/trigger/menu 等按钮状态。
- ROS 侧只有按住指定按钮时才输出非零速度。
- 未按住时持续发布零速度。

### 阶段 5：坐标标定与限幅

- 固化 `T_base_vr` 参数。
- 加入工作空间限制。
- 加入最大线速度、角速度限制。
- 加入低通滤波。

### 阶段 6：实机 dry-run

- 启动 `real.launch.py mode:=servo`。
- 不使能电机，只观察：

```bash
ros2 topic echo /left_arm_controller/controller_state
ros2 topic echo /right_arm_controller/controller_state
```

- 确认 velocity output 合理。

### 阶段 7：实机低速验证

- 设置非常低的速度限制。
- 单臂、单方向验证。
- 再验证双臂。
- 最后逐步提高速度。

## 风险

- Unity 坐标系和机器人坐标系不一致，必须标定。
- 直接绝对位姿控制会导致机械臂突跳，第一阶段禁止使用。
- VR 数据中断时必须急停。
- 实机 velocity 控制需要确认 PID、限位、急停和 SOEM 状态。
- PICO 与 ROS 电脑的 IP 会变化，不能长期依赖 Prefab 固定 IP。
- 当前 PICO SDK 是本地路径依赖，合并后需要重新整理 Unity 依赖说明。

## 推荐优先级

1. 先合并 ROS 侧：`ros_tcp_endpoint` + `vr_teleop_bridge`。
2. 再把 Unity 项目搬入 `dual_arm/unity/vrtest`。
3. 先跑仿真 Servo。
4. 加按钮使能和急停。
5. 做实机 dry-run。
6. 最后接电机低速验证。

## 最小可用版本定义

最小可用版本满足：

- 一个命令启动 ROS 仿真 VR Servo 环境。
- PICO 启动后能连接 ROS 电脑。
- 左右手柄位姿进入 ROS2。
- 按住使能按钮时，机械臂在 RViz 中跟随手柄相对运动。
- 松开按钮、VR 断连或 topic 超时时，机械臂停止。

