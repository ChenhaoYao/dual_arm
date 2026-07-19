# dual_arm 双臂机器人项目完整手册

> 面向低年级本科生的学习、仿真、调试与实物实验指南  
> 适用工作区：`/home/dell/dual_arm`  
> ROS 2 版本：Jazzy  
> 手册依据：截至 2026-07-16 的项目源码、配置、launch 文件及仓库内 Markdown 文档

## 0. 先读这一页

这个项目把两台 7 自由度机械臂、MoveIt 2、MoveIt Servo、EtherCAT 电机、PICO VR 手柄和 Unity 串成了一套双臂遥操作系统。它既包含适合入门的机器人模型与仿真，也包含会让真实电机运动的实时控制代码。

本手册有两个目标：

1. 让第一次接触 ROS 2 的同学能从模型显示开始，逐步理解每一层数据如何流动。
2. 让准备接触实物的同学知道每个软件“开关”究竟控制什么、哪些现象必须立即停止实验。

### 0.1 最高优先级安全规则

- 不熟悉本手册第 12 章的同学，不得独立进行实物运动实验。
- 实物运动前必须确认物理急停有效、机械臂运动范围内无人、线缆不会被拉扯。
- `/soem_bridge_node/enable` 只是软件命令门，不等于物理急停，也不切断驱动器功率。
- `/soem_bridge_node/stop` 会关闭软件命令门并清除旧速度目标，但仍不能替代物理急停。
- 运行 `real.launch.py` 本身通常不会运动；显式设置 `move_to_ready_on_start:=true` 会自动开门并让双臂运动。
- 不要同时启动两个 `real.launch.py`，不要让 RViz Marker、VR、键盘或其他程序同时向同一 Servo/JTC 链路发命令。
- 不要在命令门关闭时先执行轨迹、拖动 Marker，再突然 `enable`。这样可能让机械臂追赶已经积累的目标。
- `dry_run:=true` 不是“完全不接触 EtherCAT”。当前 SOEM bridge 在配置了网卡时仍可能连接总线并推进 CiA402 驱动状态；它只禁止速度命令真正下发。
- 出现方向相反、持续抖动、突然加速、编码器明显跳变、关节状态陈旧、看门狗锁定等现象时，先松开 Grip/停止输入，再调用软件 stop；无法确认时立即按物理急停。

### 0.2 如何使用本手册

推荐按以下顺序学习，不要直接跳到真机：

1. 第 1～7 章：理解目录、术语、模型和数据流。
2. 第 8～11 章：完成构建、模型显示、MoveIt 仿真、Servo 仿真和 VR 仿真。
3. 第 12～13 章：在教师或有经验同学监督下进行 EtherCAT 与实物实验。
4. 第 14～17 章：学习日志分析、故障定位和代码维护。

仓库中的历史调试记录很有价值，但可能描述旧方案。本手册遵循以下可信度顺序：

1. 当前正在编译的源码和 YAML；
2. 当前 launch 文件；
3. 本手册与 `docs/run_commands.md`；
4. 历史调试笔记、交接文档和实验报告。

如果文档与源码冲突，以当前源码为准，并补正文档。

---

## 1. 项目要解决什么问题

这套系统希望实现三种主要能力：

- **规划控制**：在 RViz 中给定目标，让 MoveIt 规划一条避障轨迹，再交给控制器执行。
- **实时伺服**：持续输入末端速度，MoveIt Servo 实时计算关节速度，适合 Marker、键盘和 VR 遥操作。
- **实物通信**：把 ROS 2 控制器输出的 14 轴速度转换为 EtherCAT CiA402 CSV 命令，同时把编码器位置送回 ROS 2。

项目当前不包含完整的移动底盘控制。URDF 中能看到 6 个轮子，但它们只是模型结构的一部分，没有接入实际底盘控制链。

### 1.1 三个容易混淆的“控制”

| 名称 | 解决的问题 | 输入 | 输出 | 是否持续实时运行 |
|---|---|---|---|---|
| MoveIt MoveGroup | 从当前位置规划到目标位置 | 目标位姿/关节角 | 完整关节轨迹 | 否，按目标规划 |
| MoveIt Servo | 根据末端速度实时做微分逆运动学 | `TwistStamped` | 短周期滚动关节轨迹 | 是 |
| ros2_control JTC | 跟踪关节轨迹并形成控制器输出 | `JointTrajectory` | 位置或速度命令 | 是 |

MoveGroup 和 Servo 是两种上层控制模式。顶层 launch 用 `mode:=moveit|servo` 二选一，正常使用时不要同时启动。

---

## 2. 工作区地图

执行 `colcon list` 可看到 9 个构建单元：

| 包/工程 | 类型 | 主要职责 | 初学者优先级 |
|---|---|---|---|
| `dual_arm_description` | ament_cmake | URDF/xacro、网格、RViz、模型小工具 | 必学 |
| `dual_arm_moveit_config` | ament_cmake | SRDF、IK、OMPL、控制器映射、MoveGroup launch | 必学 |
| `dual_arm_control` | ament_cmake/C++ | 真实反馈接入 ros2_control 的硬件插件 | 真机前必学 |
| `dual_arm_servo` | ament_cmake/Python | 双臂 Servo launch、RViz Marker、键盘输入 | 必学 |
| `dual_arm_bringup` | ament_cmake | 仿真/实物顶层启动、自动预备位 | 必学 |
| `dual_arm_soem_bridge` | ament_cmake/C++ | ROS 2 与 EtherCAT/CiA402 CSV 桥接 | 真机前必学 |
| `vr_teleop_bridge` | ament_cmake/Python | VR 位姿转 Servo/MoveGroup、轨迹日志 | VR 前必学 |
| `ros_tcp_endpoint` | ament_python | Unity 与 ROS 2 之间的 TCP 协议端点 | VR 前必学 |
| `SOEM` | 第三方 CMake | EtherCAT 主站库和底层示例 | 进阶 |

另外还有三类非 ROS 包目录：

- `vrtest-full-lite/vrtest/`：Unity 2022.3.62f3 PICO 工程。
- `third_party/PICO Unity Integration SDK-3.4.0-20260226/`：第三方 PICO SDK，不应作为本项目业务代码逐文件修改。
- `utils/` 与 `tools/`：离线分析、运行环境清理和特殊诊断脚本。

### 2.1 推荐的源码阅读顺序

```text
dual_arm_description/urdf/dual_arm_1kg.urdf.xacro
        ↓
dual_arm_moveit_config/config/dual_arm_1kg.srdf
        ↓
dual_arm_bringup/launch/control_base.launch.py
        ↓
dual_arm_bringup/launch/sim.launch.py
        ↓
dual_arm_moveit_config/launch/move_group.launch.py
        ↓
dual_arm_servo/launch/servo.launch.py
        ↓
vr_teleop_bridge/vr_teleop_bridge/vr_pose_to_servo_node.py
        ↓
dual_arm_control/src/dual_arm_hardware.cpp
        ↓
dual_arm_soem_bridge/src/soem_bridge_node.cpp
        ↓
dual_arm_soem_bridge/src/soem_master.cpp
```

先理解模型和仿真，再进入会接触实物的最后三层。

---

## 3. 入门所需的最少知识

### 3.1 ROS 2 的五个基本概念

- **节点 Node**：一个独立程序，例如 `robot_state_publisher`、`move_group`。
- **话题 Topic**：连续数据流，例如 `/joint_states`。发布者不等待订阅者回复。
- **服务 Service**：一次请求、一次响应，例如 `/soem_bridge_node/enable`。
- **动作 Action**：可持续一段时间、可反馈进度、可取消的任务，例如 FollowJointTrajectory。
- **参数 Parameter**：节点启动时的配置，例如速度上限和超时时间。

### 3.2 机器人模型术语

- **link**：刚体，例如 `laxis3_link`。
- **joint**：连接两个 link 的运动副，例如 `laxis3_joint`。
- **TF**：不同坐标系之间随时间变化的变换树。
- **正运动学 FK**：由关节角计算末端位姿。
- **逆运动学 IK**：由末端位姿求关节角。
- **奇异点**：某些方向的末端运动需要极大关节速度，或局部无法运动的姿态。
- **Jacobian**：把关节速度映射为末端线速度/角速度的矩阵。条件数越大，通常越接近奇异。

### 3.3 C++ 与 Python 阅读提示

本项目的启动、输入适配和工具主要用 Python；硬件插件与 EtherCAT 实时循环用 C++。第一次读 C++ 时重点认识：

- `std::atomic<T>`：不同线程安全共享简单状态；
- 指针 `*` 与地址 `&`：底层 PDO 内存映射经常用到；
- lambda：把回调函数就地写在订阅/服务创建处；
- 互斥锁：保护非原子复合数据；
- 实时线程：应避免阻塞、动态分配和大量日志。

更细的补课材料见 [`cpp_basics.md`](cpp_basics.md)。

---

## 4. 机器人模型与坐标系

### 4.1 模型的唯一主入口

当前 launch 使用：

```text
dual_arm_description/urdf/dual_arm_1kg.urdf.xacro
```

同目录的 `dual_arm_1kg.urdf` 是已有的展开/历史文件，不是顶层 launch 当前生成 `robot_description` 的源头。修改模型时优先改 xacro，然后重新构建或利用 `--symlink-install` 重启。

### 4.2 关节与规划组

两臂各有 7 个旋转关节：

```text
左臂：laxis1_joint ... laxis7_joint
右臂：raxis1_joint ... raxis7_joint
```

当前每个臂关节的模型限制为：

- 位置：约 `[-3.14, 3.14] rad`；
- 速度：`2.0 rad/s`；
- 力矩/effort 声明：`20`。

SRDF 定义：

- `left_arm`：`base_link` 到 `laxis7_link`；
- `right_arm`：`base_link` 到 `raxis7_link`；
- `dual_arm`：包含左右两个子组。

### 4.3 base_link 坐标方向

本项目确认的机器人坐标约定为：

- `base_link +x`：机器人左侧；
- `base_link -y`：机器人前方；
- `base_link +z`：竖直向上。

不要凭 RViz 相机视角判断正负方向。应显示 TF 坐标轴，或用小幅单方向仿真验证。

### 4.4 home、ready 与启动初值为什么不冲突

项目中能看到三类“姿态”：

| 位置 | 用途 | 是否主动产生运动 |
|---|---|---|
| xacro `<state_interface initial_value>` | mock hardware 启动时的初始关节状态 | 否，只初始化仿真状态 |
| SRDF `group_state` | MoveIt 中可选择的命名姿态 | 否，只保存目标集合 |
| `ready_pose.yaml` | 真机启动时由动作控制器执行的预备位 | 是，显式启用时运动 |

当前镜像 ready 为：

```text
左臂：[ 0.3, 0,  1.2,  1.2, 0, 0, 0] rad
右臂：[-0.3, 0, -1.2, -1.2, 0, 0, 0] rad
```

mock 初始值也使用这组镜像值，因此仿真启动后远离全零奇异姿态。SRDF 中仍可保存 `home` 和 `ready`，它不会自动覆盖 xacro 或真机编码器。

### 4.5 碰撞模型的现实限制

部分较大的 STL 已替换为盒体/圆柱碰撞几何，但 `laxis/raxis 3～7` 仍可能使用 STL。复杂网格会让 FCL 碰撞检测变慢甚至崩溃。因此当前 Servo 配置关闭了在线碰撞检查。这意味着：

- MoveGroup 规划仍应使用规划场景和碰撞检查；
- Servo 运动不能假定软件会阻止自碰撞；
- 真机 Servo 必须使用低速、小范围，并依靠操作者观察和物理急停。

如果日志出现“too many vertices”或 MoveGroup/FCL 崩溃，应继续简化剩余 link 的 collision 几何，而不是删掉 visual 网格。

---

## 5. 系统总数据流

### 5.1 仿真 MoveIt 模式

```text
RViz 目标
  → move_group（IK + 采样规划 + 时间参数化）
  → /left|right_arm_controller/follow_joint_trajectory
  → JointTrajectoryController（JTC）
  → mock_components/GenericSystem 的 position command interface
  → joint_state_broadcaster
  → /joint_states
  → robot_state_publisher
  → /tf
  → RViz
```

### 5.2 仿真 Servo 模式

```text
RViz Marker / 键盘 / VR bridge
  → /servo_left|right/delta_twist_cmds
  → 两个 moveit_servo/servo_node
  → 滚动 JointTrajectory
  → 左右 JTC
  → mock position command interface
  → /joint_states 与 TF
```

### 5.3 真机模式：本项目最容易误解的部分

```text
MoveGroup 或 Servo
  → 左右 JTC
  → controller_state.output.velocities ──────────────┐
                                                     ↓
真实编码器 → SOEM bridge → /joint_states → DualArmHardware::read()
                   ↑                    → JTC 状态接口与 TF
                   │
                   └── EtherCAT CSV 速度命令 ← soem_bridge_node
```

这里有一个不常见但很重要的设计：

1. `DualArmHardware` 订阅真实 `/joint_states`，把位置/速度写入 ros2_control 的 state interface。
2. 它的 `write()` 当前不直接访问 EtherCAT，基本是空操作。
3. `soem_bridge_node` 独立订阅左右 JTC 的 `controller_state`，取其中 `output.velocities` 再发给电机。

因此真机“不动”时不能只看 JTC action 是否成功。必须逐段检查：

```text
上层有没有给 JTC 目标？
→ JTC 有没有产生非零 output.velocities？
→ SOEM bridge 有没有收到？
→ 软件命令门是否打开？
→ 看门狗是否锁定？
→ 驱动是否 Operation enabled？
→ 编码器是否真的变化？
```

### 5.4 ROS 话题与进程内接口

ros2_control 的 command/state interface 是同一进程内共享内存接口，不是普通 ROS 话题。不能执行 `ros2 topic echo velocity_command_interface` 来看它。当前项目为了跨到独立 SOEM 进程，才读取 JTC 发布出来的 `controller_state`。

---

## 6. 各软件包详解

### 6.1 dual_arm_description：机器人“长什么样”

关键文件：

- `urdf/dual_arm_1kg.urdf.xacro`：当前模型主文件、关节、传动和 ros2_control 硬件插件参数。
- `meshes/`：视觉模型和部分碰撞模型。
- `urdf.rviz`：模型显示配置；CMake 中必须安装它，否则 RViz 可能回到 `map` 固定坐标并报错。
- `launch/display_rviz2.launch.py`：只看模型的低风险入口。
- `joint_control_panel.py`：直接发布 `/joint_states` 的 Tk 滑块演示，不是控制器。
- `scripts/move_to_pose.py`：先调用 `/compute_ik`，再向 `/move_action` 发送末端目标。

`joint_control_panel.py` 只能用于没有其他 `/joint_states` 发布者的模型演示。真机、mock ros2_control 或 SOEM 正在发布关节状态时不要启动它，否则状态源会冲突。

### 6.2 dual_arm_moveit_config：规划系统的大脑配置

关键文件：

- `dual_arm_1kg.srdf`：规划组、命名姿态、末端执行器语义。
- `kinematics.yaml`：MoveGroup 使用 PickIK，超时 `0.5 s`、3 次尝试。
- `ompl_planning.yaml`：当前主要使用 RRTConnect；Jazzy 字段是 `planning_plugins` 复数列表。
- `joint_limits.yaml`：MoveIt 速度、加速度限制。每个关节必须声明非零加速度限制，否则时间参数化会失败。
- `moveit_controllers.yaml`：把 MoveIt 的执行请求映射到左右 FollowJointTrajectory action。
- `ros2_controllers.yaml`：mock 模式，JTC 使用 position command interface。
- `ros2_controllers_real.yaml`：真机模式，JTC 使用 velocity command interface，当前 PID 约为 `p=1.0, d=0.1`。
- `launch/move_group.launch.py`：加载 xacro 和 YAML 字典，启动 MoveGroup；Octomap 因无 3D 传感器而禁用。

为什么 YAML 必须解析成字典：MoveIt 节点需要的是参数树内容，不是配置文件路径字符串。路径能被 launch 接受并不代表 MoveIt 得到了内部字段，这类错误通常启动不报错、随后规划器空配置或崩溃，调试耗时很长。

### 6.3 dual_arm_servo：实时末端速度控制

关键文件：

- `config/servo_left.yaml`、`servo_right.yaml`：左右 Servo 参数。
- `config/kinematics.yaml`：Servo 单独使用 KDL，超时 `0.005 s`、1 次尝试。
- `launch/servo.launch.py`：启动两个 Servo 节点、独立 planning scene monitor 和可选 Marker。
- `rviz_servo_marker.py`：RViz 交互球到 Twist 的闭环适配器。
- `keyboard_teleop.py`：键盘到 Twist 的简单测试输入。

当前 Servo 的主要量级：

- 发布周期 `0.01 s`，约 100 Hz；
- 线速度缩放 `0.2`；
- 角速度缩放 `0.8`；
- 输入超时 `0.1 s`；
- 奇异点阈值约 `200/400`；
- 关节边界余量 `0.12 rad`；
- 在线碰撞检查关闭；
- 额外平滑插件关闭。

MoveGroup 用 PickIK，Servo 用 KDL并不矛盾。MoveGroup 看重找到目标解的能力；Servo 每 10 ms 求一次局部速度，更看重确定性和速度。若把慢 IK 直接放入 Servo，容易超时、抖动或丢帧。

#### RViz Marker 为什么“松开鼠标才动”

Marker 采用明确的拖动状态机：

1. `MOUSE_DOWN`：停止当前命令；
2. `POSE_UPDATE`：只更新目标，不发送运动；
3. `MOUSE_UP`：锁定目标，开始闭环追踪；
4. 每个控制周期用“目标位姿 - 当前 TF 位姿”得到误差；
5. 误差乘比例增益并做速度限幅；
6. 进入位置/姿态容差后发零速度，并把 Marker 拉回真实末端。

这避免拖动过程中大量跳跃目标直接作用于机械臂，也便于初学者先观察目标再执行。

### 6.4 dual_arm_bringup：启动编排和自动预备位

关键文件：

- `launch/control_base.launch.py`：共享底座，只启动 robot_state_publisher、ros2_control 和左右 JTC。
- `launch/sim.launch.py`：mock 顶层入口。
- `launch/real.launch.py`：真机顶层入口。
- `scripts/move_to_ready.py`：有安全检查的一次性预备位动作。
- `config/ready_pose.yaml`：预备位和容差。
- `scripts/zmq_bridge_node.py`：面向 LeRobot/相机的独立 ZMQ v1 桥，当前不在主启动链路。
- `launch/lerobot_bridge.launch.py` 与 `config/lerobot_bridge.yaml`：LeRobot 数据桥的独立入口和安全默认配置。

`control_base.launch.py` 串行启动 controller spawner：先 broadcaster（仿真时），再左 JTC，再右 JTC。这样避免多个 spawner 同时等待/操作 controller manager 引发竞态。

`sim.launch.py` 和 `real.launch.py` 都保证 `mode:=moveit|servo` 二选一。VR 启用时 Marker 会自动关闭，防止两个输入适配器争用 Servo。

#### 自动预备位的完整安全逻辑

当且仅当传入 `move_to_ready_on_start:=true` 时：

1. 等待 `/joint_states` 包含 14 个关节、数值有限且连续稳定；
2. 对每只手臂分别判断当前位置接近全零或 ready；未知中间位置默认拒绝；
3. 检查绝对关节值不超过 `3.2 rad`；
4. 等待左右 JTC 都为 `active`；
5. 调用 SOEM `enable(true)` 打开软件门；
6. 同时给左右臂发送 12 秒轨迹；不是先左后右，也不是总共 24 秒；
7. 等两个 action 都结束，再用真实编码器检查最终误差，当前容差约 `0.08 rad`；
8. 任一步失败都取消动作、调用 SOEM stop，并让 `real.launch.py` 整体退出；
9. 只有成功后才启动 MoveGroup/Servo/RViz/VR，避免上层命令抢占 ready 动作。

注意：JTC 报告 goal reached 不能单独证明实物到位，所以第 7 步不可省略。

### 6.5 dual_arm_control：真实反馈如何进入 ros2_control

`DualArmHardware` 是 pluginlib 硬件插件：

- `on_init()` 读取 URDF 中定义的关节与接口；
- `on_configure()` 创建内部 rclcpp 节点，按 sensor-data/best-effort QoS 订阅 `/joint_states`；
- `export_state_interfaces()` 暴露位置、速度、力矩状态；
- `export_command_interfaces()` 暴露位置、速度命令槽；
- `read()` 调用 `spin_some()`，把最新反馈交给 controller manager；
- `write()` 当前不直接把命令送到硬件。

这是理解真机闭环的关键：如果反馈 QoS 不匹配、关节名缺失或时间太旧，JTC 看到的状态就不可信。真机出现“输出等于参考”“PID 像开环”时，优先检查 `/joint_states` 是否真正进入硬件插件，而不是立即调 PID。

### 6.6 dual_arm_soem_bridge：电机通信与最后一道软件门

这一包分成两层：

- `soem_bridge_node.cpp`：ROS 参数、话题、服务、日志和速度限幅；
- `soem_master.cpp`：SOEM 初始化、PDO 映射、1 ms 实时循环、CiA402 状态机和单位换算。

当前运行模式是 CiA402 **CSV（Cyclic Synchronous Velocity，模式 9）**。仓库中 PP（Profile Position）文档和示例是重要历史实验，但不是当前主控制路径。

当前配置的 14 个电机从站为 `2..15`，即跳过前面的分线器/非电机从站。每轴配置包括：

- EtherCAT slave 编号；
- 正负方向；
- 编码器零偏置；
- 编码器位数；
- 减速比；
- 软件关节范围；
- 最大关节速度。

配置文件是 `dual_arm_soem_bridge/config/soem_bridge.yaml`。标定值与机械结构绑定，不要为了“看起来对称”随意复制左右参数。

#### 从弧度到电机计数

概念上可写成：

```text
关节角 rad
  × 减速比
  × 编码器每转计数 / 2π
  × direction
  + zero_offset
  = 电机目标计数
```

速度换算类似，但不加零偏置。反向反馈则做逆运算。任一 `direction`、减速比或零偏置错误，都会表现为方向反、比例错或零位错。

#### CiA402 状态机

驱动不会一上电就接收速度。实时线程根据 statusword 逐步写 controlword，使驱动经过：

```text
Switch on disabled
  → Ready to switch on
  → Switched on
  → Operation enabled
```

项目默认按轴依次使能，间隔约 100 ms，减少 14 轴同时切换的冲击。错误位出现时，RT 状态机会尝试 fault reset；`clear_fault` 服务当前主要返回确认，真正复位逻辑在 RT 线程。

#### 软件命令门与 300 ms 看门狗

驱动 `Operation enabled` 和 ROS 软件命令门是两个不同状态：

- 驱动使能：电机驱动器处于可接受命令的 CiA402 状态；
- 软件门：`send_enabled` 是否允许把速度写入 PDO。

JTC 输出需要连续刷新。任一正在控制的轴超过约 300 ms 没有新命令，RT 线程会：

1. 把所有轴目标速度清零；
2. 关闭软件命令门；
3. 锁存 watchdog 状态；
4. 要求操作者重新调用 `enable(true)`，不会自动恢复旧目标。

这种“全轴停 + 人工重新武装”用于避免通信恢复后突然继续旧轨迹。

当前已知边界：代码读取 WKC，但尚未形成完整的 WKC 异常安全停机策略；编码器错误码在正常采集时可能高频出现，因此没有把每个错误码都设为严格停机条件。物理急停和现场监督仍是必须的。

### 6.7 vr_teleop_bridge：从手柄位姿到平滑速度

主路径是 `vr_pose_to_servo_node.py`：

```text
/vr/*/pose + /vr/*/enabled + /joint_states + Servo status
  → 计算手柄速度并变换坐标
  → 速度死区、滞回、缩放、限幅、超时保护
  → /servo_left|right/delta_twist_cmds
```

Grip 是 deadman/clutch：

- 按下时建立新的运动参考并允许发布速度；
- 松开时立即发布零速度并清空历史；
- Servo 节点本身保持运行，不因每次 Grip 松开而反复 pause/unpause。

这样避免 Servo 生命周期切换造成的延迟和偶发命令丢失。

#### “真正的速度死区”是什么意思

VR 设备只给位姿，不直接给可靠速度，因此仍然需要两帧或多帧做差：

```text
线速度 ≈ (当前位置 - 过去位置) / 时间差
角速度 ≈ 四元数相对旋转的旋转向量 / 时间差
```

关键区别不是“要不要做差”，而是死区作用在哪个物理量上：

- 位姿死区：单帧位移小于某个距离就丢掉。它依赖帧率，慢而连续的手部运动可能每帧都太小，机械臂完全不跟。
- 速度死区：先用约 100 ms 的多帧窗口计算 `m/s` 和 `rad/s`，再判断速度是否足够大。相同真实速度在 45 Hz 或 50 Hz 下更一致。

项目还使用滞回：线速度约在 `0.02 m/s` 以上才开始运动，开始后降到 `0.01 m/s` 以下才停止；角速度约为 `0.05/0.025 rad/s`。两个阈值能防止噪声刚好在一个边界附近反复开关。

100 ms 窗口兼顾两件事：比相邻两帧更抗抖，又不会像很长平均窗口那样明显拖手。时间戳异常、间隔过大或输入超过约 `0.2 s` 未更新时，bridge 会清空历史并发送零速度。

#### VR 与机器人坐标映射

当前 bridge 对线速度和角速度统一做一个 proper rotation：

```text
robot_x =  VR_y
robot_y = -VR_x
robot_z =  VR_z
```

这相当于绕 z 轴旋转 `-90°`，行列式为 `+1`，不会引入镜像手性错误。旋转轴必须与平移轴使用同一旋转矩阵；只交换 x/y 而漏掉负号会形成反射，roll/pitch/yaw 的直觉会混乱。

### 6.8 轨迹记录器

`trajectory_logger_node.py` 默认以约 5 Hz 新建时间戳目录，记录 6 个 CSV：

```text
vr_left_hand_trajectory.csv
vr_right_hand_trajectory.csv
robot_left_ee_trajectory.csv
robot_right_ee_trajectory.csv
control_left_trajectory.csv
control_right_trajectory.csv
```

前四个记录手柄位姿和由 TF 得到的真实末端位姿；后两个记录 bridge 输出计数以及 Servo 的非零 warning。所有文件用 `sample_index` 对齐。它能回答两个非常实用的问题：

- bridge 是否真的向 Servo 发了非零命令；
- Servo 是否收到后拒绝/抑制了命令。

但它不能直接证明 EtherCAT PDO 已写入，也不能替代电机端反馈检查。

### 6.9 Unity、PICO 与 ROS TCP Endpoint

Unity 工程主业务脚本是：

```text
vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs
```

它每帧检查左右 XR 设备，并按目标频率约 50 Hz 发布：

- `/vr/left_hand/pose`、`/vr/right_hand/pose`；
- `/vr/left_hand/enabled`、`/vr/right_hand/enabled`；
- `/vr/status`。

`CommonUsages.gripButton` 对应侧面 Grip。脚本用 Unity Robotics 的 `To<FLU>()` 把 Unity 数据转换成 ROS FLU 表示，bridge 再按本项目 base_link 约定做最终轴旋转。

ROS-TCP-Connector 在头显端是客户端，工作区中的 `ros_tcp_endpoint` 在电脑端监听 TCP 10000。Prefab 保存电脑 IP 和端口；Wi-Fi DHCP 地址可能改变，修改后需要重新构建/安装 APK。网络有客户端隔离时可用 `adb reverse`，并把 Unity 端地址改为 `127.0.0.1` 后重建。

PICO SDK 与 ROS-TCP Endpoint 都是第三方代码。学习时先理解它们的输入输出边界，不需要逐行阅读整个 SDK。

### 6.10 实验性或辅助模块

- `vr_pose_to_move_group_node.py`：把 VR 相对位姿离散地变成 IK，再发送双臂 MoveGroup 目标。默认目标周期约 1 秒、速度缩放 0.1，适合研究，不是当前柔顺 VR 主路径。
- `zmq_bridge_node.py`：把真实关节状态作为 observation、左右 JTC reference 作为
  VR 示教 action，并发布三路相机。示教 action 是向外发送的训练标签，不受动作门
  影响；只有训练后由 LeRobot 发回的策略 action 才受
  `allow_action_commands + /zmq_bridge_node/enable_actions` 限制。这个上游门与原有
  `/soem_bridge_node/enable` 电机门相互独立。它有独立 launch，但不会加入默认
  sim/real 主链。详见 `docs/lerobot_integration.md`。
- `tools/kick_servo_state_monitor.py`：给完整 `/joint_states` 副本注入极小变化，规避特定 Servo 启动等待。默认只允许 mock；真机必须明确确认且保持控制静止。
- `utils/bake_joint_offset.py`：把关节零点偏移烘焙进 URDF `origin rpy`，属于模型标定工具，输出必须经可视化和方向验证。
- `utils/plot_trajectory.py`：画 SOEM/JTC 参考位置、反馈位置和输出速度；当前参数解析较简单，主要按最新或指定 CSV 工作。

---

## 7. 构建、环境与启动规则

### 7.1 干净构建

```bash
cd /home/dell/dual_arm
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 会让 Python、YAML 和 launch 资源以软链接方式安装，许多修改只需重启节点；C++、CMake、package.xml 修改仍需重新构建。

单包构建示例：

```bash
colcon build --symlink-install --packages-select dual_arm_soem_bridge
```

修改依赖链较上游的模型或接口时，建议完整构建，避免 install 中残留旧文件。

### 7.2 每个新终端都要 source

```bash
source /opt/ros/jazzy/setup.bash
source /home/dell/dual_arm/install/setup.bash
```

如果出现 `ros2: invalid choice: control` 之类错误，通常不是项目功能错误，而是把不存在的 ros2 CLI 子命令当成命令，或相关 CLI 扩展未安装。查看控制器应调用服务：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

### 7.3 root 与普通用户的 DDS 隔离

真机通常用 `sudo bash -c "source ... && ros2 ..."` 启动。如果节点由 root 启动，诊断命令也应在相同 root 环境执行，否则 DDS 发现可能不同，看起来像“话题不存在”。不要一部分节点用 root、一部分关键控制节点用普通用户混跑。

### 7.4 清理异常残留进程

VR/Servo 异常退出后，可执行：

```bash
cd /home/dell/dual_arm
tools/clean_ros_runtime.sh
```

同时需要启动 TCP Endpoint 时：

```bash
tools/clean_ros_runtime.sh --start-endpoint
```

该脚本会终止工作区 ROS 进程、MoveIt Servo、controller manager、RViz、临时诊断进程和 ROS daemon。它会影响当前用户的相关运行任务，使用前确认没有其他实验正在进行。

---

## 8. 循序渐进实验总表

| 级别 | 实验 | 是否接触电机 | 主要学习目标 |
|---|---|---:|---|
| A0 | 阅读与构建 | 否 | 包、依赖、overlay |
| A1 | 只显示 URDF | 否 | link/joint/TF/坐标轴 |
| A2 | 滑块模型演示 | 否 | `/joint_states` 与 TF |
| B1 | MoveIt mock 规划 | 否 | IK、规划、JTC、执行 |
| B2 | `move_to_pose.py` | 否 | 服务、Action、错误码 |
| B3 | RViz Servo mock | 否 | Twist、闭环 Marker、奇异点 |
| B4 | 键盘 Servo mock | 否 | 单轴方向和命令超时 |
| C1 | Unity/TCP 只看数据 | 否 | 网络链路、频率、Grip |
| C2 | VR Servo mock | 否 | 坐标映射、速度死区、日志 |
| D1 | EtherCAT 枚举与反馈 | 是，但不应运动 | 从站、PDO、编码器、命令门 |
| D2 | 单轴低速验证 | 是 | 方向、比例、watchdog |
| E1 | 真机 MoveIt 手动恢复 | 是 | 当前姿态规划、人工开门 |
| E2 | 真机自动 ready | 是 | 启动安全检查与同步双臂 |
| E3 | 真机 RViz Servo | 是 | 低速末端伺服 |
| E4 | 真机 VR Servo | 是 | 完整遥操作链 |

每一级都应保存截图、关键日志和结论。上一层未通过，不进入下一层。

---

## 9. A 级实验：模型与 ROS 基础

### 9.1 A0：构建工作区

目的：确认依赖和安装空间完整，不启动任何机器人节点。

```bash
cd /home/dell/dual_arm
source /opt/ros/jazzy/setup.bash
colcon list
colcon build --symlink-install
```

通过标准：9 个构建单元成功完成，没有 package not found。警告可以记录，但不能忽略编译错误。

### 9.2 A1：只显示模型

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch dual_arm_description display_rviz2.launch.py
```

这个独立显示 launch 当前读取静态 `dual_arm_1kg.urdf`，并且已经自带
`joint_state_publisher_gui`。正常 sim/real 主链读取的是 xacro；如果刚修改过 xacro
但没有同步生成静态 URDF，两种显示可能不同。此时应以 sim/real 展开的 xacro 为准，
不要把静态显示差异误判为控制器故障。

观察任务：

1. RViz Fixed Frame 应为 `base_link` 或模型中存在的固定坐标；
2. 展开 RobotModel，识别左右 7 个 link；
3. 打开 TF 显示，确认两条链都从 `base_link` 连续到 7 轴末端；
4. 从坐标轴确认 `+x`、`-y`、`+z` 的实际空间方向；
5. 检查 visual 是否正常，碰撞几何是否明显过大。

通过标准：无断裂 TF、无 `map frame does not exist`、两臂模型位置合理。

### 9.3 A2：滑块理解 JointState

直接使用 A1 随 launch 启动的 `joint_state_publisher_gui`，一次只移动一个关节，
记录正角度时 link 的转向。GUI 发布的是“状态”，不是控制命令，所以模型会立即
跳到滑块值；这不代表真实控制器能这样运动。

仓库还保留 `dual_arm_description/joint_control_panel.py`，它会额外发布一份
`/joint_states`，用于早期模型演示。不要在 A1 的 GUI、ros2_control、SOEM 或其他
状态发布者运行时再启动它，否则多个状态源会互相覆盖。

---

## 10. B 级实验：MoveIt 与 Servo 仿真

### 10.1 B1：MoveIt mock 规划

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=moveit
```

在 RViz 中：

1. 选择 `left_arm`，先拖一个很小的目标；
2. 点击 Plan，只看轨迹；
3. 确认轨迹合理后点击 Execute；
4. 对右臂重复；
5. 最后再测试 `dual_arm`，观察双臂同时规划。

通过标准：左右 JTC active，规划轨迹无明显穿模，执行后 `/joint_states` 和 TF 连续变化。

常见失败：

- `NO_IK_SOLUTION`：目标不可达、姿态约束过严或接近奇异；
- 时间参数化失败：检查所有关节 acceleration limit 是否非零；
- MoveGroup 启动后崩溃：检查复杂 collision STL 和 YAML 是否以字典传入；
- 控制器不可用：检查 `moveit_controllers.yaml` 名称与 JTC action 名是否一致。

### 10.2 B2：分离 IK 与规划问题

保持 B1 运行，另开终端：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 run dual_arm_description move_to_pose.py --ros-args \
  -p group:=left_arm -p ee_link:=laxis7_link \
  -p x:=0.3 -p y:=0.0 -p z:=0.5 \
  -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0
```

脚本先调用 `/compute_ik` 并打印 MoveIt 错误码，IK 成功后才发 `/move_action`。这能区分：

- IK 本身无解；
- IK 有解但路径规划失败；
- 规划成功但控制器执行失败。

不要机械复制示例坐标。先在 RViz 中读取当前末端附近的可达目标。

### 10.3 B3：RViz Servo Marker

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_rviz_servo_marker:=true
```

选择 RViz 顶部 **Interact**。蓝色球控制左臂，橙色球控制右臂。

实验顺序：

1. 只沿一个平移轴拖动 1～2 cm，松开；
2. 对三个平移轴分别验证正负方向；
3. 再对单个旋转轴做小角度测试；
4. 最后才允许两臂分别运动。

通过标准：松开后平稳追踪，进入容差后停止；Marker 回到真实末端；不持续打印 stale state 或 singularity halt。

若 Servo 一直打印 `Waiting to receive robot state update.`，先确认完整 14 轴状态，再在仿真控制静止时运行：

```bash
python3 tools/kick_servo_state_monitor.py
```

不要把此脚本当作正常启动步骤。持续需要 kick 说明状态监视链仍需诊断。

### 10.4 B4：键盘单轴测试

保持 Servo 模式运行，但关闭 Marker 或确保不操作它：

```bash
ros2 run dual_arm_servo keyboard_teleop
```

按键：`1/2` 选左右臂，`WASD+QE` 平移，`IJKLUO` 旋转。每次短按一个键并观察。该工具的常量输入较大，主要用于仿真方向验证，不推荐直接用于首次真机。

---

## 11. C 级实验：VR 链路与仿真

### 11.1 C1：先只验证 Unity 数据，不控制机械臂

清理并启动 TCP Endpoint：

```bash
cd /home/dell/dual_arm
source install/setup.bash
tools/clean_ros_runtime.sh --start-endpoint
```

启动 PICO 应用后检查：

```bash
ros2 topic list -t --no-daemon
ros2 topic hz /vr/right_hand/pose
ros2 topic echo /vr/right_hand/enabled --once
ros2 topic echo /vr/status --once
```

通过标准：

- 左右 pose 都稳定发布，典型频率约 44～50 Hz；
- Grip 按下/松开对应 enabled true/false；
- status 显示设备 connected；
- 手柄静止时位姿只有小幅噪声，没有大跳变。

此阶段不要启动 Servo。先解决 IP、端口、APK、Wi-Fi 或 ADB 问题。

进入 C2 前先执行一次 `tools/clean_ros_runtime.sh`，关闭 C1 单独启动的 Endpoint；
C2 的顶层 launch 会自行启动新的 Endpoint，避免两个进程争用 10000 端口。

### 11.2 C2：VR Servo mock

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_vr_teleop:=true \
  enable_ros_tcp_endpoint:=true \
  ros_tcp_port:=10000
```

操作顺序：

1. 两只 Grip 松开，等待 Servo、TF、VR status 正常；
2. 双手水平伸直、手心向上，建立容易记忆的参考姿态；
3. 只按右 Grip，沿一个方向缓慢移动；
4. 松开 Grip，确认机械臂立即停止；
5. 对三个平移方向、三个旋转方向逐项记录；
6. 再测试左手；最后才测试双手。

Grip 每次按下都相当于重新接合离合器，机械臂不应跳到手柄的绝对世界位置，而是跟随按下后的相对运动。

通过标准：慢速持续运动能被识别；静止抖动不会驱动机械臂；松开 Grip 发零速度；前后、左右、上下与第 4.3 节坐标约定一致。

### 11.3 如何用日志判断卡在哪一层

默认日志目录：

```text
/home/dell/dual_arm/vr_teleop_bridge/log/<启动时间>/
```

按以下顺序比较：

1. `vr_*_hand_trajectory.csv`：enabled 是否为 1，手柄位姿是否变化；
2. `control_*_trajectory.csv`：bridge 是否产生非零 Twist；
3. `robot_*_ee_trajectory.csv`：末端实际位姿是否变化；
4. Servo status/warning：非零输入是否被奇异点、边界或状态过期抑制。

典型判断：

| 现象 | 最可能层级 |
|---|---|
| VR 位姿不变 | Unity/XR/TCP |
| VR 变化但 enabled=0 | Grip 映射 |
| enabled=1、VR 变化、control=0 | bridge 时间窗/死区/超时 |
| control 非零、末端不动、Servo warning 增加 | Servo 拒绝/抑制 |
| Servo/JTC 仿真可动、真机不动 | JTC→SOEM/命令门/驱动/编码器 |

---

## 12. 真机前必须掌握的 EtherCAT 与安全知识

这一章不是可选背景知识，而是真机实验的准入要求。

### 12.1 实物系统的四层状态

| 层 | 代表状态 | 由谁控制 | 关闭后的效果 |
|---|---|---|---|
| 物理安全层 | 急停、STO、驱动供电 | 硬件 | 切断或禁止驱动力矩，最高优先级 |
| EtherCAT/CiA402 层 | OP、Operation enabled | SOEM RT 状态机 | 驱动不接受正常速度命令 |
| ROS 软件命令门 | `send_enabled` | `/enable`、`/stop`、watchdog | PDO 速度目标清零/不发送 |
| 上层控制层 | MoveGroup/Servo/JTC | launch、Action、Servo 服务 | 不再生成新轨迹或速度 |

“电机已使能”不等于“软件门已打开”；“Servo 已暂停”也不等于“驱动失能”。排障时必须说清楚是哪一层。

### 12.2 14 轴配置与跳过分线器

`soem_bridge.yaml` 的 `axis_slaves` 当前为从站 2～15，依次对应：

```text
index 0..6   → laxis1_joint .. laxis7_joint
index 7..13  → raxis1_joint .. raxis7_joint
slave 2..8   → 左臂 7 轴
slave 9..15  → 右臂 7 轴
```

这张映射表是“ROS 关节顺序—测试 index—EtherCAT slave”的共同依据。更换接线、从站顺序或分线器后必须重新执行 `slaveinfo` 并逐轴核对，不能假设编号不变。

### 12.3 PDO 是什么

PDO 可以理解为 EtherCAT 每个周期交换的固定二进制数据区。当前主站给驱动的 RxPDO 主要含 controlword、目标速度、速度偏置、工作模式；从驱动收的 TxPDO 主要含 statusword、实际位置、实际速度、力矩、错误码和模式显示。

多字节整数通常按 little-endian 解码。仓库的 [`pdo_byte_decoding.md`](../SOEM/docs/pdo_byte_decoding.md) 给出逐字节示例。遇到位置数值荒谬时，应同时检查：

- PDO mapping 的对象顺序和位宽；
- 有符号/无符号解释；
- little-endian 字节序；
- 编码器回绕；
- 减速比、方向和零偏置。

### 12.4 分布式时钟与实时循环

`soem_master.cpp` 以约 1 ms 周期交换 PDO，并配置 Sync0。普通 Linux 调度可能造成抖动，因此线程会尝试 `SCHED_FIFO` 实时优先级 40。实时权限不足时程序可能仍运行，但周期抖动会增加。

实时循环中最忌讳长时间打印、阻塞服务调用和动态等待。ROS 服务只改变原子状态，真正的驱动切换在 RT 线程内完成，这正是线程分层的原因。

### 12.5 dry_run 的准确含义

当前实现中的 `dry_run=true` 主要让 bridge 打印转换结果并阻止速度命令提交，但如果 `ifname` 有效，节点仍会启动 EtherCAT 主站、映射 PDO并推进驱动状态。它适合检查单位换算，却不是不接硬件的纯离线模拟。

真正完全离线的验证应使用 mock launch，或在代码/配置明确不创建 EtherCAT master 的隔离环境中进行。

### 12.6 真机实验前检查单

每次真机运动前逐项口头确认：

- [ ] 至少两人在场，一人操作，一人看急停与机械环境；
- [ ] 物理急停已测试，操作者能立即触及；
- [ ] 两臂周围、上方和底座附近无人、无工具；
- [ ] 线缆有余量，不会卷入关节；
- [ ] EtherCAT 接口名仍是实际专用网卡；
- [ ] 从站数量、顺序与 14 轴映射一致；
- [ ] `/joint_states` 含完整 14 轴，位置有限且无大跳变；
- [ ] RViz 当前姿态与实物肉眼姿态一致；
- [ ] 左右 JTC 都为 active；
- [ ] 控制输入静止，左右 `output.velocities` 接近 0；
- [ ] SOEM 软件门当前关闭，除非正在执行受控 ready；
- [ ] 速度上限适合当前实验；
- [ ] 已另开终端准备 `/stop`；
- [ ] 已明确本次只测哪一臂、哪一轴、哪个方向和多长时间。

任一项不能确认，就不打开命令门。

---

## 13. D/E 级实验：从 EtherCAT 到完整真机

以下命令以当前项目约定使用 root 环境。完整速查仍以 [`run_commands.md`](run_commands.md) 为准。

### 13.1 D1：只做从站与反馈验证

先扫描从站：

```bash
sudo /home/dell/dual_arm/SOEM/build/samples/slaveinfo/slaveinfo enp0s31f6
```

观察从站数量、名称、状态、SM/FMMU/PDO 和 WKC。`slaveinfo` 会占用 EtherCAT 网卡，不能与 soem bridge 同时运行。

然后单独启动 SOEM bridge。即使软件命令门保持关闭，当前 RT 状态机仍会把驱动逐轴
推进到 Operation enabled；这可能接通电机力矩，不能把本步骤当作无能量的纯读取。
物理急停、现场清空和双人监护要求与运动实验相同：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"
```

此时保持软件门关闭，检查：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 topic echo /joint_states --once"

sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 topic hz /joint_states"
```

当前反馈典型约 20 Hz。人工缓慢移动允许自由移动的关节或按设备安全流程观察编码器，验证：

- 关节名顺序正确；
- 正方向与模型一致；
- 角度比例合理；
- 机械零位附近反馈合理；
- RViz 左右臂没有交叉或错轴。

停止条件：某一实际关节运动却是另一关节数值变化、角度突跳、方向无法解释、从站丢失或驱动意外运动。

### 13.2 D2：单电机低速测试

只能在 `real.launch.py` 和 JTC 都没有运行时进行，避免两个命令源写同一电机。

先准备软件 stop 终端，再开门：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"
```

以 50 Hz 持续刷新 index 0、`0.02 rad/s`：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 topic pub -r 50 /soem_bridge_node/test_axis \
std_msgs/msg/Float64MultiArray '{data: [0, 0.02]}'"
```

停止时先调用：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger '{}'"
```

然后再停止持续 publisher。只按 Ctrl-C 也会在约 300 ms 后触发 watchdog，但主动 stop 更清晰。watchdog 锁定后，下一次必须重新 `enable(true)`。

每次只测试一个 index 的正小速度，再测试负小速度，记录“index—slave—实物关节—ROS 关节名—正方向”。14 轴全部验证前，不进行多轴真机控制。

### 13.3 E1：MoveIt 真机手动恢复模式

这个流程故意不自动 ready，适合机械臂停在上次运行的中间位置。它让 MoveIt 先读取真实状态、规划恢复轨迹，再由人决定何时打开命令门。

终端 1：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_bringup real.launch.py mode:=moveit"
```

终端 2：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"
```

确认 14 轴状态、RViz 姿态、控制器 active，并在 RViz **先 Plan、不 Execute**。确认轨迹安全后才在终端 3 开门：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"
```

然后在 RViz Execute。恢复完成后调用 stop。

注意：如果门关闭时已经 Execute，JTC 参考可能继续变化。此时不要直接 enable，应取消轨迹、重新确认当前状态并重新规划。

### 13.4 E2：自动移动到镜像 ready

先启动 SOEM并验证反馈，再运行：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_bringup real.launch.py \
mode:=moveit move_to_ready_on_start:=true"
```

两臂应同时用约 12 秒到达 ready。只有当每只手臂起点接近全零或 ready 时才允许执行。未知中间位置被拒绝是预期安全行为，不要通过 `allow_unknown_start` 绕过。

通过标准：日志显示稳定 14 轴反馈、左右起点分类、两 JTC active、命令门打开、双臂目标接受、两个 action 成功、编码器最终误差通过，然后 MoveGroup/RViz 才启动。

### 13.5 E3：真机 RViz Servo Marker

终端 1 先启动 SOEM，终端 2 启动：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_bringup real.launch.py \
mode:=servo \
enable_rviz_servo_marker:=true \
move_to_ready_on_start:=true"
```

ready 成功后，Marker 和 Servo 才会出现。首先不拖 Marker，检查左右 `controller_state.output.velocities` 接近 0。Marker 自带自动切换 Twist 与解除 Servo pause 的逻辑；若服务未成功，再按 `run_commands.md` 手动调用。

首次测试：

1. 只选一只手臂；
2. 只沿一个轴移动约 5 mm～1 cm；
3. 松开鼠标，观察速度和停止；
4. 验证正负方向；
5. 再逐步增加到 2 cm；
6. 旋转测试最后进行。

当前 Marker 默认限速比键盘低，但 Servo 在线碰撞关闭，仍需保持小范围。

### 13.6 E4：完整真机 VR Servo

从 Marker 切到 VR 前先 stop 并退出对应 `real.launch.py`。SOEM 可保持运行，但不能启动第二个 SOEM master。

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && \
ros2 launch dual_arm_bringup real.launch.py \
mode:=servo \
move_to_ready_on_start:=true \
enable_vr_teleop:=true \
enable_ros_tcp_endpoint:=true \
ros_tcp_port:=10000"
```

启动时两只 Grip 必须松开。ready 成功、Servo/VR/TCP 启动后，依次检查 enabled、status、手柄 pose、bridge Twist 和 JTC output。第一次只按一只 Grip，只做单方向短距离运动。

完整停止顺序：

1. 松开两只 Grip；
2. 调用 `/soem_bridge_node/stop`；
3. 确认速度归零；
4. 退出 VR/Servo launch；
5. 按实验室驱动断能和断电流程结束 SOEM。

---

## 14. 调试方法：永远沿数据流逐段定位

### 14.1 通用六问

遇到“不动”“抖”“方向错”时依次问：

1. **输入存在吗？** 话题有发布者、频率正常、字段变化吗？
2. **适配器发命令了吗？** bridge/Marker 输出是不是非零？
3. **Servo/MoveGroup 接受了吗？** status、warning、action result 是什么？
4. **JTC 产生输出了吗？** `controller_state.output.velocities` 是否非零且连续？
5. **SOEM 允许发送吗？** 软件门、watchdog、从站状态、限幅如何？
6. **反馈变化了吗？** 编码器、`/joint_states`、TF 和实物是否一致？

不要一看到机械臂不动就同时改坐标、PID、死区、watchdog 和 Servo 参数。一次只改变一层，测试后保留明确结论。

### 14.2 最有用的诊断命令

仿真普通用户可直接运行。真机 root 节点应把命令包在相同的 `sudo bash -c "source ... && ..."` 中。

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /joint_states --verbose
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
ros2 topic echo /left_arm_controller/controller_state --once
ros2 run tf2_ros tf2_echo base_link laxis7_link
```

VR 逐层：

```bash
ros2 topic hz /vr/right_hand/pose
ros2 topic echo /vr/right_hand/enabled --once
ros2 topic echo /servo_right/delta_twist_cmds --once
ros2 topic echo /right_arm_controller/controller_state --once
```

### 14.3 “右手向右但右臂不动”

按这个决策顺序：

1. `/vr/right_hand/pose` 的对应位置是否连续变化？否：Unity/XR/TCP。
2. `/vr/right_hand/enabled` 是否为 true？否：Grip。
3. control CSV 或 `/servo_right/delta_twist_cmds` 是否非零？否：速度窗口、死区、时间戳或 bridge 超时。
4. Servo warning 是否出现 singularity/joint bound/stale state？是：Servo 抑制。
5. JTC output 是否非零？否：Servo→JTC 或控制器状态。
6. SOEM 日志是否 `trajectory ignored: send not enabled` 或 watchdog latched？是：重新检查后 re-arm。
7. 编码器不变但 PDO 有命令：驱动状态、轴映射、线缆或硬件安全回路。

### 14.4 前后方向跟随差

先区分“方向错”和“幅度/响应差”：

- 方向错：检查 `robot_y = -VR_x` 是否在线速度与角速度都一致，避免在多个层重复变换。
- 慢动作丢失：比较 VR 原始轨迹与 control CSV，检查是否被速度 start threshold 吃掉。
- control 正常但末端小：查看 Servo 缩放、奇异点速度缩放和 JTC输出。
- 只有某个空间方向困难：查看当前 Jacobian/奇异点，不要立刻归因于 VR。

### 14.5 旋转轴混乱

不要直接比较欧拉角数值。推荐：

1. 固定 Grip 参考姿态；
2. 每次只绕手柄自身一个直观轴旋转；
3. 用四元数相对旋转转成 rotation vector；
4. 对该向量应用与线速度相同的坐标旋转；
5. 查看 bridge angular Twist；
6. 再观察机械臂末端在 base_link 中的实际旋转轴。

欧拉角存在旋转顺序和万向锁问题，四元数的 `x/y/z` 也不是 roll/pitch/yaw 本身。

### 14.6 Servo 明明收到命令却不动

重点检查：

- `/joint_states` 是否完整、新鲜且 QoS 匹配；
- Servo 是否切到 Twist command type、是否未暂停；
- 输入 `header.frame_id` 是否为 `base_link`；
- `incoming_command_timeout` 是否被超出；
- 奇异条件数是否越过 slowdown/hard stop；
- 是否接近关节限制余量；
- 左右 Servo 是否各自拥有可用 planning scene monitor；
- JTC 是否允许滚动轨迹末点带非零速度。

本项目已设置 `allow_nonzero_velocity_at_trajectory_end: true`，否则 Servo 的短滚动轨迹可能被 JTC 拒绝。

### 14.7 真机只动一只手臂

先看两个层面：

- 启动时：两个 JTC 是否都 active？早期 ready 逻辑曾在右控制器尚未出现时过早失败，当前实现会等待两者。
- 执行时：左右 action 是否同时发送？当前 ready 是并发发送，12 秒同时完成；若只有一边动，检查另一边 action result、JTC output、SOEM axis target 与编码器。

### 14.8 抖动但几乎不位移

可能原因从上到下：

- 目标时间过长，JTC 要求速度接近噪声量级；
- 编码器噪声经速度 PID 被放大；
- 参考位置与反馈位置不在同一零点/方向；
- 控制周期或反馈频率抖动；
- 速度命令断续触发 watchdog；
- 接近奇异点导致 Servo 速度缩放反复变化。

不要先增大 PID。先画 `ref_pos`、`fb_pos`、`output_vel`，确认误差与输出的因果关系。

### 14.9 Watchdog 锁定

典型日志：

```text
command watchdog latched: all axes stopped; call ~/enable with data=true to re-arm
trajectory ignored: send not enabled
```

正确处理：

1. 找出为什么某轴命令超过 300 ms 未刷新；
2. 停止旧 publisher/轨迹；
3. 确认所有输出为零、实物安全；
4. 重新调用 `enable(true)`；
5. 重新开始一个新目标。

不要把 watchdog 无限放宽到掩盖控制链中断。300 ms 已是稳定性与停机响应之间的折中。

### 14.10 编码器错误高频出现

当前设备即使正常运行也可能高频报告某些错误码，所以软件没有采用“任意一次错误立即禁止采集”的严格策略。判断时结合：

- position/velocity 是否连续；
- statusword 主状态是否正常；
- error code 是否持续为同一硬故障；
- WKC、从站状态是否异常；
- 实物是否有异常声音/运动。

这是一项已知安全能力边界。未来应对错误码分类，而不是简单全部忽略或全部停机。

### 14.11 MoveGroup/FCL 崩溃

排查顺序：

1. 确认 `robot_description_kinematics`、planning、OMPL 和 controller YAML 以解析后的字典传入；
2. 确认 OMPL 使用 Jazzy 的 `planning_plugins` 字段；
3. 确认每关节非零 acceleration limit；
4. 简化大 STL collision mesh；
5. 没有 3D 传感器时保持 Octomap 明确禁用。

详见 [`moveit_rviz_ik_marker_debugging.md`](moveit_rviz_ik_marker_debugging.md) 和 [`ros2_debugging_notes.md`](ros2_debugging_notes.md)，但以当前配置为准。

---

## 15. 日志、数据分析与可重复实验

### 15.1 实验记录的最小内容

每次调参至少记录：

- Git commit；
- 启动命令和参数文件；
- 仿真还是真机；
- 机械臂起始姿态；
- 测试手、方向、速度和持续时间；
- ROS/VR/SOEM 日志目录；
- 预期与实际结果；
- 是否触发 Servo warning、watchdog 或物理急停。

否则很难判断“这次更顺”来自参数、起始姿态、网络帧率还是操作者动作差异。

### 15.2 VR 六文件对齐分析

建议以同一个 `sample_index` 比较：

```text
手柄位姿变化
→ enabled
→ bridge 非零 Twist 计数
→ Servo warning
→ 末端位姿变化
```

位置单位是米，角度分析应使用四元数相对旋转，不要直接对四元数分量做差。日志将位姿保留约 4 位小数，适合行为分析，不适合高精度标定。

### 15.3 SOEM/JTC 轨迹图

```bash
python3 utils/plot_trajectory.py /path/to/log.csv
```

重点看三条量：

- `ref_pos`：JTC 希望到达哪里；
- `fb_pos`：编码器实际在哪里；
- `output_vel`：控制器为消除误差给了多少速度。

常见形状：

- ref 变、fb 不变、output 饱和：硬件命令未生效或机构卡住；
- ref≈fb 但 output 大噪声：反馈速度/PID/采样问题；
- output 断成短块：上游刷新或 watchdog；
- 单个方向比例不一致：轴换算/减速比/零偏置。

### 15.4 ROS 日志位置

普通用户通常在：

```text
/home/dell/.ros/log/
```

sudo 启动通常在：

```text
/root/.ros/log/
```

查日志时先确认进程由谁启动。VR 轨迹日志则固定由其配置决定，当前在工作区 `vr_teleop_bridge/log/`。

---

## 16. 如何修改项目而不破坏已工作的链路

### 16.1 一次只改一层

推荐修改流程：

1. 写出当前现象和可测量指标；
2. 沿数据流确定最可能的一层；
3. 只改一个参数或一段逻辑；
4. 先做 mock 回归；
5. 再做低风险单轴/单臂实物验证；
6. 记录结果并提交小 commit；
7. 只有证据支持时才继续下一层。

### 16.2 常见修改对应的文件

| 想改什么 | 首要文件 | 必做回归 |
|---|---|---|
| 机器人几何/关节轴 | xacro | A1、A2、B1、方向检查 |
| MoveIt 规划/IK | MoveIt config YAML | B1、B2、碰撞检查 |
| Servo 速度/奇异阈值 | Servo YAML | B3、单轴、边界与停止 |
| Marker 手感 | `rviz_servo_marker.py` + YAML | B3 左右/六方向 |
| VR 坐标/死区 | `vr_pose_to_servo_node.py` + YAML | C1、C2、六文件日志 |
| 真机 PID | `ros2_controllers_real.yaml` | mock 不充分，D/E 低速跟踪图 |
| 电机方向/零偏置/减速比 | `soem_bridge.yaml` | D1、D2 每轴正负方向 |
| Watchdog/CiA402/PDO | SOEM C++ | 隔离测试、断流测试、物理急停 |
| ready 姿态/时间 | `ready_pose.yaml` | 仿真镜像、起点分类、并发 action、真机监督 |
| Unity 话题/Grip | `VRHandPublisher.cs` | 重新构建 APK、C1 |

### 16.3 不能靠“看起来正常”验收的修改

- 坐标变换：必须逐轴正负验证，并检查旋转矩阵行列式为 +1；
- 编码器标定：必须与独立物理基准核对；
- watchdog：必须主动中断命令流验证停机和 re-arm；
- ready：必须验证未知起点拒绝、单臂 action 失败能使双臂停止；
- QoS：必须用 `ros2 topic info --verbose` 验证端点兼容；
- Servo 平滑：必须同时比较输入、输出和实际末端，而不是只凭主观手感。

### 16.4 Git 工作方式

```bash
git status --short
git diff --check
git diff
```

提交前确保：

- 没有把 `build/`、`install/`、运行日志和 APK 临时产物误提交；
- 没有覆盖与本次问题无关的用户修改；
- 文档命令与实际 launch 参数一致；
- C++ 已重新构建，Python/YAML 至少做语法检查；
- 对应级别回归完成。

---

## 17. 当前能力边界与后续课题

这些不是“马上随手修掉”的小问题，而是适合课程设计或毕业设计继续研究的方向：

1. **Servo 碰撞保护**：简化全部 collision mesh 后重新启用并评估实时开销。
2. **EtherCAT 通信安全**：把 WKC、从站掉线、状态字异常分级接入停机策略。
3. **编码器错误分类**：统计正常噪声与真实硬故障特征，建立分级告警而非全停/全忽略。
4. **硬件写链统一**：评估把 EtherCAT 命令真正集成进 ros2_control `write()`，减少独立订阅 controller_state 的旁路架构。
5. **端到端时延测量**：给 Unity、TCP、bridge、Servo、JTC、PDO 和编码器统一时间戳。
6. **VR 旋转人机工效**：用受控 roll/pitch/yaw 实验验证直觉映射和手腕舒适性。
7. **双臂协调约束**：加入相对位姿、共享物体和自碰撞约束，而不是两臂完全独立 Servo。
8. **自动化测试与 CI**：当前一方代码没有统一测试套件/格式化/CI，应先建立配置解析、坐标变换和安全状态机单元测试。
9. **LeRobot/ZMQ 接入**：dual_arm 侧 ZMQ v1、动作限幅/超时和独立安全门已经
   完成；后续需按交接文档完成 LeRobot 接收端、数据集验证、训练和 mock rollout。
10. **移动底盘**：URDF 有轮子但没有功能控制，需要独立完成运动学、驱动与安全链。

---

## 18. 文件索引：读什么、为什么读

### 18.1 当前运行真值

| 文件 | 阅读重点 |
|---|---|
| `dual_arm_description/urdf/dual_arm_1kg.urdf.xacro` | 几何、TF 链、关节、初值、ros2_control |
| `dual_arm_moveit_config/config/dual_arm_1kg.srdf` | 规划组、命名姿态、末端语义 |
| `dual_arm_moveit_config/config/*.yaml` | IK、OMPL、限制、JTC、MoveIt 控制器映射 |
| `dual_arm_bringup/launch/*.launch.py` | mock/real 和 moveit/servo 启动边界 |
| `dual_arm_bringup/scripts/move_to_ready.py` | 真机启动安全状态机 |
| `dual_arm_bringup/config/ready_pose.yaml` | ready 目标、时间、容差与白名单 |
| `dual_arm_bringup/scripts/zmq_bridge_node.py` | LeRobot ZMQ v1、输出示教 action 与输入策略安全门 |
| `dual_arm_bringup/config/lerobot_bridge.yaml` | LeRobot 端口、相机与动作检查阈值 |
| `dual_arm_control/src/dual_arm_hardware.cpp` | 编码器反馈进入 ros2_control |
| `dual_arm_servo/config/*.yaml` | 实时伺服参数 |
| `dual_arm_servo/dual_arm_servo/rviz_servo_marker.py` | Marker 位姿误差闭环 |
| `vr_teleop_bridge/vr_teleop_bridge/vr_pose_to_servo_node.py` | 主 VR 速度桥、坐标、死区、生命周期 |
| `vr_teleop_bridge/vr_teleop_bridge/trajectory_logger_node.py` | VR/机器人/控制对齐日志 |
| `dual_arm_soem_bridge/config/soem_bridge.yaml` | 14 轴硬件标定和安全参数 |
| `dual_arm_soem_bridge/src/soem_bridge_node.cpp` | ROS 服务、JTC输出、限幅、日志 |
| `dual_arm_soem_bridge/src/soem_master.cpp` | PDO、CiA402、实时循环、watchdog |
| `vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs` | PICO 手柄、Grip、位姿发布 |

启动文件也有明确层级：

- 日常入口：`dual_arm_bringup/launch/sim.launch.py`、`real.launch.py`；
- 被顶层包含：`control_base.launch.py`、MoveIt 的 `move_group.launch.py`/`rviz.launch.py`、Servo 的 `servo.launch.py`；
- 兼容或单独组合入口：MoveIt `moveit.launch.py`、Servo `servo_control.launch.py`；正常实验优先使用 bringup 顶层，避免漏节点或重复节点；
- VR 入口：`vr_teleop_bridge.launch.py` 为当前 Servo 主桥，`vr_with_tcp_endpoint.launch.py` 额外带 TCP Endpoint，`vr_move_group_bridge.launch.py` 是离散 MoveGroup 实验路径；
- SOEM 入口：`soem_bridge.launch.py`，必须独占 EtherCAT 网卡；
- 纯模型入口：`display_rviz2.launch.py`，它读取静态 URDF，不代表当前 xacro 一定已同步。

构建入口 `CMakeLists.txt` 和 `package.xml` 决定文件是否真正安装、依赖是否可发现。
“源码目录里有文件”不等于 `ros2 run/launch` 能找到它；新增脚本或配置后必须同步
安装规则。各 Python 可执行文件、C++ 节点和配置的安装关系可从每个包的
`CMakeLists.txt` 反查。

### 18.2 当前参考文档

| 文档 | 作用 |
|---|---|
| `README.md` | 工作区总体介绍与最短启动示例 |
| `AGENTS.md` | 当前 Jazzy 架构、构建约定、已知坑和修改风险规则 |
| [`run_commands.md`](run_commands.md) | 当前命令速查，尤其是真机多终端流程 |
| [`ros2_debugging_notes.md`](ros2_debugging_notes.md) | 已解决问题的技术原因与经验 |
| [`vr_usage.md`](vr_usage.md) | VR 使用说明 |
| [`unity_pico_notes.md`](unity_pico_notes.md) | Unity/PICO 构建、网络、ADB 经验 |
| [`cpp_basics.md`](cpp_basics.md) | 读 C++ 硬件代码前的基础 |
| [`moveit_rviz_ik_marker_debugging.md`](moveit_rviz_ik_marker_debugging.md) | MoveIt/RViz/IK 历史排障 |
| [`lerobot_integration.md`](lerobot_integration.md) | LeRobot 数据语义、ZMQ 协议和分级联调 |
| [`lerobot_handoff_2026-07-16.md`](lerobot_handoff_2026-07-16.md) | LeRobot 仓库待完成接口和验收清单 |
| `dual_arm_control/REAL_HARDWARE_GUIDE.md` | 早期真机硬件插件说明；PID 示例可能已过时，需对照当前 real YAML |
| `dual_arm_control/CUSTOM_CONTROLLER_TODO.md` | 尚未实现的自定义控制器设想，不属于当前运行链 |
| `dual_arm_moveit_config/docs/control_architecture.md` | MoveIt、JTC、硬件接口的数据流说明 |
| `dual_arm_moveit_config/docs/topics_and_services.md` | mock MoveIt 运行时的话题/服务观察样例，数量可能随版本变化 |
| `dual_arm_servo/README.md` | Servo 包设计与入口 |
| `dual_arm_description/AGENTS.md`、`notes.md` | 模型目录、关节从 fixed 到 revolute 的迁移与 RViz 安装经验 |

### 18.3 历史记录：有价值但需对照当前源码

- `.codex/README.md`、`NOTES.md`、`TASKS.md`、`SERVO_HANDOFF.md`：早期问题背景和交接。
- `docs/vr_teleop_handoff_2026-07-15.md`：一次 VR 平滑与坐标修复交接快照。
- `SOEM/docs/ec_sample_pp_*.md`：PP 模式实验及 target reached 经验，非当前 CSV 主路径。
- `SOEM/docs/CIA402_diagnostic_report.md`：驱动安全回路、Quick Stop/STO 等诊断。
- `SOEM/docs/slaveinfo_analysis.md`：从站、SM、FMMU、WKC 与 PDO 分析方法。
- `SOEM/README.md`：第三方 SOEM 工程自身说明。
- `vrtest-full-lite/docs/merge_dual_arm_plan.md`：Unity/ROS 工程合并历史规划。
- `vrtest-full-lite/vrtest-package/README.md`：随 Unity 工程保留的 ROS 端工具说明。
- `vrtest-full-lite/vrtest/AGENTS.md`：Unity 子目录早期维护说明，其中 ROS 版本或脚本行数可能已过时。
- `ros_tcp_endpoint` 的 README/CHANGELOG/CONTRIBUTING/CODE_OF_CONDUCT：第三方包说明，不是本机器人操作流程。

历史文档可能包含旧 ROS 发行版、旧 PID、旧 Servo 生命周期、旧坐标映射或旧 SRDF 状态。引用时必须在当前源码中再次确认。

### 18.4 第三方代码边界

以下目录体量很大，手册不逐文件覆盖：

- `SOEM/` 核心库：本项目主要调用初始化、从站配置、PDO 交换、DC 和状态读取 API；
- `third_party/PICO Unity Integration SDK-*`：本项目主要依赖 XR 设备输入和 PICO 构建支持；
- `ros_tcp_endpoint/`：本项目使用默认 server endpoint、publisher/subscriber 和 TCP 消息封装；
- Unity XR Interaction Toolkit samples：演示资产，不是双臂控制业务逻辑。

`SOEM/LICENSE.md`、PICO SDK 的 `LICENSE.md/README.md` 用于确认第三方来源、许可和
安装要求；ROS-TCP Endpoint 的 issue/PR 模板是上游协作模板。它们已纳入项目资料
范围，但不提供本机器人运行参数。

修改第三方库前应先证明问题位于库内，而不是本项目的参数、坐标、QoS 或状态机。

---

## 19. 常用参数速查

| 参数 | 当前量级/默认 | 含义 | 调大风险 |
|---|---:|---|---|
| Servo `publish_period` | 0.01 s | Servo 输出周期 | 周期变长会降低响应 |
| Servo `incoming_command_timeout` | 0.1 s | 输入过期时间 | 陈旧命令持续更久 |
| Servo linear scale | 0.2 | 归一化线速度比例 | 末端更快 |
| Servo rotational scale | 0.8 | 归一化角速度比例 | 转腕更快 |
| Servo singular threshold | 200/400 | 降速/硬停条件数 | 太大可能更靠近奇异 |
| VR velocity window | 约 0.1 s | 多帧速度估计 | 延迟增加、噪声降低 |
| VR linear start/stop | 0.02/0.01 m/s | 线速度滞回 | start 大会丢慢动作 |
| VR angular start/stop | 0.05/0.025 rad/s | 角速度滞回 | start 大会丢慢转动 |
| VR command timeout | 约 0.2 s | 输入陈旧停止 | 太大降低断联响应 |
| SOEM feedback rate | 20 Hz | `/joint_states` 发布 | 过高增加 ROS 负担 |
| JTC state rate | 100 Hz | controller_state 发布 | 过低可能触发断续 |
| SOEM max velocity | 0.2 rad/s | 每轴真机限幅 | 直接提高实物风险 |
| SOEM watchdog | 300 ms | 命令断流全停 | 太大延迟故障停机 |
| ready duration | 12 s | 双臂并发到位时间 | 过大可能低速抖动 |
| ready tolerance | 约 0.08 rad | 编码器最终到位容差 | 过大可能误判到位 |

具体值以对应 YAML 为准；上表用于理解数量级，不应替代配置审查。

---

## 20. 术语表

| 缩写 | 全称 | 本项目中的意思 |
|---|---|---|
| DOF | Degree of Freedom | 自由度，两臂各 7 |
| FK/IK | Forward/Inverse Kinematics | 正/逆运动学 |
| JTC | JointTrajectoryController | 左右关节轨迹控制器 |
| PSM | Planning Scene Monitor | MoveIt/Servo 的机器人状态与场景监视 |
| OMPL | Open Motion Planning Library | MoveGroup 采样规划插件 |
| TF | Transform | ROS 坐标变换树 |
| PDO | Process Data Object | EtherCAT 周期过程数据 |
| SDO | Service Data Object | EtherCAT 非周期配置数据 |
| WKC | Working Counter | EtherCAT 帧被从站正确处理的计数指标 |
| SM | Sync Manager | 从站过程数据/邮箱管理 |
| FMMU | Fieldbus Memory Management Unit | EtherCAT 逻辑地址映射 |
| CiA402 | CANopen device profile 402 | 驱动器控制状态机规范 |
| CSV | Cyclic Synchronous Velocity | 当前实物速度模式，模式 9 |
| PP | Profile Position | 历史位置模式实验 |
| STO | Safe Torque Off | 硬件安全力矩关闭 |
| QoS | Quality of Service | ROS 2 可靠性、历史、持久性策略 |
| DDS | Data Distribution Service | ROS 2 默认通信中间件体系 |
| FLU | Forward-Left-Up | ROS 常见坐标约定；项目 base_link 经过机械安装定义需具体核对 |

---

## 21. 学完本项目应能回答的问题

如果能独立回答下面的问题，说明已经具备继续开发的基础：

1. xacro 初始值、SRDF named state 和真机 ready 为什么不会互相覆盖？
2. MoveGroup、Servo 与 JTC 分别负责什么？
3. 为什么真机 `DualArmHardware::write()` 为空，电机仍能收到命令？
4. `/enable`、CiA402 Operation enabled 和物理急停有何区别？
5. 为什么位姿差分仍然存在，却称为“速度死区”？
6. 为什么线速度和角速度必须应用同一个 proper rotation？
7. 为什么 JTC action success 之后还要检查编码器最终误差？
8. 为什么 `dry_run=true` 不能当成完全脱离 EtherCAT？
9. 为什么 Servo 使用 KDL，而 MoveGroup 使用 PickIK？
10. 机械臂不动时，如何证明是 bridge 未发、Servo 拒绝、JTC 无输出还是 SOEM 门关闭？
11. watchdog 触发后为什么必须人工 re-arm？
12. 为什么复杂 STL collision mesh 会影响规划甚至导致崩溃？

本项目最重要的能力不是记住某条启动命令，而是能把现象放回完整数据流，用最小、最安全的实验找到故障所在层。
