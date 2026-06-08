# SOEM PP Waypoint ROS Bridge Plan

本计划建议新增一个独立 ROS2 桥接包，把 MoveIt/RViz 侧的仿真轨迹降采样为 PP waypoint，并由 SOEM 以高频 EtherCAT 周期驱动真实机械臂同步运动。

## 当前判断

- `dual_arm` 当前链路是 `MoveIt -> joint_trajectory_controller -> ros2_control -> RViz/joint_states`。
- `sim.launch.py` 使用 `mock_components/GenericSystem`，因此在仿真模式下 `dual_arm_control/DualArmHardware` 不会被加载。
- `SOEM` 不是 ROS 包，而是独立 CMake EtherCAT 库；其中 `samples/ec_sample/ec_sample.c` 已包含 YH052/CiA402 PP 模式、PDO 映射、1ms RT 线程、0x003F PP 触发等可复用经验。
- 当前电机不能用 CSP，只能用 PP；PP 可在未到旧目标时接收新目标，所以适合“低频 waypoint + 高频 PDO/PP 握手”的过渡方案。

## 推荐架构

```text
RViz 末端位姿目标
  -> MoveIt 规划/执行
  -> left/right joint_trajectory_controller
  -> waypoint_sampler 低频采样 desired joint positions
  -> /dual_arm/pp_waypoints 或桥接节点内部队列
  -> dual_arm_soem_bridge 1kHz EtherCAT RT loop
  -> YH052 电机 PP 目标位置 0x607A + 0x6040=0x003F 触发
```

## 关键决策

- **不要直接改 `SOEM` 根目录为 ROS 包**：保留它作为第三方 EtherCAT 库，避免污染上游结构；新增 `dual_arm_soem_bridge` 作为 ROS2 包来封装它。
- **第一阶段不要把 SOEM 嵌入 `dual_arm_control`**：仿真模式下当前加载的是 `mock_components/GenericSystem`，在 `dual_arm_control` 里发话题不会生效；而且硬件插件承担 ROS publisher/复杂网络逻辑不利于调试。
- **第一阶段不要直接绕过 ROS 接 MoveIt 内部指令**：MoveIt 的执行出口已经是 `FollowJointTrajectory`/controller，先从控制器期望轨迹侧镜像更安全、可观测、改动小。
- **真实臂反馈暂不覆盖 RViz `/joint_states`**：先让 RViz 继续显示仿真执行结果，SOEM 另发真实状态话题，避免仿真状态和真实状态互相抢 `/joint_states`。

## 实施阶段

### 1. 新增 `dual_arm_soem_bridge` 包

- 使用 `ament_cmake` 创建 ROS2 C++ 包。
- 依赖 `rclcpp`、`trajectory_msgs`、`sensor_msgs`、`std_srvs`、`control_msgs`。
- 链接 SOEM：优先把 `/home/dell/dual_arm/SOEM` 作为第三方库通过 CMake 引入或安装后 `find_package`。
- 将 `ec_sample.c` 中的 EtherCAT 初始化、PDO 映射、DC Sync0、CiA402 PP 状态机抽象成 `SoemPpMaster`。

### 2. 定义 ROS 接口

- 输入：`trajectory_msgs/msg/JointTrajectory`，包含 7 或 14 个关节 waypoint，单位保持 ROS 标准 `rad`。
- 输出：`sensor_msgs/msg/JointState` 或专用状态 topic，发布真实电机位置、速度、错误码、CiA402 状态。
- 服务：`arm/enable`、`halt/stop`、`clear_fault`，真实运动必须显式使能。
- 参数：网卡名、关节名到 slave 的映射、编码器位数、减速比、零点偏置、方向、软限位、waypoint 频率、RT 周期。

### 3. Waypoint 生成策略

- 初期从 `left_arm_controller/controller_state` 和 `right_arm_controller/controller_state` 订阅 `desired.positions`。
- 以低频参数采样，例如 5-20 Hz，生成统一关节顺序的 waypoint。
- 后续如需严格“规划完成后一次性下发整条轨迹”，再增加 `FollowJointTrajectory` fanout/action wrapper：同时转发给仿真 JTC 和 SOEM bridge。

### 4. SOEM 内部 PP 队列与高频执行

- EtherCAT RT 线程保持 1kHz：`receive -> read status -> update PP state -> write PDO -> send`。
- 新 waypoint 到来后转为每轴 counts：`rad -> deg/rev -> encoder counts`，应用方向、零点和软限位。
- 对每个 waypoint：先在 `CW=0x000F` 下预载目标若干周期，再同步触发 `CW=0x003F`。
- 不等待每个中间 waypoint 到达；只要求 set-point acknowledge 完整握手，若新 waypoint 到来则更新 pending target。
- 最终 waypoint 才检查 target reached、位置误差和超时。
- 多轴同步：同一帧 waypoint 所有轴预载完成后统一触发；任一轴离开 Operation enabled 时整组暂停/停机。

### 5. Launch 与运行模式

- 新增独立 launch，例如 `soem_bridge.launch.py` 或 `sim_with_real_mirror.launch.py`。
- 默认不启动真实硬件，必须通过参数显式开启：`enable_soem:=true ifname:=enp...`。
- 保持原 `sim.launch.py` 不变，确保 RViz/MoveIt 仿真仍可单独运行。

### 6. 分阶段验证

1. **Dry-run**：不打开 EtherCAT，只打印收到的 waypoint、rad->counts 转换和限位检查。
2. **单轴空载**：复用当前 `ec_sample` 的 PP 逻辑，只让 slave1 跟随低频 waypoint。
3. **单臂 7 轴**：验证关节顺序、方向、零点、同步触发和急停。
4. **双臂 14 轴**：验证左右控制器状态合流、同步和超时策略。
5. **MoveIt/RViz 联动**：在 RViz 发末端位姿目标，确认仿真机器人和真实机械臂同时运动。

## 风险与注意事项

- PP 不是真正周期同步位置模式，低频 waypoint 会受驱动内部 profile velocity/acceleration 影响，真实轨迹不会完全等同 RViz 轨迹。
- 必须先标定每个关节的零点、方向、机械软限位，否则 `rad->counts` 可能导致危险运动。
- SOEM 进程需要网卡权限，实际运行可能需要 `sudo` 或 capabilities。
- 真实状态不要直接混入 `/joint_states`，除非后续明确要以真实机械臂为唯一状态源。

## 后续可升级路线

- 如果后续电机支持 CSP，再把 `dual_arm_soem_bridge` 的 PP 后端替换/扩展为 CSP 后端。
- 如果希望 MoveIt 直接控制真实硬件，再将成熟的 SOEM 控制逻辑迁移进 `dual_arm_control/DualArmHardware`，由 ros2_control 正式接管真实机器人。
