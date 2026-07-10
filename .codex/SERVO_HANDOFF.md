# MoveIt Servo 交接与落地路线

## 目标与边界

目标环境固定为 **Ubuntu 24.04 + ROS 2 Jazzy**。最终控制路径按三个阶段推进：

1. 在 mock hardware 中用 RViz 验证模型、控制器和 Servo 实时链路。
2. 保持同一套 Servo 后端，接入 Unity/PICO VR 输入。
3. 不改变上层命令接口，仅把 mock hardware 替换为真实硬件与 SOEM bridge。

MoveIt Servo 是实时速度控制后端，不等同于 RViz MotionPlanning 面板。MotionPlanning
交互 marker 发送的是 MoveGroup 规划目标，不能直接证明 Servo 链路正常。

## 最新 PR 审查结论

审查分支：`pr/float-dr-eam/4`，相对基线：`origin/master`，目标分支来自
`humble/22.04`。

结论：**不能整体合并，也不建议直接 merge 后修复。** 应从 Jazzy 主分支继续开发，
只手工吸收少量与版本无关的意图。

### 必须拒绝的改动

| PR 改动 | Jazzy 结果 | 处理 |
|---|---|---|
| `servo_node` 改为 `servo_node_main` | `/opt/ros/jazzy/lib/moveit_servo` 中不存在该可执行文件，launch 立即失败 | 保留 `servo_node` |
| `HardwareComponentInterfaceParams` 回退为 `HardwareInfo` | Jazzy 仍能编译，但调用已 deprecated，并丢失 executor 参数接口 | 保留主分支实现 |
| OMPL 改回 `planning_plugin` 和旧 request adapter 名称 | 与当前 Jazzy MoveIt 配置格式冲突 | 保留 `planning_plugins`、request/response adapters |
| `joint_limit_margins` 改为 `joint_limit_margin` | Jazzy Servo 只声明数组参数 `joint_limit_margins` | 保留数组参数 |
| 新增 `low_latency_mode`、`use_gazebo` | 当前 Jazzy Servo 未声明这些参数 | 删除 |
| 新增 `num_outgoing_halt_msgs_to_publish` | 当前 Jazzy Servo 未声明该参数 | 删除 |
| controller `gains` 增加 `position:` 层 | 当前 mock 控制器使用 position command interface，且此改动无 Servo 必要性 | 不合并 |
| 大量替换 OMPL planner 配置 | 与 Servo 无关，且扩大 MoveGroup 回归面 | 不合并 |

实际验证结果：相关四个包可完成构建，但 `sim.launch.py mode:=servo` 启动失败：

```text
executable 'servo_node_main' not found on the libexec directory
'/opt/ros/jazzy/lib/moveit_servo'
```

### 可选择性吸收的内容

- README 中对 Servo 数据流、话题和节点职责的说明，但必须把可执行文件改回
  `servo_node`，并移除“RViz 中的 MotionPlanning 错误属于正常主流程”等误导描述。
- `incoming_command_timeout: 0.1` 是 Jazzy 有效参数，但也是默认值；可以显式保留以表达
  安全意图。
- PR 将奇异点阈值从 `200/400` 降到 `17/30` 后，当前机器人模型的 Servo
  输出被全部缩放为零。已通过轨迹采样确认，现阶段保留 `200/400`；真机前需基于
  实测 Jacobian 条件数重新标定，不能直接套用官方 Panda 阈值。
- `publish_period: 0.034` 不是 Jazzy 官方默认值（官方示例为 `0.01`）。VR 输入为 50 Hz、
  controller update rate 为 100 Hz 时，优先保留 `0.01`，除非测量证明确实需要降到约 30 Hz。
- 平滑滤波可以启用，但要显式保留 `use_smoothing: true` 和插件名，避免依赖版本默认值。

`SOEM/CMakeLists.txt` 的最低 CMake 版本调整与 Servo 无关，应放到独立 PR 评估。

## 推荐架构

```text
                         +--------------------------+
RViz Servo marker ------>|                          |
                         | input adapter / arbiter  |-- TwistStamped --> MoveIt Servo
Unity/PICO VR bridge --->| (同一时刻只允许一个源)     |                    left + right
                         +--------------------------+                         |
                                                                              v
                                                             JointTrajectoryController
                                                                              |
                                                      mock GenericSystem / real hardware
```

顶层启动入口继续保持：

```text
sim.launch.py / real.launch.py
  -> control_base.launch.py
  -> mode:=moveit: move_group.launch.py
  -> mode:=servo:  dual_arm_servo/servo.launch.py
  -> RViz
  -> optional VR bridge / TCP endpoint
```

MoveGroup 与 Servo 继续保持启动期互斥。RViz 可以在两种模式都启动，但应使用不同配置。

## 分阶段实施与验收

### Gate 0：恢复 Jazzy 基线

从 `origin/master` 开新分支，或逐项丢弃上述 Humble 回退。至少确认：

```bash
colcon build --packages-select \
  dual_arm_control dual_arm_moveit_config dual_arm_servo dual_arm_bringup

source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=servo
```

验收：两个 Servo 节点、两个 JTC、joint state broadcaster 和 RViz 均存活；无未声明参数
或找不到可执行文件错误。

### Gate 1：RViz / mock 验证

这个阶段拆成两层，不要混为一次测试：

1. `mode:=moveit`：用 MotionPlanning marker 做 plan + execute，验证 URDF/SRDF、IK、碰撞模型、
   controller action 和 mock hardware。
2. `mode:=servo`：用独立 Servo interactive marker adapter 实时控制，验证
   marker feedback -> Twist -> Servo -> JTC -> `/joint_states`。

当前已新增：

```text
dual_arm_servo/dual_arm_servo/rviz_servo_marker.py
dual_arm_moveit_config/config/servo.rviz
```

`servo_smoke_test.py` 仍待实现。

marker adapter 已实现：左右臂独立 marker、松开后执行目标、末端 pose 误差到 Twist 的比例控制、
线/角速度限幅、到达容差、10 秒目标超时和零速度停止，并通过 `switch_command_type(TWIST)` 与
`pause_servo(false)` 配置 Jazzy Servo。后续仍需补充显式 clutch/enable。不要复用
MotionPlanning display 内部 marker。

验收：

- 无 VR、无网络时可独立控制任一手臂。
- 目标到达、10 秒目标超时、节点退出或 joint state 失联后，命令归零。
- `/servo_left/status`、`/servo_right/status` 无持续奇异/限位错误。
- 输出话题为 `/left_arm_controller/joint_trajectory` 和
  `/right_arm_controller/joint_trajectory`，关节顺序正确。
- 两个输入源不能同时向同一 Servo topic 发布。

### Gate 2：VR / mock 验证

现有 `vr_pose_to_servo_node.py` 已经把 `/vr/*/pose` 转换到同一组 Servo Twist topic，
因此 VR 阶段不应复制 Servo 逻辑，只替换输入适配器。

```bash
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_vr_teleop:=true \
  enable_ros_tcp_endpoint:=true \
  ros_tcp_port:=10000
```

进入真机前必须修改/验证：

- `require_enable_signal` 从当前 `false` 改为 `true`，扳机或握持键作为 dead-man switch。
- enable 释放、VR 断连、pose 超时、bridge 退出时均发布零 Twist，并 pause Servo。
- 校准 Unity 到 `base_link` 的轴向、符号、初始参考 pose 和左右手映射。
- 对位置增量和旋转增量分别限速；验证时间戳跳变不会产生速度尖峰。
- 引入输入仲裁：`rviz`、`vr`、`keyboard` 同时只能激活一个来源。

### Gate 3：真实机械臂

保持 Servo topic、frame 和速度限制不变，只将底层从 mock 切换到真实接口：

```bash
# Terminal 1
source install/setup.bash
ros2 launch dual_arm_bringup real.launch.py mode:=servo

# Terminal 2
source install/setup.bash
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py

# 最后一步，确认反馈、方向、急停和限位后再使能
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
```

真机首轮只允许单臂、低速、小行程。使能前检查：

- 14 个编码器方向、零点和 `/joint_states` 顺序正确。
- JTC reference 与真实 encoder feedback 同方向且误差合理。
- Servo、VR bridge、SOEM bridge 任一停止时电机命令归零。
- 软件 stop、Servo pause、SOEM disable 和物理急停逐项实测。
- 关节限位、奇异点限制和工作空间限制生效。
- 碰撞 geometry 简化并验证后再把 `check_collisions` 改为 `true`。

## 当前已知风险

- 左右 Servo 必须各自使用 primary PlanningSceneMonitor，并将
  `/get_planning_scene` 重映射为各自的私有服务。右臂作为 secondary 订阅左臂 scene 时，
  Jazzy Servo 会持续等待 robot state update。
- Servo 必须使用独立 KDL 运动学配置。MoveGroup 的 PickIK `position_threshold: 0.001`
  大于 Servo 单周期位移，会返回当前姿态并产生全零关节增量。
- JTC 必须保留 `allow_nonzero_velocity_at_trajectory_end: true`；默认值会拒绝
  Servo 的滚动 `JointTrajectory`。
- `keyboard_teleop.py` 的空格“急停”日志与实现不一致；无有效按键时仍可能发布零 Twist。
  它只能作为调试工具，不能作为安全控制器。
- 当前 VR 配置 `require_enable_signal: false`，不满足真机 dead-man switch 要求。
- laxis/raxis 3-7 仍使用较大的 STL collision mesh，碰撞检测当前关闭。
- 当前没有自动 smoke test，也没有输入源仲裁节点。
- 真机链路通过 controller state topic 到 SOEM bridge，必须单独验证 feedback QoS、控制器接口和
  失联归零，不能只根据 RViz 运动判断安全。

## 推荐下一步顺序

1. 将本轮端到端脚本整理为仓库内 `servo_smoke_test.py`，固化左右臂小位移验收。
2. 加入统一输入仲裁和 dead-man/timeout/pause 行为。
3. 在 mock 下接入现有 VR bridge，完成断连和时间戳异常测试。
4. 简化碰撞模型并恢复碰撞检测。
5. 基于真实关节状态采样 Jacobian 条件数，重新标定奇异点阈值。
6. 最后进行单臂低速真机验证，再开放双臂。

## 不要回退的决策

- 不把 Servo 节点重新塞回 `moveit.launch.py`。
- 不让 MoveGroup 与 Servo 同时作为控制后端运行。
- 不把 MotionPlanning marker 当作 Servo 输入。
- 不让 RViz、VR、键盘同时直接发布到同一个 Servo command topic。
- 不在 collision mesh 未处理、dead-man 未实现、失联归零未验证前接真机。
