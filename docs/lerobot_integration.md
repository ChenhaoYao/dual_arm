# dual_arm 与 LeRobot 数据接口使用说明

本文说明如何把双臂 ROS 2 系统的关节、相机和 VR 示教数据送入
`/home/dell/lerobot`，以及如何在不误触发真机的前提下逐级验证策略动作。

当前 dual_arm 侧已经实现 ZMQ v1 协议。LeRobot 侧仍需按照
[`lerobot_handoff_2026-07-16.md`](lerobot_handoff_2026-07-16.md) 完成适配后，
才能正式录制和训练。

## 1. 先理解三类数据

```text
真实编码器 /joint_states
  └─ observation：机械臂实际做到了哪里

左右 JTC controller_state.reference.positions
  └─ demonstration action：VR + Servo 实际交给控制器的目标

LeRobot policy action
  └─ inference action：训练后策略希望机械臂到达的关节位置
```

训练数据不能把 observation 和 action 混为一谈。如果把同一帧编码器位置同时
复制到两者，模型只能学到“保持当前状态”。本接口用 JTC reference 作为示教
action，用编码器作为 observation，两者都是弧度制、固定 14 轴顺序。

固定关节顺序为：

```text
laxis1_joint ... laxis7_joint,
raxis1_joint ... raxis7_joint
```

当前项目没有夹爪控制，因此 feature 只有 14 个手臂关节，不要在 LeRobot 侧
擅自增加不存在的 gripper feature。

### 1.1 两种 action，不是同一条数据

本文中的 `action` 有两个方向相反的含义：

| 名称 | 方向 | 用途 | 是否受 `allow_action_commands` 控制 |
|---|---|---|---|
| demonstration action（5557） | dual_arm → LeRobot | VR 示教的训练标签 | 否 |
| policy action（5558） | LeRobot → dual_arm | 训练后模型的控制命令 | 是 |

因此 `allow_action_commands: false` 是正常的采集配置：关节 observation、5557
示教 action 和三路图像仍然发布，只丢弃 5558 的策略控制输入。

### 1.2 两个 enable，不是同一个动作门

```text
LeRobot policy action
  ──> [/zmq_bridge_node/enable_actions] ──> 左右 JTC
  ──> [/soem_bridge_node/enable] ─────────> EtherCAT 电机
```

- `/zmq_bridge_node/enable_actions` 是新增的上游门，只允许或禁止训练后策略修改
  JTC 目标；数据采集阶段不调用。
- `/soem_bridge_node/enable` 是原有的下游真机门，决定 JTC 输出能否发给电机。
- 新门不会调用、替代或改变旧门。只采集和离线训练时可以永远不打开新门。

## 2. 数据流和端口

```text
ROS 2 /joint_states ────────────────> ZMQ PUB 5556 ─> Robot observation
左右 JTC reference ────────────────> ZMQ PUB 5557 ─> VR teleoperator action
三路 OpenCV camera ────────────────> ZMQ PUB 5555/5559/5560 ─> images
LeRobot Robot.send_action() ───────> ZMQ PULL 5558 ─> 左右 JTC trajectory
```

| 端口 | 方向（以 dual_arm 为中心） | 内容 |
|---|---|---|
| 5555 | 发布 | `head_camera` JPEG |
| 5556 | 发布 | 14 轴实际位置、速度、力矩 |
| 5557 | 发布 | 14 轴 JTC reference 示教 action |
| 5558 | 接收 | LeRobot 策略关节位置 action |
| 5559 | 发布 | `left_wrist_camera` JPEG |
| 5560 | 发布 | `right_wrist_camera` JPEG |

桥接节点是独立附加节点，不会自动启动 MoveIt、Servo、SOEM 或 RViz，也没有加入
已经验证通过的 `sim.launch.py` / `real.launch.py` 主链路。

## 3. 相关文件

- `dual_arm_bringup/scripts/zmq_bridge_node.py`：协议、检查、相机线程和 ROS 转发。
- `dual_arm_bringup/config/lerobot_bridge.yaml`：端口、话题、相机和安全阈值。
- `dual_arm_bringup/launch/lerobot_bridge.launch.py`：独立启动入口。
- `docs/lerobot_handoff_2026-07-16.md`：给 LeRobot 仓库 agent 的实现清单。

## 4. 构建与相机配置

```bash
cd /home/dell/dual_arm
colcon build --symlink-install --packages-select dual_arm_bringup
source install/setup.bash
```

编辑 `dual_arm_bringup/config/lerobot_bridge.yaml`，确认三台相机。`/dev/videoX`
可能在重启或重新插拔后改变，正式采集应优先填写稳定路径：

```bash
ls -l /dev/v4l/by-id/
```

如果暂时只验证关节接口，把对应 `enabled` 改为 `false`。某一相机打不开时，
桥会记录错误，但不会让关节数据停止；LeRobot 侧也必须允许按配置选用相机，
否则其 `Robot.connect()` 仍可能因为缺一台相机而失败。

## 5. 从低风险到高风险逐级测试

### 5.1 第一级：只启动桥，不接控制栈

临时关闭相机并确认节点能启动：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 run dual_arm_bringup zmq_bridge_node.py --ros-args \
  -p cameras.head_camera.enabled:=false \
  -p cameras.left_wrist_camera.enabled:=false \
  -p cameras.right_wrist_camera.enabled:=false
```

预期日志必须包含：

```text
Data-output mode: joint observations, demonstration actions, and cameras remain enabled;
incoming policy actions cannot command ROS controllers.
```

此时没有 `/joint_states` 和 controller state，等待提示是正常现象。

### 5.2 第二级：mock 仿真 + 关节数据

终端 1：

```bash
cd /home/dell/dual_arm
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=servo
```

终端 2：

```bash
cd /home/dell/dual_arm
source install/setup.bash
ros2 launch dual_arm_bringup lerobot_bridge.launch.py
```

这一步仍保持 `allow_action_commands: false`。先检查 LeRobot 能稳定接收 14 轴，
缺失或陈旧数据必须报错，不能用全零悄悄补齐。

### 5.3 第三级：单独验证相机

逐台只启用一个 camera，核对：

- 图像名称与端口一致；
- 实际数组形状恒为 `480 x 640 x 3`；
- 红蓝颜色没有交换；
- 采集循环能达到目标 30 Hz；
- 拔掉相机后 LeRobot 报陈旧/超时，不复用无限期旧帧。

桥会把不同硬件原始分辨率统一 resize 到 YAML 中的固定尺寸。当前发送端为了兼容
LeRobot 实验性 `ZMQCamera` 的行为，在 JPEG 编码前做一次 RGB 预交换；LeRobot
agent 必须按照交接文档做端到端色卡测试，不要凭函数名猜颜色顺序。

### 5.4 第四级：VR 仿真示教录制

完成 LeRobot 侧适配后：

1. 启动 mock Servo、VR 和 ROS TCP endpoint；
2. 启动 ZMQ bridge，保持 `allow_action_commands: false`；
3. 用 `dual_arm_zmq` Robot 接收 observation/camera；
4. 用 `vr_zmq` Teleoperator 接收 5557 的 JTC reference；
5. 先录 1 个短 episode，检查 action 与 observation 的 14 个 feature、时间连续性、
   相机颜色和动作幅度；
6. 确认无误后再增加 episode 数量。

LeRobot 的 `record_loop()` 会把 teleoperator action 再传给
`robot.send_action()`。采集阶段桥的“策略 action 输入门”保持关闭，因此这个回传包会被忽略，
真正的机械臂运动仍由原来的 VR→Servo 路径产生，不会形成第二条控制链。

### 5.5 第五级：mock 策略推理

只有开始策略推理时，才把 YAML 中：

```yaml
allow_action_commands: true
```

然后重启 bridge。它仍然以“策略 action 输入门关闭”的状态启动。确认 mock 左右 JTC 都 active、
`/joint_states` 新鲜后，显式打开“策略 action 输入门”：

```bash
ros2 service call /zmq_bridge_node/enable_actions \
  std_srvs/srv/SetBool "{data: true}"
```

策略断流超过 `0.30 s` 后该门会锁闭，必须人工检查并重新 enable。以下任意条件也会
拒绝单帧动作：

- 协议版本或消息类型错误；
- session/sequence 倒退或运行中换客户端；
- 不是完整且恰好 14 个关节；
- NaN、Inf 或超出 `±3.14 rad`；
- 消息时间戳太旧或明显来自未来；
- 当前编码器数据超时；
- 任一目标相对当前实际位置超过 `0.10 rad`。

### 5.6 第六级：真机策略推理

真机需要三层条件同时满足：

```text
YAML allow_action_commands=true
  + /zmq_bridge_node/enable_actions=true
  + /soem_bridge_node/enable=true
```

策略 action 输入门不会替代 SOEM 门，也不会自动打开 SOEM 门。第一次真机推理应：

1. 保留物理急停，低速、空载、单人操作；
2. 先在 mock 连续运行完整任务；
3. 真机先检查 ready 位、14 轴方向和摄像头；
4. 先只打开 bridge 门但保持 SOEM 门关闭，观察动作拒绝/轨迹目标；
5. 最后才短时打开 SOEM 门；
6. 任一异常立即关闭两个软件门并按物理急停。

关闭 bridge 门：

```bash
ros2 service call /zmq_bridge_node/enable_actions \
  std_srvs/srv/SetBool "{data: false}"
```

## 6. ZMQ v1 协议摘要

所有消息是 UTF-8 JSON。详细字段也是 LeRobot 侧应实现的测试契约。

### 6.1 Joint observation（5556）

```json
{
  "protocol_version": 1,
  "message_type": "joint_state",
  "session_id": "bridge-startup-uuid",
  "sequence": 12,
  "timestamp": 1784210000.123,
  "ros_timestamp_ns": 1784210000123000000,
  "names": ["laxis1_joint", "...", "raxis7_joint"],
  "positions": [0.0, 0.0],
  "velocities": [],
  "efforts": []
}
```

示例数组省略了中间元素，真实 `names` 和 `positions` 始终是 14 项。速度或力矩在
ROS 消息中不完整时发空数组，不能错位填充。

### 6.2 Demonstration action（5557）

```json
{
  "protocol_version": 1,
  "message_type": "demonstration_action",
  "source": "joint_trajectory_controller_reference",
  "session_id": "bridge-startup-uuid",
  "sequence": 9,
  "timestamp": 1784210000.123,
  "ros_timestamp_ns": 1784210000123000000,
  "joint_positions": {"laxis1_joint": 0.1, "raxis7_joint": -0.2},
  "vr_enabled": {"left": true, "right": true}
}
```

真实 `joint_positions` 必须恰好包含 14 个键。`vr_enabled` 是诊断元数据，目前不加入
LeRobot action feature。

### 6.3 Policy action（5558）

```json
{
  "protocol_version": 1,
  "message_type": "joint_position_action",
  "session_id": "lerobot-client-uuid",
  "sequence": 101,
  "timestamp": 1784210000.123,
  "joint_positions": {"laxis1_joint": 0.1, "raxis7_joint": -0.2}
}
```

同一个 bridge enable 周期只接受一个 `session_id`，且 sequence 必须严格递增。换
LeRobot 进程时先关闭再重新打开 bridge 的策略 action 输入门。

### 6.4 Camera frame（5555/5559/5560）

```json
{
  "protocol_version": 1,
  "message_type": "camera_frame",
  "session_id": "bridge-startup-uuid",
  "sequence": 21,
  "timestamp": 1784210000.123,
  "color_space": "rgb",
  "timestamps": {"head_camera": 1784210000.123},
  "images": {"head_camera": "base64-jpeg"}
}
```

## 7. 当前边界

- ZMQ 是可信局域网内的实验接口，没有认证和加密；不要暴露到公共网络。
- 三路图像各自采集，不保证硬件级同步；训练样本使用“各流最新帧”。
- bridge 的 action watchdog 只锁闭本桥，不直接改变 SOEM 生命周期。
- 只支持 14 轴位置 action，不支持夹爪、底盘、力矩或末端笛卡尔 action。
- v1 暂时允许无版本旧 action；LeRobot 升级后把 `accept_legacy_actions` 改为
  `false`。
- 正式训练前必须检查 LeRobot 数据集统计，确认关节单位为 rad、没有全零填充、
  没有 NaN/Inf、图像颜色正确。
