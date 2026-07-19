# LeRobot 接口交接（2026-07-16）

## 1. 给下一位 agent 的任务

请在 `/home/dell/lerobot` 中完成 `dual_arm_zmq` Robot、`vr_zmq`
Teleoperator 和 `ZMQCamera` 的 v1 适配，使它们能够：

1. 从 `/home/dell/dual_arm` 的 ZMQ bridge 可靠录制 14 轴 + 三路图像数据；
2. 明确区分 observation（编码器实际值）与 action（JTC reference）；
3. 对缺失、陈旧、错序和非有限数据直接报错，不用全零静默补齐；
4. 完成一个短 mock episode 的录制、读取和可视化验证；
5. 在数据验证通过后给出 ACT 基线训练命令；
6. 策略部署先仅在 mock 验证，未经用户明确确认不要打开任何真机命令门。

不要修改 dual_arm 的 VR、Servo、JTC 或 SOEM 主控制链。共享协议以：

- `/home/dell/dual_arm/dual_arm_bringup/scripts/zmq_bridge_node.py`
- `/home/dell/dual_arm/docs/lerobot_integration.md`

为准。

术语必须保持一致，避免把两个方向相反的 action 混在一起：

- `demonstration_action`（5557）是 dual_arm 向 LeRobot 发布的训练标签；
- `joint_position_action`（5558）是训练后的策略反向控制 JTC 的命令；
- `allow_action_commands` 和 `/zmq_bridge_node/enable_actions` 只控制 5558；
- `/soem_bridge_node/enable` 是另一道下游 EtherCAT 电机门，新服务不得代替或
  自动调用它。

## 2. 当前仓库状态

### dual_arm

工作区：`/home/dell/dual_arm`

本轮新增/修改：

- `dual_arm_bringup/scripts/zmq_bridge_node.py`
- `dual_arm_bringup/config/lerobot_bridge.yaml`
- `dual_arm_bringup/launch/lerobot_bridge.launch.py`
- `dual_arm_bringup/CMakeLists.txt`
- `dual_arm_bringup/package.xml`
- `docs/lerobot_integration.md`
- 本交接文档

注意：用户先前要求的 `README.md` 与 `docs/project_manual.md` 改动也仍在同一
dual_arm 工作区中，不要把它们误删或回滚。

dual_arm 侧已经完成：

- ZMQ v1 的 joint observation、demonstration action、camera 和 policy action；
- 固定 14 轴顺序、有限值和完整性检查；
- policy action 的时间戳、session、sequence、绝对限位和实际相对增量检查；
- `allow_action_commands=false` 默认只锁住 5558 策略输入，不影响采集输出；
- `~/enable_actions` 是 5558 → JTC 的运行时门，不是 SOEM 电机门；
- 0.30 s policy action watchdog，超时后锁闭并要求人工重新 enable；
- 相机可独立关闭、固定输出尺寸、相机/ZMQ socket 线程归属修复；
- 旧 LeRobot action 迁移兼容开关。

### LeRobot

工作区：`/home/dell/lerobot`

检查时状态：

- branch：`main`
- commit：`269c06c1`
- worktree：clean

已有半成品：

- `src/lerobot/robots/dual_arm_zmq/`
- `src/lerobot/teleoperators/vr_zmq/`
- `src/lerobot/cameras/zmq/`
- `robots/utils.py` 和 `teleoperators/utils.py` 已有 factory 分支。

不要假设这些文件已经可用；下面列出的缺陷需要逐项修复。

## 3. 必须优先修复的 LeRobot 缺陷

### P0-1：CLI 解析前没有注册自定义 config

`lerobot_record.py` 当前导入了 `ZMQCameraConfig`，但其 robots/teleoperators 导入
列表没有 `dual_arm_zmq` 和 `vr_zmq`。draccus 在解析
`--robot.type=dual_arm_zmq` / `--teleop.type=vr_zmq` 前可能根本不知道这两个
subclass。

至少检查并修复：

- `src/lerobot/scripts/lerobot_record.py`
- `src/lerobot/scripts/lerobot_teleoperate.py`
- 策略部署实际使用的 `lerobot_rollout.py` 或当前推荐入口
- `src/lerobot/robots/__init__.py`
- `src/lerobot/teleoperators/__init__.py`

不要只让 factory 能创建对象；config subclass 必须在 CLI parse 之前完成注册。

### P0-2：不能把通信失败伪装成零位

当前 `DualArmZMQRobot.get_observation()` 在没有 joint state 时返回 14 个零，
`VRZMQTeleop.get_action()` 在没有 action 时也返回 14 个零。这会把坏帧写入数据集，
还可能在允许执行时制造突然回零目标。

改为：

- `connect()` 等待第一帧完整、合法数据；超时则连接失败；
- 缓存 monotonic receipt time；
- `get_observation()` / `get_action()` 检查最大 age；
- 缺轴、NaN/Inf、陈旧、session/sequence 异常时抛出明确异常，让 episode 失败，
  不得填零。

### P0-3：实现 ZMQ v1 校验

joint subscriber 只接受：

- `protocol_version == 1`
- `message_type == "joint_state"`
- 同一 `session_id` 内 sequence 递增
- `names` 和 `positions` 对齐，且固定 14 轴全部存在
- 所有位置有限

VR teleoperator 只接受：

- `protocol_version == 1`
- `message_type == "demonstration_action"`
- 完整 14 轴 `joint_positions`
- 同一 session 内 sequence 递增
- receipt age/source timestamp 均在配置阈值内

Robot `send_action()` 必须发送：

```json
{
  "protocol_version": 1,
  "message_type": "joint_position_action",
  "session_id": "one UUID per LeRobot Robot.connect()",
  "sequence": 0,
  "timestamp": 1784210000.123,
  "joint_positions": {"14 exact joint keys": 0.0}
}
```

sequence 每次发送递增，重连生成新 UUID。dual_arm bridge 在策略 action 输入门的
一次 enable 周期内拒绝 session 改变；换客户端时需要先关闭并重新打开该输入门。

### P0-4：录制期间不得形成第二条控制路径

LeRobot `record_loop()` 的顺序是：

```text
teleop.get_action()
  -> robot.send_action()
  -> action 写入 dataset
```

而本项目 VR 已经通过 ROS/Servo 控制机械臂。采集时必须保持 dual_arm YAML：

```yaml
allow_action_commands: false
```

因此 `send_action()` 虽会发 ZMQ 包，但 bridge 会忽略它，运动仍只有原 VR 路径。
不要为了“去掉 warning”自动调用 `/zmq_bridge_node/enable_actions`；它不是采集
开关，也不是 `/soem_bridge_node/enable` 的别名。

## 4. 次优先修复项

### P1-1：相机应按配置可选

当前 `DualArmZMQRobot` 构造函数硬编码三台相机，任一不存在就会让整个 connect
失败。改成标准 `cameras: dict[str, ZMQCameraConfig]` 或等价的可选配置；
`observation_features` 必须只声明实际配置的相机。

连接过程中若第二台相机失败，需要关闭已经连接的第一台、joint socket 和线程，
保证部分失败不泄露资源。

### P1-2：修复 ZMQCamera 的一致性与线程错误

当前实现需要重点检查：

- `color_mode` 被保存但没有真正应用；
- JSON/base64/JPEG 缺少严格校验；
- message timestamp 没有进入 stale 检查；
- read thread 连续失败后直接 `raise`，异常只死在线程内，主调用方未必知道；
- `async_read()` 清 event 的方式可能让多个消费者产生假超时；
- docstring 与 `read_latest()` 实际返回值不一致。

dual_arm 发送端沿用了当前 LeRobot `ImageServer` 的兼容方式：OpenCV BGR 帧先转
RGB 数组，再用 `cv2.imencode` 编码，使现有 `cv2.imdecode` 得到的数组数值顺序
符合 RGB dataset。不要仅凭 BGR/RGB 名称修改；先用红、绿、蓝色卡做端到端测试，
然后让协议测试固定预期。

### P1-3：依赖和导入

`pyproject.toml` 已有 `pyzmq-dep`，但没有 dual_arm 专用 extra。建议增加：

```toml
dual-arm-zmq = ["lerobot[pyzmq-dep]"]
```

并确认安装方式和文档命令能让 `pyzmq`、OpenCV 和视频编码依赖同时可用。遵循
LeRobot 仓库的可选依赖/lazy import 规范，不要让没有安装 pyzmq 的普通用户在
导入整个 `lerobot` 时失败。

## 5. 推荐实现结构

建议为 joint/action 消息共享一个小型协议模块，例如：

```text
src/lerobot/transport/dual_arm_zmq_protocol.py
```

其中集中定义：

- `PROTOCOL_VERSION = 1`
- 14 轴常量
- JSON parse/validate
- session + sequence tracker
- source timestamp 和 monotonic receipt timestamp

Robot 和 Teleoperator 不要各复制一套稍有不同的校验。相机可保留自己的 payload
校验，但同样检查 version/type/session/sequence。

配置建议至少增加：

- `joint_state_timeout_ms`
- `demo_action_timeout_ms`
- `strict_protocol: bool = True`
- 可选 camera dict
- action port 和三类 observation/action port

## 6. 协议字段

完整 schema 和示例见：

```text
/home/dell/dual_arm/docs/lerobot_integration.md
```

关键语义：

| 数据 | 端口 | 学习含义 | 单位 |
|---|---:|---|---|
| `/joint_states.position` | 5556 | observation | rad |
| JTC `reference.positions` | 5557 | demonstration action | rad |
| policy joint target | 5558 | inference action | rad |
| 三路图像 | 5555/5559/5560 | visual observation | RGB uint8 |

不得把 controller `output.velocities` 当作当前数据集 action；当前 action feature 和
bridge policy input 都定义为绝对关节位置。

## 7. 测试顺序和验收条件

### 7.1 纯协议单元测试

为以下情况写 pytest：

- 正常 v1 joint/action/camera message；
- 缺一个关节、多一个关节、顺序不同但 name 完整；
- names/positions 长度不同；
- NaN/Inf；
- version/type 错误；
- sequence 重复或倒退；
- session 改变；
- receipt/source 超时；
- JPEG/base64 损坏；
- RGB 色块解码结果。

测试不应依赖 ROS、相机或真机。

### 7.2 dual_arm observation-only 联调

dual_arm：

```bash
cd /home/dell/dual_arm
source install/setup.bash
ros2 launch dual_arm_bringup lerobot_bridge.launch.py
```

LeRobot 只连接并连续读取 observation，不执行策略。验收：

- 14 轴全部存在且单位/方向正确；
- 三路图像 shape、名称、颜色正确；
- 拔掉一个数据源后在阈值内明确失败；
- disconnect 后所有线程和 socket 都退出。

### 7.3 mock 短 episode

先录 1 个 5–10 秒 episode，不上传、不训练。使用当前 CLI 的实际 `--help` 核对
参数后，命令形态应类似：

```bash
cd /home/dell/lerobot
uv run lerobot-record \
  --robot.type=dual_arm_zmq \
  --robot.id=dual_arm \
  --robot.zmq_host=127.0.0.1 \
  --teleop.type=vr_zmq \
  --teleop.id=dual_arm_vr \
  --teleop.zmq_host=127.0.0.1 \
  --dataset.repo_id=<user>/dual_arm_smoke \
  --dataset.single_task="双臂完成测试动作" \
  --dataset.num_episodes=1 \
  --dataset.fps=30 \
  --display_data=true
```

不要照抄到真机；先让 agent 根据最终 dataclass 字段和 CLI help 修正命令。

录完检查：

- feature 恰为 14 observation `.pos`、14 action `.pos` 和已配置相机；
- action 不是全零，也不是与 observation 每帧完全相同；
- 所有关节统计是合理的 rad 数值；
- 时间轴无长空洞；
- episode 视频颜色和左右腕命名正确。

### 7.4 ACT 基线

只有短 episode 和一批正式数据可视化通过后再训练。第一版建议 ACT，而不是先上
更大的 VLA。最终命令按当前 LeRobot CLI 核对，形态如下：

```bash
uv run lerobot-train \
  --dataset.repo_id=<user>/<dataset> \
  --policy.type=act \
  --policy.device=cuda \
  --output_dir=outputs/train/act_dual_arm \
  --job_name=act_dual_arm \
  --batch_size=8
```

先用小数据确认 pipeline 能跑通，再增加 episode；不要用更多坏数据掩盖接口错误。

### 7.5 策略 rollout

必须先 mock。dual_arm bridge 的策略能力和运行时门默认关闭。LeRobot agent 不得
自动调用 enable service。需要用户确认后，才按
`/home/dell/dual_arm/docs/lerobot_integration.md` 的三层门流程做真机测试。

## 8. 已完成验证与尚未验证

dual_arm 侧已经做过：

- Python 语法编译；
- `colcon build --symlink-install --packages-select dual_arm_bringup`；
- 三路相机关闭的 4 秒 observation-only 节点启动；
- 默认相机配置启动：当前主机测试环境没有 `/dev/video0/2/4`，三路均明确报
  cannot open，但 joint/action bridge 保持运行；
- `allow_action_commands=false` 日志确认；
- 未启动 MoveIt、Servo、SOEM，未发送机械臂命令。

尚未完成，需 LeRobot agent 和用户设备配合：

- 实际三相机图像与色卡验证；
- ROS mock 数据到 LeRobot feature 的端到端测试；
- VR 短 episode；
- 数据集可视化、ACT 训练与 mock rollout；
- 所有真机 policy action 测试。
