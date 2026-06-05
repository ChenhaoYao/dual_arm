# 控制架构

## 整体数据流

```
用户指定末端目标位姿
       │
       ▼
┌─────────────────────┐
│  MoveIt OMPL 规划    │  碰撞检测、运动学求解、路径搜索
│  生成完整轨迹        │  (positions + velocities + time_from_start)
└─────────┬───────────┘
          │
          ▼
┌─────────────────────────────────────────────┐
│  JointTrajectoryController (100Hz)          │  接收轨迹，按时间戳逐点插值
│  FollowJointTrajectory action               │
└─────────────────────────────────────────────┘
          │
          ▼
       电机执行
```

## 控制器层级

```
left_arm_controller  =  JointTrajectoryController（同一个东西，不是上下级）
```

- `left_arm_controller` 是 `JointTrajectoryController` 的实例名
- 定义在 `config/ros2_controllers.yaml`
- MoveIt 通过 `FollowJointTrajectory` action 发送完整轨迹给它

## ros2_control_node 内部架构

`command_interface` 和 `state_interface` 是 `ros2_control_node` 进程内的**共享内存接口**，不是话题。

```
┌─────────────────────────────────────────────────────────────┐
│  ros2_control_node 进程 (100Hz update loop)                 │
│                                                             │
│  ┌──────────────────┐         ┌─────────────────────────┐  │
│  │ left_arm_        │         │ Hardware Interface      │  │
│  │ controller       │         │ (mock_components /      │  │
│  │                  │         │  DualArmHardware)       │  │
│  │ command.position ──写入──▶  │ command.position        │  │
│  │                  │         │       │                 │  │
│  │ state.position  ◀──读取───  │       ▼                 │  │
│  │                  │         │       │                 │  │
│  │                  │         │ state.position ─────────┤  │
│  └──────────────────┘         └─────────────────────────┘  │
│                                                             │
│  /joint_states (话题，对外发布 state_interface 的值)         │
└─────────────────────────────────────────────────────────────┘
```

| 接口 | 类型 | 作用 |
|------|------|------|
| `command_interface.position` | 共享内存 | 控制器写入命令，硬件插件读取 |
| `state_interface.position` | 共享内存 | 硬件插件写入状态，控制器读取 |
| `/joint_states` | 话题 | `controller_manager` 把 `state_interface` 的值发布出去 |

## JointTrajectoryController 工作方式

MoveIt 规划完成后，一次性发送完整轨迹（几十到几百个点）：

```
trajectory = [
  { time: 0.0s, pos: [0.1, 0.2, ...], vel: [...] },
  { time: 0.1s, pos: [0.15, 0.25, ...], vel: [...] },
  { time: 0.2s, pos: [0.2, 0.3, ...], vel: [...] },
  ...
]
```

Controller 按 100Hz 插值，每 10ms 下发一个位置命令：

```
t=0.00s → pos = 0.10
t=0.01s → pos = 0.11  ← 插值
t=0.02s → pos = 0.12  ← 插值
...
```

## controller_state 话题

`/left_arm_controller/controller_state` 发布的内容：

```yaml
joint_names: [laxis1_joint, laxis2_joint, ...]
actual:
  positions: [...]    ← 来自 state_interface（硬件反馈）
  velocities: [...]
reference:
  positions: [...]    ← 当前插值目标位置
  velocities: [...]   ← 当前插值目标速度
```

## 硬件插件对比

| | mock_components/GenericSystem | DualArmHardware (真实) |
|---|---|---|
| 位置反馈 | 命令直接复制 | 编码器读数 |
| 速度反馈 | 0 或命令值 | 实际测量 |
| 碰撞 | 无反应 | 物理限制 |
| 故障 | 无反应 | 驱动器报错 |
| 跟踪误差 | 永远为 0 | 存在延迟和误差 |

## dry_run 模式链路

dry_run 模式下，从 MoveIt 到 controller_state 的链路是完整的，只是最后不连接真实电机：

```
MoveIt → JTC 插值 (100Hz) → mock 硬件 → controller_state
                                            │
                                            ▼ 10Hz 采样
                                    soem_bridge (dry_run)
                                    只打印换算结果，不发给电机
```
