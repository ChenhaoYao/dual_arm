# Codex 项目上下文

这个目录用于记录 `dual_arm` 工作区当前正在推进的事情，方便其他人或之后的 Codex 会话快速接上进度。

## 当前目标

当前重点是把 PICO/Unity VR 手柄位姿接入 ROS 2/MoveIt，实现双臂机器人遥操作。

总体链路：

```text
PICO 手柄位姿
  -> Unity 项目 vrtest
  -> ROS-TCP-Connector
  -> ros_tcp_endpoint
  -> /vr/left_hand/pose, /vr/right_hand/pose
  -> vr_teleop_bridge
  -> MoveIt Servo
  -> 双臂控制器
```

## 当前进度

- Unity 项目路径：`/home/dell/dual_arm/vrtest-full-lite/vrtest`
- ROS 工作区路径：`/home/dell/dual_arm`
- Unity Editor 使用版本：`2022.3.62f3`
- PICO SDK 已放到：`/home/dell/dual_arm/third_party/PICO Unity Integration SDK-3.4.0-20260226`
- Unity 依赖已通过 `Packages/manifest.json` 指向本地 PICO SDK
- Unity 侧 ROS 连接 prefab 位于：`vrtest-full-lite/vrtest/Assets/Resources/ROSConnectionPrefab.prefab`
- Unity 侧读取手柄位姿的脚本是：`vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs`
- ROS 侧已经加入 `vr_teleop_bridge`，用于把 VR pose 转换为 MoveIt Servo twist 命令
- `dual_arm_bringup` 的 `sim.launch.py` 和 `real.launch.py` 已加入 VR/TCP endpoint 相关启动参数
- Unity 项目已加入 `.gitignore`，忽略 `Library/`、`Logs/`、`.csproj`、APK 等生成文件
- Unity/PICO/ROS 相关知识整理在：`docs/unity_pico_notes.md`
- 启动命令整理在：`docs/run_commands.md`

## 当前状态

ROS 主机侧 `ros_tcp_endpoint` 可以启动并监听 `10000` 端口。

之前测试时，PICO/Unity 侧的 VR topic 曾短暂出现在 ROS 2 中：

```text
/vr/left_hand/pose
/vr/right_hand/pose
```

但 `ros2 topic echo` 没有稳定收到位姿消息，随后 TCP 连接消失。当前判断是主机侧基本可用，下一步应优先排查 PICO/Unity 应用是否持续运行、是否崩溃、是否实际执行了 `VRHandPublisher`。

## 推荐优先查看

1. `docs/unity_pico_notes.md`
2. `docs/run_commands.md`
3. `vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs`
4. `vrtest-full-lite/vrtest/Assets/Resources/ROSConnectionPrefab.prefab`
5. `vr_teleop_bridge/config/vr_teleop_bridge.yaml`
6. `dual_arm_bringup/launch/sim.launch.py`
7. `dual_arm_bringup/launch/real.launch.py`

具体待办见：

- `.codex/TASKS.md`：PICO/Unity VR pose 稳定发布与 VR 闭环任务。
- `.codex/SERVO_HANDOFF.md`：MoveIt Servo 模块达到最终可用状态的交接计划。
