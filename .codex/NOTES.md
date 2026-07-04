# 项目笔记

## Unity / PICO

- PICO 应用是主动连接 ROS 主机的，主机只需要启动 `ros_tcp_endpoint` 并监听端口。
- 当前项目使用 TCP，不使用 UDP。TCP 由 Unity Robotics 的 ROS-TCP-Connector 和 ROS 侧的 `ros_tcp_endpoint` 提供。
- PICO 设备断电或重启后，不需要每次通过 USB 连接主机，也不需要每次重新打开 Unity Hub。只要 APK 已安装，日常使用时在 PICO 里启动应用即可。
- 只有修改 Unity 脚本、scene、prefab、PICO 设置或 ROS 连接 IP/端口后，才需要重新 Build/Install 到 PICO。
- `ROSConnection.GetOrCreateInstance()` 会通过 Unity `Assets/Resources/` 机制自动找到 `ROSConnectionPrefab.prefab`。

## Unity 文件

- `Packages/manifest.json` 是项目声明的直接依赖，通常可以手动改。
- `Packages/packages-lock.json` 是 Unity Package Manager 解析后的锁定文件，通常让 Unity 自动更新。
- `.asset` 和 `.prefab` 是 Unity 资源文件格式，不是 C# 特性。
- `Assets/`、`Packages/`、`ProjectSettings/` 应该纳入版本管理。
- `Library/`、`Logs/`、`UserSettings/`、`.csproj`、`.sln`、APK/AAB 产物应该忽略。

## ROS / MoveIt

- 仿真入口：`dual_arm_bringup/launch/sim.launch.py`
- 实物入口：`dual_arm_bringup/launch/real.launch.py`
- 当前实物和仿真接口已经分开，不要把 `real.launch.py` 当作废弃文件。
- VR 输入话题：
  - `/vr/left_hand/pose`
  - `/vr/right_hand/pose`
- MoveIt Servo 输出话题：
  - `/servo_left/delta_twist_cmds`
  - `/servo_right/delta_twist_cmds`

## 已知排查方向

如果 ROS 侧看不到稳定 VR pose：

1. 确认主机 IP 与 `ROSConnectionPrefab.prefab` 中配置一致。
2. 启动 `ros_tcp_endpoint` 并确认监听 `10000`。
3. 在 PICO 中启动已安装的 VR 应用。
4. 用 `ros2 topic list -t` 查看 `/vr/*` topic 是否出现。
5. 如果 topic 短暂出现后消失，优先用 USB 连接 PICO，查看 `adb logcat`。

日志建议过滤：

```bash
adb logcat | grep -i -E "ros|ROSConnection|VRHandPublisher|Exception|error"
```
