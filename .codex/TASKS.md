# 当前任务

## 正在做

- [ ] 验证 PICO/Unity 应用是否能稳定向 ROS 2 发布手柄位姿。
- [ ] 确认 `/vr/left_hand/pose` 和 `/vr/right_hand/pose` 是否能持续 echo 到 `geometry_msgs/msg/PoseStamped`。
- [ ] 如果 topic 只短暂出现，抓取 PICO 端 `adb logcat`，定位 Unity 应用是否崩溃、断连或没有执行 `VRHandPublisher`。

## 优先修改文件

优先级从高到低：

1. `vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs`
   - 如果 PICO 端没有持续发布位姿，优先在这里加日志、检查 XRNode、检查 publish 频率。
2. `vrtest-full-lite/vrtest/Assets/Resources/ROSConnectionPrefab.prefab`
   - 如果 PICO 连不到 ROS 主机，检查这里的 IP 和端口。
3. `vr_teleop_bridge/config/vr_teleop_bridge.yaml`
   - 如果 ROS 已收到 VR pose，但机械臂响应不合适，调整比例、限速、死区、超时等参数。
4. `dual_arm_bringup/launch/sim.launch.py`
   - 如果仿真启动链路有问题，检查 VR bridge 和 TCP endpoint 是否随 launch 正确启动。
5. `dual_arm_bringup/launch/real.launch.py`
   - 如果实物启动链路有问题，检查真实硬件、MoveIt Servo、VR bridge、TCP endpoint 的组合启动。
6. `docs/run_commands.md`
   - 如果启动流程变化，及时更新命令。
7. `docs/unity_pico_notes.md`
   - 如果 Unity/PICO 环境或排错经验变化，及时补充。

## 推荐下一步

1. 启动 ROS TCP endpoint：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py tcp_ip:=0.0.0.0 tcp_port:=10000
```

2. 在 PICO 中打开已安装的 VR 应用。

3. 在 ROS 侧检查话题：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 topic list -t
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/right_hand/pose --once
```

4. 如果没有稳定数据，用 USB 连接 PICO 并抓日志：

```bash
adb logcat | grep -i -E "ros|ROSConnection|VRHandPublisher|Exception|error"
```

## 后续增强

- [ ] 给 `VRHandPublisher.cs` 增加明确日志：连接状态、左右手柄是否有效、发布频率。
- [ ] 考虑把 ROS 主机 IP 从 prefab 固定配置改成 PICO 端运行时可配置。
- [ ] 在 `docs/run_commands.md` 中补充 PICO 端 adb 调试命令。
- [ ] 完成一次仿真闭环测试：PICO pose -> ROS topic -> `vr_teleop_bridge` -> MoveIt Servo command。
- [ ] 完成一次实物前的安全验证：限速、死区、急停、enable 流程。
