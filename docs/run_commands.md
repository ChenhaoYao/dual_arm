# 运行指令速查

## 编译

```bash
cd /home/dell/dual_arm

# 编译全部。统一使用软连接安装，修改 YAML/Python/launch 后只需重启节点。
colcon build --symlink-install

# MoveIt Servo / RViz / VR / 实物控制相关包
colcon build --symlink-install --packages-select \
  dual_arm_description dual_arm_control dual_arm_moveit_config \
  dual_arm_servo dual_arm_bringup dual_arm_soem_bridge vr_teleop_bridge

# 编译单个包（更快）
colcon build --symlink-install --packages-select dual_arm_soem_bridge

# 编译 SOEM 示例（通过 colcon）
colcon build --symlink-install --packages-select SOEM

# 或者直接用 cmake（更快，不经过 colcon）
cmake --build /home/dell/dual_arm/SOEM/build --target csv_test
cmake --build /home/dell/dual_arm/SOEM/build --target ec_sample
cmake --build /home/dell/dual_arm/SOEM/build --target slaveinfo
```

距离上次实物运行较久或工作区改动较多时，优先在没有 source 其他 ROS 工作区的
新终端中，只加载 Jazzy 后完整构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/dell/dual_arm
colcon build --symlink-install
```

这样可避免 `install/setup.bash` 继续串入无关 overlay。构建完成后，以下实物命令
统一 source `/home/dell/dual_arm/install/setup.bash`。

## 启动

所有命令默认在 `/home/dell/dual_arm` 下执行。推荐按以下顺序验证：

1. mock 硬件 + RViz Servo marker
2. mock 硬件 + VR
3. 真实机械臂 + RViz Servo marker
4. 真实机械臂 + VR

### MoveIt 规划仿真（mock 硬件）

```bash
cd /home/dell/dual_arm
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py mode:=moveit
```

### RViz Servo 仿真验证（mock 硬件，推荐先执行）

```bash
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_rviz_servo_marker:=true
```

RViz 启动后选择顶部的 **Interact** 工具。蓝色 marker 控制左臂，橙色 marker 控制右臂；拖动过程中只更新目标，松开鼠标后机械臂开始执行。目标超时或关节状态不完整时不会发送运动命令。

### VR Servo 仿真验证（mock 硬件）

```bash
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_vr_teleop:=true \
  enable_ros_tcp_endpoint:=true \
  ros_tcp_port:=10000
```

启用 VR 后会自动关闭 RViz Servo marker，避免两个输入 adapter 同时控制机械臂。Servo 模式不会启动 `move_group`，RViz 使用独立的 `servo.rviz` 配置。

VR 模式默认以 5 Hz 将原始手柄轨迹和机械臂末端反馈分别记录为：

```text
/home/dell/dual_arm/vr_teleop_bridge/log/<启动时间>/vr_left_hand_trajectory.csv
/home/dell/dual_arm/vr_teleop_bridge/log/<启动时间>/vr_right_hand_trajectory.csv
/home/dell/dual_arm/vr_teleop_bridge/log/<启动时间>/robot_left_ee_trajectory.csv
/home/dell/dual_arm/vr_teleop_bridge/log/<启动时间>/robot_right_ee_trajectory.csv
```

四个文件使用相同的表头和 `sample_index` 对齐，`elapsed_sec` 是本次记录开始后的简化秒数。`enabled=1` 表示对应 Grip 正在按下。Servo 在 14 个关节状态完整且新鲜后启动并保持运行；Grip 只控制是否发送运动命令，松开后发送零速度，不再反复暂停 Servo。位姿坐标和四元数保留 4 位小数。机械臂文件记录由 `/joint_states` 经 TF 得到、以 `base_link` 为参考的实际末端位姿，不是 Servo 指令目标。修改采样频率时直接编辑 `vr_teleop_bridge/config/trajectory_logger.yaml` 并重启 launch，无需重新编译。

### 实物自动预备位

实物 `real.launch.py` 默认仍不会自动运动。只有显式传入
`move_to_ready_on_start:=true` 时，启动流程才会自动打开 SOEM 软件命令门并执行：

```text
左臂：[ 0.3, 0,  1.2,  1.2, 0, 0, 0] rad
右臂：[-0.3, 0, -1.2, -1.2, 0, 0, 0] rad
```

这是左右镜像姿态，当前 URDF 下两臂 Jacobian 条件数均约为 22。两臂同时运动，
默认用 12 秒完成。配置位于
`dual_arm_bringup/config/ready_pose.yaml`。

自动预备位会先要求 14 轴反馈连续完整、左右 JTC 均为 `active`，并确认每只手臂
分别接近全零位或 ready 位。随后节点打开软件命令门，同时移动左右臂；
只有两臂都到达后才启动 MoveGroup/Servo/RViz/VR。任意检查、action 或最终位置
验证失败都会调用 `/stop` 并关闭本次 `real.launch.py`，不会继续启动上层控制。

启用该参数前必须确认物理急停可用、机械臂周围无人且两臂确实位于全零或 ready
姿态。若运动中途停止在未知位置，下一次自动启动会主动拒绝；不要设置
`allow_unknown_start: true` 绕过检查，应先人工确认并安全恢复到全零或 ready。

### MoveIt 规划实物模式

此模式故意不启用自动 ready，用于机械臂停在上一次运行的中间位置时，由 MoveIt
读取当前 14 轴反馈并规划恢复轨迹。`move_to_ready_on_start` 默认就是 `false`，
因此沿用最初的三终端手动 enable 流程。

```bash
# 终端 1：启动 MoveGroup + ros2_control + RViz；不会自动移动或打开命令门。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py mode:=moveit"

# 终端 2：启动 SOEM，读取真实编码器；软件命令门仍然关闭。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 确认 /joint_states 包含完整 14 轴并稳定发布。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /joint_states --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic hz /joint_states"

# 检查左右控制器 active、RViz 当前姿态正确，且 output.velocities 接近 0。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /left_arm_controller/controller_state --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /right_arm_controller/controller_state --once"

# 终端 3：急停可用、当前姿态和恢复规划均确认无误后，手动打开软件命令门。
# 必须先 enable，再在 RViz 中点击 Execute；禁止在命令门关闭时先执行轨迹再 enable。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

# 恢复完成或出现异常时关闭命令门并清除旧速度目标。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger '{}'"
```

### RViz Servo 实物验证

先确认急停可用、EtherCAT 标定和关节方向正确，并将速度限制设为适合首次测试的低值。真机启动参数默认关闭 marker，因此这里必须显式传入 `enable_rviz_servo_marker:=true`。

```bash
# 终端 1：先启动 SOEM，等待终端打印 [ENABLE] All axes enabled。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 2：确认真实反馈完整且稳定。默认约 20 Hz。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /joint_states --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic hz /joint_states"

# 终端 3：自动移动到镜像 ready，成功后再启动 Servo + RViz marker。
# 此命令会产生实物运动并自动打开软件命令门。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py mode:=servo enable_rviz_servo_marker:=true move_to_ready_on_start:=true"

# 终端 4：确认左右 JTC active。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'"

# 显式选择 Twist 模式并解除 Servo 暂停；四次调用都应返回 success=true。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/pause_servo std_srvs/srv/SetBool '{data: false}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/pause_servo std_srvs/srv/SetBool '{data: false}'"

# 此时不要拖动 Marker。先确认两臂 output.velocities 接近 0。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /left_arm_controller/controller_state --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /right_arm_controller/controller_state --once"

# 自动预备位成功后软件命令门已经打开；确认输出正常后才允许拖动 Marker。

# 另开终端准备软件停止；物理急停仍保持随时可按。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger '{}'"
```

未使用自动预备位、需要手动调用 `enable` 时，使能之前禁止拖动 Marker：电机命令门
关闭时 Servo/JTC 仍会更新参考位置，若先移动 Marker 再使能，机械臂会立即追赶
先前目标。首次测试只移动单臂、单方向和小距离。

SOEM 的命令 watchdog 为 300 ms。若 JTC/controller_state 链路中断，所有轴目标会
清零并锁定；恢复进程后必须重新调用 `enable`，旧目标不会自动恢复。

### VR Servo 实物模式

从 RViz Marker 切换到 VR 前，先关闭软件命令门并停止 Marker 对应的
`real.launch.py`。不要同时运行两套 `real.launch.py`：

```bash
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: false}'"
```

SOEM 可以保持运行。若是全新启动，仍然先启动 SOEM 并确认 `/joint_states`，再启动
VR 实物栈：

```bash
# 终端 1（全新启动时）：SOEM 桥接节点。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 2：自动移动到镜像 ready；成功后才启动 Servo、TCP Endpoint 和 VR bridge。
# 启动前保持两只 Grip 松开。此命令会产生实物运动并自动打开软件命令门。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py mode:=servo move_to_ready_on_start:=true enable_vr_teleop:=true enable_ros_tcp_endpoint:=true ros_tcp_port:=10000"

# 终端 3：确认两只手的 Grip 均已松开，并检查 VR 输入。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/left_hand/enabled --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/right_hand/enabled --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/status --once"

# 如输入 adapter 未自动配置，手动切换到 Twist 模式
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"

# Grip 保持松开，确认 controller_state 输出接近 0；软件命令门已由预备位节点打开。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /left_arm_controller/controller_state --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /right_arm_controller/controller_state --once"
```

真机 VR 模式下 `enable_vr_teleop:=true` 会强制关闭 RViz Servo marker，即使同时传入 `enable_rviz_servo_marker:=true`。

#### 启动 LeRobot ZMQ 数据桥

需要把 VR 示教过程传给 `/home/dell/lerobot` 时，在上述 SOEM、Servo、TCP
Endpoint 和 VR bridge 都正常后，再开一个独立终端启动数据桥：

```bash
# 终端 4：只增加 LeRobot 数据传输，不重复启动 real.launch.py。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup lerobot_bridge.launch.py"
```

启动前先在 `dual_arm_bringup/config/lerobot_bridge.yaml` 中核对三路相机的
`device`。正式使用时优先填写 `/dev/v4l/by-id/...`，避免重启后
`/dev/video0`、`/dev/video2`、`/dev/video4` 对应关系改变。

默认配置为 `allow_action_commands: false`，因此该节点只向 LeRobot 发送：

- `/joint_states` 的 14 轴实际状态；
- 左右 JTC reference 组成的 14 轴示教 action；
- 头部、左腕和右腕三路相机图像。

这里有两种容易混淆的 `action`：5557 的“示教 action”是桥向 LeRobot 发送的
训练标签，始终可以记录；5558 的“策略 action”是训练后的模型反向发给 JTC 的
控制命令。`allow_action_commands` 和 `/zmq_bridge_node/enable_actions` 只控制
5558，不会关闭上述三类采集数据。

两个 `enable` 服务也处在不同位置：

| 服务 | 控制的数据段 | 采集 VR 数据时是否调用 |
|---|---|---|
| `/zmq_bridge_node/enable_actions` | LeRobot 策略 action（5558）→ 左右 JTC | 不调用 |
| `/soem_bridge_node/enable` | JTC 输出 → EtherCAT 电机 | 按现有真机流程使用 |

前者不会打开、关闭或替代后者。即使 LeRobot 策略输入门关闭，原来的
VR → Servo → JTC → SOEM 控制链仍按现有 SOEM 门状态工作。

VR 数据采集期间不要调用 `/zmq_bridge_node/enable_actions`，机械臂仍只由原来的
VR → Servo 控制路径驱动。只有经过 mock 策略推理验证、准备执行策略 action 时，
才按照 `docs/lerobot_integration.md` 的分级流程修改配置并手动打开策略 action
输入门。

`lerobot_bridge.launch.py` 只能启动一次；重复启动会因为 ZMQ 端口
`5555`–`5560` 已占用而失败。停止 VR 实物栈后，也应在终端 4 用 `Ctrl+C` 停止
数据桥。

### 只启动 Unity ROS TCP Endpoint

```bash
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

### 只启动 VR bridge（Endpoint 和 MoveIt Servo 已经运行时）

```bash
source install/setup.bash
ros2 launch vr_teleop_bridge vr_teleop_bridge.launch.py
```

### 启动 Unity ROS TCP Endpoint + VR bridge（MoveIt Servo 已经运行时）

```bash
source install/setup.bash
ros2 launch vr_teleop_bridge vr_with_tcp_endpoint.launch.py ros_tcp_port:=10000
```

### SOEM 示例程序

```bash
# 扫描从站
sudo /home/dell/dual_arm/SOEM/build/samples/slaveinfo/slaveinfo enp0s31f6

# CSV 模式测试
sudo /home/dell/dual_arm/SOEM/build/samples/test/csv_test/csv_test enp0s31f6

# PP 模式测试
sudo /home/dell/dual_arm/SOEM/build/samples/ec_sample/ec_sample enp0s31f6
```

## 调试

### 临时解除 Servo 状态等待

如果 Servo 持续打印 `Waiting to receive robot state update.`，并且已经确认
`/joint_states` 正在发布完整的 14 个关节，可以在控制输入保持静止时执行：

```bash
cd /home/dell/dual_arm
source install/setup.bash
python3 tools/kick_servo_state_monitor.py
```

脚本读取当前完整关节状态，对 `laxis1_joint` 的副本临时增加 `1e-6 rad`，随后立即
恢复原值，重复三次后退出。它只用于规避 MoveIt Servo 2.12.4 的启动等待问题，
不会向控制器发送运动命令。默认按仿真使用；连接真实硬件时不要执行，除非已经
禁止 Servo 运动输出并明确传入 `--allow-real-hardware`。

### 话题查看

仿真可直接使用 `ros2 ...`。实物栈按本文命令以 root 启动，因此实物诊断也必须使用
相同的 root 环境，否则可能看不到对应节点、话题和服务。

```bash
# 查看话题列表（sudo）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic list"

# 查看实物 VR 手柄输入
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/left_hand/pose --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/right_hand/pose --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/left_hand/enabled --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/right_hand/enabled --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /vr/status --once"

# 查看实物 VR 到 Servo 的输出
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /servo_left/delta_twist_cmds --once"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /servo_right/delta_twist_cmds --once"

# 查看关节状态（sudo）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /joint_states --once"

# 持续查看关节状态
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /joint_states"

# 查看控制器状态
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /left_arm_controller/controller_state --once"

# 查看话题频率
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic hz /joint_states"

# 查看话题发布者/订阅者数量
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic info /joint_states"

# 查看话题 QoS 信息
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic info /joint_states --verbose"
```

### 节点查看

```bash
# 查看运行中的节点
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 node list"

# 查看节点信息（订阅/发布/服务）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 node info /soem_bridge_node"

# 查看 Unity TCP Endpoint
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 node info /UnityEndpoint"

# 查看 VR bridge
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 node info /vr_pose_to_servo_node"
```

### TF 查看

```bash
# 查看 TF 树
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 run tf2_tools view_frames"

# 查看 TF 数据
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /tf --once"

# 监控 TF
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 run tf2_ros tf2_monitor"

# 查看两个 frame 之间的变换
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 run tf2_ros tf2_echo base_link laxis7_link"
```

### 服务调用

```bash
# 打开软件命令门（驱动本身已由 SOEM/CiA402 状态机使能）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

# 关闭软件命令门并清除旧目标（不会切断驱动硬件使能）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: false}'"

# 软件停止：RT 命令门关闭并清除全部旧目标。物理急停独立使用。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger '{}'"

# 查询/请求故障复位流程。当前服务仅回执；实际复位由 RT 状态机自动处理。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/clear_fault std_srvs/srv/Trigger '{}'"

# 实物：选择 MoveIt Servo Twist 命令模式
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"

# 实物：暂停 MoveIt Servo
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/pause_servo std_srvs/srv/SetBool '{data: true}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/pause_servo std_srvs/srv/SetBool '{data: true}'"

# 实物：恢复 MoveIt Servo
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/pause_servo std_srvs/srv/SetBool '{data: false}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/pause_servo std_srvs/srv/SetBool '{data: false}'"
```

### 单电机测试

```bash
# 仅在 real.launch.py / JTC 没有运行时使用，避免测试命令与控制器同时写同一电机。
# 先打开 SOEM 命令门；若上次触发了看门狗，也必须重新执行这一步。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

# 格式：data: [joint_index, velocity_rad_s]。
# 300 ms 命令看门狗要求持续刷新；示例让 laxis1_joint (index=0) 以 0.02 rad/s 低速转动。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic pub -r 50 /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray '{data: [0, 0.02]}'"

# 在另一个终端执行软件停止，再对上面的持续发布命令按 Ctrl-C。
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger '{}'"

# 若直接 Ctrl-C 而没有先调用 /stop，命令中断约 300 ms 后看门狗也会清零速度并关闭命令门；
# 下一次测试前必须再次调用 enable(data=true)。
```

### 参数查看

```bash
# 查看节点参数
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 param dump /soem_bridge_node"

# 查看单个参数
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 param get /soem_bridge_node max_velocity"
```

## 权限设置

```bash
# 给 soem_bridge_node 授权原始套接字（避免 sudo）
sudo setcap cap_net_raw,cap_net_admin+ep /home/dell/dual_arm/install/dual_arm_soem_bridge/lib/dual_arm_soem_bridge/soem_bridge_node

# 重新编译后需要重新执行
```

### MoveIt 末端位姿控制（move_to_pose.py）

```bash
# 不用 sudo（move_group 以当前用户运行时可用）
source install/setup.bash
ros2 run dual_arm_description move_to_pose.py --ros-args \
  -p x:=0.3 -p y:=0.0 -p z:=0.5 \
  -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0

# 用 sudo（move_group 以 root 运行时必须用 sudo，否则 DDS 发现不了）
sudo bash -c "source /opt/ros/jazzy/setup.bash && source /home/dell/dual_arm/install/setup.bash && ros2 run dual_arm_description move_to_pose.py --ros-args -p x:=0.3 -p y:=0.0 -p z:=0.5 -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0"

# 指定规划组和末端执行器（默认 left_arm / laxis7_link）
ros2 run dual_arm_description move_to_pose.py --ros-args \
  -p group:=right_arm -p ee_link:=raxis7_link \
  -p x:=0.3 -p y:=-0.2 -p z:=0.4
```

参数说明：
- `x/y/z`：目标位置（米），相对于 `base_link`
- `roll/pitch/yaw`：目标姿态（弧度），ZYX 欧拉角
- `group`：规划组名（`left_arm` / `right_arm` / `dual_arm`）
- `ee_link`：末端执行器 link 名

## 日志查看

```bash
# 日志目录
ls /home/dell/.ros/log/

# 查看最新 launch 日志
cat /home/dell/.ros/log/<最新目录>/launch.log
```
