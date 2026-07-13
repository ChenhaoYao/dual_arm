# 运行指令速查

## 编译

```bash
cd /home/dell/dual_arm

# 编译全部。统一使用软连接安装，修改 YAML/Python/launch 后只需重启节点。
colcon build --symlink-install

# 本次 MoveIt Servo / RViz / VR 相关包
colcon build --symlink-install --packages-select \
  dual_arm_moveit_config dual_arm_servo dual_arm_bringup vr_teleop_bridge

# 编译单个包（更快）
colcon build --symlink-install --packages-select dual_arm_soem_bridge

# 编译 SOEM 示例（通过 colcon）
colcon build --symlink-install --packages-select SOEM

# 或者直接用 cmake（更快，不经过 colcon）
cmake --build /home/dell/dual_arm/SOEM/build --target csv_test
cmake --build /home/dell/dual_arm/SOEM/build --target ec_sample
cmake --build /home/dell/dual_arm/SOEM/build --target slaveinfo
```

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

四个文件使用相同的表头和 `sample_index` 对齐，`elapsed_sec` 是本次记录开始后的简化秒数。`enabled=1` 表示对应 Grip 正在按下。Grip 按下后会在 14 个关节状态完整且新鲜时激活对应 Servo，松开后发送零速度并暂停对应 Servo。位姿坐标和四元数保留 4 位小数。机械臂文件记录由 `/joint_states` 经 TF 得到、以 `base_link` 为参考的实际末端位姿，不是 Servo 指令目标。修改采样频率时直接编辑 `vr_teleop_bridge/config/trajectory_logger.yaml` 并重启 launch，无需重新编译。

### MoveIt 规划实物模式

```bash
# 终端 1：MoveGroup + ros2_control + RViz（real.launch.py 默认 mode:=moveit, use_broadcaster:=false）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py"

# 终端 2：SOEM 桥接节点
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 3：使能电机（需要 sudo，因为 soem_bridge 以 root 运行，DDS 按用户隔离）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

sudo bash -c 'source /home/dell/dual_arm/install/setup.bash && ros2 topic echo  /left_arm_controller/controller_state'
```

### RViz Servo 实物验证

先确认急停可用、EtherCAT 标定和关节方向正确，并将速度限制设为适合首次测试的低值。真机启动参数默认关闭 marker，因此这里必须显式传入 `enable_rviz_servo_marker:=true`。

```bash
# 终端 1：MoveIt Servo + 真实硬件接口 + RViz marker
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py mode:=servo enable_rviz_servo_marker:=true"

# 终端 2：SOEM 桥接节点
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 3：确认状态正常后使能电机
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"
```

### VR Servo 实物模式

```bash
# 终端 1：MoveIt Servo + 真实硬件接口 + Unity TCP Endpoint + VR bridge
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup real.launch.py mode:=servo enable_vr_teleop:=true enable_ros_tcp_endpoint:=true ros_tcp_port:=10000"

# 终端 2：SOEM 桥接节点
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 3：使能电机
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

# 如输入 adapter 未自动配置，手动切换到 Twist 模式
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'"
```

真机 VR 模式下 `enable_vr_teleop:=true` 会强制关闭 RViz Servo marker，即使同时传入 `enable_rviz_servo_marker:=true`。

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

```bash
# 查看话题列表（sudo）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic list"

# 查看 VR 手柄输入
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/right_hand/pose --once
ros2 topic echo /vr/left_hand/enabled --once
ros2 topic echo /vr/right_hand/enabled --once
ros2 topic echo /vr/status --once

# 查看 VR 到 Servo 的输出
ros2 topic echo /servo_left/delta_twist_cmds --once
ros2 topic echo /servo_right/delta_twist_cmds --once

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
ros2 node info /UnityEndpoint

# 查看 VR bridge
ros2 node info /vr_pose_to_servo_node
```

### TF 查看

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看 TF 数据
ros2 topic echo /tf --once

# 监控 TF
ros2 run tf2_ros tf2_monitor

# 查看两个 frame 之间的变换
ros2 run tf2_ros tf2_echo base_link laxis7_link
```

### 服务调用

```bash
# 使能电机
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: true}'"

# 关闭电机
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool '{data: false}'"

# 紧急停止
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger"

# 故障复位
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 service call /soem_bridge_node/clear_fault std_srvs/srv/Trigger"

# 选择 MoveIt Servo Twist 命令模式
ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

# 暂停 MoveIt Servo
ros2 service call /servo_left/pause_servo std_srvs/srv/SetBool "{data: true}"
ros2 service call /servo_right/pause_servo std_srvs/srv/SetBool "{data: true}"

# 恢复 MoveIt Servo
ros2 service call /servo_left/pause_servo std_srvs/srv/SetBool "{data: false}"
ros2 service call /servo_right/pause_servo std_srvs/srv/SetBool "{data: false}"
```

### 单电机测试

```bash
# 格式：ros2 topic pub /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray "{data: [joint_index, velocity_rad_s]}"

# 示例：laxis1_joint (index=0) 以 0.5 rad/s 转动
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic pub /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray '{data: [0, 0.5]}'"

# 停止
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic pub /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray '{data: [1, 0.0]}'"
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
