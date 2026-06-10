# 运行指令速查

## 编译

```bash
# 编译全部
colcon build

# 编译单个包（更快）
colcon build --packages-select dual_arm_soem_bridge

# 编译 SOEM 示例
cmake --build /home/dell/dual_arm/SOEM/build --target csv_test
cmake --build /home/dell/dual_arm/SOEM/build --target ec_sample
```

## 启动

### 仿真模式（mock 硬件）

```bash
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py
```

### 实物模式

```bash
# 终端 1：MoveIt（关闭 broadcaster 和仿真时钟）
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_bringup sim.launch.py use_sim_time:=false use_broadcaster:=false"

# 终端 2：SOEM 桥接节点
sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 launch dual_arm_soem_bridge soem_bridge.launch.py"

# 终端 3：使能电机
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
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

### 话题查看

```bash
# 查看话题列表
ros2 topic list

# 查看关节状态
ros2 topic echo /joint_states --once

sudo bash -c "source /home/dell/dual_arm/install/setup.bash && ros2 topic echo /joint_states"

# 查看话题频率
ros2 topic hz /joint_states

# 查看话题 QoS 信息
ros2 topic info /joint_states --verbose
```

### 节点查看

```bash
# 查看运行中的节点
ros2 node list

# 查看节点信息（订阅/发布/服务）
ros2 node info /soem_bridge_node
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
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"

# 关闭电机
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: false}"

# 紧急停止
ros2 service call /soem_bridge_node/stop std_srvs/srv/Trigger

# 故障复位
ros2 service call /soem_bridge_node/clear_fault std_srvs/srv/Trigger
```

### 参数查看

```bash
# 查看节点参数
ros2 param dump /soem_bridge_node

# 查看单个参数
ros2 param get /robot_state_publisher use_sim_time
```

## 权限设置

```bash
# 给 soem_bridge_node 授权原始套接字（避免 sudo）
sudo setcap cap_net_raw,cap_net_admin+ep /home/dell/dual_arm/install/dual_arm_soem_bridge/lib/dual_arm_soem_bridge/soem_bridge_node

# 重新编译后需要重新执行
```

## 日志查看

```bash
# 日志目录
ls /home/dell/.ros/log/

# 查看最新 launch 日志
cat /home/dell/.ros/log/<最新目录>/launch.log
```
