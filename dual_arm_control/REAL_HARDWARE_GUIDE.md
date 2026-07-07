# 实物接口改造说明

## 改造概述

将控制模式从**位置开环**改为**速度闭环**，JTC 输出 velocity 命令，通过 soem_bridge 发送给电机（CSV 模式）。

## 数据流

```
MoveIt → JTC (PID) → controller_state.output.velocities → soem_bridge → EtherCAT → 电机
         ↑                                                                              │
         └──────────── /joint_states ← DualArmHardware ← soem_bridge ← 编码器 ←────────┘
```

## 修改文件

| 文件 | 修改内容 |
|------|----------|
| `real.launch.py` | 添加 `use_broadcaster` 参数（默认 false） |
| `dual_arm_hardware.hpp` | 添加 ROS2 节点和订阅者成员变量 |
| `dual_arm_hardware.cpp` | 订阅 `/joint_states`，read() 中更新state_interface |
| `ros2_controllers_real.yaml` | command_interfaces 改为 velocity，添加 PID 参数 |
| `soem_bridge_node.cpp` | 读取 `output.velocities`，添加单电机测试话题 |
| `CMakeLists.txt` | 添加 sensor_msgs 依赖 |
| `package.xml` | 添加 sensor_msgs 依赖 |

## 启动命令

```bash
# Terminal 1: MoveGroup + ros2_control + RViz
ros2 launch dual_arm_bringup real.launch.py

# Terminal 2: soem_bridge
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py

# Terminal 3: 使能电机
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
```

## 测试步骤

### 1. 验证数据流

```bash
# 检查 /joint_states 有数据（真实编码器位置）
ros2 topic hz /joint_states

# 检查 controller_state 有 output.velocities
ros2 topic echo /left_arm_controller/controller_state --once
```

### 2. 单电机测试

```bash
# 格式：[关节索引, 速度(rad/s)]
# laxis1_joint = index 0, 以 0.5 rad/s 转动
ros2 topic pub /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray "{data: [0, 0.5]}"

# 停止：发送 0
ros2 topic pub /soem_bridge_node/test_axis std_msgs/msg/Float64MultiArray "{data: [0, 0.0]}"
```

### 3. MoveIt 轨迹测试

在 RViz 中拖动末端执行器，点击 Plan & Execute，观察电机是否跟随。

## PID 调参

### 当前参数

```yaml
gains:
  laxis1_joint: {p: 5.0, i: 0.0, d: 0.1}
  # ... 其余关节相同
```

### 调参顺序

1. **P（比例）**：从 5.0 开始，逐步增大
   - 太小：响应慢，跟踪误差大
   - 太大：振荡
   - 目标：增大到开始振荡，然后减小 50%

2. **D（微分）**：抑制振荡
   - 从 0.1 开始，逐步增大
   - 太大：高频噪声

3. **I（积分）**：消除稳态误差
   - 从 0.0 开始，逐步增大
   - 太大：超调、积分饱和

### 动态调参

```bash
# 使用 rqt_joint_trajectory_controller
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

# 或者直接修改参数
ros2 param set /left_arm_controller laxis1_joint.p 10.0
```

## 关节索引对照

| 索引 | 关节 | 索引 | 关节 |
|------|------|------|------|
| 0 | laxis1_joint | 7 | raxis1_joint |
| 1 | laxis2_joint | 8 | raxis2_joint |
| 2 | laxis3_joint | 9 | raxis3_joint |
| 3 | laxis4_joint | 10 | raxis4_joint |
| 4 | laxis5_joint | 11 | raxis5_joint |
| 5 | laxis6_joint | 12 | raxis6_joint |
| 6 | laxis7_joint | 13 | raxis7_joint |

## 单位换算

| 单位 | 范围 | 用途 |
|------|------|------|
| rad | -3.14 ~ 3.14 | ROS2 标准 |
| degree | -180 ~ 180 | 零点偏置配置 |
| count | 0 ~ 52428800 | 电机编码器 |

PID 参数单位：p=5.0 表示误差 1 rad 时输出 5 rad/s。
