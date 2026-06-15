# ROS2 核心概念

## Spin 循环与回调

```cpp
auto node = std::make_shared<MyNode>();  // 构造函数：注册回调
rclcpp::spin(node);                       // 事件循环：触发回调
```

| 类型 | 创建函数 | 触发方式 |
|------|----------|----------|
| Subscription | `create_subscription()` | 自动（收到消息） |
| Publisher | `create_publisher()` | 手动 `publish()` |
| Timer | `create_wall_timer()` | 自动（定时） |
| Service | `create_service()` | 自动（收到请求） |

---

## QoS（Quality of Service）

```cpp
rclcpp::QoS(10)                // 队列大小 10，默认 RELIABLE
rclcpp::QoS(10).best_effort()  // BEST_EFFORT（像 UDP）
```

| 策略 | 行为 | 适合 |
|------|------|------|
| RELIABLE | 保证送达，像 TCP | 服务、重要指令 |
| BEST_EFFORT | 发了就不管，像 UDP | 传感器高频数据 |

**坑**：robot_state_publisher 默认 BEST_EFFORT，发布者必须匹配：
```cpp
create_publisher<JointState>("/joint_states", rclcpp::QoS(10).best_effort());
```

---

## TF 与 robot_state_publisher

```
/joint_states → robot_state_publisher → /tf → RViz/MoveIt
```

**作用**：把关节角度转成坐标变换，一次计算，处处使用。

**排查**：
```bash
ros2 run tf2_tools view_frames    # 查看 TF 树
ros2 topic echo /tf --once        # 检查 TF 数据
```

---

## ROS2 参数系统

```yaml
# soem_bridge.yaml
soem_bridge_node:
  ros__parameters:
    ifname: "enp0s31f6"
    gear_ratio: [100.0, 50.0]
```

```cpp
// 节点代码
auto ifname = declare_parameter<std::string>("ifname", "");
auto ratios = declare_parameter<std::vector<double>>("gear_ratio", default_vec);
if (ratios.size() < n) ratios.resize(n, 100.0);  // 补齐
```

**Launch 加载**：
```python
Node(package='soem_bridge', parameters=[config_file])  # 传路径，自动解析
```

---

## ROS2 日志

```cpp
RCLCPP_INFO(this->get_logger(), "轴数: %d", n);   // INFO/WARN/ERROR
```

---

## DDS 用户隔离

**问题**：不同用户启动的节点互相发现不了。

**解决**：
```bash
# 方案 1：全部 sudo
sudo bash -c "source install/setup.bash && ros2 launch ..."

# 方案 2：setcap 授权（推荐）
sudo setcap cap_net_raw,cap_net_admin+ep /path/to/node
```

---

## URDF 关键点

### initial_value 位置

必须写在 `<state_interface>` 内部：
```xml
<joint name="laxis1_joint">
  <state_interface name="position">
    <param name="initial_value">0.0</param>  <!-- ✅ 正确 -->
  </state_interface>
</joint>
```

### collision vs visual

```
<visual>    → RViz 显示，用真实 STL
<collision> → 碰撞检测，用简单几何体（box/cylinder）
```

### 排查清单

```
初始位姿不生效：initial_value 写对位置了吗？install 是 symlink 吗？
MoveIt 报碰撞：collision 几何体过大？朝向对齐 axis 吗？
```

---

## ros2_control 架构

```
Controller Manager
    ├── Hardware Interface (read/write)
    └── Controllers (JTC, broadcaster...)
```

| 接口 | 方向 | 用途 |
|------|------|------|
| state_interface | 硬件 → 控制器 | 位置、速度反馈 |
| command_interface | 控制器 → 硬件 | 位置、速度命令 |

### JTC Velocity 模式

```
velocity_cmd = ref_vel + Kp*(ref_pos - actual_pos) + Ki*∫error + Kd*d(error)/dt
```

配置：
```yaml
command_interfaces: [velocity]
state_interfaces: [position, velocity]
gains:
  laxis1_joint: {p: 5.0, i: 0.0, d: 0.1}
```

### 仿真 vs 实物的命令接口选择

**问题**：`ros2_controllers.yaml` 的 `command_interfaces` 设为 `velocity`，仿真时机械臂不动。

**原因**：`mock_components/GenericSystem` 的 `write()` 是空操作，`read()` 只把命令值拷贝到同名状态接口。velocity 命令只更新 velocity 状态，**不会积分成位置**，所以 position 永远是初始值 0。

**解决**：仿真用 `position` 命令接口，实物用 `velocity` 命令接口，通过 launch 参数区分：
```yaml
# ros2_controllers.yaml（仿真）
command_interfaces: [position]

# ros2_controllers_real.yaml（实物）
command_interfaces: [velocity]
```
```python
# demo.launch.py
DeclareLaunchArgument('controllers_config', default_value='ros2_controllers.yaml')
# sim.launch.py  → controllers_config: 'ros2_controllers.yaml'
# real.launch.py → controllers_config: 'ros2_controllers_real.yaml'
```

### controller_state 消息

```
reference.positions   → 期望位置
reference.velocities  → 期望速度（前馈）
output.velocities     → PID 输出（velocity 模式）
```

### JTC 持续发布 controller_state

**现象**：Execute 完成后，`/left_arm_controller/controller_state` 话题仍在持续发布。

**原因**：这是 ROS2 控制器的正常行为。JTC 会持续发布 controller_state，即使轨迹执行完成。这是因为：
- JTC 需要持续发布状态信息供其他节点监控
- controller_state 包含当前位置、速度等信息

**影响**：
- 不影响轨迹执行
- 但如果有多个控制器（左右臂），会同时发布到各自的话题
- 如果同时订阅多个控制器的 controller_state，会导致数据混合

**解决方案**：在 soem_bridge_node 中，根据 `active_arm` 参数只订阅一个臂的 controller_state：
```yaml
# soem_bridge.yaml
active_arm: left  # 只订阅左臂
```

### 重复 Plan & Execute 不生效

**现象**：起点终点不变，重新点击 Plan & Execute，controller_state 仍发布上一次的数据。

**原因**：JTC 认为新轨迹与旧轨迹相同，跳过执行。

**解决**：在 RViz Motion Planning 面板中点击 **Reset**，再重新 Plan & Execute。

---

## Hardware Interface (Jazzy 新 API)

```cpp
// ❌ 旧 API
on_init(const hardware_interface::HardwareInfo & info)

// ✅ 新 API
on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
// 访问：params.hardware_info
```

### 话题桥接

在 hardware_interface 中订阅外部话题更新 state_interface：
```cpp
on_configure() {
  node_ = std::make_shared<rclcpp::Node>("hw_node");
  sub_ = node_->create_subscription<JointState>("/joint_states", ...);
}
read() {
  rclcpp::spin_some(node_);
}
```

### 实战案例：QoS 不兼容导致 JTC 失控（pid 恒等于 p×ref）

**现象**：实物实验（`real.launch.py` + `soem_bridge.launch.py`），在 RViz Motion Planning 发指令后，所有关节匀速狂转停不下来。查 `controller_state` 日志发现：即使 `ref` 已接近真实反馈 `fb`，JTC 的 `output.velocities`（pid 列）依然是很大的速度。

**定位的关键证据**：逐行核对日志，发现每个关节都满足
```
pid = 5.0 × ref      # 5.0 正是 ros2_controllers_real.yaml 里的 p 增益
```
即 `pid = p·(ref − measured)` 中 `measured ≡ 0`。说明 **JTC 看到的实际位置一直是 0**，误差永远等于 ref，输出永远是 `p×ref`，永不收敛。而真实编码器 `fb` 在不断变化（越过目标也不减速），证明 JTC 根本没收到反馈。

**数据流**：
```
JTC ──output.velocities──> soem_bridge ──EtherCAT──> 电机
                                              │
电机编码器 ──/joint_states──> DualArmHardware::read() ──state_interface──> JTC
```
闭环靠最后这条 `/joint_states → DualArmHardware → JTC` 把真实位置喂回 JTC。

**根因**：`/joint_states` 两端 QoS 不兼容。
- 发布端 soem_bridge_node：`rclcpp::QoS(10).best_effort()`
- 订阅端 DualArmHardware：`rclcpp::QoS(10)`（默认 **RELIABLE**）

DDS 规则：**RELIABLE 订阅者无法接收 BEST_EFFORT 发布者的数据**，连接建立不起来，`joint_state_callback` 永不触发，`hw_position_states_` 永远停在 URDF 的 `initial_value=0.0`，闭环断开。

**解决**：把订阅端改成 best_effort，与发布端对齐：
```cpp
// dual_arm_control/src/dual_arm_hardware.cpp on_configure()
joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", rclcpp::QoS(10).best_effort(),   // 原为 rclcpp::QoS(10)
  [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_callback(msg); });
```

**验证**：
```bash
ros2 topic info /joint_states -v   # 订阅端 QoS 应为 BEST_EFFORT，无 incompatible 计数
```
RViz 发小幅运动，观察 pid 随 fb 接近 ref 而衰减到 ~0。

**经验**：JTC 输出 `output.velocities ≡ p×ref`（正比于参考、与反馈无关）是"反馈没喂回控制器、measured 恒为 0"的典型特征——优先怀疑 state_interface 反馈链路（QoS、话题名、关节名匹配），而不是 PID 增益本身。

### axis_directions 不会破坏 JTC 闭环（dir² 抵消）

**疑问**：在 `soem_bridge.yaml` 用 `axis_directions` 统一各电机转向，又在 `soem_master.cpp` 的 rad/count 转换里多处乘了 `direction`，会不会让 JTC 出问题？

**结论：不会。** `direction` 在闭环里出现两次正好抵消：
- 命令路径（出）：`vel_to_counts` 中 `counts = vel × scale × direction`
- 反馈路径（回）：`counts_to_rad` 中 `rad = counts / scale × direction`

推导：JTC 在 ROS 坐标系发 `+v` →电机 `tgt = +v·scale·dir` →编码器 `d(counts)/dt = +v·scale·dir` →回报 `measured = counts/scale·dir` →
```
d(measured)/dt = v · dir² = v > 0   （dir² 恒为 1）
```
所以无论 dir 是 +1 还是 −1，"发 +速度 → 上报位置增大" 永远成立，闭环恒为稳定的负反馈。`direction` 只改变电机物理转向，不影响 ROS 层的环路一致性。`zero_offset` 同理（`rad_to_counts` 与 `counts_to_rad` 互为逆运算）。

**唯一前提**：命令和反馈必须用同一个 `direction` 值（代码两处都读 `cfg.direction`，已满足）。

**注意**：
- 若闭环接通后仍有**单个轴**朝一个方向飞车，说明该轴 `direction` 与物理/编码器不符（变成正反馈），翻转该轴即可。
- `axis_zero_offsets` 标错只会让机械臂驱到错误的绝对位姿，不会失控发散（属标定问题，与稳定性无关）。

---

## Launch 参数传递

```python
# sim.launch.py → demo.launch.py → URDF
hw_plugin: 'mock_components/GenericSystem'  # 仿真
hw_plugin: 'dual_arm_control/DualArmHardware'  # 实物

# 条件启动 broadcaster
GroupAction(condition=IfCondition(use_broadcaster), ...)
```

---

## 单位换算

| 单位 | 范围 | 用途 |
|------|------|------|
| rad | -3.14 ~ 3.14 | ROS2 标准 |
| degree | -180 ~ 180 | 配置文件 |
| count | 0 ~ 2^enc_bits × gear_ratio | 电机编码器 |

```cpp
double scale = (1 << enc_bits) * gear_ratio / (2 * M_PI);
int32_t counts = round(rad * scale) * direction + zero_offset;
```

**PID 单位**：p=5.0 → 误差 1 rad 输出 5 rad/s

---

## MoveIt2 配置

### 架构

```
RViz → move_group → OMPL 规划 → 时间参数化 → ros2_control 执行
         ├── kinematics.yaml
         ├── ompl_planning.yaml
         ├── joint_limits.yaml
         └── moveit_controllers.yaml
```

### 关键规则

1. **YAML 传内容，不是路径**
```python
with open(file, 'r') as f:
    config = yaml.safe_load(f)  # 传字典
```

2. **OMPL 字段名**（Jazzy）：`planning_plugins`（复数）
```yaml
planning_plugins:
  - ompl_interface/OMPLPlanner
```

3. **加速度限制必须启用**
```yaml
has_acceleration_limits: true
max_acceleration: 2.0
```

4. **控制器映射**
```yaml
moveit_simple_controller_manager:
  controller_names: [left_arm_controller, right_arm_controller]
```

5. **禁用 Octomap**（无 3D 传感器）
```python
'octomap_frame': '', 'octomap_resolution': 0.0
```

### 碰撞几何

大 STL → 崩溃，用 box/cylinder 简化。

### 调试检查清单

```
move_group 崩溃：YAML 内容？OMPL 字段名？Octomap？
规划失败：加速度限制？控制器名称？
执行失败：控制器启动？轨迹冲突？
```

### 配置层级

```
1. move_group 启动 → kinematics.yaml + Octomap
2. OMPL 加载 → planning_plugins（复数）
3. 时间参数化 → has_acceleration_limits: true
4. 控制器映射 → controller_names 列表
```

四层都正确，Plan & Execute 才能成功。

---

## 硬件插件基础文件

### visibility_control.h

控制共享库（.so/.dll）的符号可见性，让 `pluginlib` 能在运行时加载插件。

```cpp
// Linux: 默认可见
#define DUAL_ARM_CONTROL_PUBLIC __attribute__((visibility("default")))

// Windows: 显式导出
#define DUAL_ARM_CONTROL_PUBLIC __declspec(dllexport)
```

所有 ros2_control 硬件插件的**标准模板文件**，不需要修改。

### 必要依赖（CMakeLists.txt + package.xml）

硬件插件至少需要：

```cmake
# CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_target_dependencies(my_plugin
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface hardware_plugin_description.xml)
```

```xml
<!-- package.xml -->
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
```

如需订阅 sensor_msgs（如 /joint_states），还需添加 `sensor_msgs` 依赖。
