# ROS2 Spin 循环与回调机制

## spin 循环本质

```cpp
auto node = std::make_shared<SoemBridgeNode>();  // 构造函数执行一次
rclcpp::spin(node);                               // 进入事件循环
```

- **构造函数**：只执行一次，负责注册回调（订阅者、定时器、服务）
- **spin 循环**：不断执行，负责触发已注册的回调

## 回调类型

| 类型 | 创建函数 | 触发方式 |
|------|----------|----------|
| Subscription | `create_subscription()` | ✅ 自动（收到消息时） |
| Publisher | `create_publisher()` | ❌ 手动调用 `publish()` |
| Timer | `create_wall_timer()` | ✅ 自动（定时触发） |
| Service | `create_service()` | ✅ 自动（收到请求时） |

## Publisher vs Subscription

```
Subscription（被动）：创建后自动订阅，有消息来就调用回调
    话题消息到达 → ROS2 自动调用回调

Publisher（主动）：创建后不会自动发布，需要手动调用 publish()
    需要定时器或其他触发机制来调用 publish()
```

## QoS（Quality of Service）

```cpp
rclcpp::QoS(10)  // 参数是队列大小，不是频率
```

| 参数 | 含义 |
|------|------|
| `rclcpp::QoS(10)` | 最多缓存 10 条消息，旧的会被丢弃 |

## 事件队列机制

单线程 spin 使用**事件队列 + 顺序执行**，不会同时处理两个回调。

```
事件到达 → 放入队列（不立即执行）
                │
                ▼
        ┌─────────────┐
        │  事件队列    │
        │  (先进先出)  │
        └──────┬──────┘
               │
               ▼
        spin 循环取出一个
               │
               ▼
        执行回调（不会被打断）
               │
               ▼
        执行完毕，取下一个
```

## 并发处理

| 场景 | 单线程 spin | 多线程 spin |
|------|-------------|-------------|
| 两个话题同时到 | 顺序执行 | 并行执行 |
| 发布时收到话题 | 当前回调执行完再处理 | 可能在不同线程并行 |
| 回调执行太久 | 后面事件延迟 | 其他线程不受影响 |

```cpp
// 单线程（默认）
rclcpp::spin(node);

// 多线程
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

## 代码示例

```cpp
// 订阅者：自动订阅，收到消息自动调用回调
waypoint_sub_ = create_subscription<JointTrajectory>(
    "/dual_arm/csv_waypoints",   // 参数 1：话题名
    rclcpp::QoS(10),             // 参数 2：QoS
    [this](JointTrajectory::SharedPtr msg) {  // 参数 3：回调
        on_waypoints(msg);
    }
);

// 发布者：只创建能力，不会自动发布
real_joint_state_pub_ = create_publisher<JointState>(
    "/joint_states",    // 参数 1：话题名
    rclcpp::QoS(10)     // 参数 2：QoS
);

// 定时器：周期性触发发布
feedback_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),      // 参数 1：周期
    [this] { publish_feedback(); }      // 参数 2：回调
);

// 服务：收到请求时调用回调
clear_fault_srv_ = create_service<Trigger>(
    "~/clear_fault",                     // 参数 1：服务名
    [this](auto req, auto res) {         // 参数 2：回调
        handle_clear_fault(req, res);
    }
);
```

## 创建函数对比

| 函数 | 模板参数 | 参数 1 | 参数 2 | 参数 3 |
|------|----------|--------|--------|--------|
| `create_subscription` | 消息类型 | 话题名 | QoS | 回调 |
| `create_publisher` | 消息类型 | 话题名 | QoS | - |
| `create_wall_timer` | - | 周期（时间对象，不是数字） | 回调 | - |
| `create_service` | 服务类型 | 服务名 | 回调 | - |

## ROS2 参数系统

### 参数加载流程

```
soem_bridge.yaml               launch 文件                  节点代码
┌──────────────┐    parameters=[config_file]    ┌───────────────────────────┐
│ gear_ratio:  │ ──────────────────────────────→ │ 参数服务器                 │
│   [100, 50]  │    YAML 被解析并注入参数服务器    │ gear_ratio = [100.0, 50.0]│
│ ifname:      │                                 │ ifname = "enp0s31f6"      │
│   "enp0s31f6"│                                 └────────────┬──────────────┘
└──────────────┘                                              │
                                              declare_parameter("gear_ratio", default)
                                                              │
                                                              ▼
                                                     返回 [100.0, 50.0]
                                                     （参数服务器里有值，default 被忽略）
```

### declare_parameter

声明并获取参数。如果参数服务器里已有值（从 YAML 加载的），返回它；否则返回默认值。

```cpp
// 基本用法
auto val = declare_parameter<T>(参数名, 默认值);

// 具体例子
auto ifname = declare_parameter<std::string>("ifname", "enp0s31f6");
auto gear_ratios = declare_parameter<std::vector<double>>("gear_ratio", std::vector<double>(7, 100.0));
auto enc_bits = declare_parameter<int>("enc_bits", 19);
```

### YAML 与参数的对应关系

```yaml
# soem_bridge.yaml
soem_bridge_node:
  ros__parameters:
    ifname: "enp0s31f6"
    gear_ratio: [100.0, 50.0, 80.0]
```

```cpp
// 节点代码中读取
auto ifname = declare_parameter<std::string>("ifname", "");
// ifname = "enp0s31f6"  ← 来自 YAML

auto ratios = declare_parameter<std::vector<double>>("gear_ratio", std::vector<double>(7, 100.0));
// ratios = [100.0, 50.0, 80.0]  ← 来自 YAML，不是 7 个默认值
```

### 参数数组长度不匹配的处理

当 YAML 中数组长度小于期望的轴数时，需要手动补齐：

```cpp
auto vec = declare_parameter<std::vector<double>>("gear_ratio", std::vector<double>(n, 100.0));
// YAML: gear_ratio: [100.0, 50.0]  →  vec = [100.0, 50.0]  (size=2, 不是 n)

if (vec.size() < n) vec.resize(n, 100.0);  // 补齐到 n 个
// vec = [100.0, 50.0, 100.0, 100.0, ...]  (size=n)
// 原来的值不变，后面用 100.0 填充
```

### Launch 文件中加载 YAML

```python
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    get_package_share_directory('dual_arm_soem_bridge'),  # 找到包的安装路径
    'config', 'soem_bridge.yaml'                          # 拼接出 yaml 文件路径
)
# config_file 只是路径字符串，还没读文件

Node(
    package='dual_arm_soem_bridge',
    executable='soem_bridge_node',
    parameters=[config_file],   # ← 这一步才读取 YAML 并注入参数服务器
)
```

### get_package_share_directory

返回包的 share 目录路径（安装后的位置）：

```python
get_package_share_directory('dual_arm_soem_bridge')
# → "/home/dell/dual_arm/install/dual_arm_soem_bridge/share/dual_arm_soem_bridge"
```

## ROS2 日志宏

用于输出带级别、时间戳、节点名的日志消息。

```cpp
RCLCPP_DEBUG(logger, "调试信息");      // 默认不显示
RCLCPP_INFO(logger, "一般信息");       // 正常运行信息
RCLCPP_WARN(logger, "警告信息");       // 潜在问题
RCLCPP_ERROR(logger, "错误信息");      // 出错
RCLCPP_FATAL(logger, "致命错误");      // 无法继续
```

### 用法

```cpp
// 在节点类内
RCLCPP_INFO(this->get_logger(), "EtherCAT 已启动");

// 支持 printf 格式化
RCLCPP_INFO(this->get_logger(), "共 %d 个轴, 接口: %s", n, ifname_.c_str());

// 使用外部 logger
rclcpp::Logger logger = rclcpp::get_logger("soem_bridge");
RCLCPP_ERROR(logger, "配置失败");
```

### 输出效果

```
[INFO] [1686123456.789] [soem_bridge_node]: EtherCAT 已启动
 ^^^^   ^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^
 级别    时间戳(秒)         节点名               消息内容
```

日志同时输出到终端和 `/rosout` 话题。
