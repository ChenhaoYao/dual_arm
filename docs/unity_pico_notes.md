# Unity / PICO / ROS TCP 笔记

本文整理当前 VR 遥操作 Unity 项目相关的环境、依赖、配置文件和排错知识。

Unity 项目路径：

```text
/home/dell/dual_arm/vrtest-full-lite/vrtest
```

## 组件关系

### Unity Hub

Unity Hub 只是 Unity 的管理器，负责：

- 管理 Unity Editor 版本
- 管理 Android 等 Editor 模块
- 管理许可证登录
- 从图形界面打开项目

Unity Hub 本身不编译 VR 程序。通过 Hub 打开项目时，真正启动的是对应版本的 Unity Editor。

### Unity Editor

当前项目使用的 Unity Editor 版本是：

```text
2022.3.62f3
```

版本记录在：

```text
vrtest-full-lite/vrtest/ProjectSettings/ProjectVersion.txt
```

Unity Editor 才是实际工作的程序，负责：

- 打开项目
- 导入 Unity package
- 编译 C# 脚本
- 加载 scene 和 prefab
- 构建 Android APK
- 在需要时把 APK 安装/运行到 PICO 设备

### Android Build Support

Android Build Support 是 Unity Editor 的附加模块。要给 PICO 构建 APK，必须安装这个模块。

它主要包含：

- Android Player
- Android SDK
- Android NDK
- OpenJDK

如果缺少 Android Build Support，Unity 仍然可以打开项目，但不能正常构建或部署 PICO APK。

### OpenJDK

OpenJDK 是 Android/Gradle 构建链的一部分。Unity 打包 APK 时会用到它。

它不是 PICO 专属依赖，也不是 Unity C# 脚本直接调用的库。它只在 Android 构建阶段起作用。

### PICO Unity Integration SDK

PICO SDK 是一个 Unity package。当前项目通过本地路径加载它：

```text
/home/dell/dual_arm/third_party/PICO Unity Integration SDK-3.4.0-20260226
```

项目中的声明是：

```json
"com.unity.xr.picoxr": "file:/home/dell/dual_arm/third_party/PICO Unity Integration SDK-3.4.0-20260226"
```

这个声明位于：

```text
vrtest-full-lite/vrtest/Packages/manifest.json
```

Unity Hub 不会自动安装这个 SDK。Unity Editor 打开项目后，会读取 `manifest.json`，然后从配置的本地路径加载 PICO SDK。

### ROS-TCP-Connector

ROS-TCP-Connector 是 Unity 侧与 ROS 2 通信的 Unity package。

当前项目通过 GitHub package 引入：

```json
"com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector"
```

它提供 Unity 侧的 `ROSConnection`，并负责把 Unity 中的数据发布到 ROS。

### ros_tcp_endpoint

`ros_tcp_endpoint` 是 ROS 2 侧的 TCP 通信端。

它运行在当前主机上，监听端口，等待 PICO/Unity 应用主动连接：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py tcp_ip:=0.0.0.0 tcp_port:=10000
```

`endpoint` 的意思是“通信端点”。这里它表示 TCP 连接在 ROS 2 主机侧的那一端，另一端是 PICO 上运行的 Unity 应用。

## 依赖链

实际依赖关系可以理解为：

```text
Unity Hub
  启动和管理
Unity Editor 2022.3.62f3
  使用已安装模块
Android Build Support / SDK / NDK / OpenJDK
  构建 Android APK
PICO Unity Integration SDK
  提供 PICO XR 运行时支持
ROS-TCP-Connector
  提供 Unity 到 ROS 的 TCP 通信
ros_tcp_endpoint
  在 ROS 2 侧接收 Unity 消息
```

开发和运行链路是：

```text
Unity 项目 -> 构建 APK -> 安装到 PICO -> PICO 应用连接当前主机 -> ROS 2 收到 VR 位姿话题
```

## 重要项目文件

### 包声明文件

```text
vrtest-full-lite/vrtest/Packages/manifest.json
```

这是 Unity 项目的依赖声明文件，通常可以手动修改。它说明项目直接需要哪些 package。

当前项目中的典型依赖包括：

- PICO SDK 本地 package
- ROS-TCP-Connector Git package
- OpenXR
- TextMeshPro
- Timeline
- UGUI

如果要更换 PICO SDK 路径、升级 package、删除 package，通常改的是这个文件。

### 包锁定文件

```text
vrtest-full-lite/vrtest/Packages/packages-lock.json
```

这是 Unity Package Manager 自动生成的锁定文件。它记录 Unity 实际解析出来的完整依赖树，包括：

- 每个包最终使用的版本
- 包来源：`registry`、`git`、`local`、`builtin`
- 间接依赖
- Git package 的 commit hash
- 本地 package 的路径

通常不要手动改这个文件。更改 `manifest.json` 后，让 Unity 自动更新 `packages-lock.json`。

`manifest.json` 和 `packages-lock.json` 都应该纳入版本管理。前者保证项目知道需要哪些包，后者保证不同机器尽量解析到同一批版本。

### Unity 版本文件

```text
vrtest-full-lite/vrtest/ProjectSettings/ProjectVersion.txt
```

这个文件记录项目期望使用的 Unity Editor 版本。

项目原来使用的是中国特供版本：

```text
2022.3.62f3c1
```

现在已经改为当前机器安装的全球 LTS 版本：

```text
2022.3.62f3
```

### ROS 连接 prefab

```text
vrtest-full-lite/vrtest/Assets/Resources/ROSConnectionPrefab.prefab
```

`ROSConnection.GetOrCreateInstance()` 没有显式写配置文件路径，但它仍然能找到这个 prefab，是因为 Unity 会通过 `Assets/Resources/` 目录的 Resources 机制自动加载资源。

这个 prefab 保存 ROS TCP 连接配置，包括：

- 当前主机 IP 地址
- TCP 端口，目前是 `10000`

PICO 应用会连接这里配置的主机 IP。如果主机 IP 变了，PICO 应用仍然会尝试连接旧 IP，直到修改 prefab 并重新构建/安装 APK。

### 直接读取 VR 硬件的代码

```text
vrtest-full-lite/vrtest/Assets/VRHandPublisher.cs
```

这个脚本是 PICO 手柄追踪数据进入 Unity/ROS 的直接入口。

它通过 Unity XR 输入 API 读取左右手柄位姿：

```csharp
InputDevices.GetDeviceAtXRNode(XRNode.LeftHand)
InputDevices.GetDeviceAtXRNode(XRNode.RightHand)
CommonUsages.devicePosition
CommonUsages.deviceRotation
```

它发布的话题是：

```text
/vr/left_hand/pose
/vr/right_hand/pose
```

消息类型是：

```text
geometry_msgs/msg/PoseStamped
```

## 排错记录：PICO 无法收到/发布 ROS 手柄位姿

日期：2026-07-05

### 现象

PICO 中已经启动 `vrtest`，但 ROS 2 侧最初只能看到基础话题：

```text
/parameter_events
/rosout
```

看不到：

```text
/vr/status
/vr/left_hand/pose
/vr/right_hand/pose
```

### 初步检查

确认 PICO 上运行的是新版包：

```text
com.dualarm.vrtest
```

确认电脑端 `ros_tcp_endpoint` 需要启动并监听 `10000`：

```bash
source /opt/ros/jazzy/setup.bash
source /home/dell/dual_arm/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000
```

如果需要让它在后台保持运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/dell/dual_arm/install/setup.bash
setsid -f ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000 >/tmp/ros_tcp_endpoint.log 2>&1
```

检查监听：

```bash
ss -ltnp | rg ':10000'
```

正常应看到类似：

```text
LISTEN 0 10 0.0.0.0:10000
```

### 根因

Unity 日志中出现：

```text
Connection to 10.164.118.157:10000 failed - System.Net.Sockets.SocketException: No route to host
```

当时网络地址是：

```text
PC:   10.164.118.157
PICO: 10.164.72.175
```

虽然两者都在 `10.164.0.0/17` 网段内，但双向 `ping` 都失败：

```bash
adb shell ping -c 3 -W 2 10.164.118.157
ping -c 3 -W 2 10.164.72.175
```

结论：当前 Wi-Fi 网络阻止 PICO 和 PC 客户端互通。问题不是 Unity 脚本、ROS 消息类型或手柄追踪，而是 PICO 无法通过 Wi-Fi 路由到电脑的 ROS TCP endpoint。

### 修复方案

使用 USB ADB reverse 绕过 Wi-Fi 客户端隔离：

```bash
adb reverse tcp:10000 tcp:10000
adb reverse --list
```

正常应看到：

```text
UsbFfs tcp:10000 tcp:10000
```

然后把 Unity 连接配置改为 PICO 本机回环地址：

```text
vrtest-full-lite/vrtest/Assets/Resources/ROSConnectionPrefab.prefab
```

关键字段：

```yaml
m_RosIPAddress: 127.0.0.1
m_RosPort: 10000
```

注意：prefab 改动不会影响已经安装在 PICO 上的旧 APK。必须重新 Unity `Build And Run`，让新 APK 内置 `127.0.0.1:10000` 配置。

### 验证结果

重新 Build And Run 后，ROS 2 侧出现：

```text
/tf [tf2_msgs/msg/TFMessage]
/vr/left_hand/pose [geometry_msgs/msg/PoseStamped]
/vr/right_hand/pose [geometry_msgs/msg/PoseStamped]
/vr/status [std_msgs/msg/String]
```

状态消息：

```bash
ros2 topic echo /vr/status --once
```

结果：

```text
data: left=connected right=connected
```

左右手位姿可正常收到：

```bash
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/right_hand/pose --once
```

发布频率约为 `45 Hz`：

```bash
ros2 topic hz /vr/left_hand/pose
ros2 topic hz /vr/right_hand/pose
```

### 当前推荐运行顺序

1. USB 连接 PICO，并确认 ADB 可见：

```bash
adb devices -l
```

2. 建立 ADB reverse：

```bash
adb reverse tcp:10000 tcp:10000
```

3. 启动 ROS TCP endpoint：

```bash
source /opt/ros/jazzy/setup.bash
source /home/dell/dual_arm/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000
```

4. 在 PICO 中启动 `vrtest`。

5. 检查 ROS 话题：

```bash
ros2 topic list -t
ros2 topic echo /vr/status --once
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/right_hand/pose --once
```

## `.asset` 和 `.prefab`

`.asset` 和 `.prefab` 是 Unity 的资源文件格式，不是 C# 语言特性。

### `.prefab`

`prefab` 是可复用的 GameObject 层级结构。

当前项目中的 `ROSConnectionPrefab.prefab` 保存了一个可复用的 `ROSConnection` 对象，以及它的 IP/端口配置。

### `.asset`

`.asset` 文件用于保存 Unity 序列化数据，常见用途是设置项或 ScriptableObject。

例如 PICO 项目设置、Unity package 设置等都可能以 `.asset` 文件保存。

这些文件通常是 Unity 序列化出来的 YAML 风格文本。如果它们属于项目配置，一般应该纳入版本管理。

## Unity Hub / Editor 正常弹窗

Unity 打开项目时弹窗数量不是固定的，取决于缓存、许可证、package 导入状态和项目是否有编译错误。

首次打开或配置变化时，可能出现的正常提示包括：

- Unity 版本不完全一致提示
- package resolving/importing 进度提示
- Input System 设置变化后要求重启 Editor
- XR/PICO 项目校验提示
- Android SDK/NDK/JDK 缺失提示

这些提示不一定每次都会出现。

表示异常的提示包括：

- 已经安装 `2022.3.62f3` 后仍反复提示缺少该版本
- 反复进入 Safe Mode
- 安装 Android Build Support 后仍反复提示 SDK/NDK/JDK 缺失
- 反复出现 PICO SDK package 错误
- Console 中存在红色 C# 编译错误

对当前机器来说，健康状态应该是：

```text
Hub 打开项目 -> Editor 导入/解析 package -> Console 没有红色编译错误
```

正常情况下，不应该每次打开项目都重新安装依赖。

## PICO 应用生命周期

PICO 设备每次断电或重启后，不需要每次都通过 USB 连接主机，也不需要每次都在 Unity Hub 中重新启动项目。

USB 主要在这些情况需要：

- 第一次安装 APK
- 修改 Unity 代码、scene、prefab 后重新安装
- 使用 `adb logcat` 查看 PICO 端日志
- 排查安装或运行时问题

日常使用流程应该是：

1. 在当前主机启动 ROS TCP endpoint。
2. 在 PICO 设备里打开已经安装好的 VR 应用。
3. 在 ROS 2 中确认收到 VR 位姿话题。

PICO 应用是主动连接当前主机的。当前主机通常不需要知道 PICO 的 IP，只需要监听对应端口；PICO 应用需要知道当前主机的 IP。

## TCP 与 UDP

当前项目使用 TCP，通信链路是 ROS-TCP-Connector 加 `ros_tcp_endpoint`。

TCP 适合当前项目的原因：

- 和 Unity Robotics 官方包匹配
- 保证消息顺序
- 丢包后有重传机制
- 当前实现已经完整可用

UDP 在某些高频流式系统中可能有更低延迟，但需要自己处理丢包、乱序、抖动和自定义协议。对当前项目来说，除非实测 TCP 延迟成为瓶颈，否则继续使用 TCP 更实际。

## 预期 ROS 话题

当 PICO 应用成功连接并发布数据时，ROS 2 中应该能看到：

```text
/vr/left_hand/pose    geometry_msgs/msg/PoseStamped
/vr/right_hand/pose   geometry_msgs/msg/PoseStamped
```

检查命令：

```bash
source /home/dell/dual_arm/install/setup.bash
ros2 topic list -t
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/right_hand/pose --once
```

如果话题短暂出现后又消失，通常说明 Unity/PICO 应用曾经连接成功，但随后退出、崩溃、断网，或者停止发布数据。

这时建议通过 USB 连接 PICO，查看日志：

```bash
adb logcat | grep -i -E "ros|ROSConnection|VRHandPublisher|Exception|error"
```

## 本地生成文件

Unity 会生成很多本地文件，这些文件不应该提交到 git：

- `Library/`
- `Temp/`
- `Logs/`
- `UserSettings/`
- `*.csproj`
- `*.sln`
- `*.apk`
- `*.aab`

Unity 项目自己的忽略规则在：

```text
vrtest-full-lite/vrtest/.gitignore
```

Unity 项目中最重要、应该纳入版本管理的目录是：

```text
Assets/
Packages/
ProjectSettings/
```

## Linux 大小写敏感问题

Linux 文件系统区分大小写。

PICO SDK 之前出现过这两个路径冲突：

```text
Runtime/Windows
Runtime/windows
```

其中小写的 `Runtime/windows` 是空目录，已经被移走/重命名，否则 Unity 会报“两个文件只在大小写上不同”的错误。

如果以后再次出现类似错误，优先检查 PICO SDK 包目录：

```text
/home/dell/dual_arm/third_party/PICO Unity Integration SDK-3.4.0-20260226
```

## 构建和安装

PICO APK 的构建和安装通过 Unity Editor 完成。常规路径是：

```text
Unity Editor -> Build Settings -> Android -> Build 或 Build And Run
```

这些修改之后需要重新构建/安装 APK：

- Unity C# 脚本
- scene
- prefab
- PICO SDK 设置
- `ROSConnectionPrefab.prefab` 中的 ROS 连接 IP/端口

仅仅是 PICO 设备断电或重启，不需要重新构建或安装 APK。
