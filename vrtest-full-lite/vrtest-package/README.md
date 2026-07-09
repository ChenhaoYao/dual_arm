# VR Hand Publisher - ROS2 接收端

## 前提条件

- Ubuntu 22.04
- ROS2 Humble (`sudo apt install ros-humble-desktop`)
- Python3

## 使用方法

### 1. 解压
```bash
tar xzf vrtext-package.tar.gz
cd vrtext-package
```

### 2. 一键启动
```bash
bash start_ros.sh
```

### 3. 或者手动启动

**终端1 - 启动 TCP Endpoint（桥接）:**
```bash
cd ros2-tools/ros_tcp_endpoint
pip3 install -e .
source /opt/ros/humble/setup.bash
python3 -m ros_tcp_endpoint.default_server_endpoint --ros-args -p tcp_port:=10000
```

**终端2 - 启动 Subscriber（接收数据）:**
```bash
source /opt/ros/humble/setup.bash
python3 ros2-tools/vr_hand_subscriber.py
```

### 4. 查看数据（不用 subscriber 也行）
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /vr/left_hand/pose
ros2 topic echo /vr/right_hand/pose
ros2 topic echo /vr/status
```

## 网络要求

- 你的电脑和 Unity 端（PICO 头显）必须在**同一局域网**
- 确认能 ping 通对方 IP

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vr/left_hand/pose` | `geometry_msgs/PoseStamped` | 左手位姿 |
| `/vr/right_hand/pose` | `geometry_msgs/PoseStamped` | 右手位姿 |
| `/vr/status` | `std_msgs/String` | 设备连接状态 |
