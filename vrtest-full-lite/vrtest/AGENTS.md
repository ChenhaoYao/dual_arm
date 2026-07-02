# AGENTS.md

Unity 2022.3 VR project for PICO headset + ROS 2 Humble. Publishes hand controller poses to ROS over TCP.

## Quick Facts

- **Unity version**: 2022.3.62f3c1 (China locale)
- **ROS distro**: Humble
- **Target device**: PICO (via `com.unity.xr.picoxr` local package)
- **Only custom script**: `Assets/VRHandPublisher.cs` (98 lines)
- **No tests, no CI, no lint, no build scripts**

## ROS Integration

### Architecture

PICO headset -> `VRHandPublisher.cs` -> ROS TCP Connector (TCP:10000) -> `ros_tcp_endpoint` (bridge) -> ROS 2 topics

### Key Files

- `Assets/VRHandPublisher.cs` — publishes `PoseStamped` on `/vr/left_hand/pose` and `/vr/right_hand/pose` at 50 Hz, plus `String` status on `/vr/status`
- `Assets/Resources/ROSConnectionPrefab.prefab` — connection config (IP `10.164.89.210`, port `10000`). Must be in scene for ROS to work.

### Running on Device

1. Install `ros_tcp_endpoint` into `~/ros2_ws/src/` from `main-ros2` branch of [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint), then `colcon build`
2. Start endpoint: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000`
3. Start subscriber: `python3 /home/yao/ROS-TCP-Connector-main/scripts/vr_hand_subscriber.py`
4. Build APK via Unity Editor (File > Build Settings > Android > Build and Run)
5. ADB path: `/home/yao/Unity/Hub/Editor/2022.3.62f3c1/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb`

### Gotchas

- **`ROSConnectionPrefab` must be in the scene** and `VRHandPublisher` must be attached to a GameObject. Without both, the publisher silently does nothing.
- **IP in prefab must match the PC's LAN IP** (`10.164.89.210`). It's not `127.0.0.1` — the headset is a separate device on the network.
- **`BuiltinInterfaces` namespace**: use `new TimeMsg` not `new BuiltinInterfaces.TimeMsg` — the `using` directive already imports it.
- **PICO SDK path is local**: `com.unity.xr.picoxr` resolves from `file:/home/yao/...`. Project won't build on other machines without this path.
- **Build scene is `DemoScene.unity`** (XRI Starter Assets sample), not a custom scene. Custom scenes in `Assets/Scenes/` are not in the build list.
- **Scripting defines**: Android build defines `ROS2`. This enables ROS2 code paths in the connector (e.g., `TimeMsg.sec` is `int` not `uint`).
- **`activeInputHandler: 2`** — New Input System only. Legacy input is disabled.
