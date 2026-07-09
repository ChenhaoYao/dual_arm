# VR Teleoperation Usage

## Build

```bash
cd /home/dell/dual_arm
colcon build --packages-select ros_tcp_endpoint vr_teleop_bridge dual_arm_bringup
source install/setup.bash
```

## Start Simulation Teleop

Start MoveIt Servo, the VR bridge, and the Unity TCP endpoint:

```bash
ros2 launch dual_arm_bringup sim.launch.py \
  mode:=servo \
  enable_vr_teleop:=true \
  enable_ros_tcp_endpoint:=true \
  ros_tcp_port:=10000
```

## Start Real-Hardware Teleop

Start MoveIt Servo with the real hardware interface, the VR bridge, and the Unity TCP endpoint:

```bash
ros2 launch dual_arm_bringup real.launch.py \
  mode:=servo \
  enable_vr_teleop:=true \
  enable_ros_tcp_endpoint:=true \
  ros_tcp_port:=10000
```

Start the SOEM bridge in another terminal:

```bash
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
```

If the bridge logs `Waiting for MoveIt Servo start services` after Servo has initialized, start Servo manually:

```bash
ros2 service call /servo_left/start_servo std_srvs/srv/Trigger "{}"
ros2 service call /servo_right/start_servo std_srvs/srv/Trigger "{}"
```

Unity/PICO should connect to the PC IP address on port `10000` and publish:

```text
/vr/left_hand/pose    geometry_msgs/msg/PoseStamped
/vr/right_hand/pose   geometry_msgs/msg/PoseStamped
/vr/status            std_msgs/msg/String
```

The bridge publishes MoveIt Servo commands:

```text
/servo_left/delta_twist_cmds    geometry_msgs/msg/TwistStamped
/servo_right/delta_twist_cmds   geometry_msgs/msg/TwistStamped
```

## Bridge Only

If `ros_tcp_endpoint` is already running:

```bash
ros2 launch vr_teleop_bridge vr_teleop_bridge.launch.py
```

If only MoveIt Servo is already running and you also want the Unity endpoint:

```bash
ros2 launch vr_teleop_bridge vr_with_tcp_endpoint.launch.py ros_tcp_port:=10000
```

## Parameters

Default parameters live in:

```text
vr_teleop_bridge/config/vr_teleop_bridge.yaml
```

Important tuning values:

```yaml
linear_scale: 0.5
angular_scale: 1.0
max_linear_speed: 0.15
max_angular_speed: 0.5
deadband_position: 0.005
deadband_rotation: 0.02
command_timeout: 0.2
```

The default Unity to ROS axis mapping is:

```text
ROS x = Unity z
ROS y = -Unity x
ROS z = Unity y
```

For future controller-button gating, publish `std_msgs/msg/Bool` to:

```text
/vr/left_hand/enabled
/vr/right_hand/enabled
```

Then set `require_enable_signal: true` in the bridge YAML.
