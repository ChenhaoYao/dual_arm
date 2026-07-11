# VR Teleoperation Usage

## Build

```bash
cd /home/dell/dual_arm
colcon build --symlink-install --packages-select ros_tcp_endpoint vr_teleop_bridge dual_arm_bringup
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

If the bridge cannot configure Servo automatically, select Twist mode manually:

```bash
ros2 service call /servo_left/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
ros2 service call /servo_right/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

Unity/PICO should connect to the PC IP address on port `10000` and publish:

```text
/vr/left_hand/pose    geometry_msgs/msg/PoseStamped
/vr/right_hand/pose   geometry_msgs/msg/PoseStamped
/vr/left_hand/enabled std_msgs/msg/Bool
/vr/right_hand/enabled std_msgs/msg/Bool
/vr/status            std_msgs/msg/String
```

The bridge publishes MoveIt Servo commands:

```text
/servo_left/delta_twist_cmds    geometry_msgs/msg/TwistStamped
/servo_right/delta_twist_cmds   geometry_msgs/msg/TwistStamped
```

## Trajectory Logs

VR Servo launch starts a 5 Hz trajectory logger by default. Each run creates:

```text
/home/dell/dual_arm/vr_teleop_bridge/log/<timestamp>/vr_hand_trajectory.csv
/home/dell/dual_arm/vr_teleop_bridge/log/<timestamp>/robot_ee_trajectory.csv
```

The VR file contains the raw FLU controller poses received before Servo
processing. The robot file contains the measured end-effector poses computed
from `/joint_states` through TF. Join rows using `sample_ros_time_ns` and
`side`; `enabled=1` marks samples recorded while that hand's Grip is held.

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

`VRHandPublisher` uses `ROSGeometry.To<FLU>()` before publishing. The bridge
therefore uses an identity mapping and must not convert the axes a second time:

```text
ROS x = message x
ROS y = message y
ROS z = message z
```

The Unity client publishes `std_msgs/msg/Bool` deadman signals to:

```text
/vr/left_hand/enabled
/vr/right_hand/enabled
```

The bridge defaults to `require_enable_signal: true`. Hold the corresponding
controller grip button to control that arm; releasing it immediately resets the
command and requires a fresh pose reference before motion resumes.
