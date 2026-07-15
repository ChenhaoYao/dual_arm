#!/usr/bin/env python3
"""Convert Unity/PICO VR controller poses to MoveIt Servo TwistStamped commands."""

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _normalize_quat(quat: Quat) -> Quat:
    x, y, z, w = quat
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / length, y / length, z / length, w / length


def _quat_inverse(quat: Quat) -> Quat:
    x, y, z, w = _normalize_quat(quat)
    return -x, -y, -z, w


def _quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_to_rotvec(quat: Quat) -> Vector3:
    x, y, z, w = _normalize_quat(quat)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm <= 1e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, w)))
    scale = angle / vector_norm
    return x * scale, y * scale, z * scale


@dataclass
class ArmState:
    name: str
    enabled_param: bool
    enabled_signal: bool = False
    pose_history: Deque[Tuple[float, PoseStamped]] = field(default_factory=deque)
    linear_motion_active: bool = False
    angular_motion_active: bool = False
    latest_twist: TwistStamped = field(default_factory=TwistStamped)
    latest_pose_receive_time: Optional[float] = None
    published_zero: bool = True
    servo_active: bool = False
    activation_stage: str = "idle"
    command_future: Optional[object] = None
    unpause_future: Optional[object] = None
    pause_future: Optional[object] = None
    pause_confirmed: bool = False
    startup_pause_confirmed_time: Optional[float] = None
    last_wait_log_time: float = 0.0


class VrPoseToServo(Node):
    def __init__(self):
        super().__init__("vr_pose_to_servo_node")

        self._declare_parameters()
        self.control_frame = self.get_parameter("control_frame").value
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.velocity_window = float(self.get_parameter("velocity_window").value)
        self.linear_velocity_start = float(
            self.get_parameter("linear_velocity_start").value
        )
        self.linear_velocity_stop = float(
            self.get_parameter("linear_velocity_stop").value
        )
        self.angular_velocity_start = float(
            self.get_parameter("angular_velocity_start").value
        )
        self.angular_velocity_stop = float(
            self.get_parameter("angular_velocity_stop").value
        )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.require_enable_signal = bool(self.get_parameter("require_enable_signal").value)
        self.auto_start_servo = bool(self.get_parameter("auto_start_servo").value)
        self.servo_startup_settle_time = float(
            self.get_parameter("servo_startup_settle_time").value
        )
        self.position_axis_map = list(
            self.get_parameter("unity_to_ros_axis_map").value
        )
        self.position_axis_sign = [
            float(value)
            for value in self.get_parameter("unity_to_ros_axis_sign").value
        ]
        self.rotation_axis_map = list(
            self.get_parameter("unity_to_ros_rotation_axis_map").value
        )
        self.rotation_axis_sign = [
            float(value)
            for value in self.get_parameter("unity_to_ros_rotation_axis_sign").value
        ]

        self._validate_axis_mapping()
        self._validate_velocity_filter()

        self.left = ArmState("left", bool(self.get_parameter("left_enabled").value))
        self.right = ArmState("right", bool(self.get_parameter("right_enabled").value))
        self.arms = (self.left, self.right)
        self.required_joint_names = {
            *(f"laxis{index}_joint" for index in range(1, 8)),
            *(f"raxis{index}_joint" for index in range(1, 8)),
        }
        self.last_joint_state_time: Optional[float] = None
        self.robot_state_ready_since: Optional[float] = None

        self.pub_left = self.create_publisher(
            TwistStamped, self.get_parameter("left_twist_topic").value, 10
        )
        self.pub_right = self.create_publisher(
            TwistStamped, self.get_parameter("right_twist_topic").value, 10
        )

        self.create_subscription(
            PoseStamped,
            self.get_parameter("left_pose_topic").value,
            lambda msg: self._on_pose(self.left, msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("right_pose_topic").value,
            lambda msg: self._on_pose(self.right, msg),
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("left_enable_topic").value,
            lambda msg: self._on_enable(self.left, msg),
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("right_enable_topic").value,
            lambda msg: self._on_enable(self.right, msg),
            10,
        )
        self.create_subscription(String, self.get_parameter("status_topic").value, self._on_status, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self._publish_commands)

        self._command_type_clients = {}
        self._pause_clients = {}
        if self.auto_start_servo:
            for arm in self.arms:
                self._command_type_clients[arm.name] = self.create_client(
                    ServoCommandType, f"/servo_{arm.name}/switch_command_type"
                )
                self._pause_clients[arm.name] = self.create_client(
                    SetBool, f"/servo_{arm.name}/pause_servo"
                )
            self._start_timer = self.create_timer(0.1, self._try_start_servo)
        else:
            for arm in self.arms:
                arm.servo_active = True

        self.get_logger().info(
            "VR teleop bridge ready: /vr/*/pose -> /servo_left/right/delta_twist_cmds"
        )

    def _declare_parameters(self):
        self.declare_parameter("control_frame", "base_link")
        self.declare_parameter("left_pose_topic", "/vr/left_hand/pose")
        self.declare_parameter("right_pose_topic", "/vr/right_hand/pose")
        self.declare_parameter("left_enable_topic", "/vr/left_hand/enabled")
        self.declare_parameter("right_enable_topic", "/vr/right_hand/enabled")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("left_twist_topic", "/servo_left/delta_twist_cmds")
        self.declare_parameter("right_twist_topic", "/servo_right/delta_twist_cmds")
        self.declare_parameter("left_enabled", True)
        self.declare_parameter("right_enabled", True)
        self.declare_parameter("require_enable_signal", True)
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("velocity_window", 0.1)
        self.declare_parameter("linear_velocity_start", 0.02)
        self.declare_parameter("linear_velocity_stop", 0.01)
        self.declare_parameter("angular_velocity_start", 0.05)
        self.declare_parameter("angular_velocity_stop", 0.025)
        self.declare_parameter("command_timeout", 0.2)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("auto_start_servo", True)
        self.declare_parameter("servo_startup_settle_time", 1.0)
        # VR FLU is [forward, left, up], while base_link is
        # [left, backward, up]. Apply the same proper rotation to linear and
        # angular vectors, while retaining separate parameters for calibration.
        self.declare_parameter("unity_to_ros_axis_map", [1, 0, 2])
        self.declare_parameter("unity_to_ros_axis_sign", [1.0, -1.0, 1.0])
        self.declare_parameter("unity_to_ros_rotation_axis_map", [1, 0, 2])
        self.declare_parameter(
            "unity_to_ros_rotation_axis_sign", [1.0, -1.0, 1.0]
        )

    def _validate_axis_mapping(self):
        mappings = (
            (
                "unity_to_ros_axis_map",
                self.position_axis_map,
                self.position_axis_sign,
            ),
            (
                "unity_to_ros_rotation_axis_map",
                self.rotation_axis_map,
                self.rotation_axis_sign,
            ),
        )
        for name, axis_map, axis_sign in mappings:
            if sorted(axis_map) != [0, 1, 2] or len(axis_sign) != 3:
                raise ValueError(f"{name} must be a permutation of [0, 1, 2]")

    def _validate_velocity_filter(self):
        if self.velocity_window <= 0.0:
            raise ValueError("velocity_window must be greater than zero")
        if self.velocity_window >= self.command_timeout:
            raise ValueError("velocity_window must be shorter than command_timeout")
        thresholds = (
            (
                "linear",
                self.linear_velocity_start,
                self.linear_velocity_stop,
            ),
            (
                "angular",
                self.angular_velocity_start,
                self.angular_velocity_stop,
            ),
        )
        for name, start, stop in thresholds:
            if stop < 0.0 or start <= stop:
                raise ValueError(
                    f"{name}_velocity thresholds must satisfy 0 <= stop < start"
                )

    def _try_start_servo(self):
        for arm in self.arms:
            self._advance_servo_activation(arm)

    def _advance_servo_activation(self, arm: ArmState):
        """
        One-time Servo activation sequence:
        - Start each enabled Servo independently of the Grip signal.
        - Require a complete, fresh joint state before contacting Servo.
        - Confirm TWIST mode before sending the unpause request.
        - Keep Servo active afterward; Grip only gates motion commands.
        """
        if arm.pause_future is not None:
            if not arm.pause_future.done():
                return
            try:
                response = arm.pause_future.result()
            except Exception as exc:
                self.get_logger().warn(f"{arm.name} Servo pause failed: {exc}")
                arm.pause_future = None
                return
            arm.pause_future = None
            if not response.success:
                self.get_logger().warn(
                    f"{arm.name} Servo pause rejected: {response.message}"
                )
                return
            arm.pause_confirmed = True
            arm.startup_pause_confirmed_time = time.monotonic()
            self.get_logger().info(f"{arm.name} Servo startup pause confirmed")

        if arm.servo_active:
            if not self._robot_state_ready():
                self._log_activation_wait(arm, "complete, fresh /joint_states")
                self._deactivate_arm(arm, "joint states became stale")
            return

        if arm.activation_stage == "idle" and not arm.pause_confirmed:
            client = self._pause_clients[arm.name]
            if not client.service_is_ready():
                if self._servo_activation_requested(arm):
                    self._log_activation_wait(arm, "MoveIt Servo pause service")
                return
            request = SetBool.Request()
            request.data = True
            arm.pause_future = client.call_async(request)
            return

        if not self._servo_activation_requested(arm):
            return

        if not self._robot_state_ready():
            self._log_activation_wait(arm, "complete, fresh /joint_states")
            if arm.servo_active:
                self._deactivate_arm(arm, "joint states became stale")
            return

        if (
            arm.startup_pause_confirmed_time is None
            or time.monotonic() - arm.startup_pause_confirmed_time
            < self.servo_startup_settle_time
        ):
            self._log_activation_wait(arm, "Servo CurrentStateMonitor startup")
            return

        command_client = self._command_type_clients[arm.name]
        pause_client = self._pause_clients[arm.name]
        if not command_client.service_is_ready() or not pause_client.service_is_ready():
            self._log_activation_wait(arm, "MoveIt Servo services")
            return

        if arm.activation_stage == "idle":
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.TWIST
            arm.command_future = command_client.call_async(request)
            arm.activation_stage = "switching"
            return

        if arm.activation_stage == "switching":
            if arm.command_future is None or not arm.command_future.done():
                return
            try:
                response = arm.command_future.result()
            except Exception as exc:
                self._activation_failed(arm, f"TWIST mode request failed: {exc}")
                return
            if not response.success:
                self._activation_failed(arm, "Servo rejected TWIST mode")
                return
            request = SetBool.Request()
            request.data = False
            arm.unpause_future = pause_client.call_async(request)
            arm.pause_confirmed = False
            arm.activation_stage = "unpausing"
            return

        if arm.activation_stage == "unpausing":
            if arm.unpause_future is None or not arm.unpause_future.done():
                return
            try:
                response = arm.unpause_future.result()
            except Exception as exc:
                self._activation_failed(arm, f"unpause request failed: {exc}")
                return
            if not response.success:
                self._activation_failed(arm, f"Servo stayed paused: {response.message}")
                return
            arm.servo_active = True
            arm.activation_stage = "active"
            self._reset_arm(arm)
            self.get_logger().info(
                f"{arm.name} Servo active; Grip now gates motion commands"
            )

    def _activation_failed(self, arm: ArmState, reason: str):
        self.get_logger().warn(f"{arm.name} Servo activation failed: {reason}")
        arm.activation_stage = "idle"
        arm.command_future = None
        arm.unpause_future = None

    def _log_activation_wait(self, arm: ArmState, resource: str):
        now = time.monotonic()
        if now - arm.last_wait_log_time >= 5.0:
            arm.last_wait_log_time = now
            self.get_logger().info(f"{arm.name} Servo waiting for {resource}")

    def _on_joint_state(self, msg: JointState):
        if not self.required_joint_names.issubset(msg.name):
            return
        now = time.monotonic()
        if (
            self.last_joint_state_time is None
            or now - self.last_joint_state_time > self.state_timeout
        ):
            self.robot_state_ready_since = now
        self.last_joint_state_time = now

    def _robot_state_ready(self) -> bool:
        if self.last_joint_state_time is None or self.robot_state_ready_since is None:
            return False
        now = time.monotonic()
        return (
            now - self.last_joint_state_time <= self.state_timeout
            and now - self.robot_state_ready_since >= self.servo_startup_settle_time
        )

    def _on_enable(self, arm: ArmState, msg: Bool):
        was_enabled = arm.enabled_signal
        arm.enabled_signal = bool(msg.data)
        if arm.enabled_signal and not was_enabled:
            self._reset_arm(arm)
            self.get_logger().info(f"{arm.name} Grip pressed; motion enabled")
        elif was_enabled and not arm.enabled_signal:
            self._stop_arm_motion(arm)
            self.get_logger().info(f"{arm.name} Grip released; motion stopped")

    def _on_status(self, msg: String):
        text = msg.data.lower()
        if "disable" in text or "stop" in text:
            for arm in self.arms:
                arm.enabled_signal = False
                self._stop_arm_motion(arm)

    @staticmethod
    def _servo_activation_requested(arm: ArmState) -> bool:
        return arm.enabled_param

    def _motion_requested(self, arm: ArmState) -> bool:
        if self.require_enable_signal:
            return arm.enabled_param and arm.enabled_signal
        return arm.enabled_param

    def _arm_enabled(self, arm: ArmState) -> bool:
        return arm.servo_active and self._motion_requested(arm)

    def _stop_arm_motion(self, arm: ArmState):
        self._reset_arm(arm)
        self._publisher_for(arm).publish(arm.latest_twist)
        arm.published_zero = True

    def _deactivate_arm(self, arm: ArmState, reason: str):
        was_engaged = arm.servo_active or arm.activation_stage != "idle"
        self._stop_arm_motion(arm)
        arm.servo_active = not self.auto_start_servo
        arm.activation_stage = "idle"
        arm.command_future = None
        arm.unpause_future = None
        arm.pause_future = None
        arm.pause_confirmed = not self.auto_start_servo
        arm.startup_pause_confirmed_time = None

        if self.auto_start_servo:
            client = self._pause_clients[arm.name]
            if client.service_is_ready():
                request = SetBool.Request()
                request.data = True
                arm.pause_future = client.call_async(request)
        if was_engaged:
            self.get_logger().warn(f"{arm.name} Servo paused: {reason}")

    def _publisher_for(self, arm: ArmState):
        return self.pub_left if arm is self.left else self.pub_right

    def _on_pose(self, arm: ArmState, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        arm.latest_pose_receive_time = now

        if not self._arm_enabled(arm):
            self._reset_arm(arm)
            return

        msg_time = self._stamp_to_seconds(msg)
        sample_time = msg_time if msg_time > 0.0 else now

        if arm.pose_history:
            dt = sample_time - arm.pose_history[-1][0]
            if dt <= 1e-4 or dt > self.command_timeout:
                self._reset_arm(arm)

        arm.pose_history.append((sample_time, msg))
        oldest_sample_time = arm.pose_history[0][0]
        if sample_time - oldest_sample_time + 1e-6 < self.velocity_window:
            arm.latest_twist = self._zero_twist()
            return

        cutoff_time = sample_time - self.velocity_window
        while (
            len(arm.pose_history) >= 2
            and arm.pose_history[1][0] <= cutoff_time
        ):
            arm.pose_history.popleft()

        window_start_time, window_start_pose = arm.pose_history[0]
        window_dt = sample_time - window_start_time

        """
        Why controller motion is estimated over a time window:
        - The VR and robot coordinate origins are unrelated.
        - An absolute VR pose could cause a jump toward an unreachable target.
        - Pose differences make Grip act as a clutch for relative motion only.
        - A multi-frame difference averages pose jitter better than adjacent frames.
        - Time normalization makes thresholds independent of the VR publish rate.
        - Start/stop hysteresis prevents noise near one threshold from chattering.
        """
        delta_position = self._position_delta(window_start_pose, msg)
        delta_rotation = self._rotation_delta(window_start_pose, msg)
        estimated_linear = tuple(
            component / window_dt for component in delta_position
        )
        estimated_angular = tuple(
            component / window_dt for component in delta_rotation
        )

        arm.linear_motion_active = self._update_hysteresis(
            arm.linear_motion_active,
            _norm(estimated_linear),
            self.linear_velocity_start,
            self.linear_velocity_stop,
        )
        arm.angular_motion_active = self._update_hysteresis(
            arm.angular_motion_active,
            _norm(estimated_angular),
            self.angular_velocity_start,
            self.angular_velocity_stop,
        )

        linear = (0.0, 0.0, 0.0)
        if arm.linear_motion_active:
            linear = tuple(
                component * self.linear_scale for component in estimated_linear
            )

        angular = (0.0, 0.0, 0.0)
        if arm.angular_motion_active:
            angular = tuple(
                component * self.angular_scale for component in estimated_angular
            )

        arm.latest_twist = self._make_twist(linear, angular)
        arm.published_zero = False

    @staticmethod
    def _update_hysteresis(
        active: bool,
        speed: float,
        start_threshold: float,
        stop_threshold: float,
    ) -> bool:
        if active:
            return speed > stop_threshold
        return speed >= start_threshold

    def _position_delta(self, previous: PoseStamped, current: PoseStamped) -> Vector3:
        delta = (
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
            current.pose.position.z - previous.pose.position.z,
        )
        return self._map_unity_vector_to_ros(
            delta,
            self.position_axis_map,
            self.position_axis_sign,
        )

    def _rotation_delta(self, previous: PoseStamped, current: PoseStamped) -> Vector3:
        prev_q = self._pose_quat(previous)
        curr_q = self._pose_quat(current)
        delta_q = _quat_multiply(curr_q, _quat_inverse(prev_q))
        return self._map_unity_vector_to_ros(
            _quat_to_rotvec(delta_q),
            self.rotation_axis_map,
            self.rotation_axis_sign,
        )

    @staticmethod
    def _map_unity_vector_to_ros(
        vector: Vector3,
        axis_map,
        axis_sign,
    ) -> Vector3:
        values = [vector[0], vector[1], vector[2]]
        return tuple(axis_sign[i] * values[axis_map[i]] for i in range(3))

    def _pose_quat(self, msg: PoseStamped) -> Quat:
        return (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def _make_twist(self, linear: Vector3, angular: Vector3) -> TwistStamped:
        twist = TwistStamped()
        twist.header.frame_id = self.control_frame
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = _clamp(linear[0], self.max_linear_speed)
        twist.twist.linear.y = _clamp(linear[1], self.max_linear_speed)
        twist.twist.linear.z = _clamp(linear[2], self.max_linear_speed)
        twist.twist.angular.x = _clamp(angular[0], self.max_angular_speed)
        twist.twist.angular.y = _clamp(angular[1], self.max_angular_speed)
        twist.twist.angular.z = _clamp(angular[2], self.max_angular_speed)
        return twist

    def _zero_twist(self) -> TwistStamped:
        return self._make_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _reset_arm(self, arm: ArmState):
        arm.pose_history.clear()
        arm.linear_motion_active = False
        arm.angular_motion_active = False
        arm.latest_twist = self._zero_twist()

    def _publish_commands(self):
        self._publish_arm(self.left, self.pub_left)
        self._publish_arm(self.right, self.pub_right)

    def _publish_arm(self, arm: ArmState, publisher):
        now = self.get_clock().now().nanoseconds * 1e-9
        stale = (
            arm.latest_pose_receive_time is None
            or now - arm.latest_pose_receive_time > self.command_timeout
        )
        if stale or not self._arm_enabled(arm):
            self._reset_arm(arm)
            if arm.published_zero:
                return
            arm.published_zero = True
        else:
            arm.latest_twist.header.stamp = self.get_clock().now().to_msg()
            arm.published_zero = False

        publisher.publish(arm.latest_twist)

    def _stamp_to_seconds(self, msg: PoseStamped) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def main():
    rclpy.init()
    node = VrPoseToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
