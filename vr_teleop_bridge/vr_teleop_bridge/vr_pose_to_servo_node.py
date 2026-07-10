#!/usr/bin/env python3
"""Convert Unity/PICO VR controller poses to MoveIt Servo TwistStamped commands."""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
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
    last_pose: Optional[PoseStamped] = None
    last_pose_time: Optional[float] = None
    latest_twist: TwistStamped = field(default_factory=TwistStamped)
    latest_pose_receive_time: Optional[float] = None
    published_zero: bool = True


class VrPoseToServo(Node):
    def __init__(self):
        super().__init__("vr_pose_to_servo_node")

        self._declare_parameters()
        self.control_frame = self.get_parameter("control_frame").value
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.deadband_position = float(self.get_parameter("deadband_position").value)
        self.deadband_rotation = float(self.get_parameter("deadband_rotation").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.require_enable_signal = bool(self.get_parameter("require_enable_signal").value)
        self.auto_start_servo = bool(self.get_parameter("auto_start_servo").value)
        self.axis_map = list(self.get_parameter("unity_to_ros_axis_map").value)
        self.axis_sign = [float(value) for value in self.get_parameter("unity_to_ros_axis_sign").value]

        self._validate_axis_mapping()

        self.left = ArmState("left", bool(self.get_parameter("left_enabled").value))
        self.right = ArmState("right", bool(self.get_parameter("right_enabled").value))

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

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self._publish_commands)

        self._command_type_clients: List = []
        self._pause_clients: List = []
        self._start_wait_log_time = 0.0
        if self.auto_start_servo:
            self._command_type_clients = [
                self.create_client(ServoCommandType, "/servo_left/switch_command_type"),
                self.create_client(ServoCommandType, "/servo_right/switch_command_type"),
            ]
            self._pause_clients = [
                self.create_client(SetBool, "/servo_left/pause_servo"),
                self.create_client(SetBool, "/servo_right/pause_servo"),
            ]
            self._start_timer = self.create_timer(1.0, self._try_start_servo)

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
        self.declare_parameter("require_enable_signal", False)
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("deadband_position", 0.005)
        self.declare_parameter("deadband_rotation", 0.02)
        self.declare_parameter("command_timeout", 0.2)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("auto_start_servo", True)
        self.declare_parameter("unity_to_ros_axis_map", [2, 0, 1])
        self.declare_parameter("unity_to_ros_axis_sign", [1.0, -1.0, 1.0])

    def _validate_axis_mapping(self):
        if sorted(self.axis_map) != [0, 1, 2] or len(self.axis_sign) != 3:
            raise ValueError("unity_to_ros_axis_map must be a permutation of [0, 1, 2]")

    def _try_start_servo(self):
        for client in (*self._command_type_clients, *self._pause_clients):
            if not client.wait_for_service(timeout_sec=0.2):
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self._start_wait_log_time > 5.0:
                    self._start_wait_log_time = now
                    self.get_logger().info(
                        "Waiting for MoveIt Servo configuration services"
                    )
                return
        command_request = ServoCommandType.Request()
        command_request.command_type = ServoCommandType.Request.TWIST
        for client in self._command_type_clients:
            client.call_async(command_request)
        pause_request = SetBool.Request()
        pause_request.data = False
        for client in self._pause_clients:
            client.call_async(pause_request)
        self._start_timer.cancel()
        self.get_logger().info("MoveIt Servo configured for Twist commands on both arms")

    def _on_enable(self, arm: ArmState, msg: Bool):
        arm.enabled_signal = bool(msg.data)
        if not self._arm_enabled(arm):
            self._reset_arm(arm)

    def _on_status(self, msg: String):
        text = msg.data.lower()
        if "disable" in text or "stop" in text:
            self._reset_arm(self.left)
            self._reset_arm(self.right)

    def _arm_enabled(self, arm: ArmState) -> bool:
        if self.require_enable_signal:
            return arm.enabled_param and arm.enabled_signal
        return arm.enabled_param

    def _on_pose(self, arm: ArmState, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        arm.latest_pose_receive_time = now

        if not self._arm_enabled(arm):
            self._reset_arm(arm)
            return

        msg_time = self._stamp_to_seconds(msg)
        sample_time = msg_time if msg_time > 0.0 else now

        if arm.last_pose is None or arm.last_pose_time is None:
            arm.last_pose = msg
            arm.last_pose_time = sample_time
            arm.latest_twist = self._zero_twist()
            return

        dt = sample_time - arm.last_pose_time
        if dt <= 1e-4 or dt > self.command_timeout:
            arm.last_pose = msg
            arm.last_pose_time = sample_time
            arm.latest_twist = self._zero_twist()
            return

        delta_position = self._position_delta(arm.last_pose, msg)
        delta_rotation = self._rotation_delta(arm.last_pose, msg)

        linear = (0.0, 0.0, 0.0)
        if _norm(delta_position) >= self.deadband_position:
            linear = tuple(component * self.linear_scale / dt for component in delta_position)

        angular = (0.0, 0.0, 0.0)
        if _norm(delta_rotation) >= self.deadband_rotation:
            angular = tuple(component * self.angular_scale / dt for component in delta_rotation)

        arm.latest_twist = self._make_twist(linear, angular)
        arm.last_pose = msg
        arm.last_pose_time = sample_time
        arm.published_zero = False

    def _position_delta(self, previous: PoseStamped, current: PoseStamped) -> Vector3:
        delta = (
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
            current.pose.position.z - previous.pose.position.z,
        )
        return self._map_unity_vector_to_ros(delta)

    def _rotation_delta(self, previous: PoseStamped, current: PoseStamped) -> Vector3:
        prev_q = self._pose_quat(previous)
        curr_q = self._pose_quat(current)
        delta_q = _quat_multiply(curr_q, _quat_inverse(prev_q))
        return self._map_unity_vector_to_ros(_quat_to_rotvec(delta_q))

    def _map_unity_vector_to_ros(self, vector: Vector3) -> Vector3:
        values = [vector[0], vector[1], vector[2]]
        return tuple(self.axis_sign[i] * values[self.axis_map[i]] for i in range(3))

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
        arm.last_pose = None
        arm.last_pose_time = None
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
            arm.latest_twist = self._zero_twist()
            arm.last_pose = None
            arm.last_pose_time = None
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
