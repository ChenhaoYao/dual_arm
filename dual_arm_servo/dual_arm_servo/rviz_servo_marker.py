#!/usr/bin/env python3
"""Drive both MoveIt Servo instances from RViz interactive markers."""

import copy
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, TwistStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _limit_norm(vector: Vector3, limit: float) -> Vector3:
    length = _norm(vector)
    if length <= limit or length <= 1e-12:
        return vector
    scale = limit / length
    return tuple(value * scale for value in vector)


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    length = math.sqrt(sum(value * value for value in quaternion))
    if length <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(value / length for value in quaternion)


def _quaternion_inverse(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = _normalize_quaternion(quaternion)
    return -x, -y, -z, w


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_to_rotation_vector(quaternion: Quaternion) -> Vector3:
    x, y, z, w = _normalize_quaternion(quaternion)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_length = math.sqrt(x * x + y * y + z * z)
    if vector_length <= 1e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(vector_length, max(-1.0, min(1.0, w)))
    scale = angle / vector_length
    return x * scale, y * scale, z * scale


def _pose_quaternion(pose: Pose) -> Quaternion:
    return (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


@dataclass
class ArmControl:
    name: str
    ee_frame: str
    marker_name: str
    publisher: object
    color: Tuple[float, float, float]
    target: Optional[Pose] = None
    dragging: bool = False
    active: bool = False
    marker_inserted: bool = False
    last_feedback_time: float = 0.0
    zero_sent: bool = True


class RvizServoMarker(Node):
    def __init__(self):
        super().__init__("rviz_servo_marker")
        self._declare_parameters()

        self.base_frame = self.get_parameter("base_frame").value
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.required_joint_names = {
            *(f"laxis{index}_joint" for index in range(1, 8)),
            *(f"raxis{index}_joint" for index in range(1, 8)),
        }
        self.last_joint_state_time: Optional[float] = None
        self.state_missing_logged = False
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.server = InteractiveMarkerServer(self, "rviz_servo_markers")

        self.left = ArmControl(
            "left",
            self.get_parameter("left_ee_frame").value,
            "left_servo_target",
            self.create_publisher(
                TwistStamped, self.get_parameter("left_twist_topic").value, 10
            ),
            (0.1, 0.7, 1.0),
        )
        self.right = ArmControl(
            "right",
            self.get_parameter("right_ee_frame").value,
            "right_servo_target",
            self.create_publisher(
                TwistStamped, self.get_parameter("right_twist_topic").value, 10
            ),
            (1.0, 0.55, 0.1),
        )
        self.arms = (self.left, self.right)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.control_timer = self.create_timer(1.0 / publish_rate, self._control_step)
        self.marker_timer = self.create_timer(0.2, self._initialize_markers)

        self.command_type_clients = [
            self.create_client(ServoCommandType, "/servo_left/switch_command_type"),
            self.create_client(ServoCommandType, "/servo_right/switch_command_type"),
        ]
        self.pause_clients = [
            self.create_client(SetBool, "/servo_left/pause_servo"),
            self.create_client(SetBool, "/servo_right/pause_servo"),
        ]
        self.configure_timer = None
        if bool(self.get_parameter("auto_start_servo").value):
            self.configure_timer = self.create_timer(1.0, self._try_configure_servo)

        self.get_logger().info(
            "RViz Servo markers waiting for left/right end-effector TF"
        )

    def _declare_parameters(self):
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_ee_frame", "laxis7_link")
        self.declare_parameter("right_ee_frame", "raxis7_link")
        self.declare_parameter("left_twist_topic", "/servo_left/delta_twist_cmds")
        self.declare_parameter("right_twist_topic", "/servo_right/delta_twist_cmds")
        self.declare_parameter("linear_gain", 1.5)
        self.declare_parameter("angular_gain", 1.5)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.4)
        self.declare_parameter("position_tolerance", 0.004)
        self.declare_parameter("orientation_tolerance", 0.025)
        self.declare_parameter("command_timeout", 10.0)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("auto_start_servo", True)

    def _on_joint_state(self, msg: JointState):
        if self.required_joint_names.issubset(msg.name):
            self.last_joint_state_time = time.monotonic()
            self.state_missing_logged = False

    def _robot_state_ready(self) -> bool:
        return (
            self.last_joint_state_time is not None
            and time.monotonic() - self.last_joint_state_time <= self.state_timeout
        )

    def _try_configure_servo(self):
        for client in (*self.command_type_clients, *self.pause_clients):
            if not client.wait_for_service(timeout_sec=0.1):
                return
        command_request = ServoCommandType.Request()
        command_request.command_type = ServoCommandType.Request.TWIST
        for client in self.command_type_clients:
            client.call_async(command_request)
        pause_request = SetBool.Request()
        pause_request.data = False
        for client in self.pause_clients:
            client.call_async(pause_request)
        self.configure_timer.cancel()
        self.get_logger().info("MoveIt Servo configured for Twist commands on both arms")

    def _initialize_markers(self):
        if not self._robot_state_ready():
            return
        changed = False
        for arm in self.arms:
            if arm.marker_inserted:
                continue
            pose = self._current_pose(arm)
            if pose is None:
                continue
            marker = self._make_marker(arm, pose)
            self.server.insert(
                marker,
                feedback_callback=lambda feedback, selected=arm: self._on_feedback(
                    selected, feedback
                ),
            )
            arm.marker_inserted = True
            changed = True
            self.get_logger().info(
                f"Created {arm.name} Servo marker at {arm.ee_frame}"
            )
        if changed:
            self.server.applyChanges()
        if all(arm.marker_inserted for arm in self.arms):
            self.marker_timer.cancel()

    def _make_marker(self, arm: ArmControl, pose: Pose) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.frame_id = self.base_frame
        marker.name = arm.marker_name
        marker.description = f"{arm.name.capitalize()} Servo target"
        marker.scale = 0.22
        marker.pose = pose

        visual = Marker()
        visual.type = Marker.SPHERE
        visual.scale.x = 0.065
        visual.scale.y = 0.065
        visual.scale.z = 0.065
        visual.color.r, visual.color.g, visual.color.b = arm.color
        visual.color.a = 0.85

        visual_control = InteractiveMarkerControl()
        visual_control.name = "target"
        visual_control.always_visible = True
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        self._add_axis_controls(marker, "x", (1.0, 0.0, 0.0, 1.0))
        self._add_axis_controls(marker, "y", (0.0, 0.0, 1.0, 1.0))
        self._add_axis_controls(marker, "z", (0.0, 1.0, 0.0, 1.0))
        return marker

    @staticmethod
    def _add_axis_controls(
        marker: InteractiveMarker, axis: str, orientation: Quaternion
    ):
        normalized = _normalize_quaternion(orientation)
        for prefix, mode in (
            ("rotate", InteractiveMarkerControl.ROTATE_AXIS),
            ("move", InteractiveMarkerControl.MOVE_AXIS),
        ):
            control = InteractiveMarkerControl()
            control.name = f"{prefix}_{axis}"
            (
                control.orientation.x,
                control.orientation.y,
                control.orientation.z,
                control.orientation.w,
            ) = normalized
            control.interaction_mode = mode
            marker.controls.append(control)

    def _on_feedback(self, arm: ArmControl, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            arm.dragging = True
            arm.active = False
            arm.target = None
            self._publish_zero(arm, force=True)
            return

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if arm.dragging:
                arm.target = copy.deepcopy(feedback.pose)
            return

        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP or not arm.dragging:
            return

        arm.dragging = False
        arm.target = copy.deepcopy(feedback.pose)
        arm.last_feedback_time = time.monotonic()
        arm.active = True
        arm.zero_sent = False

    def _control_step(self):
        now = time.monotonic()
        for arm in self.arms:
            if not arm.marker_inserted or not arm.active or arm.target is None:
                continue

            if not self._robot_state_ready():
                if not self.state_missing_logged:
                    self.get_logger().warn("Robot joint state is unavailable or stale")
                    self.state_missing_logged = True
                current = self._current_pose(arm)
                if current is not None:
                    self._stop_arm(arm, current)
                else:
                    self._publish_zero(arm, force=True)
                    arm.active = False
                    arm.target = None
                continue

            current = self._current_pose(arm)
            if current is None:
                self._publish_zero(arm)
                continue

            if now - arm.last_feedback_time > self.command_timeout:
                self.get_logger().warn(f"{arm.name} marker command timed out")
                self._stop_arm(arm, current)
                continue

            linear_error = (
                arm.target.position.x - current.position.x,
                arm.target.position.y - current.position.y,
                arm.target.position.z - current.position.z,
            )
            rotation_error = _quaternion_to_rotation_vector(
                _quaternion_multiply(
                    _pose_quaternion(arm.target),
                    _quaternion_inverse(_pose_quaternion(current)),
                )
            )

            if (
                _norm(linear_error) <= self.position_tolerance
                and _norm(rotation_error) <= self.orientation_tolerance
            ):
                self._stop_arm(arm, current)
                continue

            linear = _limit_norm(
                tuple(value * self.linear_gain for value in linear_error),
                self.max_linear_speed,
            )
            angular = _limit_norm(
                tuple(value * self.angular_gain for value in rotation_error),
                self.max_angular_speed,
            )
            arm.publisher.publish(self._make_twist(linear, angular))
            arm.zero_sent = False

    def _current_pose(self, arm: ArmControl) -> Optional[Pose]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, arm.ee_frame, Time()
            )
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _make_twist(self, linear: Vector3, angular: Vector3) -> TwistStamped:
        command = TwistStamped()
        command.header.frame_id = self.base_frame
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = linear
        command.twist.angular.x, command.twist.angular.y, command.twist.angular.z = angular
        return command

    def _publish_zero(self, arm: ArmControl, force: bool = False):
        if arm.zero_sent and not force:
            return
        try:
            arm.publisher.publish(
                self._make_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            )
        except RuntimeError:
            if rclpy.ok():
                raise
        arm.zero_sent = True

    def _stop_arm(self, arm: ArmControl, current: Pose):
        self._publish_zero(arm, force=True)
        arm.dragging = False
        arm.active = False
        arm.target = None
        self.server.setPose(arm.marker_name, current)
        self.server.applyChanges()

    def stop_all(self):
        for arm in self.arms:
            self._publish_zero(arm, force=True)


def main():
    rclpy.init()
    node = RvizServoMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
