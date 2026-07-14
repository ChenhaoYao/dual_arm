#!/usr/bin/env python3
"""Record low-rate VR input and robot end-effector trajectories as CSV."""

import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import ServoStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("vr_servo_trajectory_logger")
        self._declare_parameters()

        self.base_frame = self.get_parameter("base_frame").value
        self.sample_rate = float(self.get_parameter("sample_rate").value)
        session_root = Path(self.get_parameter("output_directory").value).expanduser()
        session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_directory = session_root / session_name
        self.session_directory.mkdir(parents=True, exist_ok=False)
        self.start_time = self.get_clock().now()
        self.sample_index = 0

        self.latest_pose: Dict[str, Optional[PoseStamped]] = {"left": None, "right": None}
        self.latest_twist: Dict[str, Optional[TwistStamped]] = {"left": None, "right": None}
        self.latest_servo_status: Dict[str, Optional[ServoStatus]] = {
            "left": None, "right": None,
        }
        self.interval_servo_warning: Dict[str, Optional[ServoStatus]] = {
            "left": None, "right": None,
        }
        self.twist_messages = {"left": 0, "right": 0}
        self.nonzero_twist_messages = {"left": 0, "right": 0}
        self.enabled = {"left": False, "right": False}
        self.ee_frames = {
            "left": self.get_parameter("left_ee_frame").value,
            "right": self.get_parameter("right_ee_frame").value,
        }

        self.files = {}
        self.writers = {}
        trajectory_header = [
            "sample_index", "elapsed_sec", "enabled",
            "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        ]
        control_header = [
            "sample_index", "elapsed_sec", "enabled",
            "twist_messages", "nonzero_twist_messages",
            "linear_x", "linear_y", "linear_z",
            "angular_x", "angular_y", "angular_z",
            "servo_status_code", "servo_status_message",
        ]
        for side in ("left", "right"):
            self._open_log(
                f"vr_{side}_hand_trajectory.csv", f"vr_{side}", trajectory_header
            )
            self._open_log(
                f"robot_{side}_ee_trajectory.csv", f"robot_{side}", trajectory_header
            )
            self._open_log(
                f"control_{side}.csv", f"control_{side}", control_header
            )

        self.create_subscription(
            PoseStamped, self.get_parameter("left_pose_topic").value,
            lambda msg: self._on_pose("left", msg), 10,
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("right_pose_topic").value,
            lambda msg: self._on_pose("right", msg), 10,
        )
        self.create_subscription(
            Bool, self.get_parameter("left_enable_topic").value,
            lambda msg: self._on_enable("left", msg), 10,
        )
        self.create_subscription(
            Bool, self.get_parameter("right_enable_topic").value,
            lambda msg: self._on_enable("right", msg), 10,
        )
        self.create_subscription(
            TwistStamped, self.get_parameter("left_twist_topic").value,
            lambda msg: self._on_twist("left", msg), 10,
        )
        self.create_subscription(
            TwistStamped, self.get_parameter("right_twist_topic").value,
            lambda msg: self._on_twist("right", msg), 10,
        )
        self.create_subscription(
            ServoStatus, self.get_parameter("left_servo_status_topic").value,
            lambda msg: self._on_servo_status("left", msg), 10,
        )
        self.create_subscription(
            ServoStatus, self.get_parameter("right_servo_status_topic").value,
            lambda msg: self._on_servo_status("right", msg), 10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warning_logged = {"left": False, "right": False}
        self.timer = self.create_timer(1.0 / self.sample_rate, self._sample)
        self.get_logger().info(f"Trajectory logs: {self.session_directory}")

    def _open_log(self, filename, key, header):
        file_handle = (self.session_directory / filename).open(
            "w", newline="", encoding="utf-8"
        )
        writer = csv.writer(file_handle)
        writer.writerow(header)
        file_handle.flush()
        self.files[key] = file_handle
        self.writers[key] = writer

    def _declare_parameters(self):
        self.declare_parameter("sample_rate", 5.0)
        self.declare_parameter(
            "output_directory", "/home/dell/dual_arm/vr_teleop_bridge/log"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_ee_frame", "laxis7_link")
        self.declare_parameter("right_ee_frame", "raxis7_link")
        self.declare_parameter("left_pose_topic", "/vr/left_hand/pose")
        self.declare_parameter("right_pose_topic", "/vr/right_hand/pose")
        self.declare_parameter("left_enable_topic", "/vr/left_hand/enabled")
        self.declare_parameter("right_enable_topic", "/vr/right_hand/enabled")
        self.declare_parameter("left_twist_topic", "/servo_left/delta_twist_cmds")
        self.declare_parameter("right_twist_topic", "/servo_right/delta_twist_cmds")
        self.declare_parameter("left_servo_status_topic", "/servo_left/status")
        self.declare_parameter("right_servo_status_topic", "/servo_right/status")

    def _on_pose(self, side: str, msg: PoseStamped):
        self.latest_pose[side] = msg

    def _on_enable(self, side: str, msg: Bool):
        self.enabled[side] = bool(msg.data)

    def _on_twist(self, side: str, msg: TwistStamped):
        self.latest_twist[side] = msg
        self.twist_messages[side] += 1
        components = (
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
        )
        if any(abs(component) > 1e-9 for component in components):
            self.nonzero_twist_messages[side] += 1

    def _on_servo_status(self, side: str, msg: ServoStatus):
        self.latest_servo_status[side] = msg
        if msg.code != ServoStatus.NO_WARNING:
            self.interval_servo_warning[side] = msg

    def _sample(self):
        sample_time = self.get_clock().now()
        elapsed_sec = round((sample_time - self.start_time).nanoseconds / 1e9, 3)
        for side in ("left", "right"):
            pose_msg = self.latest_pose[side]
            if pose_msg is not None:
                self.writers[f"vr_{side}"].writerow(self._pose_row(
                    self.sample_index, elapsed_sec, self.enabled[side],
                    pose_msg.pose,
                ))

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_frames[side], Time(),
                    timeout=Duration(seconds=0.02),
                )
                self._tf_warning_logged[side] = False
                self.writers[f"robot_{side}"].writerow(self._transform_row(
                    self.sample_index, elapsed_sec, self.enabled[side],
                    transform,
                ))
            except TransformException as exc:
                if not self._tf_warning_logged[side]:
                    self.get_logger().warn(
                        f"Waiting for {self.base_frame} -> {self.ee_frames[side]}: {exc}"
                    )
                    self._tf_warning_logged[side] = True

            status = self.interval_servo_warning[side]
            if status is None:
                status = self.latest_servo_status[side]
            self.writers[f"control_{side}"].writerow(self._control_row(
                self.sample_index, elapsed_sec, self.enabled[side],
                self.twist_messages[side], self.nonzero_twist_messages[side],
                self.latest_twist[side], status,
            ))
            self.twist_messages[side] = 0
            self.nonzero_twist_messages[side] = 0
            self.interval_servo_warning[side] = None

        for file_handle in self.files.values():
            file_handle.flush()
        self.sample_index += 1

    @staticmethod
    def _pose_row(sample_index, elapsed_sec, enabled, pose):
        return [
            sample_index, elapsed_sec, int(enabled),
            round(pose.position.x, 4),
            round(pose.position.y, 4),
            round(pose.position.z, 4),
            round(pose.orientation.x, 4),
            round(pose.orientation.y, 4),
            round(pose.orientation.z, 4),
            round(pose.orientation.w, 4),
        ]

    @staticmethod
    def _transform_row(sample_index, elapsed_sec, enabled, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return [
            sample_index, elapsed_sec, int(enabled),
            round(translation.x, 4),
            round(translation.y, 4),
            round(translation.z, 4),
            round(rotation.x, 4),
            round(rotation.y, 4),
            round(rotation.z, 4),
            round(rotation.w, 4),
        ]

    @staticmethod
    def _control_row(
        sample_index, elapsed_sec, enabled, twist_messages,
        nonzero_twist_messages, twist_msg, servo_status,
    ):
        if twist_msg is None:
            twist_values = [""] * 6
        else:
            twist_values = [
                round(twist_msg.twist.linear.x, 4),
                round(twist_msg.twist.linear.y, 4),
                round(twist_msg.twist.linear.z, 4),
                round(twist_msg.twist.angular.x, 4),
                round(twist_msg.twist.angular.y, 4),
                round(twist_msg.twist.angular.z, 4),
            ]
        if servo_status is None:
            status_values = ["", ""]
        else:
            status_values = [servo_status.code, servo_status.message]
        return [
            sample_index, elapsed_sec, int(enabled),
            twist_messages, nonzero_twist_messages,
            *twist_values, *status_values,
        ]

    def destroy_node(self):
        for file_handle in self.files.values():
            file_handle.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryLogger()
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
