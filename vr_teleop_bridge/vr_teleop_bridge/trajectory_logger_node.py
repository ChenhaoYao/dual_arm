#!/usr/bin/env python3
"""Record low-rate VR input and robot end-effector trajectories as CSV."""

import csv
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
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

        self.latest_pose: Dict[str, Optional[PoseStamped]] = {"left": None, "right": None}
        self.enabled = {"left": False, "right": False}
        self.ee_frames = {
            "left": self.get_parameter("left_ee_frame").value,
            "right": self.get_parameter("right_ee_frame").value,
        }

        self.vr_file = (self.session_directory / "vr_hand_trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.robot_file = (self.session_directory / "robot_ee_trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.vr_writer = csv.writer(self.vr_file)
        self.robot_writer = csv.writer(self.robot_file)
        header = [
            "sample_ros_time_ns", "wall_time_utc", "side", "enabled",
            "source_time_sec", "source_time_nanosec", "frame_id",
            "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        ]
        self.vr_writer.writerow(header)
        self.robot_writer.writerow(header)
        self.vr_file.flush()
        self.robot_file.flush()

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warning_logged = {"left": False, "right": False}
        self.timer = self.create_timer(1.0 / self.sample_rate, self._sample)
        self.get_logger().info(f"Trajectory logs: {self.session_directory}")

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

    def _on_pose(self, side: str, msg: PoseStamped):
        self.latest_pose[side] = msg

    def _on_enable(self, side: str, msg: Bool):
        self.enabled[side] = bool(msg.data)

    def _sample(self):
        sample_time = self.get_clock().now()
        wall_time = datetime.now(timezone.utc).isoformat()
        for side in ("left", "right"):
            pose_msg = self.latest_pose[side]
            if pose_msg is not None:
                self.vr_writer.writerow(self._pose_row(
                    sample_time.nanoseconds, wall_time, side, self.enabled[side],
                    pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec,
                    pose_msg.header.frame_id, pose_msg.pose,
                ))

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_frames[side], Time(),
                    timeout=Duration(seconds=0.02),
                )
                self._tf_warning_logged[side] = False
                self.robot_writer.writerow(self._transform_row(
                    sample_time.nanoseconds, wall_time, side, self.enabled[side], transform
                ))
            except TransformException as exc:
                if not self._tf_warning_logged[side]:
                    self.get_logger().warn(
                        f"Waiting for {self.base_frame} -> {self.ee_frames[side]}: {exc}"
                    )
                    self._tf_warning_logged[side] = True

        self.vr_file.flush()
        self.robot_file.flush()

    @staticmethod
    def _pose_row(sample_ns, wall_time, side, enabled, sec, nanosec, frame_id, pose):
        return [
            sample_ns, wall_time, side, int(enabled), sec, nanosec, frame_id,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ]

    @staticmethod
    def _transform_row(sample_ns, wall_time, side, enabled, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        return [
            sample_ns, wall_time, side, int(enabled), stamp.sec, stamp.nanosec,
            transform.header.frame_id,
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        ]

    def destroy_node(self):
        self.vr_file.close()
        self.robot_file.close()
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
