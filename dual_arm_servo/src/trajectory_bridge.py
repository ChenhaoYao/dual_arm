#!/usr/bin/python3
"""
Bridge Servo JointTrajectory output to JTC with controller-friendly QoS.

MoveIt Servo on Humble publishes RELIABLE + TRANSIENT_LOCAL, while the
JointTrajectoryController topic subscriber uses BEST_EFFORT + VOLATILE.
This bridge normalizes the QoS boundary and stamps zero-time trajectories.
"""

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        servo_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        controller_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub_left = self.create_publisher(
            JointTrajectory, '/left_arm_controller/joint_trajectory', controller_qos
        )
        self._pub_right = self.create_publisher(
            JointTrajectory, '/right_arm_controller/joint_trajectory', controller_qos
        )

        self.create_subscription(
            JointTrajectory, '/servo_left/raw_joint_trajectory',
            lambda msg: self._forward(msg, self._pub_left), servo_qos
        )
        self.create_subscription(
            JointTrajectory, '/servo_right/raw_joint_trajectory',
            lambda msg: self._forward(msg, self._pub_right), servo_qos
        )

    def _forward(self, msg: JointTrajectory, publisher):
        out = copy.deepcopy(msg)
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(out)


def main():
    rclpy.init()
    node = TrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
