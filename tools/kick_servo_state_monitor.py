#!/usr/bin/env python3
"""Temporarily nudge a JointState sample to release MoveIt Servo startup."""

import argparse
import copy
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String


REQUIRED_JOINT_NAMES = {
    *(f"laxis{index}_joint" for index in range(1, 8)),
    *(f"raxis{index}_joint" for index in range(1, 8)),
}


class ServoStateKick(Node):
    def __init__(self, joint_name: str, epsilon: float, cycles: int):
        super().__init__("servo_state_kick")
        self.joint_name = joint_name
        self.epsilon = epsilon
        self.cycles = cycles
        self.latest_state = None
        self.robot_description = None
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.description_subscription = self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            robot_description_qos,
        )

    def _on_joint_state(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return
        if not REQUIRED_JOINT_NAMES.issubset(msg.name):
            return
        if self.joint_name not in msg.name:
            return
        self.latest_state = copy.deepcopy(msg)

    def _on_robot_description(self, msg: String):
        self.robot_description = msg.data

    def wait_for_inputs(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and (
            self.latest_state is None or self.robot_description is None
        ):
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_state is not None and self.robot_description is not None

    def uses_mock_hardware(self) -> bool:
        return "mock_components/GenericSystem" in self.robot_description

    def publish_kick(self):
        original = copy.deepcopy(self.latest_state)
        kicked = copy.deepcopy(original)
        joint_index = kicked.name.index(self.joint_name)
        original_position = kicked.position[joint_index]
        kicked.position[joint_index] = original_position + self.epsilon

        self.destroy_subscription(self.subscription)
        self.destroy_subscription(self.description_subscription)
        publisher = self.create_publisher(
            JointState, "/joint_states", qos_profile_sensor_data
        )

        deadline = time.monotonic() + 3.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("no /joint_states subscribers were discovered")

        self.get_logger().warn(
            f"Temporarily nudging {self.joint_name}: "
            f"{original_position:.9f} -> {kicked.position[joint_index]:.9f}"
        )
        """
        Servo startup workaround:
        - Publish a tiny change copied from the current complete robot state.
        - Immediately restore the unmodified state after every changed sample.
        - Repeat briefly so both independent Servo state monitors see a change.
        - This does not command a controller, but it temporarily injects state data.
        """
        for _ in range(self.cycles):
            kicked.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(kicked)
            time.sleep(0.05)
            original.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(original)
            time.sleep(0.05)

        self.get_logger().info(
            "State kick completed; check that both Servo waiting messages stopped"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Inject a temporary 1e-6 rad JointState change to release the "
            "MoveIt Servo 2.12.4 startup wait. Use only while controls are idle."
        )
    )
    parser.add_argument("--joint", default="laxis1_joint")
    parser.add_argument("--epsilon", type=float, default=1e-6)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument(
        "--allow-real-hardware",
        action="store_true",
        help="Acknowledge temporary state injection when real hardware is connected.",
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.epsilon <= 0.0 or args.epsilon > 1e-5:
        parser.error("--epsilon must be greater than 0 and no more than 1e-5 rad")
    if args.cycles < 1 or args.cycles > 10:
        parser.error("--cycles must be between 1 and 10")
    return args


def main():
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = ServoStateKick(args.joint, args.epsilon, args.cycles)
    try:
        if not node.wait_for_inputs(timeout=10.0):
            node.get_logger().error(
                "Timed out waiting for complete /joint_states and /robot_description"
            )
            return 1
        if node.uses_mock_hardware():
            node.get_logger().info("Detected mock_components/GenericSystem")
        elif not args.allow_real_hardware:
            node.get_logger().error(
                "Non-mock hardware detected; refusing state injection without "
                "--allow-real-hardware"
            )
            return 2
        else:
            node.get_logger().warn(
                "Real-hardware acknowledgement set; ensure Servo motion is disabled"
            )
        node.publish_kick()
        return 0
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
