#!/usr/bin/env python3
"""
Keyboard teleoperation for dual-arm MoveIt Servo.

Usage:
  ros2 run dual_arm_servo keyboard_teleop.py
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from std_srvs.srv import SetBool


LINEAR = 1.0
ANGULAR = 2.0


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = "" if not rlist else sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub_left = self.create_publisher(TwistStamped, "/servo_left/delta_twist_cmds", 10)
        self.pub_right = self.create_publisher(TwistStamped, "/servo_right/delta_twist_cmds", 10)
        self.arm = "left"

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
            "双臂键盘遥操作就绪 | 1/2 切臂 | WASD+QE 平移 | IJKLUO 旋转 | 空格急停"
        )

    def _try_start_servo(self):
        for srv in (*self._command_type_clients, *self._pause_clients):
            if not srv.wait_for_service(timeout_sec=0.5):
                return
        command_request = ServoCommandType.Request()
        command_request.command_type = ServoCommandType.Request.TWIST
        for srv in self._command_type_clients:
            srv.call_async(command_request)
        pause_request = SetBool.Request()
        pause_request.data = False
        for srv in self._pause_clients:
            srv.call_async(pause_request)
        self._start_timer.cancel()
        self.get_logger().info("Servo 左右臂已切换到 Twist 模式")

    def spin(self, settings):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            key = get_key(settings)

            if key == "\x03":
                break
            if key == "1":
                self.arm = "left"
                self.get_logger().info(">>> 左臂")
                continue
            if key == "2":
                self.arm = "right"
                self.get_logger().info(">>> 右臂")
                continue

            twist = TwistStamped()
            twist.header.frame_id = "base_link"
            twist.header.stamp = self.get_clock().now().to_msg()

            if key == "w":
                twist.twist.linear.x = LINEAR
            elif key == "s":
                twist.twist.linear.x = -LINEAR
            elif key == "a":
                twist.twist.linear.y = LINEAR
            elif key == "d":
                twist.twist.linear.y = -LINEAR
            elif key == "q":
                twist.twist.linear.z = LINEAR
            elif key == "e":
                twist.twist.linear.z = -LINEAR
            elif key == "i":
                twist.twist.angular.x = ANGULAR
            elif key == "k":
                twist.twist.angular.x = -ANGULAR
            elif key == "j":
                twist.twist.angular.y = ANGULAR
            elif key == "l":
                twist.twist.angular.y = -ANGULAR
            elif key == "u":
                twist.twist.angular.z = ANGULAR
            elif key == "o":
                twist.twist.angular.z = -ANGULAR

            if self.arm == "left":
                self.pub_left.publish(twist)
            else:
                self.pub_right.publish(twist)


def main():
    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)
    node = KeyboardTeleop()
    try:
        node.spin(settings)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
