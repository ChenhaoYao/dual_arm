#!/usr/bin/python3.10
"""
keyboard_teleop.py — 双臂键盘遥操作

基于 pymoveit2 模式: TwistStamped frame_id=base_link,
线性速度始终在世界坐标系,不受机械臂姿态影响。

用法:
  ros2 run dual_arm_servo keyboard_teleop.py

按键:
  1/2    → 左臂/右臂
  W/S    → 前进/后退  (X)
  A/D    → 左移/右移  (Y)
  Q/E    → 上升/下降  (Z)
  I/K    → 绕X旋转
  J/L    → 绕Y旋转
  U/O    → 绕Z旋转
  空格   → 急停
  Ctrl-C → 退出
"""

import termios
import tty
import select
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

LINEAR = 1.0      # m/s
ANGULAR = 2.0     # rad/s


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = "" if not rlist else sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub_left = self.create_publisher(TwistStamped, '/servo_left/delta_twist_cmds', 10)
        self.pub_right = self.create_publisher(TwistStamped, '/servo_right/delta_twist_cmds', 10)
        self.arm = 'left'

        # ——— 自动启动左右臂 servo ———
        self._start_srv = self.create_client(Trigger, '/servo_left/start_servo')
        self._start_srv_right = self.create_client(Trigger, '/servo_right/start_servo')
        self._start_timer = self.create_timer(1.0, self._try_start_servo)

        self.get_logger().info(
            '双臂键盘遥操作就绪 | 1/2 切臂 | WASD+QE 平移 | IJKLUO 旋转 | 空格急停')

    def _try_start_servo(self):
        for srv, name in [(self._start_srv, 'left'), (self._start_srv_right, 'right')]:
            if not srv.wait_for_service(timeout_sec=0.5):
                return
            req = Trigger.Request()
            srv.call_async(req)
        self._start_timer.cancel()
        self.get_logger().info('Servo 左右臂已启动')

    def spin(self, settings):
        """主循环: 读取按键 → 发布 TwistStamped (frame_id=base_link)"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            key = get_key(settings)

            if key == '\x03':  # Ctrl-C
                break
            if key == '1':
                self.arm = 'left'
                self.get_logger().info('>>> 左臂')
                continue
            if key == '2':
                self.arm = 'right'
                self.get_logger().info('>>> 右臂')
                continue

            twist = TwistStamped()
            twist.header.frame_id = 'base_link'
            twist.header.stamp = self.get_clock().now().to_msg()

            # 线性 (base_link 系, 方向永远不变)
            if key == 'w':      twist.twist.linear.x = LINEAR
            elif key == 's':    twist.twist.linear.x = -LINEAR
            elif key == 'a':    twist.twist.linear.y = LINEAR
            elif key == 'd':    twist.twist.linear.y = -LINEAR
            elif key == 'q':    twist.twist.linear.z = LINEAR
            elif key == 'e':    twist.twist.linear.z = -LINEAR
            # 角速度 (base_link 系)
            elif key == 'i':    twist.twist.angular.x = ANGULAR
            elif key == 'k':    twist.twist.angular.x = -ANGULAR
            elif key == 'j':    twist.twist.angular.y = ANGULAR
            elif key == 'l':    twist.twist.angular.y = -ANGULAR
            elif key == 'u':    twist.twist.angular.z = ANGULAR
            elif key == 'o':    twist.twist.angular.z = -ANGULAR
            # 否则发布 0 (包括空格和不认识键 — 自动急停)

            if self.arm == 'left':
                self.pub_left.publish(twist)
            else:
                self.pub_right.publish(twist)


def main():
    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)
    node = KeyboardTeleop()
    try:
        node.spin(settings)
    except Exception as exc:
        print(f'Error: {exc}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
