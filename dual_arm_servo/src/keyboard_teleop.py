#!/usr/bin/python3.10
"""
keyboard_teleop.py — 双臂键盘遥操作

基于 MoveIt Servo 官方设计模式:
  1. 等待 /servo_left/start_servo 和 /servo_right/start_servo 服务就绪
  2. 调用服务启动伺服循环
  3. 进入键盘主循环，持续发布 TwistStamped 到 /servo_*/delta_twist_cmds
  4. 每周期主动臂发指令、非主动臂发零速，避免切臂后残留旧指令

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
  空格   → 急停 (双臂)
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

        # start_servo 服务的客户端
        self._srv_left = self.create_client(Trigger, '/servo_left/start_servo')
        self._srv_right = self.create_client(Trigger, '/servo_right/start_servo')

    def _call_start_servo(self, srv, name, timeout=5.0):
        """同步调用单个 start_servo，最多等待 timeout 秒"""
        # 等待服务可用
        if not srv.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'{name} start_servo 服务不可用')
            return False
        # 调用服务
        req = Trigger.Request()
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f'{name} start_servo 调用超时')
            return False
        if not future.result().success:
            self.get_logger().error(f'{name} start_servo 返回失败: {future.result().message}')
            return False
        self.get_logger().info(f'{name} Servo 已启动')
        return True

    def start_servos(self):
        """按顺序启动左右臂 servo"""
        self.get_logger().info('等待 Servo 就绪...')
        ok_left = self._call_start_servo(self._srv_left, '左臂')
        ok_right = self._call_start_servo(self._srv_right, '右臂')
        if ok_left and ok_right:
            self.get_logger().info('双臂 Servo 已就绪')
        elif ok_left:
            self.get_logger().warn('仅左臂 Servo 就绪')
        elif ok_right:
            self.get_logger().warn('仅右臂 Servo 就绪')
        else:
            self.get_logger().error('Servo 启动失败')
            return False
        return True

    def _make_twist(self, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        """构造一个 TwistStamped 消息"""
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.twist.angular.x = rx
        msg.twist.angular.y = ry
        msg.twist.angular.z = rz
        return msg

    def _stop_arm(self, arm):
        """向指定臂发零速"""
        zero = self._make_twist()
        if arm == 'left':
            self.pub_left.publish(zero)
        else:
            self.pub_right.publish(zero)

    def _stop_both(self):
        """双臂急停"""
        zero = self._make_twist()
        self.pub_left.publish(zero)
        self.pub_right.publish(zero)

    def spin(self, settings):
        """主循环"""
        # ── 1. 启动 servo ──
        if not self.start_servos():
            return

        self.get_logger().info(
            '键盘控制就绪 | 1/2 切臂 | WASD+QE 平移 | IJKLUO 旋转 | 空格急停')

        # ── 2. 主循环 ──
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            key = get_key(settings)

            if key == '\x03':  # Ctrl-C
                self._stop_both()
                break

            # ── 切臂 ──
            if key == '1':
                if self.arm != 'left':
                    self._stop_arm(self.arm)  # 停旧臂
                self.arm = 'left'
                self.get_logger().info('>>> 左臂')
                continue
            if key == '2':
                if self.arm != 'right':
                    self._stop_arm(self.arm)  # 停旧臂
                self.arm = 'right'
                self.get_logger().info('>>> 右臂')
                continue

            # ── 急停 ──
            if key == ' ':
                self._stop_both()
                self.get_logger().warn('紧急停止！')
                continue

            # ── 构建 Twist ──
            lx = ly = lz = ax = ay = az = 0.0

            if key == 'w':      lx = LINEAR
            elif key == 's':    lx = -LINEAR
            elif key == 'a':    ly = LINEAR
            elif key == 'd':    ly = -LINEAR
            elif key == 'q':    lz = LINEAR
            elif key == 'e':    lz = -LINEAR
            elif key == 'i':    ax = ANGULAR
            elif key == 'k':    ax = -ANGULAR
            elif key == 'j':    ay = ANGULAR
            elif key == 'l':    ay = -ANGULAR
            elif key == 'u':    az = ANGULAR
            elif key == 'o':    az = -ANGULAR
            # key == '' (无按键) → 全部 0，自然停止

            active_twist = self._make_twist(lx, ly, lz, ax, ay, az)
            zero_twist = self._make_twist()

            # Always zero the inactive arm so an old command cannot linger after switching arms.
            if self.arm == 'left':
                self.pub_left.publish(active_twist)
                self.pub_right.publish(zero_twist)
            else:
                self.pub_right.publish(active_twist)
                self.pub_left.publish(zero_twist)


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
