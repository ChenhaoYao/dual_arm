#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VRHandSubscriber(Node):
    def __init__(self):
        super().__init__('vr_hand_subscriber')
        
        self.left_sub = self.create_subscription(
            PoseStamped, '/vr/left_hand/pose', self.left_hand_callback, 10)
        self.right_sub = self.create_subscription(
            PoseStamped, '/vr/right_hand/pose', self.right_hand_callback, 10)
        
        self.get_logger().info('VR Hand Subscriber started')
    
    def left_hand_callback(self, msg):
        pos = msg.pose.position
        rot = msg.pose.orientation
        self.get_logger().info(
            f'Left Hand: pos({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
            f'rot({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})')
    
    def right_hand_callback(self, msg):
        pos = msg.pose.position
        rot = msg.pose.orientation
        self.get_logger().info(
            f'Right Hand: pos({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
            f'rot({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})')

def main(args=None):
    rclpy.init(args=args)
    node = VRHandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()