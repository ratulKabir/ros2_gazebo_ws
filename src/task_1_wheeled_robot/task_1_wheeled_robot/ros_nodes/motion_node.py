#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5   # forward velocity (m/s)
        msg.angular.z = 0.05 # yaw rate (rad/s)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
