#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random
import time


class RandomMotionNode(Node):
    def __init__(self):
        super().__init__('random_motion_node')

        # Publisher: velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: odometry
        self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.target = self.random_target()
        self.get_logger().info(f'🎯 New target: {self.target}')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Orientation → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def random_target(self):
        # Random point within ±5 meters
        return (random.uniform(-5, 5), random.uniform(-5, 5))

    def control_loop(self):
        msg = Twist()
        dx = self.target[0] - self.current_x
        dy = self.target[1] - self.current_y
        distance = math.hypot(dx, dy)

        if distance < 0.3:
            # Stop and create a new target
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'✅ Reached target {self.target}')
            time.sleep(1.0)
            self.target = self.random_target()
            self.get_logger().info(f'🎯 New target: {self.target}')
            return

        # Heading angle to the target
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.yaw
        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Simple proportional controller
        msg.linear.x = 0.8 * distance        # forward speed
        msg.angular.z = 2.0 * angle_error    # turning speed

        # Limit speeds
        msg.linear.x = max(min(msg.linear.x, 1.0), -1.0)
        msg.angular.z = max(min(msg.angular.z, 1.5), -1.5)

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
