#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import random
import time

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        # Get parameter from launch file (default: True)
        self.declare_parameter('USE_EKF', True)
        use_ekf = self.get_parameter('USE_EKF').get_parameter_value().bool_value

        # Decide odometry topic
        self.odom_topic = '/ekf/odom' if use_ekf else '/odom_noisy'
        self.get_logger().info(f'Using odometry topic: {self.odom_topic}')

        # Publisher: velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ✅ Publisher: target point
        self.target_pub = self.create_publisher(Point, '/target_point', 10)

        # Subscriber: odometry (noisy or EKF)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.target = self.random_target()
        self.get_logger().info(f'🎯 New target: {self.target}')

        # Publish initial target
        self.publish_target()

    # ----------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def random_target(self):
        return (random.uniform(-5, 5), random.uniform(-5, 5))

    # ✅ Publish target helper
    def publish_target(self):
        target_msg = Point()
        target_msg.x = self.target[0]
        target_msg.y = self.target[1]
        target_msg.z = 0.0
        self.target_pub.publish(target_msg)

    # ----------------------------------------------
    def control_loop(self):
        msg = Twist()
        dx = self.target[0] - self.current_x
        dy = self.target[1] - self.current_y
        distance = math.hypot(dx, dy)

        if distance < 0.3:
            msg.linear.x = msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'✅ Reached target {self.target}')
            time.sleep(1.0)
            self.target = self.random_target()
            self.get_logger().info(f'🎯 New target: {self.target}')
            self.publish_target()  # ✅ publish updated target
            return

        # Heading to target
        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_angle - self.yaw),
                                 math.cos(target_angle - self.yaw))

        msg.linear.x = 0.8 * distance
        msg.angular.z = 2.0 * angle_error

        # Clamp speeds
        msg.linear.x = max(min(msg.linear.x, 1.0), -1.0)
        msg.angular.z = max(min(msg.angular.z, 1.5), -1.5)

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
