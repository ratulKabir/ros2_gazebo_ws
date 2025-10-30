#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math


class NoiseInjector(Node):
    def __init__(self):
        super().__init__('noise_injector_node')
        self.sub = self.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(Odometry, '/odom_noisy', 10)

        # Tunable noise levels
        self.pos_std = 0.05                 # [m]
        self.yaw_std = math.radians(2.0)    # [rad]
        self.vel_std = 0.02                 # [m/s]

    # ------------------------------------------------------
    def odom_callback(self, msg):
        noisy = Odometry()
        noisy.header = msg.header
        noisy.child_frame_id = msg.child_frame_id

        # --- add Gaussian noise to position ---
        noisy.pose.pose.position.x = msg.pose.pose.position.x + np.random.randn() * self.pos_std
        noisy.pose.pose.position.y = msg.pose.pose.position.y + np.random.randn() * self.pos_std
        noisy.pose.pose.position.z = msg.pose.pose.position.z

        # --- add noise to orientation (yaw) ---
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) + np.random.randn() * self.yaw_std

        # back to quaternion
        noisy.pose.pose.orientation.x = 0.0
        noisy.pose.pose.orientation.y = 0.0
        noisy.pose.pose.orientation.z = math.sin(yaw / 2.0)
        noisy.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # --- optionally add noise to velocity ---
        noisy.twist.twist.linear.x = msg.twist.twist.linear.x + np.random.randn() * self.vel_std
        noisy.twist.twist.angular.z = msg.twist.twist.angular.z + np.random.randn() * math.radians(1.0)

        self.pub.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = NoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
