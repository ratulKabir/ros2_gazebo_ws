#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from task_1_wheeled_robot.core.ekf import EKF  # adjust path to your package


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # EKF instance
        self.ekf = EKF(
            dt=0.1,
            Q=np.diag([0.02, 0.02, np.deg2rad(2.0)])**2,
            R=np.diag([0.5, 0.5, np.deg2rad(5.0)])**2
        )

        self.v, self.w = 0.0, 0.0

        # ROS 2 I/O
        self.create_subscription(Odometry, '/odom_noisy', self.meas_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/ekf/odom', 10)

        self.create_timer(self.ekf.dt, self.timer_callback)

    # ---------------------------------------------------------
    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    # ---------------------------------------------------------
    def meas_callback(self, msg):
        z = np.zeros(3)
        z[0] = msg.pose.pose.position.x
        z[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        z[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.ekf.update(z)

    # ---------------------------------------------------------
    def timer_callback(self):
        self.ekf.predict(self.v, self.w)
        self.publish_estimate()

    # ---------------------------------------------------------
    def publish_estimate(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(self.ekf.x[0])
        msg.pose.pose.position.y = float(self.ekf.x[1])

        q = self.quaternion_from_euler(0, 0, float(self.ekf.x[2]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov = np.zeros(36)
        cov[0], cov[7], cov[35] = self.ekf.P[0, 0], self.ekf.P[1, 1], self.ekf.P[2, 2]
        msg.pose.covariance = list(cov)

        self.pub_odom.publish(msg)

    # ---------------------------------------------------------
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]


# ===============================================================
def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
